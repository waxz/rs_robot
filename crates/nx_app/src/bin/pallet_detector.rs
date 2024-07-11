use nx_common::common::math::Point3f;
use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::task::TaskManager;
use nx_common::common::thread::Thread;
use nx_common::common::types::UnsafeSender;
use nx_message_center::base::common_message::shared::{
    HeaderString, Path, PointCloud2, PoseStamped,
};
use nx_message_center::base::pointcloud_process::perception::{
    pointcloud_clip, pointcloud_transform,
};
use nx_message_center::base::{common_message, message_handler, pointcloud_process, tiny_alloc};

use itertools::{izip, Itertools};

use std::cell::RefCell;
use std::f32::consts::{PI, TAU};
use std::f64::consts::FRAC_PI_2;
use std::ffi::c_void;
use std::io::Write;
use std::ops::{BitAnd, Deref};
use std::rc::Rc;
use std::{fs, slice};
use time::{Duration, OffsetDateTime, UtcOffset};

use rand::{thread_rng, Rng};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;
use std::sync::{Arc, Mutex, MutexGuard};
// argument
use clap::Parser;

use serde::{Deserialize, Serialize};

//------------
use rolling_file::{BasicRollingFileAppender, RollingConditionBasic};
use time::macros::format_description;

use nx_common::common::time::get_now_local;
use tracing::{error, event, info, instrument, span, trace, warn, Level};
use tracing_subscriber::fmt::format;
use tracing_subscriber::fmt::time::OffsetTime;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

//-------------

const TA_BUFFER_SIZE: usize = 200000000;
static mut TA_BUFFER: [u8; TA_BUFFER_SIZE] = [0; TA_BUFFER_SIZE];

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_gui/config/gui_config.toml"
    )]
    dds_config: String,
    #[arg(short, long, default_value = "/tmp/logs")]
    log_dir: String,
    #[arg(short, long, default_value_t = 100.0)]
    task_fps: f32,
    #[arg(short, long, default_value_t = false)]
    check_stamp: bool,
}

pub(crate) mod params
{
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct GuiConfig
    {
        pub extrinsic: PoseMarker,
        pub cloud: CloudConfig,
        pub filter_ground: GroundFilter,
        pub filter_vertical: VerticalFilter,
        pub filter_pallet: PalletFilter,
    }

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]

    struct PalletFilter
    {
        output_mode: u32,

        //
        search_direction: u64,

        //
        filter_pallet_row_high: i32,
        filter_pallet_row_low: i32,

        filter_pallet_x_min: f32,
        filter_pallet_x_max: f32,
        filter_pallet_y_min: f32,
        filter_pallet_y_max: f32,
        filter_pallet_z_min: f32,
        filter_pallet_z_max: f32,
        filter_pallet_jx: f32,
        filter_pallet_jy: f32,
        filter_pallet_jz: f32,
    }

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]

    struct VerticalFilter
    {
        output_mode: u32,

        //
        search_direction: u64,

        // dim limut
        // range limit
        init_center_height_min: u64,
        init_center_height_max: u64,
        init_center_width_min: u64,
        init_center_width_max: u64,
        init_center_cx_min: f32,
        init_center_cx_max: f32,
        init_center_cy_min: f32,
        init_center_cy_max: f32,
        init_center_cz_min: f32,
        init_center_cz_max: f32,
        init_center_jx_max: f32,
        init_center_jy_max: f32,
        init_center_jz_max: f32,
    }

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    struct GroundFilter
    {
        output_mode: u32,

        //
        search_direction: u64,
        // initial ground region
        init_ground_height_min: u64,
        init_ground_height_max: u64,
        init_ground_width_min: u64,
        init_ground_width_max: u64,

        // check initial ground is ground
        init_ground_cx_min: f32,
        init_ground_cx_max: f32,
        init_ground_cy_min: f32,
        init_ground_cy_max: f32,
        init_ground_cz_min: f32,
        init_ground_cz_max: f32,

        // ground region grow to intersect vertical region
        init_ground_nz_min: f32,

        // search region
        ray_height_min: u64,
        ray_height_max: u64,
        ray_width_min: u64,
        ray_width_max: u64,

        // detect vertical plane, pallet or cargo or wall, or outlier shadow
        scan_width_step: u64,
        // find stable center[cx,cy,cz],reject outlier

        // adaptive thresh
        adaptive_x_min: f32,
        adaptive_x_max: f32,
        adaptive_y_min: f32,
        adaptive_y_max: f32,
        adaptive_z_min: f32,
        adaptive_z_max: f32,
        far_uncertain_z_max: f32,
        far_uncertain_x_change_min: f32,
        far_uncertain_adaptive_z_max: f32,
        far_uncertain_row: i32,
    }

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct CloudConfig
    {
        pub dim: CloudDimConfig,
        pub filter: CloudFilterConfig,
    }
    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct CloudFilterConfig
    {
        pub(crate) enable_mean_window: bool,
        pub(crate) mean_window_len: u32,
        pub mean_window_jump_max: f32,

        enable_dim: bool, //Option<bool>,
        pub(crate) filter_height_min: u64,
        pub(crate) filter_height_max: u64,
        pub(crate) filter_width_min: u64,
        pub(crate) filter_width_max: u64,

        enable_range: bool, //Option<bool>,
        filter_x_min: f32,
        filter_y_min: f32,
        filter_z_min: f32,
        filter_x_max: f32,
        filter_y_max: f32,
        filter_z_max: f32,
    }
    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct CloudDimConfig
    {
        pub height: usize,
        pub width: usize,
    }
    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct PoseMarker
    {
        pub pose: Pose,
        pub enable: bool,
    }
    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct Pose
    {
        pub tx: f32,
        pub ty: f32,
        pub tz: f32,
        pub roll: f32,
        pub pitch: f32,
        pub yaw: f32,
    }

    #[derive(Debug, Serialize, Deserialize, Copy, Clone)]
    pub struct AppConfig
    {
        pub pallet_detector: GuiConfig,
    }
}

pub(crate) mod io_message
{
    use serde::ser::SerializeStruct;
    use serde::{Deserialize, Serialize, Serializer};

    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppCmdBase
    {
        pub cmd_type: String,
        pub detect_cmd: Option<AppCmdDetect>,
    }

    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppCmdData
    {
        pub data: AppCmdBase,
    }
    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppCmdDetect
    {
        pub task_id: String,
        pub retry_count: u32,
    }
    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppCmdConfig {}

    #[derive(Debug, Serialize, Deserialize)]
    pub struct PalletPose
    {
        pub  tx: f64,
        pub  ty: f64,
        pub  tz: f64,
        pub  roll: f64,
        pub  pitch: f64,
        pub  yaw: f64,
    }
    #[derive(Debug, Serialize, Deserialize)]
    pub struct PalletInfo
    {
        pub  pallet: PalletPose,
        pub  confidence: f32,
        pub info : i32,
    }

    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppStatusStatistic
    {
        pub cloud_message_count: u64,
        pub boot_up_time: String,
        pub cycle_time: String,
        pub max_cycle_time: String,
        pub current_time: String,
        pub up_duration: String,
    }
    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppStatusTask
    {
        pub process: String,
        pub process_result: String,

        // pub task_id: String,
        pub cmd_ok_count: u64,
        pub cmd_nok_count: u64,

        pub success_count: u64,
        pub fail_count: u64,
        pub last_success_task_id: String,
        pub last_fail_task_id: String,
        pub message: String,
        pub pallet_result: Vec<PalletInfo>,
    }

    #[derive(Debug, Serialize, Deserialize)]
    pub struct AppStatus
    {
        pub statistic: AppStatusStatistic,
        pub task: AppStatusTask,
        pub detect_task: AppCmdDetect,
    }
}

fn main()
{
    println!("what's up");
    let args = Args::parse();
    println!("args: {:?}", args);

    let tz_offset = nx_common::common::time::get_timezone_offset();

    let boot_up_time = get_now_local();
    let mut last_current_time = get_now_local();
    let mut current_time = get_now_local();
    let mut cycle_time = current_time - last_current_time;
    let mut max_cycle_time = cycle_time;

    let up_duration = get_now_local() - boot_up_time;

    let time_fmt = OffsetTime::new(
        tz_offset,
        format_description!("[year]-[month]-[day] [hour]:[minute]:[second]:[subsecond] "),
    );
    let current_file_name = std::path::Path::new(file!())
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();
    println!("current_file_name: {}", current_file_name);

    fs::create_dir_all(&args.log_dir);
    let file_appender = BasicRollingFileAppender::new(
        format!("{}/{}", &args.log_dir, current_file_name),
        RollingConditionBasic::new().hourly().max_size(1024 * 10240),
        10,
    )
    .unwrap();
    // let wheel_file_appender = LogFileInitializer { directory: &args.log_dir, filename: &current_file_name, max_n_old_files: 2, preferred_max_file_size_mib: 1, }.init().unwrap();

    let (file_writer, _guard) = tracing_appender::non_blocking(file_appender);

    let subscriber = Registry::default()
        .with(
            fmt::Layer::default()
                .with_writer(file_writer)
                .with_ansi(false)
                .with_line_number(true)
                .with_thread_ids(true)
                .with_file(true)
                .with_timer(time_fmt.clone())
                .with_filter(filter::LevelFilter::WARN), // .with_filter(filter::filter_fn(|metadata| {
                                                         //     // println!("metadata:{:?}",metadata);
                                                         //     // *metadata.level() == filter::LevelFilter::ERROR
                                                         //     metadata.target().starts_with("hello")
                                                         // }))
        )
        .with(
            fmt::Layer::default()
                .with_writer(std::io::stdout)
                .with_line_number(true)
                .with_file(true)
                .with_thread_ids(true)
                .with_timer(time_fmt.clone())
                .with_filter(filter::LevelFilter::INFO),
        );

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");

    info!("start program");

    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );
    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 1024, 16, 8);

    let config_file = &args.dds_config;

    println!("load toml file : {:?}", config_file);

    let contents = match fs::read_to_string(config_file) {
        // If successful return the files text as `contents`.
        // `c` is a local variable.
        Ok(c) => c,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            println!("Could not read file `{}`", config_file);
            // Exit the program with exit code `1`.
            return;
        }
    };
    println!("contents:\n{:?}", contents);

    let mut app_config: params::AppConfig; // = toml::from_str(&contents).unwrap();

    match toml::from_str::<params::AppConfig>(&contents) {
        Ok(t) => app_config = t,
        Err(e) => {
            println!("toml::from_str fail: {:?}", e);
            return;
        }
    }

    let mut dds_handler = Rc::new(RefCell::new(message_handler::MessageHandler::new("dds")));
    let ok = dds_handler
        .borrow_mut()
        .create(args.dds_config.as_str(), *allocator.cfg.get());

    if !ok {
        println!("exit");
        return;
    }

    let mut app_config: params::AppConfig; // = toml::from_str(&contents).unwrap();
    match toml::from_str::<params::AppConfig>(&contents) {
        Ok(t) => app_config = t,
        Err(e) => {
            println!("toml::from_str: {:?}", e);
            return;
        }
    }

    let cloud_dim = app_config.pallet_detector.cloud.dim;
    let mut mean_window_filter_max = app_config.pallet_detector.cloud.filter.mean_window_len;
    let mut enable_mean_window = app_config.pallet_detector.cloud.filter.enable_mean_window;
    let mut mean_window_filter_count = 0u32;
    let mut mean_window_filter_count_reach = !enable_mean_window;
    let mean_window_jump_max = app_config.pallet_detector.cloud.filter.mean_window_jump_max;

    let mut new_cloud_ready = Rc::new(RefCell::new(false));
    let mut start_detect_cmd = Rc::new(RefCell::new(false));
    let mut cloud_msg_count = Rc::new(RefCell::new(0u64));

    let mut raw_float_vec =
        allocator.alloc_as_array::<f32>((cloud_dim.width * cloud_dim.height * 3) as usize);
    let mut mean_filter_float_vec =
        allocator.alloc_as_array::<f32>((cloud_dim.width * cloud_dim.height * 3) as usize);

    let mut transform_float_vec =
        allocator.alloc_as_array::<f32>((cloud_dim.width * cloud_dim.height * 3) as usize);

    let mut pallet_detector_handler = Rc::new(RefCell::new(
        pointcloud_process::perception::PointcloudPalletDetector::new(),
    ));

    let ok = pallet_detector_handler
        .borrow_mut()
        .create(args.dds_config.as_str(), *allocator.cfg.get());

    if !ok {
        println!("exit");
        return;
    }

    let mut signal = SignalHandler::default();

    let mut detector_cmd_buffer = allocator.alloc_as_array::<*mut c_void>(100 as usize);
    let mut send_status = Rc::new(RefCell::new(HeaderString::new(1024, *allocator.cfg.get())));

    send_status.borrow_mut().set_frame_id("BootUp");
    send_status.borrow_mut().set_data("ok");

    let mut send_result = Path::new(2, *allocator.cfg.get());
    send_result.set_frame_id("BootUp");
    {
        let mut send_result_poses = send_result.get_mut_data();
        send_result_poses[0].position.x = 0.0;
    }

    let mut signal = SignalHandler::default();

    let mut app_status_data = Rc::new(RefCell::new(io_message::AppStatus {
        statistic: io_message::AppStatusStatistic {
            cloud_message_count: 0,

            boot_up_time: format!("{}", boot_up_time),
            cycle_time: "".to_string(),
            max_cycle_time: "".to_string(),
            current_time: format!("{}", boot_up_time),
            up_duration: format!("{}", up_duration),
        },
        task: io_message::AppStatusTask {
            process: "".to_string(),
            success_count: 0,
            fail_count: 0,
            last_success_task_id: "".to_string(),
            last_fail_task_id: "".to_string(),
            message: "".to_string(),
            pallet_result: vec![],
            process_result: "".to_string(),
            cmd_ok_count: 0,
            cmd_nok_count: 0,
        },
        detect_task: io_message::AppCmdDetect {
            task_id: "".to_string(),
            retry_count: 0,
        },
    }));

    let mut task_manager = TaskManager::new(20, args.task_fps, 0);

    {
        let dds_handler = dds_handler.clone();
        let new_cloud_ready = new_cloud_ready.clone();
        let start_detect_cmd = start_detect_cmd.clone();
        let send_status = send_status.clone();
        let app_status_data = app_status_data.clone();

        task_manager.add(
            "recv_cmd",
            move || {
                let mut current_time = get_now_local();

                //---- recv cmd
                let mut app_status_data_binding = app_status_data.borrow_mut();
                let mut send_status_biding = send_status.borrow_mut();

                let mut dds_handler_binding = dds_handler.borrow();
                let recv_cmd = dds_handler_binding.read_data(c"detector_cmd_sub");
                for m in recv_cmd.iter() {
                    let msg: HeaderString = HeaderString::from_ptr(*m);
                    let timestamp_u64 = msg.get_stamp();
                    let stamp = nx_common::common::time::get_local_from_ns(timestamp_u64 as i128);

                    let stamp_valid = (!args.check_stamp)
                        || ((stamp > boot_up_time)
                            && ((current_time - stamp).as_seconds_f32().abs() < 0.5));
                    println!("recv_cmd: stamp_valid: {}, stamp: {}", stamp_valid, stamp);

                    if (stamp_valid) {
                        let data = msg.get_data();
                        println!("recv_cmd, stamp: {}, data: {:?}", stamp, data);



                        match toml::from_str::<io_message::AppCmdBase>(&data) {
                            Ok(mut t) => match t.cmd_type.as_str() {
                                "DetectPallet" => match t.detect_cmd {
                                    Some(mut t) => {
                                        t.retry_count = t.retry_count.max(1).min(5);

                                        if( t.task_id == app_status_data_binding.detect_task.task_id){
                                            // app_status_data_binding.detect_task = t;
                                            // app_status_data_binding.task.process_result =  "Fail".to_string();
                                            // app_status_data_binding.task.message = format!("Repeating task_id: {}", app_status_data_binding.detect_task.task_id);
                                            // app_status_data_binding.detect_task.retry_count = 0;
                                            app_status_data_binding.task.cmd_nok_count+=1;

                                        }else {
                                            app_status_data_binding.detect_task = t;
                                            app_status_data_binding.task.process = "ParseCommand".to_string();
                                            app_status_data_binding.task.process_result =
                                                "Success".to_string();
                                            app_status_data_binding.task.message = "Success".to_string();
                                            app_status_data_binding.task.cmd_ok_count+=1;

                                        }

                                    }
                                    _ => {

                                        app_status_data_binding.task.process = "ParseCommand".to_string();

                                        app_status_data_binding.task.process_result =
                                            "Fail".to_string();
                                        app_status_data_binding.task.message = format!(
                                            "undefined detect_task error: [{}]",
                                            t.cmd_type
                                        );
                                        app_status_data_binding.task.cmd_nok_count+=1;
                                        app_status_data_binding.detect_task.retry_count = 0;
                                    }
                                },
                                _ => {
                                    app_status_data_binding.task.process = "ParseCommand".to_string();

                                    app_status_data_binding.task.process_result = "Fail".to_string();
                                    app_status_data_binding.task.message =
                                        format!("undefined cmd_type error: [{}]", t.cmd_type);
                                    app_status_data_binding.detect_task.retry_count = 0;
                                    app_status_data_binding.task.cmd_nok_count+=1;


                                }
                            },
                            Err(e) => {
                                info!("parse error {}", e);
                                app_status_data_binding.task.process = "ParseCommand".to_string();

                                app_status_data_binding.task.process_result = "Fail".to_string();
                                app_status_data_binding.task.message =
                                    format!("parse data error: {:?}", e);
                                app_status_data_binding.detect_task.retry_count = 0;
                                app_status_data_binding.task.cmd_nok_count+=1;

                            }
                        }
                        let app_status_data_str =
                            toml::to_string(&*app_status_data_binding).unwrap();

                        send_status_biding.set_data(app_status_data_str.as_str());
                        info!("send app_status_data_str: {}", app_status_data_str.as_str());

                        dds_handler_binding.write_data(
                            c"detector_status_pub",
                            &mut [*send_status_biding.get_ptr() as *mut std::os::raw::c_void],
                        );
                    }else {
                        app_status_data_binding.task.cmd_nok_count+=1;
                        // app_status_data_binding.detect_task.retry_count = 0;
                    }
                }

                if( !*start_detect_cmd.borrow() &&  app_status_data_binding.detect_task.retry_count > 0){
                    info!("detector needs to run");
                    app_status_data_binding.task.process = "RunningDetector".to_string();
                    app_status_data_binding.task.process_result = "Running".to_string();
                    app_status_data_binding.task.message = "".to_string();
                    app_status_data_binding.task.pallet_result.clear();

                    *start_detect_cmd.borrow_mut() = true;
                }

                true
            },
            100.0,
        );
    }

    {
        let filter = app_config.pallet_detector.cloud.filter;
        let extrinsic = app_config.pallet_detector.extrinsic;
        let dds_handler = dds_handler.clone();
        let mut pallet_detector_handler = pallet_detector_handler.clone();
        let new_cloud_ready = new_cloud_ready.clone();
        let start_detect_cmd = start_detect_cmd.clone();
        let cloud_msg_count = cloud_msg_count.clone();
        let app_status_data = app_status_data.clone();

        let boot_up_time = boot_up_time;

        let args = args;

        task_manager.add(
            "recv_cloud",
            move || {
                let mut app_status_data_binding = app_status_data.borrow_mut();

                let cloud_dim_height = filter.filter_height_max - filter.filter_height_min;

                let cloud_dim_width = (filter.filter_width_max - filter.filter_width_min);

                //---- recv cmd
                let mut binding = dds_handler.borrow_mut();
                let recv_cloud = binding.read_data(c"cloud_sub");

                let mut current_time = get_now_local();

                for m in recv_cloud.iter() {
                    let data: PointCloud2 = PointCloud2::from_ptr(*m);

                    let timestamp_u64 = data.get_stamp();
                    let stamp =
                        OffsetDateTime::from_unix_timestamp_nanos(timestamp_u64 as i128).unwrap();
                    let stamp = nx_common::common::time::get_local_from_ns(timestamp_u64 as i128);

                    let stamp_valid = (!args.check_stamp)
                        || ((stamp > boot_up_time)
                            && ((current_time - stamp).as_seconds_f32().abs() < 0.5));
                    // info!("recv_cloud: stamp_valid: {}, stamp: {}", stamp_valid, stamp);

                    *cloud_msg_count.borrow_mut() += stamp_valid as u64;
                    if (!*start_detect_cmd.borrow()) {
                        *new_cloud_ready.borrow_mut() = false;
                        return true;
                    }

                    let current_stamp = OffsetDateTime::now_utc();

                    let stamp_diff = current_stamp - stamp;

                    let stamp_diff_s = stamp_diff.as_seconds_f32();
                    let stamp_local = stamp.to_offset(tz_offset);
                    let stamp_valid = (stamp_diff_s.abs() < 1.0);

                    let frame_id = data.get_frame_id();
                    let cloud_float_vec = data.get_data();

                    let cloud_height_width = data.get_height_width();

                    let float_vec_len = cloud_dim_height * cloud_dim_width * 3;
                    raw_float_vec =
                        allocator.realloc_as_array::<f32>(raw_float_vec, float_vec_len as usize);
                    transform_float_vec = allocator
                        .realloc_as_array::<f32>(transform_float_vec, float_vec_len as usize);
                    mean_filter_float_vec = allocator
                        .realloc_as_array::<f32>(mean_filter_float_vec, float_vec_len as usize);

                    info!("process_cloud: enable_mean_window: {}, filter: {:?}", enable_mean_window, filter);

                    if (enable_mean_window) {
                        pointcloud_clip(
                            cloud_float_vec.as_ptr() as *mut _,
                            cloud_height_width[0] as u64,
                            cloud_height_width[1] as u64,
                            mean_filter_float_vec.as_mut_ptr(),
                            filter.filter_height_min,
                            filter.filter_height_max,
                            filter.filter_width_min,
                            filter.filter_width_max,
                        );

                        pointcloud_process::perception::pointcloud_mean_filter(
                            mean_filter_float_vec.as_mut_ptr(),
                            float_vec_len / 3,
                            raw_float_vec.as_mut_ptr(),
                            &mut mean_window_filter_count,
                            mean_window_jump_max,
                        );
                        mean_window_filter_count_reach =
                            mean_window_filter_count >= mean_window_filter_max;

                        if mean_window_filter_count_reach {
                            mean_window_filter_count = 0;
                        }
                    } else {
                        pointcloud_clip(
                            cloud_float_vec.as_ptr() as *mut _,
                            cloud_height_width[0] as u64,
                            cloud_height_width[1] as u64,
                            raw_float_vec.as_mut_ptr(),
                            filter.filter_height_min,
                            filter.filter_height_max,
                            filter.filter_width_min,
                            filter.filter_width_max,
                        );
                        mean_window_filter_count_reach = true;
                    }
                }

                if (mean_window_filter_count_reach) {
                    mean_window_filter_count = 0;
                    *new_cloud_ready.borrow_mut() = true;


                    let mut extrinsic_enable = extrinsic.enable;
                    {
                        if (app_status_data_binding.task.success_count >2 ){
                            extrinsic_enable = false;
                        }
                    }


                    if(extrinsic_enable){
                        pointcloud_transform(
                            raw_float_vec.as_mut_ptr(),
                            (raw_float_vec.len() as u64) / 3,
                            transform_float_vec.as_mut_ptr(),
                            extrinsic.pose.tx,
                            extrinsic.pose.ty,
                            extrinsic.pose.tz,
                            extrinsic.pose.roll,
                            extrinsic.pose.pitch,
                            extrinsic.pose.yaw,
                        );

                        pallet_detector_handler.borrow_mut().set_input(
                            transform_float_vec.as_mut_ptr(),
                            cloud_dim_height,
                            cloud_dim_width,
                            extrinsic.pose.tx,
                            extrinsic.pose.ty,
                            extrinsic.pose.tz,
                        );
                    }else{

                        pallet_detector_handler.borrow_mut().set_input(
                            raw_float_vec.as_mut_ptr(),
                            cloud_dim_height,
                            cloud_dim_width,
                            0.0,
                            0.0,
                            0.0,
                        );
                    }


                }

                true
            },
            100.0,
        );
    }

    {
        let dds_handler = dds_handler.clone();
        let mut pallet_detector_handler = pallet_detector_handler.clone();

        let start_detect_cmd = start_detect_cmd.clone();
        let new_cloud_ready = new_cloud_ready.clone();
        let app_status_data = app_status_data.clone();

        task_manager.add(
            "detect",
            move || {
                let mut app_status_data_binding = app_status_data.borrow_mut();

                // info!("run_detector: start_detect_cmd: {}, new_cloud_ready: {}",*start_detect_cmd.borrow(),*new_cloud_ready.borrow() );

                let mut detect_ok = false;

                let new_cloud_ready_condition = *new_cloud_ready.borrow();
                // *new_cloud_ready.borrow_mut() = false;

                if *start_detect_cmd.borrow() && app_status_data_binding.detect_task.retry_count > 0 && *new_cloud_ready.borrow() {
                    *new_cloud_ready.borrow_mut() = false;
                    let mut binding = pallet_detector_handler.borrow_mut();
                    binding.filter_ground(1);
                    binding.filter_vertical(1);
                    binding.filter_pallet(1);

                    let pallet_buffer = binding.get_pallet(0);
                    info!("pallet_buffer_len: {}",pallet_buffer.len());
                    app_status_data_binding.task.pallet_result.clear();
                    if (pallet_buffer.len() > 0) {

                        for p in pallet_buffer.iter(){
                            app_status_data_binding.task.pallet_result.push(io_message::PalletInfo {
                                pallet: io_message::PalletPose {
                                    tx: p.tx,
                                    ty: p.ty,
                                    tz: p.tz,
                                    roll: p.roll,
                                    pitch: p.pitch,
                                    yaw: p.yaw,
                                },
                                confidence: p.confidence,
                                info: p.info,
                            });

                        }
                        app_status_data_binding.task.success_count += 1;
                        app_status_data_binding.detect_task.retry_count = 0;
                        detect_ok = true;
                    }else{
                        app_status_data_binding.task.fail_count += 1;
                        app_status_data_binding.detect_task.retry_count -=1;
                    }


                    if (app_status_data_binding.detect_task.retry_count == 0){
                        *start_detect_cmd.borrow_mut() = false;
                        app_status_data_binding.task.process = "RunningDetector".to_string();
                        if(detect_ok){
                            app_status_data_binding.task.process_result = "Success".to_string();
                            let task_id = app_status_data_binding.detect_task.task_id.clone();
                            app_status_data_binding.task.last_success_task_id = task_id;

                        }else{
                            app_status_data_binding.task.process_result = "Fail".to_string();
                            let task_id = app_status_data_binding.detect_task.task_id.clone();
                            app_status_data_binding.task.last_fail_task_id = task_id;
                        }
                    }
                }
                true
            },
            100.0,
        );
    }

    {
        let dds_handler = dds_handler.clone();
        current_time = get_now_local();
        last_current_time = current_time;
        let cloud_msg_count = cloud_msg_count.clone();
        let app_status_data = app_status_data.clone();

        task_manager.add(
            "update_status",
            move || {


                current_time = get_now_local();
                let mut app_status_data_binding = app_status_data.borrow_mut();
                app_status_data_binding.statistic.current_time = format!("{}", current_time);
                app_status_data_binding.statistic.up_duration =
                    format!("{}", current_time - boot_up_time);
                cycle_time = current_time - last_current_time;
                last_current_time = current_time;
                max_cycle_time = max_cycle_time.max(cycle_time);
                app_status_data_binding.statistic.cycle_time = format!("{}", cycle_time);
                app_status_data_binding.statistic.max_cycle_time = format!("{}", max_cycle_time);
                app_status_data_binding.statistic.cloud_message_count = *cloud_msg_count.borrow();

                let app_status_data_str = toml::to_string(&*app_status_data_binding).unwrap();

                send_status
                    .borrow_mut()
                    .set_data(app_status_data_str.as_str());
                {
                    let mut send_result_poses = send_result.get_mut_data();
                    send_result_poses[0].position.x = send_result_poses[0].position.x + 1.0;
                }
                // dds_handler.borrow_mut().write_data(
                //     c"detector_result_pub",
                //     &mut [*send_result.get_ptr() as *mut std::os::raw::c_void],
                // );
                info!("send app_status_data_str: {}", app_status_data_str.as_str());

                dds_handler.borrow_mut().write_data(
                    c"detector_status_pub",
                    &mut [*send_status.borrow_mut().get_ptr() as *mut std::os::raw::c_void],
                );


                true
            },
            100.0,
        );
    }

    while signal.is_run() {
        task_manager.run();
    }
}
