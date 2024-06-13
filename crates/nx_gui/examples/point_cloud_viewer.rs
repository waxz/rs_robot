use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::Thread;
use nx_message_center::base::{common_message, message_handler, tiny_alloc,pointcloud_process};

use nx_common::common::math::Point3f;

use itertools::{izip, Itertools};
use nx_common::common::types::UnsafeSender;
use nx_message_center::base::common_message::shared::PointCloud2;

use std::f32::consts::{PI, TAU};
use std::f64::consts::FRAC_PI_2;
use std::ffi::c_void;
use std::io::Write;
use std::ops::{BitAnd, Deref};
use std::{fs, slice};
use time::{OffsetDateTime, UtcOffset};

use bevy::app::{App, Startup, Update};
use bevy::log::{debug, debug_once, error, info, warn};
use bevy::math::quat;
use bevy::prelude::{
    ButtonInput, Color, Commands, Gizmos, KeyCode, MouseButton, PositionType, Quat, Query, Res,
    ResMut, Resource, Style, TextBundle, TextStyle, Transform, Val, Vec2, Vec3,
};
use bevy_egui::egui::{ComboBox, ScrollArea, Slider};
use bevy_egui::{egui, EguiContexts};
use bevy_mod_raycast::CursorRay;

use bevy::DefaultPlugins;
use rand::{thread_rng, Rng};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;
use std::sync::{Arc, Mutex, MutexGuard};
// argument
use clap::Parser;
use nx_gui::app_builder::{create_bevy_app, CameraFocusRay};
use nx_gui::shaders::{setup_shaders_render, InstanceMaterialData, ShaderResConfig};
use nx_message_center::base::pointcloud_process::perception::{
    pointcloud_clip, pointcloud_transform,
};
use serde::{Deserialize, Serialize};

//-------------

const TA_BUFFER_SIZE: usize = 100000000;
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
}

//------------------

#[derive(Copy, Clone)]
struct DetectionOperator{

    // 0: do nothing
    // 1: filter ground
    // 2: 1 + filter cargo
    pub enable : bool,
    pub detection_operator_mode: u32,
    pub detection_operator_ret : u32,
    pub filter_ground:GroundFilter,
    pub filter_vertical:VerticalFilter,
    pub filter_pallet: PalletFilter,

}
struct SimpleState
{
    pub enable_dds_update: bool,
    pub dds_msg_count: usize,
    // select point scale need be larger
    pub point_default_scale: f32,
    pub point_highlight_selected_scale: f32,
    pub point_highlight_unselected_scale: f32,
    pub tool_ray_radius: f32,
    pub tool_ray_distance: f32,

    pub start_select: bool,
    pub reset_vertex: bool,
    pub add_selected_to_vec: bool,

    pub vertex_color_hsv_ratio: f32,

    // pub exit: bool,

    pub two_cluster_distance_index: [usize; 2],
    pub filter: CloudFilterConfig,
    pub operator_mode: u32,
    pub operator_run: bool,
    pub calibration_enable :bool,

    pub detection_operator : DetectionOperator,

}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct Pose
{
    tx: f32,
    ty: f32,
    tz: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct CloudDimConfig
{
    height: usize,
    width: usize,
}
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct CloudFilterConfig
{
    enable_mean_window: bool,
    mean_window_len: u32,

    enable_dim: bool,//Option<bool>,
    filter_height_min: u64,
    filter_height_max: u64,
    filter_width_min: u64,
    filter_width_max: u64,

    enable_range: bool,//Option<bool>,
    filter_x_min: f32,
    filter_y_min: f32,
    filter_z_min: f32,
    filter_x_max: f32,
    filter_y_max: f32,
    filter_z_max: f32,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct GroundFilter{
    output_mode:u32,

    //
    search_direction :u64,
    // initial ground region
    init_ground_height_min : u64,
    init_ground_height_max : u64,
    init_ground_width_min : u64,
    init_ground_width_max : u64,

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
    ray_height_min : u64,
    ray_height_max : u64,
    ray_width_min : u64,
    ray_width_max : u64,


    // detect vertical plane, pallet or cargo or wall, or outlier shadow
    scan_width_step : u64,
    // find stable center[cx,cy,cz],reject outlier

    // adaptive thresh
    adaptive_x_min: f32,
    adaptive_x_max: f32,
    adaptive_y_min: f32,
    adaptive_y_max: f32,
    adaptive_z_min: f32,
    adaptive_z_max: f32,
    far_uncertain_z_max :f32,
    far_uncertain_x_change_min : f32,
    far_uncertain_adaptive_z_max: f32,
    far_uncertain_row : i32

}
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]

struct PalletFilter{
    output_mode:u32,

    //
    search_direction :u64,

    //
    filter_pallet_row_high : i32,
    filter_pallet_row_low : i32,

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


struct VerticalFilter{
    output_mode:u32,

    //
    search_direction :u64,

    // dim limut
    // range limit
    init_center_height_min : u64,
    init_center_height_max : u64,
    init_center_width_min : u64,
    init_center_width_max : u64,
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
struct CloudConfig
{
    dim: CloudDimConfig,
    filter: CloudFilterConfig,
}
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct GuiConfig
{
    extrinsic: PoseMarker,
    cloud: CloudConfig,
    filter_ground: GroundFilter,
    filter_vertical: VerticalFilter,
    filter_pallet:PalletFilter,

}
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct AppConfig
{
    pallet_detector: GuiConfig,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct PoseMarker
{
    pub pose: Pose,
    pub enable: bool,
}
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
struct CalibrationSelect
{
    enable: bool,
    from_index: u64,
    to_index: u64,
    program: u64,
    weight: f32,
}

struct CloudFloatVecBuffer
{
    raw_buffer: UnsafeSender<*const f32>,
    transform_buffer: UnsafeSender<*const f32>,
    render_buffer: UnsafeSender<*const f32>,
    float_num: usize,
}
struct StaticSharedData
{
    pub cloud_buffer: Arc<Mutex<CloudFloatVecBuffer>>,
    pub cloud_dim: [usize; 2],
    pub state: Arc<Mutex<SimpleState>>,

    // col and row filter
    // x,y,z range filter
    // multiple pick group 0
    // multiple pick group 1
    // multiple pick group 2
    // multiple pick group 3
    pub tool_selected_points_index: Arc<Mutex<Vec<Vec<u64>>>>,
    pub marker_pose: Arc<Mutex<Vec<PoseMarker>>>,
    pub extrinsic: Arc<Mutex<PoseMarker>>,
    pub calib_select: Arc<Mutex<Vec<CalibrationSelect>>>,
}

static GLOBAL_DATA: OnceLock<StaticSharedData> = OnceLock::new();

fn main()
{
    let tz_offset = nx_common::common::time::get_timezone_offset();
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );
    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);

    println!("what's up");
    let args = Args::parse();
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

    let mut app_config: AppConfig; // = toml::from_str(&contents).unwrap();

    match toml::from_str::<AppConfig>(&contents) {
        Ok(t) => app_config = t,
        Err(e) => {
            println!("toml::from_str: {:?}", e);
            return;
        }
    }

    let cloud_dim = app_config.pallet_detector.cloud.dim;

    let mut raw_float_vec =
        allocator.alloc_as_array::<f32>((cloud_dim.width * cloud_dim.height * 3) as usize);
    let mut transform_float_vec =
        allocator.alloc_as_array::<f32>((cloud_dim.width * cloud_dim.height * 3) as usize);

    {
        let raw_float_vec_ptr = raw_float_vec.as_ptr();
        let transform_float_vec_ptr = transform_float_vec.as_ptr();
        let raw_float_vec_len = raw_float_vec.len();

        GLOBAL_DATA.get_or_init(move || {
            println!("set crate::GLOBAL_DATA::BUFFER");
            StaticSharedData {
                cloud_dim: [cloud_dim.height, cloud_dim.width],

                cloud_buffer: Arc::new(Mutex::new(crate::CloudFloatVecBuffer {
                    raw_buffer: UnsafeSender::new(raw_float_vec_ptr),
                    transform_buffer: UnsafeSender::new(transform_float_vec_ptr),
                    render_buffer: UnsafeSender::new(raw_float_vec_ptr),
                    float_num: raw_float_vec_len,
                })),
                tool_selected_points_index: Arc::new(Mutex::new(vec![])),
                state: Arc::new(Mutex::new(SimpleState {
                    enable_dds_update: true,
                    dds_msg_count: 0,
                    point_default_scale: 0.002,
                    point_highlight_selected_scale: 0.0035,
                    point_highlight_unselected_scale: 0.002,
                    tool_ray_radius: 0.005,
                    tool_ray_distance: 0.1,
                    start_select: false,
                    reset_vertex: false,
                    add_selected_to_vec: false,
                    vertex_color_hsv_ratio: 5.0,
                    // exit: false,
                    two_cluster_distance_index: [0; 2],
                    filter:  app_config.pallet_detector.cloud.filter,
                    operator_mode: 0,
                    operator_run: false,
                    calibration_enable: false,
                    detection_operator: DetectionOperator{
                        detection_operator_mode: 0,
                        enable: false,
                        filter_ground: app_config.pallet_detector.filter_ground,
                        detection_operator_ret: 0,
                        filter_vertical: app_config.pallet_detector.filter_vertical,
                        filter_pallet: app_config.pallet_detector.filter_pallet,
                    },
                })),
                marker_pose: Arc::new(Mutex::new(vec![])),
                extrinsic: Arc::new(Mutex::new(app_config.pallet_detector.extrinsic)),
                calib_select: Arc::new(Mutex::new(vec![])),
            }
        });

        // GLOBAL_DATA.get_or_init(move || Arc::new(Mutex::new((    UnsafeSender::new(float_vec_ptr) , float_vec_len))) );
        println!("set crate::GLOBAL_DATA::BUFFER");
    }

    let mut dds_handler: message_handler::MessageHandler =
        message_handler::MessageHandler::new("dds");
    let ok = dds_handler.create(args.dds_config.as_str(), *allocator.cfg.get());

    if !ok{
        println!("exit");
        return;
    }
    let mut pallet_detector_handler = pointcloud_process::perception::PointcloudPalletDetector::new();
    let ok = pallet_detector_handler.create(args.dds_config.as_str(), *allocator.cfg.get());

    if !ok{
        println!("exit");
        return;
    }

    let mut signal = SignalHandler::default();

    let mut app = create_bevy_app([0.5, 0.5, 0.5], [500, 100]);

    setup_shaders_render(&mut app, set_shader_render_demo);

    app.add_systems(Update, update_shader_render_demo);
    app.add_systems(Update, create_egui_windows);

    app.add_systems(Update, raycast_picker);
    app.add_systems(Startup, setup_help_info);
    app.add_systems(Update, plot_marker);

    let mut dds_thread = Thread::default();

    {
        let signal = signal.clone();
        dds_thread = Thread::new(move || {
            while signal.is_run() {
                let enable_dds_update = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .enable_dds_update;
                let mut dds_msg_count = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .dds_msg_count;
                let mut cloud_height_width = [0,0];

                let mut detection_operator = GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator;

                if enable_dds_update || dds_msg_count == 0 {
                    let recv_cloud = dds_handler.read_data(c"cloud_sub");
                    for m in recv_cloud.iter() {
                        dds_msg_count += 1;
                        let data: PointCloud2 = PointCloud2::from_ptr(*m);

                        let timestamp_u64 = data.get_stamp();
                        let stamp =
                            OffsetDateTime::from_unix_timestamp_nanos(timestamp_u64 as i128)
                                .unwrap();

                        let current_stamp = OffsetDateTime::now_utc();

                        let stamp_diff = current_stamp - stamp;

                        let stamp_diff_s = stamp_diff.as_seconds_f32();
                        let stamp_local = stamp.to_offset(tz_offset);
                        let stamp_valid = (stamp_diff_s.abs() < 1.0);

                        let frame_id = data.get_frame_id();
                        let cloud_float_vec = data.get_data();

                        cloud_height_width = data.get_height_width();

                        // println!(
                        //     "recv data [{:?}] at {:?},cloud_float_vec.len : {}",
                        //     frame_id,
                        //     stamp,
                        //     cloud_float_vec.len()
                        // );

                        let filter = GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter;

                        if filter.enable_dim {
                            let float_vec_len = (filter.filter_height_max
                                - filter.filter_height_min)
                                * (filter.filter_width_max - filter.filter_width_min)
                                * 3;
                            raw_float_vec = allocator
                                .realloc_as_array::<f32>(raw_float_vec, float_vec_len as usize);
                            transform_float_vec = allocator.realloc_as_array::<f32>(
                                transform_float_vec,
                                float_vec_len as usize,
                            );

                            // info!("clip cloud to float_vec_len {} ", float_vec_len);
                            // use nx_message_center::base::pointcloud_process::perception::pointcloud_clip;
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

                            {
                                GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().raw_buffer = UnsafeSender::new(raw_float_vec.as_ptr());
                                GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().float_num = raw_float_vec.len();


                                // *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap() =
                                //     CloudFloatVecBuffer {
                                //         raw_buffer: UnsafeSender::new(raw_float_vec.as_ptr()),
                                //         transform_buffer: UnsafeSender::new(
                                //             transform_float_vec.as_ptr(),
                                //         ),
                                //         render_buffer: UnsafeSender::new(transform_float_vec.as_ptr()),
                                //         float_num: raw_float_vec.len(),
                                //     };


                            }
                        } else {
                            raw_float_vec = allocator
                                .realloc_as_array::<f32>(raw_float_vec, cloud_float_vec.len());

                            transform_float_vec = allocator.realloc_as_array::<f32>(
                                transform_float_vec,
                                cloud_float_vec.len(),
                            );

                            unsafe {
                                std::ptr::copy(
                                    cloud_float_vec.as_ptr(),
                                    raw_float_vec.as_mut_ptr(),
                                    cloud_float_vec.len(),
                                );
                            }

                            {
                                GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().raw_buffer = UnsafeSender::new(raw_float_vec.as_ptr());
                                GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().float_num = raw_float_vec.len();

                                // *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap() =
                                //     CloudFloatVecBuffer {
                                //         raw_buffer: UnsafeSender::new(raw_float_vec.as_ptr()),
                                //         transform_buffer: UnsafeSender::new(
                                //             transform_float_vec.as_ptr(),
                                //         ),
                                //         render_buffer: UnsafeSender::new(transform_float_vec.as_ptr()),
                                //         float_num: raw_float_vec.len(),
                                //     };

                            }
                        }
                    }
                }
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .dds_msg_count = dds_msg_count;

                let extrinsic = *GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap();

                {
                    if dds_msg_count > 0 {
                        if let Ok(ref mut mutex) =
                            GLOBAL_DATA.get().unwrap().cloud_buffer.try_lock()
                        {
                            let CloudFloatVecBuffer {
                                raw_buffer,
                                transform_buffer,
                                render_buffer,
                                float_num,
                            } = **mutex;
                            pointcloud_transform(
                                *raw_buffer.get() as *mut _,
                                (float_num as u64) / 3,
                                *transform_buffer.get() as *mut _,
                                extrinsic.pose.tx,
                                extrinsic.pose.ty,
                                extrinsic.pose.tz,
                                extrinsic.pose.roll,
                                extrinsic.pose.pitch,
                                extrinsic.pose.yaw,
                            );
                        }
                    }
                }
                if(!detection_operator.enable)
                {

                    if (dds_msg_count > 0 && extrinsic.enable) {


                        let transform_buffer = GLOBAL_DATA
                            .get()
                            .unwrap()
                            .cloud_buffer
                            .lock()
                            .unwrap()
                            .transform_buffer;

                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .cloud_buffer
                            .lock()
                            .unwrap()
                            .render_buffer = transform_buffer;
                        // info!("render_buffer=transform_buffer extrinsic: {:?}", extrinsic);
                    } else {
                        let raw_buffer = GLOBAL_DATA
                            .get()
                            .unwrap()
                            .cloud_buffer
                            .lock()
                            .unwrap()
                            .raw_buffer;
                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .cloud_buffer
                            .lock()
                            .unwrap()
                            .render_buffer = raw_buffer;
                        // info!("render_buffer=raw_buffer")
                    }
                }


                if dds_msg_count > 0{


                    if detection_operator.enable{


                        let filter = GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter;

                        let [cloud_height, cloud_width] = if filter.enable_dim{
                            [(filter.filter_height_max
                                - filter.filter_height_min)
                                ,  (filter.filter_width_max - filter.filter_width_min)]
                        }else{
                            [cloud_height_width[0] as u64, cloud_height_width[1] as u64]
                        };
                        let [vx, vy,vz] = if extrinsic.enable{
                            [extrinsic.pose.tx,
                                extrinsic.pose.ty,
                                extrinsic.pose.tz]
                        }else{
                            [0.0,0.0,0.0]
                        };

                        let transform_buffer = GLOBAL_DATA
                            .get()
                            .unwrap()
                            .cloud_buffer
                            .lock()
                            .unwrap()
                            .transform_buffer;
                        if( detection_operator.detection_operator_mode  > 0){
                            // set input
                            pallet_detector_handler.set_input(*transform_buffer.get() as *mut _,cloud_height,cloud_width, vx, vy, vz );

                            // set ground init dim
                            pallet_detector_handler.set_ground_init_thresh( detection_operator.filter_ground.init_ground_cx_min,detection_operator.filter_ground.init_ground_cx_max,
                                                                            detection_operator.filter_ground.init_ground_cy_min,detection_operator.filter_ground.init_ground_cy_max,
                                                                            detection_operator.filter_ground.init_ground_cz_min,detection_operator.filter_ground.init_ground_cz_max,
                                                                            detection_operator.filter_ground.init_ground_nz_min

                            );
                            pallet_detector_handler.set_ground_adaptive_thresh(
                                detection_operator.filter_ground.adaptive_x_min, detection_operator.filter_ground.adaptive_x_max,
                                detection_operator.filter_ground.adaptive_y_min, detection_operator.filter_ground.adaptive_y_max,
                                detection_operator.filter_ground.adaptive_z_min, detection_operator.filter_ground.adaptive_z_max,

                            );

                            pallet_detector_handler.set_ground_uncertain_thresh(detection_operator.filter_ground.far_uncertain_z_max, detection_operator.filter_ground.far_uncertain_x_change_min, detection_operator.filter_ground.far_uncertain_adaptive_z_max, detection_operator.filter_ground.far_uncertain_row);

                            pallet_detector_handler.set_ground_init_dim(detection_operator.filter_ground.init_ground_height_min,detection_operator.filter_ground.init_ground_height_max,
                                                                        detection_operator.filter_ground.init_ground_width_min,detection_operator.filter_ground.init_ground_width_max);
                            // call filter
                            pallet_detector_handler.set_vertical_init_dim(detection_operator.filter_vertical.init_center_height_min,detection_operator.filter_vertical.init_center_height_max,
                                                                          detection_operator.filter_vertical.init_center_width_min, detection_operator.filter_vertical.init_center_width_max);

                            pallet_detector_handler.set_vertical_init_thresh(detection_operator.filter_vertical.init_center_cx_min,detection_operator.filter_vertical.init_center_cx_max,
                                                                             detection_operator.filter_vertical.init_center_cy_min,detection_operator.filter_vertical.init_center_cy_max,
                                                                             detection_operator.filter_vertical.init_center_cz_min,detection_operator.filter_vertical.init_center_cz_max,
                                                                             detection_operator.filter_vertical.init_center_jx_max,detection_operator.filter_vertical.init_center_jy_max,
                                                                             detection_operator.filter_vertical.init_center_jz_max
                            );
                            pallet_detector_handler.set_pallet_row(detection_operator.filter_pallet.filter_pallet_row_high,detection_operator.filter_pallet.filter_pallet_row_low );
                            pallet_detector_handler.set_pallet_thresh(detection_operator.filter_pallet.filter_pallet_x_min,detection_operator.filter_pallet.filter_pallet_x_max,
                                                                      detection_operator.filter_pallet.filter_pallet_y_min,detection_operator.filter_pallet.filter_pallet_y_max,
                                                                      detection_operator.filter_pallet.filter_pallet_z_min,detection_operator.filter_pallet.filter_pallet_z_max,
                                                                      detection_operator.filter_pallet.filter_pallet_jx,detection_operator.filter_pallet.filter_pallet_jy,
                                                                      detection_operator.filter_pallet.filter_pallet_jz
                            );
                        }


                        if detection_operator.detection_operator_mode == 1{
                            println!("detection_operator.filter_ground : {:?}",detection_operator.filter_ground);



                            if let Ok(ref mut mutex) =
                                GLOBAL_DATA.get().unwrap().cloud_buffer.try_lock()
                            {
                                let CloudFloatVecBuffer {
                                    raw_buffer,
                                    transform_buffer,
                                    ref mut render_buffer,
                                    ref mut float_num,
                                } = **mutex;

                                let mut ret = pallet_detector_handler.filter_ground(detection_operator.filter_ground.output_mode );
                                let (process_buffer,  process_float_num) = ret;

                                if ! process_buffer.is_null() && process_float_num > 0 {

                                     *render_buffer = UnsafeSender::new(process_buffer);

                                     *float_num = process_float_num as usize;
                                    println!("set cloud_buffer render_buffer to process_buffer ");
                                }
                            }
                        }


                        if detection_operator.detection_operator_mode == 2{
                            println!("detection_operator.filter_ground : {:?}",detection_operator.filter_ground);
                            println!("detection_operator.filter_vertical : {:?}",detection_operator.filter_vertical);



                            if let Ok(ref mut mutex) =
                                GLOBAL_DATA.get().unwrap().cloud_buffer.try_lock()
                            {
                                let CloudFloatVecBuffer {
                                    raw_buffer,
                                    transform_buffer,
                                    ref mut render_buffer,
                                    ref mut float_num,
                                } = **mutex;

                                let mut ret = pallet_detector_handler.filter_ground(detection_operator.filter_ground.output_mode );
                                ret = pallet_detector_handler.filter_vertical(detection_operator.filter_vertical.output_mode );

                                let (process_buffer,  process_float_num) = ret;

                                if ! process_buffer.is_null() && process_float_num > 0 {

                                    *render_buffer = UnsafeSender::new(process_buffer);

                                    *float_num = process_float_num as usize;
                                    println!("set cloud_buffer render_buffer to process_buffer ");
                                }
                            }
                        }


                        if detection_operator.detection_operator_mode == 3{
                            println!("detection_operator.filter_ground : {:?}",detection_operator.filter_ground);
                            println!("detection_operator.filter_vertical : {:?}",detection_operator.filter_vertical);



                            if let Ok(ref mut mutex) =
                                GLOBAL_DATA.get().unwrap().cloud_buffer.try_lock()
                            {
                                let CloudFloatVecBuffer {
                                    raw_buffer,
                                    transform_buffer,
                                    ref mut render_buffer,
                                    ref mut float_num,
                                } = **mutex;

                                let mut ret = pallet_detector_handler.filter_ground(detection_operator.filter_ground.output_mode );
                                ret = pallet_detector_handler.filter_vertical(detection_operator.filter_vertical.output_mode );

                                ret = pallet_detector_handler.filter_pallet(detection_operator.filter_pallet.output_mode);

                                let (process_buffer,  process_float_num) = ret;

                                if ! process_buffer.is_null() && process_float_num > 0 {

                                    *render_buffer = UnsafeSender::new(process_buffer);

                                    *float_num = process_float_num as usize;
                                    println!("set cloud_buffer render_buffer to process_buffer ");
                                }
                            }

                        }


                        // detection_operator.detection_operator_mode = 0;

                    }
                    GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator = detection_operator;

                }

                std::thread::sleep(std::time::Duration::from_millis(100));
            }

            // GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit = true;
            // if GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit {
            //     info!("send exit signal");
            // }
        });
    }

    app.run();

    signal.stop();

    app_config.pallet_detector.extrinsic = *GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap();
    app_config.pallet_detector.cloud.filter = GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter;

    app_config.pallet_detector.filter_ground =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator.filter_ground;
    app_config.pallet_detector.filter_vertical =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator.filter_vertical;
    app_config.pallet_detector.filter_pallet =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator.filter_pallet;

    let output_filename = "/tmp/gui_output.toml";

    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(output_filename)
        .unwrap();

    let toml_data = toml::to_string(&app_config).unwrap();
    file.write_all(&toml_data.to_string().as_bytes());
    println!(
        "app_config data : \n{} \noutput to : {}",
        toml_data, output_filename
    );
}

fn setup_help_info(mut commands: Commands)
{
    // example instructions
    commands.spawn(
        TextBundle::from_section(
            r##"
Created by ZHANGE.WEI
            "##,
            TextStyle {
                font_size: 20.,
                ..TextStyle::default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..Style::default()
        }),
    );
}

fn set_shader_render_demo(mut config: ResMut<ShaderResConfig>)
{
    let cloud_dim = GLOBAL_DATA.get().unwrap().cloud_dim;
    config.enable = true;
    config.static_point_num = cloud_dim[0] * cloud_dim[1];
    config.dynamic_point_num = 0;
    config.run_count = 0;
    info!("set_shader_render_demo: {:?}", config);
}

//https://www.cs.rit.edu/~ncs/color/t_convert.html
fn update_shader_render_demo(
    mut query: Query<&mut InstanceMaterialData>,
    mut config: ResMut<ShaderResConfig>,
)
{
    // info!("update_shader_render_demo : {:?}", config);

    if !config.enable || config.static_point_num == 0 || config.run_count == 0 {
        // info!("update_shader_render_demo skip : {:?}", config);

        return;
    }

    //-----
    let mut reset_vertex = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .reset_vertex;

    GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .reset_vertex = false;
    let vertex_color_hsv_ratio = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .vertex_color_hsv_ratio;

    let mut add_selected_to_vec = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .add_selected_to_vec;
    GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .add_selected_to_vec = false;

    let distance_ratio = 360.0 / vertex_color_hsv_ratio;

    let mut point_default_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_default_scale;
    let mut point_highlight_selected_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_highlight_selected_scale;

    let mut point_highlight_unselected_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_highlight_unselected_scale;

    let start_select = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .start_select;

    //----

    {
        if let Some(buffer) = GLOBAL_DATA.get() {
            if let Ok(ref mut mutex) = buffer.cloud_buffer.try_lock() {
                let CloudFloatVecBuffer {
                    raw_buffer,
                    transform_buffer,
                    render_buffer,
                    float_num,
                } = **mutex;

                let vec3_data: &[[f32; 3]] = unsafe {
                    slice::from_raw_parts_mut(*render_buffer.get() as *mut [f32; 3], float_num / 3)
                };
                let vec3_data_len = vec3_data.len();

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;
                    let data_len = data.len();
                    // println!(
                    //     "update cloud scale, vec3_data_len {:?}, data_len {:?}",
                    //     vec3_data_len, data_len
                    // );

                    if vec3_data_len > data_len {
                        warn!(
                            "vec3_data_len: [{}] > allocate data_len: [{}]",
                            vec3_data_len, data_len
                        );
                        return;
                    }
                    config.dynamic_point_num = vec3_data_len;

                    for i in 0..vec3_data_len {
                        let a = vec3_data[i];
                        let b = &mut data[i];
                        b.position.x = a[0];
                        b.position.y = a[1];
                        b.position.z = a[2];
                        let r = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                        let h = (r * distance_ratio).min(360.0);

                        let h = Color::hsl( h , 0.95, 0.7).as_rgba();

                        // let X = (a[0]/r).min(1.0);
                        // let Y = (a[1]/r).min(1.0);
                        // let Z = (a[2]/r).min(1.0);

                        // let R =  3.2404542*X - 1.5371385*Y - 0.4985314*Z;
                        // let G = -0.9692660*X + 1.8760108*Y + 0.0415560*Z;
                        // let B =  0.0556434*X - 0.2040259*Y + 1.0572252*Z;

                        b.color[0] = h.r();
                        b.color[1] = h.g();
                        b.color[2] = h.b();
                        // b.color[0] = R;
                        // b.color[1] = G;
                        // b.color[2] = B;

                        b.color[3] = h.a();
                        b.scale = point_default_scale;
                    }
                    for i in vec3_data_len..data_len {
                        data[i].scale = 0.0;
                    }

                    #[cfg(notuse)]
                    for (i, b) in data.iter_mut().enumerate() {
                        if (i < vec3_data_len) {
                            let a = vec3_data[i];
                            b.position.x = a[0];
                            b.position.y = a[1];
                            b.position.z = a[2];
                            let r = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();

                            let h = Color::hsl(r * distance_ratio, 0.5, 0.5).as_rgba();
                            b.color[0] = h.r();
                            b.color[1] = h.g();
                            b.color[2] = h.b();
                            b.color[3] = h.a();
                            b.scale = point_default_scale;
                        } else {
                            b.scale = 0.0;
                        }
                    }

                    if start_select {
                        for i in 0..vec3_data_len {
                            let a = vec3_data[i];
                            let b = &mut data[i];
                            b.scale = if (b.selected > 0) {
                                point_highlight_selected_scale
                            } else {
                                point_highlight_unselected_scale
                            }
                        }
                        #[cfg(notuse)]
                        for b in data.iter_mut() {
                            b.scale = if (b.selected > 0) {
                                point_highlight_selected_scale
                            } else {
                                point_highlight_unselected_scale
                            }
                        }

                        if add_selected_to_vec {
                            let index_vec: Vec<_> = data
                                .iter()
                                .enumerate()
                                .take(vec3_data_len)
                                .filter_map(
                                    |(i, b)| if b.selected > 0 { Some(i as u64) } else { None },
                                )
                                .collect();

                            // let index_vec:Vec<_> = data.iter().enumerate().take_while(|(i,b)|{b.selected > 0}).map(|(i,b)| i).collect();
                            // info!("index_vec len: {}, {:?}",index_vec.len(),index_vec);
                            if !index_vec.is_empty() {
                                GLOBAL_DATA
                                    .get()
                                    .unwrap()
                                    .tool_selected_points_index
                                    .lock()
                                    .unwrap()
                                    .push(index_vec);
                                reset_vertex = true;
                            }
                        }

                        if reset_vertex {
                            for b in data.iter_mut() {
                                b.selected = 0;
                            }
                        }
                    }
                }
            } else {
                // println!("GLOBAL_DATA try_lock failed");
            }
        } else {
            println!("GLOBAL_DATA empty");
        }
    }
}

fn raycast_picker(
    cursor_ray: Res<CursorRay>,
    mut query: Query<&mut InstanceMaterialData>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut camera_ray: ResMut<CameraFocusRay>,
    mut config: ResMut<ShaderResConfig>,
)
{
    // start select
    let vec3_len = config.dynamic_point_num;
    if vec3_len == 0 {
        return;
    }

    let start_select = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .start_select;
    let camera_focus = camera_ray.camera_focus;

    let tool_ray_radius = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .tool_ray_radius;

    let tool_ray_distance = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .tool_ray_distance;
    camera_ray.tool_ray_distance = tool_ray_distance;
    if !start_select {
        return;
    }

    {
        if let Some(ray) = **cursor_ray {
            if key_input.pressed(KeyCode::ControlLeft) && key_input.pressed(KeyCode::ShiftLeft)
            // && mouse_button_input.pressed(MouseButton::Left)
            {
                let origin = Point3f {
                    x: ray.origin.x,
                    y: ray.origin.y,
                    z: ray.origin.z,
                };
                let direction = Point3f {
                    x: ray.direction.x,
                    y: ray.direction.y,
                    z: ray.direction.z,
                };
                let camera_focus = Point3f {
                    x: camera_focus[0],
                    y: camera_focus[1],
                    z: camera_focus[2],
                };

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;

                    // let vec3_len = GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().float_num;

                    for i in 0..vec3_len {
                        let v = &mut data[i];
                        let entity = Point3f {
                            x: v.position.x,
                            y: v.position.y,
                            z: v.position.z,
                        };
                        let distance = Point3f::distance_to_line(origin, direction, entity);
                        let distance_to_origin = (entity - camera_focus).length();

                        if (distance < tool_ray_radius && distance_to_origin < tool_ray_distance) {
                            v.selected = 1;
                        }
                    }
                }
            }
        }
    }
}

fn plot_marker(mut gizmos: Gizmos)
{
    let marker_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();
    for i in 0..marker_len {
        let marker = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i];
        if marker.enable {
            let mut p1 = [1.0f32, 0.0, 0.0];

            let rect_len = 0.3;
            let axis_len = 0.3;

            let mut p2 = [0.0f32; 3];
            let mut plain_points: [f32; 12] = [
                0.0, -rect_len, -rect_len, 0.0, -rect_len, rect_len, 0.0, rect_len, rect_len, 0.0,
                rect_len, -rect_len,
            ];

            let mut axis = [axis_len, 0.0, 0.0, 0.0, rect_len, 0.0, 0.0, 0.0, rect_len];
            let mut axis_abs = [0.0f32; 9];

            let mut plain_points_abs = [0.0; 12];
            pointcloud_transform(
                p1.as_mut_ptr(),
                1,
                p2.as_mut_ptr(),
                0.0,
                0.0,
                0.0,
                marker.pose.roll,
                marker.pose.pitch,
                marker.pose.yaw,
            );
            pointcloud_transform(
                plain_points.as_mut_ptr(),
                4,
                plain_points_abs.as_mut_ptr(),
                marker.pose.tx,
                marker.pose.ty,
                marker.pose.tz,
                marker.pose.roll,
                marker.pose.pitch,
                marker.pose.yaw,
            );
            pointcloud_transform(
                axis.as_mut_ptr(),
                3,
                axis_abs.as_mut_ptr(),
                0.0,
                0.0,
                0.0,
                marker.pose.roll,
                marker.pose.pitch,
                marker.pose.yaw,
            );

            gizmos.ray(
                Vec3 {
                    x: marker.pose.tx,
                    y: marker.pose.ty,
                    z: marker.pose.tz,
                },
                Vec3::new(axis_abs[0], axis_abs[1], axis_abs[2]),
                Color::RED,
            );
            gizmos.ray(
                Vec3 {
                    x: marker.pose.tx,
                    y: marker.pose.ty,
                    z: marker.pose.tz,
                },
                Vec3::new(axis_abs[3], axis_abs[4], axis_abs[5]),
                Color::GREEN,
            );
            gizmos.ray(
                Vec3 {
                    x: marker.pose.tx,
                    y: marker.pose.ty,
                    z: marker.pose.tz,
                },
                Vec3::new(axis_abs[6], axis_abs[7], axis_abs[8]),
                Color::BLUE,
            );

            let [qw, qx, qy, qz] = nx_common::common::transform2d::euler_to_quaternion(
                marker.pose.roll as f64,
                marker.pose.pitch as f64,
                marker.pose.yaw as f64,
            );

            gizmos.ray(
                Vec3 {
                    x: marker.pose.tx,
                    y: marker.pose.ty,
                    z: marker.pose.tz,
                },
                Vec3 {
                    x: p2[0],
                    y: p2[1],
                    z: p2[2],
                },
                Color::Rgba {
                    red: 0.0,
                    green: 1.0,
                    blue: 0.0,
                    alpha: 0.5,
                },
            );

            gizmos.linestrip(
                [
                    Vec3 {
                        x: plain_points_abs[0],
                        y: plain_points_abs[1],
                        z: plain_points_abs[2],
                    },
                    Vec3 {
                        x: plain_points_abs[3],
                        y: plain_points_abs[4],
                        z: plain_points_abs[5],
                    },
                    Vec3 {
                        x: plain_points_abs[6],
                        y: plain_points_abs[7],
                        z: plain_points_abs[8],
                    },
                    Vec3 {
                        x: plain_points_abs[9],
                        y: plain_points_abs[10],
                        z: plain_points_abs[11],
                    },
                    Vec3 {
                        x: plain_points_abs[0],
                        y: plain_points_abs[1],
                        z: plain_points_abs[2],
                    },
                ],
                Color::Rgba {
                    red: 0.0,
                    green: 1.0,
                    blue: 0.0,
                    alpha: 0.5,
                },
            );
        }
    }
}

fn create_egui_windows(mut contexts: EguiContexts, mut gizmos: Gizmos)
{
    let ctx = contexts.ctx_mut();
    // Get current context style
    let mut style = (*ctx.style()).clone();

    style.visuals = egui::Visuals::light();

    // Mutate global style with above changes
    ctx.set_style(style);

    egui::Window::new("Selector")
        .resizable(true)
        .show(ctx, |ui| {
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.collapsing("Instructions", |ui| {
                        ui.label(
                            r##"
Esc : quit
KeyControl + KeyT : toogle camera orbit control
KeyControl + MouseButtonLeft : rotate camera
MouseButtonMiddle : zoom camera
MouseButtonRight : pan camera

Press KeyControl + KeyShift, move mouse to select points.
Only points within tool_ray_radius can be selected.
Click Add to save indexes.
Click Reser to clear current indexes.
                    "##,
                        );
                    });

                    ui.horizontal(|ui| {
                        let mut start_select = GLOBAL_DATA
                            .get()
                            .unwrap()
                            .state
                            .lock()
                            .unwrap()
                            .start_select;
                        ui.checkbox(&mut start_select, "Enable");
                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .state
                            .lock()
                            .unwrap()
                            .start_select = start_select;
                        if (start_select) {
                            GLOBAL_DATA
                                .get()
                                .unwrap()
                                .state
                                .lock()
                                .unwrap()
                                .enable_dds_update = false;
                        }

                        if ui.button("Reset").clicked() {
                            GLOBAL_DATA
                                .get()
                                .unwrap()
                                .state
                                .lock()
                                .unwrap()
                                .reset_vertex = true;
                        }

                        if ui.button("Add").clicked() {
                            GLOBAL_DATA
                                .get()
                                .unwrap()
                                .state
                                .lock()
                                .unwrap()
                                .add_selected_to_vec = true;
                            info!("Add is clicked");
                        }
                    });

                    let mut vertex_color_hsv_ratio = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .vertex_color_hsv_ratio;
                    ui.add(
                        egui::DragValue::new(&mut vertex_color_hsv_ratio)
                            .speed(0.1)
                            .clamp_range(1.0..=15.0)
                            .prefix("vertex_color_hsv_ratio: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .vertex_color_hsv_ratio = vertex_color_hsv_ratio;

                    //=====
                    let mut point_default_scale = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_default_scale;
                    ui.add(
                        egui::DragValue::new(&mut point_default_scale)
                            .speed(0.001)
                            .clamp_range(0.001..=0.2)
                            .prefix("scale_default: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_default_scale = point_default_scale;

                    //====
                    //=====
                    let mut point_highlight_selected_scale = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_highlight_selected_scale;
                    ui.add(
                        egui::DragValue::new(&mut point_highlight_selected_scale)
                            .speed(0.001)
                            .clamp_range(0.001..=0.2)
                            .prefix("scale_select: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_highlight_selected_scale = point_highlight_selected_scale;

                    //====
                    //=====
                    let mut point_highlight_unselected_scale = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_highlight_unselected_scale;
                    ui.add(
                        egui::DragValue::new(&mut point_highlight_unselected_scale)
                            .speed(0.001)
                            .clamp_range(0.001..=0.2)
                            .prefix("scale_unselect: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .point_highlight_unselected_scale = point_highlight_unselected_scale;

                    //====
                    let mut tool_ray_radius = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .tool_ray_radius;
                    ui.add(
                        egui::DragValue::new(&mut tool_ray_radius)
                            .speed(0.001)
                            .clamp_range(0.001..=0.1)
                            .prefix("tool_ray_radius: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .tool_ray_radius = tool_ray_radius;

                    //==tool_ray_distance
                    let mut tool_ray_distance = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .tool_ray_distance;
                    ui.add(
                        egui::DragValue::new(&mut tool_ray_distance)
                            .speed(0.001)
                            .clamp_range(0.001..=0.9)
                            .prefix("tool_ray_distance: "),
                    );
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .tool_ray_distance = tool_ray_distance;
                });
            });
        });

    egui::Window::new("IndexViewer")
        .resizable(true)
        .show(ctx, |ui| {
            let mut tool_selected_points_index_len = GLOBAL_DATA
                .get()
                .unwrap()
                .tool_selected_points_index
                .lock()
                .unwrap()
                .len();
            ui.label(format!("Count: {}", tool_selected_points_index_len));

            if tool_selected_points_index_len > 0{
                if ui.button("Pop").clicked(){
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .tool_selected_points_index.lock().unwrap().pop();
                    tool_selected_points_index_len -=1;
                }
            }

            for i in 0..tool_selected_points_index_len {
                ui.push_id(i, |ui| {
                    ui.collapsing("index", |ui| {
                        ScrollArea::vertical().show(ui, |ui| {
                            // Add a lot of widgets here.
                            ui.vertical(|ui| {
                                for j in GLOBAL_DATA
                                    .get()
                                    .unwrap()
                                    .tool_selected_points_index
                                    .lock()
                                    .unwrap()[i]
                                    .iter()
                                {
                                    ui.label(format!("{}", j,));
                                }
                            });
                        });
                    }); // this is fine!
                });
            }
        });

    egui::Window::new("DataReader")
        .resizable(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                let mut enable_dds_update = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .enable_dds_update;
                ui.checkbox(&mut enable_dds_update, "Enable");
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .enable_dds_update = enable_dds_update;

                let dds_msg_count = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .dds_msg_count;
                ui.label(format!("Count: {}", dds_msg_count));
            });
        });
    egui::Window::new("Marker").resizable(true).show(ctx, |ui| {
        if ui.button("Add").clicked() {
            GLOBAL_DATA
                .get()
                .unwrap()
                .marker_pose
                .lock()
                .unwrap()
                .push(PoseMarker {
                    pose: Pose {
                        tx: 0.0,
                        ty: 0.0,
                        tz: 0.0,
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                    },
                    enable: false,
                });
        }
        let marker_pose_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();
        for i in 0..marker_pose_len {
            ui.horizontal(|ui| {
                let mut marker = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i];
                ui.checkbox(&mut marker.enable, "Enable");

                ui.add(
                    egui::DragValue::new(&mut marker.pose.tx)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("tx: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.ty)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("ty: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.tz)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("tz: "),
                );

                ui.add(
                    egui::DragValue::new(&mut marker.pose.roll)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("roll: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.pitch)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("pitch: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.yaw)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("yaw: "),
                );

                GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i] = marker;
            });
        }
    });

    egui::Window::new("Extrinsic")
        .resizable(true)
        .show(ctx, |ui| {
            let mut extrinsic = GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap().clone();

            ui.checkbox(&mut extrinsic.enable, "Enable extrinsic");

            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.tx)
                    .speed(0.001)
                    .clamp_range(-5.0..=5.0)
                    .prefix("tx: "),
            );

            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.ty)
                    .speed(0.001)
                    .clamp_range(-5.0..=5.0)
                    .prefix("ty: "),
            );
            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.tz)
                    .speed(0.001)
                    .clamp_range(-5.0..=5.0)
                    .prefix("tz: "),
            );
            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.roll)
                    .speed(0.00001)
                    .clamp_range(-TAU..=TAU)
                    .prefix("roll: "),
            );
            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.pitch)
                    .speed(0.00001)
                    .clamp_range(-TAU..=TAU)
                    .prefix("pitch: "),
            );
            ui.add(
                egui::DragValue::new(&mut extrinsic.pose.yaw)
                    .speed(0.00001)
                    .clamp_range(-TAU..=TAU)
                    .prefix("yaw: "),
            );

            // get cluster num and marker num
            let marker_pose_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();
            let tool_selected_points_index_len = GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap().len();

            ui.separator();

            if !extrinsic.enable
                && marker_pose_len >0
                && tool_selected_points_index_len > 0

            {

                if ui.button("Add").clicked() {
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .calib_select
                        .lock()
                        .unwrap()
                        .push(CalibrationSelect {
                            enable: false,
                            from_index: 0,
                            to_index: 0,
                            program: 0,
                            weight: 0.0,
                        });
                }

                let mut has_calib_select_enable = false;
                let calib_len = GLOBAL_DATA.get().unwrap().calib_select.lock().unwrap().len();
                for i in 0..calib_len{
                    ui.push_id(i,|ui|{

                        let mut calib = GLOBAL_DATA.get().unwrap().calib_select.lock().unwrap()[i];


                        ui.checkbox(&mut calib.enable, "Enable");
                        if calib.enable{
                            has_calib_select_enable = true;
                        }

                        ComboBox::from_label("from_index")
                            .selected_text(calib.from_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..tool_selected_points_index_len as u64{
                                    ui.selectable_value(&mut calib.from_index, i, i.to_string());
                                }
                            });

                        ComboBox::from_label("to_index")
                            .selected_text(calib.to_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..marker_pose_len as u64{
                                    ui.selectable_value(&mut calib.to_index, i, i.to_string());
                                }
                            });



                        GLOBAL_DATA.get().unwrap().calib_select.lock().unwrap()[i] = calib;

                    });
                }

                let mut calibration_enable = GLOBAL_DATA.get().unwrap().state.lock().unwrap().calibration_enable;

                if has_calib_select_enable{
                    ui.checkbox(&mut calibration_enable, "Enable");

                }else{
                    calibration_enable = false;
                }
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().calibration_enable = calibration_enable;

                if calibration_enable{



                    //------------
                    if let Ok(ref mut mutex) =
                        GLOBAL_DATA.get().unwrap().cloud_buffer.try_lock()
                    {
                        let CloudFloatVecBuffer {
                            raw_buffer,
                            transform_buffer,
                            render_buffer,
                            float_num,
                        } = **mutex;


                        let calibration_param:Vec<nx_message_center::base::pointcloud_process::perception::CalibrationParam> = GLOBAL_DATA.get().unwrap().calib_select.lock().unwrap().iter().map(|p|{
                            let source_ptr = GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap()[p.from_index as usize].as_mut_ptr();
                            let source_len = GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap()[p.from_index as usize].len() as u64;

                            let target = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[p.to_index as usize];
                            nx_message_center::base::pointcloud_process::perception::CalibrationParam{
                                index_buffer: source_ptr,
                                index_num: source_len,
                                program: p.program,
                                weight: p.weight,
                                target_tx: target.pose.tx,
                                target_ty:  target.pose.ty,
                                target_tz:  target.pose.tz,
                                target_roll:  target.pose.roll,
                                target_pitch:  target.pose.pitch,
                                target_yaw:  target.pose.yaw,
                            }
                        }).collect();

                        let init_tx:f32 = 0.0;
                        let init_ty:f32 = 0.0;
                        let init_tz:f32 = 0.0;
                        let init_roll:f32 = 0.0;
                        let init_pitch:f32 = 0.0;
                        let init_yaw:f32 = 0.0;

                        let mut  solve_tx:f32 = 0.0;
                        let mut  solve_ty:f32 = 0.0;
                        let mut  solve_tz:f32 = 0.0;
                        let mut  solve_roll:f32 = 0.0;
                        let mut  solve_pitch:f32 = 0.0;
                        let mut  solve_yaw:f32 = 0.0;



                        let solve_result = nx_message_center::base::pointcloud_process::perception::pointcloud_calib(*raw_buffer.get() as *mut _, (float_num as u64) / 3, &calibration_param, init_tx, init_ty, init_tz, init_roll, init_pitch, init_yaw,
                                                                                                  &mut solve_tx, &mut solve_ty, &mut solve_tz, &mut solve_roll,  &mut solve_pitch, &mut solve_yaw);

                        ui.label(format!("pointcloud_calib\n solve_result: {}, input: [{}, {}, {}, {}, {}, {}]\n output: [{}, {}, {}, {}, {}, {}]",
                            solve_result,
                                         init_tx, init_ty, init_tz, init_roll, init_pitch, init_yaw,
                                         solve_tx, solve_ty, solve_tz, solve_roll,  solve_pitch, solve_yaw
                        ));

                    }


                    //-------------






                }

            }

            *GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap() = extrinsic;
        });

    egui::Window::new("Filter").resizable(true).show(ctx, |ui| {
        let [height, width] = GLOBAL_DATA.get().unwrap().cloud_dim;
        let mut filter = GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter;

        //=====

        ui.add(
            Slider::new(&mut filter.filter_height_min, 0..=height as u64).text("filter_height_min"),
        );

        ui.add(
            Slider::new(
                &mut filter.filter_height_max,
                filter.filter_height_min..=height as u64,
            )
            .text("filter_height_max"),
        );

        ui.add(
            Slider::new(&mut filter.filter_width_min, 0..=width as u64).text("filter_width_min"),
        );

        ui.add(
            Slider::new(
                &mut filter.filter_width_max,
                filter.filter_width_min..=width as u64,
            )
            .text("filter_width_max"),
        );

        ui.add(Slider::new(&mut filter.filter_x_min, -5.0..=5.0).text("filter_x_min"));
        ui.add(
            Slider::new(&mut filter.filter_x_max, filter.filter_x_min..=5.0).text("filter_x_max"),
        );
        ui.add(Slider::new(&mut filter.filter_y_min, -5.0..=5.0).text("filter_y_min"));
        ui.add(
            Slider::new(&mut filter.filter_y_max, filter.filter_y_min..=5.0).text("filter_y_max"),
        );
        ui.add(Slider::new(&mut filter.filter_z_min, -5.0..=5.0).text("filter_z_min"));
        ui.add(
            Slider::new(&mut filter.filter_z_max, filter.filter_z_min..=5.0).text("filter_z_max"),
        );

        let dim_ok = (filter.filter_width_min < filter.filter_width_max
            && filter.filter_height_min < filter.filter_height_max);
        let range_ok = (filter.filter_x_min < filter.filter_x_max
            && filter.filter_y_min < filter.filter_y_max
            && filter.filter_z_min < filter.filter_z_max);

        if dim_ok {
            // let mut enable = filter.enable_dim;
            // let mut enable = enable;//.unwrap();
            ui.checkbox(&mut filter.enable_dim, "Enable dim filter");

            // filter.enable_dim = enable;//Some(enable);
        } else {
            filter.enable_dim = false;//Some(false);
        }
        if range_ok {
            // let mut enable = filter.enable_range;
            // let mut enable = enable.unwrap();
            ui.checkbox(&mut filter.enable_range, "Enable range filter");

            // filter.enable_range =enable;// Some(enable);
        } else {
            filter.enable_range = false;//Some(false);
        }

        GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter = filter;
    });
    egui::Window::new("Detection")
        .resizable(true)
        .show(ctx, |ui|{

            // enable
            let [cloud_height, cloud_width] = GLOBAL_DATA.get().unwrap().cloud_dim;

            let mut detection_operator = GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator;
            // program:
            // 0 nothing
            // 1 filter ground
                // init_ground_range
                // init_ground_thresh
                // output_mode
            ui.checkbox(&mut detection_operator.enable, "Enable detector");
            ui.add(Slider::new(&mut detection_operator.detection_operator_mode, 0..=10 as u32).text("detection_operator_mode"));

            if detection_operator.enable{
            }else{
                detection_operator.detection_operator_mode = 0;
            }

            ui.separator();

            if detection_operator.detection_operator_mode >= 1{

                ui.label("filter_ground");

                ui.collapsing("filter_ground", |ui| {
                    ScrollArea::vertical().show(ui, |ui| {
                        // Add a lot of widgets here.
                        ui.vertical(|ui| {


                            {
                                ui.add(Slider::new(&mut detection_operator.filter_ground.output_mode, 0..=10 as u32).text("output_mode"));


                                //
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_height_min, 0..=cloud_height as u64).text("init_ground_height_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_height_max, detection_operator.filter_ground.init_ground_height_min..=cloud_height as u64).text("init_ground_height_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_width_min, 0..=cloud_width as u64).text("init_ground_width_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_width_max, detection_operator.filter_ground.init_ground_width_min..=cloud_width as u64).text("init_ground_width_max"));


                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cx_min, -3.0..=3.0 as f32).text("init_ground_cx_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cx_max, detection_operator.filter_ground.init_ground_cx_min..=3.0 as f32).text("init_ground_cx_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cy_min, -3.0..=3.0 as f32).text("init_ground_cy_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cy_max, detection_operator.filter_ground.init_ground_cy_min..=3.0 as f32).text("init_ground_cy_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cz_min, -3.0..=3.0 as f32).text("init_ground_cz_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_cz_max, detection_operator.filter_ground.init_ground_cz_min..=3.0 as f32).text("init_ground_cz_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.init_ground_nz_min, 0.5..=1.0 as f32).text("init_ground_nz_min"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_x_min, -0.5..=0.5 as f32).text("adaptive_x_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_x_max, detection_operator.filter_ground.adaptive_x_min..=0.5 as f32).text("adaptive_x_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_y_min, -0.5..=1.0 as f32).text("adaptive_y_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_y_max, detection_operator.filter_ground.adaptive_y_min..=0.5 as f32).text("adaptive_y_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_z_min, -0.5..=1.0 as f32).text("adaptive_z_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.adaptive_z_max, detection_operator.filter_ground.adaptive_z_min..=0.5 as f32).text("adaptive_z_max"));


                                ui.add(Slider::new(&mut detection_operator.filter_ground.far_uncertain_z_max, -0.1..=0.1 as f32).text("far_uncertain_z_max"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.far_uncertain_adaptive_z_max, -0.1..=0.1 as f32).text("far_uncertain_adaptive_z_max"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.far_uncertain_x_change_min, -0.1..=0.1 as f32).text("far_uncertain_x_change_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_ground.far_uncertain_row, -20..=20 ).text("far_uncertain_row"));


                            }



                        });
                    });
                }); // this is fine!




            }
            if detection_operator.detection_operator_mode >= 2{


                ui.collapsing("filter_vertical", |ui| {
                    ScrollArea::vertical().show(ui, |ui| {
                        // Add a lot of widgets here.
                        ui.vertical(|ui| {


                            {

                                ui.add(Slider::new(&mut detection_operator.filter_vertical.output_mode, 0..=10 as u32).text("output_mode"));


                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_height_min, 0..=cloud_height as u64).text("init_center_height_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_height_max, detection_operator.filter_vertical.init_center_height_min..=cloud_height as u64).text("init_center_height_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_width_min, 0..=cloud_width as u64).text("init_center_width_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_width_max, detection_operator.filter_vertical.init_center_width_min..=cloud_width as u64).text("init_center_width_max"));


                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cx_min, -3.0..=3.0 as f32).text("init_center_cx_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cx_max, detection_operator.filter_vertical.init_center_cx_min..=3.0 as f32).text("init_center_cx_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cy_min, -3.0..=3.0 as f32).text("init_center_cy_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cy_max, detection_operator.filter_vertical.init_center_cy_min..=3.0 as f32).text("init_center_cy_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cz_min, -3.0..=3.0 as f32).text("init_center_cz_min"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_cz_max, detection_operator.filter_vertical.init_center_cz_min..=3.0 as f32).text("init_center_cz_max"));

                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_jx_max, -0.2..=0.2 as f32).text("init_center_jx_max"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_jy_max, -0.2..=0.2 as f32).text("init_center_jy_max"));
                                ui.add(Slider::new(&mut detection_operator.filter_vertical.init_center_jz_max, -0.2..=0.2 as f32).text("init_center_jz_max"));


                            }



                        });
                    });
                }); // this is fine!


            }

            if detection_operator.detection_operator_mode >= 3{
                ui.collapsing("filter_pallet", |ui|{

                    ScrollArea::vertical().show(ui, |ui|{
                        ui.vertical(|ui|{

                            ui.add(Slider::new(&mut detection_operator.filter_pallet.output_mode, 0..=100 as u32).text("output_mode"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_row_low, -100..=100 as i32).text("filter_pallet_row_low"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_row_high, -100..=100 as i32).text("filter_pallet_row_high"));


                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_x_min, -0.5..=0.5 as f32).text("filter_pallet_x_min"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_x_max, detection_operator.filter_pallet.filter_pallet_x_min..=0.5 as f32).text("filter_pallet_x_max"));

                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_y_min, -1.5..=1.5 as f32).text("filter_pallet_y_min"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_y_max, detection_operator.filter_pallet.filter_pallet_y_min..=1.5 as f32).text("filter_pallet_y_max"));

                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_z_min, -0.5..=0.5 as f32).text("filter_pallet_z_min"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_z_max, detection_operator.filter_pallet.filter_pallet_z_min..=0.5 as f32).text("filter_pallet_z_max"));


                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_jx, -0.2..=0.2 as f32).text("filter_pallet_jx"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_jy, -0.2..=0.2 as f32).text("filter_pallet_jy"));
                            ui.add(Slider::new(&mut detection_operator.filter_pallet.filter_pallet_jz, -0.2..=0.2 as f32).text("filter_pallet_jz"));


                        });
                    });


                });
            }


            GLOBAL_DATA.get().unwrap().state.lock().unwrap().detection_operator = detection_operator;


            // program return

            

        });

    egui::Window::new("Operator")
        .resizable(true)
        .show(ctx, |ui| {

            ui.collapsing("Instructions", |ui| {
                ui.label("Choose target and operator");
            });

            let tool_selected_points_index_len = GLOBAL_DATA
                .get()
                .unwrap()
                .tool_selected_points_index
                .lock()
                .unwrap()
                .len();


            let marker_pose_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();

            if (tool_selected_points_index_len == 0){
                ui.label("Please select points with Selector!");

                return;
            }


            let mut operator_panel = GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode;

            ui.separator();
            ui.horizontal(|ui| {
                ui.selectable_value(&mut operator_panel,  1 , "cluster");
                ui.selectable_value(&mut operator_panel,  2 , "marker");

            });
            ui.separator();
            if (operator_panel !=  GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode  ){
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = false;;

                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .two_cluster_distance_index = [0, 0];
            }
            GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode = operator_panel;

            let mut operator_run = GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run;
            ui.checkbox(&mut operator_run, "operator_run");
            GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = operator_run;


            let [mut from_index, mut to_index] = GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .two_cluster_distance_index;



            match operator_panel{
                1 => {




                     {

                        ComboBox::from_label("from_index")
                            .selected_text(from_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..tool_selected_points_index_len {
                                    ui.selectable_value(&mut from_index, i, i.to_string());
                                }
                            });

                        ComboBox::from_label("to_index")
                            .selected_text(to_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..tool_selected_points_index_len {
                                    ui.selectable_value(&mut to_index, i, i.to_string());
                                }
                            });




                         if operator_run{
                             let mut from_cluster_cx = 0.0;
                             let mut from_cluster_cy = 0.0;
                             let mut from_cluster_cz = 0.0;

                             let mut from_cluster_nx = 0.0;
                             let mut from_cluster_ny = 0.0;
                             let mut from_cluster_nz = 0.0;
                             let mut from_cluster_nd = 0.0;

                             let mut to_cluster_cx = 0.0;
                             let mut to_cluster_cy = 0.0;
                             let mut to_cluster_cz = 0.0;

                             let mut to_cluster_nx = 0.0;
                             let mut to_cluster_ny = 0.0;
                             let mut to_cluster_nz = 0.0;
                             let mut to_cluster_nd = 0.0;

                             let viewerpoint = * GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap();

                             let [vx, vy, vz] = if viewerpoint.enable{
                                 [viewerpoint.pose.tx,viewerpoint.pose.ty,viewerpoint.pose.tz, ]
                             }else{
                                 [0.0;3]
                             };

                             //==
                             // if let Some(buffer) = GLOBAL_DATA.get()

                             {

                                 let CloudFloatVecBuffer { raw_buffer, transform_buffer, render_buffer, float_num } =
                                 *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap();

                                 {

                                     let mut index = &mut GLOBAL_DATA
                                         .get()
                                         .unwrap()
                                         .tool_selected_points_index
                                         .lock()
                                         .unwrap()[from_index];


                                     nx_message_center::base::pointcloud_process::perception::pointcloud_norm(*render_buffer.get() as *mut _, (float_num as u64 )/3, index.as_mut_ptr() , index.len() as u64, vx, vy,vz, &mut from_cluster_cx, &mut from_cluster_cy, &mut from_cluster_cz, &mut from_cluster_nx, &mut from_cluster_ny, &mut from_cluster_nz, &mut from_cluster_nd);

                                 }

                                 let from_center = Point3f{
                                     x: from_cluster_cx,
                                     y: from_cluster_cy,
                                     z: from_cluster_cz,
                                 };

                                 ui.label(format!("from_index :center: [{}, {}, {}], norm: [{}, {}, {}, {}]", from_cluster_cx, from_cluster_cy, from_cluster_cz, from_cluster_nx, from_cluster_ny, from_cluster_nz, from_cluster_nd));


                                 gizmos. ray(Vec3{
                                     x: from_cluster_cx,
                                     y: from_cluster_cy,
                                     z: from_cluster_cz,
                                 }, Vec3{
                                     x: from_cluster_nx * 0.1,
                                     y: from_cluster_ny* 0.1,
                                     z: from_cluster_nz* 0.1,
                                 }, Color::GREEN);

                                 if (from_index != to_index) {

                                     {

                                         let mut index = &mut GLOBAL_DATA
                                             .get()
                                             .unwrap()
                                             .tool_selected_points_index
                                             .lock()
                                             .unwrap()[to_index];


                                         nx_message_center::base::pointcloud_process::perception::pointcloud_norm(*render_buffer.get() as *mut _, (float_num as u64 )/3, index.as_mut_ptr() , index.len() as u64, vx, vy,vz, &mut to_cluster_cx, &mut to_cluster_cy, &mut to_cluster_cz, &mut to_cluster_nx, &mut to_cluster_ny, &mut to_cluster_nz, &mut to_cluster_nd);

                                     }


                                     let to_center = Point3f{
                                         x: to_cluster_cx,
                                         y: to_cluster_cy,
                                         z: to_cluster_cz,
                                     };
                                     let from_to_distance = (from_center - to_center).length();

                                     ui.label(format!("to_index :center: [{}, {}, {}], norm: [{}, {}, {}, {}]", to_cluster_cx, to_cluster_cy, to_cluster_cz, to_cluster_nx, to_cluster_ny, to_cluster_nz, to_cluster_nd));


                                     ui.label(format!("distance: {} m ", from_to_distance));


                                     gizmos.line(Vec3{
                                         x: from_cluster_cx,
                                         y: from_cluster_cy,
                                         z: from_cluster_cz,
                                     }, Vec3{
                                         x: to_cluster_cx,
                                         y: to_cluster_cy,
                                         z: to_cluster_cz,
                                     }, Color::YELLOW);

                                     gizmos. ray(Vec3{
                                         x: to_cluster_cx,
                                         y: to_cluster_cy,
                                         z: to_cluster_cz,
                                     }, Vec3{
                                         x: to_cluster_nx* 0.1,
                                         y: to_cluster_ny* 0.1,
                                         z: to_cluster_nz* 0.1,
                                     }, Color::GREEN);
                                 }




                             }

                                     //===



                         }

                    }
                },
                2 => {
                  if marker_pose_len  == 0 {
                      ui.label("Please set marker with Marker!");

                  }else{
                      ComboBox::from_label("from_index")
                          .selected_text(from_index.to_string())
                          .show_ui(ui, |ui| {
                              for i in 0..tool_selected_points_index_len {
                                  ui.selectable_value(&mut from_index, i, i.to_string());
                              }
                          });

                      ComboBox::from_label("to_index")
                          .selected_text(to_index.to_string())
                          .show_ui(ui, |ui| {
                              for i in 0..marker_pose_len {
                                  ui.selectable_value(&mut to_index, i, i.to_string());
                              }
                          });

                      let marker = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[to_index];

                      if operator_run{

                          let mut from_cluster_cx = 0.0;
                          let mut from_cluster_cy = 0.0;
                          let mut from_cluster_cz = 0.0;

                          let mut from_cluster_nx = 0.0;
                          let mut from_cluster_ny = 0.0;
                          let mut from_cluster_nz = 0.0;
                          let mut from_cluster_nd = 0.0;

                          let mut to_cluster_cx = 0.0;
                          let mut to_cluster_cy = 0.0;
                          let mut to_cluster_cz = 0.0;

                          let mut to_cluster_nx = 0.0;
                          let mut to_cluster_ny = 0.0;
                          let mut to_cluster_nz = 0.0;
                          let mut to_cluster_nd = 0.0;

                          let viewerpoint = * GLOBAL_DATA.get().unwrap().extrinsic.lock().unwrap();

                          let [vx, vy, vz] = if viewerpoint.enable{
                              [viewerpoint.pose.tx,viewerpoint.pose.ty,viewerpoint.pose.tz, ]
                          }else{
                              [0.0;3]
                          };

                          {

                              let CloudFloatVecBuffer { raw_buffer, transform_buffer, render_buffer, float_num } =
                                  *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap();

                              {

                                  let mut index = &mut GLOBAL_DATA
                                      .get()
                                      .unwrap()
                                      .tool_selected_points_index
                                      .lock()
                                      .unwrap()[from_index];


                                  nx_message_center::base::pointcloud_process::perception::pointcloud_norm(*render_buffer.get() as *mut _, (float_num as u64 )/3, index.as_mut_ptr() , index.len() as u64, vx, vy,vz, &mut from_cluster_cx, &mut from_cluster_cy, &mut from_cluster_cz, &mut from_cluster_nx, &mut from_cluster_ny, &mut from_cluster_nz, &mut from_cluster_nd);

                              }

                              let from_center = Point3f{
                                  x: from_cluster_cx,
                                  y: from_cluster_cy,
                                  z: from_cluster_cz,
                              };

                              ui.label(format!("from_index :center: [{}, {}, {}], norm: [{}, {}, {}, {}]", from_cluster_cx, from_cluster_cy, from_cluster_cz, from_cluster_nx, from_cluster_ny, from_cluster_nz, from_cluster_nd));


                              gizmos. ray(Vec3{
                                  x: from_cluster_cx,
                                  y: from_cluster_cy,
                                  z: from_cluster_cz,
                              }, Vec3{
                                  x: from_cluster_nx * 0.1,
                                  y: from_cluster_ny* 0.1,
                                  z: from_cluster_nz* 0.1,
                              }, Color::GREEN);

                          }

                          {
                              let mut cluster_center = [from_cluster_cx, from_cluster_cy, from_cluster_cz];
                              let mut cluster_center_in_marker = [from_cluster_cx, from_cluster_cy, from_cluster_cz];

                              let mut itx = 0.0;
                              let mut ity = 0.0;
                              let mut itz = 0.0;
                              let mut iroll = 0.0;
                              let mut ipitch = 0.0;
                              let mut iyaw = 0.0;

                              nx_message_center::base::pointcloud_process::perception::se3_inverse(
                                  marker.pose.tx,
                                  marker.pose.ty,
                                  marker.pose.tz,
                                  marker.pose.roll,
                                  marker.pose.pitch,
                                  marker.pose.yaw,
                                  &mut itx,
                                  &mut ity,
                                  &mut itz,
                                  &mut iroll,
                                  &mut ipitch,
                                  &mut iyaw,

                              );

                              let marker =
                              pointcloud_transform(
                                  cluster_center.as_mut_ptr(),
                                  1,
                                  cluster_center_in_marker.as_mut_ptr(),
                                  itx,
                                  ity,
                                  itz,
                                  iroll,
                                  ipitch,
                                  iyaw,
                              );

                              ui.label(format!("from_index in marker :center: [{}, {}, {}]", cluster_center_in_marker[0], cluster_center_in_marker[1], cluster_center_in_marker[2]));


                          }



                      }



                  }

                },
                3 => {

                },
                _ => {

                }
            }

            GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .two_cluster_distance_index = [from_index, to_index];






        });
}
