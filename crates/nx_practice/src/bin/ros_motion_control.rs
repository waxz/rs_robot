// std
use serde::Deserialize;
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};
use std::fs;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::ops::Deref;
use std::os::raw::c_void;
use std::process::Command;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use time::macros::format_description;
use time::{OffsetDateTime, UtcOffset};

// argument
use clap::Parser;
// use color_eyre::owo_colors::AnsiColors::Default;
use itertools::izip;

// logger
use logs_wheel::LogFileInitializer;
use rolling_file::{BasicRollingFileAppender, RollingConditionBasic};
use tracing::{error, event, info, instrument, span, trace, warn, Level};
use tracing_subscriber::fmt::format;
use tracing_subscriber::fmt::time::OffsetTime;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

use spin_sleep::LoopHelper;

// user code
use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::task::TaskManager;
use nx_common::common::thread::Thread;
use nx_common::common::time::{get_now_local, Time};
use nx_common::common::transform2d::{
    euler_to_maxtrix, quaternion_to_euler, yaw_to_quaternion, Transform2d,
};
use nx_robot::base::motion_controller::ControllerCommand::Twist;
use nx_robot::base::motion_controller::{
    ControllerCommand, RobotControl, RobotMotionControllerConfig, SingleSteeringMotionController,
};
use nx_robot::base::motion_planner::{
    RobotMotionPlanner, RobotMotionPlannerConfig, RobotMotionPlannerError, RobotMotionPlannerState,
};
use nx_robot::base::odometry::{OdometryCalculator, SingleSteering};
use nx_robot::base::path_planner::{RobotPathPlanner, RobotPathPlannerConfig};
use nx_robot::base::TwistState;
use nx_robot::simulation::motor::{PositionMotor, SpeedMotor};

use crate::shared::RequestPath;
use nx_ros::ros::ros_handler::RosHandler;
use nx_ros::ros::ros_message::shared::{HeaderString, Path, PoseStamped};
use nx_ros::ros::tiny_alloc;

static mut TA_BUFFER: [u8; 1024 * 1000] = [0; 1024 * 1000];

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_practice/src/bin/comm_config.toml"
    )]
    ros_config: String,
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_practice/src/bin/robot_config.toml"
    )]
    motion_config: String,

    #[arg(short, long, default_value = "/tmp/logs")]
    log_dir: String,

    #[arg(short, long, default_value_t = 50.0)]
    fps: f32,
}

#[derive(Debug, Deserialize)]
struct TaskConfig
{
    pub task_id: String,
    pub parameter: String,
    pub counter: Option<usize>,
}

impl Default for TaskConfig
{
    fn default() -> Self
    {
        Self {
            task_id: "".to_string(),
            parameter: "".to_string(),
            counter: None,
        }
    }
}

impl Clone for TaskConfig
{
    fn clone(&self) -> Self
    {
        Self {
            task_id: self.task_id.clone(),
            parameter: self.parameter.clone(),
            counter: self.counter.clone(),
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
struct RerunConfig
{
    pub enable: bool,
    pub clear: bool,
    pub addr: String,
    pub robot: [Vec<[f32; 2]>; 2],
    pub robot_control: Vec<[f32; 2]>,

    pub pallet: [Vec<[f32; 2]>; 2],
    pub path_arrow_len: f32,
    pub test_relative_start: [f64; 3],
}

#[derive(Debug, Deserialize, Clone)]
enum RosState
{
    INIT,
    OK,
    TF_TIMEOUT,
    ODOM_TIMEOUT,
    REQ_TIMEOUT,
    TF_JUMP_ERROR,
    REQ_ERROR,
}
// receive cmd_vel, publish odom tf
#[derive(Debug, Deserialize, Clone)]
struct SimSingleSteeringConfig
{
    pub enable: bool,
    pub forward_vel_max: f64,
    pub forward_acc: f64,
    pub rotate_angle_max: f64,
    pub rotate_vel: f64,
}

#[derive(Debug, Deserialize, Clone)]
struct RosConfig
{
    pub sim_config: SimSingleSteeringConfig,
    // map base_link tf timeout
    pub tf_timeout_s: f32,
    // /odom timeout
    pub odom_timeout_s: f32,
    // reuqest path, goal, command timeout
    pub request_timeout_s: f32,
    // if controller is in idle state, publish stop cmdvel or not
    pub controller_idle_pub_zero_cmd_vel: bool,
    // if controller is in fault state, publish stop cmdvel or not
    pub controller_fault_pub_zero_cmd_vel: bool,

    pub stop_bash: String,
}

#[derive(Debug, Deserialize, Clone)]
pub struct AppConfig
{
    rerun_config: RerunConfig,
    ros_config: RosConfig,
    robot_path_planner_config: RobotPathPlannerConfig,
    robot_motion_planner_config: RobotMotionPlannerConfig,
    robot_motion_controller_config: RobotMotionControllerConfig,
}

mod basic
{
    use nx_common::common::transform2d::{
        euler_to_quaternion, quaternion_is_normalised, quaternion_to_euler, yaw_to_quaternion,
    };

    #[derive(Debug, Default, Copy, Clone)]
    pub struct Pose
    {
        pub position_x: f64,
        pub position_y: f64,
        pub position_z: f64,
        pub quaternion_w: f64,
        pub quaternion_x: f64,
        pub quaternion_y: f64,
        pub quaternion_z: f64,
    }

    impl Pose
    {
        pub fn to_pose_array(&self) -> [f64; 3]
        {
            let [yaw, _, _] = quaternion_to_euler(
                self.quaternion_w,
                self.quaternion_x,
                self.quaternion_y,
                self.quaternion_z,
            );
            [self.position_x as f64, self.position_y as f64, yaw as f64]
        }

        pub fn from_pose_array(&mut self, pose: [f64; 3])
        {
            let [qw, qx, qy, qz] = yaw_to_quaternion(pose[2] as f64);
            self.quaternion_w = qw;
            self.quaternion_x = qx;
            self.quaternion_y = qy;
            self.quaternion_z = qz;
            self.position_x = pose[0] as f64;
            self.position_y = pose[1] as f64;
            self.position_z = 0.0;
        }

        pub fn is_numeric_normal(&self) -> bool
        {
            self.position_x.is_finite()
                && self.position_y.is_finite()
                && self.position_z.is_finite()
                && self.quaternion_w.is_finite()
                && self.quaternion_x.is_finite()
                && self.quaternion_y.is_finite()
                && self.quaternion_z.is_finite()
                && quaternion_is_normalised(
                    self.quaternion_w,
                    self.quaternion_x,
                    self.quaternion_y,
                    self.quaternion_z,
                )
        }
    }

    #[derive(Debug, Default, Copy, Clone)]
    pub struct Twist
    {
        pub linear_x: f64,
        pub linear_y: f64,
        pub linear_z: f64,
        pub rotate_x: f64,
        pub rotate_y: f64,
        pub rotate_z: f64,
    }

    impl Twist
    {
        fn is_numeric_normal(&self) -> bool
        {
            self.linear_x.is_finite()
                && self.linear_y.is_finite()
                && self.linear_z.is_finite()
                && self.rotate_x.is_finite()
                && self.rotate_y.is_finite()
                && self.rotate_z.is_finite()
        }
    }
}

mod shared
{
    use crate::{basic, TaskConfig};
    use time::OffsetDateTime;

    #[derive(Debug, Clone)]
    pub(crate) struct RequestTask
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub stop: bool,
        pub resume: bool,
        pub cancel: bool,
        pub reset: bool,
        pub task: TaskConfig,
    }
    impl Default for RequestTask
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                stop: false,
                resume: false,
                cancel: false,
                reset: false,
                task: Default::default(),
            }
        }
    }

    impl RequestTask
    {
        pub fn reset_command(&mut self)
        {
            self.stop = false;
            self.resume = false;
            self.cancel = false;
            self.reset = false;
        }
    }

    #[derive(Debug)]
    pub(crate) struct RequestPath
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub path: Vec<basic::Pose>,
    }

    #[derive(Debug)]
    pub(crate) struct CommonFeedback
    {
        pub stamp: OffsetDateTime,
        pub counter_rerun: usize,
        pub counter_ros: usize,
        pub counter_run: usize,
        pub counter_lock: usize,
        pub task_id: String,
        pub status: String,
    }

    impl Default for CommonFeedback
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter_rerun: 0,
                counter_ros: 0,
                counter_run: 0,
                counter_lock: 0,
                task_id: "".to_string(),
                status: "".to_string(),
            }
        }
    }
    #[derive(Debug)]
    pub(crate) struct CommonPath
    {
        pub stamp: OffsetDateTime,
        pub counter_rerun: usize,
        pub counter_ros: usize,

        pub path: Vec<[f64; 3]>,
    }

    impl Default for CommonPath
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter_rerun: 0,
                counter_ros: 0,
                path: vec![],
            }
        }
    }
    impl Default for RequestPath
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                path: vec![],
            }
        }
    }

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct RequestGoal
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub goal: basic::Pose,
    }
    impl Default for RequestGoal
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                goal: Default::default(),
            }
        }
    }

    impl Clone for RequestPath
    {
        fn clone(&self) -> Self
        {
            Self {
                stamp: self.stamp,
                counter: self.counter,
                path: self.path.clone(),
            }
        }
    }
    impl Clone for CommonPath
    {
        fn clone(&self) -> Self
        {
            Self {
                stamp: self.stamp,
                counter_rerun: self.counter_rerun,
                counter_ros: self.counter_ros,
                path: self.path.clone(),
            }
        }
    }
    #[derive(Debug, Copy, Clone)]

    pub(crate) struct Pose
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub pose: basic::Pose,
    }
    impl Default for Pose
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                pose: Default::default(),
            }
        }
    }

    #[derive(Debug)]

    pub(crate) struct Odom
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub pose: basic::Pose,
        pub twist: basic::Twist,
    }

    impl Default for Odom
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                pose: Default::default(),
                twist: Default::default(),
            }
        }
    }
    #[derive(Debug)]
    pub(crate) struct CmdVel
    {
        pub stamp: OffsetDateTime,
        pub counter: usize,
        pub twist: basic::Twist,
    }

    impl Default for CmdVel
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                twist: Default::default(),
            }
        }
    }
    #[derive(Debug)]
    pub(crate) struct Command
    {
        stamp: OffsetDateTime,
        counter: usize,
        command: String,
    }

    impl Default for Command
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                command: "".to_string(),
            }
        }
    }
    #[derive(Debug)]
    pub(crate) struct Status
    {
        stamp: OffsetDateTime,
        counter: usize,
        task_id: String,
        status: String,
    }
    impl Default for Status
    {
        fn default() -> Self
        {
            Self {
                stamp: OffsetDateTime::now_utc(),
                counter: 0,
                task_id: "".to_string(),
                status: "".to_string(),
            }
        }
    }
}

struct RecordingStream(rerun::RecordingStream);

impl Drop for RecordingStream
{
    fn drop(&mut self)
    {
        println!("RecordingStream disconnect");
        self.0.disconnect();
    }
}

fn main()
{
    let args = Args::parse();
    let tz_offset = UtcOffset::current_local_offset().expect("should get local offset!");
    let current_now_local = get_now_local();

    // let t1 = Time::get_current_time_local();
    // println!("{:?}",t1);

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
    // let (file_writer, _guard) = tracing_appender::non_blocking(wheel_file_appender);

    // multiple appender
    // set format and filter

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
                .with_filter(filter::LevelFilter::WARN),
        );

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");

    info!("info log");
    trace!("track log");
    warn!("warn log");
    error!("error log");

    tiny_alloc::TinyAlloc::init(unsafe { &mut TA_BUFFER }, 512, 16, 8);
    let mut ros_handler = RosHandler::new();

    let ros_config_file = args.ros_config.as_str();
    let motion_config_file = args.motion_config.as_str();

    ros_handler.create(ros_config_file);

    let mut ros_handler_is_ok = ros_handler.is_ok();
    info!(ros_handler_is_ok);
    if !ros_handler_is_ok {
        error!("ros_handler initialise failed");
        return;
    }

    let contents = match fs::read_to_string(motion_config_file) {
        // If successful return the files text as `contents`.
        // `c` is a local variable.
        Ok(c) => c,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Could not read file `{}`", motion_config_file);
            // Exit the program with exit code `1`.
            return;
        }
    };

    let mut app_config: AppConfig; // = toml::from_str(&contents).unwrap();

    match toml::from_str::<AppConfig>(&contents) {
        Ok(t) => app_config = t,
        Err(e) => {
            error!("toml::from_str: {:?}", e);
            return;
        }
    }

    let robot_path_planner = Rc::new(RefCell::new(RobotPathPlanner::new(
        app_config.robot_path_planner_config.clone(),
    )));

    robot_path_planner.borrow().set_option("");

    let controller = Rc::new(RefCell::new(SingleSteeringMotionController::new(
        app_config.robot_motion_controller_config.clone(),
    )));
    controller.borrow().set_option("");

    let robot_motion_planner = Rc::new(RefCell::new(RobotMotionPlanner::new(
        app_config.robot_motion_planner_config.clone(),
        controller.clone(),
    )));
    let odometry_calculator = Rc::new(RefCell::new(OdometryCalculator::new()));

    let mut sim_forward_motor = Rc::new(RefCell::new(SpeedMotor::new(
        app_config.ros_config.sim_config.forward_acc,
        app_config.ros_config.sim_config.forward_vel_max,
    )));
    let mut sim_rotate_motor = Rc::new(RefCell::new(PositionMotor::new(
        app_config.ros_config.sim_config.rotate_vel,
        app_config.ros_config.sim_config.rotate_angle_max,
    )));

    robot_motion_planner.borrow().set_option("");

    // task

    let loop_fps = args.fps;

    let mut task_manager: Rc<RefCell<nx_common::common::task::TaskManager>> =
        Rc::new(RefCell::new(nx_common::common::task::TaskManager::new(
            20, loop_fps, 500_000,
        )));

    let mut signal_handler = SignalHandler::default();

    // ==== shared data

    // transform
    let ros_in_tf_map_odom: Arc<Mutex<shared::Pose>> =
        Arc::new(Mutex::new(shared::Pose::default()));

    let ros_in_tf_odom_base: Arc<Mutex<shared::Pose>> =
        Arc::new(Mutex::new(shared::Pose::default()));
    let ros_in_tf_map_base: Arc<Mutex<shared::Pose>> =
        Arc::new(Mutex::new(shared::Pose::default()));

    let ros_out_tf_map_base: Arc<Mutex<shared::Pose>> =
        Arc::new(Mutex::new(shared::Pose::default()));

    ros_out_tf_map_base
        .lock()
        .unwrap()
        .pose
        .from_pose_array([0.0, 0.0, 0.0]);

    //px,py,pz,vx,xy,xz,rx,ry,rz
    let ros_in_odom: Arc<Mutex<shared::Odom>> = Arc::new(Mutex::new(shared::Odom::default()));

    let ros_out_odom: Arc<Mutex<shared::Odom>> = Arc::new(Mutex::new(shared::Odom::default()));

    //vx,xy,xz,rx,ry,rz
    let ros_out_cmdvel: Arc<Mutex<shared::CmdVel>> =
        Arc::new(Mutex::new(shared::CmdVel::default()));

    // path
    let ros_in_path: Arc<Mutex<shared::RequestPath>> = Arc::new(Mutex::new(Default::default()));
    let ros_in_goal: Arc<Mutex<shared::RequestGoal>> = Arc::new(Mutex::new(Default::default()));

    let common_request_path: Arc<Mutex<shared::CommonPath>> =
        Arc::new(Mutex::new(Default::default()));

    let common_task_info: Arc<Mutex<shared::RequestTask>> =
        Arc::new(Mutex::new(Default::default()));

    let common_feedbak: Arc<Mutex<shared::CommonFeedback>> =
        Arc::new(Mutex::new(Default::default()));

    // command
    let ros_in_command: Arc<Mutex<Vec<shared::Command>>> = Arc::new(Mutex::new(vec![]));

    // status
    let ros_out_status: Arc<Mutex<shared::Status>> =
        Arc::new(Mutex::new(shared::Status::default()));

    let rerun_plot_data: Arc<Mutex<HashMap<&'static str, f64>>> =
        Arc::new(Mutex::new(Default::default()));
    let rerun_plot_pose: Arc<Mutex<HashMap<&'static str, [f64; 3]>>> =
        Arc::new(Mutex::new(Default::default()));

    // ==== shared data

    let mut ros_thread = Thread::default();
    {
        let app_config = app_config.clone();

        let ros_in_tf_map_odom = ros_in_tf_map_odom.clone();
        let ros_in_tf_odom_base = ros_in_tf_odom_base.clone();
        let ros_in_tf_map_base = ros_in_tf_map_base.clone();
        let ros_in_odom = ros_in_odom.clone();
        let ros_out_cmdvel = ros_out_cmdvel.clone();
        let ros_in_path = ros_in_path.clone();
        let ros_in_goal = ros_in_goal.clone();
        let mut ros_out_tf_map_base = ros_out_tf_map_base.clone();
        let mut ros_out_odom = ros_out_odom.clone();

        let mut common_request_path = common_request_path.clone();

        let mut common_feedbak = common_feedbak.clone();

        let mut common_task_info = common_task_info.clone();

        let rerun_plot_data = rerun_plot_data.clone();

        let rerun_plot_pose = rerun_plot_pose.clone();

        let ros_in_command = ros_in_command.clone();
        let ros_out_status = ros_out_status.clone();

        let mut signal_handler = signal_handler.clone();

        ros_thread = Thread::new(move || {
            let std_fps = loop_fps;
            let mut task_manager: Rc<RefCell<nx_common::common::task::TaskManager>> =
                Rc::new(RefCell::new(nx_common::common::task::TaskManager::new(
                    20, loop_fps, 500_000,
                )));

            let mut rec_option: Option<RecordingStream> = None;
            let mut rerun_stamp = OffsetDateTime::now_utc();
            let mut rerun_start_time = std::time::Instant::now();

            {
                let mut ros_in_tf_map_base = ros_in_tf_map_base.clone();
                let mut ros_handler = ros_handler.clone();
                let mut common_request_path = common_request_path.clone();
                let rerun_plot_data = rerun_plot_data.clone();

                let rerun_plot_pose = rerun_plot_pose.clone();

                if app_config.rerun_config.enable {
                    let mut rec_builder = rerun::RecordingStreamBuilder::new("application_id")
                        .is_official_example(false);
                    let rerun_addr: SocketAddr = app_config
                        .rerun_config
                        .addr
                        .parse::<SocketAddr>()
                        .unwrap()
                        .into();

                    rec_option = Some(RecordingStream(
                        rec_builder
                            .connect_opts(rerun_addr, Some(std::time::Duration::from_secs(5)))
                            .unwrap(),
                    ));

                    let rec = match rec_option.as_ref() {
                        Some(t) => &t.0,
                        None => {
                            panic!()
                        }
                    };

                    {
                        if app_config.rerun_config.clear {
                            rec.log("sim", &rerun::Clear::recursive());
                            rec.log("data", &rerun::Clear::recursive());
                        }
                        rec.set_time_nanos(
                            "sim/time",
                            rerun_start_time.elapsed().as_nanos() as i64,
                        );

                        // robot and pallet
                        rec.log("sim/bounds", &rerun::Boxes2D::from_sizes([(10.0, 10.0)]));

                        rec.log(
                            "sim/robot/fork",
                            &rerun::Boxes2D::from_mins_and_sizes(
                                &app_config.rerun_config.robot[0],
                                &app_config.rerun_config.robot[1],
                            )
                            .with_radii([0.003]),
                        );
                        rec.log(
                            "sim/robot/base",
                            &rerun::Points2D::new(&app_config.rerun_config.robot_control)
                                .with_radii([0.0025]),
                        );

                        rec.log(
                            "sim/pallet/shape",
                            &rerun::Boxes2D::from_mins_and_sizes(
                                &app_config.rerun_config.pallet[0],
                                &app_config.rerun_config.pallet[1],
                            )
                            .with_radii([0.003]),
                        );
                        rec.log(
                            "sim/pallet/axis",
                            &rerun::LineStrips2D::new([
                                [[0.1, 0.0], [0.0, 0.0]],
                                [[0.0, 0.0], [0.0, -0.1]],
                            ])
                            .with_colors([[255, 0, 0], [0, 255, 0]])
                            .with_radii([0.003]),
                        );

                        rec.log(
                            "sim/robot/axis",
                            &rerun::LineStrips2D::new([
                                [[0.1, 0.0], [0.0, 0.0]],
                                [[0.0, 0.0], [0.0, -0.1]],
                            ])
                            .with_colors([[255, 0, 0], [0, 255, 0]])
                            .with_radii([0.003]),
                        );

                        rec.log(
                            "sim/robot",
                            &rerun::Transform3D::from_translation_rotation(
                                [0.0, 0.0, 0.0],
                                rerun::RotationAxisAngle::new(
                                    [0.0, 0.0, 1.0],
                                    rerun::Angle::Radians(0.0),
                                ),
                            ),
                        );

                        rec.log(
                            "sim/pallet",
                            &rerun::Transform3D::from_translation_rotation(
                                [0.0, 0.0, 0.0],
                                rerun::RotationAxisAngle::new(
                                    [0.0, 0.0, 1.0],
                                    rerun::Angle::Radians(0.0),
                                ),
                            ),
                        );

                        // rec.log(
                        //     "sim/path/arrow",
                        //     &rerun::Arrows2D::from_vectors([[1.0, 0.0], [0.0, -1.0], [-0.7, 0.7]])
                        //         .with_radii([0.0025])
                        //         .with_origins([[0.25, 0.0], [0.25, 0.0], [-0.1, -0.1]])
                        //         .with_colors([[255, 0, 0]]),
                        // );
                    }

                    {
                        let mut rec = rec.clone();

                        let rerun_callback = move || {
                            let mut current_stamp = OffsetDateTime::now_utc();
                            let relative_stamp = (current_stamp - rerun_stamp).as_seconds_f32();
                            rec.set_time_nanos(
                                "sim/time",
                                rerun_start_time.elapsed().as_nanos() as i64,
                            );

                            {
                                let rerun_plot_data_valid =
                                    rerun_plot_data.lock().unwrap().len() > 0;

                                if rerun_plot_data_valid {
                                    for (k, v) in rerun_plot_data.lock().unwrap().iter() {
                                        rec.log(*k, &rerun::Scalar::new(*v));
                                    }
                                }
                                let rerun_plot_pose_valid =
                                    rerun_plot_pose.lock().unwrap().len() > 0;
                                if rerun_plot_pose_valid {
                                    for (k, v) in rerun_plot_pose.lock().unwrap().iter() {
                                        let [x, y, yaw] = v;
                                        let x = *x as f32;
                                        let y = -*y as f32;
                                        let yaw = -*yaw as f32;

                                        let arrow_len: f32 = 0.1;
                                        let ax = arrow_len * yaw.cos();
                                        let ay = arrow_len * yaw.sin();

                                        rec.log(
                                            *k,
                                            &rerun::Arrows2D::from_vectors(&[[ax, ay]])
                                                .with_origins(&[[x, y]])
                                                .with_radii([0.001])
                                                .with_colors([[255, 0, 0]]),
                                        );
                                    }
                                }
                            }

                            let robot_pose = ros_in_tf_map_base.lock().unwrap().pose;
                            let robot_pose_valid = ros_in_tf_map_base.lock().unwrap().counter > 0;
                            // ros_in_tf_map_base.lock().unwrap().counter = 0;

                            if robot_pose_valid {
                                let [robot_pose_x, robot_pose_y, robot_pose_yaw] =
                                    robot_pose.to_pose_array();
                                let robot_pose_yaw = -robot_pose_yaw;
                                let robot_pose_y = -robot_pose_y;

                                // update robot pose, pallet pose
                                {
                                    let mut robot_pose_transform =
                                        rerun::Transform3D::from_translation_rotation(
                                            [robot_pose_x as f32, robot_pose_y as f32, 0.0],
                                            rerun::RotationAxisAngle::new(
                                                [0.0, 0.0, 1.0],
                                                rerun::Angle::Radians(robot_pose_yaw as f32),
                                            ),
                                        );
                                    // let robot_pose_tranform_rotate_matrix = euler_to_maxtrix(robot_pose_yaw , 0.0 , std::f32::consts::PI);
                                    //
                                    // let mut robot_pose_transform = rerun::Transform3D::from_translation_mat3x3(
                                    //     [robot_pose_x,robot_pose_y, 0.0, ],rerun::Mat3x3::from(robot_pose_tranform_rotate_matrix)
                                    // );

                                    // robot_pose_transform = robot_pose_transform;

                                    rec.log("sim/robot", &robot_pose_transform);
                                }

                                // plot path
                                {
                                    if common_request_path.lock().unwrap().counter_rerun > 0 {
                                        common_request_path.lock().unwrap().counter_rerun = 0;
                                        let path = common_request_path.lock().unwrap().path.clone();

                                        let path_origins: Vec<[f32; 2]> = path
                                            .iter()
                                            .map(|x| [x[0] as f32, -x[1] as f32])
                                            .collect();
                                        let arrow_len: f32 = 0.05;
                                        let path_arrow: Vec<[f32; 2]> = path
                                            .iter()
                                            .map(|x| {
                                                [
                                                    arrow_len * (-x[2]).cos() as f32,
                                                    arrow_len * (-x[2]).sin() as f32,
                                                ]
                                            })
                                            .collect();

                                        rec.log(
                                            "sim/path/point",
                                            &rerun::Points2D::new(&path_origins)
                                                .with_radii([0.0015]),
                                        );
                                        rec.log(
                                            "sim/path/arrow",
                                            &rerun::Arrows2D::from_vectors(&path_arrow)
                                                .with_origins(&path_origins)
                                                .with_radii([0.0005])
                                                .with_colors([[0, 255, 0]]),
                                        );
                                    }
                                }
                            }

                            return true;
                        };
                        task_manager
                            .borrow()
                            .add("rerun_callback", rerun_callback, 100.0);
                    }
                }
            }

            {
                let helle_func = move || {
                    // let t1 = OffsetDateTime::now_utc().to_offset(tz_offset);
                    // info!("t1 {:?}", t1);
                    return true;
                };
                task_manager.borrow().add("hello", helle_func, 100.0);
            }

            let mut request_task = TaskConfig::default();

            {
                // high frequency
                //input: tf, odom
                //output: cmd_vel

                let mut ros_in_tf_map_base = ros_in_tf_map_base.clone();
                let mut ros_in_odom = ros_in_odom.clone();
                let mut ros_out_cmdvel = ros_out_cmdvel.clone();
                let mut ros_out_tf_map_base = ros_out_tf_map_base.clone();
                let mut ros_out_odom = ros_out_odom.clone();

                let mut ros_handler = ros_handler.clone();

                let mut send_twist = nx_ros::ros::ros_message::shared::Twist::new();
                let cmdvel = send_twist.get_mut_data();
                cmdvel.linear.x = 0.0;
                cmdvel.angular.z = 0.0;

                let mut send_odom = nx_ros::ros::ros_message::shared::Odometry::new();

                send_odom.set_frame_id("map", "base_link");

                let mut send_tf = PoseStamped::new();

                let realtime_pose_odom_cmdvel_callback = move || {
                    {
                        //recv tf
                        let mut recv_tf_buffer = ros_handler.read_data("tf_sub");
                        if let Some(r) = &mut recv_tf_buffer.take() {
                            // println!("recv_tf_buffer: size {}\n ", r.len());
                            for m in r.iter() {
                                let mut data = PoseStamped::from_ptr(*m);

                                let timestamp_u64 = data.get_stamp();
                                let stamp = OffsetDateTime::from_unix_timestamp_nanos(
                                    timestamp_u64 as i128,
                                )
                                .unwrap();

                                let current_stamp = OffsetDateTime::now_utc();

                                let stamp_diff = current_stamp - stamp;

                                let stamp_diff_s = stamp_diff.as_seconds_f32();
                                let stamp_local = stamp.to_offset(tz_offset);
                                let stamp_valid = (stamp_diff_s.abs() < 1.0);

                                // info!("stamp_local: {:?}, stamp_diff_s : {},stamp_valid: {}",stamp_local, stamp_diff_s, stamp_valid);

                                // println!(" i = {:?}",i);
                                // println!("frame_id : {}", data.get_frame_id().deref());
                                // println!("data: {:?},", data.get_data(),);
                                let pose = data.get_data();

                                ros_in_tf_map_base.lock().unwrap().stamp = stamp;
                                ros_in_tf_map_base.lock().unwrap().pose = basic::Pose {
                                    position_x: pose.position.x,
                                    position_y: pose.position.y,
                                    position_z: pose.position.z,
                                    quaternion_w: pose.quaternion.w,
                                    quaternion_x: pose.quaternion.x,
                                    quaternion_y: pose.quaternion.y,
                                    quaternion_z: pose.quaternion.z,
                                };

                                ros_in_tf_map_base.lock().unwrap().counter += 1;
                            }
                        }
                    }

                    {
                        //recv odom
                        let mut recv_odom_buffer = ros_handler.read_data("odom_sub");
                        if let Some(r) = &mut recv_odom_buffer.take() {
                            // info!("recv_odom_buffer: size {}\n ", r.len());
                            for m in r.iter() {
                                let mut data =
                                    nx_ros::ros::ros_message::shared::Odometry::from_ptr(*m);

                                let timestamp_u64 = data.get_stamp();
                                let stamp = OffsetDateTime::from_unix_timestamp_nanos(
                                    timestamp_u64 as i128,
                                )
                                .unwrap();

                                let current_stamp = OffsetDateTime::now_utc();

                                let stamp_diff = current_stamp - stamp;

                                let stamp_diff_s = stamp_diff.as_seconds_f32();
                                let stamp_local = stamp.to_offset(tz_offset);
                                let stamp_valid = (stamp_diff_s.abs() < 1.0);

                                // info!("stamp_local: {:?}, stamp_diff_s : {},stamp_valid: {}",stamp_local, stamp_diff_s, stamp_valid);

                                let odom = data.get_data();
                                let pose = odom.pose;
                                let twist = odom.twist;

                                ros_in_odom.lock().unwrap().counter += 1;
                                ros_in_odom.lock().unwrap().stamp = stamp;
                                ros_in_odom.lock().unwrap().pose = basic::Pose {
                                    position_x: pose.position.x,
                                    position_y: pose.position.y,
                                    position_z: pose.position.z,
                                    quaternion_w: pose.quaternion.w,
                                    quaternion_x: pose.quaternion.x,
                                    quaternion_y: pose.quaternion.y,
                                    quaternion_z: pose.quaternion.z,
                                };
                                ros_in_odom.lock().unwrap().twist = basic::Twist {
                                    linear_x: twist.linear.x,
                                    linear_y: twist.linear.y,
                                    linear_z: twist.linear.z,
                                    rotate_x: twist.angular.x,
                                    rotate_y: twist.angular.y,
                                    rotate_z: twist.angular.z,
                                }
                            }
                        }
                    }

                    {
                        // send cmdvel

                        if ros_out_cmdvel.lock().unwrap().counter > 0 {
                            let current_stamp = OffsetDateTime::now_utc();
                            let stamp_diff = current_stamp - ros_out_cmdvel.lock().unwrap().stamp;
                            let twist = ros_out_cmdvel.lock().unwrap().twist;

                            let stamp_valid = stamp_diff.as_seconds_f32() < 0.1;

                            let cmdvel = send_twist.get_mut_data();

                            if stamp_valid {
                                cmdvel.linear.x = twist.linear_x;
                                cmdvel.angular.z = twist.rotate_z;
                            } else {
                                cmdvel.linear.x = 0.0;
                                cmdvel.angular.z = 0.0;
                            }

                            let mut send_twist_buffer = [*send_twist.get_ptr() as *mut c_void];

                            ros_handler.write_data("twist_pub", &mut send_twist_buffer[..]);
                        }
                    }

                    if app_config.ros_config.sim_config.enable {
                        let tf_data = send_tf.get_mut_data();
                        let odom_data = send_odom.get_mut_data();

                        let pose = ros_out_tf_map_base.lock().unwrap().pose;
                        let odom_pose = ros_out_odom.lock().unwrap().pose;
                        let odom_twist = ros_out_odom.lock().unwrap().twist;

                        tf_data.position.x = pose.position_x;
                        tf_data.position.y = pose.position_y;
                        tf_data.position.z = pose.position_z;

                        tf_data.quaternion.w = pose.quaternion_w;
                        tf_data.quaternion.x = pose.quaternion_x;
                        tf_data.quaternion.y = pose.quaternion_y;
                        tf_data.quaternion.z = pose.quaternion_z;

                        odom_data.pose.position.x = odom_pose.position_x;
                        odom_data.pose.position.y = odom_pose.position_y;
                        odom_data.pose.position.z = odom_pose.position_z;

                        odom_data.pose.quaternion.w = odom_pose.quaternion_w;
                        odom_data.pose.quaternion.x = odom_pose.quaternion_x;
                        odom_data.pose.quaternion.y = odom_pose.quaternion_y;
                        odom_data.pose.quaternion.z = odom_pose.quaternion_z;

                        odom_data.twist.linear.x = odom_twist.linear_x;
                        odom_data.twist.linear.y = odom_twist.linear_y;
                        odom_data.twist.linear.z = odom_twist.linear_z;

                        odom_data.twist.angular.x = odom_twist.rotate_x;
                        odom_data.twist.angular.y = odom_twist.rotate_y;
                        odom_data.twist.angular.z = odom_twist.rotate_z;

                        let mut send_tf_buffer = [*send_tf.get_ptr() as *mut c_void];
                        ros_handler.write_data("tf_pub", &mut send_tf_buffer[..]);

                        let mut send_odom_buffer = [*send_odom.get_ptr() as *mut c_void];

                        ros_handler.write_data("odom_pub", &mut send_odom_buffer[..]);
                    }

                    return true;
                };

                task_manager.borrow().add(
                    "realtime_pose_odom_cmdvel_callback",
                    realtime_pose_odom_cmdvel_callback,
                    20.0,
                );
            }
            {
                let mut ros_handler = ros_handler.clone();

                let task_command_callback = move || {
                    return true;
                };
            }

            {
                let mut ros_handler = ros_handler.clone();
                let ros_in_path = ros_in_path.clone();
                let ros_in_goal = ros_in_goal.clone();

                let mut send_path = nx_ros::ros::ros_message::shared::Path::new(100);
                send_path.set_frame_id("map");
                let common_request_path = common_request_path.clone();

                let mut common_feedbak = common_feedbak.clone();

                let mut common_task_info = common_task_info.clone();
                let mut send_headerstring =
                    nx_ros::ros::ros_message::shared::HeaderString::new(100);

                let path_goal_callback = move || {
                    //send_headerstring
                    {
                        let current_now_local = get_now_local();

                        if common_feedbak.lock().unwrap().counter_run == 0 {
                            let status = format!("BootUp at {:?}", current_now_local);

                            // send_headerstring.set_frame_id("idle");
                            send_headerstring.set_data(status.as_str());
                        } else {
                            send_headerstring
                                .set_frame_id(common_feedbak.lock().unwrap().task_id.as_str());
                            send_headerstring
                                .set_data(common_feedbak.lock().unwrap().status.as_str());
                        }

                        let mut send_headerstring_buffer =
                            [*send_headerstring.get_ptr() as *mut c_void];

                        ros_handler
                            .write_data("headerstring_pub", &mut send_headerstring_buffer[..]);

                        common_feedbak.lock().unwrap().counter_run += 1;
                    }

                    // send_path
                    {
                        if common_request_path.lock().unwrap().counter_ros > 0 {
                            common_request_path.lock().unwrap().counter_ros = 0;
                            let path_len = common_request_path.lock().unwrap().path.len();

                            if path_len > 0 {
                                send_path.alloc_data(path_len as u32);
                                let mut send_path_pose = send_path.get_mut_data();
                                for (i, j) in izip!(
                                    send_path_pose.iter_mut(),
                                    common_request_path.lock().unwrap().path.iter_mut()
                                ) {
                                    let [qw, qx, qy, qz] = yaw_to_quaternion(j[2] as f64);
                                    i.position.x = j[0] as f64;
                                    i.position.y = j[1] as f64;
                                    i.position.z = 0.0;
                                    i.quaternion.w = qw;
                                    i.quaternion.x = qx;
                                    i.quaternion.y = qy;
                                    i.quaternion.z = qz;
                                }

                                let mut send_path_buffer = [*send_path.get_ptr() as *mut c_void];

                                ros_handler.write_data("path_pub", &mut send_path_buffer[..]);
                            }
                        }
                    }

                    // recv path
                    {
                        let mut recv_path_buffer = ros_handler.read_data("path_sub");
                        let mut data_update = false;

                        if let Some(r) = &mut recv_path_buffer.take() {
                            for m in r.iter() {
                                let mut data = Path::from_ptr(*m);
                                let timestamp_u64 = data.get_stamp();
                                let stamp = OffsetDateTime::from_unix_timestamp_nanos(
                                    timestamp_u64 as i128,
                                )
                                .unwrap();

                                let current_stamp = OffsetDateTime::now_utc();

                                let stamp_diff = current_stamp - stamp;

                                let stamp_diff_s = stamp_diff.as_seconds_f32();
                                let stamp_local = stamp.to_offset(tz_offset);
                                let stamp_valid = (stamp_diff_s.abs() < 1.0);

                                let frame_id = data.get_frame_id();
                                let frame_id = frame_id.deref();

                                let pose = data.get_data();
                                let pose_len = pose.len();
                                let path_valid = stamp_valid && (pose_len > 2);
                                info!("receive path: frame_id = {:?}, stamp_local: {:?}, pose_len: {}, stamp_valid: {}", frame_id, stamp_local,pose_len,path_valid );

                                if path_valid {
                                    ros_in_path.lock().unwrap().stamp = stamp_local;

                                    common_task_info.lock().unwrap().reset_command();
                                    if frame_id.starts_with("stop") {
                                        common_task_info.lock().unwrap().stop = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("resume") {
                                        common_task_info.lock().unwrap().resume = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("cancel") {
                                        common_task_info.lock().unwrap().cancel = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("reset") {
                                        common_task_info.lock().unwrap().reset = true;
                                        common_task_info.lock().unwrap().counter += 1;
                                    } else {
                                        request_task.counter = None;

                                        match toml::from_str::<TaskConfig>(frame_id) {
                                            Ok(t) => {
                                                request_task = t;
                                                request_task.counter = Some(1);
                                            }
                                            Err(e) => {
                                                error!("toml::from_str: {:?}", e);
                                                request_task.task_id = String::from(frame_id);
                                            }
                                        }

                                        warn!("recv goal task :{:?}", request_task);

                                        common_task_info.lock().unwrap().task =
                                            request_task.clone();
                                        common_task_info.lock().unwrap().counter += 1;

                                        ros_in_path
                                            .lock()
                                            .unwrap()
                                            .path
                                            .resize(pose_len, basic::Pose::default());
                                        for (i, j) in
                                            izip!(&mut ros_in_path.lock().unwrap().path, pose)
                                        {
                                            i.position_x = j.position.x;
                                            i.position_y = j.position.y;
                                            i.position_z = j.position.z;
                                            i.quaternion_w = j.quaternion.w;
                                            i.quaternion_x = j.quaternion.x;
                                            i.quaternion_y = j.quaternion.y;
                                            i.quaternion_z = j.quaternion.z;
                                        }
                                        data_update = true;
                                    }
                                }

                                // println!("pose: {:?},",  pose,);
                            }
                        }
                        ros_in_path.lock().unwrap().counter += data_update as usize;
                    }

                    //recv goal
                    {
                        let mut recv_pose_buffer = ros_handler.read_data("pose_sub");
                        let mut data_update = false;
                        if let Some(r) = &mut recv_pose_buffer.take() {
                            warn!("recv_pose_buffer: size {}\n ", r.len());

                            for m in r.iter() {
                                let mut data = PoseStamped::from_ptr(*m);

                                let timestamp_u64 = data.get_stamp();

                                let stamp = OffsetDateTime::from_unix_timestamp_nanos(
                                    timestamp_u64 as i128,
                                )
                                .unwrap();

                                let current_stamp = OffsetDateTime::now_utc();

                                let stamp_diff = current_stamp - stamp;

                                let stamp_diff_s = stamp_diff.as_seconds_f32();

                                let frame_id = data.get_frame_id();
                                let frame_id = frame_id.deref();

                                let stamp_local = stamp.to_offset(tz_offset);
                                let stamp_valid = (stamp_diff_s.abs() < 1.0);

                                // info!("frame_id: {:?}, stamp_local: {:?}, stamp_diff_s: {}",frame_id, stamp_local, stamp_diff_s);

                                let pose = data.get_data();

                                let q_sum = pose.quaternion.w * pose.quaternion.w
                                    + pose.quaternion.x * pose.quaternion.x
                                    + pose.quaternion.y * pose.quaternion.y
                                    + pose.quaternion.z * pose.quaternion.z;

                                let is_pose_valid = (q_sum - 1.0).abs() < 1e-3;

                                if stamp_valid {
                                    common_task_info.lock().unwrap().reset_command();
                                    if frame_id.starts_with("stop") {
                                        common_task_info.lock().unwrap().stop = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("resume") {
                                        common_task_info.lock().unwrap().resume = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("cancel") {
                                        common_task_info.lock().unwrap().cancel = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if frame_id.starts_with("reset") {
                                        common_task_info.lock().unwrap().reset = true;
                                        common_task_info.lock().unwrap().counter += 1;
                                    } else {
                                        request_task.counter = None;

                                        match toml::from_str::<TaskConfig>(frame_id) {
                                            Ok(t) => {
                                                request_task = t;
                                                request_task.counter = Some(1);
                                            }
                                            Err(e) => {
                                                error!("toml::from_str: {:?}", e);
                                                request_task.task_id = String::from(frame_id);
                                            }
                                        }

                                        warn!("recv goal task :{:?}", request_task);

                                        common_task_info.lock().unwrap().task =
                                            request_task.clone();
                                        common_task_info.lock().unwrap().counter += 1;

                                        // create path from
                                        let mut current: [f32; 3] = [0.0, 0.0, 0.0];
                                        let mut target: [f32; 3] = [0.0, 0.0, 0.0];

                                        ros_in_goal.lock().unwrap().stamp = stamp_local;
                                        ros_in_goal.lock().unwrap().goal = basic::Pose {
                                            position_x: pose.position.x,
                                            position_y: pose.position.y,
                                            position_z: pose.position.z,
                                            quaternion_w: pose.quaternion.w,
                                            quaternion_x: pose.quaternion.x,
                                            quaternion_y: pose.quaternion.y,
                                            quaternion_z: pose.quaternion.z,
                                        };
                                        data_update = true;
                                    }
                                }
                            }
                            ros_in_goal.lock().unwrap().counter += data_update as usize;
                        }
                    }

                    {
                        // recv task command
                        let mut recv_headerstring_buffer =
                            ros_handler.read_data("headerstring_sub");

                        if let Some(r) = &mut recv_headerstring_buffer.take() {
                            // println!("recv_headerstring_buffer: size {}\n ", r.len());
                            for m in r.iter() {
                                let mut data = HeaderString::from_ptr(*m);

                                let timestamp_u64 = data.get_stamp();

                                let stamp = OffsetDateTime::from_unix_timestamp_nanos(
                                    timestamp_u64 as i128,
                                )
                                .unwrap();

                                let current_stamp = OffsetDateTime::now_utc();

                                let stamp_diff = current_stamp - stamp;

                                let stamp_diff_s = stamp_diff.as_seconds_f32();

                                let frame_id = data.get_frame_id();
                                let frame_id = frame_id.deref();

                                let stamp_local = stamp.to_offset(tz_offset);
                                let stamp_valid = (stamp_diff_s.abs() < 1.0);

                                if stamp_valid {
                                    let msg = data.get_data();
                                    let msg = msg.deref();

                                    request_task.counter = None;

                                    common_task_info.lock().unwrap().reset_command();
                                    if msg.starts_with("stop") {
                                        common_task_info.lock().unwrap().stop = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if msg.starts_with("resume") {
                                        common_task_info.lock().unwrap().resume = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if msg.starts_with("cancel") {
                                        common_task_info.lock().unwrap().cancel = true;

                                        common_task_info.lock().unwrap().counter += 1;
                                    } else if msg.starts_with("reset") {
                                        common_task_info.lock().unwrap().reset = true;
                                        common_task_info.lock().unwrap().counter += 1;
                                    } else {
                                        match toml::from_str::<TaskConfig>(msg) {
                                            Ok(t) => {
                                                request_task = t;
                                                request_task.counter = Some(1);
                                            }
                                            Err(e) => {
                                                error!("toml::from_str: {:?}", e);
                                                request_task.task_id = String::from(msg);
                                            }
                                        }
                                        common_task_info.lock().unwrap().task =
                                            request_task.clone();
                                        common_task_info.lock().unwrap().counter += 1;
                                    }
                                    warn!("recv cmd task msg:{:?}", msg);
                                }

                                // info!("frame_id: {:?}, stamp_local: {:?}, stamp_diff_s: {}",frame_id, stamp_local, stamp_diff_s);
                            }
                        }
                    }

                    {}

                    return true;
                };
                task_manager
                    .borrow()
                    .add("path_goal_callback", path_goal_callback, 500.0);
            }

            // create buffer

            let mut loop_helper = LoopHelper::builder()
                .native_accuracy_ns(100_000)
                .build_with_target_rate(std_fps); // limit to 250 FPS if possible

            while signal_handler.is_run() && ros_handler.is_ok() {
                task_manager.borrow().run();
            }
            signal_handler.stop();

            warn!("exit t1");
        });
    }

    let std_fps = loop_fps;

    let mut loop_helper = LoopHelper::builder()
        .native_accuracy_ns(100_000)
        .build_with_target_rate(std_fps); // limit to 250 FPS if possible
    let mut counter = 0;
    let mut counter_max = 1000;

    let mut planner_stop_command = false;
    let mut planner_cancel_command = false;
    let mut planner_reset_command = false;

    while signal_handler.is_run() {
        loop_helper.loop_start();
        let current_stamp_local = OffsetDateTime::now_utc().to_offset(tz_offset);

        {
            let ros_in_tf_map_base_counter = ros_in_tf_map_base.lock().unwrap().counter;
            let ros_in_tf_map_base_stamp = ros_in_tf_map_base.lock().unwrap().stamp;

            let ros_in_pose_map_base_is_valid = (ros_in_tf_map_base_counter > 0)
                && ((current_stamp_local - ros_in_tf_map_base_stamp)
                    .as_seconds_f32()
                    .abs()
                    < app_config.ros_config.tf_timeout_s);
            let ros_in_odom_counter = ros_in_odom.lock().unwrap().counter;
            let ros_in_odom_stamp = ros_in_odom.lock().unwrap().stamp;

            let ros_in_odom_is_valid = (ros_in_odom_counter > 0)
                && ((current_stamp_local - ros_in_odom_stamp)
                    .as_seconds_f32()
                    .abs()
                    < app_config.ros_config.odom_timeout_s);

            let ros_odom_pose_is_valid = ros_in_pose_map_base_is_valid && ros_in_odom_is_valid;
            info!(
                ?current_stamp_local,
                ros_in_pose_map_base_is_valid,
                ros_in_tf_map_base_counter,
                ?ros_in_tf_map_base_stamp
            );
            info!(
                ?current_stamp_local,
                ros_in_odom_is_valid,
                ros_in_odom_counter,
                ?ros_in_odom_stamp
            );

            let common_task_is_valid = common_task_info.lock().unwrap().counter > 0;

            if common_task_is_valid {
                common_task_info.lock().unwrap().counter = 0;

                common_feedbak.lock().unwrap().task_id =
                    common_task_info.lock().unwrap().task.task_id.clone();
                common_feedbak.lock().unwrap().status = "Idle".to_string();

                if common_task_info.lock().unwrap().stop {
                    planner_stop_command = true;
                } else if common_task_info.lock().unwrap().resume {
                    planner_stop_command = false;
                } else if common_task_info.lock().unwrap().cancel {
                    planner_stop_command = true;
                    planner_cancel_command = true;
                } else if common_task_info.lock().unwrap().reset {
                    planner_stop_command = true;
                    planner_reset_command = true;
                } else {
                    planner_stop_command = false;
                }

                common_task_info.lock().unwrap().reset_command();
            }

            let current_pose = ros_in_tf_map_base.lock().unwrap().clone();
            let current_pose_array = current_pose.pose.to_pose_array();
            let current_odom_twist = ros_in_odom.lock().unwrap().twist;

            let current_odom_forward_vel = (current_odom_twist.linear_x
                * current_odom_twist.linear_x
                + current_odom_twist.linear_y * current_odom_twist.linear_y)
                .sqrt();

            let current_odom_forward_angle = current_odom_twist
                .linear_y
                .atan2(current_odom_twist.linear_x);
            let current_odom_rotate_vel = current_odom_twist.rotate_z;
            let current_odom: TwistState = TwistState {
                forward_vel: current_odom_forward_vel,
                forward_angle: current_odom_forward_angle,
                rotate_vel: current_odom_rotate_vel,
            };

            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/lx")
                .or_insert(0.0) = current_odom_twist.linear_x;
            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/ly")
                .or_insert(0.0) = current_odom_twist.linear_y;
            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/rz")
                .or_insert(0.0) = current_odom_twist.rotate_z;

            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/forward_vel")
                .or_insert(0.0) = current_odom_forward_vel;
            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/forward_angle")
                .or_insert(0.0) = current_odom_forward_angle;
            *rerun_plot_data
                .lock()
                .unwrap()
                .entry("data/current/twist/rotate_vel")
                .or_insert(0.0) = current_odom_rotate_vel;

            // update pose and odom
            robot_motion_planner
                .borrow()
                .set_robot_pose(&current_pose_array);

            let mut ros_in_path_is_valid = ros_in_path.lock().unwrap().counter > 0;

            if ros_in_path_is_valid {
                ros_in_path_is_valid = ros_in_path
                    .lock()
                    .unwrap()
                    .path
                    .iter()
                    .all(|x| x.is_numeric_normal());

                ros_in_path.lock().unwrap().counter = 0;
                if ros_in_path_is_valid {
                    let path = ros_in_path.lock().unwrap().clone();

                    common_request_path.lock().unwrap().path.clear();
                    common_request_path
                        .lock()
                        .unwrap()
                        .path
                        .resize(path.path.len(), Default::default());
                    for (i, j) in izip!(&mut common_request_path.lock().unwrap().path, &path.path) {
                        *i = j.to_pose_array();
                    }

                    common_request_path.lock().unwrap().stamp = current_stamp_local;
                    common_request_path.lock().unwrap().counter_rerun += 1;
                    common_request_path.lock().unwrap().counter_ros += 1;

                    robot_motion_planner
                        .borrow()
                        .request_path(&common_request_path.lock().unwrap().path);
                } else {
                    robot_motion_planner
                        .borrow()
                        .set_error(RobotMotionPlannerError::RequestError);
                }
            }

            let mut ros_in_goal_is_valid = ros_in_goal.lock().unwrap().counter > 0;
            if ros_in_goal_is_valid {
                ros_in_goal.lock().unwrap().counter = 0;

                ros_in_goal_is_valid = ros_in_goal.lock().unwrap().goal.is_numeric_normal();
                if ros_in_goal_is_valid {
                    let goal = ros_in_goal.lock().unwrap().clone();

                    warn!(
                        "receive goal :{:?}, ros_odom_pose_is_valid: {}",
                        goal, ros_odom_pose_is_valid
                    );
                    let current_pose = ros_in_tf_map_base.lock().unwrap().clone();

                    let target_pose_array = goal.goal.to_pose_array();

                    let mut current_pose_array = current_pose_array;

                    #[cfg(a)]
                    {
                        current_pose_array[0] += app_config.rerun_config.test_relative_start[0];
                        current_pose_array[1] += app_config.rerun_config.test_relative_start[1];
                        current_pose_array[2] += app_config.rerun_config.test_relative_start[2];
                    }

                    let ok = robot_path_planner
                        .borrow()
                        .request_goal(&current_pose_array, &target_pose_array);

                    if !ok {
                        warn!(
                            "Error current_pose_array={:?}, target_pose_array={:?}",
                            current_pose_array, target_pose_array
                        );
                        robot_motion_planner
                            .borrow()
                            .set_error(RobotMotionPlannerError::RequestError);
                    }

                    #[cfg(a)]
                    {
                        // add long dist point

                        let path_len = robot_path_planner.borrow().path.borrow().len();

                        let mut goal_node_2 =
                            robot_path_planner.borrow().path.borrow()[path_len - 3];

                        let mut goal_node_1 =
                            robot_path_planner.borrow().path.borrow()[path_len - 2];
                        let mut goal_node = robot_path_planner.borrow().path.borrow()[path_len - 1];
                        let offset_len = 4.5;
                        let mut direction =
                            (goal_node[1] - goal_node_1[1]).atan2(goal_node[0] - goal_node_1[0]);
                        let direction = direction + FRAC_PI_2;
                        let direction_1 = direction + 0.5 * FRAC_PI_2;

                        goal_node[0] += offset_len * direction.cos();
                        goal_node[1] += offset_len * direction.sin();
                        goal_node[2] += FRAC_PI_2;
                        goal_node_1[0] += 0.5 * offset_len * direction_1.cos();
                        goal_node_1[1] += 0.5 * offset_len * direction_1.sin();
                        goal_node_1[2] += FRAC_PI_2;

                        // robot_path_planner.borrow().path.borrow_mut()[path_len - 2][2] += FRAC_PI_2;
                        goal_node_2[0] = goal_node_2[0] + 0.1 * (goal_node_1[0] - goal_node_2[0]);
                        goal_node_2[1] = goal_node_2[1] + 0.1 * (goal_node_1[1] - goal_node_2[1]);

                        robot_path_planner.borrow().path.borrow_mut()[path_len - 3] = goal_node_2;

                        robot_path_planner.borrow().path.borrow_mut()[path_len - 2] = goal_node_1;

                        robot_path_planner.borrow().path.borrow_mut()[path_len - 1] = goal_node;
                    }

                    if ok {
                        common_request_path.lock().unwrap().path.clear();
                        common_request_path
                            .lock()
                            .unwrap()
                            .path
                            .extend(robot_path_planner.borrow().path.borrow().deref());

                        common_request_path.lock().unwrap().stamp = current_stamp_local;
                        common_request_path.lock().unwrap().counter_rerun += 1;
                        common_request_path.lock().unwrap().counter_ros += 1;

                        // let request_path = robot_path_planner.borrow().path.borrow().deref();
                        warn!(
                            "request_path : {:?}",
                            robot_path_planner.borrow().path.borrow().deref()
                        );
                        robot_motion_planner
                            .borrow()
                            .request_path(&common_request_path.lock().unwrap().path);
                        let state = robot_motion_planner.borrow().planner_data.borrow().state;
                        let error = robot_motion_planner.borrow().planner_data.borrow().error;

                        warn!(
                            "robot_motion_planner.borrow().request_path: state: {:?}, error: {:?}",
                            state, error
                        );
                    }
                } else {
                    robot_motion_planner
                        .borrow()
                        .set_error(RobotMotionPlannerError::RequestError);
                }
            }

            if planner_cancel_command {
                warn!("execute planner_cancel_command");
                planner_cancel_command = false;
                robot_motion_planner.borrow().reset();
            } else if planner_reset_command {
                warn!("execute planner_reset_command");
                planner_reset_command = false;
                robot_motion_planner.borrow().reset();
            }

            let mut state = robot_motion_planner.borrow().planner_data.borrow().state;
            let mut error = robot_motion_planner.borrow().planner_data.borrow().error;
            let mut task = robot_motion_planner.borrow().planner_data.borrow().task;

            let motion_control_is_ok = ros_odom_pose_is_valid
                && (error == RobotMotionPlannerError::OK)
                && !planner_stop_command;

            if motion_control_is_ok {
                robot_motion_planner.borrow().go();

                nx_robot::base::motion_planner_logger::log(
                    robot_motion_planner.clone(),
                    rerun_plot_data.clone(),
                    rerun_plot_pose.clone(),
                );
            } else {
                warn!(
                    ros_odom_pose_is_valid,
                    ros_in_pose_map_base_is_valid,
                    ros_in_odom_is_valid,
                    ?error,
                    ?state
                );

                robot_motion_planner.borrow().stop();
            }

            state = robot_motion_planner.borrow().planner_data.borrow().state;
            error = robot_motion_planner.borrow().planner_data.borrow().error;
            task = robot_motion_planner.borrow().planner_data.borrow().task;

            common_feedbak.lock().unwrap().status = format!("{:?}", state);

            if error != RobotMotionPlannerError::OK {
                common_feedbak.lock().unwrap().status = format!("{:?}", error);
            }

            let cmd = controller.borrow().get_cmd();
            ros_out_cmdvel.lock().unwrap().stamp = OffsetDateTime::now_utc();

            let round_precision:f64 = 1e5;
            let round_precision_inv:f64 = 1.0/ round_precision;

            match &cmd {
                ControllerCommand::Twist(t) => {
                    controller
                        .borrow()
                        .update_wheel_state_by_odom(&current_odom);

                    let twist = basic::Twist {
                        linear_x: ((t.forward_vel * t.forward_angle.cos()) as f64 * round_precision).round()
                            * round_precision_inv,
                        linear_y: ((t.forward_vel * t.forward_angle.sin()) as f64 * round_precision).round()
                            * round_precision_inv,
                        linear_z: 0.0,
                        rotate_x: 0.0,
                        rotate_y: 0.0,
                        rotate_z: (t.rotate_vel as f64 * round_precision).round() * round_precision_inv,
                    };
                    ros_out_cmdvel.lock().unwrap().twist = twist;

                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/lx")
                        .or_insert(0.0) = twist.linear_x as f64;
                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/ly")
                        .or_insert(0.0) = twist.linear_y as f64;
                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/rz")
                        .or_insert(0.0) = twist.rotate_z as f64;
                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/forward_vel")
                        .or_insert(0.0) = t.forward_vel as f64;
                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/forward_angle")
                        .or_insert(0.0) = t.forward_angle as f64;
                    *rerun_plot_data
                        .lock()
                        .unwrap()
                        .entry("data/command/twist/rotate_vel")
                        .or_insert(0.0) = t.rotate_vel as f64;
                }
                ControllerCommand::SingleSteering(t) => {
                    sim_forward_motor.borrow().set_vel(t.forward_vel);
                    sim_forward_motor.borrow().update();
                    let forward_vel = sim_forward_motor.borrow().actual_vel();

                    sim_rotate_motor.borrow().set_pos(t.rotate_angle);
                    sim_rotate_motor.borrow().update();
                    let rotate_angle = sim_rotate_motor.borrow().actual_pos();

                    controller
                        .borrow()
                        .update_wheel_state(&[forward_vel, rotate_angle]);

                    let driver_pos = controller.borrow().config.wheel_mount_pos[0];
                    let (valid, twist) =
                        SingleSteering::compute_twist(&driver_pos, &[forward_vel, rotate_angle]);

                    odometry_calculator.borrow().compute_odom(&twist);

                    let odometry_calculator_pose =
                        odometry_calculator.borrow().odom.borrow().as_array();
                    ros_out_tf_map_base
                        .lock()
                        .unwrap()
                        .pose
                        .from_pose_array(odometry_calculator_pose);
                    ros_out_tf_map_base.lock().unwrap().counter += 1;
                    ros_out_odom
                        .lock()
                        .unwrap()
                        .pose
                        .from_pose_array(odometry_calculator_pose);

                    let twist = basic::Twist {
                        linear_x: ((twist.forward_vel * twist.forward_angle.cos()) as f64 * round_precision)
                            .round()
                            * round_precision_inv,
                        linear_y: ((twist.forward_vel * twist.forward_angle.sin()) as f64 * round_precision)
                            .round()
                            * round_precision_inv,
                        linear_z: 0.0,
                        rotate_x: 0.0,
                        rotate_y: 0.0,
                        rotate_z: (twist.rotate_vel as f64 * round_precision).round() * round_precision_inv,
                    };

                    ros_out_odom.lock().unwrap().twist = twist;

                    ros_out_cmdvel.lock().unwrap().twist = twist;
                }
                _ => {}
            }

            {
                let wheel_state = *controller.borrow().wheel_state.borrow();
                let driver_command = *controller.borrow().driver_command.borrow();

                *rerun_plot_data
                    .lock()
                    .unwrap()
                    .entry("data/current/wheel_state/forward_vel")
                    .or_insert(0.0) = wheel_state.forward_vel as f64;

                *rerun_plot_data
                    .lock()
                    .unwrap()
                    .entry("data/current/wheel_state/rotate_angle")
                    .or_insert(0.0) = wheel_state.rotate_angle as f64;
                *rerun_plot_data
                    .lock()
                    .unwrap()
                    .entry("data/current/driver_command/forward_vel")
                    .or_insert(0.0) = driver_command.forward_vel as f64;

                *rerun_plot_data
                    .lock()
                    .unwrap()
                    .entry("data/current/driver_command/rotate_angle")
                    .or_insert(0.0) = driver_command.rotate_angle as f64;
            }

            ros_out_cmdvel.lock().unwrap().counter += 1;
        }

        loop_helper.loop_sleep();
    }

    let mut final_process_bash = Command::new("sh");
    // final_process_bash.arg("-c").arg(
    //     r#"
    //     rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
    // "#,
    // );
    final_process_bash
        .arg("-c")
        .arg(&app_config.ros_config.stop_bash);

    let final_process_bash_output = final_process_bash
        .output()
        .expect("failed to execute process");
    warn!(
        "final_process_bash: {:?}, output :{:?}",
        &app_config.ros_config.stop_bash, final_process_bash_output
    );

    warn!("exit t2");

    signal_handler.stop();
}
