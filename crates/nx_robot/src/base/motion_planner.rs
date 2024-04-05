use itertools::{izip, Itertools};
use nx_common::common::interpolate::compute_path_flow;
use nx_common::common::transform2d::Transform2d;
use nx_common::{angle_norm, log, vector_2d_dist2, vector_2d_norm2};
use serde::Deserialize;
use std::cell::RefCell;
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};
use std::ops::{Deref, DerefMut};
use std::rc::Rc;

use crate::base::motion_controller::{ControllerMotionDirection, RobotControl};

use nx_common::common::math::find_roots_of_quadratic_equation;
use nx_common::common::statistic::{MovingAverage, SlidingAverage};
use tracing::{error, event, info, instrument, span, trace, warn, Level};

#[derive(Debug, Deserialize, Copy, Clone)]
pub struct RobotMotionPlannerOption
{
    front_control_point: [f64; 3],
    rear_control_point: [f64; 3],

    //
    front_control_point_low_error: [f64; 3],
    rear_control_point_low_error: [f64; 3],

    control_program: u8,

    // constrain

    // start
    start_inplace_rotate_tolerance: f64,

    // new add
    valid_line_length: f64,
    valid_curve_length: f64,
    line_angle_change: f64,
    closest_node_search_dist: f64,
    switch_line_dist: f64,
    switch_line_path_dist: f64,
    curve_linear_fix: f64,
    track_curve_vel: f64,
    track_curve_end_vel: f64,
    track_curve_start_vel: f64,
    track_curve_end_dist: f64,
    track_curve_vel_decay_ratio: f64,
    track_curve_vel_decay_min: f64,

    curve_search_min: f64,
    curve_search_max: f64,
    curve_use_closest_direction_dist: f64,
    line_search_dist: f64,
    line_search_dist_min: f64,
    line_search_converge_dist: f64,
    line_search_converge_ratio: f64,
    line_switch_dist: f64,
    line_converge_y_acc: f64,
    line_converge_y_min: f64,
    line_converge_vel: f64,
    line_vel: f64,
    goal_line_vel: f64,
    goal_line_final: f64,
    line_converge_y: f64,
    line_converge_yaw: f64,
    final_force_parallel_direction: bool,
    final_relative_target_x: f64,

    speed_down_acc: f64,
    path_sparse_dist: f64,

    control_closest_alert: f64,
    control_not_converge_check_dist: f64,
    control_not_converge_alert: f64,

    pose_jump_alert_x: f64,
    pose_jump_alert_y: f64,
    pose_jump_alert_yaw: f64,

    // allow, reject
    // condition 1: first segment is not line
    // condition 2: fist segment length is not enough
    // condition 3: y or yaw offset too large

    // control record
    // start pose
    // check forward distance,
    // choose point in current direction to avid rotate
    // or choose

    // allow, reject implement position
    // request_path

    // control implement position
    // follow_path
    constrain_start_no_rotate: bool,
    constrain_start_no_rotate_run_dist: f64,
    constrain_start_no_rotate_allow_offset_y: f64,
    constrain_start_no_rotate_allow_offset_yaw: f64,
    constrain_start_no_rotate_allow_vel: f64,

    // line track point
    track_curve_max_vel: f64,
    // dist = vel * ratio
    track_line_to_curve_switch_dist_ratio: f64,
    track_line_to_curve_prediction_normal: f64,
    track_line_to_curve_allow_offset_y: f64,

    // curve converge
    track_curve_to_line_switch_dist_ratio: f64,
    track_curve_to_line_search_y: f64,
    track_curve_smooth_direction_ratio: f64,
    track_curve_smooth_direction_offset: f64,

    //track line
    // track_line_forward_vel_x_max: f64,

    // track final line
    track_final_line_forward_vel_y_acc: f64,
    track_final_line_forward_vel_x_max: f64,
    track_final_line_forward_vel_y_max: f64,
    track_final_line_forward_vel_x_min: f64,
    track_final_line_forward_vel_y_min: f64,
    track_final_line_prediction_normal: f64,

    track_final_line_rotate_vel: f64,
    track_final_line_converge_y: f64,

    track_final_line_converge_offset_base_y: f64,
    track_final_line_converge_direction_fix: f64,
    track_final_line_converge_direction_fix_min: f64,

    // y = track_line_prediction_final, ( 0.0 < x < track_line_prediction_change )
    // y = track_line_prediction_normal, (track_line_prediction_change < x )
    track_line_prediction_normal: f64,
    track_line_prediction_final: f64,
    track_line_prediction_change: f64,
    // track_line_offset_x: f64,
    track_line_allow_offset_y: f64,
    track_line_allow_offset_yaw: f64,

    // when to check mode,
    // change control_closest_node_id
    // line to curve: change search id according to
    // curve to line
    // any to goal

    // stop
    constrain_stop_no_rotate: bool,
    constrain_stop_no_rotate_run_dist: f64,
    constrain_stop_no_rotate_allow_offset_y: f64,
    constrain_stop_no_rotate_allow_offset_yaw: f64,

    constrain_stop_forward_vel_x_min: f64,
    constrain_stop_forward_vel_y_min: f64,
    reach_goal_num: u32,
    // tolerance
    constrain_stop_reaching_dist: f64,
    constrain_stop_allow_offset_x: f64,
    constrain_stop_allow_offset_y: f64,
    constrain_stop_allow_offset_yaw: f64,

    // raise error
    allow_final_rotate: bool,
    allow_final_rotate_angle: f64,

    allow_start_offset_dist: f64,
    allow_goal_dist: f64,

    // rotate, follow_path, follow_goal, final_goal

    // follow_goal use : follow_goal_no_rotate_on_line, allow_follow_goal_forward_vel
    // final_goal: allow_final_goal_forward_vel
    start_follow_goal_dist: f64,
    start_final_goal_dist: f64,
    follow_goal_no_rotate_on_line: bool,

    allow_follow_goal_forward_vel: f64,

    allow_forward_vel: f64,
    allow_curve_forward_vel: f64,
    allow_line_forward_vel: f64,

    allow_final_forward_dist: f64,
    allow_final_goal_forward_vel: f64,
    allow_final_forward_dw_acc: f64,
    allow_rotate_vel: f64,

    // offset tolerance
    line_offset_tolerance: f64,
    curve_offset_tolerance: f64,
    final_offset_tolerance: f64,
    curve_offset_tolerance_fix: f64,
    curve_offset_converge: f64,

    line_predict_dist_min: f64,
    curve_predict_dist_min: f64,
    final_predict_dist_min: f64,
    final_line_predict_dist_min: f64,
    final_curve_predict_dist_min: f64,

    line_predict_dist_max: f64,
    curve_predict_dist_max: f64,
    final_predict_dist_max: f64,
    final_line_predict_dist_max: f64,
    final_curve_predict_dist_max: f64,

    rotate_check_dist: f64,
}
#[derive(Debug, Deserialize, Clone)]
pub struct RobotMotionPlannerConfig
{
    model: String,
    default_option: String,
    options: HashMap<String, RobotMotionPlannerOption>,
    goal_reach_check_num: u32,
    sliding_len: usize,
}

#[derive(Default, Copy, Clone, Debug)]
pub struct PathNode
{
    pub pose: [f64; 3],
    flow: [f64; 3],
    pub flow_edge_min: [f64; 3],
    pub flow_edge_max: [f64; 3],
    flow_in_current: [f64; 3],
    flow_into_direction: f64,
    flow_in_current_direction: f64,
    flow_in_current_direction_min: f64,
    flow_in_current_direction_max: f64,
    segment_id: u32,
    segment_end_id: usize,
    flow_pose: Transform2d,
    flow_pose_inv: Transform2d,
    pub flow_angle_from_last: f64,

    // -1: turn negative, 0: strait line, 1 : turn positive, 2 : final goal
    flow_type: i32,
    dist_from_last: f64,
    dist_from_start: f64,
    pub dist_to_end: f64,
    dist_to_segment_end: f64,
    dist_from_segment_start: f64,

    pub allow_forward_vel: f64,
    offset_tolerance: f64,
    predict_dist_min: f64,
    predict_dist_max: f64,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RobotMotionPlannerError
{
    OK,
    LargeOffset,

    PoseJump,
    RequestError,
    ClosestDistError,
    GoalConvergeDistError,

    UnknownProgram,

    UnexpectedSparseCurve,

    UnexpectedUndefinedOption,

    UnexpectedPathEmpty,
    UnexpectedStartRotate,
    UnexpectedShortCurve,

    UnexpectedStartNoRotateDist,

    UnexpectedStopRotate,
    UnexpectedStopNoRotateDist,

    UnexpectedModel,

    UnexpectedStartOffset,
    UnexpectedGoalDist,

    UnexpectedControlError,
    UnexpectedSearchClosestError,

    UnexpectedFinalToleranceError,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RobotMotionPlannerState
{
    Idle,
    Init,
    FirstRotate,
    Prepare,
    FollowPath,
    FollowGoal,
    Finished,
    Error,

    DebugStop,
}
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RobotMotionPlannerTask
{
    Idle,
    Init,
    Line,
    // StartRotate,
    StartNoRotate,
    StopNoRotate,
    Finished,
    StopRotate,
    Curve,
    LineToCurve,
    CurveToLine,
    LineToLine,
    CurveToCurve,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SelectOptionErr
{
    Ok,
    NotFound,
    UseDefault,
}
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RobotMotionPlannerData
{
    pub state: RobotMotionPlannerState,
    pub error: RobotMotionPlannerError,
    pub task: RobotMotionPlannerTask,
    pub task_counter: usize,

    pub closest_base_id: usize,
    pub closest_control_id: usize,
    pub target_start_id: usize,
    pub target_end_id: usize,
}

impl Default for RobotMotionPlannerData
{
    fn default() -> Self
    {
        Self {
            state: RobotMotionPlannerState::Idle,
            error: RobotMotionPlannerError::OK,
            task: RobotMotionPlannerTask::Idle,
            task_counter: 0,
            closest_base_id: 0,
            closest_control_id: 0,
            target_start_id: 0,
            target_end_id: 0,
        }
    }
}

pub struct RobotMotionPlanner
{
    controller: Rc<RefCell<dyn RobotControl>>,
    pub planner_data: Rc<RefCell<RobotMotionPlannerData>>,
    config: RobotMotionPlannerConfig,
    current_pose_transform: Rc<RefCell<Transform2d>>,
    pub current_pose: Rc<RefCell<[f64; 3]>>,
    pub current_control_pose: Rc<RefCell<[f64; 3]>>,
    pub current_target_pose: Rc<RefCell<[f64; 3]>>,
    pub current_target_in_control_pose: Rc<RefCell<[f64; 3]>>,

    select_option: Rc<RefCell<Option<RobotMotionPlannerOption>>>,
    pub global_path: Rc<RefCell<Vec<PathNode>>>,
    path_flow: Rc<RefCell<Vec<[f64; 3]>>>,

    use_aux_control: bool,
    is_first_control: Rc<RefCell<bool>>,
    is_curve_converge: Rc<RefCell<bool>>,

    aux_control_point: Rc<RefCell<[f64; 3]>>,
    // robot pose at each task start point
    record_base_pose: Rc<RefCell<[f64; 3]>>,

    pub segment_end_in_control_pose: Rc<RefCell<[f64; 3]>>,

    pub path_end_in_control_pose: Rc<RefCell<[f64; 3]>>,

    pub reach_goal_num: Rc<RefCell<u32>>,
    pub track_goal_num: Rc<RefCell<u32>>,
    pub track_goal_direction: Rc<RefCell<f64>>,

    pub off_path_reach_num: Rc<RefCell<u32>>,
    pub forward_vel: Rc<RefCell<f64>>,
    pub current_control_pose_in_path_smooth_y: Rc<RefCell<MovingAverage>>,
    pub current_control_pose_relative_smooth_y: Rc<RefCell<SlidingAverage>>,
    pub current_base_pose_in_goal_smooth_x: Rc<RefCell<SlidingAverage>>,

    // new add
    pub current_control_pose_in_goal_flow: Rc<RefCell<[f64; 3]>>,
    pub current_base_pose_in_goal_flow: Rc<RefCell<[f64; 3]>>,
    //current_driver_cmd: [vel, angle, relative_angle]
    pub current_driver_state: Rc<RefCell<[f64; 3]>>,

    // target, current_in_target
    pub current_first_rotate_target: Rc<RefCell<[f64; 2]>>,

    // forward_vel,  dist, angle_diff
    pub current_target_diff: Rc<RefCell<[f64; 3]>>,
    // vx, vy
    pub current_vel_in_target_end: Rc<RefCell<[f64; 2]>>,
    //x,y,yaw
    pub current_control_pose_in_closest: Rc<RefCell<[f64; 3]>>,

    //
    pub current_control_pose_in_target_end_y: Rc<RefCell<SlidingAverage>>,
    pub current_control_pose_in_target_end_x: Rc<RefCell<SlidingAverage>>,
}

impl RobotMotionPlanner
{
    const SINGLE_DIFF_MODEL: &'static str = "SingleDiff";
    const DOUBLE_DIFF_MODEL: &'static str = "DoubleDiff";
    const DOUBLE_DIFF_ASSY_MODEL: &'static str = "DoubleDiffASSY";
    const TRIPLE_DIFF_ASSY_MODEL: &'static str = "TripleDiffASSY";

    const SINGLE_STEERING_MODEL: &'static str = "SingleSteering";
    const DOUBLE_STEERING_MODEL: &'static str = "DoubleSteering";

    pub fn new(
        mut config: RobotMotionPlannerConfig,
        controller: Rc<RefCell<dyn RobotControl>>,
    ) -> Self
    {
        let mut use_aux_control = false;
        match config.model.as_str() {
            Self::SINGLE_DIFF_MODEL => {
                use_aux_control = true;
            }
            Self::DOUBLE_DIFF_MODEL => {
                todo!()
            }
            Self::DOUBLE_DIFF_ASSY_MODEL => {
                todo!()
            }
            Self::TRIPLE_DIFF_ASSY_MODEL => {
                todo!()
            }
            Self::SINGLE_STEERING_MODEL => {
                use_aux_control = true;
            }
            Self::DOUBLE_STEERING_MODEL => {
                todo!()
            }
            _ => {}
        }

        let sliding_buffer_len = config.sliding_len;
        Self {
            controller,
            // state: Rc::new(RefCell::new(RobotMotionPlannerState::Idle)),
            // error: Rc::new(RefCell::new(RobotMotionPlannerError::OK)),
            // task: Rc::new(RefCell::new(RobotMotionPlannerTask::Idle)),
            planner_data: Rc::new(RefCell::new(Default::default())),
            config,
            current_pose_transform: Rc::new(RefCell::new(Default::default())),
            current_pose: Rc::new(RefCell::new([0.0; 3])),
            current_control_pose: Rc::new(RefCell::new([0.0; 3])),
            current_target_pose: Rc::new(RefCell::new([0.0; 3])),
            current_target_in_control_pose: Rc::new(RefCell::new([0.0; 3])),
            select_option: Rc::new(RefCell::new(None)),
            global_path: Rc::new(RefCell::new(vec![])),
            path_flow: Rc::new(RefCell::new(vec![])),

            use_aux_control,
            is_first_control: Rc::new(RefCell::new(true)),
            is_curve_converge: Rc::new(RefCell::new(false)),
            aux_control_point: Rc::new(RefCell::new([0.0; 3])),
            record_base_pose: Rc::new(RefCell::new([0.0; 3])),
            segment_end_in_control_pose: Rc::new(RefCell::new([0.0; 3])),
            path_end_in_control_pose: Rc::new(RefCell::new([0.0; 3])),
            reach_goal_num: Rc::new(RefCell::new(0)),
            track_goal_num: Rc::new(RefCell::new(0)),
            track_goal_direction: Rc::new(RefCell::new(0.0)),
            off_path_reach_num: Rc::new(RefCell::new(0)),
            forward_vel: Rc::new(RefCell::new(0.0)),
            current_control_pose_in_path_smooth_y: Rc::new(RefCell::new(Default::default())),
            current_control_pose_relative_smooth_y: Rc::new(RefCell::new(SlidingAverage::new(
                sliding_buffer_len,
            ))),
            current_base_pose_in_goal_smooth_x: Rc::new(RefCell::new(SlidingAverage::new(
                sliding_buffer_len,
            ))),
            current_control_pose_in_goal_flow: Rc::new(RefCell::new([0.0; 3])),
            current_base_pose_in_goal_flow: Rc::new(RefCell::new([0.0; 3])),
            current_driver_state: Rc::new(RefCell::new([0.0; 3])),
            current_first_rotate_target: Rc::new(RefCell::new([0.0, 0.0])),
            current_target_diff: Rc::new(RefCell::new([0.0; 3])),
            current_vel_in_target_end: Rc::new(RefCell::new([0.0; 2])),
            current_control_pose_in_closest: Rc::new(RefCell::new([0.0; 3])),
            current_control_pose_in_target_end_y: Rc::new(RefCell::new(SlidingAverage::new(
                sliding_buffer_len,
            ))),
            current_control_pose_in_target_end_x: Rc::new(RefCell::new(SlidingAverage::new(
                sliding_buffer_len,
            ))),
        }
    }

    pub fn set_option(&self, option_name: &str) -> SelectOptionErr
    {
        self.config.options.contains_key(option_name);

        let mut result = SelectOptionErr::Ok;

        let c = &self.config;
        let option_name = if c.options.contains_key(option_name) {
            option_name
        } else {
            result = SelectOptionErr::UseDefault;
            log!(
                "option_name [{}] doesn't exist, use default [{}]",
                option_name,
                c.default_option
            );
            &c.default_option
        };
        let binding = c.options.get(option_name);
        let opt = match binding.as_ref() {
            Some(t) => {
                *self.select_option.borrow_mut() = Some(**t);
                t
            }
            None => {
                result = SelectOptionErr::NotFound;
                log!("option_name [{}] doesn't exist", option_name);
                return result;
            }
        };
        result
    }

    //process path, check constrains, reject invalid task
    //rejection condition 1: stat no rotate , line segment
    //rejection condition 2: stop no rotate , line segment
    // todo: add no rotate constrains,
    // todo: check final inplace rotate, now it's invalid
    // todo: remove duplicate node
    // todo: add speed constrains
    pub fn request_path(&self, path: &Vec<[f64; 3]>) -> bool
    {
        self.reset();

        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                self.planner_data.borrow_mut().error =
                    RobotMotionPlannerError::UnexpectedUndefinedOption;
                log!("select_option doesn't exist");
                return false;
            }
        };
        let current_pose = *self.current_pose.borrow();
        let path_len = path.len();

        if path_len < 2 {
            self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedPathEmpty;
            return false;
        }
        let mut path = path.clone();

        {
            let mut check_id: i32 = 1;
            let check_id_max: i32 = path_len as i32;
            let mut valid_len: usize = 1;
            while check_id < check_id_max {
                let i = check_id as usize;
                let dist = vector_2d_dist2!(path[i], path[valid_len - 1]);
                path[valid_len] = path[i];
                valid_len += (dist > 1e-6) as usize;
                check_id += 1;
            }
            path.truncate(valid_len);
        }
        let path_len = path.len();
        info!("request_path: {:?}", path);

        // check dist to path
        {
            let mut min_dist_id = 0;
            let mut min_dist = 100000.0;
            for (i, x) in path.iter().enumerate() {
                let dist = vector_2d_dist2!(x, current_pose);

                if dist < min_dist {
                    min_dist_id = i;
                }
                min_dist = min_dist.min(dist);
            }

            min_dist = min_dist.sqrt();
            if min_dist > opt.allow_start_offset_dist {
                self.planner_data.borrow_mut().error =
                    RobotMotionPlannerError::UnexpectedStartOffset;
                return false;
            }

            self.planner_data.borrow_mut().closest_base_id = min_dist_id;
            self.planner_data.borrow_mut().closest_control_id = min_dist_id;

            log!("min_dist: {} , allow_start_offset_dist: {}, min_dist_id : {},current_pose: {:?}, path[min_dist_id] : {:?} , path[0]: {:?}",
                min_dist,opt.allow_start_offset_dist,min_dist_id, current_pose, path[min_dist_id],path[0]);
        }
        #[cfg(no_check_goal_dist)]
        {
            let dist_to_goal = (vector_2d_dist2!(path[path_len - 1], current_pose)).sqrt();

            if opt.allow_goal_dist > 0.0 && (dist_to_goal < opt.allow_goal_dist) {
                self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedGoalDist;
                return false;
            }
        }

        self.path_flow.borrow_mut().resize(path.len(), [0.0; 3]);
        self.global_path
            .borrow_mut()
            .resize(path.len(), Default::default());

        for (i, j) in izip!(&path, &mut *self.path_flow.borrow_mut()) {
            *j = *i;
        }
        for (i, j) in izip!(&path, &mut *self.global_path.borrow_mut()) {
            j.pose = *i;
        }

        compute_path_flow(self.path_flow.borrow_mut().deref_mut());

        for (i, j) in izip!(
            &mut *self.global_path.borrow_mut(),
            &mut *self.path_flow.borrow_mut()
        ) {
            i.flow = *j;
            i.flow_pose.set(j[0], j[1], j[2]);
            i.flow_pose_inv = i.flow_pose.inverse();
        }

        let path_flow_last_yaw = self.path_flow.borrow().last().unwrap()[2];
        let path_last_yaw = path.last().unwrap()[2];

        let mut final_rotate = false;
        if path.len() > 1 {
            let [x1, y1, yaw1] = path[path.len() - 2];
            let [x2, y2, yaw2] = path[path.len() - 1];

            final_rotate = (x1 - x2).abs() < 0.001
                && (y1 - y2).abs() < 0.001
                && (angle_norm!(yaw1 - yaw2)).abs() > opt.allow_final_rotate_angle;
        }

        {
            // check controller driver motion direction
            if self.config.model == Self::SINGLE_STEERING_MODEL {
                let path_last = self.global_path.borrow()[path_len - 1].pose;
                let flow_last = self.global_path.borrow()[path_len - 1].flow;
                info!(?path_last, ?flow_last);

                // if (vector_2d_dist2!(current_pose, path_last)).sqrt() < 0.05 {
                //     self.controller
                //         .borrow()
                //         .set_motion_direction(ControllerMotionDirection::Rotate);
                //     info!(?path_last, ?flow_last);
                // } else

                if (angle_norm!(path_last[2] - flow_last[2])).abs() < FRAC_PI_2 {
                    self.controller
                        .borrow()
                        .set_motion_direction(ControllerMotionDirection::Forward);
                    *self.aux_control_point.borrow_mut() = opt.front_control_point;

                    *self.current_target_in_control_pose.borrow_mut() = [1.1, 0.0, 0.0];
                    info!(?path_last, ?flow_last);
                } else {
                    self.controller
                        .borrow()
                        .set_motion_direction(ControllerMotionDirection::Backward);
                    *self.aux_control_point.borrow_mut() = opt.rear_control_point;

                    *self.current_target_in_control_pose.borrow_mut() = [-1.1, 0.0, 0.0];
                    info!(?path_last, ?flow_last);
                }
                warn!(
                    "*self.aux_control_point: {:?}",
                    *self.aux_control_point.borrow_mut()
                );
            } else {
                self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedModel;
                return false;
            }
        }

        {
            let current_pose = *self.current_pose.borrow();
            let current_pose_transform = Transform2d::new(&current_pose);
            let aux_control_point = *self.aux_control_point.borrow();

            let aux_control_point_abs = current_pose_transform.multiply_pose(&aux_control_point);

            *self.current_target_pose.borrow_mut() = aux_control_point_abs;

            *self.current_control_pose.borrow_mut() = aux_control_point_abs;

            // let closest_control_id_search_dist =
            //     (0.2 + (vector_2d_norm2!(aux_control_point)).sqrt());
            // let (closest_control_id, closest_control_end_id, closest_control_dist) = self
            //     .find_closest_node(
            //         min_dist_id,
            //         &aux_control_point_abs,
            //         closest_control_id_search_dist,
            //     );
            // self.planner_data.borrow_mut().closest_control_id = closest_control_id;
        }

        // let current_pose_x = 0.5;
        // let current_pose_y = 0.5;

        // split and meager, compute node info
        self.global_path.borrow_mut()[0].flow_angle_from_last = 0.0;

        for i in 1..path_len {
            let mut dist_from_last: f64 = 0.0;
            let mut flow_angle_from_last = 0.0;
            {
                let p1 = &self.global_path.borrow()[i - 1].pose;
                let p2 = &self.global_path.borrow()[i].pose;
                dist_from_last =
                    ((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1])).sqrt();
            }
            {
                let p1 = &self.global_path.borrow()[i - 1].flow;
                let p2 = &self.global_path.borrow()[i].flow;
                flow_angle_from_last = angle_norm!(p2[2] - p1[2]);
            }
            {
                self.global_path.borrow_mut()[i].dist_from_last = dist_from_last;
            }
            {
                self.global_path.borrow_mut()[i].flow_angle_from_last = flow_angle_from_last;
            }
        }
        {
            // let mut flow_angle_from_last = self.global_path.borrow()[1].flow_angle_from_last;
            //
            // self.global_path.borrow_mut()[0].flow_angle_from_last = flow_angle_from_last;
        }

        for i in 1..path_len {
            let mut dist_from_start: f64 = 0.0;
            {
                let p1 = &self.global_path.borrow()[i - 1];
                let p2 = &self.global_path.borrow()[i];
                dist_from_start = p1.dist_from_start + p2.dist_from_last;
            }
            {
                self.global_path.borrow_mut()[i].dist_from_start = dist_from_start;
            }
        }
        let last_dist_from_start = self.global_path.borrow().last().unwrap().dist_from_start;
        let mut sparse_line_flow_type = 11;

        for i in 0..path_len {
            let mut dist_to_end: f64 = 0.0;
            {
                let p2 = &self.global_path.borrow()[i];

                dist_to_end = last_dist_from_start - p2.dist_from_start;
            }
            self.global_path.borrow_mut()[i].dist_to_end = dist_to_end;

            self.global_path.borrow_mut()[i].allow_forward_vel = opt.allow_forward_vel;

            let flow_angle_from_last = self.global_path.borrow()[i].flow_angle_from_last;

            let mut flow_type: i32 = 0;
            {
                if (flow_angle_from_last.abs() < opt.line_angle_change) {
                    self.global_path.borrow_mut()[i].offset_tolerance = opt.line_offset_tolerance;
                    self.global_path.borrow_mut()[i].predict_dist_min = opt.line_predict_dist_min;
                    self.global_path.borrow_mut()[i].predict_dist_max = opt.line_predict_dist_max;
                    flow_type = 10;
                } else if flow_angle_from_last > 0.0 {
                    self.global_path.borrow_mut()[i].offset_tolerance = opt.curve_offset_tolerance;
                    self.global_path.borrow_mut()[i].predict_dist_min = opt.curve_predict_dist_min;
                    self.global_path.borrow_mut()[i].predict_dist_max = opt.curve_predict_dist_max;
                    flow_type = 1;
                } else {
                    self.global_path.borrow_mut()[i].offset_tolerance = opt.curve_offset_tolerance;
                    self.global_path.borrow_mut()[i].predict_dist_min = opt.curve_predict_dist_min;
                    self.global_path.borrow_mut()[i].predict_dist_max = opt.curve_predict_dist_max;
                    flow_type = -1;
                };

                if dist_to_end < opt.start_follow_goal_dist {
                    self.global_path.borrow_mut()[i].offset_tolerance = opt.final_offset_tolerance;
                    self.global_path.borrow_mut()[i].predict_dist_min = opt.final_predict_dist_min;
                    self.global_path.borrow_mut()[i].predict_dist_max = opt.final_predict_dist_max;
                }
            }

            {
                if (i > 0) && (self.global_path.borrow()[i].dist_from_last > opt.path_sparse_dist) {
                    if flow_type == 10 {
                        flow_type = sparse_line_flow_type;

                        self.global_path.borrow_mut()[i - 1].flow_type = flow_type;
                        sparse_line_flow_type += 1;
                    } else {
                        warn!(
                            "UnexpectedSparseCurve at {},flow_type:{}, node: {:?} ",
                            i,
                            flow_type,
                            self.global_path.borrow()[i]
                        );

                        warn!("global_path: {:?}", *self.global_path.borrow());
                        self.planner_data.borrow_mut().error =
                            RobotMotionPlannerError::UnexpectedSparseCurve;
                        return false;
                    }
                }
            }
            self.global_path.borrow_mut()[i].flow_type = flow_type;
        }

        for i in 1..path_len {
            let mut is_same_flow_type = false;
            let mut segment_id = 0;
            {
                let p1 = &self.global_path.borrow()[i - 1];
                let p2 = &self.global_path.borrow()[i];
                is_same_flow_type = p1.flow_type == p2.flow_type;
                segment_id = p1.segment_id;
            }
            if !is_same_flow_type {
                self.global_path.borrow_mut()[i].segment_id = segment_id + 1;
            } else {
                self.global_path.borrow_mut()[i].segment_id = segment_id;
            }
        }

        // dist to segment end
        {
            let mut check_id: i32 = path_len as i32 - 1;
            let mut dist_to_end: f64 = self.global_path.borrow().last().unwrap().dist_to_end;
            let mut segment_id = self.global_path.borrow().last().unwrap().segment_id;
            let mut end_id = check_id as usize;
            while check_id >= 0 {
                let i = check_id as usize;
                let current_segment_id = self.global_path.borrow()[i].segment_id;
                let current_dist_to_end = self.global_path.borrow()[i].dist_to_end;

                if (current_segment_id != segment_id) {
                    segment_id = current_segment_id;
                    end_id = check_id as usize;
                    dist_to_end = self.global_path.borrow()[i].dist_to_end;
                }
                self.global_path.borrow_mut()[i].dist_to_segment_end =
                    current_dist_to_end - dist_to_end;
                self.global_path.borrow_mut()[i].segment_end_id = end_id;

                check_id -= 1;
            }
        }

        {
            let mut dist_from_start = self.global_path.borrow()[0].dist_from_start;
            // let mut segment_end_id = self.global_path.borrow()[0].segment_end_id;
            let mut segment_id = self.global_path.borrow()[0].segment_id;

            for i in 0..path_len {
                if (self.global_path.borrow()[i].segment_id != segment_id) {
                    dist_from_start = self.global_path.borrow()[i].dist_from_start;
                    segment_id = self.global_path.borrow()[i].segment_id;
                }

                let dist_from_segment_start =
                    self.global_path.borrow()[i].dist_from_start - dist_from_start;
                self.global_path.borrow_mut()[i].dist_from_segment_start = dist_from_segment_start;
            }
        }
        {
            // edge
            info!("check edge");
            let mut check_id: usize = 0;
            let curve_offset_tolerance_fix = opt.curve_offset_tolerance_fix;

            while (check_id < path_len) {
                let flow_type = self.global_path.borrow()[check_id].flow_type;

                let segment_end_id = self.global_path.borrow()[check_id].segment_end_id;

                let segment_end_flow_pose_inv =
                    self.global_path.borrow()[segment_end_id].flow_pose_inv;
                let segment_end_flow_pose = self.global_path.borrow()[segment_end_id].flow_pose;

                let mut current_flow: [f64; 3] = [0.0; 3];
                let mut current_flow_in_segment_end: [f64; 3] = [0.0; 3];
                let mut current_flow_pose;
                let is_short_curve = self.global_path.borrow()[check_id].dist_to_segment_end
                    < opt.valid_curve_length;

                if (flow_type >= 10) || is_short_curve {
                    for j in check_id..=segment_end_id {
                        let offset_tolerance = self.global_path.borrow()[j].offset_tolerance;
                        let offset_tolerance_2 = 2.0 * offset_tolerance;

                        current_flow_pose = self.global_path.borrow()[j].flow_pose;

                        self.global_path.borrow_mut()[j].flow_edge_min =
                            current_flow_pose.multiply_pose(&[0.0, -offset_tolerance, 0.0]);
                        self.global_path.borrow_mut()[j].flow_edge_max =
                            current_flow_pose.multiply_pose(&[0.0, offset_tolerance, 0.0]);
                    }
                    check_id = segment_end_id + 1;
                } else {
                    // if self.global_path.borrow()[check_id].dist_to_segment_end
                    //     < opt.valid_curve_length
                    // {
                    //     info!("UnexpectedShortCurve: check_id: {}, node: {:?}, valid_curve_length : {}",
                    //         check_id, self.global_path.borrow()[check_id],opt.valid_curve_length);
                    //     info!("self.global_path.borrow() = {:?}",*self.global_path.borrow());
                    //     *self.state.borrow_mut() = RobotMotionPlannerState::UnexpectedShortCurve;
                    //     return false;
                    //
                    //
                    //     continue;
                    // }

                    {
                        let center_id = (0.5 * ((check_id + segment_end_id) as f64)) as usize;
                        for j in check_id..=segment_end_id {
                            let curve_offset_tolerance_fix = if j > center_id {
                                opt.curve_offset_tolerance_fix
                            } else {
                                0.0
                            };

                            let offset_tolerance = self.global_path.borrow()[j].offset_tolerance;
                            let offset_tolerance_2 = 2.0 * offset_tolerance;

                            current_flow = self.global_path.borrow()[j].flow;
                            current_flow_in_segment_end =
                                segment_end_flow_pose_inv.multiply_pose(&current_flow);
                            current_flow_pose = self.global_path.borrow()[j].flow_pose;

                            // self.global_path.borrow_mut()[j].flow_edge_min = current_flow_pose.multiply_pose(&[0.0,-offset_tolerance,0.0]);
                            // self.global_path.borrow_mut()[j].flow_edge_max = current_flow_pose.multiply_pose(&[0.0,-offset_tolerance_2,0.0]);
                            // info!("check id: {}, current_flow: {:?}, current_flow_in_segment_end: {:?}, current_flow_pose: {:?}", j,current_flow,current_flow_in_segment_end,current_flow_pose);

                            if current_flow_in_segment_end[1] > offset_tolerance {
                                self.global_path.borrow_mut()[j].flow_edge_min = current_flow_pose
                                    .multiply_pose(&[0.0, -curve_offset_tolerance_fix, 0.0]);
                                self.global_path.borrow_mut()[j].flow_edge_max = current_flow_pose
                                    .multiply_pose(&[0.0, -offset_tolerance_2, 0.0]);
                            } else if current_flow_in_segment_end[1] < -offset_tolerance {
                                self.global_path.borrow_mut()[j].flow_edge_min = current_flow_pose
                                    .multiply_pose(&[0.0, curve_offset_tolerance_fix, 0.0]);
                                self.global_path.borrow_mut()[j].flow_edge_max = current_flow_pose
                                    .multiply_pose(&[0.0, offset_tolerance_2, 0.0]);
                            } else {
                                if current_flow_in_segment_end[1] > 0.0 {
                                    self.global_path.borrow_mut()[j].flow_edge_min =
                                        segment_end_flow_pose.multiply_pose(&[
                                            current_flow_in_segment_end[0],
                                            0.0,
                                            0.0,
                                        ]);
                                    self.global_path.borrow_mut()[j].flow_edge_max =
                                        segment_end_flow_pose.multiply_pose(&[
                                            current_flow_in_segment_end[0],
                                            -offset_tolerance,
                                            0.0,
                                        ]);
                                } else {
                                    self.global_path.borrow_mut()[j].flow_edge_min =
                                        segment_end_flow_pose.multiply_pose(&[
                                            current_flow_in_segment_end[0],
                                            0.0,
                                            0.0,
                                        ]);
                                    self.global_path.borrow_mut()[j].flow_edge_max =
                                        segment_end_flow_pose.multiply_pose(&[
                                            current_flow_in_segment_end[0],
                                            offset_tolerance,
                                            0.0,
                                        ]);
                                }
                            }
                        }
                    }

                    check_id = segment_end_id + 1;
                }
            }
        }
        // vel

        //todo: set forward vel
        #[cfg(no)]
        {
            let mut check_id: i32 = path_len as i32 - 1;

            let mut dist_to_end: f64 = 0.0;
            let mut segment_id = 0;
            let mut allow_forward_vel: f64 = 0.0;
            while check_id >= 0 {
                let i = check_id as usize;

                if self.global_path.borrow()[i].dist_to_end < opt.allow_final_forward_dist {
                    {
                        dist_to_end = self.global_path.borrow()[i].dist_to_end;
                        segment_id = self.global_path.borrow()[i].segment_id;
                    }
                    {
                        self.global_path.borrow_mut()[i].allow_forward_vel =
                            opt.allow_final_goal_forward_vel;
                    }
                } else {
                    if (self.global_path.borrow()[i].flow_type != 0) {
                        if self.global_path.borrow()[i].dist_to_end - dist_to_end
                            > opt.rotate_check_dist
                            || segment_id != self.global_path.borrow()[i].segment_id
                        {
                            dist_to_end = self.global_path.borrow()[i].dist_to_end;
                            segment_id = self.global_path.borrow()[i].segment_id;

                            let mut j = i;
                            let mut k = i;
                            let mut dj = dist_to_end - opt.rotate_check_dist;
                            let mut dk = dist_to_end + opt.rotate_check_dist;

                            while j < path_len {
                                if self.global_path.borrow()[j].dist_to_end < dj {
                                    break;
                                }

                                j += 1;
                            }
                            while k > 0 {
                                if self.global_path.borrow()[k].dist_to_end > dk {
                                    break;
                                }
                                k -= 1;
                            }

                            {
                                allow_forward_vel = (self.global_path.borrow()[k].dist_to_end
                                    - self.global_path.borrow()[j].dist_to_end)
                                    * opt.allow_rotate_vel
                                    / (angle_norm!(
                                        self.global_path.borrow()[j].flow[2]
                                            - self.global_path.borrow()[k].flow[2]
                                    ))
                                    .abs();
                            }

                            let last_allow_forward_vel =
                                self.global_path.borrow()[i].allow_forward_vel;
                            self.global_path.borrow_mut()[i].allow_forward_vel =
                                last_allow_forward_vel.min(allow_forward_vel);
                        }
                    }
                }
                check_id -= 1;
            }
        }

        //todo: set forward vel
        #[cfg(no)]
        {
            let mut check_id: i32 = path_len as i32 - 1;
            let mut dist_to_end: f64 = 0.0;
            let mut allow_forward_vel: f64 = 0.0;
            // let mut last_select_id = path_len - 1;
            let mut global_last_allow_forward_vel = 0.0;

            while check_id >= 0 {
                let i = check_id as usize;
                dist_to_end = self.global_path.borrow()[i].dist_to_end;
                allow_forward_vel = self.global_path.borrow()[i].allow_forward_vel;

                if dist_to_end > opt.allow_final_forward_dist {
                    if global_last_allow_forward_vel < opt.allow_forward_vel
                        || allow_forward_vel < opt.allow_forward_vel
                    {
                        let mut k = i;
                        let mut dk = dist_to_end + opt.rotate_check_dist;

                        while k > 0 {
                            if self.global_path.borrow()[k].dist_to_end > dk
                                || self.global_path.borrow()[k].allow_forward_vel
                                    < opt.allow_forward_vel
                            {
                                break;
                            }
                            k -= 1;
                        }

                        let mut allow_forward_vel_i = global_last_allow_forward_vel; //self.global_path.borrow()[i].allow_forward_vel;

                        let t = (self.global_path.borrow()[k].dist_to_end - dist_to_end)
                            / allow_forward_vel_i;

                        let dv = t * opt.allow_final_forward_dw_acc;
                        let mut allow_forward_vel_k = allow_forward_vel_i + dv;

                        let d = dv / (i as f64 - k as f64);

                        let mut allow_forward_vel_k = 0.0;

                        for j in k..=i {
                            allow_forward_vel_k = self.global_path.borrow()[j].allow_forward_vel;

                            // self.global_path.borrow_mut()[j].allow_forward_vel =   allow_forward_vel_k;//.min((i as f64 - j as f64 )*d + allow_forward_vel_i)   ;
                            self.global_path.borrow_mut()[j].allow_forward_vel =
                                allow_forward_vel_k
                                    .min((i as f64 - j as f64) * d + allow_forward_vel_i);
                        }
                        // self.global_path.borrow_mut()[k].allow_forward_vel = 0.0;

                        check_id = k as i32;
                        global_last_allow_forward_vel =
                            self.global_path.borrow()[k].allow_forward_vel;
                    }
                } else {
                    global_last_allow_forward_vel = self.global_path.borrow()[i].allow_forward_vel;
                    // dist_to_end = self.global_path.borrow()[i].dist_to_end;
                    // last_select_id = i;
                }

                check_id -= 1;
            }
        }

        #[cfg(test_log)]
        {
            println!("dist_from_start");
            for i in 0..path_len {
                print!("{}, ", self.global_path.borrow_mut()[i].dist_from_start);
            }
            println!("finish dist_from_start");

            println!("dist_to_end");
            for i in 0..path_len {
                print!("{}, ", self.global_path.borrow_mut()[i].dist_to_end);
            }
            println!("finish dist_to_end");
            println!("flow_angle_from_last");
            for i in 0..path_len {
                print!(
                    "{}, ",
                    self.global_path.borrow_mut()[i].flow_angle_from_last
                );
            }
            println!("finish flow_angle_from_last");
            println!("flow_type");
            for i in 0..path_len {
                print!("{}, ", self.global_path.borrow_mut()[i].flow_type);
            }
            println!("finish flow_type");
            println!("segment_id");
            for i in 0..path_len {
                print!("{}, ", self.global_path.borrow_mut()[i].segment_id);
            }
            println!("finish segment_id");

            println!("segment_end");
            for i in 0..path_len {
                print!(
                    "[{}, {}], ",
                    self.global_path.borrow()[i].segment_end_id,
                    self.global_path.borrow()[i].dist_to_segment_end
                );
            }
            println!("finish segment_end");
        }

        ///
        ///
        ///
        // info!("self.global_path:\n{:?}", self.global_path.borrow());

        //todo: check start rotate and final rotate, check flow type and segment length
        //todo: start: check the closest node
        let closest_base_id = self.planner_data.borrow().closest_base_id;
        let closest_base_node = self.global_path.borrow()[closest_base_id];

        if self.config.model == Self::SINGLE_STEERING_MODEL {
            if opt.constrain_start_no_rotate {
                //opt.start_inplace_rotate_tolerance
                let angle_diff = angle_norm!(closest_base_node.pose[2] - current_pose[2]);
                if ((closest_base_node.flow_type < 10)
                    || (angle_diff > opt.start_inplace_rotate_tolerance))
                {
                    warn!("UnexpectedStartRotate: closest_base_node: {:?}, angle_diff: {}, start_inplace_rotate_tolerance: {}",closest_base_node, angle_diff, opt.start_inplace_rotate_tolerance);
                    self.planner_data.borrow_mut().error =
                        RobotMotionPlannerError::UnexpectedStartRotate;
                    return false;
                } else if closest_base_node.dist_to_segment_end
                    < opt.constrain_start_no_rotate_run_dist
                {
                    warn!("UnexpectedStartNoRotateDist: closest_base_node: {:?}, dist_to_segment_end: {}, constrain_start_no_rotate_run_dist: {}",closest_base_node, closest_base_node.dist_to_segment_end, opt.constrain_start_no_rotate_run_dist);
                    self.planner_data.borrow_mut().error =
                        RobotMotionPlannerError::UnexpectedStartNoRotateDist;
                    return false;
                } else {
                    self.planner_data.borrow_mut().task = RobotMotionPlannerTask::StartNoRotate;
                    // *self.task.borrow_mut() = RobotMotionPlannerTask::Init;
                }
            } else {
                #[cfg(aaa)]
                {
                    if self.global_path.borrow()[closest_base_id].flow_type == 0
                        && self.global_path.borrow()[closest_base_id].dist_to_segment_end
                            > opt.valid_line_length
                    {
                        *self.task.borrow_mut() = RobotMotionPlannerTask::Line;
                    } else {
                        *self.task.borrow_mut() = RobotMotionPlannerTask::Curve;
                    }
                }
                self.planner_data.borrow_mut().task = RobotMotionPlannerTask::Init;
            }

            if opt.constrain_stop_no_rotate {
                if (self.global_path.borrow()[path_len - 1].flow_type < 10) {
                    self.planner_data.borrow_mut().error =
                        RobotMotionPlannerError::UnexpectedStopRotate;
                    return false;
                } else if self.global_path.borrow()[path_len - 1].dist_from_segment_start
                    < opt.constrain_stop_no_rotate_run_dist
                {
                    warn!("global path: {:?}", *self.global_path.borrow());
                    warn!(
                        "UnexpectedStopNoRotateDist, len: {}, opt.constrain_stop_no_rotate_run_dist: {}",
                        self.global_path.borrow()[path_len - 1].dist_from_segment_start,
                        opt.constrain_stop_no_rotate_run_dist
                    );

                    self.planner_data.borrow_mut().error =
                        RobotMotionPlannerError::UnexpectedStopNoRotateDist;
                    return false;
                } else {
                    // *self.task.borrow_mut() = RobotMotionPlannerTask::StartNoRotate;
                }
            }
        } else {
        }

        match self.config.model.as_str() {
            Self::SINGLE_STEERING_MODEL => {
                self.planner_data.borrow_mut().state = RobotMotionPlannerState::Init;

                return true;
            }
            Self::DOUBLE_STEERING_MODEL => {
                self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedModel;
                return false;
            }
            Self::SINGLE_DIFF_MODEL => {
                self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedModel;
                return false;
            }
            _ => {
                self.planner_data.borrow_mut().error = RobotMotionPlannerError::UnexpectedModel;
                return false;
            }
        }
    }

    pub fn reset(&self)
    {
        {
            *self.planner_data.borrow_mut() = Default::default();

            self.controller.borrow().stop();
            self.controller.borrow().reset();

            // *self.state.borrow_mut() = RobotMotionPlannerState::Idle;
            // *self.task.borrow_mut() = RobotMotionPlannerTask::Idle;
            // *self.error.borrow_mut() = RobotMotionPlannerError::OK;

            *self.current_control_pose_in_goal_flow.borrow_mut() = [0.0; 3];
            *self.current_base_pose_in_goal_flow.borrow_mut() = [0.0; 3];
            *self.current_driver_state.borrow_mut() = [0.0; 3];
        }

        *self.is_first_control.borrow_mut() = true;
        *self.is_curve_converge.borrow_mut() = false;

        *self.segment_end_in_control_pose.borrow_mut() = [0.0; 3];
        *self.path_end_in_control_pose.borrow_mut() = [0.0; 3];
        *self.current_target_in_control_pose.borrow_mut() = [0.0; 3];

        *self.reach_goal_num.borrow_mut() = 0;

        *self.track_goal_num.borrow_mut() = 0;

        *self.off_path_reach_num.borrow_mut() = 0;
        *self.forward_vel.borrow_mut() = 0.0;

        self.current_control_pose_in_path_smooth_y
            .borrow_mut()
            .reset();

        self.current_control_pose_relative_smooth_y
            .borrow_mut()
            .reset();
    }

    fn find_closest_node(
        &self,
        start_id: usize,
        search_point: &[f64; 3],
        stop_search_distance: f64,
    ) -> (usize, usize, f64)
    {
        let path_len = self.global_path.borrow().len();

        let stop_search_distance = stop_search_distance * stop_search_distance;

        let mut best_id = start_id;
        let mut end_id = start_id;
        let mut best_dist: f64 = 1000000.0;

        #[cfg(aaa)]
        {
            let mut current_pose_in_node = [0.0_f64; 3];

            if forward {
                for i in closest_node_id..path_len {
                    let dist = vector_2d_dist2!(self.global_path.borrow()[i].pose, current_pose);

                    current_pose_in_node = self.global_path.borrow()[i]
                        .flow_pose_inv
                        .multiply_pose(&current_pose);

                    if (current_pose_in_node[0] > -0.005) {
                        continue;
                    };

                    if (dist < best_dist) {
                        best_dist = dist;
                        best_id = i;
                    }

                    if (dist > 1.0) {
                        break;
                    }
                }
            } else {
            }
        }

        {
            for i in start_id..path_len {
                let dist = vector_2d_dist2!(self.global_path.borrow()[i].pose, search_point);
                end_id = i;

                if (dist < best_dist) {
                    best_dist = dist;
                    best_id = i;
                }
                if (dist > stop_search_distance) {
                    break;
                }
            }
        }
        (best_id, end_id, best_dist.sqrt())
    }

    pub fn rotate(&self, target: f64) -> bool
    {
        self.controller.borrow().rotate(target)
    }
    pub fn prepare(&self) -> bool
    {
        self.controller.borrow().is_prepared()
    }

    // command is determined by current pose, path, and constrains and task parameter

    // constrains:
    // workstation, pallet, door, wall

    // check path pattern and initial offset
    // 1. line
    // 2. curve - line
    // 3. line - curve
    // 4. curve
    // 5. line - curve - line
    // 6. xxx

    // 1.

    // search target point

    // check point

    // raise error

    //todo: implement Task
    // 1. StartRotate and StartNoRotate
    // 2. Switch:  StartRotate ->
    pub fn follow_path(&self) -> bool
    {
        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                log!("select_option doesn't exist");
                return false;
            }
        };

        // start_no_rotate
        // enter
        // exit

        // stop to goal
        // enter
        // exit

        // on line segment
        // enter
        // exit

        // on curve enter
        // enter
        // exit

        //

        let control_interval_s = self.controller.borrow().config().control_interval_s;

        let current_pose = *self.current_pose.borrow();

        let record_base_pose = *self.record_base_pose.borrow();

        let path_len = self.global_path.borrow().len();

        let current_pose_transform = Transform2d::new(&current_pose);
        let record_base_pose_transform = Transform2d::new(&record_base_pose);

        let current_pose_transform_inv = current_pose_transform.inverse();
        let record_base_pose_transform_inv = record_base_pose_transform.inverse();

        let current_pose_in_record = record_base_pose_transform_inv.multiply_pose(&current_pose);

        info!(
            "record_base_pose: {:?}, current_pose: {:?}, current_pose_in_record: {:?}",
            record_base_pose, current_pose, current_pose_in_record
        );
        let aux_control_point = *self.aux_control_point.borrow();

        let aux_control_point_abs = current_pose_transform.multiply_pose(&aux_control_point);
        let record_aux_control_point_abs =
            record_base_pose_transform.multiply_pose(&aux_control_point);

        // *self.current_target_pose.borrow_mut() = aux_control_point_abs;

        *self.current_control_pose.borrow_mut() = aux_control_point_abs;

        let aux_control_point_transform = Transform2d::new(&aux_control_point_abs);
        let aux_control_point_transform_inv = aux_control_point_transform.inverse();

        let mut task = self.planner_data.borrow_mut().task;
        info!("Task running on {:?}", task);

        let current_driver_cmd = self
            .controller
            .borrow()
            .get_driver_command(&aux_control_point);

        let [mut actual_command_forward_vel, mut actual_command_forward_angle] = current_driver_cmd;

        if aux_control_point[0] > 0.0 {
        } else {
            actual_command_forward_angle = angle_norm!(actual_command_forward_angle + PI);
            actual_command_forward_vel = -actual_command_forward_vel;
        }

        // info!("current_driver_cmd  {:?}", current_driver_cmd);

        // search the closest node
        let is_first_control = *self.is_first_control.borrow();
        *self.is_first_control.borrow_mut() = false;

        let mut closest_base_id = self.planner_data.borrow().closest_base_id;
        let mut closest_control_id = self.planner_data.borrow().closest_control_id;

        let closest_base_node = self.global_path.borrow()[closest_base_id];
        let closest_control_node = self.global_path.borrow()[closest_control_id];

        let mut closest_base_id_search_dist: f64 = closest_base_node.predict_dist_max;

        //
        // let closest_control_id_search_dist_check = (0.2 + (vector_2d_norm2!(aux_control_point)).sqrt());
        //
        // if closest_control_id_search_dist > closest_control_id_search_dist_check {
        //     closest_control_id_search_dist += 0.2;
        // }

        // closest_control_id_search_dist += closest_control_node.predict_dist_max;

        let mut closest_control_id_search_dist: f64 = if is_first_control {
            (0.2 + (vector_2d_norm2!(aux_control_point)).sqrt())
        } else {
            (vector_2d_dist2!(closest_control_node.pose, current_pose)).sqrt()
                + closest_control_node.predict_dist_max
        };

        let (closest_base_id, closest_base_end_id, closest_base_dist) =
            self.find_closest_node(closest_base_id, &current_pose, closest_base_id_search_dist);
        let (closest_control_id, closest_control_end_id, closest_control_dist) = self
            .find_closest_node(
                closest_control_id,
                &aux_control_point_abs,
                closest_control_id_search_dist,
            );

        self.planner_data.borrow_mut().closest_base_id = closest_base_id;
        self.planner_data.borrow_mut().closest_control_id = closest_control_id;

        let closest_base_node = self.global_path.borrow()[closest_base_id];
        let closest_control_node = self.global_path.borrow()[closest_control_id];

        let goal_node = self.global_path.borrow()[path_len - 1];
        let segment_end_node = self.global_path.borrow()[closest_control_node.segment_end_id];

        let segment_end_in_control_pose =
            aux_control_point_transform_inv.multiply_pose(&segment_end_node.pose);

        let goal_node_in_control_pose =
            aux_control_point_transform_inv.multiply_pose(&goal_node.pose);
        let goal_flow_in_control_pose =
            aux_control_point_transform_inv.multiply_pose(&goal_node.flow);

        let control_pose_in_segment_end = segment_end_node
            .flow_pose_inv
            .multiply_pose(&aux_control_point_abs);

        let mut current_target_in_control_pose = *self.current_target_in_control_pose.borrow();

        // state switch prio

        // check path segment
        // current segment, length , type
        // next segment, length, type
        // goal segment, length, type

        let mut is_on_goal_segment = closest_control_node.segment_end_id == path_len - 1;

        let is_reaching_goal = is_on_goal_segment
            && closest_control_node.dist_to_end < opt.constrain_stop_reaching_dist;

        let mut current_segment_is_line = closest_control_node.flow_type >= 10;
        let mut goal_segment_is_line = goal_node.flow_type >= 10;

        {
            *self.current_control_pose_in_goal_flow.borrow_mut() = goal_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);
            *self.current_base_pose_in_goal_flow.borrow_mut() =
                goal_node.flow_pose_inv.multiply_pose(&current_pose);
        }

        let goal_segment_length: f64 = 0.0;

        // switch rules:
        //

        #[cfg(aaa)]
        {
            if task == RobotMotionPlannerTask::Switch {
                self.controller.borrow().stop();
                if is_on_goal_segment {
                    if (is_reaching_goal) {
                        if opt.constrain_stop_no_rotate {
                            task = RobotMotionPlannerTask::StopNoRotate;
                        } else {
                            task = RobotMotionPlannerTask::StopRotate;
                        }
                    } else {
                        if current_segment_is_line {
                            task = RobotMotionPlannerTask::Line;
                        } else {
                            task = RobotMotionPlannerTask::Curve;
                        }
                    }
                } else {
                    let next_segment_start_node =
                        self.global_path.borrow()[closest_control_node.segment_end_id + 1];
                    let mut next_segment_is_line = next_segment_start_node.flow_type == 0;

                    if current_segment_is_line && next_segment_is_line {
                        task = RobotMotionPlannerTask::LineToLine;
                    } else if current_segment_is_line && !next_segment_is_line {
                        if control_pose_in_segment_end[0] > -0.05 {
                            task = RobotMotionPlannerTask::Curve;
                        } else {
                            task = RobotMotionPlannerTask::LineToCurve;
                        }
                    } else if !current_segment_is_line && next_segment_is_line {
                        task = RobotMotionPlannerTask::CurveToLine;
                    } else if !current_segment_is_line && !next_segment_is_line {
                        task = RobotMotionPlannerTask::CurveToCurve;
                    }
                }
            }
        }

        //todo: task == RobotMotionPlannerTask::Init
        // switch to Line, LineToCurve, CureToLine, Curve
        if task == RobotMotionPlannerTask::Init {
            if is_on_goal_segment {
                if current_segment_is_line {
                    task = RobotMotionPlannerTask::Line;
                } else {
                    task = RobotMotionPlannerTask::Curve;
                }

                self.planner_data.borrow_mut().target_start_id = closest_control_id;
                self.planner_data.borrow_mut().target_end_id = closest_control_node.segment_end_id;
                *self.track_goal_direction.borrow_mut() = actual_command_forward_angle;
                self.record_base_pose();
                self.current_control_pose_relative_smooth_y
                    .borrow_mut()
                    .reset();
                *self.track_goal_num.borrow_mut() = 0;
            } else {
                let next_segment_start_node =
                    self.global_path.borrow()[closest_control_node.segment_end_id + 1];
                let next_segment_end_node =
                    self.global_path.borrow()[next_segment_start_node.segment_end_id];

                let mut next_segment_is_line = next_segment_start_node.flow_type >= 10;

                if current_segment_is_line && next_segment_is_line {
                    *self.is_curve_converge.borrow_mut() = true;

                    task = RobotMotionPlannerTask::LineToLine;
                } else if current_segment_is_line && !next_segment_is_line {
                    *self.is_curve_converge.borrow_mut() = true;

                    task = RobotMotionPlannerTask::LineToCurve;
                } else if !current_segment_is_line && next_segment_is_line {
                    *self.is_curve_converge.borrow_mut() = false;

                    task = RobotMotionPlannerTask::CurveToLine;
                } else if !current_segment_is_line && !next_segment_is_line {
                    *self.is_curve_converge.borrow_mut() = false;

                    task = RobotMotionPlannerTask::CurveToCurve;
                }

                self.planner_data.borrow_mut().target_start_id = closest_control_id;
                self.planner_data.borrow_mut().target_end_id = closest_control_node.segment_end_id;
                *self.track_goal_direction.borrow_mut() = actual_command_forward_angle;
                self.record_base_pose();
                self.current_control_pose_relative_smooth_y
                    .borrow_mut()
                    .reset();
                *self.track_goal_num.borrow_mut() = 0;
            }
            info!("Task switch to {:?}", task);

            self.controller.borrow().stop();
        } else
        //todo: task == RobotMotionPlannerTask::StartNoRotate
        if task == RobotMotionPlannerTask::StartNoRotate {
            // run and switch state
            let mut allow_forward_vel: f64 = 0.1;
            current_target_in_control_pose = aux_control_point;

            *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
            *self.current_target_pose.borrow_mut() =
                aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

            self.controller.borrow().follow_with_control(
                &current_target_in_control_pose,
                allow_forward_vel,
                &aux_control_point,
            );

            info!(
                "current_pose_in_record: {:?}, constrain_start_no_rotate_run_dist: {}",
                current_pose_in_record, opt.constrain_start_no_rotate_run_dist
            );
            if current_pose_in_record[0].abs() > opt.constrain_start_no_rotate_run_dist {
                {
                    if !is_on_goal_segment {
                        let next_segment_start_node =
                            self.global_path.borrow()[closest_control_node.segment_end_id + 1];
                        let next_segment_end_node =
                            self.global_path.borrow()[next_segment_start_node.segment_end_id];

                        if next_segment_start_node.flow_type < 10 {
                            task = RobotMotionPlannerTask::LineToCurve;
                        } else {
                            task = RobotMotionPlannerTask::LineToLine;
                        }
                        self.planner_data.borrow_mut().target_start_id = closest_control_id;
                        self.planner_data.borrow_mut().target_end_id =
                            closest_control_node.segment_end_id;
                        *self.track_goal_direction.borrow_mut() = actual_command_forward_angle;
                        self.record_base_pose();
                        self.current_control_pose_relative_smooth_y
                            .borrow_mut()
                            .reset();
                        *self.track_goal_num.borrow_mut() = 0;
                    } else {
                        if current_segment_is_line {
                            if opt.constrain_stop_no_rotate {
                                task = RobotMotionPlannerTask::StopNoRotate;
                            } else {
                                task = RobotMotionPlannerTask::Line;
                            }
                            task = RobotMotionPlannerTask::Line;
                        } else {
                            task = RobotMotionPlannerTask::Curve;
                        }
                        self.planner_data.borrow_mut().target_start_id = closest_control_id;
                        self.planner_data.borrow_mut().target_end_id =
                            closest_control_node.segment_end_id;
                        *self.track_goal_direction.borrow_mut() = actual_command_forward_angle;
                        self.record_base_pose();
                        self.current_control_pose_relative_smooth_y
                            .borrow_mut()
                            .reset();
                        *self.track_goal_num.borrow_mut() = 0;
                    }
                }

                info!("Task switch to {:?}", task);
            }
        } else
        //todo: task == RobotMotionPlannerTask::CurveToCurve
        if task == RobotMotionPlannerTask::CurveToCurve {
            let mut target_start_id = self.planner_data.borrow().target_start_id;
            let mut target_end_id = self.planner_data.borrow().target_end_id;
            let mut valid_line_id = target_end_id;

            if target_end_id == path_len - 1 {
                self.controller.borrow().stop();
            }

            for i in target_start_id..path_len {
                if self.global_path.borrow()[i].flow_type >= 10
                    && self.global_path.borrow()[i].dist_to_segment_end > opt.valid_line_length
                {
                    valid_line_id = i;
                    break;
                }
            }

            if valid_line_id > target_end_id {
                self.planner_data.borrow_mut().target_start_id = target_start_id;
                self.planner_data.borrow_mut().target_end_id = valid_line_id - 1;
                task = RobotMotionPlannerTask::CurveToLine;
            } else {
                self.controller.borrow().stop();
            }
        }
        //todo: task == RobotMotionPlannerTask::Line
        // todo: add error detection, stop at large offset
        else if task == RobotMotionPlannerTask::Line {
            let mut track_goal_num = *self.track_goal_num.borrow();
            *self.track_goal_num.borrow_mut() += 1;

            let mut target_start_id = self.planner_data.borrow().target_start_id;
            let mut target_end_id = self.planner_data.borrow().target_end_id;
            info!(target_start_id, target_end_id);

            let target_end_node = self.global_path.borrow()[target_end_id];

            let target_end_node_is_final_line = (target_end_node.flow_type >= 10)
                && (target_end_node.segment_end_id == (path_len - 1));

            let control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);

            info!(?control_pose_in_target_end, ?target_end_node);

            let control_pose_in_target_end_dist =
                (vector_2d_norm2!(control_pose_in_target_end)).sqrt();

            let current_pose_in_target_end =
                target_end_node.flow_pose_inv.multiply_pose(&current_pose);
            let record_control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&record_aux_control_point_abs);

            self.current_control_pose_relative_smooth_y
                .borrow_mut()
                .update(control_pose_in_target_end[1]);
            let current_control_pose_relative_smooth_y =
                self.current_control_pose_relative_smooth_y.borrow().mean();

            info!(?control_pose_in_target_end, ?current_driver_cmd);

            let mut driver_direction_in_target_end =
                angle_norm!(control_pose_in_target_end[2] + actual_command_forward_angle);

            let current_driver_cmd_vx =
                actual_command_forward_vel * driver_direction_in_target_end.cos();
            let current_driver_cmd_vy =
                actual_command_forward_vel * driver_direction_in_target_end.sin();
            info!(
                driver_direction_in_target_end,
                current_driver_cmd_vx,
                current_driver_cmd_vy,
                actual_command_forward_vel,
                actual_command_forward_angle
            );

            let mut last_direction =
                current_target_in_control_pose[1].atan2(current_target_in_control_pose[0]);

            if (track_goal_num == 0) {
                *self.track_goal_direction.borrow_mut() = last_direction;
            }
            let mut track_goal_direction = *self.track_goal_direction.borrow();

            let mut track_final_line_converge_direction_fix =
                opt.track_final_line_converge_direction_fix;

            let mut track_final_line_converge_direction_fix_min =
                opt.track_final_line_converge_direction_fix_min;

            // if aux_control_point[0] > 0.0 {
            //      (angle_norm!(record_control_pose_in_target_end[2])).abs()
            //  }else{
            //  };

            // todo: cause jump?
            track_final_line_converge_direction_fix =
                (angle_norm!(record_control_pose_in_target_end[2] + track_goal_direction)).abs();

            let mut converge_vel: f64 = 0.0;

            {
                // converge vel limit
                // let mut allow_forward_vel: f64 = 0.5;

                let mut a: f64 = 0.0;
                let mut b: f64 = 0.0;
                let mut c: f64 = 0.0;
                let acc: f64 = opt.track_final_line_forward_vel_y_acc;

                let v0: f64 = opt.constrain_stop_forward_vel_y_min;
                let ds = control_pose_in_target_end[1].abs();

                a = 0.5 * acc;
                b = v0;
                c = -ds;
                let result = find_roots_of_quadratic_equation(a, b, c);
                if result.0 > 0 {
                    // converge_vel = v0 + acc * result.1;
                    // todo: test linear function
                    converge_vel = v0 + acc * ds;
                }
                info!(
                    "converge_vel find_roots_of_quadratic_equation: ds: {},acc: {}, v0:{}, vel: {}",
                    ds, acc, v0, converge_vel
                );
            }
            let mut stop_vel: f64 = 0.0;

            {
                // stop vel limit
                // let mut allow_forward_vel: f64 = 0.5;

                let mut a: f64 = 0.0;
                let mut b: f64 = 0.0;
                let mut c: f64 = 0.0;
                let acc: f64 = opt.allow_final_forward_dw_acc;

                let v0: f64 = opt.constrain_stop_forward_vel_x_min;
                let ds = current_pose_in_target_end[0].abs();

                if ds > 0.0 {
                    a = 0.5 * acc;
                    b = v0;
                    c = -ds;
                    let result = find_roots_of_quadratic_equation(a, b, c);
                    if result.0 > 0 {
                        stop_vel = v0 + acc * result.1;
                    }
                    info!(
                        "stop_vel find_roots_of_quadratic_equation: ds: {},acc: {}, v0:{}, vel: {}",
                        ds, acc, v0, stop_vel
                    );
                } else {
                    stop_vel = 0.0;
                }
            }

            info!(
                ?goal_flow_in_control_pose,
                ?goal_node_in_control_pose,
                stop_vel
            );

            let mut allow_forward_vel = opt.track_final_line_forward_vel_x_max.min(stop_vel);

            let constrain_stop_no_rotate_run_dist = opt.constrain_stop_no_rotate_run_dist + 0.1;

            if (control_pose_in_target_end[0] > -constrain_stop_no_rotate_run_dist) {}
            //
            {
                let mut vel_ratio = 1.0;
                vel_ratio = converge_vel / current_driver_cmd_vy.abs();

                vel_ratio = vel_ratio.min(1.0).max(0.0);
                let converge_vel = allow_forward_vel * vel_ratio;
                allow_forward_vel = allow_forward_vel.min(converge_vel);
                info!(
                    current_driver_cmd_vy,
                    current_driver_cmd_vx,
                    converge_vel,
                    ?control_pose_in_target_end,
                    constrain_stop_no_rotate_run_dist,
                    vel_ratio,
                    allow_forward_vel,
                    stop_vel
                );
            }

            let mut converge_direction_x = goal_flow_in_control_pose[2];

            let mut converge_direction_y =
                converge_direction_x - FRAC_PI_2.copysign(control_pose_in_target_end[1]);

            info!(converge_direction_x, converge_direction_y);

            let mut target_pose_in_target_end = control_pose_in_target_end;
            target_pose_in_target_end[0] += opt.track_line_to_curve_prediction_normal;
            target_pose_in_target_end[1] = 0.0;
            target_pose_in_target_end[2] = 0.0;

            let mut target_pose = target_end_node
                .flow_pose
                .multiply_pose(&target_pose_in_target_end);

            current_target_in_control_pose =
                aux_control_point_transform_inv.multiply_pose(&target_pose);

            // converge pose

            let mut target_direction = converge_direction_x; // 0.5*(actual_command_forward_angle + converge_direction_x);

            // need converge
            let mut ratio = -current_control_pose_relative_smooth_y.1
                / opt.track_final_line_converge_offset_base_y;

            ratio = ratio.min(1.0).max(-1.0);

            let mut is_curve_converge = *self.is_curve_converge.borrow();

            // checking offset
            // if offset is too large
            // set forward_vel to min_vel, move forward_to check point

            if current_control_pose_relative_smooth_y.0 && !is_curve_converge {
                if ((record_control_pose_in_target_end[1] > 0.0
                    && current_control_pose_relative_smooth_y.1 > opt.track_final_line_converge_y)
                    || (record_control_pose_in_target_end[1] < 0.0
                        && current_control_pose_relative_smooth_y.1
                            < -opt.track_final_line_converge_y))
                {
                    // target_direction = ratio*(converge_direction_y ) + (1.0 - ratio)*converge_direction_x ;
                    let mut fix = ratio * track_final_line_converge_direction_fix;
                    fix += track_final_line_converge_direction_fix_min.copysign(fix);
                    target_direction = converge_direction_x + fix;

                    // target_direction = track_goal_direction;
                } else {
                    is_curve_converge = true;
                }

                current_target_in_control_pose[0] = 0.5 * target_direction.cos();
                current_target_in_control_pose[1] = 0.5 * target_direction.sin();
            } else {
                if control_pose_in_target_end_dist > opt.constrain_stop_no_rotate_run_dist {
                    let mut target_pose_in_target_end_node = control_pose_in_target_end;
                    target_pose_in_target_end_node[0] += opt.track_final_line_prediction_normal;
                    target_pose_in_target_end_node[2] = 0.0;
                    let mut target_pose = target_end_node
                        .flow_pose
                        .multiply_pose(&target_pose_in_target_end_node);
                    target_pose[2] = target_end_node.pose[2];
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&target_pose);
                } else {
                    // todo: this will cause rotate, use fix direction
                    current_target_in_control_pose[0] = 0.5 * target_direction.cos();
                    current_target_in_control_pose[1] = 0.5 * target_direction.sin();

                    //
                    // current_target_in_control_pose = aux_control_point;
                }
            }
            *self.is_curve_converge.borrow_mut() = is_curve_converge;

            let mut control_error = false;
            control_error = (target_end_node_is_final_line
                && (control_pose_in_target_end_dist < opt.constrain_stop_no_rotate_run_dist)
                && !is_curve_converge)
                || (target_end_node_is_final_line
                    && is_curve_converge
                    && (control_pose_in_target_end[1].abs() > opt.constrain_stop_allow_offset_y));

            //check

            if (control_error) {
                warn!(
                    control_error,
                    is_curve_converge,
                    ?control_pose_in_target_end
                );

                self.controller.borrow().stop();
            } else if !current_control_pose_relative_smooth_y.0 {
                self.controller.borrow().stop();
            } else {
                info!(
                    target_direction,
                    actual_command_forward_angle, converge_direction_x
                );
                info!(
                    target_direction,
                    ratio, converge_direction_y, converge_direction_x
                );

                *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
                *self.current_target_pose.borrow_mut() =
                    aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

                info!(?current_target_in_control_pose, allow_forward_vel);

                self.controller.borrow().follow_with_control(
                    &current_target_in_control_pose,
                    allow_forward_vel,
                    &aux_control_point,
                );
            }

            // dist to goal
            // vel decomposition

            // reach goal
            // check counter

            if target_end_node_is_final_line {
                if current_pose_in_target_end[0] > -opt.constrain_stop_allow_offset_x {
                    *self.reach_goal_num.borrow_mut() += 1;
                }
                if *self.reach_goal_num.borrow() > opt.reach_goal_num {
                    task = RobotMotionPlannerTask::Finished;
                    self.controller.borrow().stop();
                }
            } else {
                if is_curve_converge {
                    let next_segment_start_node =
                        self.global_path.borrow()[target_end_node.segment_end_id + 1];
                    let next_segment_end_node =
                        self.global_path.borrow()[next_segment_start_node.segment_end_id];

                    if next_segment_start_node.flow_type >= 10 {
                        task = RobotMotionPlannerTask::LineToLine;
                    } else {
                        task = RobotMotionPlannerTask::LineToCurve;
                    }
                }
            }

            // self.controller.borrow().stop();
        } else
        // todo: task == RobotMotionPlannerTask::Curve
        if task == RobotMotionPlannerTask::Curve {
            self.controller.borrow().stop();
        } else if task == RobotMotionPlannerTask::LineToLine {
            self.controller.borrow().stop();
        } else
        // todo: task == RobotMotionPlannerTask::CurveToLine
        if task == RobotMotionPlannerTask::CurveToLine {
            // two stage, follow point and converge line

            let mut track_goal_num = *self.track_goal_num.borrow();
            *self.track_goal_num.borrow_mut() += 1;

            let mut target_start_id = self.planner_data.borrow().target_start_id;
            let mut target_end_id = self.planner_data.borrow().target_end_id;
            info!(target_start_id, target_end_id);

            let forward_dist = actual_command_forward_vel * control_interval_s;

            let target_end_node = self.global_path.borrow()[target_end_id];
            let control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);
            let record_control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&record_aux_control_point_abs);

            info!(?control_pose_in_target_end, ?target_end_node);

            self.current_control_pose_relative_smooth_y
                .borrow_mut()
                .update(control_pose_in_target_end[1]);
            let current_control_pose_relative_smooth_y =
                self.current_control_pose_relative_smooth_y.borrow().mean();

            let mut flow_in_current_direction: f64 = 0.0;
            let mut flow_in_current_direction_inc: f64 = 0.0;
            let mut flow_in_current_direction_min: f64 = 0.0;
            let mut flow_in_current_direction_max: f64 = 0.0;
            let mut flow_into_direction: f64 = 0.0;

            let mut flow_in_current;
            let mut flow_in_current_min;
            let mut flow_in_current_max;

            let mut first_valid_id: usize = 0;
            let mut find_first_valid_id = false;

            for i in target_start_id..=target_end_id {
                flow_in_current = aux_control_point_transform_inv
                    .multiply_pose(&self.global_path.borrow()[i].flow);
                flow_in_current_min = aux_control_point_transform_inv
                    .multiply_pose(&self.global_path.borrow()[i].flow_edge_min);
                flow_in_current_max = aux_control_point_transform_inv
                    .multiply_pose(&self.global_path.borrow()[i].flow_edge_max);

                flow_in_current_direction = flow_in_current[1].atan2(flow_in_current[0]);
                flow_in_current_direction_min =
                    flow_in_current_min[1].atan2(flow_in_current_min[0]);
                flow_in_current_direction_max =
                    flow_in_current_max[1].atan2(flow_in_current_max[0]);

                flow_into_direction = angle_norm!(flow_in_current[2] - flow_in_current_direction);

                // log!("id {}, flow_into_direction: {:?}, flow_in_current: {:?}",i, flow_into_direction,flow_in_current);
                if flow_into_direction.abs() < FRAC_PI_2 && first_valid_id == 0 {
                    first_valid_id = i;
                    find_first_valid_id = true;
                }

                // if flow_into_direction.abs() < best_flow_to_direction {
                //     best_id = i;
                //     best_flow_to_direction = flow_into_direction.abs();
                // }
                self.global_path.borrow_mut()[i].flow_in_current = flow_in_current;
                self.global_path.borrow_mut()[i].flow_into_direction = flow_into_direction;
                self.global_path.borrow_mut()[i].flow_in_current_direction =
                    flow_in_current_direction;
                self.global_path.borrow_mut()[i].flow_in_current_direction_min =
                    flow_in_current_direction_min;
                self.global_path.borrow_mut()[i].flow_in_current_direction_max =
                    flow_in_current_direction_max;
            }

            // first_valid_id = first_valid_id.max((0.5*(target_start_id as f64 + target_end_id as f64)  ) as usize);

            //
            {
                let mut max_predict_control_dist2: f64 =
                    opt.curve_predict_dist_max * opt.curve_predict_dist_max;
                let mut min_predict_control_dist2: f64 =
                    opt.curve_predict_dist_min * opt.curve_predict_dist_min;

                let mut last_direction =
                    current_target_in_control_pose[1].atan2(current_target_in_control_pose[0]);

                if actual_command_forward_vel.abs() > 0.001 {
                    last_direction = actual_command_forward_angle;
                }

                let mut best_direction_diff_min: f64 = 1000.0;
                let mut best_direction_diff_max: f64 = -1000.0;
                let mut best_direction_diff_abs: f64 = 1000.0;
                let mut best_direction = last_direction;

                let mut best_direction_id = first_valid_id;
                let mut select_target_id = first_valid_id;

                for i in first_valid_id..=target_end_id {
                    let flow_in_current = self.global_path.borrow()[i].flow_in_current;

                    let dist2 = vector_2d_norm2!(flow_in_current);

                    let mut flow_in_current_direction =
                        self.global_path.borrow()[i].flow_in_current_direction;
                    let flow_in_current_direction_min =
                        self.global_path.borrow()[i].flow_in_current_direction_min;
                    let flow_in_current_direction_max =
                        self.global_path.borrow()[i].flow_in_current_direction_max;

                    let mut diff = (angle_norm!(flow_in_current_direction - last_direction));
                    let diff_min = (angle_norm!(flow_in_current_direction_min - last_direction));
                    let diff_max = (angle_norm!(flow_in_current_direction_max - last_direction));

                    let flow_into_direction = self.global_path.borrow()[i].flow_into_direction;
                    if ((diff_min < 0.0 && diff_max > 0.0) || (diff_min > 0.0 && diff_max < 0.0)) {
                        flow_in_current_direction = last_direction;
                        diff = 0.0;
                    } else if diff_min.abs() < diff_max.abs() {
                        flow_in_current_direction = flow_in_current_direction_min;
                        diff = diff_min.abs();
                    } else if diff_min.abs() > diff_max.abs() {
                        flow_in_current_direction = flow_in_current_direction_max;
                        diff = diff_max.abs();
                    }
                    diff += flow_into_direction.abs();
                    // info!("choose target point id: {}, diff_min: {},diff_max: {},last_direction: {}, flow_in_current_direction: {}, flow_in_current_direction_min: {},flow_in_current_direction_max: {}",i, diff_min,diff_max,last_direction, flow_in_current_direction,flow_in_current_direction_min,flow_in_current_direction_max);
                    // info!(track_goal_num, ?flow_in_current);
                    if (track_goal_num == 0) {
                        if flow_in_current[1].abs() < opt.track_curve_to_line_search_y {
                            best_direction = self.global_path.borrow()[i].flow_in_current_direction;
                            best_direction_diff_abs = diff;
                            best_direction_id = i;
                        } else {
                            if (dist2 < min_predict_control_dist2)
                                || (diff < best_direction_diff_abs)
                                || (diff < 1e-4)
                            {
                                best_direction = flow_in_current_direction;
                                best_direction_diff_abs = diff;
                                best_direction_id = i;
                            }
                        }
                    } else {
                        if (dist2 < min_predict_control_dist2)
                            || (diff < best_direction_diff_abs)
                            || (diff < 1e-4)
                        {
                            best_direction = flow_in_current_direction;
                            best_direction_diff_abs = diff;
                            best_direction_id = i;
                        }
                    }

                    if (track_goal_num > 0) && (dist2 > max_predict_control_dist2) {
                        break;
                    }

                    // if dist > max_predict_control_dist2{
                    //     break;
                    // }
                }

                select_target_id = best_direction_id;

                self.planner_data.borrow_mut().target_start_id = select_target_id;

                current_target_in_control_pose = aux_control_point_transform_inv
                    .multiply_pose(&self.global_path.borrow()[select_target_id].pose);
                let first_valid_node = self.global_path.borrow()[first_valid_id];

                let closest_control_node_flow_in_control_pose =
                    aux_control_point_transform_inv.multiply_pose(&closest_control_node.flow);

                // todo: how to smooth direction
                let mut smooth_direction_ratio = opt.track_curve_smooth_direction_ratio;
                let mut smooth_direction_offset = opt.track_curve_smooth_direction_offset;

                if (first_valid_node.flow_type < 10) {
                    if (closest_control_node_flow_in_control_pose[1].abs()
                        > smooth_direction_offset)
                    {
                        // best_direction = closest_control_node_flow_in_control_pose[2];
                    } else {
                        best_direction = best_direction
                            + smooth_direction_ratio
                                * angle_norm!(
                                    closest_control_node_flow_in_control_pose[2] - best_direction
                                );
                    }
                } else {
                }

                let dist = (vector_2d_norm2!(current_target_in_control_pose)).sqrt();

                current_target_in_control_pose[0] = dist * best_direction.cos();
                current_target_in_control_pose[1] = dist * best_direction.sin();
                info!(
                    "update: best_direction_id: {},best_direction: {}, best_direction_diff_abs: {}",
                    best_direction_id, best_direction, best_direction_diff_abs
                );
            }

            let mut allow_forward_vel = opt.track_curve_max_vel;

            *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
            *self.current_target_pose.borrow_mut() =
                aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

            self.controller.borrow().follow_with_control(
                &current_target_in_control_pose,
                allow_forward_vel,
                &aux_control_point,
            );

            let switch_dist = opt.track_curve_max_vel * opt.track_curve_to_line_switch_dist_ratio;
            let next_segment_start_node =
                self.global_path.borrow()[closest_control_node.segment_end_id + 1];
            let next_segment_end_node =
                self.global_path.borrow()[next_segment_start_node.segment_end_id];
            info!(
                "current_control_pose_relative_smooth_y: {}, switch_dist: {}",
                current_control_pose_relative_smooth_y.1, switch_dist
            );
            let is_nex_segment_on_goal_segment =
                next_segment_start_node.segment_end_id == path_len - 1;
            let is_nex_segment_line = next_segment_start_node.flow_type >= 10;

            let read_target_end = (target_end_id < target_start_id + 3);

            if current_control_pose_relative_smooth_y.0 {
                if read_target_end
                    || (record_control_pose_in_target_end[1] > 0.0
                        && current_control_pose_relative_smooth_y.1 < switch_dist)
                    || (record_control_pose_in_target_end[1] < 0.0
                        && current_control_pose_relative_smooth_y.1 > -switch_dist)
                {
                    if !is_nex_segment_on_goal_segment {
                        let next_next_segment_start_node =
                            self.global_path.borrow()[next_segment_end_node.segment_end_id + 1];
                        let next_next_segment_end_node =
                            self.global_path.borrow()[next_next_segment_start_node.segment_end_id];

                        //todo:test Line
                        if next_next_segment_start_node.flow_type < 10 {
                            task = RobotMotionPlannerTask::LineToCurve;
                        } else {
                            task = RobotMotionPlannerTask::LineToLine;
                        }
                        *self.is_curve_converge.borrow_mut() = false;
                    } else {
                        task = RobotMotionPlannerTask::Line;
                    }
                    self.planner_data.borrow_mut().target_start_id =
                        closest_control_node.segment_end_id + 1;
                    self.planner_data.borrow_mut().target_end_id =
                        next_segment_start_node.segment_end_id;
                    *self.track_goal_direction.borrow_mut() = actual_command_forward_angle;
                    self.record_base_pose();

                    // self.current_control_pose_relative_smooth_y.borrow_mut().reset();
                    *self.track_goal_num.borrow_mut() = 0;

                    info!("Task switch to {:?}", task);
                }
            }
        }
        // Switch condition
        //Switch: LineToCurve -> CurveToLine
        //Switch: LineToCurve -> Curve(Rotate, NoRotate)
        //todo: task == RobotMotionPlannerTask::LineToCurve
        // if last task is CurveToLine,
        else if task == RobotMotionPlannerTask::LineToCurve {
            let mut target_start_id = self.planner_data.borrow().target_start_id;
            let mut target_end_id = self.planner_data.borrow().target_end_id;
            info!(target_start_id, target_end_id);
            let target_start_node = self.global_path.borrow()[target_start_id];

            let target_end_node = self.global_path.borrow()[target_end_id];
            let control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);
            let current_pose_in_target_end =
                target_end_node.flow_pose_inv.multiply_pose(&current_pose);
            let record_control_pose_in_target_end = target_end_node
                .flow_pose_inv
                .multiply_pose(&record_aux_control_point_abs);

            self.current_control_pose_relative_smooth_y
                .borrow_mut()
                .update(control_pose_in_target_end[1]);
            let current_control_pose_relative_smooth_y =
                self.current_control_pose_relative_smooth_y.borrow().mean();

            let mut driver_direction_in_target_end =
                angle_norm!(control_pose_in_target_end[2] + actual_command_forward_angle);

            let current_driver_cmd_vx =
                actual_command_forward_vel * driver_direction_in_target_end.cos();
            let current_driver_cmd_vy =
                actual_command_forward_vel * driver_direction_in_target_end.sin();

            //
            let mut allow_forward_vel: f64 = opt.track_curve_max_vel;

            let mut a: f64 = 0.0;
            let mut b: f64 = 0.0;
            let mut c: f64 = 0.0;
            let acc: f64 = 0.3;

            let v0: f64 = opt.track_curve_max_vel;
            let ds = (vector_2d_norm2!(control_pose_in_target_end)).sqrt();

            a = 0.5 * acc;
            b = v0;
            c = -ds;
            let result = find_roots_of_quadratic_equation(a, b, c);
            if result.0 > 0 {
                allow_forward_vel = allow_forward_vel.min(v0 + acc * result.1);
            }
            info!(allow_forward_vel);

            let mut converge_vel: f64 = 0.0;

            {
                // converge vel limit
                // let mut allow_forward_vel: f64 = 0.5;

                let mut a: f64 = 0.0;
                let mut b: f64 = 0.0;
                let mut c: f64 = 0.0;
                let acc: f64 = opt.track_final_line_forward_vel_y_acc;

                let v0: f64 = opt.constrain_stop_forward_vel_y_min;
                let ds = control_pose_in_target_end[1].abs();

                a = 0.5 * acc;
                b = v0;
                c = -ds;
                let result = find_roots_of_quadratic_equation(a, b, c);
                if result.0 > 0 {
                    // converge_vel = v0 + acc * result.1;
                    // todo: test linear function
                    converge_vel = v0 + acc * ds;
                }
                info!(
                    "converge_vel find_roots_of_quadratic_equation: ds: {},acc: {}, v0:{}, vel: {}",
                    ds, acc, v0, converge_vel
                );
            }
            {
                let mut vel_ratio = 1.0;
                vel_ratio = converge_vel / current_driver_cmd_vy.abs();

                vel_ratio = vel_ratio.min(1.0).max(0.0);
                let converge_vel = allow_forward_vel * vel_ratio;

                allow_forward_vel = allow_forward_vel.min(converge_vel);
                info!(
                    current_driver_cmd_vx,
                    current_driver_cmd_vy, vel_ratio, allow_forward_vel
                );
            }
            // self.controller.borrow().follow_with_control(
            //     &aux_control_point,
            //     allow_forward_vel,
            //     &aux_control_point,
            // );

            let switch_dist = -opt.track_line_to_curve_switch_dist_ratio * opt.track_curve_max_vel;

            let next_segment_start_node =
                self.global_path.borrow()[target_end_node.segment_end_id + 1];
            let next_segment_end_node =
                self.global_path.borrow()[next_segment_start_node.segment_end_id];

            let next_segment_end_node_in_target_end_node = target_end_node
                .flow_pose_inv
                .multiply_pose(&next_segment_end_node.flow);

            info!("switch_dist: {}, next_segment_end_node_in_target_end_node: {:?}, control_pose_in_target_end: {:?}",switch_dist, next_segment_end_node_in_target_end_node , control_pose_in_target_end);

            self.current_control_pose_relative_smooth_y
                .borrow_mut()
                .update(control_pose_in_target_end[1]);
            let current_control_pose_relative_smooth_y =
                self.current_control_pose_relative_smooth_y.borrow().mean();

            if control_pose_in_target_end[0] < switch_dist {
                let mut is_curve_converge = *self.is_curve_converge.borrow();

                if is_curve_converge {
                    let mut target_pose_in_target_end_node = control_pose_in_target_end;
                    target_pose_in_target_end_node[0] += opt.track_line_to_curve_prediction_normal;
                    target_pose_in_target_end_node[2] = 0.0;

                    if (next_segment_end_node_in_target_end_node[1] < 0.0
                        && current_control_pose_relative_smooth_y.1 > 0.0)
                        || (next_segment_end_node_in_target_end_node[1] > 0.0
                            && current_control_pose_relative_smooth_y.1 < 0.0)
                    {
                        target_pose_in_target_end_node[1] = 0.0;
                        // target_pose_in_target_end_node[1] = current_control_pose_relative_smooth_y.1;
                    } else {
                        // target_pose_in_target_end_node[1] = current_control_pose_relative_smooth_y.1;
                    }

                    let mut target_pose = target_end_node
                        .flow_pose
                        .multiply_pose(&target_pose_in_target_end_node);
                    target_pose[2] = target_end_node.pose[2];

                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&target_pose);
                } else {
                    let mut track_goal_num = *self.track_goal_num.borrow();
                    *self.track_goal_num.borrow_mut() += 1;

                    let target_end_flow_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&target_end_node.flow);

                    let mut converge_direction_x = target_end_flow_in_control_pose[2];

                    let mut converge_direction_y =
                        converge_direction_x - FRAC_PI_2.copysign(control_pose_in_target_end[1]);

                    let mut target_direction = converge_direction_x; // 0.5*(actual_command_forward_angle + converge_direction_x);

                    let mut last_direction =
                        current_target_in_control_pose[1].atan2(current_target_in_control_pose[0]);

                    if (track_goal_num == 0) {
                        *self.track_goal_direction.borrow_mut() = last_direction;
                    }
                    let mut track_goal_direction = *self.track_goal_direction.borrow();

                    let mut track_final_line_converge_direction_fix =
                        opt.track_final_line_converge_direction_fix;

                    let mut track_final_line_converge_direction_fix_min =
                        opt.track_final_line_converge_direction_fix_min;

                    track_final_line_converge_direction_fix =
                        (angle_norm!(record_control_pose_in_target_end[2] + track_goal_direction))
                            .abs();

                    // info!(track_goal_num, ?record_base_pose_transform, ?record_control_pose_in_target_end,track_goal_direction  );

                    // allow_forward_vel = opt.track_final_line_forward_vel_x_max;

                    let mut ratio = -current_control_pose_relative_smooth_y.1
                        / opt.track_final_line_converge_offset_base_y;

                    ratio = ratio.min(1.0).max(-1.0);

                    if current_control_pose_relative_smooth_y.0 && !is_curve_converge {
                        if ((record_control_pose_in_target_end[1] > 0.0
                            && current_control_pose_relative_smooth_y.1
                                > opt.track_final_line_converge_y)
                            || (record_control_pose_in_target_end[1] < 0.0
                                && current_control_pose_relative_smooth_y.1
                                    < -opt.track_final_line_converge_y))
                        {
                            // target_direction = ratio*(converge_direction_y ) + (1.0 - ratio)*converge_direction_x ;
                            let actual_direction_offset =
                                angle_norm!(actual_command_forward_angle - converge_direction_x);

                            // direction_fix = direction_fix.min(actual_direction_offset.abs());
                            let mut fix = ratio * track_final_line_converge_direction_fix;
                            fix += track_final_line_converge_direction_fix_min.copysign(fix);

                            target_direction = angle_norm!(converge_direction_x + fix);
                            info!(
                                track_goal_num,
                                ?control_pose_in_target_end,
                                target_direction,
                                track_final_line_converge_direction_fix,
                                converge_direction_x,
                                ratio,
                                fix
                            );

                            #[cfg(aaa)]
                            {
                                if actual_command_forward_angle < 0.0 {
                                    target_direction =
                                        target_direction.max(-PI).min(actual_command_forward_angle);
                                } else {
                                    target_direction =
                                        target_direction.min(PI).max(actual_command_forward_angle);
                                }
                            }

                            // target_direction = track_goal_direction;
                        } else {
                            is_curve_converge = true;
                        }
                    }

                    current_target_in_control_pose[0] = 2.0 * target_direction.cos();
                    current_target_in_control_pose[1] = 2.0 * target_direction.sin();
                }

                *self.is_curve_converge.borrow_mut() = is_curve_converge;

                let mut control_error = false;
                control_error = (is_curve_converge
                    && (control_pose_in_target_end[1].abs()
                        > opt.track_line_to_curve_allow_offset_y));

                *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
                *self.current_target_pose.borrow_mut() =
                    aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

                info!(allow_forward_vel, ?current_target_in_control_pose);

                if control_error {
                    self.controller.borrow().stop();
                } else {
                    self.controller.borrow().follow_with_control(
                        &current_target_in_control_pose,
                        allow_forward_vel,
                        &aux_control_point,
                    );
                }
            } else {
                let is_nex_segment_on_goal_segment =
                    next_segment_start_node.segment_end_id == path_len - 1;

                if !is_nex_segment_on_goal_segment {
                    let next_next_segment_start_node =
                        self.global_path.borrow()[next_segment_end_node.segment_end_id + 1];
                    let next_next_segment_end_node =
                        self.global_path.borrow()[next_next_segment_start_node.segment_end_id];

                    if next_next_segment_start_node.flow_type < 10 {
                        task = RobotMotionPlannerTask::CurveToCurve;
                        *self.is_curve_converge.borrow_mut() = false;
                    } else {
                        task = RobotMotionPlannerTask::CurveToLine;
                        *self.is_curve_converge.borrow_mut() = false;
                    }
                } else {
                    task = RobotMotionPlannerTask::Curve;
                }
                self.planner_data.borrow_mut().target_start_id =
                    target_start_node.segment_end_id + 1;
                self.planner_data.borrow_mut().target_end_id =
                    next_segment_start_node.segment_end_id;

                info!("Task switch to {:?}", task);
                self.record_base_pose();
                self.current_control_pose_relative_smooth_y
                    .borrow_mut()
                    .reset();
                *self.track_goal_num.borrow_mut() = 0;
            }
        }

        self.planner_data.borrow_mut().task = task;

        return task == RobotMotionPlannerTask::Finished;

        let path_end_in_control_pose =
            aux_control_point_transform_inv.multiply_pose(&goal_node.pose);

        *self.path_end_in_control_pose.borrow_mut() = path_end_in_control_pose;

        let base_pose_in_goal = goal_node.flow_pose_inv.multiply_pose(&current_pose);
        let control_pose_in_goal = goal_node
            .flow_pose_inv
            .multiply_pose(&aux_control_point_abs);

        self.current_control_pose_relative_smooth_y
            .borrow_mut()
            .update(control_pose_in_segment_end[1]);
        let current_control_pose_in_segment_end_smooth_y =
            self.current_control_pose_relative_smooth_y.borrow().mean();

        self.current_base_pose_in_goal_smooth_x
            .borrow_mut()
            .update(base_pose_in_goal[0]);
        let current_base_pose_in_goal_smooth_x =
            self.current_base_pose_in_goal_smooth_x.borrow().mean();

        *self.segment_end_in_control_pose.borrow_mut() = segment_end_in_control_pose;

        let mut run_on_line_path =
            closest_base_node.flow_type == 0 && closest_control_node.flow_type == 0;
        let base_point_in_closest_flow =
            closest_base_node.flow_pose_inv.multiply_pose(&current_pose);
        let base_point_in_goal_flow = goal_node.flow_pose_inv.multiply_pose(&current_pose);

        let control_point_in_closest_flow = closest_control_node
            .flow_pose_inv
            .multiply_pose(&aux_control_point_abs);

        self.current_control_pose_in_path_smooth_y
            .borrow_mut()
            .update(control_point_in_closest_flow[1]);

        let current_control_pose_in_path_smooth_y =
            self.current_control_pose_in_path_smooth_y.borrow().mean();

        let mut allow_forward_vel = closest_control_node.allow_forward_vel;

        log!(
            "run_on_line_path: {}, control_point_in_closest_flow:{:?}",
            run_on_line_path,
            control_point_in_closest_flow
        );

        if control_point_in_closest_flow[1].abs() > 0.05 {
            log!("off path");
            // self.controller.borrow().stop();

            // return false;
        }

        // check off path

        let mut is_on_line_or_curve = false;

        let mut current_target_in_control_pose = *self.current_target_in_control_pose.borrow();

        info!(
            "current_target_in_control_pose: {:?}",
            current_target_in_control_pose
        );
        let mut use_search_target = true;

        // robot is reaching goal, stop

        // check is in tolerance
        if (closest_control_node.segment_end_id == path_len - 1)
            && (closest_control_node.dist_to_end < opt.start_follow_goal_dist)
        {
            if (closest_control_node.flow_type == 0) {
                if control_point_in_closest_flow[1].abs() > opt.final_offset_tolerance {
                    self.planner_data.borrow_mut().error =
                        RobotMotionPlannerError::UnexpectedFinalToleranceError;

                    self.controller.borrow().stop();
                    return false;
                }
            } else {
            }

            info!(
                "control_point_in_closest_flow: {:?}",
                control_point_in_closest_flow
            );
            info!("base_point_in_goal_flow: {:?}", base_point_in_goal_flow);
            info!(
                "base_point_in_closest_flow: {:?}",
                base_point_in_closest_flow
            );

            self.planner_data.borrow_mut().state = RobotMotionPlannerState::DebugStop;

            self.controller.borrow().stop();
            return false;

            // if(closest_control_node.dist_to_segment_end < 0.2){
            //
            //     info!("closest_control_node: {:?}",closest_control_node);
            //     info!("segment_end_node: {:?}",segment_end_node);
            //     info!("path_end_node: {:?}",path_end_node);
            //     *self.state.borrow_mut()  = RobotMotionPlannerState::DebugStop;
            //
            //     self.controller.borrow().stop();
            //
            //     return false;
            // }
        } else if (closest_control_node.segment_end_id == path_len - 1)
            && (closest_control_node.flow_type == 0)
        {
            use_search_target = false;

            let mut control_pose_in_goal = goal_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);

            control_pose_in_goal[0] = 1000.0;
            control_pose_in_goal[2] = 0.0;
            let mut target_pose = goal_node.flow_pose.multiply_pose(&control_pose_in_goal);
            target_pose[2] = goal_node.pose[2];
            current_target_in_control_pose =
                aux_control_point_transform_inv.multiply_pose(&target_pose);
            info!(
                "current_target_in_control_pose: {:?}",
                current_target_in_control_pose
            );
        } else if (closest_control_node.segment_end_id == path_len - 1)
            && (closest_control_node.flow_type != 0)
        {
        } else if (closest_control_node.segment_end_id < path_len - 1)
            && (closest_control_node.flow_type == 0)
        {
            let next_segment_start_node =
                self.global_path.borrow()[closest_control_node.segment_end_id + 1];

            if (next_segment_start_node.segment_end_id == path_len - 1)
                && (closest_control_node.flow_type != 0)
            {}
        } else if (closest_control_node.segment_end_id < path_len - 1)
            && (closest_control_node.flow_type != 0)
        {
            let next_segment_start_node =
                self.global_path.borrow()[closest_control_node.segment_end_id + 1];

            let mut curve_converge = *self.is_curve_converge.borrow();

            if current_control_pose_in_segment_end_smooth_y.0 {
                #[cfg(aaa)]
                {
                    if closest_control_node.flow_type < 0 {
                        if current_control_pose_in_segment_end_smooth_y.1
                            > -opt.curve_offset_converge
                        {
                            curve_converge = true;
                        }
                    } else {
                        if current_control_pose_in_segment_end_smooth_y.1
                            < opt.curve_offset_converge
                        {
                            curve_converge = true;
                        }
                    }
                }
                if current_control_pose_in_segment_end_smooth_y.1.abs() < opt.curve_offset_converge
                {
                    curve_converge = true;
                }
            }
            *self.is_curve_converge.borrow_mut() = curve_converge;

            if (curve_converge) {
                use_search_target = false;

                if current_control_pose_in_segment_end_smooth_y.1.abs()
                    < 0.5 * opt.curve_offset_converge
                {
                    let mut control_pose_in_goal = goal_node
                        .flow_pose_inv
                        .multiply_pose(&aux_control_point_abs);

                    control_pose_in_goal[0] = 1000.0;
                    control_pose_in_goal[2] = 0.0;
                    let mut target_pose = goal_node.flow_pose.multiply_pose(&control_pose_in_goal);
                    target_pose[2] = goal_node.pose[2];
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&target_pose);
                } else {
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&goal_node.flow);
                }

                info!(
                    "current_target_in_control_pose: {:?}",
                    current_target_in_control_pose
                );

                #[cfg(aaa)]
                {
                    info!(
                        "current_target_in_control_pose: {:?}",
                        current_target_in_control_pose
                    );

                    info!(
                        "current_control_pose_in_segment_end_smooth_y: {:?}",
                        current_control_pose_in_segment_end_smooth_y
                    );

                    info!("next_segment_start_node: {:?}", next_segment_start_node);
                    info!(
                        "control_point_in_closest_flow: {:?}",
                        control_point_in_closest_flow
                    );
                    info!("base_point_in_goal_flow: {:?}", base_point_in_goal_flow);
                    info!(
                        "base_point_in_closest_flow: {:?}",
                        base_point_in_closest_flow
                    );

                    if (next_segment_start_node.segment_end_id == path_len - 1)
                        && (next_segment_start_node.flow_type == 0)
                    {
                        *self.state.borrow_mut() = RobotMotionPlannerState::DebugStop;

                        self.controller.borrow().stop();
                        return false;
                    }
                }
            }

            if (next_segment_start_node.segment_end_id == path_len - 1)
                && (closest_control_node.flow_type == 0)
            {
                if closest_control_node.flow_type < 0 {
                } else {
                }
            }
        }

        log!(
            "closest_base_dist: {}, closest_base_id_search_dist: {},  aux_control_point: {:?}, closest_control_id: {},closest_control_node: {:?}",
            closest_base_dist,
            closest_base_id_search_dist,
            aux_control_point_abs,
            closest_control_id,
            closest_control_node
        );
        log!(
            "closest_control_dist: {}, closest_control_id_search_dist: {}, current_pose: {:?}, closest_base_id: {},closest_base_node : {:?}",
            closest_control_dist,
            closest_control_id_search_dist,
            current_pose,
            closest_base_id,
            closest_base_node
        );

        // change to follow_goal mode

        // if run_on_line_path {
        //     self.controller.borrow().stop();
        // } else
    }

    pub fn follow_goal(&self) -> bool
    {
        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                log!("select_option doesn't exist");
                return false;
            }
        };

        let path_len = self.global_path.borrow().len();
        let goal_node_id = path_len - 1;

        let control_interval_s = self.controller.borrow().config().control_interval_s;

        let current_pose = *self.current_pose.borrow();
        let record_base_pose = *self.record_base_pose.borrow();

        let current_pose_transform = Transform2d::new(&current_pose);
        let record_base_pose_transform = Transform2d::new(&record_base_pose);

        let current_pose_transform_inv = current_pose_transform.inverse();
        let record_base_pose_transform_inv = record_base_pose_transform.inverse();

        let current_pose_in_record = record_base_pose_transform_inv.multiply_pose(&current_pose);

        let aux_control_point = *self.aux_control_point.borrow();
        warn!(?aux_control_point);
        let aux_control_point_abs = current_pose_transform.multiply_pose(&aux_control_point);
        let aux_control_point_transform = Transform2d::new(&aux_control_point_abs);
        let aux_control_point_transform_inv = aux_control_point_transform.inverse();

        *self.current_control_pose.borrow_mut() = aux_control_point_abs;

        let mut current_target_in_control_pose = *self.current_target_in_control_pose.borrow();

        let goal_node = self.global_path.borrow()[goal_node_id];

        let mut task = self.planner_data.borrow_mut().task;
        info!("Task running on {:?}", task);

        let current_driver_state = self
            .controller
            .borrow()
            .get_driver_command(&aux_control_point);

        let [mut current_driver_state_forward_vel, mut current_driver_state_forward_angle] =
            current_driver_state;

        if aux_control_point[0] > 0.0 {
        } else {
            current_driver_state_forward_angle =
                angle_norm!(current_driver_state_forward_angle + PI);
            current_driver_state_forward_vel = -current_driver_state_forward_vel;
        }

        // search closest node
        let mut closest_base_id = self.planner_data.borrow().closest_base_id;
        let closest_base_node = self.global_path.borrow()[closest_base_id];
        let mut closest_base_id_search_dist: f64 = opt.closest_node_search_dist;

        let (closest_base_id, closest_base_end_id, closest_base_dist) =
            self.find_closest_node(closest_base_id, &current_pose, closest_base_id_search_dist);
        self.planner_data.borrow_mut().closest_base_id = closest_base_id;
        let closest_base_node = self.global_path.borrow()[closest_base_id];

        // search closest_control_node_id
        let mut closest_control_id = self.planner_data.borrow().closest_control_id;
        let closest_control_node = self.global_path.borrow()[closest_control_id];
        let mut closest_control_id_search_dist: f64 = if task == RobotMotionPlannerTask::Init {
            (0.2 + (vector_2d_norm2!(aux_control_point)).sqrt()) + opt.closest_node_search_dist
        } else {
            (vector_2d_dist2!(closest_control_node.pose, current_pose)).sqrt()
                + opt.closest_node_search_dist
        };
        let (closest_control_id, closest_control_end_id, closest_control_dist) = self
            .find_closest_node(
                closest_control_id,
                &aux_control_point_abs,
                closest_control_id_search_dist,
            );

        self.planner_data.borrow_mut().closest_control_id = closest_control_id;
        let closest_control_node = self.global_path.borrow()[closest_control_id];

        let current_control_pose_in_closest_node = closest_control_node
            .flow_pose_inv
            .multiply_pose(&aux_control_point_abs);
        *self.current_control_pose_in_closest.borrow_mut() = current_control_pose_in_closest_node;

        //
        {
            *self.current_control_pose_in_goal_flow.borrow_mut() = goal_node
                .flow_pose_inv
                .multiply_pose(&aux_control_point_abs);
            *self.current_base_pose_in_goal_flow.borrow_mut() =
                goal_node.flow_pose_inv.multiply_pose(&current_pose);
            *self.current_driver_state.borrow_mut() = if aux_control_point[0] > 0.0 {
                [
                    current_driver_state_forward_vel,
                    current_driver_state_forward_angle,
                    angle_norm!(current_driver_state_forward_angle - 0.0),
                ]
            } else {
                [
                    current_driver_state_forward_vel,
                    current_driver_state_forward_angle,
                    angle_norm!(current_driver_state_forward_angle - PI),
                ]
            }
        }

        #[cfg(aaa)]
        {
            // test
            let mut current_target_in_control_pose = [-aux_control_point[0], 0.001, 0.0];

            *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
            *self.current_target_pose.borrow_mut() =
                aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

            self.controller.borrow().follow_with_control(
                &current_target_in_control_pose,
                0.1,
                &aux_control_point,
            );

            return false;
        }

        match task {
            RobotMotionPlannerTask::Idle => {
                self.stop();

                task = RobotMotionPlannerTask::Finished;
            }
            RobotMotionPlannerTask::Init => {
                // success => Curve
                //         => Line

                // failed => None

                if closest_base_node.flow_type >= 10  && (self.global_path.borrow()[closest_base_id].dist_to_segment_end
                    > opt.valid_line_length){
                    task = RobotMotionPlannerTask::Line;

                    self.planner_data.borrow_mut().target_start_id = closest_base_id;
                    self.planner_data.borrow_mut().target_end_id = closest_base_node.segment_end_id;
                } else {
                    task = RobotMotionPlannerTask::Curve;

                    // find next line segment
                    let mut target_end_id = closest_base_id;
                    for i in closest_base_id..path_len {
                        target_end_id = i;
                        if (self.global_path.borrow()[i].flow_type >= 10)
                            && (self.global_path.borrow()[i].dist_to_segment_end
                                > opt.valid_line_length)
                        {
                            break;
                        }
                    }
                    self.planner_data.borrow_mut().target_start_id = closest_base_id;
                    self.planner_data.borrow_mut().target_end_id = target_end_id;
                }

                let target_end_id = self.planner_data.borrow().target_end_id;

                let target_end_node = self.global_path.borrow()[target_end_id];

                let control_pose_in_target_end = target_end_node
                    .flow_pose_inv
                    .multiply_pose(&aux_control_point_abs);

                self.current_control_pose_in_target_end_y
                    .borrow_mut()
                    .fill(control_pose_in_target_end[1]);
                self.current_control_pose_in_target_end_x
                    .borrow_mut()
                    .fill(control_pose_in_target_end[0]);
                self.record_base_pose();

                self.planner_data.borrow_mut().task_counter = 0;
                warn!("Task switch to {:?}", task);
            }
            RobotMotionPlannerTask::Line => {
                // exit condition
                // 1. on goal segment, reach goal
                // 2. reach segment end

                // success => Finished
                //         => Line
                //         => Curve

                //todo: how to handler opposite direction target
                //

                let mut target_start_id = self.planner_data.borrow().target_start_id;
                let mut target_end_id = self.planner_data.borrow().target_end_id;

                let is_goal_line = target_end_id == goal_node_id;

                let target_start_node = self.global_path.borrow()[target_start_id];

                let target_end_node = self.global_path.borrow()[target_end_id];

                let is_nex_line =
                    !is_goal_line && (self.global_path.borrow()[target_end_id + 1].flow_type > 10);

                let control_pose_in_target_end = target_end_node
                    .flow_pose_inv
                    .multiply_pose(&aux_control_point_abs);
                let control_pose_in_target_start = target_start_node
                    .flow_pose_inv
                    .multiply_pose(&aux_control_point_abs);
                let current_pose_in_target_end =
                    target_end_node.flow_pose_inv.multiply_pose(&current_pose);

                let (_, current_control_pose_in_target_end_y) =
                    self.current_control_pose_in_target_end_y.borrow().mean();
                let (_, current_control_pose_in_target_end_x) =
                    self.current_control_pose_in_target_end_x.borrow().mean();

                // warn!(target_start_id,target_end_id, current_control_pose_in_target_end_y);

                let mut driver_direction_in_target_end =
                    angle_norm!(control_pose_in_target_end[2] + current_driver_state_forward_angle);

                let current_driver_cmd_vx =
                    current_driver_state_forward_vel * driver_direction_in_target_end.cos();
                let current_driver_cmd_vy =
                    current_driver_state_forward_vel * driver_direction_in_target_end.sin();

                {
                    let predict_move_y = control_interval_s * current_driver_cmd_vy;
                    let predict_move_x = control_interval_s * current_driver_cmd_vx;
                    let real_move_y =
                        control_pose_in_target_end[1] - current_control_pose_in_target_end_y;
                    let real_move_x =
                        control_pose_in_target_end[0] - current_control_pose_in_target_end_x;
                    let move_y_diff = predict_move_y - real_move_y;
                    let move_x_diff = predict_move_x - real_move_x;

                    warn!(
                        ?control_pose_in_target_end,
                        predict_move_y,
                        predict_move_x,
                        real_move_y,
                        real_move_x,
                        move_y_diff,
                        move_x_diff,
                        current_control_pose_in_target_end_y,
                        current_control_pose_in_target_end_x
                    );
                    if (move_y_diff.abs() > opt.pose_jump_alert_y)
                        || (move_x_diff.abs() > opt.pose_jump_alert_x)
                    {
                        self.planner_data.borrow_mut().error = RobotMotionPlannerError::PoseJump;
                        self.stop();
                        return false;
                    }

                    if current_control_pose_in_target_end_y > opt.control_closest_alert {
                        self.planner_data.borrow_mut().error =
                            RobotMotionPlannerError::ClosestDistError;
                        self.stop();
                        return false;
                    }
                }

                self.current_control_pose_in_target_end_y
                    .borrow_mut()
                    .update(control_pose_in_target_end[1]);
                self.current_control_pose_in_target_end_x
                    .borrow_mut()
                    .update(control_pose_in_target_end[0]);
                let (_, current_control_pose_in_target_end_y) =
                    self.current_control_pose_in_target_end_y.borrow().mean();
                let (_, current_control_pose_in_target_end_x) =
                    self.current_control_pose_in_target_end_x.borrow().mean();

                let is_goal_line_final =
                    is_goal_line && (current_control_pose_in_target_end_x > -opt.goal_line_final);

                if is_goal_line && opt.constrain_stop_no_rotate {
                    let check_dist = -opt.constrain_stop_no_rotate_run_dist
                        - opt.control_not_converge_check_dist;
                    if current_control_pose_in_target_end_x > check_dist {
                        if current_control_pose_in_target_end_y.abs()
                            > opt.control_not_converge_alert
                        {
                            self.planner_data.borrow_mut().error =
                                RobotMotionPlannerError::GoalConvergeDistError;
                            self.stop();
                            return false;
                        }
                    }
                }

                *self.current_vel_in_target_end.borrow_mut() =
                    [current_driver_cmd_vx, current_driver_cmd_vy];

                let task_counter = self.planner_data.borrow_mut().task_counter;

                self.planner_data.borrow_mut().task_counter += 1;

                // choose direction use the closest_control_node and prediction_dist
                // set forward_vel use control_pose_in_target_end offset y

                let converge_diff_yaw = angle_norm!(current_pose[2] - target_end_node.pose[2]);
                let line_converge = (converge_diff_yaw.abs() < opt.line_converge_yaw)
                    && (current_control_pose_in_target_end_y.abs() < opt.line_converge_y);

                let mut allow_forward_vel: f64 = if is_goal_line {
                    if is_goal_line_final {
                        opt.goal_line_vel
                    } else {
                        if line_converge {
                            opt.line_vel
                        } else {
                            opt.line_converge_vel
                        }
                    }
                } else {
                    if line_converge {
                        opt.line_vel
                    } else {
                        opt.line_converge_vel
                    }
                };

                {
                    // not goal_line, => curve vel
                    //                => line vel
                    // goal_line , => track vel
                    //             => stop vel

                    let mut stop_vel: f64 = 0.0;

                    // stop vel limit
                    // let mut allow_forward_vel: f64 = 0.5;

                    let mut a: f64 = 0.0;
                    let mut b: f64 = 0.0;
                    let mut c: f64 = 0.0;
                    let acc: f64 = opt.speed_down_acc;

                    let mut v0: f64 = if is_goal_line {
                        opt.constrain_stop_forward_vel_x_min
                    } else {
                        opt.track_curve_start_vel
                    };

                    v0 = if is_goal_line {
                        if is_goal_line_final {
                            opt.constrain_stop_forward_vel_x_min
                        } else {
                            opt.goal_line_vel
                        }
                    } else {
                        if is_nex_line {
                            0.0
                        } else {
                            opt.track_curve_start_vel
                        }
                    };

                    let mut ds = if is_goal_line {
                        current_pose_in_target_end[0].abs() - opt.constrain_stop_allow_offset_x
                    } else {
                        current_pose_in_target_end[0].abs() - 0.1
                    };

                    ds = if is_goal_line {
                        if is_goal_line_final {
                            current_pose_in_target_end[0].abs() - opt.constrain_stop_allow_offset_x
                        } else {
                            current_pose_in_target_end[0].abs() - opt.goal_line_final
                        }
                    } else {
                        current_control_pose_in_target_end_x.abs()
                    };

                    if ds > 0.0 {
                        a = 0.5 * acc;
                        b = v0;
                        c = -ds;
                        let result = find_roots_of_quadratic_equation(a, b, c);
                        if result.0 > 0 {
                            stop_vel = v0 + acc * result.1;
                        }
                        warn!(
                        "stop_vel find_roots_of_quadratic_equation: ds: {},acc: {}, v0:{}, vel: {}",
                        ds, acc, v0, stop_vel
                    );
                    } else {
                        stop_vel = v0;
                    }

                    allow_forward_vel = allow_forward_vel.min(stop_vel);
                }

                let mut converge_vel: f64 = 0.0;

                {
                    // converge vel

                    // converge vel limit
                    // let mut allow_forward_vel: f64 = 0.5;

                    let mut a: f64 = 0.0;
                    let mut b: f64 = 0.0;
                    let mut c: f64 = 0.0;
                    let acc: f64 = opt.line_converge_y_acc;

                    let v0: f64 = opt.line_converge_y_min;

                    let ds = current_control_pose_in_target_end_y.abs();

                    a = 0.5 * acc;
                    b = v0;
                    c = -ds;
                    // let result = find_roots_of_quadratic_equation(a, b, c);
                    // if result.0 > 0 {
                    //     // converge_vel = v0 + acc * result.1;
                    //     // todo: test linear function
                    //     converge_vel = v0 + acc * ds;
                    // }
                    converge_vel = v0 + acc * ds;

                    // info!("converge_vel find_roots_of_quadratic_equation: ds: {},acc: {}, v0:{}, vel: {}",ds, acc, v0, converge_vel);
                }

                {
                    let mut vel_ratio = 1.0;
                    vel_ratio = converge_vel / current_driver_cmd_vy.abs();

                    vel_ratio = vel_ratio.min(1.0).max(0.0);
                    let converge_vel = allow_forward_vel * vel_ratio;
                    allow_forward_vel = allow_forward_vel.min(converge_vel);
                }
                // search point
                //
                let line_search_dist = opt.line_search_dist;
                let line_switch_dist = opt.line_switch_dist;

                let line_search_dist_2 = line_search_dist * line_search_dist;
                let aux_direction = if aux_control_point[0] > 0.0 { 0.0 } else { PI };

                // todo: decrease
                let current_control_pose_in_target_end_y_abs = current_control_pose_in_target_end_y.abs();
                let mut relative_target_x = if control_pose_in_target_end[0] < -line_switch_dist {
                    // if current_control_pose_in_target_end_y_abs< 0.01{
                    //     line_search_dist + (0.03/(current_control_pose_in_target_end_y_abs + 0.0001)).min(opt.line_search_dist_min)
                    // }else{
                    //     line_search_dist
                    // }
                    if current_control_pose_in_target_end_y_abs > opt.line_search_converge_dist{
                        line_search_dist
                            .min(current_control_pose_in_target_end_y_abs + opt.line_search_dist_min)
                    }else{
                        opt.line_search_converge_ratio * line_search_dist
                    }


                } else {
                    10.0 * line_search_dist
                };

                // if control_pose_in_target_start[0] < -0.5{
                //     relative_target_x += -control_pose_in_target_start[0];
                // }

                {
                    #[cfg(aaa)]
                    {
                        let mut dist_valid_node_id = closest_control_id;
                        for i in closest_control_id..=target_end_id {
                            let n = self.global_path.borrow()[i];

                            let relative_flow =
                                closest_control_node.flow_pose_inv.multiply_pose(&n.flow);
                            dist_valid_node_id = i;
                            if ((relative_flow[0] > opt.line_search_dist)
                                || (vector_2d_norm2!(relative_flow) > line_search_dist_2))
                            // && ( (angle_norm!(relative_flow[1].atan2(relative_flow[0]) - aux_direction)).abs() < FRAC_PI_2  )
                            {
                                break;
                            }
                        }

                        current_target_in_control_pose = aux_control_point_transform_inv
                            .multiply_pose(&self.global_path.borrow()[dist_valid_node_id].pose);
                    }

                    let mut control_pose_in_target_end_fix = control_pose_in_target_end;


                    if is_goal_line{
                        if opt.constrain_stop_no_rotate && (current_pose_in_target_end[0] > -opt.constrain_stop_no_rotate_run_dist) {
                            if opt.final_force_parallel_direction {
                                control_pose_in_target_end_fix[1] = (control_pose_in_target_end[1]);
                                control_pose_in_target_end_fix[0] = control_pose_in_target_end[0] + relative_target_x;
                            }else{

                                control_pose_in_target_end_fix[1] = 0.0;
                                control_pose_in_target_end_fix[0] = control_pose_in_target_end[0] + opt.final_relative_target_x;
                            }
                        } else{
                            control_pose_in_target_end_fix[0] = control_pose_in_target_end[0] + relative_target_x;
                            control_pose_in_target_end_fix[1] = 0.0;
                        }
                    }else{
                        control_pose_in_target_end_fix[0] = control_pose_in_target_end[0] + relative_target_x;
                        //todo: avoid rotate
                        if current_control_pose_in_target_end_y_abs > opt.line_search_converge_dist{
                            control_pose_in_target_end_fix[1] = 0.0;

                        }else{
                            control_pose_in_target_end_fix[1] = 0.5*(control_pose_in_target_end[1]);
                        }
                        control_pose_in_target_end_fix[1] = 0.0;

                    }


                    // control_pose_in_target_end_fix[1] = 0.0;



                    let mut target_posed = target_end_node
                        .flow_pose
                        .multiply_pose(&control_pose_in_target_end_fix);
                    target_posed[2] = target_end_node.pose[2];
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&target_posed);
                }

                // else {
                //     let mut target_pose_flow = self.global_path.borrow()[target_end_id].flow;
                //     let mut target_pose = self.global_path.borrow()[target_end_id].pose;
                //     target_pose[0] += 5.0 * opt.line_search_dist * target_pose_flow[2].cos();
                //     target_pose[1] += 5.0 * opt.line_search_dist * target_pose_flow[2].sin();
                //
                //     current_target_in_control_pose =
                //         aux_control_point_transform_inv.multiply_pose(&target_pose);
                // }

                if (current_target_in_control_pose[0] > 0.0) && (aux_control_point[0] < 0.0) {
                    current_target_in_control_pose[0] = -1e-3* current_target_in_control_pose[1].abs();

                } else if (current_target_in_control_pose[0] < 0.0) && (aux_control_point[0] > 0.0)
                {
                    current_target_in_control_pose[0] = 1e-3* current_target_in_control_pose[1].abs();
                }

                // current_target_in_control_pose[0] = (aux_control_point[0]  ).copysign(aux_control_point[0]);
                *self.current_target_in_control_pose.borrow_mut() = current_target_in_control_pose;
                *self.current_target_pose.borrow_mut() =
                    aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

                warn!(
                    ?current_target_in_control_pose,
                    allow_forward_vel,
                    ?aux_control_point,
                    allow_forward_vel
                );
                self.controller.borrow().follow_with_control(
                    &current_target_in_control_pose,
                    allow_forward_vel,
                    &aux_control_point,
                );

                // warn!(is_goal_line, ?current_pose_in_target_end);

                if is_goal_line {
                    if current_pose_in_target_end[0] > -opt.constrain_stop_allow_offset_x {
                        *self.reach_goal_num.borrow_mut() += 1;
                    }
                    if *self.reach_goal_num.borrow() > opt.reach_goal_num {
                        task = RobotMotionPlannerTask::Finished;
                        self.stop();
                    }
                } else {
                    let next_target_start_id = target_end_id + 1;
                    let nex_target_start = self.global_path.borrow()[next_target_start_id];
                    let nex_target_end = self.global_path.borrow()[nex_target_start.segment_end_id];
                    // warn!(?control_pose_in_target_end,?target_end_node,?nex_target_start,?nex_target_end);

                    if control_pose_in_target_end[0] > -line_switch_dist {
                        if nex_target_start.flow_type >= 10 {
                            task = RobotMotionPlannerTask::Line;

                            self.planner_data.borrow_mut().target_start_id = next_target_start_id;
                            self.planner_data.borrow_mut().target_end_id =
                                nex_target_start.segment_end_id;
                        } else {
                            task = RobotMotionPlannerTask::Curve;

                            let mut target_end_id = next_target_start_id;
                            for i in closest_base_id..path_len {
                                target_end_id = i;
                                if (self.global_path.borrow()[i].flow_type >= 10)
                                    && (self.global_path.borrow()[i].dist_to_segment_end
                                        > opt.valid_line_length)
                                {
                                    break;
                                }
                            }
                            self.planner_data.borrow_mut().target_start_id = next_target_start_id;
                            self.planner_data.borrow_mut().target_end_id = target_end_id;
                        }

                        let target_end_id = self.planner_data.borrow().target_end_id;

                        let target_end_node = self.global_path.borrow()[target_end_id];

                        let control_pose_in_target_end = target_end_node
                            .flow_pose_inv
                            .multiply_pose(&aux_control_point_abs);

                        self.current_control_pose_in_target_end_y
                            .borrow_mut()
                            .fill(control_pose_in_target_end[1]);
                        self.current_control_pose_in_target_end_x
                            .borrow_mut()
                            .fill(control_pose_in_target_end[0]);
                        self.record_base_pose();
                        self.planner_data.borrow_mut().task_counter = 0;
                        warn!("Task switch to {:?}", task);
                    }
                }
            }
            RobotMotionPlannerTask::StartNoRotate => {
                // exit condition:
                // remain_dist < 0.0

                // success => Init
                // fail => LargeOffset

                let mut allow_forward_vel: f64 = opt.constrain_start_no_rotate_allow_vel;
                let remain_dist =
                    opt.constrain_start_no_rotate_run_dist - current_pose_in_record[0].abs();

                // todo: error detection

                if current_pose_in_record[0].abs() > opt.constrain_start_no_rotate_allow_offset_y {
                    allow_forward_vel = 0.0;
                    self.planner_data.borrow_mut().error = RobotMotionPlannerError::LargeOffset;
                    warn!("LargeOffset: {:?}", current_pose_in_record);
                }

                if remain_dist > 0.0 {
                    current_target_in_control_pose = aux_control_point;
                    *self.current_target_in_control_pose.borrow_mut() =
                        current_target_in_control_pose;
                    *self.current_target_pose.borrow_mut() =
                        aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

                    *self.current_target_diff.borrow_mut() = [allow_forward_vel, remain_dist, 0.0];

                    self.controller.borrow().follow_with_control(
                        &current_target_in_control_pose,
                        allow_forward_vel,
                        &aux_control_point,
                    );
                } else {
                    task = RobotMotionPlannerTask::Init;
                    warn!("Task switch to {:?}", task);
                }
            }
            RobotMotionPlannerTask::StopNoRotate => {}
            RobotMotionPlannerTask::Finished => {}
            RobotMotionPlannerTask::StopRotate => {}
            RobotMotionPlannerTask::Curve => {
                // exit condition
                // pose in target_end.y < switch dist

                //

                let mut target_start_id = self.planner_data.borrow().target_start_id;
                let mut target_end_id = self.planner_data.borrow().target_end_id;

                let target_end_node = self.global_path.borrow()[target_end_id];
                let control_pose_in_target_end = target_end_node
                    .flow_pose_inv
                    .multiply_pose(&aux_control_point_abs);

                let current_pose_in_target_end = target_end_node
                    .flow_pose_inv
                    .multiply_pose(&current_pose);


                let (_, current_control_pose_in_target_end_y) =
                    self.current_control_pose_in_target_end_y.borrow().mean();
                let (_, current_control_pose_in_target_end_x) =
                    self.current_control_pose_in_target_end_x.borrow().mean();
                let mut driver_direction_in_target_end =
                    angle_norm!(control_pose_in_target_end[2] + current_driver_state_forward_angle);

                let current_driver_cmd_vx =
                    current_driver_state_forward_vel * driver_direction_in_target_end.cos();
                let current_driver_cmd_vy =
                    current_driver_state_forward_vel * driver_direction_in_target_end.sin();

                {
                    let predict_move_y = control_interval_s * current_driver_cmd_vy;
                    let predict_move_x = control_interval_s * current_driver_cmd_vx;
                    let real_move_y =
                        control_pose_in_target_end[1] - current_control_pose_in_target_end_y;
                    let real_move_x =
                        control_pose_in_target_end[0] - current_control_pose_in_target_end_x;
                    let move_y_diff = predict_move_y - real_move_y;
                    let move_x_diff = predict_move_x - real_move_x;

                    warn!(
                        ?control_pose_in_target_end,
                        predict_move_y,
                        predict_move_x,
                        real_move_y,
                        real_move_x,
                        move_y_diff,
                        move_x_diff,
                        current_control_pose_in_target_end_y,
                        current_control_pose_in_target_end_x
                    );
                    if (move_y_diff.abs() > opt.pose_jump_alert_y)
                        || (move_x_diff.abs() > opt.pose_jump_alert_x)
                    {
                        self.planner_data.borrow_mut().error = RobotMotionPlannerError::PoseJump;
                        self.stop();
                        return false;
                    }

                    if current_control_pose_in_closest_node[1].abs() > opt.control_closest_alert {
                        self.planner_data.borrow_mut().error =
                            RobotMotionPlannerError::ClosestDistError;
                        self.stop();
                        return false;
                    }
                }

                self.current_control_pose_in_target_end_y
                    .borrow_mut()
                    .update(control_pose_in_target_end[1]);

                self.current_control_pose_in_target_end_x
                    .borrow_mut()
                    .update(control_pose_in_target_end[0]);
                let (_, current_control_pose_in_target_end_y) =
                    self.current_control_pose_in_target_end_y.borrow().mean();
                let (_, current_control_pose_in_target_end_x) =
                    self.current_control_pose_in_target_end_x.borrow().mean();

                *self.current_vel_in_target_end.borrow_mut() =
                    [current_driver_cmd_vx, current_driver_cmd_vy];

                let mut allow_forward_vel: f64 =
                    if current_control_pose_in_target_end_x.abs() > opt.track_curve_end_dist {
                        opt.track_curve_vel
                    } else {
                        opt.track_curve_end_vel
                    };

                if current_control_pose_in_closest_node[1].abs()
                    > opt.curve_use_closest_direction_dist
                {
                    allow_forward_vel = opt.track_curve_vel_decay_min.max(
                        allow_forward_vel
                            - opt.track_curve_vel_decay_ratio
                                * current_control_pose_in_closest_node[1].abs(),
                    );
                }




                let reach_end =
                    //control_pose_in_target_end[1].abs() < opt.switch_line_dist
                    ((closest_base_node.dist_to_end - target_end_node.dist_to_end) < opt.switch_line_path_dist)
                    &&
                       (
                           ((current_pose_in_target_end[1] < 0.0 ) &&  (current_control_pose_in_target_end_y  >- opt.switch_line_dist))

                           || ( (current_pose_in_target_end[1] > 0.0 ) &&  (current_control_pose_in_target_end_y  < opt.switch_line_dist
                       ))
                       )
                    ;

                // warn!("reach_end: {}, control_pose_in_target_end: {:?},path_dist: {} ,switch_line_path_dist: {} ",reach_end,control_pose_in_target_end,closest_base_node.dist_to_end - target_end_node.dist_to_end, opt.switch_line_path_dist);

                warn!(
                    reach_end,?current_pose_in_target_end,target_start_id,target_end_id,
                    current_driver_cmd_vy, current_control_pose_in_target_end_y
                );
                if !reach_end {
                    let current_target_in_control_pose = self.search_curve(opt);
                    *self.current_target_in_control_pose.borrow_mut() =
                        current_target_in_control_pose;
                    *self.current_target_pose.borrow_mut() =
                        aux_control_point_transform.multiply_pose(&current_target_in_control_pose);

                    *self.current_target_diff.borrow_mut() = [allow_forward_vel, 0.0, 0.0];

                    self.controller.borrow().follow_with_control(
                        &current_target_in_control_pose,
                        allow_forward_vel,
                        &aux_control_point,
                    );

                    // add
                } else {
                    if target_end_id != goal_node_id {
                        self.planner_data.borrow_mut().target_start_id = target_end_id;
                        self.planner_data.borrow_mut().target_end_id =
                            target_end_node.segment_end_id;
                    }

                    task = RobotMotionPlannerTask::Line;

                    let target_end_id = self.planner_data.borrow().target_end_id;

                    let target_end_node = self.global_path.borrow()[target_end_id];

                    let control_pose_in_target_end = target_end_node
                        .flow_pose_inv
                        .multiply_pose(&aux_control_point_abs);

                    self.current_control_pose_in_target_end_y
                        .borrow_mut()
                        .fill(control_pose_in_target_end[1]);
                    self.current_control_pose_in_target_end_x
                        .borrow_mut()
                        .fill(control_pose_in_target_end[0]);
                    self.record_base_pose();
                    self.planner_data.borrow_mut().task_counter = 0;
                    warn!("Task switch to {:?}", task);
                }
            }
            RobotMotionPlannerTask::LineToCurve => {}
            RobotMotionPlannerTask::CurveToLine => {}
            RobotMotionPlannerTask::LineToLine => {}
            RobotMotionPlannerTask::CurveToCurve => {}
        }
        self.planner_data.borrow_mut().task = task;

        let finished = task == RobotMotionPlannerTask::Finished;

        finished
    }

    pub fn init(&self) -> bool
    {
        // check robot speed is stopped

        self.controller
            .borrow()
            .update_base_pose(&self.current_pose.borrow());
        self.controller.borrow().is_stopped()
    }

    fn record_base_pose(&self)
    {
        *self.record_base_pose.borrow_mut() = *self.current_pose.borrow();

        self.controller
            .borrow()
            .record_base_pose(&self.current_pose.borrow());
    }

    #[inline]
    pub fn stop(&self)
    {
        let state = self.planner_data.borrow_mut().state;
        let task = self.planner_data.borrow_mut().task;
        let error = self.planner_data.borrow_mut().error;

        warn!(
            "stop at: state: {:?}, task: {:?}, error: {:?}",
            state, task, error
        );

        self.controller.borrow().stop();
    }

    fn search_curve(&self, opt: &RobotMotionPlannerOption) -> [f64; 3]
    {
        // given target_start and target_end
        let mut target_start_id = self.planner_data.borrow().target_start_id;
        let target_end_id = self.planner_data.borrow().target_end_id;

        let closest_base_id = self.planner_data.borrow().closest_base_id;
        let closest_control_id = self.planner_data.borrow().closest_control_id;

        let closest_base_node = self.global_path.borrow()[closest_base_id];
        let closest_control_node = self.global_path.borrow()[closest_control_id];

        let search_dist_min2 = opt.curve_search_min * opt.curve_search_min;
        let search_dist_max2 = opt.curve_search_max * opt.curve_search_max;

        let current_pose = *self.current_pose.borrow();

        let aux_control_point_abs = *self.current_control_pose.borrow();
        let aux_control_point_transform = Transform2d::new(&aux_control_point_abs);
        let aux_control_point_transform_inv = aux_control_point_transform.inverse();

        let mut current_target_in_control_pose = *self.current_target_in_control_pose.borrow();
        let mut current_target_flow_in_control_pose = current_target_in_control_pose;

        let mut last_direction =
            current_target_in_control_pose[1].atan2(current_target_in_control_pose[0]);

        let [current_driver_state_forward_vel, current_driver_state_forward_angle, _] =
            *self.current_driver_state.borrow();

        #[cfg(c)]
        {
            if current_driver_state_forward_vel.abs() > 0.001 {
                last_direction = current_driver_state_forward_angle;
            }
        }

        let mut task_counter = self.planner_data.borrow_mut().task_counter;
        self.planner_data.borrow_mut().task_counter += 1;

        let current_control_pose_in_closest_node = *self.current_control_pose_in_closest.borrow();
        // closest_control_node.flow_pose_inv.multiply_pose(&aux_control_point_abs);
        // pose in closest node
        // if
        let use_closest_node_direction = (current_control_pose_in_closest_node[1].abs()
            < opt.curve_use_closest_direction_dist)
            && (closest_control_node.flow_type < 10);

        target_start_id = target_start_id.max(closest_control_id);
        let mut valid_start_id = target_start_id;
        let mut valid_end_id = target_start_id;

        if (task_counter > 0) && (use_closest_node_direction) {
            #[cfg(a)]
            {
                valid_start_id = target_start_id.max(closest_control_id);
                valid_end_id = target_end_id;

                let mut direction_valid_id = valid_start_id;

                for i in valid_start_id..=valid_end_id {
                    let mut n = &mut self.global_path.borrow_mut()[i];
                    let flow_in_current = aux_control_point_transform_inv.multiply_pose(&n.flow);
                    let flow_in_current_min =
                        aux_control_point_transform_inv.multiply_pose(&n.flow_edge_min);
                    let flow_in_current_max =
                        aux_control_point_transform_inv.multiply_pose(&n.flow_edge_max);
                    let flow_in_current_direction = flow_in_current[1].atan2(flow_in_current[0]);
                    let flow_in_current_direction_min =
                        flow_in_current_min[1].atan2(flow_in_current_min[0]);
                    let flow_in_current_direction_max =
                        flow_in_current_max[1].atan2(flow_in_current_max[0]);
                    let flow_into_direction =
                        angle_norm!(flow_in_current[2] - flow_in_current_direction);

                    n.flow_in_current = flow_in_current;
                    n.flow_into_direction = flow_into_direction;
                    n.flow_in_current_direction = flow_in_current_direction;
                    n.flow_in_current_direction_min = flow_in_current_direction_min;
                    n.flow_in_current_direction_max = flow_in_current_direction_max;

                    if flow_into_direction.abs() > FRAC_PI_2 {
                        direction_valid_id = i;
                    }
                }

                if use_closest_node_direction {
                    current_target_flow_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&closest_control_node.flow);
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&closest_control_node.pose);

                    current_target_in_control_pose[0] =
                        0.5 * current_target_flow_in_control_pose[2].cos();
                    current_target_in_control_pose[1] =
                        0.5 * current_target_flow_in_control_pose[2].sin();
                } else {
                    direction_valid_id = valid_end_id.min(direction_valid_id + 10);

                    let direction_valid_node = self.global_path.borrow()[direction_valid_id];

                    current_target_flow_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&direction_valid_node.flow);
                    current_target_in_control_pose =
                        aux_control_point_transform_inv.multiply_pose(&direction_valid_node.pose);

                    current_target_in_control_pose[0] +=
                        opt.curve_linear_fix * current_target_flow_in_control_pose[2].cos();
                    current_target_in_control_pose[1] +=
                        opt.curve_linear_fix * current_target_flow_in_control_pose[2].sin();
                }
            }

            current_target_flow_in_control_pose =
                aux_control_point_transform_inv.multiply_pose(&closest_control_node.flow);
            current_target_in_control_pose =
                aux_control_point_transform_inv.multiply_pose(&closest_control_node.pose);

            current_target_in_control_pose[0] = 0.5 * current_target_flow_in_control_pose[2].cos();
            current_target_in_control_pose[1] = 0.5 * current_target_flow_in_control_pose[2].sin();

            warn!(?current_target_in_control_pose);
        }

        if (task_counter == 0) || (!use_closest_node_direction) {
            if task_counter > 0 {
                for i in target_start_id..=target_end_id {
                    let n = self.global_path.borrow()[i];
                    let dist2 = vector_2d_dist2!(aux_control_point_abs, n.pose);

                    valid_end_id = i;
                    if dist2 < search_dist_min2 {
                        valid_start_id = i;
                    }
                    if (dist2 > search_dist_max2) {
                        break;
                    }
                }
            } else {
                for i in target_start_id..=target_end_id {
                    let n = self.global_path.borrow()[i];
                    let dist2 = vector_2d_dist2!(aux_control_point_abs, n.pose);

                    valid_end_id = i;
                    if dist2 < search_dist_min2 {
                        valid_start_id = i;
                    }
                    // if (dist2 > search_dist_max2 )  {
                    //     break;
                    // }
                }
            }

            for i in valid_start_id..=valid_end_id {
                let mut n = &mut self.global_path.borrow_mut()[i];
                let flow_in_current = aux_control_point_transform_inv.multiply_pose(&n.flow);
                let flow_in_current_min =
                    aux_control_point_transform_inv.multiply_pose(&n.flow_edge_min);
                let flow_in_current_max =
                    aux_control_point_transform_inv.multiply_pose(&n.flow_edge_max);
                let flow_in_current_direction = flow_in_current[1].atan2(flow_in_current[0]);
                let flow_in_current_direction_min =
                    flow_in_current_min[1].atan2(flow_in_current_min[0]);
                let flow_in_current_direction_max =
                    flow_in_current_max[1].atan2(flow_in_current_max[0]);
                let flow_into_direction =
                    angle_norm!(flow_in_current[2] - flow_in_current_direction);

                n.flow_in_current = flow_in_current;
                n.flow_into_direction = flow_into_direction;
                n.flow_in_current_direction = flow_in_current_direction;
                n.flow_in_current_direction_min = flow_in_current_direction_min;
                n.flow_in_current_direction_max = flow_in_current_direction_max;
                if flow_into_direction.abs() > FRAC_PI_2 {
                    valid_start_id = i;
                }
            }

            warn!(
                last_direction,
                current_driver_state_forward_angle,
                current_driver_state_forward_vel,
                ?current_target_in_control_pose
            );
            warn!(valid_start_id, valid_end_id);

            let mut best_direction = last_direction;

            let mut best_direction_id = valid_start_id;
            let mut select_target_id = valid_start_id;
            let mut best_direction_diff_abs: f64 = 1000.0;

            for i in valid_start_id..=valid_end_id {
                let mut n = &mut self.global_path.borrow_mut()[i];

                let mut flow_in_current_direction = n.flow_in_current_direction;
                let flow_in_current_direction_min = n.flow_in_current_direction_min;
                let flow_in_current_direction_max = n.flow_in_current_direction_max;
                let flow_into_direction = n.flow_into_direction;

                let mut diff = (angle_norm!(flow_in_current_direction - last_direction));
                let diff_min = (angle_norm!(flow_in_current_direction_min - last_direction));
                let diff_max = (angle_norm!(flow_in_current_direction_max - last_direction));
                if ((diff_min < 0.0 && diff_max > 0.0) || (diff_min > 0.0 && diff_max < 0.0)) {
                    flow_in_current_direction = last_direction;
                    diff = 0.0;
                } else if diff_min.abs() < diff_max.abs() {
                    flow_in_current_direction = flow_in_current_direction_min;
                    diff = diff_min.abs();
                } else if diff_min.abs() > diff_max.abs() {
                    flow_in_current_direction = flow_in_current_direction_max;
                    diff = diff_max.abs();
                }
                diff += 1.5 * flow_into_direction.abs();
                if (diff < best_direction_diff_abs) || (diff < 1e-4) {
                    best_direction = flow_in_current_direction;
                    best_direction_diff_abs = diff;
                    best_direction_id = i;
                }
            }

            warn!(best_direction_id, best_direction_diff_abs, best_direction);

            current_target_flow_in_control_pose = aux_control_point_transform_inv
                .multiply_pose(&self.global_path.borrow()[best_direction_id].flow);
            current_target_in_control_pose = aux_control_point_transform_inv
                .multiply_pose(&self.global_path.borrow()[best_direction_id].pose);

            target_start_id = best_direction_id;

            let dist = (vector_2d_norm2!(current_target_in_control_pose)).sqrt();

            //todo: check why best_direction_id move too fast

            current_target_in_control_pose[0] = dist * best_direction.cos()
                + opt.curve_linear_fix * current_target_flow_in_control_pose[2].cos();
            current_target_in_control_pose[1] = dist * best_direction.sin()
                + opt.curve_linear_fix * current_target_flow_in_control_pose[2].sin();
            warn!(?current_target_in_control_pose);
        }

        self.planner_data.borrow_mut().target_start_id = target_start_id;

        current_target_in_control_pose
    }

    pub fn go(&self) -> bool
    {
        let mut last_state = self.planner_data.borrow_mut().state;
        let mut new_state = last_state;

        let error = self.planner_data.borrow_mut().error;

        let task = self.planner_data.borrow_mut().task;

        let mut change_state = false;

        if error != RobotMotionPlannerError::OK {
            change_state = true;
            last_state = RobotMotionPlannerState::Error;
            new_state = RobotMotionPlannerState::Error;
        }

        match last_state {
            RobotMotionPlannerState::Idle => {}
            RobotMotionPlannerState::Init => {
                if self.init() {
                    change_state = true;

                    if task != RobotMotionPlannerTask::StartNoRotate {
                        let binding = self.select_option.borrow();

                        let opt = match binding.as_ref() {
                            Some(t) => t,
                            None => {
                                log!("select_option doesn't exist");
                                return false;
                            }
                        };

                        // check diff
                        let closest_base_id = self.planner_data.borrow().closest_base_id;
                        let closest_base_node = self.global_path.borrow()[closest_base_id];

                        let current_pose = *self.current_pose.borrow();
                        let aux_control_point = *self.aux_control_point.borrow();
                        let aux_control_point_dist = (vector_2d_norm2!(aux_control_point)); //.sqrt();

                        let first_angle_diff =
                            angle_norm!(closest_base_node.pose[2] - current_pose[2]);

                        if first_angle_diff.abs() > opt.start_inplace_rotate_tolerance {
                            let path_len = self.global_path.borrow().len();

                            let mut target_id: usize = closest_base_id;
                            let closest_dist_to_end = closest_base_node.dist_to_end;

                            for i in closest_base_id..path_len {
                                target_id = i;
                                let target_pose = self.global_path.borrow()[i].pose;
                                let dist = vector_2d_dist2!(target_pose, current_pose);
                                if dist > aux_control_point_dist
                                // && (
                                //     ( (aux_control_point[0] > 0.0)   )
                                //     ||
                                //         ( (aux_control_point[0] < 0.0)  )
                                //     )
                                {
                                    break;
                                }
                            }
                            let target_node = self.global_path.borrow()[target_id];
                            let mut target_node_direction = (target_node.pose[1] - current_pose[1])
                                .atan2(target_node.pose[0] - current_pose[0]);
                            warn!(
                                "first_rotate target: {:?}, target_node_direction: {}",
                                target_node, target_node_direction
                            );

                            if aux_control_point[0] > 0.0 {
                            } else {
                                target_node_direction = angle_norm!(target_node_direction + PI);
                            }

                            new_state = RobotMotionPlannerState::FirstRotate;

                            self.current_first_rotate_target.borrow_mut()[0] =
                                target_node_direction;
                        } else {
                            new_state = RobotMotionPlannerState::Prepare;
                        }
                    } else {
                        new_state = RobotMotionPlannerState::Prepare;
                    }

                    self.controller.borrow().sync_wheel();
                    self.record_base_pose();
                }
            }

            RobotMotionPlannerState::FirstRotate => {
                let target = self.current_first_rotate_target.borrow()[0];
                let current_pose_yaw = self.current_pose.borrow()[2];
                self.current_first_rotate_target.borrow_mut()[1] =
                    angle_norm!(target - current_pose_yaw);

                if self.rotate(target) {
                    change_state = true;

                    new_state = RobotMotionPlannerState::Prepare;
                    self.controller.borrow().sync_wheel();
                    self.record_base_pose();
                }
            }
            RobotMotionPlannerState::Prepare => {
                if self.prepare() {
                    change_state = true;

                    let binding = self.select_option.borrow();

                    let opt = match binding.as_ref() {
                        Some(t) => t,
                        None => {
                            log!("select_option doesn't exist");
                            return false;
                        }
                    };

                    match opt.control_program {
                        0 => {
                            new_state = RobotMotionPlannerState::FollowPath;
                        }
                        1 => {
                            new_state = RobotMotionPlannerState::FollowGoal;
                        }
                        _ => {
                            self.planner_data.borrow_mut().error =
                                RobotMotionPlannerError::UnknownProgram;
                            new_state = RobotMotionPlannerState::Error;
                            self.stop();
                        }
                    }
                    self.record_base_pose();
                }
            }
            RobotMotionPlannerState::FollowPath => {
                if self.follow_path() {
                    change_state = true;

                    new_state = RobotMotionPlannerState::Finished;
                    self.record_base_pose();
                }
            }
            RobotMotionPlannerState::FollowGoal => {
                if self.follow_goal() {
                    change_state = true;

                    new_state = RobotMotionPlannerState::Finished;
                    self.record_base_pose();
                }
            }

            RobotMotionPlannerState::Finished => {}
            RobotMotionPlannerState::Error => {}
            s => {
                // log!("{:?}",s);
            }
        }
        if (change_state) {
            self.planner_data.borrow_mut().state = new_state;
        }
        true
    }

    pub fn set_robot_pose(&self, pose: &[f64; 3])
    {
        *self.current_pose.borrow_mut() = *pose;
        self.controller.borrow().update_base_pose(pose);
    }

    pub fn set_error(&self, error: RobotMotionPlannerError)
    {
        self.planner_data.borrow_mut().error = error;
    }
}
