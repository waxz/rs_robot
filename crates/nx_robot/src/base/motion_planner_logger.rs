use crate::base::motion_planner::{RobotMotionPlanner, RobotMotionPlannerState};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

pub fn log(
    robot_motion_planner: Rc<RefCell<RobotMotionPlanner>>,
    plot_scalar: Arc<Mutex<HashMap<&'static str, f64>>>,
    plot_pose: Arc<Mutex<HashMap<&'static str, [f64; 3]>>>,
)
{
    let current_pose_array = *robot_motion_planner.borrow().current_pose.borrow();
    *plot_scalar
        .lock()
        .unwrap()
        .entry("data/current/base/x")
        .or_insert(0.0) = current_pose_array[0] as f64;
    *plot_scalar
        .lock()
        .unwrap()
        .entry("data/current/base/y")
        .or_insert(0.0) = current_pose_array[1] as f64;
    *plot_scalar
        .lock()
        .unwrap()
        .entry("data/current/base/yaw")
        .or_insert(0.0) = current_pose_array[2] as f64;

    //goal
    let current_first_rotate_target = *robot_motion_planner
        .borrow()
        .current_first_rotate_target
        .borrow();
    let current_control_pose_in_goal_flow = *robot_motion_planner
        .borrow()
        .current_control_pose_in_goal_flow
        .borrow();
    let current_base_pose_in_goal_flow = *robot_motion_planner
        .borrow()
        .current_base_pose_in_goal_flow
        .borrow();

    let closest_base_id = robot_motion_planner
        .borrow()
        .planner_data
        .borrow()
        .closest_base_id;

    let closest_control_id = robot_motion_planner
        .borrow()
        .planner_data
        .borrow()
        .closest_control_id;

    let target_start_id = robot_motion_planner
        .borrow()
        .planner_data
        .borrow()
        .target_start_id;

    let target_end_id = robot_motion_planner
        .borrow()
        .planner_data
        .borrow()
        .target_end_id;

    let (_, current_control_pose_in_target_end_y) = robot_motion_planner
        .borrow()
        .current_control_pose_in_target_end_y
        .borrow()
        .mean();
    let (_, current_control_pose_in_target_end_x) = robot_motion_planner
        .borrow()
        .current_control_pose_in_target_end_x
        .borrow()
        .mean();

    let current_vel_in_target_end = *robot_motion_planner
        .borrow()
        .current_vel_in_target_end
        .borrow();

    let current_control_pose_in_closest = *robot_motion_planner
        .borrow()
        .current_control_pose_in_closest
        .borrow();

    let path_len = robot_motion_planner.borrow().global_path.borrow().len();

    let state = robot_motion_planner.borrow().planner_data.borrow().state;

    //node
    {
        if ((state != RobotMotionPlannerState::Idle)
            && (state != RobotMotionPlannerState::Error)
            && (target_end_id < path_len)
            && (closest_base_id < path_len)
            && (closest_control_id < path_len))
        {
            let closest_base_node =
                robot_motion_planner.borrow().global_path.borrow()[closest_base_id];
            let closest_control_node =
                robot_motion_planner.borrow().global_path.borrow()[closest_control_id];

            let target_start_node =
                robot_motion_planner.borrow().global_path.borrow()[target_start_id];
            let target_end_node = robot_motion_planner.borrow().global_path.borrow()[target_end_id];

            let current_target_pose = *robot_motion_planner.borrow().current_target_pose.borrow();

            *plot_pose
                .lock()
                .unwrap()
                .entry("sim/path/closest_base_node")
                .or_insert([0.0; 3]) = closest_base_node.pose;
            *plot_pose
                .lock()
                .unwrap()
                .entry("sim/path/closest_control_node")
                .or_insert([0.0; 3]) = closest_control_node.pose;
            *plot_pose
                .lock()
                .unwrap()
                .entry("sim/path/current_target_pose")
                .or_insert([0.0; 3]) = current_target_pose;
            *plot_pose
                .lock()
                .unwrap()
                .entry("sim/path/target_start_node")
                .or_insert([0.0; 3]) = target_start_node.pose;
            *plot_pose
                .lock()
                .unwrap()
                .entry("sim/path/target_end_node")
                .or_insert([0.0; 3]) = target_end_node.pose;
        }
    }

    // scalar
    {
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current/current_control_pose_in_target_end/vx")
            .or_insert(0.0) = current_vel_in_target_end[0];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current/current_control_pose_in_target_end/vy")
            .or_insert(0.0) = current_vel_in_target_end[1];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_control_pose_in_closest/x")
            .or_insert(0.0) = current_control_pose_in_closest[0];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_control_pose_in_closest/y")
            .or_insert(0.0) = current_control_pose_in_closest[1];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_control_pose_in_closest/yaw")
            .or_insert(0.0) = current_control_pose_in_closest[2];

        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_rotate/target")
            .or_insert(0.0) = current_first_rotate_target[0];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_rotate/diff")
            .or_insert(0.0) = current_first_rotate_target[1];

        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/base/x")
            .or_insert(0.0) = current_base_pose_in_goal_flow[0];

        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/base/y")
            .or_insert(0.0) = current_base_pose_in_goal_flow[1];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/base/yaw")
            .or_insert(0.0) = current_base_pose_in_goal_flow[2];

        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/control/x")
            .or_insert(0.0) = current_control_pose_in_goal_flow[0];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/control/y")
            .or_insert(0.0) = current_control_pose_in_goal_flow[1];
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current_in_goal_flow/control/yaw")
            .or_insert(0.0) = current_control_pose_in_goal_flow[2];

        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current/current_control_pose_in_target_end/y")
            .or_insert(0.0) = current_control_pose_in_target_end_y;
        *plot_scalar
            .lock()
            .unwrap()
            .entry("data/current/current_control_pose_in_target_end/x")
            .or_insert(0.0) = current_control_pose_in_target_end_x;
    }
}
