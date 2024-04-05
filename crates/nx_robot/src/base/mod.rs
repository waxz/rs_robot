pub mod motion_controller;
pub mod motion_planner;
pub mod odometry;
pub mod path_planner;

pub mod motion_planner_logger;

use nx_common::common::interpolate::{
    build_bezier_from_pose, build_bezier_line_from_pose, compute_path_direction, compute_path_flow,
};
use nx_common::common::math::{line_line_intersect, linspace};
use nx_common::common::transform2d::Transform2d;
use nx_common::{angle_norm, bench, log, vector_2d_dist2};
use rand::Rng;
use serde::Deserialize;
use std::cell::RefCell;
use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;
use std::ops::{Deref, DerefMut};
use std::rc::Rc;
use std::time::SystemTime;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TwistState
{
    pub forward_vel: f64,
    pub forward_angle: f64,
    pub rotate_vel: f64,
}

impl From<&[f64; 3]> for TwistState
{
    fn from(value: &[f64; 3]) -> Self
    {
        Self {
            forward_vel: value[0],
            forward_angle: value[1],
            rotate_vel: value[2],
        }
    }
}

pub struct LinearTwistState
{
    linear_x: f64,
    linear_y: f64,
    angular_z: f64,
}

pub fn linear_angular_to_forward_rotate(v: &LinearTwistState) -> TwistState
{
    let forward_vel = (v.linear_x * v.linear_x + v.linear_y * v.linear_y).sqrt();
    let forward_angle = v.linear_y.atan2(v.linear_x);
    TwistState {
        forward_vel,
        forward_angle,

        rotate_vel: v.angular_z,
    }
}

pub fn forward_rotate_to_linear_angular(v: &TwistState) -> LinearTwistState
{
    let linear_x = v.forward_vel * v.forward_angle.cos();
    let linear_y = v.forward_vel * v.forward_angle.sin();
    LinearTwistState {
        linear_x,
        linear_y,
        angular_z: v.rotate_vel,
    }
}

//motion_panner,
// robot_path_planner, robot_motion_planner, robot_motion_controller, robot_hardware_control

// robot_motion_planner: path check, split, choose target point, check tolerance

// given goal and pose, compute a path

// impl Clone for RobotPathPlannerOption{
//     fn clone(&self) -> Self {
//         Self{
//             is_omnidirectional: false,
//             curve_param: self.curve_param.clone(),
//             node_interval: 0.0,
//         }
//     }
// }

// impl Clone for RobotPathPlannerConfig {
//     fn clone(&self) -> Self {
//         Self{ default_option: self.default_option.clone(), options: self.options.clone() }
//     }
// }

#[derive(Debug, Deserialize, Clone)]
pub struct RobotHardwareControlConfig
{
    balls: u8,
    bricks: u8,
}
/// # Computes rotate center of robot for control command
///
/// # parameter:
/// cmd = [forward_vel, forward_angle,  rotate_vel]
///
/// # return
/// (false, rotate_center) or (true, rotate_center)
pub fn find_rotate_center(cmd: &TwistState) -> (bool, [f64; 2])
{
    let TwistState {
        forward_vel,
        forward_angle,
        rotate_vel,
    } = cmd;

    let mut rotate_center: [f64; 2] = [0.0, 0.0];

    let [rotate_center_x, rotate_center_y] = &mut rotate_center;
    if rotate_vel.abs() < f64::MIN_POSITIVE {
        return (false, rotate_center);
    } else {
        let command_rotate_radius = forward_vel / rotate_vel;
        let command_rotate_angle = forward_angle + FRAC_PI_2.copysign(command_rotate_radius);

        let command_rotate_radius_abs = command_rotate_radius.abs();
        *rotate_center_x = command_rotate_radius_abs * command_rotate_angle.cos();
        *rotate_center_y = command_rotate_radius_abs * command_rotate_angle.sin();

        return (true, rotate_center);
    }
}

pub fn twist_to_driver_cmd(cmd: &TwistState, driver_pos: &[f64; 2]) -> (bool, [f64; 2])
{
    let mut rotate_center = find_rotate_center(&cmd);
    // rotate_center.1[0] = 0.0;

    // println!("rotate_center: {:?}",rotate_center);
    compute_driver_cmd_use_rotate_center(cmd, &rotate_center, driver_pos)
}

/// compute velocity of driver on the robot
///
/// # return
///
/// (valid_motion, )
pub fn compute_driver_cmd_use_rotate_center(
    cmd: &TwistState,
    rotate: &(bool, [f64; 2]),
    driver_pos: &[f64; 2],
) -> (bool, [f64; 2])
{
    let TwistState {
        forward_vel,
        forward_angle,
        rotate_vel,
    } = cmd;

    let (is_rotate, [rotate_center_x, rotate_center_y]) = rotate;

    let [driver_pos_x, driver_pos_y] = driver_pos;
    // vel:[forward_vel, forward_angle]

    let mut command: [f64; 2] = [0.0, 0.0];
    let [driver_forward_vel, driver_forward_angle] = &mut command;

    if !is_rotate {
        *driver_forward_vel = *forward_vel;
        *driver_forward_angle = *forward_angle;
    } else {
        let angle = (driver_pos_y - rotate_center_y).atan2(driver_pos_x - rotate_center_x);

        let angle_estimate = angle + FRAC_PI_2;
        let mut cross_product = 0.0f64;

        {

            // let v = [angle_estimate.cos(), angle_estimate.sin()];
            // let u = [
            //     driver_pos_x - rotate_center_x,
            //     driver_pos_y - rotate_center_y,
            // ];
            // // let dot_product = u[0] * v[0] + u[1] * v[1];
            // cross_product = u[0] * v[1] - u[1] * v[0];
        }

        {
            // cross_product = (driver_pos_x - rotate_center_x)*( angle_estimate.sin()) - (driver_pos_x - rotate_center_x)*(angle_estimate.cos());
        }

        let driver_dist_to_rc = ((driver_pos_x - rotate_center_x)
            * (driver_pos_x - rotate_center_x)
            + (driver_pos_y - rotate_center_y) * (driver_pos_y - rotate_center_y))
            .sqrt();
        *driver_forward_vel = rotate_vel.abs() * driver_dist_to_rc;

        {
            *driver_forward_angle = angle + FRAC_PI_2.copysign(*rotate_vel);
        }
        {
            // *driver_forward_angle = angle + if rotate_vel.signum() ==cross_product.signum(){ FRAC_PI_2 }else{ -FRAC_PI_2 }
            // *driver_forward_angle = angle + FRAC_PI_2.copysign(rotate_vel * cross_product);
        }

        // log!("cmd:{:?}, rotate: {:?}, driver_forward_vel: {}, driver_forward_angle: {}",cmd,rotate,driver_forward_vel,driver_forward_angle)
    }
    return (true, command);
}

// given current angle, target angle, actual rotate_vel

pub fn compute_rotate_direction() -> i32
{
    0
}

/// # tyy
pub fn compute_rotate_cmd(
    current_forward_vel: f64,
    step_time_s: f64,
    forward_up_acc: f64,
    forward_dw_acc: f64,

    angle_diff: f64,

    direction: f64,
    radius: f64,
    forward_tolerance: f64,
    min_forward_vel: f64,
    max_forward_vel: f64,
    max_stop_vel_follow_error: f64,
) -> (bool, f64)
{
    let forward_diff = angle_diff * radius;
    let forward_vel = current_forward_vel.abs();

    log!(
        "direction: {}, forward_diff:{}, forward_tolerance:{}",
        direction,
        forward_diff,
        forward_tolerance
    );
    if ((direction > 0.0 && forward_diff < forward_tolerance)
        || (direction < 0.0 && forward_diff > -forward_tolerance))
    {
        log!("finished ");

        return (forward_vel < 1e-3, 0.0);
    }

    // speed from start
    let forward_vel_start = forward_vel + forward_up_acc * step_time_s;
    // speed to stop
    let mut forward_vel_stop =
        (2.0 * (forward_diff.abs() - forward_tolerance) * forward_dw_acc).sqrt();

    // what if lost control
    if ((forward_vel - forward_vel_stop) > max_stop_vel_follow_error) {
        forward_vel_stop = (forward_vel_stop - 0.1).max(min_forward_vel);
    }

    // trim range
    let mut final_forward_vel = forward_vel_start
        .min(forward_vel_stop)
        .min(max_forward_vel)
        .max(min_forward_vel);

    // copy sign
    // let mut final_rotate_vel = final_forward_vel / radius;

    final_forward_vel = final_forward_vel.copysign(angle_diff);
    (false, final_forward_vel)
}

#[cfg(test)]
mod test
{
    use serde::Deserialize;

    use crate::base::odometry::{update_odometry, OdometryCalculator, SingleSteering};
    use crate::base::{compute_driver_cmd_use_rotate_center, twist_to_driver_cmd, TwistState};
    use crate::base::{compute_rotate_cmd, find_rotate_center};
    use nx_common::{bench, log};

    use nx_common::common::transform2d::Transform2d;
    use rand::Rng;
    use std::f64::consts::PI;

    #[test]
    fn test_change()
    {
        {
            let (valid, twist) = SingleSteering::compute_twist(&[-0.1, 0.0], &[0.8, PI - 3.11798]);

            let (valid, state) = twist_to_driver_cmd(&twist, &[1.2, 0.0]);

            println!("state: {:?}", state);
        }
        {
            let (valid, twist) = SingleSteering::compute_twist(&[-0.1, 0.0], &[0.8, PI - 3.104268]);

            let (valid, state) = twist_to_driver_cmd(&twist, &[1.2, 0.0]);

            println!("state: {:?}", state);
        }
    }

    #[test]
    fn test_zero()
    {
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[0.0_f64, 0.0_f64, 0.0_f64]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[0.0, 0.0, 1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[0.0, 0.0, -1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[1.0, 0.0, 1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[1.0, 0.0, -1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[-1.0, 0.0, 1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
        {
            let (valid, cmd) =
                twist_to_driver_cmd(&TwistState::from(&[-1.0, 0.0, -1.0]), &[0.1, 0.0]);
            println!("valid: {}, cmd: {:?}", valid, cmd);
        }
    }

    #[test]
    fn test_cmd_to_rotate_center()
    {
        {
            let cmd: [f64; 3] = [0.1, 0.0, 0.0];
            let center = find_rotate_center(&TwistState::from(&cmd));
            println!("center:{:?}", center);
            let point = [0.0, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.0, -0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.5, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);
        }
        {
            let cmd: [f64; 3] = [0.0, 0.0, 0.1];
            let center = find_rotate_center(&TwistState::from(&cmd));
            println!("center:{:?}", center);
            let point = [0.0, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.0, -0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.5, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);
        }
        {
            let cmd: [f64; 3] = [0.1, 0.0, 0.1];
            let center = find_rotate_center(&TwistState::from(&cmd));
            log!("center:{:?}", center);

            let point = [0.0, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.0, -0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            let point = [0.5, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            println!("point: {:?}, vel: {:?}", point, vel);

            SingleSteering::compute_twist(&point, &(vel.1));
        }
        {
            let cmd: [f64; 3] = [0.1, 0.0, 0.1];
            let center = find_rotate_center(&TwistState::from(&cmd));
            let point = [0.5, 0.5];
            let vel =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);
            SingleSteering::compute_twist(&point, &(vel.1));
        }
    }

    #[test]
    fn test_nan()
    {
        {
            let cmd: [f64; 3] = [0.1, 0.0, 0.1];
            let center = find_rotate_center(&TwistState::from(&cmd));
            let point = [0.0, 0.5];
            let (valid, vel) =
                compute_driver_cmd_use_rotate_center(&TwistState::from(&cmd), &center, &point);

            let (valid, twist) = SingleSteering::compute_twist(&point, &vel);

            log!("valid: {}, twist: {:?}", valid, twist);
        }
    }

    #[test]
    fn test_update_odom()
    {
        let mut robot_pose = Transform2d::new(&[0.0, 0.0, 0.0]);
        let mut robot_pose2 = Transform2d::new(&[0.0, 0.0, 0.0]);
        robot_pose = robot_pose.multiply(&robot_pose2);

        bench!(
            {
                for _ in 0..1000 {
                    robot_pose = robot_pose.multiply(&robot_pose2);
                }
            },
            5
        );

        bench!(
            {
                for _ in 0..1000 {
                    robot_pose.multiply_inplace(&robot_pose2);
                }
            },
            5
        );

        let mut robot_pose = Transform2d::new(&[0.0, 0.0, 0.0]);
        let mut robot_state = [0.0, 0.0, 0.0];

        let mut robot_feed_back = [1.0, 0.0, 0.0];

        bench!(
            {
                update_odometry(&mut robot_pose, 1.0005, &TwistState::from(&robot_feed_back));
            },
            5
        );
        let mut robot_pose = Transform2d::new(&[0.0, 0.0, 0.0]);

        update_odometry(&mut robot_pose, 1.0005, &TwistState::from(&robot_feed_back));

        println!("robot_pose: {}", robot_pose);
    }

    #[test]
    fn test_rotate()
    {
        //
        let target_angle = 1.56;

        let robot = OdometryCalculator::new();

        let mut current_angle = robot.odom.borrow().yaw();

        let wheel_mount_pos: [f64; 2] = [1.5, 0.0];
        let wheel_mount_radius = (wheel_mount_pos[0] * wheel_mount_pos[0]
            + wheel_mount_pos[1] * wheel_mount_pos[1])
            .sqrt();

        let direction = 1.0;
        {
            current_angle = robot.odom.borrow().yaw();
        }
    }

    #[test]
    fn test_random()
    {
        {
            let mut rng = rand::thread_rng();
            let mut cmd: [f64; 3] = [0.0, 0.0, 0.0];
            cmd[0] = rng.gen_range(-0.5..0.5);
            cmd[2] = rng.gen_range(-0.5..0.5);
            bench!(
                {
                    let center = find_rotate_center(&TwistState::from(&cmd));
                },
                1
            );

            //===================

            bench!(
                {
                    let mut cmd: [f64; 3] = [0.0, 0.0, 0.0];

                    cmd[0] = rng.gen_range(-0.5..0.5);

                    cmd[2] = rng.gen_range(-0.5..0.5);
                    let mut point = [0.5, 0.0];
                    point[0] = rng.gen_range(0.1..0.5);
                    point[1] = rng.gen_range(-0.5..0.5);
                    let mut center = find_rotate_center(&TwistState::from(&cmd));
                    center.1[0] = 0.0;
                    //===============
                    //===============

                    let (valid, vel) = compute_driver_cmd_use_rotate_center(
                        &TwistState::from(&cmd),
                        &center,
                        &point,
                    );
                    // println!("cmd:{:?}, vel:{:?}",cmd, vel);

                    let (valid, twist) = SingleSteering::compute_twist(&point, &vel);

                    let twist = [twist.forward_vel, twist.forward_angle, twist.rotate_vel];

                    // println!("cmd:{:?}, twist:{:?}",cmd, twist);
                    for i in 0..3 {
                        assert!((cmd[i] - twist[i]).abs() < 0.001);
                    }

                    // let vel = [-vel[0], vel[1] + PI];
                    // let (valid, twist) = driver_to_twist_single_steering(&point, &vel);
                    //
                    //
                    // println!("cmd:{:?}, twist:{:?}",cmd, twist);
                    // for i in 0..3 {
                    //     assert!((cmd[i] - twist[i]).abs() < 0.001);
                    // }
                },
                5
            );
        }
    }
}

// filter feedback data
pub struct Driver
{
    command: f64,
    feedback: f64,
    stamp: SystemTime,
    counter: u64,
}

enum DriverState
{
    Uninitialized,
}

impl Driver
{
    pub fn new() -> Self
    {
        Self {
            command: 0.0,
            feedback: 0.0,
            stamp: SystemTime::now(),
            counter: 0,
        }
    }

    pub fn set_feedback(&mut self, feedback: f64)
    {
        self.stamp = SystemTime::now();
        self.feedback = feedback;
        self.counter += 1;
    }
    pub fn set_feedback_stamped(&mut self, feedback: f64, stamp: SystemTime)
    {
        self.stamp = stamp;
        self.feedback = feedback;
        self.counter += 1;
    }
    pub fn set_command(&mut self, set_value: f64)
    {
        self.command = set_value;
    }

    pub fn get_command(&self) -> f64
    {
        self.command
    }

    pub fn get_feedback(&self) -> f64
    {
        self.feedback
    }
    pub fn is_valid(&self) -> bool
    {
        self.counter > 10
    }
}

pub struct TractionWheel
{
    traction: Driver,
    mount_pose: [f64; 3],
}

impl TractionWheel
{
    pub fn new(mount_pose: &[f64; 3]) -> Self
    {
        Self {
            traction: Driver::new(),
            mount_pose: *mount_pose,
        }
    }
}

pub struct SteeringWheel
{
    traction: Driver,
    steering: Driver,
    mount_pose: [f64; 3],
}
impl SteeringWheel
{
    pub fn new(mount_pose: &[f64; 3]) -> Self
    {
        Self {
            traction: Driver::new(),
            steering: Driver::new(),
            mount_pose: *mount_pose,
        }
    }

    pub fn set_feedback_stamped(
        &mut self,
        traction_vel: f64,
        steering_angle: f64,
        stamp: SystemTime,
    )
    {
        // let now = SystemTime::now();
        self.traction.set_feedback_stamped(traction_vel, stamp);
        self.steering.set_feedback_stamped(steering_angle, stamp);
    }
}
