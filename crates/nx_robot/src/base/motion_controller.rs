use crate::base::motion_planner::RobotMotionPlannerOption;
use crate::base::odometry::SingleSteering;
use crate::base::{compute_rotate_cmd, twist_to_driver_cmd, TwistState};
use nx_common::{angle_norm, log, vector_2d_norm2};
use serde::Deserialize;
use std::cell::RefCell;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::os::linux::raw::stat;
use std::rc::Rc;
use tracing::{info, warn};

#[derive(Debug, Deserialize, Clone)]
pub struct RobotMotionControllerOption
{
    rotate_predict_vel: f64,
    rotate_predict_dist: f64,

    inplace_rotate_start_diff: f64,
    // forward tolerance , mm
    inplace_rotate_stop_tolerance: f64,
    inplace_rotate_min_forward_vel: f64,
    inplace_rotate_max_forward_vel: f64,
    inplace_rotate_forward_up_acc: f64,
    inplace_rotate_forward_dw_acc: f64,
    inplace_max_stop_vel_follow_error: f64,

    follow_path_max_forward_vel: f64,
    follow_path_max_forward_up_acc: f64,
    follow_path_max_forward_dw_acc: f64,
}
#[derive(Debug, Copy, Clone, PartialEq)]

pub struct WheelState
{
    pub forward_vel: f64,
    pub rotate_angle: f64,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ControllerCommand
{
    None,
    // forward_vel, forward_angle, rotate_vel,
    Twist(TwistState),
    SingleDifferential([f64; 2]),
    DoubleDifferential([f64; 4]),
    // forward_vel, rotate_angle
    SingleSteering(WheelState),
    DoubleSteering([f64; 4]),
}

#[derive(PartialEq)]
pub enum ControllerMotionDirection
{
    Forward,
    Rotate,
    Backward,
}

#[derive(Debug, Deserialize, Clone)]
pub struct RobotMotionControllerConfig
{
    model: String,
    default_option: String,
    options: HashMap<String, RobotMotionControllerOption>,
    //SingleSteering: [[x,y]]
    pub wheel_mount_pos: Vec<[f64; 2]>,
    //SingleSteering:  [[forward_vel_range, forward_vel_acc],[rotate_angle_range, rotate_angle_vel]]
    pub wheel_control_limit: Vec<[f64; 2]>,
    use_hardware_controller: bool,
    pub control_interval_s: f64,
    pub rotate_decay_diff: f64,
    pub rotate_decay_vel: f64,

    //[vx,vy]
    pub wheel_vel_limit: [f64; 2],
}

pub trait RobotControl
{
    fn config(&self) -> &RobotMotionControllerConfig;

    fn check_model_name(&self, model: &str) -> bool;
    fn is_stopped(&self) -> bool;
    fn is_prepared(&self) -> bool;

    fn stop(&self);

    fn reset(&self);

    fn set_motion_direction(&self, direction: ControllerMotionDirection);

    //
    fn update_wheel_state(&self, state: &[f64]);
    fn update_wheel_state_by_odom(&self, state: &TwistState);

    fn update_base_pose(&self, pose: &[f64; 3]);
    fn record_base_pose(&self, pose: &[f64; 3]);

    fn sync_wheel(&self);

    fn rotate(&self, target: f64) -> bool;

    fn follow(&self, target: &[f64; 3], max_vel: f64) -> bool;

    fn follow_with_control(
        &self,
        target: &[f64; 3],
        max_vel: f64,
        control_point: &[f64; 3],
    ) -> bool;

    fn control(
        &self,
        control_point: &[f64; 3],
        target_pose: &[f64; 3],
        allow_forward_vel: f64,
    ) -> bool;

    fn set_init_cmd(&self, cmd: ControllerCommand);

    fn get_cmd(&self) -> ControllerCommand;

    fn smooth_command(&self, twist: &TwistState, max_forward_vel: f64);

    // get

    fn get_driver_command(&self, control_point: &[f64; 3]) -> [f64; 2];
}
pub struct SingleSteeringMotionController
{
    pub config: RobotMotionControllerConfig,
    pub select_option: Rc<RefCell<Option<RobotMotionControllerOption>>>,

    //wheel_state
    pub wheel_state: Rc<RefCell<WheelState>>,
    current_twist: Rc<RefCell<TwistState>>,
    base_pose: Rc<RefCell<[f64; 3]>>,
    record_base_pose: Rc<RefCell<[f64; 3]>>,

    wheel_need_sync: Rc<RefCell<bool>>,
    wheel_mount_radius: f64,
    command: Rc<RefCell<ControllerCommand>>,
    pub driver_command: Rc<RefCell<WheelState>>,

    motion_direction: Rc<RefCell<ControllerMotionDirection>>,
}

impl SingleSteeringMotionController
{
    // const SINGLE_DIFF_MODEL: &'static str = "SingleDiff";
    // const DOUBLE_DIFF_MODEL: &'static str = "DoubleDiff";
    const MODEL_NAME: &'static str = "SingleSteering";
    // const DOUBLE_STEERING_MODEL: &'static str = "DoubleSteering";
    const ZERO_SPEED_EPSILON: f64 = 1e-4;

    pub fn new(mut config: RobotMotionControllerConfig) -> Self
    {
        #[cfg(aaa)]
        {
            match config.model.as_str() {
                Self::SINGLE_DIFF_MODEL => {
                    assert!(config.wheel_mount_pos.len() == 1);
                }
                Self::DOUBLE_DIFF_MODEL => {
                    assert!(config.wheel_mount_pos.len() == 1);
                }
                Self::SINGLE_STEERING_MODEL => {
                    assert!(config.wheel_mount_pos.len() == 1);
                }
                Self::DOUBLE_STEERING_MODEL => {
                    assert!(config.wheel_mount_pos.len() == 1);
                }
                &_ => {}
            }
        }

        assert_eq!(config.wheel_mount_pos.len(), 1);
        assert_eq!(config.wheel_control_limit.len(), 2);

        assert_eq!(config.model, Self::MODEL_NAME);

        let wheel_mount_radius = (config.wheel_mount_pos[0][0] * config.wheel_mount_pos[0][0]
            + config.wheel_mount_pos[0][1] * config.wheel_mount_pos[0][1])
            .sqrt();

        Self {
            config,
            select_option: Rc::new(RefCell::new(None)),
            wheel_state: Rc::new(RefCell::new(WheelState {
                forward_vel: 0.0,
                rotate_angle: 0.0,
            })),
            current_twist: Rc::new(RefCell::new(TwistState {
                forward_vel: 0.0,
                forward_angle: 0.0,
                rotate_vel: 0.0,
            })),
            base_pose: Rc::new(RefCell::new([0.0; 3])),
            record_base_pose: Rc::new(RefCell::new([0.0; 3])),
            wheel_need_sync: Rc::new(RefCell::new(true)),
            wheel_mount_radius,
            command: Rc::new(RefCell::new(ControllerCommand::None)),
            driver_command: Rc::new(RefCell::new(WheelState {
                forward_vel: 0.0,
                rotate_angle: 0.0,
            })),
            motion_direction: Rc::new(RefCell::new(ControllerMotionDirection::Forward)),
        }
    }

    pub fn set_option(&self, option_name: &str) -> bool
    {
        self.config.options.contains_key(option_name);

        let c = &self.config;
        let option_name = if c.options.contains_key(option_name) {
            option_name
        } else {
            log!(
                "option_name [{}] doesn't exist, use default [{}]",
                option_name,
                c.default_option
            );
            &c.default_option
        };
        let binding = c.options.get(option_name);
        let opt = match &binding {
            Some(t) => t,
            None => {
                log!("option_name [{}] doesn't exist", option_name);
                return false;
            }
        };
        *self.select_option.borrow_mut() = Some((*opt).clone());

        true
    }
}

impl RobotControl for SingleSteeringMotionController
{
    fn config(&self) -> &RobotMotionControllerConfig
    {
        &self.config
    }

    fn check_model_name(&self, model: &str) -> bool
    {
        self.config.model == model
    }

    fn is_stopped(&self) -> bool
    {
        self.wheel_state.borrow_mut().forward_vel < 1e-4
    }

    fn is_prepared(&self) -> bool
    {
        true
    }

    fn stop(&self)
    {
        self.smooth_command(
            &TwistState {
                forward_vel: 0.0,
                forward_angle: 0.0,
                rotate_vel: 0.0,
            },
            0.0,
        );
    }

    fn reset(&self)
    {
        self.wheel_state.borrow_mut().rotate_angle = 0.0;
        self.wheel_state.borrow_mut().forward_vel = 0.0;

        self.current_twist.borrow_mut().forward_vel = 0.0;
        self.current_twist.borrow_mut().forward_angle = 0.0;
        self.current_twist.borrow_mut().rotate_vel = 0.0;
        *self.command.borrow_mut() = ControllerCommand::None;
        *self.wheel_need_sync.borrow_mut() = true;
    }

    fn set_motion_direction(&self, direction: ControllerMotionDirection)
    {
        *self.motion_direction.borrow_mut() = direction;
    }

    fn update_wheel_state(&self, state: &[f64])
    {
        assert!(self.config.use_hardware_controller);

        self.wheel_state.borrow_mut().forward_vel = state[0];
        self.wheel_state.borrow_mut().rotate_angle = state[1];

        let (valid, twist) =
            SingleSteering::compute_twist(&self.config.wheel_mount_pos[0], &[state[0], state[1]]);

        if valid {
            *self.current_twist.borrow_mut() = twist;
        }
    }

    fn update_wheel_state_by_odom(&self, twist: &TwistState)
    {
        assert!(!self.config.use_hardware_controller);

        let (valid, mut state) = twist_to_driver_cmd(twist, &self.config.wheel_mount_pos[0]);

        if (*self.motion_direction.borrow() == ControllerMotionDirection::Forward
            && state[0] < -1e-6)
            || (*self.motion_direction.borrow() == ControllerMotionDirection::Backward
                && state[0] > 1e-6)
        {
            state[0] = -state[0];
            state[1] = angle_norm!(state[1] + PI);
        }

        // self.update_wheel_state(&state);
        self.wheel_state.borrow_mut().forward_vel = state[0];
        if state[0].abs() > 1e-6 {
            self.wheel_state.borrow_mut().rotate_angle = state[1];
        }

        self.current_twist.borrow_mut().forward_vel = twist.forward_vel;
        self.current_twist.borrow_mut().forward_angle = twist.forward_angle;
        self.current_twist.borrow_mut().rotate_vel = twist.rotate_vel;

        // log!("update_wheel_state_by_odom: state: {:?}", state);
        // log!("update_wheel_state_by_odom: twist: {:?}", twist);
    }

    fn update_base_pose(&self, pose: &[f64; 3])
    {
        *self.base_pose.borrow_mut() = *pose;
    }

    // call once at each movement
    // such as rotate, follow, follow_with_control
    fn record_base_pose(&self, pose: &[f64; 3])
    {
        *self.record_base_pose.borrow_mut() = *pose;
    }

    fn sync_wheel(&self)
    {
        *self.wheel_need_sync.borrow_mut() = true;
    }

    // driver rotate_angle is useless, only use forward_vel.abs()
    fn rotate(&self, target: f64) -> bool
    {
        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                log!("select_option doesn't exist");
                return false;
            }
        };
        let mut twist = TwistState::from(&[0.0, 0.0, 0.0]);

        let target_angle = target;

        let current_angle = self.base_pose.borrow()[2];
        let direction = angle_norm!(target - self.record_base_pose.borrow()[2]);
        let angle_diff = angle_norm!(target - current_angle);

        log!("target_angle: {}, current_angle: {}, direction: {}, angle_diff: {}, inplace_rotate_start_diff: {}",target_angle,current_angle,   direction, angle_diff, opt.inplace_rotate_start_diff);

        #[cfg(aass)]
        {
            if (direction.abs() < opt.inplace_rotate_start_diff) {
                self.smooth_command(&twist, 0.0);
                return true;
            }
        }

        let step_time_s = self.config.control_interval_s;
        let forward_up_acc = opt.inplace_rotate_forward_up_acc;
        let forward_dw_acc = opt.inplace_rotate_forward_dw_acc;

        let current_angle = self.base_pose.borrow()[2];
        let radius = self.wheel_mount_radius;
        let forward_tolerance = opt.inplace_rotate_stop_tolerance;
        let min_forward_vel = opt.inplace_rotate_min_forward_vel;
        let max_forward_vel = opt.inplace_rotate_max_forward_vel;

        let [forward_limit_vel, forward_limit_acc] = self.config.wheel_control_limit[0];
        let [rotate_limit_angle, rotate_limit_vel] = self.config.wheel_control_limit[1];

        let max_stop_vel_follow_error = opt.inplace_max_stop_vel_follow_error;

        let current_forward_vel = self.wheel_state.borrow_mut().forward_vel;

        let angle_diff = angle_norm!(target_angle - current_angle);

        let (finished, forward_vel) = compute_rotate_cmd(
            current_forward_vel,
            step_time_s,
            forward_up_acc,
            forward_dw_acc,
            angle_diff,
            direction,
            radius,
            forward_tolerance,
            min_forward_vel,
            max_forward_vel,
            max_stop_vel_follow_error,
        );

        let mut rotate_vel = if (!finished) {
            forward_vel / radius
        } else {
            0.0
        };

        twist.rotate_vel = rotate_vel;

        // twist to driver command
        let mut max_forward_vel = opt.inplace_rotate_max_forward_vel;
        let mut up_forward_vel = current_forward_vel.abs() + step_time_s * forward_up_acc;
        max_forward_vel = max_forward_vel.min(up_forward_vel);
        max_forward_vel = max_forward_vel.min(forward_limit_vel);
        max_forward_vel = max_forward_vel.min(forward_vel.abs());
        log!(
            "finished: {}, rotate_vel: {}, forward_vel: {}, max_forward_vel:{}",
            finished,
            rotate_vel,
            forward_vel,
            max_forward_vel
        );

        self.smooth_command(&twist, max_forward_vel);

        finished
    }

    fn follow(&self, target: &[f64; 3], max_vel: f64) -> bool
    {
        todo!()
    }

    fn follow_with_control(&self, target: &[f64; 3], max_vel: f64, control_point: &[f64; 3])
        -> bool
    {
        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                log!("select_option doesn't exist");
                return false;
            }
        };

        let step_time_s = self.config.control_interval_s;
        let forward_up_acc = opt.inplace_rotate_forward_up_acc;
        let forward_dw_acc = opt.inplace_rotate_forward_dw_acc;
        let mut max_forward_vel = opt.follow_path_max_forward_vel;

        let current_forward_vel = self.wheel_state.borrow_mut().forward_vel;
        let current_rotate_angle = self.wheel_state.borrow_mut().rotate_angle;

        // let mut  forward_vel_abs = max_vel.abs();
        let up_forward_vel = current_forward_vel.abs() + step_time_s * forward_up_acc;

        info!(current_forward_vel, step_time_s, forward_up_acc);
        // forward_vel_abs = forward_vel_abs.min(max_forward_vel);
        // forward_vel_abs = forward_vel_abs.min(up_forward_vel);

        let mut driver_command_rotate_angle = target[1].atan2(target[0]);

        let [forward_limit_vel, forward_limit_acc] = self.config.wheel_control_limit[0];
        let [rotate_limit_angle, rotate_limit_vel] = self.config.wheel_control_limit[1];

        let mut state = [max_vel.abs(), driver_command_rotate_angle];

        // log!(
        //     "follow_with_control control_point: {:?},state : {:?}",
        //     control_point,
        //     state
        // );

        // aux driver command to twist
        let (valid, twist) = SingleSteering::compute_twist(
            &[control_point[0], control_point[1]],
            &[state[0], state[1]],
        );

        info!(
            "raw twist: {:?}, max_forward_vel:{}",
            twist, max_forward_vel
        );
        // twist to driver command
        // let (valid, mut state) = twist_to_driver_cmd(&twist, &self.config.wheel_mount_pos[0]);
        info!(max_vel, max_forward_vel, up_forward_vel, forward_limit_vel);

        max_forward_vel = max_forward_vel.min(up_forward_vel);
        max_forward_vel = max_forward_vel.min(forward_limit_vel);

        info!(
            "before smooth twist: {:?}, max_forward_vel: {}",
            twist, max_forward_vel
        );

        self.smooth_command(&twist, max_forward_vel);

        // convert to twist

        // if (*self.motion_direction.borrow() == ControllerMotionDirection::Forward
        //     && state[0] < -1e-3)
        //     || (*self.motion_direction.borrow() == ControllerMotionDirection::Backward
        //         && state[0] > 1e-3)
        // {
        //     state[0] = -state[0];
        //     state[1] = angle_norm!(state[1] + PI);
        // }
        //
        // // t = angle_diff/rot_vel
        // // forward_vel = dist/t
        // // forward_vel = dist * rot_vel/ angle_diff
        // let angle_diff = (angle_norm!(state[1] - current_rotate_angle)).abs();
        // let mut rotate_limit_forward_vel: f64 = forward_vel_abs;
        // let mut predict_dist = (vector_2d_norm2!(target)).sqrt();
        //
        // if angle_diff > 0.001 {
        //     rotate_limit_forward_vel = opt.rotate_predict_dist * opt.rotate_predict_vel / angle_diff;
        //     // rotate_limit_forward_vel = predict_dist * opt.rotate_predict_vel / angle_diff;
        // }
        // //
        // forward_vel_abs = forward_vel_abs.min(rotate_limit_forward_vel);
        // state[0] = forward_vel_abs.copysign(state[0]);
        //
        // if self.config.use_hardware_controller {
        //     *self.command.borrow_mut() = ControllerCommand::SingleSteering(WheelState { forward_vel:state[0], rotate_angle: state[1] });
        //
        // } else {
        //     let (valid, twist) = SingleSteering::compute_twist(
        //         &self.config.wheel_mount_pos[0],
        //         &[state[0], state[1]],
        //     );
        //     *self.command.borrow_mut() = ControllerCommand::Twist(twist);
        // }

        true
    }

    fn control(
        &self,
        control_point: &[f64; 3],
        target_pose: &[f64; 3],
        allow_forward_vel: f64,
    ) -> bool
    {
        todo!()
    }

    fn set_init_cmd(&self, cmd: ControllerCommand)
    {
        todo!()
    }

    fn get_cmd(&self) -> ControllerCommand
    {
        if *self.command.borrow() == ControllerCommand::None {
            self.stop();
        }
        self.command.borrow().clone()
    }

    fn smooth_command(&self, twist: &TwistState, max_forward_vel: f64)
    {
        let current_forward_vel = self.wheel_state.borrow_mut().forward_vel;
        let current_rotate_angle = self.wheel_state.borrow_mut().rotate_angle;

        let mut driver_cmd: [f64; 2] = [0.0, 0.0];
        if max_forward_vel >= Self::ZERO_SPEED_EPSILON {
            let (valid, mut state) = twist_to_driver_cmd(&twist, &self.config.wheel_mount_pos[0]);

            info!(?state);
            let mut forward_vel_abs = state[0].abs();

            forward_vel_abs = forward_vel_abs.min(max_forward_vel);

            state[0] = forward_vel_abs.copysign(state[0]);
            // log!("normal speed with state:{:?}", state);
            info!(?state);

            driver_cmd = state;
        } else {
            // log!(
            //     "zero speed with current_rotate_angle:{}",
            //     current_rotate_angle
            // );
            driver_cmd = [0.0, current_rotate_angle];
            if self.config.use_hardware_controller {
                let wheel_state = WheelState {
                    forward_vel: driver_cmd[0],
                    rotate_angle: driver_cmd[1],
                };
                *self.command.borrow_mut() = ControllerCommand::SingleSteering(wheel_state);
            } else {
                *self.command.borrow_mut() = ControllerCommand::Twist(TwistState {
                    forward_vel: 0.0,
                    forward_angle: 0.0,
                    rotate_vel: 0.0,
                });
            }
            *self.driver_command.borrow_mut() = WheelState {
                forward_vel: driver_cmd[0],
                rotate_angle: driver_cmd[1],
            };
            return;
        }

        // info!(?twist, ?driver_cmd);
        *self.driver_command.borrow_mut() = WheelState {
            forward_vel: driver_cmd[0],
            rotate_angle: driver_cmd[1],
        };

        if (*self.motion_direction.borrow() == ControllerMotionDirection::Forward
            && driver_cmd[0] < 0.0)
            || (*self.motion_direction.borrow() == ControllerMotionDirection::Backward
                && driver_cmd[0] > 0.0)
        {
            driver_cmd[0] = -driver_cmd[0];
            driver_cmd[1] = angle_norm!(driver_cmd[1] + PI);
        }
        info!(?twist, ?driver_cmd);

        let current_rotate_angle_diff = angle_norm!(current_rotate_angle - driver_cmd[1]).abs();

        // set forward_vel with current_rotate_angle_diff
        let max_rot = self.config.rotate_decay_diff;

        // todo: rotate motor
        if current_rotate_angle_diff > max_rot {
            let mut driver_cmd_vel = driver_cmd[0].abs();

            // let t0 = current_rotate_angle_diff / self.config.wheel_control_limit[1][1];
            // let t1 = 10.0* self.config.control_interval_s;
            // let ds:f64 = driver_cmd_vel*t1;

            // let v0 = ds/t0;

            let v0 = self.config.rotate_decay_vel;

            driver_cmd_vel = driver_cmd_vel.min(v0);

            // warn!(driver_cmd_vel, v0,t0,t1,ds,current_rotate_angle_diff,max_rot);

            driver_cmd[0] = driver_cmd_vel.copysign(driver_cmd[0]);
        }
        {
            let mut driver_cmd_vel = driver_cmd[0].abs();

            let vy = driver_cmd_vel * driver_cmd[1].sin().abs();
            let vx = driver_cmd_vel * driver_cmd[1].cos().abs();

            let r1 = (self.config.wheel_vel_limit[1] / vy);
            let r2 = (self.config.wheel_vel_limit[0] / vx);
            let r = r1.min(r2).min(1.0);
            driver_cmd_vel = driver_cmd_vel * r;
            driver_cmd[0] = driver_cmd_vel.copysign(driver_cmd[0]);
        }

        if self.config.use_hardware_controller {
            // match self.config.model.as_str() {
            //     Self::MODEL_NAME => {
            //
            //
            //     },
            //     _ =>{}
            // }

            #[cfg(aaa)]
            {
                if (*self.motion_direction.borrow() == ControllerMotionDirection::Forward
                    && driver_cmd[0] < -Self::ZERO_SPEED_EPSILON)
                    || (*self.motion_direction.borrow() == ControllerMotionDirection::Backward
                        && driver_cmd[0] > Self::ZERO_SPEED_EPSILON)
                {
                    driver_cmd[0] = -driver_cmd[0];
                    driver_cmd[1] = angle_norm!(driver_cmd[1] + PI);
                }
            }

            let mut wheel_need_sync = *self.wheel_need_sync.borrow();

            if wheel_need_sync {
                // log!("driver_cmd: {:?}, current_rotate_angle: {}",driver_cmd,current_rotate_angle);

                if current_rotate_angle_diff > 0.001 {
                    driver_cmd[0] = 0.0;
                } else {
                    wheel_need_sync = false;
                }
            }
            *self.wheel_need_sync.borrow_mut() = wheel_need_sync;

            let wheel_state = WheelState {
                forward_vel: driver_cmd[0],
                rotate_angle: driver_cmd[1],
            };

            *self.command.borrow_mut() = ControllerCommand::SingleSteering(wheel_state);
        } else {
            let (valid, mut twist) = SingleSteering::compute_twist(
                &self.config.wheel_mount_pos[0],
                &[driver_cmd[0], driver_cmd[1]],
            );
            *self.command.borrow_mut() = ControllerCommand::Twist(twist);
        }
    }

    fn get_driver_command(&self, control_point: &[f64; 3]) -> [f64; 2]
    {
        let (valid, mut driver_cmd) = twist_to_driver_cmd(
            &self.current_twist.borrow(),
            &[control_point[0], control_point[1]],
        );
        if (*self.motion_direction.borrow() == ControllerMotionDirection::Forward
            && driver_cmd[0] < 0.0)
            || (*self.motion_direction.borrow() == ControllerMotionDirection::Backward
                && driver_cmd[0] > 0.0)
        {
            driver_cmd[0] = -driver_cmd[0];
            driver_cmd[1] = angle_norm!(driver_cmd[1] + PI);
        }
        driver_cmd
    }
}
