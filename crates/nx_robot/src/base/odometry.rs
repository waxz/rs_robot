use crate::base::TwistState;
use nx_common::common::math::line_line_intersect;
use nx_common::common::transform2d::Transform2d;
use nx_common::{vector_2d_dist2, vector_2d_norm2};
use std::cell::RefCell;
use std::f64::consts::FRAC_PI_2;
use std::ops::Deref;
use std::rc::Rc;

pub fn update_odometry(last_odom: &mut Transform2d, mut step_time: f64, feed_back: &TwistState)
{
    let TwistState {
        forward_vel,
        forward_angle,
        rotate_vel,
    } = feed_back;

    let is_forward = forward_vel.abs() > 0.001;
    let is_rotate = rotate_vel.abs() > 0.001;

    if (!(is_forward || is_rotate)) {
        return;
    }

    let mut last_odom_x = last_odom.x();
    let mut last_odom_y = last_odom.y();
    let mut last_odom_yaw = last_odom.yaw();

    if (is_forward) {
        let base_forward_vel_x = forward_vel * forward_angle.cos();
        let base_forward_vel_y = forward_vel * forward_angle.sin();

        let mut relative_x = 0.0;
        let mut relative_y = 0.0;
        let mut relative_yaw = 0.0;

        let mut update_step: f64 = 0.001;

        update_step = update_step.min(step_time);

        let mut relative_pose = Transform2d::new(&[
            base_forward_vel_x * update_step,
            base_forward_vel_y * update_step,
            rotate_vel * update_step,
        ]);
        while step_time > update_step {
            *last_odom = last_odom.multiply(&relative_pose);
            step_time -= update_step;
        }

        update_step = step_time;
        relative_pose.set(
            base_forward_vel_x * update_step,
            base_forward_vel_y * update_step,
            rotate_vel * update_step,
        );
        *last_odom = last_odom.multiply(&relative_pose);
    } else {
        last_odom_yaw += step_time * rotate_vel;
        last_odom.set(last_odom_x, last_odom_y, last_odom_yaw);
    }
}

pub struct OdometryCalculator
{
    pub odom: Rc<RefCell<Transform2d>>,
    stamp: Rc<RefCell<std::time::Instant>>,
    counter: Rc<RefCell<u64>>,
    pub current_twist: Rc<RefCell<TwistState>>,
}

impl OdometryCalculator
{
    pub fn new() -> Self
    {
        Self {
            odom: Rc::new(RefCell::new(Default::default())),
            stamp: Rc::new(RefCell::new(std::time::Instant::now())),
            counter: Rc::new(RefCell::new(0)),
            current_twist: Rc::new(RefCell::new(TwistState {
                forward_vel: 0.0,
                forward_angle: 0.0,
                rotate_vel: 0.0,
            })),
        }
    }
    pub fn set_odom(&self, odom: &Transform2d)
    {
        *self.odom.borrow_mut() = *odom;
        *self.counter.borrow_mut() = 0;
    }
    pub fn compute_odom(&self, twist: &TwistState)
    {
        self.current_twist.borrow_mut().forward_vel = twist.forward_vel;
        self.current_twist.borrow_mut().forward_angle = twist.forward_angle;
        self.current_twist.borrow_mut().rotate_vel = twist.rotate_vel;

        *self.counter.borrow_mut() += 1;

        let step_time_us = self.stamp.borrow().elapsed().as_micros();
        *self.stamp.borrow_mut() = std::time::Instant::now();

        if *self.counter.borrow() == 1 {
            return;
        }
        let step_time_s = step_time_us as f64 * 1e-6;

        update_odometry(&mut self.odom.borrow_mut(), step_time_s, twist);
    }
}

pub struct SingleSteering {}

impl SingleSteering
{
    const MIN_DRIVER_POS_X: f64 = 1e-5;
    const MIN_RADIUS: f64 = 1e-5;

    /// given vel of each driver, compute twist
    ///
    pub fn compute_twist(driver_pos: &[f64; 2], driver_command: &[f64; 2]) -> (bool, TwistState)
    {
        let mut twist = TwistState {
            forward_vel: 0.0,
            forward_angle: 0.0,
            rotate_vel: 0.0,
        };

        let [driver_pos_x, driver_pos_y] = driver_pos;

        assert!(driver_pos_x.abs() > Self::MIN_DRIVER_POS_X);
        if (driver_pos_x.abs() < Self::MIN_DRIVER_POS_X) {
            return (false, twist);
        }

        let [driver_command_vel, driver_command_angle] = driver_command;

        let TwistState {
            forward_vel,
            forward_angle,
            rotate_vel,
        } = &mut twist;

        let driver_rotate_axis_angle = driver_command_angle + FRAC_PI_2;

        // log!("driver_pos : {:?}, driver_command : {:?}",driver_pos,driver_command);

        let mut driver_rotate_axis_x = driver_pos_x + driver_rotate_axis_angle.cos();
        // driver_rotate_axis_x = 0.0;
        let driver_rotate_axis_y = driver_pos_y + driver_rotate_axis_angle.sin();
        // log!("driver_rotate_axis_x : { }, driver_rotate_axis_y : {}",driver_rotate_axis_x,driver_rotate_axis_y);

        let rotate = line_line_intersect(
            0.0_f64,
            0.0_f64,
            0.0_f64,
            1.0_f64,
            *driver_pos_x,
            *driver_pos_y,
            driver_rotate_axis_x,
            driver_rotate_axis_y,
        );
        let (is_rotate, rotate_center) = &rotate;
        let [rotate_center_x, rotate_center_y] = rotate_center;
        // log!("rotate : {:?}", rotate);

        // log!("rotate_center : {:?}",rotate_center);
        if !is_rotate {
            // forward_angle = 0.0;
            *rotate_vel = 0.0;
            *forward_vel = *driver_command_vel * driver_command_angle.cos();
        } else {
            // let driver_dist_to_rc = ((driver_pos_x - rotate_center_x)* (driver_pos_x - rotate_center_x)+ (driver_pos_y - rotate_center_y) * (driver_pos_y - rotate_center_y)).sqrt();
            let driver_dist_to_rc = (vector_2d_dist2!(driver_pos, rotate_center)).sqrt();

            #[cfg(reject)]
            {
                assert!(driver_dist_to_rc > Self::MIN_RADIUS);
                if (driver_dist_to_rc == 0.0) {
                    return (false, twist);
                }
            }

            // let base_dist_to_rc = ((rotate_center_x) * (rotate_center_x) + (rotate_center_y) * (rotate_center_y)).sqrt();
            let base_dist_to_rc = (vector_2d_norm2!(rotate_center)).sqrt();

            let mut cross_product = 0.0_f64;
            {

                // //
                // let v = [
                //     driver_command_vel * driver_command_angle.cos(),
                //     driver_command_vel * driver_command_angle.sin(),
                // ];
                // let u = [
                //     driver_pos_x - rotate_center_x,
                //     driver_pos_y - rotate_center_y,
                // ];
                // // let dot_product = u[0] * v[0] + u[1] * v[1];
                // cross_product = u[0] * v[1] - u[1] * v[0];
            }
            {
                cross_product = (driver_pos_x - rotate_center_x)
                    * (*driver_command_vel * driver_command_angle.sin())
                    - (driver_pos_y - rotate_center_y)
                        * (*driver_command_vel * driver_command_angle.cos());
            }
            // let angle = (cross_product / ((u[0] * u[0] + u[1] * u[1]) * (v[0] * v[0] + v[1] * v[1])).sqrt()).asin();
            // log!("u: {:?}, v: {:?}, dot_product: {}, cross_product: {},angle: {}",u,v, dot_product,cross_product,angle);

            *rotate_vel = (driver_command_vel / driver_dist_to_rc).copysign(cross_product);

            {
                // let v = [1.0, 0.0];
                // let u = [-rotate_center_x, -rotate_center_y];
                // let dot_product = u[0] * v[0] + u[1] * v[1];
                // let cross_product = u[0] * v[1] - u[1] * v[0];
            }
            {
                cross_product = *rotate_center_y;
            }
            // let angle = (cross_product   / ((u[0] * u[0] + u[1] * u[1]) * (v[0] * v[0] + v[1] * v[1])).sqrt()) .asin();
            // log!("u: {:?}, v: {:?}, dot_product: {}, cross_product: {},rotate_vel: {},driver_dist_to_rc: {}",u,v,dot_product,cross_product,rotate_vel,driver_dist_to_rc);
            // let flip = rotate_vel.signum() != cross_product.signum();

            *forward_vel = (*rotate_vel * base_dist_to_rc).copysign(*rotate_vel * cross_product);
            // *forward_vel = (*rotate_vel * base_dist_to_rc).copysign(*rotate_vel );

            // log!("forward_vel: {}, rotate_vel: {}", forward_vel, rotate_vel);
        }

        (true, twist)
    }
}
