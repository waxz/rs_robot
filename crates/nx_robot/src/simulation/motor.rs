use std::cmp::max;
use std::time::SystemTime;

use std::cell::RefCell;
use std::rc::Rc;
use PartialOrd;

pub struct SpeedMotor
{
    max_acc: f64,
    max_vel: f64,
    stamp: Rc<RefCell<SystemTime>>,
    actual_vel: Rc<RefCell<f64>>,
    set_vel: Rc<RefCell<f64>>,
}

impl SpeedMotor
{
    pub fn new(max_acc: f64, max_vel: f64) -> Self
    {
        Self {
            max_acc,
            max_vel,
            stamp: Rc::new(RefCell::new(SystemTime::now())),
            actual_vel: Rc::new(RefCell::new(0.0)),
            set_vel: Rc::new(RefCell::new(0.0)),
        }
    }

    pub fn config(&mut self, max_acc: f64, max_vel: f64)
    {
        self.max_acc = max_acc;
        self.max_vel = max_vel;
    }
    pub fn set_vel(&self, set_vel: f64)
    {
        *self.set_vel.borrow_mut() = set_vel;
    }
    pub fn actual_vel(&self) -> f64
    {
        *self.actual_vel.borrow()
    }

    pub fn update(&self)
    {
        let vel_change = *self.set_vel.borrow() - *self.actual_vel.borrow();

        // let max_acc = self.max_acc.copysign(vel_change);

        let interval_s = self.stamp.borrow().elapsed().unwrap().as_micros() as f64 * 1e-6;
        *self.stamp.borrow_mut() = SystemTime::now();

        let mut need_acc = vel_change.abs() / interval_s;

        need_acc = need_acc.min(self.max_acc).copysign(vel_change);
        // println!("vel_change: {} m/s, interval_s: {} s, need_acc : {} m2/s", vel_change, interval_s,need_acc);
        let mut actual_vel = *self.actual_vel.borrow();

        actual_vel += need_acc * interval_s;
        actual_vel = actual_vel.min(self.max_vel).max(-self.max_vel);
        *self.actual_vel.borrow_mut() = actual_vel;
        // let interval =  now.elapsed() - self.stamp.elapsed();
        // self.stamp = now;
        // println!("self.set_vel: {} m/s, self.actual_vel: {} m/s, need_acc : {} m2/s", self.set_vel, self.actual_vel,need_acc);
    }
}

// unit: m or rad
pub struct PositionMotor
{
    max_vel: f64,
    max_pos: f64,
    stamp: Rc<RefCell<SystemTime>>,
    actual_pos: Rc<RefCell<f64>>,
    set_pos: Rc<RefCell<f64>>,
}

impl PositionMotor
{
    pub fn new(max_vel: f64, max_pos: f64) -> Self
    {
        Self {
            max_vel,
            max_pos,
            stamp: Rc::new(RefCell::new(SystemTime::now())),
            actual_pos: Rc::new(RefCell::new(0.0)),
            set_pos: Rc::new(RefCell::new(0.0)),
        }
    }

    pub fn config(&mut self, max_vel: f64)
    {
        self.max_vel = max_vel;
    }
    pub fn set_pos(&self, set_pos: f64)
    {
        *self.set_pos.borrow_mut() = set_pos;
    }

    pub fn actual_pos(&self) -> f64
    {
        *self.actual_pos.borrow()
    }

    pub fn update(&self)
    {
        let pos_change = *self.set_pos.borrow() - *self.actual_pos.borrow();

        // let max_acc = self.max_acc.copysign(vel_change);

        let interval_s = self.stamp.borrow().elapsed().unwrap().as_micros() as f64 * 1e-6;
        *self.stamp.borrow_mut() = SystemTime::now();
        let mut need_vel = pos_change.abs() / interval_s;

        need_vel = need_vel.min(self.max_vel).copysign(pos_change);
        // println!("pos_change: {} m, interval_s: {} s, need_vel : {} m/s", pos_change, interval_s,need_vel);
        let mut actual_pos = *self.actual_pos.borrow();

        actual_pos += need_vel * interval_s;

        actual_pos = actual_pos.min(self.max_pos).max(-self.max_pos);

        *self.actual_pos.borrow_mut() = actual_pos;

        // let interval =  now.elapsed() - self.stamp.elapsed();
        // self.stamp = now;
        // println!("self.set_pos: {} m, self.actual_pos: {} m, need_vel : {} m/s", self.set_pos, self.actual_pos,need_vel);
    }
}

#[cfg(test)]
mod test
{
    use crate::simulation::motor::{PositionMotor, SpeedMotor};
    use std::time::Duration;

    #[test]
    fn test()
    {
        let mut speed_motor = SpeedMotor::new(1.0, 3.0);
        let mut rotate_motor = PositionMotor::new(3.0, 2.0);

        speed_motor.set_vel(1.5);
        rotate_motor.set_pos(1.9);

        for i in 1..200 {
            std::thread::sleep(Duration::from_millis(10));

            speed_motor.update();
            rotate_motor.update();
        }
        speed_motor.set_vel(-1.5);
        rotate_motor.set_pos(-1.9);

        for i in 1..400 {
            std::thread::sleep(Duration::from_millis(10));

            speed_motor.update();
            rotate_motor.update();
        }
    }
}
