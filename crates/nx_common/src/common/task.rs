use spin_sleep::Seconds;
use std::cell::RefCell;
use std::cmp::Ordering;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use time::OffsetDateTime;

// use crate::common::loop_helper::LoopHelper;
use spin_sleep::LoopHelper;

struct Task
{
    f: Box<dyn FnMut() -> bool + 'static>,
    prio: u64,
    slot: u64,
    name: String,
    run_counter: u32,
    run_time_us: u128,
    max_run_time_us: u128,
    delay_us: u128,
    stamp: std::time::Instant,
    valid: bool,
}

// time based task scheduler
// two type of task
// 1. loss restrained, low time precision
pub struct TaskManager
{
    // vec member size should be known, use reference with lifetime annotation
    // tasks: Vec<&'a mut dyn FnMut() -> bool>,
    running_task: Rc<RefCell<Vec<Task>>>,
    prepare_task: Rc<RefCell<Vec<Task>>>,
    task_update: Rc<RefCell<bool>>,
    max_prio: u64,
    tic: Rc<RefCell<u64>>,
    task_counter: Rc<RefCell<Vec<u64>>>,
    sleeper: Rc<RefCell<LoopHelper>>,
    sleeper_ms: f32,
}

impl TaskManager
{
    pub fn new(max_prio: u64, sleeper_fps: f32, sleep_accuracy_ns: u32) -> Self
    {
        Self {
            running_task: Rc::new(RefCell::new(vec![])),
            prepare_task: Rc::new(RefCell::new(vec![])),
            task_update: Default::default(),
            max_prio,
            tic: Default::default(),
            task_counter: Rc::new(RefCell::new(vec![0; max_prio as usize + 1])),
            sleeper: Rc::new(RefCell::new(
                LoopHelper::builder()
                    .native_accuracy_ns(sleep_accuracy_ns) // report every half a second
                    .build_with_target_rate(sleeper_fps),
            )), // limit to 250 FPS if possible,
            sleeper_ms: 1000.0 / sleeper_fps,
        }
    }

    pub fn set_loop(&mut self, sleeper_fps: f32, sleep_accuracy_ns: u32)
    {
        self.sleeper_ms = 1000.0 / sleeper_fps;
        self.sleeper = Rc::new(RefCell::new(LoopHelper::builder()
            .native_accuracy_ns(sleep_accuracy_ns) // report every half a second
            .build_with_target_rate(sleeper_fps) )) // limit to 250 FPS if possible,;
        ;
    }
    pub fn add<F: FnMut() -> bool + 'static>(&self, name: &str, f: F, sleeper_ms: f32)
    {
        let mut prio = ((sleeper_ms / self.sleeper_ms).round()) as u64;
        let delay_us: u128 = (sleeper_ms * 1000.0) as u128;

        prio = prio.min(self.max_prio);
        let mut slot: u64 = 0;

        if (prio > 0) {
            slot = self.task_counter.borrow()[prio as usize] % (prio);
        } else {
            slot = 0;
        }

        self.task_counter.borrow_mut()[prio as usize] += 1;

        // println!("add task");
        self.prepare_task.borrow_mut().push(Task {
            name: name.to_string(),
            run_counter: 0,
            run_time_us: 0,
            max_run_time_us: 0,

            f: Box::new(f),
            prio,
            slot,
            delay_us,
            stamp: std::time::Instant::now(),
            valid: true,
        });

        *self.task_update.borrow_mut() = true;
    }

    pub fn sort(&self)
    {
        if (*self.task_update.borrow()) {
            self.running_task
                .borrow_mut()
                .append(&mut self.prepare_task.borrow_mut());
            self.running_task.borrow_mut().sort_by(|a, b| {
                if (a.prio == b.prio) {
                    return a.delay_us.partial_cmp(&b.delay_us).unwrap();
                } else {
                    return a.prio.partial_cmp(&b.prio).unwrap();
                }
            });
        }
        *self.task_update.borrow_mut() = false;
    }

    pub fn run(&self)
    {
        self.sleeper.borrow_mut().loop_start();
        self.sort();
        let now = std::time::Instant::now();

        let mut need_remove = false;
        for (
            i,
            Task {
                f: f,
                prio,
                slot,
                stamp,
                delay_us,
                valid,
                run_time_us,
                max_run_time_us,
                run_counter,
                ..
            },
        ) in self.running_task.borrow_mut().iter_mut().enumerate()
        {
            let toc_run = (*prio == 0) || ((*self.tic.borrow()) % (*prio) == *slot);

            let is_lazy_task = *prio == self.max_prio;
            let is_lazy_task_toc = is_lazy_task && stamp.elapsed().as_micros() > *delay_us;

            // let time_diff = stamp.elapsed().as_micros();

            if ((toc_run && !is_lazy_task)
                || (
                    is_lazy_task_toc
                    //is_lazy_task// && time_diff > *delay_us
                ))
            {
                // let mut t = std::time::Instant::now();
                *valid = f();
                // *run_time_us = now.elapsed().as_micros();
                // *max_run_time_us = (*max_run_time_us).max(*run_time_us);
                *stamp = now;
                *run_counter += 1;
                need_remove = need_remove || !*valid;
            }
        }
        if need_remove {
            self.running_task.borrow_mut().retain(|x| x.valid);
            *self.task_update.borrow_mut() = true;
            self.sort();
        }

        self.sleeper.borrow_mut().loop_sleep();
        *self.tic.borrow_mut() += 1;
    }
}
