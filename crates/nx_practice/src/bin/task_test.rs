//https://stackoverflow.com/questions/71233393/rust-dyn-fn-cannot-be-shared-between-threads-safely
// https://stackoverflow.com/questions/32370021/sharing-a-reference-to-an-instance-of-trait-between-threads
//https://stackoverflow.com/questions/74130761/dyn-fnstring-cannot-be-shared-between-threads-safely

//https://stackoverflow.com/questions/72836307/executing-a-function-periodically-on-accurate-and-precise-intervals
use spin_sleep::SpinSleeper;
use std::cell::RefCell;
use std::cmp::max;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use time::{format_description, OffsetDateTime};

use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::Thread;

use chrono::DateTime;
fn main()
{
    let mut t1: OffsetDateTime = std::time::SystemTime::now().into();

    println!("start {:?}", t1);
    let mut max_error_s = 0.0_f32;

    let signal_handler = SignalHandler::default();

    let mut task_manager: Rc<RefCell<nx_common::common::task::TaskManager>> =
        Rc::new(RefCell::new(nx_common::common::task::TaskManager::new(
            20, 100.0, 500_000,
        )));

    let cnt = Arc::new(Mutex::new(0));
    {
        let mut task_1_time: OffsetDateTime = std::time::SystemTime::now().into();

        let cnt = cnt.clone();
        let task_manager1 = Rc::clone(&task_manager);

        let mut task1 = move || {
            *cnt.lock().unwrap() += 1;
            let now: OffsetDateTime = std::time::SystemTime::now().into();
            let dur = (now - task_1_time);
            task_1_time = now;
            let d_s = (0.02 - dur.as_seconds_f32());

            let flag = *cnt.lock().unwrap();
            if (flag > 100) {
                max_error_s = if (max_error_s.abs() < d_s.abs()) {
                    d_s
                } else {
                    max_error_s
                };

                if flag % 100 == 0 {
                    print!(
                        "run task 0, now : {}, interval: {}, error :{} us, max_error ;{} us\n",
                        now,
                        dur,
                        d_s * 1e6,
                        max_error_s * 1e6
                    );
                }
            }

            if (*cnt.lock().unwrap() == 1) {
                let task_manager2 = task_manager1.clone();
                let task2 = move || {
                    let now: OffsetDateTime = std::time::SystemTime::now().into();
                    println!("task 1 run, now {}", now);
                    return false;
                };
                task_manager2.borrow().add("task2", task2, 100, 1000_000);
            }
            if (*cnt.lock().unwrap() == 2) {
                let task_manager2 = task_manager1.clone();
                let task2 = move || {
                    let now: OffsetDateTime = std::time::SystemTime::now().into();
                    println!("task 2 run, now {}", now);
                    return true;
                };
                task_manager2.borrow().add("task3", task2, 100, 1000_000);
            }

            return true;
        };
        task_manager.borrow().add("task1", task1, 1, 100);
    }

    while signal_handler.is_run() {
        task_manager.borrow().run();
    }
    println!("cnt = {}", cnt.lock().unwrap());
}
