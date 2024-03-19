//https://crates.io/crates/spin_sleep
use std::time::Duration;
use time::{format_description, OffsetDateTime};
use tracing::info;

use nx_common::common::signal_handler::SignalHandler;

// use nx_common::common::loop_helper::LoopHelper;
use spin_sleep::LoopHelper;

fn test()
{
    let mut signal_handler = SignalHandler::default();
    let std_fps = 20.0f32;

    let mut loop_helper = LoopHelper::builder().build_with_target_rate(std_fps); // limit to 250 FPS if possible

    while signal_handler.is_run() {
        loop_helper.loop_start();
        let t2 = OffsetDateTime::now_utc();
        info!("t2 {:?}", t2);
        loop_helper.loop_sleep();
    }

    signal_handler.stop();
}

fn main()
{
    let signal_handler = SignalHandler::default();

    // Create a new sleeper that trusts native thread::sleep with 100μs accuracy
    let spin_sleeper = spin_sleep::SpinSleeper::new(100_000)
        .with_spin_strategy(spin_sleep::SpinStrategy::YieldThread);

    // Sleep for 1.01255 seconds, this will:
    //  - thread:sleep for 1.01245 seconds, i.e., 100μs less than the requested duration
    //  - spin until total 1.01255 seconds have elapsed
    spin_sleeper.sleep(Duration::new(1, 1_000));

    {
        let t1 = std::time::SystemTime::now();
        let t1: OffsetDateTime = t1.into();

        std::thread::sleep(std::time::Duration::from_millis(50));
        let t2 = std::time::SystemTime::now();
        let t2: OffsetDateTime = t2.into();

        let d1 = t2 - t1;
        println!("t1 = {}, t2 = {}, d = {}", t1, t2, d1);
    }
    {
        let t1 = std::time::SystemTime::now();
        let t1: OffsetDateTime = t1.into();

        spin_sleeper.sleep_s(1.12345678);

        let t2 = std::time::SystemTime::now();
        let t2: OffsetDateTime = t2.into();

        let d1 = t2 - t1;
        println!("t1 = {}, t2 = {}, d = {}", t1, t2, d1);
    }

    let mut t1: OffsetDateTime = std::time::SystemTime::now().into();
    let mut t2: OffsetDateTime = std::time::SystemTime::now().into();

    // while signal_handler.is_run(){
    //     spin_sleeper.sleep_s(0.005);
    //     t2 = std::time::SystemTime::now().into();
    //     println!("loop d = {}", t2-t1);
    //     t1 = t2;
    // }

    let std_fps = 100.0f32;

    let mut loop_helper = LoopHelper::builder()
        .report_interval_s(0.5) // report every half a second
        // .native_accuracy_ns(10_000)
        .build_with_target_rate(std_fps); // limit to 250 FPS if possible

    let mut current_fps = None;

    let mut max_error = 0.0_f32;
    let std_interval = 1.0 / std_fps;

    while signal_handler.is_run() {
        let delta = loop_helper.loop_start(); // or .loop_start_s() for f64 seconds

        // compute_something(delta);
        spin_sleeper.sleep_s(0.002);

        {
            let t1: OffsetDateTime = std::time::SystemTime::now().into();

            // std::thread::sleep(Duration::from_millis(6));

            let mut acc = 0;
            for i in 1..10 {
                for j in 1..10 {
                    acc = j * i;
                }
            }

            let t2: OffsetDateTime = std::time::SystemTime::now().into();
            let d = t2 - t1;
            println!("sleep take {} s", d.as_seconds_f32());
        }

        if let Some(fps) = loop_helper.report_rate() {
            current_fps = Some(fps);
            println!("fps {}", fps);
        }

        loop_helper.loop_sleep(); // sleeps to achieve a 250 FPS rate
        t2 = std::time::SystemTime::now().into();
        let d = (t2 - t1);
        t1 = t2;

        let d = d.as_seconds_f32();

        let error = (d - std_interval);
        max_error = {
            if error.abs() > max_error.abs() {
                error
            } else {
                max_error
            }
        };
        println!(
            "loop d = {} s,std_interval:{} s, error : {} us, max_error : {} us",
            d,
            std_interval,
            error * 1e6,
            max_error * 1e6
        );
    }
}
