// std
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicI64, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use std::ops::Deref;
use std::os::raw::c_void;
use std::sync::atomic::Ordering::Relaxed;
use std::time::{Duration, SystemTime};
use time::macros::format_description;
use time::{error, format_description, OffsetDateTime, UtcOffset};
//https://stackoverflow.com/questions/38957718/format-stdtime-output
use chrono::{NaiveDateTime, TimeZone, Timelike, Utc};

// argument
use clap::Parser;

// logger
use logs_wheel::LogFileInitializer;
use rolling_file::{BasicRollingFileAppender, RollingConditionBasic};
use tracing::{error, event, info, instrument, span, trace, warn, Level};
use tracing_subscriber::fmt::format;
use tracing_subscriber::fmt::time::OffsetTime;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

// user code
use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::task::TaskManager;
use nx_common::common::thread::{get_current_cpu, Thread};
// use nx_common::common::loop_helper::LoopHelper;
use spin_sleep::LoopHelper;

//atomic
use nx_common::common::atomic::{AtomicF32, AtomicF64};

use libc;
use std::io;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_practice/src/bin/comm_config.toml"
    )]
    filename: String,
    #[arg(short, long, default_value = "/tmp/logs")]
    log_dir: String,
}

fn main()
{
    let args = Args::parse();
    //pub(super) fn local_offset_at(datetime: OffsetDateTime) -> Option<UtcOffset> {
    // Continue to obtaining the UTC offset if and only if the call is sound or the user has
    // explicitly opted out of soundness.
    //
    // Soundness can be guaranteed either by knowledge of the operating system or knowledge that the
    // process is single-threaded. If the process is single-threaded, then the environment cannot
    // be mutated by a different thread in the process while execution of this function is taking
    // place, which can cause a segmentation fault by dereferencing a dangling pointer.
    //
    // If the `num_threads` crate is incapable of determining the number of running threads, then
    // we conservatively return `None` to avoid a soundness bug.

    let tz_offset = UtcOffset::current_local_offset().expect("should get local offset!");
    let time_fmt = OffsetTime::new(
        tz_offset,
        format_description!("[year]-[month]-[day] [hour]:[minute]:[second]:[subsecond] "),
    );

    let current_file_name = std::path::Path::new(file!())
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();

    let file_appender = BasicRollingFileAppender::new(
        format!("{}/{}", &args.log_dir, current_file_name),
        RollingConditionBasic::new().hourly().max_size(1024 * 1024),
        10,
    )
    .unwrap();
    let wheel_file_appender = LogFileInitializer {
        directory: &args.log_dir,
        filename: &current_file_name,
        max_n_old_files: 2,
        preferred_max_file_size_mib: 1,
    }
    .init()
    .unwrap();

    // let (file_writer, file_writer_guard) = tracing_appender::non_blocking(wheel_file_appender);
    let (file_writer, file_writer_guard) = tracing_appender::non_blocking(file_appender);

    let t1 = OffsetDateTime::now_utc().to_offset(tz_offset);
    println!("{}, t1 {:?}", line!(), t1);

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
                .with_filter(filter::LevelFilter::INFO), // .with_filter(filter::filter_fn(|metadata| {
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

    // Retrieve the IDs of all cores on which the current
    // thread is allowed to run.
    // NOTE: If you want ALL the possible cores, you should
    // use num_cpus.

    let mut signal_handler = SignalHandler::default();

    let atomic_number_f32 = Arc::new(AtomicF32::new(0.0));
    let atomic_number_u32 = Arc::new(AtomicU32::new(0));

    let mut test_thread = Thread::default();
    {
        let signal_handler = signal_handler.clone();

        // let atomic_number_f32 = atomic_number_f32.clone();
        let mut atomic_number_u32 = atomic_number_u32.clone();
        let mut atomic_number_f32 = atomic_number_f32.clone();

        test_thread = Thread::new(move || {
            let std_fps = 100.0f32;

            //100_000
            let mut loop_helper = LoopHelper::builder()
                .native_accuracy_ns(100_000)
                .build_with_target_rate(std_fps); // limit to 250 FPS if possible
            let mut counter = 0;

            // let core_ids = core_affinity::get_core_ids().unwrap();
            // let id = core_ids[4];
            // let res = core_affinity::set_for_current(id);
            // if (res) {
            //     // Do more work after this.
            //     warn!("set affinity to {:?}", id);
            //
            // }
            //
            // let cores: Vec<usize> = (0.. affinity::get_core_num()).step_by(2).collect();
            let cores: [usize; 1] = [0];
            println!("Binding thread to cores : {:?}", &cores);
            // Output : "Binding thread to cores : [0, 2, 4, 6]"

            affinity::set_thread_affinity(&cores).unwrap();
            println!(
                "Current thread affinity : {:?}",
                affinity::get_thread_affinity().unwrap()
            );
            // Output : "Current thread affinity : [0, 2, 4, 6]"

            while signal_handler.is_run() {
                loop_helper.loop_start();

                warn!(
                    "I'm thread {:?} on cpu {:?}",
                    std::thread::current().id(),
                    get_current_cpu()
                );
                counter += 1;
                let v = atomic_number_u32.load(Relaxed);
                atomic_number_u32.store(v + 1, Relaxed);
                let v = atomic_number_f32.load(Relaxed);
                atomic_number_f32.store(v + 1.0, Relaxed);
                let t1 = OffsetDateTime::now_utc().to_offset(tz_offset);
                info!("t1 {:?}", t1);

                loop_helper.loop_sleep();
            }
            warn!("exit t1");
        });
    }

    let std_fps = 100.0f32;

    let mut loop_helper = LoopHelper::builder()
        .native_accuracy_ns(100_000)
        .build_with_target_rate(std_fps); // limit to 250 FPS if possible

    let mut counter = 0;
    let mut counter_max = 100;

    let core_ids = core_affinity::get_core_ids().unwrap();
    let id = core_ids[9];
    let res = core_affinity::set_for_current(id);
    if (res) {
        warn!("set affinity to {:?}", id);
        // Do more work after this.
    }

    // let cores: Vec<usize> = (0.. affinity::get_core_num()).step_by(2).collect();
    let cores: [usize; 2] = [0, 1];
    println!("Binding thread to cores : {:?}", &cores);
    // Output : "Binding thread to cores : [0, 2, 4, 6]"

    affinity::set_thread_affinity(&cores).unwrap();
    println!(
        "Current thread affinity : {:?}",
        affinity::get_thread_affinity().unwrap()
    );
    // Output : "Current thread affinity : [0, 2, 4, 6]"

    while signal_handler.is_run() && counter < counter_max {
        loop_helper.loop_start();

        warn!(
            "I'm thread {:?} on cpu {:?}",
            std::thread::current().id(),
            get_current_cpu()
        );

        let v = atomic_number_u32.load(Relaxed);
        atomic_number_u32.store(v + 1, Relaxed);
        let v = atomic_number_f32.load(Relaxed);
        atomic_number_f32.store(v + 1.0, Relaxed);
        let t2 = OffsetDateTime::now_utc().to_offset(tz_offset);
        info!("t2 {:?}, counter:{} ", t2, counter);
        for i in 0..100 {
            let t2 = OffsetDateTime::now_utc().to_offset(tz_offset);
            info!("t2 {:?}", t2);
        }
        counter += 1;

        loop_helper.loop_sleep();
    }
    warn!("exit t2");

    signal_handler.stop();

    warn!("signal_handler.stop()");
    let atomic_number_u32 = atomic_number_u32.load(Relaxed);
    warn!("atomic_number_u32:{}", atomic_number_u32);
    let atomic_number_f32 = atomic_number_f32.load(Relaxed);
    warn!("atomic_number_f32:{}", atomic_number_f32);
}
