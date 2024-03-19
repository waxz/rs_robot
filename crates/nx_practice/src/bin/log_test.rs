//https://github.com/tokio-rs/tracing/issues/971
//https://stackoverflow.com/questions/72947600/how-to-show-line-number-and-file-on-tracing-events

use color_eyre::{eyre::eyre, Result};
use colored::*;
use nx_common::common::signal_handler::SignalHandler;
use rolling_file::{BasicRollingFileAppender, RollingConditionBasic};
use std::net::Ipv4Addr;
use std::path::Path;
use time::macros::format_description;
use time::UtcOffset;
use tracing::{debug, error, event, info, instrument, span, trace, warn, Level};
use tracing_subscriber::fmt::time::OffsetTime;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

use logs_wheel::LogFileInitializer;

fn test_vec_iter()
{
    let v1 = [1, 2, 3, 4, 5, 6];

    let has_2 = v1.iter().any(|x| *x == 2);
    println!("v1 = {:?}, has_2 = {}", v1, has_2);
    info!("v1 = {:?}, has_2 = {}", v1, has_2);
}

struct Connection
{
    port: i32,
    speed: f64,
}

#[instrument]
fn foo(ans: i32)
{
    info!("in foo");
}
#[instrument]
fn return_err() -> Result<()>
{
    Err(eyre!("Something went wrong"))
}

#[instrument]
fn call_return_err()
{
    info!("going to log error");
    if let Err(err) = return_err() {
        // 推荐大家运行下，看看这里的输出效果
        error!(?err, "error");
    }
}

fn main()
{
    println!("{}", "hello".red().on_black());
    println!("{}", "hello".blue().on_red());

    let signal_handler = SignalHandler::default();

    let offset = UtcOffset::current_local_offset().expect("should get local offset!");
    let time_fmt = OffsetTime::new(
        offset,
        format_description!("[year]-[month]-[day] [hour]:[minute]:[second]:[subsecond] "),
    );

    let current_file_name = Path::new(file!()).file_stem().unwrap().to_str().unwrap();

    println!("current_file_name: {}", current_file_name);

    let file_appender = BasicRollingFileAppender::new(
        format!("./logs/{}", current_file_name),
        RollingConditionBasic::new().hourly().max_size(1024 * 1024),
        10,
    )
    .unwrap();

    let wheel_file_appender = LogFileInitializer {
        directory: "logs/log",
        filename: "test",
        max_n_old_files: 2,
        preferred_max_file_size_mib: 1,
    }
    .init()
    .unwrap();

    let (file_writer, _guard) = tracing_appender::non_blocking(file_appender);
    let (file_writer, _guard) = tracing_appender::non_blocking(wheel_file_appender);

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
                .with_filter(filter::LevelFilter::ERROR),
        );

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");

    debug!("info debug");
    info!("info log");
    trace!("track log");
    warn!("warn log");
    error!("error log");

    return;
    // 安裝 color-eyre 的 panic 处理句柄
    color_eyre::install();

    call_return_err();

    test_vec_iter();

    // log in thread
    let handlers: Vec<_> = (1..10)
        .map(|i| {
            let signal_handler = signal_handler.clone();
            std::thread::spawn(move || {
                let mut cnt = 0;
                while signal_handler.is_run() {
                    // 在 span 的上下文之外记录一次 event 事件
                    event!(Level::INFO, "something happened");
                    {
                        let span = span!(Level::TRACE, "my_span");

                        // `enter` 返回一个 RAII ，当其被 drop 时，将自动结束该 span
                        let enter = span.enter();
                        info!("run in thread {}", i);

                        foo(cnt);
                        cnt += 1;
                        // 在 "my_span" 的上下文中记录一次 event
                        event!(Level::DEBUG, "something happened inside my_span");
                    }
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
            })
        })
        .collect();

    let addr = Ipv4Addr::new(127, 0, 0, 1);
    let conn = Connection {
        port: 40,
        speed: 3.20,
    };

    let mut cnt = 0;

    let mut vint = [1, 2, 3, 4];

    while signal_handler.is_run() {
        info!( name: "completed", target: "hello", "main loop count : {}",  cnt);
        info!(name: "use_name",target:"hello",port="12345");
        info!(
            target: "connection_events",
            ip = ?addr,
            conn.port,
            ?conn.speed,
            vint = ?vint
        );
        vint[0] = vint[1..].iter().sum();

        cnt += 1;
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    println!("Exit loop");

    for j in handlers {
        j.join().unwrap();
    }
}
