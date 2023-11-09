//https://github.com/tokio-rs/tracing/issues/971
//https://stackoverflow.com/questions/72947600/how-to-show-line-number-and-file-on-tracing-events

use std::net::Ipv4Addr;
use nx_common::common::signal_handler::SignalHandler;
use tracing::{error, info, trace, warn};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

fn test_vec_iter() {
    let v1 = [1, 2, 3, 4, 5, 6];

    let has_2 = v1.iter().any(|x| *x == 2);
    println!("v1 = {:?}, has_2 = {}", v1, has_2);
    info!("v1 = {:?}, has_2 = {}", v1, has_2);
}

struct Connection {
    port: i32,
    speed: f64
}

fn main() {
    let signal_handler = SignalHandler::default();

    let file_appender = tracing_appender::rolling::hourly("./logs", "log");

    let (file_writer, _guard) = tracing_appender::non_blocking(file_appender);

    // multiple appender
    // set format and filter
    let subscriber = Registry::default()
        .with(
            fmt::Layer::default()
                .with_writer(file_writer)
                .with_line_number(true)
                .with_thread_ids(true)
                .with_file(true)
                .with_filter(filter::filter_fn(|metadata| {
                    // println!("metadata:{:?}",metadata);
                    // *metadata.level() == filter::LevelFilter::ERROR
                    metadata.target().starts_with("hello")
                }))

            ,
        )
        .with(
            fmt::Layer::default()
                .with_writer(std::io::stdout)
                .with_line_number(true)
                .with_file(true)
                .with_thread_ids(true)
                .with_filter(filter::LevelFilter::INFO),
        );

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");

    trace!("track log");
    warn!("warn log");
    error!("error log");

    test_vec_iter();

    // log in thread
    let handlers: Vec<_> = (1..10)
        .map(|i| {
            let signal_handler = signal_handler.clone();
            std::thread::spawn(move || {
                while signal_handler.is_run() {
                    info!("run in thread {}", i);
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

    let mut vint = [1,2,3,4];
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
