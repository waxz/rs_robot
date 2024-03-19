//https://www.thorsten-hans.com/weekly-rust-trivia-share-state-between-threads/
use rand::distributions::uniform::SampleBorrow;
use std::cell::RefCell;
use std::sync::RwLock;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::{io, thread};

use tracing::{error, info, trace, warn};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

use nx_common::common::signal_handler::SignalHandler;
use std::time::Duration;

fn main1()
{
    let (sender, receiver) = std::sync::mpsc::channel();

    let sending_thread = std::thread::spawn(move || {
        for i in 0..10 {
            println!("[{:?}] Sending: {}", std::thread::current().id(), i);
            sender.send(i).unwrap();
            std::thread::sleep(Duration::from_secs(1));
        }
    });

    let receiving_thread = std::thread::spawn(move || {
        for i in receiver {
            println!("[{:?}] Received: {}", std::thread::current().id(), i);
        }
    });

    let _ = sending_thread.join();
    let _ = receiving_thread.join();
    println!("Done");
}

struct Thread
{
    handler: Option<JoinHandle<()>>,
}

impl Thread
{
    pub fn new<F: FnMut() + Send + 'static>(f: F) -> Self
    {
        Thread {
            handler: Some(std::thread::spawn(f)),
        }
    }
}

impl Default for Thread
{
    fn default() -> Self
    {
        Thread { handler: None }
    }
}

impl Drop for Thread
{
    fn drop(&mut self)
    {
        if let Some(h) = self.handler.take() {
            info!("join thread");
            h.join();
        } else {
            info!("none thread");
        }
    }
}

struct ThreadHolder
{
    t1: Thread,
    t2: Thread,
}
fn main() -> io::Result<()>
{
    let signal_handler = SignalHandler::default();
    let file_appender = tracing_appender::rolling::hourly("./logs", "thread_test");

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
                })),
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

    info!("start thread test");

    let array_int = Arc::new(Mutex::new(vec![]));
    let mut cnt = Arc::new(Mutex::new(0));

    {
        // let th = ThreadHolder{ t1: Thread::default(), t2: Thread::default() };
    }

    {
        info!("thread drop test");
        let mut t1 = Thread::default();
        t1 = Thread::default();
        info!("thread drop test done");
    }

    {
        info!("init th");
        let mut th = ThreadHolder {
            t1: Thread::default(),
            t2: Thread::default(),
        };

        info!("init tf done");

        {
            let cnt = cnt.clone();
            th.t1 = Thread::new(move || {
                println!("run in thread 1");

                for i in 1..10 {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    println!("run in thread 1");
                    *cnt.lock().unwrap() += 10;
                }

                let x = 1;
            });
            info!("init tf t1");
        }
        {
            let cnt = cnt.clone();
            th.t2 = Thread::new(move || {
                for i in 1..10 {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    println!("run in thread 2");
                    *cnt.lock().unwrap() += 100;
                }
                let x = 1;
            });
            info!("init tf t2");
        }

        println!("th drop");
    }

    println!("cnt: {}", cnt.lock().unwrap());
    return Ok(());

    let counter = Arc::new(Mutex::new(0));

    let mut a = vec![1, 2, 3];
    let mut x = 0;

    thread::scope(|s| {
        s.spawn(|| {
            for i in 1..10 {
                std::thread::sleep(std::time::Duration::from_millis(100));
                println!("hello from the first scoped thread");
                // We can borrow `a` here.
                dbg!(&a);
            }
        });
        s.spawn(|| {
            for i in 1..10 {
                std::thread::sleep(std::time::Duration::from_millis(100));

                println!("hello from the second scoped thread");
                // We can even mutably borrow `x` here,
                // because no other threads are using it.
                x += a[0] + a[2];
            }
        });
        println!("hello from the main thread");
    });

    // After the scope, we can modify and access our variables again:
    a.push(4);

    let handles: Vec<_> = (1..3)
        .map(|thread_id| {
            let counter = Arc::clone(&counter);
            let array_int: Arc<Mutex<Vec<i32>>> = Arc::clone(&array_int);
            let signal_handler = signal_handler.clone();
            println!("create thread {}", thread_id);
            let t = thread::spawn(move || {
                *counter.lock().unwrap() += 1;

                for i in 1..10 {
                    println!("run thread {}, count {}", thread_id, i);
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }
                println!("stop thread {}", thread_id);
                while signal_handler.is_run() {
                    {
                        let num = counter.lock().unwrap();
                    }

                    {
                        array_int.lock().unwrap().push(thread_id);
                    }
                    std::thread::sleep(std::time::Duration::from_millis(1000));
                    println!("thread_id {} update", thread_id);
                }
            });
            t
        })
        .collect();
    println!("************create all thread");

    while signal_handler.is_run() {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
    for handle in handles {
        handle.join().unwrap();
    }

    println!("Result: {}", *counter.lock().unwrap());
    println!("array_int: {:?}", *array_int.lock().unwrap());

    Ok(())
}
