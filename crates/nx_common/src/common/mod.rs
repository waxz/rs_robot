#[macro_export]
macro_rules! ok(($expression:expr) => ($expression.unwrap()));
#[macro_export]
macro_rules! log {
    ($fmt:expr) => (println!(concat!("{}:{}:{}: ", $fmt),file!(), line!(),column!() ));
    ($fmt:expr, $($arg:tt)*) => (println!(concat!("{}:{}:{}: ", $fmt),file!(),
    line!(),column!(), $($arg)*));
}

#[macro_export]
macro_rules! bench {
    ($func:expr,$test_time:expr ) => {

        println!("===============================================================================================");
        println!("{}:{}:{}",file!(), line!(),column!());
        println!("-----------------------------------------------------------------------------------------------");
        println!("|     batch     |    time_run_through(us)    |    time_per_run(us)    |    time_per_run(ns)    |");


        let timer = std::time::Instant::now();

        {
            $func
        }
        let timer_ns = timer.elapsed().as_nanos();

        let test_time_us = $test_time as f32 * 1e6;
        let init_batch =( ($test_time as f32 * 1e9) /(timer_ns as f32)) as u32;
        let mut batch = init_batch;


        while true{


        let timer = std::time::Instant::now();

        for _ in 0 .. batch{
            $func
        }

        let timer_us = timer.elapsed().as_micros() as f32;

        let dur_per_run_us = (timer_us ) / (batch as f32);

        let dur_per_run_ns = dur_per_run_us * 1e3;

        println!(
            "|{:^15}|{:^28}|{:^24}|{:^24}|",
            batch,
            timer_us,
            dur_per_run_us,
            dur_per_run_ns
        );



        if timer_us > test_time_us{
            break;
        }else{
            batch = batch.max(((batch as f32) * 1.5  * test_time_us / timer_us) as u32);
        }

        }
    };
}

pub mod signal_handler;
pub mod socket_client;
pub mod socket_server;

pub mod bench;

pub mod graph;
pub mod interpolate;
pub mod loop_helper;
pub mod math;
pub mod statistic;

pub mod string;
pub mod task;
pub mod thread;
pub mod transform2d;
pub mod types;

pub mod atomic;

pub mod memory;
pub mod time;

#[cfg(test)]
mod test
{
    use crate::common::bench::Bench;
    use itertools::izip;
    #[test]
    fn test_bench()
    {

        log!("run bench test");
        log!("run bench test : {}", "hello");

        Bench::run(
            || {
                let mut x = 0;
                for i in 1..1000 {
                    x += i;
                }
            },
            5,
        );
        Bench::run(
            || {
                let mut v1 = vec![0; 2000];
                for (i, j) in izip!(&mut v1, (1..2000)) {
                    *i = j;
                }
            },
            5,
        );
        Bench::run(
            || {
                let mut v1 = vec![0; 2000];
                v1 = (1..2000).into_iter().map(|x| x + 10).collect();
            },
            5,
        );
    }
}
