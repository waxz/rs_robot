use std::time::Instant;

pub struct Bench {}

impl Bench
{
    pub fn run<F: FnMut() + Send + 'static>(mut f: F, run_time: u64)
    {
        // let mut full_time_s = 0.0;
        let mut full_time_ns = 0;
        let test_start = Instant::now();

        let mut batch = 0;
        while test_start.elapsed().as_secs() < run_time {
            // let t1: OffsetDateTime = std::time::SystemTime::now().into();
            let start = Instant::now();
            f();
            let elapsed = start.elapsed();

            // let t2: OffsetDateTime = std::time::SystemTime::now().into();
            // let dur_per_run_s = (t2 - t1).as_seconds_f32();
            // full_time_s += dur_per_run_s;
            full_time_ns += elapsed.as_nanos();
            batch += 1;
        }

        // (0..self.batch).into_iter().for_each(|_|{
        //     f()
        // });
        // let dur_per_run_us = full_time_s/ (self.batch as f32) * 1e6;
        // let dur_per_run_us = full_time_s/ (self.batch as f32) * 1e6;
        let dur_per_run_ns = (full_time_ns as f32) / (batch as f32);

        let dur_per_run_us = dur_per_run_ns * 1e-3;
        println!("-----------------------------------------------------------------------------------------------");
        println!("|     batch     |    time_run_through(us)    |    time_per_run(us)    |    time_per_run(ns)    |");
        println!(
            "|{:^15}|{:^28}|{:^24}|{:^24}|",
            batch,
            test_start.elapsed().as_micros(),
            dur_per_run_us,
            dur_per_run_ns
        );
    }
}
