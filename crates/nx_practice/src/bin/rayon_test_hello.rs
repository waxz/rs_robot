//https://github.com/rayon-rs/rayon
// https://blog.logrocket.com/implementing-data-parallelism-rayon-rust/

use nx_common::common::signal_handler::SignalHandler;
use rayon::prelude::*;
use std::io;
use std::sync::{Arc, Mutex};
use tokio::signal::unix::signal;

fn sum_of_squares(input: &[i32]) -> i32
{
    input
        .par_iter() // <-- just change that!
        .map(|&i| i * i)
        .sum()
}

#[test]
fn spwan()
{
    rayon::spawn(|| {
        println!("spawn 1");
    });
}
#[test]
fn test_sum()
{
    let v1 = [1, 2, 3, 4, 5, 6];
    let sum = sum_of_squares(&v1);

    println!("sum = {}", sum);

    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(8)
        .build()
        .unwrap();

    let n = pool.install(|| fib(20));
    println!("{}", n);
}

fn fib(n: usize) -> usize
{
    if n == 0 || n == 1 {
        return n;
    }
    let (a, b) = rayon::join(|| fib(n - 1), || fib(n - 2)); // runs inside of `pool`
    return a + b;
}

fn main() -> io::Result<()>
{
    let signal_handler = SignalHandler::default();

    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(8)
        .build()
        .unwrap();
    let array_int: Arc<Mutex<Vec<i32>>> = Arc::new(Mutex::new(vec![]));

    let signal_handler = SignalHandler::default();

    let t = rayon::spawn(|| {
        println!("run");
    });

    Ok(())
}
