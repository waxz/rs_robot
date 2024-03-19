// #![feature(lazy_cell)]
// use std::sync::LazyLock;
use std::sync::OnceLock;
use time::{OffsetDateTime, UtcOffset};

fn get_now_local() -> OffsetDateTime
{
    static TZ_OFFSET: OnceLock<UtcOffset> = OnceLock::new();
    TZ_OFFSET.get_or_init(|| UtcOffset::current_local_offset().expect("should get local offset!"));
    OffsetDateTime::now_utc().to_offset(*TZ_OFFSET.get().unwrap())
}
fn get_now() -> OffsetDateTime
{
    OffsetDateTime::now_utc()
}

static THE_THING: OnceLock<i32> = OnceLock::new();
fn init_the_thing(value: i32) -> &'static i32
{
    // Perfectly fine
    THE_THING.get_or_init(|| value)
}

struct Animal {}

impl Animal
{
    const THE_THING: OnceLock<i32> = OnceLock::new();
}

fn main()
{
    init_the_thing(55);
    println!("THE_THING: {:?} ", THE_THING);
    println!("Animal::THE_THING: {:?} ", Animal::THE_THING);

    get_now_local();
    std::thread::spawn(|| {
        println!("{:?}", get_now_local());
    })
    .join()
    .unwrap();
    std::thread::spawn(|| {
        println!("{:?}", get_now_local());
    })
    .join()
    .unwrap();

    {
        let s = std::time::Instant::now();
        {
            for i in 0..10000 {
                get_now_local();
            }
        }
        println!("get_now_local: {:?}", s.elapsed());
    }

    {
        let s = std::time::Instant::now();
        {
            for i in 0..10000 {
                get_now();
            }
        }
        println!("get_now: {:?}", s.elapsed());
    }

    {
        let s = std::time::Instant::now();
        {
            for i in 0..10000 {
                nx_common::common::time::get_now_local();
            }
        }
        println!("nx_common get_now_local: {:?}", s.elapsed());
    }
}
