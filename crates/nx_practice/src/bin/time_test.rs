use std::time::{Duration, SystemTime};
use time::{format_description, OffsetDateTime, UtcOffset};
//https://stackoverflow.com/questions/38957718/format-stdtime-output
use chrono::{NaiveDateTime, TimeZone, Timelike, Utc};

use nx_common::common::signal_handler::SignalHandler;

fn get_naive_date_time(unix_ns: u64) -> NaiveDateTime
{
    NaiveDateTime::from_timestamp_opt(
        (unix_ns / 1_000_000_000) as i64,
        (unix_ns % 1_000_000_000) as u32,
    )
    .unwrap()
}

fn main()
{
    //================

    let signal_handler = SignalHandler::default();

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

    let timestamp_u64: u64 = 1709864651172254133;

    let date_time = get_naive_date_time(timestamp_u64);

    println!("date_time: {:?}", date_time);
    println!("date_time: {:?}", date_time.timestamp_nanos_opt());

    let t1 = OffsetDateTime::from_unix_timestamp_nanos(timestamp_u64 as i128).unwrap();
    let t2 = OffsetDateTime::now_local().unwrap();
    let t3 = OffsetDateTime::now_utc();

    println!(
        "t1: {:?}, stamp: {}, offset: {:?}",
        t1,
        t1.unix_timestamp_nanos(),
        t1.offset()
    );
    println!(
        "t2: {:?}, stamp: {}, offset: {:?}",
        t2,
        t2.unix_timestamp_nanos(),
        t2.offset()
    );
    println!(
        "t3: {:?}, stamp: {}, offset: {:?}",
        t3,
        t3.unix_timestamp_nanos(),
        t3.offset()
    );
    let t4 = t3.to_offset(tz_offset);
    let t5 = t1.to_offset(tz_offset);

    println!(
        "t4: {:?}, stamp: {}, offset: {:?}",
        t4,
        t4.unix_timestamp_nanos(),
        t4.offset()
    );
    println!(
        "t5: {:?}, stamp: {}, offset: {:?}",
        t5,
        t5.unix_timestamp_nanos(),
        t5.offset()
    );

    return;

    let t1 = std::time::SystemTime::now();
    let mut t1: OffsetDateTime = t1.into();

    std::thread::sleep(std::time::Duration::from_millis(1_200));
    let t2 = std::time::SystemTime::now();
    let mut t2: OffsetDateTime = t2.into();

    println!("t1 : {t1}, t2 {t2}");
    t2 = t2.replace_nanosecond(t1.nanosecond()).unwrap();
    // t2.replace_microsecond(t1.microsecond());
    // t2.replace_millisecond(t1.millisecond());

    println!("t1 : {t1}, t2 {t2}");

    let format = format_description::parse(
        "[year]-[month]-[day] [hour]:[minute]:[second] [offset_hour \
         sign:mandatory]:[offset_minute]:[offset_second]",
    )
    .unwrap();

    let d1 = t2 - t1;
    let d2 = t1 - t2;
    println!("d1 = {}, d2 = {}", d1, d2);

    println!(
        "t1 = {:?}, t2 = {:?}, (t2 > t1 = {}) d1 = {}, d1_s = {}",
        t1,
        t2,
        t2 > t1,
        d1,
        d1.as_seconds_f32()
    );

    println!(
        "unix_timestamp_nanos: t1 = {},t2 = {}, t2 - t1 = {}",
        t1.unix_timestamp_nanos(),
        t2.unix_timestamp_nanos(),
        t2.unix_timestamp_nanos() - t1.unix_timestamp_nanos(),
    );

    println!(
        "unix = {}",
        t1.unix_timestamp_nanos() / ((1e9 as i128) * 3600 * 24 * 365)
    );
    println!(
        "unix_timestamp_nanos = {}, {}",
        t1.unix_timestamp_nanos(),
        t1.unix_timestamp_nanos() as u64
    );

    println!(
        "t1 = {:?}, t2 = {:?}",
        t1.format(&format),
        t2.format(&format)
    );
}
