use std::fmt::Debug;

use time::{format_description, OffsetDateTime};


//https://stackoverflow.com/questions/38957718/format-stdtime-output

fn main(){


    let t1 = std::time::SystemTime::now();
    let t1:OffsetDateTime = t1.into();

    std::thread::sleep(std::time::Duration::from_millis(100));
    let t2 = std::time::SystemTime::now();
    let t2:OffsetDateTime = t2.into();

    let format = format_description::parse(
        "[year]-[month]-[day] [hour]:[minute]:[second] [offset_hour \
         sign:mandatory]:[offset_minute]:[offset_second]",
    ).unwrap() ;

    let d1 = t2 - t1;
    println!("t1 = {:?}, t2 = {:?}, (t2 > t1 = {}) d1 = {}, d1_s = {}", t1,t2,t2>t1,d1,d1.as_seconds_f32());

    println!("unix_timestamp_nanos: t1 = {},t2 = {}, t2 - t1 = {}", t1.unix_timestamp_nanos(), t2.unix_timestamp_nanos() , t2.unix_timestamp_nanos() - t1.unix_timestamp_nanos(), );

    println!("unix = {}", t1.unix_timestamp_nanos()/((1e9 as i128) * 3600 * 24 * 365));
    println!("unix_timestamp_nanos = {}, {}",t1.unix_timestamp_nanos(), t1.unix_timestamp_nanos() as u64);

    println!("t1 = {:?}, t2 = {:?}",t1.format(&format),t2.format(&format));

}