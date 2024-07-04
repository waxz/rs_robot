use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::sync::OnceLock;
use time::OffsetDateTime;
// use time::{OffsetDateTime, UtcOffset};

#[inline]
pub fn get_timezone_offset() -> time::UtcOffset
{
    static TZ_OFFSET: OnceLock<time::UtcOffset> = OnceLock::new();
    TZ_OFFSET.get_or_init(|| {
        time::UtcOffset::current_local_offset().expect("Warning! get local offset failed!")
    });
    *TZ_OFFSET.get().unwrap()
}
/// must call before thread creation
#[inline]

pub fn get_now_local() -> time::OffsetDateTime
{
    time::OffsetDateTime::now_utc().to_offset(get_timezone_offset())
}

pub fn get_local_from_ns(ns: i128) -> time::OffsetDateTime
{
    time::OffsetDateTime::from_unix_timestamp_nanos(ns)
        .unwrap()
        .to_offset(get_timezone_offset())
}

pub struct Time {}

impl Time
{
    // const TZ_OFFSET:UtcOffset = UtcOffset::current_local_offset().expect("should get local offset!");
    //
    // pub fn get_current_time_local() ->OffsetDateTime{
    //     OffsetDateTime::now_utc().to_offset(Self::TZ_OFFSET)
    // }
}
