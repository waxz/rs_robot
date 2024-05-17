use std::sync::OnceLock;
use time::{OffsetDateTime, UtcOffset};

#[inline]
pub fn get_timezone_offset() -> UtcOffset
{
    static TZ_OFFSET: OnceLock<UtcOffset> = OnceLock::new();
    TZ_OFFSET.get_or_init(|| {
        UtcOffset::current_local_offset().expect("Warning! get local offset failed!")
    });
    *TZ_OFFSET.get().unwrap()
}
/// must call before thread creation
#[inline]

pub fn get_now_local() -> OffsetDateTime
{
    OffsetDateTime::now_utc().to_offset(get_timezone_offset())
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
