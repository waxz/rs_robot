pub mod perception
{

    pub fn pointcloud_clip(
        src_buffer: *mut f32,
        height: u64,
        width: u64,
        dst_buffer: *mut f32,
        filter_height_min: u64,
        filter_height_max: u64,
        filter_width_min: u64,
        filter_width_max: u64,
    ) -> i32
    {
        unsafe {
            crate::binding::pointcloud_clip(
                src_buffer,
                height,
                width,
                dst_buffer,
                filter_height_min,
                filter_height_max,
                filter_width_min,
                filter_width_max,
            )
        }
    }

    pub fn pointcloud_transform(
        src_buffer: *mut f32,
        point_num: u64,
        dst_buffer: *mut f32,
        tx: f32,
        ty: f32,
        tz: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    ) -> i32
    {
        unsafe {
            crate::binding::pointcloud_transform(
                src_buffer, point_num, dst_buffer, tx, ty, tz, roll, pitch, yaw,
            )
        }
    }
}
