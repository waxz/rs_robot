pub mod perception
{

    pub fn se3_inverse(   tx: f32,
                          ty: f32,
                          tz: f32,
                          roll: f32,
                          pitch: f32,
                          yaw: f32,
                          itx : *mut f32,
                          ity : *mut f32,
                          itz : *mut f32,
                          iroll : *mut f32,
                          ipitch : *mut f32,
                          iyaw : *mut f32,


    ) ->i32{

        unsafe {
            crate::binding::se3_inverse( tx, ty, tz, roll, pitch, yaw, itx, ity, itz, iroll, ipitch, iyaw)
        }
}

    pub fn pointcloud_norm(
        src_buffer: *mut f32,
        point_num: u64,
        index_buffer: *mut u64,
        index_num : u64,
        vx: f32,
        vy: f32,
        vz: f32,
        cx: *mut f32,
        cy: *mut f32,
        cz: *mut f32,
        nx: *mut f32,
        ny: *mut f32,
        nz: *mut f32,
        nd: *mut f32,
    )->i32{
        unsafe {
            crate::binding::pointcloud_norm(src_buffer, point_num, index_buffer, index_num, vx, vy, vz, cx, cy, cz, nx , ny, nz, nd)
        }
    }

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
