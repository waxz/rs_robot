pub mod perception
{
    use crate::binding::{pointcloud_pallet_detector_t, ta_cfg_t, PalletInfo_ptr};
    use nx_common::common::types::UnsafeMutexSender;
    use std::ffi::CString;
    use std::ops::Deref;
    use std::ptr::null_mut;
    use std::slice;

    #[derive(Copy, Clone, Debug)]
    pub struct PalletInfo
    {
        pub confidence: f32,
        pub info: i32,
        pub tx: f64,
        pub ty: f64,
        pub tz: f64,
        pub roll: f64,
        pub pitch: f64,
        pub yaw: f64,
    }

    pub struct PointcloudPalletDetector
    {
        handler: UnsafeMutexSender<pointcloud_pallet_detector_t>, //
        pallets: Vec<PalletInfo>,
    }
    impl PointcloudPalletDetector
    {
        pub fn new() -> Self
        {
            let ptr = unsafe { crate::binding::pointcloud_pallet_detector_create() };

            Self {
                handler: UnsafeMutexSender::new(ptr),
                pallets: vec![],
            }
        }

        pub fn create(&mut self, filename: &str, ta_cfg: ta_cfg_t) -> bool
        {
            let filename: CString = CString::new(filename).unwrap();

            // let ptr = &mut self.handler.lock().unwrap().ptr;
            let ptr = self.handler.get();

            unsafe {
                ptr.create.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    filename.as_ptr(),
                    &ta_cfg,
                )
            }
        }

        pub fn set_ground_adaptive_thresh(
            &mut self,
            x_min: f32,
            x_max: f32,
            y_min: f32,
            y_max: f32,
            z_min: f32,
            z_max: f32,
        )
        {
            let ptr = self.handler.get();

            unsafe {
                ptr.set_ground_adaptive_thresh.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    z_min,
                    z_max,
                )
            }
        }
        pub fn set_input(
            &mut self,
            buffer: *mut f32,
            height: u64,
            width: u64,
            vx: f32,
            vy: f32,
            vz: f32,
        )
        {
            let ptr = self.handler.get();
            unsafe {
                ptr.set_input.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    buffer,
                    height,
                    width,
                    vx,
                    vy,
                    vz,
                )
            }
        }

        pub fn set_ground_init_dim(
            &mut self,
            height_min: u64,
            height_max: u64,
            width_min: u64,
            width_max: u64,
        )
        {
            let ptr = self.handler.get();

            println!(
                "set_ground_init_dim: {}, {}, {}, {} ",
                height_min, height_max, width_min, width_max
            );
            unsafe {
                ptr.set_ground_init_dim.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    height_min,
                    height_max,
                    width_min,
                    width_max,
                )
            }
        }

        pub fn set_ground_init_thresh(
            &mut self,
            x_min: f32,
            x_max: f32,
            y_min: f32,
            y_max: f32,
            z_min: f32,
            z_max: f32,
            nz_min: f32,
        )
        {
            let ptr = self.handler.get();
            println!(
                "set_ground_init_dim: {}, {}, {}, {}, {}, {} ",
                x_min, x_max, y_min, y_max, z_min, z_max
            );

            unsafe {
                ptr.set_ground_init_thresh.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    z_min,
                    z_max,
                    nz_min,
                )
            }
        }

        pub fn filter_ground(&mut self, output_mode: u32) -> (*mut f32, u64)
        {
            let ptr = self.handler.get();

            let ret = unsafe {
                ptr.filter_ground.unwrap()(ptr.deref() as *const _ as *mut _, output_mode)
            };

            if ret.is_null() {
                (null_mut(), 0 as u64)
            } else {
                unsafe { ((*ret).buffer, (*ret).float_num) }
            }
        }

        pub fn set_vertical_init_dim(
            &mut self,
            height_min: u64,
            height_max: u64,
            width_min: u64,
            width_max: u64,
        )
        {
            let ptr = self.handler.get();

            println!(
                "set_ground_init_dim: {}, {}, {}, {} ",
                height_min, height_max, width_min, width_max
            );
            unsafe {
                ptr.set_vertical_init_dim.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    height_min,
                    height_max,
                    width_min,
                    width_max,
                )
            }
        }

        pub fn set_vertical_init_thresh(
            &mut self,
            x_min: f32,
            x_max: f32,
            y_min: f32,
            y_max: f32,
            z_min: f32,
            z_max: f32,
            jx_max: f32,
            jy_max: f32,
            jz_max: f32,
        )
        {
            let ptr = self.handler.get();
            println!(
                "set_ground_init_dim: {}, {}, {}, {}, {}, {} ",
                x_min, x_max, y_min, y_max, z_min, z_max
            );

            unsafe {
                ptr.set_vertical_init_thresh.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    z_min,
                    z_max,
                    jx_max,
                    jy_max,
                    jz_max,
                )
            }
        }

        pub fn filter_vertical(&mut self, output_mode: u32) -> (*mut f32, u64)
        {
            let ptr = self.handler.get();

            let ret = unsafe {
                ptr.filter_vertical.unwrap()(ptr.deref() as *const _ as *mut _, output_mode)
            };

            if ret.is_null() {
                (null_mut(), 0 as u64)
            } else {
                unsafe { ((*ret).buffer, (*ret).float_num) }
            }
        }

        pub fn set_pallet_row(&mut self, row_high: i32, row_low: i32)
        {
            let ptr = self.handler.get();

            unsafe {
                ptr.set_pallet_row.unwrap()(ptr.deref() as *const _ as *mut _, row_high, row_low)
            }
        }

        pub fn set_pallet_thresh(
            &mut self,
            x_min: f32,
            x_max: f32,
            y_min: f32,
            y_max: f32,
            z_min: f32,
            z_max: f32,
            jx_max: f32,
            jy_max: f32,
            jz_max: f32,
        )
        {
            let ptr = self.handler.get();
            println!(
                "set_ground_init_dim: {}, {}, {}, {}, {}, {} ",
                x_min, x_max, y_min, y_max, z_min, z_max
            );

            unsafe {
                ptr.set_pallet_thresh.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    z_min,
                    z_max,
                    jx_max,
                    jy_max,
                    jz_max,
                )
            }
        }

        pub fn set_ground_uncertain_thresh(
            &mut self,
            far_uncertain_z_max: f32,
            far_uncertain_x_change_min: f32,
            far_uncertain_adaptive_z_max: f32,
            far_uncertain_row: i32,
        )
        {
            let ptr = self.handler.get();

            unsafe {
                ptr.set_ground_uncertain_thresh.unwrap()(
                    ptr.deref() as *const _ as *mut _,
                    far_uncertain_z_max,
                    far_uncertain_x_change_min,
                    far_uncertain_adaptive_z_max,
                    far_uncertain_row,
                )
            }
        }
        pub fn filter_pallet(&mut self, output_mode: u32) -> (*mut f32, u64)
        {
            let ptr = self.handler.get();

            let ret = unsafe {
                ptr.filter_pallet.unwrap()(ptr.deref() as *const _ as *mut _, output_mode)
            };

            if ret.is_null() {
                (null_mut(), 0 as u64)
            } else {
                unsafe { ((*ret).buffer, (*ret).float_num) }
            }
        }
        pub fn get_pallet(&mut self, output_mode: u32) -> &[PalletInfo]
        {
            let ptr = self.handler.get();

            let recv_ptr =
                unsafe { ptr.get_pallet.unwrap()(ptr.deref() as *const _ as *mut _, output_mode) };
            self.pallets.clear();

            if !recv_ptr.is_null() {
                unsafe {
                    let recv_data =
                        slice::from_raw_parts((*recv_ptr).buffer, (*recv_ptr).pallet_num as usize);
                    for p in recv_data {
                        self.pallets.push(PalletInfo {
                            confidence: p.confidence,
                            info: p.info,
                            tx: p.tx,
                            ty: p.ty,
                            tz: p.tz,
                            roll: p.roll,
                            pitch: p.pitch,
                            yaw: p.yaw,
                        })
                    }
                }
            }

            &self.pallets
        }
    }

    impl Drop for PointcloudPalletDetector
    {
        fn drop(&mut self)
        {
            let counter = self.handler.strong_count();
            println!("base handler reference counter: {}", counter);
            if counter == 1 {
                // let ptr = &mut self.handler.lock().unwrap().ptr;
                let ptr = self.handler.get();

                unsafe { ptr.close.unwrap()(ptr.deref() as *const _ as *mut _) };
            }
        }
    }

    pub struct CalibrationParam
    {
        pub index_buffer: *mut u64,
        pub index_num: u64,
        pub program: u64,
        pub weight: f32,
        pub target_tx: f32,
        pub target_ty: f32,
        pub target_tz: f32,
        pub target_roll: f32,
        pub target_pitch: f32,
        pub target_yaw: f32,
    }

    pub fn pointcloud_calib(
        src_buffer: *mut f32,
        point_num: u64,

        params: &Vec<CalibrationParam>,
        tx: f32,
        ty: f32,
        tz: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        itx: *mut f32,
        ity: *mut f32,
        itz: *mut f32,
        iroll: *mut f32,
        ipitch: *mut f32,
        iyaw: *mut f32,
    ) -> i32
    {
        let mut params: Vec<crate::binding::PointIndex> = params
            .iter()
            .map(|p| crate::binding::PointIndex {
                src_buffer,
                point_num,
                index_buffer: p.index_buffer,
                index_num: p.index_num,
                program: p.program,
                weight: p.weight,
                target_tx: p.target_tx,
                target_ty: p.target_ty,
                target_tz: p.target_tz,
                target_roll: p.target_roll,
                target_pitch: p.target_pitch,
                target_yaw: p.target_yaw,
            })
            .collect();

        #[cfg(use_calib)]
        unsafe {
            crate::binding::pointcloud_calib(
                params.as_mut_ptr(),
                params.len() as u64,
                tx,
                ty,
                tz,
                roll,
                pitch,
                yaw,
                itx,
                ity,
                itz,
                iroll,
                ipitch,
                iyaw,
            )
        }
        0
    }

    pub fn se3_inverse(
        tx: f32,
        ty: f32,
        tz: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        itx: *mut f32,
        ity: *mut f32,
        itz: *mut f32,
        iroll: *mut f32,
        ipitch: *mut f32,
        iyaw: *mut f32,
    ) -> i32
    {
        unsafe {
            crate::binding::se3_inverse(
                tx, ty, tz, roll, pitch, yaw, itx, ity, itz, iroll, ipitch, iyaw,
            )
        }
    }

    pub fn pointcloud_norm(
        src_buffer: *mut f32,
        point_num: u64,
        index_buffer: *mut u64,
        index_num: u64,
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
    ) -> i32
    {
        unsafe {
            crate::binding::pointcloud_norm(
                src_buffer,
                point_num,
                index_buffer,
                index_num,
                vx,
                vy,
                vz,
                cx,
                cy,
                cz,
                nx,
                ny,
                nz,
                nd,
            )
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
    pub fn pointcloud_mean_filter(
        src_buffer: *mut f32,
        point_num: u64,
        dst_buffer: *mut f32,
        count: *mut u32,
        jump_max: f32,
    ) -> i32
    {
        unsafe {
            crate::binding::pointcloud_mean_filter(
                src_buffer, point_num, dst_buffer, count, jump_max,
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
