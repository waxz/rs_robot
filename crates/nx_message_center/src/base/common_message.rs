use crate::base::tiny_alloc::TinyAlloc;
use crate::binding::{
    HeaderString, HeaderString_alloc, HeaderString_ptr, HeaderString_set_buffer, Path, Path_alloc,
    Path_ptr, Path_set_buffer, PoseStamped, PoseStamped_alloc, PoseStamped_ptr,
    MSG_STRUCT_MAX_FRAME_ID_LEN,
};
use itertools::izip;
use nx_common::common::types::{UnsafeMutexSender, UnsafeSender};
use os::raw::c_char;
use std::ffi::{c_uchar, c_void, CStr, CString};
use std::{os, slice};

#[cfg(aaaa)]
mod common_remove
{
    use crate::binding::ROS_MSG_STRUCT_MAX_FRAME_ID_LEN;
    use std::mem::ManuallyDrop;
    use std::os::raw::c_char;

    #[derive(Debug)]

    pub struct Point
    {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[derive(Debug)]
    pub struct Quaternion
    {
        pub w: f64,
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Debug)]
    pub struct Pose
    {
        pub position: Point,
        pub orientation: Quaternion,
    }

    pub struct Twist
    {
        pub linear: Point,
        pub angular: Point,
    }

    pub struct Odometry
    {
        pub(crate) frame_id: ManuallyDrop<String>,
        pub(crate) child_frame_id: ManuallyDrop<String>,

        pub(crate) pose: Pose,
        pub(crate) pose_cov: [f64; 36],
        pub(crate) twist: Twist,
        pub(crate) twist_cov: [f64; 36],
    }
}

pub mod shared
{
    use crate::base::tiny_alloc::TinyAlloc;
    use crate::binding::{
        ta_cfg_t, HeaderString_alloc, HeaderString_ptr, HeaderString_realloc,
        HeaderString_set_buffer, Odometry_alloc, Odometry_ptr, Path_alloc, Path_ptr, Path_realloc,
        Path_set_buffer, PointCloud2_alloc, PointCloud2_ptr, PointCloud2_realloc,
        PointCloud2_set_buffer, PoseStamped_alloc, PoseStamped_ptr, Twist_alloc, Twist_ptr,
    };
    use itertools::{izip, Itertools};
    use nx_common::common::types::UnsafeSender;
    use std::ffi::CStr;
    use std::mem::ManuallyDrop;
    use std::ops::Deref;
    use std::os::raw::{c_char, c_void};
    use std::slice;
    use std::sync::OnceLock;
    use std::sync::{Arc, Mutex, MutexGuard};

    pub struct DefaultAllocatorLock;
    impl DefaultAllocatorLock
    {
        thread_local! {
            // Could add pub to make it public to whatever Foo already is public to.
             pub static ALLOCATOR: OnceLock<TinyAlloc> = OnceLock::new();
        }
    }

    //get frame_id and data, convert between [uchar] and string
    // set-()
    // get-()

    // frame id map to manually drop string
    // dynamic array should be mapped to array reference

    #[derive(Debug)]
    pub struct HeaderString
    {
        // pub inner: &'a mut HeaderStringT,
        // pub ptr: HeaderStringT_ptr,
        ptr: UnsafeSender<HeaderString_ptr>,
        cfg: Option<ta_cfg_t>,
    }

    impl<'a> HeaderString
    {
        pub fn new(size: u32, cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

            let ptr = unsafe { HeaderString_alloc(size, &cfg) };
            // let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                //inner,
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }

        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as HeaderString_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                //inner,
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }

        pub fn set_data(&mut self, data: &str)
        {
            let data_slice = data.as_bytes();
            let data_slice_len = data_slice.len() as u32;
            // println!("data_slice_len: {}", data_slice_len);
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            if data_slice_len != inner.element_size {
                unsafe {
                    //void HeaderStringT_set_buffer(HeaderStringT* t, u32_t size){
                    //     t->element_size = size;
                    //     t->buffer_size = (size + 1) * sizeof (char);
                    // }
                    HeaderString_set_buffer(inner, data_slice_len);
                }
                if data_slice_len > inner.element_size {
                    let ptr = unsafe {
                        // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

                        HeaderString_realloc(data_slice_len, inner, &self.cfg.unwrap())

                        // TinyAlloc::realloc::<HeaderStringT>(
                        //     inner as *mut _ as *mut c_void,
                        //     (inner.base_size + inner.buffer_size) as usize,
                        // )
                    };
                    self.ptr = UnsafeSender::new(ptr);
                    inner = unsafe { self.ptr.get().as_mut().unwrap() };
                    //
                    // let inner_data = unsafe {
                    //     slice::from_raw_parts_mut(
                    //         inner.data.as_mut_ptr(),
                    //         inner.element_size as usize + 1,
                    //     )
                    // };
                    // inner_data[inner.element_size as usize] = 0;
                }
            }
            // for   (x,y) in izip!( &mut self.inner.data, data_slice) {
            //     println!("map {} = {}", *x, *y);
            //     *x = *y as c_char;
            // }
            let inner_data = unsafe {
                slice::from_raw_parts_mut(inner.data.as_mut_ptr(), inner.element_size as usize + 1)
            };

            for i in 0..inner.element_size as usize {
                inner_data[i] = data_slice[i] as c_char;
            }
            inner_data[inner.element_size as usize] = 0;

            // println!("inner_data: {:?}", inner_data);
        }
        pub fn set_frame_id(&mut self, data: &str)
        {
            let data_slice = data.as_bytes();
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            // izip!() accepts iterators and/or values with IntoIterator.
            for (x, y) in izip!(&mut inner.frame_id, data_slice) {
                *x = *y as c_char;
            }
            inner.frame_id[data_slice.len()] = 0;
            // println!("self.inner.frame_id: {:?}", self.inner.frame_id);

            // self.inner.frame_id.iter_mut().enumerate().for_each(|(i,x)|{
            //     *x = data_slice[i] as c_char;
            // });
            // let char_vec:Vec<c_char> = data.bytes().map(|x|{ x as c_char}).collect();
            // self.inner.frame_id.copy_from_slice(  &char_vec[..]);
        }

        pub fn get_frame_id(&self) -> ManuallyDrop<String>
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.frame_id.as_mut_ptr() as *mut u8,
                inner.frame_id.len(),
            )
        }
        pub fn get_stamp(&self) -> u64
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner.stamp
        }
        pub fn get_data(&self) -> ManuallyDrop<String>
        {
            // let inner_data = unsafe {
            //     slice::from_raw_parts_mut(
            //         self.inner.data.as_mut_ptr(),
            //         self.inner.element_size as usize,
            //     )
            // };
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.data.as_mut_ptr() as *mut u8,
                inner.element_size as usize,
            )
        }

        pub fn get_ptr(&self) -> &HeaderString_ptr
        {
            self.ptr.get()
        }
    }
    impl Clone for HeaderString
    {
        fn clone(&self) -> Self
        {
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }
    // all zero in quaternion will make all data to be NAN
    #[derive(Debug)]
    pub struct PoseStamped
    {
        ptr: UnsafeSender<PoseStamped_ptr>,
        cfg: Option<ta_cfg_t>,
    }

    impl PoseStamped
    {
        pub fn new(cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

            let ptr = unsafe { PoseStamped_alloc(&cfg) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as PoseStamped_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }

        pub fn set_frame_id(&mut self, data: &str)
        {
            let data_slice = data.as_bytes();
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            // izip!() accepts iterators and/or values with IntoIterator.
            for (x, y) in izip!(&mut inner.frame_id, data_slice) {
                *x = *y as c_char;
            }
            inner.frame_id[data_slice.len()] = 0;
            // println!("self.inner.frame_id: {:?}",self.inner.frame_id);
        }
        pub fn get_frame_id(&mut self) -> ManuallyDrop<String>
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.frame_id.as_mut_ptr() as *mut u8,
                inner.frame_id.len(),
            )
        }
        pub fn get_stamp(&self) -> u64
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner.stamp
        }
        pub fn get_data(&self) -> &crate::binding::PoseStamped
        {
            let inner = unsafe { self.ptr.get().as_ref().unwrap() };
            inner
        }
        pub fn get_mut_data(&mut self) -> &mut crate::binding::PoseStamped
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_ptr(&self) -> &PoseStamped_ptr
        {
            self.ptr.get()
        }
    }
    impl Clone for PoseStamped
    {
        fn clone(&self) -> Self
        {
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }

    #[derive(Debug)]
    pub struct PointCloud2
    {
        ptr: UnsafeSender<PointCloud2_ptr>,
        cfg: Option<ta_cfg_t>,
    }
    impl Clone for PointCloud2
    {
        fn clone(&self) -> Self
        {
            // Odometry::from_ptr(self.ptr.ptr as *mut c_void)
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }
    impl PointCloud2
    {
        pub fn new(height: u32, width: u32, channel: u32, cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

            let ptr = unsafe { PointCloud2_alloc(height, width, channel, &cfg) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }

        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as PointCloud2_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }
        pub fn set_frame_id(&mut self, data: &str)
        {
            let data_slice = data.as_bytes();
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            // izip!() accepts iterators and/or values with IntoIterator.
            // for (x, y) in izip!(&mut inner.frame_id, data_slice) {
            //     *x = *y as c_char;
            // }
            unsafe {
                std::ptr::copy(
                    data.as_ptr(),
                    inner.frame_id.as_mut_ptr() as *mut _,
                    data.len(),
                );
            }
            // inner.frame_id[data_slice.len()] = 0;
            // println!("self.inner.frame_id: {:?}",self.inner.frame_id);
        }
        pub fn get_frame_id(&self) -> ManuallyDrop<String>
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.frame_id.as_mut_ptr() as *mut u8,
                inner.frame_id.len(),
            )
        }

        pub fn get_stamp(&self) -> u64
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner.stamp
        }

        pub fn set_stamp(&mut self, stamp: u64)
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            inner.stamp = stamp;
        }
        pub fn alloc_data(&mut self, height: u32, width: u32, channel: u32)
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            {
                let ptr = unsafe {
                    // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

                    PointCloud2_realloc(height, width, channel, inner, &self.cfg.unwrap())

                    // TinyAlloc::realloc::<PathT>(
                    //     inner as *mut _ as *mut c_void,
                    //     (inner.base_size + inner.buffer_size) as usize,
                    // )
                };
                self.ptr = UnsafeSender::new(ptr);
            }
        }

        pub fn get_data(&self) -> &[f32]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            let float_num = inner.height * inner.width * inner.channel;
            let inner_data =
                unsafe { slice::from_raw_parts_mut(inner.buffer.as_mut_ptr(), float_num as usize) };
            inner_data
        }

        pub fn get_height_width(&self) -> [u32;2]{
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            [inner.height, inner.width]
        }

        fn get_dim(&self) -> [u32; 3]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            [inner.height, inner.width, inner.channel]
        }

        pub fn get_mut_data(&mut self) -> &mut [f32]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            let float_num = inner.height * inner.width * inner.channel;

            let inner_data =
                unsafe { slice::from_raw_parts_mut(inner.buffer.as_mut_ptr(), float_num as usize) };
            inner_data
        }

        pub fn get_ptr(&self) -> &PointCloud2_ptr
        {
            self.ptr.get()
        }
    }

    #[derive(Debug)]
    pub struct Path
    {
        // pub inner: &'a mut PathT,
        // pub ptr: PathT_ptr,
        // pub frame_id: String,
        // pub data: Vec<PoseStamped<'a>>,
        ptr: UnsafeSender<Path_ptr>,
        cfg: Option<ta_cfg_t>,
    }

    impl Path
    {
        pub fn new(size: u32, cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

            let ptr = unsafe { Path_alloc(size, &cfg) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as Path_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }

        pub fn set_frame_id(&mut self, data: &str)
        {
            let data_slice = data.as_bytes();
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            // izip!() accepts iterators and/or values with IntoIterator.
            for (x, y) in izip!(&mut inner.frame_id, data_slice) {
                *x = *y as c_char;
            }
            inner.frame_id[data_slice.len()] = 0;
            // println!("self.inner.frame_id: {:?}",self.inner.frame_id);
        }
        pub fn get_frame_id(&mut self) -> ManuallyDrop<String>
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.frame_id.as_mut_ptr() as *mut u8,
                inner.frame_id.len(),
            )
        }

        pub fn get_stamp(&self) -> u64
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner.stamp
        }

        pub fn set_stamp(&mut self, stamp: u64)
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            inner.stamp = stamp;
        }

        pub fn alloc_data(&mut self, size: u32)
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            {
                let ptr = unsafe {
                    // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

                    Path_realloc(size, inner, &self.cfg.unwrap())

                    // TinyAlloc::realloc::<PathT>(
                    //     inner as *mut _ as *mut c_void,
                    //     (inner.base_size + inner.buffer_size) as usize,
                    // )
                };
                self.ptr = UnsafeSender::new(ptr);
            }
        }
        pub fn get_data(&self) -> &[crate::binding::PoseStamped]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            let inner_data = unsafe {
                slice::from_raw_parts_mut(inner.data.as_mut_ptr(), inner.element_size as usize)
            };
            inner_data
        }

        pub fn get_mut_data(&mut self) -> &mut [crate::binding::PoseStamped]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            let inner_data = unsafe {
                slice::from_raw_parts_mut(inner.data.as_mut_ptr(), inner.element_size as usize)
            };
            inner_data
        }

        pub fn get_ptr(&self) -> &Path_ptr
        {
            self.ptr.get()
        }
    }

    impl Clone for Path
    {
        fn clone(&self) -> Self
        {
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }

    #[derive(Debug)]
    pub struct Odometry
    {
        // pub inner: &'a mut OdometryT,
        pub ptr: UnsafeSender<Odometry_ptr>,
        cfg: Option<ta_cfg_t>, // pub data: common::Odometry,
    }

    impl Odometry
    {
        pub fn new(cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);

            let ptr = unsafe { Odometry_alloc(&cfg) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                // inner,
                // ptr
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }

        pub fn get_ptr(&self) -> &Odometry_ptr
        {
            self.ptr.get()
        }

        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as Odometry_ptr;
            let inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                // inner,
                // ptr
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }
        pub fn set_frame_id(&mut self, frame_id: &str, child_frame_id: &str)
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            {
                let data_slice = frame_id.as_bytes();

                // izip!() accepts iterators and/or values with IntoIterator.
                for (x, y) in izip!(&mut inner.frame_id, data_slice) {
                    *x = *y as c_char;
                }
                inner.frame_id[data_slice.len()] = 0;
            }
            {
                let data_slice = child_frame_id.as_bytes();

                // izip!() accepts iterators and/or values with IntoIterator.
                for (x, y) in izip!(&mut inner.child_frame_id, data_slice) {
                    *x = *y as c_char;
                }
                inner.child_frame_id[data_slice.len()] = 0;
            }
        }

        pub fn get_frame_id(&mut self) -> ManuallyDrop<String>
        {
            let inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.frame_id.as_mut_ptr() as *mut u8,
                inner.frame_id.len(),
            )
        }

        pub fn get_stamp(&self) -> u64
        {
            let inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner.stamp
        }
        pub fn get_child_frame_id(&mut self) -> ManuallyDrop<String>
        {
            let inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.child_frame_id.as_mut_ptr() as *mut u8,
                inner.child_frame_id.len(),
            )
        }
        pub fn get_data(&self) -> &crate::binding::Odometry
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            inner
        }
        pub fn get_mut_data(&mut self) -> &mut crate::binding::Odometry
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            inner
        }
        // pub fn get_data(&mut self) -> &mut common::Odometry {
        //     self.data = common::Odometry {
        //         frame_id: nx_common::common::string::string_from_cstr_ptr(
        //             self.inner.frame_id.as_mut_ptr() as *mut u8,
        //             self.inner.frame_id.len(),
        //         ),
        //         child_frame_id: nx_common::common::string::string_from_cstr_ptr(
        //             self.inner.child_frame_id.as_mut_ptr() as *mut u8,
        //             self.inner.child_frame_id.len(),
        //         ),
        //         pose: Pose {
        //             position: Point {
        //                 x: self.inner.pose.position.x,
        //                 y: self.inner.pose.position.y,
        //                 z: self.inner.pose.position.z,
        //             },
        //             orientation: Quaternion {
        //                 w: self.inner.pose.quaternion.w,
        //                 x: self.inner.pose.quaternion.x,
        //                 y: self.inner.pose.quaternion.y,
        //                 z: self.inner.pose.quaternion.z,
        //             },
        //         },
        //         pose_cov: self.inner.pose_cov,
        //         twist: Twist {
        //             linear: Point {
        //                 x: self.inner.twist.linear.x,
        //                 y: self.inner.twist.linear.y,
        //                 z: self.inner.twist.linear.z,
        //             },
        //             angular: Point {
        //                 x: self.inner.twist.angular.x,
        //                 y: self.inner.twist.angular.y,
        //                 z: self.inner.twist.angular.z,
        //             },
        //         },
        //         twist_cov: self.inner.twist_cov,
        //     };
        //
        //     &mut self.data
        // }
        //
    }

    impl Clone for Odometry
    {
        fn clone(&self) -> Self
        {
            // Odometry::from_ptr(self.ptr.ptr as *mut c_void)
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }

    #[derive(Debug)]
    pub struct Twist
    {
        ptr: UnsafeSender<Twist_ptr>,
        cfg: Option<ta_cfg_t>,
    }

    impl Twist
    {
        pub fn new(cfg: ta_cfg_t) -> Self
        {
            // let cfg = DefaultAllocator::ALLOCATOR.with(|x| x.get().unwrap().cfg);
            let ptr = unsafe { Twist_alloc(&cfg) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: Some(cfg),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as Twist_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
                cfg: None,
            }
        }

        pub fn get_data(&self) -> &mut crate::binding::Twist
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_mut_data(&mut self) -> &mut crate::binding::Twist
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_ptr(&self) -> &Twist_ptr
        {
            self.ptr.get()
        }
    }
    impl Clone for Twist
    {
        fn clone(&self) -> Self
        {
            // Odometry::from_ptr(self.ptr.ptr as *mut c_void)
            Self {
                ptr: self.ptr.clone(),
                cfg: self.cfg,
            }
        }
    }
}
