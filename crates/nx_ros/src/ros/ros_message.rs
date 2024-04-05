use crate::ros::tiny_alloc::TinyAlloc;
use crate::ros_api::{
    HeaderStringT, HeaderStringT_alloc, HeaderStringT_ptr, HeaderStringT_set_buffer, PathT,
    PathT_alloc, PathT_ptr, PathT_set_buffer, PoseStampedT, PoseStampedT_alloc, PoseStampedT_ptr,
    ROS_MSG_STRUCT_MAX_FRAME_ID_LEN,
};
use itertools::izip;
use nx_common::common::types::{UnsafeMutexSender, UnsafeSender};
use os::raw::c_char;
use std::ffi::{c_uchar, c_void, CString};
use std::{os, slice};

mod common_remove
{
    use crate::ros_api::ROS_MSG_STRUCT_MAX_FRAME_ID_LEN;
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

    use crate::ros::tiny_alloc::TinyAlloc;
    use crate::ros_api::{
        HeaderStringT, HeaderStringT_alloc, HeaderStringT_ptr, HeaderStringT_set_buffer, OdometryT,
        OdometryT_alloc, OdometryT_ptr, PathT, PathT_alloc, PathT_ptr, PathT_set_buffer,
        PoseStampedT, PoseStampedT_alloc, PoseStampedT_ptr, TwistT, TwistT_alloc, TwistT_ptr,
    };
    use itertools::{izip, Itertools};
    use nx_common::common::types::UnsafeSender;
    use std::mem::ManuallyDrop;
    use std::ops::Deref;
    use std::os::raw::{c_char, c_void};
    use std::slice;
    use std::sync::{Arc, Mutex, MutexGuard};

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
        ptr: UnsafeSender<HeaderStringT_ptr>,
    }

    impl<'a> HeaderString
    {
        pub fn new(size: u32) -> Self
        {
            let ptr = unsafe { HeaderStringT_alloc(size) };
            // let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                //inner,
                ptr: UnsafeSender::new(ptr),
            }
        }

        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as HeaderStringT_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                //inner,
                ptr: UnsafeSender::new(ptr),
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
                    HeaderStringT_set_buffer(inner, data_slice_len);
                }
                if data_slice_len > inner.element_size {
                    let ptr = unsafe {
                        TinyAlloc::realloc::<HeaderStringT>(
                            inner as *mut _ as *mut c_void,
                            (inner.base_size + inner.buffer_size) as usize,
                        )
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

        pub fn get_ptr(&self) -> &HeaderStringT_ptr
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
            }
        }
    }
    // all zero in quaternion will make all data to be NAN
    #[derive(Debug)]
    pub struct PoseStamped
    {
        ptr: UnsafeSender<PoseStampedT_ptr>,
    }

    impl PoseStamped
    {
        pub fn new() -> Self
        {
            let ptr = unsafe { PoseStampedT_alloc(0) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as PoseStampedT_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                ptr: UnsafeSender::new(ptr),
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
        pub fn get_data(&self) -> &PoseStampedT
        {
            let inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_mut_data(&mut self) -> &mut PoseStampedT
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_ptr(&self) -> &PoseStampedT_ptr
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
            }
        }
    }
    #[derive(Debug)]
    pub struct Path
    {
        // pub inner: &'a mut PathT,
        // pub ptr: PathT_ptr,
        // pub frame_id: String,
        // pub data: Vec<PoseStamped<'a>>,
        ptr: UnsafeSender<PathT_ptr>,
    }

    impl Path
    {
        pub fn new(size: u32) -> Self
        {
            let ptr = unsafe { PathT_alloc(size) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as PathT_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };

            Self {
                ptr: UnsafeSender::new(ptr),
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

            unsafe {
                PathT_set_buffer(inner, size);
            }

            if size > inner.element_size {
                let ptr = unsafe {
                    TinyAlloc::realloc::<PathT>(
                        inner as *mut _ as *mut c_void,
                        (inner.base_size + inner.buffer_size) as usize,
                    )
                };
                self.ptr = UnsafeSender::new(ptr);
            }
        }
        pub fn get_data(&self) -> &[PoseStampedT]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            let inner_data = unsafe {
                slice::from_raw_parts_mut(inner.data.as_mut_ptr(), inner.element_size as usize)
            };
            inner_data
        }

        pub fn get_mut_data(&mut self) -> &mut [PoseStampedT]
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            let inner_data = unsafe {
                slice::from_raw_parts_mut(inner.data.as_mut_ptr(), inner.element_size as usize)
            };
            inner_data
        }

        pub fn get_ptr(&self) -> &PathT_ptr
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
            }
        }
    }

    #[derive(Debug)]
    pub struct Odometry
    {
        // pub inner: &'a mut OdometryT,
        pub ptr: UnsafeSender<OdometryT_ptr>,
        // pub data: common::Odometry,
    }

    impl Odometry
    {
        pub fn new() -> Self
        {
            let ptr = unsafe { OdometryT_alloc(0) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                // inner,
                // ptr
                ptr: UnsafeSender::new(ptr),
            }
        }

        pub fn get_ptr(&self) -> &OdometryT_ptr
        {
            self.ptr.get()
        }

        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as OdometryT_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                // inner,
                // ptr
                ptr: UnsafeSender::new(ptr),
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
        pub fn get_child_frame_id(&mut self) -> ManuallyDrop<String>
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            nx_common::common::string::string_from_cstr_ptr(
                inner.child_frame_id.as_mut_ptr() as *mut u8,
                inner.child_frame_id.len(),
            )
        }
        pub fn get_data(&self) -> &OdometryT
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };

            inner
        }
        pub fn get_mut_data(&mut self) -> &mut OdometryT
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
            }
        }
    }

    #[derive(Debug)]
    pub struct Twist
    {
        ptr: UnsafeSender<TwistT_ptr>,
    }

    impl Twist
    {
        pub fn new() -> Self
        {
            let ptr = unsafe { TwistT_alloc(0) };
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
            }
        }
        pub fn from_ptr(ptr: *mut c_void) -> Self
        {
            let ptr = ptr as TwistT_ptr;
            let mut inner = unsafe { ptr.as_mut().unwrap() };
            Self {
                ptr: UnsafeSender::new(ptr),
            }
        }

        pub fn get_data(&self) -> &TwistT
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_mut_data(&mut self) -> &mut TwistT
        {
            let mut inner = unsafe { self.ptr.get().as_mut().unwrap() };
            inner
        }
        pub fn get_ptr(&self) -> &TwistT_ptr
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
            }
        }
    }
}
