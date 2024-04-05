pub(crate) mod ros_api
{
    #![allow(unused)]
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #[allow(unused_variables)]
    #[allow(unused_unsafe)]
    #[allow(warnings)]
    include!("./generated.rs");
    // #[allow(unused_imports)]
    // include!(concat!(env!("PWD"), "/src/generated.rs"));
}

pub mod ros;

#[derive(Debug)]
struct Point
{
    pub x: f32,
    pub y: f32,
}

#[cfg(test)]
mod test
{
    use crate::ros::ros_handler::RosHandler;
    use crate::ros::ros_message::common_remove::Pose;
    use crate::ros::ros_message::shared::{HeaderString, Odometry, Path, PoseStamped, Twist};
    use crate::ros::tiny_alloc::TinyAlloc;
    use crate::{ros_api, Point};
    use nx_common::common::thread::Thread;
    use std::ffi::c_void;
    use std::mem::size_of;
    use std::ops::Deref;
    use std::slice;

    static mut TA_BUFFER: [u8; 1024 * 1000] = [0; 1024 * 1000];

    // #[test]
    // fn headerstring_test() {
    //     TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);
    //
    //     let mut h1 = HeaderString::new(100);
    //     let mut h2 = HeaderString::new(100);
    //     let mut h3 = HeaderString::from_ptr(h2.ptr.get() as *mut c_void);
    //
    //     println!("h1.inner.element_size {:?}", h1.inner.element_size);
    //
    //     println!("h1.ptr {:?}", h1.ptr as *mut _);
    //     println!("h2.ptr {:?}", h2.ptr as *mut _);
    //     println!("h3.ptr {:?}", h3.ptr as *mut _);
    //
    //     println!("h2.ptr - h1.ptr = {}", h2.ptr as u64 - h1.ptr as u64);
    //     println!("h1 size = {}", h1.inner.base_size + h1.inner.buffer_size);
    //     println!("h3 size = {}", h3.inner.base_size + h3.inner.buffer_size);
    //     h2.set_frame_id("base_link");
    //     h2.set_frame_id("map");
    //
    //     println!("h2 get_frame_id = {}", h2.get_frame_id().deref());
    //
    //     h2.set_data("qwer  1234");
    //     println!("h2 get_data = {}", h2.get_data().deref());
    //     h2.set_data("zxcvbnm 1234567890");
    //     println!("h2 get_data = {}", h2.get_data().deref());
    //     println!("h3 get_data = {}", h3.get_data().deref());
    // }

    #[test]
    // fn path_test() {
    //     TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);
    //
    //     let mut p1 = Path::new(10);
    //
    //     p1.set_frame_id("map");
    //     println!("p1.get_frame_id() = {}", p1.get_frame_id().deref());
    //
    //     let mut p1_data = p1.get_data();
    //     for p in p1_data.iter_mut() {
    //         p.position.x = 0.1;
    //         p.position.y = 0.2;
    //         p.position.z = 0.3;
    //
    //         p.quaternion.w = 1.0;
    //         p.quaternion.x = 0.1;
    //         p.quaternion.y = 0.2;
    //         p.quaternion.z = 0.3;
    //     }
    //
    //     println!("p1_data: {:?}", p1_data);
    //
    //     let mut p1_data = p1.get_data();
    //
    //     println!("p1_data: {:?}", p1_data);
    //
    //     let mut p2 = Path::from_ptr(p1.ptr as *mut c_void);
    //     let mut p2_data = p2.get_data();
    //
    //     println!("p2_data: {:?}", p2_data);
    //
    //     println!("p2.get_frame_id() = {}", p2.get_frame_id().deref());
    // }
    #[test]
    fn odom_twist_test()
    {
        TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);

        let mut twist = Twist::new();

        let twist_data = twist.get_mut_data();

        twist_data.linear.x = 1.0;
        twist_data.linear.y = 2.0;

        println!("twist_data: {:?}", twist_data);

        let mut odom = Odometry::new();
        // let odom_data = odom.get_data();
        // println!("odom_data: {:?}", odom_data);

        {
            // let mut odom_2 = odom.clone();

            let t = nx_common::common::thread::Thread::new(move || {
                let odom_2_data = odom.get_data();
                println!("odom_data: {:?}", odom_2_data);
            });
        }
    }

    #[test]
    fn pose_test()
    {
        TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);
        let mut pose = PoseStamped::new();
        println!("pose: {:?}", pose.get_data());
    }

    #[test]
    fn test()
    {
        TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);
        let s = unsafe { ros_api::LaserScanT_create() };
        let s2 = unsafe { ros_api::LaserScanT_alloc(100) };

        let ptr_point_raw: *mut Point = TinyAlloc::alloc::<Point>(size_of::<Point>());

        unsafe {
            let TA_BUFFER_ptr = &mut TA_BUFFER as *mut _;
            let TA_BUFFER_ptr_u64 = TA_BUFFER_ptr as u64;
            println!(
                "TA_BUFFER_ptr : {:?}, {}, {}",
                TA_BUFFER_ptr,
                TA_BUFFER_ptr_u64,
                TA_BUFFER_ptr_u64 % 4
            );
            (*ptr_point_raw).x = 1.0;
            (*ptr_point_raw).y = 2.0;

            println!("ptr_point_raw {:?}", (*ptr_point_raw));

            let mut point_read = ptr_point_raw.as_mut().unwrap();
            println!("point_read {:?}", point_read);
            point_read.x = 11.0;
            point_read.y = 22.0;
            println!("point_read {:?}", point_read);
            println!("ptr_point_raw {:?}", (*ptr_point_raw));
        }
    }

    #[test]
    fn test_ros()
    {
        TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);

        let mut ros_handler = RosHandler::new();

        let toml_file = "/home/waxz/RustroverProjects/rust_practice/crates/nx_practice/src/bin/comm_config.toml";

        ros_handler.create(toml_file);

        println!("check is_ok  {}", ros_handler.is_ok());

        let mut t1 = Thread::default();

        {
            let mut ros_handler = ros_handler.clone();
            t1 = nx_common::common::thread::Thread::new(move || {
                ros_handler.is_ok();
            });
        }

        let mut t2 = Thread::default();

        {
            let mut ros_handler = ros_handler.clone();
            t2 = nx_common::common::thread::Thread::new(move || {
                ros_handler.is_ok();
            });
        }
    }

    // Odometry
    // posestamped
    // twist
    // path

    #[test]
    fn move_data()
    {
        TinyAlloc::init(unsafe { &mut TA_BUFFER }, 256, 16, 4);

        let mut odom_data = Odometry::new();
        let mut headerstring_data = HeaderString::new(100);
        let mut twist_data = Twist::new();
        let mut posestamped_data = PoseStamped::new();
        let mut path_data = Path::new(100);

        {
            let mut t1 = Thread::default();

            let mut odom_data = odom_data.clone();
            let mut headerstring_data = headerstring_data.clone();
            let mut twist_data = twist_data.clone();
            let mut posestamped_data = posestamped_data.clone();
            let mut path_data = path_data.clone();

            t1 = Thread::new(move || {
                let mut odom_data = Odometry::from_ptr(*odom_data.get_ptr() as *mut c_void);
                let mut headerstring_data =
                    HeaderString::from_ptr(*headerstring_data.get_ptr() as *mut c_void);
                let mut twist_data = Twist::from_ptr(*twist_data.get_ptr() as *mut c_void);
                let mut posestamped_data =
                    PoseStamped::from_ptr(*posestamped_data.get_ptr() as *mut c_void);
                let mut path_data = Path::from_ptr(*path_data.get_ptr() as *mut c_void);

                odom_data.get_mut_data().pose.position.x = 0.1;
                odom_data.get_mut_data().pose.position.y = 0.2;
                odom_data.get_mut_data().pose.position.z = 0.3;
                odom_data.set_frame_id("odom", "base_link");

                headerstring_data.set_frame_id("task");
                headerstring_data.set_data("finished");

                twist_data.get_mut_data().linear.x = 1.0;
                twist_data.get_mut_data().linear.y = 1.0;
                twist_data.get_mut_data().linear.z = 1.0;

                posestamped_data.get_mut_data().position.x = 1.0;
                posestamped_data.get_mut_data().position.y = 1.0;

                let path_data_poses = path_data.get_mut_data();
                println!("path_data_poses: {}", path_data_poses.len());
                path_data_poses[0].position.x = 0.1;
            });
        }
        println!(
            "odom_data: {:?}, data: {:?}",
            odom_data,
            odom_data.get_data()
        );
        println!(
            "headerstring_data: {:?}, data: {}, framed_id : {}",
            headerstring_data,
            headerstring_data.get_data().deref(),
            headerstring_data.get_frame_id().deref()
        );
        println!(
            "twist_data: {:?}, data : {:?}",
            twist_data,
            twist_data.get_data()
        );
        println!("posestamped_data: {:?}", posestamped_data);
        println!("path_data: {:?}", path_data);
    }
}
