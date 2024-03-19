use gltf::Node;
use nx_ros::ros::tiny_alloc;
use std::mem::size_of;
use std::os::raw::c_void;
use std::slice;

//https://users.rust-lang.org/t/how-to-mutably-borrow-elements-in-a-vec/70962/3

//https://stackoverflow.com/questions/59749412/why-does-my-static-variable-get-dis-aligned-on-running
//note: TA_BUFFER is not aligned
static mut TA_BUFFER: [u8; 1024 * 1000] = [0; 1024 * 1000];

fn GetElement<T>(data: &T) -> &mut T
{
    unsafe { (data as *const T as *mut T).as_mut().unwrap() }
}

#[derive(Debug)]
struct Point
{
    pub x: f32,
    pub y: f32,
}
fn main()
{
    tiny_alloc::TinyAlloc::init(unsafe { &mut TA_BUFFER }, 512, 16, 8);

    {
        let buf_i32_num = 10;

        let buf_i32_1 = tiny_alloc::TinyAlloc::alloc(buf_i32_num * 4) as *mut i32;

        let mut buf_i32_1_array = unsafe { slice::from_raw_parts_mut(buf_i32_1, buf_i32_num) };
        buf_i32_1_array[0] = 11;

        buf_i32_1_array[9] = 22;
        buf_i32_1_array[8] = 44;

        println!("buf_i32_1_array: {:?}", buf_i32_1_array);
        let buf_i32_2 =
            tiny_alloc::TinyAlloc::realloc(buf_i32_1 as *mut c_void, (buf_i32_num - 1) * 4)
                as *mut i32;
        let mut buf_i32_2_array =
            unsafe { slice::from_raw_parts_mut(buf_i32_2, (buf_i32_num - 1)) };
        println!("buf_i32_2_array: {:?}", buf_i32_2_array);
        println!("buf_i32_1: {:?}, buf_i32_2: {:?}", buf_i32_1, buf_i32_2);
    }
    let buf_i32_num = 10;
    let buf_i32_1 = tiny_alloc::TinyAlloc::alloc(buf_i32_num * 4) as *mut i32;

    let buf_i32_2 = tiny_alloc::TinyAlloc::alloc::<i32>(buf_i32_num * 4);
    let buf_f32_1 = tiny_alloc::TinyAlloc::alloc::<f32>(buf_i32_num * 4);
    let buf_f32_2 = tiny_alloc::TinyAlloc::alloc::<f32>(buf_i32_num * 4);

    // overlap array
    let mut buf_i32_1 = unsafe { slice::from_raw_parts_mut(buf_i32_1, buf_i32_num + 5) };
    let mut buf_i32_2 = unsafe { slice::from_raw_parts_mut(buf_i32_2, buf_i32_num) };

    let mut buf_f32_1 = unsafe { slice::from_raw_parts_mut(buf_f32_1, buf_i32_num + 5) };
    let mut buf_f32_2 = unsafe { slice::from_raw_parts_mut(buf_f32_2, buf_i32_num) };

    let mut buf_point_1 = tiny_alloc::TinyAlloc::alloc_as_array::<Point>(10);
    let mut buf_point_2 = tiny_alloc::TinyAlloc::alloc_as_array::<Point>(10);

    for (i, f) in buf_i32_1.iter_mut().enumerate() {
        *f = i as i32;
    }
    for (i, f) in buf_f32_1.iter_mut().enumerate() {
        *f = (i as f32) + 0.5;
    }

    for (i, p) in buf_point_1.iter_mut().enumerate() {
        p.x = i as f32 + 0.1;
        p.y = i as f32 + 0.5;
    }

    println!("buf_i32_1: {:?}", buf_i32_1);
    println!("buf_i32_2: {:?}", buf_i32_2);
    println!("buf_f32_1: {:?}", buf_f32_1);
    println!("buf_f32_2: {:?}", buf_f32_2);
    println!("buf_point_1: {:?}", buf_point_1);

    let ptr_point_raw: *mut Point = tiny_alloc::TinyAlloc::alloc::<Point>(size_of::<Point>());

    unsafe {
        (*ptr_point_raw).x = 1.0;
        (*ptr_point_raw).y = 1.0;
        println!("ptr_point_raw: {:?}", *ptr_point_raw);
    }
    let point_ref = unsafe { ptr_point_raw.as_mut() }.unwrap();
    println!("point_ref: {:?}", point_ref);
    point_ref.x = 2.0;
    point_ref.y = 2.0;
    unsafe {
        println!("ptr_point_raw: {:?}", *ptr_point_raw);
    }

    {
        let ptr_point_raw_array = tiny_alloc::TinyAlloc::alloc_as_array::<Point>(2);

        ptr_point_raw_array[0].x = 1.0;
        ptr_point_raw_array[0].y = 1.0;

        println!("ptr_point_raw_array: {:?}", ptr_point_raw_array);

        let ptr_point_raw_array_0: &mut _ = &mut ptr_point_raw_array[0];
        ptr_point_raw_array_0.x = 2.0;
        ptr_point_raw_array_0.y = 2.0;
        let ptr_point_raw_array_1: &mut _ = &mut (ptr_point_raw_array[1]);
        ptr_point_raw_array_1.x = 3.0;
        ptr_point_raw_array_1.y = 3.0;
        // println!("ptr_point_raw_array_0: {:?}",ptr_point_raw_array_0);
        // println!("ptr_point_raw_array_1: {:?}",ptr_point_raw_array_1);
    }
    {
        let ptr_point_raw_array = tiny_alloc::TinyAlloc::alloc_as_array::<Point>(2);
        let ptr_point_raw_array_0: *mut Point = &mut ptr_point_raw_array[0] as *mut _;
        let ptr_point_raw_array_1: *mut Point = &mut ptr_point_raw_array[1]; //as *mut _;
        let p0 = unsafe { ptr_point_raw_array_0.as_mut() }.unwrap();
        let p0 = unsafe { (&mut ptr_point_raw_array[0] as *mut Point).as_mut() }.unwrap();
        let p0_1 = unsafe { (&mut ptr_point_raw_array[0] as *mut Point).as_mut() }.unwrap();

        let p1 = unsafe { ptr_point_raw_array_1.as_mut() }.unwrap();
        let p1 = unsafe { (&mut ptr_point_raw_array[1] as *mut Point).as_mut() }.unwrap();
        // let p1 = GetElement::<Point>(ptr_point_raw_array,1);

        p0.x = 1.0;
        p0.y = 1.0;

        p1.x = 2.0;
        p1.y = 2.0;

        p0_1.x = 3.0;

        println!("p0: {:?}", p0);
        println!("p1: {:?}", p1);

        println!("ptr_point_raw_array: {:?}", ptr_point_raw_array);
    }
}
