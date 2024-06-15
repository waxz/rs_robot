use nx_message_center::base::common_message::shared::HeaderString;
use nx_message_center::base::{
    common_message, message_handler, pointcloud_process, tcc, tiny_alloc,
};
use std::ffi::CStr;
use std::ops::Deref;
use std::os::raw::c_void;

const TA_BUFFER_SIZE: usize = 1024000;
static mut TA_BUFFER: [u8; 1024 * 1000] = [0; TA_BUFFER_SIZE];
type Printop = fn(*mut std::os::raw::c_char);

fn main()
{
    println!("hello ros");
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );

    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);
    {
        let allocator = allocator.clone();
        common_message::shared::DefaultAllocatorLock::ALLOCATOR.with(|x| {
            x.get_or_init(move || allocator);
            println!(
                "set nx_message_center::base::ros_message::shared::DefaultAllocator::ALLOCATOR"
            );
        });
    }

    let float_vec_size: usize = 5;

    let mut float_vec = allocator.alloc_as_array::<f32>(float_vec_size);
    float_vec[0] = 1.0;
    float_vec[float_vec_size - 1] = 1.0;
    println!("float_vec: {:?}", float_vec);

    let float_vec_size: usize = 10;
    float_vec = allocator.realloc_as_array::<f32>(float_vec, float_vec_size);
    float_vec[0] = 1.0;
    float_vec[float_vec_size - 1] = 1.0;
    println!("float_vec: {:?}", float_vec);

    let mut ros_handler: message_handler::MessageHandler =
        message_handler::MessageHandler::new("ros");
    let mut dds_handler: message_handler::MessageHandler =
        message_handler::MessageHandler::new("dds");

    let mut detector_handler: pointcloud_process::perception::PointcloudPalletDetector =
        pointcloud_process::perception::PointcloudPalletDetector::new();

    ros_handler.create("a.toml", *allocator.cfg.get());
    dds_handler.create("a.toml", *allocator.cfg.get());
    detector_handler.create("a.toml", *allocator.cfg.get());

    let mut send_status = HeaderString::new(100, *allocator.cfg.get());
    send_status.set_data("hello");
    let status_data = send_status.get_data();
    println!("status {:?}", status_data.deref());

    let mut tcc = tcc::Tcc::new();
    let code: &CStr = cr##"
#include <tcclib.h>
void print_msg(const char* msg){
printf("run in tcc: print_msg, msg = %s\\n",msg);
}

// bb
"##;
    tcc.compile(code);

    let mut rust_num: i32 = 100;
    tcc.add(
        "rust_num",
        &mut rust_num as *mut _ as *mut std::os::raw::c_void,
    );

    tcc.output();
    let print_msg = tcc.get("print_msg");
    if !print_msg.is_null() {
        let print_msg: Printop = unsafe { std::mem::transmute_copy(&print_msg) };
        print_msg(code.as_ptr() as *mut _);
    }
}
