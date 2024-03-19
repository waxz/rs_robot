use clap::builder::Str;
use std::ffi::CStr;
use std::io::Read;
use std::mem::size_of;
use std::os::raw::{c_char, c_schar, c_uchar, c_void};

//https://stackoverflow.com/questions/37421392/convert-from-a-fixed-sized-c-char-array-to-cstring

fn main()
{
    let mut v1 = [1, 2, 3, 4, 5];
    let vb = v1.as_ptr();
    let ve = v1.last_mut().unwrap() as *mut _;
    let byte_num = (ve as u64) - (vb as u64);
    let i32_num = byte_num / size_of::<i32>() as u64;

    println!(
        "vb {:?}, ve {:?}, byte_num = {}, i32_num : {}",
        vb, ve, byte_num, i32_num
    );

    let vb: *const ::std::os::raw::c_void = v1.as_ptr() as *mut _;
    let ve: *const ::std::os::raw::c_void = v1.last_mut().unwrap() as *mut _ as *mut c_void;
    let byte_num = (ve as u64) - (vb as u64);
    let i32_num = byte_num / size_of::<i32>() as u64;

    println!(
        "vb {:?}, ve {:?}, byte_num = {}, i32_num : {}",
        vb, ve, byte_num, i32_num
    );
    {
        let mut id = [0 as c_char; 50];
        for (i, j) in id.iter_mut().enumerate() {
            *j = 30 as c_char + i as c_char;
        }

        let rust_id = unsafe { CStr::from_ptr(id.as_ptr()) };
        let rust_id = rust_id.to_owned();
        let rust_id: String = String::from(rust_id.to_string_lossy());

        println!("rust_id = {:?} ", rust_id);
        println!("cmp rust_id = {:?}", String::cmp(&rust_id, &rust_id));
        id[0] += 1;
        println!("cmp id = {:?}", id.cmp(&id));
    }
    {
        let mut v1 = 12;
        v1 = 1;
        let pv1 = &mut v1 as *mut _ as *mut c_void;

        let pv2 = pv1 as *mut i32;
        let mut v2 = unsafe { pv2.as_mut().unwrap() };
        *v2 = 100;
        println!("v1: {}", v1);
    }
}
