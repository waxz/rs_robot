#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// #[allow(unused_variables)]
// #[allow(unused_imports)]
// include!(concat!(env!("PWD"), "/src/generated.rs"));
include!("../generated.rs");

type Addop = fn(i32, i32) -> i32;
type UnsafeAddop = unsafe extern "C" fn(i32, i32) -> i32;

pub(crate) mod safe_wrapper
{
    use crate::{add_int, Addop};

    // use rust_practice::add_int;
    pub fn safe_add_int(a: i32, b: i32) -> i32
    {
        println!("[rust safe_add_int]: a = {}, b = {}", a, b);
        unsafe { add_int(a, b) }
    }
    pub fn safe_add_int_func(a:i32, b:i32, f:Addop ){
        println!("[rust safe_add_int_func]: a = {}, b = {}", a, b);
        f(a,b);

    }


}

use nx_ros::ros::tcc;
use std::ops::Deref;

//https://stackoverflow.com/questions/46134477/how-can-i-call-a-raw-address-from-rust
//https://stackoverflow.com/questions/58416511/casting-a-function-reference-producing-an-invalid-pointer
//https://github.com/rust-lang/rust/issues/65499

pub fn foo()
{
    println!("running foo!");
}

fn test_fn_pointer()
{
    unsafe {
        let ptr1 = foo as fn() as *const ();
        println!("ptr1: {:?}", ptr1);

        let f: fn() = std::mem::transmute(ptr1);
        (f)();
    }
    unsafe {
        let ptr1 = foo as *const ();
        println!("ptr1: {:?}", ptr1);

        let f: fn() = std::mem::transmute(ptr1);
        (f)();
    }
    unsafe {
        let ptr1 = safe_wrapper::safe_add_int as *const (i32, i32);
        println!("ptr1: {:?}", ptr1);
    }
}


fn call_func<F>(a: i32, b: i32, f: F)
    where
        F: Fn(i32,i32) -> i32,
{
    f(a,b);
}

fn main()
{
    {
        // string
        let msg = "hello ffi \0 from rust";
        println!("msg = {:?}", msg);
        unsafe {
            print_msg(msg as *const _ as *mut _);
        }
    }

    {
        // string
        let msg = c"hello ffi  from rust";
        println!("msg = {:?}", msg);
        unsafe {
            print_msg(msg as *const _ as *mut _);
        }
    }
    {
        let msg :std::ffi::CString= std::ffi::CString::new("Hello, ffi!").expect("CString::new failed");
        println!("msg = {:?}", msg);
        unsafe {
            print_msg(msg.as_ptr());
        }

    }


    {
        // function
        let a = line!() as i32;
        let b = line!() as i32;

        let add_op: Addop =   safe_wrapper::safe_add_int  ;

        let unsafe_add_op: UnsafeAddop = add_int;

        let c = add_op(a, b);
        let c = unsafe{ unsafe_add_op(a, b) };
        safe_wrapper::safe_add_int_func(a,b,add_op);
        safe_wrapper::safe_add_int_func(a,b,safe_wrapper::safe_add_int);
        call_func(a,b, add_op);
        call_func(a,b, safe_wrapper::safe_add_int);

    }
    {
        // function pointer
        let ptr_add_int_1 = safe_wrapper::safe_add_int as *const ();
        let ptr_add_int_2 = &(safe_wrapper::safe_add_int as Addop);



        println!("ptr_add_int_1: {:?}",ptr_add_int_1);

        let add_int_1: Addop = unsafe { std::mem::transmute_copy(&ptr_add_int_1) };
        let add_int_2: Addop = unsafe { std::mem::transmute_copy(ptr_add_int_2) };
        let a = line!() as i32;
        let b = line!() as i32;
        let c = add_int_1(a, b);
        let c = add_int_2(a, b);
    }
    {
        // get function pointer from ffi
        let cpp_add_int = unsafe{ get_func() };
        let add_int_1: Addop = unsafe { std::mem::transmute_copy(&cpp_add_int) };
        let a = line!() as i32;
        let b = line!() as i32;
        let c = add_int_1(a, b);
    }
    {
        // send function pointer to ffi
        unsafe {
            let unsafe_add_op: UnsafeAddop = add_int;


            let a = line!() as i32;
            let b = line!() as i32;
            add_func(a, b, Some(unsafe_add_op));
        }
        unsafe {
            let unsafe_add_op: UnsafeAddop =
                std::mem::transmute_copy(&(safe_wrapper::safe_add_int as *const ()));

            let a = line!() as i32;
            let b = line!() as i32;
            add_func(a, b, Some(unsafe_add_op));
        }
    }


}
