#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// #[allow(unused_variables)]
// #[allow(unused_imports)]
// include!(concat!(env!("PWD"), "/src/generated.rs"));
include!("../generated.rs");

pub(crate) mod safe_wrapper
{
    use crate::add_int;

    // use rust_practice::add_int;
    pub(crate) fn safe_add_int(a: i32, b: i32) -> i32
    {
        println!("[rust add_int]: a = {}, b = {}", a, b);
        unsafe { add_int(a, b) }
    }
}

fn main()
{
    let a = 1;
    let b = 2;
    let c = safe_wrapper::safe_add_int(a, b);
    println!("a = {}, b = {}, c = {}", a, b, c);
}
