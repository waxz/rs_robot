pub(crate) mod binding
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

pub mod base;

#[derive(Debug)]
struct Point
{
    pub x: f32,
    pub y: f32,
}
