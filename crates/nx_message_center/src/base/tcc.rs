use super::super::binding::*;
use std::ffi::CStr;

pub struct Tcc
{
    inner: *mut std::os::raw::c_void,
}

impl Tcc
{
    pub fn new() -> Self
    {
        Self {
            inner: unsafe { tcc_build() },
        }
    }

    pub fn compile(&mut self, code: &CStr)
    {
        unsafe {
            tcc_compile(self.inner, code as *const _ as *mut _);
        }
    }
    pub fn output(&mut self)
    {
        unsafe {
            tcc_output(self.inner);
        }
    }

    pub fn get(&mut self, name: &str) -> *mut std::os::raw::c_void
    {
        let mut c_str: [u8; 100] = [0; 100];

        unsafe {
            std::ptr::copy(name.as_ptr(), c_str.as_mut_ptr(), name.len());
        }
        unsafe { tcc_get(self.inner, &c_str as *const _ as *mut _) }
    }

    pub fn add(&mut self, name: &str, value: *mut std::os::raw::c_void) -> i32
    {
        let mut c_str: [u8; 100] = [0; 100];

        unsafe {
            std::ptr::copy(name.as_ptr(), c_str.as_mut_ptr(), name.len());
        }
        unsafe { tcc_add(self.inner, &c_str as *const _ as *mut _, value) }
    }
}

impl Drop for Tcc
{
    fn drop(&mut self)
    {
        unsafe {
            tcc_drop(self.inner);
        }
    }
}

pub fn test()
{
    unsafe {
        let tcc = tcc_build();

        let mut code = "
        void hello_tcc(int a){

        }



//\0
//12
        ";
        let mut code2 = "
        int add_int(int a,int b){
             printf(\"tcc: add_int, a = %i, b = %i  \",a,b);

             return a+b;
        }



//\0
//12
        ";

        let mut bytes = code2.as_bytes().to_vec();
        *bytes.last_mut().unwrap() = 0; // Set the second byte to 0

        tcc_compile(tcc, code as *const _ as *mut _);

        tcc_compile(tcc, bytes.as_ptr() as *const _ as *mut _);

        tcc_output(tcc);

        let add_int_func = tcc_get(tcc, "add_int\0" as *const _ as *mut _);

        println!("add_int_func: {:?}", add_int_func);
        if !add_int_func.is_null() {
            type Addop = fn(i32, i32) -> i32;
            type UnsafeAddop = unsafe extern "C" fn(i32, i32) -> i32;

            // let add_int_func = add_int_func as *mut Addop;
            // let add_int_func = *add_int_func;

            let add_int_func: Addop = std::mem::transmute_copy(&add_int_func);

            let c = add_int_func(1, 2);
            println!("c = {}", c);
        }

        tcc_drop(tcc);
    }
}
