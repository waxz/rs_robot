use crate::binding::{dds_handler_create, message_handler_t, ros_handler_create, ta_cfg_t};
use itertools::{izip, zip};
use log::{info, log};
use std::ffi::{c_void, CStr, CString};
use std::ops::{Deref, DerefMut};
use std::os::raw::{c_char, c_int};
use std::slice;
use std::sync::{Arc, Mutex};

struct SafeRosHandlerT
{
    pub ptr: message_handler_t,
}
unsafe impl Send for SafeRosHandlerT {}

use nx_common::common::types::UnsafeMutexSender;

pub struct MessageHandler
{
    // handler:Arc<Mutex<SafeRosHandlerT>>,// UnsafeMutexSender<message_handler_t>,//
    handler: UnsafeMutexSender<message_handler_t>, //
}

impl MessageHandler
{
    pub fn new(handler: &str) -> Self
    {
        let ptr = unsafe {
            match handler {
                "ros" => ros_handler_create(),
                "dds" => dds_handler_create(),
                _ => panic!("Unsupported handler {}", handler),
            }
        };
        Self {
            // handler: Arc::new(Mutex::new(SafeRosHandlerT{ptr})),
            handler: UnsafeMutexSender::new(ptr),
        }
    }

    pub fn create(&mut self, filename: &str, ta_cfg: ta_cfg_t)->bool
    {
        let filename: CString = CString::new(filename).unwrap();

        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe {
            ptr.create.unwrap()(
                ptr.deref() as *const _ as *mut _,
                filename.as_ptr(),
                &ta_cfg,
            )
        }
    }

    pub fn is_ok(&self) -> bool
    {
        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe { ptr.is_ok.unwrap()(ptr.deref() as *const _ as *mut _) }
    }

    pub fn write_data(&mut self, channel_name: &CStr, buffer: &mut [*mut c_void]) -> c_int
    {
        // // let channel_name = CString::new(channel_name).unwrap();
        // let mut channel_name_vec: [c_char; 50] = [0; 50];
        // // for (a, b) in izip!(&mut channel_name_vec, channel_name.as_bytes()) {
        // //     *a = *b as c_char;
        // // }
        // unsafe{
        //     std::ptr::copy(channel_name.as_ptr(), channel_name_vec.as_mut_ptr() as *mut _, channel_name.len());
        // }
        //
        // let channel_name_ptr = channel_name_vec.as_ptr();
        let channel_name_ptr = channel_name.as_ptr();

        // let channel_name_ptr = channel_name.as_ptr();
        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe {
            ptr.write_data.unwrap()(
                ptr.deref() as *const _ as *mut _,
                channel_name_ptr,
                buffer.as_mut_ptr(),
                buffer.len() as u32,
            )
        }
    }

    pub fn read_data_option(&mut self, channel_name: &CStr) -> Option<&[*mut c_void]>
    {
        // let mut channel_name_vec: [c_char; 50] = [0; 50];
        // unsafe{
        //     std::ptr::copy(channel_name.as_ptr(), channel_name_vec.as_mut_ptr() as *mut _, channel_name.len());
        // }
        //
        // let channel_name_ptr = channel_name_vec.as_ptr();

        let channel_name_ptr = channel_name.as_ptr();

        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        let recv_ptr =
            unsafe { ptr.read_data.unwrap()(ptr.deref() as *const _ as *mut _, channel_name_ptr) };

        let recv_valid = !recv_ptr.is_null(); // && (( unsafe{*recv_ptr}).buffer_size > 0);

        if !recv_valid {
            None
        } else {
            Some(unsafe {
                let buf =
                    slice::from_raw_parts((*recv_ptr).buffer, (*recv_ptr).buffer_size as usize);
                buf
            })
        }

        // if recv_ptr.is_null() || (( unsafe{*recv_ptr}).buffer_size == 0) {
        //     None
        // } else {
        //     Some()
        // }
    }
    pub fn read_data(&mut self, channel_name: &CStr) -> &[*mut c_void]
    {
        // let mut channel_name_vec: [c_char; 50] = [0; 50];
        // unsafe{
        //     std::ptr::copy(channel_name.as_ptr(), channel_name_vec.as_mut_ptr() as *mut _, channel_name.len());
        // }
        //
        // let channel_name_ptr = channel_name_vec.as_ptr();

        let channel_name_ptr = channel_name.as_ptr();

        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        let recv_ptr =
            unsafe { ptr.read_data.unwrap()(ptr.deref() as *const _ as *mut _, channel_name_ptr) };

        let recv_valid = !recv_ptr.is_null(); // && (( unsafe{*recv_ptr}).buffer_size > 0);

        if !recv_valid {
            unsafe { slice::from_raw_parts(0 as *mut *mut c_void, 0) }
        } else {
            unsafe { slice::from_raw_parts((*recv_ptr).buffer, (*recv_ptr).buffer_size as usize) }
        }
    }

    pub fn stop(&mut self)
    {
        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe { ptr.close.unwrap()(ptr.deref() as *const _ as *mut _) };
    }
}

impl Drop for MessageHandler
{
    fn drop(&mut self)
    {
        let counter = self.handler.strong_count();
        println!("base handler reference counter: {}", counter);
        if counter == 1 {
            // let ptr = &mut self.handler.lock().unwrap().ptr;
            let ptr = self.handler.get();

            unsafe { ptr.close.unwrap()(ptr.deref() as *const _ as *mut _) };
        }
    }
}

impl Clone for MessageHandler
{
    fn clone(&self) -> Self
    {
        Self {
            handler: self.handler.clone(),
        }
    }
}
