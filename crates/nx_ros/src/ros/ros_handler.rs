use crate::ros_api::{ros_handler_t, ros_handler_t_create, ChannelBufferT, ChannelBufferT_ptr};
use itertools::{izip, zip};
use log::{info, log};
use std::ffi::{c_void, CString};
use std::ops::{Deref, DerefMut};
use std::os::raw::{c_char, c_int};
use std::slice;
use std::sync::{Arc, Mutex};

struct SafeRosHandlerT
{
    pub ptr: ros_handler_t,
}
unsafe impl Send for SafeRosHandlerT {}

use nx_common::common::types::UnsafeMutexSender;

pub struct RosHandler
{
    // handler:Arc<Mutex<SafeRosHandlerT>>,// UnsafeMutexSender<ros_handler_t>,//
    handler: UnsafeMutexSender<ros_handler_t>, //
}

impl RosHandler
{
    pub fn new() -> Self
    {
        let ptr = unsafe { ros_handler_t_create() };
        Self {
            // handler: Arc::new(Mutex::new(SafeRosHandlerT{ptr})),
            handler: UnsafeMutexSender::new(ptr),
        }
    }

    pub fn create(&mut self, filename: &str)
    {
        let filename: CString = CString::new(filename).unwrap();

        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe {
            ptr.create.unwrap()(ptr.deref() as *const _ as *mut _, filename.as_ptr());
        }
    }

    pub fn is_ok(&self) -> bool
    {
        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe { ptr.is_ok.unwrap()(ptr.deref() as *const _ as *mut _) }
    }

    pub fn write_data(&mut self, channel_name: &str, buffer: &mut [*mut c_void]) -> c_int
    {
        // let channel_name = CString::new(channel_name).unwrap();
        let mut channel_name_vec: [c_char; 50] = [0; 50];
        for (a, b) in izip!(&mut channel_name_vec, channel_name.as_bytes()) {
            *a = *b as c_char;
        }
        let channel_name_ptr = channel_name_vec.as_ptr();
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

    pub fn read_data(&mut self, channel_name: &str) -> Option<&[*mut c_void]>
    {
        let mut channel_name_vec: [c_char; 50] = [0; 50];
        for (a, b) in izip!(&mut channel_name_vec, channel_name.as_bytes()) {
            *a = *b as c_char;
        }

        let channel_name_ptr = channel_name_vec.as_ptr();

        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        let recv_ptr =
            unsafe { ptr.read_data.unwrap()(ptr.deref() as *const _ as *mut _, channel_name_ptr) };
        if recv_ptr.is_null() {
            None
        } else {
            Some(unsafe {
                let buf = slice::from_raw_parts_mut(
                    (*recv_ptr).buffer,
                    recv_ptr.read().buffer_size as usize,
                );
                buf
            })
        }
    }

    pub fn stop(&mut self)
    {
        // let ptr = &mut self.handler.lock().unwrap().ptr;
        let ptr = self.handler.get();

        unsafe { ptr.close.unwrap()(ptr.deref() as *const _ as *mut _) };
    }
}

impl Drop for RosHandler
{
    fn drop(&mut self)
    {
        let counter = self.handler.strong_count();
        println!("ros handler reference counter: {}", counter);
        if (counter == 1) {
            // let ptr = &mut self.handler.lock().unwrap().ptr;
            let ptr = self.handler.get();

            unsafe { ptr.close.unwrap()(ptr.deref() as *const _ as *mut _) };
        }
    }
}

impl Clone for RosHandler
{
    fn clone(&self) -> Self
    {
        Self {
            handler: self.handler.clone(),
        }
    }
}
