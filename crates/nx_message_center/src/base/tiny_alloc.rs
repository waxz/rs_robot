use super::super::binding::ta_init;
use crate::binding::{ta_alloc, ta_calloc, ta_cfg_t, ta_realloc};
use nx_common::common::types::UnsafeSender;
use std::mem::size_of;
use std::os::raw::c_void;
use std::slice;

#[derive(Copy, Clone)]
pub struct TinyAlloc
{
    pub cfg: UnsafeSender<ta_cfg_t>,
}

impl TinyAlloc
{
    pub fn new(
        buffer: *mut c_void,
        buffer_size: usize,
        max_blocks: usize,
        split_thresh: usize,
        alignment: usize,
    ) -> Self
    {
        let cfg = ta_cfg_t {
            base: buffer,
            limit: unsafe { buffer.add(buffer_size) },
            max_blocks,
            split_thresh,
            alignment,
        };

        unsafe { ta_init(&cfg) };

        Self {
            cfg: UnsafeSender::new(cfg),
        }
    }

    pub fn alloc<T>(&self, bytesize: usize) -> *mut T
    {
        let ptr = unsafe { ta_alloc(self.cfg.get(), bytesize) };
        assert!(!ptr.is_null());

        ptr as *mut T
    }

    pub fn realloc<T>(&self, ptr: *mut T, bytesize: usize) -> *mut T
    {
        let ptr = unsafe { ta_realloc(self.cfg.get(), ptr as *mut c_void, bytesize) };
        assert!(!ptr.is_null());

        ptr as *mut T
    }

    pub fn calloc<T>(&self, num: usize, bytesize: usize) -> *mut T
    {
        let ptr = unsafe { ta_calloc(self.cfg.get(), num, bytesize) };
        assert!(!ptr.is_null());

        ptr as *mut T
    }

    pub fn alloc_as_array<'a, T>(&self, num: usize) -> &'a mut [T]
    {
        let ptr = self.alloc::<T>(num * size_of::<T>());
        assert!(!ptr.is_null());

        unsafe { slice::from_raw_parts_mut(ptr, num) }
    }
    pub fn realloc_as_array<'a, T>(&self, ptr: &mut [T], num: usize) -> &'a mut [T]
    {
        let ptr = self.realloc::<T>(ptr.as_mut_ptr(), num * size_of::<T>());
        assert!(!ptr.is_null());
        unsafe { slice::from_raw_parts_mut(ptr, num) }
    }
}
