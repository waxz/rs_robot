use super::super::ros_api::ta_init;
use crate::ros_api::{ta_alloc, ta_calloc, ta_realloc};
use std::mem::size_of;
use std::os::raw::c_void;
use std::slice;
pub struct TinyAlloc {}

impl TinyAlloc
{
    pub fn init(buffer: &mut [u8], heap_blocks: usize, split_thresh: usize, alignment: usize)
    {
        let mut base: *const ::std::os::raw::c_void = buffer.as_ptr() as *mut _;
        let mut base_u64 = base as usize;
        let mut unaligned_byte = base_u64 % alignment;
        if (unaligned_byte != 0) {
            println!("base: {}, unaligned_byte: {}", base_u64, unaligned_byte);
            base = unsafe { base.add(alignment - unaligned_byte) };
            base_u64 = base as usize;
            unaligned_byte = base_u64 % alignment;

            println!("base: {}, unaligned_byte: {}", base_u64, unaligned_byte);
        }

        let limit: *const ::std::os::raw::c_void =
            buffer.last_mut().unwrap() as *mut _ as *mut std::os::raw::c_void;
        println!("create memory pool, base = {:?}, limit = {:?}, total size = {:?}, base to limit byte number = {}",
                 base, limit, buffer.len(), (limit as u64) - (base as u64));

        unsafe { ta_init(base, limit, heap_blocks, split_thresh, alignment) };
    }

    pub fn alloc<T>(size: usize) -> *mut T
    {
        let ptr = unsafe { ta_alloc(size) };

        ptr as *mut T
    }

    pub fn realloc<T>(ptr: *mut c_void, num: usize) -> *mut T
    {
        let ptr = unsafe { ta_realloc(ptr, num) };
        ptr as *mut T
    }

    pub fn calloc<T>(num: usize, size: usize) -> *mut T
    {
        let ptr = unsafe { ta_calloc(num, size) };
        ptr as *mut T
    }

    pub fn alloc_as_array<'a, T>(num: usize) -> &'a mut [T]
    {
        let ptr = TinyAlloc::alloc::<T>(num * size_of::<T>());
        unsafe { slice::from_raw_parts_mut(ptr, num) }
    }
}
