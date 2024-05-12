//https://course.rs/basic/trait/advance-trait.html#%E5%9C%A8%E5%A4%96%E9%83%A8%E7%B1%BB%E5%9E%8B%E4%B8%8A%E5%AE%9E%E7%8E%B0%E5%A4%96%E9%83%A8%E7%89%B9%E5%BE%81newtype

use std::ops::Deref;
use std::sync::{Arc, Mutex, MutexGuard};

pub struct UnsafeMutexSender<T: Clone>(Arc<Mutex<T>>);

unsafe impl<T: Clone> Send for UnsafeMutexSender<T> {}
impl<T: Clone> UnsafeMutexSender<T>
{
    pub fn new(data: T) -> Self
    {
        Self(Arc::new(Mutex::new(data)))
    }
    pub fn get(&self) -> MutexGuard<'_, T>
    {
        self.0.lock().unwrap()
    }
    pub fn strong_count(&self) -> usize
    {
        Arc::strong_count(&self.0)
    }
}
impl<T: Clone> Clone for UnsafeMutexSender<T>
{
    fn clone(&self) -> Self
    {
        Self(self.0.clone())
    }
}

#[derive(Debug, Copy)]
pub struct UnsafeSender<T: Clone>(T);

unsafe impl<T: Clone> Send for UnsafeSender<T> {}
impl<T: Clone> UnsafeSender<T>
{
    pub fn new(data: T) -> Self
    {
        Self(data)
    }
    pub fn get(&self) -> &T
    {
        &self.0
    }
    pub fn get_mut(&mut self) -> &mut T
    {
        &mut self.0
    }
}

impl<T: Clone> Clone for UnsafeSender<T>
{
    fn clone(&self) -> Self
    {
        Self(self.0.clone())
    }
}
