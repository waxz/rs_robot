use std::thread::JoinHandle;

use libc;
use std::io;

pub fn get_current_cpu() -> Result<usize, io::Error>
{
    let ret = unsafe { libc::sched_getcpu() };

    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret as usize)
    }
}
pub struct Thread
{
    handler: Option<JoinHandle<()>>,
}

impl Thread
{
    pub fn new<F: FnMut() + Send + 'static>(f: F) -> Self
    {
        Thread {
            handler: Some(std::thread::spawn(f)),
        }
    }

    pub fn is_none(&self) -> bool
    {
        self.handler.is_none()
    }
}

impl Default for Thread
{
    fn default() -> Self
    {
        Thread { handler: None }
    }
}

impl Drop for Thread
{
    fn drop(&mut self)
    {
        if let Some(h) = self.handler.take() {
            println!("join thread");
            h.join();
        } else {
            println!("none thread");
        }
    }
}
