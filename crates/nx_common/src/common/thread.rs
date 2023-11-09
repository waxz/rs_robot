use std::thread::JoinHandle;

pub struct Thread{
    handler: Option<JoinHandle<()>>,
}

impl Thread{
    pub fn new<F :FnMut() + Send + 'static>(f:F) -> Self {
        Thread{ handler: Some(std::thread::spawn( f)) }
    }
}

impl Default for Thread {
    fn default() -> Self {
        Thread{ handler: None }
    }
}

impl Drop for Thread{
    fn drop(&mut self) {
        if let Some(h) = self.handler.take(){
            println!("join thread");
            h.join();
        }else{
            println!("none thread");
        }
    }
}