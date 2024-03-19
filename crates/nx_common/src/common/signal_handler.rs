use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub struct SignalHandler
{
    term: Arc<AtomicBool>,
}

impl Default for SignalHandler
{
    fn default() -> Self
    {
        let s = SignalHandler {
            term: Arc::new(AtomicBool::new(false)),
        };
        signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&s.term)).unwrap();
        signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&s.term)).unwrap();

        s
    }
}

impl Clone for SignalHandler
{
    fn clone(&self) -> Self
    {
        SignalHandler {
            term: self.term.clone(),
        }
    }
}

impl SignalHandler
{
    pub fn is_run(&self) -> bool
    {
        !self.term.load(Ordering::Relaxed)
    }
    pub fn stop(&mut self)
    {
        self.term.store(true, Ordering::Relaxed)
    }
}
