use socket2::{Domain, SockAddr, Socket, Type};
use std::collections::VecDeque;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

pub struct SocketServer
{
    addr: SockAddr,
    listener: TcpListener,
    pub listen_thread: Option<std::thread::JoinHandle<()>>,
    // pub thread_vec: Vec<std::thread::JoinHandle<()>>,
    // pub i32_vec: Vec<i32>,
    is_run: Arc<AtomicBool>,
    pub stream_vec: Arc<Mutex<VecDeque<TcpStream>>>,
}

impl SocketServer
{
    pub fn new(addr: &SockAddr) -> Self
    {
        let socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
        socket.bind(addr).unwrap();
        socket.listen(128).unwrap();

        SocketServer {
            addr: addr.clone(),
            listener: socket.into(),
            listen_thread: None,
            is_run: Arc::new(AtomicBool::from(false)),
            stream_vec: Arc::new(Mutex::new(Default::default())),
            // thread_vec:vec![],
            // i32_vec:vec![]
        }
    }

    pub fn new_arc(addr: &SockAddr) -> Arc<Mutex<SocketServer>>
    {
        Arc::new(Mutex::new(SocketServer::new(&addr)))
    }

    pub fn get_stream(&mut self) -> MutexGuard<'_, VecDeque<TcpStream>>
    {
        self.stream_vec.lock().unwrap()
    }

    pub fn listen(&mut self)
    {
        println!("[server]: start thread");
        let is_run = self.is_run.clone();
        is_run.store(true, Ordering::Relaxed);

        let stream_vec = Arc::clone(&self.stream_vec);

        let listener = self.listener.try_clone().unwrap();
        self.listen_thread = Some(std::thread::spawn(move || {
            while is_run.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(100));

                match listener.accept() {
                    Ok((mut stream, peer_addr)) => {
                        println!("recv incoming connection from :{}", peer_addr);
                        stream_vec.lock().unwrap().push_back(stream);

                        if stream_vec.lock().unwrap().len() > 5 {
                            stream_vec.lock().unwrap().pop_front();
                            println!("remove old connection");
                        }
                    }
                    Err(e) => {
                        println!("error");
                    }
                }
            }
            println!("[server]: thread exit");
        }));
    }

    pub fn start_listen(this: &mut Arc<Mutex<SocketServer>>)
    {
        let copy = this.clone();

        println!("[server]: start thread");
        let is_run = this.lock().unwrap().is_run.clone();
        is_run.store(true, Ordering::Relaxed);

        let stream_vec = Arc::clone(&this.lock().unwrap().stream_vec);

        let listener = this.lock().unwrap().listener.try_clone().unwrap();

        this.lock().unwrap().listen_thread = Some(std::thread::spawn(move || {
            while is_run.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(100));

                match listener.accept() {
                    Ok((mut stream, peer_addr)) => {
                        println!("recv incoming connection from :{}", peer_addr);
                        stream_vec.lock().unwrap().push_back(stream);

                        if stream_vec.lock().unwrap().len() > 5 {
                            stream_vec.lock().unwrap().pop_front();
                            println!("remove old connection");
                        }
                    }
                    Err(e) => {
                        println!("error");
                    }
                }
            }
            println!("[server]: thread exit");
        }));
    }

    pub fn stop(&mut self)
    {
        println!("[server]: stop");
        self.is_run.store(false, Ordering::Relaxed);
        let mut socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
        match socket.connect_timeout(&self.addr, std::time::Duration::from_millis(250)) {
            Ok(_) => println!("success"),
            Err(ref err) if err.kind() == std::io::ErrorKind::TimedOut => {}
            Err(err) => println!("unexpected error {}", err),
        }
    }
}

impl Drop for SocketServer
{
    fn drop(&mut self)
    {
        if let Some(handle) = self.listen_thread.take() {
            handle.join().unwrap();
        } else {
        };
    }
}
