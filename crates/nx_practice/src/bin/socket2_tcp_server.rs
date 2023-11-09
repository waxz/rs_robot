use socket2::{Domain, Socket, Type};
use std::collections::VecDeque;
use std::io;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use std::sync::{Arc, Mutex, RwLock};
use std::thread;

use nx_common::common::signal_handler::SignalHandler;

fn main() -> io::Result<()> {
    let signal_handler = SignalHandler::default();

    // Create a TCP listener bound to two addresses.
    let socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
    let addr = "127.0.0.1:12345".parse::<SocketAddr>().unwrap().into();
    socket.bind(&addr).unwrap();

    socket.listen(128)?;

    let listener: TcpListener = socket.into();

    // listener.set_nonblocking(true);

    let server_listen_thread;
    let stream_vec: Arc<Mutex<VecDeque<TcpStream>>> = Arc::new(Mutex::new(Default::default()));

    {
        let stream_vec = Arc::clone(&stream_vec);

        let signal_handler = signal_handler.clone();

        server_listen_thread = std::thread::spawn(move || {
            println!("wait for connection");
            while signal_handler.is_run() {
                match listener.accept() {
                    Ok((mut stream, peer_addr)) => {
                        println!("recv incoming connection from :{}", peer_addr);
                        stream_vec.lock().unwrap().push_back(stream);

                        if (stream_vec.lock().unwrap().len() > 5) {
                            stream_vec.lock().unwrap().pop_front();
                            println!("remove old connection");
                        }
                    }
                    Err(e) => {
                        println!("error");
                    }
                }
            }
        });
    }

    {
        let stream_vec = Arc::clone(&stream_vec);

        while signal_handler.is_run() {
            std::thread::sleep(std::time::Duration::from_millis(100));

            for mut stream in &mut *stream_vec.lock().unwrap() {
                let mut recv_buf = [0_u8; 512];
                let n = stream.read(&mut recv_buf).unwrap();
                if (n > 0) {
                    println!("recv data: n: {}, data:{:?}", n, &recv_buf[0..10]);
                    stream.write(&recv_buf[0..n]);
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }

    {
        let mut socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
        match socket.connect_timeout(&addr, Duration::from_millis(250)) {
            Ok(_) => println!("success"),
            Err(ref err) if err.kind() == io::ErrorKind::TimedOut => {}
            Err(err) => println!("unexpected error {}", err),
        }
    }
    server_listen_thread.join().unwrap();

    println!("Exit");
    Ok(())
}
