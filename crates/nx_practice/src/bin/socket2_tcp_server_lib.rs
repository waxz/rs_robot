use socket2::{Domain, Socket, Type};
use std::collections::VecDeque;
use std::io;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use std::sync::{Arc, Mutex, RwLock};
use std::thread;

use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::socket_server::SocketServer;

fn main()
{
    let addr = "127.0.0.1:12345".parse::<SocketAddr>().unwrap().into();

    // let mut server = Arc::new(Mutex::new(SocketServer::new(&addr)));
    let mut server = SocketServer::new(&addr);

    // SocketServer::start_listen(&mut server);

    server.listen();

    let signal_handler = SignalHandler::default();

    {
        // let mut stream_vec = Arc::clone(&server.stream_vec);
        println!("start main thread");
        while signal_handler.is_run() {
            std::thread::sleep(std::time::Duration::from_millis(10));

            for mut stream in &mut *server.get_stream() {
                let mut recv_buf = [0_u8; 5];
                let n = stream.read(&mut recv_buf).unwrap();
                if (n > 0) {
                    println!("recv data: n: {}, data:{:?}", n, &recv_buf[0..5]);
                    stream.write(&recv_buf[0..n]);
                }
            }
        }
        println!("exit main loop");
    }

    server.stop();
}
