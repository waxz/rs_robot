use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::socket_client::SocketClient;
use socket2::{Domain, Socket, Type};
use std::io;
use std::io::Read;
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

#[test]
fn connect_timeout_unrouteable()
{
    // This IP is unroutable, so connections should always time out.
    let addr = "127.0.0.1:7878".parse::<SocketAddr>().unwrap().into();

    let socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
    match socket.connect_timeout(&addr, Duration::from_millis(250)) {
        Ok(_) => println!("success"),
        Err(ref err) if err.kind() == io::ErrorKind::TimedOut => {}
        Err(err) => println!("unexpected error {}", err),
    }
    let mut input = format!("hello socket2");
    let mut buf = [0; 512];

    // socket.send(input.as_bytes()).expect("send fail");

    while true {
        if let Ok(_) = socket.send(input.as_bytes()) {
            println!("send ok");
            break;
        } else {
            println!("send fail, try reconnect");
            socket.connect(&addr);
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}

fn main() -> io::Result<()>
{
    let signal_handler = SignalHandler::default();

    let addr = "127.0.0.1:12345".parse::<SocketAddr>().unwrap().into();

    let mut client = SocketClient::new(&addr);

    let mut input = format!("hello from socket2 client");
    let mut buf = [0; 512];

    while signal_handler.is_run() {
        match client.send(input.as_bytes()) {
            Ok(_) => {}
            Err(e) => {
                println!("error: {}", e);
            }
        }
        match client.read(&mut buf) {
            Ok(_) => {}
            Err(e) => {
                println!("error: {}", e);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    println!("Exit");

    Ok(())
}
