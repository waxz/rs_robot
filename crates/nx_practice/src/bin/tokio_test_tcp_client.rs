// https://medium.com/go-rust/rust-day-6-tokio-simple-tcp-client-8dde6c6bd8ea

use std::cell::{Cell, RefCell};
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;

use std::error::Error;
use std::ptr::null;

use nx_common::common::common::SignalHandler;

#[tokio::main]
pub async fn main() -> Result<(), Box<dyn Error>>
{
    let signal_handler = SignalHandler::default();

    // Open a TCP stream to the socket address.
    //
    // Note that this is the Tokio TcpStream, which is fully async.
    let mut stream = TcpStream::connect("127.0.0.1:8080").await;
    println!("created stream");

    let mut connect_error = false;

    let send_buf = "hello";
    while signal_handler.is_run() {
        std::thread::sleep(std::time::Duration::from_millis(100));

        match &mut stream {
            Ok(ref mut o) => {
                println!("connect ok and send data");
                o.write(send_buf.as_ref()).await;
                let e = o.take_error().unwrap();
                println!("connect ok, get e :{:?}", e);

                connect_error = false;
            }
            Err(ref e) => {
                println!("connect fail");
                connect_error = true;
            }
        }

        if (connect_error) {
            std::thread::sleep(std::time::Duration::from_millis(1000));
            println!("reconnect");
            stream = TcpStream::connect("127.0.0.1:8080").await;
        }
    }

    println!("Exit");
    Ok(())
}
