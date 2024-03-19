use socket2::{Domain, SockAddr, Socket, Type};
use std::io;
use std::time::Duration;

use std::io::Read;

pub struct SocketClient
{
    addr: SockAddr,
    socket: Socket,
}

impl SocketClient
{
    pub fn new(addr: &SockAddr) -> Self
    {
        let mut socket = Socket::new(Domain::IPV4, Type::STREAM, None).unwrap();
        match socket.connect_timeout(&addr, Duration::from_millis(250)) {
            Ok(_) => println!("success"),
            Err(ref err) if err.kind() == io::ErrorKind::TimedOut => {}
            Err(err) => println!("unexpected error {}", err),
        }

        SocketClient {
            addr: addr.clone(),
            socket,
        }
    }

    pub fn reconnect(&mut self)
    {
        println!("try reconnect");
        match self
            .socket
            .connect_timeout(&self.addr, Duration::from_millis(1000))
        {
            Ok(_) => println!("success"),
            Err(ref err) if err.kind() == io::ErrorKind::TimedOut => {}
            Err(err) => println!("unexpected error {}", err),
        }
    }

    pub fn send(&mut self, buffer: &[u8]) -> std::io::Result<usize>
    {
        let ret = self.socket.send(buffer);

        match &ret {
            Ok(_) => {}
            Err(e) => {
                self.reconnect();
            }
        }
        ret
    }

    pub fn read(&mut self, mut buffer: &mut [u8]) -> std::io::Result<usize>
    {
        let ret = self.socket.read(&mut buffer);
        match &ret {
            Ok(_) => {}
            Err(e) => {
                self.reconnect();
            }
        }
        ret
    }
}
