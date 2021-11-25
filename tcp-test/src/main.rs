use std::io::prelude::*;
use std::net::{TcpListener, TcpStream};

fn handle_client(mut stream: TcpStream) {
    println!("Stream from: {}", stream.local_addr().unwrap().ip());

    // ...
    //
    loop {
        let mut buf = [0; 1024];
        let len = stream.read(&mut buf).unwrap();
        println!("Got data: {:?}", &buf[0..len]);
    }
}

fn main() -> std::io::Result<()> {
    let listener = TcpListener::bind("0.0.0.0:7878")?;

    // accept connections and process them serially
    for stream in listener.incoming() {
        handle_client(stream?);
    }
    Ok(())
}
