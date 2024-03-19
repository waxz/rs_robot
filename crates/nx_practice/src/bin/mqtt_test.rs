use base64::{alphabet, engine, Engine as _};
use base64::{engine::general_purpose, Engine as _};
use itertools::Itertools;
use rumqttc::{Client, Event, MqttOptions, Packet, QoS};
use std::slice;
use std::{thread, time::Duration};

use nx_common::common::signal_handler::SignalHandler;
fn main()
{
    let mut mqttoptions = MqttOptions::new("rumqtt-sync", "test.mosquitto.org", 1883);

    let mut mqttoptions = MqttOptions::new("rumqtt-sync", "127.0.0.1", 1883);

    mqttoptions.set_keep_alive(Duration::from_secs(5));

    let (mut client, mut connection) = Client::new(mqttoptions, 10);
    client.subscribe("demo/mqtt233", QoS::AtMostOnce).unwrap();
    client.subscribe("hello_robot", QoS::AtMostOnce).unwrap();

    let mut send_buf = [0; 100];
    send_buf.iter_mut().for_each(|x| *x = *x % 3_u8);

    let h = thread::spawn(move || {
        for i in 0..20 {
            match client.try_publish("demo/mqtt233", QoS::AtLeastOnce, false, &send_buf[0..i]) {
                Ok(o) => {}
                Err(e) => {
                    println!("publish error: {}", e)
                }
            }
            thread::sleep(Duration::from_millis(10));
        }
        println!("publish thead exit");
    });

    let mut vec_f32: Vec<f32> = vec![];
    let signal_handler = SignalHandler::default();
    while signal_handler.is_run() {
        thread::sleep(Duration::from_millis(10));
        // println!("run once");

        connection.iter().take(1).for_each(|notification| {
            match notification {
                Ok(o) => {
                    match o {
                        Event::Incoming(Packet::Publish(p)) => {
                            println!("Received from {}, payload: {:?}", p.topic, p.payload);

                            if (p.topic == "hello_robot") {
                                println!("base64 decode");
                                let mut decode_data: &Vec<u8> =
                                    &general_purpose::STANDARD_NO_PAD.decode(p.payload).unwrap();
                                let decode_data_size = decode_data.len();
                                let ptr_i16 = decode_data.as_ptr() as *mut i16;
                                let vec_i16 = unsafe {
                                    slice::from_raw_parts_mut(ptr_i16, decode_data_size / 2)
                                };
                                println!("vec_i16: {:?}", vec_i16);

                                vec_f32 = vec_i16.iter().map(|x| *x as f32 * 0.0001).collect();
                                println!("vec_f32: {:?}", vec_f32);
                            }
                        }
                        Event::Outgoing(_) => {
                            // println!("Outgoing");
                        }
                        _ => {
                            // println!("Other");
                        }
                    }
                }
                Err(e) => {}
            }
        });
    }
    println!("Exit main loop");

    h.join();

    println!("Exit");
}
