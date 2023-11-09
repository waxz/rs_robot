use std::{thread, time::Duration};
use rumqttc::{Client, Event, MqttOptions, Packet, QoS};

use nx_common::common::signal_handler::SignalHandler;
fn main() {

    let signal_handler = SignalHandler::default();




    let mut mqttoptions = MqttOptions::new("rumqtt-sync", "test.mosquitto.org", 1883);
    mqttoptions.set_keep_alive(Duration::from_secs(5));

    let (mut client, mut connection) = Client::new(mqttoptions, 10);
    client.subscribe("demo/mqtt233", QoS::AtMostOnce).unwrap();

    let mut send_buf = [0;100];
    send_buf.iter_mut().for_each(|x|{ *x  = *x % 3_u8});

    let h = thread::spawn(move || {
        for i in 0..20 {
            client
                .publish("demo/mqtt233", QoS::AtLeastOnce, false, &send_buf[0..i]  )
                .unwrap();
            thread::sleep(Duration::from_millis(10));
        }
    });

    for (_, notification) in connection.iter().enumerate() {
        match notification.unwrap() {
            Event::Incoming(Packet::Publish(p)) => {
                println!("Received: {:?}", p.payload);
            }

            Event::Outgoing(_) => {
                // println!("Outgoing");
            }

            _ => {
                // println!("Other");
            }
        }

        if(!signal_handler.is_run()){
            break;
        }
    }
    h.join();

    println!("Exit");

}