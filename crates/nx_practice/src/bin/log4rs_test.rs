use log::{debug, error, info, log, warn};
use log4rs;

fn main()
{
    log4rs::init_file("config/log4rs.yaml", Default::default()).unwrap();

    info!("booting up");
    for i in 0..1000 {
        info!("{}", i);
        info!("info");
        warn!("warn");
        error!("error");
    }

    // ...
}
