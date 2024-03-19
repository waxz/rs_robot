use std::thread::JoinHandle;
use tracing::{error, info, trace, warn};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{filter, fmt, Layer, Registry};

// clone copy
#[derive(Clone, Copy, Debug)]
struct SCopy
{
    a: i32,
}

impl SCopy
{
    pub fn new(a: i32) -> Self
    {
        info!("Scopy new");
        SCopy { a }
    }
}

impl Default for SCopy
{
    fn default() -> Self
    {
        info!("Scopy default");
        SCopy { a: 0 }
    }
}
// drop

#[derive(Debug)]
struct SDrop
{
    a: i32,
}

impl SDrop
{
    pub fn new(a: i32) -> Self
    {
        info!("SDrop new");
        SDrop { a }
    }
}

impl Default for SDrop
{
    fn default() -> Self
    {
        info!("SDrop default");
        SDrop { a: 0 }
    }
}
impl Drop for SDrop
{
    fn drop(&mut self)
    {
        info!("SDrop drop, a = {}", self.a);
    }
}

impl Clone for SDrop
{
    fn clone(&self) -> Self
    {
        SDrop { a: self.a }
    }
}

// drop is in reverse order
// drop and copy cannot exist at the same time

fn main()
{
    let file_appender = tracing_appender::rolling::hourly("./logs", "struct_test");

    let (file_writer, _guard) = tracing_appender::non_blocking(file_appender);

    // multiple appender
    // set format and filter
    let subscriber = Registry::default()
        .with(
            fmt::Layer::default()
                .with_writer(file_writer)
                .with_line_number(true)
                .with_thread_ids(true)
                .with_file(true)
                .with_filter(filter::filter_fn(|metadata| {
                    // println!("metadata:{:?}",metadata);
                    // *metadata.level() == filter::LevelFilter::ERROR
                    metadata.target().starts_with("hello")
                })),
        )
        .with(
            fmt::Layer::default()
                .with_writer(std::io::stdout)
                .with_line_number(true)
                .with_file(true)
                .with_thread_ids(true)
                .with_filter(filter::LevelFilter::INFO),
        );

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");

    let mut s1 = SCopy::default();

    let mut s2 = SCopy::default();

    let mut s3 = SCopy::new(1);
    let mut s4 = SCopy::new(2);
    let mut s5 = SCopy::new(3);

    let mut s6 = SCopy::new(4);
    let mut s7 = SCopy::new(5);
    let mut s8 = SCopy::new(6);

    let mut vs1: Vec<SCopy> = vec![];

    vs1.push(s6);
    vs1.push(s7);
    vs1.push(s8);

    let sd1 = SDrop::new(0);
    let sd2 = sd1.clone();
    let sd3 = sd2;

    {
        let mut vi1 = vec![1, 2, 3];
        let mut vi0 = vec![1, 2, 3];
        vi1 = vi0;
        let mut vi2 = [1, 2, 3];
        let mut vi3 = [1, 2, 3];
        vi2 = vi3;
        info!(vi1 = ?vi1);

        let mut vi4: Vec<_> = vi1.iter().map(|x| *x).collect();

        let vi6: &[i32] = &vi1[..3];
        info!(v6len = vi6.len());
        let mut vi5: Vec<_> = vi1.iter_mut().map(|x| x).collect();

        vi4[0] = 100;

        *(vi5[0]) = 500;

        info!(vi2 = ?vi2);
        info!(vi4 = ?vi4);
        info!(vi5 = ?vi5);
        info!(vi1 = ?vi1);
        // info!(vi5 = ?vi5);
        for i in vi1.iter() {}
        for i in vi1.iter_mut() {}

        for i in vi1.into_iter() {}

        {
            info!("====== option SCopy");

            {
                let mut op1: Option<SCopy> = Some(SCopy::new(20));
                if let Some(a) = op1.take() {
                    info!( a = ?a);
                } else {
                    info!("none");
                }
                if let Some(a) = op1.take() {
                    info!( a = ?a);
                } else {
                    info!("none");
                }
            }
            info!("====== option SCopy");
        }
        {
            info!("====== option SDrop");

            {
                let mut op1: Option<SDrop> = Some(SDrop::new(20));
                if let Some(a) = op1.take() {
                    info!( a = ?a);
                } else {
                    info!("none");
                }
                if let Some(a) = op1.take() {
                    info!( a = ?a);
                } else {
                    info!("none");
                }
            }
            info!("====== option SDrop");
        }

        {
            error!("======= mem SCopy ");
            let mut s1 = SCopy::new(23);
            let mut s2 = SCopy::new(24);
            info!("before swap s1 = {:?}, s2 = {:?}", s1, s2);

            core::mem::swap(&mut s1, &mut s2);
            info!("after swap s1 = {:?}, s2 = {:?}", s1, s2);

            info!("before take s1 = {:?}", s1);

            core::mem::take(&mut s1);

            info!("after take s1 = {:?}", s1);
            error!("======= mem SCopy ");
        }

        {
            error!("======= mem SDrop ");

            let mut s1 = SDrop::new(23);
            let mut s2 = SDrop::new(24);
            info!("before swap s1 = {:?}, s2 = {:?}", s1, s2);

            core::mem::swap(&mut s1, &mut s2);
            info!("after swap s1 = {:?}, s2 = {:?}", s1, s2);

            info!("before take s1 = {:?}", s1);

            core::mem::take(&mut s1);

            info!("after take s1 = {:?}", s1);
            error!("======= mem SDrop ");
        }
    }
}
