use rand::Rng;

fn main()
{
    let mut rng = rand::thread_rng();

    for i in 0..10 {
        let n = rng.gen_range(-3.0..3.0);
        println!(" n = {}", n);
    }
}
