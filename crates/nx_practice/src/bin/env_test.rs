use std::env;
//https://www.thorsten-hans.com/working-with-environment-variables-in-rust/
fn main()
{
    for (n, v) in env::vars() {
        println!("{}: {}", n, v);
    }

    let pwd = env::var("PWD").expect("PWD is not set");
    println!("get pwd : {}", pwd);
}
