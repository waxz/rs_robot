use std::process::Command;

fn main()
{
    let mut echo_hello = Command::new("sh");
    echo_hello.arg("-c").arg("echo hello");
    let hello_1 = echo_hello.output().expect("failed to execute process");
    let hello_2 = echo_hello.output().expect("failed to execute process");
    println!("hello_1: {:?}", hello_1);
    println!("hello_2: {:?}", hello_2);
    let mut list_dir = Command::new("sh");
    list_dir.arg("-c").arg("ls ");
    let list_dir_1 = list_dir.output().expect("failed to execute process");
    println!("list_dir_1: {:?}", list_dir_1);
}
