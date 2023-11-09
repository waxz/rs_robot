use std::fs;
use toml::Table;
use clap::Parser;


//https://codingpackets.com/blog/rust-load-a-toml-file/


/// Simple program to greet a person
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    filename: String,
    /// Number of times to greet
    #[arg(short, long, default_value_t = 1)]
    count: u8,
}

fn main(){
    let args = Args::parse();


    let filename = & args.filename;
    // Read the contents of the file using a `match` block
    // to return the `data: Ok(c)` as a `String`
    // or handle any `errors: Err(_)`.
    let contents = match fs::read_to_string(filename) {
        // If successful return the files text as `contents`.
        // `c` is a local variable.
        Ok(c) => c,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Could not read file `{}`", filename);
            // Exit the program with exit code `1`.
            return;
        }
    };

    println!("content: {}",contents);

    // Use a `match` block to return the
    // file `contents` as a `Data struct: Ok(d)`
    // or handle any `errors: Err(_)`.
    let data : Table= match toml::from_str(&contents) {
        // If successful, return data as `Data` struct.
        // `d` is a local variable.
        Ok(d) => d,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Unable to load data from `{}`", filename);
            // Exit the program with exit code `1`.
            return;
        }
    };

    println!("parse data: \n{}",data);

    if data.contains_key("general"){
        let v: i32 = data.get("general").unwrap().get("version").unwrap().as_integer().unwrap() as i32;

         let v2 = data["general"].get("version").unwrap().as_integer().unwrap();

        let v3 = (data["general"])["version"].as_integer().unwrap();
        println!("version:{:?}",v);
        println!("v2:{:?}",v2);
        println!("v3:{:?}",v3);

    }

}