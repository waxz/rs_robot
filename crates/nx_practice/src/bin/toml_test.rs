use clap::Parser;
use serde::{Deserialize, Serialize};
use std::default::Default;
use std::fs;
use toml::Table;

use figment::{providers::Format, Figment};
use std::collections::HashMap;
use toml;
#[derive(Deserialize)]
struct Package
{
    name: String,
    authors: Vec<String>,
    publish: Option<bool>,
    // ... and so on ...
}

#[derive(Deserialize)]
struct Config
{
    package: Package,
    rustc: Option<String>,
    // ... and so on ...
}
//https://codingpackets.com/blog/rust-load-a-toml-file/
//https://users.rust-lang.org/t/how-to-deserialize-nested-toml-structures-into-custom-structs/65920/2
//https://toml-to-json.matiaskorhonen.fi/

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_practice/src/bin/toml_test.toml"
    )]
    filename: String,
    /// Number of times to greet
    #[arg(short, long, default_value_t = 1)]
    count: u8,
}

#[derive(Debug, Deserialize)]
struct Toys
{
    balls: u8,
    bricks: u8,
    cars: HashMap<String, Car>,
    trucks: HashMap<String, Truck>,
}

#[derive(Debug, Deserialize)]
struct Car
{
    color: String,
    speed: u8,
}

#[derive(Debug, Deserialize, Serialize)]
struct Truck
{
    identity: u8,
    load: u16,
    hp: Option<u16>,
}

impl Default for Truck
{
    fn default() -> Self
    {
        Self {
            identity: 111,
            load: 222,
            hp: Some(333),
        }
    }
}

fn main()
{
    {
        const TEXT: &str = r#"
identity = 12
load = 15
#hp = 670
"#;

        let mut truck = Truck {
            identity: 0,
            load: 0,
            hp: Some(0),
        };
        match toml::from_str::<Truck>(TEXT) {
            Ok(t) => truck = t,
            Err(e) => {
                println!("errr: {:?}", e)
            }
        }

        println!("truck: {:#?}", truck);
        if truck.identity == 0 {}

        let truck2: Truck = toml::from_str(TEXT).unwrap();
        println!("truck2: {:#?}", truck2);

        let toml_str = toml::to_string(&truck2).unwrap();
        println!("toml_str: {:#?}", toml_str);

        return;
    }
    {
        const TEXT: &str = r#"
balls = 5
bricks = 250
[cars]
  [cars.car1]
  color = "green"
  speed = 3
  [cars.car2]
  color = "red"
  speed = 10
[trucks]
  [trucks.truck1]
  load = 15
  hp = 670
  [trucks.truck2]
  load = 25
  hp = 800
"#;

        let toys: Toys = toml::from_str(TEXT).unwrap();
        println!("toys: {:#?}", toys);
    }
    let args = Args::parse();

    let filename = &args.filename;
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

    println!("content: {}", contents);

    // Use a `match` block to return the
    // file `contents` as a `Data struct: Ok(d)`
    // or handle any `errors: Err(_)`.
    let data: Table = match toml::from_str(&contents) {
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

    println!("parse data: \n{}", data);

    if data.contains_key("general") {
        let v: i32 = data
            .get("general")
            .unwrap()
            .get("version")
            .unwrap()
            .as_integer()
            .unwrap() as i32;

        let v2 = data["general"]
            .get("version")
            .unwrap()
            .as_integer()
            .unwrap();

        let v3 = (data["general"])["version"].as_integer().unwrap();
        println!("version:{:?}", v);
        println!("v2:{:?}", v2);
        println!("v3:{:?}", v3);
    }
}
