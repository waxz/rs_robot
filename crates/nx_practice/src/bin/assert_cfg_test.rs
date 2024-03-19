#[cfg(debug_assertions)]
fn example()
{
    println!("Debugging enabled");
}

#[cfg(not(debug_assertions))]
fn example()
{
    println!("Debugging disabled");
}

fn main()
{
    if cfg!(debug_assertions) {
        println!("Debugging enabled");
    } else {
        println!("Debugging disabled");
    }

    #[cfg(debug_assertions)]
    println!("Debugging enabled");

    #[cfg(test)]
    println!("Test Debugging enabled");

    #[cfg(target_os = "linux")]
    println!("Linux Debugging enabled");

    #[cfg(target_os = "linux2")]
    println!("Linux2 Debugging enabled");

    // debug_assert!(2>11);
    // assert!(2>11);

    #[cfg(not(debug_assertions))]
    println!("Debugging disabled");

    example();
}
