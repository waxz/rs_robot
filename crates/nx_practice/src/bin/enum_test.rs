struct Dog {}

enum Animal
{
    Dog(Dog),
    Cat(u8),
    Fish,
}

fn main()
{
    let a = Animal::Dog(Dog {});
    let a = Animal::Cat(1);
    let a = Animal::Fish;

    match a {
        Animal::Dog(a) => {
            println!("is dog")
        }
        Animal::Cat(b) => {
            println!("is cat")
        }
        _ => {
            println!("is else")
        }
    }
}
