// #[macro_export]
// macro_rules! my_assert (($expression:expr) => (
//
//     struct AssertGe<const N: usize, const D: usize>;
//
//
//     impl<const N: usize, const D: usize> AssertGe<N, D> {
//     const OK: () =
//         assert!(N  >= D, "{}", format!("{}:{}:{} warning: N must be greater", file!(), line!(),column!() ) );
//     }
//
//
//
// ));

use num_traits::Float;
use static_assertions::const_assert;
use std::fmt::Display;

struct AssertGe<const N: usize, const D: usize>;

impl<const N: usize, const D: usize> AssertGe<N, D>
{
    const OK: () = assert!(N >= D, "N must be greater");
}

fn add<T>(a: T) -> f32
where
    T: Into<f32> + Copy,
{
    let b: f32 = a.into();
    b * 0.1
}

enum Assert<const COND: bool> {}

trait IsTrue {}

impl IsTrue for Assert<true> {}
fn test_template_func<T, const N: usize>(data: &mut Vec<[T; N]>)
where
    T: Float + std::fmt::Debug,
{
    #[cfg(debug_assertions)]
    // let () = AssertGe::<N, 3>::OK;
    [AssertGe::<N, 3>::OK; 1];
    // const OK: () = assert!(N >= 2, "N must be greater");
    // const_assert!( N > 2);

    // let _ = If::<{N  > 2}>::T;
    // const OK: () = assert!(N  > 2, "must be divisible");

    println!("data: {:?}", data);
}

fn main()
{
    {
        if cfg!(debug_assertions) {
            println!("Debugging enabled");
        } else {
            println!("Debugging disabled");
        }

        #[cfg(debug_assertions)]
        println!("Debugging enabled");

        #[cfg(not(debug_assertions))]
        println!("Debugging disabled");
    }

    {
        let mut v1: Vec<[f32; 3]> = vec![];
        test_template_func(&mut v1);
    }
    {
        let mut v1: Vec<[f32; 2]> = vec![];
        // test_template_func(&mut v1);
    }
}
