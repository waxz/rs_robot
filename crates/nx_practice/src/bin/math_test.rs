use num_traits::Float;
use nx_common::common::bench::Bench;
use nx_common::common::math::{cross_product_2d, dot_product_2d};
use nx_common::{bench, log};
use std::time::SystemTime;

pub fn test_cross_product_2d(u: &[f32; 2], v: &[f32; 2]) -> f32
{
    u[0] * v[1] - u[1] * v[0]
}

fn main()
{
    let mut n1 = 0.2_f32;
    let mut n2 = -0.3_f32;

    let mut b1 = true;

    n1.copysign(n2);

    log!("n1: {}", n1);
    n1.is_sign_negative();

    let v1: [f32; 2] = [0.0, 1.0];
    let v2: [f32; 2] = [0.0, 1.0];

    log!("dot_product_2d : {}", dot_product_2d(&v1, &v2));
    log!("cross_product_2d : {}", cross_product_2d(&v1, &v2));
    let v1: [f64; 2] = [0.0, 1.0];
    let v2: [f64; 2] = [0.0, 1.0];

    log!("dot_product_2d : {}", dot_product_2d(&v1, &v2));
    log!("cross_product_2d : {}", cross_product_2d(&v1, &v2));

    bench!(
        {
            let u: [f32; 2] = [0.0, 1.0];
            let v: [f32; 2] = [0.0, 1.0];
            let c = dot_product_2d(&u, &v);
        },
        10
    );

    //
    bench!(
        {
            let u: [f32; 2] = [0.0, 1.0];
            let v: [f32; 2] = [0.0, 1.0];
            let c = u[0] * v[1] - u[1] * v[0];
        },
        10
    );
    bench!(
        {
            let u: [f32; 2] = [0.0, 1.0];
            let v: [f32; 2] = [0.0, 1.0];
            let c = u[0];
        },
        10
    );
}
