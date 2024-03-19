// https://course.rs/basic/trait/generic.html
//https://course.rs/basic/trait/trait-object.html

use std::cell::RefCell;
use std::convert::{From, Into, TryFrom, TryInto};
use std::fmt::{Debug, Display};
use std::ops::Add;
use std::os::unix::raw::pthread_t;
use std::rc::Rc;

use num_traits::Float;

fn some_function<T, U>(t: &T, u: &U) -> i32
where
    T: Display + Debug + Clone + std::ops::Add<Output = U> + PartialOrd + Copy + TryInto<i32>,

    U: Clone + Debug + std::ops::Add<Output = T> + PartialOrd + Copy,
{
    0
}

fn add<T>(a: T) -> f32
where
    T: Into<f32> + Copy,
{
    let b: f32 = a.into();
    b * 0.1
}

fn add_two<T>(a: T, b: T) -> T
where
    T: Copy + Add<Output = T>,
{
    a + b
}

fn add_two_i32<T>(a: T, b: T) -> i32
where
    T: Copy + Add<Output = T>,
    // T: TryInto<i32>,
    // <T as TryInto<i32>>::Error: Debug,
    i32: From<T>,
{
    let a: i32 = a.try_into().unwrap();
    let b: i32 = b.try_into().unwrap();
    a + b
}

#[test]
fn test()
{
    let c = add_two(1, 2);
    add_two_i32(1_i32, 2_i32);

    println!("c = {}", c);

    let a = 1234_u16;

    let b = add(a);

    println!("b = {} ", b);

    let b = 2;
    some_function(&a, &b);

    let c: i64 = a.try_into().unwrap();
}

trait Draw
{
    fn draw(&self) -> String;
}

impl Draw for u8
{
    fn draw(&self) -> String
    {
        format!("u8: {}", *self)
    }
}

impl Draw for f64
{
    fn draw(&self) -> String
    {
        format!("f64: {}", *self)
    }
}

// 若 T 实现了 Draw 特征， 则调用该函数时传入的 Box<T> 可以被隐式转换成函数参数签名中的 Box<dyn Draw>
fn draw1(x: Box<dyn Draw>)
{
    // 由于实现了 Deref 特征，Box 智能指针会自动解引用为它所包裹的值，然后调用该值对应的类型上定义的 `draw` 方法
    x.draw();
}

fn draw2(x: &dyn Draw)
{
    x.draw();
}
fn draw3(x: &impl Draw)
{
    x.draw();
}

fn draw4<T: Draw + Display>(x: &T)
{
    x.draw();
}
fn draw5<T>(x: &T)
where
    T: Draw + Display,
{
    x.draw();
}

fn return_draw(switch: bool) -> Box<dyn Draw>
{
    if switch {
        Box::new(1.1f64)
    } else {
        Box::new(8u8)
    }
}

struct Dog
{
    v: Rc<RefCell<dyn Draw>>,
    vv: Vec<Box<dyn Draw>>,
    vv2: Vec<Rc<RefCell<dyn Draw>>>,
}
fn main()
{
    let x = 1.1f64;
    // do_something(&x);
    let y = 8u8;

    let z = Box::new(4u8);

    {}
    let mut z1 = z.clone();
    *z1 = 23;

    let zz1 = Rc::new(RefCell::new(1u8));

    let zz2 = zz1.clone();
    *zz2.borrow_mut() = 234;
    let zz3 = zz1.clone();

    let dd = Dog {
        v: zz2,
        vv: vec![Box::new(1.1_f64), Box::new(1u8), z1],
        vv2: vec![Rc::new(RefCell::new(1.0f64)), zz3],
    };
    println!("dd.v.draw {}", dd.v.borrow_mut().draw());

    for i in &dd.vv2 {
        println!("i: {}", i.borrow().draw());
    }
    for i in &dd.vv2 {
        println!("i: {}", i.borrow().draw());
    }

    println!("z: {}", z);
    println!("zz1: {:?}", zz1);

    println!("x = {}", x.draw());
    println!("y = {}", y.draw());

    // x 和 y 的类型 T 都实现了 `Draw` 特征，因为 Box<T> 可以在函数调用时隐式地被转换为特征对象 Box<dyn Draw>
    // 基于 x 的值创建一个 Box<f64> 类型的智能指针，指针指向的数据被放置在了堆上
    draw1(Box::new(x));
    // 基于 y 的值创建一个 Box<u8> 类型的智能指针
    draw1(Box::new(y));
    draw2(&x);
    draw2(&y);
    draw3(&x);
    draw3(&y);
    draw4(&x);
    draw4(&y);
    draw5(&x);
    draw5(&y);
}
