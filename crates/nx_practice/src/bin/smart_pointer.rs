use std::cell::RefCell;
use std::ops::{Deref, DerefMut};
use std::rc::Rc;
use std::string::String;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
// use toml::Value::String;

struct C
{
    c: u32,
    v: Vec<u32>,
}

fn print_vec_mut(v: &mut Vec<f32>)
{
    println!("print_vec: {:?}", v);
}
fn print_vec(v: &Vec<f32>)
{
    println!("print_vec: {:?}", v);
}

struct Money
{
    m: f32,
}

fn main()
{
    {
        let v1 = Rc::new(RefCell::new(Money { m: 0.0 }));

        let v2 = v1.borrow().m;

        let v3 = v2;

        v1.borrow_mut().m = 2.0;
    }
    {
        let mut v: Rc<RefCell<Vec<f32>>> = Rc::new(RefCell::new(vec![]));

        v.borrow_mut().push(1.0);

        print_vec_mut(v.borrow_mut().deref_mut());
        print_vec(v.borrow().deref());
    }

    {
        let s1: String = "ddd".to_string();

        let s2 = &*s1;
    }
    {
        let v1 = vec![1, 2, 3, 4];
        let v2 = &*v1;
    }

    {
        let atomic_bool = AtomicBool::new(false);
        let shared_atomic_bool = Arc::new(atomic_bool);

        let term = Arc::new(AtomicBool::new(false));

        term.load(Ordering::Relaxed);

        term.store(true, Ordering::Relaxed);
    }

    {
        let mut num: Rc<RefCell<i32>> = Rc::new(RefCell::new(1));

        println!("num: {:?}", num);
        // take()
        //Takes the wrapped value, leaving Default::default() in its place
        let value = num.take();
        println!("num: {:?}, value: {}", num, value);
    }
    {
        let mut v1: Vec<Box<dyn FnMut() -> bool + 'static>> = vec![];

        let mut num: Rc<RefCell<i32>> = Rc::new(RefCell::new(1));
        let mut num_vec: Rc<RefCell<Vec<i32>>> = Rc::new(RefCell::new(vec![]));
        let mut cc = Rc::new(RefCell::new(C { c: 1, v: vec![] }));

        {
            let mut num = num.clone();
            let num_vec = num_vec.clone();
            let cc = cc.clone();

            v1.push(Box::new(move || {
                println!("1 num: {:?}", num.borrow());
                println!("2 num: {:?}", num.borrow());

                *num.borrow_mut() += 100;
                cc.borrow_mut().c = 6;

                // println!("2 num: {:?}", num.take());
                num_vec.borrow_mut().push(1);
                return true;
            }));
        }
        {
            let num = Rc::clone(&num);
            let num_vec = num_vec.clone();
            let cc = cc.clone();

            v1.push(Box::new(move || {
                println!("3 num: {:?}", num.borrow());
                *num.borrow_mut() += 1000;

                println!("4 num: {:?}", num.borrow());
                num_vec.borrow_mut().push(2);
                num.replace(num.take() + 20000);
                println!("5 num: {:?}", num.borrow());
                cc.borrow_mut().c = 4;

                return true;
            }));
        }

        for (i, j) in v1.iter_mut().enumerate() {
            j();

            if (i == 5) {}
        }

        println!("final num: {:?}", num);
        println!("final num_vec: {:?}", num_vec);

        return;
    }
}
