//https://stackoverflow.com/questions/50258359/can-a-struct-containing-a-raw-pointer-implement-send-and-be-ffi-safe
// https://users.rust-lang.org/t/force-send-of-raw-pointer/83160/3
use nx_common::common::task::TaskManager;
use std::os::raw::c_void;
use std::sync::{Arc, Mutex, MutexGuard};

struct S_i
{
    x: i8,
}

impl Clone for S_i
{
    fn clone(&self) -> Self
    {
        Self { x: self.x }
    }
}

impl Copy for S_i {}
struct Useless
{
    x: *mut i32,
}

impl Useless
{
    fn new() -> Self
    {
        Self {
            x: std::ptr::null_mut() as *mut i32,
        }
    }
}

// This is clearly safe because nothing can be done
// with a Useless
unsafe impl Send for Useless {}

// Likewise, nothing can be done with &Useless
unsafe impl Sync for Useless {}

struct Foo(*mut i32);
unsafe impl Send for Foo {}

impl Clone for Foo
{
    fn clone(&self) -> Self
    {
        Self(self.0)
    }
}
struct Boo
{
    ptr: *mut i32,
}
unsafe impl Send for Boo {}
impl Clone for Boo
{
    fn clone(&self) -> Self
    {
        Self { ptr: self.ptr }
    }
}
struct S_iptr
{
    x: Boo,
}

impl Clone for S_iptr
{
    fn clone(&self) -> Self
    {
        Self { x: self.x.clone() }
    }
}
unsafe impl Send for S_iptr {}

pub struct UnsafeSenderWrapper<T>
{
    handler: Arc<Mutex<T>>,
}
unsafe impl<T> Send for UnsafeSenderWrapper<T> {}
impl<T> UnsafeSenderWrapper<T>
{
    pub fn new(data: T) -> Self
    {
        Self {
            handler: Arc::new(Mutex::new(data)),
        }
    }
    pub fn get(&self) -> MutexGuard<'_, T>
    {
        self.handler.lock().unwrap()
    }
}
impl<T> Clone for UnsafeSenderWrapper<T>
{
    fn clone(&self) -> Self
    {
        Self {
            handler: self.handler.clone(),
        }
    }
}
pub struct RawPtri32
{
    x: UnsafeSenderWrapper<*mut i32>,
}

// move is used with copy
fn main()
{
    let mut tasks = TaskManager::default();

    let mut c = 1;
    let mut s1 = S_i { x: 1 };

    let mut v1 = vec![1, 2, 3];

    {
        let mut func_1 = move || {
            c = 2;
            s1.x = 1;
            v1[0] = 1;
            println!("f1: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
            println!("f1: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

            true
        };
        func_1();
    }
    {
        let mut func_2 = move || {
            c = 2;
            s1.x = 1;

            println!("f2: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
            println!("f2: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

            true
        };

        func_2();
    }

    {
        let mut s2 = S_iptr {
            // x: Foo(std::ptr::null_mut())//Arc::new(Mutex::new(Some()))
            x: Boo {
                ptr: std::ptr::null_mut(),
            }, //Arc::new(Mutex::new(Some()))
        };
        {
            let mut s2 = s2.clone();
            let mut func_2 = move || {
                c = 2;
                s1.x = 1;

                // s2.x   = Foo(std::ptr::null_mut());
                s2.x = Boo {
                    ptr: std::ptr::null_mut(),
                };

                println!("f2: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
                println!("f2: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

                true
            };

            func_2();
        }
        {
            let mut s2 = s2.clone();

            let mut func_2 = move || {
                c = 2;
                s1.x = 1;

                // s2.x   =Foo(std::ptr::null_mut());
                s2.x = Boo {
                    ptr: std::ptr::null_mut(),
                };
                println!("f2: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
                println!("f2: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

                true
            };

            func_2();
        }
        {
            let mut s2 = s2.clone();

            let mut func_2 = move || {
                c = 2;
                s1.x = 1;

                // s2.x   =Foo(std::ptr::null_mut());
                s2.x = Boo {
                    ptr: std::ptr::null_mut(),
                };

                println!("f2: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
                println!("f2: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

                true;
            };

            nx_common::common::thread::Thread::new(func_2);
        }
    }

    println!("c = {}", c);
    println!("f0: c ptr = {:?}", &mut c as *mut _ as *mut c_void);
    println!("f0: s1 ptr = {:?}", &mut s1 as *mut _ as *mut c_void);

    {
        let m1 = UnsafeSenderWrapper::<i32>::new(5);
        println!("m1 = {}", *m1.get());

        let mut t1 = nx_common::common::thread::Thread::default();
        let mut t2 = nx_common::common::thread::Thread::default();

        let mut p_data = 0;
        let pi = &mut p_data as *mut i32;

        let p1 = RawPtri32 {
            x: UnsafeSenderWrapper::new(pi),
        };
        {
            let m1 = m1.clone();
            t1 = nx_common::common::thread::Thread::new(move || {
                println!("p1 = {:?}", p1.x.get());
                unsafe {
                    **p1.x.get() = 1003;
                }

                *m1.get() += 1;
                println!("t1 m1 = {}", *m1.get());
            });
        }
        {
            let m1 = m1.clone();

            t2 = nx_common::common::thread::Thread::new(move || {
                *m1.get() += 10;
                println!("t2 m1 = {}", *m1.get());
            });
        }

        *m1.get() += 100;

        println!("t0 m1 = {}", *m1.get());

        println!("p_data: {}", p_data);
    }
}
