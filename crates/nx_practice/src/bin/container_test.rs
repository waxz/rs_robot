use itertools::{izip, Itertools};
use std::cmp::Ordering;
use std::mem::ManuallyDrop;
use std::ops::Deref;

use nx_common::common::bench::Bench;
use nx_common::{bench, log};

#[derive(Debug)]
struct Point
{
    x: f32,
    m: i32,
}

fn string_from_ptr(data: &mut [u8]) -> ManuallyDrop<String>
{
    let pos_terminal = data
        .iter_mut()
        .position(|x| *x == 0)
        .unwrap_or((data.len()));
    ManuallyDrop::new(unsafe {
        String::from_raw_parts(data.as_mut_ptr(), pos_terminal, data.len())
    })
}

#[test]
fn test_collect()
{
    let mut v1 = vec![0; 20];

    println!("v1 ptr : {:?}, data : {:?}", v1.as_ptr(), v1);

    for n in 10..25 {
        v1.resize(n, 0);
        println!(
            "before izip: v1 resize {}, ptr : {:?}, data : {:?}",
            n,
            v1.as_ptr(),
            v1
        );

        for (i, j) in izip!(&mut v1, (1..n)) {
            *i = j;
        }
        println!(
            "after izip: v1 resize {}, ptr : {:?}, data : {:?}",
            n,
            v1.as_ptr(),
            v1
        );
    }

    println!("v1 ptr : {:?}, data : {:?}", v1.as_ptr(), v1);

    v1 = (1..5).into_iter().map(|x| x + 10).collect();

    println!("v1 ptr : {:?}, data : {:?}", v1.as_ptr(), v1);
}

fn main()
{
    {
        let mut v1: Vec<i32> = vec![0; 10];
        let mut v2: Vec<i32> = vec![1; 10];
        // v1.extend(&v2);

        v1.append(&mut v2);

        println!("v1: {:?}", v1);
        println!("v2: {:?}", v2);
    }

    let mut v1 = vec![1, 2, 3, 4, 5, 6];

    println!("v1 = {:?}", v1);

    v1.retain(|x| *x != 3);
    println!("v1 = {:?}", v1);

    let mut v2: Vec<Point> = vec![];

    for i in 1..3 {
        v2.push(Point { x: i as f32, m: 0 })
    }
    for i in 1..3 {
        v2.push(Point { x: i as f32, m: 1 })
    }
    for i in 1..3 {
        v2.push(Point {
            x: i as f32 + 0.5,
            m: 0,
        })
    }

    for i in 1..3 {
        v2.push(Point {
            x: i as f32 + 0.6,
            m: 1,
        })
    }

    println!("before sort v2: {:?}", v2);

    v2.sort_by(|a, b| {
        if (a.m == b.m) {
            return a.x.partial_cmp(&b.x).unwrap();
        } else {
            return a.m.partial_cmp(&b.m).unwrap();
        }

        Ordering::Equal
    });

    println!("after sort v2: {:?}", v2);

    {
        let mut buffer: [i8; 10] = [1; 10];
        let ptr = buffer.as_mut_ptr() as *mut _ as *mut i8;

        let base: *const u8 = buffer.first_mut().unwrap() as *mut _ as *mut u8;
        let base_first: *const u8 = buffer.first().unwrap() as *const _ as *const u8;

        // buffer.first().as_mut().unwrap() is wrong
        // but what is it
        let base_first_as_mut: *const u8 =
            buffer.first().as_mut().take().unwrap() as *const _ as *const u8;

        if let Some(x) = buffer.first().take() {
            println!("take x: {} ", x);
        }

        let limit: *const u8 = buffer.last_mut().unwrap() as *mut _ as *mut u8;
        println!("ptr: {:?}, *ptr = {}", ptr, unsafe { *ptr });
        println!("base: {:?}, *base = {:?}", base, unsafe { *base });
        println!("base_first: {:?}, *base_first = {:?}", base_first, unsafe {
            *base_first
        });
        println!(
            "base_first_as_mut: {:?}, *base_first_as_mut = {:?}",
            base_first_as_mut,
            unsafe { *base_first_as_mut }
        );

        println!("limit: {:?}, *limit = {:?}", limit, unsafe { *limit });

        println!("limit - ptr = {}", (limit as u64 - ptr as u64));
        println!(
            "limit - base_first_as_mut = {}",
            (limit as i64 - base_first_as_mut as i64)
        );
    }
    {
        use itertools::izip;
        let a = [1, 2, 3];
        let b = [4, 5, 6];
        let c = [7, 8, 9];

        // izip!() accepts iterators and/or values with IntoIterator.
        for (x, y, z) in izip!(&a, &b, &c) {}
    }

    {
        let frame_id = [66, 67, 68, 69, 70];

        let frame_id = String::from_iter(
            frame_id
                .iter()
                .take_while(|c| **c != 0)
                .map(|c| *c as u8 as char),
        );

        println!("frame_id: {}", frame_id);
    }
    {
        let mut frame_id_array = [66, 67, 68, 69, 70, 10, 10, 9];

        let p = frame_id_array
            .iter_mut()
            .position(|x| *x == 0)
            .unwrap_or((frame_id_array.len()));

        println!("p : {:?}", p);

        // // Prevent automatically dropping the String's data
        // let mut frame_id_array = std::mem::ManuallyDrop::new(frame_id_array);
        //
        // let frame_id:String = unsafe{String::from_raw_parts({frame_id_array.as_mut_ptr()},5,5)};

        let mut frame_id: ManuallyDrop<String> = std::mem::ManuallyDrop::new(unsafe {
            String::from_raw_parts({ frame_id_array.as_mut_ptr() }, p, p)
        });

        // println!("frame_id: {}",frame_id);
        frame_id_array[0] = 71;

        println!(
            "ManuallyDrop frame_id: {:?}, {:?}",
            frame_id,
            frame_id.as_bytes()
        );
        println!(
            "ManuallyDrop frame_id:  {:?}, cmp : {} , {} ",
            frame_id,
            frame_id.deref() == "GCDEF\n\n\t",
            frame_id.deref() == "aCDEF\n\n\t"
        );
    }
    {
        println!("to_string");
        let mut frame_id_array = [66_u8, 67, 68, 69, 70, 10, 10, 9];

        let frame_id = string_from_ptr(&mut frame_id_array);
        frame_id_array[0] = 71;

        println!(
            "ManuallyDrop frame_id: {:?}, {:?}",
            frame_id,
            frame_id.as_bytes()
        );
        println!(
            "ManuallyDrop frame_id:  {:?}, cmp : {} , {} ",
            frame_id,
            frame_id.deref() == "GCDEF\n\n\t",
            frame_id.deref() == "aCDEF\n\n\t"
        );
    }
    {
        use std::mem;

        unsafe {
            let s = String::from("hello");

            // Prevent automatically dropping the String's data
            let mut s = mem::ManuallyDrop::new(s);

            let ptr = s.as_mut_ptr();
            let len = s.len();
            let capacity = s.capacity();

            let s = String::from_raw_parts(ptr, len, capacity);

            assert_eq!(String::from("hello"), s);
            println!("s = {}", s.deref());
        }
    }

    bench!(
        {
            let mut x = 0;
            x += 1;
        },
        5
    );

    bench!(
        {
            let mut x = [1, 2, 3, 4, 5];

            let mut sum = 0;

            for i in &mut x {
                sum += *i;
            }
        },
        5
    );
    bench!(
        {
            let mut x = [1, 2, 3, 4, 5];

            let mut sum = 0;
            sum = x.iter().sum();
        },
        5
    );
}
