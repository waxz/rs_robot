use std::mem::ManuallyDrop;
use std::ops::Deref;
use std::slice;

pub fn string_from_cstr_array(data: &mut [u8]) -> ManuallyDrop<String>
{
    let pos_terminal = data
        .iter_mut()
        .position(|x| *x == 0)
        .unwrap_or((data.len()));
    ManuallyDrop::new(unsafe {
        String::from_raw_parts(data.as_mut_ptr(), pos_terminal, data.len())
    })
}
pub fn string_from_cstr_ptr(data: *mut u8, size: usize) -> ManuallyDrop<String>
{
    let data = unsafe { slice::from_raw_parts_mut(data, size) };
    string_from_cstr_array(data)
}

#[test]
fn test()
{
    {
        let mut array = [65_u8, 66, 67, 68, 69, 70, 71, 72];
        let s = string_from_cstr_array(&mut array);

        println!("s={}, {}", s.deref(), s.deref() == "ABCDEFGH");
    }

    {
        let mut array = [65_u8, 66, 67, 68, 69, 0, 71, 72];
        let s = string_from_cstr_array(&mut array);

        println!("s={}, {}", s.deref(), s.deref() == "ABCDE");
    }
    {
        let mut array = [65_i8, 66, 67, 68, 69, 70, 71, 72];
        let s = string_from_cstr_ptr(array.as_mut_ptr() as *mut u8, array.len());

        println!("s={}, {}", s.deref(), s.deref() == "ABCDEFGH");
    }
    {
        let mut array = [65_i8, 66, 67, 68, 69, 70, 0, 72];
        let s = string_from_cstr_ptr(array.as_mut_ptr() as *mut u8, array.len());

        println!("s={}, {}", s.deref(), s.deref() == "ABCDEF");
    }
    {
        let mut array = [0_i8, 66, 67, 68, 69, 70, 0, 72];
        let s = string_from_cstr_ptr(array.as_mut_ptr() as *mut u8, array.len());

        println!("s={}, {}", s.deref(), s.deref() == "");
    }
}
