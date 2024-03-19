use std::env;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::{self, Write};
use std::ops::DerefMut;
use std::slice::{from_raw_parts, from_raw_parts_mut};

use memmap2::Mmap;

/// Output a file's contents to stdout. The file path must be provided as the first process
/// argument.
fn main()
{
    let path = "config/readme.md";

    let mut file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .open(path)
        .unwrap();
    file.set_len(20);

    let mut mmap = unsafe { Mmap::map(&file).expect("failed to map the file") };
    println!("mmap.len() = {}", mmap.len());

    let mut mut_mmap = mmap.make_mut().unwrap();
    mut_mmap.deref_mut().write_all(b"hello, world!");
    mut_mmap.deref_mut().flush().unwrap();
    // let mut buffer =  mmap.as_ptr()  ;

    // let buffer_array =unsafe{from_raw_parts_mut(buffer as *mut u8, 3)} ;
    // buffer_array[0] = 1;

    // println!("buffer_array: {:?}",buffer_array);

    // io::stdout()         .write_all(&mmap[..])  .expect("failed to output the file contents");
}
