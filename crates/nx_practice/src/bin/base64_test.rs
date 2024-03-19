use base64::{alphabet, engine, Engine as _};
use base64::{engine::general_purpose, Engine as _};
use std::slice;
fn main()
{
    {
        let mut buffer: [i8; 100] = [10; 100];
        let alignment = 4;

        let mut base: *const u8 = buffer.as_ptr() as *mut _;
        let mut base_u64 = base as usize;
        let mut unaligned_byte = base_u64 % alignment;
        println!("base: {}, unaligned_byte: {}", base_u64, unaligned_byte);
        base = unsafe { base.add(alignment - unaligned_byte) };
        base_u64 = base as usize;
        unaligned_byte = base_u64 % alignment;

        println!("base: {}, unaligned_byte: {}", base_u64, unaligned_byte);

        let limit: *const u8 = buffer.last_mut().unwrap() as *mut _ as *mut u8;
        println!("base: {:?}, *base = {:?}", base, unsafe { *base });
        println!("limit: {:?}, *limit = {:?}", limit, unsafe { *limit });

        // println!("create memory pool, base = {:?}, limit = {:?}, total size = {:?}",
        //          base, limit, buffer.len()
        // );
        println!("create memory pool, base = {:?}, limit = {:?}, total size = {:?}, base to limit byte number = {}",
                 base, limit, buffer.len(), (limit as u64) - (base as u64)
        );
    }
    {
        let mut buffer: [i8; 4] = [1, 2, 3, 4];
        let ptr = buffer.as_mut_ptr() as *mut _ as *mut i8;

        println!("ptr: {:?}, *ptr = {}", ptr, unsafe { *ptr });

        let base: *const u8 = buffer.first_mut().unwrap() as *mut _ as *mut u8;
        let base_first: *const u8 = buffer.first().unwrap() as *const _ as *const u8;
        let base_first_as_mut: *const u8 =
            buffer.first().as_mut().unwrap() as *const _ as *const u8;

        let limit: *const u8 = buffer.last_mut().unwrap() as *mut _ as *mut u8;
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
    }
    {
        let recv_data =
        "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"



            ;

        let mut decode_data: &Vec<u8> =
            &general_purpose::STANDARD_NO_PAD.decode(recv_data).unwrap();
        let decode_data_size = decode_data.len();
        println!(
            "decode_data_size STANDARD_NO_PAD: {}, decode_data: {:?}",
            decode_data_size, decode_data
        );

        let mut decode_data: &Vec<u8> = &general_purpose::STANDARD.decode(recv_data).unwrap();
        let decode_data_size = decode_data.len();
        println!(
            "decode_data_size STANDARD       : {}, decode_data: {:?}",
            decode_data_size, decode_data
        );

        let ptr_i8 = decode_data.as_ptr() as *mut u8;
        let ptr_i16 = decode_data.as_ptr() as *mut i16;

        println!(
            "ptr_i8 = {:?}, ptr_i8[0] = {}, ptr_i8[1] = {}",
            ptr_i8,
            unsafe { *ptr_i8 },
            unsafe { *ptr_i8.add(1) }
        );

        let vec_i16 = unsafe { slice::from_raw_parts_mut(ptr_i16, decode_data_size / 2) };
        println!("vec_i16: {:?}", vec_i16);

        let real_i16 = [
            191, 0, 0, 15707, 6999, 0, 0, 0, 15707, 6999, 0, 0, 0, 0, 299, 0, 0, 15707, 6999, 0, 0,
            0, 15707, 6999, 0, 0, 0, 0, 499, 0, 0, 15707, 7199, 0, 0, 0, 15707, 7199, 0, 0, 0, 0,
            588, 0, 0, 15707, 7399, 0, 0, 0, 15707, 7399, 0, 0, 0, 0, 799, 0, 0, 15707, 7599, 0, 0,
            0, 15707, 7599, 0, 0, 0, 0, 889, 0, 0, 15707, 7599, 0, 0, 0, 15707, 7599, 0, 0, 0, 0,
            1099, 0, 0, 15707, 7799, 0, 0, 0, 15707, 7799, 0, 0, 0, 0, 1200, 0, 0, 15707, 7999, 0,
            0, 0, 15707, 7999, 0, 0, 0, 0, 1400, 0, 0, 15707, 8200, 0, 0, 0, 15707, 8200, 0, 0, 0,
            0, 1500, 0, 0, 15707, 8200, 0, 0, 0, 15707, 8200, 0, 0, 0, 0, 1699, 0, 0, 15707, 8400,
            0, 0, 0, 15707, 8400, 0, 0, 0, 0, 1800, 0, 0, 15707, 8599, 0, 0, 0, 15707, 8599, 0, 0,
            0, 0, 2001, 0, 0, 15707, 8800, 0, 0, 0, 15707, 8800, 0, 0, 0, 0, 2100, 0, 0, 15707,
            8800, 0, 0, 0, 15707, 8800, 0, 0, 0, 0, 2289, 0, 0, 15707, 8999, 0, 0, 0, 15707, 8999,
            0, 0, 0, 0, 2400, 0, 0, 15707, 9199, 0, 0, 0, 15707, 9199, 0, 0, 0, 0, 2588, 0, 0,
            15707, 9399, 0, 0, 0, 15707, 9399, 0, 0, 0, 0, 2699, 0, 0, 15707, 9399, 0, 0, 0, 15707,
            9399, 0, 0, 0, 0, 2889, 0, 0, 15707, 9599, 0, 0, 0, 15707, 9599, 0, 0, 0, 0, 3000, 0,
            0, 15707, 9799, 0, 0, 0, 15707, 9799, 0, 0, 0, 0, 3199, 0, 0, 15707, 9999, 0, 0, 0,
            15707, 9999, 0, 0, 0, 0, 3289, 0, 0, 15707, 9999, 0, 0, 0, 15707, 9999, 0, 0, 0, 0,
            3499, 0, 0, 15707, 10199, 0, 0, 0, 15707, 10199, 0, 0, 0, 0, 3588, 0, 0, 15707, 10399,
            0, 0, 0, 15707, 10399, 0, 0, 0, 0, 3798, 0, 0, 15707, 10599, 0, 0, 0, 15707, 10599, 0,
            0, 0, 0, 3889, 0, 0, 15707, 10599, 0, 0, 0, 15707, 10599, 0, 0, 0, 0, 4099, 0, 0,
            15707, 10799, 0, 0, 0, 15707, 10799, 0, 0, 0, 0, 4200, 0, 0, 15707, 10999, 0, 0, 0,
            15707, 10999, 0, 0, 0, 0, 4399, 0, 0, 15707, 11199, 0, 0, 0, 15707, 11199, 0, 0, 0, 0,
            4499, 0, 0, 15707, 11199, 0, 0, 0, 15707, 11199, 0, 0, 0, 0, 4698, 0, 0, 15707, 11400,
            0, 0, 0, 15707, 11400, 0, 0, 0, 0, 4800, 0, 0, 15707, 11600, 0, 0, 0, 15707, 11600, 0,
            0, 0, 0, 5000, 0, 0, 15707, 11800, 0, 0, 0, 15707, 11800, 0, 0, 0, 0, 5099, 0, 0,
            15707, 11800, 0, 0, 0, 15707, 11800, 0, 0, 0, 0, 5289, 0, 0, 15707, 12000, 0, 0, 0,
            15707, 12000, 0, 0, 0, 0, 5400, 0, 0, 15707, 12199, 0, 0, 0, 15707, 12199, 0, 0, 0, 0,
            5590, 0, 0, 15707, 12400, 0, 0, 0, 15707, 12400, 0, 0, 0, 0, 5700, 0, 0, 15707, 12400,
            0, 0, 0, 15707, 12400, 0, 0, 0, 0, 5895, 0, 0, 15707, 12599, 0, 0, 0, 15707, 12599, 0,
            0, 0, 0, 5998, 0, 0, 15707, 12799, 0, 0, 0, 15707, 12799, 0, 0, 0, 0, 6200, 0, 0,
            15707, 12999, 0, 0, 0, 15707, 12999, 0, 0, 0, 0, 6289, 0, 0, 15707, 12999, 0, 0, 0,
            15707, 12999, 0, 0, 0, 0, 6500, 0, 0, 15707, 13200, 0, 0, 0, 15707, 13200, 0, 0, 0, 0,
            6589, 0, 0, 15707, 13399, 0, 0, 0, 15707, 13399, 0, 0, 0, 0, 6800, 0, 0, 15707, 13599,
            0, 0, 0, 15707, 13599, 0, 0, 0, 0, 6901, 0, 0, 15707, 13599, 0, 0, 0, 15707, 13599, 0,
            0, 0, 0, 7100, 0, 0, 15707, 13799, 0, 0, 0, 15707, 13799, 0, 0, 0, 0, 7200, 0, 0,
            15707, 14000, 0, 0, 0, 15707, 14000, 0, 0, 0, 0, 7400, 0, 0, 15707, 14200, 0, 0, 0,
            15707, 14200, 0, 0, 0, 0, 7499, 0, 0, 15707, 14200, 0, 0, 0, 15707, 14200, 0, 0, 0, 0,
            7700, 0, 0, 15707, 14399, 0, 0, 0, 15707, 14399, 0, 0, 0, 0, 7800, 0, 0, 15707, 14599,
            0, 0, 0, 15707, 14599, 0, 0, 0, 0, 7989, 0, 0, 15707, 14799, 0, 0, 0, 15707, 14799, 0,
            0, 0, 0, 8100, 0, 0, 15707, 14799, 0, 0, 0, 15707, 14799, 0, 0, 0, 0, 8289, 0, 0,
            15707, 14999, 0, 0, 0, 15707, 14999, 0, 0, 0, 0, 8400, 0, 0, 15707, 15199, 0, 0, 0,
            15707, 15199, 0, 0, 0, 0, 8590, 0, 0, 15707, 15399, 0, 0, 0, 15707, 15399, 0, 0, 0, 0,
            8700, 0, 0, 15707, 15399, 0, 0, 0, 15707, 15399, 0, 0, 0, 0, 8899, 0, 0, 15707, 15600,
            0, 0, 0, 15707, 15600, 0, 0, 0, 0, 8990, 0, 0, 15707, 15707, 0, 0, 0, 15707, 15707, 0,
            0, 0, 0,
        ];

        println!("vec_i16 == real_i16 : {} ", vec_i16 == real_i16);

        // let n1: i16 = decode_data[28]as i16  + decode_data[29] as i16;

        // println!("n1 = {}", n1);

        return;
    }

    let orig = b"data";
    let encoded: String = general_purpose::STANDARD_NO_PAD.encode(orig);
    assert_eq!("ZGF0YQ", encoded);
    assert_eq!(
        orig.as_slice(),
        &general_purpose::STANDARD_NO_PAD.decode(encoded).unwrap()
    );

    // or, URL-safe
    let encoded_url = general_purpose::URL_SAFE_NO_PAD.encode(orig);
}
