# rs_robot
There are libraries and applications for robot perception and control.
- [Pallet detection application](crates/nx_app/src/bin/pallet_detector.rs)
- [PointCloud Viewer for pallet detection](crates/nx_gui/examples/point_cloud_viewer.rs)
- [Motion control application for Single Steering Robot](crates/nx_app/src/bin/ros_motion_control_rerun.rs)
- [Motion control Library for Single Steering Robot](crates/nx_robot)
- [Ros, Fastdds and pallet detection bindings](crates/nx_message_center)

# cpp bindings
[Pallet detection application](crates/nx_app/src/bin/pallet_detector.rs) and [Ros, Fastdds and pallet detection bindings](crates/nx_message_center) have dependency on [cpp_robot](https://github.com/waxz/cpp_robot).
[cpp_robot](https://github.com/waxz/cpp_robot) needs to be compiled and installed first.
Its install path is like `/tmp/cpp-target/install/lib`.


Modify `clang_args`, `rustc-link-search` ,  `rustc-link-arg` in 
[nx_message_center/build.rs](crates/nx_message_center/build.rs)
```rust
fn main() {
    std::env::set_var("REBUILD", format!("{:?}", std::time::Instant::now()));
    println!("cargo:rerun-if-env-changed=REBUILD");
    let clang_args = ["-I/tmp/cpp-target/install/include"];

    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
    println!("cargo:rustc-link-arg=-fuse-ld=gold");

    println!("cargo:rustc-link-search=native=/tmp/cpp-target/install/lib");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/tmp/cpp-target/install/lib");
    println!("cargo:rustc-link-lib=ros_helper");
    println!("cargo:rustc-link-lib=dds_helper_shared");
    println!("cargo:rustc-link-lib=tinyalloc");
    println!("cargo:rustc-link-lib=tcc_builder");
    println!("cargo:rustc-link-lib=pointcloud_process");
}
```



and [nx_app/build.rs](crates/nx_app/build.rs)
. 
```rust
fn main()
{
    std::env::set_var("REBUILD", format!("{:?}", std::time::Instant::now()));
    println!("cargo:rerun-if-env-changed=REBUILD");

    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
    println!("cargo:rustc-link-arg=-fuse-ld=gold");

    println!("cargo:rustc-link-search=native=/tmp/cpp-target/install/lib");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/tmp/cpp-target/install/lib");
}
```

# install application in nx_app
```shell
cargo install --path crates/nx_app  --offline  --root /tmp/cargo-target/install  --bins 
```