fn main()
{
    std::env::set_var("REBUILD", format!("{:?}", std::time::Instant::now()));
    println!("cargo:rerun-if-env-changed=REBUILD");

    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
    println!("cargo:rustc-link-arg=-fuse-ld=gold");

    println!("cargo:rustc-link-search=native=/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/install/lib");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/install/lib");
    println!("cargo:rustc-link-lib=ros_helper");
    println!("cargo:rustc-link-lib=dds_helper_shared");
    println!("cargo:rustc-link-lib=tinyalloc");
    println!("cargo:rustc-link-lib=tcc_builder");
}
