use std::env;
use std::fs;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;
macro_rules! ok(($expression:expr) => ($expression.unwrap()));
macro_rules! log {
    ($fmt:expr) => (println!(concat!("build.rs:{}: ", $fmt), line!()));
    ($fmt:expr, $($arg:tt)*) => (println!(concat!("build.rs:{}: ", $fmt),
    line!(), $($arg)*));
}

#[allow(dead_code)]
fn gcc_simple(source: &str, libname: &str)
{
    let out_dir = env::var("OUT_DIR").unwrap();

    Command::new("gcc")
        .args(&[source, "-c", "-fPIC", "-o"])
        .arg(&format!("{}/{}.o", out_dir, libname))
        .status()
        .unwrap();

    Command::new("ar")
        .arg("curs")
        .arg(&format!("lib{}.a", libname))
        .arg(&format!("{}.o", libname))
        .current_dir(&Path::new(&out_dir))
        .status()
        .unwrap();

    println!("cargo:rustc-link-search=native={}", out_dir);
    println!("cargo:rustc-link-lib=static={}", libname);
    println!("cargo:rerun-if-changed={}", source);
}

fn run<F>(name: &str, mut configure: F)
where
    F: FnMut(&mut Command) -> &mut Command,
{
    let mut command = Command::new(name);
    let configured = configure(&mut command);
    log!("Executing {:?}", configured);
    if !ok!(configured.status()).success() {
        panic!("failed to execute {:?}", configured);
    }
}
#[allow(dead_code)]
fn cmake_simple(source: &str, shared: bool, libname_list: &[&str])
{
    let outdir = env::var("OUT_DIR").expect("OUT_DIR is not set");

    let source_path = Path::new(source);

    run("mkdir", |command| {
        command
            .arg("-p")
            .arg("build")
            .current_dir(source_path.to_str().unwrap())
    });

    run("cmake", |command| {
        command
            .arg("-Bbuild")
            .arg("-DBUILD_DOC=OFF")
            .arg(format!("-DCMAKE_INSTALL_PREFIX={}/install", outdir))
            .current_dir(source_path.to_str().unwrap())
    });

    run("cmake", |command| {
        command
            .arg("--build")
            .arg("build")
            .current_dir(source_path.to_str().unwrap())
    });
    run("cmake", |command| {
        command
            .arg("--build")
            .arg("build")
            .arg("--target")
            .arg("install")
            .current_dir(source_path.to_str().unwrap())
    });

    println!("cargo:rustc-link-search=native={}/install/lib", outdir);

    for l in libname_list {
        println!(
            "cargo:rustc-link-lib={}={}",
            if shared { "dylib" } else { "static" },
            l
        );
    }
}

#[allow(dead_code)]
fn build_use_cc()
{
    cc::Build::new()
        .file("cpp/cpp_liba.cpp")
        .cpp(true)
        .include("cpp/include")
        .compile("cpp_liba");
}

#[allow(dead_code)]
fn buld_use_rust_cmake_config()
{
    use cmake::Config;

    let dst = Config::new("cpp")
        // .define("FOO", "BAR")
        // .cflag("-foo")
        .build();
    println!("cargo:rustc-link-search=native={}", dst.display());
}
fn main()
{
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=wrapper.h");
    std::env::set_var("REBUILD", format!("{:?}", std::time::Instant::now()));
    println!("cargo:rerun-if-env-changed=REBUILD");
    // println!("cargo:rustc-link-lib=dylib=stdc++");

    cmake_simple("src/cpp", true, &["math_op_shared"]);

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("wrapper.h")
        .clang_args(&[
            "-Icpp/include",
            "-I/usr/include/qt6",
            "-I/usr/include/qt6/QtCore",
            "-I/usr/include/qt6/QtGui",
        ])
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bindings_path = out_path.join("generated.rs");

    bindings
        .write_to_file(bindings_path.clone())
        .expect("Couldn't write bindings!");
    fs::copy(
        PathBuf::from(bindings_path),
        PathBuf::from("src/generated.rs"),
    )
    .unwrap();
}
