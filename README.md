


# set cargo mirror






# cargo rustcflags
https://github.com/rust-lang/cargo/issues/5376
https://course.rs/cargo/reference/configuration.html












# useful link
### blog and book
- https://course.rs/about-book.html
- https://www.thorsten-hans.com/tags/rust/
- https://kaisery.github.io/trpl-zh-cn/

### tips
- Project structure https://dev.to/ghost/rust-project-structure-example-step-by-step-3ee 
- edition change https://doc.rust-lang.org/edition-guide/rust-2018/path-changes.html
- workspace https://course.rs/cargo/reference/workspaces.html
















# command
### install bin
```shell
cargo install --path crates/nx_practice --root /tmp/install  --bin robot_control_test
```





# linker
### rust
https://stackoverflow.com/questions/57812916/how-do-i-change-the-default-rustc-cargo-linker

- create `.cargo/config.toml` in project directory
- set linker
```toml
[target.x86_64-unknown-linux-gnu]
rustflags = [
    "-L" ,"/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/install/lib",
    "-l", "ros_helper",
    "-l", "tinyalloc",
    "-l", "ros_helper_message",
    "-l", "ros_helper_impl",
    "-C", "link-arg=-Wl,-rpath,$ORIGIN/../lib",
    "-C", "link-arg=-fuse-ld=gold",
    "-C", "link-arg=-Wl,-rpath,/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/install/lib"
]

```


### c++
cmake set linker

### pointer
https://stackoverflow.com/questions/24191249/working-with-c-void-in-an-ffi

https://stackoverflow.com/questions/75426661/why-do-these-unaligned-pointer-deferences-work
https://stackoverflow.com/questions/76647616/facing-unexpected-misaligned-pointer-dereference-address-must-be-a-multiple-of

### array string map zip

https://stackoverflow.com/questions/62480743/how-to-change-str-into-array-in-rust
https://stackoverflow.com/questions/38839258/converting-between-veci8-and-str

### cstring
https://stackoverflow.com/questions/66747717/how-do-you-create-a-cstring-from-a-string-in-rust


### runpath
https://github.com/rust-lang/cargo/issues/5077

### linker
https://stackoverflow.com/questions/39310905/how-to-get-the-linker-to-produce-a-map-file-using-cargo


### impl
https://stackoverflow.com/questions/32034529/the-impl-does-not-reference-any-types-defined-in-this-crate



### function
https://stackoverflow.com/questions/38331779/iterate-over-and-call-closures-in-fnmut-vector
https://stackoverflow.com/questions/27589054/what-is-the-correct-way-to-use-lifetimes-with-a-struct-in-rust

https://stackoverflow.com/questions/49703990/cant-borrow-mutably-within-two-different-closures-in-the-same-scope

### smart pointer
https://stackoverflow.com/questions/30275982/when-i-can-use-either-cell-or-refcell-which-should-i-choose
https://stackoverflow.com/questions/30831037/situations-where-cell-or-refcell-is-the-best-choice



### import
https://stackoverflow.com/questions/75237045/how-to-properly-import-other-files-in-rust
https://github.com/shadowmint/rust-starter
https://panaetius.io/post/2020/11/the-difference-between-mod-and-use-in-rust/
https://www.sheshbabu.com/posts/rust-module-system/
https://hackernoon.com/including-files-and-deeply-directories-in-rust-q35o3yer
https://aloso.github.io/2021/03/28/module-system.html


### function pointer
https://stackoverflow.com/questions/27895946/expected-fn-item-found-a-different-fn-item-when-working-with-function-pointer




### file and toml
https://codingpackets.com/blog/rust-load-a-toml-file/

### pub
https://doc.rust-lang.org/beta/reference/visibility-and-privacy.html


### install
```shell
cargo install --path . --root /tmp/install
```

### run
```shell
cargo run --package rs-test1 --bin clap_test -- --name test_app
```

```shell
cargo run  --release --bin spin_sleep_test --manifest-path /home/waxz/RustroverProjects/rust_practice/crates/nx_practice/Cargo.toml
```

### gui
rerun https://www.rerun.io/
egui https://www.egui.rs/#demo
```shell
rerun --server-memory-limit 10% --memory-limit 10%  --web-viewer 
```
https://unix.stackexchange.com/questions/719772/error-running-vulkan-with-igpu