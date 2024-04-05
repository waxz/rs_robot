
ros handler and message struct is raw C pointer.

Raw C pointer must be wrapped and impl Send trait.
```rust
pub struct SafeRosHandlerT{
    pub ptr : ros_handler_t
}
unsafe impl Send for SafeRosHandlerT{}
```

To make data safely shared in threads, use `Arc<Mute>`
```rust
pub struct RosHandler {
    handler: Arc<Mutex<SafeRosHandlerT>>,
}
```

1. ros_handler is thread safe, can be sent to move closure 
2. ros message is not thread safe, but can be sent to move closure