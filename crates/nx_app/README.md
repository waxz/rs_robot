
# install files
```shell
cargo install --path crates/nx_app  --offline  --root /tmp/cargo-target/install  --bins 
```

```shell
rsync -azPn /home/waxz/RustroverProjects/rust_practice/crates/nx_app/src/config/ xx_1:/home/hitrobot/control/app/install/bin/
rsync -azPn /home/waxz/RustroverProjects/rust_practice/crates/nx_gui/config/ xx_1:/home/hitrobot/control/app/install/bin/

```

```shell
rsync -azPn /tmp/cargo-target/install/ xx_1:/home/hitrobot/control/app/install
rsync -azPn /tmp/cpp-target/install/ xx_1:/home/hitrobot/control/app/install

```

# test ros message publisher

```python

import rospy
from rospy_tutorials.msg import HeaderString

NAME = 'talker_header'

def callback(msg):  
    print("recieve header:\n",msg.header)
    print("recieve data:\n",msg.data)       
       
def talker_header():
    pub = rospy.Publisher("pallet_detector/cmd", HeaderString, queue_size=10)

    rospy.init_node(NAME) #blocks until registered with master
    count = 0
    sub = rospy.Subscriber('pallet_detector/status', HeaderString, callback)  
    send_str = """
cmd_type = "DetectPallet"
detect_cmd.task_id = "a2"
detect_cmd.retry_count = 11

"""
    header_string = HeaderString(None, send_str)
    while not rospy.is_shutdown():
        print(send_str)
        # If None is used as the header value, rospy will automatically
        # fill it in.
        
        header_string.header.stamp = rospy.Time.now()
        pub.publish(header_string)
        count += 1
        rospy.sleep(1)
 
if __name__ == '__main__':
    talker_header()
```