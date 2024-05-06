# Teleop: Low Cost Dexterous Manipulator Robot


## Plan of Action

0. Pre-requisites
1. Digital Twin
2. Control
3. Kinematics
4. Application
5. LLM


----------------------------

## 0. Pre-requisites

### 0.1 Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='simple_publisher')
        self.publisher = self.create_publisher(String, topic='chatter', qos_profile=10)
        self.counter = 0
        self.frequency = 1.0
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        self.get_logger().info('Publishing at {:.1f} Hz'.format(self.frequency))

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello ROS 2 - counter: {self.counter}"
        self.publisher.publish(msg)
        self.counter += 1

def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

1. Add the dependencies in ```package.xml``` file:

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

2. We need to instruct the compiler on how it should build our script by adding the line below in ```setup.py```:

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = teleop_robot_py_pkg.simple_publisher:main',
        ],
    },
```


3. Build the workspace:

```shell
colcon build
```

4. Source the workspace:

```shell
. install/setup.bash
```

5. Run the simple_publisher node:

```shell
ros2 run teleopt_py_pkg simple_publisher
```

6. View topics at the moment:

```shell
ros2 topic list
```


7. Retrieve data from topic:

```
ros2 topic echo /chatter
```

8. Get more information about your topic using:

```shell
ros2 topic info /chatter
ros2 topic info /chatter --verbose
```


### 0.2 Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self) -> None:
        super().__init__(node_name="simple_subscriber")
        self.sub_ = self.create_subscription(msg_type=String, topic="chatter", callback=self.msg_callback, qos_profile=10)
        
    def msg_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```




1. Repeat steps 1 - 4 from above.

2. Run the simple_subscriber node:

```shell
ros2 run teleopt_py_pkg simple_subscriber
```

3. Publish from publisher node to subscriber node:

```shell
ros2 run teleopt_py_pkg simple_publisher
```

4. Publish a new message on the chatter topic:

```shell
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello, World'"
```

--------------------------------------

## 1. Digital Twin


Create a package in the **src** folder named ```teleopt_description``` where we include all components related to the **URDF** of our model as **XML** files:

```
ros2 pkg create --build-type ament_cmake teleopt_description
```

Built the package:

```shell
colcon build
```

----------------------

## References














