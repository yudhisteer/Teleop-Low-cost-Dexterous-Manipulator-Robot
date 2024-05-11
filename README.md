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

### 1.1 URDF
The URDF convention allows you to represent the structure and the components of any robot through XML tags

Create a package in the **src** folder named ```teleopt_description``` where we include all components related to the **URDF** of our model as **XML** files:

```
ros2 pkg create --build-type ament_cmake teleopt_description
```

Built the package:

```shell
colcon build
```

```shell
sudo apt-get install ros-humble-urdf-tutorial
```

Visualize in **Rviz**:
```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/toto/teleop_ws/src/teleopt_description/urdf/teleop.urdf.xacro
```



```xml
<?xml version="1.0" encoding="UTF-8"?>

<!-- Root element defining the robot with a unique name and xacro namespace inclusion for macros -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="teleop_robot">

    <xacro:property name="PI" value="3.14159"/>
    <xacro:property name="effort" value="30.0"/>
    <xacro:property name="velocity" value="10.0"/>


    <!-- Link element defining the 'world' link, usually serves as the reference frame for other elements -->
    <link name="world"/>

    <!-- Link element for 'base_link', the primary link of the robot -->
    <link name="base_link">
        <!-- Visual properties of the 'base_link', defining how it looks in the simulation -->
        <visual>
            <!-- Origin of the visual representation with zero rotation and translation -->
            <origin rpy="0 0 0" xyz="-0.5 -0.5 0"/>
            <!-- Geometry definition using a mesh file -->
            <geometry>
                <!-- Mesh file location and scale. 'package://teleopt_robot/meshes/basement.STL' refers to the STL file in the specified package with a scaling factor -->
                <mesh filename="package://teleopt_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
            </geometry>
        </visual>
    </link>

    <!-- Joint element defining a fixed connection between 'world' and 'base_link' -->
    <joint name="virtual_joint" type="fixed">
        <!-- Parent link of the joint, the 'world' link in this case -->
        <parent link="world"/>
        <!-- Child link of the joint, the 'base_link' in this case -->
        <child link="base_link"/>
        <!-- Origin of the joint with zero rotation and translation, aligning 'base_link' directly with 'world' -->
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>

    <!-- Link element for 'base_plate', includes visual representation settings -->
    <link name="base_plate">
        <!-- Visual element specifying the appearance and position of 'base_plate' -->
        <visual>
            <!-- Origin of the visual element relative to the parent link -->
            <origin rpy="0 0 0" xyz="-0.39 -0.39 -0.56"/>
            <!-- Geometry defining the shape from an external mesh file and its scale -->
            <geometry>
                <mesh filename="package://teleopt_description/meshes/base_plate.STL" scale="0.01 0.01 0.01"/>
            </geometry>
        </visual>
    </link>

    <!-- Revolute joint 'joint_1' connecting 'base_link' to 'base_plate' -->
    <joint name="joint_1" type="revolute">
        <!-- Parent link of the joint -->
        <parent link="base_link"/>
        <!-- Child link of the joint -->
        <child link="base_plate"/>
        <!-- Axis of rotation for the revolute joint -->
        <axis xyz="0 0 1"/>
        <!-- Origin of the joint relative to the parent link -->
        <origin rpy="0 0 0" xyz="0 0 0.307"/>
        <!-- Limits of the joint motion with parameters for rotation angle, force, and speed -->
        <limit lower="-${PI/2}" upper="${PI/2}" effort="${effort}" velocity="${velocity}"/>
    </joint>

</robot>
```


| Before | After|
|---------|---------|
| ![image](https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/0a594279-f8b1-44e9-b3b9-746f2ab5e908) | ![image](https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/934e48ed-1a89-4d47-8614-f18fd680b108) |


The robot's configuration begins with a 'world' link serving as the global reference frame. The primary 'base_link', depicted with an STL mesh and positioned at coordinates (-0.5, -0.5, 0), is statically connected to the 'world' via a 'virtual_joint'. Additionally, the 'base_plate' link, illustrated with its own STL mesh and positioned at coordinates (-0.39, -0.39, -0.56), attaches to the 'base_link' through a 'revolute joint'. This joint, which rotates around the Z-axis (0, 0, 1), allows controlled rotational movement within a range of -π/2 to π/2, with an effort limit of 30.0 and a velocity limit of 10.0. 


<p align="center">
  <img src="https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/66046ea7-2efd-4fb4-a28e-83ca93bf4fe6" width="70%" />
</p>

### 1.2 Parameters



### 1.3 Launch Files



### 1.4 Gazebo

----------------------

## References














