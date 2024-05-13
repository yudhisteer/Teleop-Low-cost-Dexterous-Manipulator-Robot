# Teleop: Low Cost Dexterous Manipulator Robot


## Plan of Action

0. [Pre-requisites](#pr)
1. [Digital Twin](#dt)
2. [Control](#c)
3. [Kinematics](#k)
4. [Application](#a)
5. [LLM](#llm)


-------------------------
<a name="pr"></a>

## 0. Pre-requisites

In ROS 2, the **publisher** and **subscriber** nodes are fundamental building blocks for **communication** between different parts of a robotic system. A ```publisher node``` is responsible for **generating** and **sending** **messages** to a specific **topic**, while a ```subscriber node``` listens to that topic and **processes** the **received** messages. This communication pattern allows different components of a robot to exchange information efficiently and in a decoupled manner.

The primary purpose of publisher and subscriber nodes is to enable ```real-time data exchange``` within a robotic system. This facilitates the integration of various **sensors**, **actuators**, and **control algorithms**, allowing for a **modular** and **scalable** architecture. By using topics, ```multiple nodes``` can publish or subscribe to the **same data stream**, promoting flexibility and reusability of code.

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

1. First, add the required dependencies in your ```package.xml``` file. This ensures that the necessary packages are included during the **build** process.

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

2. Next, instruct the **compiler** on how it should **build** your script by adding the appropriate **entry points** in ```setup.py```. This will specify the **executable scripts** for the package.

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = teleop_robot_py_pkg.simple_publisher:main',
        ],
    },
```


3. Now, **build** the workspace using ```colcon build```. This will **compile** the package and make it ready for use.

```shell
colcon build
```

4. Before running any nodes, **source** the workspace to overlay this workspace on top of your environment.

```shell
. install/setup.bash
```

5. Run the ```simple_publisher``` node to start publishing messages.

```shell
ros2 run teleopt_py_pkg simple_publisher
```

6. List all active **topics** to see which topics are currently being **published** or **subscribed** to.

```shell
ros2 topic list
```


7. **Echo** the data being published on a specific topic. For example, to retrieve data from the ```/chatter``` topic:

```
ros2 topic echo /chatter
```

8. For more detailed information about a topic, you can use the ```ros2 topic info``` command. The ```--verbose``` flag provides even more detailed information.

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

- Added dependencies in ```package.xml```.
- Modified ```setup.py```.
- Build the workspace using ```colcon build```.
- Sourced the workspace with ```. install/setup.bash```.

2. Start the ```simple_subscriber``` node to begin **receiving** messages on the topic.

```shell
ros2 run teleopt_py_pkg simple_subscriber
```

3. Now, run the ```simple_publisher``` node to start **sending** messages to the ```simple_subscriber``` node.

```shell
ros2 run teleopt_py_pkg simple_publisher
```

4. You can manually **publish** a **new message** to the ```/chatter``` topic using the ```ros2 topic pub``` command.

```shell
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello, World'"
```
-------------------------
<a name="dt"></a>

## 1. Digital Twin
A Digital Twin is a detailed ```virtual model``` of a ```physical object```, process, or system. It uses **simulations** to support better decision-making. In robotics, a digital twin allows us to test and refine a robot's behavior and performance in a virtual setting before putting it to work in the real world. This approach helps us spot potential issues early, improve the design, and boost overall efficiency. More on this in my other project: [Digital Twin of Anthropomorphic Robotic Arm using AR](https://github.com/yudhisteer/Digital-Twin-of-Anthropomorphic-Robotic-Arm) and [Vision Transformer-Based Multi-Class Classification for Simulated 6DoF Robot ](https://github.com/yudhisteer/Vision-Transformer-Based-Multi-Class-Classification-for-Simulated-6DoF-Robot).

To create an accurate digital twin of a robot, we need to define its** physical structure** and **components** in a ```standardized format```. This is where **URDF** (```Unified Robot Description Format```) comes into play.

### 1.1 URDF
The **URDF** (```Unified Robot Description Format```) convention allows you to represent the **structure** and the **components** of any robot through ```XML``` tags.

Create a package in the src folder named ```teleopt_description```, where we include all components related to the **URDF** of our model as **XML** files.

```
ros2 pkg create --build-type ament_cmake teleopt_description
```

Once the package is created, we need to build it using ```colcon```.

```shell
colcon build
```

To visualize the URDF model, install the URDF **tutorial** package.
```shell
sudo apt-get install ros-humble-urdf-tutorial
```
Let's create our first URDF depicting the base of the robot. The robot's configuration begins with a ```world``` **link** serving as the ```global reference frame```. The primary ```base_link```, illustrated with an ```STL``` mesh and positioned at coordinates ```(-0.5, -0.5, 0)```, is **statically** connected to the ```world``` via a ```virtual_joint```. Additionally, the ```base_plate``` link, illustrated with its own ```STL``` mesh and positioned at coordinates ```(-0.39, -0.39, -0.56)```, attaches to the ```base_link``` through a ```revolute joint```. This joint, which **rotates** around the **Z-axis** ```(0, 0, 1)```, allows controlled **rotational** movement within a range of ```-π/2``` to ```π/2```, with an **effort** limit of ```30.0``` and a **velocity** limit of ```10.0```. 


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


Finally, to visualize the URDF model in **RViz**, use the following ```launch``` command. Ensure the path to your URDF file is correct.

```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/toto/teleop_ws/src/teleopt_description/urdf/teleop.urdf.xacro
```


We want to position the ```base_link``` at the same position as the ```world``` link. The image below shows before and after we move the ```base_link``` by ```(-0.5, -0.5, 0)```.

| Before | After|
|---------|---------|
| ![image](https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/0a594279-f8b1-44e9-b3b9-746f2ab5e908) | ![image](https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/934e48ed-1a89-4d47-8614-f18fd680b108) |

Below is the visualization of the ```base_plate``` rotating about the ```base_link``` around the **Z-axis** ```(0, 0, 1)```. This joint allows controlled rotational movement within a range of ```-π/2``` to ```π/2```, with an effort limit of ```30.0``` and a velocity limit of ```10.0```.


<p align="center">
  <img src="https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/66046ea7-2efd-4fb4-a28e-83ca93bf4fe6" width="70%" />
</p>

Now we will finish our URDF for our robot and visualize its all 4 joints:



### 1.2 Parameters



### 1.3 Launch Files



### 1.4 Gazebo

-------------------------
<a name="c"></a>

## References














