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

1. First, add the required dependencies in the ```package.xml``` file. This ensures that the necessary packages are included during the **build** process.

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

2. Next, instruct the **compiler** on how it should **build** the script by adding the appropriate **entry points** in ```setup.py```. This will specify the **executable scripts** for the package.

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

4. Before running any nodes, **source** the workspace to overlay this workspace on top of the environment.

```shell
. install/setup.bash
```

5. Run the ```simple_publisher``` node to start publishing messages.

```shell
ros2 run teleop_py_pkg simple_publisher
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
ros2 run teleop_py_pkg simple_subscriber
```

3. Now, run the ```simple_publisher``` node to start **sending** messages to the ```simple_subscriber``` node.

```shell
ros2 run teleop_py_pkg simple_publisher
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

Create a package in the src folder named ```teleop_description```, where we include all components related to the **URDF** of our model as **XML** files.

```
ros2 pkg create --build-type ament_cmake teleop_description
```

Once the package is created, we need to build it using ```colcon```.

```shell
colcon build
```

To ensure that ROS2 recognizes the **meshes** and **urdf** folders, we need to update the ```CMakeLists.txt``` file in our package. This will install these directories in the correct location when the package is built.
```
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
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
                <!-- Mesh file location and scale. 'package://teleop_description/meshes/basement.STL' refers to the STL file in the specified package with a scaling factor -->
                <mesh filename="package://teleop_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
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
                <mesh filename="package://teleop_description/meshes/base_plate.STL" scale="0.01 0.01 0.01"/>
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


Finally, to visualize the URDF model in **RViz**, use the following ```launch``` command. Ensure the path to the URDF file is correct.

```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/toto/teleop_ws/src/teleop_description/urdf/teleop.urdf.xacro
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
In ROS2, parameters are settings that control how nodes behave. They allow you to adjust things like thresholds or modes without changing the code, making it easier to configure and manage nodes. Let's see a simple example.

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    """A ROS 2 Node that demonstrates parameter declaration and dynamic parameter updates"""
    
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter(name="simple_int_param", value=42)  # Declare an integer parameter with default value 42
        self.declare_parameter(name="simple_string_param", value="iRobot")  # Declare a string parameter with default value 'iRobot'

        # Add a callback to handle parameter changes
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()  # Create a result object to indicate the success of the parameter update

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                # Log the new value of the parameter
                self.get_logger().info("Param simple_int_param changed! New value is %d" % param.value)
                result.successful = True

            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                # Log the new value of the parameter
                self.get_logger().info("Param simple_string_param changed! New value is %s" % param.value)
                result.successful = True  # Indicate successful update

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)  # Keep the node running to listen for parameter updates
    simple_parameter.destroy_node()  # Destroy the node after spinning
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

1. Again, repeat steps 1 - 4 from above.

- Added dependencies in ```package.xml```.
- Modified ```setup.py```.
- Build the workspace using ```colcon build```.
- Sourced the workspace with ```. install/setup.bash```.


2. To start the ```simple_parameter``` node

```shell
ros2 run teleop_robot_py_pkg simple_parameter 
```


3. List all the parameters of the ```simple_parameter``` node using this command:

```shell
ros2 param list

>>/simple_parameter:
  simple_int_param
  simple_string_param
  use_sim_time
```

4. To read the values of specific parameters, use the ```ros2 param get``` command:

```shell
ros2 param get /simple_parameter simple_string_param
ros2 param get /simple_parameter simple_int_param

>> String value is: iRobot
>> Integer value is: 42
```

5. To set a **new value** for a parameter before starting the node, use the ```--ros-args -p``` option:

```shell
ros2 run teleop_robot_py_pkg simple_parameter --ros-args -p simple_int_param:=24
```

6. You can also change the parameter value while the node is running by using the ```ros2 param set``` command:

```shell
ros2 param set /simple_parameter simple_string_param "Hello, World!"

>> Set parameter successful
>> [INFO] [1715633615.899038632] [simple_parameter]: Param simple_string_param changed! New value is Hello, World!
```

Now that we know how to get and set a parameter, let's configure some nodes that will allow us to publish the URDF model within some Ros2 topics such that Rviz can read the information to display the links and joints on its GUI.

7. The ```robot_state_publisher``` node publishes the **state** of the robot to the ```/robot_description``` topic. This command uses **xacro** to process the **URDF** file and pass it to the ```robot_state_publisher```:
   
```shell
ros2 run robot_state_publisher robot_state_publisher  --ros-args -p robot_description:="$(xacro /home/toto/teleop_ws/src/teleop_description/urdf/teleop.urdf.xacro)"
```

8. The ```joint_state_publisher_gui``` node provides a **GUI** to manually **control** and **publish** the joint states of the robot. This is useful for testing and visualizing the robot's joint configurations:

```shell
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
```

9. Finally, launch **RViz** to visualize the robot model and its states. RViz **subscribes** to the topics **published** by ```robot_state_publisher``` and ```joint_state_publisher_gui``` to display the robot model and joint states in real-time:

```shell
ros2 run rviz2 rviz2
```

Once Rviz is opened, add the **TF** and **Robot model** into the environemnt and it is now ready to be played.

<p align="center">
  <img src="https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/345bf5fd-479a-474b-99d4-8ad2521dc1ac" width="60%" />
</p>



### 1.3 Launch Files

We can see it is a pain to run all these commands in separate terminals to display our robot on Rviz. What instead we can do is build a ```launch``` file that has all these commands such that we will be able to run Rviz displaying our robot with only ```1``` command. 

The code below does the steps ```7```, ```8``` and ```9``` as shown above in one simple command:

```python
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Generate a launch description for launching nodes to visualize the robot model in RViz.

    This function sets up the necessary launch arguments, parameters, and nodes to visualize the
    robot model using the robot_state_publisher, joint_state_publisher_gui, and RViz.
    """
    # Get the directory of the 'teleop_description' package
    teleop_description_dir = get_package_share_directory('teleop_description')

    # Declare a launch argument for the robot model file path
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(teleop_description_dir, 'urdf', 'teleop.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Define the robot description parameter using the xacro command to process the URDF file
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Define the robot_state_publisher node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Define the joint_state_publisher_gui node to provide a GUI for joint states
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Define the RViz node to visualize the robot model
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(teleop_description_dir, 'rviz', 'display.rviz')],
    )

    # Return the launch description including all defined launch arguments and nodes
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

1. Again, we will repeat some earlier process:

- Add dependencies in ```package.xml```.
- Update the ```CMakeLists.txt``` file in our package with **launch** and **rviz**.
- Build the workspace using ```colcon build```.
- Source the workspace with ```. install/setup.bash```.

2. To visualize the URDF model, execute the following launch command. This will start the necessary nodes and processes defined in the ```display.launch.py``` file within the ```teleop_description``` package.

```shell
ros2 launch teleop_description display.launch.py
```

### 1.4 Gazebo
Gazebo is a robotics simulator that provides accurate and efficient simulation of robots in complex environments. It supports high-fidelity physics simulation, allowing to testing and validatation of robotic systems before deploying them in the real world. As such, ```inertia``` is essential for simulating realistic physical behaviors, such as **gravity**, **momentum**, and **stability**. ```Collision``` properties enable accurate **detection** and **response** to interactions between objects, ensuring realistic simulations of **impacts**, **friction**, and **constraints**.

Before our URDF file did not have any inertia or collision components. Hence, Rviz did a coarse visualization in a none physics-applied environment. However, in order to simulate the real-world we will need to modify our URDF.

```xml
<!-- Link element for 'base_link', the primary link of the robot -->
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="-0.5 -0.5 0"/>
            <geometry>
                <mesh filename="package://teleop_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
            </geometry>
        </visual>
    </link>
```

Here are some changes to our URDF file:

- Before no links had any **inertial** properties. So now we add ```<xacro:default_inertial mass="value"/>``` to the links to define its inertial properties, such as **mass**. This is important for physics simulations in Gazebo, as it allows the simulation to account for the link's **mass** and **inertia**, resulting in more **realistic** behavior.

- Before no links had any **collision** properties defined. Now we add a ```<collision>``` element to the links to define its **collision** properties. This includes specifying the origin and geometry for the collision detection using a mesh file. Collision properties are crucial for realistic interactions in Gazebo, as they allow the simulation to **detect** and **respond** to **collisions** between objects.


```xml
<!-- Link element for 'base_link', the primary link of the robot -->
<link name="base_link">
    <xacro:default_inertial mass="1.0"/>  <!-- Default inertial properties with mass 1.0 kg -->
    <visual>
        <origin rpy="0 0 0" xyz="-0.5 -0.5 0"/>
        <geometry>
            <mesh filename="package://teleop_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
        </geometry>
    </visual>
    <!-- Collision properties of the 'base_link', used for physics simulations and collision detection -->
    <collision>
        <!-- Origin of the collision representation with zero rotation and translation -->
        <origin rpy="0 0 0" xyz="-0.5 -0.5 0"/>
        <!-- Geometry definition for collision detection using a mesh file -->
        <geometry>
            <!-- Mesh file location and scale for collision detection -->
            <mesh filename="package://teleop_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
        </geometry>
    </collision>
</link>
```

This command runs the launch file ```gazebo.launch.py``` from the ```teleop_description``` package to visualize the robot in Gazebo, 

```shell
ros2 launch teleop_description gazebo.launch.py
```
By making these changes, the URDF is now better suited for simulation in Gazebo, providing more accurate and realistic physics and collision interactions.

<p align="center">
  <img src="https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/b73e0f15-e6e5-49d2-9f81-6379f7a45c27" width="90%" />
</p>

To visualize the content of the camera, we need to simulate the RGB camera sensor in **Gazebo**. Gazebo will **publish** the video stream from the simulated camera to a ```ROS 2 topic```. To achieve this, you need to add a ```<gazebo>``` tag to the URDF model of your robot. Within this tag, use a ```<sensor>``` tag to activate the simulation of the desired sensor in Gazebo. Below, we have adjusted the configuration parameters of our simulated camera to match the hardware specifications of a real ```Raspberry Pi Camera 3```. Note that we can either insert the ```<gazebo>``` tags within the URDF file directly or we can create a separate file - ```.xacro``` file and reference it in our main URDF file.

```xml
    <!-- Gazebo plugin configuration for the 'rgb_camera' reference -->
    <gazebo reference="rgb_camera">

        <!-- Define the camera sensor -->
        <sensor type="camera" name="rgb_camera">
            <!-- Ensure the camera is always on -->
            <always_on>true</always_on>
            <!-- Set the update rate of the camera to 30 frames per second -->
            <update_rate>30.0</update_rate>
            
            <!-- Camera-specific parameters -->
            <camera name="rgb_camera">
                <!-- Define the horizontal field of view of the camera -->
                <horizontal_fov>1.15</horizontal_fov>
                <!-- Define the vertical field of view of the camera -->
                <vertical_fov>0.71</vertical_fov>
                
                <!-- Image properties -->
                <image>
                    <!-- Image width in pixels -->
                    <width>2304</width>
                    <!-- Image height in pixels -->
                    <height>1296</height>
                    <!-- Image format -->
                    <format>R8G8B8</format>
                </image>
                
                <!-- Distortion parameters -->
                <distortion>
                    <!-- Radial distortion coefficient k1 -->
                    <k1>0.0</k1>
                    <!-- Radial distortion coefficient k2 -->
                    <k2>0.0</k2>
                    <!-- Radial distortion coefficient k3 -->
                    <k3>0.0</k3>
                    <!-- Tangential distortion coefficient p1 -->
                    <p1>0.0</p1>
                    <!-- Tangential distortion coefficient p2 -->
                    <p2>0.0</p2>
                    <!-- Distortion center in normalized coordinates -->
                    <center>0.5 0.5</center>
                </distortion>
            </camera>
            
            <!-- Plugin for interfacing with ROS -->
            <plugin name="plugin_name" filename="libgazebo_ros_camera.so">
                <!-- ROS remapping for image and camera info topics -->
                <ros>
                    <remapping>~/image_raw:=image_raw</remapping>
                    <remapping>~/camera_info:=camera_info</remapping>
                </ros>
                <!-- Name of the camera in ROS -->
                <camera_name>rgb_camera</camera_name>
                <!-- Name of the frame for the camera in ROS -->
                <frame_name>rgb_camera</frame_name>
                <!-- Baseline hack parameter -->
                <hack_baseline>0.2</hack_baseline>
            </plugin>
        </sensor>
```

Gazebo will simulate the camera within the virtual environment, while Rviz will visualize the content of the topics. Specifically, Rviz will **subscribe** to the camera topic and display its content.

-------------------------
<a name="c"></a>
## 2. Control

Control is about sending commands to a robot to make sure it moves as **desired**. This involves creating a **controller** that activates systems, like **motors**, based on **inputs** (e.g., rotating to 90 degrees). The system checks the **current state** against the **desired state** to find an ```error```, that the controller will aim to minimize. In ROS 2, the ```ros2_control``` framework handles this using hardware resources, like **actuators** and **sensors**. The ```resource manager``` abstracts hardware details, while the ```controller manager``` connects **control logic** to the hardware, using predefined controllers from the ROS 2 library.


The ROS 2 Control library offers three control interfaces: **position**, **velocity**, and **force/torque**. ```Position control```, which we will implement, involves moving a robot from its **current position** to a **desired one**, potentially following a **specific trajectory**. For instance, if a robotic arm's motor needs to rotate from ```0``` to ```90``` degrees, the control system calculates the ```error``` (90 degrees initially) and sends commands to minimize this error, making the motor rotate step-by-step until the error is ```zero```. The system continues to maintain the position, correcting for disturbances. ROS 2 Control also supports ```velocity control``` (**moving at a desired speed**) and ```force/torque control``` (**applying a specific force**), useful for delicate tasks like grasping fragile objects.

We create a new ```teleop_ros2_control.xacro``` file in our ```urdf``` folder in the ```teleop_description``` directory. To connect the ROS 2 control library to hardware resources, we define two interfaces for communication with the hardware: the **command interface** and the **state interface**. The command interface **writes** commands to the ```hardware```, such as telling a motor to move. The state interface **reads** the current state of the hardware, like the motor's position. In this configuration, both interfaces are set to control the motor's **position**. The command interface has limits set from ```-90``` to ```90``` degrees. This ensures the motor operates within safe rotational bounds. Since all the movable joints of the robot will use the same command interface and also will provide the same state interface, we can copy and paste this tag joint for joint ```1```, ```2```, ```3``` and ```4```.

The last two joints control the gripper fingers and have different motion ranges. Joint 4 has a range from ```-90``` degrees to``` 0``` degrees. Joint 5 ranges from ```0``` degrees to ```90``` degrees. Joint 5 is **mechanically linked** to Joint 4, moving together through a gear system. To reflect this in the control system, we use two parameters: **mimic**, which makes Joint 5 copy Joint 4's **behavior**, and **multiplier**, set to ```-1``` to make the fingers open **symmetrically** in **opposite** directions. 

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="teleop_robot">

    <!-- Define the ros2_control block for the robot system -->
    <ros2_control name="RobotSystem" type="system">
        
        <!-- Define a property named 'PI' with the value of Pi -->
        <xacro:property name="PI" value="3.14159265359" />

        <!-- Specify the hardware plugin to be used -->
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        
        <!-- Define the common interfaces for ros2_control -->
        <!-- Joint 1 configuration -->
        <joint name="joint_1">
            <command_interface name="position">
                <!-- Set the minimum and maximum command values for joint_1 -->
                <param name="min">-${PI / 2}</param>
                <param name="max">${PI / 2}</param>
            </command_interface>
            <!-- State interface for joint_1 position -->
            <state_interface name="position"/>
        </joint>

        <!-- Joint 2 configuration -->
        <joint name="joint_2">
            <command_interface name="position">
                <!-- Set the minimum and maximum command values for joint_2 -->
                <param name="min">-${PI / 2}</param>
                <param name="max">${PI / 2}</param>
            </command_interface>
            <!-- State interface for joint_2 position -->
            <state_interface name="position"/>
        </joint>

        <!-- Joint 3 configuration -->
        <joint name="joint_3">
            <command_interface name="position">
                <!-- Set the minimum and maximum command values for joint_3 -->
                <param name="min">-${PI / 2}</param>
                <param name="max">${PI / 2}</param>
            </command_interface>
            <!-- State interface for joint_3 position -->
            <state_interface name="position"/>
        </joint>

        <!-- Joint 4 configuration -->
        <joint name="joint_4">
            <command_interface name="position">
                <!-- Set the minimum and maximum command values for joint_4 -->
                <param name="min">-${PI / 2}</param>
                <param name="max">0.0</param>
            </command_interface>
            <!-- State interface for joint_4 position -->
            <state_interface name="position"/>
        </joint>

        <!-- Joint 5 configuration -->
        <joint name="joint_5">
            <!-- Mimic joint_4 with an inverted multiplier -->
            <param name="mimic">joint_4</param>
            <param name="multiplier">-1</param>
            <command_interface name="position">
                <!-- Set the minimum and maximum command values for joint_5 -->
                <param name="min">0.0</param>
                <param name="max">${PI / 2}</param>
            </command_interface>
        </joint>
        
    </ros2_control>

</robot>
```

In our ```teleop.urdf.xacro``` file we will define a macro that will assign a transmission to each of the movable joints of the robot. The transmission tag indicates the presence of a mechanical transmission that connects each motor of the robot to each link of the arm.

```xml
    <!-- Define a Xacro macro named 'default_transmission' with a parameter 'number' -->
    <xacro:macro name="default_transmission" params="number">
        <!-- Define a transmission element with a unique name based on the 'number' parameter -->
        <transmission name="transmission_${number}">
            <!-- Specify the transmission plugin type -->
            <plugin>transmission_interface/SimpleTransmission</plugin>
            <!-- Define an actuator with a unique name based on the 'number' parameter and assign a role -->
            <actuator name="motor_${number}" role="actuator1"/>
            <!-- Define a joint with a unique name based on the 'number' parameter and assign a role -->
            <joint name="joint_${number}" role="joint1">
                <!-- Set the mechanical reduction factor for the joint -->
                <mechanical_reduction>1.0</mechanical_reduction>
            </joint>
        </transmission>
    </xacro:macro>

    <!-- Call the 'default_transmission' macro with the 'number' parameter set to 1 -->
    <xacro:default_transmission number="1"/>
    <!-- Call the 'default_transmission' macro with the 'number' parameter set to 2 -->
    <xacro:default_transmission number="2"/>
    <!-- Call the 'default_transmission' macro with the 'number' parameter set to 3 -->
    <xacro:default_transmission number="3"/>
    <!-- Call the 'default_transmission' macro with the 'number' parameter set to 4 -->
    <xacro:default_transmission number="4"/>
```

In our ```teleop_gazebo.xacro``` file, we add the ```ros2_control``` **plugin**:

```xml
    <!-- Gazebo ros2_control plugin -->
    <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <robot_param>robot_description</robot_param>
        <robot_param_node>robot_state_publisher</robot_param_node>
        <parameters>$(find teleop_controller)/config/teleop_controllers.yaml</parameters>
    </plugin>
    </gazebo>
```



Next, we create a new package in our ```src``` directory named ```teleop_controller```. Do not forget to build the workspace using ```colcon build```.

```shell
ros2 pkg create --build-type ament_cmake teleop_controller
```

We create a ```config``` folder and create the ```teleop_controllers.yaml``` file where we will configure our robots parameters. In ROS 2, ```YAML``` configuration files simplify setting **parameters** for **nodes** and **applications**. For example, the ```controller manager``` node configuration starts with the node's name, followed by ```ros__parameters``` to indicate ROS 2 parameters.

In our configuration:

- We set an ```update_rate``` of ```10 Hz``` for the control **loop**.
- We define two **controllers**: ```arm_controller``` and ```gripper_controller```, both using ```joint_trajectory_controller``` type.
- The ```joint_state_broadcaster``` **publishes** the robot's joint states.

Each controller has parameters for the joints it controls, specifying **command** and **state interfaces** for position. The ```arm_controller``` manages joints ```1```, ```2```, and ```3```, while the ```gripper_controller``` manages joint ```4```. Parameters like ```open_loop_control``` and ```allow_integration_in_goal_trajectories``` are also set to **true** for both controllers.


```yaml
controller_manager:
  ros__parameters:
    update_rate: 10 # Hz

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster


### ---------------------------------------------------------------- ###
arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3

    command_interfaces:
      - position

    state_interfaces:
      - position

    open_loop_control: true
    allow_integration_in_goal_trajectories: true

gripper_controller:
  ros__parameters:
    joints:
      - joint_4
    interface_name: position
      
    command_interfaces:
      - position

    state_interfaces:
      - position

    open_loop_control: true
    allow_integration_in_goal_trajectories: true
### ---------------------------------------------------------------- ###****
```

Now if we use the following command the log confirms the interface between **Gazebo** and **ros2_control** is working fine:

```shell
ros2 launch teleop_description gazebo.launch.py
```

```shell
###---
[gzserver-1] [INFO] [1716315275.264922604] [gazebo_ros2_control]: connected to service!! robot_state_publisher
[gzserver-1] [INFO] [1716315275.265763371] [gazebo_ros2_control]: Received urdf from param server, parsing...
[gzserver-1] [INFO] [1716315275.265818645] [gazebo_ros2_control]: Loading parameter files /home/toto/teleop_ws/install/teleop_controller/share/teleop_controller/config/teleop_controllers.yaml
[gzserver-1] [INFO] [1716315275.278676671] [gazebo_ros2_control]: Loading joint: joint_1
[gzserver-1] [INFO] [1716315275.278802758] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716315275.278810553] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.278816734] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716315275.278820892] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279263172] [gazebo_ros2_control]: Loading joint: joint_2
[gzserver-1] [INFO] [1716315275.279287688] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716315275.279292146] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279300843] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716315275.279304650] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279326441] [gazebo_ros2_control]: Loading joint: joint_3
[gzserver-1] [INFO] [1716315275.279336890] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716315275.279342691] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279346799] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716315275.279350275] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279367858] [gazebo_ros2_control]: Loading joint: joint_4
[gzserver-1] [INFO] [1716315275.279376695] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716315275.279382937] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279395220] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716315275.279403716] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279410899] [gazebo_ros2_control]: Loading joint: joint_5
[gzserver-1] [INFO] [1716315275.279443771] [gazebo_ros2_control]: Joint 'joint_5'is mimicking joint 'joint_4' with mutiplier: -1
[gzserver-1] [INFO] [1716315275.279463388] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716315275.279480029] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716315275.279485960] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716315275.279577852] [resource_manager]: Initialize hardware 'RobotSystem' 
[gzserver-1] [INFO] [1716315275.279855052] [resource_manager]: Successful initialization of hardware 'RobotSystem'
[gzserver-1] [INFO] [1716315275.280013540] [resource_manager]: 'configure' hardware 'RobotSystem' 
[gzserver-1] [INFO] [1716315275.280031113] [resource_manager]: Successful 'configure' of hardware 'RobotSystem'
[gzserver-1] [INFO] [1716315275.280036292] [resource_manager]: 'activate' hardware 'RobotSystem' 
[gzserver-1] [INFO] [1716315275.280049557] [resource_manager]: Successful 'activate' of hardware 'RobotSystem'
[gzserver-1] [INFO] [1716315275.280174221] [gazebo_ros2_control]: Loading controller_manager
[gzserver-1] [WARN] [1716315275.315894292] [gazebo_ros2_control]:  Desired controller update period (0.1 s) is slower than the gazebo simulation period (0.001 s).
[gzserver-1] [INFO] [1716315275.316217919] [gazebo_ros2_control]: Loaded gazebo_ros2_control.
```

The URDF model is set up to load the Gazebo ROS 2 control plugin, interfacing the simulated robot with the ROS 2 Control library. We have defined the input and output interfaces for each joint and configured the **controller manager** to interact with the robot's hardware and other applications. Now, we will create a new ```launch file``` to initiate all these functionalities and ensure the control logic operates smoothly.

```python
def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("teleop_description"),
                    "urdf",
                    "teleop.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
```

We launch the launch file (no pun intended):

```shell
ros2 launch teleop_controller controller.launch.py
```

The log indicates the successful loading, configuring, and activation of the ```gripper_controller```, ```arm_controller``` and ```joint_state_broadcaster``` for the robot.

```shell
###---
[gzserver-1] [INFO] [1716315539.228513760] [gazebo_ros2_control]: Loaded gazebo_ros2_control.
[gzserver-1] [INFO] [1716315558.147759954] [controller_manager]: Loading controller 'gripper_controller'
[gzserver-1] [INFO] [1716315558.256109911] [controller_manager]: Loading controller 'arm_controller'
[gzserver-1] [WARN] [1716315558.289050687] [arm_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
[gzserver-1] [INFO] [1716315558.355968644] [controller_manager]: Loading controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1716315558.456053992] [controller_manager]: Configuring controller 'gripper_controller'
[gzserver-1] [INFO] [1716315558.458171646] [gripper_controller]: configure successful
[gzserver-1] [INFO] [1716315558.556961896] [controller_manager]: Configuring controller 'arm_controller'
[gzserver-1] [INFO] [1716315558.557458688] [arm_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[gzserver-1] [INFO] [1716315558.557746257] [arm_controller]: Command interfaces are [position] and state interfaces are [position].
[gzserver-1] [INFO] [1716315558.557872895] [arm_controller]: Using 'splines' interpolation method.
[gzserver-1] [INFO] [1716315558.561692300] [arm_controller]: Controller state will be published at 50.00 Hz.
[gzserver-1] [INFO] [1716315558.576925156] [arm_controller]: Action status changes will be monitored at 20.00 Hz.
[gzserver-1] [INFO] [1716315558.657490767] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1716315558.657799466] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[gzserver-1] [INFO] [1716315558.856286022] [gripper_controller]: activate successful
```

```shell
###---
[robot_state_publisher-1] [INFO] [1716315557.077416363] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1716315557.077623602] [robot_state_publisher]: got segment base_plate
[robot_state_publisher-1] [INFO] [1716315557.077635805] [robot_state_publisher]: got segment claw_support
[robot_state_publisher-1] [INFO] [1716315557.077640083] [robot_state_publisher]: got segment forward_drive_arm
[robot_state_publisher-1] [INFO] [1716315557.077643670] [robot_state_publisher]: got segment gripper_left
[robot_state_publisher-1] [INFO] [1716315557.077647307] [robot_state_publisher]: got segment gripper_right
[robot_state_publisher-1] [INFO] [1716315557.077650523] [robot_state_publisher]: got segment horizontal_arm
[robot_state_publisher-1] [INFO] [1716315557.077654971] [robot_state_publisher]: got segment rgb_camera
[robot_state_publisher-1] [INFO] [1716315557.077661463] [robot_state_publisher]: got segment world
[spawner-4] [INFO] [1716315558.257217217] [spawner_gripper_controller]: Loaded gripper_controller
[spawner-3] [INFO] [1716315558.357286928] [spawner_arm_controller]: Loaded arm_controller
[spawner-2] [INFO] [1716315558.456997833] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[spawner-4] [INFO] [1716315558.957633921] [spawner_gripper_controller]: Configured and activated gripper_controller
[spawner-3] [INFO] [1716315559.158649522] [spawner_arm_controller]: Configured and activated arm_controller
[INFO] [spawner-4]: process has finished cleanly [pid 31173]
[spawner-2] [INFO] [1716315559.358607590] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
###---
```
Now we can use the CLI, to control the gripper of the robot.
If we want to get a list of all the currently configured and active controllers for our robot:

```shell
ros2 control list_controllers
```

If we want to get the list of hardware components currently available to work with the ROS2 control interface:

```shell
ros2 control list_hardware_components 
```

When we use ```ros2 topic list```, we have the topic ```/gripper_controller/commands``` which we can use to publish new commands to the gripper. If we want to open our gripper, do:


```shell
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [-1]" 
```

If we want to close the gripper:

```shell
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0]" 
```

<p align="center">
  <img src="https://github.com/yudhisteer/Teleop-Low-cost-Dexterous-Manipulator-Robot/assets/59663734/74a133cd-0318-406d-aaf6-38c1f9b3bcd7" width="90%" />
</p>









-------------------------
<a name="k"></a>
## 3. Kinematics



-------------------------
## References














