# 🐢 ROS 2 Workshop: Communication Between Nodes with Turtlesim

## 🎯 Workshop Objective

Develop two ROS 2 nodes that communicate with each other through topics to control the movement of *Turtlesim* in a **triangular** pattern, using odometry information.

---

## 🧱 Part 1: Create the Workspace

Before starting development with ROS 2, you need to create a **workspace**, which is a working environment where all project packages will be stored and built.

The following commands allow you to create and prepare this environment:

```bash
mkdir -p ~/ros2_ws/src
```

This command creates a folder called `ros2_ws` inside your home directory (`~`) and, within it, a subfolder called `src`. The `src` folder is essential because it is where the ROS packages you create or clone will be placed. `colcon build` automatically searches for packages inside this folder to build them.

```bash
cd ~/ros2_ws
colcon build
```

After creating the workspace structure, navigate to the root directory (`ros2_ws`) and run `colcon build`, the recommended tool for building ROS 2 workspaces. This command builds all packages inside `src` and generates the `build/`, `install/`, and `log/` directories. The `install/` directory contains the files needed to run the nodes and scripts.

```bash
source install/setup.bash
```

This command configures the current terminal environment so that it recognizes the packages and nodes you have built in your workspace. It is **very important** to run this command **in every new terminal** before running any node.

## 📦 Part 2: Create the Package

Create a package named `turtle_controller` with Python support and the required dependencies:

```bash
cd ~/ros2_ws/src
ros2 pkg create turtle_controller --build-type ament_python --dependencies rclpy geometry_msgs turtlesim
```

Expected package structure:

```text
turtle_controller/
├── turtle_controller/
│   └── __init__.py
├── package.xml
├── setup.cfg
├── setup.py
```

---

## 🐢 Part 3: Run Turtlesim

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

---

## 📥 Part 4: Subscriber Node — `odom_listener.py`

Create the file:

```bash
cd ~/ros2_ws/src/turtle_controller/turtle_controller
touch odom_listener.py
chmod +x odom_listener.py
```

### 📘 What does this node do?

* Subscribes to the `/turtle1/pose` topic, where Turtlesim publishes its pose information.
* Calculates the total distance traveled by the turtle by adding the displacements between each pair of consecutive points.
* Publishes the accumulated distance to the `/turtle1/path_length` topic.
* Displays the total distance in the terminal and allows it to be visualized using `rqt_plot`.

### 📄 Code:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Float32, '/turtle1/path_length', 10)

        self.last_pose = None
        self.total_distance = 0.0

    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y
            increment = math.hypot(dx, dy)
            self.total_distance += increment

        self.last_pose = msg

        msg_out = Float32()
        msg_out.data = self.total_distance
        self.publisher.publish(msg_out)

        self.get_logger().info(f"Total distance traveled: {self.total_distance:.2f} units")

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 📄 Code Explanation:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math
```

* The required modules are imported:

  * `Pose`: Receives the turtle's position.
  * `Float32`: Publishes the total distance traveled.
  * `math`: Provides functions such as `hypot`, which calculates the Euclidean distance.

```python
class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
```

* Defines a node named `odom_listener`.

```python
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Float32, '/turtle1/path_length', 10)
```

* The node:

  * Subscribes to the `/turtle1/pose` topic.
  * Publishes the accumulated total distance traveled to the `/turtle1/path_length` topic.

```python
        self.last_pose = None
        self.total_distance = 0.0
```

* Stores the last known position so that distance increments can be calculated.

```python
    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y
            increment = math.hypot(dx, dy)
            self.total_distance += increment

        self.last_pose = msg
```

* Every time a new pose is received:

  * Calculates the distance traveled since the previous pose.
  * Adds it to the accumulated total.
  * Updates the last known position.

```python
        msg_out = Float32()
        msg_out.data = self.total_distance
        self.publisher.publish(msg_out)
```

* Publishes the accumulated total as a `Float32` message.

```python
        self.get_logger().info(f"Total distance traveled: {self.total_distance:.2f} units")
```

* Prints the total distance in the terminal to two decimal places.

```python
def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

* Standard ROS 2 structure for initializing, running, and shutting down the node.

---

## 📤 Part 5: Publisher Node — `triangle_stepper.py`

First, create the node file:

```bash
touch triangle_stepper.py
chmod +x triangle_stepper.py
```

### 📘 What does this node do?

This node controls the turtle's movement so that it follows an **equilateral triangle** accurately, using the pose information published on `/turtle1/pose`.

Its behavior is as follows:

* The turtle **calculates three vertices** that form an equilateral triangle, using its starting position as the first point.
* It **rotates in place** until it is aligned with the next vertex.
* It then **moves straight ahead** for exactly 3 units to reach that vertex.
* It pauses briefly, **turns toward the next vertex**, and repeats the process.
* The triangular pattern repeats **indefinitely**, following the intended geometric path.

### 📄 Code:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TriangleStepper(Node):
    def __init__(self):
        super().__init__('triangle_stepper')
        self.pose = None
        self.state = 'turn'
        self.origin = None
        self.target = None
        self.angle_target = None
        self.pause_time = None

        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.L = 3.0
        self.epsilon = 0.01
        self.vertices = []
        self.vertex_index = 0

    def update_pose(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.pose:
            return

        msg = Twist()

        # Initialize the triangle only once
        if not self.vertices:
            x0, y0 = self.pose.x, self.pose.y
            p1 = (x0, y0)
            p2 = (x0 + self.L, y0)
            p3 = (x0 + self.L / 2, y0 + (self.L * math.sqrt(3)) / 2)
            self.vertices = [p1, p2, p3]
            self.origin = p1
            self.target = p2
            self.vertex_index = 1
            self.state = 'turn'
            return

        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.pose.theta)

        if self.state == 'turn':
            if abs(angle_diff) > self.epsilon:
                msg.angular.z = 1.0 * angle_diff
            else:
                msg.angular.z = 0.0
                self.state = 'pause'
                self.pause_time = self.get_clock().now()
                self.origin = (self.pose.x, self.pose.y)

        elif self.state == 'pause':
            now = self.get_clock().now()
            if (now - self.pause_time).nanoseconds / 1e9 > 0.3:
                self.state = 'move'

        elif self.state == 'move':
            traveled = math.hypot(self.pose.x - self.origin[0], self.pose.y - self.origin[1])
            if traveled < self.L:
                msg.linear.x = 1.5
            else:
                msg.linear.x = 0.0
                self.state = 'turn'
                self.vertex_index = (self.vertex_index + 1) % 3
                self.target = self.vertices[self.vertex_index]

        self.publisher.publish(msg)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = TriangleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 📄 Code Explanation:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
```

* Imports the required modules to create a node, subscribe to pose information, and publish movement commands.

```python
class TriangleStepper(Node):
    def __init__(self):
        ...
```

* Defines the `triangle_stepper` node, which controls movement using pose information and trajectory-planning logic.

```python
        self.vertices = []
        self.vertex_index = 0
```

* Defines a list containing the three triangle vertices that the turtle will visit in a loop.

```python
    def control_loop(self):
        ...
```

* This function runs every 0.1 seconds and defines the sequence of actions:

  * **`turn`**: Rotate toward the next vertex.
  * **`pause`**: Stop briefly to ensure stability.
  * **`move`**: Move in a straight line until the next vertex is reached.

```python
        if not self.vertices:
            ...
```

* Initializes the three triangle vertices based on the turtle's current position.

```python
        self.angle_target = self.normalize_angle(...)
```

* Calculates the angle the turtle needs to reach to align itself with the next vertex.

```python
        self.publisher.publish(msg)
```

* Publishes velocity commands to `/turtle1/cmd_vel` to move the turtle.

---

## ⚙️ Part 6: Configure the `setup.py` File

For the package's Python nodes to be executed directly using `ros2 run`, they must be registered in the `entry_points` section of the `setup.py` file.

If this file was generated automatically, modify it as follows:

### 📝 Modify the `setup.py` File

```python
from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_listener = turtle_controller.odom_listener:main',
            'triangle_mover = turtle_controller.triangle_stepper:main',
        ],
    },
)
```

### 🔍 Explanation

* In `console_scripts`, you define a list of scripts that ROS 2 can execute as nodes.
* Each entry follows this format:

  ```bash
  command_name = package.module_name:main_function
  ```

  For example:

  * `odom_listener` is the command used to run the node:

    ```bash
    ros2 run turtle_controller odom_listener
    ```

  * This command is linked to the `odom_listener.py` file inside the package directory and its `main()` function.

⚠️ **Important note:** Make sure both files (`odom_listener.py` and `triangle_stepper.py`) are inside the `turtle_controller/turtle_controller/` directory and have execution permissions. Navigate to the directory containing the node files and run:

```bash
chmod +x odom_listener.py
chmod +x triangle_stepper.py
```

---

## 🚀 Part 7: Build and Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Launch the nodes in separate terminals:

```bash
ros2 run turtle_controller odom_listener
ros2 run turtle_controller triangle_mover
```

---

## 📤 Deliverables

Each student must submit:

- ✅ The `odom_listener.py` file.
- ✅ The `triangle_stepper.py` file.
- ✅ A video showing the turtle moving in a triangular pattern while both nodes are running.
