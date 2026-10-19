🐢 ROS 2 Workshop: Communication Between Nodes with Turtlesim
🎯 Workshop Objective

Develop two ROS 2 nodes that communicate through topics to control the movement of Turtlesim in a triangular pattern using pose (odometry-style) information.

The workshop consists of two nodes:

odom_listener.py: Measures and publishes the total distance traveled by the turtle.
triangle_stepper.py: Controls the turtle to follow an equilateral triangular path.
🧱 Part 1: Create the ROS 2 Workspace

Create the workspace and its source directory:

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash


The workspace contains the following directories:

src/: Source code for ROS 2 packages.
build/: Intermediate build files.
install/: Installed packages and executable scripts.
log/: Build logs.

Note: Run source install/setup.bash in every new terminal before executing your custom nodes.

📦 Part 2: Create the Package

Navigate to the source directory and create the Python package:

cd ~/ros2_ws/src

ros2 pkg create turtle_controller \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs turtlesim std_msgs


The expected package structure is:

ros2_ws/
└── src/
    └── turtle_controller/
        ├── resource/
        │   └── turtle_controller
        ├── turtle_controller/
        │   └── __init__.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py

🐢 Part 3: Run Turtlesim

Open a new terminal and start Turtlesim:

source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node


Keep this terminal running.

📥 Part 4: Subscriber Node — odom_listener.py

This node subscribes to /turtle1/pose, calculates the distance traveled between consecutive poses, and publishes the accumulated distance to /turtle1/path_length.

Create the file:

cd ~/ros2_ws/src/turtle_controller/turtle_controller
touch odom_listener.py


Add the following code:

import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/turtle1/path_length',
            10
        )

        self.last_pose = None
        self.total_distance = 0.0

    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y

            increment = math.hypot(dx, dy)
            self.total_distance += increment

        self.last_pose = msg

        distance_msg = Float32()
        distance_msg.data = float(self.total_distance)

        self.publisher.publish(distance_msg)

        self.get_logger().info(
            f'Total distance traveled: '
            f'{self.total_distance:.2f} units'
        )


def main(args=None):
    rclpy.init(args=args)

    node = OdomListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

How It Works
Subscribes to /turtle1/pose.
Calculates the displacement between consecutive positions using:
increment = math.hypot(dx, dy)

Adds the displacement to the accumulated distance.
Publishes the total distance to /turtle1/path_length.
Prints the total distance in the terminal.

The total distance can be visualized using rqt_plot:

ros2 run rqt_plot rqt_plot


Enter /turtle1/path_length/data in the plot field.

📤 Part 5: Publisher Node — triangle_stepper.py

This node controls the turtle to follow an equilateral triangle with a side length of 3.0 units.

It uses the following topics:

Subscriber: /turtle1/pose
Publisher: /turtle1/cmd_vel

The node rotates toward each vertex, moves forward, and repeats the sequence indefinitely.

Create the file:

cd ~/ros2_ws/src/turtle_controller/turtle_controller
touch triangle_stepper.py


Add the following code:

import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TriangleStepper(Node):
    def __init__(self):
        super().__init__('triangle_stepper')

        self.pose = None
        self.state = 'initialize'

        self.vertices = []
        self.vertex_index = 1
        self.target = None
        self.origin = None
        self.pause_time = None

        self.L = 3.0
        self.epsilon = 0.01

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.update_pose,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    def update_pose(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        cmd = Twist()

        # Initialize the triangle using the initial turtle position.
        if self.state == 'initialize':
            x0 = self.pose.x
            y0 = self.pose.y

            p1 = (x0, y0)
            p2 = (x0 + self.L, y0)
            p3 = (
                x0 + self.L / 2.0,
                y0 + self.L * math.sqrt(3.0) / 2.0
            )

            self.vertices = [p1, p2, p3]
            self.vertex_index = 1
            self.target = p2
            self.origin = p1
            self.state = 'turn'

            self.get_logger().info(
                'Equilateral triangle initialized.'
            )
            return

        # Calculate the direction toward the next vertex.
        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y

        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(
            angle_to_target - self.pose.theta
        )

        # State 1: Rotate toward the next vertex.
        if self.state == 'turn':
            if abs(angle_diff) > self.epsilon:
                cmd.angular.z = 1.5 * angle_diff
            else:
                cmd.angular.z = 0.0
                self.state = 'pause'
                self.pause_time = self.get_clock().now()
                self.origin = (self.pose.x, self.pose.y)

        # State 2: Pause briefly before moving.
        elif self.state == 'pause':
            now = self.get_clock().now()

            elapsed = (
                now - self.pause_time
            ).nanoseconds / 1e9

            if elapsed >= 0.3:
                self.state = 'move'

        # State 3: Move toward the next vertex.
        elif self.state == 'move':
            traveled = math.hypot(
                self.pose.x - self.origin[0],
                self.pose.y - self.origin[1]
            )

            if traveled < self.L:
                cmd.linear.x = min(1.5, 1.5 * (self.L - traveled))
            else:
                cmd.linear.x = 0.0

                self.vertex_index = (
                    self.vertex_index + 1
                ) % 3

                self.target = self.vertices[self.vertex_index]
                self.state = 'turn'

        self.publisher.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )


def main(args=None):
    rclpy.init(args=args)

    node = TriangleStepper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the turtle when the node is shut down.
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

How It Works

The node uses a state machine with three main states:

turn: Rotates the turtle toward the next vertex.
pause: Waits briefly before moving.
move: Moves the turtle toward the target vertex.

The three vertices are calculated using the initial position and the side length:

p1 = (x0, y0)
p2 = (x0 + L, y0)
p3 = (
    x0 + L / 2.0,
    y0 + L * math.sqrt(3.0) / 2.0
)


The target vertex changes after each side is completed, allowing the turtle to repeat the triangular trajectory.

Important: The turtle must start in a position where all three vertices fit inside the Turtlesim window. Otherwise, the turtle may reach a boundary before completing the triangle.

⚙️ Part 6: Configure setup.py

Open the package's setup.py file:

cd ~/ros2_ws/src/turtle_controller


Replace its contents with:

from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ROS 2 Turtlesim triangular motion controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_listener = turtle_controller.odom_listener:main',
            'triangle_mover = turtle_controller.triangle_stepper:main',
        ],
    },
)

Explanation

The console_scripts section registers the Python nodes so they can be launched using ros2 run.

The format is:

command_name = package.module_name:main_function


For example:

ros2 run turtle_controller odom_listener


runs the main() function from odom_listener.py.

The second command:

ros2 run turtle_controller triangle_mover


runs the main() function from triangle_stepper.py.

The command is called triangle_mover, even though the Python file is named triangle_stepper.py. This is intentional because the executable name is defined in setup.py.

🛠️ Part 7: Build the Workspace

Run:

cd ~/ros2_ws
colcon build --packages-select turtle_controller


After the build finishes, source the workspace:

source ~/ros2_ws/install/setup.bash


If you open a new terminal, source both the ROS 2 installation and your workspace:

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

🚀 Part 8: Run the Nodes

Use three separate terminals.

Terminal 1: Start Turtlesim
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node

Terminal 2: Start the Distance Listener
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run turtle_controller odom_listener

Terminal 3: Start the Triangle Controller
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run turtle_controller triangle_mover


The turtle should begin moving along a triangular trajectory, while the distance listener reports the accumulated distance.

🔎 Part 9: Verify Topic Communication

List the active topics:

ros2 topic list


Inspect the turtle's pose:

ros2 topic echo /turtle1/pose


Inspect the accumulated distance:

ros2 topic echo /turtle1/path_length


Inspect the velocity commands:

ros2 topic echo /turtle1/cmd_vel


You can also inspect the node connections:

ros2 node list
ros2 topic info /turtle1/pose
ros2 topic info /turtle1/cmd_vel

📤 Deliverables

Each student must submit:

odom_listener.py
triangle_stepper.py
A video showing the turtle moving in a triangular pattern while both custom nodes are running.
The complete ROS 2 package, including setup.py and package.xml.
📚 Concepts Learned

By completing this workshop, you will learn how to:

Create and build a ROS 2 Python package.
Implement publisher and subscriber nodes.
Subscribe to Turtlesim pose information.
Publish velocity commands using geometry_msgs/msg/Twist.
Calculate traveled distance using coordinate differences.
Use timers and state machines to control robot movement.
Run and inspect ROS 2 nodes and topics.
📝 Notes
This workshop targets ROS 2 Humble.
The turtle's pose topic provides position and orientation information; it is not a full physical robot odometry message.
The distance calculation approximates the traveled path by summing consecutive pose displacements.
The triangular controller uses pose feedback, but its movement is not guaranteed to be perfectly precise. Actual motion depends on the update rate, controller gains, and Turtlesim boundaries.
Make sure both Python files are inside turtle_controller/turtle_controller/.
Always rebuild the workspace after changing package configuration or Python entry points.

Happy coding! 🐢🚀
