---
title: "Chapter 3 - Getting Started with ROS 2 for Humanoid Robotics"
sidebar_position: 3
---

# Chapter 3: Getting Started with ROS 2 for Humanoid Robotics

## 3.1 Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex robot applications. Its distributed architecture is well-suited for modular robotics, including humanoid platforms.

Key architectural concepts in ROS 2:

*   **Nodes:** Executable processes that perform computation (e.g., a node for reading camera data, another for controlling motors, or one for path planning). Each node should ideally be responsible for a single, modular task.
*   **Topics:** A named bus over which nodes exchange messages. One node can publish messages to a topic, and other nodes can subscribe to that topic to receive the messages. This is a many-to-many, asynchronous communication model.

    ```text
    +-----------------+        +-----------------+
    |   Node A        |        |   Node B        |
    | (Publisher)     | <----> | (Subscriber)    |
    +-----------------+        +-----------------+
            |                            |
            +---------- Topic --------+
    ```

*   **Services:** A request/reply communication mechanism. A client node sends a request message to a service server node, and the server processes it and returns a response. This is a one-to-one, synchronous communication model, ideal for immediate operations.
*   **Actions:** A long-running, goal-oriented communication mechanism. Similar to services but designed for tasks that take a long time to complete and require active feedback. A client sends a goal, the server provides continuous feedback as it executes, and finally returns a result. Think of it for navigation or complex manipulation tasks.

    ```text
    +-----------------+        +-----------------+
    |   Action Client | <----> |   Action Server |
    +-----------------+        +-----------------+
            Goal / Feedback / Result
    ```

## 3.2 Setting up a Basic ROS 2 Workspace

To begin developing with ROS 2, you need a workspace, which is a directory containing your project packages. Here's how to set up a basic one (assuming ROS 2 is already installed):

1.  **Create the workspace directory:**

    ```bash
    mkdir -p ~/ros2_humanoid_ws/src
    cd ~/ros2_humanoid_ws
    ```

2.  **Initialize `colcon` (the ROS 2 build tool) in the workspace:**

    ```bash
    colcon build --symlink-install
    ```

    This command will build any packages found in the `src` directory and create `install`, `log`, and `build` directories.

3.  **Source the setup file to overlay your workspace on top of your ROS 2 installation:**

    ```bash
    # For Bash/Zsh
    source install/setup.bash
    # For PowerShell (Windows)
    # .\install\setup.ps1
    ```

    You should source this file in every new terminal you open to use your workspace packages. For convenience, you can add it to your `~/.bashrc` or equivalent shell startup file.

## 3.3 Using Common ROS 2 Tools (rviz, rqt)

ROS 2 provides powerful visualization and debugging tools.

*   **`rviz2`:** A 3D visualizer for robots and sensor data. It allows you to visualize robot models (URDF), sensor readings (point clouds, images), occupancy grids, paths, and more. Indispensable for understanding what your robot is perceiving and how it's moving.

    To run `rviz2` (after sourcing your ROS 2 environment):

    ```bash
    rviz2
    ```

*   **`rqt`:** A Qt-based framework for GUI tools in ROS. It provides a collection of plugins that can be used to introspect and debug a running ROS 2 system. Examples include `rqt_graph` (to visualize node and topic connections), `rqt_console` (for logging messages), `rqt_plot` (for plotting data).

    To run `rqt`:

    ```bash
    rqt
    ```

    From within `rqt`, you can select various plugins to open.

## 3.4 Creating a Simple ROS 2 Package for a Humanoid Robot

Let's create a simple Python package that publishes a "hello" message to a topic and subscribes to another topic to receive a command.

1.  **Create a new package in your `src` directory:**

    ```bash
    cd ~/ros2_humanoid_ws/src
    ros2 pkg create --build-type ament_python humanoid_controller
    ```

2.  **Navigate into the package and create a Python script `talker.py` and `listener.py` in `humanoid_controller/humanoid_controller/`:**

    `humanoid_controller/humanoid_controller/talker.py`:

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'robot_chatter', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0
            self.get_logger().info('Publisher node started.')

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello, humanoid! %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    `humanoid_controller/humanoid_controller/listener.py`:

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalSubscriber(Node):

        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'robot_command',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.get_logger().info('Subscriber node started. Waiting for commands...')

        def listener_callback(self, msg):
            self.get_logger().info('I heard a command: "%s"' % msg.data)
            # In a real robot, this would trigger an action, e.g., move arm

    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Edit `setup.py` in `humanoid_controller/` to include your executables:**

    ```python
    from setuptools import setup

    package_name = 'humanoid_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', ['launch/humanoid_launch.py']), # Add this line for launch file
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'talker = humanoid_controller.talker:main',
                'listener = humanoid_controller.listener:main',
            ],
        },
    )
    ```

4.  **Build your workspace:**

    ```bash
    cd ~/ros2_humanoid_ws
    colcon build --packages-select humanoid_controller
    source install/setup.bash
    ```

5.  **Run the nodes:**

    Open two separate terminals, source your workspace in each, and run:

    Terminal 1:
    ```bash
    ros2 run humanoid_controller talker
    ```

    Terminal 2 (to manually send a command):
    ```bash
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'move_forward'}"
    ```

    You should see the `listener` in Terminal 2 receiving the command.

## 3.5 Explain ROS 2 Launch Files

ROS 2 launch files are used to easily start multiple ROS 2 nodes and configure their parameters in a structured way. They are written in Python or XML and define a system's startup configuration.

Let's create a simple Python launch file `humanoid_launch.py` in `humanoid_controller/launch/`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory (optional, but good practice)
    humanoid_controller_share_dir = get_package_share_directory('humanoid_controller')

    return LaunchDescription([
        Node(
            package='humanoid_controller',
            executable='talker',
            name='robot_talker',
            output='screen',
            parameters=[{'timer_period': 0.25}] # Example of setting a parameter
        ),
        Node(
            package='humanoid_controller',
            executable='listener',
            name='robot_listener',
            output='screen'
        ),
        # Add more nodes here for a complete robot system
    ])
```

To run this launch file (after rebuilding your package and sourcing):

```bash
ros2 launch humanoid_controller humanoid_launch.py
```

This will start both the `talker` and `listener` nodes with a single command.

## 3.6 Q&A RAG Micro-sections

### Q1: What is the primary purpose of a ROS 2 Node?
**A1:** A ROS 2 Node is an executable process that performs a specific, modular computation within the ROS 2 ecosystem. It's designed to be a self-contained unit responsible for a particular task, such as reading sensor data, controlling a motor, or executing a high-level algorithm.

### Q2: When should you use ROS 2 Services instead of Topics for communication?
**A2:** ROS 2 Services should be used for synchronous, request/reply communication when a client needs an immediate response from a server. This is suitable for operations like querying a sensor for a single reading or triggering an immediate, short-duration action. Topics, on the other hand, are for asynchronous, streaming data where continuous updates are needed and an immediate response isn't required.
