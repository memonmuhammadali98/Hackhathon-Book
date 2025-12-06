---
title: "Chapter 5: Gazebo Simulation for Humanoid Robots"
sidebar_position: 5
---

# Chapter 5: Gazebo Simulation for Humanoid Robots

This chapter provides a comprehensive guide to simulating humanoid robots in Gazebo, an essential tool for developing and testing robotics applications without the need for physical hardware. We will cover everything from creating detailed robot models to integrating with ROS 2 for advanced control.

## 5.1 Detailed Guide to Creating URDF/SDF Models for Humanoid Robots

Unified Robot Description Format (URDF) and Simulation Description Format (SDF) are XML-based file formats used to describe robots and environments in Gazebo. URDF is primarily for describing the robot's kinematics and dynamics, while SDF is more comprehensive, capable of describing entire environments, including robots, static objects, and sensors.

### 5.1.1 Understanding URDF for Humanoids

A humanoid URDF typically consists of `link` and `joint` elements.

- **Links:** Represent the rigid bodies of the robot (e.g., torso, head, upper arm, forearm, hand, thigh, shank, foot).
- **Joints:** Define the connections between links, specifying their type (revolute, prismatic, fixed) and motion limits.

**Example URDF Structure (Simplified Humanoid Arm):**

```xml
<robot name="simple_humanoid_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" velocity="1.0" effort="10.0"/>
  </joint>
</robot>
```

**Key Elements in URDF:**

- **`<visual>`:** Defines the visual representation of a link (mesh, box, cylinder, sphere).
- **`<collision>`:** Defines the collision properties of a link. This is used by the physics engine.
- **`<inertial>`:** Specifies the mass and inertia tensor of a link, crucial for accurate physics simulation.
- **`<origin>`:** Defines the position and orientation of a link or joint relative to its parent.
- **`<axis>`:** Specifies the axis of rotation for revolute joints or translation for prismatic joints.
- **`<limit>`:** Sets the joint's lower and upper position limits, velocity limits, and effort limits.
- **`<gazebo>`:** Used to add Gazebo-specific properties, such as materials, friction, and sensor plugins.

### 5.1.2 SDF for Comprehensive Environment and Robot Description

While URDF is good for robots, SDF can describe everything in a simulation. When converting URDF to SDF, Gazebo often adds additional tags for physics and rendering.

**Basic SDF Structure:**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Humanoid robot model would be inserted here -->

  </world>
</sdf>
```

**Q&A RAG Micro-section:**

**Q: When should I use URDF vs. SDF for my robot model?**
A: Use URDF primarily for defining the kinematic and dynamic properties of your robot. If you need to describe an entire simulation environment including multiple robots, static objects, sensors, and lighting, SDF is the more suitable and comprehensive format. Gazebo natively uses SDF, so URDF files are typically converted to SDF internally.

**Q: How do I handle complex geometries in URDF/SDF?**
A: For complex geometries, it's best to use mesh files (e.g., `.dae` for Collada, `.stl` for StereoLithography). These can be referenced within the `<geometry>` tag using `<mesh filename="package://your_robot_description/meshes/your_part.dae"/>`. Ensure your package paths are correctly set up in your ROS environment.

## 5.2 Environment Design in Gazebo (Worlds, Models)

Gazebo worlds define the entire simulation environment, including gravity, physics engine parameters, lights, and static/dynamic models.

### 5.2.1 Creating Custom World Files

World files are SDF files that specify all aspects of a simulation.

**Example: Simple Room World:**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_humanoid_world">
    <gravity>0 0 -9.8</gravity>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <light name="sun" type="directional">...</light>
    <model name="ground_plane">...</model>

    <model name="wall_1">
      <static>true</static>
      <pose>5 0 2.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box size="10 0.1 5"/></geometry>
        </visual>
        <collision name="collision">
          <geometry><box size="10 0.1 5"/></geometry>
        </collision>
      </link>
    </model>
    <!-- Add more walls, obstacles, furniture as needed -->

    <!-- Placeholder for humanoid robot model -->
    <include>
      <uri>model://humanoid_robot_model</uri> <!-- This would be your robot model package -->
      <pose>0 0 1.0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### 5.2.2 Using and Creating Reusable Models

Gazebo has a model database, and you can create your own models. A model is essentially an SDF file along with its mesh files and textures, packaged in a directory structure that Gazebo can understand.

**Model Directory Structure:**

```
my_humanoid_robot/
├── model.sdf
├── meshes/
│   ├── torso.dae
│   ├── arm.dae
│   └── ...
└── materials/
    └── scripts/
        └── my_robot.material
```

The `model.sdf` file would reference these meshes:

```xml
<model name="my_humanoid_robot">
  <link name="torso">
    <visual name="visual">
      <geometry><mesh><uri>model://my_humanoid_robot/meshes/torso.dae</uri></mesh></geometry>
    </visual>
    <!-- ... other link and joint definitions ... -->
  </link>
</model>
```

**Q&A RAG Micro-section:**

**Q: How can I make my custom Gazebo models available for inclusion in world files?**
A: Gazebo searches for models in specific paths. You can add your model's parent directory to the `GAZEBO_MODEL_PATH` environment variable. For example, if your `my_humanoid_robot` folder is in `~/gazebo_models/`, you would add `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models` to your `~/.bashrc` or `~/.zshrc`.

## 5.3 Physics Simulation and Tuning

Accurate physics simulation is critical for realistic robot behavior. Gazebo allows tuning various physics parameters.

### 5.3.1 Physics Engines

Gazebo supports several physics engines:
- **ODE (Open Dynamics Engine):** Default and generally good for rigid body dynamics.
- **DART (Dynamic Animation and Robotics Toolkit):** Known for its stability and suitability for complex robotic systems.
- **Bullet:** High-performance engine.
- **Simbody:** Good for biomechanical simulations.

You select the engine in your world file: `<physics type="ode">`

### 5.3.2 Tuning Parameters

Key parameters in the `<physics>` tag:

- `<max_step_size>`: The maximum time step size of the physics engine. Smaller values increase accuracy but decrease simulation speed.
- `<real_time_factor>`: The ratio of simulated time to real time. A value of 1.0 means the simulation runs at real-time speed.
- `<real_time_update_rate>`: The frequency at which the physics engine updates.
- **Solver parameters (e.g., for ODE):**
    - `<iters>`: Number of iterations for the solver. Higher values improve contact resolution and stability.
    - `<sor>`: Successive Over-Relaxation parameter.
    - `<friction>`: Surface friction parameters.

**Example Physics Tuning:**

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
      <friction_model>cone</friction_model>
    </solver>
    <constraints>
      <cfm>0.00001</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

**Q&A RAG Micro-section:**

**Q: My robot falls over or behaves erratically in Gazebo. What should I check first?**
A: This is a common issue. Check the following:
1.  **Inertial Properties:** Ensure accurate `<inertial>` tags for all links in your URDF/SDF (mass, inertia matrix). Incorrect values can lead to unstable physics.
2.  **Joint Limits and Dynamics:** Verify `lower`, `upper`, `velocity`, and `effort` limits for all joints.
3.  **Contact Parameters:** Tune friction parameters (`<mu>`, `<mu2>`) in `<surface>` tags for realistic interaction with the ground and other objects.
4.  **Physics Solver:** Increase `<iters>` in the `<solver>` section of your world file to improve contact resolution.
5.  **`max_step_size`:** A smaller `max_step_size` can improve stability at the cost of simulation speed.

## 5.4 Integrating ROS 2 with Gazebo for Simulated Humanoid Control

ROS 2 provides a robust framework for controlling robots, and its integration with Gazebo is seamless, allowing you to develop and test control algorithms.

### 5.4.1 Gazebo ROS 2 Packages

The `ros_gz_sim` package (previously `gazebo_ros_pkgs`) provides the bridge between Gazebo and ROS 2. Key components include:
- `ros_gz_bridge`: For bridging topics between Gazebo and ROS 2.
- `ros_gz_sim_demos`: Examples for integration.

### 5.4.2 Spawning Robots with ROS 2

You can spawn your robot model into Gazebo using a ROS 2 launch file.

**Example ROS 2 Launch File (`spawn_humanoid.launch.py`):**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the robot description package
    robot_description_package_dir = get_package_share_directory('your_robot_description_package')
    urdf_path = os.path.join(robot_description_package_dir, 'urdf', 'humanoid.urdf')

    # Get the path to the ros_gz_sim launch files
    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='empty.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world_name',
            default_value='C:\Users\shoai\Desktop\Hackhathon-ai-book\physical-ai-book\docs\chapters\my_humanoid_world.sdf', # Path to your custom world
            description='Gazebo world file name'),

        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': [' -r -s ', world_name]}.items()
        ),

        # Publish the robot description (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_path).read()}],
            arguments=[urdf_path]
        ),

        # Spawn the robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_humanoid',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'humanoid', '-x', '0', '-y', '0', '-z', '1.0']
        ),
    ])
```

### 5.4.3 Controlling Joints with ROS 2

You can use `ros2_control` to interface your humanoid robot with ROS 2 controllers. This involves:
1.  Adding `ros2_control` tags to your URDF to define hardware interfaces.
2.  Writing a controller configuration file (`.yaml`).
3.  Loading and starting controllers via a ROS 2 launch file.

**URDF with `ros2_control` (simplified):**

```xml
<robot name="my_humanoid">
  <!-- ... existing links and joints ... -->

  <ros2_control name="humanoid_controller" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="shoulder_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <!-- Define interfaces for all controllable joints -->
  </ros2_control>

  <!-- Add gazebo_ros2_control plugin to the gazebo tag -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find your_robot_description_package)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

**Controller Configuration (`humanoid_controllers.yaml`):**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

humanoid_joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - shoulder_joint
      # ... other controllable joints
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50 # Hz
    action_monitor_rate: 20 # Hz
```

**Q&A RAG Micro-section:**

**Q: How do I send commands to my simulated humanoid robot using ROS 2?**
A: With `ros2_control` and a `joint_trajectory_controller` configured, you can send `JointTrajectory` messages to the controller's action server. For example, using the `control_msgs/action/FollowJointTrajectory` action:

```bash
ros2 action send_goal /humanoid_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['shoulder_joint'], points: [{positions: [1.0], time_from_start: {sec: 2}}]}}"
```

## 5.5 Code Examples for Spawning Robots and Applying Forces

### 5.5.1 Programmatically Spawning a Robot (Python)

You can use the Gazebo `spawn_entity.py` script or directly interact with Gazebo's service calls from a Python script.

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
from ament_index_python.packages import get_package_share_directory

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def spawn_robot(self, robot_name, urdf_path, x, y, z):
        self.get_logger().info(f'Spawning robot {robot_name} from {urdf_path}')
        with open(urdf_path, 'r') as f:
            robot_xml = f.read()

        self.req.name = robot_name
        self.req.xml = robot_xml
        self.req.robot_namespace = robot_name
        self.req.initial_pose.position.x = float(x)
        self.req.initial_pose.position.y = float(y)
        self.req.initial_pose.position.z = float(z)

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Spawned {robot_name}: {future.result().status_message}')
        else:
            self.get_logger().error(f'Failed to spawn {robot_name}')

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()

    robot_description_package_dir = get_package_share_directory('your_robot_description_package')
    urdf_path = os.path.join(robot_description_package_dir, 'urdf', 'humanoid.urdf')

    spawner.spawn_robot('my_humanoid_instance', urdf_path, 0, 0, 1.5)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.5.2 Applying Forces and Torques (Python)

You can apply forces or torques to specific links or joints of a simulated robot using the Gazebo physics interface. This often involves publishing to Gazebo topics or calling Gazebo services.

**Example: Applying a force to a link:**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from builtin_interfaces.msg import Duration

class ForceApplier(Node):
    def __init__(self):
        super().__init__('force_applier')
        self.publisher = self.create_publisher(WrenchStamped, '/model/humanoid/link/torso/apply_wrench', 10)
        self.timer = self.create_timer(1.0, self.apply_force_callback) # Apply force every second
        self.get_logger().info('Force applier node started.')

    def apply_force_callback(self):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'torso' # Link name

        # Apply a force in the positive X direction
        wrench_msg.wrench.force.x = 10.0
        wrench_msg.wrench.force.y = 0.0
        wrench_msg.wrench.force.z = 0.0

        # Apply a torque (optional)
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0

        self.publisher.publish(wrench_msg)
        self.get_logger().info('Applied force to torso.')

def main(args=None):
    rclpy.init(args=args)
    force_applier = ForceApplier()
    rclpy.spin(force_applier)
    force_applier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Q&A RAG Micro-section:**

**Q: What is the difference between applying force to a link and applying torque to a joint?**
A: Applying force to a link (`/model/<model_name>/link/<link_name>/apply_wrench`) will cause linear acceleration and potentially angular acceleration of that link based on its mass and inertia. Applying torque to a joint (typically through `ros2_control` commands) directly affects the rotational motion of the child link relative to the parent link around the joint's axis. The `apply_wrench` topic applies a general wrench (force and torque) to a specified link.
