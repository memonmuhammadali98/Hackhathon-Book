---
sidebar_position: 100
title: Glossary
---

# Glossary

## A

**Actuator**: A component that converts energy into motion, enabling robots to interact with their environment (e.g., motors, servos, pneumatic cylinders).

**Affordance**: The possibilities for action that an environment or object offers to an agent. For example, a flat surface affords walking, and a handle affords grasping.

**Artificial Intelligence (AI)**: The simulation of human intelligence processes by machines, especially computer systems, including learning, reasoning, and self-correction.

**Articulation**: A joint or connection between rigid body parts that allows relative motion.

**Autonomous Navigation**: The ability of a robot to navigate through its environment without human intervention.

## B

**Base Link**: In robotics, the reference frame attached to the robot's base, used as the origin for all other coordinate transformations.

**Behavior Tree**: A hierarchical structure for organizing and executing robot behaviors, commonly used in game AI and robotics.

**Bi-pedal**: Having two feet or legs; refers to humanoid robots that walk on two legs.

## C

**Camera Calibration**: The process of determining the intrinsic and extrinsic parameters of a camera to correct for distortions and enable accurate measurements.

**Cartesian Space**: A coordinate system that specifies point positions by their distances from perpendicular axes (x, y, z).

**Configuration Space (C-Space)**: The space of all possible configurations of a robot, used in motion planning.

**Control Loop**: A feedback system that continuously monitors and adjusts a robot's behavior to achieve desired outcomes.

**Coordinate Transformation**: Converting coordinates from one reference frame to another.

## D

**Dead Reckoning**: Estimating current position based on previously determined position and velocity, without external references.

**Degrees of Freedom (DOF)**: The number of independent parameters that define the configuration of a robotic system.

**Deliberative Control**: A control architecture that uses planning and world models to make decisions, as opposed to reactive control.

**Dense SLAM**: SLAM methods that create detailed, dense 3D reconstructions of environments.

**Depth Camera**: A camera that captures distance information for each pixel, creating a depth map of the scene.

## E

**Embodied AI**: Artificial intelligence systems that have a physical form and interact with the real world through sensors and actuators.

**Encoder**: A sensor that measures rotational or linear position, commonly used in motors.

**End Effector**: The device at the end of a robotic arm designed to interact with the environment (e.g., gripper, tool).

**Extrinsic Parameters**: External parameters of a camera that define its position and orientation in 3D space.

## F

**Feature Extraction**: Identifying and isolating meaningful patterns or structures in sensory data.

**Feature Matching**: Finding corresponding features between different images or sensor readings.

**Force-Torque Sensor**: A device that measures forces and torques applied to it, used for contact detection and manipulation.

**Forward Kinematics**: Computing the position and orientation of the robot's end effector given the joint angles.

**Frame**: A coordinate system used to define positions and orientations in 3D space.

## G

**Gait**: The pattern of movement of the limbs during locomotion.

**Gazebo**: A popular open-source robotics simulator that provides accurate physics simulation.

**Global Planner**: In navigation, a planner that computes a path from the robot's current position to a goal across the entire map.

**Ground Truth**: Accurate reference data used to evaluate the performance of algorithms.

## H

**Humanoid Robot**: A robot with a body shape resembling that of a human, typically with a torso, head, two arms, and two legs.

**Hybrid Control**: Control systems that combine both reactive and deliberative approaches.

## I

**IMU (Inertial Measurement Unit)**: A sensor that measures acceleration, angular velocity, and sometimes magnetic field to estimate orientation and motion.

**Intrinsic Parameters**: Internal camera parameters like focal length and principal point.

**Inverse Kinematics**: Computing the joint angles needed to achieve a desired end effector position and orientation.

**Isaac Sim**: NVIDIA's robotics simulator built on Omniverse, offering photorealistic rendering and physics simulation.

## J

**Joint**: A connection between two or more rigid bodies that allows relative motion.

**Joint Space**: The space defined by all possible joint angle combinations of a robot.

## K

**Kalman Filter**: An algorithm that uses sensor measurements over time to estimate unknown variables, commonly used for state estimation.

**Keyframe**: A selected frame in visual SLAM that is used for mapping and localization.

**Kinematic Chain**: A series of rigid bodies connected by joints, forming a robotic manipulator or limb.

**Kinematic Model**: A mathematical model describing the motion of a robot without considering forces.

## L

**Lidar (Light Detection and Ranging)**: A sensor that measures distance by illuminating a target with laser light and measuring the reflected light.

**Local Planner**: In navigation, a planner that computes immediate motion commands to follow a global path while avoiding obstacles.

**Localization**: Determining a robot's position and orientation within an environment.

**Loop Closure**: In SLAM, detecting that the robot has returned to a previously visited location, which helps correct accumulated errors.

## M

**Manipulation**: The act of moving or controlling objects in the environment using a robotic arm or gripper.

**Map**: A representation of the environment, which can be metric (e.g., grid maps) or topological.

**Motion Planning**: Computing a collision-free path for a robot from a start to a goal configuration.

**Multi-modal Sensor Fusion**: Combining data from different types of sensors to improve perception.

## N

**Nav2 (Navigation2)**: The ROS 2 navigation stack that provides capabilities for autonomous mobile robot navigation.

**Node**: In ROS, an executable that performs computation and communicates with other nodes.

**Nonholonomic Constraint**: A constraint on the motion of a system that cannot be integrated to form a relationship between coordinates alone (e.g., a car cannot move sideways).

## O

**Obstacle Avoidance**: The ability of a robot to detect and navigate around obstacles.

**Occupancy Grid**: A 2D or 3D map representing the probability that each cell is occupied by an obstacle.

**Odometry**: The use of motion sensors to estimate change in position over time.

**Open Loop Control**: Control without feedback, where commands are executed without measuring their effect.

## P

**Path Planning**: Finding a geometric path from a start to a goal location.

**Perception**: The process of interpreting sensory data to understand the environment.

**Physical AI**: See Embodied AI.

**PID Controller**: A control loop mechanism using Proportional, Integral, and Derivative terms to minimize error.

**Pitch**: Rotation around the lateral (side-to-side) axis.

**Point Cloud**: A set of data points in 3D space, typically from a depth camera or Lidar.

**Pose**: The position and orientation of a robot or object in 3D space.

## Q

**Quaternion**: A mathematical representation of 3D rotation that avoids gimbal lock and interpolates smoothly.

**Qdrant**: An open-source vector database used for semantic search and similarity matching.

## R

**RAG (Retrieval-Augmented Generation)**: An AI approach that retrieves relevant information from a knowledge base before generating responses.

**Reactive Control**: Control based on immediate sensory input without planning, enabling fast responses.

**RGB-D Camera**: A camera that captures both color (RGB) and depth (D) information.

**Robot Operating System (ROS)**: An open-source framework for robot software development.

**ROS 2**: The second generation of ROS, redesigned for production systems with improved performance and security.

**Roll**: Rotation around the longitudinal (front-to-back) axis.

## S

**SLAM (Simultaneous Localization and Mapping)**: The process of building a map of an environment while simultaneously tracking the robot's location within it.

**Semantic Segmentation**: Classifying each pixel in an image into predefined categories.

**Sensor Fusion**: Combining data from multiple sensors to produce more accurate, reliable information.

**Servo Motor**: A motor with built-in position feedback for precise control.

**State Estimation**: Determining the current state of a system (position, velocity, etc.) from sensor measurements.

## T

**TF (Transform)**: In ROS, the system for managing coordinate frame transformations over time.

**Topic**: In ROS, a named bus over which nodes exchange messages in a publish-subscribe pattern.

**Trajectory**: A time-parameterized path describing how a robot should move.

**Trajectory Planning**: Determining not just where to go, but also when and how fast.

## U

**Unity**: A popular game engine also used for robotics simulation and visualization.

**URDF (Unified Robot Description Format)**: An XML format for describing robot models in ROS.

## V

**Vector Database**: A database optimized for storing and querying high-dimensional vectors, used in semantic search.

**Visual Odometry**: Estimating motion by analyzing successive camera images.

**Visual SLAM (VSLAM)**: SLAM using visual sensors (cameras) as the primary input.

**Voxel**: A volume element, the 3D equivalent of a pixel.

## W

**World Frame**: A fixed reference frame representing the global coordinate system.

**Workspace**: The region of space that a robot's end effector can reach.

## X

**XACRO**: An XML macro language that extends URDF to allow more compact and reusable robot descriptions.

## Y

**Yaw**: Rotation around the vertical axis.

## Z

**Zero-Moment Point (ZMP)**: A concept used in bipedal robot balance, representing the point where the total inertia force equals zero.

---

## Additional Resources

For more detailed explanations of these terms, please refer to the relevant chapters or use the interactive chatbot to ask specific questions.

**Can't find a term?** Try using the chatbot's search feature or check the index!
