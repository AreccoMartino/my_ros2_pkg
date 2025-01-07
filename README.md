# Robot Snake Movement Node

## Description

The `move_robot` node is a ROS2-based application designed to control a robot snake's movement based on odometry data. The node subscribes to the `/odom` topic to receive the robot's current position and publishes velocity commands to the `/cmd_vel` topic to control its movement. The robot moves in a snake-like pattern and stops once it overtake a selected distance.

## How to use the Robot Snake Movement Node

### 0. Clone the repository

Navigate to your workspace and clone the repository into the `src` folder:

```bash
git clone <URL_of_this_repository>
```

### 1. Open a terminal and run this command:

```bash
ros2 launch robot_urdf gazebo.launch.py
```
### 2. Open a new terminal and run this command:

```bash
ros2 run my_ros2_pkg move_robot
```

## Code Description

The `move_robot` code is a ROS2 node that controls a robot's movement based on its position data obtained from the odometry topic (`/odom`). The node performs the following functions:

1. **Odometry Callback**: The `odomCallback` function processes the robot's current position from the `/odom` topic. It adjusts the robot's linear and angular velocities based on its `x` and `y` coordinates to create a snake-like movement pattern.

2. **Movement Control**: 
   - The robot moves forward and turns when its `x` position reaches 4.0 or -4.0.
   - It continues moving straight if it hasn't reached these boundaries.
   - The robot stops completely when its `y` position exceeds 9.0, indicating task completion.

3. **Velocity Commands**: The node publishes velocity commands to the `/cmd_vel` topic using `geometry_msgs/msg/Twist`. These commands control the robot's movement by setting linear and angular velocities.

4. **ROS2 Integration**: The node is initialized in the `main` function, which sets up the publisher and subscriber for the velocity and odometry topics, respectively. It uses ROS2 logging to provide real-time feedback on the robot's status and actions.







