# Robotic Arm Object Manipulation

This project implements an end-to-end robotic arm object manipulation system using ROS, MoveIt!, and YOLOv5 perception.

## Features

- YOLOv5-based object detection for perception
- 3D object localization from camera input
- MoveIt! integration for inverse kinematics and motion planning of UR5 robotic arm
- Motor control for executing pick-and-place tasks
- Gazebo simulation environment for testing

## Technologies Used

- ROS (Melodic/Noetic)
- Python, C++
- MoveIt!
- PyTorch (YOLOv5)
- OpenCV
- Gazebo

## Project Structure

```
catkin_ws/
├── src/
│   ├── moveit_integration/     # MoveIt! integration for UR5
│   ├── custom_nodes/           # Perception & motor control nodes
│   ├── simulation/             # Gazebo simulation environment for UR5
└── README.md
```

## Installation & Setup

### Prerequisites

- ROS Melodic or Noetic
- Gazebo
- MoveIt!
- PyTorch and OpenCV for perception node

### Build Instructions

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

1. Launch Gazebo simulation with UR5 robot and environment:
```bash
roslaunch simulation simulation.launch
```

2. Launch MoveIt! motion planning:
```bash
roslaunch moveit_integration moveit.launch
```

3. Launch custom nodes (perception, motor control):
```bash
roslaunch custom_nodes robot_nodes.launch
```

## Notes

- The perception node uses the Ultralytics YOLOv5 PyTorch model.
- The motor control node currently publishes example commands and should be integrated with motion planning output.
- Further integration and testing are required for full end-to-end operation.

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details.

## Contact

Mail - ayushvjha@gmail.com

Project Link: [https://github.com/AyushVJha/Robotic-Arm-Object-Manipulation](https://github.com/AyushVJha/Robotic-Arm-Object-Manipulation)
