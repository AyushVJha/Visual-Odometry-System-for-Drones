# Robotic Arm Object Manipulation

This package contains custom ROS nodes for perception, motor control, and integration with MoveIt! for robotic arm object manipulation using a UR5 robot.

## Nodes

### perception_node.py
ROS node that subscribes to camera images, performs object detection using a YOLOv5 model, and publishes detected object poses.

### motor_control.py
Publishes motor commands to control the UR5 robotic arm actuators.

### sensor_processing_node
Placeholder node for processing sensor data.

## Usage

To launch the custom nodes including perception, motor control, and sensor processing:

```bash
roslaunch custom_nodes robot_nodes.launch
```

Make sure the camera topic `/camera/rgb/image_raw` is available and YOLOv5 weights are correctly set in `perception_node.py`.

## Project Structure

```
catkin_ws/
├── src/
│   ├── moveit_integration/     # MoveIt! integration for UR5
│   ├── custom_nodes/           # Perception & motor control nodes
│   ├── simulation/             # Gazebo simulation environment for UR5
└── README.md
```

## Prerequisites

- ROS Melodic or Noetic
- Gazebo
- MoveIt!
- PyTorch and OpenCV for perception node

## Build Instructions

1. Build the workspace:
```bash
cd catkin_ws
catkin_make
```

2. Source the workspace:
```bash
source devel/setup.bash
```

## Running the System

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
