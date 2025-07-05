# Autonomous Mobile Robot Navigation

A comprehensive ROS-based autonomous navigation system for mobile robots featuring SLAM, localization, path planning, and motion control capabilities.

## Features

- **SLAM (Simultaneous Localization and Mapping)** using GMapping
- **Localization** with Adaptive Monte Carlo Localization (AMCL)
- **Global Path Planning** using A* algorithm
- **Local Path Planning** with Dynamic Window Approach (DWA)
- **Motion Planning** integration with MoveIt!
- **Custom Sensor Processing** and Motor Control nodes
- **Gazebo Simulation** with TurtleBot3 in indoor environment

## Technologies Used

- **ROS** (Melodic/Noetic)
- **C++** and **Python**
- **Gazebo** simulation
- **TurtleBot3** robot platform
- **AMCL** for localization
- **MoveIt!** for motion planning
- **GMapping** for SLAM

## Project Structure

```
catkin_ws/
├── src/
│   ├── slam_gmapping/          # SLAM implementation
│   ├── amcl_localization/      # Localization package
│   ├── path_planning/          # A* and DWA planners
│   ├── moveit_integration/     # MoveIt! integration
│   ├── custom_nodes/           # Sensor processing & motor control
│   └── simulation/             # Gazebo simulation environment
└── README.md
```

## 🔧 Installation & Setup

### Prerequisites

- ROS Melodic or Noetic
- Gazebo
- TurtleBot3 packages
- MoveIt!

### Build Instructions

1. Clone this repository:
```bash
git clone https://github.com/yourusername/autonomous-mobile-robot-navigation.git
cd autonomous-mobile-robot-navigation
```

2. Build the workspace:
```bash
cd catkin_ws
catkin_make
```

3. Source the workspace:
```bash
source devel/setup.bash
```

## Usage

### 1. Launch Gazebo Simulation
```bash
roslaunch simulation simulation.launch
```

### 2. Run SLAM (GMapping)
```bash
roslaunch slam_gmapping slam.launch
```

### 3. Run Localization (AMCL)
```bash
roslaunch amcl_localization amcl.launch
```

### 4. Run Path Planners
```bash
roslaunch path_planning planner.launch
```

### 5. Run MoveIt! Integration
```bash
roslaunch moveit_integration moveit.launch
```

### 6. Run Custom Nodes
```bash
roslaunch custom_nodes robot_nodes.launch
```

## 📊 Performance Metrics

- **Localization Error**: <0.1m accuracy
- **Navigation Success Rate**: 95%
- **Real-time Mapping**: Lidar-based SLAM with GMapping
- **Collision-free Navigation**: A* global + DWA local planning

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details.

## Contact

Mail - ayushvjha@gmail.com

Project Link: [https://github.com/ayushvjha/autonomous-mobile-robot-navigation](https://github.com/ayushvjha/autonomous-mobile-robot-navigation)
