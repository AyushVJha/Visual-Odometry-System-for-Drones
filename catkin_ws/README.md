# Autonomous Mobile Robot Navigation - ROS Workspace

## Setup and Build

1. Install ROS Melodic or Noetic and dependencies (Gazebo, TurtleBot3 packages, MoveIt!, etc.).

2. Clone this workspace or navigate to the `catkin_ws` directory.

3. Build the workspace:

```bash
catkin_make
```

4. Source the workspace:

```bash
source devel/setup.bash
```

## Running the Simulation and Navigation Stack

### 1. Launch Gazebo Simulation with TurtleBot3 in Indoor World

```bash
roslaunch simulation simulation.launch
```

### 2. Run SLAM (GMapping) for Mapping

In a new terminal (source workspace first):

```bash
roslaunch slam_gmapping slam.launch
```

### 3. Run Localization (AMCL)

In a new terminal:

```bash
roslaunch amcl_localization amcl.launch
```

### 4. Run Path Planners (A* and DWA)

In a new terminal:

```bash
roslaunch path_planning planner.launch
```

### 5. Run MoveIt! Integration

In a new terminal:

```bash
roslaunch moveit_integration moveit.launch
```

### 6. Run Custom Sensor Processing and Motor Control Nodes

In a new terminal:

```bash
roslaunch custom_nodes robot_nodes.launch
```

## Notes

- Make sure all terminals have sourced the workspace setup file.

- Adjust parameters in the config files as needed for your environment.

- Use `rviz` or other visualization tools to monitor robot state and navigation.

- For any issues, check ROS logs and console outputs for errors.
