# ROS2 Description Package -

The STL files for the biped are borrowed from [link](https://www.instructables.com/Arduino-Controlled-Robotic-Biped/). The project is to create a ROS2 package that describes the biped robot. The package includes the following:

1. **URDF Description:** The URDF file describes the robot's structure, including links, joints, and visual properties. The URDF is used for visualization and simulation in ROS2.

2. **Launch Files:** The launch files are used to start the robot model in RViz, a 3D visualization tool for ROS2. The launch files set up the robot model, sensor plugins, and visualization parameters.

3. **Mesh Files:** The mesh files are used to visualize the robot's appearance in RViz. The mesh files are linked in the URDF to provide a realistic representation of the robot.

## ROS2 Command -

```bash
ros2 launch rsa_biped view_robot.launch.py
```

```bash
ros2 launch rsa_biped biped_bringup.launch.py
```
