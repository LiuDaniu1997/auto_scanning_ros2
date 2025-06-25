# auto_scanning

This package controls a KUKA iiwa robotic arm using MoveIt 2 to perform automated scanning of an object from multiple viewpoints. At each pose, a partial point cloud is captured. Specifically, the object is scanned from three different angles, with each scan oriented at 30°, 90°, and -30° relative to the horizontal direction.

The acquired point clouds are then merged using algorithms from the Point Cloud Library (PCL) to reconstruct a complete 3D representation of the object.

## Usage
1. In terminal 1, source your ROS2 workspace and start the simulation:
    ```
    source install/setup.bash
    ros2 launch deburring_robot_bringup simulated_robot.launch.py
    ```

2. In terminal 2, start automated scanning:
    ```
    source install/setup.bash
    ros2 launch auto_scanning auto_scanning.launch.py
    ```
    The partial point clouds captured during the scanning process are saved in the `tmp` folder under the `data` directory of the workspace.

    Each time the launch file is executed, the merged point cloud resulting from the registration process is saved in the `merged_point_cloud` folder under the same data directory.