# auto_scanning_ros2

This project is a ROS 2 jazzy and C++ based system for simulating automated point cloud scanning during a deburring process. It is designed for a KUKA iiwa robotic arm equipped with a Mech-Mind PRO S 3D camera.

The core functionalities of the system include:

1. Hand-Eye Calibration

    The system samples and stores both the robot poses and the target tracking poses. It uses OpenCV-based methods to compute and save the hand-eye calibration results.

2. Automated Scanning and Point Cloud Merging

    The robot autonomously scans the object from multiple viewpoints. The acquired point clouds are automatically aligned and merged to reconstruct a complete point cloud. The final merged point cloud is saved in PCD for further processing or analysis.

## Installation (Without Docker)
1. Clone this repository:
2. Install dependencies: 

    ROS2 packages:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

    PCL:
    ```bash
    sudo apt install -y libpcl-dev
    ```

    YAML:
    ```bash
    sudo apt install -y libyaml-cpp-dev
    ```

    Gazebo Harmonic:
    [Install](https://gazebosim.org/docs/harmonic/install_ubuntu/)

    The following dependencies are used to rotate the object during simulation:

    Gazebo Messages:
    ```bash
    sudo apt install -y libgz-msgs11-dev
    ```

    Gazebo Transport: [Install](https://gazebosim.org/api/transport/14/installation.html)

3. Build the package
    ```
    colcon build
    ```

## Usage
With Docker:
1. In terminal 1, start the simulation:
    ```
    start_sim.sh
    ```
2. In terminal 2, start automated scanning:
    ```
    start_auto_scanning.sh
    ```
3. The scanned results are saved in the `merged_point_cloud` and `aligned_point_cloud` folders under the `data` directory.

Without docker:
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

3. If a single scan does not capture a complete point cloud, run the following code to simulate the process of rotating the object and performing the scan again:
    ```
    ros2 launch deburring_robot_gazebo rotate_model.launch.py
    ros2 launch auto_scanning auto_scanning.launch.py
    ```

    Then run the following command to merge the point cloud files located in the merged_point_cloud folder:
    ```
    ros2 run auto_scanning align_merged_clouds_node 
    ```

4. The scanned results are saved in the `merged_point_cloud` and `aligned_point_cloud` folders under the `data` directory.

## Example

[![](https://img.youtube.com/vi/x9MbWzEmqSc/0.jpg)](https://www.youtube.com/watch?v=x9MbWzEmqSc)

## Acknowledgements
[KUKA iiwa model](https://github.com/lbr-stack/lbr_fri_ros2_stack)

[Mech Mind PRO S model](https://community.mech-mind.com/t/topic/2119)

[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)

[PCL](https://pointclouds.org/)

[MoveIt2](https://moveit.picknik.ai/main/index.html)

[Handeye Calibration Python](https://github.com/shengyangzhuang/handeye_calibration_ros2)
