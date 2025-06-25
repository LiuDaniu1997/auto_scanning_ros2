# Handeye Calibration

This package performs hand-eye calibration to compute the transformation from the camera frame to the robot base (base_link). This transformation is essential for converting point clouds from the camera coordinate system into the robot’s base frame, enabling spatial alignment. 

The calibration is performed by scanning an ArUco marker from multiple viewpoints and calculating the marker's pose in each frame. Based on these observations, the hand-eye transformation is estimated.

## Usage
1. In terminal 1, source your ROS2 workspace and start the simulation with Aruco Marker:
    ```
    source install/setup.bash
    ros2 launch deburring_robot_bringup handeye_calibration.launch.py
    ```

2. In terminal 2, start Marker scanning:
    ```
    source install/setup.bash
    ros2 launch handeye_calibration handeye_calibration.launch.py
    ```
    Each time a scan is performed, the result is stored in the file `handeye_samples.yaml` under the `handeye_calibration/data` directory.The resulting transformation will be saved in `handeye_result.yaml` under the same data directory.