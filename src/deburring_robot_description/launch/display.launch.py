from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    lbr_rgbd_dir = get_package_share_directory("deburring_robot_description")
    default_model_path = os.path.join(lbr_rgbd_dir,"urdf","iiwa_with_rgbd.urdf.xacro")
    model = DeclareLaunchArgument(name="model", default_value=default_model_path)
    default_rviz_path = os.path.join(lbr_rgbd_dir,"rviz","lbr_with_rgbd_display.rviz")

    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2"
    )
    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])