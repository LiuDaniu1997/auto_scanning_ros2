import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    lbr_rgbd_dir = get_package_share_directory("deburring_robot_description")
    default_model_path = os.path.join(lbr_rgbd_dir,"urdf","iiwa_with_rgbd.urdf.xacro")
    
    moveit_config = (
        MoveItConfigsBuilder("iiwa_with_rgbd", package_name="deburring_robot_moveit_config")
        .robot_description(file_path=default_model_path)
        .robot_description_semantic(file_path="config/iiwa_with_rgbd.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    moveit_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": True},
                    {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("deburring_robot_moveit_config"),
            "config",
            "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        moveit_group_node,
        rviz_node
    ])