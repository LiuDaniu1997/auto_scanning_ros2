from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_debug_mode = SetEnvironmentVariable(
        name="GZ_VERSIONED_MSGS_SUPPRESS_DESC_ERRORS",
        value="1"
    )

    set_model_pose_node = Node(
        package="deburring_robot_gazebo",
        executable="set_model_pose_node",
        name="set_model_pose_node",
        output="screen",
    )


    return LaunchDescription([
        gazebo_debug_mode,
        set_model_pose_node
    ])