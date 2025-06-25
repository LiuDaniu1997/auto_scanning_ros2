import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    deburring_robot_gazebo_path = get_package_share_directory("deburring_robot_gazebo")
    world_path = os.path.join(deburring_robot_gazebo_path, "worlds", "teapot.sdf")
    models_path = os.path.join(deburring_robot_gazebo_path, "models")

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path,
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("deburring_robot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={"world_path": world_path}.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("deburring_robot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("deburring_robot_moveit_config"),
            "launch",
            "moveit.launch.py"
        ),
    )
    
    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        controller,
        moveit,
    ])