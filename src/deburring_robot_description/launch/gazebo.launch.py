import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    deburring_robot_description = get_package_share_directory("deburring_robot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        deburring_robot_description, "urdf", "iiwa_with_rgbd.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )
   
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    world_path_arg = DeclareLaunchArgument(name="world_path", 
        default_value="empty.sdf",
        description="Absolute path to world file"
    )

    world_path = LaunchConfiguration("world_path")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", [world_path,
                " -v 4",
                " -r"]
            )
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "deburring_robot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # "/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        parameters=[{"use_sim_time": True}]
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        world_path_arg,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_image_bridge_node,
        relay_camera_info_node
    ])
