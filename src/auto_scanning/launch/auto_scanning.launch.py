from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, Shutdown
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    server_node = Node(
        package='auto_scanning',
        executable='auto_scanning_server_node',
        name='auto_scanning_server',
        output='screen'
    )

    client_node_action = Node(
        package='auto_scanning',
        executable='auto_scanning_client_node',
        name='auto_scanning_client',
        output='screen'
    )
    delayed_client_node = TimerAction(
        period=5.0,
        actions=[client_node_action]
    )

    merger_node = Node(
        package='auto_scanning',
        executable='pointcloud_merger_node',
        name='pointcloud_merger_node',
        output='screen'
    )
    start_merger_after_client = RegisterEventHandler(
        OnProcessExit(
            target_action=client_node_action,
            on_exit=[merger_node]
        )
    )

    shutdown_after_merger = RegisterEventHandler(
        OnProcessExit(
            target_action=merger_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        server_node,
        delayed_client_node,
        start_merger_after_client,
        shutdown_after_merger
    ])