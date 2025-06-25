import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, Shutdown
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events.process import ShutdownProcess
from launch.events import matches_action
from launch_ros.actions import Node

def generate_launch_description():
    server_node = Node(
        package='handeye_calibration',
        executable='handeye_estimation_server_node',
        name='handeye_server',
        output='screen'
    )

    client_node = Node(
        package='handeye_calibration',
        executable='handeye_estimation_client_node',
        name='handeye_client',
        output='screen'
    )

    delayed_client = TimerAction(
        period=5.0,
        actions=[client_node]
    )

    shutdown_server_after_client = RegisterEventHandler(
        OnProcessExit(
            target_action=client_node,
            on_exit=[
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(server_node)
                    )
                )
            ]
        )
    )

    solver_node = Node(
        package='handeye_calibration',
        executable='handeye_solver',
        name='handeye_solver',
        output='screen'
    )
    start_solver_after_client = RegisterEventHandler(
        OnProcessExit(
            target_action=client_node,
            on_exit=[solver_node]
        )
    )

    shutdown_after_solver = RegisterEventHandler(
        OnProcessExit(
            target_action=solver_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        server_node,
        delayed_client,
        shutdown_server_after_client,
        start_solver_after_client,
        shutdown_after_solver,
    ])