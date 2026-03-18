"""
lidar_validation_all.launch.py

Runs TurtleBot3 LIDAR validation tests sequentially
and prints a summary report at the end.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # Reset results file before starting tests
    reset_results = Node(
        package='tb3_lidar_validation',
        executable='reset_results',
        output='screen'
    )

    # Test 1: LIDAR message rate
    message_rate = Node(
        package='tb3_lidar_validation',
        executable='message_rate',
        output='screen'
    )

    # Test 2: Range validation
    range_validation = Node(
        package='tb3_lidar_validation',
        executable='range_validation',
        output='screen'
    )

    # Test 3: Stationary noise
    noise_stationary = Node(
        package='tb3_lidar_validation',
        executable='noise_stationary',
        output='screen'
    )

    # Test 4: Front obstacle detection
    front_obstacle_detection = Node(
        package='tb3_lidar_validation',
        executable='front_obstacle_detection',
        output='screen'
    )

    # Test 5: Nearby obstacle detection
    nearby_obstacle_detection = Node(
        package='tb3_lidar_validation',
        executable='nearby_obstacle_detection',
        output='screen'
    )

    # Final summary report
    summary_report = Node(
        package='tb3_lidar_validation',
        executable='summary_report',
        output='screen'
    )

    return LaunchDescription([

        # Start by resetting results file
        reset_results,

        # When reset_results exits, start message_rate
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_results,
                on_exit=[TimerAction(period=1.0, actions=[message_rate])]
            )
        ),

        # When lidar_message_rate exits, start range_validation
        RegisterEventHandler(
            OnProcessExit(
                target_action=message_rate,
                on_exit=[TimerAction(period=1.0, actions=[range_validation])]
            )
        ),

        # When range_validation exits, start noise_stationary
        RegisterEventHandler(
            OnProcessExit(
                target_action=range_validation,
                on_exit=[TimerAction(period=1.0, actions=[noise_stationary])]
            )
        ),

        # When noise_stationary exits, start front_obstacle_detection
        RegisterEventHandler(
            OnProcessExit(
                target_action=noise_stationary,
                on_exit=[TimerAction(period=1.0, actions=[front_obstacle_detection])]
            )
        ),

        # When front_obstacle_detection exits, start nearby_obstacle_detection
        RegisterEventHandler(
            OnProcessExit(
                target_action=front_obstacle_detection,
                on_exit=[TimerAction(period=1.0, actions=[nearby_obstacle_detection])]
            )
        ),

        # When nearby_obstacle_detection exits, start summary_report
        RegisterEventHandler(
            OnProcessExit(
                target_action=nearby_obstacle_detection,
                on_exit=[TimerAction(period=1.0, actions=[summary_report])]
            )
        ),

    ])