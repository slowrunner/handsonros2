import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='gopigoMinimal'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='False'
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {
                    'robot_description': None
                },
                {
                    '/use_gui': launch.substitutions.LaunchConfiguration('gui')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'robot_description': None
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
