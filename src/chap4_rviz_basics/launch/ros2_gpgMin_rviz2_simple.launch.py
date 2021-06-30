import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = "gopigoMinimal.urdf"
    urdf = os.path.join(
        get_package_share_directory('rviz2_basics'),'urdf',
        urdf_file_name)
    print("\n***** launch: urdf_file - {}".format(urdf))
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim (Gazebo) clock when True'),
        #     'node_prefix',
        #     default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
        #     description='Prefix for node names'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
            # name=[launch.substitutions.LaunchConfiguration('node_prefix'), '']),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'/use_gui': True}],
            arguments=[urdf]),
            # name=[launch.substitutions.LaunchConfiguration('node_prefix'), '']),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     args="-d get_package_share_directory('rviz_basics')/rviz/gpgMin.rviz2.rviz",
        #     output='screen'),
    ])