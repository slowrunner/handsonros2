import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# colcon expects and will execute: generate_launch_description function
# Two philosophies:
#  1) define and create the LaunchDescription object in the return statement
#  2) create an empty LaunchDescription object,
#     create DeclareLaunchArgument and LaunchConfiguration objects,
#     create launch_ros.action.Node objects,
#     then add launch actions with ld.add_action()

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gui = LaunchConfiguration('gui', default='True')
    urdf_fn_arg = LaunchConfiguration('urdf_fn_arg', default='gpgMin.urdf')

    urdf_file_name = "gpgMin.urdf"
    urdf = os.path.join(
        get_package_share_directory('rviz2_basics'),'urdf',
        urdf_file_name)
    print("\n***** launch: use_sim_time -   {}".format(use_sim_time))
    print("\n***** launch: gui -            {}".format(gui))
    print("\n***** launch: urdf_fn_arg -    {}".format(urdf_fn_arg))
    print("\n***** launch: urdf_file_name - {}".format(urdf_file_name))

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim (Gazebo) clock when True'),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='True causes joint_state_publisher to launch joint_state_publisher_gui'),
        DeclareLaunchArgument(
            'model',
            default_value='gpgMin',
            description='Only visual and links'),
        DeclareLaunchArgument(
            'urdf_fn_arg',
            default_value='gpgMin.urdf',
            description='Only visual and links'),


        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),

        # Supposedly in ROS2 passing use_gui to joint_state_publisher will attempt
        # to launcdh joint_state_publisher_gui (if installed)
        # (put joint_state_publisher_gui in an exec_depend in package.xml and
        # run rosdep install -i --from-path src in the workspace root)
        # BUT-BUT-BUT it doesn't
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            # package='joint_state_publisher',
            # executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            # parameters=[{'use_gui' : True }],
            parameters=[],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[],
            arguments="-d get_package_share_directory('rviz2_basics')/rviz/$(arg model).rviz2",
            output='screen'),
    ])
