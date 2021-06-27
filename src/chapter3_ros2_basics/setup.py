from setuptools import setup

package_name = 'chapter3_ros2_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slowrunner',
    maintainer_email='slowrunner@users.noreply.github.com',
    description='Migration using ROS2 Tutorial "Writing a simple publisher and subscriber (Python)"',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = chapter3_ros2_basics.publisher_member_function:main',
            'listener = chapter3_ros2_basics.subscriber_member_function:main',
            'topic_pub = chapter3_ros2_basics.topic_publisher:main',
            'topic_sub = chapter3_ros2_basics.topic_subscriber:main',
            'topic_pub_node = chapter3_ros2_basics.topic_pub_node:main',
            'topic_sub_node = chapter3_ros2_basics.topic_sub_node:main',
            'doubler = chapter3_ros2_basics.doubler:main',
            'log_w_dt  = chapter3_ros2_basics.log_w_dt:main',
        ],
    },
)
