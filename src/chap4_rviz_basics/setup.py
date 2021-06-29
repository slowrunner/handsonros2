import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'rviz2_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.py)),
        # Include model and simulation files
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slowrunner',
    maintainer_email='slowrunner@users.noreply.github.com',
    description='ROS2 Version of Hands-On ROS - Chap4 RVIZ Basics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #     ],
    },
)
