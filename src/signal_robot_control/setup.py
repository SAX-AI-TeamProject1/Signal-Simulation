# Build definition for an ament_python package. The part worth reading is
# entry_points: each line there becomes a `ros2 run signal_robot_control <name>`
# executable, and is how the launch file refers to these nodes.

from setuptools import find_packages, setup

package_name = 'signal_robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeungHo',
    maintainer_email='shp9826@naver.com',
    description='Signal-worker command to robot motion.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'command_to_twist = signal_robot_control.command_to_twist:main',
            'mock_signal_publisher = signal_robot_control.mock_signal_publisher:main',
        ],
    },
)
