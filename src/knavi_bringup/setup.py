# glob/os: data_files 에서 launch/, config/ 폴더 파일을 자동 수집하려고 추가.
# (ros2 pkg create 기본 setup.py 에는 없다 — robot_control/setup.py 와 같은 이유)
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'knavi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 이 패키지의 알맹이는 전부 여기 두 줄이다 — 실행 노드가 없고 launch/config 만 담는다.
        # launch/ 에 bringup.launch.py 가 들어왔다(robot_control 에서 이동).
        # config/ 는 아직 비어 있다 — bridge.yaml, flat_ground.sdf 는 robot_control 에 그대로 두고
        # launch 가 robot_control 의 share 에서 읽는다. 옮길지는 별도 판단.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='K-NAVI system bringup: Gazebo world, ros_gz bridge, and the top-level launch '
                'that assembles robot_control / signal_vision / auto_drive',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # 실행 노드 없음. 조립 전용 패키지다.
        'console_scripts': [
        ],
    },
)
