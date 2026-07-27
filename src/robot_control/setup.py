# os, glob: 아래 data_files 에서 launch/urdf/worlds 폴더의 파일 목록을 자동으로 긁어오려고 추가함.
# (ros2 pkg create 기본 setup.py 에는 없던 import — 데이터 파일 설치를 위해 직접 넣음)
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ---- 아래 2줄은 ros2 pkg create 기본값에 없어서 직접 추가한 부분 ----
        # 왜: ament_python 은 소스 폴더의 파이썬 모듈만 install 하고,
        #     launch/urdf 같은 데이터 파일은 명시적으로 등록하지 않으면 install/ 로 복사되지 않는다.
        #     복사가 안 되면 `ros2 launch robot_control ...` 가 파일을 못 찾아 실패한다.
        # 무엇: 각 폴더의 파일을 install/.../share/robot_control/<폴더>/ 로 복사하도록 등록.
        #     (형식: (설치될 위치, [원본 파일 목록]) — glob 으로 폴더 안 파일을 자동 수집)
        # 주의: worlds(환경)는 이 패키지가 소유하지 않는다. robot_control = 로봇만.
        #       월드는 launch 의 world 인자로 외부에서 주입한다(기본값 gz 내장 empty.sdf).
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),  # bridge.yaml 등
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='K-NAVI robot driving: diff-drive URDF, Gazebo Harmonic bringup, ros_gz bridge',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # ros2 run robot_control <이름> 으로 실행될 노드들.
        # twist_mux 는 여기 없다 — 설치 패키지(ros-jazzy-twist-mux)를 쓰므로 config/launch 만 담당.
        'console_scripts': [
            'camera_node = robot_control.camera_node:main',
        ],
    },
)
