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
        #     urdf/config 같은 데이터 파일은 명시적으로 등록하지 않으면 install/ 로 복사되지 않는다.
        #     복사가 안 되면 knavi_bringup 의 launch 가 이 share 에서 파일을 못 찾아 실패한다.
        # 무엇: 각 폴더의 파일을 install/.../share/robot_control/<폴더>/ 로 복사하도록 등록.
        #     (형식: (설치될 위치, [원본 파일 목록]) — glob 으로 폴더 안 파일을 자동 수집)
        # 주의: worlds(환경)는 이 패키지가 소유하지 않는다. robot_control = 로봇의 몸과 배선만.
        # launch 줄은 남겨 둔다: bringup.launch.py 는 knavi_bringup 으로 옮겨 지금은 glob 이
        # 빈 리스트지만, 이 패키지 전용 launch(예: urdf 단독 확인용)가 생기면 바로 걸린다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),  # bridge.yaml 등
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='K-NAVI robot body and wiring: robot URDF (xacro) and the bridge / twist_mux / '
                'flat_ground config the knavi_bringup launch consumes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # 이 패키지에는 실행 노드가 없다 — 로봇의 "몸"(urdf)과 배선(config/launch)만 담당한다.
        #   camera_node        → signal_vision  (수신호 인식)
        #   waypoint_follower  → auto_drive     (경로 추종)
        #   twist_mux          → 설치 패키지(ros-jazzy-twist-mux). 여기선 config 만 제공.
        'console_scripts': [
        ],
    },
)
