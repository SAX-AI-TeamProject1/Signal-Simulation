# glob/os: data_files 에서 launch/, config/ 폴더 파일을 자동 수집하려고 추가.
# (ros2 pkg create 기본 setup.py 에는 없다 — robot_control/setup.py 와 같은 이유)
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ament_python 은 파이썬 모듈만 install 한다. launch/config 는 여기 등록하지 않으면
        # install/ 로 복사되지 않아 `ros2 launch auto_drive ...` 가 파일을 못 찾는다.
        # 폴더가 비어 있으면 glob 이 빈 리스트라 설치만 건너뛰고 빌드는 통과한다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # YOLO 가중치. 용량이 커지면 git 에서 빼고 런타임 다운로드로 바꾼다.
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='K-NAVI obstacle reaction: LiDAR emergency stop, sim-camera object detection, '
                'slow-down arbitration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 트랙 왕복 추종: pose_gt 를 보고 cmd_vel_auto 발행(twist_mux 최하위 우선순위).
            # 시스템 python3.12 로 실행된다 — rclpy 외에 별도 의존성이 없다.
            'waypoint_follower = auto_drive.patrol.waypoint_follower:main',
            # estop_node: /robot2/scan → cmd_vel_estop. 시스템 python3.12 로 실행된다.
            # detect_node(YOLO)와 slowdown_node 는 구현하면서 추가한다.
            #   YOLO 노드는 ros2 run 으로는 venv 에 못 닿는다(설치 스크립트 shebang 이 /usr/bin/python3).
            #   launch 에서 Node(prefix='<venv>/bin/python') 로 띄워야 한다 — 검증 완료.
        ],
    },
)
