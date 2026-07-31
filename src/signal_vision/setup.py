# glob/os: data_files 에서 launch/, config/ 폴더 파일을 자동 수집하려고 추가.
# (ros2 pkg create 기본 setup.py 에는 없다 — robot_control/setup.py 와 같은 이유)
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'signal_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 모델 가중치는 share/ 가 아니라 파이썬 패키지 안(signal_vision/models/)에 둔다.
    # 이유: vision_hand 는 태스크 4(scripts/setup_infer_env.py)가 외부 리포에서 통째로 복사해
    #       오는 벤더 사본이고, 그 코드가 자기 위치 기준(__file__)으로 models/ 를 찾는다.
    #       우리가 그 경로 규칙을 바꿔 봐야 다음 복사 때 덮이므로, 파일 쪽을 규칙에 맞춘다.
    # package_data 로 등록해 두면 --symlink-install 이 아닌 일반 빌드에서도 install/ 로 복사된다.
    package_data={package_name: ['models/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ament_python 은 파이썬 모듈만 install 한다. launch/config 는 여기 등록하지 않으면
        # install/ 로 복사되지 않아 `ros2 launch signal_vision ...` 가 파일을 못 찾는다.
        # 폴더가 비어 있으면 glob 이 빈 리스트라 설치만 건너뛰고 빌드는 통과한다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='K-NAVI signal-worker gesture recognition: webcam capture, mediapipe landmarks, '
                'LSTM sign classification, gesture -> cmd_vel',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 실물 웹캠 → 랜드마크 추출 → 수신호 분류 → /gesture, cmd_vel_gesture 발행.
            # 시스템 python3.12 로 실행된다(mediapipe/torch 가 거기 설치돼 있어야 한다).
            'camera_node = signal_vision.camera_node.node:main',
        ],
    },
)
