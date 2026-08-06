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
            # 옛 레이아웃(signal_u 루프) 하드코딩이라 새 월드에서는 mission_follower 를 쓴다.
            'waypoint_follower = auto_drive.patrol.waypoint_follower:main',
            # 수신호 파견 주행: 수신호석에서 대기하다 signal_dispatch(존 이름)를 받으면
            # tracks.yaml 의 해당 경로로 원판을 찍고 복귀한다. 제어 법칙은
            # waypoint_follower 의 것을 import 해서 쓴다(튜닝 소스는 한 곳 유지).
            'mission_follower = auto_drive.patrol.mission_follower:main',
            # 코너 표지(TrackMarker) 감속: 카메라로 마커를 보고 waypoint_follower의
            # cmd_vel_auto를 줄여 재발행. ultralytics가 필요해 시스템 python3.12로 실행된다.
            'marker_vision = auto_drive.patrol.marker_vision:main',
            # 시뮬 카메라에 뭐가 보이는지 판별해 박스를 그려 재발행하는 뷰어 전용 노드.
            # marker_vision과 같은 YOLO를 쓰지만 속도 명령은 만들지 않는다 — 주행에
            # 영향이 없다. 무거워서 기본은 꺼져 있고(enable_detect), 구독자가 없으면
            # 추론 자체를 건너뛴다.
            'detect_node = auto_drive.perception.detect_node:main',
            # 라이다 스캔을 광선 선분(Marker)으로 다시 그리는 뷰어 전용 노드.
            # 주행에는 아무 영향이 없어서 RViz 를 띄울 때만 함께 뜬다.
            'scan_rays = auto_drive.viz.scan_rays:main',
            # 월드 SDF 의 visual 을 RViz 마커로 옮겨 그리는 뷰어 전용 노드.
            # 스캔 점 옆에 창고 형상이 같이 보여야 그 점이 뭘 맞힌 건지 알 수 있다.
            'world_markers = auto_drive.viz.world_markers:main',
            # 월드를 돌아다니는 사람(<actor>)을 SDF 궤적대로 움직이는 마커로 그린다.
            # 소품과 달리 움직이므로 한 번 그리고 끝낼 수 없어 노드를 나눴다.
            'actor_markers = auto_drive.viz.actor_markers:main',
            # map → <ns>/odom 을 진짜 좌표(pose_gt)로 보정해 발행. 바퀴 적산치의
            # 누적 오차 때문에 RViz 와 gz 의 로봇 위치가 벌어지는 걸 없앤다.
            # 나중에 SLAM 이 이 자리를 대신한다.
            'ground_truth_tf = auto_drive.localization.ground_truth_tf:main',
            # 진행 통로 안의 장애물을 보고 세운다: <ns>/scan → cmd_vel_estop.
            # 회피는 하지 않는다 — 길 위에 사람이 있으면 서고, 지나가면 다시 간다.
            'estop_node = auto_drive.safety.estop_node:main',
            # 정지 기록 전용 관측 노드: odom(실제로 섰는가)과 estop·cmd_vel_gesture
            # (왜 섰는가)의 전환 순간만 CSV 로 남긴다. 주행에는 관여하지 않는다.
            # 구간으로 잇고 그리는 건 tools/plot_stops.py 몫.
            'stop_logger = auto_drive.safety.stop_logger:main',
            # detect_node(YOLO)와 slowdown_node 는 구현하면서 추가한다.
            #   YOLO 노드는 ros2 run 으로는 venv 에 못 닿는다(설치 스크립트 shebang 이 /usr/bin/python3).
            #   launch 에서 Node(prefix='<venv>/bin/python') 로 띄워야 한다 — 검증 완료.
        ],
    },
)
