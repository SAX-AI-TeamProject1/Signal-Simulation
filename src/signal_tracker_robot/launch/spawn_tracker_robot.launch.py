# Last updated: 2026-07-15
"""tracker_robot을 gz-sim(Gazebo Harmonic)에 스폰하고 ros2_control 컨트롤러를 띄운다.

흐름: gz-sim 실행(빈 월드) → robot_state_publisher(URDF 발행)
     → ros_gz_sim create(스폰) → /clock 브릿지 → joint_state_broadcaster/arm_position_controller
     스폰(순서상 로봇이 gz-sim에 실제로 생긴 뒤에 컨트롤러 매니저 서비스가 뜨므로
     TimerAction으로 살짝 지연을 둔다).

사용법:
    ros2 launch signal_tracker_robot spawn_tracker_robot.launch.py

이후 motion_retarget_node가 Signal-Vision의 /upper_body_pose를 구독해
/arm_position_controller/commands로 관절 위치 명령을 보낸다 (별도 노드, 이 launch엔
포함하지 않음 — 카메라 연결 여부와 무관하게 로봇 스폰 자체는 항상 되게 하기 위함).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PACKAGE_NAME = "signal_tracker_robot"
ROBOT_NAME = "tracker_robot"


def _load_robot_description() -> str:
    """URDF 파일을 문자열로 읽어 컨트롤러 설정 파일의 실제 절대 경로를 채워 넣는다.

    URDF 안의 "__CONTROLLERS_YAML_PATH__"는 xacro 없이 순수 URDF만 쓰기로 한 대신 남겨둔
    플레이스홀더다 — $(find pkg) 같은 xacro 전용 치환 문법은 이 파일을 그대로 읽는
    robot_state_publisher/gz_ros2_control 쪽에서 해석되지 않으므로, 여기서 직접 문자열
    치환한다.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(share_dir, "urdf", "tracker_robot.urdf")
    controllers_path = os.path.join(share_dir, "config", "controllers.yaml")
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read()
    return urdf_text.replace("__CONTROLLERS_YAML_PATH__", controllers_path)


def generate_launch_description() -> LaunchDescription:
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # 1) gz-sim 실행 — 빈 월드로 시작 (창고 월드와 합치고 싶으면 gz_args를 그 .sdf로 바꾸면 됨)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    # 2) robot_state_publisher — URDF를 robot_description 파라미터/토픽으로 발행
    #    (ros_gz_sim의 create 노드가 이 토픽을 읽어서 스폰한다)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": _load_robot_description(), "use_sim_time": True}],
    )

    # 3) gz-sim 안에 실제로 로봇 엔티티 생성 — robot_description 토픽을 읽어 스폰
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", ROBOT_NAME, "-z", "0.0"],
        output="screen",
    )

    # 4) 시뮬레이션 시계를 ROS 2 /clock으로 브릿지 — ros2_control이 use_sim_time과 맞물려
    #    시뮬레이션 시간 기준으로 동작하려면 필요
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # 5) 컨트롤러 스폰 — gz_ros2_control 플러그인(URDF의 <gazebo><plugin>)이 로봇과 함께
    #    시작하는 controller_manager에 joint_state_broadcaster/arm_position_controller를 로드한다.
    #    로봇이 실제로 스폰된 뒤 controller_manager 서비스가 뜨므로 약간 지연을 둔다.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_position_controller"],
        output="screen",
    )
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, arm_position_controller_spawner],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        delayed_controllers,
    ])
