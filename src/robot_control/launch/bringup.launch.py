# K-NAVI 로봇 구동 bringup.
#   흐름: xacro→URDF → robot_state_publisher(tf) → gz sim(월드) → 로봇 스폰 → ros_gz_bridge(토픽 변환)
#   검증: 이걸 띄운 뒤 teleop_twist_keyboard 로 /cmd_vel 을 주면 로봇이 움직여야 한다(핸드오프 §6-1).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control')

    # 머신별 절대경로 하드코딩 제거 — 팀원 누구 경로에서도 동작하게.
    # 워크스페이스 루트 = 설치본에서 4단계 위 (install/robot_control/share/robot_control → WS).
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    navi_dir = os.path.join(ws_root, 'worlds', 'navi_factory')
    default_world = os.path.join(navi_dir, 'world', 'navi_factory', 'navi_factory.sdf')
    models_path = os.path.join(navi_dir, 'models')  # GZ_SIM_RESOURCE_PATH 용

    # 월드는 이 패키지가 소유하지 않는다(로봇만 격리). 외부에서 world 인자로 주입.
    # 창고 월드로 띄우려면:
    #   ros2 launch robot_control bringup.launch.py \
    #     world:=<repo>/worlds/navi_factory/world/navi_factory/navi_factory.sdf
    #   (창고 모델 로딩은 GZ_SIM_RESOURCE_PATH 에 그 models 폴더가 잡혀 있어야 함)


    #LaunchConfiguration()은 인자로 받을 객체에 대한 변수? => 위에 world:=~와 같이 ~를 저장할 객체

    #world 변수를 저장
    world = LaunchConfiguration('world')

    # sim time: Gazebo 가 /clock 을 발행하고, ROS 노드들은 그 시계를 따라야 tf 타임스탬프가 맞는다.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # headless: 테스트/서버 환경에선 GUI 없이(-s) 돌리려고 노출.
    headless = LaunchConfiguration('headless')

    # 실제로 채워 놓은 값, 기본값만
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true 면 gz 를 GUI 없이 서버만(-s) 실행'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='이번 프로젝트의 월드를 넘김(기본: navi_factory, 절대경로 자동계산)'),
    ]





    # 이부분은 로봇이 여러대면 여러개 작성해야함
    # @우진 - 이거 최초 위치 정보도 세팅 가능한데, 해주면 좋을듯?
    robot_info = [
        ('robot.urdf.xacro','robot1','knavi_robot')
    ]
    robot_nodes = []
    for info in robot_info:
        model_info = info[0] # 어떤 모델 urdf를 읽을지
        namespace = info[1]  # 이 로봇의 식별 정보
        name = info[2]       # 이 로봇의 이름(가제보 GUI)
        # xacro 를 실행 시점에 펼쳐 URDF 문자열을 만든다(파일에 미리 펼쳐두지 않음 → 파라미터 바뀌면 자동 반영).
        xacro_file = os.path.join(pkg_share, 'urdf', model_info) # 3번 인자의 객체의 urdf 파일
        # ns 인자를 xacro 에 넘겨 gz 토픽을 /robot1/... 으로 분리(bridge.yaml 과 일치).
        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, ' ns:=', namespace]), value_type=str)

        # 1) robot_state_publisher: URDF + /joint_states → 각 링크의 tf 발행
        # 여기서 state는 상태가 아니라, 로봇의 부품이 어떤 방향인지에 대한 좌표정보들(베터리 정보 이런거x)
        # 이 정보는 처음에는 spawn 토픽으로 pub되어 객체를 스폰 할 수 있게 한다[지금은 이 용도만]

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace =namespace, # 이 namespace를 붙여서 토픽 발생
            parameters=[{
                'robot_description': robot_description,
                         'use_sim_time': use_sim_time}
                         ],
        )

        # 3) 로봇 스폰: robot_state_publisher 가 발행하는 robot_description 토픽에서 모델을 읽어 월드에 생성.
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', namespace+'/robot_description',
                       '-name', name,
                       # 창고 바닥 범위: x[-15~15], y[-25~25], 바닥 z≈0.1.
                       # 이 범위 밖에 스폰하면 허공 낙하하니 반드시 안쪽으로.
                       '-x', '-10',
                       '-y', '-10',
                       '-z', '0.3'],   # 바닥(0.1) 살짝 위에서 떨궈 안착
            output='screen',
        )
        robot_nodes += [rsp, spawn]

    # 2) Gazebo(gz sim) 실행. ros_gz_sim 이 제공하는 표준 런치를 include.
    #    gz_args: 월드 파일 + '-r'(즉시 시뮬 시작). headless 면 '-s'(서버 전용) 추가.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ), #ros_gz_sim 패키지 안에 gz sim 해주는 그런게 있다네요
        launch_arguments={
            # headless=true 면 ' -s'(서버 전용, GUI 없음)를 뒤에 붙인다.
            'gz_args': [world, ' -r -v 4',
                        PythonExpression(["' -s' if '", headless, "' == 'true' else ''"])],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 4) ros_gz_bridge: ROS 2 ↔ Gazebo 메시지 변환.
    #    토픽 목록은 config/bridge.yaml 에 두고 config_file 로 읽는다(CLI 인자 대신 YAML).
    #    → 토픽/로봇 늘 때 yaml 만 고치면 되고, 브릿지 노드는 하나로 충분.
    #    executable='bridge_node' 가 config_file 파라미터를 받는 노드(공식 RosGzBridge 액션과 동일).

    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')  # 브릿지 토픽 목록
    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        parameters=[{'config_file': bridge_config,
                     'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 안정용 평평한 바닥판 스폰 (창고 STL 바닥 접촉 불안정 회피용).
    # STL 바닥(z≈0.2)보다 살짝 위(0.25)에 깔아 로봇이 flat_ground 에 먼저 닿게 함 → 결함 STL 바닥 무시.
    # (벽 충돌은 STL 유지, 바닥만 이 평면이 대신)
    ground_sdf = os.path.join(pkg_share, 'config', 'flat_ground.sdf')
    ground = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', ground_sdf, '-name', 'flat_ground', '-z', '0.25'],
        output='screen',
    )

    # navi_factory 월드의 model:// 참조(바닥 충돌 STL·창고·모델들·OGV 메시)를 gz 가 찾게 리소스 경로 등록.
    # models_path 는 위에서 ws_root 기준으로 계산됨(절대경로 하드코딩 제거).
    # 기존 GZ_SIM_RESOURCE_PATH 값이 있으면 덮어쓰지 않고 뒤에 이어붙인다.
    _existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    set_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path + (os.pathsep + _existing if _existing else ''),
    )

    return LaunchDescription([set_resource] + declare_args + robot_nodes + [gz_sim, ground, bridge])

# rsp → 로봇당 1개 (URDF에 묶임)
# spawn → 로봇당 1번 (각자 생성)
# 브릿지 → 1개 공유 (토픽만 나열)
# gz → 1개 공유 (같은 월드)
#ros2 run teleop_twist_keyboard teleop_twist_keyboard