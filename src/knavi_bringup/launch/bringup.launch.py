# K-NAVI 시스템 bringup — robot_control(몸/배선) + signal_vision(수신호) + auto_drive(주행)를 조립한다.
#   흐름: xacro→URDF → robot_state_publisher(tf) → gz sim(월드) → 로봇 스폰 → ros_gz_bridge(토픽 변환)
#   검증: 이걸 띄운 뒤 teleop_twist_keyboard 로 /cmd_vel 을 주면 로봇이 움직여야 한다(핸드오프 §6-1).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 이 패키지(조립 담당)의 share. 여기엔 이 launch 파일뿐이고, 실제 자원은 아래 robot_share 에 있다.
    pkg_share = get_package_share_directory('knavi_bringup')
    # urdf 와 config(bridge.yaml, twist_mux.yaml, flat_ground.sdf)는 robot_control 이 소유한다 —
    # 이 패키지는 "무엇을 어떤 순서로 띄울지"만 알고, 로봇의 몸과 배선 파일은 갖지 않는다.
    robot_share = get_package_share_directory('robot_control')

    # 머신별 절대경로 하드코딩 제거 — 팀원 누구 경로에서도 동작하게.
    # 워크스페이스 루트 = 설치본에서 4단계 위 (install/knavi_bringup/share/knavi_bringup → WS).
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    navi_dir = os.path.join(ws_root, 'worlds', 'navi_factory')
    default_world = os.path.join(navi_dir, 'world', 'navi_factory', 'navi_factory.sdf')
    models_path = os.path.join(navi_dir, 'models')  # GZ_SIM_RESOURCE_PATH 용

    # 월드는 어느 패키지도 소유하지 않는다(worlds/ 는 ROS 패키지가 아니다). 외부에서 world 인자로 주입.
    # 창고 월드로 띄우려면:
    #   ros2 launch knavi_bringup bringup.launch.py \
    #     world:=<repo>/worlds/navi_factory/world/navi_factory/navi_factory.sdf
    #   (창고 모델 로딩은 GZ_SIM_RESOURCE_PATH 에 그 models 폴더가 잡혀 있어야 함)

    # LaunchConfiguration()은 인자로 받을 객체에 대한 변수? => 위에 world:=~와 같이 ~를 저장할 객체

    # world 변수를 저장
    world = LaunchConfiguration('world')

    # sim time: Gazebo 가 /clock 을 발행하고, ROS 노드들은 그 시계를 따라야 tf 타임스탬프가 맞는다.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # headless: 테스트/서버 환경에선 GUI 없이(-s) 돌리려고 노출.
    headless = LaunchConfiguration('headless')
    # 인식 파이프라인은 실물 웹캠이 필요해서 기본 off.
    # 웹캠 없는 머신에서 bringup 이 실패하지 않도록 기존 시뮬 경로의 동작을 그대로 유지한다.
    enable_camera = LaunchConfiguration('enable_camera')
    enable_patrol = LaunchConfiguration('enable_patrol')    # +
    enable_flat_ground = LaunchConfiguration('enable_flat_ground')
    # 라이다 스캔을 눈으로 보는 뷰어. gz GUI 에서는 레이저가 로봇 주변 선으로만 보이고
    # ROS 쪽으로 실제로 넘어왔는지는 알 수 없어서, 스캔을 확인하려면 어차피 이게 필요하다.
    # 그래서 bringup 에 포함시킨다 — 창이 하나 더 뜨는 게 부담이면 enable_rviz:=false.
    enable_rviz = LaunchConfiguration('enable_rviz')

    # 웹캠 장치 번호. /dev/video0 이 늘 있다는 보장이 없어서 인자로 뺐다 —
    # USB 를 다시 꽂거나 다른 포트에 연결하면 커널이 번호를 다시 매긴다.
    # 확인: ls /dev/video*  (scripts/run_camera_node.sh 는 스스로 골라 준다)
    #
    # 카메라는 로봇당 1대(1:1)이므로 장치 번호도 로봇당 하나여야 한다 — 그래서 실제
    # 값은 아래 robot_info 의 5번째 필드에 로봇별로 적는다. 이 인자는 그 중 첫 로봇의
    # 값만 덮어쓴다. 이 인자가 생긴 이유 자체가 "로봇 한 대인데 웹캠 번호가 바뀌었다"는
    # 상황이고, 로봇이 여러 대면 스칼라 인자 하나로는 어차피 표현할 수 없기 때문이다.
    #
    # 기본값이 '0' 이 아니라 빈 문자열인 이유: '0' 으로 두면 인자를 안 줘도 첫 로봇은
    # 항상 이 값이 이겨서 robot_info 의 5번째 필드가 읽히지 않는 죽은 값이 된다.
    # 빈 문자열을 "안 줬음"으로 쓰면 두 자리 중 어느 쪽이 유효한지가 뒤섞이지 않는다.
    camera_device_id = LaunchConfiguration('camera_device_id')

    # 실제로 채워 놓은 값, 기본값만
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true 면 gz 를 GUI 없이 서버만(-s) 실행'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='이번 프로젝트의 월드를 넘김(기본: navi_factory, 절대경로 자동계산)'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='true 면 실물 웹캠 노드(camera_node)를 함께 띄운다'),
        DeclareLaunchArgument('camera_device_id', default_value='2',
                              description='첫 번째 로봇의 웹캠 장치 번호(/dev/video<N> 의 N)를 '
                                          '덮어쓴다. ls /dev/video* 로 확인. 비워 두면 '
                                          '모든 로봇이 robot_info 에 적힌 자기 값을 쓴다'),
        DeclareLaunchArgument('enable_patrol', default_value='true',
                              description='true 면 트랙 왕복 노드(waypoint_follower)를 함께 띄운다'),
        DeclareLaunchArgument('enable_flat_ground', default_value='true',
                              description='false 면 안정용 flat_ground 를 스폰하지 않는다 '
                                          '(DART+Bullet 에서 바퀴 접지를 막는 문제 있음 — '
                                          '실제로 구르는 DiffDrive 로봇에는 false 권장)'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                              description='false 면 RViz2 를 띄우지 않는다 '
                                          '(라이다 스캔·tf 뷰어, config/knavi.rviz)'),
    ]

    # 이부분은 로봇이 여러대면 여러개 작성해야함
    # @우진 - 이거 최초 위치 정보도 세팅 가능한데, 해주면 좋을듯?
    # 4번째 값은 스폰 pose (x, y, z, yaw) — 로봇마다 겹치지 않게 각자 지정.
    # 5번째 값은 그 로봇을 지휘하는 웹캠의 장치 번호(/dev/video<N> 의 N).
    # robot1(knavi_robot, 센서 없음)은 물리 검증이 끝나서 제거 — 물리 로봇 2대를
    # 같이 돌리면 이 무거운 월드에서 성능 부담이 커진다. 센서(라이다+카메라) 있는
    # mecanum_lift_robot만 남긴다.
    #
    # 여기에 로봇을 하나 더 추가할 때 같이 해야 하는 일: map 루트 프레임 만들기.
    # 로봇은 전역 /tf 안에서 프레임 "이름"으로 갈리는데(rsp 의 frame_prefix), 그래서
    # robot1/odom 과 robot2/odom 이 서로 부모 없는 별개 조각으로 남는다. RViz 는
    # fixed frame 을 하나만 갖기 때문에 그 상태로는 어느 한 대만 보이고 나머지는
    # "No transform from [robot1/base_link] to [robot2/odom]" 만 뜬다.
    # → 이 루프 안에서 로봇마다 static_transform_publisher 로 map → <ns>/odom 을
    #   하나씩 발행할 것. 값은 아래 4번째 필드(스폰 pose)를 그대로 쓰면 된다.
    #   config/knavi.rviz 의 Fixed Frame 도 map 으로 옮긴다.
    #   나중에 SLAM 이 들어오면 이 정적 변환을 로봇별로 대체한다(doc/design.md).
    robot_info = [
        ('mecanum_lift_robot.urdf.xacro', 'robot2', 'mecanum_lift_robot',
         ('0.0', '32.25', '0.3', '-1.5708'),  # entry 트랙 진행 방향(남쪽)으로 정렬
         0),
    ]
    robot_nodes = []
    for index, info in enumerate(robot_info):
        model_info = info[0]  # 어떤 모델 urdf를 읽을지
        namespace = info[1]   # 이 로봇의 식별 정보
        name = info[2]        # 이 로봇의 이름(가제보 GUI)
        spawn_x, spawn_y, spawn_z, spawn_yaw = info[3]
        # 첫 로봇만 launch 인자로 덮어쓸 수 있게 한다(위 camera_device_id 주석 참고).
        # 인자 값이 substitution 이라 launch 파일을 만드는 시점엔 내용을 볼 수 없다. 그래서
        # 파이썬 if 가 아니라 PythonExpression 으로 "인자가 비었으면 robot_info 값"을
        # 실행 시점에 고르게 한다 — 빈 문자열은 파이썬에서 거짓이라 or 가 그대로 동작한다.
        if index == 0:
            device_id = PythonExpression(
                ["'", camera_device_id, "' or '", str(info[4]), "'"])
        else:
            device_id = str(info[4])
        # xacro 를 실행 시점에 펼쳐 URDF 문자열을 만든다(파일에 미리 펼쳐두지 않음 → 파라미터 바뀌면 자동 반영).
        xacro_file = os.path.join(robot_share, 'urdf', model_info)  # 3번 인자의 객체의 urdf 파일
        # ns 인자를 xacro 에 넘겨 gz 토픽을 /robot2/... 으로 분리(bridge.yaml 과 일치).
        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, ' ns:=', namespace]), value_type=str)

        # 1) robot_state_publisher: URDF + /joint_states → 각 링크의 tf 발행
        # 여기서 state는 상태가 아니라, 로봇의 부품이 어떤 방향인지에 대한 좌표정보들(베터리 정보 이런거x)
        # 이 정보는 처음에는 spawn 토픽으로 pub되어 객체를 스폰 할 수 있게 한다[지금은 이 용도만]

        #
        # frame_prefix 를 주는 이유: 네임스페이스는 tf 를 갈라 주지 못한다. tf2_ros 가
        # 토픽 이름을 앞 슬래시 붙은 절대 경로 "/tf" / "/tf_static" 으로 박아 놔서
        # (transform_broadcaster.hpp, transform_listener.hpp), ns 를 robot2 로 줘도 이
        # 노드는 여전히 전역 /tf 로 발행한다. 로봇이 2대가 되면 양쪽이 같은 토픽에 같은
        # 이름 base_link 를 쏴서 tf 트리가 뒤엉킨다. 갈리는 건 프레임 "이름"뿐이라
        # 여기서 robot2/ 를 붙인다 — urdf 의 DiffDrive frame_id 와 센서 gz_frame_id 도
        # 같은 접두어를 쓰므로($(arg ns)/...) 두 쪽이 한 트리로 이어진다.
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,  # 이 namespace를 붙여서 토픽 발생
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': namespace + '/',
                'use_sim_time': use_sim_time}],
        )

        # 3) 로봇 스폰: robot_state_publisher 가 발행하는 robot_description 토픽에서 모델을 읽어 월드에 생성.
        # spawn = Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     arguments=['-topic', namespace+'/robot_description',
        #                '-name', name,
        #                # 창고 바닥 범위: x[-15~15], y[-25~25], 바닥 z≈0.1.
        #                # 이 범위 밖에 스폰하면 허공 낙하하니 반드시 안쪽으로.
        #                '-x', '-10',
        #                '-y', '-10',
        #                '-z', '0.3'],   # 바닥(0.1) 살짝 위에서 떨궈 안착
        #     output='screen',
        # )
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', namespace+'/robot_description',
                       '-name', name,
                       '-x', spawn_x,
                       '-y', spawn_y,
                       '-z', spawn_z,
                       '-Y', spawn_yaw],
            output='screen',
        )

        # 4) twist_mux: 여러 속도 명령 소스 중 우선순위가 가장 높은 하나만 cmd_vel 로 통과시킨다.
        #    자작 노드가 아니라 설치 패키지(ros-jazzy-twist-mux)를 그대로 쓴다.
        #    - 로봇마다 하나씩 필요하므로 이 루프 안에 둔다(네임스페이스로 분리).
        #    - 'cmd_vel_out' 은 twist_mux 가 쓰는 기본 출력 토픽 이름. 이걸 'cmd_vel' 로 리맵하면
        #      네임스페이스가 붙어 /robot2/cmd_vel 이 되고, bridge.yaml 항목과 맞아떨어진다.
        #    - 입력 토픽/우선순위/timeout 은 config/twist_mux.yaml 참고.
        twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            namespace=namespace,
            parameters=[os.path.join(robot_share, 'config', 'twist_mux.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            output='screen',
        )

        # 5) 왕복 트랙 팔로워 : odom 보고 cmd_vel_auto 발행 (twist_mux 최하위 우선순위로 들어감).
        patrol = Node(
            package='auto_drive',
            executable='waypoint_follower',
            namespace=namespace,
            condition=IfCondition(enable_patrol),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )

        # 6) 웹캠 노드 (P0 입력) : 로봇 1대에 카메라 1대. 그래서 로봇 루프 안에 있다.
        #    로봇마다 자기를 지휘하는 신호수를 자기 카메라로 본다는 뜻이고, 그 대응은
        #    robot_info 의 5번째 필드(장치 번호)가 정한다.
        #
        #    리맵이 없는 이유: 네임스페이스를 붙였으니 이 노드가 발행하는 cmd_vel_gesture 는
        #    자동으로 <ns>/cmd_vel_gesture 가 되고, 같은 네임스페이스의 twist_mux 가 그대로
        #    구독한다. 예전에는 이 노드가 로봇 밖에 있어서 '/robot1/cmd_vel_gesture' 로
        #    리맵했는데, robot1 을 robot_info 에서 빼면서 그 리맵이 아무도 구독하지 않는
        #    토픽을 가리키게 됐다 — 수신호를 인식해도 로봇이 안 움직였다. 네임스페이스로
        #    묶으면 그런 식으로 어긋날 자리가 없어진다.
        #
        #    use_sim_time 을 주지 않는 이유: 실물 카메라는 Gazebo 시계가 아니라 실제 시간으로 돈다.
        #
        #    device_id 를 넘기는 이유: 노드 기본값은 0 인데 /dev/video0 이 늘 있는 건 아니다.
        #    USB 를 다시 꽂거나 다른 포트에 연결하면 커널이 번호를 다시 매기고, 그러면
        #    camera_node 가 장치를 못 열어 RuntimeError 로 죽는다.
        #      ros2 launch knavi_bringup bringup.launch.py camera_device_id:=1
        #    주의: 로봇 수만큼 실물 웹캠이 꽂혀 있어야 한다. 장치가 모자라면 그 로봇의
        #    camera_node 만 못 뜨고, 나머지 로봇과 시뮬레이션은 그대로 돈다.
        camera = Node(
            package='signal_vision',
            executable='camera_node',
            namespace=namespace,
            condition=IfCondition(enable_camera),
            parameters=[{'device_id': ParameterValue(device_id, value_type=int)}],
            output='screen',
        )
        robot_nodes += [rsp, spawn, twist_mux, patrol, camera]

    # 2) Gazebo(gz sim) 실행.
    #    ros_gz_sim 이 제공하는 gz_sim.launch.py 를 include 하던 걸 걷어냈다 — 그건
    #    'ruby <gz경로> sim <gz_args> --force-version 8' 을 shell=True 로 감싸 돌리는
    #    래퍼인데, 이 래퍼를 통해 실행하면 이 월드(mesh 수백 개)에서 몇 초 뒤 gz sim 이
    #    크래시(SIGSEGV 등) 없이 조용히 종료돼 버리는 문제가 있었다 — camera_node 유무,
    #    -v 4 유무와 무관하게 재현됐다. 반면 태스크 2(run_gazebo.sh)처럼 'gz sim' 을
    #    ExecuteProcess 로 직접 부르면(ruby/shell 래핑 없이) 문제없이 계속 돈다. 그래서
    #    여기서도 태스크 2와 동일한 방식으로 직접 실행한다.
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '--render-engine', 'ogre', world, '-r'],
        # output='screen',
        output='log',
        on_exit=Shutdown(),
    )
    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-s', '--render-engine', 'ogre', world, '-r'],
        # output='screen',
        output='log',
        on_exit=Shutdown(),
    )

    # 4) ros_gz_bridge: ROS 2 ↔ Gazebo 메시지 변환.
    #    토픽 목록은 config/bridge.yaml 에 두고 config_file 로 읽는다(CLI 인자 대신 YAML).
    #    → 토픽/로봇 늘 때 yaml 만 고치면 되고, 브릿지 노드는 하나로 충분.
    #    executable='bridge_node' 가 config_file 파라미터를 받는 노드(공식 RosGzBridge 액션과 동일).

    bridge_config = os.path.join(robot_share, 'config', 'bridge.yaml')  # 브릿지 토픽 목록
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
    #
    # 주의: 이 얇은 박스가 이 월드의 물리 설정(DART+Bullet 충돌 검출기)과 만나면
    # 바퀴 접지가 사실상 고정돼(뒤틀린 접촉 해석으로 추정) 로봇이 cmd_vel 을 받아도
    # 전혀 전진하지 못하는 문제가 있었다 — 빈 월드/이 월드의 STL 바닥에 직접 스폰했을
    # 때는 정상 이동함을 확인. DiffDrive로 실제 굴러가야 하는 로봇(Method B)에는
    # enable_flat_ground:=false 로 꺼서 STL 바닥에 직접 놓는다.
    ground_sdf = os.path.join(robot_share, 'config', 'flat_ground.sdf')
    ground = Node(
        package='ros_gz_sim',
        executable='create',
        condition=IfCondition(enable_flat_ground),
        arguments=['-file', ground_sdf, '-name', 'flat_ground', '-z', '0.25'],
        output='screen',
    )

    # RViz2: 라이다 스캔과 tf 트리를 보는 뷰어. 로봇 루프 밖에 하나만 둔다 — RViz 는
    # 로봇별 노드가 아니라 전역 /tf 를 통째로 보는 뷰어라서, 로봇이 늘어도 창 하나면 된다
    # (늘어난 로봇의 스캔은 knavi.rviz 에 LaserScan 디스플레이를 추가해서 본다).
    #
    # use_sim_time 을 주는 이유: 스캔과 tf 의 타임스탬프가 Gazebo 시계다. RViz 가 벽시계로
    # 돌면 "메시지가 미래에서 왔다 / 너무 오래됐다"로 판단해 스캔이 안 그려진다.
    # headless 를 함께 보는 이유: enable_rviz 가 기본 켜짐이라, 그것만 보면
    # 'headless:=true' (GUI 없이 서버만) 를 준 실행에서도 RViz 창이 떠 버린다 —
    # headless 를 준 사람이 원한 것과 정반대다. 두 인자 중 headless 를 이기게 둔다.
    rviz_config = os.path.join(robot_share, 'config', 'knavi.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(PythonExpression(
            ["'", enable_rviz, "' == 'true' and '", headless, "' != 'true'"])),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    # navi_factory 월드의 model:// 참조(바닥 충돌 STL·창고·모델들·OGV 메시)를 gz 가 찾게 리소스 경로 등록.
    # models_path 는 위에서 ws_root 기준으로 계산됨(절대경로 하드코딩 제거).
    # 기존 GZ_SIM_RESOURCE_PATH 값이 있으면 덮어쓰지 않고 뒤에 이어붙인다.
    _existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    set_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path + (os.pathsep + _existing if _existing else ''),
    )

    # 웹캠 노드는 로봇당 1개라서 위 robot_info 루프 안에서 만들어진다(robot_nodes 에 포함).
    return LaunchDescription([set_resource] + declare_args + robot_nodes +
                             [gz_sim_gui, gz_sim_headless, ground, bridge, rviz])

# rsp → 로봇당 1개 (URDF에 묶임)
# spawn → 로봇당 1번 (각자 생성)
# twist_mux → 로봇당 1개 (명령 소스 중재)
# patrol → 로봇당 1개 (자기 트랙을 따라감)
# camera → 로봇당 1개 (로봇 1대에 웹캠 1대, 네임스페이스로 분리)
# 브릿지 → 1개 공유 (토픽만 나열)
# gz → 1개 공유 (같은 월드)
#
# 수동 주행 검증: twist_mux 가 붙은 뒤로는 /robot2/cmd_vel 에 직접 쓰지 않고
# mux 입력(cmd_vel_teleop)으로 넣는다. 안 그러면 mux 출력과 충돌한다.
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#     --ros-args -r /cmd_vel:=/robot2/cmd_vel_teleop
