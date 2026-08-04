# Graph Report - /Users/jaeseong/TeamProject1/Signal-Simulation/src  (2026-08-04)

## Corpus Check
- Corpus is ~38,178 words - fits in a single context window. You may not need a graph.

## Summary
- 645 nodes · 997 edges · 78 communities (66 shown, 12 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 38 edges (avg confidence: 0.66)
- Token cost: 69,304 input · 0 output

## Community Hubs (Navigation)
- Mission Follower Patrol
- Marker Vision Node
- Hand Capture Warning
- Vision Env Setup Tools
- Ground Truth TF Publisher
- Camera Capture Rendering
- Landmarker Model Loading
- Actor Geometry Transforms
- E-Stop Safety Node
- Gesture Command Publisher
- Camera Node Main Loop
- Vision Inference Runtime
- Inference Console Publisher
- Webcam Frame Source
- Inference UI HUD
- Gesture Inference Wrapper
- Inference Base Class
- Signal Zone Gating
- Inference Metrics Tracking
- Vision Track Following
- Gesture Model Training
- Model Load Signal Stabilizer
- Hand Feature Extractor
- Lidar Scan Ray Viz
- World Markers SDF Compose
- PEP257 Style Tests
- Frame Handoff Buffer
- Actor Marker Loader
- Actor Track Interpolation
- CameraNode Init Params
- Training Data Augmentation
- Bridge TwistMux Rationale
- Webcam Image Publisher
- Dev Watch Header Tools
- Twist Mux Priority Topics
- SignLSTM Model
- Copyright Style Test (auto_drive)
- Copyright Style Test (knavi_bringup)
- Copyright Style Test (robot_control)
- Copyright Style Test (signal_vision)
- Model URI Resolution
- Flake8 Style Test (auto_drive)
- Flake8 Style Test (knavi_bringup)
- PEP257 Style Test (knavi_bringup)
- Flake8 Style Test (robot_control)
- Flake8 Style Test (signal_vision)
- PEP257 Style Test (signal_vision)
- Robot Info Dead Robot1
- Cmd_vel Twist Type Contract
- Gesture Command Topics
- Teleop Topic
- Camera Node Package Init
- Clock Topic Bridge
- Camera Image Topic Bridge
- Joint States Topic Bridge
- Odometry Topic Bridge
- Pose Ground Truth Bridge
- Lidar Scan Topic Bridge
- ROS-GZ Bridge Node

## God Nodes (most connected - your core abstractions)
1. `CameraNode` - 20 edges
2. `HandWarning` - 19 edges
3. `FeatureExtractor` - 18 edges
4. `SignalStabilizer` - 18 edges
5. `main()` - 16 edges
6. `GestureInference` - 15 edges
7. `main()` - 15 edges
8. `EstopNode` - 14 edges
9. `draw_detections()` - 13 edges
10. `ButtonBar` - 13 edges

## Surprising Connections (you probably didn't know these)
- `test_copyright()` --calls--> `main()`  [INFERRED]
  auto_drive/test/test_copyright.py → signal_vision/signal_vision/vision_hand/training/stress_test.py
- `test_copyright()` --calls--> `main()`  [INFERRED]
  knavi_bringup/test/test_copyright.py → signal_vision/signal_vision/vision_hand/training/stress_test.py
- `test_pep257()` --calls--> `main()`  [INFERRED]
  knavi_bringup/test/test_pep257.py → signal_vision/signal_vision/vision_hand/training/stress_test.py
- `test_copyright()` --calls--> `main()`  [INFERRED]
  robot_control/test/test_copyright.py → signal_vision/signal_vision/vision_hand/training/stress_test.py
- `test_pep257()` --calls--> `main()`  [INFERRED]
  auto_drive/test/test_pep257.py → signal_vision/signal_vision/vision_hand/training/stress_test.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **robot2 sensor/control topics bridged as one set by ros_gz_bridge** — robot_control_config_bridge_ros_gz_bridge_node, robot_control_config_bridge_robot2_cmd_vel, robot_control_config_bridge_robot2_odom, robot_control_config_bridge_robot2_joint_states, robot_control_config_bridge_tf, robot_control_config_bridge_robot2_scan, robot_control_config_bridge_robot2_pose_gt, robot_control_config_bridge_robot2_camera_image [EXTRACTED 1.00]
- **twist_mux priority-ordered velocity source arbitration** — robot_control_config_twist_mux_node, robot_control_config_twist_mux_estop, robot_control_config_twist_mux_gesture, robot_control_config_twist_mux_teleop, robot_control_config_twist_mux_estop_slow, robot_control_config_twist_mux_marker_slow, robot_control_config_twist_mux_auto [EXTRACTED 1.00]
- **cross-file Twist type contract between bridge and twist_mux** — robot_control_config_bridge_robot2_cmd_vel, robot_control_config_twist_mux_use_stamped, robot_control_config_twist_mux_node [EXTRACTED 1.00]

## Communities (78 total, 12 thin omitted)

### Community 0 - "Mission Follower Patrol"
Cohesion: 0.07
Nodes (40): load_dispatch_routes(), main(), MissionFollower, Node, 경로점 중 어느 인덱스가 station(원판) 위인지 집합으로 돌려준다., 수신호석 대기 → 파견 명령 수신 → 존 왕복 → 복귀 대기를 반복한다., tracks.yaml 의 경로를 읽고, 대기(WAITING) 상태로 시작한다., 파견 명령(존 이름) 수신. 대기 중일 때만 받는다. camera_node 는 라벨을 인식하는 동안 매 프레임(~12회/초) 다시 보내므로,… (+32 more)

### Community 1 - "Marker Vision Node"
Cohesion: 0.07
Nodes (26): main(), MarkerVision, Node, 근접도에 따라 0~1 사이로 부드럽게 줄어드는 배율. 너무 멀면 None(감속 불필요)., load_model(), predict(), bgr_to_imgmsg(), DetectNode (+18 more)

### Community 2 - "Hand Capture Warning"
Cohesion: 0.14
Nodes (17): HandWarning, process_frame(), 프레임 1장을 읽어 검출까지 수행 (수집·추론 공용). 반환: (frame, 손결과, 포즈결과) 또는 (None,)*3. t0는 이 캡처…, 학습 데이터 수집 중 손이 프레임 밖으로 나가면 그 구간은 0으로 채워진 저품질 샘플이 되므로, 수집자가 바로 알아챌 수 있도록 손 이탈,…, choose_label(), countdown(), hand_detected_ratio(), main() (+9 more)

### Community 3 - "Vision Env Setup Tools"
Cohesion: 0.19
Nodes (18): main(), main(), find_python312(), install_python312(), main(), Path, 3.12 인터프리터를 실행할 커맨드(argv)를 찾는다. 없으면 None., Debian/Ubuntu는 ensurepip이 core python3.X 패키지가 아니라 별도 python3.X-venv 패키지에 들어있어,… (+10 more)

### Community 4 - "Ground Truth TF Publisher"
Cohesion: 0.15
Nodes (16): GroundTruthTf, main(), pose_to_transform(), Node, geometry_msgs/Pose 를 (translation, rotation) 튜플로., pose_gt 와 odom 을 받아 map → <ns>/odom 보정 변환을 발행한다., quaternion_inverse(), quaternion_multiply() (+8 more)

### Community 5 - "Camera Capture Rendering"
Cohesion: 0.18
Nodes (17): _camera_backend(), choose_camera_index(), draw_detections(), draw_landmarks(), draw_pose(), draw_skeleton(), LensHealthMonitor, list_cameras() (+9 more)

### Community 6 - "Landmarker Model Loading"
Cohesion: 0.16
Nodes (18): create_hand_landmarker(), create_pose_landmarker(), ensure_hand_model(), _ensure_model(), ensure_pose_model(), Path, OpenCV BGR 프레임 → MediaPipe 입력 이미지로 변환한다. OpenCV는 색상 채널을 BGR 순서로 다루지만 MediaPipe는…, 모델 파일이 없으면 다운로드한다 (손/포즈 모델이 공유하는 로직). (+10 more)

### Community 7 - "Actor Geometry Transforms"
Cohesion: 0.19
Nodes (15): quaternion_from_rpy(), roll-pitch-yaw 를 쿼터니언으로 (Z-Y-X 순, SDF·ROS 공통 규약)., apply_up_axis_fix(), dae_header(), parse_pose(), parse_pose_text(), parse_scale(), <mesh><scale> 을 (sx, sy, sz) 로. 없으면 1 배. (+7 more)

### Community 8 - "E-Stop Safety Node"
Cohesion: 0.15
Nodes (8): EstopNode, main(), Node, 라이다 스캔을 보고 진행 경로에 장애물이 있으면 로봇을 세운다. 용도는 자율 회피가 아니다. 로봇은 정해진 트랙을 가고, 그 길 위로 사람이…, 진행 통로(폭 2*half_width, 길이 limit) 안에 들어온 반사들. msg 에서 읽는 필드: angle_min, angle_max…, 연속한 광선 window 칸 안에 유효 반사가 min_hits 개 이상인 구간만 남긴다. 남은 반사 수와 그중 최근접 거리를 돌려준다. 뭉치지…, 감속 링 안이면 순찰 명령의 전진 속도를 제동 곡선 아래로 깎아 재발행한다., 진행 통로 안의 장애물 거리에 따라 감속(cmd_vel_estop_slow)·정지(cmd_vel_estop)한다.

### Community 9 - "Gesture Command Publisher"
Cohesion: 0.16
Nodes (11): GestureCommandPublisher, 책임 4 — 라벨을 로봇 속도 명령으로 바꿔 발행. config/twist_mux.yaml 은 이 변환을 별도 command_node 가 맡는…, 라벨 문자열을 받아 세 토픽으로 내보낸다. gesture (std_msgs/String) — 관측·기록용. rosbag 에 남겨야 "언제 무슨…, 퍼블리셔 2개를 만들고, 라벨별 Twist 를 미리 만들어 캐싱한다. 인자: linear_speed: FORWARD 전진 속도 (m/s)…, 라벨 하나를 발행한다. 워커 스레드에서 호출된다. (rclpy 퍼블리셔는 스레드 안전하므로 워커에서 바로 발행해도 된다) 인자: label:…, 책임 2 — 이미지 발행. 웹캠 프레임을 sensor_msgs/Image 로 만들어 image_webcam 토픽에 내보낸다. "발행"은 다른…, 수신호 라벨의 단일 출처(single source of truth). 왜 별도 파일인가: 추론(inference.py)은 라벨을 "만들고",…, 조립자 — 부품들을 배선하고 타이머/스레드만 관리한다. 이 파일은 cv2 도 모델도 Twist 도 직접 다루지 않는다. 그래서 import… (+3 more)

### Community 10 - "Camera Node Main Loop"
Cohesion: 0.13
Nodes (11): CameraNode, main(), Node, pose_gt 콜백(실행기 스레드). 게이트를 갱신하고 전환만 로그로 남긴다., 렌더 워커 본체. 추론 워커가 깨울 때만 한 번 그리고 다시 잠든다., 워커 스레드에서 노드 종료를 요청한다., 워커 본체. 프레임을 읽어 그 자리에서 추론하고 라벨을 발행한다. 주기를 관리하지 않는 이유: read() 가 다음 프레임까지 블로킹이라 이…, 이번 프레임의 추론 결과를 렌더 워커에게 넘기고 깨운다. 워커 스레드 전용. (+3 more)

### Community 11 - "Vision Inference Runtime"
Cohesion: 0.14
Nodes (13): close(), _get_runtime(), infer(), Path, 웹캠 한 프레임을 읽어 특징추출 → LSTM 추론 → 안정화 필터까지 진행하고 (signal, confidence)를 반환한다. 사람…, 가장 최근 infer() 결과를 관절/확률 패널 GUI로 띄운다. True를 반환하면 종료 요청(q/ESC)., 카메라/HUD/추출기 리소스를 정리한다. 종료 시 반드시 호출., infer()/show_gui()가 공유하는 상태. 모듈 전역에 하나만 만든다(_get_runtime). (+5 more)

### Community 12 - "Inference Console Publisher"
Cohesion: 0.15
Nodes (8): ConsolePublisher, make_publisher(), predict.py의 --publish 옵션에 따라 백엔드를 골라 생성하는 팩토리. kind: "udp" | "rosbridge" | 그…, 전송 없이 콘솔 출력만 (기본값, --publish 미지정 시). predict.py가 확정/하트비트 시점에 이미 print()로 로그를…, JSON을 UDP로 송신한다 (의존성·서버 불필요 — 로컬 디버깅용 백엔드)., rosbridge 웹소켓으로 ROS 2 토픽(std_msgs/String)을 발행한다. 서버가 없거나 끊겨도 추론을 막지 않는다 — 전송…, RosbridgePublisher, UdpPublisher

### Community 13 - "Webcam Frame Source"
Cohesion: 0.12
Nodes (9): FrameSource, 책임 1 — 하드웨어 캡처. 이 파일은 rclpy 를 import 하지 않는다. 일부러 그렇다. - ROS 없이 단독으로 돌려볼 수…, 아직 장치를 들고 있으면 True. release() 후에는 False., 프레임 한 장을 읽는다. 반환: numpy.ndarray (H, W, 3) uint8 BGR — 성공 None — 실패했거나 이미…, 장치를 반납한다. 여러 번 불러도 안전하다., 웹캠 장치 하나를 열고 프레임을 읽어 준다. cv2.VideoCapture 의 얇은 래퍼. 로그를 직접 찍지 않는다. 상태는 프로퍼티로 노출만…, 장치를 열고 해상도/FPS 를 설정한다. 열지 못하면 RuntimeError. 인자: device_id: /dev/video<N> 의 N…, 요청했던 (width, height). (+1 more)

### Community 14 - "Inference UI HUD"
Cohesion: 0.17
Nodes (11): 책임 3 — 추론. 이 프로젝트의 인지(perception) 경계면. doc/design.md: 카메라 기반 이미지 프로세싱은 별도…, draw_emergency_toast(), draw_probability_panel(), find_font(), Hud, make_text_drawer(), predict.py 메인 루프가 매 프레임 호출하는 렌더러. 폰트 로딩처럼 한 번만 준비하면 되는 상태를 인스턴스에 들고 있다가…, 프레임에 확률 패널 + 헤더/상태 텍스트를 그려 창에 표시한다. 반환: 종료 요청 여부(q/ESC). (+3 more)

### Community 15 - "Gesture Inference Wrapper"
Cohesion: 0.16
Nodes (7): GestureInference, 프레임 한 장을 보고 수신호 라벨을 반환한다. 워커 스레드에서 호출된다. 인자: frame: numpy.ndarray,…, 이번 프레임의 결과를 렌더 스레드에 넘길 불변 묶음으로 뜬다. 추론 스레드 전용., 직전 infer() 의 스냅샷을 꺼내고 비운다. 추론 스레드에서만 부른다., infer() 의 실제 본문. 계약과 주의사항은 infer() 독스트링 참고., 추론 결과를 HUD 창에 그린다. 렌더 스레드 전용. Signal-Vision 의 show_gui() 를 쓰지 않는 이유: 그쪽은…, HUD 창을 닫는다. 렌더 스레드가 끝난 뒤 조립자가 부른다.

### Community 16 - "Inference Base Class"
Cohesion: 0.14
Nodes (8): InferenceBase, 모델/파이프라인을 로딩해 self 에 보관한다. 워커 시작 전 1회 호출된다. 무거운 초기화(가중치 로딩 등)는 반드시 여기서 한 번만 한다.…, 프레임 한 장을 보고 수신호 라벨을 반환한다. 워커 스레드에서 호출된다. 인자: frame: numpy.ndarray,…, 직전 infer() 가 남긴 렌더용 결과를 꺼낸다(꺼내면 비운다). 워커 스레드에서 호출. 반환: show() 에 그대로 넘길 객체, 또는…, take_render_payload() 가 준 결과를 화면에 그린다. **렌더 스레드**에서 호출된다. 인자: payload:…, GUI 창 등 리소스를 정리한다. 렌더 스레드가 끝난 뒤 1회 호출된다., 프레임 → 수신호 라벨. 실제 모델은 이 클래스를 상속해서 붙인다. ┌─ 붙이는 방법…, 로거만 받아 둔다. 모델 로딩은 load_model() 에서 한다. 인자: logger: .info/.warn/.error 를 가진…

### Community 17 - "Signal Zone Gating"
Cohesion: 0.16
Nodes (10): normalize_angle(), 책임 5 — 수신호를 "받아도 되는 자리"인지 판정하는 게이트. 웹캠은 로봇 위치를 모르므로 수신호는 어디서든 인식된다. 그대로 두면 트랙을…, Pose 하나로 판정을 갱신하고 그 결과를 돌려준다(로그용)., 지금 명령을 내보내도 되는가. 워커 스레드에서 매 라벨마다 불린다., 쿼터니언에서 yaw(rad)만 뽑는다. waypoint_follower.py 의 동일 함수를 복사한 것 — 수식 6줄 때문에…, 각도를 [-pi, pi] 로 접는다. 출처는 yaw_from_quaternion 과 같다., "신호 대기 지점에서 신호수 쪽을 보고 있는가"를 pose 만으로 판정한다. 스레드 규칙: update_pose() 는 실행기…, 게이트 파라미터를 검증하고 저장한다. 인자: signal_point: 로봇이 서서 신호를 받는 지점 (x, y) spot: 신호수가 서는 지점… (+2 more)

### Community 18 - "Inference Metrics Tracking"
Cohesion: 0.15
Nodes (7): InferenceMetrics, 추론 파이프라인의 처리량·지연 통계를 모으는 유틸리티. 노드 파일에서 분리한 이유: inline 방식(camera_node 안에서 추론)과…, 캡처/추론/유실 횟수와 추론 소요시간을 모아 구간 통계로 낸다., 프레임을 한 장 캡처했다. 캡처 스레드에서만 호출한다., 큐가 차서 프레임을 한 장 버렸다. 캡처 스레드에서만 호출한다., 추론 한 번이 끝났다(elapsed_ms = 소요시간). 추론 워커에서만 호출한다., 마지막 호출 이후 구간의 통계를 한 줄 문자열로 만든다. 카운터를 0 으로 리셋하지 않고 직전값과의 차이로 구간값을 구한다. 리셋하면 읽는…

### Community 19 - "Vision Track Following"
Cohesion: 0.29
Nodes (11): estimate_track(), fit_curve(), obstacle_mask(), ndarray, 로봇 바로 앞 트랙의 (offset, curvature, confidence)를 추정한다. offset: 화면 하단(로봇 바로 앞)에서 트랙…, YOLO 탐지 박스(Vehicle/Person 등) 영역을 True로 표시한 마스크. 트랙 색상 마스크에서 이 영역을 빼서, 트랙 위에 서…, 화면에 여러 트랙 분기가 동시에 보여도, 로봇 바로 앞(화면 하단 중앙)에 맞닿은 연결 성분 하나만 "지금 타고 있는 트랙"으로 고른다.…, 선택된 트랙 성분을 가까운 밴드 → 먼 밴드로 나눠 각 밴드의 x중심을 뽑고, 다항식으로 피팅한다(x = f(y)). 코너에서 앞으로 얼마나… (+3 more)

### Community 20 - "Gesture Model Training"
Cohesion: 0.27
Nodes (10): evaluate(), load_dataset(), main(), ndarray, Path, train 70 / val 15 / test 15 층화 분할. stratify=y로 각 분할에 클래스 비율이 원본과 동일하게 유지되도록 한다…, 정확도와 예측값을 반환한다 (val/test 평가, stress_test.py도 재사용)., 데이터 로드 → 층화 분할 → LSTM 학습(early stopping) → test 평가 → 모델 저장. val 정확도가 --patience… (+2 more)

### Community 21 - "Model Load Signal Stabilizer"
Cohesion: 0.22
Nodes (6): 모델/파이프라인을 로딩해 self 에 보관한다. 워커 시작 전 1회 호출된다. 예) import mediapipe as mp…, ndarray, 윈도우 대부분에서 사람(포즈)이 안 잡힐 때 — 유예 없이 즉시 해제. is_blurred가 True면 "사람이 없는 게 아니라 렌즈가 막혀서…, 실제 예측 확률 하나로 안정화 필터를 한 스텝 진행한다., 모델 확률을 안정화 필터(임계값 + 연속 일치 + 히스테리시스)에 통과시켜 확정 신호를 관리한다. "확정…, SignalStabilizer

### Community 22 - "Hand Feature Extractor"
Cohesion: 0.24
Nodes (6): FeatureExtractor, ndarray, 수집,추론이 공통으로 쓰는 진입점. HandLandmarker + PoseLandmarker를 한 번에 들고 있다가, 프레임을 넣으면 두 모델…, 랜드마커가 들고 있는 네이티브 리소스 해제 (종료 시 반드시 호출)., 그레이스케일 프레임의 선명도를 라플라시안(2차 미분) 분산으로 측정한다. 라플라시안은 엣지(밝기가 급변하는 지점)에서 값이 크고, 평탄한…, _sharpness_score()

### Community 23 - "Lidar Scan Ray Viz"
Cohesion: 0.27
Nodes (5): main(), Node, LaserScan 을 광선 선분(Marker LINE_LIST)으로 다시 그려 발행한다., 원점 → (angle, length) 선분 하나를 LINE_LIST 용으로 두 점씩 넣는다., ScanRays

### Community 24 - "World Markers SDF Compose"
Cohesion: 0.24
Nodes (7): compose(), 부모 pose 위에 자식 pose 를 얹는다. 위치는 부모 yaw 로 돌리고 각도는 더한다. 일반적인 3축 회전 합성이 아니라 이 형태로…, main(), Node, 월드 SDF 의 visual 을 읽어 MarkerArray 한 벌로 발행한다., 월드를 다시 읽어 마커 한 벌을 만들고 발행한다., WorldMarkers

### Community 25 - "PEP257 Style Tests"
Cohesion: 0.20
Nodes (8): linter, pep257, test_pep257(), linter, pep257, test_pep257(), main(), 저장된 모델을 로드해 조건별(노이즈/손 소실/시간 신축/이동·스케일/렌즈 왜곡)로 test 셋을 오염시키고, 깨끗한 test 셋 대비 정확도가…

### Community 26 - "Frame Handoff Buffer"
Cohesion: 0.22
Nodes (5): LatestFrameBuffer, 캡처 스레드와 추론 워커 사이의 프레임 전달 통로. 책임 넷 중 하나가 아니라 배관(plumbing)이다. 다만 "낡은 프레임을 버린다"는…, 캡처 스레드 → 워커 스레드로 프레임을 넘기는 한 칸짜리 버퍼. 보관 정책은 옛 LatestFrameQueue 와 같다. 항상 최신 한 장만…, 프레임을 넣는다. 안에 있던 낡은 프레임은 덮어쓴다. 반환: True — 낡은 프레임을 한 장 버렸다(호출자가 유실로 집계) False —…, 프레임을 꺼낸다. timeout 초 안에 없으면 None. 타임아웃을 두는 이유: 프레임이 안 와도 워커가 주기적으로 깨어나 종료 플래그를…

### Community 27 - "Actor Marker Loader"
Cohesion: 0.32
Nodes (5): ActorMarkers, main(), Node, 이 actor 를 그릴 메시를 고른다: link 의 visual 이 먼저, 없으면 skin. 이 월드의 actor 8명 중 7명은 <skin>…, SDF 의 <actor> 궤적을 풀어 사람 마커를 주기적으로 발행한다.

### Community 28 - "Actor Track Interpolation"
Cohesion: 0.29
Nodes (5): ActorTrack, 두 각도 사이의 최단 차이를 (-pi, pi] 로 돌려준다., actor 하나의 메시와 궤적. 시각을 주면 그때의 pose 를 돌려준다., 위치는 선형, 각도는 최단 경로로 보간한다., shortest_angle()

### Community 29 - "CameraNode Init Params"
Cohesion: 0.25
Nodes (5): 파라미터를 선언하고 값을 검증해서 dict 로 돌려준다. 생성자에서 한 번만 읽으므로 런타임 변경은 반영되지 않는다. ros2 param…, 추론 구현체를 만든다. 모델을 갈아끼우는 지점은 여기 한 곳뿐이다. 외부 리포지토리 모델이 준비되면 이 한 줄만 바꾼다: return…, 파라미터를 읽고, 부품 넷을 만들고, 워커와 타이머를 띄운다., load_zone_points(), tracks.yaml 에서 signal_point 와 hand_signal_spot 좌표를 읽는다. 파일이나 키가 없으면…

### Community 30 - "Training Data Augmentation"
Cohesion: 0.39
Nodes (7): augment_batch(), augment_sequence(), ndarray, 배치 (B, T, 150) 전체에 시퀀스별로 각각 독립적인 무작위 변형을 적용한다. 시퀀스마다 augment_sequence를 따로…, 재생 속도를 바꿨다가 원래 길이(T)로 다시 리샘플링한다. 예를 들어 factor=1.25면 "1.25배 빠르게 재생한 동작"의 프레임 위치를…, 시퀀스 1개 (T, 150)에 무작위 악조건 변형을 순서대로 적용한다. 각 변형은 독립적으로 "적용할지(P_*)"를 굴리고, 적용되면…, _time_warp()

### Community 31 - "Bridge TwistMux Rationale"
Cohesion: 0.29
Nodes (7): robot_state_publisher (ns robot2, publishes link tf to global /tf), /tf bridge (ROS /tf <-> gz /robot2/tf, global not namespaced), tf2_ros hard-coded absolute /tf, /tf_static topic names (transform_listener.hpp, transform_broadcaster.hpp), 핸드오프 (handoff) document, sections 6 and 10, twist_mux locks feature (Bool switch to block priorities below a threshold; unused), twist_mux node/config (/** ros__parameters, arbitrates cmd_vel sources), ros-jazzy-twist-mux 4.5.0 package's twist_mux_topics.yaml format (followed as syntax basis)

### Community 32 - "Webcam Image Publisher"
Cohesion: 0.29
Nodes (4): ImagePublisher, 프레임을 sensor_msgs/Image 로 발행한다. 기본은 꺼져 있다(enabled=False). 이 노드는 추론을 내부 큐로 넘기므로…, 퍼블리셔를 만든다. enabled=False 여도 퍼블리셔 자체는 만들어 둔다. (토픽 그래프에 항상 보이는 편이 디버깅에 낫다. ros2…, 프레임 한 장을 발행한다. 반환: True — 발행했거나, 꺼져 있어서 의도적으로 건너뛰었다 False — 종료 중이라 발행하지…

### Community 33 - "Dev Watch Header Tools"
Cohesion: 0.48
Nodes (4): stamp(), iter_py_files(), Path, watch()

### Community 34 - "Twist Mux Priority Topics"
Cohesion: 0.33
Nodes (6): auto input topic cmd_vel_auto (priority 10, lowest, Nav2/patrol), estop input topic cmd_vel_estop (priority 100, timeout 0.3s), estop_node (external, decides when to stop / slow; not defined in this file), estop_slow input topic cmd_vel_estop_slow (priority 18, deceleration ring 1.5-5m), marker_slow input topic cmd_vel_marker_slow (priority 15, corner marker deceleration), marker_vision (external, publishes marker_slow deceleration)

### Community 35 - "SignLSTM Model"
Cohesion: 0.33
Nodes (4): 랜드마크 시퀀스 (batch, T, 150) → 수신호 클래스 로짓 (batch, num_classes). 구조: LSTM(다층)으로 시퀀스를…, x: (batch, T, input_dim) 시퀀스 배치 → (batch, num_classes) 로짓., SignLSTM, Tensor

### Community 36 - "Copyright Style Test (auto_drive)"
Cohesion: 0.40
Nodes (4): copyright, linter, skip, test_copyright()

### Community 37 - "Copyright Style Test (knavi_bringup)"
Cohesion: 0.40
Nodes (4): copyright, linter, skip, test_copyright()

### Community 38 - "Copyright Style Test (robot_control)"
Cohesion: 0.40
Nodes (4): copyright, linter, skip, test_copyright()

### Community 39 - "Copyright Style Test (signal_vision)"
Cohesion: 0.40
Nodes (4): copyright, linter, skip, test_copyright()

### Community 40 - "Model URI Resolution"
Cohesion: 0.50
Nodes (3): model://Foo/meshes/bar.dae 를 models_root 아래 절대경로로 푼다. RViz 의 resource_retriever…, resolve_model_uri(), 메시 경로를 풀되, 못 풀면 경고만 남기고 None 을 돌려준다. 여기서 예외를 올리지 않는 이유: 소품 하나가 빠질 뿐인데 노드가 죽으면…

### Community 41 - "Flake8 Style Test (auto_drive)"
Cohesion: 0.50
Nodes (3): flake8, linter, test_flake8()

### Community 42 - "Flake8 Style Test (knavi_bringup)"
Cohesion: 0.50
Nodes (3): flake8, linter, test_flake8()

### Community 43 - "PEP257 Style Test (knavi_bringup)"
Cohesion: 0.50
Nodes (3): linter, pep257, test_pep257()

### Community 44 - "Flake8 Style Test (robot_control)"
Cohesion: 0.50
Nodes (3): flake8, linter, test_flake8()

### Community 45 - "Flake8 Style Test (signal_vision)"
Cohesion: 0.50
Nodes (3): flake8, linter, test_flake8()

### Community 46 - "PEP257 Style Test (signal_vision)"
Cohesion: 0.50
Nodes (3): linter, pep257, test_pep257()

## Knowledge Gaps
- **21 isolated node(s):** `/clock topic bridge (world-wide, GZ_TO_ROS)`, `robot1 bridge block (commented out, dead)`, `/robot2/cmd_vel bridge (ROS_TO_GZ, Twist)`, `/robot2/odom bridge (GZ_TO_ROS, Odometry)`, `/robot2/joint_states bridge (GZ_TO_ROS)` (+16 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **12 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `GestureCommandPublisher` connect `Gesture Command Publisher` to `Camera Node Main Loop`, `CameraNode Init Params`?**
  _High betweenness centrality (0.194) - this node is a cross-community bridge._
- **Why does `GestureInference` connect `Gesture Inference Wrapper` to `Gesture Command Publisher`, `Camera Node Main Loop`, `Inference UI HUD`, `Inference Base Class`, `Model Load Signal Stabilizer`, `Hand Feature Extractor`, `CameraNode Init Params`?**
  _High betweenness centrality (0.099) - this node is a cross-community bridge._
- **Why does `SignLSTM` connect `SignLSTM Model` to `Camera Capture Rendering`, `Vision Inference Runtime`, `Gesture Model Training`, `Model Load Signal Stabilizer`, `PEP257 Style Tests`?**
  _High betweenness centrality (0.088) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `CameraNode` (e.g. with `GestureCommandPublisher` and `FrameSource`) actually correct?**
  _`CameraNode` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `HandWarning` (e.g. with `ButtonBar` and `SignalStabilizer`) actually correct?**
  _`HandWarning` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `FeatureExtractor` (e.g. with `GestureInference` and `InferenceBase`) actually correct?**
  _`FeatureExtractor` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 7 inferred relationships involving `SignalStabilizer` (e.g. with `GestureInference` and `InferenceBase`) actually correct?**
  _`SignalStabilizer` has 7 INFERRED edges - model-reasoned connections that need verification._