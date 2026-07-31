# camera_node 리팩토링 — 변경 사항 정리

## 1. 이 문서가 다루는 것

`camera_node`를 클래스 단위로 나눈 리팩토링, 그 과정에서 고친 결함, 그리고 결정하지 않고 남겨둔 것들을 기록한다.

- **새 파일**: `src/robot_control/robot_control/camera_node_refactory.py`
- **원본 `camera_node.py`는 그대로 둔다.** 설치도 유지해서 두 방식을 나란히 돌려 비교할 수 있다.
- `setup.py`에 `console_scripts` 항목 하나(`camera_node_refactory`)를 추가했다. 패키지의 다른 부분은 바꾸지 않았다.
- `launch/bringup.launch.py`는 **의도적으로 아직 원본 `camera_node`를 띄운다.** 이유는 9장 참고.

---

## 2. 왜 나눴는가

한 클래스가 네 가지 책임을 들고 있었는데, 각각 **"바뀌는 이유"가 서로 아무 관계가 없다.**

| 책임 | 언제 바뀌나 |
|---|---|
| 웹캠 캡처 | 카메라나 운영체제가 바뀔 때 |
| 이미지 발행 | 토픽이나 QoS 정책이 바뀔 때 |
| 수신호 추론 | 모델이 바뀔 때 |
| 라벨 → 속도 매핑 | 로봇의 속도 제한이 바뀔 때 |

한 곳에 섞여 있으면 카메라 하나 바꾸려고 모델 코드까지 같이 읽어야 한다.

게다가 **이 중 둘은 기존 설계 문서상 이 리포지토리 소관이 아니다.**

- `doc/design.md` — 카메라 기반 이미지 프로세싱은 **별도 리포지토리**에서 개발하며, 처리된 인지 데이터는 여기서 **외부 입력**으로 취급해야 한다.
- `config/twist_mux.yaml` — 라벨을 `Twist`로 바꾸는 주체는 **별도의 `command_node`**다.

다만 지금 당장 노드를 나누지는 않았다. 노드를 나누면 매 프레임 이미지 직렬화 비용이 주 경로에 붙는데, 이게 지금 `publish_image=false`로 피하고 있는 바로 그 비용이다. **대신 경계를 클래스로 미리 그어 두었다.** 나중에 분리할 때 다시 짜지 않고 "옮기기만" 하면 된다.

---

## 3. 클래스 구성

```
FrameSource              1. 하드웨어 캡처 (cv2.VideoCapture)     ← rclpy 안 씀
ImagePublisher           2. 이미지 발행 (sensor_msgs/Image)
GestureInference         3. 추론 인터페이스   ← 본래 외부 리포 소관
  └ StubGestureInference    실제 모델 붙기 전까지 쓰는 자리표시자
GestureCommandPublisher  4. 라벨 → Twist      ← 본래 command_node 소관
────────────────────────────────────────────────────────────────────
is_shutting_down()          종료 판단 공용 함수            (배관)
LatestFrameQueue            깊이 1 큐, 최신 프레임 우선     (배관)
CameraNode(Node)            배선 + 타이머 + 워커 스레드     (조립자)
```

- **`CameraNode`는 더 이상 OpenCV도 모델도 `Twist`도 직접 만지지 않는다.** 부품을 배선하고 두 개의 실행 흐름만 소유한다.
- `FrameSource`와 `GestureInference`는 `rclpy`를 import하지 않는다. → **ROS 없이도 돌려볼 수 있다.**
- `FrameSource`는 로그도 직접 남기지 않는다. 상태를 프로퍼티로 노출만 하고 무엇을 경고할지는 호출자가 정한다. 로거를 주입받으면 다시 ROS에 묶이기 때문이다.
- `LABELS`와 `LABEL_MOTION`은 모듈 최상단으로 올렸다. **3번이 라벨을 만들고 4번이 소비하므로** 어느 한쪽이 정의를 소유할 수 없다.

### 스레드 구조는 그대로 뒀다

이미 올바르고, 실전에서 어렵게 얻어낸 부분이기 때문이다.

- 실행기 스레드가 캡처해서 큐에 넣고, **워커 스레드 정확히 1개**가 추론하고 발행한다.
- **워커는 반드시 1개여야 한다.** 2개 이상이면 추론 시간 편차 때문에 완료 순서가 뒤바뀐다. 라벨은 이벤트가 아니라 **상태**라서, 낡은 라벨이 최신 라벨을 덮으면 `STOP` 다음에 `FORWARD`가 나가는 사고가 된다.
- **이중 종료 방어**를 유지했다: 발행 전에 컨텍스트가 살아 있는지 확인하고, 확인과 발행 사이에 종료가 끼어들 경우를 대비해 발행을 `RuntimeError`로 한 번 더 감싼다.

---

## 4. 함께 고친 결함 8건

### ① 큐 깊이가 주석은 1, 코드는 5

주석은 파일 상단과 선언부 두 곳 모두 "깊이 1"이라고 적혀 있는데 실제 코드는 `maxsize=5`였다.

깊이 5면 워커가 **최대 5프레임 지난 화면**으로 판단한다. 스텁 추론 50ms 기준 약 **250ms 뒤처진다.** `STOP`이 늦는 것은 안전 문제다. → `LatestFrameQueue`가 깊이 1을 강제하게 했다.

### ② 매 주기마다 numpy 프레임 전체를 로그로 출력

```python
self.get_logger().info(f'frame Info: {frame}', throttle_duration_sec=1.0)
```

`throttle_duration_sec`은 **출력만** 막을 뿐 f-string 평가는 막지 못한다. 초당 20회 0.9MB 배열의 `repr()`을 만들고 버리고 있었다. → 제거.

### ③ `cv2.CAP_PROP_BUFFERSIZE` 미설정

이게 없으면 드라이버가 자체 대기열을 유지한다. 카메라가 30fps로 채우는데 20Hz로 읽으면 초당 10장이 드라이버 버퍼에 쌓이고, `read()`는 **가장 오래된 것부터** 돌려준다. 애플리케이션 큐를 1로 줄여도 이 지연은 안 없어진다. → `1`로 설정.

### ④ 정의되지 않은 라벨을 `fatal`로 기록

노드는 그 라벨을 무시하고 **계속 돈다.** 치명적 상황이 아니다. → `warn`으로 변경.

### ⑤ `LABELS`와 `LABEL_MOTION` 키를 손으로 이중 관리

주석으로 "키는 LABELS와 일치해야 한다"고 사람 손에 맡기고 있었다. → `LABELS = tuple(LABEL_MOTION)`로 파생시켜 **어긋날 수 없게** 했다.

### ⑥ 캡처 백엔드 `cv2.CAP_V4L2` 하드코딩

Linux가 아닌 개발 머신에서는 장치가 아예 안 열린다. → `capture_backend` 파라미터로 분리. 기본값은 여전히 `v4l2`.

### ⑦ 카메라가 요청 해상도를 거절해도 조용함

카메라가 요청 크기를 거절하는 일은 흔하다. 원본은 `actual_w`/`actual_h`를 읽어두고도 로그에 찍기만 했다. → 실제 크기가 요청과 다르면 **경고**한다. 나중에 "모델 입력 크기가 왜 다르지?"로 시간 버리지 않게.


### (부수) 파라미터 검증 정리

검증이 흩어져 있고 `fps` 하나만 보고 있었다. → `_declare_and_read_parameters` 한 곳에 모았고, **카메라를 열기 전에** 수행한다. 잘못된 값 때문에 반납해야 할 장치 핸들이 남는 일이 없다. `FrameSource`는 자기 몫인 해상도·백엔드 인자를 따로 검증한다.

---

## 5. 의도적으로 손대지 않은 것 — 팀 결정 필요

아래는 **정리가 아니라 동작을 바꾸는 일**이라, 리팩토링 범위 밖으로 뒀다.

### ⚠️ 라벨 유지(hold) 정책 — 실제 주행 문제

`twist_mux`의 `gesture` 입력은 **timeout이 0.5초**다. 그런데 `infer()`가 `None`을 반환하는 동안에는 아무것도 발행되지 않는다.

→ 인식 공백이 0.5초를 넘으면 `twist_mux`가 gesture 소스를 버리고 **속도 0으로 돌아간다.**
→ `FORWARD`로 주행 중인 로봇이 인식이 흔들릴 때마다 **덜컥거린다.**

마지막 라벨을 유지하면 해결되지만, **대칭적으로 유지하는 것은 아마 틀렸다.** 안전 관점에서는 `STOP`을 `FORWARD`보다 오래 유지해야 한다. 이건 안전 정책 결정이라 제 판단으로 정하지 않았다.

결정 지점은 소스의 `GestureCommandPublisher.publish` 독스트링에 박스 주석으로 표시해 뒀다.

### 런타임 파라미터 반영

파라미터는 여전히 생성자에서 한 번만 읽는다. `ros2 param set`은 저장된 값만 바꾸고 동작은 안 바꾼다.

지원하려면 `add_on_set_parameters_callback`에 더해 **타이머 재생성과 카메라 재설정**이 얽힌다. 구조 변경이 아니라 기능 추가라서 뺐다.

### 노드 분리

3번과 4번은 클래스로는 분리했지만 여전히 같은 프로세스 안에 있다. 밖으로 빼면 매 프레임 이미지 직렬화 비용이 든다. **결정 전에 `metrics` 출력으로 실제 비용을 측정할 것.**

---

## 6. 실제 추론 모델 붙이기

`GestureInference`를 상속해서 `LABELS` 중 하나를 반환한다. 판단할 수 없는 프레임이면 `None`.

```python
class MyHandGesture(GestureInference):

    def load_model(self):
        import mediapipe as mp
        self._hands = mp.solutions.hands.Hands(max_num_hands=1)

    def infer(self, frame):
        # frame: numpy.ndarray (H, W, 3) uint8, BGR 순서
        ...
        return 'STOP'
```

그다음 **`CameraNode._build_inference`의 한 줄만 바꾸고**, `StubGestureInference`는 삭제한다.

```python
def _build_inference(self):
    return MyHandGesture(self.get_logger())   # ← 여기 한 곳뿐
```

### 지켜야 할 계약

- `load_model()`은 워커가 시작하기 **전에 1회** 실행된다. 무거운 초기화(가중치 로딩 등)는 전부 여기서 한다. `infer()` 안에서 매 프레임 하면 처리율이 무너진다.
- `load_model()`과 `infer()` 모두 **워커 스레드 하나에서만** 호출된다. 따라서 거기서 만든 객체는 스레드 안전할 필요가 없다(MediaPipe Hands처럼 스레드 안전하지 않은 객체도 그대로 써도 된다).
- `infer()`가 던진 예외는 호출부가 잡아서 로그로 남기므로 **워커는 죽지 않는다.** 다만 그 프레임의 라벨은 안 나간다.
- `LABELS`에 없는 문자열을 반환하면 아무것도 발행되지 않고 경고만 남는다.
- 프레임은 **BGR**이다(OpenCV 기본). 모델이 RGB를 기대하면 `cv2.cvtColor`로 직접 변환할 것.

---

## 7. 파라미터

**새로 생긴 것은 하나뿐**이고, 나머지는 이름·기본값·의미 모두 원본과 같다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| **`capture_backend`** | **`v4l2`** | **신규.** `v4l2` / `any` / `avfoundation` / `dshow` |
| `device_id` | `0` | `/dev/video<N>`의 N |
| `frame_width` / `frame_height` | `640` / `480` | 요청 해상도 (카메라가 거절할 수 있음) |
| `fps` | `20.0` | 캡처·발행 주기 |
| `frame_id` | `webcam` | Image 헤더의 frame_id |
| `enable_inference` | `true` | 끄면 순수 카메라 노드 |
| `publish_image` | `false` | 이미지 토픽 발행 |
| `stats_period` | `1.0` | 통계 출력 주기(초). 0 이하면 끔 |
| `linear_speed` | `0.2` | `FORWARD` 전진 속도 (m/s) |
| `angular_speed` | `0.5` | `LEFT`/`RIGHT` 회전 속도 (rad/s) |

Linux가 아닌 곳에서 개발한다면 `capture_backend`를 `any`나 해당 플랫폼 값으로 바꾼다.

### 기본값 두 개는 자주 오해받으니 다시 적어둔다

**`publish_image`가 `false`인 이유** — 추론은 프로세스 안의 큐로 넘기므로 평상시 이미지 토픽을 **구독하는 노드가 없다.** 구독자가 0명이어도 변환 비용(640×480×3 = 0.9MB, 20fps면 초당 18MB 복사)은 그대로 나간다. `rosbag2` 녹화나 `rviz2` 확인이 필요할 때만 켠다.

**`fps`가 `20`인 이유** — 워커 1개에서 50ms 추론의 처리량 한계가 **초당 20장**이기 때문이다. 추론 처리율보다 높이면 유실 수만 늘어난다.

```
추론 50ms → 처리 한계 20장/초
fps 30 → 캡처 30, 처리 20 → 초당 10장 유실 (33%)
fps 20 → 캡처 20, 처리 20 → 유실 0
```

---

## 8. 실행 방법

```bash
colcon build --packages-select robot_control
source install/setup.bash

ros2 run robot_control camera_node_refactory
ros2 run robot_control camera_node_refactory --ros-args -p capture_backend:=any
ros2 run robot_control camera_node_refactory --ros-args -p enable_inference:=false
```

### 주의 2가지

**노드 이름은 여전히 `camera_node`다.** 파라미터 파일과 리맵을 그대로 쓸 수 있는 드롭인 교체를 노린 것이다. 같은 이유로 **두 변형을 동시에 띄우면 이름이 충돌한다.** 한 번에 하나만 실행할 것.

**단독 실행으로는 로봇이 안 움직인다.** `ros2 run`으로 띄우면 `/cmd_vel_gesture`로 발행되는데, `twist_mux`는 `/robot2/cmd_vel_gesture`를 구독한다. bringup 이 이 노드를 로봇 네임스페이스 안에서 띄워 주는 구조라, 단독 실행하면 그 네임스페이스가 안 붙는다.

### 통계 읽는 법

```
[stats] 캡처=20 추론=19 유실=1 | 추론 median=52.3ms max=71.0ms
```

`stats_period`가 `1.0`이면 위 숫자가 **그대로 Hz**로 읽힌다. 실제 모델을 붙인 뒤에는 이 출력을 보고 `fps`를 다시 맞춘다.

---

## 9. 검증 상태 ⚠️

**이 파일을 신뢰하기 전에 이 절을 먼저 읽을 것.**

### 실제로 확인한 것

ROS도 OpenCV도 없는 Windows 머신에서, `cv2`·`rclpy`·`cv_bridge`·메시지 모듈을 **가짜로 대체해 노드를 실행**하여 확인했다.

- 큐 깊이가 1이고, `CAP_PROP_BUFFERSIZE`가 1로 설정되며, 타이머 2개가 0.05초/1.0초 주기로 생성됨
- 캐시된 `Twist` 4개가 기본 속도에서 의도한 매핑과 일치 (`STOP`/`FORWARD`/`LEFT`/`RIGHT`)
- 프레임 유실이 집계되고, 큐가 **가장 오래된 것이 아니라 최신 프레임**을 유지함
- 정의되지 않은 라벨은 경고만 남기고 발행되지 않으며, 어디서도 `fatal`이 나오지 않음
- `publish_image`가 기본 꺼짐이고 아무것도 발행하지 않음. 유효한 라벨은 `gesture` → `cmd_vel_gesture` 순으로 발행됨
- 종료 시 워커를 join하고 카메라를 반납함. 반납된 소스는 `is_open`이 `False`, `read()`가 `None`
- 잘못된 해상도 / 알 수 없는 백엔드 / 열리지 않는 장치 각각이 의도한 `ValueError` 또는 `RuntimeError`를 던짐
- 파일이 컴파일되고, 99자 초과 줄 0개, 줄 끝 공백 0개 (`test/`의 `ament_flake8` 기준)

### 확인하지 **못한** 것 — 리눅스 VM에서 반드시 볼 것

- `colcon build`, 그리고 `ament_flake8`·`ament_pep257` 테스트의 실제 실행
- 실제 `cv2.VideoCapture` 동작 — V4L2 백엔드가 요청 해상도·FPS·`CAP_PROP_BUFFERSIZE`를 정말 적용하는지
- 실제 DDS 발행, `rviz2`와의 QoS 매칭, Gazebo에서의 `twist_mux` 연동
- **진짜 Ctrl-C에서의 종료 동작.** 이중 종료 방어가 존재하는 이유가 바로 이 경로다

**이것이 `bringup.launch.py`를 아직 안 바꾼 이유다.** 위 확인이 통과하면 `executable='camera_node'`를 `executable='camera_node_refactory'`로 바꾼다.

---

## 10. 후속 작업

1. 9장의 VM 확인을 수행 → launch 파일 전환 → `camera_node.py` 삭제
2. 5장의 **라벨 유지 정책을 결정**하고 `GestureCommandPublisher` 안에 구현
3. 6장에 따라 실제 모델을 붙이고, 그 모델의 실제 추론 시간에 맞춰 `fps` 재측정
4. 소스 주석이 "핸드오프 §4/§6/§8/§10"을 참조하는데 **그 문서가 리포지토리에 없다.** `doc/work.md`와 `doc/ml-flow.md`는 둘 다 빈 파일이다. 문서를 추가하든가 참조를 제거할 것
5. `doc/AGENT.md`가 아직 "벤더링한 창고 월드 외에 패키지가 없다"고 적고 있다. `robot_control`보다 앞선 내용이므로 launch 전환이 끝나면 갱신할 것

'''
지금 같은 노드가 3벌 있습니다:

camera_node.py — 원본
camera_node_refactory.py — 클래스 분리, 단일 파일
camera_node_division/ — 파일까지 분리 ← 방금 만든 것
셋 다 노드 이름이 camera_node라 동시에 띄우면 충돌합니다. 비교가 끝나면 하나만 남기는 게 좋은데, 어느 걸 남길지는 말씀해주시면 나머지 삭제하고 bringup.launch.py와 문서까지 맞추겠습니다. 지금은 비교 중이실 것 같아 셋 다 뒀습니다.

'''