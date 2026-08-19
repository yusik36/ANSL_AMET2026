# ANSL AMET2026

2026 AMET 자율주행 해커톤 팀 개발 저장소 (충남대 무인이동체 항법연구실).

대회 실차 스펙: Raspberry Pi 5, Ubuntu 24.04 LTS, ROS2 Jazzy, RPLidar C1, 9축 IMU, 100° 야간카메라.
이 저장소의 SW 스택은 실차와 동일하게 맞췄다. 실차 대여 전까지는 랩실 부품으로 구성한 연습용
플랫폼(RPi4 + RPLidar A1 + ELP USB 카메라 + HandsFree USB IMU)과 대회 공식 웹 시뮬레이터
위에서 개발/검증한다.

> **주행 로직 전체 설명은 별도 문서에 있다.**
> 이미지 한 장이 바퀴까지 가는 7단계와 각 단계의 설계 근거:
> <https://claude.ai/code/artifact/46e8273c-71b2-4064-aa21-a3422120d365>

---

## 한눈에

```
/camera/image_raw ──► planner_node ──► plan/speed      ──► judgment_node ──► /speed
                          │            plan/steering                         /steering
                          │            plan/valid            ▲
                          │                                  │
                      corridor.py                  traffic_light_node
                    (ROS 없는 순수 로직)            traffic/light_state
                                                    traffic/valid
```

**카메라 이미지에서 "차가 지나갈 수 있는 통로"를 직접 찾고, 그 통로 한가운데를 향해 몬다.**
차선을 따라가는 것도, 장애물을 피하는 것도 아니다 — 장애물은 그냥 통로가 좁아진 지점이라
회피 모드로 들어가고 나오는 코드 자체가 없다.

## 워크스페이스 구조

이 저장소 자체가 ROS2 워크스페이스 루트다 (`colcon build`를 저장소 루트에서 실행).

```
src/
  physicar_planner/    ★ 주행. 카메라 → 통로 → 속도/조향 (corridor.py + planner_node.py)
  physicar_judgment/   ★ 최종 중재. 신호등 게이트 + 페일세이프 → /speed, /steering
  physicar_vision/     ★ traffic_light_node (신호등) + hsv_calibrate_node
                         lane_follow_node 는 대체됨 — 아래 "대체된 코드" 참고
  physicar_bringup/    ★ launch 모음 (real_autonomy_launch.py 가 대회용)
  physicar_nav/          avoid_node — 대체됨, 연습 섀시 launch에서만 사용
  physicar_driver/       연습용 플랫폼 전용 Arduino ESC/서보 드라이버 (실차는 공식 노드 사용)
  physicar_imu/          연습용 플랫폼 전용 HandsFree USB IMU 드라이버 (실차는 공식 노드 사용)
  sllidar_ros2/          RPLidar 공식 드라이버 (Slamtec 원본, 벤더링됨)
tools/                   YS_* 계측/검증 도구 — 아래 "도구" 참고
```

★ = 대회 실차에서 실제로 뜨는 것. 나머지는 연습 플랫폼 전용이거나 대체된 코드다.

카메라는 별도 패키지 없이 apt로 설치되는 `ros-jazzy-usb-cam`을 그대로 사용한다(연습 플랫폼).

---

## 실행

### 대회 실차 / 시뮬레이터

```bash
source install/setup.bash
ros2 launch physicar_bringup real_autonomy_launch.py
```

센서 드라이버는 **띄우지 않는다.** 실차는 카메라·라이다·IMU 드라이버와 공식
`physicar_driver_node`를 시스템 서비스로 이미 돌리고 있어서, 여기서 다시 띄우면 충돌하거나
중복된다. 이 launch가 올리는 건 `planner_node` + `judgment_node` + `traffic_light_node` 셋뿐이다.

파라미터 오버라이드는 **launch 시점에** 넘겨야 한다. 노드는 생성 시 파라미터를 한 번 읽고
캐시하므로 실행 중인 노드에 `ros2 param set`을 해도 조용히 무시된다.

```bash
ros2 launch physicar_bringup real_autonomy_launch.py \
    road_h_min:=95 road_h_max:=125 aggression:=3.0 debug:=true
```

### 연습용 플랫폼 (RPi4 + Arduino 섀시)

```bash
ros2 launch physicar_bringup sensors_launch.py     # 센서만
ros2 launch physicar_bringup autonomy_launch.py    # 센서 + 드라이버 + 구 파이프라인
```

`autonomy_launch.py`는 연습 섀시 전용이다 — 라이다가 뒤로 달려 있고 서보가 반대로 배선돼
있어서 `front_offset_deg:=180.0`, `avoid_steer_sign:=-1.0`을 오버라이드해서 띄운다.
**실차에 이 launch를 쓰면 안 된다.**

**알려진 하드웨어 한계 (연습 플랫폼)**: 라이다+카메라+IMU를 동시에 실행하면 USB 허브 전력
부족으로 장치가 끊길 수 있다(라이다 모터 기동 전류 스파이크가 다른 USB 장치를 리셋, dmesg 확인).
self-powered USB 허브 도입 전까지는 필요한 센서만 개별로 켤 것.

---

## 도구 (`tools/`, 전부 `YS_` 접두사)

팀 공유 환경에서 우리 파일을 구분하기 위한 접두사 규칙이다. **다른 팀원 파일은 건드리지 않는다.**

| 도구 | 답하는 질문 | ROS 필요 |
|---|---|---|
| `YS_calibrate.py` | 도로가 주변과 **어느 채널**에서 갈리나, 경계값은 얼마인가 | ○ |
| `YS_trace.py` | 명령한 속도와 실제 속도가 같은가 — 판단이 틀렸나 명령이 안 갔나 | ○ |
| `YS_run.sh` | 한 바퀴 벤치마크 — 리셋·구동·채점·정리를 한 명령으로 | ○ |
| `YS_bench.py` | 랩타임/코스이탈 자동 채점 (시뮬레이터 HTTP API, 읽기 전용) | ✕ |
| `YS_perception_probe.py` | IPM 거리 추정이 실제와 맞는가 | ○ |
| `YS_steer_check.py` | 조향 부호와 라이다 장착각이 실제로 어느 쪽인가 | ○ |
| `YS_raceline.py` | 최소곡률 레이싱라인 (평가용 — 주행에는 직접 못 씀, 아래 참고) | ✕ |
| `YS_corridor_test.py` | 플래너 로직 30개 검증 — **시뮬레이터 없이** | ✕ |
| `YS_bench_selftest.py` | 채점기 자체 검증 | ✕ |
| `YS_trafficlight_test.py` | 신호등 판정 검증 | ✕ |

대회장에서 가장 먼저 돌릴 것:

```bash
python3 tools/YS_calibrate.py --explain      # 차를 트랙 위에 세워두고
```

차가 **서 있는 바닥**(색이 아니라 기하로 정의된 도로)에서 경계값을 뽑아 launch 인자를
그대로 출력한다. H·S·V 세 채널을 다 채점해서 어느 채널이 실제로 가르는지도 이름을 대준다.

---

## 인터페이스 (실차 확정 스펙)

### 입력

| 토픽 | 타입 | 주기 | 비고 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | ~15Hz | **실차/시뮬레이터**. 연습 플랫폼은 `/image_raw` — `image_topic` launch 인자로 흡수 |
| `/scan` | `sensor_msgs/LaserScan` | ~10Hz | 실차 C1 / 연습 A1, 토픽명 동일 |
| `/imu/data` | `sensor_msgs/Imu` | 50Hz(실차) | **orientation 없음** — 아래 주의 |
| `/imu/mag` | `sensor_msgs/MagneticField` | | 실내 신뢰도 낮음 |

**⚠️ IMU orientation 필드를 쓰지 말 것** (2026-08-13 담당자 확인, 08-16 공식 소스 재확인).
실차 센서는 9축이지만 `/imu` 토픽은 6축(자이로+가속도)만 준다. `physicar_driver_node.cpp`가
`msg.orientation_covariance[0] = -1.0`으로 명시 — ROS 표준 관례로 "orientation 없음/신뢰 불가".
연습용 IMU 드라이버는 편의상 쿼터니언을 채워 보내지만 **실차에는 그 필드가 없다.**
현재 파이프라인은 애초에 orientation을 안 쓴다.

### 출력 — 팀 코드가 내야 하는 건 이 둘

| 토픽 | 타입 | 단위 |
|---|---|---|
| `/speed` | `std_msgs/Float64` | m/s |
| `/steering` | `std_msgs/Float64` | rad (Ackermann, **양수 = 좌회전**) |

`/cmd_vel`(Twist)이 아니라 애커만이라 속도/조향각을 따로 낸다. 실차 드라이버도 `/cmd_vel`을
구독하긴 하지만(`steering = atan(ω·L/v)`로 변환), 이 저장소는 `/speed`+`/steering` 직접
퍼블리시로 설계했고 그게 더 명시적이라 유지한다.

**클램프/안전장치는 드라이버 노드에 이미 하드코딩돼 있어 우리가 구현할 필요 없음**
(2026-08-16 공식 `physicar_driver_node.cpp` / `driver_params.yaml` 소스 확인):

- 최대 속도 **3.0 m/s**, 최대 조향각 **±20°** (기계적 한계 24°, 드라이버에서 20°로 제한)
- ESC 데드존 `min_speed` **0.3 m/s** — 이보다 작은 명령은 사실상 무반응
- 명령 유효시간 `cmd_timeout` **1초** — 갱신 안 되면 speed 자동 0 (steering은 마지막 값 유지)
  → 판단 노드는 1Hz보다 빠르게 계속 퍼블리시해야 한다 (현재 20Hz)
- 휠베이스 **0.18 m**, 트랙폭 0.16 m, 휠 반지름 0.0375 m
- 서보 각속도 통상 스펙 600°/s — 시정수보다 각속도 제한으로 모델링 권장
- 시뮬레이터도 동일 상수

---

## 대회장 캘리브레이션 체크리스트

시뮬레이터에서 측정한 색상값 중 대회장까지 살아남는 건 거의 없다. 바닥은 잔디가 아니라
전시장 바닥이고, 보라색 경계벽은 존재하지 않고, 조명은 그 홀의 조명이다.
**살아남는 건 규칙의 모양이지 숫자가 아니다.**

| 항목 | 파라미터 | 비고 |
|---|---|---|
| 도로 색상 | `road_h_min` / `road_h_max` | **반드시 재측정.** `YS_calibrate.py` |
| 차선 페인트 | `paint_s_max` / `paint_v_min` | **반드시.** 조명 바뀌면 V 임계가 흔들림 |
| 중앙 마킹 | `mark_h_min/max`, `mark_s_min` | **반드시.** 없거나 다른 색일 수 있음 |
| 최대 통로폭 | `max_span` | 트랙 폭에 비례 (현재 1.2 m ← 실측 폭 0.70~0.87 m) |
| 그립 | `aggression` | 현재 4.0 m/s². 낮게 시작해서 올린다 |
| 가감속 | `max_accel` / `max_decel` | 현재 2.0 / 5.0 m/s². 실차가 못 내면 낮춘다 |
| 신뢰 거리 | `max_range` | 현재 2.5 m. 카메라 마운트 같으면 유지 |
| Lookahead | `lookahead_base` | 현재 0.70 m. 차 기하에서 나옴 — 유지 |
| 조향 부호 | — | `YS_steer_check.py`로 실측. 실차는 양수=좌회전 (확인됨) |
| 카메라 화각 | — | 실차 OV5647, IR-cut 없음, 640×480 캡처 → 480×360 출력, ~15fps, FOV ~98° |

**가장 큰 미지수는 숫자가 아니다.** 시뮬레이터에서는 색상(H)이 도로와 주변을 갈랐다 —
잔디는 초록이고 도로는 아니니까. 전시장에서 회색 트랙이 회색 바닥 위에 있으면 H로는 아무것도
안 갈린다. 그건 임계값 문제가 아니라 **규칙 자체를 바꿔야 하는 문제**이고,
`YS_calibrate.py`가 그 경우를 명시적으로 경고한다.

---

## 실차 배포

**팀 코드는 `/opt/physicar/userdata/myapp.sh`에 넣는다** (2026-08-16 공식 소스 확인).
실차 웹 UI(`:5000`, "MyApp" 패널)에서 bash 스크립트를 업로드하면
`physicar-myapp.service`(systemd, 실패 시 자동 재시작)가 실행한다. 핵심 SW는 안 건드리고
이 슬롯에만 배포하므로 "SW 임의 변경 금지" 규정과 무관하다 — 대회 설계 자체가 이 슬롯을
통한 팀 로직 경쟁을 의도한다.

```bash
#!/bin/bash
source /opt/physicar/install/setup.bash   # 실차 워크스페이스 경로는 인수 시 확인
source ~/physicar_ws/install/setup.bash   # 우리 팀 패키지
ros2 launch physicar_bringup real_autonomy_launch.py
```

시뮬레이터의 Evaluation 하네스는 별도로 `source ~/physicar_ws/run.sh`를 진입점으로 쓴다.
**아직 우리 launch가 여기 연결돼 있지 않다** (아래 미해결 참고).

---

## 대체된 코드

지우지 않고 남겨뒀다. 되살리려면 `real_autonomy_launch.py`를 고쳐야 하고, 그냥 실행만
해서는 대회 경로에 들어가지 않는다.

| 파일 | 대체된 이유 |
|---|---|
| `physicar_vision/lane_follow_node.py` | 차선을 따라가는 대신 통로를 직접 찾는다 |
| `physicar_nav/avoid_node.py` | 장애물이 통로의 좁아짐이라 별도 회피 계층이 필요없다 |

**왜 분리 구조를 버렸나.** 차선 노드가 평소 운전하고 회피 노드가 끼어드는 구조에서는
제어권이 코너 한복판에서 넘어간다. 회피 노드가 차를 어디에 놔두고 나갈지 모르는데 차선
노드는 거기서부터 다시 차선을 찾아야 한다. 넘겨주는 순간과 돌려받는 순간이 각각 버그
자리이고, 둘 다 가장 위험한 타이밍이다.

`lane_follow_node`는 통로 규칙이 대회장에서 도로와 바닥을 못 가를 경우의 유일한 대안이라
남겨뒀다.

---

## 전역 위치추정을 포기한 근거

휠 엔코더가 없고, 위치는 레이저 스캔 매칭으로만 나온다. 대회장 펜스는 20 cm인데 라이다
평면은 17 cm이고, **연습장에는 펜스가 아예 없다** (대회 측 답변). 그래서 최적 레이싱라인을
계산해둬도 차가 트랙 위 자기 위치를 모른다.

→ `YS_raceline.py`의 결과는 **주행에 직접 못 쓰고 시뮬레이터 평가에만 쓴다.**
지금 보이는 것만으로 운전하는 통로 기반 설계가 여기서 나왔다.

참고로 대회 트랙(`71e69ee9…`)은 30.50 m, 폭 0.70~0.87 m, 최적라인 최소반경 0.581 m
(조향 17.2°). **중심선은 최소반경 0.223 m로 차가 못 돈다** — 코너 커팅은 최적화가 아니라
완주의 전제조건이다.

---

## 개발 환경 세팅

1. Ubuntu 24.04 LTS + ROS2 Jazzy (RPi 4/5 또는 x86 노트북, WSL2 가능)
2. ```bash
   sudo apt install ros-jazzy-usb-cam ros-dev-tools python3-serial v4l-utils \
                    ros-jazzy-cv-bridge python3-opencv
   ```
3. ```bash
   git clone https://github.com/yusik36/ANSL_AMET2026.git physicar_ws
   cd physicar_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```
4. `echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc` — 랩실 네트워크의 다른 프로젝트와 토픽이
   섞이지 않도록 팀원 전원 42로 통일

### 하드웨어 없이 개발하기

플래너 로직은 ROS 없이 검증된다. `corridor.py`에 rclpy가 하나도 안 들어가 있는 이유가 이것:

```bash
python3 tools/YS_corridor_test.py      # 30개 체크, 시뮬레이터/차량 불필요
```

---

## 대회 규정 주의사항

- 실전 예선/본선은 반드시 대여받는 공식 Physicar 차량으로만 진행 (자체 제작 차량 참가 불가, 실격)
- **Stateless 요건**: 트랙 어느 지점에서 시작해도 동작해야 함. `judgment_node`는 차의 트랙 상
  위치에 대해 상태를 갖지 않는다 — 기억하는 건 마지막 조향값과 타임스탬프뿐이고 둘 다 자동
  만료된다
- **저장소 공개 상태 (2026-08-15)**: 원래 "다른 팀과 코드/기록 공유 시 실격 사유"를 이유로
  비공개였으나 팀 판단으로 public 전환. 이 전환이 규정과 충돌할 수 있다는 점은 인지한 상태 —
  필요하면 대회 측(AI CASTLE 이동재 기술팀장)에 재확인할 것
- 트랙은 8/18 공개분으로 고정, 본선에서는 **장애물 위치만** 바뀐다

---

## 미해결

1. **시뮬레이터 한 바퀴 완주** — 아직 없다. 발진 버그(가속 제한 없음 + 정지 시 lookahead 과소)를
   2026-08-19에 고쳤고 다음 주행이 판가름한다
2. **`run.sh` 연결** — 시뮬레이터 Evaluation 진입점에서 우리 launch가 아직 안 불린다
3. **라이다 비상정지** — 플래닝 루프 *바깥*에 둘 것. 카메라가 틀렸을 때의 마지막 방어선이라
   카메라를 믿는 코드와 같은 경로에 있으면 안 된다
4. **신호등 화각** — 출발 자세에서 신호등이 이미지 열 410~478(프레임 폭 480)에 걸쳐 거의
   화면 밖이다. 카메라 팬(±30°)이 후보인데 미검증
