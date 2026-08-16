# ANSL AMET2026

2026 AMET 자율주행 해커톤 팀 개발 저장소. 대회 실차(Physicar)는 대여 전까지 접근 불가하므로,
랩실 부품으로 구성한 연습용 플랫폼(Raspberry Pi 4 + RPLidar A1 + ELP USB 카메라 + HandsFree USB IMU)
위에서 ROS2 자율주행 스택을 미리 개발/검증한다.

대회 실차 스펙: Raspberry Pi 5, Ubuntu 24.04 LTS, ROS2 Jazzy, RPLidar C1, 9축 IMU, 100° 야간카메라.
이 저장소의 소프트웨어 스택(Ubuntu 24.04 + ROS2 Jazzy)은 실차와 동일하게 맞췄으나,
센서 하드웨어 자체는 완전히 동일하지 않다. 목표는 하드웨어를 정확히 복제하는 게 아니라
**대회 전까지 인지-판단-제어 전체 프레임워크를 완성/검증**하는 것이다.

## 워크스페이스 구조

이 저장소 자체가 ROS2 워크스페이스 루트다 (`colcon build`를 저장소 루트에서 실행).

```
src/
  sllidar_ros2/       # RPLidar 공식 드라이버 (Slamtec 원본, 벤더링됨)
  physicar_imu/        # HandsFree USB IMU용 자체 작성 드라이버
  physicar_driver/      # /speed + /steering 구독 -> Arduino ESC/서보 드라이버 (연습용 플랫폼)
  physicar_nav/          # 라이다 기반 반응형 장애물 회피 (obstacle/* 토픽 발행)
  physicar_vision/        # 카메라 기반 인지: 차선 추종 + 신호등 감지 + HSV 캘리브레이션 헬퍼
  physicar_judgment/       # 위 셋을 종합해 최종 /speed + /steering을 내는 판단(중재) 노드
  physicar_bringup/         # 센서 전체 + 전체 자율주행 스택을 한 번에 띄우는 launch 패키지
```

카메라는 별도 패키지 없이 apt로 설치되는 `ros-jazzy-usb-cam`을 그대로 사용한다.

## 개발 환경 세팅 (새로 합류하는 팀원)

1. Ubuntu 24.04 LTS + ROS2 Jazzy 설치 (Raspberry Pi 4/5 또는 x86 노트북 어디든 무방, WSL2도 가능)
2. `sudo apt install ros-jazzy-usb-cam ros-dev-tools python3-serial v4l-utils ros-jazzy-cv-bridge python3-opencv`
3. 이 저장소 클론 후 빌드:
   ```bash
   git clone https://github.com/yusik36/ANSL_AMET2026.git physicar_ws
   cd physicar_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```
4. `echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc` — 랩실 네트워크의 다른 프로젝트와 토픽이 섞이지 않도록 반드시 설정 (팀원 전원 동일하게 42로 맞출 것)

## 센서 전체 실행

실제 센서가 연결된 보드(현재는 팀 공용 RPi4, hostname `2026AMET`)에서:

```bash
ros2 launch physicar_bringup sensors_launch.py
```

**알려진 하드웨어 한계**: 라이다+카메라+IMU를 동시에 실행하면 USB 허브 전력 부족으로 장치가
끊길 수 있다(라이다 모터 기동 순간 전류 스파이크가 다른 USB 장치를 리셋시킴, dmesg로 확인됨).
자체 전원(self-powered) USB 허브 도입 전까지는 필요한 센서만 개별로 켜서 개발할 것.

## 토픽 인터페이스 (이 위에서 인지/판단/제어 코드를 개발하면 됨)

| 토픽 | 메시지 타입 | 발행 주기 | 설명 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | ~10Hz | RPLidar A1, 360도, 최대 12m (실차는 C1, 토픽명 동일) |
| `/image_raw` | `sensor_msgs/msg/Image` | ~30Hz | ELP USB 카메라, 1280x720 — **⚠️ 실차는 `/camera/image_raw`** (2026-08-16 공식 소스로 확인, 네임스페이스 다름). `physicar_vision`은 `image_topic` launch 인자로 이 차이를 흡수함 (`vision_launch.py` 참고) |
| `/imu/data` | `sensor_msgs/msg/Imu` | ~162Hz | orientation(쿼터니언)+각속도+선형가속도 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | ~162Hz | 지자기 (실내에서는 신뢰도 낮음, 참고용) |

**⚠️ 실차 IMU 인터페이스 주의사항 (대회 기술 담당자 확인 2026-08-13, 공식 소스코드로 재확인 2026-08-16)**
실차 센서 칩은 9축이지만, 실차의 `/imu` 토픽은 **6축(자이로+가속도)만 제공하고 50Hz**로 나온다.
지자기 3축은 별도 `/imu/mag` 토픽으로 온다. `physicar_driver_node.cpp`에서 직접 확인됨:
`msg.orientation_covariance[0] = -1.0`으로 명시적으로 세팅함 — 이건 ROS 표준 관례로 "orientation
추정값 없음/신뢰 불가"를 뜻한다. 이 연습용 IMU 드라이버는 편의상 `/imu/data.orientation`에 쿼터니언
(자체 센서 퓨전 결과)을 채워서 보내지만, **실차에는 이 필드가 없다(공식 확인됨, 추측 아님).**
→ **`orientation` 필드에 의존하는 로직을 짜지 말 것.** 각속도(`angular_velocity`)와 선형가속도
(`linear_acceleration`)만 신뢰하고, 자세 추정이 필요하면 직접 상보/칼만 필터를 만들어 쓸 것.
주기 차이(162Hz vs 실차 50Hz)는 문제 없음(더 빠른 건 상관없음).

## 출력 인터페이스 (실차 확정 스펙, 대회 기술 담당자 확인, 2026-08-13)

**팀 코드(인지/판단)가 최종적으로 내보내야 하는 토픽은 이 두 개다.** `/cmd_vel`(Twist)이 아니라
애커만 조향 방식이라 속도/조향각을 따로 낸다.

| 토픽 | 메시지 타입 | 단위 | 설명 |
|---|---|---|---|
| `/speed` | `std_msgs/msg/Float64` | m/s | 목표 속도 |
| `/steering` | `std_msgs/msg/Float64` | rad | 목표 조향각 (Ackermann) |

이 두 토픽을 실차의 `physicar_driver_node`가 구독해서 UART로 확장보드(ESC+서보)에 내려보낸다.

**클램프/안전장치는 드라이버 노드(SDK 계층)에 이미 하드코딩되어 있어서 우리 코드가 직접 구현할 필요 없음
(2026-08-16, 공식 `physicar_driver_node.cpp`/`driver_params.yaml` 소스코드로 재확인 — 담당자에게
전해들은 값과 전부 일치):**
- 최대 속도 **3.0 m/s**, 최대 조향각 **±20°** — 초과값은 자동 클램프
- ESC 데드존(공식 파라미터 `min_speed`) **0.3 m/s** — 이보다 작은 speed 명령은 사실상 무반응
- 서보 각도 0–180° 클램프 + 채널별 리밋, ESC 듀티 범위 클램프, 배터리 전압 보상
- **명령 유효시간(`cmd_timeout`) 1초** — 그 안에 갱신 안 되면 speed는 자동 0으로(steering은 마지막 값 유지, 재중앙정렬 안 됨). 판단 노드는 최소 1Hz보다는 빠르게 계속 퍼블리시해야 함
- 휠베이스 **0.18m**, 트랙폭 0.16m, 휠 반지름 0.0375m — 공식 값
- 시뮬레이터도 동일 상수(3.0 m/s, ±20°)로 동작

**`/cmd_vel`(Twist) — 2026-08-16 공식 소스코드로 확인**: 실차의 `physicar_driver_node`도
`/cmd_vel`을 직접 구독하며, 시뮬레이터와 동일하게 `steering = atan(ω·L/v)` (휠베이스 L=0.18m,
공식 값 확인됨)로 변환해 같은 클램프를 거쳐 처리한다. 즉 `/cmd_vel`을 써도 실차에서 문제없이
동작한다 — 다만 이 저장소는 처음부터 `/speed`+`/steering` 직접 퍼블리시로 설계했고 그게 더
명시적이라 그대로 유지한다.

**우리 연습용 플랫폼(RC카+Arduino+ESC)에도 이 인터페이스(`/speed`, `/steering` 구독,
동일 클램프 적용)를 그대로 구현한 드라이버 노드가 있다** — `physicar_driver/driver_node.py`,
Arduino 펌웨어(`physicar_driver_fw.ino`)와 시리얼로 통신. 실차의 `physicar_driver_node`와
토픽 계약은 동일하므로, 인지/판단 쪽 노드(아래 참조)는 어느 플랫폼에서 실행하든 코드 변경 없이
그대로 동작한다.

## 인지-판단 파이프라인

전체 자율주행 스택은 한 번에 이렇게 띄운다:

```bash
ros2 launch physicar_bringup autonomy_launch.py
```

내부적으로 센서 → `physicar_driver` → `physicar_nav`(장애물 회피) → `physicar_vision`(차선 추종
+ 신호등) → `physicar_judgment`(최종 중재, `/speed`+`/steering` 발행) 순서로 뜬다. 카메라 없이
라이다만으로 벤치 테스트하려면 `ros2 launch physicar_nav avoid_test_launch.py`(차선/신호등 게이트
비활성화 상태로 `physicar_judgment`까지 포함해서 뜸).

### 왜 avoid_node가 더 이상 /speed, /steering을 직접 발행하지 않는가

초기 버전(`physicar_nav`만 있던 시점)의 `avoid_node`는 `/speed`+`/steering`을 직접 발행해서 사실상
유일한 컨트롤러였다. 차선 추종/신호등 인식을 추가하면서 이 방식은 깨진다 — 세 인지 결과를 하나의
최종 명령으로 합칠 중재자가 필요하기 때문에, `avoid_node`는 이제 `obstacle/speed_cap`,
`obstacle/steer_override`, `obstacle/override_active` 세 토픽만 내고, 최종 `/speed`+`/steering`은
`physicar_judgment/judgment_node`가 낸다. 우선순위는 **장애물 회피/정지 안전 > 신호등 게이트 > 차선
추종** 순 — 자세한 근거는 `judgment_node.py` 모듈 docstring 참고.

### 차선 추종 — 트랙 색상을 모르는 채로 어떻게 개발했나

8/18에 공개되는 코스 규격은 예선용이고, 본선 트랙은 당일 랜덤 공개다(위 "대회 규정" 참고). 즉 지금
시점에 트랙 경계 마킹의 실제 색상을 알 수 없고, 8/18 이후에도 본선 트랙은 또 다를 수 있다. 그래서
`physicar_vision/lane_follow_node.py`는 색상을 코드에 박아넣지 않고 HSV 임계값을 전부 ROS2
파라미터로 뒀다(기본값은 "어두운 바닥 위 밝은 선"을 가정한 placeholder일 뿐, 실제 마킹 색과 다를 수
있음). 현장에서 카메라로 트랙 표면을 비추고 `hsv_calibrate_node`를 돌리면 터미널에 추천 HSV 범위가
찍히므로, 그 값으로 파라미터만 바꿔 끼우면 된다(코드/로직 변경 불필요):

```bash
ros2 run physicar_vision hsv_calibrate_node
# 출력된 h_min/h_max/s_max/v_min 값을 lane_follow_node 파라미터로 전달
ros2 run physicar_vision lane_follow_node --ros-args -p h_min:=<값> -p s_max:=<값> -p v_min:=<값>
```

핵심 알고리즘(ROI 내 최대 컨투어의 중심 오프셋 → 조향각)은 임계값이 다소 부정확해도 어느 정도
견디도록 설계했지만, 임계값 자체는 반드시 실제 트랙 표면에서 재보정해야 한다.

### 신호등 게이트 — "감지 안 됨"을 언제 정지로 볼 것인가

이건 담당자에게 확인한 게 아니라 이 파이프라인을 만들며 내린 설계 판단이다: 대회 규정상 페널티는
"빨간불 출발"에만 붙고, 트랙 대부분 구간에는 애초에 신호등이 안 보인다. 그래서
`traffic_light_node`는 **"카메라가 죽었다"(`traffic/valid=False`)**와 **"카메라는 살아있는데 이번
프레임엔 신호등이 안 보인다"(`traffic/light_state="NONE"`)**를 구분해서 발행하고,
`judgment_node`는 전자만 정지로 취급한다 — 후자(NONE)는 "가도 됨"으로 처리한다(그렇지 않으면
신호등이 안 보이는 직선 구간에서도 계속 멈춰 있게 됨). 실제로 빨간불이 잡혔을 때만
(`light_state="RED"` and `traffic/valid=True`) 속도를 0으로 만든다.

### 캘리브레이션 체크리스트 (8/25 실차 인수 시)

| 항목 | 위치 | 비고 |
|---|---|---|
| 카메라 토픽 네임스페이스 | `real_autonomy_launch.py`의 `image_topic` 인자 | ✅ 해결됨(2026-08-16) — `/camera/image_raw`로 이미 remap되어 있음, 추가 조치 불필요 |
| 라이다 장착각 보정 | `physicar_nav` 파라미터 `front_offset_deg` | 연습 섀시 전용 실측값(180°)이 들어있음 — 실차는 다를 수 있으므로 재측정 필수. avoid_node.py 모듈 docstring 참고 |
| 회피 조향 부호 | `physicar_nav` 파라미터 `avoid_steer_sign` | 연습 섀시 전용 실측값(-1.0) — 실차에서 반대로 돌면 다시 플립 |
| 차선 HSV 임계값 | `physicar_vision` 파라미터 `h_min/h_max/s_max/v_min` | `hsv_calibrate_node`로 실측, 8/18 코스 공개 후 1차 보정, 8/25 실차 인수 후 재보정 |
| 차선 조향 부호 | `physicar_vision` 파라미터 `lane_steer_sign` | 반대로 돌면 -1.0으로 플립 |
| 신호등 HSV 임계값 | `physicar_vision` 파라미터 `sat_min/val_min` | 실제 조명/노출 환경에서 재확인 |
| 카메라 화각/왜곡 | (미보정) | 실차 카메라 공식 스펙(2026-08-16 확인): OV5647, IR-cut 없음, 640x480 캡처→480x360 출력, ~15fps, FOV 약 98°(undistort.dist_scale=0.7 기준). 연습 카메라(ELP)와 여전히 다르므로 ROI 비율(`roi_top_frac` 등) 재조정 필요할 수 있음 |
| IMU orientation 필드 | (해당 없음, 의도적으로 미사용) | 실차 `/imu`는 6축뿐, `orientation_covariance[0]=-1`로 공식 확인됨 — 이 파이프라인은 애초에 orientation을 안 씀 |

### 실차 배포 — myapp.sh (2026-08-16 공식 소스코드로 확인)

**팀 코드는 `/opt/physicar/userdata/myapp.sh`에 넣는다.** 실차 웹 UI(`:5000`, "MyApp" 패널)에서
bash 스크립트를 업로드하면 `physicar-myapp.service`(systemd, 실패 시 자동 재시작)가 그 스크립트를
실행한다. 핵심 SW(`physicar_driver_node`, 카메라/라이다 드라이버 등)는 절대 안 건드리고 이 슬롯에만
배포하면 되므로 "SW 임의 변경 금지" 규정과 무관하다 — 대회 설계 자체가 이 슬롯을 통한 팀 로직
경쟁을 의도하고 있다.

myapp.sh에 넣을 내용 (예상, 실차 인수 후 확정):
```bash
#!/bin/bash
source /opt/physicar/install/setup.bash   # 실차 워크스페이스 경로는 인수 시 확인
source ~/physicar_ws/install/setup.bash   # 우리 팀 패키지
ros2 launch physicar_bringup real_autonomy_launch.py
```

**주의**: 실차는 센서(카메라/라이다/IMU)와 `physicar_driver_node`가 이미 시스템 서비스로 떠있는
상태다. `autonomy_launch.py`(연습용, sensors_launch.py+우리 driver_node까지 같이 띄움)를 그대로
쓰면 안 되고, **`real_autonomy_launch.py`**(인지/판단 노드만 띄움, 카메라 토픽 remap 포함)를 써야
한다.

## 하드웨어 접근이 없는 팀원용 개발 방법

실물 센서 없이도 개발 가능:
- 위 토픽 인터페이스(메시지 타입)만 맞춰서 노드를 작성하고, `ros2 bag record`로 녹화된 실제 센서 데이터로 재생하며 테스트
- 또는 직접 더미 퍼블리셔로 가짜 `/scan`, `/image_raw` 데이터를 흘려보내며 로직만 먼저 검증

## 대회 규정 관련 주의사항

- 실전 예선/본선은 반드시 대여받는 공식 Physicar 차량으로만 진행 (자체 제작 차량으로 참가 불가, 규정 위반 시 실격)
- **저장소 공개 상태 (2026-08-15)**: 이 저장소는 원래 "대회 규정상 다른 팀과 코드/기록 공유 시 실격
  사유"라는 이유로 비공개(private)로 유지하기로 했었으나, 2026-08-15에 팀 판단으로 public으로
  전환했다. 이 전환이 위 실격 사유와 충돌할 수 있다는 점은 전환 전에 명시적으로 인지한 상태였다 —
  필요하면 대회 측(AI CASTLE 이동재 기술팀장)에 공개 저장소 운영이 규정상 문제없는지 재확인할 것.
- Stateless 주행 요건: 자율주행 로직은 트랙 어느 지점에서 시작해도 동작해야 함
