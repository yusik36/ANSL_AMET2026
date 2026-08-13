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
  physicar_bringup/    # 센서 전체를 한 번에 띄우는 launch 패키지
```

카메라는 별도 패키지 없이 apt로 설치되는 `ros-jazzy-usb-cam`을 그대로 사용한다.

## 개발 환경 세팅 (새로 합류하는 팀원)

1. Ubuntu 24.04 LTS + ROS2 Jazzy 설치 (Raspberry Pi 4/5 또는 x86 노트북 어디든 무방, WSL2도 가능)
2. `sudo apt install ros-jazzy-usb-cam ros-dev-tools python3-serial v4l-utils`
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
| `/scan` | `sensor_msgs/msg/LaserScan` | ~10Hz | RPLidar A1, 360도, 최대 12m |
| `/image_raw` | `sensor_msgs/msg/Image` | ~30Hz | ELP USB 카메라, 1280x720 |
| `/imu/data` | `sensor_msgs/msg/Imu` | ~162Hz | orientation(쿼터니언)+각속도+선형가속도 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | ~162Hz | 지자기 (실내에서는 신뢰도 낮음, 참고용) |

**출력 인터페이스(각자 작성할 노드가 최종적으로 내보내야 하는 것)는 아직 미정** — 모터 제어기 인터페이스
확인되는 대로 이 표에 추가할 것 (예: `/cmd_vel` geometry_msgs/Twist 등).

## 하드웨어 접근이 없는 팀원용 개발 방법

실물 센서 없이도 개발 가능:
- 위 토픽 인터페이스(메시지 타입)만 맞춰서 노드를 작성하고, `ros2 bag record`로 녹화된 실제 센서 데이터로 재생하며 테스트
- 또는 직접 더미 퍼블리셔로 가짜 `/scan`, `/image_raw` 데이터를 흘려보내며 로직만 먼저 검증

## 대회 규정 관련 주의사항

- 실전 예선/본선은 반드시 대여받는 공식 Physicar 차량으로만 진행 (자체 제작 차량으로 참가 불가, 규정 위반 시 실격)
- 이 저장소는 **비공개(private)** 로 유지할 것 — 대회 규정상 다른 팀과 코드/기록 공유 시 실격 사유
- Stateless 주행 요건: 자율주행 로직은 트랙 어느 지점에서 시작해도 동작해야 함
