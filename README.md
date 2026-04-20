# 🏥 자율주행 기반 스마트 병동 관리 시스템

> TurtleBot3 × 2 + Intel RealSense D435 + Arduino × 2 기반 자율 순찰 · 낙상 감지 · 음성 케어 로봇 시스템  
> Intel AI SW Academy 9기 2차 팀 프로젝트 (2026.04)

---

## 🔧 Tech Stack

![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white)
![Qt](https://img.shields.io/badge/Qt6-41CD52?style=flat-square&logo=qt&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO11n--pose-pretrained-orange?style=flat-square)
![Whisper](https://img.shields.io/badge/Whisper-tiny-lightgrey?style=flat-square)

---

## 💡 Motivation

감염병 확산 환경에서 의료진과 환자 간 불필요한 접촉을 최소화하는 동시에,  
거동이 불편한 입원 환자의 안전과 편의를 24시간 보장할 필요가 있다.

기존 병원 시스템의 문제점:
- 낙상 사고 발견 지연
- 야간 인력 부족
- 환자 호출에 대한 즉각적인 대응 어려움

👉 본 프로젝트는 자율주행 로봇이 병실을 순찰하며 낙상을 감지하고,  
환자가 버튼 호출 및 음성으로 필요한 사항을 전달할 수 있는 **비대면 케어 인프라**를 구축한다.

> "단순 기능 구현이 아니라, 실제 병원 운영 구조를 로봇 시스템으로 설계하는 프로젝트"

---

## 📌 Key Features

- YOLO11n-pose pretrained 모델 기반 낙상 감지 (keypoint 분석)
- Intel RealSense D435 탑뷰 이중 감지 구조 (낙상 의심 + 쓰레기통 모니터링)
- Whisper tiny 한국어 STT 기반 음성 인터랙션
- micro-ROS 기반 버튼 호출 + LED 상태 표시
- Nav2 + AMCL 자율주행 및 멀티 로봇 출동 판단

---

## 🚀 Quick Start

```bash
# 관제 PC - ROS2 환경 소싱
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash

# micro-ROS agent 실행 (아두이노 2개)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1

# 전체 시스템 실행
ros2 launch hospital_task_manager hospital_bringup.launch.py
```

---

## 🏗️ Architecture

```
[관제 PC - Ubuntu 22.04]
   ├── Nav2 / AMCL / map_server
   ├── hospital_task_manager  (C++)
   ├── yolo_node              (Python)  ← YOLO11n-pose
   ├── d435_node              (Python)  ← RealSense D435
   ├── whisper_node           (Python)  ← Whisper tiny STT
   ├── tts_node               (Python)  ← gTTS
   ├── gui_node               (C++/Qt6)
   ├── 아두이노1 (USB /dev/ttyUSB0) ← 방1 버튼/LED
   └── 아두이노2 (USB /dev/ttyUSB1) ← 방2 버튼/LED

[터틀봇1 RPi4]
   ├── turtlebot3_bringup
   ├── mic_node       (C++)  ← USB 마이크 녹음
   └── tts_play_node  (C++)  ← MAX98357A 재생

[터틀봇2 RPi4]
   ├── turtlebot3_bringup
   └── mic_node       (C++)  ← USB 마이크 녹음
```

| 노드 | 언어 | 실행 위치 | 역할 |
|---|---|---|---|
| `hospital_task_manager` | C++ | 관제 PC | 이벤트 판단 + 출동 명령 |
| `yolo_node` | Python | 관제 PC | 카메라 영상 → YOLO 낙상 감지 |
| `d435_node` | Python | 관제 PC | D435 깊이값 → 낙상 의심 / 쓰레기 감지 |
| `whisper_node` | Python | 관제 PC | 오디오 → STT → 키워드 매칭 |
| `tts_node` | Python | 관제 PC | TTS 생성 → 재생 명령 발행 |
| `gui_node` | C++ (Qt6) | 관제 PC | 관제 대시보드 |
| `mic_node` | C++ | 터틀봇 RPi4 | USB 마이크 녹음 → 오디오 토픽 발행 |
| `tts_play_node` | C++ | 터틀봇1 RPi4 | TTS 수신 → MAX98357A 재생 |
| `button_led_node` | Arduino C++ | 아두이노1, 2 | 버튼 감지 + LED 상태 표시 |

---

## 🔄 System Flow

### 평상시 순찰

```
두 터틀봇 스테이션 대기
    → 마지막 순찰 종료 후 10초 경과
    → 배터리 잔량 비교 → 잔량 많은 터틀봇 순찰 출발
    → 순찰 경로: 스테이션 → 복도 → 방1 → 방2
    → yolo_node: 카메라 토픽 구독 → YOLO11n-pose 추론
        ├── 정상 자세 → 계속 순찰
        └── 낙상 감지 → /hospital/emergency_call 발행
```

### 낙상 이중 감지

```
D435 낙상 의심 (침대 ROI 깊이값 변화)
    → /hospital/fall_suspected 발행
    → 가까운 터틀봇 출동
    → yolo_node 현장 확인
        ├── YOLO True  → 낙상 확정 🔴 /hospital/emergency_call
        └── YOLO False → 오탐지 처리 ✅ 순찰 복귀
```

### 음성 인터랙션 (터틀봇1 전담)

```
터틀봇1 병실 도착
    → "필요한 거 있으실까요?" TTS 재생 (~2초)
    → USB 마이크 녹음 (3~5초) → /robot_1/audio 발행
    → Whisper tiny STT 변환 (~0.5초)
    → 키워드 매칭
        ├── "약" / "약 주세요"  → /hospital/medicine_request
        ├── "간호사" / "도와줘" → /hospital/emergency_call
        └── "괜찮아" / 미인식   → 순찰 복귀

전체 소요 시간: 약 6~7초
```

---

## 📡 ROS2 Topic Structure

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/hospital/emergency_call` | String | 낙상 확정 or 긴급 버튼 (즉시 출동) |
| `/hospital/fall_suspected` | String | D435 낙상 의심 (YOLO 확인 출동) |
| `/hospital/call` | String | 일반 버튼 호출 |
| `/hospital/facility_status` | String | 쓰레기통 가득 참 |
| `/hospital/medicine_request` | String | 음성 키워드 "약" 감지 |
| `/hospital/emergency_event` | String | LED 제어용 (micro-ROS) |
| `/hospital/tts_trigger` | String | TTS 재생 트리거 |
| `/robot_1/audio` | ByteMultiArray | 터틀봇1 마이크 녹음 |
| `/robot_2/audio` | ByteMultiArray | 터틀봇2 마이크 녹음 |
| `/robot_1/tts_play` | String | 터틀봇1 TTS 재생 명령 |

---

## ⚙️ 이벤트 처리 구조

| 이벤트 | 감지 방법 | 우선순위 | 로봇 동작 |
|---|---|---|---|
| 낙상 확정 | YOLO keypoint | 🔴 즉시 출동 | 해당 위치 → 음성 확인 (터틀봇1) |
| 낙상 의심 | D435 깊이값 | 🔴 즉시 출동 | 가까운 터틀봇 → YOLO 확인 |
| 긴급 버튼 | 버튼 2회 연속 | 🔴 즉시 출동 | 가까운 터틀봇 출동 |
| 일반 버튼 | 버튼 1회 | 🟡 하던 일 후 | 가까운 터틀봇 출동 |
| 쓰레기 가득 | D435 깊이값 | 🟡 하던 일 후 | 수거 → 쓰레기 구역 이동 |

---

## 🛠️ 패키지 구조

```
hospital_robot_ws/
├── hospital_task_manager/     ← C++ 패키지
├── hospital_vision/           ← Python 패키지 (yolo_node, d435_node)
├── hospital_voice/            ← 혼용 패키지 (mic_node, tts_play_node, whisper_node, tts_node)
├── hospital_interface/        ← Arduino C++ (button_led_node)
└── hospital_gui/              ← C++ 패키지 (Qt6 gui_node)
```

---

## 🔧 Hardware

| 장치 | 수량 | 역할 |
|---|---|---|
| TurtleBot3 (RPi4 내장) | 2 | 자율주행 · 카메라 영상 발행 |
| Intel RealSense D435 | 1 | 탑뷰 낙상 의심 + 쓰레기통 깊이 감지 |
| MAX98357A | 1 | 터틀봇1 음성 안내 재생 |
| USB 마이크 | 2 | 환자 음성 녹음 |
| Arduino (micro-ROS) | 2 | 호출 버튼 + LED 상태 표시 |

---

## 👨‍💻 팀 구성

| 이름 | 역할 | 담당 영역 |
|---|---|---|
| 구영모 | Field Interface | mic_node · whisper_node · tts_node · tts_play_node · button_led_node (micro-ROS) |
| 안해성 | System Integrator | hospital_task_manager · Nav2 · AMCL · 멀티로봇 출동 판단 |
| 인수민 | Vision & Sensor | yolo_node (YOLO11n-pose) · d435_node (RealSense) |
| 곽종현 | GUI & Mapping | gui_node (Qt6) · SLAM 맵핑 · 세트장 구성 · 통합 테스트 |
