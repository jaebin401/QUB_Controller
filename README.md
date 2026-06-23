# QUB_Controller

Robstride RS-series 액추에이터 + MicroStrain CV7-AHRS IMU + ONNX 정책을
연동하는 QUB v1.2 휴머노이드(13-DOF) 실시간 제어기.

PREEMPT-RT 커널 + PEAK PCAN-M.2 4ch (PCANBasic chardev) 기반의 C++ 코드.

---

## 디렉토리 구조

```
QUB_Controller/
├── include/                       # 헤더만 (.hpp)
│   ├── robot_config.hpp           # 채널/모터/관절 매핑, RT 설정 상수
│   ├── pcan_channel.hpp           # PCANBasic SDK 래퍼
│   ├── robstride.hpp              # Robstride CAN 프로토콜 (MIT mode)
│   ├── shared_state.hpp           # Seqlock 기반 lock-free 공유 상태
│   ├── rt_utils.hpp               # RT 스레드 유틸 (SCHED_FIFO, 주기 슬립)
│   ├── motor_thread.hpp           # 채널당 1개 RT 모터 스레드
│   ├── imu_driver.hpp             # CV7-AHRS mip_sdk 래퍼 + IMU 스레드
│   └── policy_runner.hpp          # (예정) ONNX Runtime 정책 추론
├── src/                           # 라이브러리 구현 (qub_hw / qub_imu 로 묶임)
│   ├── pcan_channel.cpp
│   ├── robstride.cpp
│   ├── motor_thread.cpp
│   ├── imu_driver.cpp
│   └── policy_runner.cpp          # (예정)
├── apps/                          # 실행파일 진입점 (main 있음)
│   ├── test_can_init.cpp          # Step-1 검증
│   ├── test_rt_thread.cpp         # Step-2 검증
│   ├── test_4ch.cpp               # Step-3 검증
│   ├── test_imu.cpp               # Step-4 검증
│   └── main.cpp                   # (예정) 최종 컨트롤러
├── Samples/                       # 옛 참고용 (motorPCAN, Robstride 원본)
├── CMakeLists.txt
├── .gitignore
└── README.md
```

> `include/`와 `src/`는 1:1 매칭이 아니다.
> `robot_config`, `shared_state`, `rt_utils`는 헤더 전용 (header-only) —
> 모든 함수가 inline 또는 템플릿이라 .cpp가 없다.

---

## 의존성

| 라이브러리       | 용도                    | 설치 방법                                   |
|------------------|-------------------------|---------------------------------------------|
| libpcanbasic     | CAN 통신 (chardev)      | peak-linux-driver `make && make install`    |
| pthread          | RT 스레드               | 시스템 기본                                 |
| rt               | clock_nanosleep         | 시스템 기본 (`-lrt`)                        |
| mip_sdk          | CV7-AHRS IMU 데이터     | 아래 [mip_sdk 빌드](#mip_sdk-빌드) 참고     |
| onnxruntime      | 정책 추론               | `cmake -DONNXRUNTIME_ROOT=<path>`           |
| yaml-cpp         | config 로드 (예정)      | `apt install libyaml-cpp-dev`               |

---

## 빌드

### 기본 빌드 (PCAN + 모터 스레드만, mip_sdk 없이)

```bash
cd QUB_Controller
mkdir build && cd build
cmake ..
make -j$(nproc)
```

빌드 결과:
- `libqub_hw.a` — PCAN + 모터 스레드 정적 라이브러리
- `test_can_init`, `test_rt_thread`, `test_4ch`

### mip_sdk 포함 빌드 (IMU 스레드 추가)

mip_sdk를 먼저 빌드/설치한 뒤:

```bash
cmake .. -DMIP_SDK_ROOT=/opt/mip_sdk
make -j$(nproc)
```

추가 빌드 결과:
- `libqub_imu.a` — IMU 드라이버 정적 라이브러리
- `test_imu`

### PCAN 경로가 비표준이면

```bash
cmake .. -DPCAN_INCLUDE_DIR=/path/to/include -DPCAN_LIB_DIR=/path/to/lib
```

---

## 실행

PCAN 드라이버 접근에 root 권한 필요:

```bash
sudo ./test_can_init      # Step-1: CAN 채널 + 모터 단독 동작 확인
sudo ./test_rt_thread     # Step-2: 1채널 RT 500Hz 루프 타이밍 검증
sudo ./test_4ch           # Step-3: 4채널 동시 RT + 13-DOF 합치기
sudo ./test_imu           # Step-4: IMU 스레드 단독 동작 확인
sudo ./test_imu /dev/ttyACM0   # 포트 직접 지정
```

RT 우선순위를 쓰려면 (`/etc/security/limits.conf`):

```
<user>  -  rtprio  99
<user>  -  memlock unlimited
```

설정 후 재로그인 필요. sudo 없이도 RT 루프 동작 가능.

---

## 개발 단계 (Incremental Integration)

| 단계 | 파일                  | 내용                                                    |
|------|-----------------------|---------------------------------------------------------|
| ✅ 1 | `test_can_init`       | 싱글스레드 CAN 초기화, MIT 명령, 피드백 파싱            |
| ✅ 2 | `test_rt_thread`      | 1채널 SCHED_FIFO 500Hz 루프, clock_nanosleep 타이밍     |
| ✅ 3 | `test_4ch`            | 4채널 동시 RT 모터 스레드, 13-DOF 합치기                |
| ✅ 4 | `test_imu`            | CV7-AHRS mip_sdk 연동, IMU 스레드 200Hz                 |
| 🔲 5 | Policy 스레드         | ONNX Runtime, obs 조립, ActionState 게시                |
| 🔲 6 | `main.cpp`            | 전체 조립, config 로드, graceful shutdown               |

---

## CAN 채널 ↔ 모터 매핑

| 채널               | 모터 ID  | 관절                                          |
|--------------------|----------|-----------------------------------------------|
| PCAN_PCIBUS1 (can0) | 4,5,6   | R_knee, R_ankle_pitch, R_ankle_roll           |
| PCAN_PCIBUS2 (can1) | 0,1,2,3 | torso_yaw, R_hip_pitch, R_hip_roll, R_hip_yaw |
| PCAN_PCIBUS3 (can2) | 1,2,3   | L_hip_pitch, L_hip_roll, L_hip_yaw            |
| PCAN_PCIBUS4 (can3) | 4,5,6   | L_knee, L_ankle_pitch, L_ankle_roll           |

전체 13-DOF 인덱스 순서는 `include/robot_config.hpp` 참고
(QUB_RL_v2 URDF 선언 순서와 일치).

---

## 아키텍처 개요

스레드 구성 (PREEMPT-RT, NUC 11):

| 스레드          | 우선순위 | 주기   | 역할                                |
|-----------------|----------|--------|-------------------------------------|
| Motor (×4채널)  | 90       | 500Hz  | MIT 명령 송신, 피드백 파싱          |
| IMU             | 80       | 200Hz  | CV7 quat/gyro/accel 수신            |
| Policy          | 70       | 50Hz   | obs 조립, ONNX 추론, action 생성    |

스레드 간 데이터 공유는 `SharedState` (Seqlock 패턴, lock-free):
- `channel[0..3]` — 각 motor_thread가 자기 슬롯만 write, Policy가 read
- `imu` — IMU 스레드가 write, Policy가 read
- `action` — Policy가 write, Motor 스레드가 read

---

## mip_sdk 빌드

CV7-AHRS IMU를 쓰기 위해 MicroStrain 공식 mip_sdk를 직접 빌드해야 한다.
MSCL(구형)이 아닌 mip_sdk(신형)를 사용한다.

### NUC (Ubuntu)에서 mip_sdk 빌드 및 설치

```bash
# 소스 클론 (이미 받아둔 경우 생략)
git clone https://github.com/LORD-MicroStrain/mip_sdk.git
cd mip_sdk

# 빌드
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/mip_sdk \
    -DMIP_BUILD_EXAMPLES=OFF \
    -DMIP_BUILD_TESTS=OFF

make -j$(nproc)
sudo make install
```

설치 후 `/opt/mip_sdk/` 아래에 `include/`, `lib/`가 생김.

### QUB_Controller에서 mip_sdk 연동

```bash
cd QUB_Controller/build
cmake .. -DMIP_SDK_ROOT=/opt/mip_sdk
make -j$(nproc)
```

---

## NUC에서 CV7-AHRS 초기 세팅

### 1. udev rule 설정 (포트 고정)

CV7-AHRS를 USB로 연결하면 `/dev/ttyUSB0` 또는 `/dev/ttyACM0`으로 잡힌다.
재부팅 또는 재연결 시 번호가 바뀔 수 있으므로 udev rule로 고정 권장.

```bash
# 연결된 장치의 ID 확인
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'

# /etc/udev/rules.d/99-cv7.rules 생성
sudo tee /etc/udev/rules.d/99-cv7.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="199b", ATTRS{idProduct}=="3065", \
    SYMLINK+="cv7ahrs", MODE="0666"
EOF

# 규칙 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
```

이후 `/dev/cv7ahrs` 로 접근 가능. `robot_config.hpp`의 `IMU_SERIAL_PORT`를 이 경로로 수정.

> idVendor/idProduct는 실제 장치에서 확인한 값으로 교체 필요.

### 2. 시리얼 포트 권한

```bash
# 현재 사용자를 dialout 그룹에 추가 (재로그인 후 적용)
sudo usermod -aG dialout $USER

# 또는 임시로 권한 부여
sudo chmod 666 /dev/ttyUSB0
```

### 3. 연결 확인

```bash
# 장치 인식 여부 확인
ls /dev/ttyUSB* /dev/ttyACM*

# 시리얼 통신 확인 (minicom 사용)
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
# MIP 패킷 바이트가 흘러나오면 정상
# Ctrl+A → X 로 종료
```

### 4. CV7-AHRS 설정 기본값

`imu_driver.cpp`의 `configure()` 시퀀스에서 적용하는 설정:

| 항목 | 값 | 비고 |
|------|----|------|
| Baudrate | 115200 | CV7 기본값 |
| 필터 출력 | 250Hz | base rate 500Hz, decimation=2 |
| 수신 데이터 | AttitudeQuaternion, CompensatedAngularRate, CompensatedAcceleration, Status | |
| Heading aiding | 자력계 (magnetometer) | AHRS 모드 진입 조건 |
| 필터 모드 | AHRS (mode=3) | Full NAV(4)는 GNSS 없이 불가 |

> 필터 base rate는 실제 장치에서 `getBaseRate()` 응답으로 확인.
> 500Hz가 아닐 경우 `imu_driver.cpp`의 `IMU_FILTER_BASE_RATE_HZ` 상수 수정 필요.

### 5. 좌표계 주의사항

CV7-AHRS는 `base_link` (torso)에 마운트되어 있어 pelvis 프레임이 아님.
Policy thread에서 obs를 조립할 때 `torso_yaw` 인코더값으로 pelvis 프레임 변환 필요:

```
pelvis_vx = cos(θ) * base_vx + sin(θ) * base_vy
pelvis_vy = -sin(θ) * base_vx + cos(θ) * base_vy
```

`θ` = `SharedState::channel[1].q[0]` (JOINT_TORSO_YAW, joint_idx=0, can1 첫 번째 모터)

---

## 다음에 할 일

### 코드 검증 (빌드 전 수정)
- [ ] `test_can_init.cpp` — `bool ok` 미사용 변수 제거
- [ ] `motor_thread.hpp` — `last_action_gen_` 죽은 멤버 제거
- [ ] `motor_thread.hpp` — 헤더 주석 갱신 (`SharedState::motor` → `channel[ch]`)
- [ ] `imu_driver.cpp` — `CompensatedAngularRate::gyro`, `CompensatedAcceleration::accel` 실제 멤버명 NUC 빌드 후 확인

### NUC 빌드 및 검증
- [ ] PCAN-M.2 드라이버 설치 → `test_can_init` 빌드 + 실행
- [ ] mip_sdk 빌드/설치 → `test_imu` 빌드 + CV7 연결 후 실행
- [ ] `cyclictest`로 PREEMPT-RT jitter 측정 (목표 <100µs)
- [ ] `test_4ch` 30분 장시간 실행 → `missed_deadlines` 분포 확인

### Step-5 (Policy 스레드)
- [ ] ONNX Runtime 설치 (`cmake -DONNXRUNTIME_ROOT=<path>`)
- [ ] `policy_runner.hpp/.cpp` — obs 조립 + ONNX 추론 + ActionState 게시
- [ ] `apps/test_policy.cpp` — 정책 추론 단독 검증

### 코드 이해 (학습)
- [ ] Robstride 매뉴얼 4장 (CAN 프로토콜, MIT 모드 인코딩)
- [ ] mip_sdk examples 직접 읽기 (`7_series_ahrs_example.cpp`)
- [ ] Effective Modern C++ 1~6장 (RAII, unique_ptr, move)
- [ ] Preshing 블로그 → C++ Concurrency in Action 5장 (memory order, seqlock)

---

## 라이선스

- `libpcanbasic` : LGPL-2.1 (동적 링크, 소스 공개 의무 없음)
- `mip_sdk` : MIT
- 이 프로젝트 코드 : MIT (예정)