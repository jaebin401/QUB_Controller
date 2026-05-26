#pragma once
// =============================================================================
// robot_config.hpp
//
// QUB v1.2 휴머노이드 하드웨어 설정 상수 모음.
//
// 매직 넘버를 한 곳에 모아 두기 위한 헤더. 런타임 설정(Kp/Kd 게인,
// 기준 자세 등)은 별도 YAML 로 빠지고, 여기에는 "물리적으로 바뀌지
// 않는" 정보만 둔다:
//   - CAN 채널 ↔ 모터 ID 매핑
//   - 관절 인덱스 ↔ (채널, 모터 ID) 매핑
//   - 자유도 수 등 컴파일타임 상수
// =============================================================================

#include <PCANBasic.h>
#include <array>
#include <cstdint>

namespace qub {

// =============================================================================
// 자유도 (DOF) 정의
// =============================================================================

// 전체 자유도: 다리 12 + torso 1 = 13
constexpr int NUM_JOINTS    = 13;

// CAN 채널 수
constexpr int NUM_CHANNELS  = 4;

// =============================================================================
// 관절 인덱스
//
// 주의: Isaac Gym 의 DOF 순서와 일치시킨다.
//
// QUB_RL_v2 (tron1 fork / legged_gym 기반) 기준 — URDF 선언 순서:
//   0:  torso_yaw
//   1:  L_hip_pitch
//   2:  L_hip_roll
//   3:  L_hip_yaw
//   4:  L_knee_pitch
//   5:  L_ankle_pitch
//   6:  L_ankle_roll
//   7:  R_hip_pitch
//   8:  R_hip_roll
//   9:  R_hip_yaw
//   10: R_knee_pitch
//   11: R_ankle_pitch
//   12: R_ankle_roll
//
// (QUB_RL_v1 humanoid-gym 은 알파벳 순서였음. v2 와 호환을 우선시함.)
// =============================================================================

enum JointIdx : int {
    JOINT_TORSO_YAW     = 0,

    JOINT_L_HIP_PITCH   = 1,
    JOINT_L_HIP_ROLL    = 2,
    JOINT_L_HIP_YAW     = 3,
    JOINT_L_KNEE_PITCH  = 4,
    JOINT_L_ANKLE_PITCH = 5,
    JOINT_L_ANKLE_ROLL  = 6,

    JOINT_R_HIP_PITCH   = 7,
    JOINT_R_HIP_ROLL    = 8,
    JOINT_R_HIP_YAW     = 9,
    JOINT_R_KNEE_PITCH  = 10,
    JOINT_R_ANKLE_PITCH = 11,
    JOINT_R_ANKLE_ROLL  = 12,
};

// =============================================================================
// CAN 채널 매핑
//
// PCAN-M.2 4ch 보드의 채널 핸들. PCANBasic.h 에서 정의된 PCIBUS 상수 사용.
// =============================================================================

constexpr std::array<TPCANHandle, NUM_CHANNELS> CAN_CHANNELS = {
    PCAN_PCIBUS1,   // can0
    PCAN_PCIBUS2,   // can1
    PCAN_PCIBUS3,   // can2
    PCAN_PCIBUS4,   // can3
};

// CAN 통신 속도 (Robstride RS02/03/04 모두 1Mbps)
constexpr TPCANBaudrate CAN_BAUDRATE = PCAN_BAUD_1M;

// =============================================================================
// 모터 매핑
//
// 각 모터는 (CAN 채널, Robstride CAN ID, 관절 인덱스) 의 3-튜플로 표현.
// 이 배열을 motor_thread 가 받아서 채널별로 분배한다.
//
// 채널 ↔ 모터 그룹:
//   can0 (PCIBUS1) : R_knee(4), R_ankle_pitch(5), R_ankle_roll(6)
//   can1 (PCIBUS2) : torso_yaw(0), R_hip_pitch(1), R_hip_roll(2), R_hip_yaw(3)
//   can2 (PCIBUS3) : L_hip_pitch(1), L_hip_roll(2), L_hip_yaw(3)
//   can3 (PCIBUS4) : L_knee(4), L_ankle_pitch(5), L_ankle_roll(6)
//
// 모터의 CAN ID 는 채널 내에서만 유일하면 된다 (서로 다른 채널은 같은
// CAN ID 를 가질 수 있음).
// =============================================================================

struct MotorMapEntry {
    int          channel_idx;   // 0..3 (CAN_CHANNELS 배열의 인덱스)
    uint8_t      motor_id;      // Robstride CAN ID (1..253)
    int          joint_idx;     // JointIdx (0..12)
    const char*  name;          // 디버그/로그용
};

constexpr std::array<MotorMapEntry, NUM_JOINTS> MOTOR_MAP = {{
    // ───── can1 (PCIBUS2) — 4 모터: torso + R hip ─────
    {1, 0, JOINT_TORSO_YAW,     "torso_yaw"},
    {1, 1, JOINT_R_HIP_PITCH,   "R_hip_pitch"},
    {1, 2, JOINT_R_HIP_ROLL,    "R_hip_roll"},
    {1, 3, JOINT_R_HIP_YAW,     "R_hip_yaw"},

    // ───── can0 (PCIBUS1) — 3 모터: R knee/ankle ─────
    {0, 4, JOINT_R_KNEE_PITCH,  "R_knee_pitch"},
    {0, 5, JOINT_R_ANKLE_PITCH, "R_ankle_pitch"},
    {0, 6, JOINT_R_ANKLE_ROLL,  "R_ankle_roll"},

    // ───── can2 (PCIBUS3) — 3 모터: L hip ─────
    {2, 1, JOINT_L_HIP_PITCH,   "L_hip_pitch"},
    {2, 2, JOINT_L_HIP_ROLL,    "L_hip_roll"},
    {2, 3, JOINT_L_HIP_YAW,     "L_hip_yaw"},

    // ───── can3 (PCIBUS4) — 3 모터: L knee/ankle ─────
    {3, 4, JOINT_L_KNEE_PITCH,  "L_knee_pitch"},
    {3, 5, JOINT_L_ANKLE_PITCH, "L_ankle_pitch"},
    {3, 6, JOINT_L_ANKLE_ROLL,  "L_ankle_roll"},
}};

// =============================================================================
// 실시간 스레드 설정
// =============================================================================

// SCHED_FIFO 우선순위
constexpr int RT_PRIO_MOTOR  = 90;   // 가장 높음 (하드 RT)
constexpr int RT_PRIO_IMU    = 80;
constexpr int RT_PRIO_POLICY = 70;

// 루프 주기
constexpr int MOTOR_LOOP_HZ  = 500;  // 2ms
constexpr int IMU_LOOP_HZ    = 200;  // 5ms
constexpr int POLICY_LOOP_HZ = 50;   // 20ms

// 주기를 나노초 단위로 변환 (clock_nanosleep 용)
constexpr long MOTOR_PERIOD_NS  = 1'000'000'000L / MOTOR_LOOP_HZ;
constexpr long IMU_PERIOD_NS    = 1'000'000'000L / IMU_LOOP_HZ;
constexpr long POLICY_PERIOD_NS = 1'000'000'000L / POLICY_LOOP_HZ;

// =============================================================================
// MIT 모드 기본 게인 (튜닝 시작점, 실제 값은 YAML 에서 override)
// =============================================================================

// 모든 관절 공통 시작값 — 실제로는 관절별로 다르게 가야 함
constexpr float DEFAULT_KP = 30.0f;
constexpr float DEFAULT_KD = 1.0f;

// =============================================================================
// CAN 통신 타임아웃
// =============================================================================

// MIT 명령 송신 후 피드백 대기 최대 시간 (마이크로초)
constexpr long CAN_FEEDBACK_TIMEOUT_US = 1000;  // 1ms

// 피드백이 N 사이클 이상 안 들어오면 통신 끊김으로 판단
constexpr int  MOTOR_TIMEOUT_CYCLES = 10;

} // namespace qub
