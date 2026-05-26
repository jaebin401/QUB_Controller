#pragma once
// =============================================================================
// shared_state.hpp
//
// 스레드 간 데이터 교환을 위한 lock-free 공유 상태.
//
// 패턴: seqlock (single-writer, multi-reader, RT-safe)
//
//   - Writer 는 절대 막히지 않는다 (모터/IMU 스레드의 RT 보장 핵심).
//   - Reader 는 일관된 스냅샷을 얻는다 (찢어진 데이터를 보지 않음).
//   - mutex/cond var 없음 -> priority inversion 위험 없음.
//
// 동작 방식:
//   1. Writer 가 시작할 때 seq 를 1 증가시키고 (홀수 -> "쓰는 중"),
//      데이터를 쓴 뒤 seq 를 또 1 증가 (짝수 -> "쓰기 완료")
//   2. Reader 는 seq 를 읽고 (짝수가 될 때까지 기다림), 데이터 복사,
//      seq 를 다시 읽음. 두 seq 가 같으면 일관된 스냅샷이다.
//
// --- Step-3 리팩토링 노트 -----------------------------------------------
// 모터 상태는 채널별 슬롯 (ChannelState[NUM_CHANNELS]) 로 분리한다.
// 이유:
//   - 4개 MotorThread 가 같은 SeqLocked<MotorState> 에 write 하면
//     "single writer" 가정이 깨지고 reader 가 무한 retry 할 수 있다.
//   - 채널마다 자기 슬롯만 가지면 각 SeqLocked 는 진짜 single writer.
//   - Policy thread 는 4개 슬롯을 각각 load() 해서 하나의 전체 obs
//     로 합치는 책임을 진다.
// =============================================================================

#include <atomic>
#include <array>
#include <cstdint>

#include "robot_config.hpp"

namespace qub {

// =============================================================================
// 채널 1개 분량의 모터 상태
//
// 각 채널에는 최대 4개의 모터가 매달려 있다 (현재 매핑상 한 채널당
// 3~4 모터). 한 채널의 motor_thread 가 자기 슬롯만 write 한다.
//
// 인덱싱:
//   - num_motors: 이 채널에 실제로 매달린 모터 수
//   - joint_idx[i]: i번째 모터가 담당하는 전체 13-DOF 중의 관절 인덱스
//   - q[i], dq[i], tau_est[i], online[i]: i번째 모터의 측정값
//
// Policy thread 는 4채널을 합쳐 13-DOF 배열을 만들 때:
//   for ch in 0..3:
//     s = channel[ch].load();
//     for i in 0..s.num_motors:
//       global_q[s.joint_idx[i]] = s.q[i];
// =============================================================================

constexpr int MAX_MOTORS_PER_CHANNEL = 4;  // can1 이 4개로 가장 많음

struct ChannelState {
    int     num_motors = 0;
    std::array<int,   MAX_MOTORS_PER_CHANNEL> joint_idx = {};
    std::array<float, MAX_MOTORS_PER_CHANNEL> q         = {};
    std::array<float, MAX_MOTORS_PER_CHANNEL> dq        = {};
    std::array<float, MAX_MOTORS_PER_CHANNEL> tau_est   = {};
    std::array<bool,  MAX_MOTORS_PER_CHANNEL> online    = {};

    uint64_t timestamp_ns = 0;
};

// =============================================================================
// IMU 상태 (imu thread 가 쓰고, policy thread 가 읽음)
//
// 좌표계: base_link (torso). pelvis-frame 변환은 policy thread 에서
// torso_yaw 인코더값과 함께 계산.
// =============================================================================

struct ImuState {
    std::array<float, 4> quat       = {1, 0, 0, 0};  // (w, x, y, z)
    std::array<float, 3> gyro       = {};            // [rad/s]   base frame
    std::array<float, 3> accel      = {};            // [m/s^2]   base frame
    std::array<float, 3> base_lin_v = {};            // [m/s]     base frame

    uint64_t timestamp_ns = 0;
};

// =============================================================================
// 액션 (policy thread 가 쓰고, motor thread 가 읽음)
//
// 13-DOF 전체 배열. Motor thread 는 자기 채널의 모터 인덱스만 읽어 사용.
// =============================================================================

struct ActionState {
    std::array<float, NUM_JOINTS> q_des   = {};
    std::array<float, NUM_JOINTS> dq_des  = {};
    std::array<float, NUM_JOINTS> kp      = {};
    std::array<float, NUM_JOINTS> kd      = {};
    std::array<float, NUM_JOINTS> tau_ff  = {};

    uint64_t timestamp_ns = 0;
    bool     valid        = false;
};

// =============================================================================
// Seqlock 템플릿
//
// 사용법:
//   SeqLocked<ChannelState> ch;
//   ch.store(s);            // writer (이 인스턴스에 대해 단 하나의 스레드)
//   auto s = ch.load();     // reader (여러 스레드 가능)
// =============================================================================

template <typename T>
class SeqLocked {
public:
    SeqLocked() : seq_(0) {}

    void store(const T& value) {
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);   // 홀수: 쓰는 중

        data_ = value;

        seq_.store(s + 2, std::memory_order_release);   // 짝수: 완료
    }

    T load() const {
        T snapshot;
        while (true) {
            uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u) continue;
            snapshot = data_;
            uint32_t s2 = seq_.load(std::memory_order_acquire);
            if (s1 == s2) return snapshot;
        }
    }

    uint32_t generation() const {
        return seq_.load(std::memory_order_relaxed) >> 1;
    }

private:
    mutable std::atomic<uint32_t> seq_;
    T                             data_{};
};

// =============================================================================
// 전체 공유 상태 컨테이너
// =============================================================================

struct SharedState {
    // 채널별 모터 상태 - 각 슬롯은 해당 motor_thread 가 단독 writer
    std::array<SeqLocked<ChannelState>, NUM_CHANNELS> channel;

    // IMU 와 액션은 각각 1개 writer 이므로 단일 슬롯
    SeqLocked<ImuState>    imu;
    SeqLocked<ActionState> action;

    // 전역 종료 플래그 - 모든 스레드가 폴링
    std::atomic<bool> shutdown{false};
};

} // namespace qub