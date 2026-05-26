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
//   - mutex/cond var 없음 → priority inversion 위험 없음.
//
// 동작 방식:
//   1. Writer 가 시작할 때 seq 를 1 증가시키고 (홀수 → "쓰는 중"),
//      데이터를 쓴 뒤 seq 를 또 1 증가 (짝수 → "쓰기 완료")
//   2. Reader 는 seq 를 읽고 (짝수가 될 때까지 기다림), 데이터 복사,
//      seq 를 다시 읽음. 두 seq 가 같으면 일관된 스냅샷이다.
//
// 이 구조의 트레이드오프:
//   - Reader 가 가끔 retry 해야 함 (writer 와 충돌 시) — 평균은 무시 가능
//   - 데이터 크기가 클수록 retry 비용이 커짐 — 우리 데이터는 충분히 작음
// =============================================================================

#include <atomic>
#include <array>
#include <cstdint>

#include "robot_config.hpp"

namespace qub {

// =============================================================================
// 모터 상태 (motor thread 가 쓰고, policy thread 가 읽음)
// =============================================================================

struct MotorState {
    // 관절별 측정값 (인덱스는 JointIdx 와 동일, 0..12)
    std::array<float, NUM_JOINTS> q       = {};  // 위치 [rad]
    std::array<float, NUM_JOINTS> dq      = {};  // 속도 [rad/s]
    std::array<float, NUM_JOINTS> tau_est = {};  // 추정 토크 [Nm]

    // 관절별 통신 상태 (false 면 해당 관절은 피드백 끊김)
    std::array<bool,  NUM_JOINTS> online  = {};

    // 갱신 시각 (CLOCK_MONOTONIC ns)
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
    std::array<float, 3> base_lin_v = {};            // [m/s]     base frame (선택)

    uint64_t timestamp_ns = 0;
};

// =============================================================================
// 액션 (policy thread 가 쓰고, motor thread 가 읽음)
// =============================================================================

struct ActionState {
    // 관절별 목표값 (MIT 모드)
    std::array<float, NUM_JOINTS> q_des   = {};  // 목표 위치 [rad]
    std::array<float, NUM_JOINTS> dq_des  = {};  // 목표 속도 [rad/s], 보통 0
    std::array<float, NUM_JOINTS> kp      = {};  // Kp
    std::array<float, NUM_JOINTS> kd      = {};  // Kd
    std::array<float, NUM_JOINTS> tau_ff  = {};  // 피드포워드 토크 [Nm]

    uint64_t timestamp_ns = 0;
    bool     valid        = false;  // 정책이 한 번이라도 추론을 끝냈는가
};

// =============================================================================
// Seqlock 템플릿 — 위 구조체들을 감싼다
//
// 사용법:
//   SeqLocked<MotorState> motor_buf;
//   // writer (motor thread):
//   MotorState s = ...;
//   motor_buf.store(s);
//   // reader (policy thread):
//   MotorState s = motor_buf.load();   // 항상 일관된 스냅샷
// =============================================================================

template <typename T>
class SeqLocked {
public:
    SeqLocked() : seq_(0) {}

    // Writer: 단일 스레드에서만 호출 가정
    void store(const T& value) {
        // seq 를 홀수로: "지금 쓰는 중"
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);

        // 데이터 복사 — writer 는 막히지 않는다
        data_ = value;

        // seq 를 짝수로: "쓰기 완료"
        seq_.store(s + 2, std::memory_order_release);
    }

    // Reader: 여러 스레드에서 동시 호출 가능
    // 반환: 일관된 스냅샷. 최악의 경우 잠시 spin 하지만 평균은 1회 통과.
    T load() const {
        T snapshot;
        while (true) {
            uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u) continue;          // writer 가 쓰는 중 → 재시도

            snapshot = data_;               // 데이터 복사

            uint32_t s2 = seq_.load(std::memory_order_acquire);
            if (s1 == s2) return snapshot;  // 일관된 스냅샷
            // s1 != s2 면 도중에 writer 가 끼어든 것 → 재시도
        }
    }

    // 갱신 횟수 / 4 (간단한 통신 살아있나 체크용)
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
    SeqLocked<MotorState>  motor;
    SeqLocked<ImuState>    imu;
    SeqLocked<ActionState> action;

    // 전역 종료 플래그 — 모든 스레드가 폴링
    std::atomic<bool> shutdown{false};
};

} // namespace qub
