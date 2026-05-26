// =============================================================================
// apps/test_rt_thread.cpp
//
// Step-2 검증: 채널 1개에 RT 모터 스레드(SCHED_FIFO 500Hz)를 띄워서
//
//   - clock_nanosleep 기반 루프 주기가 실제로 2ms ± ? 인지
//   - missed_deadlines 가 얼마나 발생하는지
//   - SharedState::motor 가 정상적으로 갱신되는지
//
// 를 확인한다.
//
// 정책 스레드는 아직 없으므로, main 이 일정 자세를 ActionState 에
// 주입하는 역할을 대신 한다 (1초 동안 0 → 0.3 rad 까지 ramp 후 hold).
//
// 실행:
//   sudo ./test_rt_thread
//
// 출력 예시:
//   loop=1000  period=2003ns  miss=0  q[0]=0.0152rad  dq[0]=0.42 rad/s
//   ...
// =============================================================================

#include "pcan_channel.hpp"
#include "robstride.hpp"
#include "shared_state.hpp"
#include "motor_thread.hpp"
#include "robot_config.hpp"
#include "rt_utils.hpp"

#include <cstdio>
#include <csignal>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>

using namespace qub;

// =============================================================================
// 테스트 설정 — 한 채널만, 그 채널에 매달린 모든 모터 사용
// =============================================================================

// 어떤 채널을 테스트할지 (0..3, CAN_CHANNELS 인덱스)
static constexpr int TEST_CHANNEL_IDX = 1;   // can1 = torso + R hip 그룹

// 총 실행 시간
static constexpr int TEST_DURATION_SEC = 5;

// Ramp 목표 (모든 관절 동일)
static constexpr float RAMP_TARGET_RAD = 0.0f;   // 안전을 위해 0으로
static constexpr float RAMP_KP         = 5.0f;   // 작게 — 첫 테스트
static constexpr float RAMP_KD         = 0.3f;

// =============================================================================
// SIGINT 핸들러
// =============================================================================

static SharedState* g_shared_ptr = nullptr;

static void signal_handler(int)
{
    if (g_shared_ptr) g_shared_ptr->shutdown.store(true);
}

// =============================================================================
// main
// =============================================================================

int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    printf("=== QUB_Controller Step-2 테스트: RT 모터 스레드 ===\n\n");

    // ─── 1. 메모리 락 (RT 안정성) ──────────────────────────────────────────
    rt::lock_memory();

    // ─── 2. CAN 채널 초기화 ────────────────────────────────────────────────
    printf("[1] CAN 채널 초기화 (ch%d) ...\n", TEST_CHANNEL_IDX);
    std::unique_ptr<PcanChannel> ch;
    try {
        ch = std::make_unique<PcanChannel>(
            CAN_CHANNELS[TEST_CHANNEL_IDX], CAN_BAUDRATE);
        printf("    → 성공\n\n");
    } catch (const std::exception& e) {
        printf("    → 실패: %s\n", e.what());
        return 1;
    }

    // ─── 3. 이 채널에 속한 모터들 생성 ─────────────────────────────────────
    printf("[2] 이 채널의 모터 인스턴스 생성 ...\n");

    std::vector<std::unique_ptr<Robstride>> motors;
    std::vector<MotorThread::MotorBinding> bindings;

    for (const auto& entry : MOTOR_MAP) {
        if (entry.channel_idx != TEST_CHANNEL_IDX) continue;

        auto m = std::make_unique<Robstride>(ch.get(), entry.motor_id);
        printf("    - %s (motor_id=%d, joint_idx=%d)\n",
               entry.name, entry.motor_id, entry.joint_idx);

        bindings.push_back({m.get(), entry.joint_idx});
        motors.push_back(std::move(m));
    }
    printf("    총 %zu개 모터\n\n", motors.size());

    // ─── 4. SharedState 준비 + 초기 액션 주입 ─────────────────────────────
    SharedState shared;
    g_shared_ptr = &shared;

    {
        ActionState a;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            a.q_des[j]  = 0.f;
            a.dq_des[j] = 0.f;
            a.kp[j]     = RAMP_KP;
            a.kd[j]     = RAMP_KD;
            a.tau_ff[j] = 0.f;
        }
        a.valid = true;
        a.timestamp_ns = rt::now_ns();
        shared.action.store(a);
    }

    // ─── 5. RT 모터 스레드 시작 ────────────────────────────────────────────
    printf("[3] MotorThread 시작 (SCHED_FIFO %d, %d Hz) ...\n",
           RT_PRIO_MOTOR, MOTOR_LOOP_HZ);

    MotorThread mt(&shared, TEST_CHANNEL_IDX, ch.get(), std::move(bindings));
    if (mt.start() != 0) {
        printf("    → 시작 실패\n");
        return 1;
    }
    printf("    → 시작됨\n\n");

    // ─── 6. 메인 루프: 상태 모니터링 (1Hz) ────────────────────────────────
    printf("[4] 모니터링 (%d초간, Ctrl+C 로 중단) ...\n\n", TEST_DURATION_SEC);

    auto start = std::chrono::steady_clock::now();
    while (!shared.shutdown.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= TEST_DURATION_SEC) break;

        MotorState ms = shared.motor.load();

        printf("[t=%lds] loop=%llu  period_last=%ld ns (target=%ld)  miss=%d\n",
               (long)elapsed,
               (unsigned long long)mt.loop_count(),
               mt.last_period_ns(),
               MOTOR_PERIOD_NS,
               mt.missed_deadlines());

        for (const auto& entry : MOTOR_MAP) {
            if (entry.channel_idx != TEST_CHANNEL_IDX) continue;
            int j = entry.joint_idx;
            printf("    %-16s j=%2d  q=%+.4f  dq=%+.4f  tau=%+.4f  online=%d\n",
                   entry.name, j,
                   ms.q[j], ms.dq[j], ms.tau_est[j], (int)ms.online[j]);
        }
        printf("\n");

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ─── 7. 정지 ────────────────────────────────────────────────────────────
    printf("[5] 정지 신호 → motor stop → join ...\n");
    shared.shutdown.store(true);
    mt.stop();

    printf("\n=== 결과 요약 ===\n");
    printf("  총 루프      : %llu\n", (unsigned long long)mt.loop_count());
    printf("  미스 데드라인 : %d\n", mt.missed_deadlines());
    printf("  마지막 주기   : %ld ns (목표 %ld ns)\n",
           mt.last_period_ns(), MOTOR_PERIOD_NS);

    printf("\n다음 단계: 4채널 동시 띄우기 + IMU 스레드 추가\n");
    return 0;
}
