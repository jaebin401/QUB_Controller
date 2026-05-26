// =============================================================================
// apps/test_rt_thread.cpp
//
// Step-2 검증: 채널 1개에 RT 모터 스레드(SCHED_FIFO 500Hz)를 띄워서
//
//   - clock_nanosleep 기반 루프 주기가 실제로 2ms +- ? 인지
//   - missed_deadlines 가 얼마나 발생하는지
//   - SharedState::channel[ch] 슬롯이 정상적으로 갱신되는지
//
// 를 확인한다.
//
// 정책 스레드는 아직 없으므로, main 이 ActionState 를 직접 주입한다.
// 안전을 위해 q_des=0, Kp=5, Kd=0.3 hold 만 한다.
//
// 실행:
//   sudo ./test_rt_thread
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

static constexpr int TEST_CHANNEL_IDX   = 1;   // can1 = torso + R hip 그룹
static constexpr int TEST_DURATION_SEC  = 5;

static constexpr float HOLD_KP = 5.0f;
static constexpr float HOLD_KD = 0.3f;

static SharedState* g_shared_ptr = nullptr;

static void signal_handler(int)
{
    if (g_shared_ptr) g_shared_ptr->shutdown.store(true);
}

int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    printf("=== QUB_Controller Step-2 테스트: RT 모터 스레드 (1 채널) ===\n\n");

    rt::lock_memory();

    // --- CAN 채널 초기화 ----------------------------------------------------
    printf("[1] CAN 채널 초기화 (ch%d) ...\n", TEST_CHANNEL_IDX);
    std::unique_ptr<PcanChannel> ch;
    try {
        ch = std::make_unique<PcanChannel>(
            CAN_CHANNELS[TEST_CHANNEL_IDX], CAN_BAUDRATE);
        printf("    -> 성공\n\n");
    } catch (const std::exception& e) {
        printf("    -> 실패: %s\n", e.what());
        return 1;
    }

    // --- 이 채널의 모터들 생성 ----------------------------------------------
    printf("[2] 이 채널의 모터 인스턴스 생성 ...\n");

    std::vector<std::unique_ptr<Robstride>> motors;
    std::vector<MotorThread::MotorBinding>  bindings;

    for (const auto& entry : MOTOR_MAP) {
        if (entry.channel_idx != TEST_CHANNEL_IDX) continue;
        auto m = std::make_unique<Robstride>(ch.get(), entry.motor_id);
        printf("    - %s (motor_id=%d, joint_idx=%d)\n",
               entry.name, entry.motor_id, entry.joint_idx);
        bindings.push_back({m.get(), entry.joint_idx});
        motors.push_back(std::move(m));
    }
    printf("    총 %zu개 모터\n\n", motors.size());

    // --- SharedState + hold 액션 주입 ---------------------------------------
    SharedState shared;
    g_shared_ptr = &shared;

    {
        ActionState a;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            a.q_des[j]  = 0.f;
            a.dq_des[j] = 0.f;
            a.kp[j]     = HOLD_KP;
            a.kd[j]     = HOLD_KD;
            a.tau_ff[j] = 0.f;
        }
        a.valid = true;
        a.timestamp_ns = rt::now_ns();
        shared.action.store(a);
    }

    // --- RT 모터 스레드 시작 ------------------------------------------------
    printf("[3] MotorThread 시작 (SCHED_FIFO %d, %d Hz) ...\n",
           RT_PRIO_MOTOR, MOTOR_LOOP_HZ);

    MotorThread mt(&shared, TEST_CHANNEL_IDX, ch.get(), std::move(bindings));
    if (mt.start() != 0) {
        printf("    -> 시작 실패\n");
        return 1;
    }
    printf("    -> 시작됨\n\n");

    // --- 모니터링 (1Hz) -----------------------------------------------------
    printf("[4] 모니터링 (%d초간, Ctrl+C 로 중단) ...\n\n", TEST_DURATION_SEC);

    auto start = std::chrono::steady_clock::now();
    while (!shared.shutdown.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= TEST_DURATION_SEC) break;

        ChannelState cs = shared.channel[TEST_CHANNEL_IDX].load();

        printf("[t=%lds] loop=%llu  period_last=%ld ns (target=%ld)  miss=%d  gen=%u\n",
               (long)elapsed,
               (unsigned long long)mt.loop_count(),
               mt.last_period_ns(),
               MOTOR_PERIOD_NS,
               mt.missed_deadlines(),
               shared.channel[TEST_CHANNEL_IDX].generation());

        for (int i = 0; i < cs.num_motors; ++i) {
            printf("    j=%2d  q=%+.4f  dq=%+.4f  tau=%+.4f  online=%d\n",
                   cs.joint_idx[i], cs.q[i], cs.dq[i], cs.tau_est[i],
                   (int)cs.online[i]);
        }
        printf("\n");

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // --- 정지 --------------------------------------------------------------
    printf("[5] 정지 신호 -> motor stop -> join ...\n");
    shared.shutdown.store(true);
    mt.stop();

    printf("\n=== 결과 요약 ===\n");
    printf("  총 루프      : %llu\n", (unsigned long long)mt.loop_count());
    printf("  미스 데드라인 : %d\n", mt.missed_deadlines());
    printf("  마지막 주기   : %ld ns (목표 %ld ns)\n",
           mt.last_period_ns(), MOTOR_PERIOD_NS);
    return 0;
}