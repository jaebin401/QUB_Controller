// =============================================================================
// apps/test_4ch.cpp
//
// Step-3 검증: 4개 CAN 채널에 각각 RT 모터 스레드를 띄워서 모든 13개
// 관절을 동시에 hold 한다.
//
// 검증 항목:
//   - 4개 채널 동시 초기화 성공 여부
//   - 4개 SCHED_FIFO 스레드의 500Hz 루프 타이밍 (각 채널 독립)
//   - SharedState::channel[0..3] 슬롯이 각각 갱신되는지
//   - 13-DOF 합치기 가 정상 동작하는지 (policy thread 역할 흉내)
//
// 안전 설정:
//   - 모든 관절 q_des=0, Kp=5, Kd=0.3 hold
//   - SIGINT 시 모든 모터 stop() 까지 확실히 진행 후 종료
//
// 실행:
//   sudo ./test_4ch
//
// PCAN-M.2 4채널이 없는 환경 (예: Raspberry Pi + CANable) 에서는
// CAN_CHANNELS 상수를 SocketCAN 디바이스로 교체해서 빌드해야 한다.
// 하지만 현재 구조는 PCANBasic 전용이므로 NUC + PCAN-M.2 환경에서만
// 의미가 있다.
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
#include <array>

using namespace qub;

// =============================================================================
// 테스트 설정
// =============================================================================

static constexpr int TEST_DURATION_SEC = 10;
static constexpr float HOLD_KP = 5.0f;
static constexpr float HOLD_KD = 0.3f;

// =============================================================================
// 전역 종료 핸들러
// =============================================================================
static SharedState* g_shared_ptr = nullptr;

static void signal_handler(int)
{
    if (g_shared_ptr) g_shared_ptr->shutdown.store(true);
}

// =============================================================================
// 13-DOF 합치기 (policy thread 가 할 일을 흉내)
//
// SharedState::channel[0..3] 슬롯을 각각 load 해서 전체 13-DOF 배열
// (q, dq) 로 합친다. 한 슬롯이 갱신 중이면 seqlock 이 알아서 일관된
// 스냅샷을 보장.
// =============================================================================

struct CombinedState {
    std::array<float, NUM_JOINTS> q       = {};
    std::array<float, NUM_JOINTS> dq      = {};
    std::array<float, NUM_JOINTS> tau_est = {};
    std::array<bool,  NUM_JOINTS> online  = {};
};

static CombinedState combine_channels(const SharedState& shared)
{
    CombinedState out;
    // 기본은 모두 offline 으로
    for (int j = 0; j < NUM_JOINTS; ++j) out.online[j] = false;

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        ChannelState cs = shared.channel[ch].load();
        for (int i = 0; i < cs.num_motors; ++i) {
            int j = cs.joint_idx[i];
            if (j < 0 || j >= NUM_JOINTS) continue;
            out.q[j]       = cs.q[i];
            out.dq[j]      = cs.dq[i];
            out.tau_est[j] = cs.tau_est[i];
            out.online[j]  = cs.online[i];
        }
    }
    return out;
}

// =============================================================================
// main
// =============================================================================

int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    printf("=== QUB_Controller Step-3: 4채널 동시 RT 모터 스레드 ===\n\n");

    rt::lock_memory();

    // ─── 1. 4채널 초기화 ───────────────────────────────────────────────────
    printf("[1] 4채널 동시 초기화 ...\n");

    std::array<std::unique_ptr<PcanChannel>, NUM_CHANNELS> channels;
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        try {
            channels[ch] = std::make_unique<PcanChannel>(
                CAN_CHANNELS[ch], CAN_BAUDRATE);
            printf("    can%d (PCIBUS%d) → 성공\n", ch, ch + 1);
        } catch (const std::exception& e) {
            printf("    can%d → 실패: %s\n", ch, e.what());
            return 1;
        }
    }
    printf("\n");

    // ─── 2. 모터 인스턴스 생성 (채널별 분류) ───────────────────────────────
    printf("[2] 모터 인스턴스 생성 ...\n");

    std::vector<std::unique_ptr<Robstride>> motors;   // 소유권
    std::array<std::vector<MotorThread::MotorBinding>, NUM_CHANNELS> bindings_per_ch;

    for (const auto& entry : MOTOR_MAP) {
        auto m = std::make_unique<Robstride>(
            channels[entry.channel_idx].get(), entry.motor_id);
        bindings_per_ch[entry.channel_idx].push_back(
            {m.get(), entry.joint_idx});
        motors.push_back(std::move(m));
    }

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        printf("    can%d: %zu개 모터\n", ch, bindings_per_ch[ch].size());
    }
    printf("    총 %zu개 모터 = %d-DOF\n\n", motors.size(), NUM_JOINTS);

    // ─── 3. SharedState + hold 액션 ────────────────────────────────────────
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

    // ─── 4. 4개 motor_thread 생성 + 시작 ──────────────────────────────────
    printf("[3] 4개 RT 모터 스레드 시작 (SCHED_FIFO %d, %d Hz 각각) ...\n",
           RT_PRIO_MOTOR, MOTOR_LOOP_HZ);

    std::array<std::unique_ptr<MotorThread>, NUM_CHANNELS> threads;
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        threads[ch] = std::make_unique<MotorThread>(
            &shared, ch, channels[ch].get(), std::move(bindings_per_ch[ch]));
        if (threads[ch]->start() != 0) {
            printf("    ch%d → 시작 실패\n", ch);
            shared.shutdown.store(true);
            return 1;
        }
        printf("    ch%d → 시작됨\n", ch);
    }
    printf("\n");

    // ─── 5. 모니터링 (1Hz, %d초) ──────────────────────────────────────────
    printf("[4] 모니터링 (%d초간, Ctrl+C 로 중단) ...\n\n", TEST_DURATION_SEC);

    auto start = std::chrono::steady_clock::now();
    while (!shared.shutdown.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= TEST_DURATION_SEC) break;

        // 채널별 통계
        printf("─── t=%lds ────────────────────────────────────────────\n",
               (long)elapsed);
        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
            printf("  ch%d  loop=%-8llu  period=%-7ld ns  miss=%-3d  gen=%u\n",
                   ch,
                   (unsigned long long)threads[ch]->loop_count(),
                   threads[ch]->last_period_ns(),
                   threads[ch]->missed_deadlines(),
                   shared.channel[ch].generation());
        }

        // 13-DOF 합쳐서 한 줄로 (q 값만)
        CombinedState c = combine_channels(shared);
        printf("  q[0..12]:");
        for (int j = 0; j < NUM_JOINTS; ++j) {
            printf(" %+.3f%c", c.q[j], c.online[j] ? ' ' : '?');
        }
        printf("\n\n");

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ─── 6. 정지 ───────────────────────────────────────────────────────────
    printf("[5] 정지 신호 → 모든 모터 stop → join ...\n");
    shared.shutdown.store(true);

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        threads[ch]->stop();
    }

    // ─── 7. 결과 요약 ──────────────────────────────────────────────────────
    printf("\n=== 결과 요약 ===\n");
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        printf("  ch%d  총 루프=%llu  miss=%d\n",
               ch,
               (unsigned long long)threads[ch]->loop_count(),
               threads[ch]->missed_deadlines());
    }

    printf("\n다음 단계: IMU 스레드 (Step-4) + Policy 스레드 (Step-5)\n");
    return 0;
}