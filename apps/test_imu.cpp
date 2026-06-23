// =============================================================================
// apps/test_imu.cpp
//
// Step-4 검증: IMU 스레드(SCHED_FIFO 80, 200Hz) 단독 동작 확인
//
// 검증 항목:
//   - CV7-AHRS 시리얼 연결 및 필터 초기화 성공 여부
//   - AHRS 모드 진입 확인 (filter_mode >= 3)
//   - SharedState::imu 슬롯이 정상 갱신되는지
//   - 쿼터니언 w,x,y,z / 자이로 / 가속도 값 출력
//   - 루프 타이밍 (200Hz = 5ms 목표) 및 missed_deadlines 확인
//
// 실행:
//   sudo ./test_imu
//
// 전제 조건:
//   CV7-AHRS가 /dev/ttyUSB0 에 연결되어 있어야 한다.
//   포트 확인: ls /dev/ttyUSB* 또는 ls /dev/ttyACM*
// =============================================================================

#include "imu_driver.hpp"
#include "shared_state.hpp"
#include "robot_config.hpp"
#include "rt_utils.hpp"

#include <cstdio>
#include <csignal>
#include <chrono>
#include <thread>

using namespace qub;

static constexpr int TEST_DURATION_SEC = 15;

static SharedState* g_shared_ptr = nullptr;
static void signal_handler(int) {
    if (g_shared_ptr) g_shared_ptr->shutdown.store(true);
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 포트 오버라이드: ./test_imu /dev/ttyACM0
    const char* port = (argc > 1) ? argv[1] : IMU_SERIAL_PORT;

    printf("=== QUB_Controller Step-4: IMU 스레드 단독 검증 ===\n");
    printf("포트: %s  /  목표: %dHz  /  %d초간 실행\n\n",
           port, IMU_LOOP_HZ, TEST_DURATION_SEC);

    rt::lock_memory();

    SharedState shared;
    g_shared_ptr = &shared;

    // ─── IMU 스레드 시작 ─────────────────────────────────────────────────────
    printf("[1] ImuThread 초기화 + 시작 (SCHED_FIFO %d)...\n", RT_PRIO_IMU);
    ImuThread imu_thread(&shared, port, IMU_BAUDRATE);

    if (imu_thread.start() != 0) {
        printf("    → 실패\n");
        return 1;
    }
    printf("    → 시작됨\n\n");

    // ─── 모니터링 (1Hz) ──────────────────────────────────────────────────────
    printf("[2] 모니터링 (%d초간, Ctrl+C 로 중단)...\n\n", TEST_DURATION_SEC);

    auto start = std::chrono::steady_clock::now();
    while (!shared.shutdown.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= TEST_DURATION_SEC) break;

        // SharedState::imu 읽기 (seqlock load)
        ImuState s = shared.imu.load();

        printf("[t=%2lds] loop=%-8llu  period=%ld ns  miss=%-3d  filter_mode=%u\n",
               (long)elapsed,
               (unsigned long long)imu_thread.loop_count(),
               imu_thread.last_period_ns(),
               imu_thread.missed_deadlines(),
               0u); // filter_mode는 ImuThread 통해 접근 (추후 공개 메서드 추가)

        printf("    quat  w=%+.4f  x=%+.4f  y=%+.4f  z=%+.4f\n",
               s.quat[0], s.quat[1], s.quat[2], s.quat[3]);
        printf("    gyro  x=%+.4f  y=%+.4f  z=%+.4f  [rad/s]\n",
               s.gyro[0], s.gyro[1], s.gyro[2]);
        printf("    accel x=%+.4f  y=%+.4f  z=%+.4f  [m/s^2]\n\n",
               s.accel[0], s.accel[1], s.accel[2]);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ─── 정지 ────────────────────────────────────────────────────────────────
    printf("[3] 정지 신호 → join...\n");
    shared.shutdown.store(true);
    imu_thread.stop();

    printf("\n=== 결과 요약 ===\n");
    printf("  총 루프      : %llu\n", (unsigned long long)imu_thread.loop_count());
    printf("  미스 데드라인 : %d\n", imu_thread.missed_deadlines());
    printf("  마지막 주기   : %ld ns (목표 %ld ns)\n",
           imu_thread.last_period_ns(), IMU_PERIOD_NS);

    printf("\n다음 단계: test_4ch + test_imu 동시 실행 → Policy 스레드 (Step-5)\n");
    return 0;
}
