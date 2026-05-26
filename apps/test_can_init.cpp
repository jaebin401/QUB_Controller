// =============================================================================
// apps/test_can_init.cpp
//
// Step-1 단독 테스트: 싱글스레드, RT 설정 없음
//
// 목적:
//   - PcanChannel 초기화가 정상적으로 되는지 확인
//   - Robstride enable() → MIT zero command → stop() 사이클 동작 확인
//   - 피드백 프레임 수신 및 파싱 확인
//
// 실행 전 준비:
//   sudo ip link set can0 up type can bitrate 1000000   ← SocketCAN 검증 환경용
//   또는 NUC에서 peak-linux-driver 설치 후 PCAN-M.2 인식 확인
//
// 빌드:
//   cd build && cmake .. && make test_can_init
//   sudo ./test_can_init
//
// 주의: PCAN 드라이버 접근에 root 또는 적절한 udev rule 필요
// =============================================================================

#include "pcan_channel.hpp"
#include "robstride.hpp"

#include <PCANBasic.h>
#include <cstdio>
#include <csignal>
#include <cstring>
#include <thread>
#include <chrono>

// =============================================================================
// 설정 — 테스트 환경에 맞게 수정
// =============================================================================

// 테스트할 CAN 채널 (PCAN-M.2 Ch1 = PCAN_PCIBUS1)
static constexpr TPCANHandle TEST_CHANNEL  = PCAN_PCIBUS1;

// 테스트할 모터 ID (CAN ID)
static constexpr uint8_t     TEST_MOTOR_ID = 1;

// MIT 명령 루프 횟수
static constexpr int         LOOP_COUNT    = 200;

// MIT 명령 주기 (단독 테스트라 50Hz로, 실제 RT 루프는 500Hz)
static constexpr int         LOOP_PERIOD_MS = 20;

// =============================================================================
// 전역 종료 플래그
// =============================================================================
static volatile bool g_running = true;

static void signal_handler(int) { g_running = false; }

// =============================================================================
// main
// =============================================================================
int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    printf("=== QUB_Controller Step-1 테스트: PcanChannel + Robstride ===\n\n");

    // ─── 1. CAN 채널 초기화 ────────────────────────────────────────────────
    printf("[1] CAN 채널 초기화 (PCAN_PCIBUS1, 1Mbps) ...\n");

    PcanChannel* ch = nullptr;
    try {
        ch = new PcanChannel(TEST_CHANNEL, PCAN_BAUD_1M);
        printf("    → 성공\n\n");
    } catch (const std::exception& e) {
        printf("    → 실패: %s\n", e.what());
        printf("\n힌트: peak-linux-driver 가 설치되어 있는지, PCAN-M.2 가 인식되어 있는지 확인.\n");
        return 1;
    }

    // ─── 2. Robstride 인스턴스 생성 ───────────────────────────────────────
    printf("[2] Robstride 모터 인스턴스 생성 (motor_id=%d) ...\n", TEST_MOTOR_ID);
    Robstride motor(ch, TEST_MOTOR_ID);
    printf("    → 성공\n\n");

    // ─── 3. 모터 활성화 ────────────────────────────────────────────────────
    printf("[3] enable() 송신 ...\n");
    if (motor.enable()) {
        printf("    → 전송 성공\n");
    } else {
        printf("    → 전송 실패 (CAN 버스 연결 확인)\n");
    }
    // 모터 활성화 응답 대기 (100ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ─── 4. MIT 영점 유지 커맨드 루프 ──────────────────────────────────────
    printf("\n[4] MIT zero-position 커맨드 %d회 전송 (Kp=10, Kd=0.5) ...\n", LOOP_COUNT);
    printf("    Ctrl+C 로 조기 종료 가능\n\n");

    int loop = 0;
    MotorFeedback fb{};

    while (g_running && loop < LOOP_COUNT) {
        // MIT 커맨드 송신 (목표 위치 0.0rad, 속도 0, Kp=10, Kd=0.5, ff=0)
        bool ok = motor.send_mit_command(0.0f, 0.0f, 10.0f, 0.5f, 0.0f);

        // 수신 큐 비울 때까지 읽기
        TPCANMsg msg{};
        TPCANStatus sts;
        while ((sts = ch->read(msg)) == PCAN_ERROR_OK) {
            if (motor.try_parse_feedback(msg, fb)) {
                // 20번째마다 출력 (터미널 홍수 방지)
                if (loop % 20 == 0) {
                    printf("  [%3d] pos=%.4f rad  vel=%.4f rad/s  tor=%.4f Nm  "
                           "temp=%.1f°C  mode=%d  fault=0x%02X\n",
                           loop,
                           fb.position, fb.velocity, fb.torque,
                           fb.temperature, fb.mode_status, fb.fault_flags);
                }
            }
        }

        // 비정상 CAN 에러 처리
        if (sts != PCAN_ERROR_QRCVEMPTY) {
            printf("  CAN 수신 에러: %s\n",
                   PcanChannel::status_to_string(sts).c_str());
        }

        loop++;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
    }

    // ─── 5. 모터 정지 ──────────────────────────────────────────────────────
    printf("\n[5] stop() 송신 (폴트 클리어 포함) ...\n");
    if (motor.stop(true)) {
        printf("    → 전송 성공\n");
    } else {
        printf("    → 전송 실패\n");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ─── 6. 정리 ───────────────────────────────────────────────────────────
    delete ch;

    printf("\n=== 테스트 완료 ===\n");
    printf("다음 단계: motor_thread (SCHED_FIFO, 500Hz) 추가\n");
    return 0;
}
