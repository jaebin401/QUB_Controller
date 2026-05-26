#pragma once
// =============================================================================
// motor_thread.hpp
//
// CAN 채널 1개를 전담하는 RT 모터 스레드.
//
// 책임:
//   - 채널에 속한 모터들의 enable / disable
//   - 500Hz 루프:
//       1) SharedState::action 에서 최신 목표값을 읽어와
//       2) 채널의 각 모터에 MIT 커맨드 송신
//       3) 채널의 수신 큐를 비우면서 피드백을 파싱
//       4) SharedState::motor 에 갱신된 측정값 게시
//
// 인스턴스화 패턴:
//   MotorThread mt(&shared, channel_idx=0, &channel0, motors_on_ch0);
//   mt.start();   // pthread 생성, SCHED_FIFO 90
//   ...
//   mt.stop();    // shared.shutdown=true 후 join
// =============================================================================

#include <pthread.h>
#include <vector>
#include <memory>

#include "pcan_channel.hpp"
#include "robstride.hpp"
#include "shared_state.hpp"
#include "robot_config.hpp"

namespace qub {

class MotorThread {
public:
    // -------------------------------------------------------------------------
    // 생성
    //
    // shared        : 공유 상태 컨테이너 포인터 (소유 안 함)
    // channel_idx   : 0..3 (CAN_CHANNELS 인덱스, 로그용)
    // channel       : 이 스레드가 담당하는 CAN 채널 (소유 안 함)
    // motors        : 이 채널에 매달린 Robstride 인스턴스 + joint_idx 매핑
    //
    // motors 의 각 항목은 (Robstride*, joint_idx) 쌍. Robstride 소유권은
    // 외부 (main) 에 있다.
    // -------------------------------------------------------------------------

    struct MotorBinding {
        Robstride* motor;       // 외부 소유
        int        joint_idx;   // 0..NUM_JOINTS-1
    };

    MotorThread(SharedState* shared,
                int          channel_idx,
                PcanChannel* channel,
                std::vector<MotorBinding> motors);

    ~MotorThread();

    // 복사/이동 금지
    MotorThread(const MotorThread&)            = delete;
    MotorThread& operator=(const MotorThread&) = delete;

    // -------------------------------------------------------------------------
    // 라이프사이클
    // -------------------------------------------------------------------------

    // 스레드 생성 + RT 우선순위 설정 + 모터 enable
    // 성공 시 0, 실패 시 errno
    int start();

    // shared->shutdown=true 가 외부에서 세팅된 뒤 호출.
    // 모터 stop 후 pthread_join.
    void stop();

    // -------------------------------------------------------------------------
    // 상태 조회 (디버그용)
    // -------------------------------------------------------------------------

    int      channel_idx()    const { return channel_idx_; }
    uint64_t loop_count()     const { return loop_count_; }
    long     last_period_ns() const { return last_period_ns_; }
    int      missed_deadlines() const { return missed_deadlines_; }

private:
    // -------------------------------------------------------------------------
    // 메인 루프 (pthread 진입점)
    // -------------------------------------------------------------------------
    static void* thread_entry(void* arg);
    void         run();

    // 한 주기 작업
    void send_commands();
    void drain_feedback();
    void publish_state();

    // -------------------------------------------------------------------------
    // 멤버
    // -------------------------------------------------------------------------

    SharedState*              shared_;
    int                       channel_idx_;
    PcanChannel*              channel_;
    std::vector<MotorBinding> motors_;

    pthread_t                 thread_      = 0;
    bool                      started_     = false;

    // 통계
    uint64_t loop_count_        = 0;
    long     last_period_ns_    = 0;
    int      missed_deadlines_  = 0;

    // 마지막으로 본 action generation (변경 감지용)
    uint32_t last_action_gen_   = 0;

    // 정책이 아직 valid 가 아닐 때 쓸 디폴트 자세 (모두 0 rad = T-pose 가정)
    // 실제로는 init 시 현재 위치를 잡아 hold 하도록 확장 권장
};

} // namespace qub
