// =============================================================================
// motor_thread.cpp
// =============================================================================

#include "motor_thread.hpp"
#include "rt_utils.hpp"

#include <cstdio>
#include <cstring>
#include <climits>

namespace qub {

// =============================================================================
// 생성 / 소멸
// =============================================================================

MotorThread::MotorThread(SharedState*              shared,
                         int                       channel_idx,
                         PcanChannel*              channel,
                         std::vector<MotorBinding> motors)
    : shared_(shared)
    , channel_idx_(channel_idx)
    , channel_(channel)
    , motors_(std::move(motors))
{
}

MotorThread::~MotorThread()
{
    if (started_) stop();
}

// =============================================================================
// start / stop
// =============================================================================

int MotorThread::start()
{
    if (started_) return 0;

    // 모터들 enable (개별 실패는 경고만)
    for (auto& bind : motors_) {
        if (!bind.motor->enable()) {
            fprintf(stderr,
                    "[motor_thread ch%d] motor_id=%d enable() 실패\n",
                    channel_idx_, bind.motor->motor_id());
        }
    }

    int err = pthread_create(&thread_, nullptr, &MotorThread::thread_entry, this);
    if (err != 0) {
        fprintf(stderr,
                "[motor_thread ch%d] pthread_create 실패: %s\n",
                channel_idx_, strerror(err));
        return err;
    }

    // RT 우선순위 설정 (권한 없으면 경고만 뜨고 일반 스케줄로 진행)
    rt::set_thread_priority(thread_, RT_PRIO_MOTOR);

    started_ = true;
    return 0;
}

void MotorThread::stop()
{
    if (!started_) return;

    shared_->shutdown.store(true);
    pthread_join(thread_, nullptr);

    // 안전 정지 (한 번은 놓칠 수 있어 두 번)
    for (auto& bind : motors_) {
        bind.motor->stop(false);
        bind.motor->stop(false);
    }

    started_ = false;
}

// =============================================================================
// 스레드 진입점
// =============================================================================

void* MotorThread::thread_entry(void* arg)
{
    static_cast<MotorThread*>(arg)->run();
    return nullptr;
}

// =============================================================================
// 메인 루프
//
// 주기당 작업:
//   1) send_commands()  - 각 모터에 MIT 커맨드 송신
//   2) drain_feedback() - 채널 수신 큐 비우면서 피드백 파싱
//   3) publish_state()  - SharedState::channel[ch] 슬롯에 게시
//
// 타이밍은 clock_nanosleep(TIMER_ABSTIME) 으로 drift-free.
// =============================================================================

void MotorThread::run()
{
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    struct timespec prev_wake = next;

    while (!shared_->shutdown.load(std::memory_order_relaxed)) {

        send_commands();
        drain_feedback();
        publish_state();

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        last_period_ns_ = rt::timespec_diff_ns(now, prev_wake);
        prev_wake       = now;

        if (last_period_ns_ > MOTOR_PERIOD_NS * 3 / 2)
            missed_deadlines_++;

        loop_count_++;

        rt::sleep_until_next_period(next, MOTOR_PERIOD_NS);
    }
}

// =============================================================================
// 1) 명령 송신
//
// SharedState::action 에서 한 번에 스냅샷을 읽고, 이 채널이 담당하는
// 모터들에 대해서만 MIT 커맨드를 송신.
//
// action.valid==false 면 (정책이 아직 추론을 끝내기 전) Kp=0, Kd=0 으로
// 보내서 안전하게 자유 매달림(free) 상태 유지.
// =============================================================================

void MotorThread::send_commands()
{
    ActionState act = shared_->action.load();

    for (auto& bind : motors_) {
        int j = bind.joint_idx;

        if (!act.valid) {
            bind.motor->send_mit_command(0.f, 0.f, 0.f, 0.f, 0.f);
        } else {
            bind.motor->send_mit_command(
                act.q_des[j],
                act.dq_des[j],
                act.kp[j],
                act.kd[j],
                act.tau_ff[j]);
        }
    }
}

// =============================================================================
// 2) 피드백 수신
//
// 채널 수신 큐를 비울 때까지 read() 반복. 각 프레임을 채널의 모든
// 모터에 대해 try_parse_feedback() 호출 - 일치하는 모터가 알아서 파싱.
// =============================================================================

void MotorThread::drain_feedback()
{
    TPCANMsg      msg;
    TPCANStatus   sts;
    MotorFeedback fb;

    constexpr int MAX_FRAMES_PER_CYCLE = 32;
    int count = 0;

    while ((sts = channel_->read(msg)) == PCAN_ERROR_OK && count < MAX_FRAMES_PER_CYCLE) {
        for (auto& bind : motors_) {
            if (bind.motor->try_parse_feedback(msg, fb))
                break;
        }
        count++;
    }

    if (sts != PCAN_ERROR_OK && sts != PCAN_ERROR_QRCVEMPTY) {
        // 잦으면 별도 로그 큐로 빼야 함 - 지금은 단순 출력
        fprintf(stderr, "[motor_thread ch%d] read 에러: %s\n",
                channel_idx_, PcanChannel::status_to_string(sts).c_str());
    }
}

// =============================================================================
// 3) 상태 게시
//
// 이 스레드는 SharedState::channel[channel_idx_] 슬롯에만 write 한다.
// 다른 채널과 충돌 없음 (각자 자기 슬롯).
//
// ChannelState 는 채널-로컬 인덱스 (0..num_motors-1) 로 채우고,
// joint_idx[i] 에 전체 13-DOF 인덱스를 함께 저장. Policy thread 가 이를
// 풀어서 13-DOF 배열로 합친다.
// =============================================================================

void MotorThread::publish_state()
{
    ChannelState s;
    s.num_motors = static_cast<int>(motors_.size());

    uint64_t now = rt::now_ns();

    for (int i = 0; i < s.num_motors; ++i) {
        auto& bind = motors_[i];
        const MotorFeedback& fb = bind.motor->last_feedback();

        s.joint_idx[i] = bind.joint_idx;
        s.q[i]         = fb.position;
        s.dq[i]        = fb.velocity;
        s.tau_est[i]   = fb.torque;

        // 피드백 timestamp_ns == 0 (한 번도 파싱 안 됨) 이면 오프라인
        uint64_t age = (fb.timestamp_ns == 0)
                       ? ULLONG_MAX
                       : (now - fb.timestamp_ns);
        s.online[i]   = (age < uint64_t(MOTOR_PERIOD_NS) * MOTOR_TIMEOUT_CYCLES);
    }

    s.timestamp_ns = now;
    shared_->channel[channel_idx_].store(s);
}

} // namespace qub