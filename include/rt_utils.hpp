#pragma once
// =============================================================================
// rt_utils.hpp
//
// 실시간 스레드 관리 유틸리티 함수 모음.
//
// 모든 함수는 PREEMPT-RT 커널이 깔린 리눅스를 가정한다. 일반 커널에서도
// 동작은 하지만 지터(jitter)가 크다. RT 권한이 없으면 함수는 경고만
// 출력하고 계속 진행한다 (개발 편의를 위해).
//
// RT 권한 설정:
//   /etc/security/limits.conf 에 다음 줄 추가 후 재로그인:
//     <user>  -  rtprio  99
//     <user>  -  memlock unlimited
// =============================================================================

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <cerrno>
#include <cstdio>
#include <cstring>

namespace qub::rt {

// =============================================================================
// SCHED_FIFO 우선순위 설정
//
// thread     : pthread_self() 또는 std::thread::native_handle()
// priority   : 1..99 (99가 가장 높음)
// 반환값      : 0 = 성공, errno = 실패 (보통 EPERM = 권한 부족)
// =============================================================================

inline int set_thread_priority(pthread_t thread, int priority)
{
    struct sched_param param;
    param.sched_priority = priority;

    int err = pthread_setschedparam(thread, SCHED_FIFO, &param);
    if (err != 0) {
        // sudo 없이 실행하면 보통 여기로 떨어짐 — 경고만 출력하고 계속
        fprintf(stderr,
                "[rt_utils] pthread_setschedparam(SCHED_FIFO, %d) 실패: %s\n"
                "           RT 권한이 없으면 일반 스케줄로 동작합니다.\n"
                "           /etc/security/limits.conf 확인 권장.\n",
                priority, strerror(err));
    }
    return err;
}

// =============================================================================
// 현재 스레드를 특정 CPU 코어에 핀(pin) — 선택적
//
// RT 환경에서는 모터/IMU/Policy 스레드를 각각 다른 코어에 고정하면
// 캐시 친화성과 인터럽트 격리 효과가 있다. NUC 11 (4코어 8스레드 i7)
// 기준 권장:
//   core 0       : 일반 시스템 / IRQ
//   core 1       : motor threads
//   core 2       : IMU thread
//   core 3       : policy thread
// =============================================================================

inline int pin_to_cpu(int cpu_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    int err = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (err != 0) {
        fprintf(stderr, "[rt_utils] pthread_setaffinity_np(CPU %d) 실패: %s\n",
                cpu_id, strerror(err));
    }
    return err;
}

// =============================================================================
// 페이지 폴트 방지를 위한 메모리 락
//
// RT 루프 중에 swap-in 으로 인한 지연이 발생하지 않도록 프로세스의
// 모든 메모리를 RAM 에 고정. main() 초기에 한 번만 호출.
// =============================================================================

inline int lock_memory()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "[rt_utils] mlockall 실패: %s\n", strerror(errno));
        fprintf(stderr, "           ulimit -l (memlock) 가 충분한지 확인.\n");
        return errno;
    }
    return 0;
}

// =============================================================================
// timespec 산술
// =============================================================================

constexpr long NS_PER_SEC = 1'000'000'000L;

inline void timespec_add_ns(struct timespec& ts, long ns)
{
    ts.tv_nsec += ns;
    while (ts.tv_nsec >= NS_PER_SEC) {
        ts.tv_nsec -= NS_PER_SEC;
        ts.tv_sec  += 1;
    }
}

inline long timespec_diff_ns(const struct timespec& a, const struct timespec& b)
{
    return (a.tv_sec - b.tv_sec) * NS_PER_SEC + (a.tv_nsec - b.tv_nsec);
}

// =============================================================================
// 정밀 주기 슬립 (drift-free)
//
// 사용법:
//   struct timespec next;
//   clock_gettime(CLOCK_MONOTONIC, &next);
//   while (running) {
//       // ── 작업 수행 ──
//       sleep_until_next_period(next, qub::MOTOR_PERIOD_NS);
//   }
//
// clock_nanosleep(TIMER_ABSTIME) 을 쓰기 때문에 누적 드리프트가 없다.
// 작업이 한 주기를 초과하면 다음 주기는 즉시 시작된다 (deadline miss
// 감지를 별도로 하고 싶으면 timespec_diff_ns 로 측정).
// =============================================================================

inline void sleep_until_next_period(struct timespec& next, long period_ns)
{
    timespec_add_ns(next, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
}

// =============================================================================
// 현재 시각을 ns 단위로 (CLOCK_MONOTONIC)
// =============================================================================

inline uint64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * NS_PER_SEC + uint64_t(ts.tv_nsec);
}

} // namespace qub::rt
