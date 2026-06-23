#pragma once
// =============================================================================
// imu_driver.hpp
//
// MicroStrain 3DM-CV7-AHRS 드라이버 래퍼.
// mip_sdk (LORD-MicroStrain/mip_sdk) 위에서 동작한다.
//
// 참고 출처:
//   - mip_sdk/examples/cpp/7_series/ahrs/7_series_ahrs_example.cpp
//     → 초기화 시퀀스, 필터 설정, registerExtractor 패턴
//   - mip_sdk/examples/cpp/7_series/threading/7_series_threading_example.cpp
//     → update(0) non-blocking 패턴, 스레드 분리 구조
//   - mip_sdk/src/cpp/mip/mip_interface.hpp
//     → Interface::update(Timeout, bool from_cmd) 시그니처
//   - mip_sdk/src/cpp/mip/definitions/data_filter.hpp
//     → AttitudeQuaternion, CompensatedAngularRate, CompensatedAcceleration 구조체
//
// 설계 원칙:
//   - motor_thread 와 같은 패턴: 생성 → configure() → start() → (루프) → stop()
//   - IMU 스레드(SCHED_FIFO 80, 200Hz)에서 매 주기 poll_and_publish() 호출
//   - SharedState::imu 슬롯에만 write (단일 writer 보장)
//   - mip_sdk가 내부 스레드를 쓰지 않도록 update(0) 직접 호출
// =============================================================================

#include "shared_state.hpp"
#include "rt_utils.hpp"

// mip_sdk C++ API
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>
#include <microstrain/connections/serial/serial_connection.hpp>

#include <pthread.h>
#include <string>
#include <memory>

namespace qub {

// =============================================================================
// IMU 설정 상수
// =============================================================================

// CV7-AHRS USB-시리얼 포트 (NUC Ubuntu 기본값, udev rule로 고정 권장)
constexpr const char* IMU_SERIAL_PORT    = "/dev/ttyUSB0";
constexpr uint32_t    IMU_BAUDRATE       = 115200;

// CV7-AHRS 필터 base rate = 500Hz (데이터시트 기준)
// 200Hz로 받고 싶으면 decimation = 500/200 = 2.5 → 정수 아님
// 250Hz (decimation=2) 또는 100Hz (decimation=5) 중 선택
// → 250Hz 설정 후 IMU 스레드에서 200Hz로 읽어도 충분 (데이터 손실 없음)
constexpr uint16_t    IMU_FILTER_BASE_RATE_HZ = 500;
constexpr uint16_t    IMU_STREAM_HZ           = 250;   // 실제 CV7 스트림 주기
constexpr uint16_t    IMU_DECIMATION          = IMU_FILTER_BASE_RATE_HZ / IMU_STREAM_HZ;  // = 2

// mip_sdk 타임아웃 (ms)
constexpr int         IMU_CMD_TIMEOUT_MS      = 500;
constexpr int         IMU_REPLY_TIMEOUT_MS    = 2000;

// =============================================================================
// ImuDriver 클래스
//
// CV7-AHRS 하나를 표현하는 드라이버.
// IMU 스레드(imu_thread.cpp에서 생성)가 이 클래스를 소유하고,
// 매 루프 주기에 poll_and_publish()를 호출한다.
// =============================================================================

class ImuDriver {
public:
    // -------------------------------------------------------------------------
    // 생성 / 소멸
    //
    // shared      : 공유 상태 컨테이너 포인터 (소유 안 함)
    // port        : 시리얼 포트 (기본값 IMU_SERIAL_PORT)
    // baudrate    : 보드레이트 (기본값 IMU_BAUDRATE)
    // -------------------------------------------------------------------------
    explicit ImuDriver(SharedState* shared,
                       const std::string& port     = IMU_SERIAL_PORT,
                       uint32_t           baudrate = IMU_BAUDRATE);

    ~ImuDriver();

    // 복사/이동 금지
    ImuDriver(const ImuDriver&)            = delete;
    ImuDriver& operator=(const ImuDriver&) = delete;

    // -------------------------------------------------------------------------
    // 초기화 / 해제
    //
    // configure() : 시리얼 연결 → ping → idle → 필터 설정 → stream 시작
    //               성공 시 true. RT 루프 시작 전 한 번 호출.
    // disconnect() : 스트림 중지 → 시리얼 연결 해제
    // -------------------------------------------------------------------------
    bool configure();
    void disconnect();

    // -------------------------------------------------------------------------
    // RT 루프에서 매 주기 호출 (200Hz)
    //
    // 동작:
    //   1) device_.update(0) — non-blocking, 시리얼 버퍼에 있는 것만 처리
    //                          새 데이터 있으면 내부 콜백으로 quat_/gyro_/accel_ 갱신
    //   2) 갱신된 값을 SharedState::imu 에 store()
    //
    // 반환값: true = 정상, false = 통신 끊김 (스레드 종료 트리거)
    // -------------------------------------------------------------------------
    bool poll_and_publish();

    // -------------------------------------------------------------------------
    // 상태 조회
    // -------------------------------------------------------------------------
    bool     is_connected()    const { return connected_; }
    uint64_t update_count()    const { return update_count_; }
    uint32_t filter_mode()     const { return filter_mode_; }

private:
    // -------------------------------------------------------------------------
    // 초기화 헬퍼
    // -------------------------------------------------------------------------
    bool ping_and_idle();
    bool load_default_settings();
    bool configure_filter_stream();
    bool enable_ahrs_filter();
    bool start_stream();

    // -------------------------------------------------------------------------
    // mip_sdk 콜백 데이터 저장소
    //
    // registerExtractor() 로 등록하면 update() 호출 시 SDK가 자동으로 채워줌.
    // RT 루프에서 읽기만 하면 됨.
    // 참고: mip/definitions/data_filter.hpp의 구조체들
    // -------------------------------------------------------------------------
    mip::data_filter::AttitudeQuaternion     quat_{};    // q[0..3] = (w,x,y,z)
    mip::data_filter::CompensatedAngularRate gyro_{};    // x,y,z [rad/s]
    mip::data_filter::CompensatedAcceleration accel_{};  // x,y,z [m/s^2]
    mip::data_filter::Status                 status_{};  // filter_state (AHRS 여부 판단)

    // DispatchHandler: registerExtractor() 의 결과물.
    // 각 데이터 타입마다 하나씩 필요. (mip_interface.hpp 참고)
    mip::DispatchHandler handler_quat_;
    mip::DispatchHandler handler_gyro_;
    mip::DispatchHandler handler_accel_;
    mip::DispatchHandler handler_status_;

    // -------------------------------------------------------------------------
    // mip_sdk 객체
    //
    // SerialConnection : 시리얼 포트를 열고 바이트를 주고받음 (serial_connection.hpp)
    // mip::Interface   : 바이트 스트림 → MIP 패킷 파싱 → 콜백 트리거 (mip_interface.hpp)
    // -------------------------------------------------------------------------
    std::unique_ptr<microstrain::connections::SerialConnection> conn_;
    std::unique_ptr<mip::Interface>                             device_;

    // -------------------------------------------------------------------------
    // 멤버
    // -------------------------------------------------------------------------
    SharedState* shared_;
    std::string  port_;
    uint32_t     baudrate_;

    bool     connected_    = false;
    uint64_t update_count_ = 0;
    uint32_t filter_mode_  = 0;
};

// =============================================================================
// IMU 스레드 클래스
//
// ImuDriver를 소유하고 SCHED_FIFO 80, 200Hz 루프를 돌린다.
// motor_thread.hpp 구조와 동일한 패턴.
// =============================================================================

class ImuThread {
public:
    ImuThread(SharedState* shared,
              const std::string& port     = IMU_SERIAL_PORT,
              uint32_t           baudrate = IMU_BAUDRATE);

    ~ImuThread();

    ImuThread(const ImuThread&)            = delete;
    ImuThread& operator=(const ImuThread&) = delete;

    // ImuDriver::configure() 후 pthread 생성 + SCHED_FIFO 설정
    int  start();

    // shared->shutdown 이 세팅된 뒤 join
    void stop();

    // 상태 조회 (디버그용)
    uint64_t loop_count()     const { return loop_count_; }
    long     last_period_ns() const { return last_period_ns_; }
    int      missed_deadlines() const { return missed_deadlines_; }

private:
    static void* thread_entry(void* arg);
    void         run();

    ImuDriver  driver_;
    pthread_t  thread_     = 0;
    bool       started_    = false;

    uint64_t loop_count_       = 0;
    long     last_period_ns_   = 0;
    int      missed_deadlines_ = 0;
};

} // namespace qub
