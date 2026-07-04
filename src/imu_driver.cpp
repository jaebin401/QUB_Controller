// =============================================================================
// imu_driver.cpp
//
// 참고 출처:
//   [A] mip_sdk/examples/cpp/7_series/ahrs/7_series_ahrs_example.cpp
//       → ping_and_idle(), configure_filter_stream(), enable_ahrs_filter()
//   [B] mip_sdk/examples/cpp/7_series/threading/7_series_threading_example.cpp
//       → update(0) non-blocking 패턴, 스레드 분리
//   [C] motor_thread.cpp (QUB_Controller)
//       → RT 루프 구조, SCHED_FIFO 설정, clock_nanosleep 타이밍
// =============================================================================

#include "imu_driver.hpp"

// mip_sdk 명령 헤더
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_filter.hpp>

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

namespace qub {

// =============================================================================
// ImuDriver — 생성 / 소멸
// =============================================================================

ImuDriver::ImuDriver(SharedState* shared,
                     const std::string& port,
                     uint32_t baudrate)
    : shared_(shared), port_(port), baudrate_(baudrate)
{}

ImuDriver::~ImuDriver()
{
    disconnect();
}

// =============================================================================
// configure()
//
// 7_series_ahrs_example.cpp 의 main() 초기화 시퀀스를 그대로 따른다.
// 순서:
//   1. SerialConnection 열기
//   2. mip::Interface 생성
//   3. ping → idle (통신 확인 + 설정 중 데이터 트래픽 차단)
//   4. defaultDeviceSettings (알려진 상태로 초기화)
//   5. 필터 스트림 포맷 설정 (quat, gyro, accel, status)
//   6. AHRS 필터 활성화 (자력계 aiding 활성 + filter reset)
//   7. registerExtractor 콜백 등록
//   8. resume (데이터 스트림 시작)
// =============================================================================

bool ImuDriver::configure()
{
    // ─── 1. SerialConnection 열기 ────────────────────────────────────────────
    // [A] main() → SerialConnection connection(PORT_NAME, BAUDRATE)
    conn_ = std::make_unique<microstrain::connections::SerialConnection>(
        port_.c_str(), baudrate_);

    if (!conn_->connect()) {
        fprintf(stderr, "[imu_driver] 시리얼 포트 열기 실패: %s\n", port_.c_str());
        return false;
    }
    fprintf(stderr, "[imu_driver] 연결됨: %s @ %u baud\n", port_.c_str(), baudrate_);

    // ─── 2. mip::Interface 생성 ──────────────────────────────────────────────
    // [A] mip::Interface device(&connection, timeout, replyTimeout)
    // mip_timeout_from_baudrate: 보드레이트에 맞는 parse timeout 자동 계산
    device_ = std::make_unique<mip::Interface>(
        conn_.get(),
        mip::C::mip_timeout_from_baudrate(baudrate_),
        IMU_REPLY_TIMEOUT_MS
    );

    // ─── 3. ping → idle ─────────────────────────────────────────────────────
    if (!ping_and_idle())   return false;

    // ─── 4. 기본 설정 로드 ──────────────────────────────────────────────────
    if (!load_default_settings()) return false;

    // ─── 5. 필터 스트림 포맷 설정 ────────────────────────────────────────────
    if (!configure_filter_stream()) return false;

    // ─── 6. AHRS 필터 활성화 ─────────────────────────────────────────────────
    if (!enable_ahrs_filter()) return false;

    // ─── 7. 콜백 등록 ────────────────────────────────────────────────────────
    // [A] device.registerExtractor(handler, &data, DESCRIPTOR_SET)
    // SDK가 update() 호출 시 해당 데이터 타입의 패킷을 받으면 자동으로 구조체를 채워줌
    device_->registerExtractor(handler_quat_,   &quat_,   mip::data_filter::DESCRIPTOR_SET);
    device_->registerExtractor(handler_gyro_,   &gyro_,   mip::data_filter::DESCRIPTOR_SET);
    device_->registerExtractor(handler_accel_,  &accel_,  mip::data_filter::DESCRIPTOR_SET);
    device_->registerExtractor(handler_status_, &status_, mip::data_filter::DESCRIPTOR_SET);

    // ─── 8. 데이터 스트림 시작 ───────────────────────────────────────────────
    if (!start_stream()) return false;

    connected_ = true;
    fprintf(stderr, "[imu_driver] 초기화 완료. 필터 AHRS 모드 대기 중...\n");
    return true;
}

// =============================================================================
// disconnect()
// =============================================================================

void ImuDriver::disconnect()
{
    if (!connected_) return;

    if (device_) {
        // 데이터 스트림 정지 (idle 상태로)
        mip::commands_base::setIdle(*device_);
    }
    if (conn_ && conn_->isConnected()) {
        conn_->disconnect();
    }
    connected_ = false;
    fprintf(stderr, "[imu_driver] 연결 해제\n");
}

// =============================================================================
// poll_and_publish()
//
// IMU 스레드에서 매 주기(200Hz = 5ms) 호출.
//
// [B] threading_example.cpp 의 dataCollectionThread() 에서:
//     device.update(10, false) → 우리는 clock_nanosleep으로 타이밍을 잡으므로
//     update(0) non-blocking 으로 변경. 0ms = "버퍼에 있는 것만 처리하고 즉시 리턴"
//
// update(0) 호출 → SDK가 시리얼 버퍼 읽기 → MIP 패킷 파싱 →
//   새 데이터 있으면 registerExtractor 콜백 트리거 →
//   quat_/gyro_/accel_/status_ 에 자동으로 채워짐
// =============================================================================

bool ImuDriver::poll_and_publish()
{
    if (!device_ || !connected_) return false;

    // [B] device.update(0, false) — non-blocking
    bool ok = device_->update(0, false);

    if (!ok) {
        fprintf(stderr, "[imu_driver] update() 실패 - 통신 오류\n");
        return false;
    }

    update_count_++;
    filter_mode_ = static_cast<uint32_t>(status_.filter_state);

    // 필터가 AHRS 모드(=3) 이상일 때만 유효한 데이터로 게시
    // FilterMode::AHRS = 3 (data_filter.hpp 참고)
    const bool filter_valid =
        (status_.filter_state >= mip::data_filter::FilterMode::AHRS);

    // SharedState::imu 슬롯에 게시 (seqlock write)
    // ImuState는 shared_state.hpp에 정의됨
    ImuState s;

    if (filter_valid && (quat_.valid_flags != 0)) {
        // AttitudeQuaternion.q 는 Quatf 타입 = float[4] 순서: [w, x, y, z]
        s.quat[0] = quat_.q[0];  // w
        s.quat[1] = quat_.q[1];  // x
        s.quat[2] = quat_.q[2];  // y
        s.quat[3] = quat_.q[3];  // z
    } else {
        // 유효하지 않으면 단위 쿼터니언 유지
        s.quat = {1.f, 0.f, 0.f, 0.f};
    }

    if (filter_valid && (gyro_.valid_flags != 0)) {
        // CompensatedAngularRate.gyro = Vector3f [rad/s] base frame
        s.gyro[0] = gyro_.gyro[0];
        s.gyro[1] = gyro_.gyro[1];
        s.gyro[2] = gyro_.gyro[2];
    }

    if (filter_valid && (accel_.valid_flags != 0)) {
        // CompensatedAcceleration.accel = Vector3f [m/s^2] base frame
        s.accel[0] = accel_.accel[0];
        s.accel[1] = accel_.accel[1];
        s.accel[2] = accel_.accel[2];
    }

    s.timestamp_ns = rt::now_ns();
    shared_->imu.store(s);

    return true;
}

// =============================================================================
// 초기화 헬퍼 구현
// =============================================================================

// [A] initializeDevice() 참고: ping → idle → getDeviceInfo → defaultSettings
bool ImuDriver::ping_and_idle()
{
    mip::CmdResult r = mip::commands_base::ping(*device_);
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] ping 실패 (%s)\n", r.name());
        return false;
    }

    r = mip::commands_base::setIdle(*device_);
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] setIdle 실패 (%s)\n", r.name());
        return false;
    }

    fprintf(stderr, "[imu_driver] ping OK, idle 진입\n");
    return true;
}

bool ImuDriver::load_default_settings()
{
    // [A] commands_3dm::defaultDeviceSettings(device)
    // 알려진 상태에서 시작하기 위해 기본 설정 로드.
    // 주의: 이 명령은 baudrate를 115200으로 재설정할 수 있음.
    mip::CmdResult r = mip::commands_3dm::defaultDeviceSettings(*device_);
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] defaultDeviceSettings 실패 (%s)\n", r.name());
        return false;
    }
    fprintf(stderr, "[imu_driver] 기본 설정 로드 완료\n");
    return true;
}

bool ImuDriver::configure_filter_stream()
{
    // ─ base rate 조회 ────────────────────────────────────────────────────────
    // [A] configureFilterMessageFormat() 참고
    // CV7-AHRS 필터 base rate = 500Hz (데이터시트 기준)
    uint16_t base_rate = 0;
    mip::CmdResult r = mip::commands_3dm::getBaseRate(
        *device_,
        mip::data_filter::DESCRIPTOR_SET,
        &base_rate
    );
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] getBaseRate 실패 (%s)\n", r.name());
        return false;
    }
    fprintf(stderr, "[imu_driver] 필터 base rate: %d Hz\n", base_rate);

    // ─ decimation 계산 ───────────────────────────────────────────────────────
    // IMU_STREAM_HZ = 250Hz → decimation = base_rate / 250
    if (base_rate == 0 || base_rate < IMU_STREAM_HZ) {
        fprintf(stderr, "[imu_driver] base rate(%d)가 목표 스트림 주기(%d)보다 낮음\n",
                base_rate, IMU_STREAM_HZ);
        return false;
    }
    uint16_t decimation = base_rate / IMU_STREAM_HZ;
    fprintf(stderr, "[imu_driver] decimation = %d (%d Hz 스트림)\n",
            decimation, base_rate / decimation);

    // ─ 메시지 포맷 설정 ──────────────────────────────────────────────────────
    // 받을 데이터 타입 4종 지정:
    //   DATA_ATT_QUATERNION          (0x03) — 쿼터니언 자세
    //   DATA_COMPENSATED_ANGULAR_RATE (0x0E) — 보정된 각속도 (자이로)
    //   DATA_COMPENSATED_ACCELERATION (0x1C) — 보정된 선형 가속도
    //   DATA_FILTER_STATUS            (0x10) — 필터 상태 (AHRS 여부 판단용)
    const mip::DescriptorRate descriptors[] = {
        {mip::data_filter::AttitudeQuaternion::FIELD_DESCRIPTOR, decimation},
        {mip::data_filter::CompAngularRate::FIELD_DESCRIPTOR,    decimation},
        {mip::data_filter::CompAccel::FIELD_DESCRIPTOR,          decimation},
        {mip::data_filter::Status::FIELD_DESCRIPTOR,             decimation},
    };
    constexpr int NUM_DESC = sizeof(descriptors) / sizeof(descriptors[0]);

    r = mip::commands_3dm::writeMessageFormat(
        *device_,
        mip::data_filter::DESCRIPTOR_SET,
        NUM_DESC,
        descriptors
    );
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] writeMessageFormat 실패 (%s)\n", r.name());
        return false;
    }
    fprintf(stderr, "[imu_driver] 필터 메시지 포맷 설정 완료 (quat+gyro+accel+status @ %dHz)\n",
            base_rate / decimation);
    return true;
}

bool ImuDriver::enable_ahrs_filter()
{
    // [A] initializeFilter() 참고
    // 자력계 aiding 활성화 → AHRS 모드 진입 조건
    mip::CmdResult r = mip::commands_filter::writeAidingMeasurementEnable(
        *device_,
        mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER,
        true
    );
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] magnetometer aiding 활성화 실패 (%s)\n", r.name());
        return false;
    }

    // 필터 리셋 (새 설정 적용)
    r = mip::commands_filter::reset(*device_);
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] filter reset 실패 (%s)\n", r.name());
        return false;
    }

    fprintf(stderr, "[imu_driver] AHRS 필터 활성화 완료\n");
    return true;
}

bool ImuDriver::start_stream()
{
    // [A] resume(device) — idle 상태에서 데이터 스트림 재개
    mip::CmdResult r = mip::commands_base::resume(*device_);
    if (!r.isAck()) {
        fprintf(stderr, "[imu_driver] resume 실패 (%s)\n", r.name());
        return false;
    }
    fprintf(stderr, "[imu_driver] 스트림 시작\n");
    return true;
}

// =============================================================================
// ImuThread — 생성 / 소멸
// =============================================================================

ImuThread::ImuThread(SharedState* shared,
                     const std::string& port,
                     uint32_t baudrate)
    : driver_(shared, port, baudrate)
{}

ImuThread::~ImuThread()
{
    if (started_) stop();
}

// =============================================================================
// start()
//
// [C] motor_thread.cpp::start() 구조를 그대로 따름
// =============================================================================

int ImuThread::start()
{
    if (started_) return 0;

    // 먼저 드라이버 초기화 (시리얼 연결 + 필터 설정)
    if (!driver_.configure()) {
        fprintf(stderr, "[imu_thread] ImuDriver::configure() 실패\n");
        return -1;
    }

    int err = pthread_create(&thread_, nullptr, &ImuThread::thread_entry, this);
    if (err != 0) {
        fprintf(stderr, "[imu_thread] pthread_create 실패: %s\n", strerror(err));
        return err;
    }

    // SCHED_FIFO 80 — 모터 스레드(90)보다 낮고 policy(70)보다 높음
    rt::set_thread_priority(thread_, RT_PRIO_IMU);

    started_ = true;
    return 0;
}

void ImuThread::stop()
{
    if (!started_) return;
    // shutdown 플래그는 외부에서 shared->shutdown = true 로 세팅
    pthread_join(thread_, nullptr);
    driver_.disconnect();
    started_ = false;
}

// =============================================================================
// 스레드 진입점 / 메인 루프
//
// [C] motor_thread.cpp::run() 구조 동일
// 타이밍: clock_nanosleep(TIMER_ABSTIME) drift-free 200Hz 루프
// =============================================================================

void* ImuThread::thread_entry(void* arg)
{
    static_cast<ImuThread*>(arg)->run();
    return nullptr;
}

void ImuThread::run()
{
    // SharedState 포인터는 driver_ 를 통해 접근
    // shutdown 플래그는 driver_.shared_->shutdown 으로 직접 접근 불가
    // → driver_.poll_and_publish() 가 false 리턴하면 종료
    //   또는 외부에서 shared->shutdown 세팅 후 join

    // RT 루프 타이밍 설정 (IMU_PERIOD_NS = 5ms)
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    struct timespec prev_wake = next;

    // 필터 AHRS 모드 진입 대기 (최대 30초)
    // [A] 예제에서 AHRS 모드 대기 후 데이터 수집 시작
    fprintf(stderr, "[imu_thread] 필터 AHRS 모드 대기 중 (최대 30초)...\n");
    auto wait_start = std::chrono::steady_clock::now();
    while (driver_.filter_mode() < static_cast<uint32_t>(mip::data_filter::FilterMode::AHRS)) {
        driver_.poll_and_publish();
        auto elapsed = std::chrono::steady_clock::now() - wait_start;
        if (elapsed > std::chrono::seconds(30)) {
            fprintf(stderr, "[imu_thread] 경고: AHRS 모드 진입 타임아웃. 그냥 진행.\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    fprintf(stderr, "[imu_thread] 필터 모드: %u. 200Hz 루프 시작\n",
            driver_.filter_mode());

    // 메인 200Hz RT 루프
    // shutdown 플래그 접근을 위해 driver_ 내 shared_ 를 직접 읽기엔
    // friend 선언이 필요하므로, 루프 종료는 poll_and_publish() 실패로만 처리
    while (true) {
        bool ok = driver_.poll_and_publish();
        if (!ok) {
            fprintf(stderr, "[imu_thread] poll_and_publish 실패. 스레드 종료\n");
            break;
        }

        // 타이밍 통계
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        last_period_ns_ = rt::timespec_diff_ns(now_ts, prev_wake);
        prev_wake = now_ts;

        if (last_period_ns_ > IMU_PERIOD_NS * 3 / 2)
            missed_deadlines_++;

        loop_count_++;

        // 다음 주기까지 정밀 슬립 (drift-free)
        rt::sleep_until_next_period(next, IMU_PERIOD_NS);
    }
}

} // namespace qub
