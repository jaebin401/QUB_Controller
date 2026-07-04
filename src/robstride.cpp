// =============================================================================
// robstride.cpp
//
// Robstride RS-series 모터 CAN 프로토콜 구현.
// 프로토콜 로직은 QUB_Controller/src/Robstride.cpp 를 기반으로 하되,
// 하드웨어 레이어를 PcanChannel 로 교체.
// =============================================================================

#include "robstride.hpp"

#include <cstring>    // memcpy
#include <ctime>      // clock_gettime
#include <stdexcept>

using namespace robstride;

// =============================================================================
// 생성자
// =============================================================================

Robstride::Robstride(PcanChannel* can_channel,
                     uint8_t motor_id,
                     uint8_t host_id)
    : can_(can_channel), motor_id_(motor_id), host_id_(host_id)
{
    if (can_ == nullptr)
        throw std::invalid_argument("Robstride: can_channel cannot be null");
}

// =============================================================================
// 29-bit 확장 CAN ID 조립
//
// 매뉴얼 4장 프레임 구조:
//   Bit[28:24] = 통신 타입 (5 bits)
//   Bit[23:8]  = data_field (16 bits) — MIT 명령에서는 torque_uint, 나머지는 host_id
//   Bit[7:0]   = 대상 모터 ID (8 bits)
// =============================================================================

uint32_t Robstride::build_ext_id(uint8_t comm_type, uint16_t data_field) const
{
    return (uint32_t(comm_type & 0x1F) << 24) |
           (uint32_t(data_field)       <<  8) |
           (uint32_t(motor_id_));
}

// =============================================================================
// float ↔ uint16 선형 매핑 (MIT 모드)
//
// 매뉴얼 4.4 예제와 동일 로직:
//   uint = (x - x_min) / (x_max - x_min) * 65535
// =============================================================================

uint16_t Robstride::float_to_uint(float x, float x_min, float x_max)
{
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return static_cast<uint16_t>((x - x_min) * 65535.0f / (x_max - x_min));
}

float Robstride::uint_to_float(uint16_t x, float x_min, float x_max)
{
    return float(x) * (x_max - x_min) / 65535.0f + x_min;
}

// =============================================================================
// 모터 활성화 (Communication Type 3)
// 매뉴얼 4.1.3: 데이터 필드 전부 0, host_id 를 data_field 자리에
// =============================================================================

bool Robstride::enable()
{
    uint32_t ext_id = build_ext_id(COMM_ENABLE, host_id_);
    uint8_t  data[8] = {0};

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

// =============================================================================
// 모터 정지 (Communication Type 4)
// 매뉴얼 4.1.4: clear_fault=true 면 Byte[0]=1
// =============================================================================

bool Robstride::stop(bool clear_fault)
{
    uint32_t ext_id = build_ext_id(COMM_STOP, host_id_);
    uint8_t  data[8] = {0};

    if (clear_fault)
        data[0] = 1;

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

// =============================================================================
// MIT 제어 명령 (Communication Type 1)
// 매뉴얼 4.1.2
//
// 29-bit ID:
//   Bit[28:24] = 0x01
//   Bit[23:8]  = torque_ff 를 uint16 으로 매핑한 값
//   Bit[7:0]   = motor_id
//
// 8바이트 데이터 (big-endian):
//   Byte[0:1] = position uint16
//   Byte[2:3] = velocity uint16
//   Byte[4:5] = kp uint16
//   Byte[6:7] = kd uint16
// =============================================================================

bool Robstride::send_mit_command(float position, float velocity,
                                 float kp, float kd, float torque_ff)
{
    uint16_t tor_u = float_to_uint(torque_ff, T_MIN, T_MAX);
    uint32_t ext_id = build_ext_id(COMM_MIT_CONTROL, tor_u);

    uint16_t pos_u = float_to_uint(position, P_MIN, P_MAX);
    uint16_t vel_u = float_to_uint(velocity, V_MIN, V_MAX);
    uint16_t kp_u  = float_to_uint(kp,       KP_MIN, KP_MAX);
    uint16_t kd_u  = float_to_uint(kd,       KD_MIN, KD_MAX);

    uint8_t data[8];
    // big-endian 패킹 (high byte first)
    data[0] = uint8_t(pos_u >> 8);   data[1] = uint8_t(pos_u & 0xFF);
    data[2] = uint8_t(vel_u >> 8);   data[3] = uint8_t(vel_u & 0xFF);
    data[4] = uint8_t(kp_u  >> 8);   data[5] = uint8_t(kp_u  & 0xFF);
    data[6] = uint8_t(kd_u  >> 8);   data[7] = uint8_t(kd_u  & 0xFF);

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

// =============================================================================
// 운영 모드 변경 (파라미터 0x7005 쓰기)
// =============================================================================

bool Robstride::set_run_mode(RunMode mode)
{
    return write_param_u8(PARAM_RUN_MODE, static_cast<uint8_t>(mode));
}

// =============================================================================
// 기계적 영점 설정 (Communication Type 6)
// 매뉴얼 4.1.5: Byte[0]=1
// =============================================================================

bool Robstride::set_mechanical_zero()
{
    uint32_t ext_id = build_ext_id(COMM_SET_ZERO, host_id_);
    uint8_t  data[8] = {0};
    data[0] = 1;

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

// =============================================================================
// 파라미터 쓰기 내부 헬퍼 (Communication Type 0x12)
// 매뉴얼 4.1.7
//
// 8바이트 데이터 (little-endian):
//   Byte[0:1] = parameter index
//   Byte[2:3] = 0x00
//   Byte[4:7] = parameter value
// =============================================================================

bool Robstride::write_param_u8(uint16_t index, uint8_t value)
{
    uint32_t ext_id = build_ext_id(COMM_PARAM_WRITE, host_id_);
    uint8_t  data[8] = {0};

    // little-endian
    data[0] = uint8_t(index & 0xFF);
    data[1] = uint8_t(index >> 8);
    data[4] = value;

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

bool Robstride::write_param_float(uint16_t index, float value)
{
    uint32_t ext_id = build_ext_id(COMM_PARAM_WRITE, host_id_);
    uint8_t  data[8] = {0};

    data[0] = uint8_t(index & 0xFF);
    data[1] = uint8_t(index >> 8);

    // IEEE-754 float 를 4바이트 little-endian 으로 복사 (type-punning safe)
    std::memcpy(&data[4], &value, sizeof(float));

    TPCANStatus sts = can_->write_extended(ext_id, data, 8);
    return (sts == PCAN_ERROR_OK);
}

// =============================================================================
// 피드백 파싱 (Communication Type 2)
// 매뉴얼 4.1.2 응답 프레임
//
// 29-bit ID:
//   Bit[28:24] = 0x02 (피드백)
//   Bit[23:22] = mode_status  (2 bits)
//   Bit[21:16] = fault_flags  (6 bits)
//   Bit[15:8]  = 이 피드백을 보낸 모터 ID
//   Bit[7:0]   = host ID
//
// 8바이트 데이터 (big-endian uint16):
//   Byte[0:1] = position
//   Byte[2:3] = velocity
//   Byte[4:5] = torque
//   Byte[6:7] = temperature (* 0.1 = °C)
// =============================================================================

bool Robstride::try_parse_feedback(const TPCANMsg& msg, MotorFeedback& out)
{
    // 확장 프레임인지 확인
    if ((msg.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0) return false;

    // 통신 타입 확인 (Bit[28:24])
    uint8_t comm_type = uint8_t((msg.ID >> 24) & 0x1F);
    if (comm_type != COMM_FEEDBACK) return false;

    // 이 모터의 피드백인지 확인 (Bit[15:8])
    uint8_t src_motor_id = uint8_t((msg.ID >> 8) & 0xFF);
    if (src_motor_id != motor_id_) return false;

    // ───── 이 모터의 피드백 확인됨, 파싱 시작 ─────

    // ID 에서 상태 / 폴트 추출
    out.mode_status = uint8_t((msg.ID >> 22) & 0x03);   // bit23~22
    out.fault_flags = uint8_t((msg.ID >> 16) & 0x3F);   // bit21~16

    // 데이터 필드: big-endian uint16 → float
    uint16_t pos_u  = (uint16_t(msg.DATA[0]) << 8) | msg.DATA[1];
    uint16_t vel_u  = (uint16_t(msg.DATA[2]) << 8) | msg.DATA[3];
    uint16_t tor_u  = (uint16_t(msg.DATA[4]) << 8) | msg.DATA[5];
    uint16_t temp_u = (uint16_t(msg.DATA[6]) << 8) | msg.DATA[7];

    out.position    = uint_to_float(pos_u, P_MIN, P_MAX);
    out.velocity    = uint_to_float(vel_u, V_MIN, V_MAX);
    out.torque      = uint_to_float(tor_u, T_MIN, T_MAX);
    out.temperature = float(temp_u) * 0.1f;  // 매뉴얼: 온도 × 10 인코딩

    // 수신 타임스탬프 (CLOCK_MONOTONIC)
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    out.timestamp_ns =
        uint64_t(ts.tv_sec) * 1'000'000'000ULL + uint64_t(ts.tv_nsec);

    last_feedback_ = out;
    return true;
}
