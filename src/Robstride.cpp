#include "Robstride.hpp"
#include <cstring>   // memcpy
#include <ctime>     // clock_gettime
#include <iostream>

using namespace robstride;

// ============================================================
// 생성자
// ============================================================
Robstride::Robstride(MOTOR_PCAN* can_channel, 
                     uint8_t motor_id, 
                     uint8_t host_id)
    : can_(can_channel), motor_id_(motor_id), host_id_(host_id)
{
    if (can_ == nullptr) {
        throw std::invalid_argument("Robstride: can_channel cannot be null");
    }
}

// ============================================================
// 29-bit 확장 ID 조립
// 매뉴얼 4장: Bit[28:24]=comm_type, Bit[23:8]=data, Bit[7:0]=motor_id
// ============================================================
uint32_t Robstride::build_ext_id(uint8_t comm_type, uint16_t data) const {
    return (uint32_t(comm_type & 0x1F) << 24) |
           (uint32_t(data) << 8) |
           (uint32_t(motor_id_));
}

// ============================================================
// float ↔ uint16 매핑 (MIT 모드)
// 매뉴얼 4.4 예시 코드의 float_to_uint와 동일 로직
// ============================================================
uint16_t Robstride::float_to_uint(float x, float x_min, float x_max) {
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    
    float span = x_max - x_min;
    return static_cast<uint16_t>((x - x_min) * 65535.0f / span);
}

float Robstride::uint_to_float(uint16_t x, float x_min, float x_max) {
    float span = x_max - x_min;
    return float(x) * span / 65535.0f + x_min;
}

// ============================================================
// 모터 활성화 (Communication Type 3)
// 매뉴얼 4.1.3
// ============================================================
bool Robstride::enable() {
    uint32_t ext_id = build_ext_id(COMM_ENABLE, host_id_);
    uint8_t data[8] = {0};  // 데이터 필드는 모두 0
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// 모터 정지 (Communication Type 4)
// 매뉴얼 4.1.4
// clear_fault=true 일 때 Byte[0]=1
// ============================================================
bool Robstride::stop(bool clear_fault) {
    uint32_t ext_id = build_ext_id(COMM_STOP, host_id_);
    uint8_t data[8] = {0};
    
    if (clear_fault) {
        data[0] = 1;  // 폴트 클리어 플래그
    }
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// MIT 제어 명령 (Communication Type 1)
// 매뉴얼 4.1.2
// 
// 29-bit ID 구조:
//   Bit[28:24] = 0x01
//   Bit[23:8]  = torque를 uint16으로 매핑한 값
//   Bit[7:0]   = motor_id
// 
// 8바이트 데이터:
//   Byte[0:1]  = position (big-endian, uint16)
//   Byte[2:3]  = velocity (big-endian, uint16)
//   Byte[4:5]  = kp (big-endian, uint16)
//   Byte[6:7]  = kd (big-endian, uint16)
// ============================================================
bool Robstride::send_mit_command(float position, float velocity, 
                                 float kp, float kd, float torque_ff) {
    // torque는 ID에 들어감
    uint16_t torque_u = float_to_uint(torque_ff, T_MIN, T_MAX);
    uint32_t ext_id = build_ext_id(COMM_MIT_CONTROL, torque_u);
    
    // position, velocity, kp, kd는 데이터 필드에
    uint16_t pos_u = float_to_uint(position, P_MIN, P_MAX);
    uint16_t vel_u = float_to_uint(velocity, V_MIN, V_MAX);
    uint16_t kp_u  = float_to_uint(kp,       KP_MIN, KP_MAX);
    uint16_t kd_u  = float_to_uint(kd,       KD_MIN, KD_MAX);
    
    uint8_t data[8];
    // big-endian 패킹 (high byte first)
    data[0] = uint8_t(pos_u >> 8);
    data[1] = uint8_t(pos_u & 0xFF);
    data[2] = uint8_t(vel_u >> 8);
    data[3] = uint8_t(vel_u & 0xFF);
    data[4] = uint8_t(kp_u >> 8);
    data[5] = uint8_t(kp_u & 0xFF);
    data[6] = uint8_t(kd_u >> 8);
    data[7] = uint8_t(kd_u & 0xFF);
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// 운영 모드 변경
// 파라미터 0x7005에 RunMode 값을 쓰는 동작
// ============================================================
bool Robstride::set_run_mode(RunMode mode) {
    return write_param_uint8(PARAM_RUN_MODE, static_cast<uint8_t>(mode));
}

// ============================================================
// 기계적 영점 설정 (Communication Type 6)
// 매뉴얼 4.1.5: Byte[0]=1
// ============================================================
bool Robstride::set_mechanical_zero() {
    uint32_t ext_id = build_ext_id(COMM_SET_ZERO, host_id_);
    uint8_t data[8] = {0};
    data[0] = 1;
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// 파라미터 쓰기 (Communication Type 18) - uint8 버전
// 매뉴얼 4.1.7
// 
// 8바이트 데이터:
//   Byte[0:1] = parameter index (little-endian)
//   Byte[2:3] = 0
//   Byte[4:7] = parameter value (little-endian)
// ============================================================
bool Robstride::write_param_uint8(uint16_t index, uint8_t value) {
    uint32_t ext_id = build_ext_id(COMM_PARAM_WRITE, host_id_);
    
    uint8_t data[8] = {0};
    // little-endian (low byte first)
    data[0] = uint8_t(index & 0xFF);
    data[1] = uint8_t(index >> 8);
    data[4] = value;
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// 파라미터 쓰기 - float 버전
// ============================================================
bool Robstride::write_param_float(uint16_t index, float value) {
    uint32_t ext_id = build_ext_id(COMM_PARAM_WRITE, host_id_);
    
    uint8_t data[8] = {0};
    data[0] = uint8_t(index & 0xFF);
    data[1] = uint8_t(index >> 8);
    
    // float를 4바이트 little-endian으로 복사
    // (memcpy로 type-punning, IEEE-754 32bit 가정)
    std::memcpy(&data[4], &value, sizeof(float));
    
    int result = can_->PCAN_Write(ext_id, 8, MSGTYPE_EXTENDED, data);
    return (result == 1);
}

// ============================================================
// 피드백 파싱
// 매뉴얼 4.1.2 응답 프레임 (Communication Type 2)
// 
// 들어온 프레임이 이 모터의 피드백인지 확인하고, 맞으면 파싱
// 
// 29-bit ID:
//   Bit[28:24] = 0x02 (피드백 타입)
//   Bit[15:8]  = 모터 CAN ID (어느 모터의 피드백인가)
//   Bit[21:16] = fault flags
//   Bit[23:22] = mode status
//   Bit[7:0]   = host CAN ID
// 
// 8바이트 데이터: position, velocity, torque, temp (big-endian uint16)
// ============================================================
bool Robstride::try_parse_feedback(const TPCANMsg& msg, MotorFeedback& out) {
    // 확장 프레임만 처리
    if (msg.MSGTYPE != MSGTYPE_EXTENDED) return false;
    
    // 통신 타입이 0x02 (피드백)인지 확인
    uint8_t comm_type = (msg.ID >> 24) & 0x1F;
    if (comm_type != COMM_FEEDBACK) return false;
    
    // 모터 ID가 내 ID인지 확인
    uint8_t feedback_motor_id = (msg.ID >> 8) & 0xFF;
    if (feedback_motor_id != motor_id_) return false;
    
    // ───── 여기까지 왔으면 내 피드백이 맞음, 파싱 시작 ─────
    
    // ID에서 상태 정보 추출
    out.fault_flags = (msg.ID >> 16) & 0x3F;   // bit21~16 (6 bits)
    out.mode_status = (msg.ID >> 22) & 0x03;   // bit23~22 (2 bits)
    
    // 데이터 필드에서 물리량 추출 (big-endian)
    uint16_t pos_u  = (uint16_t(msg.DATA[0]) << 8) | msg.DATA[1];
    uint16_t vel_u  = (uint16_t(msg.DATA[2]) << 8) | msg.DATA[3];
    uint16_t tor_u  = (uint16_t(msg.DATA[4]) << 8) | msg.DATA[5];
    uint16_t temp_u = (uint16_t(msg.DATA[6]) << 8) | msg.DATA[7];
    
    out.position    = uint_to_float(pos_u, P_MIN, P_MAX);
    out.velocity    = uint_to_float(vel_u, V_MIN, V_MAX);
    out.torque      = uint_to_float(tor_u, T_MIN, T_MAX);
    out.temperature = float(temp_u) * 0.1f;  // 매뉴얼: Temp(Celsius) * 10
    
    // 타임스탬프 기록
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    out.timestamp_ns = uint64_t(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
    
    // 캐시에 저장
    last_feedback_ = out;
    
    return true;
}