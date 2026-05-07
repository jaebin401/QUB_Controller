#ifndef ROBSTRIDE_HPP
#define ROBSTRIDE_HPP

#include <cstdint>
#include <atomic>
#include "motorPCAN.hpp"

// ============================================================
// Robstride RS02 통신 프로토콜 상수
// (매뉴얼 4장 참고)
// ============================================================
namespace robstride {

// 통신 타입 (29-bit ID의 Bit[28:24])
enum CommType : uint8_t {
    COMM_GET_ID         = 0x00,  // 디바이스 ID 조회
    COMM_MIT_CONTROL    = 0x01,  // MIT 모드 제어 명령
    COMM_FEEDBACK       = 0x02,  // 모터 피드백 (응답)
    COMM_ENABLE         = 0x03,  // 모터 활성화
    COMM_STOP           = 0x04,  // 모터 정지
    COMM_SET_ZERO       = 0x06,  // 기계적 영점 설정
    COMM_SET_CAN_ID     = 0x07,  // CAN ID 변경
    COMM_PARAM_READ     = 0x11,  // 파라미터 읽기
    COMM_PARAM_WRITE    = 0x12,  // 파라미터 쓰기
    COMM_FAULT          = 0x15,  // 폴트 피드백
    COMM_SAVE           = 0x16,  // 파라미터 저장
};

// 모터 운영 모드 (파라미터 0x7005에 쓰는 값)
enum RunMode : uint8_t {
    MODE_MIT      = 0,  // MIT (operation control)
    MODE_POSITION = 1,  // 위치 모드 (PP)
    MODE_VELOCITY = 2,  // 속도 모드
    MODE_CURRENT  = 3,  // 전류 모드
    MODE_CSP      = 5,  // 위치 모드 (CSP)
};

// 파라미터 인덱스 (매뉴얼 4.1.14 참고, 자주 쓰는 것만)
enum ParamIdx : uint16_t {
    PARAM_RUN_MODE     = 0x7005,
    PARAM_IQ_REF       = 0x7006,
    PARAM_SPD_REF      = 0x700A,
    PARAM_LIMIT_TORQUE = 0x700B,
    PARAM_LOC_REF      = 0x7016,
    PARAM_LIMIT_SPD    = 0x7017,
    PARAM_LIMIT_CUR    = 0x7018,
    PARAM_MECH_POS     = 0x7019,
    PARAM_MECH_VEL     = 0x701B,
};

// MIT 모드 매핑 범위 (매뉴얼 4.1.2)
constexpr float P_MIN = -12.57f;
constexpr float P_MAX =  12.57f;
constexpr float V_MIN = -44.0f;
constexpr float V_MAX =  44.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float T_MIN = -17.0f;
constexpr float T_MAX =  17.0f;

}  // namespace robstride


// ============================================================
// 모터 피드백 데이터 구조체
// ============================================================
struct MotorFeedback {
    float position;     // [rad]
    float velocity;     // [rad/s]
    float torque;       // [Nm]
    float temperature;  // [°C]
    
    uint8_t mode_status;    // 0:Reset, 1:Cali, 2:Motor mode
    uint8_t fault_flags;    // bit별 폴트 (매뉴얼 4.1.2 bit16~21)
    
    uint64_t timestamp_ns;  // 수신 시각 (CLOCK_MONOTONIC)
};


// ============================================================
// 단일 Robstride 모터를 표현하는 클래스
// - PCAN 채널 위에 얹혀서 동작
// - 한 PCAN 채널에 여러 Robstride 인스턴스가 매달릴 수 있음
//   (각자 다른 motor_id를 가짐)
// ============================================================
class Robstride {
public:
    // 생성자: 어떤 PCAN 채널에 매달릴지, 모터 ID는 몇인지
    // host_id는 호스트(Linux) 측 식별자, 보통 0xFD 사용
    Robstride(MOTOR_PCAN* can_channel, 
              uint8_t motor_id, 
              uint8_t host_id = 0xFD);
    
    ~Robstride() = default;

    // ─────────────────────────────────────────────────
    // 명령 송신 (write-only)
    // ─────────────────────────────────────────────────
    
    // 모터 활성화 (Communication Type 3)
    bool enable();
    
    // 모터 정지 (Communication Type 4)
    // clear_fault=true 면 폴트도 함께 클리어
    bool stop(bool clear_fault = false);
    
    // MIT 제어 명령 송신 (Communication Type 1)
    // 모든 5개 파라미터를 한 번에 보냄
    bool send_mit_command(float position,    // [rad]
                          float velocity,    // [rad/s]
                          float kp,          // [0~500]
                          float kd,          // [0~5]
                          float torque_ff);  // [Nm], feedforward
    
    // 운영 모드 변경 (파라미터 0x7005 쓰기)
    bool set_run_mode(robstride::RunMode mode);
    
    // 기계적 영점 설정 (Communication Type 6)
    bool set_mechanical_zero();
    
    // ─────────────────────────────────────────────────
    // 응답 수신 (read-only)
    // ─────────────────────────────────────────────────
    
    // 채널에서 들어온 한 프레임이 이 모터의 피드백인지 확인하고
    // 맞으면 MotorFeedback에 파싱해 채워넣음
    // 반환값: true=내 피드백이었음, false=다른 모터 것이거나 다른 타입
    bool try_parse_feedback(const TPCANMsg& msg, 
                            MotorFeedback& out);
    
    // ─────────────────────────────────────────────────
    // 상태 조회 (캐시된 마지막 피드백)
    // ─────────────────────────────────────────────────
    
    MotorFeedback get_last_feedback() const { return last_feedback_; }
    uint8_t get_motor_id() const { return motor_id_; }

private:
    // ─────────────────────────────────────────────────
    // 내부 헬퍼
    // ─────────────────────────────────────────────────
    
    // 29-bit 확장 ID 조립 (Bit[28:24]=comm_type, Bit[23:8]=data, Bit[7:0]=motor_id)
    uint32_t build_ext_id(uint8_t comm_type, uint16_t data) const;
    
    // float ↔ uint16 매핑 (MIT 모드용)
    static uint16_t float_to_uint(float x, float x_min, float x_max);
    static float uint_to_float(uint16_t x, float x_min, float x_max);
    
    // 파라미터 쓰기 헬퍼 (Communication Type 18)
    bool write_param_uint8(uint16_t index, uint8_t value);
    bool write_param_float(uint16_t index, float value);
    
    // ─────────────────────────────────────────────────
    // 멤버 변수
    // ─────────────────────────────────────────────────
    
    MOTOR_PCAN* can_;       // CAN 채널 (소유하지 않음, 외부에서 주입)
    uint8_t motor_id_;      // 이 모터의 CAN ID
    uint8_t host_id_;       // 호스트 ID (보통 0xFD)
    
    MotorFeedback last_feedback_{};  // 마지막 피드백 캐시
};

#endif  // ROBSTRIDE_HPP