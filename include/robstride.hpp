#pragma once
// =============================================================================
// robstride.hpp
//
// Robstride RS-series 모터 1개를 표현하는 클래스.
// PcanChannel 위에서 동작하며, Robstride CAN 2.0 확장 프레임 프로토콜을
// 인코딩/디코딩한다. (매뉴얼 4장 기준)
//
// 기존 QUB_Controller/src/Robstride.hpp의 구조와 프로토콜 상수를 그대로
// 계승하되, 하드웨어 의존성을 MOTOR_PCAN에서 PcanChannel로 교체.
// =============================================================================

#include "pcan_channel.hpp"
#include <PCANBasic.h>
#include <cstdint>
#include <ctime>

// =============================================================================
// 프로토콜 상수 (매뉴얼 4장)
// =============================================================================
namespace robstride {

// 29-bit ID 의 Bit[28:24] — 통신 타입
enum CommType : uint8_t {
    COMM_GET_ID      = 0x00,  // 디바이스 ID 조회
    COMM_MIT_CONTROL = 0x01,  // MIT 모드 제어 명령
    COMM_FEEDBACK    = 0x02,  // 모터 피드백 (응답)
    COMM_ENABLE      = 0x03,  // 모터 활성화
    COMM_STOP        = 0x04,  // 모터 정지 / 폴트 클리어
    COMM_SET_ZERO    = 0x06,  // 기계적 영점 설정
    COMM_SET_CAN_ID  = 0x07,  // CAN ID 변경
    COMM_PARAM_READ  = 0x11,  // 파라미터 읽기
    COMM_PARAM_WRITE = 0x12,  // 파라미터 쓰기
    COMM_FAULT       = 0x15,  // 폴트 피드백
    COMM_SAVE        = 0x16,  // 파라미터 저장 (Flash)
};

// 운영 모드 (파라미터 PARAM_RUN_MODE 에 쓰는 값)
enum RunMode : uint8_t {
    MODE_MIT      = 0,  // MIT (operation control) ← 주력
    MODE_POSITION = 1,  // 위치 제어 (PP)
    MODE_VELOCITY = 2,  // 속도 제어
    MODE_CURRENT  = 3,  // 전류 제어
    MODE_CSP      = 5,  // 위치 제어 (CSP)
};

// 자주 쓰는 파라미터 인덱스 (매뉴얼 4.1.14)
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

// MIT 모드 물리량 범위 (매뉴얼 4.1.2)
// RS02 기준 — RS03/RS04 는 V_MAX, T_MAX가 다를 수 있음
constexpr float P_MIN  = -12.57f;  // [rad]
constexpr float P_MAX  =  12.57f;
constexpr float V_MIN  = -44.0f;   // [rad/s]
constexpr float V_MAX  =  44.0f;
constexpr float KP_MIN =   0.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN =   0.0f;
constexpr float KD_MAX =   5.0f;
constexpr float T_MIN  = -17.0f;   // [Nm]
constexpr float T_MAX  =  17.0f;

// 호스트 ID (Linux 쪽 식별자, Robstride 관례상 0xFD)
constexpr uint8_t HOST_ID_DEFAULT = 0xFD;

} // namespace robstride


// =============================================================================
// 모터 피드백 구조체
// =============================================================================
struct MotorFeedback {
    float position    = 0.f;  // [rad]
    float velocity    = 0.f;  // [rad/s]
    float torque      = 0.f;  // [Nm]
    float temperature = 0.f;  // [°C]

    uint8_t mode_status = 0;  // 0:Reset  1:Cali  2:Motor
    uint8_t fault_flags = 0;  // 비트별 폴트 (매뉴얼 4.1.2 bit16~21)

    uint64_t timestamp_ns = 0;  // CLOCK_MONOTONIC 기준 수신 시각
};


// =============================================================================
// Robstride 모터 드라이버 클래스
// =============================================================================
class Robstride {
public:
    // -------------------------------------------------------------------------
    // 생성 / 소멸
    // -------------------------------------------------------------------------

    // can_channel : 이 모터가 물려있는 CAN 채널 (소유권 없음)
    // motor_id    : 모터에 설정된 CAN ID (1~253)
    // host_id     : 호스트 식별자 (기본 0xFD)
    Robstride(PcanChannel* can_channel,
              uint8_t motor_id,
              uint8_t host_id = robstride::HOST_ID_DEFAULT);

    ~Robstride() = default;

    // 복사/이동 금지
    Robstride(const Robstride&)            = delete;
    Robstride& operator=(const Robstride&) = delete;

    // -------------------------------------------------------------------------
    // 명령 송신
    // -------------------------------------------------------------------------

    // 모터 활성화 (Comm Type 3)
    bool enable();

    // 모터 정지 (Comm Type 4)
    // clear_fault=true 면 폴트도 함께 클리어
    bool stop(bool clear_fault = false);

    // MIT 제어 명령 (Comm Type 1)
    // RT 루프에서 매 주기 호출하는 핵심 함수
    bool send_mit_command(float position,    // [rad]
                          float velocity,    // [rad/s]
                          float kp,
                          float kd,
                          float torque_ff);  // [Nm] feedforward

    // 운영 모드 설정 (파라미터 쓰기)
    bool set_run_mode(robstride::RunMode mode);

    // 기계적 영점 설정 (Comm Type 6)
    bool set_mechanical_zero();

    // -------------------------------------------------------------------------
    // 피드백 수신
    // -------------------------------------------------------------------------

    // 채널에서 읽어온 CAN 프레임이 이 모터의 피드백인지 확인 후 파싱.
    // Motor thread 의 수신 루프에서 아래 패턴으로 사용한다:
    //
    //   TPCANMsg msg;
    //   while (ch->read(msg) == PCAN_ERROR_OK) {
    //       for (auto& motor : motors_on_this_channel)
    //           motor->try_parse_feedback(msg, ...);
    //   }
    //
    // 반환값: true = 이 모터의 피드백을 파싱함, false = 관계없는 프레임
    bool try_parse_feedback(const TPCANMsg& msg, MotorFeedback& out);

    // -------------------------------------------------------------------------
    // 상태 조회
    // -------------------------------------------------------------------------

    // 마지막으로 파싱한 피드백 (파싱된 적 없으면 기본값)
    const MotorFeedback& last_feedback() const { return last_feedback_; }

    uint8_t motor_id() const { return motor_id_; }

private:
    // -------------------------------------------------------------------------
    // 내부 헬퍼
    // -------------------------------------------------------------------------

    // 29-bit 확장 ID 조립
    // layout: [28:24]=comm_type  [23:8]=data_field  [7:0]=motor_id
    uint32_t build_ext_id(uint8_t comm_type, uint16_t data_field) const;

    // float ↔ uint16 선형 매핑 (MIT 모드 인코딩)
    static uint16_t float_to_uint(float x, float x_min, float x_max);
    static float    uint_to_float(uint16_t x, float x_min, float x_max);

    // 파라미터 쓰기 내부 헬퍼 (Comm Type 0x12)
    bool write_param_u8   (uint16_t index, uint8_t  value);
    bool write_param_float(uint16_t index, float    value);

    // -------------------------------------------------------------------------
    // 멤버 변수
    // -------------------------------------------------------------------------

    PcanChannel* can_;      // CAN 채널 포인터 (소유 안 함)
    uint8_t motor_id_;      // 이 모터의 CAN ID
    uint8_t host_id_;       // 호스트 ID

    MotorFeedback last_feedback_{};  // 마지막 피드백 캐시
};
