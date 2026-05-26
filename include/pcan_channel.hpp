#pragma once
// =============================================================================
// pcan_channel.hpp
//
// PCANBasic SDK를 감싸는 CAN 채널 래퍼 클래스.
// PCANBasic.h 외 다른 의존성을 갖지 않는다.
//
// 설계 원칙:
//   - 채널 하나 = 인스턴스 하나 (PCAN_PCIBUS1 ~ PCAN_PCIBUS4)
//   - Read/Write는 non-blocking (큐가 비면 즉시 리턴)
//   - RT 루프에서 직접 호출해도 안전하도록 동적 할당 없음
//   - 에러 처리: 예외 없이 리턴값 또는 TPCANStatus로 전달
//
// 라이선스: 이 파일은 PEAK-System PCANBasic API(LGPL-2.1)를
//           사용하지만, 이 파일 자체는 QUB 프로젝트 고유 코드.
// =============================================================================

#include <PCANBasic.h>
#include <cstdint>
#include <string>

class PcanChannel {
public:
    // -------------------------------------------------------------------------
    // 생성 / 소멸
    // -------------------------------------------------------------------------

    // channel  : PCAN_PCIBUS1 ~ PCAN_PCIBUS4 (PCANBasic.h 상수)
    // baudrate : PCAN_BAUD_1M (Robstride RS02/03/04 기본값)
    // 초기화 실패 시 std::runtime_error 발생
    PcanChannel(TPCANHandle channel, TPCANBaudrate baudrate = PCAN_BAUD_1M);

    // CAN_Uninitialize 호출
    ~PcanChannel();

    // 복사/이동 금지 (채널 핸들은 유일해야 함)
    PcanChannel(const PcanChannel&)            = delete;
    PcanChannel& operator=(const PcanChannel&) = delete;

    // -------------------------------------------------------------------------
    // 수신 (non-blocking)
    // -------------------------------------------------------------------------

    // 큐에서 프레임 하나를 꺼낸다.
    // 반환값:
    //   PCAN_ERROR_OK       : msg에 수신 데이터가 채워짐
    //   PCAN_ERROR_QRCVEMPTY: 수신 큐가 비어있음 (정상, 루프에서 계속)
    //   그 외               : CAN 버스 에러
    TPCANStatus read(TPCANMsg& msg_out, TPCANTimestamp* ts_out = nullptr) const;

    // -------------------------------------------------------------------------
    // 송신
    // -------------------------------------------------------------------------

    // 29-bit 확장 프레임 전송 (Robstride 프로토콜 전용)
    // ext_id : 29-bit CAN ID (comm_type | data | motor_id 조립된 값)
    // data   : 8바이트 페이로드 포인터
    // 반환값: PCAN_ERROR_OK 이면 성공
    TPCANStatus write_extended(uint32_t ext_id,
                               const uint8_t* data,
                               uint8_t len = 8) const;

    // -------------------------------------------------------------------------
    // 상태 / 유틸
    // -------------------------------------------------------------------------

    TPCANHandle handle() const { return handle_; }

    // CAN 버스 재설정 (버스-오프 복구용)
    TPCANStatus reset() const;

    // TPCANStatus 코드를 사람이 읽을 수 있는 문자열로 변환
    static std::string status_to_string(TPCANStatus status);

private:
    TPCANHandle  handle_;
    TPCANBaudrate baudrate_;
};
