// =============================================================================
// pcan_channel.cpp
// =============================================================================

#include "pcan_channel.hpp"
#include <stdexcept>

// =============================================================================
// 생성자 / 소멸자
// =============================================================================

PcanChannel::PcanChannel(TPCANHandle channel, TPCANBaudrate baudrate)
    : handle_(channel), baudrate_(baudrate)
{
    // CAN_Initialize: 세 번째~다섯 번째 인자는 deprecated (0 전달)
    TPCANStatus sts = CAN_Initialize(handle_, baudrate_, 0, 0, 0);
    if (sts != PCAN_ERROR_OK) {
        throw std::runtime_error(
            "PcanChannel: CAN_Initialize 실패 - " + status_to_string(sts));
    }
}

PcanChannel::~PcanChannel()
{
    CAN_Uninitialize(handle_);
}

// =============================================================================
// 수신
// =============================================================================

TPCANStatus PcanChannel::read(TPCANMsg& msg_out, TPCANTimestamp* ts_out) const
{
    // CAN_Read는 내부적으로 libpcanbasic mutex로 보호되어 있어
    // 멀티스레드에서 직접 호출해도 안전하다.
    // 단, 채널 하나를 전담 스레드 하나가 읽는 구조를 권장 (경합 최소화)
    return CAN_Read(handle_, &msg_out, ts_out);
}

// =============================================================================
// 송신
// =============================================================================

TPCANStatus PcanChannel::write_extended(uint32_t ext_id,
                                        const uint8_t* data,
                                        uint8_t len) const
{
    TPCANMsg msg{};
    msg.ID      = ext_id;
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;   // 29-bit 확장 프레임
    msg.LEN     = len;

    for (int i = 0; i < len; ++i)
        msg.DATA[i] = data[i];

    return CAN_Write(handle_, &msg);
}

// =============================================================================
// 유틸
// =============================================================================

TPCANStatus PcanChannel::reset() const
{
    return CAN_Reset(handle_);
}

std::string PcanChannel::status_to_string(TPCANStatus status)
{
    char buf[256] = {};
    // Language=0x09 (영어)
    if (CAN_GetErrorText(status, 0x09, buf) != PCAN_ERROR_OK)
        return "unknown error (code=" + std::to_string(status) + ")";
    return std::string(buf);
}
