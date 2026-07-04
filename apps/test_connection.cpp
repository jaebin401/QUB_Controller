#include "pcan_channel.hpp"
#include "robot_config.hpp"
#include "robstride.hpp"

#include <PCANBasic.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

using namespace qub;

namespace {

constexpr uint8_t PROBE_HOST_ID = robstride::HOST_ID_DEFAULT;
constexpr int PROBE_TIMEOUT_MS = 20;
constexpr int PROBE_RETRIES = 2;
constexpr std::array<uint16_t, 2> PROBE_PARAMS = {
    robstride::PARAM_RUN_MODE,
    robstride::PARAM_MECH_POS,
};

struct ProbeResult {
    bool        channel_ready     = false;
    bool        tx_ok             = false;
    bool        responded         = false;
    int         attempt           = 0;
    const char* stage             = "none";
    TPCANStatus last_status       = PCAN_ERROR_OK;
    TPCANMsg    response_msg      = {};
};

uint32_t build_ext_id(uint8_t comm_type, uint16_t data_field, uint8_t motor_id)
{
    return (uint32_t(comm_type & 0x1F) << 24) |
           (uint32_t(data_field) << 8) |
           uint32_t(motor_id);
}

const char* comm_type_to_string(uint8_t comm_type)
{
    switch (comm_type) {
    case robstride::COMM_GET_ID:      return "GET_ID";
    case robstride::COMM_MIT_CONTROL: return "MIT_CONTROL";
    case robstride::COMM_FEEDBACK:    return "FEEDBACK";
    case robstride::COMM_ENABLE:      return "ENABLE";
    case robstride::COMM_STOP:        return "STOP";
    case robstride::COMM_SET_ZERO:    return "SET_ZERO";
    case robstride::COMM_SET_CAN_ID:  return "SET_CAN_ID";
    case robstride::COMM_PARAM_READ:  return "PARAM_READ";
    case robstride::COMM_PARAM_WRITE: return "PARAM_WRITE";
    case robstride::COMM_FAULT:       return "FAULT";
    case robstride::COMM_SAVE:        return "SAVE";
    default:                          return "UNKNOWN";
    }
}

void print_frame(const TPCANMsg& msg)
{
    std::printf("raw_id=0x%08X data=", msg.ID);
    for (int i = 0; i < msg.LEN; ++i)
        std::printf("%02X", msg.DATA[i]);
}

void drain_rx_queue(PcanChannel& ch)
{
    TPCANMsg msg{};
    while (ch.read(msg) == PCAN_ERROR_OK) {
    }
}

TPCANStatus send_get_id(PcanChannel& ch, uint8_t motor_id)
{
    const uint8_t data[8] = {0};
    return ch.write_extended(
        build_ext_id(robstride::COMM_GET_ID, PROBE_HOST_ID, motor_id), data, 8);
}

TPCANStatus send_param_read(PcanChannel& ch, uint8_t motor_id, uint16_t index)
{
    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(index & 0xFF);
    data[1] = static_cast<uint8_t>(index >> 8);

    return ch.write_extended(
        build_ext_id(robstride::COMM_PARAM_READ, PROBE_HOST_ID, motor_id), data, 8);
}

bool is_reply_for_motor(const TPCANMsg& msg, uint8_t motor_id)
{
    if ((msg.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0)
        return false;

    const uint8_t dst_host = static_cast<uint8_t>(msg.ID & 0xFF);
    const uint8_t src_motor = static_cast<uint8_t>((msg.ID >> 8) & 0xFF);
    return (dst_host == PROBE_HOST_ID) && (src_motor == motor_id);
}

bool wait_for_reply(PcanChannel& ch,
                    uint8_t motor_id,
                    std::chrono::milliseconds timeout,
                    ProbeResult& result)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        TPCANMsg msg{};
        TPCANStatus sts = ch.read(msg);

        while (sts == PCAN_ERROR_OK) {
            if (is_reply_for_motor(msg, motor_id)) {
                result.responded = true;
                result.response_msg = msg;
                result.last_status = PCAN_ERROR_OK;
                return true;
            }
            sts = ch.read(msg);
        }

        if (sts != PCAN_ERROR_QRCVEMPTY) {
            result.last_status = sts;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    return false;
}

ProbeResult probe_motor(PcanChannel& ch, uint8_t motor_id)
{
    ProbeResult result;
    result.channel_ready = true;

    for (int attempt = 1; attempt <= PROBE_RETRIES; ++attempt) {
        result.attempt = attempt;

        drain_rx_queue(ch);

        TPCANStatus sts = send_get_id(ch, motor_id);
        result.last_status = sts;
        result.tx_ok = (sts == PCAN_ERROR_OK);
        result.stage = "GET_ID";
        if (!result.tx_ok)
            continue;

        if (wait_for_reply(ch, motor_id,
                           std::chrono::milliseconds(PROBE_TIMEOUT_MS), result)) {
            return result;
        }

        for (uint16_t param_idx : PROBE_PARAMS) {
            sts = send_param_read(ch, motor_id, param_idx);
            result.last_status = sts;
            result.tx_ok = (sts == PCAN_ERROR_OK);
            result.stage = (param_idx == robstride::PARAM_RUN_MODE)
                ? "PARAM_READ(run_mode)"
                : "PARAM_READ(mech_pos)";

            if (!result.tx_ok)
                break;

            if (wait_for_reply(ch, motor_id,
                               std::chrono::milliseconds(PROBE_TIMEOUT_MS), result)) {
                return result;
            }
        }
    }

    return result;
}

void print_result(const MotorMapEntry& entry, const ProbeResult& result)
{
    std::printf("  can%d  %-15s id=%3u  ",
                entry.channel_idx, entry.name, entry.motor_id);

    if (!result.channel_ready) {
        std::printf("CHANNEL_INIT_FAILED\n");
        return;
    }

    if (!result.tx_ok) {
        std::printf("TX_FAIL stage=%s status=%s\n",
                    result.stage,
                    PcanChannel::status_to_string(result.last_status).c_str());
        return;
    }

    if (!result.responded) {
        std::printf("NO_REPLY attempts=%d last_stage=%s\n",
                    result.attempt, result.stage);
        return;
    }

    const uint8_t comm_type = static_cast<uint8_t>((result.response_msg.ID >> 24) & 0x1F);
    std::printf("OK attempt=%d via=%s reply=%s ",
                result.attempt, result.stage, comm_type_to_string(comm_type));
    print_frame(result.response_msg);
    std::printf("\n");
}

}  // namespace

int main()
{
    std::printf("=== QUB_Controller Connection Test (non-actuating) ===\n\n");
    std::printf("안전 원칙:\n");
    std::printf("  - enable() / MIT 제어 명령을 보내지 않습니다.\n");
    std::printf("  - GET_ID / PARAM_READ 만으로 13개 모터 연결 상태를 확인합니다.\n");
    std::printf("  - 전원 인가 상태에서는 로봇을 공중에 띄우거나 지지한 뒤 실행하세요.\n\n");

    std::array<std::unique_ptr<PcanChannel>, NUM_CHANNELS> channels;
    std::array<bool, NUM_CHANNELS> channel_ok = {};

    std::printf("[1] CAN 채널 초기화\n");
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        try {
            channels[ch] = std::make_unique<PcanChannel>(CAN_CHANNELS[ch], CAN_BAUDRATE);
            channel_ok[ch] = true;
            std::printf("  can%d -> OK\n", ch);
        } catch (const std::exception& e) {
            std::printf("  can%d -> FAIL: %s\n", ch, e.what());
        }
    }
    std::printf("\n");

    std::printf("[2] 13개 모터 연결 확인\n");

    int ok_count = 0;
    int fail_count = 0;

    for (const auto& entry : MOTOR_MAP) {
        ProbeResult result;
        if (channel_ok[entry.channel_idx]) {
            result = probe_motor(*channels[entry.channel_idx], entry.motor_id);
        }

        result.channel_ready = channel_ok[entry.channel_idx];
        print_result(entry, result);

        if (result.channel_ready && result.tx_ok && result.responded) {
            ok_count++;
        } else {
            fail_count++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::printf("\n[3] 요약\n");
    std::printf("  응답 확인: %d / %d\n", ok_count, NUM_JOINTS);
    std::printf("  미응답    : %d / %d\n", fail_count, NUM_JOINTS);

    if (fail_count == 0) {
        std::printf("\n결과: 모든 모터에서 응답을 확인했습니다.\n");
        return 0;
    }

    std::printf("\n결과: 일부 모터 또는 채널에서 응답이 없습니다.\n");
    std::printf("힌트: 전원, CAN-H/L, 종단저항, 채널 매핑, motor_id 설정을 확인하세요.\n");
    return 2;
}
