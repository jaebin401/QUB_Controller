/*
 * QUB RobStride position monitor and enable/disable terminal GUI.
 *
 * Startup automatically scans all four PCAN channels. The selected motor can
 * be enabled or stopped with Space. Disabled motors are polled with the
 * non-actuating mechPos parameter read and shown with their channel and ID.
 *
 * The 50 Hz hold loop follows the control structure of position_control.cpp,
 * but the transport is QUB PcanChannel and the initial hold target is the
 * measured current position rather than zero.
 */

#include "pcan_channel.hpp"
#include "robstride.hpp"
#include "robot_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

constexpr int MIN_MOTOR_ID = 0;
constexpr int MAX_MOTOR_ID = 127;
constexpr int DEFAULT_SCAN_TIMEOUT_MS = 8;
constexpr int POSITION_TIMEOUT_MS = 30;
constexpr int CONTROL_PERIOD_MS = 20;      // 50 Hz, matching position_control.cpp
constexpr int OFF_POLL_DIVIDER = 5;        // one OFF query/channel every 100 ms
constexpr int UI_REFRESH_DIVIDER = 5;      // 10 Hz rendering
constexpr float DEFAULT_KP = 10.0F;
constexpr float DEFAULT_KD = 0.5F;
constexpr auto EMPTY_QUEUE_SLEEP = std::chrono::microseconds(100);
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

volatile std::sig_atomic_t g_running = 1;

struct Config {
    uint8_t host_id = robstride::HOST_ID_DEFAULT;
    int scan_timeout_ms = DEFAULT_SCAN_TIMEOUT_MS;
    float kp = DEFAULT_KP;
    float kd = DEFAULT_KD;
};

struct DeviceInfo {
    int channel_idx = 0;
    uint8_t motor_id = 0;
    std::array<uint8_t, 8> uid{};
};

struct MotorState {
    int channel_idx = 0;
    uint8_t motor_id = 0;
    std::array<uint8_t, 8> uid{};
    const char* joint_name = nullptr;
    std::unique_ptr<Robstride> driver;

    bool enabled = false;
    bool position_valid = false;
    float position_rad = 0.0F;
    float velocity_rad_s = 0.0F;
    float torque_nm = 0.0F;
    float temperature_c = 0.0F;
    float hold_position_rad = 0.0F;
    uint8_t mode_status = 0;
    uint8_t fault_flags = 0;
    uint64_t position_generation = 0;
    Clock::time_point last_update{};
};

uint32_t build_ext_id(uint8_t comm_type, uint16_t data_field, uint8_t motor_id)
{
    return (uint32_t(comm_type & 0x1F) << 24) |
           (uint32_t(data_field) << 8) |
           uint32_t(motor_id);
}

uint8_t comm_type(const TPCANMsg& msg)
{
    return static_cast<uint8_t>((msg.ID >> 24) & 0x1F);
}

uint8_t source_motor_id(const TPCANMsg& msg)
{
    return static_cast<uint8_t>((msg.ID >> 8) & 0xFF);
}

uint8_t destination_id(const TPCANMsg& msg)
{
    return static_cast<uint8_t>(msg.ID & 0xFF);
}

const char* mapped_joint_name(int channel_idx, uint8_t motor_id)
{
    for (const auto& entry : qub::MOTOR_MAP) {
        if (entry.channel_idx == channel_idx && entry.motor_id == motor_id)
            return entry.name;
    }
    return nullptr;
}

std::string uid_to_string(const std::array<uint8_t, 8>& uid)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t byte : uid)
        out << std::setw(2) << unsigned(byte);
    return out.str();
}

void drain_channel(PcanChannel& channel)
{
    TPCANMsg ignored{};
    for (int count = 0; count < 256; ++count) {
        const TPCANStatus status = channel.read(ignored);
        if (status != PCAN_ERROR_OK)
            return;
    }
}

std::optional<DeviceInfo> query_device(PcanChannel& channel,
                                       int channel_idx,
                                       uint8_t motor_id,
                                       const Config& config)
{
    const std::array<uint8_t, 8> data{};
    drain_channel(channel);

    const TPCANStatus write_status = channel.write_extended(
        build_ext_id(robstride::COMM_GET_ID, config.host_id, motor_id),
        data.data(), static_cast<uint8_t>(data.size()));
    if (write_status != PCAN_ERROR_OK)
        return std::nullopt;

    const auto deadline = Clock::now() +
                          std::chrono::milliseconds(config.scan_timeout_ms);
    while (Clock::now() < deadline) {
        TPCANMsg msg{};
        const TPCANStatus status = channel.read(msg);
        if (status == PCAN_ERROR_OK) {
            if ((msg.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0)
                continue;
            if (msg.LEN != 8)
                continue;
            if (comm_type(msg) != robstride::COMM_GET_ID ||
                source_motor_id(msg) != motor_id) {
                continue;
            }
            const uint8_t destination = destination_id(msg);
            if (destination != 0xFE && destination != config.host_id)
                continue;

            DeviceInfo device{};
            device.channel_idx = channel_idx;
            device.motor_id = source_motor_id(msg);
            std::copy_n(msg.DATA, device.uid.size(), device.uid.begin());
            return device;
        }
        if (status != PCAN_ERROR_QRCVEMPTY)
            return std::nullopt;
        std::this_thread::sleep_for(EMPTY_QUEUE_SLEEP);
    }
    return std::nullopt;
}

std::vector<DeviceInfo> scan_channels(
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    const Config& config)
{
    std::vector<DeviceInfo> devices;
    for (int channel_idx = 0; channel_idx < qub::NUM_CHANNELS; ++channel_idx) {
        if (!channels[static_cast<size_t>(channel_idx)]) {
            try {
                channels[static_cast<size_t>(channel_idx)] =
                    std::make_unique<PcanChannel>(
                        qub::CAN_CHANNELS[static_cast<size_t>(channel_idx)],
                        qub::CAN_BAUDRATE);
                std::cout << "ch" << channel_idx << ": initialized\n";
            } catch (const std::exception& error) {
                std::cerr << "Warning: ch" << channel_idx
                          << " initialization failed: " << error.what() << '\n';
                continue;
            }
        }

        std::cout << "ch" << channel_idx << ": scanning IDs "
                  << MIN_MOTOR_ID << ".." << MAX_MOTOR_ID << " ... "
                  << std::flush;
        const size_t before = devices.size();
        for (int id = MIN_MOTOR_ID; id <= MAX_MOTOR_ID && g_running; ++id) {
            auto device = query_device(
                *channels[static_cast<size_t>(channel_idx)], channel_idx,
                static_cast<uint8_t>(id), config);
            if (device)
                devices.push_back(*device);
        }
        std::cout << devices.size() - before << " found\n";
    }

    std::sort(devices.begin(), devices.end(), [](const DeviceInfo& lhs,
                                                  const DeviceInfo& rhs) {
        if (lhs.channel_idx != rhs.channel_idx)
            return lhs.channel_idx < rhs.channel_idx;
        return lhs.motor_id < rhs.motor_id;
    });
    return devices;
}

std::vector<MotorState> make_motor_states(
    const std::vector<DeviceInfo>& devices,
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    const Config& config)
{
    std::vector<MotorState> motors;
    motors.reserve(devices.size());
    for (const auto& device : devices) {
        MotorState motor{};
        motor.channel_idx = device.channel_idx;
        motor.motor_id = device.motor_id;
        motor.uid = device.uid;
        motor.joint_name = mapped_joint_name(device.channel_idx, device.motor_id);
        motor.driver = std::make_unique<Robstride>(
            channels[static_cast<size_t>(device.channel_idx)].get(),
            device.motor_id, config.host_id);
        motors.push_back(std::move(motor));
    }
    return motors;
}

TPCANStatus request_mechanical_position(PcanChannel& channel,
                                        uint8_t host_id,
                                        uint8_t motor_id)
{
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(robstride::PARAM_MECH_POS & 0xFF);
    data[1] = static_cast<uint8_t>(robstride::PARAM_MECH_POS >> 8);
    return channel.write_extended(
        build_ext_id(robstride::COMM_PARAM_READ, host_id, motor_id),
        data.data(), static_cast<uint8_t>(data.size()));
}

bool parse_mechanical_position(const TPCANMsg& msg,
                               uint8_t host_id,
                               uint8_t& motor_id,
                               float& position)
{
    if ((msg.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0 || msg.LEN != 8)
        return false;
    if (comm_type(msg) != robstride::COMM_PARAM_READ ||
        destination_id(msg) != host_id) {
        return false;
    }
    const uint16_t parameter = static_cast<uint16_t>(msg.DATA[0]) |
                               (static_cast<uint16_t>(msg.DATA[1]) << 8);
    if (parameter != robstride::PARAM_MECH_POS)
        return false;

    motor_id = source_motor_id(msg);
    std::memcpy(&position, &msg.DATA[4], sizeof(position));
    return std::isfinite(position);
}

void process_message(int channel_idx,
                     const TPCANMsg& msg,
                     uint8_t host_id,
                     std::vector<MotorState>& motors)
{
    if ((msg.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0)
        return;

    if (comm_type(msg) == robstride::COMM_FEEDBACK) {
        for (auto& motor : motors) {
            if (motor.channel_idx != channel_idx)
                continue;
            MotorFeedback feedback{};
            if (!motor.driver->try_parse_feedback(msg, feedback))
                continue;

            motor.position_valid = true;
            motor.position_rad = feedback.position;
            motor.velocity_rad_s = feedback.velocity;
            motor.torque_nm = feedback.torque;
            motor.temperature_c = feedback.temperature;
            motor.mode_status = feedback.mode_status;
            motor.fault_flags = feedback.fault_flags;
            ++motor.position_generation;
            motor.last_update = Clock::now();
            return;
        }
    }

    uint8_t motor_id = 0;
    float position = 0.0F;
    if (!parse_mechanical_position(msg, host_id, motor_id, position))
        return;
    for (auto& motor : motors) {
        if (motor.channel_idx == channel_idx && motor.motor_id == motor_id) {
            motor.position_valid = true;
            motor.position_rad = position;
            ++motor.position_generation;
            motor.last_update = Clock::now();
            return;
        }
    }
}

void drain_and_process(
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    uint8_t host_id,
    std::vector<MotorState>& motors)
{
    for (int channel_idx = 0; channel_idx < qub::NUM_CHANNELS; ++channel_idx) {
        auto& channel = channels[static_cast<size_t>(channel_idx)];
        if (!channel)
            continue;

        for (int count = 0; count < 256; ++count) {
            TPCANMsg msg{};
            const TPCANStatus status = channel->read(msg);
            if (status == PCAN_ERROR_QRCVEMPTY)
                break;
            if (status != PCAN_ERROR_OK)
                break;
            process_message(channel_idx, msg, host_id, motors);
        }
    }
}

bool refresh_position_sync(
    MotorState& target,
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    uint8_t host_id,
    std::vector<MotorState>& motors,
    int timeout_ms)
{
    auto& channel = channels[static_cast<size_t>(target.channel_idx)];
    if (!channel)
        return false;

    const uint64_t previous_generation = target.position_generation;
    if (request_mechanical_position(*channel, host_id, target.motor_id) !=
        PCAN_ERROR_OK) {
        return false;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (Clock::now() < deadline) {
        TPCANMsg msg{};
        const TPCANStatus status = channel->read(msg);
        if (status == PCAN_ERROR_OK) {
            process_message(target.channel_idx, msg, host_id, motors);
            if (target.position_generation != previous_generation)
                return true;
            continue;
        }
        if (status != PCAN_ERROR_QRCVEMPTY)
            return false;
        std::this_thread::sleep_for(EMPTY_QUEUE_SLEEP);
    }
    return false;
}

void refresh_initial_positions(
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    const Config& config,
    std::vector<MotorState>& motors)
{
    for (size_t index = 0; index < motors.size() && g_running; ++index) {
        refresh_position_sync(
            motors[index], channels, config.host_id, motors, POSITION_TIMEOUT_MS);
    }
}

bool enable_selected_motor(
    MotorState& motor,
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    const Config& config,
    std::vector<MotorState>& motors,
    std::string& status_message)
{
    if (!refresh_position_sync(
            motor, channels, config.host_id, motors, POSITION_TIMEOUT_MS)) {
        status_message = "ON refused: current position could not be read.";
        return false;
    }
    if (motor.position_rad < robstride::P_MIN ||
        motor.position_rad > robstride::P_MAX) {
        status_message = "ON refused: measured position is outside the MIT range.";
        return false;
    }

    motor.hold_position_rad = motor.position_rad;
    if (!motor.driver->stop(false)) {
        status_message = "ON failed: STOP frame could not be sent.";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!motor.driver->set_run_mode(robstride::MODE_MIT)) {
        status_message = "ON failed: MIT run_mode could not be set.";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Prime the measured hold target before enable so ON never commands zero.
    if (!motor.driver->send_mit_command(
            motor.hold_position_rad, 0.0F, config.kp, config.kd, 0.0F) ||
        !motor.driver->enable()) {
        motor.driver->stop(false);
        status_message = "ON failed: hold or ENABLE frame could not be sent.";
        return false;
    }

    motor.enabled = true;
    status_message = "Motor enabled at its measured hold position.";
    return true;
}

bool disable_selected_motor(MotorState& motor, std::string& status_message)
{
    if (!motor.driver->stop(false)) {
        status_message = "OFF failed: STOP frame could not be sent.";
        return false;
    }
    motor.enabled = false;
    status_message = "Motor stopped; position polling continues.";
    return true;
}

void stop_all(std::vector<MotorState>& motors)
{
    for (auto& motor : motors) {
        if (!motor.enabled)
            continue;
        motor.driver->stop(false);
        motor.enabled = false;
    }
}

void send_hold_commands(const Config& config,
                        std::vector<MotorState>& motors,
                        std::string& status_message)
{
    for (auto& motor : motors) {
        if (!motor.enabled)
            continue;
        if (!motor.driver->send_mit_command(
                motor.hold_position_rad, 0.0F, config.kp, config.kd, 0.0F)) {
            motor.driver->stop(false);
            motor.enabled = false;
            status_message = "Hold TX failed on ch" +
                std::to_string(motor.channel_idx) + ", ID " +
                std::to_string(motor.motor_id) + "; STOP requested.";
        }
    }
}

void poll_disabled_positions(
    std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS>& channels,
    uint8_t host_id,
    const std::vector<MotorState>& motors,
    std::array<size_t, qub::NUM_CHANNELS>& cursors)
{
    for (int channel_idx = 0; channel_idx < qub::NUM_CHANNELS; ++channel_idx) {
        auto& channel = channels[static_cast<size_t>(channel_idx)];
        if (!channel || motors.empty())
            continue;

        size_t& cursor = cursors[static_cast<size_t>(channel_idx)];
        for (size_t checked = 0; checked < motors.size(); ++checked) {
            const size_t index = (cursor + checked) % motors.size();
            const auto& motor = motors[index];
            if (motor.channel_idx != channel_idx || motor.enabled)
                continue;

            request_mechanical_position(*channel, host_id, motor.motor_id);
            cursor = (index + 1) % motors.size();
            break;
        }
    }
}

class TerminalGui {
public:
    enum class Key {
        NONE,
        UP,
        DOWN,
        TOGGLE,
        QUIT,
    };

    TerminalGui()
    {
        if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO))
            throw std::runtime_error("position_read requires an interactive terminal");
        if (::tcgetattr(STDIN_FILENO, &original_) != 0)
            throw std::runtime_error("cannot read terminal settings");

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            throw std::runtime_error("cannot enable raw terminal mode");
        active_ = true;
        std::cout << "\x1b[?25l\x1b[2J\x1b[H" << std::flush;
    }

    ~TerminalGui()
    {
        if (active_)
            ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
        std::cout << "\x1b[0m\x1b[?25h\n" << std::flush;
    }

    TerminalGui(const TerminalGui&) = delete;
    TerminalGui& operator=(const TerminalGui&) = delete;

    Key read_key() const
    {
        char buffer[8]{};
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count <= 0)
            return Key::NONE;
        if (buffer[0] == 'q' || buffer[0] == 'Q')
            return Key::QUIT;
        if (buffer[0] == ' ')
            return Key::TOGGLE;
        if (buffer[0] == 'k' || buffer[0] == 'K' ||
            (count >= 3 && buffer[0] == '\x1b' && buffer[1] == '[' &&
             buffer[2] == 'A')) {
            return Key::UP;
        }
        if (buffer[0] == 'j' || buffer[0] == 'J' ||
            (count >= 3 && buffer[0] == '\x1b' && buffer[1] == '[' &&
             buffer[2] == 'B')) {
            return Key::DOWN;
        }
        return Key::NONE;
    }

    void render(const std::vector<MotorState>& motors,
                size_t selected,
                const Config& config,
                const std::string& status_message) const
    {
        std::ostringstream out;
        out << "\x1b[H\x1b[2J"
            << "QUB RobStride Position Read / Hold\n"
            << "Arrows or j/k: select    Space: ON/OFF    q: quit\n"
            << "Automatic scan: ch0..ch3, IDs 0..127    Hold: 50 Hz, Kp="
            << config.kp << ", Kd=" << config.kd << "\n\n";

        if (motors.empty()) {
            out << "No motors found. Check power, CAN wiring, PCAN driver, and permissions.\n";
        } else {
            out << "    CH   ID   STATE    POSITION (rad / deg)       JOINT"
                   "                 MCU UID\n";
            out << "--------------------------------------------------------------------------"
                   "----------------\n";
            for (size_t index = 0; index < motors.size(); ++index) {
                const auto& motor = motors[index];
                out << (index == selected ? " >  " : "    ")
                    << std::setw(2) << motor.channel_idx << "   "
                    << std::setw(3) << unsigned(motor.motor_id) << "   ";
                if (motor.enabled)
                    out << "\x1b[32mON \x1b[0m     ";
                else
                    out << "\x1b[90mOFF\x1b[0m     ";

                if (motor.position_valid) {
                    out << std::fixed << std::setprecision(4)
                        << std::setw(9) << motor.position_rad << " / "
                        << std::setw(8) << std::setprecision(2)
                        << double(motor.position_rad) * RAD_TO_DEG << " deg   ";
                } else {
                    out << "       -- /       -- deg   ";
                }
                out << std::left << std::setw(21)
                    << (motor.joint_name ? motor.joint_name : "unmapped")
                    << std::right << uid_to_string(motor.uid);
                if (motor.fault_flags != 0)
                    out << "  fault=0x" << std::hex << std::uppercase
                        << unsigned(motor.fault_flags) << std::dec;
                out << '\n';
            }
        }

        out << "\nStatus: " << status_message << "\n"
            << "Safety: ON holds the measured current angle; OFF sends STOP and"
               " continues non-actuating position reads.\n";
        std::cout << out.str() << std::flush;
    }

private:
    termios original_{};
    bool active_ = false;
};

long parse_integer(const std::string& text, long minimum, long maximum,
                   const char* label)
{
    size_t parsed = 0;
    long value = 0;
    try {
        value = std::stol(text, &parsed, 0);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid ") + label + ": " + text);
    }
    if (parsed != text.size() || value < minimum || value > maximum)
        throw std::runtime_error(std::string(label) + " out of range: " + text);
    return value;
}

float parse_float(const std::string& text, float minimum, float maximum,
                  const char* label)
{
    size_t parsed = 0;
    float value = 0.0F;
    try {
        value = std::stof(text, &parsed);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid ") + label + ": " + text);
    }
    if (parsed != text.size() || !std::isfinite(value) ||
        value < minimum || value > maximum) {
        throw std::runtime_error(std::string(label) + " out of range: " + text);
    }
    return value;
}

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program << " [options]\n\n"
        << "Options:\n"
        << "  --host-id ID       Host ID, decimal or 0xNN (default: 0xFD)\n"
        << "  --scan-timeout MS  GET_ID timeout per ID (default: 8)\n"
        << "  --kp VALUE         Hold stiffness 0..500 (default: 10)\n"
        << "  --kd VALUE         Hold damping 0..5 (default: 0.5)\n"
        << "  -h, --help         Show this help\n";
}

Config parse_arguments(int argc, char** argv)
{
    Config config;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "-h" || argument == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (argument == "--host-id") {
            if (++index >= argc)
                throw std::runtime_error(argument + " requires a value");
            config.host_id = static_cast<uint8_t>(
                parse_integer(argv[index], 0, 255, "host ID"));
        } else if (argument == "--scan-timeout") {
            if (++index >= argc)
                throw std::runtime_error(argument + " requires a value");
            config.scan_timeout_ms = static_cast<int>(
                parse_integer(argv[index], 1, 1000, "scan timeout"));
        } else if (argument == "--kp") {
            if (++index >= argc)
                throw std::runtime_error(argument + " requires a value");
            config.kp = parse_float(
                argv[index], robstride::KP_MIN, robstride::KP_MAX, "Kp");
        } else if (argument == "--kd") {
            if (++index >= argc)
                throw std::runtime_error(argument + " requires a value");
            config.kd = parse_float(
                argv[index], robstride::KD_MIN, robstride::KD_MAX, "Kd");
        } else {
            throw std::runtime_error("unknown option: " + argument);
        }
    }
    return config;
}

void signal_handler(int)
{
    g_running = 0;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const Config config = parse_arguments(argc, argv);
        if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO))
            throw std::runtime_error("position_read must run in an interactive terminal");

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        std::cout << "QUB position_read: automatic PCAN scan\n";
        std::array<std::unique_ptr<PcanChannel>, qub::NUM_CHANNELS> channels;
        const auto devices = scan_channels(channels, config);
        auto motors = make_motor_states(devices, channels, config);
        refresh_initial_positions(channels, config, motors);

        size_t selected = 0;
        std::string status_message = motors.empty()
            ? "Scan completed: no motors found."
            : "Scan completed: select a motor and press Space.";
        std::array<size_t, qub::NUM_CHANNELS> poll_cursors{};

        try {
            TerminalGui gui;
            gui.render(motors, selected, config, status_message);

            int tick = 0;
            auto next_tick = Clock::now();
            while (g_running) {
                next_tick += std::chrono::milliseconds(CONTROL_PERIOD_MS);

                const TerminalGui::Key key = gui.read_key();
                if (key == TerminalGui::Key::QUIT) {
                    g_running = 0;
                } else if (!motors.empty() && key == TerminalGui::Key::UP) {
                    selected = (selected + motors.size() - 1) % motors.size();
                } else if (!motors.empty() && key == TerminalGui::Key::DOWN) {
                    selected = (selected + 1) % motors.size();
                } else if (!motors.empty() && key == TerminalGui::Key::TOGGLE) {
                    MotorState& motor = motors[selected];
                    if (motor.enabled)
                        disable_selected_motor(motor, status_message);
                    else
                        enable_selected_motor(
                            motor, channels, config, motors, status_message);
                }

                send_hold_commands(config, motors, status_message);
                if ((tick % OFF_POLL_DIVIDER) == 0) {
                    poll_disabled_positions(
                        channels, config.host_id, motors, poll_cursors);
                }
                drain_and_process(channels, config.host_id, motors);

                if ((tick % UI_REFRESH_DIVIDER) == 0 ||
                    key != TerminalGui::Key::NONE) {
                    gui.render(motors, selected, config, status_message);
                }
                ++tick;

                const auto now = Clock::now();
                if (next_tick > now)
                    std::this_thread::sleep_until(next_tick);
                else
                    next_tick = now;
            }
        } catch (...) {
            stop_all(motors);
            throw;
        }

        stop_all(motors);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
