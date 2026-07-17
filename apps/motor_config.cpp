/*
 * RobStride CAN maintenance utility (private protocol / PCANBasic)
 *
 * Supported operations:
 *   - discover connected motor CAN IDs on one or all QUB PCAN channels
 *   - read one motor's ID and MCU unique identifier
 *   - change a motor CAN ID
 *   - set the current mechanical position as zero
 *
 * Protocol reference: RS02User Manual 260112, sections 4.1 and 4.2.6.
 * This is a maintenance CLI. It does not use or modify the hard-RT control path.
 */

#include "pcan_channel.hpp"
#include "robstride.hpp"
#include "robot_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

constexpr int DEFAULT_TIMEOUT_MS = 20;
constexpr int DEFAULT_RETRIES = 2;
constexpr int MIN_MOTOR_ID = 0;
constexpr int MAX_MOTOR_ID = 127;
constexpr auto EMPTY_QUEUE_SLEEP = std::chrono::microseconds(100);

struct Config {
    std::optional<int> channel_idx;
    uint8_t host_id = robstride::HOST_ID_DEFAULT;
    int timeout_ms = DEFAULT_TIMEOUT_MS;
    int retries = DEFAULT_RETRIES;
    bool assume_yes = false;
    bool dry_run = false;
    bool verbose = false;
};

struct DeviceInfo {
    uint8_t motor_id = 0;
    std::array<uint8_t, 8> uid{};
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

std::string frame_to_string(uint32_t raw_id, const uint8_t* data, size_t size)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0')
        << std::setw(8) << raw_id << '#';
    for (size_t i = 0; i < size; ++i)
        out << std::setw(2) << unsigned(data[i]);
    return out.str();
}

std::string frame_to_string(const TPCANMsg& msg)
{
    return frame_to_string(msg.ID, msg.DATA, msg.LEN);
}

std::string dry_run_frame_to_string(int channel_idx, uint32_t raw_id,
                                    const uint8_t* data, size_t size)
{
    std::ostringstream out;
    out << "ch" << channel_idx << ' '
        << std::uppercase << std::hex << std::setfill('0')
        << std::setw(2) << unsigned((raw_id >> 24) & 0x1F) << ' '
        << std::setw(4) << unsigned((raw_id >> 8) & 0xFFFF) << ' '
        << std::setw(2) << unsigned(raw_id & 0xFF) << "  data=";
    for (size_t i = 0; i < size; ++i)
        out << std::setw(2) << unsigned(data[i]);
    return out.str();
}

class CanBus {
public:
    CanBus(const Config& config, int channel_idx)
        : config_(config), channel_idx_(channel_idx)
    {
        if (config_.dry_run)
            return;

        channel_ = std::make_unique<PcanChannel>(
            qub::CAN_CHANNELS[static_cast<size_t>(channel_idx_)], qub::CAN_BAUDRATE);
    }

    ~CanBus() = default;

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    bool is_dry_run() const { return config_.dry_run; }
    int channel_idx() const { return channel_idx_; }

    bool send(uint32_t raw_id, const std::array<uint8_t, 8>& data)
    {
        if (config_.dry_run) {
            std::cout << "DRY-RUN "
                      << dry_run_frame_to_string(
                             channel_idx_, raw_id, data.data(), data.size())
                      << '\n';
            return true;
        }

        const TPCANStatus status = channel_->write_extended(
            raw_id, data.data(), static_cast<uint8_t>(data.size()));
        if (status != PCAN_ERROR_OK) {
            std::cerr << "CAN write failed on ch" << channel_idx_ << ": "
                      << PcanChannel::status_to_string(status) << '\n';
            return false;
        }
        if (config_.verbose)
            std::cout << "TX ch" << channel_idx_ << ' '
                      << frame_to_string(raw_id, data.data(), data.size()) << '\n';
        return true;
    }

    bool receive(TPCANMsg& msg, int timeout_ms)
    {
        if (config_.dry_run)
            return false;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(std::max(0, timeout_ms));
        do {
            TPCANMsg candidate{};
            const TPCANStatus status = channel_->read(candidate);
            if (status == PCAN_ERROR_OK) {
                // TPCANMsg::ID is already the raw 29-bit ID. Only extended
                // frames belong to the RobStride private protocol handled here.
                if ((candidate.MSGTYPE & PCAN_MESSAGE_EXTENDED) == 0)
                    continue;
                msg = candidate;
                if (config_.verbose)
                    std::cout << "RX ch" << channel_idx_ << ' '
                              << frame_to_string(msg) << '\n';
                return true;
            }

            if (status != PCAN_ERROR_QRCVEMPTY) {
                std::cerr << "CAN read failed on ch" << channel_idx_ << ": "
                          << PcanChannel::status_to_string(status) << '\n';
                return false;
            }

            if (timeout_ms <= 0)
                return false;
            std::this_thread::sleep_for(EMPTY_QUEUE_SLEEP);
        } while (std::chrono::steady_clock::now() < deadline);

        return false;
    }

    void drain()
    {
        TPCANMsg ignored{};
        // Do not spin forever when firmware active reporting (Type 24) is enabled.
        for (int count = 0; count < 256 && receive(ignored, 0); ++count) {
        }
    }

private:
    const Config& config_;
    int channel_idx_;
    std::unique_ptr<PcanChannel> channel_;
};

std::optional<TPCANMsg> wait_for(
    CanBus& bus, int timeout_ms, const std::function<bool(const TPCANMsg&)>& predicate)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        TPCANMsg msg{};
        if (!bus.receive(msg, std::max(1, static_cast<int>(remaining.count()))))
            continue;
        if (predicate(msg))
            return msg;
    }
    return std::nullopt;
}

std::optional<DeviceInfo> get_device(CanBus& bus, const Config& config, uint8_t motor_id)
{
    const std::array<uint8_t, 8> empty{};
    for (int attempt = 0; attempt < config.retries; ++attempt) {
        bus.drain();
        const uint32_t id = build_ext_id(robstride::COMM_GET_ID, config.host_id, motor_id);
        if (!bus.send(id, empty))
            return std::nullopt;
        if (bus.is_dry_run())
            return DeviceInfo{motor_id, {}};

        auto reply = wait_for(bus, config.timeout_ms, [&](const TPCANMsg& msg) {
            if (comm_type(msg) != robstride::COMM_GET_ID ||
                source_motor_id(msg) != motor_id) {
                return false;
            }
            // The manual fixes GET_ID's reply destination at 0xFE. Some firmware
            // revisions instead echo the configured host ID, so accept both.
            const uint8_t destination = destination_id(msg);
            return destination == 0xFE || destination == config.host_id;
        });
        if (reply) {
            DeviceInfo device{};
            device.motor_id = source_motor_id(*reply);
            std::copy_n(reply->DATA, device.uid.size(), device.uid.begin());
            return device;
        }
    }
    return std::nullopt;
}

bool send_stop(CanBus& bus, const Config& config, uint8_t motor_id)
{
    const std::array<uint8_t, 8> empty{};
    bus.drain();
    return bus.send(build_ext_id(robstride::COMM_STOP, config.host_id, motor_id), empty);
}

bool send_set_id(CanBus& bus, const Config& config, uint8_t old_id, uint8_t new_id)
{
    const std::array<uint8_t, 8> empty{};
    const uint16_t data_field = (uint16_t(new_id) << 8) | config.host_id;
    bus.drain();
    return bus.send(build_ext_id(robstride::COMM_SET_CAN_ID, data_field, old_id), empty);
}

bool send_set_zero(CanBus& bus, const Config& config, uint8_t motor_id)
{
    std::array<uint8_t, 8> data{};
    data[0] = 1;
    bus.drain();
    return bus.send(build_ext_id(robstride::COMM_SET_ZERO, config.host_id, motor_id), data);
}

std::optional<double> read_mechanical_position(
    CanBus& bus, const Config& config, uint8_t motor_id)
{
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(robstride::PARAM_MECH_POS & 0xFF);
    data[1] = static_cast<uint8_t>(robstride::PARAM_MECH_POS >> 8);
    bus.drain();
    if (!bus.send(build_ext_id(robstride::COMM_PARAM_READ, config.host_id, motor_id), data))
        return std::nullopt;
    if (bus.is_dry_run())
        return 0.0;

    auto reply = wait_for(bus, config.timeout_ms * 3, [&](const TPCANMsg& msg) {
        return comm_type(msg) == robstride::COMM_PARAM_READ &&
               source_motor_id(msg) == motor_id &&
               destination_id(msg) == config.host_id &&
               msg.LEN == 8 && msg.DATA[0] == data[0] && msg.DATA[1] == data[1];
    });
    if (!reply)
        return std::nullopt;

    float position = 0.0F;
    std::memcpy(&position, &reply->DATA[4], sizeof(position));
    return static_cast<double>(position);
}

std::string uid_to_string(const DeviceInfo& device)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t byte : device.uid)
        out << std::setw(2) << unsigned(byte);
    return out.str();
}

const char* mapped_joint_name(int channel_idx, uint8_t motor_id)
{
    for (const auto& entry : qub::MOTOR_MAP) {
        if (entry.channel_idx == channel_idx && entry.motor_id == motor_id)
            return entry.name;
    }
    return nullptr;
}

bool confirm_destructive(const Config& config, const std::string& question)
{
    if (config.assume_yes || config.dry_run)
        return true;
    if (!::isatty(STDIN_FILENO)) {
        std::cerr << "Refusing a non-interactive destructive operation. Add --yes to confirm.\n";
        return false;
    }

    std::cout << question << " [y/N] " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "y" || answer == "Y" || answer == "yes" || answer == "YES";
}

int parse_integer(const std::string& text, int minimum, int maximum, const char* label)
{
    size_t parsed = 0;
    long value;
    try {
        value = std::stol(text, &parsed, 0);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("Invalid ") + label + ": " + text);
    }
    if (parsed != text.size() || value < minimum || value > maximum)
        throw std::runtime_error(std::string(label) + " must be in range " +
                                 std::to_string(minimum) + ".." +
                                 std::to_string(maximum) + ": " + text);
    return static_cast<int>(value);
}

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program << " [options] <command> [arguments]\n\n"
        << "Commands:\n"
        << "  scan [MIN [MAX]]             Discover motor IDs (default: 0..127)\n"
        << "  get --channel N ID           Read one motor ID and MCU UID\n"
        << "  set-id --channel N OLD NEW   Change a motor ID and verify the new ID\n"
        << "  set-zero --channel N ID      Stop the motor and set its current angle to zero\n\n"
        << "Options:\n"
        << "  --channel N            PCAN channel index 0..3 (optional only for scan)\n"
        << "  --host-id ID           Host ID in decimal or 0xNN (default: 0xFD)\n"
        << "  --timeout MS           Reply timeout per attempt (default: 20)\n"
        << "  --retries N            Query attempts (default: 2)\n"
        << "  -y, --yes              Confirm set-id/set-zero without a prompt\n"
        << "  --dry-run              Print PCAN channel and frames without sending\n"
        << "  -v, --verbose          Print transmitted and received CAN frames\n"
        << "  -h, --help             Show this help\n\n"
        << "Examples:\n"
        << "  " << program << " scan\n"
        << "  " << program << " scan --channel 2 0 10\n"
        << "  " << program << " get --channel 1 3\n"
        << "  " << program << " --dry-run set-id --channel 1 3 7\n"
        << "  " << program << " --yes set-zero --channel 0 4\n";
}

int command_scan(const Config& config, const std::vector<std::string>& args)
{
    if (args.size() > 2)
        throw std::runtime_error("scan accepts at most MIN and MAX");
    const int minimum = args.empty() ? MIN_MOTOR_ID :
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID");
    const int maximum = args.size() < 2 ? MAX_MOTOR_ID :
        parse_integer(args[1], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID");
    if (minimum > maximum)
        throw std::runtime_error("scan MIN must not be greater than MAX");

    std::vector<int> channels;
    if (config.channel_idx) {
        channels.push_back(*config.channel_idx);
    } else {
        for (int channel_idx = 0; channel_idx < qub::NUM_CHANNELS; ++channel_idx)
            channels.push_back(channel_idx);
    }

    size_t total_devices = 0;
    int opened_channels = 0;
    for (int channel_idx : channels) {
        try {
            CanBus bus(config, channel_idx);
            ++opened_channels;
            if (bus.is_dry_run()) {
                std::cout << "Dry-run scan on ch" << channel_idx
                          << " prints one representative query only.\n";
                get_device(bus, config, static_cast<uint8_t>(minimum));
                continue;
            }

            std::cout << "Scanning ch" << channel_idx << " for RobStride IDs "
                      << minimum << ".." << maximum << " ...\n";
            size_t channel_devices = 0;
            for (int id = minimum; id <= maximum; ++id) {
                auto device = get_device(bus, config, static_cast<uint8_t>(id));
                if (!device)
                    continue;

                ++channel_devices;
                ++total_devices;
                std::cout << "  (channel_idx=" << channel_idx
                          << ", motor_id=" << std::setw(3) << id
                          << ", MCU UID=" << uid_to_string(*device) << ')';
                if (const char* name = mapped_joint_name(channel_idx, device->motor_id))
                    std::cout << "  MOTOR_MAP=" << name;
                std::cout << '\n';
            }
            if (channel_devices == 0)
                std::cout << "  No motors responded on ch" << channel_idx << ".\n";
        } catch (const std::exception& error) {
            std::cerr << "Warning: ch" << channel_idx
                      << " initialization failed; skipping channel: "
                      << error.what() << '\n';
        }
    }

    if (config.dry_run)
        return 0;
    if (total_devices == 0) {
        if (opened_channels == 0)
            std::cerr << "No PCAN channels could be initialized.\n";
        std::cout << "No motors responded. Check power, CAN-H/L, 120 ohm termination, "
                     "1 Mbps bitrate, PCAN driver, and protocol mode.\n";
        return 1;
    }
    std::cout << "Found " << total_devices << " motor(s) across "
              << opened_channels << " initialized channel(s).\n";
    return 0;
}

int command_get(CanBus& bus, const Config& config, const std::vector<std::string>& args)
{
    if (args.size() != 1)
        throw std::runtime_error("get requires exactly one motor ID");
    const auto id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID"));
    auto device = get_device(bus, config, id);
    if (!device) {
        std::cerr << "Motor ID " << unsigned(id) << " on ch" << bus.channel_idx()
                  << " did not respond.\n";
        return 1;
    }
    if (!bus.is_dry_run()) {
        std::cout << "Channel:  " << bus.channel_idx()
                  << "\nMotor ID: " << unsigned(device->motor_id)
                  << "\nMCU UID:  " << uid_to_string(*device) << '\n';
    }
    return 0;
}

int command_set_id(CanBus& bus, const Config& config,
                   const std::vector<std::string>& args)
{
    if (args.size() != 2)
        throw std::runtime_error("set-id requires OLD_ID and NEW_ID");
    const auto old_id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "old motor ID"));
    const auto new_id = static_cast<uint8_t>(
        parse_integer(args[1], MIN_MOTOR_ID, MAX_MOTOR_ID, "new motor ID"));
    if (old_id == new_id)
        throw std::runtime_error("OLD_ID and NEW_ID are identical");

    std::optional<DeviceInfo> old_device;
    if (!bus.is_dry_run()) {
        old_device = get_device(bus, config, old_id);
        if (!old_device) {
            std::cerr << "Motor ID " << unsigned(old_id) << " on ch"
                      << bus.channel_idx() << " did not respond; no change made.\n";
            return 1;
        }
        if (get_device(bus, config, new_id)) {
            std::cerr << "Motor ID " << unsigned(new_id) << " on ch"
                      << bus.channel_idx() << " is already in use; no change made.\n";
            return 1;
        }
        std::cout << "Target ch" << bus.channel_idx()
                  << " MCU UID: " << uid_to_string(*old_device) << '\n';
    }

    if (!confirm_destructive(config,
            "Change motor ID " + std::to_string(old_id) + " -> " +
            std::to_string(new_id) + " on ch" +
            std::to_string(bus.channel_idx()) + "?")) {
        std::cout << "Cancelled.\n";
        return 1;
    }

    // Stop first so an ID maintenance operation cannot leave an enabled motor moving.
    if (!send_stop(bus, config, old_id) ||
        !send_set_id(bus, config, old_id, new_id)) {
        return 1;
    }
    if (bus.is_dry_run())
        return 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    auto changed = get_device(bus, config, new_id);
    if (!changed) {
        std::cerr << "ID-change frame was sent, but motor ID " << unsigned(new_id)
                  << " on ch" << bus.channel_idx()
                  << " did not answer verification. Power-cycle only after checking the bus.\n";
        return 1;
    }
    if (old_device && changed->uid != old_device->uid) {
        std::cerr << "Motor ID " << unsigned(new_id) << " on ch"
                  << bus.channel_idx()
                  << " answered with a different MCU UID; check for an ID collision.\n";
        return 1;
    }

    std::cout << "Motor ID changed successfully on ch" << bus.channel_idx()
              << ": " << unsigned(old_id) << " -> " << unsigned(new_id)
              << "\nMCU UID: " << uid_to_string(*changed) << '\n';
    return 0;
}

int command_set_zero(CanBus& bus, const Config& config,
                     const std::vector<std::string>& args)
{
    if (args.size() != 1)
        throw std::runtime_error("set-zero requires exactly one motor ID");
    const auto id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID"));

    if (!bus.is_dry_run()) {
        auto device = get_device(bus, config, id);
        if (!device) {
            std::cerr << "Motor ID " << unsigned(id) << " on ch"
                      << bus.channel_idx() << " did not respond; no change made.\n";
            return 1;
        }
        std::cout << "Target ch" << bus.channel_idx()
                  << " MCU UID: " << uid_to_string(*device) << '\n';
        if (auto before = read_mechanical_position(bus, config, id))
            std::cout << "Current mechanical position: " << *before << " rad\n";
    }

    if (!confirm_destructive(config,
            "Stop motor ID " + std::to_string(id) + " on ch" +
            std::to_string(bus.channel_idx()) +
            " and set its current mechanical position to zero?")) {
        std::cout << "Cancelled.\n";
        return 1;
    }

    if (!send_stop(bus, config, id))
        return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Manual 4.2.6 supports zero calibration in Motion Control and CSP, but
    // blocks it in PP. This tool neither enables the motor nor changes run_mode;
    // it issues STOP first and relies on the power-on default Motion Control mode.
    // Type 6 is documented as immediate. Do not add COMM_SAVE (Type 22): the
    // manual does not require it here and saving all 0x20xx parameters may have
    // unrelated side effects. Verify persistence across a hardware power cycle.
    if (!send_set_zero(bus, config, id))
        return 1;
    if (bus.is_dry_run())
        return 0;

    auto acknowledgement = wait_for(bus, config.timeout_ms * 3, [&](const TPCANMsg& msg) {
        return comm_type(msg) == robstride::COMM_FEEDBACK &&
               source_motor_id(msg) == id && destination_id(msg) == config.host_id;
    });
    if (!acknowledgement) {
        std::cerr << "Zero-setting frame was sent, but no feedback acknowledgement arrived.\n";
        return 1;
    }

    std::optional<double> position;
    for (int attempt = 0; attempt < 3 && !position; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        position = read_mechanical_position(bus, config, id);
    }
    if (position) {
        if (std::abs(*position) > 0.1) {
            std::cerr << "Zero command was acknowledged, but read-back is " << *position
                      << " rad (expected within +/-0.1 rad). Check firmware and shaft state.\n";
            return 1;
        }
        std::cout << "Mechanical zero set and verified. Reported position: "
                  << *position << " rad\n";
    } else {
        std::cout << "Mechanical zero command acknowledged. Position read-back was unavailable.\n";
    }
    return 0;
}

bool command_requires_channel(const std::string& command)
{
    return command == "get" || command == "set-id" || command == "set-zero";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config config;
        std::string command;
        std::vector<std::string> command_args;

        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            if (arg == "-h" || arg == "--help") {
                print_usage(argv[0]);
                return 0;
            }
            if (arg == "--channel") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.channel_idx = parse_integer(
                    argv[index], 0, qub::NUM_CHANNELS - 1, "channel");
            } else if (arg == "--host-id") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.host_id = static_cast<uint8_t>(
                    parse_integer(argv[index], 0, 255, "host ID"));
            } else if (arg == "--timeout") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.timeout_ms = parse_integer(argv[index], 1, 10000, "timeout");
            } else if (arg == "--retries") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.retries = parse_integer(argv[index], 1, 20, "retries");
            } else if (arg == "-y" || arg == "--yes") {
                config.assume_yes = true;
            } else if (arg == "--dry-run") {
                config.dry_run = true;
            } else if (arg == "-v" || arg == "--verbose") {
                config.verbose = true;
            } else if (!arg.empty() && arg[0] == '-') {
                throw std::runtime_error("Unknown option: " + arg);
            } else if (command.empty()) {
                command = arg;
            } else {
                command_args.push_back(arg);
            }
        }

        if (command.empty()) {
            print_usage(argv[0]);
            return 2;
        }
        if (command_requires_channel(command) && !config.channel_idx) {
            print_usage(argv[0]);
            throw std::runtime_error(command + " requires --channel N (0..3)");
        }

        if (command == "scan")
            return command_scan(config, command_args);

        if (command == "get" || command == "set-id" || command == "set-zero") {
            CanBus bus(config, *config.channel_idx);
            if (command == "get")
                return command_get(bus, config, command_args);
            if (command == "set-id")
                return command_set_id(bus, config, command_args);
            return command_set_zero(bus, config, command_args);
        }

        throw std::runtime_error("Unknown command: " + command);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
