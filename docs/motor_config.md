# RobStride Motor Configuration

`motor_config` is a PCANBasic maintenance tool for the RobStride private
protocol. It scans the four QUB PCAN channels, reads motor IDs and MCU UIDs,
changes a CAN ID, and resets the current shaft position as mechanical zero.
It does not enable motors, send MIT commands, change `run_mode`, or enter the
hard-RT controller path.

The implementation follows sections 4.1 and 4.2.6 of `RS02User Manual 260112`:

| Operation | Communication type | 29-bit extended CAN ID |
|---|---:|---|
| Get device ID | 0 | `0x00_00_HOST_MOTOR` |
| Stop motor | 4 | `0x04_00_HOST_MOTOR` |
| Set mechanical zero | 6 | `0x06_00_HOST_MOTOR`, data byte 0 = `01` |
| Set motor CAN ID | 7 | `0x07_NEW_HOST_OLD` |
| Read mechanical position | 17 | `0x11_00_HOST_MOTOR`, index `0x7019` |

The default host ID is `0xFD`. Motor IDs are limited to `0..127`. Protocol
constants come from `include/robstride.hpp`; the channel and motor topology
comes from `include/robot_config.hpp`.

## QUB channel map

Channel indices are local PCANBasic channels, not SocketCAN interface names.
The same motor ID may legitimately occur on more than one channel.

| `--channel` | PCAN handle | Configured motor IDs | Joints |
|---:|---|---|---|
| 0 | `PCAN_PCIBUS1` | 4, 5, 6 | right knee and ankle |
| 1 | `PCAN_PCIBUS2` | 0, 1, 2, 3 | torso and right hip |
| 2 | `PCAN_PCIBUS3` | 1, 2, 3 | left hip |
| 3 | `PCAN_PCIBUS4` | 4, 5, 6 | left knee and ankle |

`scan` opens all four channels by default. A channel that cannot be initialized
is reported as a warning and skipped while the remaining channels continue.
Use `--channel N` to scan only one channel. `get`, `set-id`, and `set-zero`
always require `--channel N` so that an ID is never ambiguous.

## Build

On the NUC with PCANBasic installed:

```bash
cmake -B build -S .
cmake --build build --target motor_config -j$(nproc)
```

If PCANBasic is installed in nonstandard locations:

```bash
cmake -B build -S . \
  -DPCAN_INCLUDE_DIR=/path/to/include \
  -DPCAN_LIB_DIR=/path/to/lib
cmake --build build --target motor_config -j$(nproc)
```

The target links only to `qub_hw`, `libpcanbasic`, `pthread`, and `rt`; it does
not depend on mip_sdk or ONNX Runtime.

PCAN chardev device access commonly needs root privileges, so hardware commands
below use `sudo`. `--dry-run` does not initialize PCAN and does not need `sudo`.

## Commands

Discover every responding ID on all four channels:

```bash
sudo ./build/motor_config scan
```

Scan a smaller inclusive range on every channel, or scan one channel only:

```bash
sudo ./build/motor_config scan 0 10
sudo ./build/motor_config scan --channel 2 0 10
```

Read motor ID 3 and its MCU UID on channel 1:

```bash
sudo ./build/motor_config get --channel 1 3
```

Change ID 3 to ID 7 on channel 1:

```bash
sudo ./build/motor_config set-id --channel 1 3 7
```

The tool verifies that the old ID responds and the new ID is unused on the
selected channel. It then sends STOP, changes the ID, queries the new ID, and
requires its MCU UID to match the original device. Destructive commands prompt
for confirmation. Use `--yes` only in a controlled non-interactive script.

Set the current shaft position of channel 0, motor ID 4 as mechanical zero:

```bash
sudo ./build/motor_config set-zero --channel 0 4
```

The sequence is GET_ID and MCU UID identification, `mechPos` read, confirmation,
STOP, SET_ZERO, Type 2 feedback acknowledgement, and `mechPos` read-back. A
read-back magnitude above `0.1 rad` is treated as a failure.

### Zero calibration caveats

Manual section 4.2.6 says zero calibration is supported in Motion Control and
CSP modes and blocked in PP mode. This tool never enables the motor or changes
`run_mode`; it sends STOP before Type 6 and relies on the normal power-on Motion
Control state.

The manual describes Type 6 as immediately effective but does not say that a
Type 22 data-save frame is required. This tool intentionally does not send Type
22 because it stores the full `0x20xx` parameter group and may have unrelated
side effects. Verify zero persistence after a hardware power cycle. Add an
explicit save option only if that test shows the zero is not retained.

## Dry run and common options

To inspect exact frames without opening or writing a PCAN channel:

```bash
./build/motor_config --dry-run scan 0 0
./build/motor_config --dry-run get --channel 1 127
./build/motor_config --dry-run set-id --channel 1 127 1
./build/motor_config --dry-run set-zero --channel 1 1
```

Expected default output includes:

```text
DRY-RUN ch0 00 00FD 00  data=0000000000000000
DRY-RUN ch1 00 00FD 7F  data=0000000000000000

DRY-RUN ch1 04 00FD 7F  data=0000000000000000
DRY-RUN ch1 07 01FD 7F  data=0000000000000000

DRY-RUN ch1 04 00FD 01  data=0000000000000000
DRY-RUN ch1 06 00FD 01  data=0100000000000000
```

Options may appear before or after the subcommand:

- `--channel N`: PCAN channel index `0..3`; optional only for `scan`
- `--host-id ID`: decimal or `0xNN`, default `0xFD`
- `--timeout MS`: reply timeout per attempt, default `20`
- `--retries N`: query attempts, default `2`
- `-y`, `--yes`: confirm `set-id` or `set-zero` without a prompt
- `--dry-run`: print frames without initializing PCAN
- `-v`, `--verbose`: print transmitted and received frames
- `-h`, `--help`: show usage

## Troubleshooting

A scan that finds nothing usually indicates motor power, CAN-H/CAN-L wiring,
120-ohm termination, 1 Mbps bitrate, device permissions, or private-protocol
mode. Run hardware access with `sudo` unless an appropriate udev rule exists.

PCANBasic chardev requires the `pcan` module. If the SocketCAN driver claimed
the board instead, switch drivers with:

```bash
sudo modprobe -r peak_pciefd && sudo modprobe pcan
```
