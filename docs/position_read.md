# RobStride Position Read / Hold GUI

`position_read` is a full-screen terminal GUI for discovering and checking the
RobStride motors connected to the four QUB PCAN channels. It automatically
scans IDs `0..127` at startup; no separate `motor_config scan` command is
required.

The GUI uses the control-loop structure of the reference
`position_control.cpp`, adapted to `PcanChannel` and the QUB channel map. It
does not enter the hard-RT controller path.

## Safety behavior

- OFF motors receive no MIT command. Their `mechPos` parameter (`0x7019`) is
  read periodically, so the screen continues showing channel, ID, current
  angle, configured joint name, and MCU UID.
- Pressing Space first reads the selected motor's current position. ON is
  refused if that read fails or the value is outside the RS02 MIT position
  range.
- The measured position becomes the hold target before ENABLE is sent. The
  program never uses zero as the automatic ON target.
- While ON, the program sends the measured hold target at 50 Hz, matching the
  basic cyclic structure of `position_control.cpp`.
- Pressing Space again sends STOP. Quitting or receiving SIGINT/SIGTERM stops
  every motor that this program enabled.

Only one process should own/read a PCAN channel at a time. Do not run this GUI
alongside `test_4ch`, the RT controller, `motor_config`, or another CAN reader.
Support the robot mechanically before enabling any actuator.

## Build and run

```bash
cmake -B build -S .
cmake --build build --target position_read -j$(nproc)
sudo ./build/position_read
```

PCAN chardev access generally requires `sudo` unless a suitable udev rule is
installed. The program must run in an interactive terminal.

Controls:

- Up/Down arrows or `k`/`j`: select a motor
- Space: toggle the selected motor ON/OFF
- `q`: stop motors and quit

Optional settings:

```bash
sudo ./build/position_read --host-id 0xFD --scan-timeout 8 --kp 10 --kd 0.5
```

`--kp` accepts `0..500` and `--kd` accepts `0..5`, matching the RS02 ranges in
`robstride.hpp`. The low defaults are intentional starting values; raise them
only after hardware testing confirms the mechanism is safely supported.

## Startup scan

Each PCAN channel is initialized independently. A failed channel is reported
and skipped while the remaining channels continue. GET_ID replies are stored
as `(channel_idx, motor_id, MCU UID)`, so IDs duplicated across channels remain
distinct. Entries matching `qub::MOTOR_MAP` also show the configured joint
name.

A complete default scan sends 128 GET_ID queries per initialized channel and
can take several seconds. Increase `--scan-timeout` if the adapter or firmware
responds slowly.
