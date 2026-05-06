# Flight Controller

Embedded flight software for an ESP32-based CubeSat flight controller. Manages attitude determination and control (ADCS), battery monitoring (BMS), camera image capture and transmission (CAM), and LoRa radio communications.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [File Structure](#file-structure)
- [Subsystems](#subsystems)
  - [RadioComms](#radiocomms)
  - [ADCS](#adcs-attitude-determination-and-control-system)
  - [BMS](#bms-battery-management-system)
  - [CAM](#cam-camera)
  - [main.cpp](#maincpp--task-scheduler)
- [Packet Protocol](#packet-protocol)
- [Command Reference](#command-reference)
- [Telemetry Reference](#telemetry-reference)
- [Task Timing Summary](#task-timing-summary)
- [Hardware Pin Assignments](#hardware-pin-assignments)
- [Dependencies](#dependencies)

---

## Architecture Overview

```
Ground Station ↔ [LoRa 915 MHz] ↔ Flight Controller (0xAA)
                                            │
                                 RadioComms (packet router)
                                            │
                       ┌────────────────────┼─────────────────────┐
                       │                    │                     │
                    ADCS                  CAM                   BMS
             (attitude control)     (image capture &       (battery monitor)
                    │                 transmission)              │
           Reaction Wheels                  │               BQ76905 (I2C)
           (Serial1, 115200)         Camera (Serial2)
           BNO055 IMU (I2C)
```

The design is modular and namespace-isolated. Each subsystem registers its own packet callbacks with `RadioComms` during initialization. A custom microsecond-based task scheduler in `main.cpp` calls subsystem task functions at their required rates without blocking.

---

## File Structure

```
flight-controller/
├── Common.h          # Shared packet IDs, Packet struct, Task struct
├── RadioComms.h/.cpp # LoRa driver and packet dispatcher
├── ADCS.h/.cpp       # Attitude Determination and Control System
├── BMS.h/.cpp        # Battery Management System
├── CAM.h/.cpp        # Camera capture and LoRa image transmission
└── main.cpp          # Entry point, task scheduler, deployment logic
```

---

## Subsystems

### RadioComms

**Files:** `RadioComms.h`, `RadioComms.cpp`

Owns the LoRa radio and routes incoming packets to registered callbacks.

#### LoRa Configuration

| Parameter        | Value        |
|------------------|--------------|
| Frequency        | 915 MHz      |
| Spreading Factor | 7            |
| Signal Bandwidth | 500 kHz      |
| Coding Rate      | 4/5          |
| CRC              | Enabled      |
| Local Address    | `0xAA`       |
| Broadcast        | `0xFF`       |

**SPI Pins:** CS=10, RESET=9, MOSI=11, MISO=12, SCK=13, EN=14

#### Key Functions

| Function | Description |
|----------|-------------|
| `init()` | Initializes LoRa hardware and settings |
| `registerCallback(id, fn)` | Registers a handler for a given packet ID |
| `processWaitingPackets()` | Polls radio, validates recipient address, dispatches callbacks |
| `emitPacket(Packet*)` | Transmits a packet with broadcast recipient |
| `packetAddFloat/Uint32/Uint16/Uint8()` | Serializes values into packet payload (little-endian) |
| `packetGetFloat/Uint32/Uint8()` | Deserializes values from packet payload (little-endian) |

Callback dispatch uses `std::map<uint8_t, commFunction>`. Packets not addressed to `0xAA` or `0xFF` are silently dropped.

---

### ADCS (Attitude Determination and Control System)

**Files:** `ADCS.h`, `ADCS.cpp`

Implements closed-loop quaternion attitude control at 50 Hz using a BNO055 IMU and three reaction wheels driven over UART.

#### Hardware

| Component     | Interface | Pins / Address         |
|---------------|-----------|------------------------|
| BNO055 IMU    | I2C       | SDA=8, SCL=7, Addr=0x28|
| Reaction Wheels | Serial1  | TX/RX, 115200 baud     |

#### Control Law

The controller uses a quaternion error formulation:

```
q_error = q_desired ⊗ conj(q_current)
τ_body = -Kp * q_error_vector - Ki * integral - Kd * omega
```

Key control parameters:

| Parameter          | Value          | Description                            |
|--------------------|----------------|----------------------------------------|
| Control period     | 20 ms (50 Hz)  | Main ADCS loop rate                    |
| `I_MAX`            | 0.1 rad·s      | Integrator windup clamp                |
| `JW_X/Y/Z`         | 3.11e-6 kg·m²  | Reaction wheel moment of inertia       |
| `TAU_MAX`          | 3.11e-4 N·m    | Maximum torque per axis                |
| `RAMP_MAX`         | 100.0          | Maximum wheel speed ramp rate          |
| Gyro deadband      | 0.20 rad/s     | Suppress control below this rate       |
| Quat error deadband| 0.03           | Suppress control below this error      |

Axis enables: X and Z enabled by default; Y disabled.

#### Key Functions

| Function | Description |
|----------|-------------|
| `init()` | Initializes BNO055, waits for calibration, registers command callbacks |
| `task_runADCS()` | 50 Hz control loop: read IMU → compute error → PID → send wheel torques |
| `task_sendADCSTelem()` | 1 Hz: emit 56-byte telemetry (setpoint, attitude, gyro, integrators) |
| `task_sendADCSParams()` | 0.2 Hz: emit 39-byte PID gains and axis enable state |
| `setDesiredAttitude(Packet)` | Set target quaternion from ground command |
| `enableADCS(Packet)` | Enable/disable control per axis |
| `setPIDGains(Packet)` | Adjust Kp/Ki/Kd per axis at runtime |
| `zeroWheelsCmd(Packet)` | Disable all axes and zero wheel speeds |

Control output is suppressed entirely while the camera is transmitting (`CAM::isTransmitting()`) to avoid power draw and electrical noise.

---

### BMS (Battery Management System)

**Files:** `BMS.h`, `BMS.cpp`

Monitors the 4S lithium battery pack via a BQ76905 BMS IC.

#### Hardware

| Component | Interface | Pins              |
|-----------|-----------|-------------------|
| BQ76905   | I2C       | SDA=4, SCL=48, 100 kHz |

#### Configuration

| Parameter       | Value         |
|-----------------|---------------|
| Cell configuration | 4S (series) |
| Shunt resistor  | 10 mOhm       |
| NTC pull-up     | 20 kOhm       |
| NTC R0 / T0     | 10 kOhm / 298.15 K |
| NTC Beta        | 3380          |

#### Key Functions

| Function | Description |
|----------|-------------|
| `init()` | Initializes I2C and BMS with recovery mechanism |
| `task_sendBMSTelem()` | 1 Hz: emit 22-byte telemetry (cell voltages, stack voltage, current, temperatures) |

Telemetry transmission is paused when the camera is actively sending image data.

---

### CAM (Camera)

**Files:** `CAM.h`, `CAM.cpp`

Manages image capture from a UART-connected camera module and downlinks images over LoRa using chunked ARQ transfer.

#### Hardware

| Component | Interface | Pins              |
|-----------|-----------|-------------------|
| Camera    | Serial2   | RX=35, TX=21, 115200 baud |
| RX buffer | —         | 8192 bytes        |

#### Transfer Protocol

Images are broken into 246-byte chunks and sent as `CAM_IMAGE_DATA` packets. Each chunk is acknowledged individually; unacknowledged chunks are retransmitted on NACK.

| Parameter       | Value       |
|-----------------|-------------|
| Max image size  | 131072 bytes |
| Chunk size      | 246 bytes   |
| Max retries     | 3           |
| Retry timeout   | 3000 ms     |

**TX state machine phases:**

```
TX_IDLE → TX_META → TX_CHUNKS → TX_SEND_DONE → TX_WAIT_FEEDBACK → TX_IDLE
```

#### UART Command Protocol

| Symbol           | Byte | Description               |
|------------------|------|---------------------------|
| `UART_CMD_BURST` | `B`  | Capture one or more photos |
| `UART_CMD_NEXT`  | `N`  | Advance to next image      |
| `UART_CMD_RESET` | `R`  | Reset camera state         |
| `UART_CMD_SET_RES`| `S` | Set capture resolution     |

3-byte sync headers used in the UART framing: `ACK`, `RES`, `IMG`, `END`.

#### Supported Resolutions

| Name        | Value |
|-------------|-------|
| QVGA        | 0     |
| VGA         | 1     |
| SVGA        | 2     |
| XGA         | 3     |
| HD          | 4     |
| SXGA        | 5     |
| UXGA        | 6     |
| QXGA        | 7     |

#### Key Functions

| Function | Description |
|----------|-------------|
| `init()` | Configures Serial2, registers command callbacks |
| `task_processCamera()` | Main poll loop: drives TX state machine, timeouts, UART parsing |
| `processCameraSerial()` | 8-state UART parser (sync → header → payload) |
| `handleTakePhoto(Packet)` | Initiates a burst capture sequence |
| `handleSetResolution(Packet)` | Changes capture resolution |
| `handleNack(Packet)` | Processes retransmission request from ground |
| `handleImageAck(Packet)` | Marks image transfer as complete |
| `isTransmitting()` | Returns true when TX is not idle (used by ADCS and BMS) |

---

### main.cpp — Task Scheduler

**File:** `main.cpp`

Entry point that initializes all subsystems and runs a cooperative task scheduler.

#### Task Scheduler

A custom microsecond-based scheduler avoids Arduino `delay()` blocking. Each entry in `taskTable[]` holds:

- `enabled` — whether the task is active
- function pointer
- `nexttime` — microsecond timestamp for next execution

Tasks fire when `nexttime - currentTime > UINT32_MAX / 2`, which correctly handles 32-bit timer wrap-around.

#### Task Table

| Task | Rate | Period |
|------|------|--------|
| `task_autoDeploy` | — | State machine (fires periodically) |
| `task_blinkLEDs` | 1 Hz | 1 s |
| `ADCS::task_runADCS` | 50 Hz | 20 ms |
| `ADCS::task_sendADCSTelem` | 1 Hz | 1 s |
| `ADCS::task_sendADCSParams` | 0.2 Hz | 5 s |
| `CAM::task_processCamera` | Adaptive | 5–100 ms |

#### Deployment Logic

The deployment burn wire is controlled by a three-state machine:

```
State 0 (idle)  →  CMD_DEPLOY received  →  State 1 (30s hold-down wait)
State 1          →  30 s elapsed         →  State 2 (burn wire active, 7 s)
State 2          →  7 s elapsed          →  State 0 (idle, wire off)
```

`autoDeploy` is `false` by default; deployment requires an explicit `CMD_DEPLOY` command from the ground.

#### Global Command Handlers

| Command | Handler | Description |
|---------|---------|-------------|
| `CMD_PING` | `ping()` | Echo test; returns received uint32 + fixed value 42 |
| `CMD_DEPLOY` | `deployTrigger()` | Initiates deployment state machine |
| `CMD_CTRL_5V` | `control5V()` | Toggle GPIO pin 42 (5V power rail) |

---

## Packet Protocol

All packets share a fixed 4-byte header followed by a variable payload:

```
┌──────────┬────────┬────┬────────┬─────────────────────────────────┐
│ Recipient│ Sender │ ID │ Length │ Payload (up to 251 bytes)        │
│  1 byte  │ 1 byte │1 B │ 1 byte │                                 │
└──────────┴────────┴────┴────────┴─────────────────────────────────┘
```

- All multi-byte payload values are **little-endian**.
- Max LoRa frame is 255 bytes, leaving 251 bytes for payload.
- The flight controller address is `0xAA`; ground station uses a distinct address; broadcast is `0xFF`.

---

## Command Reference

| ID  | Name              | Payload | Description |
|-----|-------------------|---------|-------------|
| `CMD_PING` | Ping | uint32 value | Echo test |
| `CMD_TAKE_PHOTO` | Take Photo | burst count + interval | Trigger camera burst |
| `CMD_ADCS_SETPOINT` | Set Attitude | float q_w, q_x, q_y, q_z | Set desired quaternion |
| `CMD_ADCS_ENABLE` | Enable ADCS | uint8 x, y, z | Enable/disable axes |
| `CMD_ADCS_ZERO` | Zero Wheels | — | Disable control, zero wheels |
| `CMD_ADCS_SET_PID` | Set PID | axis, Kp, Ki, Kd (floats) | Tune PID gains |
| `CMD_DEPLOY` | Deploy | — | Initiate antenna/sail deployment |
| `CMD_SET_CAMERA_RES` | Set Resolution | uint8 resolution | Change camera resolution |
| `CMD_CTRL_5V` | Control 5V Rail | uint8 state | Enable/disable 5V GPIO |

---

## Telemetry Reference

### HSK Telemetry (`HSK_TELEMETRY` = 100)

Housekeeping data (emitted by main.cpp).

### BMS Telemetry (`BMS_TELEMETRY` = 101) — 27 bytes

| Field | Type | Unit |
|-------|------|------|
| Cell 1–4 voltage | uint16 × 4 | mV |
| Stack voltage | uint16 | mV |
| Current | int32 | mA |
| Internal temperature | float | °C |
| Thermistor temperature | float | °C (NaN if out of range) |
| Uptime | uint32 | s |
| Last reset reason | uint8 | `esp_reset_reason_t` code |

### ADCS Telemetry (`ADCS_TELEMETRY` = 102) — 56 bytes

| Field | Type | Description |
|-------|------|-------------|
| q_des_w/x/y/z | float × 4 | Desired quaternion |
| q_w/x/y/z | float × 4 | Current quaternion |
| omega_x/y/z | float × 3 | Body angular rates (rad/s) |
| int_x/y/z | float × 3 | Integrator accumulators |

### ADCS Params (`ADCS_PARAMS` = 103) — 39 bytes

| Field | Type | Description |
|-------|------|-------------|
| Kp_x/y/z | float × 3 | Proportional gains |
| Ki_x/y/z | float × 3 | Integral gains |
| Kd_x/y/z | float × 3 | Derivative gains |
| enable_x/y/z | uint8 × 3 | Axis enable flags |

### Camera Packets

| ID | Name | Description |
|----|------|-------------|
| 200 | `CAM_IMAGE_META` | Image metadata (size, transfer ID, chunk count) |
| 201 | `CAM_IMAGE_DATA` | 246-byte image chunk |
| 202 | `CAM_IMAGE_DONE` | All chunks sent |
| 203 | `CAM_NACK` | Request retransmission of a chunk |
| 204 | `CAM_IMAGE_ACK` | Ground acknowledges successful transfer |

---

## Task Timing Summary

| Subsystem | Task | Rate | Period |
|-----------|------|------|--------|
| ADCS | Control loop | 50 Hz | 20 ms |
| ADCS | Telemetry | 1 Hz | 1 s |
| ADCS | PID params | 0.2 Hz | 5 s |
| BMS | Telemetry | 1 Hz | 1 s |
| CAM | Processing | Adaptive | 5–100 ms |
| Main | LED blink | 1 Hz | 1 s |
| Main | Deployment check | State machine | — |

---

## Hardware Pin Assignments

| Pin | Signal | Subsystem |
|-----|--------|-----------|
| 3 | LED1 | main |
| 46 | LED2 | main |
| 18 | DEPLOY_CTRL | main (burn wire) |
| 42 | CTRL_5V | main (5V rail) |
| 7 | IMU SCL | ADCS |
| 8 | IMU SDA | ADCS |
| 4 | BMS SDA | BMS |
| 48 | BMS SCL | BMS |
| 21 | CAM TX | CAM |
| 35 | CAM RX | CAM |
| 9 | LoRa RESET | RadioComms |
| 10 | LoRa CS | RadioComms |
| 11 | LoRa MOSI | RadioComms |
| 12 | LoRa MISO | RadioComms |
| 13 | LoRa SCK | RadioComms |
| 14 | LoRa EN | RadioComms |

---

## Dependencies

| Library | Used By |
|---------|---------|
| Arduino / ESP32 HAL | All |
| Adafruit BNO055 | ADCS |
| BQ76905 driver | BMS |
| LoRa | RadioComms |
| Wire (I2C) | ADCS, BMS |
| HardwareSerial | ADCS (wheels), CAM |
| SPI | RadioComms |
| `<map>` (C++ STL) | RadioComms |
