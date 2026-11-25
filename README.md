**English** | [繁體中文](README.zh-TW.md)

# TMCM-6110 TMCL Protocol & 3D Motion Controller Simulator (1–6 Axes)

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![3D Modes](https://img.shields.io/badge/3D%20Modes-1--6%20Axes%20%7C%2012%20Scenes-1976D2)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
![Firmware Spec](https://img.shields.io/badge/Firmware%20Spec-TMCM--6110%20FW%20V1.47%20(Rev%201.23)-FF6F00)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

> **Firmware Specification Notice:** This simulator now strictly follows **[`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf)** (TMCM-6110 TMCL Firmware V1.47 / Rev 1.23, reporting `6110V147` / `0x17DE012F` via `Cmd 136`). The legacy [`TMCL_reference.pdf`](./TMCL_reference.pdf) (V2.22) is retained in the repository for historical comparison only.

![TMCM-6110 3D Simulator Demo](./demo.gif)

▶ [Watch Demo Video](#demo-video) | [Direct MP4 (`demo_highlight.mp4`)](./demo_highlight.mp4)

## Why This Project?

Motion-control and host software development typically has to wait until physical controller boards, motor drives, or assembled machines arrive before integration testing can begin.

**TMCM-6110 3D Simulator** provides a standalone Windows **Trinamic (ADI) TMCM-6110** simulated controller with real-time 3D machine visualization:

> Host Software / Scripts → 9-Byte Binary TMCL Packets → TCP / Serial COM → TMCM-6110 Simulated Controller → 3D Machine Visualization

It lets developers verify and debug host integrations before hardware is ready:

- 9-Byte Binary TMCL packet framing and 8-bit additive checksum validation strictly aligned with [`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf) (`6110V147`)
- Common command workflows (`SAP`, `GAP`, `MVP`, `RFS`, `SCO`/`GCO`/`CCO`/`ACO`, `SIO`/`GIO`, `Cmd 138`, `Cmd 255`) and strict parameter range/read-only validation
- TCP / Serial COM master-slave communication, `GAP 8` status polling, `#87` secondary serial address, `#255` reply suppression, and pre-armed `Cmd 138` position-reached events
- 1–6 axis motion (32-bit positions, up to 3-axis `Bit 6` linear interpolation per command), limit/home switches, StallGuard2 (`#207` auto-clear) & TMC262 (`#208`) faults, and RFS homing sequences (`1..8`, `+64`, `+128`)
- Custom host UI, motion-control logic, or automated test scripts
- TMCL-IDE `.ini` / `.tmc` configuration loading and physical-channel / 3D role mapping

Built with C# / .NET 8.0 and native WPF as a Windows **Digital Twin & Virtual Commissioning** tool for early development, protocol integration, and demos; it **does not replace physical hardware verification** for electrical wiring, safety circuits, or real-machine dynamics.

## 30-Second Quick Start

### Option 1: Download Prebuilt Binary (No Compiler Needed)

1. Download the latest `win-x64` `.zip` from [GitHub Releases](https://github.com/DicoLian/tmcm-6110-3d-simulator/releases) and extract it.
2. Run `Tmcm6110.App.exe` to launch the 3D simulator (automatically starts the local TCP server on `127.0.0.1:4001`).
3. Run `Tmcm6110.HostTester.exe` and choose **`[2]`** (Local TCP `127.0.0.1:4001`, default) to watch the 3D gantry move immediately.

> **Note:** Because open-source release binaries are unsigned, Windows SmartScreen may show a warning on first launch—click **More info → Run anyway**.

### Option 2: Build & Run from Source

Requires Windows 10/11 and the [.NET 8.0 SDK](https://dotnet.microsoft.com/download/dotnet/8.0) (or Visual Studio 2022 with the **.NET desktop development** workload).

Build, run unit tests, and launch the simulator:

```powershell
dotnet build Tmcm6110Simulator.sln
dotnet test Tmcm6110Simulator.sln
dotnet run --project src/Tmcm6110.App
```

In a second terminal, run the host tester (choose `[2]` for Local TCP `127.0.0.1:4001`):

```powershell
dotnet run --project src/Tmcm6110.HostTester
```

Run the automated protocol self-test:

```powershell
dotnet run --project src/Tmcm6110.HostTester -- --self-test
```

- **Visual Studio 2022**: Open `Tmcm6110Simulator.sln`, set `Tmcm6110.App` as the startup project, and press `F5`.

## Core Features

### Communication & Protocol

- Full 9-Byte Binary TMCL packet parsing/assembly with 8-bit additive checksum validation
- Dual transport via Serial COM (`com0com` / VSPD, up to `230400` baud) and zero-driver Local TCP `127.0.0.1:4001`
- Real-time RX/TX 9-byte hex Packet Sniffer with mnemonic disassembly and high-frequency polling filter (`GAP`/`GGP`/`GIO`/`RFS STATUS`)
- Sliding-window packet synchronization, malformed-packet detection, configurable `ModAddr (#66)` / `HostAddr (#76)`, Secondary Serial Address (`#87`), and Suppress Reply (`#255`)
- Supports both `GAP 8` polling and FW V1.47 pre-armed `Cmd 138` asynchronous position-reached events (`Type=0` next move one-shot / `Type=1` continuous; `Status=100` ACK + unsolicited `Status=128`)
- Retains all 6 internal axis registers (`Axis 0..5`) even when UI is configured for 1–5 active axes so multi-axis host polling never times out

### Simulated Motion Control

- Full **32-bit** signed position (`-2,147,483,648 .. 2,147,483,647`), signed `ROR`/`ROL` (`-2047..2047`), and **Soft Stop** `MST` (ramping down cleanly using `MaxAcceleration #5`)
- Supports 1–6 simulated axes with absolute (`MVP ABS`), relative (`MVP REL`), and coordinated moves (`MVP COORD`: `Bit 6 / 0x40` linear interpolation up to 3 axes with scaled speed & accel; `Bit 7 / 0x80` unlimited non-interpolated multi-axis start; 3D Interp Demo automatically splits `>3` axes into 3-axis groups)
- Supports FW V1.47 reference search homing (`RFS` Modes `1..8`, `+64`, `+128` with `#194/#195` `0..2047` speeds and `#196/#197` calibration readouts), soft/virtual limit switches (`#9/#10/#11`, `#12/#13`, `#149` soft-stop flag), and alarm states
- Supports microstep (`#140` `0..8`), current (`#6/#7` `0..255`), actual accel (`#135`), SmartEnergy actual current (`#180`), velocity, acceleration, and `Steps/mm` travel scaling
- Real-time bijective swapping of physical motor channels (`Motor #0..#5`) and 3D machine roles (with automatic stroke-limit transfer) plus `Default Z-Y-X` / `Standard X-Y-Z` presets
- Manual forced `±Limit` and StallGuard2 (`#207`, auto-clearing on `GAP 207` read or motion) / fatal TMC262 (`#208`) alarm injection (`CLE` clears standalone flags `0..5`; fatal shutdown requires fault clear / `Cmd 255` reset)

### 3D Visualization & Machine Configuration

- Native WPF `Viewport3D` with zero third-party 3D dependencies, featuring 3D limit-switch blocks and golden Tool Center Point (TCP) trails
- 12 built-in machine & gantry scenes filtered by active axis count (`1..6`), with mouse orbit/zoom and camera reset
- 5 XY origin presets & 3 Z origin presets (with optional `RFS #193` direction sync), plus per-axis `-Limit / +Limit (mm)` travel bounds
- Auto-detects Windows locale on startup with a runtime English / Traditional Chinese UI switch
- Loads TMCL-IDE `.ini`, `.tmc`, and axis-mapping `.txt` configuration files

---

## Demo Video

- Watch the highlight clip (~27s, 1.6 MB): [`demo_highlight.mp4`](./demo_highlight.mp4) *(captures the classic 3-axis gantry view prior to the 1–6 axis and 12-mode upgrade)*

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

## Supported Scope & Limitations

This project targets the **TMCM-6110 / TMCL 9-Byte Binary** protocol per **[`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf)** (Firmware **V1.47 / Rev 1.23**), simulating the controller behavior needed for development, communication debugging, and host workflow testing.

Implemented commands and key parameters include:

| Category | Supported Commands / Parameters (FW V1.47) |
| :--- | :--- |
| **Motion** | Signed `ROR (1)` / `ROL (2)` (`-2047..2047`), Soft-Stop `MST (3)`, 32-bit `MVP (4)` (`ABS`, `REL`, `COORD` `0x40\|mask` up to **3-axis** linear interpolation, `0x80\|mask` unlimited multi-axis start) |
| **Parameters** | Strictly validated `SAP (5)`, `GAP (6)`, `STAP (7)`, `RSAP (8)`, `SGP (9)`, `GGP (10)`, `STGP (11)`, `RSGP (12)` (`SAC 29` removed; Bank 0/2/3 enforced) |
| **Reference Search** | `RFS (13)` (`START`, `STOP`, `STATUS`) with Modes `1..8`, `+64`, `+128`, `#194/#195` (`0..2047`), `#196` EndSwitchDistance, `#197` LastReferencePosition |
| **I/O & Coordinates** | `SIO (14)` / `GIO (15)` (`0..7` DI & DO, 12-bit `AIN0(0)`/`AIN4(4)`, `Port 8` supply voltage `240` = 24.0 V, `SIO 0,0,0..3` pull-ups); Coordinates `0..20` (`SCO 30`, `GCO 31`, `CCO 32`, `ACO 39`, `255` batch copy, `#84`) |
| **System & Events** | `CLE (36)` (standalone flags `0..5`), `Cmd 128..133` app control, `GetAppStatus (135)` (`Types 0..3`), `GetFirmwareVersion (136)` (`"6110V147"` / `0x17DE012F`), `RestoreFactorySettings (137)` (`1234`, no reply), Pre-armed `RequestTargetPositionReachedEvent (138)` (`Type 0/1`), `SoftwareReset (255)` (`1234`) |
| **Key Axis / Global Params** | Axis (Table 16): 32-bit Pos (`#0/#1`), Speed/Accel (`#2/#3/#4/#5/#135`), Current (`#6/#7` `0..255`), InPos (`#8`), Home/Limit Switches (`#9/#10/#11`, `#12/#13`), Microstep (`#140` `0..8`), SoftStopFlag (`#149`), CoolStep/SG2 (`#180/#181/#206`), Auto-clear `#207`, TMC262 `#208`; Global: `Baud (#65)`, `ModAddr (#66)`, `HostAddr (#76)`, `SecAddr (#87)`, `Random (#133)`, `SuppressReply (#255)`, Bank 2 (`0..255`), Bank 3 (`0..2`, `27..46`) |

> **Compatibility Note for Legacy `TMCL_reference.pdf` (V2.22) Hosts:** Compared to legacy V2.22 behavior, FW V1.47 enforces **pre-MVP** arming for `Cmd 138`, soft-decelerating `MST`, max 3-axis `Bit 6` interpolation, `"6110V147"` / `0x17DE012F` version IDs, `#207` auto-clear on read, `CLE` restricted to standalone flags, `0..20` coordinates, and rejects obsolete opcodes/parameters (`SAC 29`, `#64`, `#73`, `#141/#142/#147/#151/#205/#209..#212`).

### Limitations & Scope Boundaries

- **Protocol & Logic Simulation**: Simulates TMCL protocol handling and control logic, not a physical stepper drive stage or mechanical load system.
- **No Electrical or Safety Verification**: Does not cover electrical wiring, safety circuits (E-Stop / Safety PLC), or hardware over-current/thermal protection verification.
- **Unimplemented Commands**: TMCL commands or advanced firmware parameters not listed above should not be assumed to behave identically to real firmware.
- **Hardware Integration Required**: Physical TMCM-6110 boards, specific firmware revisions, and machine wiring must still be validated during hardware commissioning.
- **Simplified Behavior Details**: Certain motion profiles and fault behaviors are simplified for development and testing and may differ in detail from physical hardware.

## 1–6 Active Axes & 12 Built-In 3D Machine Modes

Select **Active Axes (`1..6`)** in **Tab 2 (`Machine & System Settings`)** to dynamically resize the active axis cards in Tab 1 and filter the available **3D Machine Mode** options:

| Active Axes | Available 3D Machine Modes | Logical Axis / Role Assignment (`Role 0..N-1`) |
| :--- | :--- | :--- |
| **1 Axis** | `1-Axis Linear Stage (X)`<br>`1-Axis Rotary Table (R)`<br>`1-Rail Linear Test Bench (Ch 0)` | `X` linear carriage (`±Limit` & `0 mm` origin)<br>`R` rotary indexing platter (`±180°` span)<br>`Ch 0` dynamic linear rail |
| **2 Axes** | `2-Axis XY Stage (X/Y)`<br>`2-Axis XZ Gantry (X/Z)`<br>`2-Rail Linear Test Bench (Ch 0–1)` | `X` / `Y` planar positioning gantry<br>`Z` vertical spindle / `X` crossbeam<br>`Ch 0..1` dynamic parallel rails |
| **3 Axes** | `3-Axis Gantry (Default Z/Y/X)` *(default)*<br>`3-Axis XY + Rotary Table (X/Y/R)`<br>`3-Rail Linear Test Bench (Ch 0–2)` | `Z` (Blue) / `Y` (Green) / `X` (Red) gantry<br>`R` rotary platter / `Y` bridge / `X` carriage<br>`Ch 0..2` dynamic parallel rails |
| **4 Axes** | `4-Axis Gantry + Rotary Spindle (XYZ+R)`<br>`4-Rail Linear Test Bench (Ch 0–3)` | `Z` / `Y` / `X` + `R` rotary spindle chuck<br>`Ch 0..3` dynamic parallel rails |
| **5 Axes** | `5-Axis Gantry + Dual Rotary (XYZ+R/A)`<br>`5-Rail Linear Test Bench (Ch 0–4)` | `Z` / `Y` / `X` + `R` spindle + `A` rotary table<br>`Ch 0..4` dynamic parallel rails |
| **6 Axes** | `6-Axis Dual Gantry (Stations 1 & 2)`<br>`6-Axis Gantry + Rotary/Aux (XYZ + R/A/B)`<br>`6-Axis Robot Arm (J1–J6)`<br>`6-Rail Linear Test Bench (Ch 0–5)` | `Z1/Y1/X1` + `Z2/Y2/X2` dual stations<br>`Z/Y/X` + `R` spindle + `A` table + `B` aux feeder<br>`J1..J6` articulated 6-DOF robot arm (FK trail)<br>`Ch 0..5` dynamic parallel rails |

> **Internal 6-Axis Register Retention**: Even when configured for 1–5 active axes in the UI, `Axis 0..5` remain active in the TMCL protocol engine so multi-axis host startup and polling never fail.

<details>
<summary><strong>Expand Full UI Workspace & Interactive Controls Guide (Top Bar, 3D Viewport, Tab 1/2, Packet Sniffer)</strong></summary>

1. **Top Connection & Motion Bar**:
   - **Serial COM Controls**: COM port selector, **`Rescan`**, baud rate (`9600`–`230400`), **`Open Serial Port`**, and live Serial / TCP listening status indicator.
   - **Quick Global Actions**: **`3D Interp Demo (MVP COORD)`** (arming `Cmd 138` before grouped `<=3`-axis `MVP COORD` sweeps), **`Home All (RFS)`**, and **`Stop All (MST)`**.
2. **Left 3D Machine Viewport (`Viewport3D`)**:
   - Orbit camera with **Left/Right-button drag** and **scroll wheel zoom**, plus **`Clear 3D Trail`** and **`Reset Camera`** buttons.
   - Dynamic top-left legend overlay and bottom-left live coordinate readout adapted to the active machine scene and motor-channel mapping.
3. **Right Panel — Tab 1 (`Axes Monitor & Control`)**:
   - Vertically scrollable card list displaying all currently active axes (`1..6`).
   - **Dual Channel / Role Selectors**: Swap the physical TMCM-6110 motor channel (`Motor #0..#5`) and logical 3D role in place (bijective swap transfers configured travel bounds; role selector automatically disables when only 1 axis is active).
   - **Telemetry, Manual Motion & Fault Controls**: Status badge + 5 LEDs (`InPos #8`, `MOVE`, `+Limit #10`, `-Limit #11`, `ALARM #207/#208`), positions/speed/microstep/RFS readouts, **`MVP ABS`**, **`MVP REL +/-`**, **`ROL`/`ROR`** jog, **`■ MST Stop`**, **`RFS Home`**, **`Pos = 0`**, forced `±Limit` checkboxes, and StallGuard2/TMC262 alarm injection.
4. **Right Panel — Tab 2 (`Machine & System Settings`)**:
   - **3D Machine Mode & Origin Position**: Select **Active Axes (`1..6`)**, filtered **3D Machine Mode**, **XY Machine Origin** (`Center`, `Front-Left`, `Back-Left`, `Front-Right`, `Back-Right`), **Z-Axis Origin** (`Center`, `Top`, `Bottom`), optional `RFS (#193)` direction sync, and **`Default Z-Y-X`** / **`Standard X-Y-Z`** presets (`Standard X-Y-Z` active when `>= 3` axes).
   - **Per-Axis Travel Stroke & Limit Stop (mm)**: Edit `Steps/mm`, per-axis `-Limit` / `+Limit` positions (mm, showing converted microstep bounds), and `#12/#13` hardware limit-stop enable checkboxes.
   - **TMCL Node Addressing & Transport**: Edit `ModAddr(#66)` and `HostAddr(#76)` and monitor local TCP listening status (`127.0.0.1:4001`).
   - **Configuration File & UI Language**: **`Load .ini / .tmc`** configuration button, loaded summary banner, and **`🌐 EN / 繁中`** runtime language switch.
5. **Bottom 9-Byte Binary Packet Sniffer**:
   - Real-time RX/TX 9-byte hex log with TMCL mnemonic disassembly, **Total RX Packets** counter, polling packet filter (`GAP`/`GGP`/`GIO`/`RFS STATUS`), **`Clear All Alarms`**, and **`Clear Log`**.

</details>

## Connecting Your Host & Custom Configuration

### Connecting Your Host Application

1. **Via Virtual Serial Port (`com0com` / VSPD)**:
   - Create a linked COM port pair (e.g., `COM10 <===> COM11`).
   - In the **top bar** of `Tmcm6110.App`, select `COM11` and baud rate (e.g., `9600` or `115200`), verify `ModAddr(#66)=1` and `HostAddr(#76)=2` in **Tab 2 (`Machine & System Settings`)**, and click **`Open Serial Port`**.
   - Point your host software (or `Tmcm6110.HostTester`) to `COM10`.
2. **Via Local TCP (`127.0.0.1:4001`)**:
   - Run `Tmcm6110.HostTester` (or `dotnet run --project src/Tmcm6110.HostTester`) and choose option **`[2]`** to test commands immediately without installing virtual COM drivers.

### Custom Configuration Loading (`.ini` / `.tmc` / Axis Mapping `.txt`)

- Open **Tab 2 (`Machine & System Settings`) → `Load .ini / .tmc`** to load TMCL-IDE Setup Wizard `.ini` exports, `.tmc` (`SAP`/`SGP`) scripts, or axis-mapping `.txt` files at runtime (startup also auto-detects matching `.ini` and axis-mapping `.txt` files in the application/parent directories).
- Axis-mapping `.txt` files match lines containing `axis <n> (...) <LABEL>` (for example `axis 0 (1st Axis) Z`, `axis 1 (2nd Axis) Y`, `axis 2 (3rd Axis) X`) and automatically synchronize role slots for the first three axes when `Z`/`Y`/`X` labels are present.
- Ready-to-try public sample configurations are included in the repository:
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — Generic 3-axis gantry parameter preset (`[module_settings_axis%0..2]` & `[global_settings]`).
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — Generic `SAP` / `SGP` initialization script.
- Note: Private `.ini`, `.tmc`, and machine-specific `.txt` mapping files remain excluded in `.gitignore` (only `samples/*.ini` and `samples/*.tmc` are tracked) so production parameters stay local.

## Project Architecture & Workflow

```text
src/
├── Tmcm6110.Core/
│   ├── Abstractions/       IMotionController, IMotionAxis, IProtocolAdapter, Telemetry
│   ├── Protocol/           TmclProtocolAdapter, TmclPacket, TmclDefinitions
│   ├── Transport/          ByteStreamTransportServer (TCP / Serial COM), TmclSerialServer
│   ├── Engine/             Tmcm6110Controller, AxisState, AxisRoleMap
│   └── Config/             TmcmConfigLoader (.ini / .tmc / axis-map .txt)
├── Tmcm6110.App/
│   ├── ViewModels/         MainViewModel, AxisViewModel
│   ├── Rendering/          IMachine3DScene, Gantry3DSceneBuilder & Scenes/ (12 scenes)
│   └── Resources/          Strings.en-US.xaml, Strings.zh-TW.xaml & Localization/
└── Tmcm6110.HostTester/    Console host client & automated protocol self-test (--self-test)

samples/                    Public sample presets (demo_gantry.ini, demo_gantry.tmc)
tests/
└── Tmcm6110.Core.Tests/    xUnit tests (packet, controller, limits/alarms, role map, config, decoupling)
```

`ByteStreamTransportServer` exchanges raw byte streams over TCP or Serial COM, while `TmclProtocolAdapter` parses and dispatches 9-byte binary TMCL frames to `Tmcm6110Controller` to update motion state. The WPF ViewModels read live position, speed, limit, and alarm telemetry from the controller and drive `IMachine3DScene` to render the active 3D scene.

## Custom Development & Commercial Inquiries

Looking for custom industrial software, virtual commissioning tools, or motion-control solutions? Feel free to reach out for project inquiries or consulting:

- **Email**: [dicokabn@gmail.com](mailto:dicokabn@gmail.com)
- **Services**:
  - Custom **C# / .NET (WPF)** host software & industrial HMI development
  - **Motion Controller Simulators & Virtual Commissioning** (Trinamic, Delta, Advantech, Galil, Modbus / EtherCAT, etc.)
  - Custom **Serial / CAN / TCP** protocol drivers & 3D Digital Twin visualization

## License & Third-Party Notice

- This simulator's source code is licensed under the [MIT License](./LICENSE).
- [`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf) and [`TMCL_reference.pdf`](./TMCL_reference.pdf) are copyright **Analog Devices / Trinamic** and are included solely as protocol reference documentation.
- **Disclaimer**: This is an independent, unofficial open-source project and is **not** affiliated with, endorsed by, or sponsored by **Analog Devices, Inc.** or **Trinamic**. `TMCM-6110`, `TMCL`, `StallGuard2`, and `TMC262` are trademarks or product names of their respective owners.
