**English** | [繁體中文](README.zh-TW.md)

# TMCM-6110 Motion Controller 3D Simulator (1–6 Axes)

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![3D Modes](https://img.shields.io/badge/3D%20Modes-1--6%20Axes%20%7C%2012%20Scenes-1976D2)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

![TMCM-6110 3D Simulator Demo](./demo.gif)

## What It Is

A standalone Windows **Digital Twin & Virtual Commissioning 3D Simulator** for the **Trinamic (ADI) TMCM-6110** 6-axis stepper motor controller, built with C# and .NET 8.0. It lets motion-control and host software developers develop, test, and verify **9-Byte Binary TMCL** applications without physical hardware or collision risk—complete with a configurable **1–6 axis** control dashboard, **12 native WPF 3D machine scenes**, live Packet Sniffer, dynamic motor/role mapping, and interactive fault injection.

## Key Highlights

- **9-Byte Binary TMCL Engine**: Full framing, 8-bit additive checksum validation, and sliding-window synchronization per [`TMCL_reference.pdf`](./TMCL_reference.pdf). All 6 axes (`Axis 0..5`) remain active internally regardless of the selected UI axis count so multi-axis host polling never times out.
- **1–6 Active Axes & 12 Native WPF `Viewport3D` Scenes**: Zero third-party 3D dependencies; switch between **1, 2, 3, 4, 5, or 6 active axes** in **Tab 2 (`Machine & System Settings`)** to filter and preview 12 machine kinematics (linear stages, rotary tables, XY/XZ gantries, 3-axis gantry + rotary, 4/5-axis rotary machines, dual-station 6-axis gantry, 6-DOF robot arm, and a dynamic 1–6 rail linear test bench) with 3D limit switch blocks and golden Tool Center Point (TCP) trails.
- **Live Physical Motor Channel (`Motor #0..#5`) & 3D Role Swapping**: Each axis card in **Tab 1 (`Axes Monitor & Control`)** provides dual dropdowns to reassign both the physical controller channel (`Motor #0..#5`) and its 3D machine role in real time—with automatic bijective swapping, stroke-limit transfer, and quick **`Default Z-Y-X`** / **`Standard X-Y-Z`** mapping presets in Tab 2.
- **Origin Presets, Custom Travel Strokes & Limit Stop**: Configure 5 XY origin presets (Table Center + 4 corners) and 3 Z origin presets (Center / Top / Bottom) with optional `RFS (#193)` homing-direction sync, per-axis `Min/Max (mm)` stroke bounds, `Steps/mm` scaling, and `#12/#13` limit-switch auto-stop toggles.
- **Dual Transport & Fault Injection**: Connects via virtual COM port pairs (`com0com` `COM10 <-> COM11`) or the built-in zero-driver TCP server (`127.0.0.1:4001`), supporting `GAP 8` polling, `Cmd 138` asynchronous position-reached events (`Status=100` ACK + unsolicited `Status=128`), virtual/forced limit switches (`+Limit #10` / `-Limit #11` / `#9`), and StallGuard2 (`#207`) / TMC262 (`#208`) alarm injection with `CLE (Cmd 36)` recovery.
- **Tabbed Bilingual UI & Decoupled Extensible Core**: Auto-detects Windows culture on startup with a **`🌐 EN / 繁中`** toggle in **Tab 2 (`Machine & System Settings`)**, backed by decoupled motion (`IMotionController`/`IMotionAxis`), protocol (`IProtocolAdapter`/`ByteStreamTransportServer`), and 3D scene (`IMachine3DScene`) abstractions.

## Quick Start (Zero Hardware / 3 Steps)

### Option A — Prebuilt Binary (No Compiler Needed)

1. Download the latest `win-x64` `.zip` from [GitHub Releases](https://github.com/DicoLian/tmcm-6110-3d-simulator/releases) and extract it.
2. Double-click `Tmcm6110.App.exe` to launch the 3D simulator (Local TCP `127.0.0.1:4001` starts automatically).
3. Run `Tmcm6110.HostTester.exe` and choose **`[2]`** (Local TCP `127.0.0.1:4001`) to watch the 3D gantry move immediately.

> **Note:** Because open-source release binaries are unsigned, Windows SmartScreen may show a warning on first launch—click **More info → Run anyway**.

### Option B — Build & Run from Source

Requires Windows 10/11 and the [.NET 8.0 SDK](https://dotnet.microsoft.com/download/dotnet/8.0) (or Visual Studio 2022 with the **.NET desktop development** workload):

```powershell
dotnet build Tmcm6110Simulator.sln
dotnet test Tmcm6110Simulator.sln
dotnet run --project src/Tmcm6110.App
# In a second terminal, run the host tester and choose [2] (Local TCP 127.0.0.1:4001):
dotnet run --project src/Tmcm6110.HostTester
```

- **Visual Studio 2022**: Open `Tmcm6110Simulator.sln`, set `Tmcm6110.App` as the startup project, and press `F5`.
- **Automated Protocol Self-Test**:
  ```powershell
  dotnet run --project src/Tmcm6110.HostTester -- --self-test
  ```

---

## Demo Video

- Watch the highlight clip (~27s, 1.6 MB): [`demo_highlight.mp4`](./demo_highlight.mp4) *(captures the classic 3-axis gantry view prior to the 1–6 axis and 12-mode upgrade)*

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

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

## UI Workspace & Interactive Controls

1. **Top Connection & Motion Bar**:
   - **Serial COM Controls**: COM port selector, **`Rescan`**, baud rate (`9600`–`115200`), **`Open Serial Port`**, and live Serial / TCP listening status indicator.
   - **Quick Global Actions**: **`3D Interp Demo (MVP COORD)`** (interpolated waypoint sweep across active axes), **`Home All (RFS)`**, and **`Stop All (MST)`**.
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
   - Real-time RX/TX 9-byte hex log with TMCL mnemonic disassembly, **Total RX Packets** counter, polling packet filter (`GAP`/`GGP`/`GIO`/`RFS STATUS`), **`Clear All Alarms (CLE)`**, and **`Clear Log`**.

## Supported TMCL Commands & Key Parameters

| Category | Supported Commands / Parameters |
| :--- | :--- |
| **Motion** | `ROR (1)`, `ROL (2)`, `MST (3)`, `MVP (4)` (`ABS`, `REL`, `COORD` `0x40 \| mask` across active axes up to 6 axes, incl. `$47` 3-axis interpolation) |
| **Parameters** | `SAP (5)`, `GAP (6)`, `STAP (7)`, `RSAP (8)`, `SGP (9)`, `GGP (10)`, `STGP (11)`, `RSGP (12)` |
| **Reference Search** | `RFS (13)` (`START`, `STOP`, `STATUS`), including Mode `65` (`+Limit` homing) and Mode `1` (`-Limit` homing) |
| **I/O & Coordinates** | `SIO (14)`, `GIO (15)`, `SCO (30)`, `GCO (31)`, `CCO (32)` |
| **System & Events** | `CLE (36)`, `GetAppStatus (135)`, `GetFirmwareVersion (136)`, `RequestTargetPositionReachedEvent (138)` |
| **Key Axis / Global Params** | Axis: Target/Actual Pos (`#0/#1`), Max Speed/Accel (`#4/#5`), InPos (`#8`), End Switches (`#9/#10/#11`), Limit Disable (`#12/#13`), Microstep (`#140`), RFS Mode (`#193`), SG2 Load & Alarms (`#206/#207/#208`); Global: `ModAddr (#66)`, `HostAddr (#76)` |

## Connecting Your Host Application

1. **Via Virtual Serial Port (`com0com` / VSPD)**:
   - Create a linked COM port pair (e.g., `COM10 <===> COM11`).
   - In the **top bar** of `Tmcm6110.App`, select `COM11` and baud rate (e.g., `9600` or `115200`), verify `ModAddr(#66)=1` and `HostAddr(#76)=2` in **Tab 2 (`Machine & System Settings`)**, and click **`Open Serial Port`**.
   - Point your host software (or `Tmcm6110.HostTester`) to `COM10`.
2. **Via Local TCP (`127.0.0.1:4001`)**:
   - Run `Tmcm6110.HostTester` (or `dotnet run --project src/Tmcm6110.HostTester`) and choose option **`[2]`** to test commands immediately without installing virtual COM drivers.

## Custom Configuration Loading (`.ini` / `.tmc` / Axis Mapping `.txt`)

- Open **Tab 2 (`Machine & System Settings`) → `Load .ini / .tmc`** to load TMCL-IDE Setup Wizard `.ini` exports, `.tmc` (`SAP`/`SGP`) scripts, or axis-mapping `.txt` files at runtime (startup also auto-detects matching `.ini` and axis-mapping `.txt` files in the application/parent directories).
- Axis-mapping `.txt` files match lines containing `axis <n> (...) <LABEL>` (for example `axis 0 (1st Axis) Z`, `axis 1 (2nd Axis) Y`, `axis 2 (3rd Axis) X`) and automatically synchronize role slots for the first three axes when `Z`/`Y`/`X` labels are present.
- Ready-to-try public sample configurations are included in the repository:
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — Generic 3-axis gantry parameter preset (`[module_settings_axis%0..2]` & `[global_settings]`).
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — Generic `SAP` / `SGP` initialization script.
- Note: Private `.ini`, `.tmc`, and machine-specific `.txt` mapping files remain excluded in `.gitignore` (only `samples/*.ini` and `samples/*.tmc` are tracked) so production parameters stay local.

## Decoupled Architecture & Project Layout

- **`src/Tmcm6110.Core`**: Hardware-agnostic motion and protocol simulation engine:
  - `Abstractions/`: `IMotionController`, `IMotionAxis`, `IProtocolAdapter`, `MotionAxisTelemetry`, `ProtocolTraceEvent`
  - `Protocol/`: `TmclProtocolAdapter` (9-byte binary parser/dispatcher), `TmclPacket`, `TmclDefinitions`
  - `Transport/`: `ByteStreamTransportServer` (Serial COM + TCP multiplexer) & `TmclSerialServer` facade
  - `Engine/` & `Config/`: `Tmcm6110Controller`, `AxisState`, `AxisRoleMap` (bijective channel/role mapper), `TmcmConfigLoader`
- **`src/Tmcm6110.App`**: Native WPF application cleanly separated from core protocol logic:
  - `Rendering/`: `IMachine3DScene`, `SceneDescriptor`, `Axis3DFrame`, `Mesh3DPrimitives`, `Gantry3DSceneBuilder` (scene host/registry), plus all 12 modular scene classes under `Rendering/Scenes/`
  - `ViewModels/` & `Resources/`: `MainViewModel`, `AxisViewModel`, `Strings.en-US.xaml`, `Strings.zh-TW.xaml`
- **`src/Tmcm6110.HostTester`**: Console host client & automated protocol self-test (`--self-test`).
- **`tests/Tmcm6110.Core.Tests`**: xUnit test suite (`TmclPacketTests`, `Tmcm6110ControllerTests`, `TmcmConfigLoaderTests`, `AxisLimitsAndAlarmsTests`, `AxisRoleMapTests`, `ProtocolDecouplingTests`).

## Custom Development & Commercial Inquiries

Looking for custom industrial software, virtual commissioning tools, or motion-control solutions? Feel free to reach out for project inquiries or consulting:

- **Email**: [dicokabn@gmail.com](mailto:dicokabn@gmail.com)
- **Services**:
  - Custom **C# / .NET (WPF)** host software & industrial HMI development
  - **Motion Controller Simulators & Virtual Commissioning** (Trinamic, Delta, Advantech, Galil, Modbus / EtherCAT, etc.)
  - Custom **Serial / CAN / TCP** protocol drivers & 3D Digital Twin visualization

## License & Third-Party Notice

- This simulator's source code is licensed under the [MIT License](./LICENSE).
- [`TMCL_reference.pdf`](./TMCL_reference.pdf) is copyright **Analog Devices / Trinamic** and is included solely as protocol reference documentation.
- **Disclaimer**: This is an independent, unofficial open-source project and is **not** affiliated with, endorsed by, or sponsored by **Analog Devices, Inc.** or **Trinamic**. `TMCM-6110`, `TMCL`, `StallGuard2`, and `TMC262` are trademarks or product names of their respective owners.
