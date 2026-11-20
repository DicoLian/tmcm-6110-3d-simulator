**English** | [繁體中文](README.zh-TW.md)

# TMCM-6110 3-Axis Motion Controller 3D Simulator

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

![TMCM-6110 3D Simulator Demo](./demo.gif)

## What It Is

A standalone Windows **Digital Twin & Virtual Commissioning 3D Simulator** for the **Trinamic (ADI) TMCM-6110** 6-axis stepper motor controller, built with C# and .NET 8.0. It lets motion-control and host software developers develop, test, and verify **9-Byte Binary TMCL** applications without physical hardware or collision risk—complete with a native WPF 3D XYZ gantry visualizer, live Packet Sniffer, and interactive hardware fault injection.

## Key Highlights

- **9-Byte Binary TMCL Engine**: Full framing, 8-bit additive checksum validation, and sliding-window synchronization per [`TMCL_reference.pdf`](./TMCL_reference.pdf), with all 6 axes (`Axis 0..5`) retained internally so multi-axis polling never times out.
- **Dual Transport (Serial COM + Zero-Driver TCP)**: Connects seamlessly over virtual COM port pairs (`com0com` `COM10 <-> COM11`) or the built-in local TCP server (`127.0.0.1:4001`) that starts automatically.
- **Native WPF `Viewport3D` Gantry**: Zero third-party 3D dependencies; renders real-time Z/Y/X motion, 3D limit switch status blocks, orbit camera controls, and a golden 3D **Tool Center Point (TCP)** trajectory trail.
- **Async Events & Hardware Fault Injection**: Supports `GAP 8` polling and `Cmd 138` asynchronous position-reached push (`Status=100` ACK + unsolicited `Status=128`), virtual/forced limit switches (`+Limit #10` / `-Limit #11` / `#9`), and StallGuard2 (`#207`) / TMC262 (`#208`) alarm injection with `CLE (Cmd 36)` recovery.
- **Live Packet Sniffer & Bilingual UI**: Real-time RX/TX 9-byte hex viewer with TMCL mnemonic disassembly, polling packet filter, and a one-click **`🌐 EN / 繁中`** toolbar toggle.

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
dotnet run --project src/Tmcm6110.App
# In a second terminal, run the host tester and choose [2] (Local TCP 127.0.0.1:4001):
dotnet run --project src/Tmcm6110.HostTester
```

- **Visual Studio 2022**: Open `Tmcm6110Simulator.sln`, set `Tmcm6110.App` as the startup project, and press `F5`.
- **Automated Self-Test**:
  ```powershell
  dotnet run --project src/Tmcm6110.HostTester -- --self-test
  ```

---

## Demo Video

- Watch the full highlight clip (~27s, 1.6 MB): [`demo_highlight.mp4`](./demo_highlight.mp4)

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

## Architecture & Feature Details

1. **Default 3-Axis Gantry Mapping (with 6-Axis Register Retention)**:
   - **Motor #0 (`Axis 0`) = Z-Axis** (Vertical spindle, Blue)
   - **Motor #1 (`Axis 1`) = Y-Axis** (Front/Back gantry bridge, Green)
   - **Motor #2 (`Axis 2`) = X-Axis** (Left/Right carriage, Red)
   - `Axis 3..5` remain active in the protocol engine so host initialization and polling across all 6 axes succeed without timeouts.
2. **Motion States, Async Events & Fault Injection**:
   - **Position Reached**: Supports both polling (`GAP 8, <axis>`) and **Command `138`** asynchronous push notifications (immediate `Status=100` ACK followed by an unsolicited `Status=128 (0x80)` packet upon arrival).
   - **Virtual & Forced Limit Switches (`+Limit #10` / `-Limit #11` / `#9`)**: Automatic trip beyond virtual travel bounds (`±100,000 steps` default) or manual UI override checkboxes.
   - **Alarm Fault Injection (`#207` StallGuard2 / `#208` TMC262 Driver Error)**: One-click alarm injection and immediate emergency stop to verify host recovery (`CLE`, Cmd `36`).
3. **Live 9-Byte Packet Sniffer & Bilingual UI**:
   - Real-time RX/TX Hex viewer with automatic TMCL mnemonic disassembly and optional polling packet filter (`GAP`, `GGP`, `GIO`, `RFS STATUS`).
   - Auto-detects Windows UI culture on startup and includes a one-click **`🌐 EN / 繁中`** toggle in the toolbar.

## Supported TMCL Commands & Parameters

| Category | Supported Commands / Features |
| :--- | :--- |
| **Motion** | `ROR (1)`, `ROL (2)`, `MST (3)`, `MVP (4)` (`ABS`, `REL`, `COORD` incl. `$47` 3-axis linear interpolation) |
| **Parameters** | `SAP (5)`, `GAP (6)`, `STAP (7)`, `RSAP (8)`, `SGP (9)`, `GGP (10)`, `STGP (11)`, `RSGP (12)` |
| **Reference Search** | `RFS (13)` (`START`, `STOP`, `STATUS`), including Mode `65` (`+Limit` homing) |
| **I/O & Coordinates** | `SIO (14)`, `GIO (15)`, `SCO (30)`, `GCO (31)`, `CCO (32)` |
| **System & Events** | `CLE (36)`, `GetAppStatus (135)`, `GetFirmwareVersion (136)`, `RequestTargetPositionReachedEvent (138)` |

## Connecting Your Host Application

1. **Via Virtual Serial Port (`com0com` / VSPD)**:
   - Create a linked COM port pair (e.g., `COM10 <===> COM11`).
   - In `Tmcm6110.App`, select `COM11`, set baud rate (e.g., `9600` or `115200`), verify `ModAddr=1` and `HostAddr=2`, and click **Open Serial Port**.
   - Point your host software (or `Tmcm6110.HostTester`) to `COM10`.
2. **Via Local TCP (`127.0.0.1:4001`)**:
   - Run `Tmcm6110.HostTester` (or `dotnet run --project src/Tmcm6110.HostTester`) and choose option `[2]` to test commands immediately without installing virtual COM drivers.

## Custom Configuration Loading (`.ini` / `.tmc` / Axis Mapping `.txt`)

- Load TMCL-IDE Setup Wizard `.ini` exports, `.tmc` (`SAP`/`SGP`) scripts, or axis-mapping `.txt` files at runtime via the **Load .ini / .tmc** toolbar button.
- Ready-to-try public sample configurations are included in the repository:
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — Generic 3-axis gantry parameter preset (`[module_settings_axis%0..2]` & `[global_settings]`).
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — Generic `SAP` / `SGP` initialization script.
- Note: Private `.ini`, `.tmc`, and machine-specific axis-mapping files remain excluded in `.gitignore` (only `samples/*.ini` and `samples/*.tmc` are tracked) so production parameters stay local.

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
