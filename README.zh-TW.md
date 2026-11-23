[English](README.md) | **繁體中文**

# TMCM-6110 多軸運動控制器 3D 模擬器（1～6 軸）

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![3D Modes](https://img.shields.io/badge/3D%20Modes-1--6%20Axes%20%7C%2012%20Scenes-1976D2)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

![TMCM-6110 3D Simulator Demo](./demo.gif)

## 專案簡介

本專案為針對 **Trinamic (ADI) TMCM-6110** 六軸步進馬達控制器所開發的獨立 Windows **數位孿生（Digital Twin）與虛擬調試 3D 模擬器**，採用 C# / .NET 8.0 與原生 WPF 開發。讓運動控制與上位機開發者在**沒有實體控制卡或機台尚未組裝完成時**，即可透過 **9-Byte Binary TMCL 協定**進行通訊驗證與除錯，並提供 **1～6 使用軸數可調**、**12 種原生 WPF 3D 機台機構場景**、即時封包監控、馬達通道與軸向即時互換及互動式故障注入功能。

## 核心亮點

- **完整 9-Byte Binary TMCL 協定引擎**：依循 [`TMCL_reference.pdf`](./TMCL_reference.pdf) 實作封包解析、8-bit 累加 Checksum 驗證與滑動視窗同步；無論介面設定為幾軸，內部始終完整保留 `Axis 0~5` 暫存器，確保上位機輪詢全部 6 軸絕不 Timeout。
- **1～6 使用軸數與 12 種原生 WPF `Viewport3D` 機台場景**：零第三方 3D 套件依賴；於 **Tab 2（機台與系統設定）** 選擇 **1～6 軸**後自動篩選對應 3D 機構模式（涵蓋單軸滑台、單軸分度盤、XY/XZ 雙軸機構、三軸龍門／XY+旋轉台、四軸／五軸龍門旋轉機、六軸雙龍門、六軸龍門+輔助軸、六軸關節手臂及可隨軸數縮放的 1～6 軌線性測試台），即時顯示 3D 極限燈號與工具尖端（TCP）金色軌跡光帶。
- **實體馬達通道 (`Motor #0..#5`) 與 3D 軸向角色即時對調**：**Tab 1（軸向監控與控制）** 每張軸卡皆提供雙下拉選單，可即時將實體通道與 3D 角色 1-對-1 互換（並自動搬移對應行程範圍），Tab 2 亦提供 **`預設 Z-Y-X`** 與 **`標準 X-Y-Z`** 快速預設按鈕。
- **機械原點預設、自訂行程 (mm) 與極限停機控制**：支援 XY 中心與四角落共 5 種原點與 Z 軸 3 種原點（中心／頂部／底部）預設，可選擇同步更新 `RFS (#193)` 原點搜尋方向；各軸支援獨立設定 `-Limit / +Limit (mm)` 行程、`Steps/mm` 換算與 `#12/#13` 極限停機保護。
- **雙傳輸通道與硬體故障注入**：支援虛擬串列埠（`com0com` `COM10 <-> COM11`）與自動啟動的本機 TCP Server（`127.0.0.1:4001`），並支援 `GAP 8` 輪詢、`Cmd 138` 非同步到位推播（`Status=100` 回覆 + 主動推播 `Status=128`）、虛擬/強制限位開關（`+Limit #10` / `-Limit #11` / `#9`）與 StallGuard2（`#207`）/ TMC262（`#208`）Alarm 注入與 `CLE (Cmd 36)` 清除復歸。
- **分頁雙語介面與解耦可擴充架構**：啟動時自動偵測系統語系並於 **Tab 2（機台與系統設定）** 提供 **`🌐 EN / 繁中`** 切換；底層已拆分為 `IMotionController`/`IMotionAxis`、`IProtocolAdapter`/`ByteStreamTransportServer` 與 `IMachine3DScene` 插件式 3D 架構。

## 快速開始（免硬體 / 3 步驟上手）

### 方式 A — 直接下載執行檔（免安裝編譯器）

1. 前往 [GitHub Releases](https://github.com/DicoLian/tmcm-6110-3d-simulator/releases) 下載最新版本的 `win-x64` `.zip` 壓縮檔並解壓縮。
2. 雙擊 `Tmcm6110.App.exe` 啟動 3D 模擬器（本機 TCP `127.0.0.1:4001` 會自動開始監聽）。
3. 執行 `Tmcm6110.HostTester.exe` 並選擇 **`[2]`**（本機 TCP `127.0.0.1:4001`），即可立即看到 3D 龍門機構開始移動。

> **提示**：由於開源釋出的執行檔未經商業憑證簽章，首次執行時 Windows SmartScreen 可能會跳出提醒，點擊 **「其他資訊 → 仍要執行」** 即可。

### 方式 B — 從原始碼建置與執行

環境需求：Windows 10 / 11 與 [.NET 8.0 SDK](https://dotnet.microsoft.com/download/dotnet/8.0)（或安裝「**.NET 桌面開發**」工作負載的 Visual Studio 2022）：

```powershell
dotnet build Tmcm6110Simulator.sln
dotnet test Tmcm6110Simulator.sln
dotnet run --project src/Tmcm6110.App
# 在第二個終端機執行測試端，並選擇 [2]（本機 TCP 127.0.0.1:4001）：
dotnet run --project src/Tmcm6110.HostTester
```

- **Visual Studio 2022**：開啟 `Tmcm6110Simulator.sln`，將 `Tmcm6110.App` 設為啟動專案後按 `F5`。
- **執行自動化協定自我測試**：
  ```powershell
  dotnet run --project src/Tmcm6110.HostTester -- --self-test
  ```

---

## 操作展示影片

- 精華操作短片（約 27 秒，1.6 MB）：[`demo_highlight.mp4`](./demo_highlight.mp4) *（錄製於改版前的經典 3 軸龍門介面，1～6 軸與 12 種 3D 機構模式已於後續版本實作）*

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

## 1～6 使用軸數與 12 種 3D 機台模式

於 **Tab 2（機台與系統設定）** 選擇 **使用軸數 (`1..6`)** 後，系統會同步調整 Tab 1 顯示的軸卡數量，並於 **3D 機構模式** 下拉選單篩選對應機構場景：

| 使用軸數 | 可選 3D 機台模式 | 邏輯角色配置 (`Role 0..N-1`) |
| :--- | :--- | :--- |
| **1 軸** | `1 軸線性滑台 (X)`<br>`1 軸旋轉分度盤 (R)`<br>`1 軸獨立線性測試台 (Ch 0)` | `X` 單軸線性滑台（含 `±Limit` 與 `0 mm` 原點基準）<br>`R` 單軸旋轉分度盤（行程對應 `±180°`）<br>`Ch 0` 動態線性測試滑軌 |
| **2 軸** | `2 軸 XY 定位平台 (X/Y)`<br>`2 軸 XZ 垂直龍門 (X/Z)`<br>`2 軸獨立線性測試台 (Ch 0~1)` | `X` / `Y` 平面二軸定位台<br>`Z` 垂直主軸 / `X` 橫樑滑座<br>`Ch 0~1` 動態並行測試滑軌 |
| **3 軸** | `3 軸龍門（預設 Z/Y/X）` *(預設)*<br>`3 軸 XY + 旋轉台 (X/Y/R)`<br>`3 軸獨立線性測試台 (Ch 0~2)` | `Z`（藍）/ `Y`（綠）/ `X`（紅）標準三軸龍門<br>`R` 旋轉載台 / `Y` 前後橋架 / `X` 左右滑座<br>`Ch 0~2` 動態並行測試滑軌 |
| **4 軸** | `4 軸龍門 + 旋轉主軸 (XYZ + R)`<br>`4 軸獨立線性測試台 (Ch 0~3)` | `Z` / `Y` / `X` 龍門 + `R` 旋轉主軸夾頭<br>`Ch 0~3` 動態並行測試滑軌 |
| **5 軸** | `5 軸龍門 + 雙旋轉軸 (XYZ + R/A)`<br>`5 軸獨立線性測試台 (Ch 0~4)` | `Z` / `Y` / `X` 龍門 + `R` 主軸旋轉 + `A` 旋轉工作台<br>`Ch 0~4` 動態並行測試滑軌 |
| **6 軸** | `6 軸雙龍門（雙工位 Z1/Y1/X1 + Z2/Y2/X2）`<br>`6 軸龍門 + 旋轉/輔助軸 (XYZ + R/A/B)`<br>`6 軸關節機械手臂 (J1~J6)`<br>`6 軸獨立線性測試台 (Ch 0~5)` | 左站 `Z1/Y1/X1` + 右站 `Z2/Y2/X2` 雙獨立工位<br>`Z/Y/X` + `R` 主軸旋 + `A` 旋轉台 + `B` 輔助供料滑台<br>`J1~J6` 六軸關節機械手臂（含正解 TCP 空間軌跡）<br>`Ch 0~5` 動態並行測試滑軌 |

> **內部 6 軸暫存器完整保留**：即使畫面只啟用 1～5 軸，`Axis 0~5` 在底層 TMCL 引擎仍持續運作與回應，確保既有上位機輪詢不中斷。

## 操作介面與互動控制

1. **頂部連線與全局快捷列**：
   - **Serial COM 連線區**：COM Port 選擇、**`重新掃描`**、鮑率（`9600`～`115200`）、**`開啟 Serial Port 監聽`** 與即時 Serial / TCP 狀態指示燈。
   - **快捷全域動作**：**`3D 插補展示 (MVP COORD)`**（依當前啟用軸數執行多軸插補巡航）、**`全軸原點復歸 (RFS)`**、**`全軸急停 (MST)`**。
2. **左側 3D 機台視窗 (`Viewport3D`)**：
   - 支援滑鼠**左/右鍵拖曳旋轉視角**、**滾輪縮放**，並提供 **`清除 3D 軌跡`** 與 **`重置 3D 視角`** 按鈕。
   - 左上角圖例與左下角即時座標列會依當前選用的 3D 機台模式與馬達通道映射自動切換格式。
3. **右側分頁 Tab 1 — `軸向監控與控制`**：
   - 支援捲動瀏覽當前啟用的 `1～6` 張軸向控制卡。
   - **實體通道 / 3D 角色雙下拉選單**：每張軸卡可即時選擇實體 **`Motor #0..#5`** 與 3D 軸向角色，自動進行 1-對-1 對調並搬移行程設定（單軸模式下角色下拉選單自動鎖定）。
   - **即時狀態與手動操作**：狀態徽章與 5 顆 LED（`到位(#8)`、`MOVE`、`+Limit(#10)`、`-Limit(#11)`、`ALARM(#207/#208)`）、步數/mm/速度/微步/RFS 顯示、**`MVP ABS`** / **`MVP REL +/-`** / **`ROL`/`ROR`** / **`■ MST停`** / **`RFS 原點`** / **`Pos = 0`**，以及強制觸發 `±Limit` 與 Alarm 故障注入。
4. **右側分頁 Tab 2 — `機台與系統設定`**：
   - **3D 機構模式與原點位置**：選擇 **使用軸數 (`1～6`)**、**3D 機構模式**、**XY 機械原點**（工作台中心 + 四個角落共 5 種）、**Z 軸原點**（中心 / 頂部 / 底部）、勾選同步更新 `RFS (#193)` 原點方向，以及 **`預設 Z-Y-X`** / **`標準 X-Y-Z`** 快速預設（`>= 3` 軸時啟用 `X-Y-Z` 按鈕）。
   - **各軸移動行程與極限開關 (mm)**：設定全局 **`Steps/mm 換算`**、各軸 `-Limit` / `+Limit (mm)` 區間（即時顯示換算步數）與 **`極限停機 (#12/#13)`** 開關。
   - **TMCL 節點位址與傳輸資訊**：調整 **`ModAddr(#66)`** 與 **`HostAddr(#76)`**，檢視本機 TCP `127.0.0.1:4001` 監聽狀態。
   - **設定檔載入與介面語系**：**`載入設定 (.ini / .tmc)`** 按鈕、已載入設定摘要與 **`🌐 EN / 繁中`** 即時語系切換。
5. **底部 9-Byte Binary 封包監控與即時反組譯**：
   - 即時呈現 RX/TX 9-Byte Hex 封包與 TMCL 指令反組譯、**累計接收封包數**、高頻輪詢過濾勾選（`GAP`/`GGP`/`GIO`/`RFS STATUS`）、**`清除全部 Alarm (CLE)`** 與 **`清空封包紀錄`**。

## 支援的 TMCL 指令集與常用參數

| 類別 | 支援指令與參數功能 |
| :--- | :--- |
| **運動控制** | `ROR (1)`、`ROL (2)`、`MST (3)`、`MVP (4)`（支援 `ABS`、`REL`、`COORD` `0x40 \| mask` 最多 6 軸同動插補，含 `$47` 三軸線性插補） |
| **參數存取** | `SAP (5)`、`GAP (6)`、`STAP (7)`、`RSAP (8)`、`SGP (9)`、`GGP (10)`、`STGP (11)`、`RSGP (12)` |
| **原點搜尋** | `RFS (13)`（`START` / `STOP` / `STATUS`，支援 Mode `65` 往 `+Limit` 與 Mode `1` 往 `-Limit` 搜尋原點） |
| **I/O 與座標** | `SIO (14)`、`GIO (15)`、`SCO (30)`、`GCO (31)`、`CCO (32)` |
| **系統與事件** | `CLE (36)`、`GetAppStatus (135)`、`GetFirmwareVersion (136)`、`RequestTargetPositionReachedEvent (138)` |
| **常用軸 / 全域參數** | Axis：目標/實際位置 (`#0/#1`)、最高速度/加減速 (`#4/#5`)、到位旗標 (`#8`)、極限開關 (`#9/#10/#11`)、極限停機停用 (`#12/#13`)、微步 (`#140`)、RFS 模式 (`#193`)、SG2 負載與異常 (`#206/#207/#208`)；Global：`ModAddr (#66)`、`HostAddr (#76)` |

## 上位機連線方式

1. **透過虛擬串列埠 (`com0com` 或 VSPD)**：
   - 建立一對互連虛擬 COM Port（例如 `COM10 <===> COM11`）。
   - 在 `Tmcm6110.App` **頂部連線列**選擇 `COM11` 與鮑率（如 `9600` 或 `115200`），確認 **Tab 2（機台與系統設定）** 的 `ModAddr(#66)=1`、`HostAddr(#76)=2`，點擊 **`開啟 Serial Port 監聽`**。
   - 將您的上位機（或內附 `Tmcm6110.HostTester`）連線至 `COM10` 即可開始雙向通訊。
2. **透過本機 TCP (`127.0.0.1:4001`)**：
   - 直接執行 `Tmcm6110.HostTester`（或 `dotnet run --project src/Tmcm6110.HostTester`）並選擇 **`[2]`**，免安裝任何虛擬串口驅動即可測試所有指令與 `Cmd 138` 非同步推播。

## 載入自訂參數檔 (.ini / .tmc / 軸向定義 .txt)

- 可於 **Tab 2（機台與系統設定）→ `載入設定 (.ini / .tmc)`** 手動載入 TMCL-IDE 匯出的 `.ini`、`.tmc` (`SAP`/`SGP`) 腳本或軸向定義 `.txt` 檔；若啟動目錄或上層目錄存在相符的 `.ini` 或軸向定義檔，程式啟動時也會自動偵測套用。
- 軸向定義 `.txt` 可透過 `axis <n> (...) <LABEL>` 格式（例如 `axis 0 (1st Axis) Z`、`axis 1 (2nd Axis) Y`、`axis 2 (3rd Axis) X`）自動同步前三軸的 3D 角色對映。
- 專案內附可直接試用的公開範例設定檔：
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — 通用三軸龍門參數預設檔（`[module_settings_axis%0..2]` 與 `[global_settings]`）。
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — 通用 `SAP` / `SGP` 初始化腳本。
- `.gitignore` 預設排除所有私人 `*.ini`、`*.tmc` 與機台軸向定義檔（僅追蹤 `samples/*.ini` 與 `samples/*.tmc` 公開範例），確保客戶敏感參數不會被上傳至公開儲存庫。

## 解耦擴充架構與專案結構

- **`src/Tmcm6110.Core`**：與 UI 框架解耦的運動模擬與協定轉譯核心：
  - `Abstractions/`：`IMotionController`、`IMotionAxis`、`IProtocolAdapter`、`MotionAxisTelemetry`、`ProtocolTraceEvent`
  - `Protocol/`：`TmclProtocolAdapter`（9-Byte Binary TMCL 解析與回應轉譯）、`TmclPacket`、`TmclDefinitions`
  - `Transport/`：`ByteStreamTransportServer`（Serial COM + TCP 多通道串流伺服器）與 `TmclSerialServer` 外觀類別
  - `Engine/` 與 `Config/`：`Tmcm6110Controller`、`AxisState`、`AxisRoleMap`（1-對-1 通道/角色雙射對映）、`TmcmConfigLoader`
- **`src/Tmcm6110.App`**：原生 WPF 應用程式層：
  - `Rendering/`：`IMachine3DScene`、`SceneDescriptor`、`Axis3DFrame`、`Mesh3DPrimitives`、`Gantry3DSceneBuilder`（場景註冊表）及 `Rendering/Scenes/` 下 12 個獨立機台場景實作
  - `ViewModels/` 與 `Resources/`：`MainViewModel`、`AxisViewModel`、`Strings.zh-TW.xaml`、`Strings.en-US.xaml`
- **`src/Tmcm6110.HostTester`**：上位機連線測試工具與自動化協定自我測試（`--self-test`）。
- **`tests/Tmcm6110.Core.Tests`**：xUnit 單元測試套件（`TmclPacketTests`、`Tmcm6110ControllerTests`、`TmcmConfigLoaderTests`、`AxisLimitsAndAlarmsTests`、`AxisRoleMapTests`、`ProtocolDecouplingTests`）。

## 客製化開發與專案合作洽詢

若貴公司或團隊有工控上位機開發、虛擬調試模擬器或客製化通訊整合需求，歡迎直接與我聯繫洽談合作或技術顧問服務：

- **聯絡信箱**：[dicokabn@gmail.com](mailto:dicokabn@gmail.com)
- **可承接服務範圍**：
  - **C# / .NET (WPF)** 自動化設備上位機與人機介面（HMI）客製開發
  - **各品牌運動控制模擬器與虛擬調試系統**（如 Trinamic、台達、研華、Galil、Modbus / EtherCAT 等）
  - **Serial / CAN / TCP** 底層通訊協定整合與 **3D 數位孿生（Digital Twin）** 視覺化系統

## 授權條款與第三方聲明

- 本模擬器原始碼採用 [MIT License](./LICENSE) 授權。
- [`TMCL_reference.pdf`](./TMCL_reference.pdf) 版權歸屬 **Analog Devices / Trinamic**，僅作為通訊協定參考文件隨附於本專案中。
- **免責聲明**：本專案為獨立開發的**非官方開源模擬器**，與 **Analog Devices, Inc.** 或 **Trinamic** 無任何隸屬、背書或贊助關係。`TMCM-6110`、`TMCL`、`StallGuard2`、`TMC262` 等名稱與商標均歸屬其各自權利人所有。
