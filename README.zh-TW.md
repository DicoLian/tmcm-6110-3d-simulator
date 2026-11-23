[English](README.md) | **繁體中文**

# TMCM-6110 TMCL 協定與 3D 運動控制器模擬器（1～6 軸）

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![3D Modes](https://img.shields.io/badge/3D%20Modes-1--6%20Axes%20%7C%2012%20Scenes-1976D2)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

![TMCM-6110 3D Simulator Demo](./demo.gif)

▶ [觀看操作展示影片](#操作展示影片) ｜ [檢視 MP4 檔案（demo_highlight.mp4）](./demo_highlight.mp4)

## 為什麼需要這個專案？

運動控制與上位機開發通常需要等待實體控制卡、驅動器或機台組裝完成，才能開始進行通訊與整合測試。

**TMCM-6110 3D Simulator** 提供一個可在本機獨立執行的 **Trinamic (ADI) TMCM-6110** 模擬控制器與 3D 機構視覺化工具：

> 上位機與開發程式 → TMCL 9-Byte Binary 封包 → TCP／Serial COM → TMCM-6110 模擬控制器 → 3D 機構視覺化

讓開發者在沒有實體控制卡或機台尚未組裝完成前，就能預先驗證與除錯：

- 9-Byte Binary TMCL 封包格式與 8-bit 累加 Checksum（依循 [`TMCL_reference.pdf`](./TMCL_reference.pdf)）
- `SAP`、`GAP`、`MVP`、`RFS` 等常用指令流程與參數讀寫
- TCP／Serial COM 主從通訊、`GAP 8` 狀態輪詢與 `Cmd 138` 非同步到位推播
- 1～6 軸協同移動、極限開關、StallGuard2／TMC262 Alarm 與回原點流程
- 自有上位機介面、運動控制邏輯或自動化測試腳本
- TMCL-IDE `.ini`／`.tmc` 參數載入與實體通道／3D 軸向角色對映

本專案採用 C# / .NET 8.0 與原生 WPF 建置，定位為開發前期、通訊整合、虛擬調試（Virtual Commissioning）與系統展示的 Windows **數位孿生模擬器**；**不應取代實體控制卡**在電氣接線、安全迴路與實機動態方面的最終驗證。

## 30 秒快速開始

### 方式一：下載預編譯版本（免安裝編譯器）

1. 前往 [GitHub Releases](https://github.com/DicoLian/tmcm-6110-3d-simulator/releases) 下載最新的 `win-x64` `.zip` 壓縮檔並解壓縮。
2. 執行 `Tmcm6110.App.exe` 啟動 3D 模擬器（會自動於 `127.0.0.1:4001` 啟動本機 TCP Server）。
3. 執行 `Tmcm6110.HostTester.exe` 並選擇 **`[2]`**（本機 TCP `127.0.0.1:4001`，預設選項），即可立即看到 3D 龍門機構開始移動。

> **提示**：由於開源釋出的執行檔尚未使用商業憑證簽章，首次執行時 Windows SmartScreen 可能會跳出警告，請確認來源後選擇 **「其他資訊 → 仍要執行」**。

### 方式二：從原始碼建置與執行

環境需求：Windows 10 / 11 與 [.NET 8.0 SDK](https://dotnet.microsoft.com/download/dotnet/8.0)（或安裝「**.NET 桌面開發**」工作負載的 Visual Studio 2022）。

建置、執行單元測試並啟動模擬器：

```powershell
dotnet build Tmcm6110Simulator.sln
dotnet test Tmcm6110Simulator.sln
dotnet run --project src/Tmcm6110.App
```

另開終端機執行上位機測試工具（選擇 `[2]` 使用本機 TCP `127.0.0.1:4001`）：

```powershell
dotnet run --project src/Tmcm6110.HostTester
```

執行協定自動化自我測試：

```powershell
dotnet run --project src/Tmcm6110.HostTester -- --self-test
```

- **Visual Studio 2022**：直接開啟 `Tmcm6110Simulator.sln`，將 `Tmcm6110.App` 設為啟動專案後按 `F5`。

## 核心功能

### 通訊與協定

- 完整解析與組裝 9-Byte Binary TMCL 封包，支援 8-bit 累加 Checksum 驗證
- 支援 Serial COM（`com0com`／VSPD）與免驅動本機 TCP `127.0.0.1:4001` 雙傳輸通道
- 即時 RX／TX 9-Byte Hex 封包監控、TMCL 指令反組譯與高頻輪詢過濾（`GAP`/`GGP`/`GIO`/`RFS STATUS`）
- 支援滑動視窗封包同步、異常封包偵測與可調式 `ModAddr (#66)`／`HostAddr (#76)` 位址
- 支援 `GAP 8` 輪詢與 `Cmd 138` 非同步到位推播（即時回覆 `Status=100` + 到位主動推播 `Status=128`）
- 無論畫面設定為幾軸，底層始終保留完整 6 軸暫存器（`Axis 0..5`），確保上位機全軸輪詢絕不 Timeout

### 模擬運動控制

- 支援 1～6 個模擬軸，涵蓋絕對（`MVP ABS`）、相對（`MVP REL`）與多軸線性插補（`MVP COORD`）移動
- 支援回原點搜尋（`RFS` Mode `65`/`1`）、急停（`MST`）、軟硬體限位（`#9/#10/#11`、`#12/#13`）與 Alarm 狀態
- 支援微步（`#140`）、速度、加速度與 `Steps/mm` 行程換算
- 支援實體馬達通道（`Motor #0..#5`）與 3D 軸向角色即時 1-對-1 對調（自動搬移行程範圍），並提供 `預設 Z-Y-X` / `標準 X-Y-Z` 快速預設
- 支援手動強制注入 `±Limit` 與 StallGuard2（`#207`）／TMC262（`#208`）Alarm，並透過 `CLE (Cmd 36)` 復歸測試上位機容錯處理

### 3D 檢視與機台設定

- 採用原生 WPF `Viewport3D`，零第三方 3D 套件依賴，具 3D 極限燈號方塊與工具尖端（TCP）金色軌跡光帶
- 內建 12 種機台與龍門場景（依啟用軸數動態篩選），支援滑鼠拖曳旋轉、滾輪縮放與視角重置
- 提供 5 種 XY 原點與 3 種 Z 軸原點預設（可選同步更新 `RFS #193` 搜尋方向），以及各軸獨立 `-Limit / +Limit (mm)` 行程設定
- 啟動時自動偵測系統語系，並可於執行期間即時切換繁體中文／英文介面
- 支援手動與自動載入 TMCL-IDE `.ini`、`.tmc` 與軸向對應 `.txt` 設定檔

---

## 操作展示影片

- 精華操作短片（約 27 秒，1.6 MB）：[`demo_highlight.mp4`](./demo_highlight.mp4) *（錄製於改版前的經典 3 軸龍門介面，1～6 軸與 12 種 3D 機構模式已於後續版本實作）*

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

## 支援範圍與限制

本專案以 **TMCM-6110／TMCL 9-Byte Binary 協定**為目標，模擬開發、通訊除錯及上位機流程所需的控制器行為。

目前已實作常用指令與關鍵參數如下：

| 類別 | 支援指令與參數功能 |
| :--- | :--- |
| **運動控制** | `ROR (1)`、`ROL (2)`、`MST (3)`、`MVP (4)`（支援 `ABS`、`REL`、`COORD` `0x40 \| mask` 最多 6 軸同動插補，含 `$47` 三軸線性插補） |
| **參數存取** | `SAP (5)`、`GAP (6)`、`STAP (7)`、`RSAP (8)`、`SGP (9)`、`GGP (10)`、`STGP (11)`、`RSGP (12)` |
| **原點搜尋** | `RFS (13)`（`START` / `STOP` / `STATUS`，支援 Mode `65` 往 `+Limit` 與 Mode `1` 往 `-Limit` 搜尋原點） |
| **I/O 與座標** | `SIO (14)`、`GIO (15)`、`SCO (30)`、`GCO (31)`、`CCO (32)` |
| **系統與事件** | `CLE (36)`、`GetAppStatus (135)`、`GetFirmwareVersion (136)`、`RequestTargetPositionReachedEvent (138)` |
| **常用軸 / 全域參數** | Axis：目標/實際位置 (`#0/#1`)、最高速度/加減速 (`#4/#5`)、到位旗標 (`#8`)、極限開關 (`#9/#10/#11`)、極限停機停用 (`#12/#13`)、微步 (`#140`)、RFS 模式 (`#193`)、SG2 負載與異常 (`#206/#207/#208`)；Global：`ModAddr (#66)`、`HostAddr (#76)` |

### 使用限制與注意事項

- **協定與邏輯模擬**：本模擬器模擬的是 TMCL 協定與運動控制邏輯，並非實體步進馬達驅動級或真實機械負載系統。
- **不含電氣與安全驗證**：不包含實際電氣配線、急停安全迴路（E-Stop / Safety PLC）或硬體過電流／過溫保護的驗證。
- **未實作指令勿推論**：未列於上表或尚未驗證的 TMCL 指令與進階韌體參數，不應視為與真實韌體完全一致。
- **實機整合仍需測試**：真實 TMCM-6110 控制卡、特定韌體版本與實機機構接線，仍須於後續硬體整合階段另行實測確認。
- **行為細節簡化**：部分加減速曲線與故障注入邏輯為配合開發測試而設計，細節可能與實體控制卡略有差異。

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

<details>
<summary><strong>展開完整操作介面與互動控制說明（頂部連線列、3D 視窗、Tab 1/2、封包監控）</strong></summary>

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

</details>

## 上位機連線與自訂參數檔

### 上位機連線方式

1. **透過虛擬串列埠 (`com0com` 或 VSPD)**：
   - 建立一對互連虛擬 COM Port（例如 `COM10 <===> COM11`）。
   - 在 `Tmcm6110.App` **頂部連線列**選擇 `COM11` 與鮑率（如 `9600` 或 `115200`），確認 **Tab 2（機台與系統設定）** 的 `ModAddr(#66)=1`、`HostAddr(#76)=2`，點擊 **`開啟 Serial Port 監聽`**。
   - 將您的上位機（或內附 `Tmcm6110.HostTester`）連線至 `COM10` 即可開始雙向通訊。
2. **透過本機 TCP (`127.0.0.1:4001`)**：
   - 直接執行 `Tmcm6110.HostTester`（或 `dotnet run --project src/Tmcm6110.HostTester`）並選擇 **`[2]`**，免安裝任何虛擬串口驅動即可測試所有指令與 `Cmd 138` 非同步推播。

### 載入自訂參數檔（.ini / .tmc / 軸向定義 .txt）

- 可於 **Tab 2（機台與系統設定）→ `載入設定 (.ini / .tmc)`** 手動載入 TMCL-IDE 匯出的 `.ini`、`.tmc` (`SAP`/`SGP`) 腳本或軸向定義 `.txt` 檔；若啟動目錄或上層目錄存在相符的 `.ini` 或軸向定義檔，程式啟動時也會自動偵測套用。
- 軸向定義 `.txt` 可透過 `axis <n> (...) <LABEL>` 格式（例如 `axis 0 (1st Axis) Z`、`axis 1 (2nd Axis) Y`、`axis 2 (3rd Axis) X`）自動同步前三軸的 3D 角色對映。
- 專案內附可直接試用的公開範例設定檔：
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — 通用三軸龍門參數預設檔（`[module_settings_axis%0..2]` 與 `[global_settings]`）。
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — 通用 `SAP` / `SGP` 初始化腳本。
- `.gitignore` 預設排除所有私人 `*.ini`、`*.tmc` 與機台軸向定義檔（僅追蹤 `samples/*.ini` 與 `samples/*.tmc` 公開範例），確保客戶敏感參數不會被上傳至公開儲存庫。

## 專案結構與運作流程

```text
src/
├── Tmcm6110.Core/
│   ├── Abstractions/       IMotionController、IMotionAxis、IProtocolAdapter、Telemetry
│   ├── Protocol/           TmclProtocolAdapter、TmclPacket、TmclDefinitions
│   ├── Transport/          ByteStreamTransportServer（TCP / Serial COM）、TmclSerialServer
│   ├── Engine/             Tmcm6110Controller、AxisState、AxisRoleMap
│   └── Config/             TmcmConfigLoader（.ini / .tmc / 軸向對應 .txt 載入）
├── Tmcm6110.App/
│   ├── ViewModels/         MainViewModel、AxisViewModel
│   ├── Rendering/          IMachine3DScene、Gantry3DSceneBuilder 與 Scenes/（12 種機台場景）
│   └── Resources/          Strings.zh-TW.xaml、Strings.en-US.xaml 與 Localization/
└── Tmcm6110.HostTester/    上位機測試工具與協定自動自我測試（--self-test）

samples/                    公開三軸龍門範例設定檔（demo_gantry.ini、demo_gantry.tmc）
tests/
└── Tmcm6110.Core.Tests/    xUnit 單元測試（封包、控制器、極限/異常、軸角色對映、設定、解耦）
```

`ByteStreamTransportServer` 負責透過 TCP 或 Serial COM 收送位元串流，`TmclProtocolAdapter` 進行 9-Byte Binary TMCL 封包解析與回應分派，再交由 `Tmcm6110Controller` 執行運動運算並更新各軸狀態；最後由 WPF ViewModel 讀取控制器即時位置、速度、極限與 Alarm 狀態並驅動 `IMachine3DScene` 更新 3D 機台畫面。

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
