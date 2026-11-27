[English](README.md) | **繁體中文**

# TMCM-6110 TMCL 協定與 3D 運動控制器模擬器（1～6 軸）

![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)
![WPF 3D](https://img.shields.io/badge/UI-WPF%20Native%203D-0078D4?logo=windows)
![3D Modes](https://img.shields.io/badge/3D%20Modes-1--6%20Axes%20%7C%2012%20Scenes-1976D2)
![Protocol](https://img.shields.io/badge/Protocol-TMCL%209--Byte%20Binary-00A86B)
![Firmware Spec](https://img.shields.io/badge/Firmware%20Spec-TMCM--6110%20FW%20V1.47%20(Rev%201.23)-FF6F00)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Build & Release](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml/badge.svg)](https://github.com/DicoLian/tmcm-6110-3d-simulator/actions/workflows/release.yml)

> **韌體規格說明：** 本模擬器目前已全面改為採用 **[`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf)** 內容（TMCM-6110 TMCL Firmware V1.47 / Rev 1.23，`Cmd 136` 回傳 `6110V147` / `0x17DE012F`）。原附之 [`TMCL_reference.pdf`](./TMCL_reference.pdf)（V2.22 舊版通用手冊）僅保留作為歷史規格對照參考。

![TMCM-6110 3D Simulator Demo](./demo.gif)

▶ [觀看操作展示影片](#操作展示影片) ｜ [檢視 MP4 檔案（demo_highlight.mp4）](./demo_highlight.mp4)

## 為什麼需要這個專案？

運動控制與上位機開發通常需要等待實體控制卡、驅動器或機台組裝完成，才能開始進行通訊與整合測試。

**TMCM-6110 3D Simulator** 提供一個可在本機獨立執行的 **Trinamic (ADI) TMCM-6110** 模擬控制器與 3D 機構視覺化工具：

> 上位機與開發程式 → TMCL 9-Byte Binary 封包 → TCP／Serial COM → TMCM-6110 模擬控制器 → 3D 機構視覺化

讓開發者在沒有實體控制卡或機台尚未組裝完成前，就能預先驗證與除錯：

- 9-Byte Binary TMCL 封包格式與 8-bit 累加 Checksum 驗證（嚴格依循 [`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf)，版本 `6110V147`）
- `SAP`、`GAP`、`MVP`、`RFS`、`SCO`/`GCO`/`CCO`/`ACO`、`SIO`/`GIO`、`Cmd 138`、`Cmd 255` 等指令流程與嚴格參數範圍／唯讀檢查
- **TMCL-IDE 程式下載與獨立執行（Standalone VM）**：完整 2048-slot EEPROM 程式記憶體（`Cmd 132`/`133` 回應 `Status=101`、`Cmd 134` 記憶體回讀、`#81` 程式碼保護）、獨立執行 VM（`16..57`、`80`、`Cmd 128..131`、`#77` AutoStart）、47 向量中斷控制器（`VECT`/`EI`/`DI`/`RETI`）與內建雙階段組譯器 `TmclAssembler`
- TCP／Serial COM 主從通訊、`GAP 8` 輪詢、`#87` 次要串列位址、`#255` 關閉主動回覆，以及先布防 `Cmd 138` 的非同步到位推播
- 1～6 軸運動控制（32-bit 位置、單次最多 3 軸 `Bit 6` 線性插補）、極限與原點開關、StallGuard2（`#207` 讀取自動清除）／TMC262（`#208`）異常與 `RFS`（Mode `1..8`、`+64`、`+128`）回原點流程
- 自有上位機介面、運動控制邏輯或自動化測試腳本
- TMCL-IDE `.ini`／`.tmc` 參數載入與實體通道／3D 軸向角色對映

本專案採用 C# / .NET 8.0 與原生 WPF 建置，定位為開發前期、通訊整合、虛擬調試（Virtual Commissioning）與系統展示的 Windows **數位孿生模擬器**；**不應取代實體控制卡**在電氣接線、安全迴路與實機動態方面的最終驗證。

## 30 秒快速開始

### 方式一：下載預編譯版本（免安裝編譯器）

1. 前往 [GitHub Releases](https://github.com/DicoLian/tmcm-6110-3d-simulator/releases) 下載最新的 `win-x64` `.zip` 壓縮檔並解壓縮。
2. 執行 `Tmcm6110.App.exe` 啟動 3D 模擬器（會自動於 `127.0.0.1:4001` 啟動本機 TCP Server）。
3. 執行 `Tmcm6110.HostTester.exe` 並選擇 **`[2]`**（本機 TCP `127.0.0.1:4001`，預設選項），即可立即看到 3D 龍門機構開始移動（可選擇 **`[7]`** 執行 TMCL-IDE 程式下載與 Standalone VM 展示）。

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

執行協定自動化自我測試（涵蓋 Standalone 下載、VM 與中斷驗證）：

```powershell
dotnet run --project src/Tmcm6110.HostTester -- --self-test
```

- **Visual Studio 2022**：直接開啟 `Tmcm6110Simulator.sln`，將 `Tmcm6110.App` 設為啟動專案後按 `F5`。

## 核心功能

### 通訊與協定

- 完整解析與組裝 9-Byte Binary TMCL 封包，支援 8-bit 累加 Checksum 驗證
- 支援 Serial COM（`com0com`／VSPD，鮑率支援至 `230400`）與免驅動本機 TCP `127.0.0.1:4001` 雙傳輸通道
- 即時 RX／TX 9-Byte Hex 封包監控、TMCL 指令反組譯（含 Standalone 指令 `16..57`、`80`、控制指令 `128..134` 與 `Cmd 134` `"MemRead:"` 特殊 9-Byte 回覆）以及高頻輪詢過濾（`GAP`/`GGP`/`GIO`/`RFS STATUS`）
- 支援滑動視窗封包同步、異常封包偵測、可調式 `ModAddr (#66)`／`HostAddr (#76)` 位址、次要位址 `#87` 與抑制回覆 `#255`
- 支援 `GAP 8` 輪詢與 FW V1.47 **運動前預先布防** `Cmd 138` 非同步到位推播（`Type=0` 單次／`Type=1` 連續；即時回覆 `Status=100` + 到位主動推播 `Status=128`）
- 無論畫面設定為幾軸，底層始終保留完整 6 軸暫存器（`Axis 0..5`），確保上位機全軸輪詢絕不 Timeout

### TMCL-IDE 程式下載、Standalone VM 與中斷控制器

- **TMCL-IDE 程式下載與 EEPROM 記憶體 (`TmclProgramMemory`)**：
  - 2048-slot EEPROM 程式記憶體，完整實作 TMCL-IDE 下載流程（`Cmd 128 Stop -> Cmd 132 StartDownload -> 各指令與結束碼 Opcode 0 皆回傳 Status=101 -> Cmd 133 QuitDownload`），同步維護 `#129` 下載模式與 `#130` 寫入指標，下載期間 `< 128` 指令僅寫入記憶體而不立即執行
  - `ReadTmclMemory (Cmd 134)` 依 FW V1.47 Section 3.2 回傳無 Checksum 之特殊 9-Byte 封包 `[Host, Mod, Op, Type, Mot, V3..V0]`
  - `#81` 程式碼保護（`1`=讀取鎖定、`2`=寫入鎖定、`3`=雙向鎖定；依 Table 18 從 `1`/`3` 解除為 `0`/`2` 時自動清除已下載程式）；`Cmd 255` 保留程式記憶體，而 `Cmd 137` 會清除程式記憶體
- **獨立執行 TMCL 虛擬機 (`TmclStandaloneVm`)**：
  - 完整支援 Standalone 與 FW V1.45+ 指令（`16..57`、`80`）：`CALC`/`CALCX`/`COMP`/`JC`（`ZE..EPO`）、`JA`/`CSUB`/`RSUB`/`CALL`（8 層呼叫堆疊與溢位保護）、`RST`/`DJNZ`、`AAP`/`AGP`/`ACO`、`CALCV*`（`CALCVV/VA/AV/VX/XV/V`）、暫存器/索引運動（`MVPA/MVPXA/ROLA/RORA/ROLXA/RORXA/MSTX/SAPX/GAPX/AAPX`）與索引變數（`SIV/GIV/AIV`）
  - 精確 **10 ms/tick** `WAIT` 等待引擎（`TICKS`、`POS`、`REFSW`、`LIMSW`、`RFS`，支援 `Value == -1` 讀取 `Accumulator` 延遲與事件逾時設定 `ETO` 旗標）
  - 支援執行與單步除錯控制（`Cmd 128 Stop`、`Cmd 129 Run`、`Cmd 130 Step`、`Cmd 131 Reset`、`Cmd 135 GetAppStatus Types 0..3`）、`#77 AutoStart` 開機自動執行，以及直連模式 `GAP`/`GGP`/`GIO`/`GCO` 與 VM `Accumulator` 嚴格隔離
- **47 向量中斷控制器 (`VECT 37`, `EI 25`, `DI 26`, `RETI 38`)**：
  - 支援 IRQ `0..2`（Bank 3 `#0..#2` 毫秒週期計時器）、`3..8`（到達目標位置）、`15..20`（StallGuard 異常）、`27..38`（極限開關邊緣觸發，Bank 3 `#27..#38`）與 `39..46`（數位輸入邊緣觸發，Bank 3 `#39..#46`，支援 `1=上升沿`、`2=下降沿`、`3=雙邊緣`）
  - 依 IRQ 編號優先權派送（編號越小優先權越高），並於 `RETI` 完整還原 `PC`、`Accumulator`、`XRegister`、比較旗標與進行中的 `WAIT` 狀態
- **內建雙階段組譯器 (`TmclAssembler`) 與範例腳本**：
  - 支援 `//`／`;` 註解、`#define` 與 `NAME = VALUE` 常數（十進位、負數、`0x`、`$` 十六進位）、前後向與行內標籤、符號化軸參數／運算子／條件名稱、全部 FW V1.47 指令（`SIV=55`、`GIV=56`、`AIV=57`、`CALL=80`）與含行號的語法錯誤診斷
  - 隨附三軸取放循環範例 [`samples/demo_standalone.tmc`](./samples/demo_standalone.tmc)，包含 100 ms Timer 0 中斷切換 `DOUT 0` 心跳燈

### 模擬運動控制

- 完整支援 **32-bit** 有號位置（`-2,147,483,648 .. 2,147,483,647`）、有號 `ROR`/`ROL`（`-2047..2047`），以及 **Soft Stop 緩減速停止** `MST`（依 `#5` 加速度線性減速至零）
- 支援 1～6 個模擬軸，涵蓋絕對（`MVP ABS`）、相對（`MVP REL`）與協同座標移動（`MVP COORD`：`Bit 6 / 0x40` 單次最多 3 軸同步速度與加速度插補；`Bit 7 / 0x80` 不限軸數非插補啟動；`3D 插補展示` 會自動將超過 3 軸拆分為多組 `<=3` 軸群組）
- 支援 FW V1.47 原點搜尋（`RFS` Mode `1..8`、`+64`、`+128`，含 `#194/#195` `0..2047` 速度與 `#196/#197` 校準回讀）、軟硬體限位（`#9/#10/#11`、`#12/#13`、`#149` 限位緩停旗標）與 Alarm 狀態
- 支援微步（`#140` `0..8`）、電流（`#6/#7` `0..255`）、即時加速度（`#135`）、SmartEnergy 實際電流（`#180`）、速度、加速度與 `Steps/mm` 行程換算
- 支援實體馬達通道（`Motor #0..#5`）與 3D 軸向角色即時 1-對-1 對調（自動搬移行程範圍），並提供 `預設 Z-Y-X` / `標準 X-Y-Z` 快速預設
- 支援手動強制注入 `±Limit` 與 StallGuard2（`#207` 讀取或移動時自動清除）／致命 TMC262（`#208`）Alarm（`CLE` 僅清除獨立錯誤旗標 `0..5`，致命驅動停機需透過清除故障或 `Cmd 255` 軟體重置復歸）

### 3D 檢視與機台設定

- 採用原生 WPF `Viewport3D`，零第三方 3D 套件依賴，具 3D 極限燈號方塊與工具尖端（TCP）金色軌跡光帶
- 內建 12 種機台與龍門場景（依啟用軸數動態篩選），支援滑鼠拖曳旋轉、滾輪縮放與視角重置
- 提供 5 種 XY 原點與 3 種 Z 軸原點預設（可選同步更新 `RFS #193` 搜尋方向），以及各軸獨立 `-Limit / +Limit (mm)` 行程設定
- 啟動時自動偵測系統語系，並可於執行期間即時切換繁體中文／英文介面
- 支援手動與自動載入 TMCL-IDE `.ini`、`.tmc` 與軸向對應 `.txt` 設定檔，並提供 **Tab 3（`TMCL 程式 / Standalone VM`）** 即時 VM 監控、`.tmc` 組譯下載與反組譯單步除錯介面

---

## 操作展示影片

- 精華操作短片（約 27 秒，1.6 MB）：[`demo_highlight.mp4`](./demo_highlight.mp4) *（錄製於改版前的經典 3 軸龍門介面，1～6 軸與 12 種 3D 機構模式已於後續版本實作）*

https://github.com/user-attachments/assets/3f8b5a76-0ad5-4a7a-8779-d88812af3ebb

## 支援範圍與限制

本專案目前採用 **[`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf)**（Firmware **V1.47 / Rev 1.23**）作為 **TMCM-6110／TMCL 9-Byte Binary 協定**規格基準，模擬開發、通訊除錯及上位機流程所需的控制器行為。

目前已實作指令與關鍵參數如下：

| 類別 | 支援指令與參數功能（FW V1.47） |
| :--- | :--- |
| **運動控制** | 有號 `ROR (1)`／`ROL (2)`（`-2047..2047`）、緩減速停止 `MST (3)`、32-bit `MVP (4)`（`ABS`、`REL`、`COORD` `0x40\|mask` 單次最多 **3 軸**線性插補、`0x80\|mask` 不限軸數同動啟動）、`MVPA (46)`、`MVPXA (47)`、`ROLA (50)`、`RORA (51)`、`ROLXA (52)`、`RORXA (53)`、`MSTX (54)` |
| **參數存取** | 嚴格驗證 `SAP (5)`、`GAP (6)`、`STAP (7)`、`RSAP (8)`、`SGP (9)`、`GGP (10)`、`STGP (11)`、`RSGP (12)`，以及索引／累加器存取 `SAPX (16)`、`GAPX (17)`、`AAPX (18)`、`AAP (34)`、`AGP (35)`、`SIV (55)`、`GIV (56)`、`AIV (57)`（已移除 `SAC 29`；嚴格區分 Bank 0/2/3） |
| **原點搜尋** | `RFS (13)`（`START` / `STOP` / `STATUS`），支援 Mode `1..8`、`+64`、`+128`，`#194/#195`（`0..2047`）、`#196` EndSwitchDistance、`#197` LastReferencePosition |
| **I/O 與座標** | `SIO (14)`／`GIO (15)`（`0..7` DI/DO、12-bit `AIN0(0)`/`AIN4(4)`、`Port 8` 供電電壓 `240`=24.0V、`SIO 0,0,0..3` 上拉電阻）；座標 `0..20`（`SCO 30`、`GCO 31`、`CCO 32`、`ACO 39`、`255` 批次複製、`#84`） |
| **Standalone VM 與流程** | `CALC (19)`（`0..9`）、`COMP (20)`、`JC (21)`（`ZE..EPO`）、`JA (22)`、`CSUB (23)`、`RSUB (24)`、`WAIT (27)`（`10 ms/tick`：`TICKS/POS/REFSW/LIMSW/RFS` 與 `ETO` 逾時）、`STOP (28)`、`CALCX (33)`（`0..10`）、`CALCVV (40)`、`CALCVA (41)`、`CALCAV (42)`、`CALCVX (43)`、`CALCXV (44)`、`CALCV (45)`、`RST (48)`、`DJNZ (49)`、`CALL (80)`（8 層呼叫堆疊） |
| **中斷控制（47 向量）** | `EI (25)`、`DI (26)`、`VECT (37)`、`RETI (38)`：支援 IRQ `0..2`（計時器）、`3..8`（到位）、`15..20`（StallGuard）、`27..38`（極限邊緣 `1/2/3`）、`39..46`（數位輸入邊緣 `1/2/3`）；編號越小優先權越高；`RETI` 完整還原 `WAIT` 與暫存器上下文 |
| **下載與系統控制** | `Stop (128)`、`Run (129)`、`Step (130)`、`Reset (131)`、`StartDownload (132)`／`QuitDownload (133)`（`Status=101`）、`ReadTmclMemory (134)`（特殊 9-Byte 回覆、`#81` 鎖定）、`GetAppStatus (135)`（`Type 0..3`）、`GetFirmwareVersion (136)`（`"6110V147"`／`0x17DE012F`）、`RestoreFactorySettings (137)`（`1234` 無回覆並清除程式）、預先布防 `RequestTargetPositionReachedEvent (138)`（`Type 0/1`）、`CLE (36)`（`0..5`）、`SoftwareReset (255)`（`1234` 保留程式並支援 `#77` AutoStart） |
| **常用軸 / 全域參數** | Axis（Table 16）：32-bit 位置 (`#0/#1`)、速度/加減速 (`#2/#3/#4/#5/#135`)、電流 (`#6/#7` `0..255`)、到位 (`#8`)、原點/極限開關 (`#9/#10/#11`, `#12/#13`)、微步 (`#140` `0..8`)、限位緩停 (`#149`)、CoolStep/SG2 (`#180/#181/#206`)、讀取自動清除 `#207`、TMC262 `#208`；Global：鮑率 `#65`、`ModAddr #66`、`HostAddr #76`、`AutoStart #77`、`CodeProtection #81`、次要位址 `#87`、`AppStatus #128`、`DownloadMode #129`、`PC #130`、亂數 `#133`、關閉回覆 `#255`、Bank 2 (`0..255`)、Bank 3 (`0..2`, `27..46`) |

> **相容性提醒（對比舊版 `TMCL_reference.pdf` V2.22）：** 若現有上位機依舊版通用手冊撰寫，改為 FW V1.47 模式後需注意：`Cmd 138` 須於 `MVP` **之前**布防、`MST` 為緩減速停止、`MVP COORD Bit 6` 上限為 3 軸、`Cmd 136` 回傳 `6110V147` / `0x17DE012F`、`#207` 讀取即自動清除、`CLE` 不清除 TMC262 致命保護、座標擴大至 `0..20`，且已移除 `SAC (29)` 與舊版參數（`#64`、`#73`、`#141/#142/#147/#151/#205/#209..#212`）。

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
<summary><strong>展開完整操作介面與互動控制說明（頂部連線列、3D 視窗、Tab 1/2/3、封包監控）</strong></summary>

1. **頂部連線與全局快捷列**：
   - **Serial COM 連線區**：COM Port 選擇、**`重新掃描`**、鮑率（`9600`～`230400`）、**`開啟 Serial Port 監聽`** 與即時 Serial / TCP 狀態指示燈。
   - **快捷全域動作**：**`3D 插補展示 (MVP COORD)`**（先布防 `Cmd 138` 並分組執行 `<=3` 軸 `MVP COORD` 巡航）、**`全軸原點復歸 (RFS)`**、**`全軸停止 (MST)`**。
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
5. **右側分頁 Tab 3 — `TMCL 程式 / Standalone VM`**：
   - **VM 狀態與暫存器**：彩色狀態徽章（`DOWNLOAD`、`RUN`、`WAIT`、`STEP`、`RESET`/`STOP`）、`WAIT` 等待細節與中斷指示（`IRQ #n ACTIVE`／`IRQ ON`／`IRQ OFF`），以及即時暫存器（`PC`、`指令數`、`Accu`、`X Reg`、`Stack 0..8`、`錯誤旗標`、`保護 (#81)`、`已執行指令數`）。
   - **下載與執行控制**：**`載入並下載 (.tmc)`**（透過 `TmclAssembler` 組譯並依 `Cmd 128 -> 132 -> Status 101 -> 0 -> 133` 流程寫入 EEPROM，同時於底部紀錄完整 9-Byte 封包）、**`▶ 執行 (129)`**、**`⏭ 單步 (130)`**、**`■ 停止 (128)`**、**`↺ 重置 (131)`**、**`清除 EEPROM`** 與 **`開機自動執行 (#77)`** 核取方塊。
   - **程式記憶體反組譯檢視**：即時顯示 `PC` `▶` 指標、`位址 (Addr)`、7-Byte `Hex` 與 `TMCL 反組譯助記詞`。
6. **底部 9-Byte Binary 封包監控與即時反組譯**：
   - 即時呈現 RX/TX 9-Byte Hex 封包與 TMCL 指令反組譯、**累計接收封包數**、高頻輪詢過濾勾選（`GAP`/`GGP`/`GIO`/`RFS STATUS`）、**`清除全部 Alarm`** 與 **`清空封包紀錄`**。

</details>

## 上位機連線與自訂參數檔

### 上位機連線方式

1. **透過虛擬串列埠 (`com0com` 或 VSPD)**：
   - 建立一對互連虛擬 COM Port（例如 `COM10 <===> COM11`）。
   - 在 `Tmcm6110.App` **頂部連線列**選擇 `COM11` 與鮑率（如 `9600` 或 `115200`），確認 **Tab 2（機台與系統設定）** 的 `ModAddr(#66)=1`、`HostAddr(#76)=2`，點擊 **`開啟 Serial Port 監聽`**。
   - 將您的上位機（或內附 `Tmcm6110.HostTester`）連線至 `COM10` 即可開始雙向通訊。
2. **透過本機 TCP (`127.0.0.1:4001`)**：
   - 直接執行 `Tmcm6110.HostTester`（或 `dotnet run --project src/Tmcm6110.HostTester`）並選擇 **`[2]`**，免安裝任何虛擬串口驅動即可測試所有指令與 `Cmd 138` 非同步推播（選擇 **`[7]`** 可互動執行 TMCL-IDE 下載、`Cmd 134` 回讀與 Standalone VM 三軸展示）。

### 載入自訂參數檔（.ini / .tmc / 軸向定義 .txt）

- 可於 **Tab 2（機台與系統設定）→ `載入設定 (.ini / .tmc)`** 手動載入 TMCL-IDE 匯出的 `.ini`、`.tmc` (`SAP`/`SGP`) 腳本或軸向定義 `.txt` 檔；若啟動目錄或上層目錄存在相符的 `.ini` 或軸向定義檔，程式啟動時也會自動偵測套用。
- 可於 **Tab 3（TMCL 程式 / Standalone VM）→ `載入並下載 (.tmc)`** 組譯並下載完整的 Standalone TMCL 程式（支援標籤、`#define` 常數、副程式、`WAIT` 與中斷服務常式）至 EEPROM 程式記憶體。
- 軸向定義 `.txt` 可透過 `axis <n> (...) <LABEL>` 格式（例如 `axis 0 (1st Axis) Z`、`axis 1 (2nd Axis) Y`、`axis 2 (3rd Axis) X`）自動同步前三軸的 3D 角色對映。
- 專案內附可直接試用的公開範例設定與程式檔：
  - [`samples/demo_gantry.ini`](./samples/demo_gantry.ini) — 通用三軸龍門參數預設檔（`[module_settings_axis%0..2]` 與 `[global_settings]`）。
  - [`samples/demo_gantry.tmc`](./samples/demo_gantry.tmc) — 通用 `SAP` / `SGP` 初始化腳本。
  - [`samples/demo_standalone.tmc`](./samples/demo_standalone.tmc) — 三軸取放循環 Standalone TMCL 範例程式（相容 TMCL-IDE `NAME = VALUE` 語法），展示 `CSUB`/`RSUB`、`GGP`/`CALC`/`AGP`/`COMP`/`JC` 迴圈計數、`WAIT POS`/`WAIT TICKS` 與 100 ms Timer 0 `DOUT 0` 心跳中斷。
- `.gitignore` 預設排除所有私人 `*.ini`、`*.tmc` 與機台軸向定義檔（僅追蹤 `samples/*.ini` 與 `samples/*.tmc` 公開範例），確保客戶敏感參數不會被上傳至公開儲存庫。

## 專案結構與運作流程

```text
src/
├── Tmcm6110.Core/
│   ├── Abstractions/       IMotionController、IMotionAxis、IProtocolAdapter、Telemetry
│   ├── Protocol/           TmclProtocolAdapter、TmclPacket、TmclDefinitions
│   ├── Transport/          ByteStreamTransportServer（TCP / Serial COM）、TmclSerialServer
│   ├── Engine/             Tmcm6110Controller、AxisState、AxisRoleMap、
│   │                       TmclProgramMemory、TmclStandaloneVm、TmclAssembler
│   └── Config/             TmcmConfigLoader（.ini / .tmc / 軸向對應 .txt 載入）
├── Tmcm6110.App/
│   ├── ViewModels/         MainViewModel、AxisViewModel
│   ├── Rendering/          IMachine3DScene、Gantry3DSceneBuilder 與 Scenes/（12 種機台場景）
│   └── Resources/          Strings.zh-TW.xaml、Strings.en-US.xaml 與 Localization/
└── Tmcm6110.HostTester/    上位機測試工具與協定自動自我測試（--self-test）

samples/                    公開三軸龍門範例設定與獨立程式檔（demo_gantry.ini/.tmc、demo_standalone.tmc）
tests/
└── Tmcm6110.Core.Tests/    xUnit 單元測試（封包、控制器、極限/異常、軸角色對映、設定、解耦、
                            程式下載與記憶體、Standalone VM、中斷與除錯、雙階段組譯器）
```

`ByteStreamTransportServer` 負責透過 TCP 或 Serial COM 收送位元串流，`TmclProtocolAdapter` 進行 9-Byte Binary TMCL 封包解析與回應分派，再交由 `Tmcm6110Controller` 更新各軸運動狀態、管理 `TmclProgramMemory` 程式記憶體，並驅動 `TmclStandaloneVm` 與 47 向量中斷控制器；`TmclAssembler` 負責將 `.tmc` 原始碼組譯為 9-Byte 指令封包，而 WPF ViewModel 則即時讀取控制器軸向與 VM 狀態，驅動 `IMachine3DScene` 3D 畫面與 Tab 3 反組譯除錯器。

## 客製化開發與專案合作洽詢

若貴公司或團隊有工控上位機開發、虛擬調試模擬器或客製化通訊整合需求，歡迎直接與我聯繫洽談合作或技術顧問服務：

- **聯絡信箱**：[dicokabn@gmail.com](mailto:dicokabn@gmail.com)
- **可承接服務範圍**：
  - **C# / .NET (WPF)** 自動化設備上位機與人機介面（HMI）客製開發
  - **各品牌運動控制模擬器與虛擬調試系統**（如 Trinamic、台達、研華、Galil、Modbus / EtherCAT 等）
  - **Serial / CAN / TCP** 底層通訊協定整合與 **3D 數位孿生（Digital Twin）** 視覺化系統

## 授權條款與第三方聲明

- 本模擬器原始碼採用 [MIT License](./LICENSE) 授權。
- [`TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf`](./TMCM-6110-TMCL_firmware_manual_fw1.47_rev1.23.pdf) 與 [`TMCL_reference.pdf`](./TMCL_reference.pdf) 版權歸屬 **Analog Devices / Trinamic**，僅作為通訊協定參考文件隨附於本專案中。
- **免責聲明**：本專案為獨立開發的**非官方開源模擬器**，與 **Analog Devices, Inc.** 或 **Trinamic** 無任何隸屬、背書或贊助關係。`TMCM-6110`、`TMCL`、`StallGuard2`、`TMC262` 等名稱與商標均歸屬其各自權利人所有。
