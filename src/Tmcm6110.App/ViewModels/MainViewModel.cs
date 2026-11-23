using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows;
using Tmcm6110.App.Localization;
using Tmcm6110.App.Rendering;
using Tmcm6110.Core.Config;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Tmcm6110.Core.Transport;

namespace Tmcm6110.App.ViewModels;

public record PacketLogEntry(
    string Timestamp,
    string Direction,
    string RawHex,
    string DecodedSummary,
    string ReplyHex,
    string ReplySummary
);

/// <summary>
/// Master ViewModel coordinating the TMCM-6110 Simulator Engine, Serial Port Server,
/// 3-Axis (Axis 0=Z, Axis 1=Y, Axis 2=X) ViewModels, and Binary Packet Sniffer Log.
/// </summary>
public class MainViewModel : INotifyPropertyChanged, IDisposable
{
    public Tmcm6110Controller Controller { get; }
    public TmclSerialServer SerialServer { get; }

    public ObservableCollection<AxisViewModel> AllAxes { get; } = new();
    public ObservableCollection<AxisViewModel> ActiveAxes { get; } = new();
    public ObservableCollection<SelectableOption<MachineSceneMode>> AvailableSceneModes { get; } = new();
    public ObservableCollection<SelectableOption<XyOriginPreset>> AvailableXyOrigins { get; } = new();
    public ObservableCollection<SelectableOption<ZOriginPreset>> AvailableZOrigins { get; } = new();
    public ObservableCollection<string> AvailableComPorts { get; } = new();
    public ObservableCollection<int> AvailableBaudRates { get; } = new() { 9600, 14400, 19200, 28800, 38400, 57600, 115200 };
    public ObservableCollection<PacketLogEntry> PacketLogs { get; } = new();

    private enum ConnectionStateKind { Disconnected, Connected, Error }
    private enum ConfigSourceKind { BuiltInDefault, AutoIni, ManualIni, ManualTmc, ManualMapping, AutoLoadFallback }

    private string _selectedComPort = "COM1";
    private int _selectedBaudRate = 9600;
    private bool _isSerialConnected;
    private ConnectionStateKind _connectionState = ConnectionStateKind.Disconnected;
    private string _lastSerialError = string.Empty;
    private string _connectionStatusText = string.Empty;
    private string _connectionStatusColor = "#FFA000";
    private ConfigSourceKind _configSource = ConfigSourceKind.BuiltInDefault;
    private string _loadedConfigFileName = string.Empty;
    private int _loadedConfigParamCount;
    private string _loadedConfigError = string.Empty;
    private string _loadedConfigSummary = string.Empty;
    private bool _hidePollingPacketsInLog = true;
    private double _stepsPerMm = 1000.0; // Default: 1000 microsteps = 1.0 mm in 3D view (-100mm..+100mm)
    private long _totalRxPackets;
    private MachineSceneMode _selectedSceneMode = MachineSceneMode.Gantry3Axis;
    private XyOriginPreset _selectedXyOrigin = XyOriginPreset.Center;
    private ZOriginPreset _selectedZOrigin = ZOriginPreset.Center;
    private bool _syncRfsDirectionWithOrigin;
    private string _legendTitle = string.Empty;
    private string _legendLine1 = string.Empty;
    private string _legendLine2 = string.Empty;
    private string _legendLine3 = string.Empty;
    private string _legendLine4 = string.Empty;

    public event Action<MachineSceneMode>? SceneModeChanged;

    public string SelectedComPort
    {
        get => _selectedComPort;
        set { _selectedComPort = value; OnPropertyChanged(); }
    }

    public int SelectedBaudRate
    {
        get => _selectedBaudRate;
        set { _selectedBaudRate = value; OnPropertyChanged(); }
    }

    public bool IsSerialConnected
    {
        get => _isSerialConnected;
        set
        {
            _isSerialConnected = value;
            OnPropertyChanged();
            OnPropertyChanged(nameof(ConnectButtonText));
        }
    }

    public string ConnectButtonText => IsSerialConnected
        ? LanguageManager.Get("中斷 Serial 連線", "Disconnect Serial")
        : LanguageManager.Get("開啟 Serial Port 監聽", "Open Serial Port");

    public string ConnectionStatusText
    {
        get => _connectionStatusText;
        set { _connectionStatusText = value; OnPropertyChanged(); }
    }

    public string ConnectionStatusColor
    {
        get => _connectionStatusColor;
        set { _connectionStatusColor = value; OnPropertyChanged(); }
    }

    public string LoadedConfigSummary
    {
        get => _loadedConfigSummary;
        set { _loadedConfigSummary = value; OnPropertyChanged(); }
    }

    public bool HidePollingPacketsInLog
    {
        get => _hidePollingPacketsInLog;
        set { _hidePollingPacketsInLog = value; OnPropertyChanged(); }
    }

    public double StepsPerMm
    {
        get => _stepsPerMm;
        set
        {
            if (value >= 10 && Math.Abs(_stepsPerMm - value) > 1e-6)
            {
                _stepsPerMm = value;
                OnPropertyChanged();
                foreach (var axisVm in AllAxes)
                    axisVm.ApplyStrokeLimitsToAxis();
            }
        }
    }

    public MachineSceneMode SelectedSceneMode
    {
        get => _selectedSceneMode;
        set
        {
            if (_selectedSceneMode == value) return;
            _selectedSceneMode = value;
            OnPropertyChanged();
            RebuildActiveAxes();
            foreach (var vm in AllAxes)
                vm.UpdateAxisTitle(_selectedSceneMode);
            RefreshLegendTexts();
            SceneModeChanged?.Invoke(_selectedSceneMode);
        }
    }

    public XyOriginPreset SelectedXyOrigin
    {
        get => _selectedXyOrigin;
        set
        {
            if (_selectedXyOrigin == value) return;
            _selectedXyOrigin = value;
            OnPropertyChanged();
            ApplyOriginPresets();
        }
    }

    public ZOriginPreset SelectedZOrigin
    {
        get => _selectedZOrigin;
        set
        {
            if (_selectedZOrigin == value) return;
            _selectedZOrigin = value;
            OnPropertyChanged();
            ApplyOriginPresets();
        }
    }

    public bool SyncRfsDirectionWithOrigin
    {
        get => _syncRfsDirectionWithOrigin;
        set
        {
            if (_syncRfsDirectionWithOrigin == value) return;
            _syncRfsDirectionWithOrigin = value;
            OnPropertyChanged();
            if (value)
                ApplyOriginPresets();
        }
    }

    public string LegendTitle
    {
        get => _legendTitle;
        set { _legendTitle = value; OnPropertyChanged(); }
    }

    public string LegendLine1
    {
        get => _legendLine1;
        set { _legendLine1 = value; OnPropertyChanged(); }
    }

    public string LegendLine2
    {
        get => _legendLine2;
        set { _legendLine2 = value; OnPropertyChanged(); }
    }

    public string LegendLine3
    {
        get => _legendLine3;
        set { _legendLine3 = value; OnPropertyChanged(); }
    }

    public string LegendLine4
    {
        get => _legendLine4;
        set { _legendLine4 = value; OnPropertyChanged(); }
    }

    public long TotalRxPackets
    {
        get => _totalRxPackets;
        set { _totalRxPackets = value; OnPropertyChanged(); }
    }

    public byte ModuleAddress
    {
        get => Controller.ModuleAddress;
        set
        {
            Controller.ModuleAddress = value;
            OnPropertyChanged();
        }
    }

    public byte HostAddress
    {
        get => Controller.HostAddress;
        set
        {
            Controller.HostAddress = value;
            OnPropertyChanged();
        }
    }

    public MainViewModel()
    {
        Controller = new Tmcm6110Controller();
        SerialServer = new TmclSerialServer(Controller);

        // Create ViewModels for all 6 TMCM-6110 axes; ActiveAxes exposes 3 or 6 depending on SceneMode
        for (int i = 0; i < Tmcm6110Controller.TotalAxes; i++)
            AllAxes.Add(new AxisViewModel(Controller.GetAxis(i), Controller, () => StepsPerMm));

        InitializeSelectableOptions();
        RebuildActiveAxes();

        Controller.PacketProcessed += OnControllerPacketProcessed;
        SerialServer.StatusLogMessage += OnServerStatusLogMessage;
        LanguageManager.LanguageChanged += OnLanguageChanged;

        RefreshComPorts();
        TryAutoLoadLocalConfigFiles();

        // Also start local TCP Server on Port 4001 for convenient zero-driver testing
        SerialServer.StartTcpServer(4001);
        RefreshLocalizedTexts();
    }

    public void RefreshComPorts()
    {
        string prev = SelectedComPort;
        AvailableComPorts.Clear();
        var ports = TmclSerialServer.GetAvailableComPorts();
        if (ports.Length == 0)
        {
            // Populate common virtual COM port names if no physical port is currently attached
            foreach (var p in new[] { "COM1", "COM2", "COM3", "COM4", "COM10", "COM11" })
                AvailableComPorts.Add(p);
        }
        else
        {
            foreach (var p in ports)
                AvailableComPorts.Add(p);
        }

        SelectedComPort = AvailableComPorts.Contains(prev) ? prev : AvailableComPorts[0];
    }

    public void ToggleSerialConnection()
    {
        if (SerialServer.IsSerialOpen)
        {
            SerialServer.CloseSerialPort();
            IsSerialConnected = false;
            _connectionState = ConnectionStateKind.Disconnected;
            ConnectionStatusColor = "#FFA000";
            RefreshConnectionStatusText();
        }
        else
        {
            try
            {
                SerialServer.OpenSerialPort(SelectedComPort, SelectedBaudRate);
                IsSerialConnected = true;
                _connectionState = ConnectionStateKind.Connected;
                ConnectionStatusColor = "#00E676";
                RefreshConnectionStatusText();
            }
            catch (Exception ex)
            {
                IsSerialConnected = false;
                _connectionState = ConnectionStateKind.Error;
                _lastSerialError = ex.Message;
                ConnectionStatusColor = "#FF5252";
                RefreshConnectionStatusText();
                MessageBox.Show(
                    LanguageManager.Get(
                        $"無法開啟串列埠 {SelectedComPort}：\n{ex.Message}\n\n提示：若要在同一台電腦與上位機對接，請使用 com0com 建立虛擬成對串列埠（例如 COM10 <-> COM11），上位機連 COM10，本模擬器連 COM11。",
                        $"Unable to open Serial Port {SelectedComPort}:\n{ex.Message}\n\nTip: To connect locally with a host app on the same PC, create a virtual COM port pair using com0com (e.g., COM10 <-> COM11), or connect directly via TCP 127.0.0.1:4001."),
                    LanguageManager.Get("Serial Port 連線提示", "Serial Port Connection"),
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
            }
        }
    }

    private void OnLanguageChanged()
    {
        RefreshLocalizedTexts();
    }

    private void InitializeSelectableOptions()
    {
        AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(MachineSceneMode.Gantry3Axis, string.Empty));
        AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(MachineSceneMode.DualGantry6Axis, string.Empty));
        AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(MachineSceneMode.GantryRotaryAux6Axis, string.Empty));
        AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(MachineSceneMode.RobotArm6Dof, string.Empty));
        AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(MachineSceneMode.LinearBench6Axis, string.Empty));

        AvailableXyOrigins.Add(new SelectableOption<XyOriginPreset>(XyOriginPreset.Center, string.Empty));
        AvailableXyOrigins.Add(new SelectableOption<XyOriginPreset>(XyOriginPreset.FrontLeft, string.Empty));
        AvailableXyOrigins.Add(new SelectableOption<XyOriginPreset>(XyOriginPreset.BackLeft, string.Empty));
        AvailableXyOrigins.Add(new SelectableOption<XyOriginPreset>(XyOriginPreset.FrontRight, string.Empty));
        AvailableXyOrigins.Add(new SelectableOption<XyOriginPreset>(XyOriginPreset.BackRight, string.Empty));

        AvailableZOrigins.Add(new SelectableOption<ZOriginPreset>(ZOriginPreset.Center, string.Empty));
        AvailableZOrigins.Add(new SelectableOption<ZOriginPreset>(ZOriginPreset.Top, string.Empty));
        AvailableZOrigins.Add(new SelectableOption<ZOriginPreset>(ZOriginPreset.Bottom, string.Empty));
    }

    private void RebuildActiveAxes()
    {
        int count = _selectedSceneMode == MachineSceneMode.Gantry3Axis ? 3 : Tmcm6110Controller.TotalAxes;
        ActiveAxes.Clear();
        for (int i = 0; i < count && i < AllAxes.Count; i++)
            ActiveAxes.Add(AllAxes[i]);
    }

    private void ApplyOriginPresets()
    {
        ApplyXyOriginToAxisPair(xIdx: 2, yIdx: 1);
        ApplyZOriginToAxis(zIdx: 0);

        if (_selectedSceneMode == MachineSceneMode.DualGantry6Axis)
        {
            ApplyXyOriginToAxisPair(xIdx: 5, yIdx: 4);
            ApplyZOriginToAxis(zIdx: 3);
        }
    }

    private void ApplyXyOriginToAxisPair(int xIdx, int yIdx)
    {
        var axX = AllAxes[xIdx];
        var axY = AllAxes[yIdx];
        double lenX = axX.StrokeLengthMm;
        double lenY = axY.StrokeLengthMm;

        (double minX, double maxX) = _selectedXyOrigin switch
        {
            XyOriginPreset.FrontLeft or XyOriginPreset.BackLeft => (0.0, lenX),
            XyOriginPreset.FrontRight or XyOriginPreset.BackRight => (-lenX, 0.0),
            _ => (-lenX * 0.5, lenX * 0.5)
        };

        (double minY, double maxY) = _selectedXyOrigin switch
        {
            XyOriginPreset.FrontLeft or XyOriginPreset.FrontRight => (0.0, lenY),
            XyOriginPreset.BackLeft or XyOriginPreset.BackRight => (-lenY, 0.0),
            _ => (-lenY * 0.5, lenY * 0.5)
        };

        axX.SetStrokeLimitsMm(minX, maxX);
        axY.SetStrokeLimitsMm(minY, maxY);

        if (_syncRfsDirectionWithOrigin)
        {
            SetRfsModeForOrigin(xIdx, maxX <= 0.0 ? 65 : 1);
            SetRfsModeForOrigin(yIdx, maxY <= 0.0 ? 65 : 1);
        }
    }

    private void ApplyZOriginToAxis(int zIdx)
    {
        var axZ = AllAxes[zIdx];
        double lenZ = axZ.StrokeLengthMm;

        (double minZ, double maxZ) = _selectedZOrigin switch
        {
            ZOriginPreset.Top => (-lenZ, 0.0),
            ZOriginPreset.Bottom => (0.0, lenZ),
            _ => (-lenZ * 0.5, lenZ * 0.5)
        };

        axZ.SetStrokeLimitsMm(minZ, maxZ);

        if (_syncRfsDirectionWithOrigin)
            SetRfsModeForOrigin(zIdx, maxZ <= 0.0 ? 65 : 1);
    }

    private void SetRfsModeForOrigin(int axisIndex, int rfsMode)
    {
        Controller.ProcessCommand(new TmclCommandPacket(
            ModuleAddress, (byte)TmclCommand.SAP, TmclAxisParam.ReferenceSearchMode, (byte)axisIndex, rfsMode));
    }

    private void RefreshLocalizedTexts()
    {
        OnPropertyChanged(nameof(ConnectButtonText));
        RefreshConnectionStatusText();
        RefreshLoadedConfigSummary();
        RefreshOptionLabels();
        RefreshLegendTexts();
        foreach (var vm in AllAxes)
            vm.UpdateAxisTitle(_selectedSceneMode);
    }

    private void RefreshOptionLabels()
    {
        foreach (var opt in AvailableSceneModes)
        {
            opt.Label = opt.Value switch
            {
                MachineSceneMode.Gantry3Axis => LanguageManager.Get("3 軸龍門（預設 Z/Y/X）", "3-Axis Gantry (Default Z/Y/X)"),
                MachineSceneMode.DualGantry6Axis => LanguageManager.Get("6 軸雙龍門（雙工位 Z1/Y1/X1 + Z2/Y2/X2）", "6-Axis Dual Gantry (Stations 1 & 2)"),
                MachineSceneMode.GantryRotaryAux6Axis => LanguageManager.Get("6 軸龍門 + 旋轉/輔助軸 (XYZ + R/A/B)", "6-Axis Gantry + Rotary/Aux (XYZ + R/A/B)"),
                MachineSceneMode.RobotArm6Dof => LanguageManager.Get("6 軸關節機械手臂 (J1~J6)", "6-Axis Robot Arm (J1-J6)"),
                MachineSceneMode.LinearBench6Axis => LanguageManager.Get("6 軸獨立線性測試台 (Ch 0~5)", "6-Axis Linear Test Bench (Ch 0-5)"),
                _ => opt.Value.ToString()
            };
        }

        foreach (var opt in AvailableXyOrigins)
        {
            opt.Label = opt.Value switch
            {
                XyOriginPreset.Center => LanguageManager.Get("工作台中心 Center (0, 0)", "Table Center (0, 0)"),
                XyOriginPreset.FrontLeft => LanguageManager.Get("左下角 Front-Left (-X, -Y)", "Front-Left Corner (-X, -Y)"),
                XyOriginPreset.BackLeft => LanguageManager.Get("左上角 Back-Left (-X, +Y)", "Back-Left Corner (-X, +Y)"),
                XyOriginPreset.FrontRight => LanguageManager.Get("右下角 Front-Right (+X, -Y)", "Front-Right Corner (+X, -Y)"),
                XyOriginPreset.BackRight => LanguageManager.Get("右上角 Back-Right (+X, +Y)", "Back-Right Corner (+X, +Y)"),
                _ => opt.Value.ToString()
            };
        }

        foreach (var opt in AvailableZOrigins)
        {
            opt.Label = opt.Value switch
            {
                ZOriginPreset.Center => LanguageManager.Get("Z 軸中心 Center (-Lz/2 ~ +Lz/2)", "Center (-Lz/2 ~ +Lz/2)"),
                ZOriginPreset.Top => LanguageManager.Get("頂部 Top (+Z 工業標準 -Lz ~ 0)", "Top (+Z Industrial -Lz ~ 0)"),
                ZOriginPreset.Bottom => LanguageManager.Get("底部 Bottom (-Z 工作台面 0 ~ +Lz)", "Bottom (-Z Table 0 ~ +Lz)"),
                _ => opt.Value.ToString()
            };
        }
    }

    private void RefreshLegendTexts()
    {
        switch (_selectedSceneMode)
        {
            case MachineSceneMode.DualGantry6Axis:
                LegendTitle = LanguageManager.Get("3D 雙龍門雙工位視圖 (6 軸)", "3D Dual-Gantry Dual-Station (6-Axis)");
                LegendLine1 = LanguageManager.Get("• 左站 (Axis 0~2): Z1 藍 / Y1 綠 / X1 紅", "• Left Station (Axis 0-2): Z1 / Y1 / X1");
                LegendLine2 = LanguageManager.Get("• 右站 (Axis 3~5): Z2 青 / Y2 藍綠 / X2 紫", "• Right Station (Axis 3-5): Z2 / Y2 / X2");
                LegendLine3 = LanguageManager.Get("• 白點三色標 = 各站機械原點 (0, 0)", "• White Marker = Station Origin (0, 0)");
                LegendLine4 = LanguageManager.Get("• 金色 / 青色光帶 = 左站 / 右站 TCP 軌跡", "• Gold / Cyan Ribbon = Station 1 / 2 Trails");
                break;
            case MachineSceneMode.GantryRotaryAux6Axis:
                LegendTitle = LanguageManager.Get("3D 龍門 + 旋轉/輔助視圖 (6 軸)", "3D Gantry + Rotary/Aux Stage (6-Axis)");
                LegendLine1 = LanguageManager.Get("• 龍門主軸 (Axis 0~2): Z 藍 / Y 綠 / X 紅", "• Main Gantry (Axis 0-2): Z / Y / X");
                LegendLine2 = LanguageManager.Get("• Axis 3 (R) = 主軸旋轉 | Axis 4 (A) = 轉盤", "• Axis 3 (R) = Spindle | Axis 4 (A) = Table");
                LegendLine3 = LanguageManager.Get("• Axis 5 (B) = 右側靛色輔助供料滑台", "• Axis 5 (B) = Indigo Aux Feeder Slide");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 3D 工具尖端 (TCP) 軌跡", "• Gold Ribbon = 3D Tool Center Point Trail");
                break;
            case MachineSceneMode.RobotArm6Dof:
                LegendTitle = LanguageManager.Get("3D 六軸關節機械手臂 (J1~J6)", "3D 6-DOF Articulated Robot Arm (J1-J6)");
                LegendLine1 = LanguageManager.Get("• J1 (Axis 0) 底座 | J2 (Axis 1) 肩 | J3 (Axis 2) 肘", "• J1 (Axis 0) Base | J2 (Axis 1) Shoulder | J3 (Axis 2) Elbow");
                LegendLine2 = LanguageManager.Get("• J4 (Axis 3) 前臂 | J5 (Axis 4) 腕 | J6 (Axis 5) 法蘭", "• J4 (Axis 3) Forearm | J5 (Axis 4) Wrist | J6 (Axis 5) Flange");
                LegendLine3 = LanguageManager.Get("• 關節行程對應各軸 ±Limit 範圍 (-100%~+100%)", "• Joint angles map linearly across axis ±Limit span");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 手臂法蘭 TCP 空間軌跡", "• Gold Ribbon = Flange TCP 3D Trajectory");
                break;
            case MachineSceneMode.LinearBench6Axis:
                LegendTitle = LanguageManager.Get("3D 六軸線性測試台 (Ch 0~5)", "3D 6-Axis Linear Test Bench (Ch 0-5)");
                LegendLine1 = LanguageManager.Get("• 6 條並排滑軌分別對應 Motor #0 ~ #5", "• 6 Parallel Rails mapped to Motor #0 - #5");
                LegendLine2 = LanguageManager.Get("• 中央灰藍橫條 = 0 mm 原點參考線", "• Center Blue-Gray Bar = 0 mm Reference Line");
                LegendLine3 = LanguageManager.Get("• 滑軌兩端 LED = 獨立 -Limit / +Limit 狀態", "• Rail End LEDs = Independent -Limit / +Limit");
                LegendLine4 = LanguageManager.Get("• 適用全 6 軸通訊與極限批量驗證", "• Ideal for 6-axis protocol & limit testing");
                break;
            default:
                LegendTitle = LanguageManager.Get("3D 龍門機構動態視圖 (Z-Y-X 映射)", "3D Gantry Stage Live View (Z-Y-X Mapping)");
                LegendLine1 = LanguageManager.Get("• 藍色垂直主軸 = Motor #0 (Z 軸 - 升降)", "• Blue Vertical Spindle = Motor #0 (Z-Axis)");
                LegendLine2 = LanguageManager.Get("• 綠色前後龍門 = Motor #1 (Y 軸 - 前後)", "• Green Gantry Bridge = Motor #1 (Y-Axis)");
                LegendLine3 = LanguageManager.Get("• 紅色左右橫移 = Motor #2 (X 軸 - 左右)", "• Red Cross Carriage = Motor #2 (X-Axis)");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = TCP 軌跡 | 白點 = 機械原點 (0,0)", "• Gold Trail = TCP | White Cube = Origin (0,0)");
                break;
        }
    }

    private void RefreshConnectionStatusText()
    {
        ConnectionStatusText = _connectionState switch
        {
            ConnectionStateKind.Connected => LanguageManager.Get(
                $"Serial 監聽中: {SelectedComPort} @ {SelectedBaudRate} bps | Addr={ModuleAddress}, Host={HostAddress}",
                $"Serial Listening: {SelectedComPort} @ {SelectedBaudRate} bps | Addr={ModuleAddress}, Host={HostAddress}"),
            ConnectionStateKind.Error => LanguageManager.Get(
                $"開啟 {SelectedComPort} 失敗: {_lastSerialError}",
                $"Failed to open {SelectedComPort}: {_lastSerialError}"),
            _ => LanguageManager.Get(
                $"Serial Port 未連線 (TCP 127.0.0.1:{SerialServer.TcpPort} 監聽中)",
                $"Serial Disconnected (TCP 127.0.0.1:{SerialServer.TcpPort} Listening)")
        };
    }

    private void RefreshLoadedConfigSummary()
    {
        LoadedConfigSummary = _configSource switch
        {
            ConfigSourceKind.AutoIni => LanguageManager.Get(
                $"已自動載入設定檔: {_loadedConfigFileName} ({_loadedConfigParamCount} 項參數 | 0=Z, 1=Y, 2=X | RFS Mode=65)",
                $"Auto-loaded config: {_loadedConfigFileName} ({_loadedConfigParamCount} params | 0=Z, 1=Y, 2=X | RFS Mode=65)"),
            ConfigSourceKind.ManualIni => LanguageManager.Get(
                $"已載入 INI 設定檔: {_loadedConfigFileName} (套用 {_loadedConfigParamCount} 項參數)",
                $"Loaded INI config: {_loadedConfigFileName} ({_loadedConfigParamCount} params applied)"),
            ConfigSourceKind.ManualTmc => LanguageManager.Get(
                $"已載入 TMC 腳本檔: {_loadedConfigFileName} (套用 {_loadedConfigParamCount} 筆 SAP/SGP 指令)",
                $"Loaded TMC script: {_loadedConfigFileName} ({_loadedConfigParamCount} SAP/SGP commands applied)"),
            ConfigSourceKind.ManualMapping => LanguageManager.Get(
                $"已載入軸向定義檔: {_loadedConfigFileName}",
                $"Loaded axis mapping: {_loadedConfigFileName}"),
            ConfigSourceKind.AutoLoadFallback => LanguageManager.Get(
                $"預設參數已就緒 (載入外部檔提示: {_loadedConfigError})",
                $"Built-in defaults active (config note: {_loadedConfigError})"),
            _ => LanguageManager.Get(
                "已載入內建預設參數 (Axis0=Z, Axis1=Y, Axis2=X | RFS Mode=65)",
                "Built-in Defaults Active (Axis0=Z, Axis1=Y, Axis2=X | RFS Mode=65)")
        };
    }

    private void TryAutoLoadLocalConfigFiles()
    {
        try
        {
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            string? labelPath = FindAxisMappingFileUpwards(baseDir);
            if (!string.IsNullOrEmpty(labelPath))
            {
                TmcmConfigLoader.LoadAxisLabelsFromFile(labelPath, Controller);
                foreach (var vm in AllAxes) vm.UpdateAxisTitle(_selectedSceneMode);
            }

            string? iniPath = FindFirstFileUpwards(baseDir, "*.ini");
            if (!string.IsNullOrEmpty(iniPath))
            {
                int count = TmcmConfigLoader.LoadFromIniFile(iniPath, Controller);
                _configSource = ConfigSourceKind.AutoIni;
                _loadedConfigFileName = Path.GetFileName(iniPath);
                _loadedConfigParamCount = count;
            }
            else
            {
                _configSource = ConfigSourceKind.BuiltInDefault;
            }
        }
        catch (Exception ex)
        {
            _configSource = ConfigSourceKind.AutoLoadFallback;
            _loadedConfigError = ex.Message;
        }

        RefreshLoadedConfigSummary();
    }

    private static string? FindFirstFileUpwards(string startDir, string searchPattern)
    {
        DirectoryInfo? dir = new DirectoryInfo(startDir);
        for (int i = 0; i < 6 && dir != null; i++)
        {
            string? match = dir.EnumerateFiles(searchPattern).Select(f => f.FullName).FirstOrDefault();
            if (!string.IsNullOrEmpty(match)) return match;
            dir = dir.Parent;
        }
        return null;
    }

    private static string? FindAxisMappingFileUpwards(string startDir)
    {
        DirectoryInfo? dir = new DirectoryInfo(startDir);
        for (int i = 0; i < 6 && dir != null; i++)
        {
            foreach (var file in dir.EnumerateFiles("*.txt"))
            {
                if (File.ReadLines(file.FullName).Take(20).Any(l => l.Contains("axis 0", StringComparison.OrdinalIgnoreCase)))
                    return file.FullName;
            }
            dir = dir.Parent;
        }
        return null;
    }

    public void LoadConfigFileFromDialog(string filePath)
    {
        try
        {
            string ext = Path.GetExtension(filePath).ToLowerInvariant();
            if (ext == ".ini")
            {
                int count = TmcmConfigLoader.LoadFromIniFile(filePath, Controller);
                _configSource = ConfigSourceKind.ManualIni;
                _loadedConfigFileName = Path.GetFileName(filePath);
                _loadedConfigParamCount = count;
            }
            else if (ext == ".tmc")
            {
                int count = TmcmConfigLoader.LoadFromTmcFile(filePath, Controller);
                _configSource = ConfigSourceKind.ManualTmc;
                _loadedConfigFileName = Path.GetFileName(filePath);
                _loadedConfigParamCount = count;
            }
            else if (ext == ".txt")
            {
                TmcmConfigLoader.LoadAxisLabelsFromFile(filePath, Controller);
                foreach (var vm in AllAxes) vm.UpdateAxisTitle(_selectedSceneMode);
                _configSource = ConfigSourceKind.ManualMapping;
                _loadedConfigFileName = Path.GetFileName(filePath);
            }

            RefreshLoadedConfigSummary();
            OnPropertyChanged(nameof(ModuleAddress));
            OnPropertyChanged(nameof(HostAddress));
        }
        catch (Exception ex)
        {
            MessageBox.Show(
                LanguageManager.Get($"載入設定檔失敗:\n{ex.Message}", $"Failed to load configuration file:\n{ex.Message}"),
                LanguageManager.Get("載入錯誤", "Load Error"),
                MessageBoxButton.OK,
                MessageBoxImage.Error);
        }
    }

    /// <summary>
    /// Demo macro: Executes a multi-axis Coordinated Interpolated Move (MVP COORD, 0x40|mask)
    /// across all currently active axes using normalized stroke waypoints (safe for Center &amp; Corner origins),
    /// and arms Command 138 Position Reached Notification.
    /// </summary>
    public void ExecuteDemoInterpolatedStep(int stepIndex)
    {
        double[][] waypoints =
        [
            [0.85, 0.82, 0.85, 0.20, 0.80, 0.25],
            [0.25, 0.82, 0.15, 0.80, 0.20, 0.75],
            [0.72, 0.18, 0.15, 0.30, 0.20, 0.80],
            [0.18, 0.18, 0.85, 0.75, 0.80, 0.20],
            [0.50, 0.50, 0.50, 0.50, 0.50, 0.50]
        ];

        double[] frac = waypoints[stepIndex % waypoints.Length];
        int mask = 0;

        foreach (var axVm in ActiveAxes)
        {
            int idx = axVm.AxisIndex;
            var snap = Controller.GetAxis(idx).GetSnapshot();
            double minS = snap.VirtualMinStrokeSteps;
            double maxS = snap.VirtualMaxStrokeSteps;
            int targetPos = (int)Math.Round(minS + (maxS - minS) * frac[idx]);

            Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.SCO, 1, (byte)idx, targetPos));
            mask |= (1 << idx);
        }

        if (mask == 0) return;

        byte coordMotMask = (byte)(0x40 | mask);
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.MVP, 2, coordMotMask, 1));
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, mask));
    }

    public void StopAllAxes()
    {
        foreach (var axVm in ActiveAxes)
            Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.MST, 0, (byte)axVm.AxisIndex, 0));
    }

    public void HomeAllAxes()
    {
        foreach (var axVm in ActiveAxes)
            Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.RFS, 0, (byte)axVm.AxisIndex, 0));
    }

    public void ClearAllAlarms()
    {
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.CLE, 0, 0, 0));
    }

    private void OnControllerPacketProcessed(TmclCommandPacket cmd, TmclReplyPacket reply)
    {
        Interlocked.Increment(ref _totalRxPackets);

        if (HidePollingPacketsInLog && cmd.IsPollingQuery)
        {
            Application.Current?.Dispatcher.BeginInvoke(() => OnPropertyChanged(nameof(TotalRxPackets)));
            return;
        }

        var entry = new PacketLogEntry(
            DateTime.Now.ToString("HH:mm:ss.fff"),
            "RX -> TX",
            cmd.ToHexString(),
            cmd.ToMnemonicDescription(Controller.GetAxisDisplayLabel),
            reply.ToHexString(),
            reply.ToDescription()
        );

        Application.Current?.Dispatcher.BeginInvoke(() =>
        {
            OnPropertyChanged(nameof(TotalRxPackets));
            PacketLogs.Insert(0, entry);
            if (PacketLogs.Count > 250)
                PacketLogs.RemoveAt(PacketLogs.Count - 1);
        });
    }

    private void OnServerStatusLogMessage(string msg)
    {
        var entry = new PacketLogEntry(
            DateTime.Now.ToString("HH:mm:ss.fff"),
            "SYS/EVENT",
            "-",
            msg,
            "-",
            "-"
        );

        Application.Current?.Dispatcher.BeginInvoke(() =>
        {
            PacketLogs.Insert(0, entry);
            if (PacketLogs.Count > 250)
                PacketLogs.RemoveAt(PacketLogs.Count - 1);
        });
    }

    public void Dispose()
    {
        LanguageManager.LanguageChanged -= OnLanguageChanged;
        Controller.PacketProcessed -= OnControllerPacketProcessed;
        SerialServer.StatusLogMessage -= OnServerStatusLogMessage;
        SerialServer.Dispose();
    }

    public event PropertyChangedEventHandler? PropertyChanged;
    protected void OnPropertyChanged([CallerMemberName] string? name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
