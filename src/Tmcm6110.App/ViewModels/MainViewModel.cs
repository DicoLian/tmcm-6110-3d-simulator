using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Threading;
using Tmcm6110.App.Localization;
using Tmcm6110.App.Rendering;
using Tmcm6110.Core.Abstractions;
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
    public AxisRoleMap RoleMap { get; } = new();

    public ObservableCollection<AxisViewModel> AllAxes { get; } = new();
    public ObservableCollection<AxisViewModel> ActiveAxes { get; } = new();
    public ObservableCollection<SelectableOption<int>> AvailableAxisCounts { get; } = new();
    public ObservableCollection<SelectableOption<int>> AvailableMotorOptions { get; } = new();
    public ObservableCollection<SelectableOption<int>> AvailableRoleOptions { get; } = new();
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
    private int _selectedAxisCount = 3;
    private MachineSceneMode _selectedSceneMode = MachineSceneMode.Gantry3Axis;
    private XyOriginPreset _selectedXyOrigin = XyOriginPreset.Center;
    private ZOriginPreset _selectedZOrigin = ZOriginPreset.Center;
    private bool _syncRfsDirectionWithOrigin;
    private string _legendTitle = string.Empty;
    private string _legendLine1 = string.Empty;
    private string _legendLine2 = string.Empty;
    private string _legendLine3 = string.Empty;
    private string _legendLine4 = string.Empty;
    private bool _isSwappingRoles;
    private bool _isUpdatingSceneOptions;

    public event Action<MachineSceneMode>? SceneModeChanged;
    public event Action? AxisMappingChanged;

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

    public int SelectedAxisCount
    {
        get => _selectedAxisCount;
        set
        {
            int clamped = Math.Clamp(value, 1, Tmcm6110Controller.TotalAxes);
            if (_selectedAxisCount == clamped) return;

            _selectedAxisCount = clamped;
            OnPropertyChanged();
            OnPropertyChanged(nameof(CanApplyStandardXyzMapping));

            InvokeDeferred(ApplyAxisCountSelection);
        }
    }

    public bool CanApplyStandardXyzMapping => _selectedAxisCount >= 3;

    public MachineSceneMode SelectedSceneMode
    {
        get => _selectedSceneMode;
        set
        {
            if (_isUpdatingSceneOptions || _selectedSceneMode == value) return;
            _selectedSceneMode = value;
            OnPropertyChanged();
            InvokeDeferred(() => ApplySceneAndAxisCountUpdate(_selectedSceneMode));
        }
    }

    private static void InvokeDeferred(Action action)
    {
        var dispatcher = Application.Current?.Dispatcher;
        if (dispatcher != null)
            dispatcher.BeginInvoke(DispatcherPriority.Background, action);
        else
            action();
    }

    private void ApplyAxisCountSelection()
    {
        var supportedModes = MachineSceneCatalog.GetModesForAxisCount(_selectedAxisCount);
        MachineSceneMode targetMode = supportedModes.Contains(_selectedSceneMode)
            ? _selectedSceneMode
            : supportedModes[0];

        SyncAvailableSceneModesForAxisCount(supportedModes, targetMode);
        ApplySceneAndAxisCountUpdate(targetMode);
    }

    private void SyncAvailableSceneModesForAxisCount(IReadOnlyList<MachineSceneMode> supportedModes, MachineSceneMode targetMode)
    {
        _isUpdatingSceneOptions = true;
        try
        {
            for (int i = 0; i < supportedModes.Count; i++)
            {
                var mode = supportedModes[i];
                if (AvailableSceneModes.All(opt => opt.Value != mode))
                    AvailableSceneModes.Insert(Math.Min(i, AvailableSceneModes.Count), new SelectableOption<MachineSceneMode>(mode, string.Empty));
            }

            _selectedSceneMode = targetMode;
            OnPropertyChanged(nameof(SelectedSceneMode));

            for (int i = AvailableSceneModes.Count - 1; i >= 0; i--)
            {
                if (!supportedModes.Contains(AvailableSceneModes[i].Value))
                    AvailableSceneModes.RemoveAt(i);
            }

            RefreshSceneModeOptionLabels();
        }
        finally
        {
            _isUpdatingSceneOptions = false;
        }
    }

    private void ApplySceneAndAxisCountUpdate(MachineSceneMode targetMode)
    {
        _selectedSceneMode = targetMode;
        OnPropertyChanged(nameof(SelectedSceneMode));

        if (_selectedAxisCount > AvailableRoleOptions.Count)
        {
            SyncAvailableRoleOptions();
            RebuildActiveAxes();
        }
        else
        {
            RebuildActiveAxes();
            SyncAvailableRoleOptions();
        }

        _isSwappingRoles = true;
        try
        {
            for (int m = 0; m < AllAxes.Count; m++)
                AllAxes[m].SetAssignedRoleSlotInternal(RoleMap.GetRoleForMotor(m), _selectedSceneMode);
        }
        finally
        {
            _isSwappingRoles = false;
        }

        SyncControllerAxisNamesFromRoles();
        RefreshLegendTexts();
        SceneModeChanged?.Invoke(_selectedSceneMode);
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

        // Create ViewModels for all 6 TMCM-6110 axes; ActiveAxes exposes 1..6 depending on SelectedAxisCount
        for (int i = 0; i < Tmcm6110Controller.TotalAxes; i++)
        {
            AllAxes.Add(new AxisViewModel(
                Controller.GetAxis(i),
                Controller,
                () => StepsPerMm,
                AvailableRoleOptions,
                OnAxisRoleChangeRequested,
                AvailableMotorOptions,
                OnMotorChannelChangeRequested));
        }

        InitializeSelectableOptions();
        SyncAvailableRoleOptions();
        RebuildActiveAxes();
        SyncControllerAxisNamesFromRoles();

        SerialServer.ProtocolAdapter.TraceGenerated += OnProtocolTraceGenerated;
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
        for (int count = 1; count <= Tmcm6110Controller.TotalAxes; count++)
            AvailableAxisCounts.Add(new SelectableOption<int>(count, string.Empty));

        for (int m = 0; m < Tmcm6110Controller.TotalAxes; m++)
            AvailableMotorOptions.Add(new SelectableOption<int>(m, $"Motor #{m}"));

        foreach (var mode in MachineSceneCatalog.GetModesForAxisCount(_selectedAxisCount))
            AvailableSceneModes.Add(new SelectableOption<MachineSceneMode>(mode, string.Empty));

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
        var desired = new List<AxisViewModel>(_selectedAxisCount);
        for (int i = 0; i < AllAxes.Count; i++)
        {
            if (RoleMap.GetRoleForMotor(i) < _selectedAxisCount)
                desired.Add(AllAxes[i]);
        }

        if (ActiveAxes.Count == desired.Count)
        {
            bool identical = true;
            for (int i = 0; i < desired.Count; i++)
            {
                if (!ReferenceEquals(ActiveAxes[i], desired[i]))
                {
                    identical = false;
                    break;
                }
            }
            if (identical)
                return;
        }

        ActiveAxes.Clear();
        foreach (var ax in desired)
            ActiveAxes.Add(ax);
    }

    public int GetMotorIndexForRole(int roleSlot) => RoleMap.GetMotorForRole(roleSlot);

    public void ResetDefaultAxisMapping() => ApplyFirstThreeRoleOrder(0, 1, 2);

    public void ApplyStandardXyzMapping()
    {
        if (_selectedAxisCount >= 3)
            ApplyFirstThreeRoleOrder(2, 1, 0);
    }

    private void ApplyFirstThreeRoleOrder(int r0, int r1, int r2)
    {
        if (_isSwappingRoles) return;
        _isSwappingRoles = true;
        try
        {
            var strokeByRole = CaptureStrokesByRole();
            if (r0 == 0 && r1 == 1 && r2 == 2)
                RoleMap.ResetToIdentity();
            else
                RoleMap.ApplyOrderForFirstThree(r0, r1, r2);

            RestoreStrokesByRole(strokeByRole);
            for (int m = 0; m < AllAxes.Count; m++)
                AllAxes[m].SetAssignedRoleSlotInternal(RoleMap.GetRoleForMotor(m), _selectedSceneMode);
            RebuildActiveAxes();

            SyncControllerAxisNamesFromRoles();
            RefreshLegendTexts();
            AxisMappingChanged?.Invoke();
        }
        finally
        {
            _isSwappingRoles = false;
        }
    }

    private void OnAxisRoleChangeRequested(int motorIndex, int targetRoleSlot)
    {
        if (_isSwappingRoles) return;
        _isSwappingRoles = true;
        try
        {
            if (!RoleMap.TrySwapMotorRole(motorIndex, targetRoleSlot, _selectedAxisCount, out int otherMotor, out int oldRole))
            {
                AllAxes[motorIndex].SetAssignedRoleSlotInternal(RoleMap.GetRoleForMotor(motorIndex), _selectedSceneMode);
                return;
            }

            double minA = AllAxes[motorIndex].MinLimitMm;
            double maxA = AllAxes[motorIndex].MaxLimitMm;
            double minB = AllAxes[otherMotor].MinLimitMm;
            double maxB = AllAxes[otherMotor].MaxLimitMm;
            AllAxes[motorIndex].SetStrokeLimitsMm(minB, maxB);
            AllAxes[otherMotor].SetStrokeLimitsMm(minA, maxA);

            AllAxes[motorIndex].SetAssignedRoleSlotInternal(targetRoleSlot, _selectedSceneMode);
            AllAxes[otherMotor].SetAssignedRoleSlotInternal(oldRole, _selectedSceneMode);

            SyncControllerAxisNamesFromRoles();
            RefreshLegendTexts();
            AxisMappingChanged?.Invoke();
        }
        finally
        {
            _isSwappingRoles = false;
        }
    }

    private void OnMotorChannelChangeRequested(int currentMotorIndex, int targetMotorIndex)
    {
        if (_isSwappingRoles) return;
        _isSwappingRoles = true;
        try
        {
            if (!RoleMap.TrySwapMotorChannels(currentMotorIndex, targetMotorIndex))
            {
                AllAxes[currentMotorIndex].ResetSelectedMotorChannelInternal();
                return;
            }

            double minA = AllAxes[currentMotorIndex].MinLimitMm;
            double maxA = AllAxes[currentMotorIndex].MaxLimitMm;
            double minB = AllAxes[targetMotorIndex].MinLimitMm;
            double maxB = AllAxes[targetMotorIndex].MaxLimitMm;
            AllAxes[currentMotorIndex].SetStrokeLimitsMm(minB, maxB);
            AllAxes[targetMotorIndex].SetStrokeLimitsMm(minA, maxA);

            for (int m = 0; m < AllAxes.Count; m++)
                AllAxes[m].SetAssignedRoleSlotInternal(RoleMap.GetRoleForMotor(m), _selectedSceneMode);
            RebuildActiveAxes();

            SyncControllerAxisNamesFromRoles();
            RefreshLegendTexts();
            AxisMappingChanged?.Invoke();
        }
        finally
        {
            _isSwappingRoles = false;
        }
    }

    private (double min, double max)[] CaptureStrokesByRole()
    {
        var strokes = new (double min, double max)[Tmcm6110Controller.TotalAxes];
        for (int role = 0; role < Tmcm6110Controller.TotalAxes; role++)
        {
            int motor = RoleMap.GetMotorForRole(role);
            strokes[role] = (AllAxes[motor].MinLimitMm, AllAxes[motor].MaxLimitMm);
        }
        return strokes;
    }

    private void RestoreStrokesByRole((double min, double max)[] strokeByRole)
    {
        for (int m = 0; m < AllAxes.Count; m++)
        {
            int newRole = RoleMap.GetRoleForMotor(m);
            AllAxes[m].SetStrokeLimitsMm(strokeByRole[newRole].min, strokeByRole[newRole].max);
        }
    }

    private void SyncAvailableRoleOptions()
    {
        int targetCount = _selectedAxisCount;
        while (AvailableRoleOptions.Count < targetCount)
            AvailableRoleOptions.Add(new SelectableOption<int>(AvailableRoleOptions.Count, string.Empty));
        while (AvailableRoleOptions.Count > targetCount)
            AvailableRoleOptions.RemoveAt(AvailableRoleOptions.Count - 1);

        for (int slot = 0; slot < AvailableRoleOptions.Count; slot++)
            AvailableRoleOptions[slot].Label = GetRoleOptionLabel(slot, _selectedSceneMode);

        foreach (var vm in AllAxes)
            vm.NotifyRoleOptionsChanged();
    }

    private static string GetRoleOptionLabel(int roleSlot, MachineSceneMode mode)
    {
        return mode switch
        {
            MachineSceneMode.LinearStage1Axis => roleSlot switch
            {
                0 => LanguageManager.Get("X 軸 (線性滑台)", "X-Axis (Linear Stage)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.RotaryTable1Axis => roleSlot switch
            {
                0 => LanguageManager.Get("R 軸 (旋轉分度盤)", "R-Axis (Rotary Table)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.XyStage2Axis => roleSlot switch
            {
                0 => LanguageManager.Get("X 軸 (左右橫移)", "X-Axis (Left/Right)"),
                1 => LanguageManager.Get("Y 軸 (前後移動)", "Y-Axis (Front/Back)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.XzGantry2Axis => roleSlot switch
            {
                0 => LanguageManager.Get("Z 軸 (升降)", "Z-Axis (Vertical)"),
                1 => LanguageManager.Get("X 軸 (左右)", "X-Axis (Left/Right)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.XyRotary3Axis => roleSlot switch
            {
                0 => LanguageManager.Get("R 軸 (旋轉載台)", "R-Axis (Rotary)"),
                1 => LanguageManager.Get("Y 軸 (前後)", "Y-Axis (Front/Back)"),
                2 => LanguageManager.Get("X 軸 (左右)", "X-Axis (Left/Right)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.DualGantry6Axis => roleSlot switch
            {
                0 => LanguageManager.Get("Z1 (左站升降)", "Z1 (Left Vert)"),
                1 => LanguageManager.Get("Y1 (左站前後)", "Y1 (Left Depth)"),
                2 => LanguageManager.Get("X1 (左站左右)", "X1 (Left Cross)"),
                3 => LanguageManager.Get("Z2 (右站升降)", "Z2 (Right Vert)"),
                4 => LanguageManager.Get("Y2 (右站前後)", "Y2 (Right Depth)"),
                5 => LanguageManager.Get("X2 (右站左右)", "X2 (Right Cross)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.GantryRotary4Axis or
            MachineSceneMode.GantryDualRotary5Axis or
            MachineSceneMode.GantryRotaryAux6Axis => roleSlot switch
            {
                0 => LanguageManager.Get("Z 軸 (升降)", "Z-Axis (Vertical)"),
                1 => LanguageManager.Get("Y 軸 (前後)", "Y-Axis (Front/Back)"),
                2 => LanguageManager.Get("X 軸 (左右)", "X-Axis (Left/Right)"),
                3 => LanguageManager.Get("R 軸 (主軸旋轉)", "R-Axis (Spindle)"),
                4 => LanguageManager.Get("A 軸 (旋轉台)", "A-Axis (Table)"),
                5 => LanguageManager.Get("B 軸 (輔助滑台)", "B-Axis (Feeder)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.RobotArm6Dof => roleSlot switch
            {
                0 => LanguageManager.Get("J1 (底座旋轉)", "J1 (Waist)"),
                1 => LanguageManager.Get("J2 (肩部俯仰)", "J2 (Shoulder)"),
                2 => LanguageManager.Get("J3 (肘部俯仰)", "J3 (Elbow)"),
                3 => LanguageManager.Get("J4 (前臂滾轉)", "J4 (Forearm)"),
                4 => LanguageManager.Get("J5 (手腕俯仰)", "J5 (Wrist)"),
                5 => LanguageManager.Get("J6 (法蘭旋轉)", "J6 (Flange)"),
                _ => $"Role {roleSlot}"
            },
            MachineSceneMode.LinearBench6Axis =>
                LanguageManager.Get($"Ch {roleSlot} (滑軌 #{roleSlot})", $"Ch {roleSlot} (Rail #{roleSlot})"),
            _ => roleSlot switch
            {
                0 => LanguageManager.Get("Z 軸 (升降)", "Z-Axis (Vertical)"),
                1 => LanguageManager.Get("Y 軸 (前後)", "Y-Axis (Front/Back)"),
                2 => LanguageManager.Get("X 軸 (左右)", "X-Axis (Left/Right)"),
                _ => $"Role {roleSlot}"
            }
        };
    }

    private void SyncControllerAxisNamesFromRoles()
    {
        for (int m = 0; m < Tmcm6110Controller.TotalAxes; m++)
        {
            int role = RoleMap.GetRoleForMotor(m);
            if (role >= _selectedAxisCount)
            {
                Controller.GetAxis(m).AxisName = $"A{m} (Unused)";
                continue;
            }

            Controller.GetAxis(m).AxisName = _selectedSceneMode switch
            {
                MachineSceneMode.LinearStage1Axis => role == 0 ? "X" : $"A{m}",
                MachineSceneMode.RotaryTable1Axis => role == 0 ? "R" : $"A{m}",
                MachineSceneMode.XyStage2Axis => role switch { 0 => "X", 1 => "Y", _ => $"A{m}" },
                MachineSceneMode.XzGantry2Axis => role switch { 0 => "Z", 1 => "X", _ => $"A{m}" },
                MachineSceneMode.XyRotary3Axis => role switch { 0 => "R", 1 => "Y", 2 => "X", _ => $"A{m}" },
                MachineSceneMode.DualGantry6Axis => role switch
                {
                    0 => "Z1", 1 => "Y1", 2 => "X1", 3 => "Z2", 4 => "Y2", 5 => "X2", _ => $"A{m}"
                },
                MachineSceneMode.GantryRotary4Axis or
                MachineSceneMode.GantryDualRotary5Axis or
                MachineSceneMode.GantryRotaryAux6Axis => role switch
                {
                    0 => "Z", 1 => "Y", 2 => "X", 3 => "R", 4 => "A", 5 => "B", _ => $"A{m}"
                },
                MachineSceneMode.RobotArm6Dof => $"J{role + 1}",
                MachineSceneMode.LinearBench6Axis => $"Ch{role}",
                _ => role switch
                {
                    0 => "Z", 1 => "Y", 2 => "X", _ => $"A{m} (Unused)"
                }
            };
        }
    }

    private void TrySyncRolesFromLoadedAxisLabels()
    {
        string l0 = Controller.GetAxis(0).AxisName;
        string l1 = Controller.GetAxis(1).AxisName;
        string l2 = Controller.GetAxis(2).AxisName;

        var strokeByRole = CaptureStrokesByRole();
        if (!RoleMap.TryApplyFromXyzLabels(l0, l1, l2))
            return;

        _isSwappingRoles = true;
        try
        {
            RestoreStrokesByRole(strokeByRole);
            for (int m = 0; m < AllAxes.Count; m++)
                AllAxes[m].SetAssignedRoleSlotInternal(RoleMap.GetRoleForMotor(m), _selectedSceneMode);
            RebuildActiveAxes();
            SyncControllerAxisNamesFromRoles();
            RefreshLegendTexts();
            AxisMappingChanged?.Invoke();
        }
        finally
        {
            _isSwappingRoles = false;
        }
    }

    private void ApplyOriginPresets()
    {
        switch (_selectedSceneMode)
        {
            case MachineSceneMode.LinearStage1Axis:
                ApplyXOriginToAxis(GetMotorIndexForRole(0));
                break;
            case MachineSceneMode.XyStage2Axis:
                ApplyXyOriginToAxisPair(xIdx: GetMotorIndexForRole(0), yIdx: GetMotorIndexForRole(1));
                break;
            case MachineSceneMode.XzGantry2Axis:
                ApplyXOriginToAxis(GetMotorIndexForRole(1));
                ApplyZOriginToAxis(GetMotorIndexForRole(0));
                break;
            case MachineSceneMode.XyRotary3Axis:
                ApplyXyOriginToAxisPair(xIdx: GetMotorIndexForRole(2), yIdx: GetMotorIndexForRole(1));
                break;
            case MachineSceneMode.Gantry3Axis:
            case MachineSceneMode.GantryRotary4Axis:
            case MachineSceneMode.GantryDualRotary5Axis:
            case MachineSceneMode.GantryRotaryAux6Axis:
                ApplyXyOriginToAxisPair(xIdx: GetMotorIndexForRole(2), yIdx: GetMotorIndexForRole(1));
                ApplyZOriginToAxis(GetMotorIndexForRole(0));
                break;
            case MachineSceneMode.DualGantry6Axis:
                ApplyXyOriginToAxisPair(xIdx: GetMotorIndexForRole(2), yIdx: GetMotorIndexForRole(1));
                ApplyZOriginToAxis(GetMotorIndexForRole(0));
                ApplyXyOriginToAxisPair(xIdx: GetMotorIndexForRole(5), yIdx: GetMotorIndexForRole(4));
                ApplyZOriginToAxis(GetMotorIndexForRole(3));
                break;
        }
    }

    private void ApplyXyOriginToAxisPair(int xIdx, int yIdx)
    {
        ApplyXOriginToAxis(xIdx);
        ApplyYOriginToAxis(yIdx);
    }

    private void ApplyXOriginToAxis(int xIdx)
    {
        var axX = AllAxes[xIdx];
        double lenX = axX.StrokeLengthMm;
        (double minX, double maxX) = _selectedXyOrigin switch
        {
            XyOriginPreset.FrontLeft or XyOriginPreset.BackLeft => (0.0, lenX),
            XyOriginPreset.FrontRight or XyOriginPreset.BackRight => (-lenX, 0.0),
            _ => (-lenX * 0.5, lenX * 0.5)
        };
        axX.SetStrokeLimitsMm(minX, maxX);
        if (_syncRfsDirectionWithOrigin)
            SetRfsModeForOrigin(xIdx, maxX <= 0.0 ? 65 : 1);
    }

    private void ApplyYOriginToAxis(int yIdx)
    {
        var axY = AllAxes[yIdx];
        double lenY = axY.StrokeLengthMm;
        (double minY, double maxY) = _selectedXyOrigin switch
        {
            XyOriginPreset.FrontLeft or XyOriginPreset.FrontRight => (0.0, lenY),
            XyOriginPreset.BackLeft or XyOriginPreset.BackRight => (-lenY, 0.0),
            _ => (-lenY * 0.5, lenY * 0.5)
        };
        axY.SetStrokeLimitsMm(minY, maxY);
        if (_syncRfsDirectionWithOrigin)
            SetRfsModeForOrigin(yIdx, maxY <= 0.0 ? 65 : 1);
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
        foreach (var opt in AvailableAxisCounts)
        {
            opt.Label = opt.Value == 1
                ? LanguageManager.Get("1 軸 (Single Axis)", "1 Axis (Single)")
                : LanguageManager.Get($"{opt.Value} 軸 ({opt.Value} Axes)", $"{opt.Value} Axes");
        }

        SyncAvailableRoleOptions();
        RefreshSceneModeOptionLabels();

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

    private void RefreshSceneModeOptionLabels()
    {
        foreach (var opt in AvailableSceneModes)
        {
            opt.Label = opt.Value switch
            {
                MachineSceneMode.LinearStage1Axis => LanguageManager.Get("1 軸線性滑台 (X)", "1-Axis Linear Stage (X)"),
                MachineSceneMode.RotaryTable1Axis => LanguageManager.Get("1 軸旋轉分度盤 (R)", "1-Axis Rotary Table (R)"),
                MachineSceneMode.XyStage2Axis => LanguageManager.Get("2 軸 XY 定位平台 (X/Y)", "2-Axis XY Stage (X/Y)"),
                MachineSceneMode.XzGantry2Axis => LanguageManager.Get("2 軸 XZ 垂直龍門 (X/Z)", "2-Axis XZ Gantry (X/Z)"),
                MachineSceneMode.Gantry3Axis => LanguageManager.Get("3 軸龍門（預設 Z/Y/X）", "3-Axis Gantry (Default Z/Y/X)"),
                MachineSceneMode.XyRotary3Axis => LanguageManager.Get("3 軸 XY + 旋轉台 (X/Y/R)", "3-Axis XY + Rotary Table (X/Y/R)"),
                MachineSceneMode.GantryRotary4Axis => LanguageManager.Get("4 軸龍門 + 旋轉主軸 (XYZ + R)", "4-Axis Gantry + Rotary Spindle (XYZ+R)"),
                MachineSceneMode.GantryDualRotary5Axis => LanguageManager.Get("5 軸龍門 + 雙旋轉軸 (XYZ + R/A)", "5-Axis Gantry + Dual Rotary (XYZ+R/A)"),
                MachineSceneMode.DualGantry6Axis => LanguageManager.Get("6 軸雙龍門（雙工位 Z1/Y1/X1 + Z2/Y2/X2）", "6-Axis Dual Gantry (Stations 1 & 2)"),
                MachineSceneMode.GantryRotaryAux6Axis => LanguageManager.Get("6 軸龍門 + 旋轉/輔助軸 (XYZ + R/A/B)", "6-Axis Gantry + Rotary/Aux (XYZ + R/A/B)"),
                MachineSceneMode.RobotArm6Dof => LanguageManager.Get("6 軸關節機械手臂 (J1~J6)", "6-Axis Robot Arm (J1-J6)"),
                MachineSceneMode.LinearBench6Axis => LanguageManager.Get($"{_selectedAxisCount} 軸獨立線性測試台 (Ch 0~{_selectedAxisCount - 1})", $"{_selectedAxisCount}-Rail Linear Test Bench (Ch 0-{_selectedAxisCount - 1})"),
                _ => opt.Value.ToString()
            };
        }
    }

    private void RefreshLegendTexts()
    {
        int m0 = GetMotorIndexForRole(0);
        int m1 = GetMotorIndexForRole(1);
        int m2 = GetMotorIndexForRole(2);
        int m3 = GetMotorIndexForRole(3);
        int m4 = GetMotorIndexForRole(4);
        int m5 = GetMotorIndexForRole(5);

        switch (_selectedSceneMode)
        {
            case MachineSceneMode.LinearStage1Axis:
                LegendTitle = LanguageManager.Get("3D 單軸線性滑台視圖 (1 軸)", "3D 1-Axis Linear Stage View");
                LegendLine1 = LanguageManager.Get($"• 紅色滑座 X 軸 = Motor #{m0} (左右線性移動)", $"• Red Linear Stage X = Motor #{m0}");
                LegendLine2 = LanguageManager.Get("• 兩端紅綠燈號 = -Limit / +Limit 硬體極限", "• End LEDs = -Limit / +Limit Switches");
                LegendLine3 = LanguageManager.Get("• 白色方塊 = 0 mm 機械原點基準", "• White Block = 0 mm Origin Reference");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 單軸掃描往復軌跡", "• Gold Trail = Linear Axis Trajectory");
                break;
            case MachineSceneMode.RotaryTable1Axis:
                LegendTitle = LanguageManager.Get("3D 單軸旋轉分度盤視圖 (1 軸)", "3D 1-Axis Rotary Table View");
                LegendLine1 = LanguageManager.Get($"• 旋轉盤面 R 軸 = Motor #{m0} (±Limit 對應 ±180°)", $"• Rotary Platter R = Motor #{m0} (±180° Span)");
                LegendLine2 = LanguageManager.Get("• 金黃色指針 bar = 目前旋轉角度方向", "• Amber Pointer Bar = Angular Index");
                LegendLine3 = LanguageManager.Get("• 白色基準塊 = 0 mm (0°) 原點指標", "• White Post = 0 mm (0°) Reference");
                LegendLine4 = LanguageManager.Get("• 金色弧線光帶 = 圓周分度運動軌跡", "• Gold Arc = Rotary Sweep Trajectory");
                break;
            case MachineSceneMode.XyStage2Axis:
                LegendTitle = LanguageManager.Get("3D 雙軸 XY 定位平台視圖 (2 軸)", "3D 2-Axis XY Positioning Stage");
                LegendLine1 = LanguageManager.Get($"• 紅色左右橫移 X 軸 = Motor #{m0}", $"• Red X Carriage = Motor #{m0}");
                LegendLine2 = LanguageManager.Get($"• 綠色前後橋架 Y 軸 = Motor #{m1}", $"• Green Y Bridge = Motor #{m1}");
                LegendLine3 = LanguageManager.Get("• 三色原點座標軸 = 0 mm 平面基準 (支援四角預設)", "• White Triad = 0 mm Planar Origin");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = XY 平面二軸插補軌跡", "• Gold Trail = 2-Axis XY Trajectory");
                break;
            case MachineSceneMode.XzGantry2Axis:
                LegendTitle = LanguageManager.Get("3D 雙軸 XZ 垂直龍門視圖 (2 軸)", "3D 2-Axis XZ Vertical Gantry");
                LegendLine1 = LanguageManager.Get($"• 藍色垂直主軸 Z 軸 = Motor #{m0}", $"• Blue Vertical Spindle Z = Motor #{m0}");
                LegendLine2 = LanguageManager.Get($"• 紅色橫樑滑座 X 軸 = Motor #{m1}", $"• Red Crossbeam Carriage X = Motor #{m1}");
                LegendLine3 = LanguageManager.Get("• 各軸兩側 LED = ±Limit 極限開關狀態", "• Rail LEDs = ±Limit Switch Indicators");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = XZ 垂直剖面二軸軌跡", "• Gold Trail = 2-Axis XZ Trajectory");
                break;
            case MachineSceneMode.XyRotary3Axis:
                LegendTitle = LanguageManager.Get("3D XY + 旋轉載台視圖 (3 軸)", "3D XY + Rotary Platter Stage (3-Axis)");
                LegendLine1 = LanguageManager.Get($"• 旋轉載台 R = Motor #{m0} | Y = M#{m1} | X = M#{m2}", $"• Rotary R = M#{m0} | Y = M#{m1} | X = M#{m2}");
                LegendLine2 = LanguageManager.Get("• 上方 XY 橋架帶動固定高度加工頭位移", "• Overhead XY Frame Moves Processing Head");
                LegendLine3 = LanguageManager.Get("• 底部轉盤即時反應 R 軸角度與極限狀態", "• Platter Rotates & Displays R ±Limits");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 機台 XY 平面運動軌跡", "• Gold Trail = XY Planar Motion Path");
                break;
            case MachineSceneMode.GantryRotary4Axis:
                LegendTitle = LanguageManager.Get("3D 龍門 + 旋轉主軸視圖 (4 軸)", "3D Gantry + Rotary Spindle (4-Axis)");
                LegendLine1 = LanguageManager.Get($"• 龍門主軸: Z=M#{m0} 藍 / Y=M#{m1} 綠 / X=M#{m2} 紅", $"• Gantry: Z=M#{m0} / Y=M#{m1} / X=M#{m2}");
                LegendLine2 = LanguageManager.Get($"• R 軸旋轉夾頭 = Motor #{m3} (橘黃指針標示)", $"• R Rotary Chuck = Motor #{m3} (Amber Flag)");
                LegendLine3 = LanguageManager.Get("• 支援 Z/Y/X 四角原點切換與四軸插補演示", "• Supports Corner Origins & 4-Axis Demo");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 3D 工具尖端 (TCP) 軌跡", "• Gold Ribbon = 3D Tool Center Point Trail");
                break;
            case MachineSceneMode.GantryDualRotary5Axis:
                LegendTitle = LanguageManager.Get("3D 五軸龍門雙旋轉視圖 (5 軸)", "3D 5-Axis Gantry + Dual Rotary");
                LegendLine1 = LanguageManager.Get($"• 龍門主軸: Z=M#{m0} 藍 / Y=M#{m1} 綠 / X=M#{m2} 紅", $"• Gantry: Z=M#{m0} / Y=M#{m1} / X=M#{m2}");
                LegendLine2 = LanguageManager.Get($"• R (主軸旋)=M#{m3} | A (旋轉台)=M#{m4}", $"• R (Spindle)=M#{m3} | A (Table)=M#{m4}");
                LegendLine3 = LanguageManager.Get("• 雙獨立旋轉極限 LED 與三色 0 mm 基準", "• Dual Rotary ±Limit LEDs & White Origin");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 3D 工具尖端 (TCP) 軌跡", "• Gold Ribbon = 3D Tool Center Point Trail");
                break;
            case MachineSceneMode.DualGantry6Axis:
                LegendTitle = LanguageManager.Get("3D 雙龍門雙工位視圖 (6 軸)", "3D Dual-Gantry Dual-Station (6-Axis)");
                LegendLine1 = LanguageManager.Get($"• 左站: Z1=M#{m0} / Y1=M#{m1} / X1=M#{m2}", $"• Left: Z1=M#{m0} / Y1=M#{m1} / X1=M#{m2}");
                LegendLine2 = LanguageManager.Get($"• 右站: Z2=M#{m3} / Y2=M#{m4} / X2=M#{m5}", $"• Right: Z2=M#{m3} / Y2=M#{m4} / X2=M#{m5}");
                LegendLine3 = LanguageManager.Get("• 白點三色標 = 各站機械原點 (0, 0)", "• White Marker = Station Origin (0, 0)");
                LegendLine4 = LanguageManager.Get("• 金色 / 青色光帶 = 左站 / 右站 TCP 軌跡", "• Gold / Cyan Ribbon = Station 1 / 2 Trails");
                break;
            case MachineSceneMode.GantryRotaryAux6Axis:
                LegendTitle = LanguageManager.Get("3D 龍門 + 旋轉/輔助視圖 (6 軸)", "3D Gantry + Rotary/Aux Stage (6-Axis)");
                LegendLine1 = LanguageManager.Get($"• 龍門主軸: Z=M#{m0} 藍 / Y=M#{m1} 綠 / X=M#{m2} 紅", $"• Gantry: Z=M#{m0} / Y=M#{m1} / X=M#{m2}");
                LegendLine2 = LanguageManager.Get($"• R (主軸旋)=M#{m3} | A (轉盤)=M#{m4}", $"• R (Spindle)=M#{m3} | A (Table)=M#{m4}");
                LegendLine3 = LanguageManager.Get($"• B (右側輔助供料滑台) = Motor #{m5}", $"• B (Indigo Aux Feeder) = Motor #{m5}");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 3D 工具尖端 (TCP) 軌跡", "• Gold Ribbon = 3D Tool Center Point Trail");
                break;
            case MachineSceneMode.RobotArm6Dof:
                LegendTitle = LanguageManager.Get("3D 六軸關節機械手臂 (J1~J6)", "3D 6-DOF Articulated Robot Arm (J1-J6)");
                LegendLine1 = LanguageManager.Get($"• J1=M#{m0} 底座 | J2=M#{m1} 肩 | J3=M#{m2} 肘", $"• J1=M#{m0} Base | J2=M#{m1} Shoulder | J3=M#{m2} Elbow");
                LegendLine2 = LanguageManager.Get($"• J4=M#{m3} 前臂 | J5=M#{m4} 腕 | J6=M#{m5} 法蘭", $"• J4=M#{m3} Forearm | J5=M#{m4} Wrist | J6=M#{m5} Flange");
                LegendLine3 = LanguageManager.Get("• 關節行程對應各軸 ±Limit 範圍 (-100%~+100%)", "• Joint angles map linearly across axis ±Limit span");
                LegendLine4 = LanguageManager.Get("• 金色光帶 = 手臂法蘭 TCP 空間軌跡", "• Gold Ribbon = Flange TCP 3D Trajectory");
                break;
            case MachineSceneMode.LinearBench6Axis:
                LegendTitle = LanguageManager.Get($"3D {_selectedAxisCount} 軸獨立線性測試台 (Ch 0~{_selectedAxisCount - 1})", $"3D {_selectedAxisCount}-Rail Linear Test Bench (Ch 0-{_selectedAxisCount - 1})");
                LegendLine1 = "• " + string.Join(", ", Enumerable.Range(0, Math.Min(3, _selectedAxisCount)).Select(r => $"Ch{r}=M#{GetMotorIndexForRole(r)}"));
                LegendLine2 = _selectedAxisCount > 3
                    ? "• " + string.Join(", ", Enumerable.Range(3, _selectedAxisCount - 3).Select(r => $"Ch{r}=M#{GetMotorIndexForRole(r)}"))
                    : LanguageManager.Get("• 依上方「使用軸數」動態調整並行滑軌數量", "• Rail count scales dynamically with Active Axes");
                LegendLine3 = LanguageManager.Get("• 中央灰藍橫條 = 0 mm 原點 | 兩端 = ±Limit LED", "• Center Bar = 0 mm | Ends = ±Limit LEDs");
                LegendLine4 = LanguageManager.Get("• 適用全通道通訊與極限批量驗證", "• Ideal for per-channel protocol & limit testing");
                break;
            default:
                LegendTitle = LanguageManager.Get("3D 龍門機構動態視圖 (Z-Y-X 映射)", "3D Gantry Stage Live View (Z-Y-X Mapping)");
                LegendLine1 = LanguageManager.Get($"• 藍色垂直主軸 = Motor #{m0} (Z 軸 - 升降)", $"• Blue Vertical Spindle = Motor #{m0} (Z-Axis)");
                LegendLine2 = LanguageManager.Get($"• 綠色前後龍門 = Motor #{m1} (Y 軸 - 前後)", $"• Green Gantry Bridge = Motor #{m1} (Y-Axis)");
                LegendLine3 = LanguageManager.Get($"• 紅色左右橫移 = Motor #{m2} (X 軸 - 左右)", $"• Red Cross Carriage = Motor #{m2} (X-Axis)");
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
                TrySyncRolesFromLoadedAxisLabels();
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
                TrySyncRolesFromLoadedAxisLabels();
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
            int roleSlot = axVm.AssignedRoleSlot;
            var snap = Controller.GetAxis(idx).GetSnapshot();
            double minS = snap.VirtualMinStrokeSteps;
            double maxS = snap.VirtualMaxStrokeSteps;
            int targetPos = (int)Math.Round(minS + (maxS - minS) * frac[roleSlot]);

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

    private void OnProtocolTraceGenerated(ProtocolTraceEvent trace)
    {
        Interlocked.Increment(ref _totalRxPackets);

        if (HidePollingPacketsInLog && trace.IsPollingQuery)
        {
            Application.Current?.Dispatcher.BeginInvoke(() => OnPropertyChanged(nameof(TotalRxPackets)));
            return;
        }

        var entry = new PacketLogEntry(
            DateTime.Now.ToString("HH:mm:ss.fff"),
            trace.Direction,
            trace.RawRequest,
            trace.DecodedRequest,
            trace.RawReply,
            trace.DecodedReply
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
        SerialServer.ProtocolAdapter.TraceGenerated -= OnProtocolTraceGenerated;
        SerialServer.StatusLogMessage -= OnServerStatusLogMessage;
        SerialServer.Dispose();
    }

    public event PropertyChangedEventHandler? PropertyChanged;
    protected void OnPropertyChanged([CallerMemberName] string? name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
