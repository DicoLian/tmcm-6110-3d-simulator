using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows;
using Tmcm6110.App.Localization;
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

    public ObservableCollection<AxisViewModel> ActiveAxes { get; } = new();
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
            if (value >= 10)
            {
                _stepsPerMm = value;
                OnPropertyChanged();
            }
        }
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

        // Create ViewModels for the 3 active axes (Axis 0 = Z, Axis 1 = Y, Axis 2 = X)
        ActiveAxes.Add(new AxisViewModel(Controller.GetAxis(0), Controller));
        ActiveAxes.Add(new AxisViewModel(Controller.GetAxis(1), Controller));
        ActiveAxes.Add(new AxisViewModel(Controller.GetAxis(2), Controller));

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

    private void RefreshLocalizedTexts()
    {
        OnPropertyChanged(nameof(ConnectButtonText));
        RefreshConnectionStatusText();
        RefreshLoadedConfigSummary();
        foreach (var vm in ActiveAxes)
            vm.UpdateAxisTitle();
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
                foreach (var vm in ActiveAxes) vm.UpdateAxisTitle();
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
                foreach (var vm in ActiveAxes) vm.UpdateAxisTitle();
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
    /// Demo macro: Executes a 3D Coordinated Interpolated Move (MVP COORD, $47) across Z, Y, X axes
    /// and arms Command 138 Position Reached Notification.
    /// </summary>
    public void ExecuteDemo3DInterpolatedMove(int targetZ, int targetY, int targetX)
    {
        // 1. Store Coordinate #1 for Axis 0 (Z), Axis 1 (Y), Axis 2 (X)
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.SCO, 1, 0, targetZ));
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.SCO, 1, 1, targetY));
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.SCO, 1, 2, targetX));

        // 2. Execute MVP COORD, $47 (Bit 6 = Interpolated + Bits 0,1,2 = Axes 0,1,2), Coordinate #1
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.MVP, 2, 0x47, 1));

        // 3. Arm Command 138 (Request Target Position Reached Event for Axes 0,1,2 -> Mask = 0x07)
        Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x07));
    }

    public void StopAllAxes()
    {
        for (byte i = 0; i < 3; i++)
            Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.MST, 0, i, 0));
    }

    public void HomeAllAxes()
    {
        for (byte i = 0; i < 3; i++)
            Controller.ProcessCommand(new TmclCommandPacket(ModuleAddress, (byte)TmclCommand.RFS, 0, i, 0));
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
