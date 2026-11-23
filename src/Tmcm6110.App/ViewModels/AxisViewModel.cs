using System.ComponentModel;
using System.Runtime.CompilerServices;
using Tmcm6110.App.Localization;
using Tmcm6110.App.Rendering;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.App.ViewModels;

/// <summary>
/// UI ViewModel representing one physical TMCM-6110 axis (Axis 0 = Z, Axis 1 = Y, Axis 2 = X).
/// Provides live telemetry (Idle / Move / Homing / +Limit / -Limit / Alarm / Position Reached)
/// and interactive controls for manual jog, homing, limit switch forcing, and alarm injection.
/// </summary>
public class AxisViewModel : INotifyPropertyChanged
{
    private readonly AxisState _axis;
    private readonly Tmcm6110Controller _controller;
    private readonly Func<double> _getStepsPerMm;

    private string _axisTitle = string.Empty;
    private string _statusBadgeText = "IDLE";
    private string _statusBadgeColor = "#2E7D32";
    private int _actualPosition;
    private int _targetPosition;
    private double _positionMm;
    private int _actualSpeed;
    private int _maxSpeed;
    private int _maxAcceleration;
    private int _microstepResolution;
    private int _rfsMode;
    private bool _isPosReached = true;
    private bool _isMoving;
    private bool _isRightLimit;
    private bool _isLeftLimit;
    private bool _isAlarm;
    private int _actualLoad;
    private string _errorFlagsSummary = "None (0x00)";
    private int _jogVelocity = 1000;
    private int _moveTargetInput = 25000;
    private double _minLimitMm;
    private double _maxLimitMm;
    private int _lastAppliedMinSteps;
    private int _lastAppliedMaxSteps;
    private bool _isLimitStopEnabled;
    private MachineSceneMode _currentSceneMode = MachineSceneMode.Gantry3Axis;

    public int AxisIndex => _axis.AxisIndex;
    public AxisState UnderlyingAxis => _axis;

    public string AxisTitle
    {
        get => _axisTitle;
        set { _axisTitle = value; OnPropertyChanged(); }
    }

    public string StatusBadgeText
    {
        get => _statusBadgeText;
        set { _statusBadgeText = value; OnPropertyChanged(); }
    }

    public string StatusBadgeColor
    {
        get => _statusBadgeColor;
        set { _statusBadgeColor = value; OnPropertyChanged(); }
    }

    public int ActualPosition
    {
        get => _actualPosition;
        set { _actualPosition = value; OnPropertyChanged(); }
    }

    public int TargetPosition
    {
        get => _targetPosition;
        set { _targetPosition = value; OnPropertyChanged(); }
    }

    public double PositionMm
    {
        get => _positionMm;
        set { _positionMm = value; OnPropertyChanged(); }
    }

    public int ActualSpeed
    {
        get => _actualSpeed;
        set { _actualSpeed = value; OnPropertyChanged(); }
    }

    public int MaxSpeed
    {
        get => _maxSpeed;
        set { _maxSpeed = value; OnPropertyChanged(); }
    }

    public int MaxAcceleration
    {
        get => _maxAcceleration;
        set { _maxAcceleration = value; OnPropertyChanged(); }
    }

    public int MicrostepResolution
    {
        get => _microstepResolution;
        set
        {
            _microstepResolution = value;
            OnPropertyChanged();
            OnPropertyChanged(nameof(MicrostepDisplay));
        }
    }

    public string MicrostepDisplay => _microstepResolution switch
    {
        0 => "Full (0)",
        1 => "Half (1)",
        2 => "4 uStep (2)",
        3 => "8 uStep (3)",
        4 => "16 uStep (4)",
        5 => "32 uStep (5)",
        6 => "64 uStep (6)",
        7 => "128 uStep (7)",
        8 => "256 uStep (8)",
        _ => $"Code {_microstepResolution}"
    };

    public int RfsMode
    {
        get => _rfsMode;
        set
        {
            _rfsMode = value;
            OnPropertyChanged();
            OnPropertyChanged(nameof(RfsModeDisplay));
        }
    }

    public string RfsModeDisplay => (_rfsMode & 64) != 0
        ? $"Mode {_rfsMode} (+Lim/Right)"
        : $"Mode {_rfsMode} (-Lim/Left)";

    public bool IsPosReached
    {
        get => _isPosReached;
        set { _isPosReached = value; OnPropertyChanged(); }
    }

    public bool IsMoving
    {
        get => _isMoving;
        set { _isMoving = value; OnPropertyChanged(); }
    }

    public bool IsRightLimit
    {
        get => _isRightLimit;
        set { _isRightLimit = value; OnPropertyChanged(); }
    }

    public bool IsLeftLimit
    {
        get => _isLeftLimit;
        set { _isLeftLimit = value; OnPropertyChanged(); }
    }

    public bool IsAlarm
    {
        get => _isAlarm;
        set { _isAlarm = value; OnPropertyChanged(); }
    }

    public int ActualLoad
    {
        get => _actualLoad;
        set { _actualLoad = value; OnPropertyChanged(); }
    }

    public string ErrorFlagsSummary
    {
        get => _errorFlagsSummary;
        set { _errorFlagsSummary = value; OnPropertyChanged(); }
    }

    public int JogVelocity
    {
        get => _jogVelocity;
        set { _jogVelocity = value; OnPropertyChanged(); }
    }

    public int MoveTargetInput
    {
        get => _moveTargetInput;
        set { _moveTargetInput = value; OnPropertyChanged(); }
    }

    public bool ForceRightLimit
    {
        get => _axis.ManualRightLimitOverride;
        set
        {
            _axis.ManualRightLimitOverride = value;
            OnPropertyChanged();
        }
    }

    public bool ForceLeftLimit
    {
        get => _axis.ManualLeftLimitOverride;
        set
        {
            _axis.ManualLeftLimitOverride = value;
            OnPropertyChanged();
        }
    }

    public double MinLimitMm
    {
        get => _minLimitMm;
        set
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) return;
            double clamped = Math.Round(Math.Clamp(value, -5000.0, 5000.0), 2);
            if (clamped >= _maxLimitMm)
            {
                if (clamped >= 5000.0)
                {
                    clamped = 4999.0;
                    _maxLimitMm = 5000.0;
                }
                else
                {
                    _maxLimitMm = Math.Min(5000.0, Math.Round(clamped + 10.0, 2));
                }
                OnPropertyChanged(nameof(MaxLimitMm));
            }
            _minLimitMm = clamped;
            OnPropertyChanged();
            OnPropertyChanged(nameof(ShortStrokeBadge));
            ApplyStrokeLimitsToAxis();
        }
    }

    public double MaxLimitMm
    {
        get => _maxLimitMm;
        set
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) return;
            double clamped = Math.Round(Math.Clamp(value, -5000.0, 5000.0), 2);
            if (clamped <= _minLimitMm)
            {
                if (clamped <= -5000.0)
                {
                    clamped = -4999.0;
                    _minLimitMm = -5000.0;
                }
                else
                {
                    _minLimitMm = Math.Max(-5000.0, Math.Round(clamped - 10.0, 2));
                }
                OnPropertyChanged(nameof(MinLimitMm));
            }
            _maxLimitMm = clamped;
            OnPropertyChanged();
            OnPropertyChanged(nameof(ShortStrokeBadge));
            ApplyStrokeLimitsToAxis();
        }
    }

    public string ShortStrokeBadge => $"{_minLimitMm:0.#} ~ {_maxLimitMm:0.#} mm";
    public double StrokeLengthMm => Math.Max(1.0, Math.Round(_maxLimitMm - _minLimitMm, 2));
    public string StrokeStepsDisplay => $"[{_lastAppliedMinSteps:+#,0;-#,0;0} ~ {_lastAppliedMaxSteps:+#,0;-#,0;0} steps]";

    public bool IsLimitStopEnabled
    {
        get => _isLimitStopEnabled;
        set
        {
            if (_isLimitStopEnabled == value) return;
            _isLimitStopEnabled = value;
            int disableVal = value ? 0 : 1;
            _controller.ProcessCommand(new TmclCommandPacket(
                _controller.ModuleAddress, (byte)TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, (byte)AxisIndex, disableVal));
            _controller.ProcessCommand(new TmclCommandPacket(
                _controller.ModuleAddress, (byte)TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, (byte)AxisIndex, disableVal));
            OnPropertyChanged();
        }
    }

    public AxisViewModel(AxisState axis, Tmcm6110Controller controller, Func<double>? getStepsPerMm = null)
    {
        _axis = axis;
        _controller = controller;
        _getStepsPerMm = getStepsPerMm ?? (() => 1000.0);

        double stepsPerMm = Math.Max(10.0, _getStepsPerMm());
        _lastAppliedMinSteps = _axis.VirtualMinStrokeSteps;
        _lastAppliedMaxSteps = _axis.VirtualMaxStrokeSteps;
        _minLimitMm = Math.Round(_lastAppliedMinSteps / stepsPerMm, 2);
        _maxLimitMm = Math.Round(_lastAppliedMaxSteps / stepsPerMm, 2);
        _isLimitStopEnabled =
            _axis.GetParameter(TmclAxisParam.RightLimitSwitchDisable, clearReadFlags: false) == 0 &&
            _axis.GetParameter(TmclAxisParam.LeftLimitSwitchDisable, clearReadFlags: false) == 0;

        UpdateAxisTitle();
    }

    public void SetStrokeLimitsMm(double minMm, double maxMm)
    {
        if (double.IsNaN(minMm) || double.IsInfinity(minMm) ||
            double.IsNaN(maxMm) || double.IsInfinity(maxMm))
            return;

        double minC = Math.Round(Math.Clamp(minMm, -5000.0, 5000.0), 2);
        double maxC = Math.Round(Math.Clamp(maxMm, -5000.0, 5000.0), 2);
        if (minC > maxC)
            (minC, maxC) = (maxC, minC);
        if (Math.Abs(maxC - minC) < 0.01)
        {
            if (maxC < 5000.0) maxC = Math.Min(5000.0, minC + 10.0);
            else minC = Math.Max(-5000.0, maxC - 10.0);
        }

        _minLimitMm = minC;
        _maxLimitMm = maxC;
        OnPropertyChanged(nameof(MinLimitMm));
        OnPropertyChanged(nameof(MaxLimitMm));
        OnPropertyChanged(nameof(ShortStrokeBadge));
        ApplyStrokeLimitsToAxis();
    }

    public void ApplyStrokeLimitsToAxis()
    {
        double stepsPerMm = Math.Max(10.0, _getStepsPerMm());
        long minRaw = (long)Math.Round(_minLimitMm * stepsPerMm);
        long maxRaw = (long)Math.Round(_maxLimitMm * stepsPerMm);

        int minSteps = (int)Math.Clamp(minRaw, -8_388_608L, 8_388_607L);
        int maxSteps = (int)Math.Clamp(maxRaw, -8_388_608L, 8_388_607L);
        if (minSteps >= maxSteps)
        {
            if (maxSteps < 8_388_607)
                maxSteps = minSteps + 1;
            else
                minSteps = maxSteps - 1;
        }

        _lastAppliedMinSteps = minSteps;
        _lastAppliedMaxSteps = maxSteps;
        _axis.SetVirtualStrokeLimits(minSteps, maxSteps);
        OnPropertyChanged(nameof(StrokeStepsDisplay));
    }

    public void UpdateAxisTitle(MachineSceneMode? sceneMode = null)
    {
        if (sceneMode.HasValue)
            _currentSceneMode = sceneMode.Value;

        string roleDesc = _currentSceneMode switch
        {
            MachineSceneMode.DualGantry6Axis => _axis.AxisIndex switch
            {
                0 => LanguageManager.Get("Z1 軸 - 左站垂直升降", "Z1 - Left Vertical"),
                1 => LanguageManager.Get("Y1 軸 - 左站前後移動", "Y1 - Left Front/Back"),
                2 => LanguageManager.Get("X1 軸 - 左站左右橫移", "X1 - Left Left/Right"),
                3 => LanguageManager.Get("Z2 軸 - 右站垂直升降", "Z2 - Right Vertical"),
                4 => LanguageManager.Get("Y2 軸 - 右站前後移動", "Y2 - Right Front/Back"),
                5 => LanguageManager.Get("X2 軸 - 右站左右橫移", "X2 - Right Left/Right"),
                _ => _axis.AxisName
            },
            MachineSceneMode.GantryRotaryAux6Axis => _axis.AxisIndex switch
            {
                0 => LanguageManager.Get("Z 軸 - 垂直升降", "Z-Axis - Vertical"),
                1 => LanguageManager.Get("Y 軸 - 前後移動", "Y-Axis - Front/Back"),
                2 => LanguageManager.Get("X 軸 - 左右橫移", "X-Axis - Left/Right"),
                3 => LanguageManager.Get("R 軸 - 主軸旋轉", "R-Axis - Spindle Rotation"),
                4 => LanguageManager.Get("A 軸 - 旋轉工作台", "A-Axis - Rotary Table"),
                5 => LanguageManager.Get("B 軸 - 輔助供料滑台", "B-Axis - Aux Feeder Slide"),
                _ => _axis.AxisName
            },
            MachineSceneMode.RobotArm6Dof => _axis.AxisIndex switch
            {
                0 => LanguageManager.Get("J1 - 底座旋轉 (Waist)", "J1 - Base Waist"),
                1 => LanguageManager.Get("J2 - 肩部俯仰 (Shoulder)", "J2 - Shoulder Pitch"),
                2 => LanguageManager.Get("J3 - 肘部俯仰 (Elbow)", "J3 - Elbow Pitch"),
                3 => LanguageManager.Get("J4 - 前臂滾轉 (Forearm)", "J4 - Forearm Roll"),
                4 => LanguageManager.Get("J5 - 手腕俯仰 (Wrist)", "J5 - Wrist Pitch"),
                5 => LanguageManager.Get("J6 - 法蘭旋轉 (Flange)", "J6 - Tool Flange"),
                _ => _axis.AxisName
            },
            MachineSceneMode.LinearBench6Axis => _axis.AxisIndex switch
            {
                0 => LanguageManager.Get("Ch 0 - 線性測試軸 #0", "Ch 0 - Linear Rail #0"),
                1 => LanguageManager.Get("Ch 1 - 線性測試軸 #1", "Ch 1 - Linear Rail #1"),
                2 => LanguageManager.Get("Ch 2 - 線性測試軸 #2", "Ch 2 - Linear Rail #2"),
                3 => LanguageManager.Get("Ch 3 - 線性測試軸 #3", "Ch 3 - Linear Rail #3"),
                4 => LanguageManager.Get("Ch 4 - 線性測試軸 #4", "Ch 4 - Linear Rail #4"),
                5 => LanguageManager.Get("Ch 5 - 線性測試軸 #5", "Ch 5 - Linear Rail #5"),
                _ => _axis.AxisName
            },
            _ => _axis.AxisName switch
            {
                "Z" => LanguageManager.Get("Z 軸 - 垂直升降", "Z-Axis - Vertical"),
                "Y" => LanguageManager.Get("Y 軸 - 前後移動", "Y-Axis - Front/Back"),
                "X" => LanguageManager.Get("X 軸 - 左右橫移", "X-Axis - Left/Right"),
                _ => _axis.AxisName
            }
        };
        AxisTitle = $"Motor #{_axis.AxisIndex}  [{roleDesc}]";
    }

    public void RefreshFromSnapshot(AxisSnapshot snap, double stepsPerMm)
    {
        ActualPosition = snap.ActualPosition;
        TargetPosition = snap.TargetPosition;
        PositionMm = Math.Round(snap.ExactPosition / Math.Max(1.0, stepsPerMm), 2);
        ActualSpeed = snap.ActualSpeed;
        MaxSpeed = snap.MaxSpeed;
        MaxAcceleration = snap.MaxAcceleration;
        MicrostepResolution = snap.MicrostepResolution;
        RfsMode = snap.RfsMode;
        IsPosReached = snap.IsPositionReached;
        IsRightLimit = snap.IsRightLimitTriggered;
        IsLeftLimit = snap.IsLeftLimitTriggered;
        IsAlarm = snap.IsAlarmActive;
        ActualLoad = snap.ActualLoadValue;

        IsMoving = snap.MotionState is AxisMotionState.MovingPosition
            or AxisMotionState.MovingVelocity
            or AxisMotionState.HomingRfs;

        switch (snap.MotionState)
        {
            case AxisMotionState.AlarmStopped:
                StatusBadgeText = LanguageManager.Get("ALARM (警報停機)", "ALARM (Fault Stop)");
                StatusBadgeColor = "#D32F2F";
                break;
            case AxisMotionState.HomingRfs:
                StatusBadgeText = $"HOMING (RFS Phase {snap.RfsPhase})";
                StatusBadgeColor = "#F57C00";
                break;
            case AxisMotionState.MovingPosition:
                StatusBadgeText = LanguageManager.Get("MOVE (定位移動中)", "MOVE (Positioning)");
                StatusBadgeColor = "#1976D2";
                break;
            case AxisMotionState.MovingVelocity:
                StatusBadgeText = LanguageManager.Get("MOVE (連續旋轉中)", "MOVE (Velocity)");
                StatusBadgeColor = "#0288D1";
                break;
            default:
                StatusBadgeText = LanguageManager.Get("IDLE (待機到位)", "IDLE (In Position)");
                StatusBadgeColor = "#388E3C";
                break;
        }

        int drvNoStandstill = snap.DriverErrorFlags & ~(int)Tmc262DriverErrorFlags.StandstillIndicator;
        if (snap.ExtendedErrorFlags == 0 && drvNoStandstill == 0)
        {
            ErrorFlagsSummary = "Normal (No Error)";
        }
        else
        {
            ErrorFlagsSummary = $"Stall(#207)={snap.ExtendedErrorFlags}, TMC262(#208)=0x{snap.DriverErrorFlags:X2}";
        }

        double safeStepsPerMm = Math.Max(10.0, stepsPerMm);
        if (snap.VirtualMinStrokeSteps != _lastAppliedMinSteps || snap.VirtualMaxStrokeSteps != _lastAppliedMaxSteps)
        {
            _lastAppliedMinSteps = snap.VirtualMinStrokeSteps;
            _lastAppliedMaxSteps = snap.VirtualMaxStrokeSteps;
            _minLimitMm = Math.Round(snap.VirtualMinStrokeSteps / safeStepsPerMm, 2);
            _maxLimitMm = Math.Round(snap.VirtualMaxStrokeSteps / safeStepsPerMm, 2);
            OnPropertyChanged(nameof(MinLimitMm));
            OnPropertyChanged(nameof(MaxLimitMm));
            OnPropertyChanged(nameof(ShortStrokeBadge));
            OnPropertyChanged(nameof(StrokeStepsDisplay));
        }

        bool limitStopEnabled = !snap.IsRightLimitDisabled && !snap.IsLeftLimitDisabled;
        if (_isLimitStopEnabled != limitStopEnabled)
        {
            _isLimitStopEnabled = limitStopEnabled;
            OnPropertyChanged(nameof(IsLimitStopEnabled));
        }
    }

    // Local UI Test Actions (sends actual 9-byte binary TMCL command through the controller)
    public void CommandMoveAbsolute() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.MVP, 0, (byte)AxisIndex, MoveTargetInput));

    public void CommandMoveRelative(int sign) =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.MVP, 1, (byte)AxisIndex, sign * Math.Abs(MoveTargetInput)));

    public void CommandJogRight() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.ROR, 0, (byte)AxisIndex, JogVelocity));

    public void CommandJogLeft() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.ROL, 0, (byte)AxisIndex, JogVelocity));

    public void CommandStop() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.MST, 0, (byte)AxisIndex, 0));

    public void CommandStartHoming() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.RFS, 0, (byte)AxisIndex, 0));

    public void CommandZeroPosition() =>
        _controller.ProcessCommand(new TmclCommandPacket(_controller.ModuleAddress, (byte)TmclCommand.SAP, TmclAxisParam.ActualPosition, (byte)AxisIndex, 0));

    public void ToggleAlarmInjection()
    {
        if (_axis.HasActiveAlarm)
            _axis.ClearAlarm();
        else
            _axis.TriggerAlarm();
    }

    public event PropertyChangedEventHandler? PropertyChanged;
    protected void OnPropertyChanged([CallerMemberName] string? name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
