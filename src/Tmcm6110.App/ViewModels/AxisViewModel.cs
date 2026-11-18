using System.ComponentModel;
using System.Runtime.CompilerServices;
using Tmcm6110.App.Localization;
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

    public AxisViewModel(AxisState axis, Tmcm6110Controller controller)
    {
        _axis = axis;
        _controller = controller;
        UpdateAxisTitle();
    }

    public void UpdateAxisTitle()
    {
        string roleDesc = _axis.AxisName switch
        {
            "Z" => LanguageManager.Get("Z 軸 - 垂直升降", "Z-Axis - Vertical"),
            "Y" => LanguageManager.Get("Y 軸 - 前後移動", "Y-Axis - Front/Back"),
            "X" => LanguageManager.Get("X 軸 - 左右橫移", "X-Axis - Left/Right"),
            _ => _axis.AxisName
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
