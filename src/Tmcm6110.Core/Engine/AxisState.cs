using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Simulates a single stepper motor axis on the TMCM-6110 (TMC429 motion controller + TMC262 driver)
/// following TMCM-6110 TMCL Firmware Manual V1.47 (Rev V1.23).
/// Handles 32-bit trapezoidal ramping, ROR/ROL/MST/MVP, Reference Search (RFS Modes 1..8 &amp; +64/+128),
/// Limit/Home switches (+Limit / -Limit / Home), and StallGuard2 (#206/#207) / TMC262 (#208) flags.
/// </summary>
public class AxisState : IMotionAxis
{
    private const Tmc262DriverErrorFlags FatalDriverShutdownMask =
        Tmc262DriverErrorFlags.OvertemperatureShutdown |
        Tmc262DriverErrorFlags.ShortToGroundA |
        Tmc262DriverErrorFlags.ShortToGroundB;

    private readonly object _syncLock = new();
    private readonly Dictionary<byte, int> _params = new();
    private readonly Dictionary<byte, int> _eepromParams = new();
    private readonly int[] _coordinates = new int[21]; // Coordinate registers 0..20 (FW 1.47)
    private readonly int[] _eepromCoordinates = new int[21]; // EEPROM mirror for coordinates 1..20

    // High-precision floating-point accumulators:
    // _physicalPosition tracks the physical carriage position against virtual stroke limits & 3D rendering.
    // _exactPosition tracks the TMC429 logical position counter (ActualPosition / SAP 1 / GAP 1).
    private double _physicalPosition;
    private double _exactPosition;
    private double _exactSpeed;
    private double _interpolationSpeedOverride; // Non-zero when executing interpolated MVP COORD
    private double _interpolationAccelOverride; // Non-zero when executing interpolated MVP COORD

    // Reference Search (RFS) internal state machine
    // 0 = Inactive, 1 = Searching first/primary/home switch, 2 = Searching primary limit (Modes 2/3),
    // 3 = Backing off switch at switch speed (and calibrating center)
    private int _rfsState;
    private int _rfsSearchDir;
    private int _rfsFirstSwitchPos;
    private int _rfsSwitchEnterPos;

    // Physical stroke limits (in microsteps) for automatic virtual limit switch triggering
    private int _virtualMinStrokeSteps = -100_000;
    private int _virtualMaxStrokeSteps = 100_000;
    private bool _autoTriggerStrokeLimits = true;

    public int VirtualMinStrokeSteps
    {
        get { lock (_syncLock) return _virtualMinStrokeSteps; }
        set
        {
            lock (_syncLock)
            {
                _virtualMinStrokeSteps = value;
                UpdateSwitchAndStatusRegistersUnsafe();
            }
        }
    }

    public int VirtualMaxStrokeSteps
    {
        get { lock (_syncLock) return _virtualMaxStrokeSteps; }
        set
        {
            lock (_syncLock)
            {
                _virtualMaxStrokeSteps = value;
                UpdateSwitchAndStatusRegistersUnsafe();
            }
        }
    }

    public bool AutoTriggerStrokeLimits
    {
        get { lock (_syncLock) return _autoTriggerStrokeLimits; }
        set
        {
            lock (_syncLock)
            {
                _autoTriggerStrokeLimits = value;
                UpdateSwitchAndStatusRegistersUnsafe();
            }
        }
    }

    /// <summary>
    /// Atomically updates both virtual stroke limits (normalizing min &lt; max) and refreshes switch registers.
    /// </summary>
    public void SetVirtualStrokeLimits(int minSteps, int maxSteps)
    {
        lock (_syncLock)
        {
            if (minSteps > maxSteps)
                (minSteps, maxSteps) = (maxSteps, minSteps);
            if (minSteps == maxSteps)
                maxSteps = minSteps < int.MaxValue ? minSteps + 1 : minSteps - 1;

            _virtualMinStrokeSteps = minSteps;
            _virtualMaxStrokeSteps = maxSteps;
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    // Manual override toggles from UI for testing +Limit, -Limit, Ref Switch, and Alarm
    public bool ManualRightLimitOverride { get; set; }
    public bool ManualLeftLimitOverride { get; set; }
    public bool ManualRefSwitchOverride { get; set; }

    public int AxisIndex { get; }
    public string AxisName { get; set; } // e.g., "Z" for Axis 0, "Y" for Axis 1, "X" for Axis 2

    public AxisState(int axisIndex, string axisName)
    {
        AxisIndex = axisIndex;
        AxisName = axisName;
        ResetToDefaults(axisIndex);
    }

    /// <summary>
    /// Initializes axis parameters matching default 3-axis XYZ gantry settings.
    /// </summary>
    public void ResetToDefaults(int axisIndex)
    {
        lock (_syncLock)
        {
            _params.Clear();
            bool isActiveXyz = axisIndex <= 2;

            _params[TmclAxisParam.TargetPosition] = 0;
            _params[TmclAxisParam.ActualPosition] = 0;
            _params[TmclAxisParam.TargetSpeed] = 0;
            _params[TmclAxisParam.ActualSpeed] = 0;
            _params[TmclAxisParam.MaxPositioningSpeed] = isActiveXyz ? 2000 : 1000;
            _params[TmclAxisParam.MaxAcceleration] = 1000;
            _params[TmclAxisParam.AbsoluteMaxCurrent] = 128;
            _params[TmclAxisParam.StandbyCurrent] = 8;
            _params[TmclAxisParam.TargetPosReached] = 1;
            _params[TmclAxisParam.HomeSwitchState] = 0;
            _params[TmclAxisParam.RightLimitSwitchStatus] = 0;
            _params[TmclAxisParam.LeftLimitSwitchStatus] = 0;
            _params[TmclAxisParam.RightLimitSwitchDisable] = 1; // Default gantry setting: 1
            _params[TmclAxisParam.LeftLimitSwitchDisable] = 1;  // Default gantry setting: 1
            _params[TmclAxisParam.MinimumSpeed] = 1;
            _params[TmclAxisParam.ActualAcceleration] = 0;
            _params[TmclAxisParam.RampMode] = 0;                // 0 = Position mode
            _params[TmclAxisParam.MicrostepResolution] = isActiveXyz ? 6 : 8; // 6 = 64 microsteps, 8 = 256 microsteps
            _params[TmclAxisParam.SoftStopFlag] = 0;
            _params[TmclAxisParam.EndSwitchPowerDownMode] = 0;
            _params[TmclAxisParam.RampDivisor] = 7;
            _params[TmclAxisParam.PulseDivisor] = 3;
            _params[TmclAxisParam.StepInterpolation] = 0;
            _params[TmclAxisParam.DoubleStep] = 0;
            _params[TmclAxisParam.ChopperBlankTime] = 2;
            _params[TmclAxisParam.ConstantTOffMode] = 0;
            _params[TmclAxisParam.DisableFastDecayComparator] = 0;
            _params[TmclAxisParam.ChopperHysteresisEnd] = 2;
            _params[TmclAxisParam.ChopperHysteresisStart] = 3;
            _params[TmclAxisParam.ChopperOffTime] = 5;
            _params[TmclAxisParam.SmartEnergyCurrentMin] = 0;
            _params[TmclAxisParam.SmartEnergyCurrentDownStep] = 0;
            _params[TmclAxisParam.SmartEnergyHysteresis] = 0;
            _params[TmclAxisParam.SmartEnergyCurrentUpStep] = 0;
            _params[TmclAxisParam.SmartEnergyHysteresisStart] = 0;
            _params[TmclAxisParam.StallGuard2FilterEnable] = 1;
            _params[TmclAxisParam.StallGuard2Threshold] = 5;
            _params[TmclAxisParam.SlopeControlHighSide] = 3;
            _params[TmclAxisParam.SlopeControlLowSide] = 3;
            _params[TmclAxisParam.ShortToGroundProtection] = 0;
            _params[TmclAxisParam.ShortDetectionTimer] = 0;
            _params[TmclAxisParam.VSense] = 1;
            _params[TmclAxisParam.SmartEnergyActualCurrent] = Math.Clamp(_params[TmclAxisParam.StandbyCurrent] / 8, 0, 31);
            _params[TmclAxisParam.SmartEnergyStallVelocity] = 0;
            _params[TmclAxisParam.SmartEnergyThresholdSpeed] = 0;
            _params[TmclAxisParam.SmartEnergySlowRunCurrent] = 0;
            _params[TmclAxisParam.RandomChopperOffTime] = 0;
            _params[TmclAxisParam.ReferenceSearchMode] = isActiveXyz ? 65 : 1; // 65 = Search Right Limit Switch
            _params[TmclAxisParam.ReferenceSearchSpeed] = isActiveXyz ? 1000 : 500;
            _params[TmclAxisParam.ReferenceSwitchSpeed] = isActiveXyz ? 500 : 100;
            _params[TmclAxisParam.EndSwitchDistance] = 0;
            _params[TmclAxisParam.LastReferencePosition] = 0;
            _params[TmclAxisParam.BoostCurrent] = 0;
            _params[TmclAxisParam.FreewheelingDelay] = 0;
            _params[TmclAxisParam.ActualLoadValue] = 250;       // Normal nominal load at standstill (0..1023, lower = higher load)
            _params[TmclAxisParam.ExtendedErrorFlags] = 0;      // 0 = No StallGuard error
            _params[TmclAxisParam.DriverErrorFlags] = (int)Tmc262DriverErrorFlags.StandstillIndicator;
            _params[TmclAxisParam.GroupIndex] = 0;
            _params[TmclAxisParam.PowerDownDelay] = 200;

            _eepromParams.Clear();
            foreach (var kv in _params)
                _eepromParams[kv.Key] = kv.Value;

            Array.Clear(_coordinates);
            Array.Clear(_eepromCoordinates);

            _physicalPosition = 0;
            _exactPosition = 0;
            _exactSpeed = 0;
            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _rfsState = 0;
            _rfsSearchDir = 0;
            _rfsFirstSwitchPos = 0;
            _rfsSwitchEnterPos = 0;
            _manualAlarmLatched = false;

            int defaultHalfStroke = axisIndex == 0 ? 45_000 : 100_000;
            _virtualMinStrokeSteps = -defaultHalfStroke;
            _virtualMaxStrokeSteps = defaultHalfStroke;
            _autoTriggerStrokeLimits = true;
        }
    }

    public int GetParameter(byte paramNumber, bool clearReadFlags = true)
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();
            int val = _params.TryGetValue(paramNumber, out int v) ? v : 0;

            // According to TMCM-6110 FW 1.47 Table 16, reading #207 automatically clears the StallGuard error flag
            if (clearReadFlags && paramNumber == TmclAxisParam.ExtendedErrorFlags)
            {
                ClearStallGuardErrorUnsafe();
            }
            return val;
        }
    }

    public void SetParameter(byte paramNumber, int value)
    {
        lock (_syncLock)
        {
            _params[paramNumber] = value;

            switch (paramNumber)
            {
                case TmclAxisParam.TargetPosition: // SAP 0: acts like MVP ABS
                    ClearStallGuardErrorUnsafe();
                    if (!HasFatalDriverFaultUnsafe())
                    {
                        _params[TmclAxisParam.RampMode] = 0;
                        _interpolationSpeedOverride = 0;
                        _interpolationAccelOverride = 0;
                        _rfsState = 0;
                    }
                    UpdateSwitchAndStatusRegistersUnsafe();
                    break;

                case TmclAxisParam.ActualPosition: // SAP 1: overwrites position counter (e.g. homing zero)
                    _exactPosition = value;
                    if (Math.Abs(_exactSpeed) < 0.01 && _rfsState == 0)
                    {
                        // If motor was idle, sync TargetPosition so setting ActualPosition=0 doesn't cause unwanted runaway
                        _params[TmclAxisParam.TargetPosition] = value;
                    }
                    else if (_params[TmclAxisParam.TargetPosition] == value && _rfsState == 0)
                    {
                        // TargetPosition already matches the new ActualPosition (e.g. SAP 0 = 0 then SAP 1 = 0):
                        // clear any velocity accumulated between the two commands so the axis settles without jumping.
                        _exactSpeed = 0;
                        _params[TmclAxisParam.ActualSpeed] = 0;
                        _params[TmclAxisParam.TargetSpeed] = 0;
                        _params[TmclAxisParam.ActualAcceleration] = 0;
                    }
                    UpdateSwitchAndStatusRegistersUnsafe();
                    break;

                case TmclAxisParam.TargetSpeed: // SAP 2: in velocity mode sets speed
                    if (_params[TmclAxisParam.RampMode] == 2)
                    {
                        _rfsState = 0;
                    }
                    break;
            }
        }
    }

    public void StoreParameterToEeprom(byte paramNumber)
    {
        lock (_syncLock)
        {
            if (_params.TryGetValue(paramNumber, out int val))
                _eepromParams[paramNumber] = val;
        }
    }

    public void RestoreParameterFromEeprom(byte paramNumber)
    {
        lock (_syncLock)
        {
            if (_eepromParams.TryGetValue(paramNumber, out int val))
                SetParameter(paramNumber, val);
        }
    }

    /// <summary>
    /// Simulates power-cycle/software reset restoration from EEPROM (Command 255).
    /// </summary>
    public void RestoreAllFromEeprom(bool restoreCoordinates)
    {
        lock (_syncLock)
        {
            foreach (var kv in _eepromParams)
                _params[kv.Key] = kv.Value;

            _params[TmclAxisParam.TargetPosition] = 0;
            _params[TmclAxisParam.ActualPosition] = 0;
            _params[TmclAxisParam.TargetSpeed] = 0;
            _params[TmclAxisParam.ActualSpeed] = 0;
            _params[TmclAxisParam.TargetPosReached] = 1;
            _params[TmclAxisParam.ActualAcceleration] = 0;
            _params[TmclAxisParam.RampMode] = 0;
            _params[TmclAxisParam.EndSwitchDistance] = 0;
            _params[TmclAxisParam.LastReferencePosition] = 0;
            _params[TmclAxisParam.ActualLoadValue] = 250;
            _params[TmclAxisParam.ExtendedErrorFlags] = 0;
            _params[TmclAxisParam.DriverErrorFlags] = (int)Tmc262DriverErrorFlags.StandstillIndicator;

            Array.Clear(_coordinates);
            if (restoreCoordinates)
            {
                for (int i = 1; i < _coordinates.Length; i++)
                    _coordinates[i] = _eepromCoordinates[i];
            }

            _physicalPosition = 0;
            _exactPosition = 0;
            _exactSpeed = 0;
            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _rfsState = 0;
            _rfsSearchDir = 0;
            _rfsFirstSwitchPos = 0;
            _rfsSwitchEnterPos = 0;
            _manualAlarmLatched = false;
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    // Coordinate memory (SCO / GCO / CCO / ACO) for indices 0..20
    public void SetCoordinate(int index, int value, bool storeInEeprom = false)
    {
        lock (_syncLock)
        {
            if (index >= 0 && index < _coordinates.Length)
            {
                _coordinates[index] = value;
                // Coordinate #0 is always stored in RAM only per FW 1.47 Section 3.7.24
                if (storeInEeprom && index > 0)
                    _eepromCoordinates[index] = value;
            }
        }
    }

    public int GetCoordinate(int index)
    {
        lock (_syncLock)
        {
            return (index >= 0 && index < _coordinates.Length) ? _coordinates[index] : 0;
        }
    }

    public int CaptureCoordinate(int index, bool storeInEeprom = false)
    {
        lock (_syncLock)
        {
            if (index >= 0 && index < _coordinates.Length)
            {
                int pos = _params[TmclAxisParam.ActualPosition];
                _coordinates[index] = pos;
                if (storeInEeprom && index > 0)
                    _eepromCoordinates[index] = pos;
                return pos;
            }
            return 0;
        }
    }

    public void CopyCoordinateRamToEeprom(int index)
    {
        lock (_syncLock)
        {
            if (index == 0)
            {
                for (int i = 1; i < _coordinates.Length; i++)
                    _eepromCoordinates[i] = _coordinates[i];
            }
            else if (index >= 1 && index < _coordinates.Length)
            {
                _eepromCoordinates[index] = _coordinates[index];
            }
        }
    }

    public void CopyCoordinateEepromToRam(int index)
    {
        lock (_syncLock)
        {
            if (index == 0)
            {
                for (int i = 1; i < _coordinates.Length; i++)
                    _coordinates[i] = _eepromCoordinates[i];
            }
            else if (index >= 1 && index < _coordinates.Length)
            {
                _coordinates[index] = _eepromCoordinates[index];
            }
        }
    }

    // Motion Commands
    public void RotateRight(int velocity)
    {
        lock (_syncLock)
        {
            ClearStallGuardErrorUnsafe();
            if (HasFatalDriverFaultUnsafe()) return;
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _params[TmclAxisParam.RampMode] = 2; // Velocity mode
            _params[TmclAxisParam.TargetSpeed] = Math.Clamp(velocity, -2047, 2047);
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    public void RotateLeft(int velocity)
    {
        lock (_syncLock)
        {
            ClearStallGuardErrorUnsafe();
            if (HasFatalDriverFaultUnsafe()) return;
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _params[TmclAxisParam.RampMode] = 2; // Velocity mode
            _params[TmclAxisParam.TargetSpeed] = Math.Clamp(-velocity, -2047, 2047);
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    /// <summary>
    /// MST (Motor Stop) per FW 1.47 Section 3.7.3: selects velocity mode and sets target speed to 0 (soft stop).
    /// </summary>
    public void StopMotor()
    {
        lock (_syncLock)
        {
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _params[TmclAxisParam.RampMode] = 2; // Velocity mode
            _params[TmclAxisParam.TargetSpeed] = 0;
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    private void HardStopUnsafe()
    {
        _rfsState = 0;
        _interpolationSpeedOverride = 0;
        _interpolationAccelOverride = 0;
        _params[TmclAxisParam.TargetSpeed] = 0;
        _exactSpeed = 0;
        _params[TmclAxisParam.ActualSpeed] = 0;
        _params[TmclAxisParam.ActualAcceleration] = 0;
        _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
        UpdateSwitchAndStatusRegistersUnsafe();
    }

    public void MoveToAbsolute(int targetPos, double? interpolatedMaxSpeed = null) =>
        MoveToAbsolute(targetPos, interpolatedMaxSpeed, null);

    public void MoveToAbsolute(int targetPos, double? interpolatedMaxSpeed, double? interpolatedMaxAccel)
    {
        lock (_syncLock)
        {
            ClearStallGuardErrorUnsafe();
            if (HasFatalDriverFaultUnsafe()) return;
            _rfsState = 0;
            _params[TmclAxisParam.RampMode] = 0; // Position mode
            _params[TmclAxisParam.TargetPosition] = targetPos; // Full 32-bit signed position (FW 1.47)
            _interpolationSpeedOverride = interpolatedMaxSpeed ?? 0;
            _interpolationAccelOverride = interpolatedMaxAccel ?? 0;
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    public void MoveRelative(int offset)
    {
        lock (_syncLock)
        {
            ClearStallGuardErrorUnsafe();
            if (HasFatalDriverFaultUnsafe()) return;
            long target = (long)_params[TmclAxisParam.ActualPosition] + offset;
            int clamped = (int)Math.Clamp(target, int.MinValue, int.MaxValue);
            MoveToAbsolute(clamped, null, null);
        }
    }

    // Reference Search (RFS)
    public void StartReferenceSearch()
    {
        lock (_syncLock)
        {
            ClearStallGuardErrorUnsafe();
            if (HasFatalDriverFaultUnsafe()) return;

            int rfsMode = _params[TmclAxisParam.ReferenceSearchMode];
            int baseMode = rfsMode & 0x0F;
            bool rightInsteadOfLeft = (rfsMode & 64) != 0;

            _rfsState = 1;
            _rfsFirstSwitchPos = _params[TmclAxisParam.ActualPosition];
            _rfsSwitchEnterPos = _params[TmclAxisParam.ActualPosition];
            _rfsSearchDir = baseMode switch
            {
                2 or 3 => rightInsteadOfLeft ? -1 : 1, // First search opposite limit
                5 or 7 => -1,
                6 or 8 => 1,
                _ => rightInsteadOfLeft ? 1 : -1
            };

            _interpolationSpeedOverride = 0;
            _interpolationAccelOverride = 0;
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    public void StopReferenceSearch()
    {
        lock (_syncLock)
        {
            HardStopUnsafe();
        }
    }

    public int GetReferenceSearchStatus()
    {
        lock (_syncLock)
        {
            return _rfsState; // 0 = inactive, >0 = active phase
        }
    }

    // Alarm / Fault Injection & Clearing
    private bool _manualAlarmLatched;

    private bool HasFatalDriverFaultUnsafe() =>
        (_params[TmclAxisParam.DriverErrorFlags] & (int)FatalDriverShutdownMask) != 0;

    private void ClearStallGuardErrorUnsafe()
    {
        _params[TmclAxisParam.ExtendedErrorFlags] = 0;
        _params[TmclAxisParam.DriverErrorFlags] &= ~(int)Tmc262DriverErrorFlags.StallGuard2Status;
        if (_params[TmclAxisParam.ActualLoadValue] == 0)
            _params[TmclAxisParam.ActualLoadValue] = Math.Abs(_exactSpeed) < 0.05 ? 250 : 110;

        int remainingFaultBits = _params[TmclAxisParam.DriverErrorFlags] & ~(int)Tmc262DriverErrorFlags.StandstillIndicator;
        if (remainingFaultBits == 0)
            _manualAlarmLatched = false;
    }

    public bool HasActiveAlarm
    {
        get
        {
            lock (_syncLock)
            {
                int extErr = _params[TmclAxisParam.ExtendedErrorFlags];
                int drvErr = _params[TmclAxisParam.DriverErrorFlags] & ~(int)Tmc262DriverErrorFlags.StandstillIndicator;
                return _manualAlarmLatched || extErr != 0 || drvErr != 0;
            }
        }
    }

    public void TriggerAlarm(
        Tmc262DriverErrorFlags driverError = Tmc262DriverErrorFlags.StallGuard2Status | Tmc262DriverErrorFlags.OvertemperatureShutdown,
        bool stallError = true)
    {
        lock (_syncLock)
        {
            _manualAlarmLatched = true;
            if (stallError)
                _params[TmclAxisParam.ExtendedErrorFlags] = 1; // StallGuard error
            _params[TmclAxisParam.DriverErrorFlags] = (int)(driverError | Tmc262DriverErrorFlags.StandstillIndicator);
            _params[TmclAxisParam.ActualLoadValue] = 0; // 0 = Max stall load in StallGuard2
            HardStopUnsafe();
        }
    }

    public void ClearAlarm()
    {
        lock (_syncLock)
        {
            _manualAlarmLatched = false;
            _params[TmclAxisParam.ExtendedErrorFlags] = 0;
            _params[TmclAxisParam.DriverErrorFlags] = Math.Abs(_exactSpeed) < 0.1
                ? (int)Tmc262DriverErrorFlags.StandstillIndicator
                : 0;
            _params[TmclAxisParam.ActualLoadValue] = Math.Abs(_exactSpeed) < 0.1 ? 250 : 110;
        }
    }

    /// <summary>
    /// Advances the physics simulation by dtSeconds (called at ~100-500 Hz).
    /// Speed scale factor converts TMCL speed parameter (-2047..2047) to steps/sec.
    /// </summary>
    public void StepPhysics(double dtSeconds, double speedMultiplier = 10.0)
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();

            if (HasFatalDriverFaultUnsafe())
            {
                _exactSpeed = 0;
                _params[TmclAxisParam.ActualSpeed] = 0;
                _params[TmclAxisParam.ActualAcceleration] = 0;
                return;
            }

            // 1. Handle Reference Search (RFS) State Machine
            if (_rfsState != 0)
            {
                StepReferenceSearchUnsafe(dtSeconds, speedMultiplier);
                UpdateSwitchAndStatusRegistersUnsafe();
                return;
            }

            double prevSpeed = _exactSpeed;
            int rampMode = _params[TmclAxisParam.RampMode];
            double maxAccelParam = _interpolationAccelOverride > 0
                ? _interpolationAccelOverride
                : Math.Max(1, _params[TmclAxisParam.MaxAcceleration]);
            double accelStepsPerSec2 = maxAccelParam * speedMultiplier * 2.5;
            double maxDeltaV = accelStepsPerSec2 * dtSeconds;

            bool rightLimHit = _params[TmclAxisParam.RightLimitSwitchStatus] != 0;
            bool leftLimHit = _params[TmclAxisParam.LeftLimitSwitchStatus] != 0;
            bool rightLimDisabled = _params[TmclAxisParam.RightLimitSwitchDisable] != 0;
            bool leftLimDisabled = _params[TmclAxisParam.LeftLimitSwitchDisable] != 0;
            bool softStopOnLimit = _params[TmclAxisParam.SoftStopFlag] != 0;
            int effectiveMin = Math.Min(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);
            int effectiveMax = Math.Max(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);

            if (rampMode == 2)
            {
                // Velocity Mode (ROR / ROL / MST Soft Stop)
                double desiredParamSpeed = _params[TmclAxisParam.TargetSpeed];

                bool blockedRight = (desiredParamSpeed > 0 || _exactSpeed > 0.05) && rightLimHit && !rightLimDisabled;
                bool blockedLeft = (desiredParamSpeed < 0 || _exactSpeed < -0.05) && leftLimHit && !leftLimDisabled;

                if (blockedRight || blockedLeft)
                {
                    _params[TmclAxisParam.TargetSpeed] = 0;
                    if (!softStopOnLimit)
                    {
                        _exactSpeed = 0;
                    }
                    else
                    {
                        if (Math.Abs(_exactSpeed) <= maxDeltaV)
                            _exactSpeed = 0;
                        else
                            _exactSpeed -= Math.Sign(_exactSpeed) * maxDeltaV;
                        double dx = _exactSpeed * dtSeconds;
                        _physicalPosition += dx;
                        _exactPosition += dx;
                    }
                }
                else
                {
                    double desiredStepsPerSec = desiredParamSpeed * speedMultiplier;
                    if (Math.Abs(desiredStepsPerSec - _exactSpeed) <= maxDeltaV)
                        _exactSpeed = desiredStepsPerSec;
                    else
                        _exactSpeed += Math.Sign(desiredStepsPerSec - _exactSpeed) * maxDeltaV;

                    double dx = _exactSpeed * dtSeconds;
                    _physicalPosition += dx;
                    _exactPosition += dx;

                    if (_autoTriggerStrokeLimits)
                    {
                        if (_exactSpeed > 0 && !rightLimDisabled && _physicalPosition >= effectiveMax)
                        {
                            if (!softStopOnLimit)
                            {
                                double correction = effectiveMax - _physicalPosition;
                                _physicalPosition = effectiveMax;
                                _exactPosition += correction;
                            }
                            _exactSpeed = softStopOnLimit ? Math.Max(0, _exactSpeed - maxDeltaV) : 0;
                            _params[TmclAxisParam.TargetSpeed] = 0;
                        }
                        else if (_exactSpeed < 0 && !leftLimDisabled && _physicalPosition <= effectiveMin)
                        {
                            if (!softStopOnLimit)
                            {
                                double correction = effectiveMin - _physicalPosition;
                                _physicalPosition = effectiveMin;
                                _exactPosition += correction;
                            }
                            _exactSpeed = softStopOnLimit ? Math.Min(0, _exactSpeed + maxDeltaV) : 0;
                            _params[TmclAxisParam.TargetSpeed] = 0;
                        }
                    }
                }
            }
            else
            {
                // Position Mode (RampMode 0 or 1: MVP ABS / REL / COORD)
                int targetPos = _params[TmclAxisParam.TargetPosition];
                double dist = (double)targetPos - _exactPosition;

                if (Math.Abs(dist) < 0.5 && Math.Abs(_exactSpeed) <= maxDeltaV)
                {
                    double dx = (double)targetPos - _exactPosition;
                    _exactPosition = targetPos;
                    _physicalPosition += dx;
                    _exactSpeed = 0;
                    _interpolationSpeedOverride = 0;
                    _interpolationAccelOverride = 0;
                    _params[TmclAxisParam.TargetSpeed] = 0;
                    _params[TmclAxisParam.ActualAcceleration] = 0;
                }
                else
                {
                    int dir = Math.Abs(dist) >= 0.5 ? Math.Sign(dist) : Math.Sign(_exactSpeed);

                    // Hardware limit stop check when limit switch disable is 0
                    if ((dir > 0 && rightLimHit && !rightLimDisabled) ||
                        (dir < 0 && leftLimHit && !leftLimDisabled))
                    {
                        if (!softStopOnLimit || Math.Abs(_exactSpeed) <= maxDeltaV)
                        {
                            _exactSpeed = 0;
                            _interpolationSpeedOverride = 0;
                            _interpolationAccelOverride = 0;
                            _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                            _params[TmclAxisParam.TargetSpeed] = 0;
                        }
                        else
                        {
                            _exactSpeed -= Math.Sign(_exactSpeed) * maxDeltaV;
                            double dx = _exactSpeed * dtSeconds;
                            _physicalPosition += dx;
                            _exactPosition += dx;
                        }
                    }
                    else
                    {
                        double maxSpeedParam = _interpolationSpeedOverride > 0
                            ? _interpolationSpeedOverride
                            : Math.Max(1, _params[TmclAxisParam.MaxPositioningSpeed]);

                        double maxSpeedStepsPerSec = maxSpeedParam * speedMultiplier;

                        // Trapezoidal braking distance: v_braking = sqrt(2 * a * |dist|)
                        double brakingSpeed = Math.Sqrt(2.0 * accelStepsPerSec2 * Math.Abs(dist));
                        double targetSpeedMag = Math.Min(maxSpeedStepsPerSec, brakingSpeed);
                        double desiredVel = Math.Sign(dist) * targetSpeedMag;

                        double nextSpeed;
                        if (Math.Abs(desiredVel - _exactSpeed) <= maxDeltaV)
                            nextSpeed = desiredVel;
                        else
                            nextSpeed = _exactSpeed + Math.Sign(desiredVel - _exactSpeed) * maxDeltaV;

                        double stepDelta = nextSpeed * dtSeconds;
                        // Prevent overshooting target position within a single tick
                        if (Math.Abs(stepDelta) >= Math.Abs(dist) && Math.Sign(stepDelta) == Math.Sign(dist))
                        {
                            double dx = (double)targetPos - _exactPosition;
                            _exactPosition = targetPos;
                            _physicalPosition += dx;
                            _exactSpeed = 0;
                            _interpolationSpeedOverride = 0;
                            _interpolationAccelOverride = 0;
                        }
                        else
                        {
                            _exactPosition += stepDelta;
                            _physicalPosition += stepDelta;
                            _exactSpeed = nextSpeed;
                        }

                        if (_autoTriggerStrokeLimits)
                        {
                            if (dir > 0 && !rightLimDisabled && _physicalPosition >= effectiveMax)
                            {
                                if (!softStopOnLimit)
                                {
                                    double correction = effectiveMax - _physicalPosition;
                                    _physicalPosition = effectiveMax;
                                    _exactPosition += correction;
                                    _exactSpeed = 0;
                                    _interpolationSpeedOverride = 0;
                                    _interpolationAccelOverride = 0;
                                    _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                                    _params[TmclAxisParam.TargetSpeed] = 0;
                                }
                                else
                                {
                                    _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                                }
                            }
                            else if (dir < 0 && !leftLimDisabled && _physicalPosition <= effectiveMin)
                            {
                                if (!softStopOnLimit)
                                {
                                    double correction = effectiveMin - _physicalPosition;
                                    _physicalPosition = effectiveMin;
                                    _exactPosition += correction;
                                    _exactSpeed = 0;
                                    _interpolationSpeedOverride = 0;
                                    _interpolationAccelOverride = 0;
                                    _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                                    _params[TmclAxisParam.TargetSpeed] = 0;
                                }
                                else
                                {
                                    _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                                }
                            }
                        }

                        _params[TmclAxisParam.TargetSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);
                    }
                }
            }

            _params[TmclAxisParam.ActualPosition] = (int)Math.Round(_exactPosition);
            _params[TmclAxisParam.ActualSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);

            if (dtSeconds > 1e-9 && Math.Abs(_exactSpeed - prevSpeed) > 1e-6)
            {
                double accelInternal = ((_exactSpeed - prevSpeed) / dtSeconds) / (speedMultiplier * 2.5);
                _params[TmclAxisParam.ActualAcceleration] = Math.Clamp((int)Math.Round(accelInternal), -2047, 2047);
            }
            else
            {
                _params[TmclAxisParam.ActualAcceleration] = 0;
            }

            // Automatic StallGuard2 stop on stall (#181)
            int stallVel = _params[TmclAxisParam.SmartEnergyStallVelocity];
            if (stallVel > 0 && Math.Abs(_params[TmclAxisParam.ActualSpeed]) > stallVel && _params[TmclAxisParam.ActualLoadValue] == 0)
            {
                _manualAlarmLatched = true;
                _params[TmclAxisParam.ExtendedErrorFlags] = 1;
                _params[TmclAxisParam.DriverErrorFlags] |= (int)Tmc262DriverErrorFlags.StallGuard2Status;
                HardStopUnsafe();
            }

            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    private void StepReferenceSearchUnsafe(double dtSeconds, double speedMultiplier)
    {
        int rfsMode = _params[TmclAxisParam.ReferenceSearchMode];
        int baseMode = rfsMode & 0x0F;
        bool rightInsteadOfLeft = (rfsMode & 64) != 0;
        bool invertHome = (rfsMode & 128) != 0;

        double searchSpeedParam = Math.Max(0, _params[TmclAxisParam.ReferenceSearchSpeed]);
        double switchSpeedParam = Math.Max(0, _params[TmclAxisParam.ReferenceSwitchSpeed]);

        bool leftHit = _params[TmclAxisParam.LeftLimitSwitchStatus] != 0;
        bool rightHit = _params[TmclAxisParam.RightLimitSwitchStatus] != 0;
        bool primaryLimitHit = rightInsteadOfLeft ? rightHit : leftHit;
        bool oppositeLimitHit = rightInsteadOfLeft ? leftHit : rightHit;
        bool homeActive = invertHome ? !ManualRefSwitchOverride : ManualRefSwitchOverride;

        int primaryDir = rightInsteadOfLeft ? 1 : -1;
        bool clampToLimits = _autoTriggerStrokeLimits && baseMode is >= 1 and <= 6;
        int effectiveMin = Math.Min(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);
        int effectiveMax = Math.Max(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);

        void AdvancePosition(double velocityStepsPerSec)
        {
            _exactSpeed = velocityStepsPerSec;
            double prevPhysical = _physicalPosition;
            double nextPhysical = prevPhysical + _exactSpeed * dtSeconds;
            if (clampToLimits)
            {
                if (nextPhysical > effectiveMax) nextPhysical = effectiveMax;
                else if (nextPhysical < effectiveMin) nextPhysical = effectiveMin;
            }
            double actualDx = nextPhysical - prevPhysical;
            _physicalPosition = nextPhysical;
            _exactPosition += actualDx;
        }

        void CompleteHoming(int calibratedPos)
        {
            _params[TmclAxisParam.LastReferencePosition] = calibratedPos;
            // Only zero the logical position counter; keep _physicalPosition at the actual carriage position
            _exactPosition = 0;
            _exactSpeed = 0;
            _params[TmclAxisParam.ActualPosition] = 0;
            _params[TmclAxisParam.TargetPosition] = 0;
            _params[TmclAxisParam.ActualSpeed] = 0;
            _params[TmclAxisParam.TargetSpeed] = 0;
            _params[TmclAxisParam.ActualAcceleration] = 0;
            _params[TmclAxisParam.RampMode] = 0;
            _params[TmclAxisParam.TargetPosReached] = 1;
            _rfsState = 0;
        }

        switch (baseMode)
        {
            case 2: // Search opposite end switch, then primary end switch
                if (_rfsState == 1)
                {
                    if (oppositeLimitHit)
                    {
                        _rfsFirstSwitchPos = (int)Math.Round(_exactPosition);
                        _rfsState = 2;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 2)
                {
                    if (primaryLimitHit)
                    {
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!primaryLimitHit)
                    {
                        int exitPos = (int)Math.Round(_exactPosition);
                        _params[TmclAxisParam.EndSwitchDistance] = Math.Abs(exitPos - _rfsFirstSwitchPos);
                        CompleteHoming(exitPos);
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;

            case 3: // Search opposite end switch, then primary end switch from both sides
                if (_rfsState == 1)
                {
                    if (oppositeLimitHit)
                    {
                        _rfsFirstSwitchPos = (int)Math.Round(_exactPosition);
                        _rfsState = 2;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 2)
                {
                    if (primaryLimitHit)
                    {
                        _rfsSwitchEnterPos = (int)Math.Round(_exactPosition);
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!primaryLimitHit)
                    {
                        int exitPos = (int)Math.Round(_exactPosition);
                        int centerPos = (_rfsSwitchEnterPos + exitPos) / 2;
                        _params[TmclAxisParam.EndSwitchDistance] = Math.Abs(centerPos - _rfsFirstSwitchPos);
                        CompleteHoming(centerPos);
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;

            case 4: // Search primary end switch from both sides
                if (_rfsState == 1)
                {
                    if (primaryLimitHit)
                    {
                        _rfsSwitchEnterPos = (int)Math.Round(_exactPosition);
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!primaryLimitHit)
                    {
                        int exitPos = (int)Math.Round(_exactPosition);
                        CompleteHoming((_rfsSwitchEnterPos + exitPos) / 2);
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;

            case 5: // Search home switch negative, reverse at left limit
            case 6: // Search home switch positive, reverse at right limit
                if (_rfsState == 1)
                {
                    if (homeActive)
                    {
                        _rfsSwitchEnterPos = (int)Math.Round(_exactPosition);
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else if ((_rfsSearchDir < 0 && leftHit) || (_rfsSearchDir > 0 && rightHit))
                    {
                        _rfsSearchDir = -_rfsSearchDir;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(_rfsSearchDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!homeActive)
                    {
                        int exitPos = (int)Math.Round(_exactPosition);
                        CompleteHoming((_rfsSwitchEnterPos + exitPos) / 2);
                    }
                    else
                    {
                        AdvancePosition(-_rfsSearchDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;

            case 7: // Search home switch negative, ignore end switches
            case 8: // Search home switch positive, ignore end switches
                if (_rfsState == 1)
                {
                    if (homeActive)
                    {
                        _rfsSwitchEnterPos = (int)Math.Round(_exactPosition);
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(_rfsSearchDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!homeActive)
                    {
                        int exitPos = (int)Math.Round(_exactPosition);
                        CompleteHoming((_rfsSwitchEnterPos + exitPos) / 2);
                    }
                    else
                    {
                        AdvancePosition(-_rfsSearchDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;

            default: // Mode 1 / 65: Search primary end switch only
                if (_rfsState == 1)
                {
                    if (primaryLimitHit)
                    {
                        _rfsState = 3;
                        _exactSpeed = 0;
                    }
                    else
                    {
                        AdvancePosition(primaryDir * searchSpeedParam * speedMultiplier);
                    }
                }
                else if (_rfsState == 3)
                {
                    if (!primaryLimitHit)
                    {
                        CompleteHoming((int)Math.Round(_exactPosition));
                    }
                    else
                    {
                        AdvancePosition(-primaryDir * switchSpeedParam * speedMultiplier);
                    }
                }
                break;
        }

        _params[TmclAxisParam.ActualPosition] = (int)Math.Round(_exactPosition);
        _params[TmclAxisParam.ActualSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);
    }

    private void UpdateSwitchAndStatusRegistersUnsafe()
    {
        int logicalPos = (int)Math.Round(_exactPosition);
        int physicalPos = (int)Math.Round(_physicalPosition);
        int target = _params[TmclAxisParam.TargetPosition];

        int effectiveMin = Math.Min(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);
        int effectiveMax = Math.Max(_virtualMinStrokeSteps, _virtualMaxStrokeSteps);
        bool autoRight = _autoTriggerStrokeLimits && physicalPos >= effectiveMax;
        bool autoLeft = _autoTriggerStrokeLimits && physicalPos <= effectiveMin;

        bool rightActive = ManualRightLimitOverride || autoRight;
        bool leftActive = ManualLeftLimitOverride || autoLeft;
        bool refActive = ManualRefSwitchOverride || leftActive;

        _params[TmclAxisParam.RightLimitSwitchStatus] = rightActive ? 1 : 0;
        _params[TmclAxisParam.LeftLimitSwitchStatus] = leftActive ? 1 : 0;
        _params[TmclAxisParam.HomeSwitchState] = refActive ? 1 : 0;

        bool isStopped = Math.Abs(_exactSpeed) < 0.05 && _rfsState == 0;
        bool posReached = isStopped && (_params[TmclAxisParam.RampMode] == 0) && (logicalPos == target);
        _params[TmclAxisParam.TargetPosReached] = posReached ? 1 : 0;

        int rawCurrent = isStopped
            ? _params[TmclAxisParam.StandbyCurrent]
            : (_params[TmclAxisParam.ActualAcceleration] != 0 && _params[TmclAxisParam.BoostCurrent] > 0
                ? _params[TmclAxisParam.BoostCurrent]
                : _params[TmclAxisParam.AbsoluteMaxCurrent]);
        _params[TmclAxisParam.SmartEnergyActualCurrent] = Math.Clamp(rawCurrent / 8, 0, 31);

        // Update dynamic load value (#206) and TMC262 standstill flag (#208) when no alarm is latched
        if (!_manualAlarmLatched)
        {
            if (isStopped)
            {
                _params[TmclAxisParam.DriverErrorFlags] |= (int)Tmc262DriverErrorFlags.StandstillIndicator;
                _params[TmclAxisParam.ActualLoadValue] = 250; // Light load at standstill
            }
            else
            {
                _params[TmclAxisParam.DriverErrorFlags] &= ~(int)Tmc262DriverErrorFlags.StandstillIndicator;
                _params[TmclAxisParam.ActualLoadValue] = 110; // Nominal moving load
            }
        }
    }

    /// <summary>
    /// Snapshot of current axis state for thread-safe UI binding
    /// </summary>
    public AxisSnapshot GetSnapshot()
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();
            int actualPos = _params[TmclAxisParam.ActualPosition];
            int targetPos = _params[TmclAxisParam.TargetPosition];
            int actualSpeed = _params[TmclAxisParam.ActualSpeed];
            bool posReached = _params[TmclAxisParam.TargetPosReached] != 0;
            bool rightLim = _params[TmclAxisParam.RightLimitSwitchStatus] != 0;
            bool leftLim = _params[TmclAxisParam.LeftLimitSwitchStatus] != 0;
            bool refSw = _params[TmclAxisParam.RefSwitchStatus] != 0;
            bool alarm = HasActiveAlarm;

            AxisMotionState motionState;
            if (alarm)
                motionState = AxisMotionState.AlarmStopped;
            else if (_rfsState != 0)
                motionState = AxisMotionState.HomingRfs;
            else if (Math.Abs(actualSpeed) > 0 || (!posReached && _params[TmclAxisParam.RampMode] == 0))
                motionState = _params[TmclAxisParam.RampMode] == 2 ? AxisMotionState.MovingVelocity : AxisMotionState.MovingPosition;
            else
                motionState = AxisMotionState.Idle;

            return new AxisSnapshot(
                AxisIndex,
                AxisName,
                motionState,
                _exactPosition,
                actualPos,
                targetPos,
                actualSpeed,
                _params[TmclAxisParam.MaxPositioningSpeed],
                _params[TmclAxisParam.MaxAcceleration],
                _params[TmclAxisParam.MicrostepResolution],
                _params[TmclAxisParam.ReferenceSearchMode],
                posReached,
                rightLim,
                leftLim,
                refSw,
                alarm,
                _rfsState,
                _params[TmclAxisParam.ActualLoadValue],
                _params[TmclAxisParam.ExtendedErrorFlags],
                _params[TmclAxisParam.DriverErrorFlags],
                _virtualMinStrokeSteps,
                _virtualMaxStrokeSteps,
                _params[TmclAxisParam.RightLimitSwitchDisable] != 0,
                _params[TmclAxisParam.LeftLimitSwitchDisable] != 0,
                _physicalPosition
            );
        }
    }

    public void StartHoming() => StartReferenceSearch();
    public void StopHoming() => StopReferenceSearch();
    public void SetActualPosition(int position) => SetParameter(TmclAxisParam.ActualPosition, position);

    public void SetLimitStopEnabled(bool rightEnabled, bool leftEnabled)
    {
        SetParameter(TmclAxisParam.RightLimitSwitchDisable, rightEnabled ? 0 : 1);
        SetParameter(TmclAxisParam.LeftLimitSwitchDisable, leftEnabled ? 0 : 1);
    }

    public void TriggerFault() => TriggerAlarm();
    public void ClearFault() => ClearAlarm();

    public MotionAxisTelemetry GetTelemetry()
    {
        var snap = GetSnapshot();
        return new MotionAxisTelemetry(
            snap.AxisIndex,
            snap.AxisName,
            snap.MotionState,
            snap.ExactPosition,
            snap.ActualPosition,
            snap.TargetPosition,
            snap.ActualSpeed,
            snap.MaxSpeed,
            snap.MaxAcceleration,
            snap.IsPositionReached,
            snap.IsRightLimitTriggered,
            snap.IsLeftLimitTriggered,
            snap.IsRefSwitchTriggered,
            snap.IsAlarmActive,
            snap.RfsPhase,
            snap.VirtualMinStrokeSteps,
            snap.VirtualMaxStrokeSteps,
            !snap.IsRightLimitDisabled,
            !snap.IsLeftLimitDisabled
        );
    }

    public IReadOnlyList<VendorDiagnosticItem> GetVendorDiagnostics()
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();
            return new VendorDiagnosticItem[]
            {
                new("SG2 Load (#206)", _params[TmclAxisParam.ActualLoadValue].ToString()),
                new("Stall Error (#207)", _params[TmclAxisParam.ExtendedErrorFlags].ToString()),
                new("TMC262 Flags (#208)", $"0x{_params[TmclAxisParam.DriverErrorFlags]:X2}"),
                new("Microstep (#140)", _params[TmclAxisParam.MicrostepResolution].ToString()),
                new("RFS Mode (#193)", _params[TmclAxisParam.ReferenceSearchMode].ToString())
            };
        }
    }
}

public readonly record struct AxisSnapshot(
    int AxisIndex,
    string AxisName,
    AxisMotionState MotionState,
    double ExactPosition,
    int ActualPosition,
    int TargetPosition,
    int ActualSpeed,
    int MaxSpeed,
    int MaxAcceleration,
    int MicrostepResolution,
    int RfsMode,
    bool IsPositionReached,
    bool IsRightLimitTriggered,
    bool IsLeftLimitTriggered,
    bool IsRefSwitchTriggered,
    bool IsAlarmActive,
    int RfsPhase,
    int ActualLoadValue,
    int ExtendedErrorFlags,
    int DriverErrorFlags,
    int VirtualMinStrokeSteps,
    int VirtualMaxStrokeSteps,
    bool IsRightLimitDisabled,
    bool IsLeftLimitDisabled,
    double PhysicalPosition
);
