using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Simulates a single stepper motor axis on the TMCM-6110 (TMC429 motion controller + TMC262 driver).
/// Handles trapezoidal ramping, ROR/ROL/MST/MVP, Reference Search (RFS Mode 1..8 & +64/+128),
/// Limit switches (+Limit / -Limit / Ref), and Alarm flags (StallGuard2 #207 & Driver Errors #208).
/// </summary>
public class AxisState
{
    private readonly object _syncLock = new();
    private readonly Dictionary<byte, int> _params = new();
    private readonly Dictionary<byte, int> _eepromParams = new();
    private readonly int[] _coordinates = new int[64]; // Supports 0..56 coordinate registers

    // High-precision floating-point accumulators for smooth 3D animation & physics
    private double _exactPosition;
    private double _exactSpeed;
    private double _interpolationSpeedOverride; // Non-zero when executing interpolated MVP COORD

    // Reference Search (RFS) internal state machine
    // 0 = Inactive, 1 = Searching primary switch, 2 = Backing off switch, 3 = Final slow calibration
    private int _rfsState;

    // Physical stroke limits (in microsteps) for automatic virtual limit switch triggering
    public int VirtualMinStrokeSteps { get; set; } = -100_000;
    public int VirtualMaxStrokeSteps { get; set; } = 100_000;
    public bool AutoTriggerStrokeLimits { get; set; } = true;

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
            _params[TmclAxisParam.RefSwitchStatus] = 0;
            _params[TmclAxisParam.RightLimitSwitchStatus] = 0;
            _params[TmclAxisParam.LeftLimitSwitchStatus] = 0;
            _params[TmclAxisParam.RightLimitSwitchDisable] = 1; // Default gantry setting: 1
            _params[TmclAxisParam.LeftLimitSwitchDisable] = 1;  // Default gantry setting: 1
            _params[TmclAxisParam.MinimumSpeed] = 1;
            _params[TmclAxisParam.ActualAcceleration] = 0;
            _params[TmclAxisParam.RampMode] = 0;                // 0 = Position mode
            _params[TmclAxisParam.MicrostepResolution] = isActiveXyz ? 6 : 8; // 6 = 64 microsteps
            _params[TmclAxisParam.RefSwitchTolerance] = 0;
            _params[TmclAxisParam.SnapshotPosition] = 0;
            _params[TmclAxisParam.SoftStopFlag] = 0;
            _params[TmclAxisParam.EndSwitchPowerDownMode] = 0;
            _params[TmclAxisParam.PositionLatchFlag] = 0;
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
            _params[TmclAxisParam.SmartEnergyStallVelocity] = 0;
            _params[TmclAxisParam.SmartEnergyThresholdSpeed] = 0;
            _params[TmclAxisParam.SmartEnergySlowRunCurrent] = 0;
            _params[TmclAxisParam.RandomChopperOffTime] = 0;
            _params[TmclAxisParam.ReferenceSearchMode] = isActiveXyz ? 65 : 1; // 65 = Search Right Limit Switch
            _params[TmclAxisParam.ReferenceSearchSpeed] = isActiveXyz ? 1000 : 500;
            _params[TmclAxisParam.ReferenceSwitchSpeed] = isActiveXyz ? 500 : 100;
            _params[TmclAxisParam.BoostCurrent] = 0;
            _params[TmclAxisParam.FreewheelingDelay] = 0;
            _params[TmclAxisParam.ActualLoadValue] = 120;       // Normal nominal load (0..1023, lower = higher load)
            _params[TmclAxisParam.ExtendedErrorFlags] = 0;      // 0 = No StallGuard error
            _params[TmclAxisParam.DriverErrorFlags] = (int)Tmc262DriverErrorFlags.StandstillIndicator;
            _params[TmclAxisParam.EncoderPosition] = 0;
            _params[TmclAxisParam.GroupIndex] = 0;
            _params[TmclAxisParam.PowerDownDelay] = 200;

            _eepromParams.Clear();
            foreach (var kv in _params)
                _eepromParams[kv.Key] = kv.Value;

            _exactPosition = 0;
            _exactSpeed = 0;
            _interpolationSpeedOverride = 0;
            _rfsState = 0;
        }
    }

    public int GetParameter(byte paramNumber, bool clearReadFlags = true)
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();
            int val = _params.TryGetValue(paramNumber, out int v) ? v : 0;

            // According to TMCM-6110 manual, reading #207 clears the StallGuard error flag
            // However, if UI manual alarm is latched, keep it until user clears it or CLE/motion occurs
            if (clearReadFlags && paramNumber == TmclAxisParam.ExtendedErrorFlags && !_manualAlarmLatched)
            {
                _params[TmclAxisParam.ExtendedErrorFlags] = 0;
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
                    _params[TmclAxisParam.RampMode] = 0;
                    _interpolationSpeedOverride = 0;
                    _rfsState = 0;
                    UpdateSwitchAndStatusRegistersUnsafe();
                    break;

                case TmclAxisParam.ActualPosition: // SAP 1: overwrites position counter (e.g. homing zero)
                    _exactPosition = value;
                    _params[TmclAxisParam.EncoderPosition] = value;
                    // If motor was idle, sync TargetPosition so setting ActualPosition=0 doesn't cause unwanted runaway
                    if (Math.Abs(_exactSpeed) < 0.01 && _rfsState == 0)
                    {
                        _params[TmclAxisParam.TargetPosition] = value;
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

    // Coordinate memory (SCO / GCO / CCO)
    public void SetCoordinate(int index, int value)
    {
        lock (_syncLock)
        {
            if (index >= 0 && index < _coordinates.Length)
                _coordinates[index] = value;
        }
    }

    public int GetCoordinate(int index)
    {
        lock (_syncLock)
        {
            return (index >= 0 && index < _coordinates.Length) ? _coordinates[index] : 0;
        }
    }

    public void CaptureCoordinate(int index)
    {
        lock (_syncLock)
        {
            if (index >= 0 && index < _coordinates.Length)
                _coordinates[index] = _params[TmclAxisParam.ActualPosition];
        }
    }

    // Motion Commands
    public void RotateRight(int velocity)
    {
        lock (_syncLock)
        {
            if (_manualAlarmLatched) return;
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _params[TmclAxisParam.RampMode] = 2; // Velocity mode
            _params[TmclAxisParam.TargetSpeed] = Math.Clamp(Math.Abs(velocity), 0, 2047);
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    public void RotateLeft(int velocity)
    {
        lock (_syncLock)
        {
            if (_manualAlarmLatched) return;
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _params[TmclAxisParam.RampMode] = 2; // Velocity mode
            _params[TmclAxisParam.TargetSpeed] = -Math.Clamp(Math.Abs(velocity), 0, 2047);
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    public void StopMotor()
    {
        lock (_syncLock)
        {
            _rfsState = 0;
            _interpolationSpeedOverride = 0;
            _params[TmclAxisParam.TargetSpeed] = 0;
            _exactSpeed = 0;
            _params[TmclAxisParam.ActualSpeed] = 0;
            _params[TmclAxisParam.ActualAcceleration] = 0;
            _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    public void MoveToAbsolute(int targetPos, double? interpolatedMaxSpeed = null)
    {
        lock (_syncLock)
        {
            if (_manualAlarmLatched) return;
            _rfsState = 0;
            _params[TmclAxisParam.RampMode] = 0; // Position mode
            _params[TmclAxisParam.TargetPosition] = Math.Clamp(targetPos, -8388608, 8388607);
            _interpolationSpeedOverride = interpolatedMaxSpeed ?? 0;
            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    public void MoveRelative(int offset)
    {
        lock (_syncLock)
        {
            if (_manualAlarmLatched) return;
            int currentTargetOrActual = _params[TmclAxisParam.ActualPosition];
            MoveToAbsolute(currentTargetOrActual + offset);
        }
    }

    // Reference Search (RFS)
    public void StartReferenceSearch()
    {
        lock (_syncLock)
        {
            if (_manualAlarmLatched) return;
            _rfsState = 1; // Phase 1: Search reference/limit switch at ReferenceSearchSpeed (#194)
            _interpolationSpeedOverride = 0;
            _params[TmclAxisParam.TargetPosReached] = 0;
        }
    }

    public void StopReferenceSearch()
    {
        lock (_syncLock)
        {
            _rfsState = 0;
            StopMotor();
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

    public void TriggerAlarm(Tmc262DriverErrorFlags driverError = Tmc262DriverErrorFlags.StallGuard2Status | Tmc262DriverErrorFlags.OvertemperatureWarning, bool stallError = true)
    {
        lock (_syncLock)
        {
            _manualAlarmLatched = true;
            if (stallError)
                _params[TmclAxisParam.ExtendedErrorFlags] = 1; // StallGuard error
            _params[TmclAxisParam.DriverErrorFlags] = (int)(driverError | Tmc262DriverErrorFlags.StandstillIndicator);
            _params[TmclAxisParam.ActualLoadValue] = 0; // 0 = Max stall load in StallGuard2
            StopMotor();
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
            _params[TmclAxisParam.ActualLoadValue] = 120;
        }
    }

    /// <summary>
    /// Advances the physics simulation by dtSeconds (called at ~100-500 Hz).
    /// Speed scale factor converts TMCL speed parameter (0..2047) to steps/sec.
    /// </summary>
    public void StepPhysics(double dtSeconds, double speedMultiplier = 10.0)
    {
        lock (_syncLock)
        {
            UpdateSwitchAndStatusRegistersUnsafe();

            if (_manualAlarmLatched)
            {
                _exactSpeed = 0;
                _params[TmclAxisParam.ActualSpeed] = 0;
                return;
            }

            // 1. Handle Reference Search (RFS) State Machine
            if (_rfsState != 0)
            {
                StepReferenceSearchUnsafe(dtSeconds, speedMultiplier);
                UpdateSwitchAndStatusRegistersUnsafe();
                return;
            }

            int rampMode = _params[TmclAxisParam.RampMode];
            double maxAccelParam = Math.Max(10, _params[TmclAxisParam.MaxAcceleration]);
            double accelStepsPerSec2 = maxAccelParam * speedMultiplier * 2.5;

            bool rightLimHit = _params[TmclAxisParam.RightLimitSwitchStatus] != 0;
            bool leftLimHit = _params[TmclAxisParam.LeftLimitSwitchStatus] != 0;
            bool rightLimDisabled = _params[TmclAxisParam.RightLimitSwitchDisable] != 0;
            bool leftLimDisabled = _params[TmclAxisParam.LeftLimitSwitchDisable] != 0;

            if (rampMode == 2)
            {
                // Velocity Mode (ROR / ROL)
                double desiredParamSpeed = _params[TmclAxisParam.TargetSpeed];

                // Check hardware limit switch stop if enabled (#12 == 0 or #13 == 0)
                if ((desiredParamSpeed > 0 && rightLimHit && !rightLimDisabled) ||
                    (desiredParamSpeed < 0 && leftLimHit && !leftLimDisabled))
                {
                    _exactSpeed = 0;
                    _params[TmclAxisParam.TargetSpeed] = 0;
                }
                else
                {
                    double desiredStepsPerSec = desiredParamSpeed * speedMultiplier;
                    double maxDeltaV = accelStepsPerSec2 * dtSeconds;
                    if (Math.Abs(desiredStepsPerSec - _exactSpeed) <= maxDeltaV)
                        _exactSpeed = desiredStepsPerSec;
                    else
                        _exactSpeed += Math.Sign(desiredStepsPerSec - _exactSpeed) * maxDeltaV;

                    _exactPosition += _exactSpeed * dtSeconds;
                }
            }
            else
            {
                // Position Mode (RampMode 0 or 1: MVP ABS / REL / COORD)
                int targetPos = _params[TmclAxisParam.TargetPosition];
                double dist = targetPos - _exactPosition;

                if (Math.Abs(dist) < 0.5)
                {
                    _exactPosition = targetPos;
                    _exactSpeed = 0;
                    _interpolationSpeedOverride = 0;
                    _params[TmclAxisParam.TargetSpeed] = 0;
                    _params[TmclAxisParam.ActualAcceleration] = 0;
                }
                else
                {
                    int dir = Math.Sign(dist);

                    // Hardware limit stop check when limit switch disable is 0
                    if ((dir > 0 && rightLimHit && !rightLimDisabled) ||
                        (dir < 0 && leftLimHit && !leftLimDisabled))
                    {
                        _exactSpeed = 0;
                        _params[TmclAxisParam.TargetPosition] = (int)Math.Round(_exactPosition);
                        _params[TmclAxisParam.TargetSpeed] = 0;
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
                        double desiredVel = dir * targetSpeedMag;

                        double maxDeltaV = accelStepsPerSec2 * dtSeconds;
                        double nextSpeed;
                        if (Math.Abs(desiredVel - _exactSpeed) <= maxDeltaV)
                            nextSpeed = desiredVel;
                        else
                            nextSpeed = _exactSpeed + Math.Sign(desiredVel - _exactSpeed) * maxDeltaV;

                        double stepDelta = nextSpeed * dtSeconds;
                        // Prevent overshooting target position within a single tick
                        if (Math.Abs(stepDelta) >= Math.Abs(dist))
                        {
                            _exactPosition = targetPos;
                            _exactSpeed = 0;
                            _interpolationSpeedOverride = 0;
                        }
                        else
                        {
                            _exactPosition += stepDelta;
                            _exactSpeed = nextSpeed;
                        }

                        _params[TmclAxisParam.TargetSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);
                    }
                }
            }

            _params[TmclAxisParam.ActualPosition] = (int)Math.Round(_exactPosition);
            _params[TmclAxisParam.EncoderPosition] = _params[TmclAxisParam.ActualPosition];
            _params[TmclAxisParam.ActualSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);

            UpdateSwitchAndStatusRegistersUnsafe();
        }
    }

    private void StepReferenceSearchUnsafe(double dtSeconds, double speedMultiplier)
    {
        int rfsMode = _params[TmclAxisParam.ReferenceSearchMode];
        // In TMCM-6110: Bit 6 (+64) = Search Right Limit Switch instead of Left Limit Switch
        // Default gantry profile uses Mode 65 (1 + 64) for Z, Y, X axes.
        bool searchRightSwitch = (rfsMode & 64) != 0;
        int searchDir = searchRightSwitch ? 1 : -1;

        double searchSpeedParam = Math.Max(10, _params[TmclAxisParam.ReferenceSearchSpeed]);
        double switchSpeedParam = Math.Max(5, _params[TmclAxisParam.ReferenceSwitchSpeed]);

        bool targetSwitchActive = searchRightSwitch
            ? (_params[TmclAxisParam.RightLimitSwitchStatus] != 0)
            : (_params[TmclAxisParam.LeftLimitSwitchStatus] != 0 || _params[TmclAxisParam.RefSwitchStatus] != 0);

        if (_rfsState == 1)
        {
            // Phase 1: Move towards primary reference switch at ReferenceSearchSpeed (#194)
            if (targetSwitchActive)
            {
                _rfsState = 2; // Switch hit! Transition to backoff phase
                _exactSpeed = 0;
            }
            else
            {
                _exactSpeed = searchDir * searchSpeedParam * speedMultiplier;
                _exactPosition += _exactSpeed * dtSeconds;
            }
        }
        else if (_rfsState == 2)
        {
            // Phase 2: Move slowly in opposite direction until switch is released, then zero position
            if (!targetSwitchActive)
            {
                // Reference point calibrated! Set ActualPosition (#1) and TargetPosition (#0) to 0
                _params[TmclAxisParam.LastReferencePosition] = (int)Math.Round(_exactPosition);
                _exactPosition = 0;
                _exactSpeed = 0;
                _params[TmclAxisParam.ActualPosition] = 0;
                _params[TmclAxisParam.TargetPosition] = 0;
                _params[TmclAxisParam.EncoderPosition] = 0;
                _params[TmclAxisParam.ActualSpeed] = 0;
                _params[TmclAxisParam.TargetSpeed] = 0;
                _rfsState = 0; // Homing complete!
            }
            else
            {
                _exactSpeed = -searchDir * switchSpeedParam * speedMultiplier;
                _exactPosition += _exactSpeed * dtSeconds;
            }
        }

        _params[TmclAxisParam.ActualPosition] = (int)Math.Round(_exactPosition);
        _params[TmclAxisParam.ActualSpeed] = (int)Math.Round(_exactSpeed / speedMultiplier);
    }

    private void UpdateSwitchAndStatusRegistersUnsafe()
    {
        int pos = (int)Math.Round(_exactPosition);
        int target = _params[TmclAxisParam.TargetPosition];

        bool autoRight = AutoTriggerStrokeLimits && pos >= VirtualMaxStrokeSteps;
        bool autoLeft = AutoTriggerStrokeLimits && pos <= VirtualMinStrokeSteps;

        bool rightActive = ManualRightLimitOverride || autoRight;
        bool leftActive = ManualLeftLimitOverride || autoLeft;
        bool refActive = ManualRefSwitchOverride || leftActive;

        _params[TmclAxisParam.RightLimitSwitchStatus] = rightActive ? 1 : 0;
        _params[TmclAxisParam.LeftLimitSwitchStatus] = leftActive ? 1 : 0;
        _params[TmclAxisParam.RefSwitchStatus] = refActive ? 1 : 0;

        bool isStopped = Math.Abs(_exactSpeed) < 0.05 && _rfsState == 0;
        bool posReached = isStopped && (_params[TmclAxisParam.RampMode] == 0) && (pos == target);
        _params[TmclAxisParam.TargetPosReached] = posReached ? 1 : 0;

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
                _params[TmclAxisParam.DriverErrorFlags]
            );
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
    int DriverErrorFlags
);
