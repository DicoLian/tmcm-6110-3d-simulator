namespace Tmcm6110.Core.Protocol;

/// <summary>
/// TMCL Binary Opcodes according to TMCM-6110 TMCL Firmware Manual V1.47 (Rev V1.23)
/// </summary>
public enum TmclCommand : byte
{
    ROR = 1,            // Rotate Right
    ROL = 2,            // Rotate Left
    MST = 3,            // Motor Stop (Soft Stop)
    MVP = 4,            // Move to Position (Type: 0=ABS, 1=REL, 2=COORD)
    SAP = 5,            // Set Axis Parameter
    GAP = 6,            // Get Axis Parameter
    STAP = 7,           // Store Axis Parameter into EEPROM
    RSAP = 8,           // Restore Axis Parameter from EEPROM
    SGP = 9,            // Set Global Parameter
    GGP = 10,           // Get Global Parameter
    STGP = 11,          // Store Global Parameter into EEPROM
    RSGP = 12,          // Restore Global Parameter from EEPROM
    RFS = 13,           // Reference Search (Type: 0=START, 1=STOP, 2=STATUS)
    SIO = 14,           // Set Digital Output / Pull-up Resistors
    GIO = 15,           // Get Digital/Analog Input/Output
    SAPX = 16,          // Set Axis Parameter, Axis specified by X Register
    GAPX = 17,          // Get Axis Parameter, Axis specified by X Register
    AAPX = 18,          // Accu to Axis Parameter, Axis specified by X Register
    CALC = 19,          // Calculate using Accumulator
    COMP = 20,          // Compare
    JC = 21,            // Jump Conditional
    JA = 22,            // Jump Always
    CSUB = 23,          // Call Subroutine
    RSUB = 24,          // Return from Subroutine
    EI = 25,            // Enable Interrupt
    DI = 26,            // Disable Interrupt
    WAIT = 27,          // Wait for Event
    STOP = 28,          // Stop TMCL Program
    SCO = 30,           // Set Coordinate
    GCO = 31,           // Get Coordinate
    CCO = 32,           // Capture Coordinate
    CALCX = 33,         // Calculate using X Register
    AAP = 34,           // Accumulator to Axis Parameter
    AGP = 35,           // Accumulator to Global Parameter
    CLE = 36,           // Clear Error Flags
    VECT = 37,          // Define Interrupt Vector
    RETI = 38,          // Return from Interrupt
    ACO = 39,           // Accu to Coordinate
    CALCVV = 40,        // Calculate using two User Variables
    CALCVA = 41,        // Calculate using User Variable and Accumulator
    CALCAV = 42,        // Calculate using Accumulator and User Variable
    CALCVX = 43,        // Calculate using User Variable and X Register
    CALCXV = 44,        // Calculate using X Register and User Variable
    CALCV = 45,         // Calculate using User Variable and Direct Value
    MVPA = 46,          // Move to Position specified by Accumulator
    MVPXA = 47,         // Move to Position specified by Accumulator and X Register
    RST = 48,           // Restart Program
    DJNZ = 49,          // Decrement and Jump if not Zero
    ROLA = 50,          // Rotate Left using Accumulator
    RORA = 51,          // Rotate Right using Accumulator
    ROLXA = 52,         // Rotate Left using Accumulator and X Register
    RORXA = 53,         // Rotate Right using Accumulator and X Register
    MSTX = 54,          // Motor Stop using X Register
    SIV = 55,           // Set Indexed Variable
    GIV = 56,           // Get Indexed Variable
    AIV = 57,           // Accumulator to Indexed Variable
    CALL = 80,          // Conditional Subroutine Call

    // Host Control Functions
    StopApplication = 128,
    RunApplication = 129,
    StepApplication = 130,
    ResetApplication = 131,
    StartDownload = 132,
    QuitDownload = 133,
    ReadTmclMemory = 134,
    GetAppStatus = 135,
    GetFirmwareVersion = 136,
    RestoreFactorySettings = 137,
    RequestTargetPositionReachedEvent = 138, // Sends Status=100 immediately, then Status=128 when reached
    EnterAsciiMode = 139,
    SoftwareReset = 255
}

/// <summary>
/// TMCL Reply Status Codes (TMCM-6110 FW V1.47 Manual Section 3.2.1 &amp; 3.7.59)
/// </summary>
public enum TmclStatus : byte
{
    WrongChecksum = 1,
    InvalidCommand = 2,
    WrongType = 3,
    InvalidValue = 4,
    ConfigurationEepromLocked = 5,
    CommandNotAvailable = 6,

    SuccessOk = 100,
    CommandLoadedIntoEeprom = 101,
    TargetPositionReachedEvent = 128 // Asynchronous 2nd reply of Command 138
}

/// <summary>
/// High-level motion state of an axis for UI visualization and status monitoring
/// </summary>
public enum AxisMotionState
{
    Idle = 0,
    MovingPosition = 1,     // MVP ABS / REL / COORD
    MovingVelocity = 2,     // ROR / ROL
    HomingRfs = 3,          // Reference Search (RFS) in progress
    AlarmStopped = 4        // Stopped due to Alarm (StallGuard2 / Driver Error)
}

/// <summary>
/// TMC262 Driver Error Bit Flags (Axis Parameter #208)
/// </summary>
[Flags]
public enum Tmc262DriverErrorFlags : int
{
    None = 0,
    StallGuard2Status = 1 << 0,         // Bit 0 (0x01): StallGuard2 threshold reached
    OvertemperatureShutdown = 1 << 1,   // Bit 1 (0x02): Overtemperature shutdown
    OvertemperatureWarning = 1 << 2,    // Bit 2 (0x04): Overtemperature pre-warning
    ShortToGroundA = 1 << 3,            // Bit 3 (0x08): Short to ground phase A
    ShortToGroundB = 1 << 4,            // Bit 4 (0x10): Short to ground phase B
    OpenLoadA = 1 << 5,                 // Bit 5 (0x20): Open load phase A
    OpenLoadB = 1 << 6,                 // Bit 6 (0x40): Open load phase B
    StandstillIndicator = 1 << 7        // Bit 7 (0x80): Standstill indicator
}

/// <summary>
/// Known Axis Parameter Numbers for TMCM-6110 (TMC429 + TMC262, FW V1.47 Chapter 4 Table 16)
/// </summary>
public static class TmclAxisParam
{
    public const byte TargetPosition = 0;
    public const byte ActualPosition = 1;
    public const byte TargetSpeed = 2;
    public const byte ActualSpeed = 3;
    public const byte MaxPositioningSpeed = 4;
    public const byte MaxAcceleration = 5;
    public const byte AbsoluteMaxCurrent = 6;
    public const byte StandbyCurrent = 7;
    public const byte TargetPosReached = 8;
    public const byte HomeSwitchState = 9;          // Home Reference Switch Input
    public const byte RefSwitchStatus = HomeSwitchState;
    public const byte RightLimitSwitchStatus = 10;  // +Limit Switch
    public const byte LeftLimitSwitchStatus = 11;   // -Limit Switch
    public const byte RightLimitSwitchDisable = 12;
    public const byte LeftLimitSwitchDisable = 13;
    public const byte MinimumSpeed = 130;
    public const byte ActualAcceleration = 135;
    public const byte RampMode = 138;               // 0=Position, 1=Soft, 2=Velocity
    public const byte MicrostepResolution = 140;    // 0=Full..6=64..8=256
    public const byte SoftStopFlag = 149;
    public const byte EndSwitchPowerDownMode = 150;
    public const byte RampDivisor = 153;
    public const byte PulseDivisor = 154;
    public const byte StepInterpolation = 160;
    public const byte DoubleStep = 161;
    public const byte ChopperBlankTime = 162;
    public const byte ConstantTOffMode = 163;
    public const byte DisableFastDecayComparator = 164;
    public const byte ChopperHysteresisEnd = 165;
    public const byte ChopperHysteresisStart = 166;
    public const byte ChopperOffTime = 167;
    public const byte SmartEnergyCurrentMin = 168;
    public const byte SmartEnergyCurrentDownStep = 169;
    public const byte SmartEnergyHysteresis = 170;
    public const byte SmartEnergyCurrentUpStep = 171;
    public const byte SmartEnergyHysteresisStart = 172;
    public const byte StallGuard2FilterEnable = 173;
    public const byte StallGuard2Threshold = 174;
    public const byte SlopeControlHighSide = 175;
    public const byte SlopeControlLowSide = 176;
    public const byte ShortToGroundProtection = 177;
    public const byte ShortDetectionTimer = 178;
    public const byte VSense = 179;
    public const byte SmartEnergyActualCurrent = 180;
    public const byte SmartEnergyStallVelocity = 181;
    public const byte SmartEnergyThresholdSpeed = 182;
    public const byte SmartEnergySlowRunCurrent = 183;
    public const byte RandomChopperOffTime = 184;
    public const byte ReferenceSearchMode = 193;    // 1..8, +64=Right limit, +128=Invert home
    public const byte ReferenceSearchSpeed = 194;
    public const byte ReferenceSwitchSpeed = 195;
    public const byte EndSwitchDistance = 196;
    public const byte LastReferencePosition = 197;
    public const byte BoostCurrent = 200;
    public const byte FreewheelingDelay = 204;
    public const byte ActualLoadValue = 206;        // StallGuard2 Load (0..1023)
    public const byte ExtendedErrorFlags = 207;     // Bit 0: StallGuard Error (Alarm)
    public const byte DriverErrorFlags = 208;       // TMC262 Driver Error Flags (Alarm)
    public const byte GroupIndex = 213;
    public const byte PowerDownDelay = 214;

    /// <summary>
    /// Returns true if <paramref name="param"/> is defined in TMCM-6110 FW V1.47 Table 16.
    /// </summary>
    public static bool IsValidParameter(byte param) => param switch
    {
        >= 0 and <= 13 => true,
        130 or 135 or 138 or 140 or 149 or 150 or 153 or 154 => true,
        >= 160 and <= 184 => true,
        >= 193 and <= 197 => true,
        200 or 204 or 206 or 207 or 208 or 213 or 214 => true,
        _ => false
    };

    /// <summary>
    /// Returns true if <paramref name="param"/> is read-only (R access only) in TMCM-6110 FW V1.47 Table 16.
    /// </summary>
    public static bool IsReadOnly(byte param) => param switch
    {
        ActualSpeed or TargetPosReached or HomeSwitchState or RightLimitSwitchStatus or LeftLimitSwitchStatus
            or ActualAcceleration or VSense or SmartEnergyActualCurrent or EndSwitchDistance
            or LastReferencePosition or ActualLoadValue or ExtendedErrorFlags or DriverErrorFlags => true,
        _ => false
    };

    /// <summary>
    /// Validates whether <paramref name="value"/> falls within the documented FW V1.47 Table 16 range for <paramref name="param"/>.
    /// </summary>
    public static bool IsValueValid(byte param, int value) => param switch
    {
        TargetPosition or ActualPosition => true,
        TargetSpeed => value >= -2047 && value <= 2047,
        MaxPositioningSpeed or MaxAcceleration or MinimumSpeed => value >= 1 && value <= 2047,
        AbsoluteMaxCurrent or StandbyCurrent or SmartEnergySlowRunCurrent or BoostCurrent or GroupIndex => value >= 0 && value <= 255,
        RightLimitSwitchDisable or LeftLimitSwitchDisable or SoftStopFlag or EndSwitchPowerDownMode
            or StepInterpolation or DoubleStep or ConstantTOffMode or DisableFastDecayComparator
            or SmartEnergyCurrentMin or StallGuard2FilterEnable or ShortToGroundProtection or RandomChopperOffTime => value is 0 or 1,
        RampMode => value is 0 or 1 or 2,
        MicrostepResolution or ChopperHysteresisStart => value >= 0 && value <= 8,
        RampDivisor or PulseDivisor => value >= 0 && value <= 13,
        ChopperBlankTime or SmartEnergyCurrentDownStep or SmartEnergyCurrentUpStep
            or SlopeControlHighSide or SlopeControlLowSide or ShortDetectionTimer => value >= 0 && value <= 3,
        ChopperHysteresisEnd or ChopperOffTime or SmartEnergyHysteresis or SmartEnergyHysteresisStart => value >= 0 && value <= 15,
        StallGuard2Threshold => value >= -64 && value <= 63,
        SmartEnergyStallVelocity or SmartEnergyThresholdSpeed or ReferenceSearchSpeed or ReferenceSwitchSpeed => value >= 0 && value <= 2047,
        ReferenceSearchMode => (value >= 1 && value <= 8) || (value >= 65 && value <= 68) || (value >= 133 && value <= 136),
        FreewheelingDelay => value >= 0 && value <= 65535,
        PowerDownDelay => value >= 1 && value <= 65535,
        _ => false
    };

    public static string GetParameterName(byte param) => param switch
    {
        0 => "TargetPos(#0)",
        1 => "ActualPos(#1)",
        2 => "TargetSpeed(#2)",
        3 => "ActualSpeed(#3)",
        4 => "MaxPosSpeed(#4)",
        5 => "MaxAccel(#5)",
        6 => "MaxCurrent(#6)",
        7 => "StandbyCurrent(#7)",
        8 => "PosReached(#8)",
        9 => "HomeSwitch(#9)",
        10 => "+LimitSwitch(#10)",
        11 => "-LimitSwitch(#11)",
        12 => "+LimitDisable(#12)",
        13 => "-LimitDisable(#13)",
        130 => "MinSpeed(#130)",
        135 => "ActualAccel(#135)",
        138 => "RampMode(#138)",
        140 => "MicrostepRes(#140)",
        149 => "SoftStop(#149)",
        150 => "EndSwitchPowerDown(#150)",
        153 => "RampDiv(#153)",
        154 => "PulseDiv(#154)",
        173 => "SG2Filter(#173)",
        174 => "SG2Threshold(#174)",
        180 => "SmartEnergyActualCurrent(#180)",
        181 => "StopOnStallVel(#181)",
        193 => "RfsMode(#193)",
        194 => "RfsSearchSpeed(#194)",
        195 => "RfsSwitchSpeed(#195)",
        196 => "EndSwitchDist(#196)",
        197 => "LastRefPos(#197)",
        200 => "BoostCurrent(#200)",
        204 => "Freewheeling(#204)",
        206 => "ActualLoad(#206)",
        207 => "StallErrorFlag(#207)",
        208 => "DriverErrorFlag(#208)",
        213 => "GroupIndex(#213)",
        214 => "PowerDownDelay(#214)",
        _ => $"Param#{param}"
    };
}
