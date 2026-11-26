using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Standalone TMCL application execution status (Global Parameter #128, FW V1.47 Table 18).
/// </summary>
public enum TmclAppStatus : int
{
    Stop = 0,
    Run = 1,
    Step = 2,
    Reset = 3
}

/// <summary>
/// Thread-safe snapshot of the standalone TMCL virtual machine state for UI and diagnostics.
/// </summary>
public readonly record struct StandaloneVmSnapshot(
    TmclAppStatus Status,
    bool IsDownloadMode,
    int ProgramCounter,
    int WritePointer,
    int InstructionCount,
    int Accumulator,
    int XRegister,
    int ErrorFlags,
    bool IsWaiting,
    string WaitDescription,
    int CallStackDepth,
    bool GlobalInterruptsEnabled,
    bool IsInInterrupt,
    int ActiveInterruptNumber,
    int CodeProtectionMode,
    bool AutoStartEnabled,
    long ExecutedInstructionCount);

internal interface ITmclVmHost
{
    int AxisCount { get; }
    int GetAxisParameterRaw(int axis, byte param);
    int GetReferenceSearchStatus(int axis);
    bool GetDigitalInput(int port);
    int GetGlobalParameterRaw(byte bank, byte param);
    void SetUserVariableRaw(byte index, int value);
    void SyncErrorFlagsFromVm(int errorFlags);
    TmclStatus ExecuteHardwareCommand(
        byte opcode,
        byte type,
        byte motorOrBank,
        int value,
        bool isStandalone,
        out int readResult,
        out bool hasReadResult);
}

/// <summary>
/// Executes standalone TMCL programs stored in <see cref="TmclProgramMemory"/>, including
/// Accumulator/X-Register ALU operations, subroutines (8-level stack), WAIT timers/events,
/// and the FW V1.47 47-vector interrupt controller (timers, position reached, stall, stop switches, digital inputs).
/// All methods expect to be invoked under <see cref="Tmcm6110Controller"/>'s synchronization lock.
/// </summary>
internal sealed class TmclStandaloneVm
{
    public const int MaxCallStackDepth = 8;
    public const int MaxInterruptCount = 47;
    public const int MaxInstructionsPerTick = 64;
    private const double TickDurationMs = 10.0;

    private readonly struct InterruptContext
    {
        public readonly int ProgramCounter;
        public readonly int Accumulator;
        public readonly int XRegister;
        public readonly bool FlagZero;
        public readonly bool FlagEqual;
        public readonly bool FlagGreater;
        public readonly bool FlagLess;
        public readonly bool IsWaiting;
        public readonly byte WaitType;
        public readonly byte WaitMotor;
        public readonly double WaitElapsedMs;
        public readonly double WaitTimeoutMs;

        public InterruptContext(
            int programCounter,
            int accumulator,
            int xRegister,
            bool flagZero,
            bool flagEqual,
            bool flagGreater,
            bool flagLess,
            bool isWaiting,
            byte waitType,
            byte waitMotor,
            double waitElapsedMs,
            double waitTimeoutMs)
        {
            ProgramCounter = programCounter;
            Accumulator = accumulator;
            XRegister = xRegister;
            FlagZero = flagZero;
            FlagEqual = flagEqual;
            FlagGreater = flagGreater;
            FlagLess = flagLess;
            IsWaiting = isWaiting;
            WaitType = waitType;
            WaitMotor = waitMotor;
            WaitElapsedMs = waitElapsedMs;
            WaitTimeoutMs = waitTimeoutMs;
        }
    }

    private readonly TmclProgramMemory _memory;
    private readonly ITmclVmHost _host;

    private readonly int[] _callStack = new int[MaxCallStackDepth];
    private int _callStackDepth;

    private readonly int[] _interruptVectors = new int[MaxInterruptCount];
    private readonly bool[] _interruptEnabled = new bool[MaxInterruptCount];
    private readonly bool[] _pendingInterrupts = new bool[MaxInterruptCount];
    private readonly double[] _timerAccumMs = new double[3];

    private readonly bool[] _prevPosReached = new bool[Tmcm6110Controller.TotalAxes];
    private readonly bool[] _prevStall = new bool[Tmcm6110Controller.TotalAxes];
    private readonly bool[] _prevLeftLimit = new bool[Tmcm6110Controller.TotalAxes];
    private readonly bool[] _prevRightLimit = new bool[Tmcm6110Controller.TotalAxes];
    private readonly bool[] _prevDigitalInputs = new bool[8];

    private InterruptContext _savedContext;
    private byte _waitType;
    private byte _waitMotor;
    private double _waitElapsedMs;
    private double _waitTimeoutMs;

    public TmclAppStatus Status { get; set; } = TmclAppStatus.Stop;
    public int ProgramCounter { get; set; }
    public int Accumulator { get; set; }
    public int XRegister { get; set; }
    public int ErrorFlags { get; private set; }
    public bool FlagZero { get; private set; }
    public bool FlagEqual { get; private set; }
    public bool FlagGreater { get; private set; }
    public bool FlagLess { get; private set; }
    public bool IsWaiting { get; private set; }
    public bool GlobalInterruptsEnabled { get; private set; }
    public bool IsInInterrupt { get; private set; }
    public int ActiveInterruptNumber { get; private set; } = -1;
    public long ExecutedInstructionCount { get; private set; }

    public TmclStandaloneVm(TmclProgramMemory memory, ITmclVmHost host)
    {
        _memory = memory ?? throw new ArgumentNullException(nameof(memory));
        _host = host ?? throw new ArgumentNullException(nameof(host));
        ResetRuntimeState(TmclAppStatus.Stop);
    }

    public void Stop()
    {
        Status = TmclAppStatus.Stop;
        ClearWaitState();
    }

    public void Run(int? startAddress = null)
    {
        if (startAddress.HasValue)
        {
            ProgramCounter = startAddress.Value;
            ClearWaitState();
            _callStackDepth = 0;
        }
        Status = TmclAppStatus.Run;
    }

    public void ResetApplication() => ResetRuntimeState(TmclAppStatus.Reset);

    public void ResetToStopped() => ResetRuntimeState(TmclAppStatus.Stop);

    public void ResetForPowerCycle(bool autoStart)
    {
        ResetRuntimeState(autoStart ? TmclAppStatus.Run : TmclAppStatus.Stop);
    }

    private void ResetRuntimeState(TmclAppStatus initialStatus)
    {
        Status = initialStatus;
        ExecutedInstructionCount = 0;
        ResetVmExecutionContext(targetAddress: 0);
    }

    private void ResetVmExecutionContext(int targetAddress)
    {
        ProgramCounter = targetAddress;
        Accumulator = 0;
        XRegister = 0;
        ErrorFlags = 0;
        _host.SyncErrorFlagsFromVm(0);
        FlagZero = false;
        FlagEqual = false;
        FlagGreater = false;
        FlagLess = false;
        ClearWaitState();
        _callStackDepth = 0;
        Array.Clear(_callStack);

        GlobalInterruptsEnabled = false;
        IsInInterrupt = false;
        ActiveInterruptNumber = -1;
        _savedContext = default;
        Array.Fill(_interruptVectors, -1);
        Array.Clear(_interruptEnabled);
        Array.Clear(_pendingInterrupts);
        Array.Clear(_timerAccumMs);

        SampleInitialEdgeBaseline();
    }

    private void SampleInitialEdgeBaseline()
    {
        int axes = _host.AxisCount;
        for (int i = 0; i < axes; i++)
        {
            _prevPosReached[i] = _host.GetAxisParameterRaw(i, TmclAxisParam.TargetPosReached) != 0;
            _prevStall[i] = _host.GetAxisParameterRaw(i, TmclAxisParam.ExtendedErrorFlags) != 0;
            _prevLeftLimit[i] = _host.GetAxisParameterRaw(i, TmclAxisParam.LeftLimitSwitchStatus) != 0;
            _prevRightLimit[i] = _host.GetAxisParameterRaw(i, TmclAxisParam.RightLimitSwitchStatus) != 0;
        }

        for (int p = 0; p < _prevDigitalInputs.Length; p++)
            _prevDigitalInputs[p] = _host.GetDigitalInput(p);
    }

    public void SetAccumulatorWithZeroFlag(int value)
    {
        Accumulator = value;
        FlagZero = value == 0;
    }

    public void ClearErrorFlag(byte flagType)
    {
        if (flagType == 0)
            ErrorFlags = 0;
        else if (flagType <= 5)
            ErrorFlags &= ~(1 << (flagType - 1));
    }

    public void EnableInterrupt(byte irq)
    {
        if (irq == 255)
            GlobalInterruptsEnabled = true;
        else if (irq < MaxInterruptCount)
            _interruptEnabled[irq] = true;
    }

    public void DisableInterrupt(byte irq)
    {
        if (irq == 255)
            GlobalInterruptsEnabled = false;
        else if (irq < MaxInterruptCount)
        {
            _interruptEnabled[irq] = false;
            _pendingInterrupts[irq] = false;
        }
    }

    public void SetComparisonFlags(int lhs, int rhs)
    {
        FlagZero = lhs == rhs;
        FlagEqual = lhs == rhs;
        FlagGreater = lhs > rhs;
        FlagLess = lhs < rhs;
    }

    public bool EvaluateCondition(byte condType) => condType switch
    {
        0 => FlagZero,
        1 => !FlagZero,
        2 => FlagEqual,
        3 => !FlagEqual,
        4 => FlagGreater,
        5 => FlagGreater || FlagEqual,
        6 => FlagLess,
        7 => FlagLess || FlagEqual,
        8 => (ErrorFlags & 1) != 0,
        9 => (ErrorFlags & 2) != 0,
        10 => (ErrorFlags & 4) != 0,
        11 => (ErrorFlags & 8) != 0,
        _ => false
    };

    /// <summary>
    /// Shared arithmetic/logic kernel for CALC, CALCX, and CALCV* operations (0..9).
    /// Returns false if operation is invalid or divides/modulos by zero.
    /// </summary>
    public static bool TryComputeAluOp(byte op, int lhs, int rhs, out int result, out bool isInvalidValue)
    {
        isInvalidValue = false;
        switch (op)
        {
            case 0: result = unchecked(lhs + rhs); return true;
            case 1: result = unchecked(lhs - rhs); return true;
            case 2: result = unchecked(lhs * rhs); return true;
            case 3:
                if (rhs == 0) { result = 0; isInvalidValue = true; return false; }
                result = (lhs == int.MinValue && rhs == -1) ? int.MinValue : lhs / rhs;
                return true;
            case 4:
                if (rhs == 0) { result = 0; isInvalidValue = true; return false; }
                result = (lhs == int.MinValue && rhs == -1) ? 0 : lhs % rhs;
                return true;
            case 5: result = lhs & rhs; return true;
            case 6: result = lhs | rhs; return true;
            case 7: result = lhs ^ rhs; return true;
            case 8: result = ~rhs; return true;
            case 9: result = rhs; return true;
            default: result = 0; return false;
        }
    }

    public TmclStatus ExecuteDirectCalc(byte op, int value)
    {
        if (op > 9)
            return TmclStatus.WrongType;
        int rhs = op == 8 ? Accumulator : value;
        if (!TryComputeAluOp(op, Accumulator, rhs, out int res, out bool isInvalid))
            return isInvalid ? TmclStatus.InvalidValue : TmclStatus.WrongType;

        SetAccumulatorWithZeroFlag(res);
        return TmclStatus.SuccessOk;
    }

    public TmclStatus ExecuteDirectCalcX(byte op)
    {
        if (op > 10)
            return TmclStatus.WrongType;

        if (op == 9)
        {
            XRegister = Accumulator;
            return TmclStatus.SuccessOk;
        }
        if (op == 10)
        {
            (Accumulator, XRegister) = (XRegister, Accumulator);
            return TmclStatus.SuccessOk;
        }

        if (!TryComputeAluOp(op, Accumulator, XRegister, out int res, out bool isInvalid))
            return isInvalid ? TmclStatus.InvalidValue : TmclStatus.WrongType;

        SetAccumulatorWithZeroFlag(res);
        return TmclStatus.SuccessOk;
    }

    public TmclStatus ExecuteVarCalc(byte opcode, byte op, byte varIndex, int valOrVar2, bool isDirectMode, out int directReplyVal)
    {
        directReplyVal = 0;
        bool allowSwap = opcode != (byte)TmclCommand.CALCV;
        if (op > 11 || (!allowSwap && op == 10))
            return TmclStatus.WrongType;

        if (opcode == (byte)TmclCommand.CALCVV && (valOrVar2 < 0 || valOrVar2 > 255))
            return TmclStatus.InvalidValue;

        int var1Val = _host.GetGlobalParameterRaw(2, varIndex);
        int lhs;
        int rhs;
        switch ((TmclCommand)opcode)
        {
            case TmclCommand.CALCVV:
                lhs = var1Val;
                rhs = _host.GetGlobalParameterRaw(2, (byte)valOrVar2);
                break;
            case TmclCommand.CALCVA:
                lhs = var1Val;
                rhs = Accumulator;
                break;
            case TmclCommand.CALCAV:
                lhs = Accumulator;
                rhs = var1Val;
                break;
            case TmclCommand.CALCVX:
                lhs = var1Val;
                rhs = XRegister;
                break;
            case TmclCommand.CALCXV:
                lhs = XRegister;
                rhs = var1Val;
                break;
            case TmclCommand.CALCV:
                lhs = var1Val;
                rhs = op == 8 ? var1Val : valOrVar2;
                break;
            default:
                return TmclStatus.InvalidCommand;
        }

        if (op == 11)
        {
            SetComparisonFlags(lhs, rhs);
            return TmclStatus.SuccessOk;
        }

        if (op == 10)
        {
            switch ((TmclCommand)opcode)
            {
                case TmclCommand.CALCVV:
                    byte var2 = (byte)valOrVar2;
                    int var2Val = _host.GetGlobalParameterRaw(2, var2);
                    _host.SetUserVariableRaw(varIndex, var2Val);
                    _host.SetUserVariableRaw(var2, var1Val);
                    break;
                case TmclCommand.CALCVA:
                case TmclCommand.CALCAV:
                    _host.SetUserVariableRaw(varIndex, Accumulator);
                    Accumulator = var1Val;
                    if (opcode == (byte)TmclCommand.CALCAV)
                        directReplyVal = Accumulator;
                    break;
                case TmclCommand.CALCVX:
                case TmclCommand.CALCXV:
                    _host.SetUserVariableRaw(varIndex, XRegister);
                    XRegister = var1Val;
                    if (opcode == (byte)TmclCommand.CALCXV)
                        directReplyVal = XRegister;
                    break;
            }
            return TmclStatus.SuccessOk;
        }

        if (!TryComputeAluOp(op, lhs, rhs, out int result, out bool isInvalid))
            return isInvalid ? TmclStatus.InvalidValue : TmclStatus.WrongType;

        switch ((TmclCommand)opcode)
        {
            case TmclCommand.CALCAV:
                SetAccumulatorWithZeroFlag(result);
                directReplyVal = result;
                break;
            case TmclCommand.CALCXV:
                XRegister = result;
                FlagZero = result == 0;
                directReplyVal = result;
                break;
            default:
                _host.SetUserVariableRaw(varIndex, result);
                FlagZero = result == 0;
                break;
        }

        _ = isDirectMode;
        return TmclStatus.SuccessOk;
    }

    /// <summary>
    /// Advances timers, samples edge interrupts, dispatches pending IRQs, and executes up to
    /// <see cref="MaxInstructionsPerTick"/> instructions when <see cref="Status"/> is <see cref="TmclAppStatus.Run"/>.
    /// </summary>
    public void StepTick(double dtSeconds)
    {
        if (Status != TmclAppStatus.Run)
            return;

        if (dtSeconds < 0)
            dtSeconds = 0;

        double dtMs = dtSeconds * 1000.0;
        if (IsWaiting && !IsInInterrupt)
            _waitElapsedMs += dtMs;

        SampleEdgesAndTimers(dtMs);

        int executed = 0;
        while (executed < MaxInstructionsPerTick && Status == TmclAppStatus.Run)
        {
            if (GlobalInterruptsEnabled && !IsInInterrupt && TryTakeLowestPendingIrq(out int irq))
            {
                EnterInterrupt(irq);
                continue;
            }

            if (IsWaiting)
            {
                if (!TryCompleteActiveWait(checkTimeout: true))
                    break;
                executed++;
                continue;
            }

            if (!_memory.TryRead(ProgramCounter, out var instr) || instr.IsEndMarker)
            {
                Status = TmclAppStatus.Stop;
                ClearWaitState();
                break;
            }

            ExecuteStandaloneInstruction(instr);
            ExecutedInstructionCount++;
            executed++;
        }
    }

    /// <summary>
    /// Executes a single instruction (or evaluates an active wait once) for Command 130 (StepApplication).
    /// </summary>
    public void StepOnce()
    {
        if (IsWaiting)
        {
            TryCompleteActiveWait(checkTimeout: true);
            Status = TmclAppStatus.Step;
            return;
        }

        if (!_memory.TryRead(ProgramCounter, out var instr) || instr.IsEndMarker)
        {
            Status = TmclAppStatus.Stop;
            ClearWaitState();
            return;
        }

        Status = TmclAppStatus.Step;
        ExecuteStandaloneInstruction(instr);
        ExecutedInstructionCount++;
    }

    private void SampleEdgesAndTimers(double dtMs)
    {
        for (int t = 0; t < 3; t++)
        {
            int periodMs = _host.GetGlobalParameterRaw(3, (byte)t);
            if (_interruptEnabled[t] && _interruptVectors[t] >= 0 && periodMs > 0)
            {
                _timerAccumMs[t] += dtMs;
                if (_timerAccumMs[t] + 1e-6 >= periodMs)
                {
                    double remainder = _timerAccumMs[t] - periodMs;
                    _timerAccumMs[t] = remainder > 0 ? remainder % periodMs : 0.0;
                    _pendingInterrupts[t] = true;
                }
            }
            else
            {
                _timerAccumMs[t] = 0;
            }
        }

        int axes = _host.AxisCount;
        for (int axis = 0; axis < axes; axis++)
        {
            bool posReached = _host.GetAxisParameterRaw(axis, TmclAxisParam.TargetPosReached) != 0;
            if (!_prevPosReached[axis] && posReached)
                LatchIfArmed(3 + axis);
            _prevPosReached[axis] = posReached;

            bool stall = _host.GetAxisParameterRaw(axis, TmclAxisParam.ExtendedErrorFlags) != 0;
            if (!_prevStall[axis] && stall)
                LatchIfArmed(15 + axis);
            _prevStall[axis] = stall;

            int leftIrq = 27 + axis * 2;
            bool leftLim = _host.GetAxisParameterRaw(axis, TmclAxisParam.LeftLimitSwitchStatus) != 0;
            if (MatchesEdgeMode(_prevLeftLimit[axis], leftLim, _host.GetGlobalParameterRaw(3, (byte)leftIrq)))
                LatchIfArmed(leftIrq);
            _prevLeftLimit[axis] = leftLim;

            int rightIrq = 28 + axis * 2;
            bool rightLim = _host.GetAxisParameterRaw(axis, TmclAxisParam.RightLimitSwitchStatus) != 0;
            if (MatchesEdgeMode(_prevRightLimit[axis], rightLim, _host.GetGlobalParameterRaw(3, (byte)rightIrq)))
                LatchIfArmed(rightIrq);
            _prevRightLimit[axis] = rightLim;
        }

        for (int port = 0; port < _prevDigitalInputs.Length; port++)
        {
            int inputIrq = 39 + port;
            bool din = _host.GetDigitalInput(port);
            if (MatchesEdgeMode(_prevDigitalInputs[port], din, _host.GetGlobalParameterRaw(3, (byte)inputIrq)))
                LatchIfArmed(inputIrq);
            _prevDigitalInputs[port] = din;
        }
    }

    private static bool MatchesEdgeMode(bool prev, bool curr, int mode) => mode switch
    {
        1 => !prev && curr,
        2 => prev && !curr,
        3 => prev != curr,
        _ => false
    };

    private void LatchIfArmed(int irq)
    {
        if (_interruptEnabled[irq] && _interruptVectors[irq] >= 0)
            _pendingInterrupts[irq] = true;
    }

    private bool TryTakeLowestPendingIrq(out int irq)
    {
        for (int i = 0; i < MaxInterruptCount; i++)
        {
            if (_pendingInterrupts[i] && _interruptEnabled[i] && _interruptVectors[i] >= 0)
            {
                _pendingInterrupts[i] = false;
                irq = i;
                return true;
            }
        }
        irq = -1;
        return false;
    }

    private void EnterInterrupt(int irq)
    {
        _savedContext = new InterruptContext(
            ProgramCounter,
            Accumulator,
            XRegister,
            FlagZero,
            FlagEqual,
            FlagGreater,
            FlagLess,
            IsWaiting,
            _waitType,
            _waitMotor,
            _waitElapsedMs,
            _waitTimeoutMs);

        IsInInterrupt = true;
        ActiveInterruptNumber = irq;
        ClearWaitState();
        ProgramCounter = _interruptVectors[irq];
    }

    private void ReturnFromInterrupt()
    {
        if (!IsInInterrupt)
        {
            ProgramCounter++;
            return;
        }

        ProgramCounter = _savedContext.ProgramCounter;
        Accumulator = _savedContext.Accumulator;
        XRegister = _savedContext.XRegister;
        FlagZero = _savedContext.FlagZero;
        FlagEqual = _savedContext.FlagEqual;
        FlagGreater = _savedContext.FlagGreater;
        FlagLess = _savedContext.FlagLess;
        IsWaiting = _savedContext.IsWaiting;
        _waitType = _savedContext.WaitType;
        _waitMotor = _savedContext.WaitMotor;
        _waitElapsedMs = _savedContext.WaitElapsedMs;
        _waitTimeoutMs = _savedContext.WaitTimeoutMs;
        IsInInterrupt = false;
        ActiveInterruptNumber = -1;
    }

    private void ExecuteStandaloneInstruction(TmclInstruction instr)
    {
        byte opcode = instr.Opcode;
        byte type = instr.Type;
        byte motOrBank = instr.MotorOrBank;
        int val = instr.Value;

        switch ((TmclCommand)opcode)
        {
            case TmclCommand.CALC:
                ExecuteDirectCalc(type, val);
                ProgramCounter++;
                return;

            case TmclCommand.CALCX:
                ExecuteDirectCalcX(type);
                ProgramCounter++;
                return;

            case TmclCommand.CALCVV:
            case TmclCommand.CALCVA:
            case TmclCommand.CALCAV:
            case TmclCommand.CALCVX:
            case TmclCommand.CALCXV:
            case TmclCommand.CALCV:
                ExecuteVarCalc(opcode, type, motOrBank, val, isDirectMode: false, out _);
                ProgramCounter++;
                return;

            case TmclCommand.COMP:
                SetComparisonFlags(Accumulator, val);
                ProgramCounter++;
                return;

            case TmclCommand.JC:
                ProgramCounter = EvaluateCondition(type) ? val : ProgramCounter + 1;
                return;

            case TmclCommand.JA:
                ProgramCounter = val;
                return;

            case TmclCommand.CSUB:
                if (_callStackDepth < MaxCallStackDepth)
                {
                    _callStack[_callStackDepth++] = ProgramCounter + 1;
                    ProgramCounter = val;
                }
                else
                {
                    ProgramCounter++;
                }
                return;

            case TmclCommand.RSUB:
                if (_callStackDepth > 0)
                    ProgramCounter = _callStack[--_callStackDepth];
                else
                    ProgramCounter++;
                return;

            case TmclCommand.CALL:
                if (EvaluateCondition(type) && _callStackDepth < MaxCallStackDepth)
                {
                    _callStack[_callStackDepth++] = ProgramCounter + 1;
                    ProgramCounter = val;
                }
                else
                {
                    ProgramCounter++;
                }
                return;

            case TmclCommand.EI:
                EnableInterrupt(type);
                ProgramCounter++;
                return;

            case TmclCommand.DI:
                DisableInterrupt(type);
                ProgramCounter++;
                return;

            case TmclCommand.VECT:
                if (type < MaxInterruptCount)
                    _interruptVectors[type] = val;
                ProgramCounter++;
                return;

            case TmclCommand.RETI:
                ReturnFromInterrupt();
                return;

            case TmclCommand.WAIT:
                EnterWait(type, motOrBank, val);
                return;

            case TmclCommand.STOP:
                Status = TmclAppStatus.Stop;
                ClearWaitState();
                return;

            case TmclCommand.RST:
                ResetVmExecutionContext(targetAddress: val);
                return;

            case TmclCommand.DJNZ:
            {
                int nextVal = unchecked(_host.GetGlobalParameterRaw(2, type) - 1);
                _host.SetUserVariableRaw(type, nextVal);
                FlagZero = nextVal == 0;
                ProgramCounter = nextVal != 0 ? val : ProgramCounter + 1;
                return;
            }

            default:
                var status = _host.ExecuteHardwareCommand(
                    opcode,
                    type,
                    motOrBank,
                    val,
                    isStandalone: true,
                    out int readResult,
                    out bool hasReadResult);

                if (status == TmclStatus.SuccessOk && hasReadResult)
                    SetAccumulatorWithZeroFlag(readResult);

                // Guard against a standalone command (e.g. SGP 81) clearing and stopping the program mid-tick.
                if (Status != TmclAppStatus.Stop && _memory.TryRead(ProgramCounter, out var current) && current == instr)
                    ProgramCounter++;
                return;
        }
    }

    private void EnterWait(byte waitType, byte motor, int rawTicks)
    {
        if (waitType is >= 1 and <= 4 && motor >= _host.AxisCount)
        {
            ProgramCounter++;
            return;
        }

        if (waitType > 4)
        {
            ProgramCounter++;
            return;
        }

        int ticks = rawTicks == -1 ? Math.Max(0, Accumulator) : Math.Max(0, rawTicks);
        IsWaiting = true;
        _waitType = waitType;
        _waitMotor = motor;
        _waitElapsedMs = 0.0;
        _waitTimeoutMs = ticks * TickDurationMs;

        // Immediately advance if already satisfied at entry (e.g., WAIT TICKS 0 or TargetPosReached == 1).
        TryCompleteActiveWait(checkTimeout: false);
    }

    private bool TryCompleteActiveWait(bool checkTimeout)
    {
        if (IsWaitConditionMet())
        {
            ClearWaitState();
            ProgramCounter++;
            return true;
        }

        if (checkTimeout && _waitType is >= 1 and <= 4 && _waitTimeoutMs > 0 && _waitElapsedMs + 1e-6 >= _waitTimeoutMs)
        {
            ErrorFlags |= 1; // ETO
            _host.SyncErrorFlagsFromVm(ErrorFlags);
            ClearWaitState();
            ProgramCounter++;
            return true;
        }

        return false;
    }

    private bool IsWaitConditionMet() => _waitType switch
    {
        0 => _waitElapsedMs + 1e-6 >= _waitTimeoutMs,
        1 => _waitMotor < _host.AxisCount && _host.GetAxisParameterRaw(_waitMotor, TmclAxisParam.TargetPosReached) != 0,
        2 => _waitMotor < _host.AxisCount && _host.GetAxisParameterRaw(_waitMotor, TmclAxisParam.HomeSwitchState) != 0,
        3 => _waitMotor < _host.AxisCount &&
             (_host.GetAxisParameterRaw(_waitMotor, TmclAxisParam.RightLimitSwitchStatus) != 0 ||
              _host.GetAxisParameterRaw(_waitMotor, TmclAxisParam.LeftLimitSwitchStatus) != 0),
        4 => _waitMotor < _host.AxisCount && _host.GetReferenceSearchStatus(_waitMotor) == 0,
        _ => true
    };

    private void ClearWaitState()
    {
        IsWaiting = false;
        _waitType = 0;
        _waitMotor = 0;
        _waitElapsedMs = 0;
        _waitTimeoutMs = 0;
    }

    private string BuildWaitDescription()
    {
        if (!IsWaiting)
            return string.Empty;

        string timeoutSuffix = _waitTimeoutMs > 0 ? $" ({_waitElapsedMs:0}/{_waitTimeoutMs:0} ms)" : string.Empty;
        return _waitType switch
        {
            0 => $"TICKS {_waitElapsedMs:0}/{_waitTimeoutMs:0} ms",
            1 => $"POS Axis {_waitMotor}{timeoutSuffix}",
            2 => $"REFSW Axis {_waitMotor}{timeoutSuffix}",
            3 => $"LIMSW Axis {_waitMotor}{timeoutSuffix}",
            4 => $"RFS Axis {_waitMotor}{timeoutSuffix}",
            _ => $"Type {_waitType}{timeoutSuffix}"
        };
    }

    public StandaloneVmSnapshot GetSnapshot(int codeProtectionMode, bool autoStartEnabled) => new(
        Status,
        _memory.IsDownloadMode,
        ProgramCounter,
        _memory.WritePointer,
        _memory.InstructionCount,
        Accumulator,
        XRegister,
        ErrorFlags,
        IsWaiting,
        BuildWaitDescription(),
        _callStackDepth,
        GlobalInterruptsEnabled,
        IsInInterrupt,
        ActiveInterruptNumber,
        codeProtectionMode,
        autoStartEnabled,
        ExecutedInstructionCount);
}
