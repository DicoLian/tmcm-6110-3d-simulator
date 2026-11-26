using System.Text;
using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Full TMCM-6110 6-Axis Motion Controller Simulator Engine implementing
/// TMCM-6110 TMCL Firmware Manual V1.47 (Rev V1.23).
/// Processes 9-Byte Binary TMCL Packets, manages Axis 0..5 states, Bank 0/2/3 Global Parameters,
/// Digital/12-bit Analog I/O, Multi-Axis Interpolation (MVP COORD, up to 3 axes per call),
/// Command 138 Async Position Reached Events, and Standalone TMCL Program Memory &amp; VM execution.
/// </summary>
public class Tmcm6110Controller : IMotionController, ITmclVmHost
{
    public const int TotalAxes = 6;
    public const int MaxCoordinateIndex = 20;
    public string ControllerName => "TMCM-6110 (TMCL FW V1.47)";
    public int AxisCount => TotalAxes;

    private readonly object _controllerLock = new();
    private readonly AxisState[] _axes = new AxisState[TotalAxes];
    private readonly Dictionary<(byte Bank, byte Param), int> _globalParams = new();
    private readonly int[] _userVarEeprom = new int[256]; // EEPROM mirror for Bank 2 User Variables
    private readonly int[] _digitalInputs = new int[8];   // Bank 0: AIN0, IN1..IN3, AIN4, IN5..IN7
    private readonly int[] _digitalOutputs = new int[8];  // Bank 2: DOUT0..7 (0/1)
    private int _analogIn0 = 2048;                        // Bank 1 Port 0: AIN0 (12-bit, 0..4095)
    private int _analogIn4 = 2048;                        // Bank 1 Port 4: AIN4 (12-bit, 0..4095)
    private int _supplyVoltageDeciVolts = 240;            // Bank 1 Port 8: Supply Voltage [1/10V] (24.0V)
    private int _stopSwitchPullUpMask = 3;                // SIO 0, 0, <0..3> (default 3 = all pull-ups on)

    // Standalone Program Memory & VM
    private readonly TmclProgramMemory _programMemory = new();
    private readonly TmclStandaloneVm _vm;
    private long _programRevision;
    private int _internalErrorFlags; // Bits: 0=ETO, 1=EAL, 2=EDV, 3=EPO, 4=ESD
    private Random _random = new();

    // Command 138 (Request target position reached event) subscription & pending mask
    private int _event138Mask;
    private bool _event138Continuous;
    private int _event138PendingMask;

    public byte ModuleAddress
    {
        get => (byte)GetGlobalParameter(0, 66);
        set => SetGlobalParameter(0, 66, Math.Clamp((int)value, 1, 255));
    }

    public byte HostAddress
    {
        get => (byte)GetGlobalParameter(0, 76);
        set => SetGlobalParameter(0, 76, value);
    }

    public byte SecondarySerialAddress => (byte)GetGlobalParameter(0, 87);
    public bool SuppressDirectReplies => GetGlobalParameter(0, 255) != 0;

    public long ProgramRevision
    {
        get { lock (_controllerLock) return _programRevision; }
    }

    public int SupplyVoltageDeciVolts
    {
        get { lock (_controllerLock) return _supplyVoltageDeciVolts; }
        set { lock (_controllerLock) _supplyVoltageDeciVolts = Math.Max(0, value); }
    }

    /// <summary>
    /// Speed scaling factor converting TMCL speed units (0..2047) into microsteps/second in the 3D simulator.
    /// Default 15.0 means speed=2000 moves at 30,000 microsteps/sec.
    /// </summary>
    public double SimulationSpeedMultiplier { get; set; } = 15.0;

    /// <summary>
    /// Fired when Command 138 asynchronous second reply (Status = 128) is triggered upon target position reached.
    /// </summary>
    public event Action<TmclReplyPacket>? AsyncEventReplyGenerated;

    /// <summary>
    /// Fired whenever a command packet is processed (for UI Packet Sniffer log).
    /// </summary>
    public event Action<TmclCommandPacket, TmclReplyPacket>? PacketProcessed;

    public Tmcm6110Controller()
    {
        // Default 3-axis gantry mapping:
        // axis 0 (1st Axis) = Z
        // axis 1 (2nd Axis) = Y
        // axis 2 (3rd Axis) = X
        _axes[0] = new AxisState(0, "Z");
        _axes[1] = new AxisState(1, "Y");
        _axes[2] = new AxisState(2, "X");
        _axes[3] = new AxisState(3, "A3 (Unused)");
        _axes[4] = new AxisState(4, "A4 (Unused)");
        _axes[5] = new AxisState(5, "A5 (Unused)");

        _vm = new TmclStandaloneVm(_programMemory, this);
        ResetGlobalParametersToDefaults();
    }

    public AxisState GetAxis(int index)
    {
        if (index < 0 || index >= TotalAxes)
            throw new ArgumentOutOfRangeException(nameof(index));
        return _axes[index];
    }

    public IMotionAxis GetMotionAxis(int index) => GetAxis(index);

    public void StopAll()
    {
        for (int i = 0; i < TotalAxes; i++)
            _axes[i].StopMotor();
    }

    public void HomeAll()
    {
        for (int i = 0; i < TotalAxes; i++)
            _axes[i].StartHoming();
    }

    public void ClearAllFaults()
    {
        for (int i = 0; i < TotalAxes; i++)
            _axes[i].ClearFault();
    }

    public IReadOnlyList<AxisState> Axes => _axes;

    public string GetAxisDisplayLabel(int axisIndex)
    {
        if (axisIndex >= 0 && axisIndex < TotalAxes)
            return $"Axis {axisIndex} ({_axes[axisIndex].AxisName})";
        return $"Axis {axisIndex}";
    }

    public StandaloneVmSnapshot GetStandaloneSnapshot()
    {
        lock (_controllerLock)
        {
            return _vm.GetSnapshot(
                codeProtectionMode: GetGlobalParameterUnsafe(0, 81),
                autoStartEnabled: GetGlobalParameterUnsafe(0, 77) != 0);
        }
    }

    public IReadOnlyList<TmclProgramLine> GetDisassembledProgram()
    {
        lock (_controllerLock)
        {
            return _programMemory.GetDisassembly(_vm.ProgramCounter, GetAxisDisplayLabel);
        }
    }

    public void ClearProgramMemory()
    {
        lock (_controllerLock)
        {
            ClearProgramMemoryUnsafe();
        }
    }

    private void ClearProgramMemoryUnsafe()
    {
        _programMemory.Clear();
        _vm.ResetToStopped();
        _internalErrorFlags = 0;
        _programRevision++;
        SyncVmGlobalParametersUnsafe();
    }

    private void SyncVmGlobalParametersUnsafe()
    {
        _globalParams[(0, 128)] = (int)_vm.Status;
        _globalParams[(0, 129)] = _programMemory.IsDownloadMode ? 1 : 0;
        _globalParams[(0, 130)] = _programMemory.IsDownloadMode ? _programMemory.WritePointer : _vm.ProgramCounter;
    }

    public void ResetGlobalParametersToDefaults()
    {
        lock (_controllerLock)
        {
            _globalParams.Clear();
            // Default TMCM-6110 FW V1.47 Bank 0 global settings (Table 18)
            _globalParams[(0, 65)] = 0;    // RS485 baud rate (0 = 9600)
            _globalParams[(0, 66)] = 1;    // Serial address (1..255, default 1)
            _globalParams[(0, 67)] = 0;    // ASCII mode = 0 (Binary 9-Byte mode)
            _globalParams[(0, 68)] = 0;    // Serial heartbeat (0 = off)
            _globalParams[(0, 69)] = 8;    // CAN Bitrate = 8 (1000 kBit/s)
            _globalParams[(0, 70)] = 2;    // CAN Reply ID = 2
            _globalParams[(0, 71)] = 1;    // CAN ID = 1
            _globalParams[(0, 75)] = 0;    // Telegram pause time = 0 ms
            _globalParams[(0, 76)] = 2;    // Serial host address = 2
            _globalParams[(0, 77)] = 0;    // Auto start mode = 0
            _globalParams[(0, 79)] = 1;    // End switch polarity = 1
            _globalParams[(0, 81)] = 0;    // TMCL code protection = 0
            _globalParams[(0, 82)] = 0;    // CAN heartbeat (0 = off)
            _globalParams[(0, 83)] = 0;    // CAN secondary address = 0
            _globalParams[(0, 84)] = 0;    // Coordinate storage = 0 (RAM only)
            _globalParams[(0, 85)] = 0;    // Do not restore user variables = 0 (restore on startup)
            _globalParams[(0, 87)] = 0;    // Serial secondary address = 0
            _globalParams[(0, 90)] = 0;    // Reverse shaft = 0
            _globalParams[(0, 128)] = 0;   // TMCL application status (0 = stop)
            _globalParams[(0, 129)] = 0;   // Download mode (0 = normal)
            _globalParams[(0, 130)] = 0;   // TMCL program counter = 0
            _globalParams[(0, 132)] = 0;   // TMCL tick timer (ms)
            _globalParams[(0, 133)] = 0;   // Random number seed
            _globalParams[(0, 255)] = 0;   // Suppress reply = 0

            // Initialize Bank 3 Interrupt Parameters (Table 20)
            _globalParams[(3, 0)] = 0;
            _globalParams[(3, 1)] = 0;
            _globalParams[(3, 2)] = 0;
            for (byte p = 27; p <= 46; p++)
                _globalParams[(3, p)] = 0;

            Array.Clear(_userVarEeprom);
            Array.Clear(_digitalInputs);
            Array.Clear(_digitalOutputs);
            _analogIn0 = 2048;
            _analogIn4 = 2048;
            _supplyVoltageDeciVolts = 240;
            _stopSwitchPullUpMask = 3;
            _internalErrorFlags = 0;
            _event138Mask = 0;
            _event138Continuous = false;
            _event138PendingMask = 0;

            _vm.ResetToStopped();
            SyncVmGlobalParametersUnsafe();
        }
    }

    public int GetGlobalParameter(byte bank, byte param)
    {
        lock (_controllerLock)
        {
            return GetGlobalParameterUnsafe(bank, param);
        }
    }

    public void SetGlobalParameter(byte bank, byte param, int value)
    {
        lock (_controllerLock)
        {
            if (bank == 0 && param == 133)
                _random = new Random(value);

            if (bank == 0 && param == 81 && value is >= 0 and <= 3)
                HandleCodeProtectionChangeUnsafe(value);

            _globalParams[(bank, param)] = value;
        }
    }

    public bool GetDigitalOutput(int port)
    {
        lock (_controllerLock)
            return port >= 0 && port < _digitalOutputs.Length && _digitalOutputs[port] != 0;
    }

    public void SetDigitalOutput(int port, bool state)
    {
        lock (_controllerLock)
        {
            if (port >= 0 && port < _digitalOutputs.Length)
                _digitalOutputs[port] = state ? 1 : 0;
        }
    }

    public bool GetDigitalInput(int port)
    {
        lock (_controllerLock)
            return port >= 0 && port < _digitalInputs.Length && _digitalInputs[port] != 0;
    }

    public void SetDigitalInput(int port, bool state)
    {
        lock (_controllerLock)
        {
            if (port >= 0 && port < _digitalInputs.Length)
                _digitalInputs[port] = state ? 1 : 0;
        }
    }

    public int GetAnalogInput(int port)
    {
        lock (_controllerLock)
        {
            return port switch
            {
                0 => _analogIn0,
                4 => _analogIn4,
                8 => _supplyVoltageDeciVolts,
                _ => 0
            };
        }
    }

    public void SetAnalogInput(int port, int value)
    {
        lock (_controllerLock)
        {
            switch (port)
            {
                case 0:
                    _analogIn0 = Math.Clamp(value, 0, 4095);
                    break;
                case 4:
                    _analogIn4 = Math.Clamp(value, 0, 4095);
                    break;
                case 8:
                    _supplyVoltageDeciVolts = Math.Max(0, value);
                    break;
            }
        }
    }

    /// <summary>
    /// Returns true if the reply for <paramref name="cmd"/> should be suppressed on Serial/USB
    /// (Cmd 137, Secondary Serial Address #87, or Suppress Reply #255 for non-read commands).
    /// </summary>
    public bool ShouldSuppressReply(TmclCommandPacket cmd)
    {
        if (cmd.CommandEnum == TmclCommand.RestoreFactorySettings)
            return true;

        lock (_controllerLock)
        {
            byte secAddr = _globalParams.TryGetValue((0, 87), out int s) ? (byte)s : (byte)0;
            byte modAddr = _globalParams.TryGetValue((0, 66), out int m) ? (byte)m : (byte)1;
            if (secAddr != 0 && cmd.ModuleAddress == secAddr && secAddr != modAddr)
                return true;

            bool suppress = _globalParams.TryGetValue((0, 255), out int sup) && sup != 0;
            if (suppress && cmd.CommandEnum is not (TmclCommand.GAP or TmclCommand.GGP or TmclCommand.GIO))
                return true;

            return false;
        }
    }

    /// <summary>
    /// Advances physics for all axes, emits Command 138 Position Reached Events when armed targets arrive,
    /// and steps the standalone TMCL VM &amp; interrupt controller.
    /// </summary>
    public void StepSimulation(double dtSeconds)
    {
        TmclReplyPacket? asyncEventPacket = null;

        lock (_controllerLock)
        {
            // Increment millisecond tick timer (Global Param #132)
            _globalParams.TryGetValue(((byte)0, (byte)132), out int currentTick);
            _globalParams[((byte)0, (byte)132)] = unchecked((currentTick + (int)(dtSeconds * 1000.0)) & int.MaxValue);

            for (int i = 0; i < TotalAxes; i++)
            {
                _axes[i].StepPhysics(dtSeconds, SimulationSpeedMultiplier);
            }

            // Check if Command 138 (Request target position reached event) has pending axes
            if (_event138PendingMask != 0)
            {
                int reachedNow = 0;
                for (int i = 0; i < TotalAxes; i++)
                {
                    if ((_event138PendingMask & (1 << i)) != 0 &&
                        _axes[i].GetParameter(TmclAxisParam.TargetPosReached, clearReadFlags: false) != 0)
                    {
                        reachedNow |= (1 << i);
                    }
                }

                if (reachedNow != 0)
                {
                    _event138PendingMask &= ~reachedNow;
                    asyncEventPacket = new TmclReplyPacket(
                        HostAddress,
                        ModuleAddress,
                        TmclStatus.TargetPositionReachedEvent,
                        (byte)TmclCommand.RequestTargetPositionReachedEvent,
                        reachedNow);
                }
            }

            _vm.StepTick(dtSeconds);
            SyncVmGlobalParametersUnsafe();
        }

        if (asyncEventPacket.HasValue)
        {
            AsyncEventReplyGenerated?.Invoke(asyncEventPacket.Value);
        }
    }

    /// <summary>
    /// Processes an incoming 9-Byte Binary TMCL Command Packet and returns the 9-Byte Binary Reply Packet.
    /// </summary>
    public TmclReplyPacket ProcessCommand(TmclCommandPacket cmd)
    {
        TmclReplyPacket reply;

        lock (_controllerLock)
        {
            if (!cmd.IsChecksumValid)
            {
                reply = new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongChecksum, cmd.Command, 0);
            }
            else
            {
                reply = ExecuteValidCommandUnsafe(cmd);
            }
        }

        PacketProcessed?.Invoke(cmd, reply);
        return reply;
    }

    private TmclReplyPacket ExecuteValidCommandUnsafe(TmclCommandPacket cmd)
    {
        byte replyAddr = HostAddress;
        byte modAddr = ModuleAddress;
        byte opcode = cmd.Command;
        byte type = cmd.TypeNumber;
        byte motOrBank = cmd.MotorOrBank;
        int val = cmd.Value;

        // FW V1.47 Section 3.7.60: In download mode, non-control commands (< 128) are stored in EEPROM, not executed.
        if (_programMemory.IsDownloadMode && opcode < 128)
        {
            var instr = new TmclInstruction(opcode, type, motOrBank, val);
            if (!_programMemory.TryWriteNext(instr, out int writtenAddr))
                return Reply(TmclStatus.InvalidValue, opcode, 0);

            _programRevision++;
            SyncVmGlobalParametersUnsafe();
            return Reply(TmclStatus.CommandLoadedIntoEeprom, opcode, writtenAddr);
        }

        switch (cmd.CommandEnum)
        {
            case TmclCommand.CALC:
            {
                var st = _vm.ExecuteDirectCalc(type, val);
                return Reply(st, opcode, st == TmclStatus.SuccessOk ? val : 0);
            }

            case TmclCommand.CALCX:
            {
                var st = _vm.ExecuteDirectCalcX(type);
                return Reply(st, opcode, st == TmclStatus.SuccessOk ? _vm.Accumulator : 0);
            }

            case TmclCommand.CALCVV:
            case TmclCommand.CALCVA:
            case TmclCommand.CALCAV:
            case TmclCommand.CALCVX:
            case TmclCommand.CALCXV:
            case TmclCommand.CALCV:
            {
                var st = _vm.ExecuteVarCalc(opcode, type, motOrBank, val, isDirectMode: true, out int calcReplyVal);
                return Reply(st, opcode, st == TmclStatus.SuccessOk ? calcReplyVal : 0);
            }

            case TmclCommand.EI:
                if (type != 255 && type >= TmclStandaloneVm.MaxInterruptCount)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _vm.EnableInterrupt(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.DI:
                if (type != 255 && type >= TmclStandaloneVm.MaxInterruptCount)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _vm.DisableInterrupt(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            // Standalone-only flow / interrupt vector opcodes
            case TmclCommand.COMP:
            case TmclCommand.JC:
            case TmclCommand.JA:
            case TmclCommand.CSUB:
            case TmclCommand.RSUB:
            case TmclCommand.WAIT:
            case TmclCommand.STOP:
            case TmclCommand.VECT:
            case TmclCommand.RETI:
            case TmclCommand.RST:
            case TmclCommand.DJNZ:
            case TmclCommand.CALL:
                return Reply(TmclStatus.CommandNotAvailable, opcode, 0);

            // Control commands (128..138, 255)
            case TmclCommand.StopApplication: // Cmd 128
                _vm.Stop();
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RunApplication: // Cmd 129
                if (type > 1)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                if (type == 1 && (val < 0 || val >= _programMemory.Capacity))
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _vm.Run(type == 1 ? val : null);
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.StepApplication: // Cmd 130
                if (_programMemory.IsDownloadMode)
                    return Reply(TmclStatus.CommandNotAvailable, opcode, 0);
                _vm.StepOnce();
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.ResetApplication: // Cmd 131
                _vm.ResetApplication();
                _internalErrorFlags = 0;
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.StartDownload: // Cmd 132
                if ((GetGlobalParameterUnsafe(0, 81) & 2) != 0)
                    return Reply(TmclStatus.ConfigurationEepromLocked, opcode, 0);
                if (val < 0 || val >= _programMemory.Capacity)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (_vm.Status == TmclAppStatus.Run)
                    _vm.Stop();
                _programMemory.StartDownload(val);
                if (val == 0)
                    _programRevision++;
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.QuitDownload: // Cmd 133
                _programMemory.QuitDownload();
                _vm.ProgramCounter = 0;
                SyncVmGlobalParametersUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.ReadTmclMemory: // Cmd 134
                if ((GetGlobalParameterUnsafe(0, 81) & 1) != 0)
                    return Reply(TmclStatus.ConfigurationEepromLocked, opcode, 0);
                if (!_programMemory.TryRead(val, out var slotInstr))
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                return TmclReplyPacket.CreateMemoryReadReply(
                    replyAddr,
                    modAddr,
                    slotInstr.Opcode,
                    slotInstr.Type,
                    slotInstr.MotorOrBank,
                    slotInstr.Value,
                    GetAxisDisplayLabel);

            case TmclCommand.GetAppStatus: // Cmd 135
                return type switch
                {
                    0 => Reply(TmclStatus.SuccessOk, opcode, (int)_vm.Status),
                    1 => Reply(TmclStatus.SuccessOk, opcode, _vm.ProgramCounter),
                    2 => Reply(TmclStatus.SuccessOk, opcode, _vm.Accumulator),
                    3 => Reply(TmclStatus.SuccessOk, opcode, _vm.XRegister),
                    _ => Reply(TmclStatus.WrongType, opcode, 0)
                };

            case TmclCommand.GetFirmwareVersion: // Cmd 136
                if (type == 0)
                {
                    // String format: [0] HostAddr, [1..8] "6110V147" (8 ASCII chars, no checksum!)
                    byte[] raw = new byte[9];
                    raw[0] = replyAddr;
                    byte[] ascii = Encoding.ASCII.GetBytes("6110V147");
                    Array.Copy(ascii, 0, raw, 1, 8);
                    return new TmclReplyPacket(raw);
                }
                if (type == 1)
                {
                    // Binary format: Module 6110 (0x17DE), Firmware V1.47 (0x012F) -> 0x17DE012F
                    return Reply(TmclStatus.SuccessOk, opcode, 0x17DE012F);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.RestoreFactorySettings: // Cmd 137
                if (val == 1234)
                {
                    _programMemory.Clear();
                    _programRevision++;
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].ResetToDefaults(i);
                    ResetGlobalParametersToDefaults();
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                return Reply(TmclStatus.InvalidValue, opcode, 0);

            case TmclCommand.RequestTargetPositionReachedEvent: // Cmd 138
                if (type > 1)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _event138Mask = val & 0x3F;
                _event138Continuous = type == 1;
                if (_event138Mask == 0)
                    _event138PendingMask = 0;
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.SoftwareReset: // Cmd 255
                if (val != 1234)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                PerformSoftwareResetUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            default:
            {
                var hwStatus = ExecuteHardwareCommandUnsafe(
                    opcode,
                    type,
                    motOrBank,
                    val,
                    isStandalone: false,
                    out int directReplyVal,
                    out _,
                    out _);
                return Reply(hwStatus, opcode, hwStatus == TmclStatus.SuccessOk ? directReplyVal : 0);
            }
        }

        TmclReplyPacket Reply(TmclStatus st, byte op, int v) =>
            new(replyAddr, modAddr, st, op, v);
    }

    private TmclStatus ExecuteHardwareCommandUnsafe(
        byte opcode,
        byte type,
        byte motOrBank,
        int val,
        bool isStandalone,
        out int directReplyVal,
        out int readResult,
        out bool hasReadResult)
    {
        directReplyVal = 0;
        readResult = 0;
        hasReadResult = false;
        int accu = _vm.Accumulator;
        int xReg = _vm.XRegister;

        switch ((TmclCommand)opcode)
        {
            case TmclCommand.ROR:
                if (motOrBank >= TotalAxes || val < -2047 || val > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateRight(val));
                directReplyVal = val;
                return TmclStatus.SuccessOk;

            case TmclCommand.ROL:
                if (motOrBank >= TotalAxes || val < -2047 || val > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateLeft(val));
                directReplyVal = val;
                return TmclStatus.SuccessOk;

            case TmclCommand.RORA:
                if (motOrBank >= TotalAxes || accu < -2047 || accu > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateRight(accu));
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.ROLA:
                if (motOrBank >= TotalAxes || accu < -2047 || accu > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateLeft(accu));
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.RORXA:
                if (xReg < 0 || xReg >= TotalAxes || accu < -2047 || accu > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(xReg, ax => ax.RotateRight(accu));
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.ROLXA:
                if (xReg < 0 || xReg >= TotalAxes || accu < -2047 || accu > 2047)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(xReg, ax => ax.RotateLeft(accu));
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.MST:
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(motOrBank, ax => ax.StopMotor());
                return TmclStatus.SuccessOk;

            case TmclCommand.MSTX:
                if (xReg < 0 || xReg >= TotalAxes)
                    return TmclStatus.InvalidValue;
                ExecuteWithGroupIndex(xReg, ax => ax.StopMotor());
                return TmclStatus.SuccessOk;

            case TmclCommand.MVP:
                return HandleMvpCommandUnsafe(type, motOrBank, val, out directReplyVal);

            case TmclCommand.MVPA:
                return HandleMvpCommandUnsafe(type, motOrBank, accu, out directReplyVal);

            case TmclCommand.MVPXA:
                if (xReg < 0 || xReg > 255)
                    return TmclStatus.InvalidValue;
                return HandleMvpCommandUnsafe(type, (byte)xReg, accu, out directReplyVal);

            case TmclCommand.SAP:
                return SetAxisParamChecked(motOrBank, type, val, out directReplyVal);

            case TmclCommand.SAPX:
                if (xReg < 0 || xReg >= TotalAxes)
                    return TmclStatus.InvalidValue;
                return SetAxisParamChecked(xReg, type, val, out directReplyVal);

            case TmclCommand.AAP:
                return SetAxisParamChecked(motOrBank, type, accu, out directReplyVal);

            case TmclCommand.AAPX:
                if (xReg < 0 || xReg >= TotalAxes)
                    return TmclStatus.InvalidValue;
                return SetAxisParamChecked(xReg, type, accu, out directReplyVal);

            case TmclCommand.GAP:
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                if (!TmclAxisParam.IsValidParameter(type))
                    return TmclStatus.WrongType;
                readResult = _axes[motOrBank].GetParameter(type);
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;

            case TmclCommand.GAPX:
                if (xReg < 0 || xReg >= TotalAxes)
                    return TmclStatus.InvalidValue;
                if (!TmclAxisParam.IsValidParameter(type))
                    return TmclStatus.WrongType;
                readResult = _axes[xReg].GetParameter(type);
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;

            case TmclCommand.STAP:
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                if (!TmclAxisParam.IsValidParameter(type) || TmclAxisParam.IsReadOnly(type))
                    return TmclStatus.WrongType;
                _axes[motOrBank].StoreParameterToEeprom(type);
                return TmclStatus.SuccessOk;

            case TmclCommand.RSAP:
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                if (!TmclAxisParam.IsValidParameter(type) || TmclAxisParam.IsReadOnly(type))
                    return TmclStatus.WrongType;
                _axes[motOrBank].RestoreParameterFromEeprom(type);
                return TmclStatus.SuccessOk;

            case TmclCommand.SGP:
            {
                var st = HandleSgpCommandUnsafe(type, motOrBank, val);
                directReplyVal = val;
                return st;
            }

            case TmclCommand.AGP:
            {
                var st = HandleSgpCommandUnsafe(type, motOrBank, accu);
                directReplyVal = accu;
                return st;
            }

            case TmclCommand.GGP:
            {
                var st = HandleGgpCommandUnsafe(type, motOrBank, out readResult);
                if (st == TmclStatus.SuccessOk)
                {
                    hasReadResult = true;
                    directReplyVal = readResult;
                }
                return st;
            }

            case TmclCommand.STGP:
                if (motOrBank != 2)
                    return TmclStatus.InvalidValue;
                _userVarEeprom[type] = _globalParams.TryGetValue((2, type), out int uv) ? uv : 0;
                return TmclStatus.SuccessOk;

            case TmclCommand.RSGP:
                if (motOrBank != 2)
                    return TmclStatus.InvalidValue;
                _globalParams[(2, type)] = _userVarEeprom[type];
                return TmclStatus.SuccessOk;

            case TmclCommand.SIV:
                if (xReg < 0 || xReg > 255)
                    return TmclStatus.InvalidValue;
                _globalParams[(2, (byte)xReg)] = val;
                directReplyVal = val;
                return TmclStatus.SuccessOk;

            case TmclCommand.GIV:
                if (xReg < 0 || xReg > 255)
                    return TmclStatus.InvalidValue;
                readResult = GetGlobalParameterUnsafe(2, (byte)xReg);
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;

            case TmclCommand.AIV:
                if (xReg < 0 || xReg > 255)
                    return TmclStatus.InvalidValue;
                _globalParams[(2, (byte)xReg)] = accu;
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.RFS:
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                if (type == 0)
                {
                    ExecuteWithGroupIndex(motOrBank, ax => ax.StartReferenceSearch());
                    return TmclStatus.SuccessOk;
                }
                if (type == 1)
                {
                    ExecuteWithGroupIndex(motOrBank, ax => ax.StopReferenceSearch());
                    return TmclStatus.SuccessOk;
                }
                if (type == 2)
                {
                    readResult = _axes[motOrBank].GetReferenceSearchStatus();
                    hasReadResult = true;
                    directReplyVal = readResult;
                    return TmclStatus.SuccessOk;
                }
                return TmclStatus.WrongType;

            case TmclCommand.SIO:
                if (motOrBank == 2)
                {
                    if (type == 255)
                    {
                        int outMask = val == -1 ? (accu & 0xFF) : val;
                        if (outMask < 0 || outMask > 255)
                            return TmclStatus.InvalidValue;
                        for (int i = 0; i < 8; i++)
                            _digitalOutputs[i] = (outMask & (1 << i)) != 0 ? 1 : 0;
                        directReplyVal = val;
                        return TmclStatus.SuccessOk;
                    }
                    if (type < 8)
                    {
                        if (val != 0 && val != 1)
                            return TmclStatus.InvalidValue;
                        _digitalOutputs[type] = val;
                        directReplyVal = val;
                        return TmclStatus.SuccessOk;
                    }
                    return TmclStatus.WrongType;
                }
                if (motOrBank == 0 && type == 0)
                {
                    if (val < 0 || val > 3)
                        return TmclStatus.InvalidValue;
                    _stopSwitchPullUpMask = val;
                    directReplyVal = val;
                    return TmclStatus.SuccessOk;
                }
                return TmclStatus.WrongType;

            case TmclCommand.GIO:
                return HandleGioCommandUnsafe(type, motOrBank, out readResult, out hasReadResult, out directReplyVal);

            case TmclCommand.SCO:
                if (type > MaxCoordinateIndex)
                    return TmclStatus.InvalidValue;
                if (motOrBank == 255)
                {
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].CopyCoordinateRamToEeprom(type);
                    return TmclStatus.SuccessOk;
                }
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                bool storeCoordEeprom = GetGlobalParameterUnsafe(0, 84) != 0;
                _axes[motOrBank].SetCoordinate(type, val, storeCoordEeprom);
                directReplyVal = val;
                return TmclStatus.SuccessOk;

            case TmclCommand.GCO:
                if (type > MaxCoordinateIndex)
                    return TmclStatus.InvalidValue;
                if (motOrBank == 255)
                {
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].CopyCoordinateEepromToRam(type);
                    return TmclStatus.SuccessOk;
                }
                if (motOrBank >= TotalAxes)
                    return TmclStatus.InvalidValue;
                readResult = _axes[motOrBank].GetCoordinate(type);
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;

            case TmclCommand.CCO:
                if (motOrBank >= TotalAxes || type > MaxCoordinateIndex)
                    return TmclStatus.InvalidValue;
                bool storeCcoEeprom = GetGlobalParameterUnsafe(0, 84) != 0;
                int captured = _axes[motOrBank].CaptureCoordinate(type, storeCcoEeprom);
                directReplyVal = captured;
                return TmclStatus.SuccessOk;

            case TmclCommand.ACO:
                if (motOrBank >= TotalAxes || type > MaxCoordinateIndex)
                    return TmclStatus.InvalidValue;
                bool storeAcoEeprom = GetGlobalParameterUnsafe(0, 84) != 0;
                _axes[motOrBank].SetCoordinate(type, accu, storeAcoEeprom);
                directReplyVal = accu;
                return TmclStatus.SuccessOk;

            case TmclCommand.CLE:
                if (type > 5)
                    return TmclStatus.WrongType;
                if (type == 0)
                    _internalErrorFlags = 0;
                else
                    _internalErrorFlags &= ~(1 << (type - 1));
                _vm.ClearErrorFlag(type);
                return TmclStatus.SuccessOk;

            default:
                _ = isStandalone;
                return TmclStatus.InvalidCommand;
        }
    }

    private TmclStatus SetAxisParamChecked(int axisIndex, byte param, int value, out int replyVal)
    {
        replyVal = 0;
        if (axisIndex < 0 || axisIndex >= TotalAxes)
            return TmclStatus.InvalidValue;
        if (!TmclAxisParam.IsValidParameter(param) || TmclAxisParam.IsReadOnly(param))
            return TmclStatus.WrongType;
        if (!TmclAxisParam.IsValueValid(param, value))
            return TmclStatus.InvalidValue;

        _axes[axisIndex].SetParameter(param, value);
        replyVal = value;
        return TmclStatus.SuccessOk;
    }

    private TmclStatus HandleGioCommandUnsafe(
        byte port,
        byte bank,
        out int readResult,
        out bool hasReadResult,
        out int directReplyVal)
    {
        readResult = 0;
        hasReadResult = false;
        directReplyVal = 0;

        if (bank == 0)
        {
            if (port == 255)
            {
                int mask = 0;
                for (int i = 0; i < 8; i++)
                    if (_digitalInputs[i] != 0) mask |= (1 << i);
                readResult = mask;
                hasReadResult = true;
                directReplyVal = mask;
                return TmclStatus.SuccessOk;
            }
            if (port < _digitalInputs.Length)
            {
                readResult = _digitalInputs[port];
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;
            }
            return TmclStatus.WrongType;
        }

        if (bank == 1)
        {
            if (port == 0) readResult = _analogIn0;
            else if (port == 4) readResult = _analogIn4;
            else if (port == 8) readResult = _supplyVoltageDeciVolts;
            else return TmclStatus.WrongType;

            hasReadResult = true;
            directReplyVal = readResult;
            return TmclStatus.SuccessOk;
        }

        if (bank == 2)
        {
            if (port == 255)
            {
                int mask = 0;
                for (int i = 0; i < 8; i++)
                    if (_digitalOutputs[i] != 0) mask |= (1 << i);
                readResult = mask;
                hasReadResult = true;
                directReplyVal = mask;
                return TmclStatus.SuccessOk;
            }
            if (port < _digitalOutputs.Length)
            {
                readResult = _digitalOutputs[port];
                hasReadResult = true;
                directReplyVal = readResult;
                return TmclStatus.SuccessOk;
            }
            return TmclStatus.WrongType;
        }

        return TmclStatus.WrongType;
    }

    private int GetGlobalParameterUnsafe(byte bank, byte param)
    {
        if (bank == 0)
        {
            if (param == 128) return (int)_vm.Status;
            if (param == 129) return _programMemory.IsDownloadMode ? 1 : 0;
            if (param == 130) return _programMemory.IsDownloadMode ? _programMemory.WritePointer : _vm.ProgramCounter;
            if (param == 133) return _random.Next(0, int.MaxValue);
        }
        return _globalParams.TryGetValue((bank, param), out int val) ? val : 0;
    }

    private void HandleCodeProtectionChangeUnsafe(int newValue)
    {
        int oldVal = GetGlobalParameterUnsafe(0, 81);
        bool wasReadProtected = (oldVal & 1) != 0;   // 1 or 3
        bool willBeReadProtected = (newValue & 1) != 0; // 0 or 2 => false
        if (wasReadProtected && !willBeReadProtected)
        {
            ClearProgramMemoryUnsafe();
        }
    }

    private void PerformSoftwareResetUnsafe()
    {
        bool restoreCoords = GetGlobalParameterUnsafe(0, 84) != 0;
        for (int i = 0; i < TotalAxes; i++)
            _axes[i].RestoreAllFromEeprom(restoreCoords);

        bool doNotRestoreVars = GetGlobalParameterUnsafe(0, 85) != 0;
        for (int i = 0; i < 256; i++)
            _globalParams[(2, (byte)i)] = doNotRestoreVars ? 0 : _userVarEeprom[i];

        // Preserve _programMemory across power-cycle reset (Cmd 255), exiting RAM download mode if active.
        _programMemory.QuitDownload();
        bool autoStart = GetGlobalParameterUnsafe(0, 77) != 0 && _programMemory.InstructionCount > 0;
        _vm.ResetForPowerCycle(autoStart);

        _globalParams[(0, 132)] = 0;
        _globalParams[(0, 255)] = 0;
        _internalErrorFlags = 0;
        _event138Mask = 0;
        _event138Continuous = false;
        _event138PendingMask = 0;

        SyncVmGlobalParametersUnsafe();
    }

    private TmclStatus HandleSgpCommandUnsafe(byte type, byte bank, int val)
    {
        if (bank == 0)
        {
            if (!IsValidBank0GlobalParam(type) || IsReadOnlyBank0GlobalParam(type))
                return TmclStatus.WrongType;
            if (!IsBank0ValueValid(type, val))
                return TmclStatus.InvalidValue;

            if (type == 133)
                _random = new Random(val);
            if (type == 81)
                HandleCodeProtectionChangeUnsafe(val);

            _globalParams[(0, type)] = val;
            return TmclStatus.SuccessOk;
        }
        if (bank == 2)
        {
            _globalParams[(2, type)] = val;
            return TmclStatus.SuccessOk;
        }
        if (bank == 3)
        {
            if (type <= 2)
            {
                _globalParams[(3, type)] = val;
                return TmclStatus.SuccessOk;
            }
            if (type is >= 27 and <= 46)
            {
                if (val < 0 || val > 3)
                    return TmclStatus.InvalidValue;
                _globalParams[(3, type)] = val;
                return TmclStatus.SuccessOk;
            }
            return TmclStatus.WrongType;
        }

        return TmclStatus.InvalidValue;
    }

    private TmclStatus HandleGgpCommandUnsafe(byte type, byte bank, out int val)
    {
        val = 0;
        if (bank == 0)
        {
            if (!IsValidBank0GlobalParam(type))
                return TmclStatus.WrongType;
            val = GetGlobalParameterUnsafe(0, type);
            return TmclStatus.SuccessOk;
        }
        if (bank == 2)
        {
            val = GetGlobalParameterUnsafe(2, type);
            return TmclStatus.SuccessOk;
        }
        if (bank == 3)
        {
            if (type <= 2 || type is >= 27 and <= 46)
            {
                val = GetGlobalParameterUnsafe(3, type);
                return TmclStatus.SuccessOk;
            }
            return TmclStatus.WrongType;
        }

        return TmclStatus.InvalidValue;
    }

    private static bool IsValidBank0GlobalParam(byte param) => param switch
    {
        65 or 66 or 67 or 68 or 69 or 70 or 71 or 75 or 76 or 77 or 79 or 81 or 82 or 83 or 84 or 85 or 87 or 90
            or 128 or 129 or 130 or 132 or 133 or 255 => true,
        _ => false
    };

    private static bool IsReadOnlyBank0GlobalParam(byte param) => param is 128 or 129 or 130;

    private static bool IsBank0ValueValid(byte param, int value) => param switch
    {
        65 => value is >= 0 and <= 8,
        66 => value is >= 1 and <= 255,
        67 => value is >= 0 and <= 63,
        68 or 82 => value is >= 0 and <= 65535,
        69 => value is >= 2 and <= 8,
        70 or 71 or 83 => value is >= 0 and <= 2047,
        75 or 76 or 87 => value is >= 0 and <= 255,
        77 or 84 or 85 or 255 => value is 0 or 1,
        79 or 81 or 90 => value is >= 0 and <= 3,
        132 or 133 => value >= 0,
        _ => false
    };

    private void ArmEvent138ForAxesUnsafe(int movedAxesMask)
    {
        int matched = movedAxesMask & _event138Mask;
        if (matched == 0) return;

        _event138PendingMask |= matched;
        if (!_event138Continuous)
            _event138Mask = 0;
    }

    private TmclStatus HandleMvpCommandUnsafe(byte type, byte motOrBank, int val, out int directReplyVal)
    {
        directReplyVal = 0;

        if (type == 0) // MVP ABS
        {
            if (motOrBank >= TotalAxes)
                return TmclStatus.InvalidValue;

            int movedMask = ExecuteWithGroupIndex(motOrBank, ax => ax.MoveToAbsolute(val));
            ArmEvent138ForAxesUnsafe(movedMask);
            directReplyVal = val;
            return TmclStatus.SuccessOk;
        }
        else if (type == 1) // MVP REL
        {
            if (motOrBank >= TotalAxes)
                return TmclStatus.InvalidValue;

            int movedMask = ExecuteWithGroupIndex(motOrBank, ax => ax.MoveRelative(val));
            ArmEvent138ForAxesUnsafe(movedMask);
            directReplyVal = val;
            return TmclStatus.SuccessOk;
        }
        else if (type == 2) // MVP COORD
        {
            if (val < 0 || val > MaxCoordinateIndex)
                return TmclStatus.InvalidValue;

            // TMCM-6110 FW 1.47 Section 3.7.4:
            // Bit 6 (0x40) = Linear interpolation (max 3 motors; if >3 bits set, only first 3 are used).
            // Bit 7 (0x80) = Move selected motors without interpolation (unlimited).
            if ((motOrBank & 0xC0) != 0)
            {
                bool interpolate = (motOrBank & 0x40) != 0;
                int mask = motOrBank & 0x3F;
                var selectedAxes = new List<AxisState>();
                for (int i = 0; i < TotalAxes; i++)
                {
                    if ((mask & (1 << i)) != 0)
                        selectedAxes.Add(_axes[i]);
                }

                if (interpolate && selectedAxes.Count > 3)
                    selectedAxes = selectedAxes.GetRange(0, 3);

                int startedMask = 0;
                if (selectedAxes.Count > 0)
                {
                    if (interpolate && selectedAxes.Count > 1)
                    {
                        double maxTimeRatio = 0;
                        double minAccelOverDist = double.MaxValue;
                        foreach (var ax in selectedAxes)
                        {
                            double dist = Math.Abs((double)ax.GetCoordinate(val) - ax.GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
                            if (dist < 0.5) continue;
                            double maxSpd = Math.Max(1, ax.GetParameter(TmclAxisParam.MaxPositioningSpeed, clearReadFlags: false));
                            double maxAcc = Math.Max(1, ax.GetParameter(TmclAxisParam.MaxAcceleration, clearReadFlags: false));
                            double ratio = dist / maxSpd;
                            if (ratio > maxTimeRatio) maxTimeRatio = ratio;
                            double accRatio = maxAcc / dist;
                            if (accRatio < minAccelOverDist) minAccelOverDist = accRatio;
                        }

                        foreach (var ax in selectedAxes)
                        {
                            int targetCoord = ax.GetCoordinate(val);
                            double dist = Math.Abs((double)targetCoord - ax.GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
                            if (dist >= 0.5 && maxTimeRatio > 1e-6)
                            {
                                double interpSpeed = Math.Max(0.01, dist / maxTimeRatio);
                                double interpAccel = minAccelOverDist < double.MaxValue
                                    ? Math.Max(0.01, dist * minAccelOverDist)
                                    : ax.GetParameter(TmclAxisParam.MaxAcceleration, clearReadFlags: false);
                                ax.MoveToAbsolute(targetCoord, interpSpeed, interpAccel);
                            }
                            else
                            {
                                ax.MoveToAbsolute(targetCoord);
                            }
                            startedMask |= (1 << ax.AxisIndex);
                        }
                    }
                    else
                    {
                        foreach (var ax in selectedAxes)
                        {
                            ax.MoveToAbsolute(ax.GetCoordinate(val));
                            startedMask |= (1 << ax.AxisIndex);
                        }
                    }
                }
                ArmEvent138ForAxesUnsafe(startedMask);
                directReplyVal = val;
                return TmclStatus.SuccessOk;
            }
            else if (motOrBank < TotalAxes)
            {
                // Single axis MVP COORD, <axis>, <coord_index>
                int targetCoord = _axes[motOrBank].GetCoordinate(val);
                _axes[motOrBank].MoveToAbsolute(targetCoord);
                ArmEvent138ForAxesUnsafe(1 << motOrBank);
                directReplyVal = val;
                return TmclStatus.SuccessOk;
            }
            return TmclStatus.InvalidValue;
        }

        return TmclStatus.WrongType;
    }

    private int ExecuteWithGroupIndex(int targetAxisIndex, Action<AxisState> action)
    {
        var primary = _axes[targetAxisIndex];
        action(primary);
        int affectedMask = 1 << targetAxisIndex;

        // TMCM-6110 Parameter #213 Group Index: if > 0, all axes with the same group index execute the command
        int groupIdx = primary.GetParameter(TmclAxisParam.GroupIndex, clearReadFlags: false);
        if (groupIdx > 0)
        {
            for (int i = 0; i < TotalAxes; i++)
            {
                if (i != targetAxisIndex && _axes[i].GetParameter(TmclAxisParam.GroupIndex, clearReadFlags: false) == groupIdx)
                {
                    action(_axes[i]);
                    affectedMask |= (1 << i);
                }
            }
        }
        return affectedMask;
    }

    // Explicit ITmclVmHost implementation (invoked only under _controllerLock)
    int ITmclVmHost.AxisCount => TotalAxes;
    int ITmclVmHost.GetAxisParameterRaw(int axis, byte param) => _axes[axis].GetParameter(param, clearReadFlags: false);
    int ITmclVmHost.GetReferenceSearchStatus(int axis) => _axes[axis].GetReferenceSearchStatus();
    bool ITmclVmHost.GetDigitalInput(int port) => port >= 0 && port < _digitalInputs.Length && _digitalInputs[port] != 0;
    int ITmclVmHost.GetGlobalParameterRaw(byte bank, byte param) => GetGlobalParameterUnsafe(bank, param);
    void ITmclVmHost.SetUserVariableRaw(byte index, int value) => _globalParams[(2, index)] = value;
    void ITmclVmHost.SyncErrorFlagsFromVm(int errorFlags) => _internalErrorFlags = errorFlags;

    TmclStatus ITmclVmHost.ExecuteHardwareCommand(
        byte opcode,
        byte type,
        byte motorOrBank,
        int value,
        bool isStandalone,
        out int readResult,
        out bool hasReadResult) =>
        ExecuteHardwareCommandUnsafe(opcode, type, motorOrBank, value, isStandalone, out _, out readResult, out hasReadResult);
}
