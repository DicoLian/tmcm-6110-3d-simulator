using System.Text;
using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Full TMCM-6110 6-Axis Motion Controller Simulator Engine implementing
/// TMCM-6110 TMCL Firmware Manual V1.47 (Rev V1.23).
/// Processes 9-Byte Binary TMCL Packets, manages Axis 0..5 states, Bank 0/2/3 Global Parameters,
/// Digital/12-bit Analog I/O, Multi-Axis Interpolation (MVP COORD, up to 3 axes per call),
/// and Command 138 Async Position Reached Events.
/// </summary>
public class Tmcm6110Controller : IMotionController
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

    // Standalone / Control registers
    private int _accumulator;
    private int _xRegister;
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
            _accumulator = 0;
            _xRegister = 0;
            _internalErrorFlags = 0;
            _event138Mask = 0;
            _event138Continuous = false;
            _event138PendingMask = 0;
        }
    }

    public int GetGlobalParameter(byte bank, byte param)
    {
        lock (_controllerLock)
        {
            if (bank == 0 && param == 133)
                return _random.Next(0, int.MaxValue);

            return _globalParams.TryGetValue((bank, param), out int val) ? val : 0;
        }
    }

    public void SetGlobalParameter(byte bank, byte param, int value)
    {
        lock (_controllerLock)
        {
            if (bank == 0 && param == 133)
                _random = new Random(value);

            _globalParams[(bank, param)] = value;
        }
    }

    public bool GetDigitalOutput(int port) => port >= 0 && port < _digitalOutputs.Length && _digitalOutputs[port] != 0;
    public void SetDigitalOutput(int port, bool state)
    {
        if (port >= 0 && port < _digitalOutputs.Length)
            _digitalOutputs[port] = state ? 1 : 0;
    }

    public bool GetDigitalInput(int port) => port >= 0 && port < _digitalInputs.Length && _digitalInputs[port] != 0;
    public void SetDigitalInput(int port, bool state)
    {
        if (port >= 0 && port < _digitalInputs.Length)
            _digitalInputs[port] = state ? 1 : 0;
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
    /// Advances physics for all axes and emits Command 138 Position Reached Events when armed targets arrive.
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

        switch (cmd.CommandEnum)
        {
            case TmclCommand.ROR:
                if (motOrBank >= TotalAxes || val < -2047 || val > 2047)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateRight(val));
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.ROL:
                if (motOrBank >= TotalAxes || val < -2047 || val > 2047)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateLeft(val));
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.MST:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                ExecuteWithGroupIndex(motOrBank, ax => ax.StopMotor());
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.MVP:
                return HandleMvpCommandUnsafe(type, motOrBank, val);

            case TmclCommand.SAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (!TmclAxisParam.IsValidParameter(type) || TmclAxisParam.IsReadOnly(type))
                    return Reply(TmclStatus.WrongType, opcode, 0);
                if (!TmclAxisParam.IsValueValid(type, val))
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _axes[motOrBank].SetParameter(type, val);
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (!TmclAxisParam.IsValidParameter(type))
                    return Reply(TmclStatus.WrongType, opcode, 0);
                int paramVal = _axes[motOrBank].GetParameter(type);
                return Reply(TmclStatus.SuccessOk, opcode, paramVal);

            case TmclCommand.STAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (!TmclAxisParam.IsValidParameter(type) || TmclAxisParam.IsReadOnly(type))
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _axes[motOrBank].StoreParameterToEeprom(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RSAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (!TmclAxisParam.IsValidParameter(type) || TmclAxisParam.IsReadOnly(type))
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _axes[motOrBank].RestoreParameterFromEeprom(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.SGP:
                return HandleSgpCommandUnsafe(type, motOrBank, val);

            case TmclCommand.GGP:
                return HandleGgpCommandUnsafe(type, motOrBank);

            case TmclCommand.STGP:
                if (motOrBank != 2)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _userVarEeprom[type] = _globalParams.TryGetValue((2, type), out int uv) ? uv : 0;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RSGP:
                if (motOrBank != 2)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _globalParams[(2, type)] = _userVarEeprom[type];
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RFS:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (type == 0) // RFS START
                {
                    ExecuteWithGroupIndex(motOrBank, ax => ax.StartReferenceSearch());
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                else if (type == 1) // RFS STOP
                {
                    ExecuteWithGroupIndex(motOrBank, ax => ax.StopReferenceSearch());
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                else if (type == 2) // RFS STATUS
                {
                    int status = _axes[motOrBank].GetReferenceSearchStatus();
                    return Reply(TmclStatus.SuccessOk, opcode, status);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.SIO:
                if (motOrBank == 2)
                {
                    if (type == 255)
                    {
                        int outMask = val == -1 ? (_accumulator & 0xFF) : val;
                        if (outMask < 0 || outMask > 255)
                            return Reply(TmclStatus.InvalidValue, opcode, 0);
                        for (int i = 0; i < 8; i++)
                            _digitalOutputs[i] = (outMask & (1 << i)) != 0 ? 1 : 0;
                        return Reply(TmclStatus.SuccessOk, opcode, val);
                    }
                    if (type < 8)
                    {
                        if (val != 0 && val != 1)
                            return Reply(TmclStatus.InvalidValue, opcode, 0);
                        _digitalOutputs[type] = val;
                        return Reply(TmclStatus.SuccessOk, opcode, val);
                    }
                    return Reply(TmclStatus.WrongType, opcode, 0);
                }
                else if (motOrBank == 0 && type == 0)
                {
                    // Pull-up resistor configuration for STOP_L/STOP_R (SIO 0, 0, <0..3>)
                    if (val < 0 || val > 3)
                        return Reply(TmclStatus.InvalidValue, opcode, 0);
                    _stopSwitchPullUpMask = val;
                    return Reply(TmclStatus.SuccessOk, opcode, val);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.GIO:
                if (motOrBank == 0) // Bank 0: Digital inputs (Ports 0..7 and 255)
                {
                    if (type == 255)
                    {
                        int mask = 0;
                        for (int i = 0; i < 8; i++)
                            if (_digitalInputs[i] != 0) mask |= (1 << i);
                        return Reply(TmclStatus.SuccessOk, opcode, mask);
                    }
                    if (type < _digitalInputs.Length)
                        return Reply(TmclStatus.SuccessOk, opcode, _digitalInputs[type]);
                    return Reply(TmclStatus.WrongType, opcode, 0);
                }
                else if (motOrBank == 1) // Bank 1: Analog inputs (AIN0=0, AIN4=4, SupplyVoltage=8)
                {
                    if (type == 0)
                        return Reply(TmclStatus.SuccessOk, opcode, _analogIn0);
                    if (type == 4)
                        return Reply(TmclStatus.SuccessOk, opcode, _analogIn4);
                    if (type == 8)
                        return Reply(TmclStatus.SuccessOk, opcode, _supplyVoltageDeciVolts);
                    return Reply(TmclStatus.WrongType, opcode, 0);
                }
                else if (motOrBank == 2) // Bank 2: Digital outputs readback (Ports 0..7 and 255)
                {
                    if (type == 255)
                    {
                        int mask = 0;
                        for (int i = 0; i < 8; i++)
                            if (_digitalOutputs[i] != 0) mask |= (1 << i);
                        return Reply(TmclStatus.SuccessOk, opcode, mask);
                    }
                    if (type < _digitalOutputs.Length)
                        return Reply(TmclStatus.SuccessOk, opcode, _digitalOutputs[type]);
                    return Reply(TmclStatus.WrongType, opcode, 0);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.SCO:
                if (type > MaxCoordinateIndex)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (motOrBank == 255)
                {
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].CopyCoordinateRamToEeprom(type);
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                bool storeCoordEeprom = _globalParams.TryGetValue((0, 84), out int csSco) && csSco != 0;
                _axes[motOrBank].SetCoordinate(type, val, storeCoordEeprom);
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GCO:
                if (type > MaxCoordinateIndex)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (motOrBank == 255)
                {
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].CopyCoordinateEepromToRam(type);
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                return Reply(TmclStatus.SuccessOk, opcode, _axes[motOrBank].GetCoordinate(type));

            case TmclCommand.CCO:
                if (motOrBank >= TotalAxes || type > MaxCoordinateIndex)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                bool storeCcoEeprom = _globalParams.TryGetValue((0, 84), out int csCco) && csCco != 0;
                int captured = _axes[motOrBank].CaptureCoordinate(type, storeCcoEeprom);
                return Reply(TmclStatus.SuccessOk, opcode, captured);

            case TmclCommand.CALC:
                switch (type)
                {
                    case 0: _accumulator = unchecked(_accumulator + val); break;
                    case 1: _accumulator = unchecked(_accumulator - val); break;
                    case 2: _accumulator = unchecked(_accumulator * val); break;
                    case 3:
                        if (val == 0) return Reply(TmclStatus.InvalidValue, opcode, 0);
                        _accumulator /= val;
                        break;
                    case 4:
                        if (val == 0) return Reply(TmclStatus.InvalidValue, opcode, 0);
                        _accumulator %= val;
                        break;
                    case 5: _accumulator &= val; break;
                    case 6: _accumulator |= val; break;
                    case 7: _accumulator ^= val; break;
                    case 8: _accumulator = ~_accumulator; break;
                    case 9: _accumulator = val; break;
                    default: return Reply(TmclStatus.WrongType, opcode, 0);
                }
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.ACO:
                if (motOrBank >= TotalAxes || type > MaxCoordinateIndex)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                bool storeAcoEeprom = _globalParams.TryGetValue((0, 84), out int csAco) && csAco != 0;
                _axes[motOrBank].SetCoordinate(type, _accumulator, storeAcoEeprom);
                return Reply(TmclStatus.SuccessOk, opcode, _accumulator);

            case TmclCommand.CLE:
                if (type > 5)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                if (type == 0)
                    _internalErrorFlags = 0;
                else
                    _internalErrorFlags &= ~(1 << (type - 1));
                return Reply(TmclStatus.SuccessOk, opcode, 0);

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

            case TmclCommand.RequestTargetPositionReachedEvent: // Cmd 138
                if (type > 1)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                _event138Mask = val & 0x3F;
                _event138Continuous = type == 1;
                if (_event138Mask == 0)
                    _event138PendingMask = 0;
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GetAppStatus: // Cmd 135
                return type switch
                {
                    0 => Reply(TmclStatus.SuccessOk, opcode, GetGlobalParameterUnsafe(0, 128)),
                    1 => Reply(TmclStatus.SuccessOk, opcode, GetGlobalParameterUnsafe(0, 130)),
                    2 => Reply(TmclStatus.SuccessOk, opcode, _accumulator),
                    3 => Reply(TmclStatus.SuccessOk, opcode, _xRegister),
                    _ => Reply(TmclStatus.WrongType, opcode, 0)
                };

            case TmclCommand.StopApplication: // Cmd 128
                _globalParams[(0, 128)] = 0;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RunApplication: // Cmd 129
                if (type > 1)
                    return Reply(TmclStatus.WrongType, opcode, 0);
                if (type == 1)
                    _globalParams[(0, 130)] = val;
                _globalParams[(0, 128)] = 1;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.StepApplication: // Cmd 130
                _globalParams[(0, 128)] = 2;
                _globalParams[(0, 130)] = GetGlobalParameterUnsafe(0, 130) + 1;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.ResetApplication: // Cmd 131
                _globalParams[(0, 128)] = 3;
                _globalParams[(0, 130)] = 0;
                _accumulator = 0;
                _xRegister = 0;
                _internalErrorFlags = 0;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.StartDownload: // Cmd 132
                _globalParams[(0, 129)] = 1;
                _globalParams[(0, 130)] = val;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.QuitDownload: // Cmd 133
                _globalParams[(0, 129)] = 0;
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RestoreFactorySettings: // Cmd 137
                if (val == 1234)
                {
                    for (int i = 0; i < TotalAxes; i++)
                        _axes[i].ResetToDefaults(i);
                    ResetGlobalParametersToDefaults();
                    return Reply(TmclStatus.SuccessOk, opcode, 0);
                }
                return Reply(TmclStatus.InvalidValue, opcode, 0);

            case TmclCommand.SoftwareReset: // Cmd 255
                if (val != 1234)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                PerformSoftwareResetUnsafe();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            default:
                return Reply(TmclStatus.InvalidCommand, opcode, 0);
        }

        TmclReplyPacket Reply(TmclStatus st, byte op, int v) =>
            new(replyAddr, modAddr, st, op, v);
    }

    private int GetGlobalParameterUnsafe(byte bank, byte param) =>
        _globalParams.TryGetValue((bank, param), out int val) ? val : 0;

    private void PerformSoftwareResetUnsafe()
    {
        bool restoreCoords = GetGlobalParameterUnsafe(0, 84) != 0;
        for (int i = 0; i < TotalAxes; i++)
            _axes[i].RestoreAllFromEeprom(restoreCoords);

        bool doNotRestoreVars = GetGlobalParameterUnsafe(0, 85) != 0;
        for (int i = 0; i < 256; i++)
            _globalParams[(2, (byte)i)] = doNotRestoreVars ? 0 : _userVarEeprom[i];

        _globalParams[(0, 128)] = 0;
        _globalParams[(0, 129)] = 0;
        _globalParams[(0, 130)] = 0;
        _globalParams[(0, 132)] = 0;
        _globalParams[(0, 255)] = 0;

        _accumulator = 0;
        _xRegister = 0;
        _internalErrorFlags = 0;
        _event138Mask = 0;
        _event138Continuous = false;
        _event138PendingMask = 0;
    }

    private TmclReplyPacket HandleSgpCommandUnsafe(byte type, byte bank, int val)
    {
        byte opcode = (byte)TmclCommand.SGP;
        if (bank == 0)
        {
            if (!IsValidBank0GlobalParam(type) || IsReadOnlyBank0GlobalParam(type))
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
            if (!IsBank0ValueValid(type, val))
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            if (type == 133)
                _random = new Random(val);
            _globalParams[(0, type)] = val;
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        if (bank == 2)
        {
            _globalParams[(2, type)] = val;
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        if (bank == 3)
        {
            if (type <= 2)
            {
                _globalParams[(3, type)] = val;
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            if (type is >= 27 and <= 46)
            {
                if (val < 0 || val > 3)
                    return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);
                _globalParams[(3, type)] = val;
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
        }

        return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);
    }

    private TmclReplyPacket HandleGgpCommandUnsafe(byte type, byte bank)
    {
        byte opcode = (byte)TmclCommand.GGP;
        if (bank == 0)
        {
            if (!IsValidBank0GlobalParam(type))
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
            int val = type == 133 ? _random.Next(0, int.MaxValue) : GetGlobalParameterUnsafe(0, type);
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        if (bank == 2)
        {
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, GetGlobalParameterUnsafe(2, type));
        }
        if (bank == 3)
        {
            if (type <= 2 || type is >= 27 and <= 46)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, GetGlobalParameterUnsafe(3, type));
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
        }

        return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);
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

    private TmclReplyPacket HandleMvpCommandUnsafe(byte type, byte motOrBank, int val)
    {
        byte opcode = (byte)TmclCommand.MVP;

        if (type == 0) // MVP ABS
        {
            if (motOrBank >= TotalAxes)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            int movedMask = ExecuteWithGroupIndex(motOrBank, ax => ax.MoveToAbsolute(val));
            ArmEvent138ForAxesUnsafe(movedMask);
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        else if (type == 1) // MVP REL
        {
            if (motOrBank >= TotalAxes)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            int movedMask = ExecuteWithGroupIndex(motOrBank, ax => ax.MoveRelative(val));
            ArmEvent138ForAxesUnsafe(movedMask);
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        else if (type == 2) // MVP COORD
        {
            if (val < 0 || val > MaxCoordinateIndex)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

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
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            else if (motOrBank < TotalAxes)
            {
                // Single axis MVP COORD, <axis>, <coord_index>
                int targetCoord = _axes[motOrBank].GetCoordinate(val);
                _axes[motOrBank].MoveToAbsolute(targetCoord);
                ArmEvent138ForAxesUnsafe(1 << motOrBank);
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);
        }

        return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
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
}
