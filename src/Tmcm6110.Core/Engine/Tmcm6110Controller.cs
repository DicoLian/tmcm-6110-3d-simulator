using System.Text;
using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Full TMCM-6110 6-Axis Motion Controller Simulator Engine (focused on 3-Axis Z/Y/X active operation).
/// Processes 9-Byte Binary TMCL Packets, manages Axis 0..5 states, Bank 0/1/2 Global Parameters,
/// Digital/Analog I/O, Multi-Axis Interpolation (MVP COORD), and Command 138 Async Position Reached Events.
/// </summary>
public class Tmcm6110Controller : IMotionController
{
    public const int TotalAxes = 6;
    public string ControllerName => "TMCM-6110 (TMCL)";
    public int AxisCount => TotalAxes;

    private readonly object _controllerLock = new();
    private readonly AxisState[] _axes = new AxisState[TotalAxes];
    private readonly Dictionary<(byte Bank, byte Param), int> _globalParams = new();
    private readonly int[] _digitalInputs = new int[11];   // Bank 0: ADIN0..7, OPTO1..2, SHUTDOWN
    private readonly int[] _analogInputs = new int[8];     // Bank 1: ADIN0..7 (0..1023)
    private readonly int[] _digitalOutputs = new int[8];   // Bank 2: DOUT0..7 (0/1)

    // Pending Command 138 (Request target position reached event) bitmask
    // When non-zero, the controller monitors the specified axes and fires an async Status=128 reply once all reach target.
    private int _pendingPositionReachedEventMask;
    private bool _pendingEventArmed;

    public byte ModuleAddress
    {
        get => (byte)GetGlobalParameter(0, 66);
        set => SetGlobalParameter(0, 66, value);
    }

    public byte HostAddress
    {
        get => (byte)GetGlobalParameter(0, 76);
        set => SetGlobalParameter(0, 76, value);
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
            // Default TMCM-6110 global settings
            _globalParams[(0, 64)] = 0xE4; // EEPROM magic
            _globalParams[(0, 65)] = 0;    // RS485 baud rate (0 = 9600)
            _globalParams[(0, 66)] = 1;    // Serial address = 1
            _globalParams[(0, 67)] = 0;    // ASCII mode = 0 (Binary 9-Byte mode)
            _globalParams[(0, 68)] = 0;    // Serial heartbeat
            _globalParams[(0, 69)] = 8;    // CAN Bitrate = 8 (1000 kBit/s)
            _globalParams[(0, 70)] = 2;    // CAN Send ID = 2
            _globalParams[(0, 71)] = 1;    // CAN Receive ID = 1
            _globalParams[(0, 73)] = 0;    // EEPROM lock flag (0 = unlocked)
            _globalParams[(0, 75)] = 0;    // Telegram pause time = 0
            _globalParams[(0, 76)] = 2;    // Serial host address = 2
            _globalParams[(0, 77)] = 0;    // Auto start mode = 0
            _globalParams[(0, 79)] = 1;    // Limit switch polarity = 1
            _globalParams[(0, 81)] = 0;    // TMCL Protection Mode = 0
            _globalParams[(0, 82)] = 0;    // CAN heartbeat
            _globalParams[(0, 83)] = 0;    // CAN secondary address
            _globalParams[(0, 84)] = 0;    // EEPROM Coordinate Store
            _globalParams[(0, 85)] = 0;    // Do not restore user variables
            _globalParams[(0, 87)] = 0;    // Secondary serial address
            _globalParams[(0, 90)] = 0;    // Reverse shaft
            _globalParams[(0, 128)] = 0;   // Application status (0 = stop)
            _globalParams[(0, 132)] = 0;   // Tick timer (ms)

            Array.Clear(_digitalInputs);
            Array.Clear(_digitalOutputs);
            for (int i = 0; i < _analogInputs.Length; i++)
                _analogInputs[i] = 512;
        }
    }

    public int GetGlobalParameter(byte bank, byte param)
    {
        lock (_controllerLock)
        {
            return _globalParams.TryGetValue((bank, param), out int val) ? val : 0;
        }
    }

    public void SetGlobalParameter(byte bank, byte param, int value)
    {
        lock (_controllerLock)
        {
            if (bank == 0 && param == 73)
            {
                // EEPROM lock/unlock magic numbers
                if (value == 1234) _globalParams[(0, 73)] = 1;
                else if (value == 4321) _globalParams[(0, 73)] = 0;
                return;
            }
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

    /// <summary>
    /// Advances physics for all axes and checks if any armed Command 138 Position Reached Event should fire.
    /// </summary>
    public void StepSimulation(double dtSeconds)
    {
        TmclReplyPacket? asyncEventPacket = null;

        lock (_controllerLock)
        {
            // Increment millisecond tick timer (Global Param #132)
            _globalParams.TryGetValue(((byte)0, (byte)132), out int currentTick);
            _globalParams[((byte)0, (byte)132)] = unchecked(currentTick + (int)(dtSeconds * 1000.0));

            for (int i = 0; i < TotalAxes; i++)
            {
                _axes[i].StepPhysics(dtSeconds, SimulationSpeedMultiplier);
            }

            // Check if Command 138 (Request target position reached event) is armed
            if (_pendingEventArmed && _pendingPositionReachedEventMask != 0)
            {
                bool allReached = true;
                for (int i = 0; i < TotalAxes; i++)
                {
                    if ((_pendingPositionReachedEventMask & (1 << i)) != 0)
                    {
                        if (_axes[i].GetParameter(TmclAxisParam.TargetPosReached) == 0)
                        {
                            allReached = false;
                            break;
                        }
                    }
                }

                if (allReached)
                {
                    // According to TMCL Reference Manual Section 3.33:
                    // Additional reply after motors have reached target positions:
                    // Status = 128, Instruction = 138, Value = Motor bit mask
                    asyncEventPacket = new TmclReplyPacket(
                        HostAddress,
                        ModuleAddress,
                        TmclStatus.TargetPositionReachedEvent,
                        (byte)TmclCommand.RequestTargetPositionReachedEvent,
                        _pendingPositionReachedEventMask);

                    _pendingEventArmed = false;
                    _pendingPositionReachedEventMask = 0;
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
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                ExecuteWithGroupIndex(motOrBank, ax => ax.RotateRight(val));
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.ROL:
                if (motOrBank >= TotalAxes)
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
                _axes[motOrBank].SetParameter(type, val);
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                int paramVal = _axes[motOrBank].GetParameter(type);
                return Reply(TmclStatus.SuccessOk, opcode, paramVal);

            case TmclCommand.STAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                if (GetGlobalParameter(0, 73) != 0)
                    return Reply(TmclStatus.ConfigurationEepromLocked, opcode, 0);
                _axes[motOrBank].StoreParameterToEeprom(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.RSAP:
                if (motOrBank >= TotalAxes)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _axes[motOrBank].RestoreParameterFromEeprom(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.SGP:
                if (motOrBank > 2)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                SetGlobalParameter(motOrBank, type, val);
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GGP:
                if (motOrBank > 2)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                return Reply(TmclStatus.SuccessOk, opcode, GetGlobalParameter(motOrBank, type));

            case TmclCommand.STGP:
            case TmclCommand.RSGP:
                if (GetGlobalParameter(0, 73) != 0 && cmd.CommandEnum == TmclCommand.STGP)
                    return Reply(TmclStatus.ConfigurationEepromLocked, opcode, 0);
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
                        for (int i = 0; i < 8; i++)
                            _digitalOutputs[i] = (val & (1 << i)) != 0 ? 1 : 0;
                        return Reply(TmclStatus.SuccessOk, opcode, val);
                    }
                    if (type < 8)
                    {
                        _digitalOutputs[type] = val != 0 ? 1 : 0;
                        return Reply(TmclStatus.SuccessOk, opcode, val);
                    }
                }
                else if (motOrBank == 0 && type < 8)
                {
                    // Pull-up resistor configuration on TMCM-61x
                    return Reply(TmclStatus.SuccessOk, opcode, val);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.GIO:
                if (motOrBank == 0) // Digital inputs
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
                }
                else if (motOrBank == 1 && type < _analogInputs.Length) // Analog inputs
                {
                    return Reply(TmclStatus.SuccessOk, opcode, _analogInputs[type]);
                }
                else if (motOrBank == 2 && type < _digitalOutputs.Length) // Digital outputs readback
                {
                    return Reply(TmclStatus.SuccessOk, opcode, _digitalOutputs[type]);
                }
                return Reply(TmclStatus.WrongType, opcode, 0);

            case TmclCommand.SCO:
                if (motOrBank >= TotalAxes || type > 56)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _axes[motOrBank].SetCoordinate(type, val);
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GCO:
                if (motOrBank >= TotalAxes || type > 56)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                return Reply(TmclStatus.SuccessOk, opcode, _axes[motOrBank].GetCoordinate(type));

            case TmclCommand.CCO:
                if (motOrBank >= TotalAxes || type > 56)
                    return Reply(TmclStatus.InvalidValue, opcode, 0);
                _axes[motOrBank].CaptureCoordinate(type);
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.CLE:
                for (int i = 0; i < TotalAxes; i++)
                    _axes[i].ClearAlarm();
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            case TmclCommand.GetFirmwareVersion: // Cmd 136
                if (type == 0)
                {
                    // String format: [0] HostAddr, [1..8] "0610V631" (8 ASCII chars, no checksum!)
                    byte[] raw = new byte[9];
                    raw[0] = replyAddr;
                    byte[] ascii = Encoding.ASCII.GetBytes("0610V631");
                    Array.Copy(ascii, 0, raw, 1, 8);
                    return new TmclReplyPacket(raw);
                }
                else
                {
                    // Binary format: 0x06100631 (Module 610/6110, FW V6.31)
                    return Reply(TmclStatus.SuccessOk, opcode, 0x06100631);
                }

            case TmclCommand.RequestTargetPositionReachedEvent: // Cmd 138
                _pendingPositionReachedEventMask = val & 0x3F;
                _pendingEventArmed = _pendingPositionReachedEventMask != 0;
                return Reply(TmclStatus.SuccessOk, opcode, val);

            case TmclCommand.GetAppStatus:
                return Reply(TmclStatus.SuccessOk, opcode, GetGlobalParameter(0, 128));

            case TmclCommand.StopApplication:
            case TmclCommand.RunApplication:
            case TmclCommand.StepApplication:
            case TmclCommand.ResetApplication:
                return Reply(TmclStatus.SuccessOk, opcode, 0);

            default:
                return Reply(TmclStatus.InvalidCommand, opcode, 0);
        }

        TmclReplyPacket Reply(TmclStatus st, byte op, int v) =>
            new(replyAddr, modAddr, st, op, v);
    }

    private TmclReplyPacket HandleMvpCommandUnsafe(byte type, byte motOrBank, int val)
    {
        byte opcode = (byte)TmclCommand.MVP;

        if (type == 0) // MVP ABS
        {
            if (motOrBank >= TotalAxes)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            ExecuteWithGroupIndex(motOrBank, ax => ax.MoveToAbsolute(val));
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        else if (type == 1) // MVP REL
        {
            if (motOrBank >= TotalAxes)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            ExecuteWithGroupIndex(motOrBank, ax => ax.MoveRelative(val));
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
        }
        else if (type == 2) // MVP COORD
        {
            if (val < 0 || val > 56)
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);

            // TMCM-61x MVP COORD bitmask support (Section 3.4 of TMCL Reference Manual):
            // If bit 6 (0x40) or bit 7 (0x80) is set, bits 0..5 specify the motor bitmask!
            // Bit 6 (0x40) = Use multi-axis linear interpolation so all selected motors reach target simultaneously.
            // Bit 7 (0x80) = Move selected motors without interpolation.
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

                if (selectedAxes.Count > 0)
                {
                    if (interpolate && selectedAxes.Count > 1)
                    {
                        // Calculate largest distance and scale velocities proportionally so all axes arrive together
                        double maxTimeRatio = 0;
                        foreach (var ax in selectedAxes)
                        {
                            double dist = Math.Abs(ax.GetCoordinate(val) - ax.GetParameter(TmclAxisParam.ActualPosition));
                            double maxSpd = Math.Max(1, ax.GetParameter(TmclAxisParam.MaxPositioningSpeed));
                            double ratio = dist / maxSpd;
                            if (ratio > maxTimeRatio) maxTimeRatio = ratio;
                        }

                        foreach (var ax in selectedAxes)
                        {
                            int targetCoord = ax.GetCoordinate(val);
                            double dist = Math.Abs(targetCoord - ax.GetParameter(TmclAxisParam.ActualPosition));
                            double interpSpeed = maxTimeRatio > 1e-6 ? Math.Max(1.0, dist / maxTimeRatio) : ax.GetParameter(TmclAxisParam.MaxPositioningSpeed);
                            ax.MoveToAbsolute(targetCoord, interpSpeed);
                        }
                    }
                    else
                    {
                        foreach (var ax in selectedAxes)
                            ax.MoveToAbsolute(ax.GetCoordinate(val));
                    }
                }
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            else if (motOrBank < TotalAxes)
            {
                // Single axis MVP COORD, <axis>, <coord_index>
                int targetCoord = _axes[motOrBank].GetCoordinate(val);
                _axes[motOrBank].MoveToAbsolute(targetCoord);
                return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.SuccessOk, opcode, val);
            }
            return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.InvalidValue, opcode, 0);
        }

        return new TmclReplyPacket(HostAddress, ModuleAddress, TmclStatus.WrongType, opcode, 0);
    }

    private void ExecuteWithGroupIndex(int targetAxisIndex, Action<AxisState> action)
    {
        var primary = _axes[targetAxisIndex];
        action(primary);

        // TMCM-61x Parameter #213 Group Index: if > 0, all axes with the same group index execute the command
        int groupIdx = primary.GetParameter(TmclAxisParam.GroupIndex, clearReadFlags: false);
        if (groupIdx > 0)
        {
            for (int i = 0; i < TotalAxes; i++)
            {
                if (i != targetAxisIndex && _axes[i].GetParameter(TmclAxisParam.GroupIndex, clearReadFlags: false) == groupIdx)
                {
                    action(_axes[i]);
                }
            }
        }
    }
}
