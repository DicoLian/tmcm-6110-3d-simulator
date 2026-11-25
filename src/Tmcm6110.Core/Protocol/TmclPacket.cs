namespace Tmcm6110.Core.Protocol;

/// <summary>
/// Represents a 9-Byte Binary TMCL Command Packet sent from Host (C# Moto module) to TMCM-6110.
/// Format: [0] ModuleAddr, [1] Command, [2] Type, [3] Motor/Bank, [4..7] Value (MSB First), [8] Checksum
/// </summary>
public readonly struct TmclCommandPacket
{
    public byte ModuleAddress { get; }
    public byte Command { get; }
    public byte TypeNumber { get; }
    public byte MotorOrBank { get; }
    public int Value { get; }
    public byte Checksum { get; }
    public byte CalculatedChecksum { get; }
    public bool IsChecksumValid => Checksum == CalculatedChecksum;

    public TmclCommand CommandEnum => (TmclCommand)Command;

    public TmclCommandPacket(byte moduleAddress, byte command, byte typeNumber, byte motorOrBank, int value, byte? checksum = null)
    {
        ModuleAddress = moduleAddress;
        Command = command;
        TypeNumber = typeNumber;
        MotorOrBank = motorOrBank;
        Value = value;

        byte b3 = (byte)((value >> 24) & 0xFF);
        byte b2 = (byte)((value >> 16) & 0xFF);
        byte b1 = (byte)((value >> 8) & 0xFF);
        byte b0 = (byte)(value & 0xFF);

        CalculatedChecksum = unchecked((byte)(moduleAddress + command + typeNumber + motorOrBank + b3 + b2 + b1 + b0));
        Checksum = checksum ?? CalculatedChecksum;
    }

    public static TmclCommandPacket FromBytes(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < 9)
            throw new ArgumentException("TMCL binary command packet must be 9 bytes.", nameof(bytes));

        int val = (bytes[4] << 24) | (bytes[5] << 16) | (bytes[6] << 8) | bytes[7];
        return new TmclCommandPacket(bytes[0], bytes[1], bytes[2], bytes[3], val, bytes[8]);
    }

    public byte[] ToBytes()
    {
        return new byte[9]
        {
            ModuleAddress,
            Command,
            TypeNumber,
            MotorOrBank,
            (byte)((Value >> 24) & 0xFF),
            (byte)((Value >> 16) & 0xFF),
            (byte)((Value >> 8) & 0xFF),
            (byte)(Value & 0xFF),
            Checksum
        };
    }

    public string ToHexString() => BitConverter.ToString(ToBytes()).Replace("-", " ");

    /// <summary>
    /// Disassembles the 9-byte binary packet into a human-readable TMCL mnemonic string.
    /// </summary>
    public string ToMnemonicDescription(Func<int, string>? axisLabelResolver = null)
    {
        string axisLabel = axisLabelResolver != null ? axisLabelResolver(MotorOrBank) : $"Axis {MotorOrBank}";

        return CommandEnum switch
        {
            TmclCommand.ROR => $"ROR {axisLabel}, Vel={Value}",
            TmclCommand.ROL => $"ROL {axisLabel}, Vel={Value}",
            TmclCommand.MST => $"MST {axisLabel} (Stop Motor)",
            TmclCommand.MVP => TypeNumber switch
            {
                0 => $"MVP ABS, {axisLabel}, Target={Value}",
                1 => $"MVP REL, {axisLabel}, Offset={Value}",
                2 => $"MVP COORD, Mask=0x{MotorOrBank:X2}, Coord#{Value}",
                _ => $"MVP Type={TypeNumber}, {axisLabel}, Val={Value}"
            },
            TmclCommand.SAP => $"SAP {TmclAxisParam.GetParameterName(TypeNumber)}, {axisLabel}, Val={Value}",
            TmclCommand.GAP => $"GAP {TmclAxisParam.GetParameterName(TypeNumber)}, {axisLabel}",
            TmclCommand.STAP => $"STAP {TmclAxisParam.GetParameterName(TypeNumber)}, {axisLabel}",
            TmclCommand.RSAP => $"RSAP {TmclAxisParam.GetParameterName(TypeNumber)}, {axisLabel}",
            TmclCommand.SGP => $"SGP Param#{TypeNumber}, Bank {MotorOrBank}, Val={Value}",
            TmclCommand.GGP => $"GGP Param#{TypeNumber}, Bank {MotorOrBank}",
            TmclCommand.STGP => $"STGP Param#{TypeNumber}, Bank {MotorOrBank}",
            TmclCommand.RSGP => $"RSGP Param#{TypeNumber}, Bank {MotorOrBank}",
            TmclCommand.RFS => TypeNumber switch
            {
                0 => $"RFS START, {axisLabel}",
                1 => $"RFS STOP, {axisLabel}",
                2 => $"RFS STATUS, {axisLabel}",
                _ => $"RFS Type={TypeNumber}, {axisLabel}"
            },
            TmclCommand.SIO => $"SIO Port#{TypeNumber}, Bank {MotorOrBank}, Val={Value}",
            TmclCommand.GIO => $"GIO Port#{TypeNumber}, Bank {MotorOrBank}",
            TmclCommand.SCO => MotorOrBank == 255
                ? $"SCO Coord#{TypeNumber}, 255 (RAM->EEPROM)"
                : $"SCO Coord#{TypeNumber}, {axisLabel}, Pos={Value}",
            TmclCommand.GCO => MotorOrBank == 255
                ? $"GCO Coord#{TypeNumber}, 255 (EEPROM->RAM)"
                : $"GCO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.CCO => $"CCO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.ACO => $"ACO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.CLE => TypeNumber switch
            {
                0 => "CLE ALL (Clear All Error Flags)",
                1 => "CLE ETO (Clear Timeout Flag)",
                2 => "CLE EAL (Clear External Alarm Flag)",
                3 => "CLE EDV (Clear Deviation Flag)",
                4 => "CLE EPO (Clear Position Flag)",
                5 => "CLE ESD (Clear Shutdown Flag)",
                _ => $"CLE Type={TypeNumber}"
            },
            TmclCommand.GetAppStatus => $"GetAppStatus (135), Type={TypeNumber}",
            TmclCommand.GetFirmwareVersion => $"GetFirmwareVersion (136), Type={TypeNumber}",
            TmclCommand.RestoreFactorySettings => $"RestoreFactorySettings (137), Magic={Value}",
            TmclCommand.RequestTargetPositionReachedEvent => $"ReqPosReachedEvent (138), Type={TypeNumber} ({(TypeNumber == 1 ? "EveryMVP" : "NextMVP")}), AxisMask=0x{Value:X2}",
            TmclCommand.SoftwareReset => $"SoftwareReset (255), Magic={Value}",
            _ => $"Cmd#{Command} ({CommandEnum}), Type={TypeNumber}, Mot/Bank={MotorOrBank}, Val={Value}"
        };
    }

    public bool IsPollingQuery =>
        CommandEnum == TmclCommand.GAP ||
        CommandEnum == TmclCommand.GGP ||
        CommandEnum == TmclCommand.GIO ||
        (CommandEnum == TmclCommand.RFS && TypeNumber == 2);
}

/// <summary>
/// Represents a 9-Byte Binary TMCL Reply Packet sent from TMCM-6110 back to the Host.
/// Format: [0] ReplyAddr, [1] ModuleAddr, [2] Status, [3] Command, [4..7] Value (MSB First), [8] Checksum
/// </summary>
public readonly struct TmclReplyPacket
{
    public byte ReplyAddress { get; }
    public byte ModuleAddress { get; }
    public byte Status { get; }
    public byte Command { get; }
    public int Value { get; }
    public byte Checksum { get; }
    public byte[]? RawOverrideBytes { get; } // Used for Cmd 136 Type 0 string reply (no checksum)

    public TmclStatus StatusEnum => (TmclStatus)Status;

    public TmclReplyPacket(byte replyAddress, byte moduleAddress, TmclStatus status, byte command, int value)
    {
        ReplyAddress = replyAddress;
        ModuleAddress = moduleAddress;
        Status = (byte)status;
        Command = command;
        Value = value;
        RawOverrideBytes = null;

        byte b3 = (byte)((value >> 24) & 0xFF);
        byte b2 = (byte)((value >> 16) & 0xFF);
        byte b1 = (byte)((value >> 8) & 0xFF);
        byte b0 = (byte)(value & 0xFF);

        Checksum = unchecked((byte)(replyAddress + moduleAddress + Status + command + b3 + b2 + b1 + b0));
    }

    public TmclReplyPacket(byte[] rawBytes)
    {
        if (rawBytes.Length != 9)
            throw new ArgumentException("Raw reply must be 9 bytes.", nameof(rawBytes));

        ReplyAddress = rawBytes[0];
        ModuleAddress = rawBytes[1];
        Status = rawBytes[2];
        Command = rawBytes[3];
        Value = (rawBytes[4] << 24) | (rawBytes[5] << 16) | (rawBytes[6] << 8) | rawBytes[7];
        Checksum = rawBytes[8];
        RawOverrideBytes = rawBytes;
    }

    public byte[] ToBytes()
    {
        if (RawOverrideBytes != null)
            return RawOverrideBytes;

        return new byte[9]
        {
            ReplyAddress,
            ModuleAddress,
            Status,
            Command,
            (byte)((Value >> 24) & 0xFF),
            (byte)((Value >> 16) & 0xFF),
            (byte)((Value >> 8) & 0xFF),
            (byte)(Value & 0xFF),
            Checksum
        };
    }

    public static TmclReplyPacket FromBytes(ReadOnlySpan<byte> bytes)
    {
        int val = (bytes[4] << 24) | (bytes[5] << 16) | (bytes[6] << 8) | bytes[7];
        return new TmclReplyPacket(bytes[0], bytes[1], (TmclStatus)bytes[2], bytes[3], val);
    }

    public string ToHexString() => BitConverter.ToString(ToBytes()).Replace("-", " ");

    public string ToDescription()
    {
        if (RawOverrideBytes != null)
        {
            string verStr = System.Text.Encoding.ASCII.GetString(RawOverrideBytes, 1, 8);
            return $"FirmwareString=\"{verStr}\"";
        }
        return $"Status={Status} ({StatusEnum}), Cmd={Command} ({(TmclCommand)Command}), Val={Value}";
    }
}
