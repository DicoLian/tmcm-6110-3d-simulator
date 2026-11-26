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
        if (Command == 0)
            return "END_MARKER (0x00)";

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
            TmclCommand.SAPX => $"SAPX {TmclAxisParam.GetParameterName(TypeNumber)}, [X], Val={Value}",
            TmclCommand.GAPX => $"GAPX {TmclAxisParam.GetParameterName(TypeNumber)}, [X]",
            TmclCommand.AAPX => $"AAPX {TmclAxisParam.GetParameterName(TypeNumber)}, [X]",
            TmclCommand.CALC => TypeNumber == 8
                ? "CALC NOT"
                : $"CALC {GetCalcOperationName(TypeNumber)}, {Value}",
            TmclCommand.COMP => $"COMP {Value}",
            TmclCommand.JC => $"JC {GetConditionName(TypeNumber)}, Addr={Value}",
            TmclCommand.JA => $"JA Addr={Value}",
            TmclCommand.CSUB => $"CSUB Addr={Value}",
            TmclCommand.RSUB => "RSUB",
            TmclCommand.EI => TypeNumber == 255 ? "EI ALL (255)" : $"EI IRQ#{TypeNumber}",
            TmclCommand.DI => TypeNumber == 255 ? "DI ALL (255)" : $"DI IRQ#{TypeNumber}",
            TmclCommand.WAIT => TypeNumber switch
            {
                0 => Value == -1 ? "WAIT TICKS, Accu" : $"WAIT TICKS, {Value}",
                1 => $"WAIT POS, {axisLabel}, Timeout={Value}",
                2 => $"WAIT REFSW, {axisLabel}, Timeout={Value}",
                3 => $"WAIT LIMSW, {axisLabel}, Timeout={Value}",
                4 => $"WAIT RFS, {axisLabel}, Timeout={Value}",
                _ => $"WAIT Type={TypeNumber}, {axisLabel}, Ticks={Value}"
            },
            TmclCommand.STOP => "STOP",
            TmclCommand.SCO => MotorOrBank == 255
                ? $"SCO Coord#{TypeNumber}, 255 (RAM->EEPROM)"
                : $"SCO Coord#{TypeNumber}, {axisLabel}, Pos={Value}",
            TmclCommand.GCO => MotorOrBank == 255
                ? $"GCO Coord#{TypeNumber}, 255 (EEPROM->RAM)"
                : $"GCO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.CCO => $"CCO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.CALCX => $"CALCX {GetCalcOperationName(TypeNumber)}",
            TmclCommand.AAP => $"AAP {TmclAxisParam.GetParameterName(TypeNumber)}, {axisLabel}",
            TmclCommand.AGP => $"AGP Param#{TypeNumber}, Bank {MotorOrBank}",
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
            TmclCommand.VECT => $"VECT IRQ#{TypeNumber}, Addr={Value}",
            TmclCommand.RETI => "RETI",
            TmclCommand.ACO => $"ACO Coord#{TypeNumber}, {axisLabel}",
            TmclCommand.CALCVV => $"CALCVV {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}, Var#{Value}",
            TmclCommand.CALCVA => $"CALCVA {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}",
            TmclCommand.CALCAV => $"CALCAV {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}",
            TmclCommand.CALCVX => $"CALCVX {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}",
            TmclCommand.CALCXV => $"CALCXV {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}",
            TmclCommand.CALCV => TypeNumber == 8
                ? $"CALCV NOT, Var#{MotorOrBank}"
                : $"CALCV {GetCalcOperationName(TypeNumber)}, Var#{MotorOrBank}, {Value}",
            TmclCommand.MVPA => TypeNumber switch
            {
                0 => $"MVPA ABS, {axisLabel}, Accu",
                1 => $"MVPA REL, {axisLabel}, Accu",
                2 => $"MVPA COORD, Mask=0x{MotorOrBank:X2}, Accu",
                _ => $"MVPA Type={TypeNumber}, {axisLabel}, Accu"
            },
            TmclCommand.MVPXA => TypeNumber switch
            {
                0 => "MVPXA ABS, [X], Accu",
                1 => "MVPXA REL, [X], Accu",
                2 => "MVPXA COORD, [X], Accu",
                _ => $"MVPXA Type={TypeNumber}, [X], Accu"
            },
            TmclCommand.RST => $"RST Addr={Value}",
            TmclCommand.DJNZ => $"DJNZ Var#{TypeNumber}, Addr={Value}",
            TmclCommand.ROLA => $"ROLA {axisLabel}, Accu",
            TmclCommand.RORA => $"RORA {axisLabel}, Accu",
            TmclCommand.ROLXA => "ROLXA [X], Accu",
            TmclCommand.RORXA => "RORXA [X], Accu",
            TmclCommand.MSTX => "MSTX [X]",
            TmclCommand.SIV => $"SIV [X], Val={Value}",
            TmclCommand.GIV => "GIV [X]",
            TmclCommand.AIV => "AIV [X]",
            TmclCommand.CALL => $"CALL {GetConditionName(TypeNumber)}, Addr={Value}",
            TmclCommand.StopApplication => "StopApplication (128)",
            TmclCommand.RunApplication => TypeNumber == 1
                ? $"RunApplication (129), Addr={Value}"
                : $"RunApplication (129), Type={TypeNumber}",
            TmclCommand.StepApplication => "StepApplication (130)",
            TmclCommand.ResetApplication => "ResetApplication (131)",
            TmclCommand.StartDownload => $"StartDownload (132), Addr={Value}",
            TmclCommand.QuitDownload => "QuitDownload (133)",
            TmclCommand.ReadTmclMemory => $"ReadTmclMemory (134), Addr={Value}",
            TmclCommand.GetAppStatus => $"GetAppStatus (135), Type={TypeNumber}",
            TmclCommand.GetFirmwareVersion => $"GetFirmwareVersion (136), Type={TypeNumber}",
            TmclCommand.RestoreFactorySettings => $"RestoreFactorySettings (137), Magic={Value}",
            TmclCommand.RequestTargetPositionReachedEvent => $"ReqPosReachedEvent (138), Type={TypeNumber} ({(TypeNumber == 1 ? "EveryMVP" : "NextMVP")}), AxisMask=0x{Value:X2}",
            TmclCommand.EnterAsciiMode => "EnterAsciiMode (139)",
            TmclCommand.SoftwareReset => $"SoftwareReset (255), Magic={Value}",
            _ => $"Cmd#{Command} ({CommandEnum}), Type={TypeNumber}, Mot/Bank={MotorOrBank}, Val={Value}"
        };
    }

    private static string GetCalcOperationName(byte op) => op switch
    {
        0 => "ADD",
        1 => "SUB",
        2 => "MUL",
        3 => "DIV",
        4 => "MOD",
        5 => "AND",
        6 => "OR",
        7 => "XOR",
        8 => "NOT",
        9 => "LOAD",
        10 => "SWAP",
        11 => "COMP",
        _ => $"Op#{op}"
    };

    private static string GetConditionName(byte cond) => cond switch
    {
        0 => "ZE",
        1 => "NZ",
        2 => "EQ",
        3 => "NE",
        4 => "GT",
        5 => "GE",
        6 => "LT",
        7 => "LE",
        8 => "ETO",
        9 => "EAL",
        10 => "EDV",
        11 => "EPO",
        _ => $"Cond#{cond}"
    };

    public bool IsPollingQuery =>
        CommandEnum == TmclCommand.GAP ||
        CommandEnum == TmclCommand.GGP ||
        CommandEnum == TmclCommand.GIO ||
        (CommandEnum == TmclCommand.RFS && TypeNumber == 2);
}

/// <summary>
/// Distinguishes special non-checksum 9-byte TMCL reply payloads.
/// </summary>
public enum TmclRawReplyKind
{
    None = 0,
    FirmwareVersionString = 1,
    ProgramMemorySlot = 2
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
    public byte[]? RawOverrideBytes { get; } // Used for Cmd 136 Type 0 string reply or Cmd 134 memory slot reply (no checksum)
    public TmclRawReplyKind RawKind { get; }
    public string? DescriptionOverride { get; }

    public TmclStatus StatusEnum => (TmclStatus)Status;

    public TmclReplyPacket(byte replyAddress, byte moduleAddress, TmclStatus status, byte command, int value)
    {
        ReplyAddress = replyAddress;
        ModuleAddress = moduleAddress;
        Status = (byte)status;
        Command = command;
        Value = value;
        RawOverrideBytes = null;
        RawKind = TmclRawReplyKind.None;
        DescriptionOverride = null;

        byte b3 = (byte)((value >> 24) & 0xFF);
        byte b2 = (byte)((value >> 16) & 0xFF);
        byte b1 = (byte)((value >> 8) & 0xFF);
        byte b0 = (byte)(value & 0xFF);

        Checksum = unchecked((byte)(replyAddress + moduleAddress + Status + command + b3 + b2 + b1 + b0));
    }

    public TmclReplyPacket(byte[] rawBytes)
        : this(rawBytes, TmclRawReplyKind.FirmwareVersionString, null)
    {
    }

    public TmclReplyPacket(byte[] rawBytes, TmclRawReplyKind rawKind, string? descriptionOverride = null)
    {
        if (rawBytes is null || rawBytes.Length != 9)
            throw new ArgumentException("Raw reply must be 9 bytes.", nameof(rawBytes));

        ReplyAddress = rawBytes[0];
        ModuleAddress = rawBytes[1];
        Status = rawBytes[2];
        Command = rawBytes[3];
        Value = rawKind == TmclRawReplyKind.ProgramMemorySlot
            ? (rawBytes[5] << 24) | (rawBytes[6] << 16) | (rawBytes[7] << 8) | rawBytes[8]
            : (rawBytes[4] << 24) | (rawBytes[5] << 16) | (rawBytes[6] << 8) | rawBytes[7];
        Checksum = rawBytes[8];
        RawOverrideBytes = rawBytes;
        RawKind = rawKind;
        DescriptionOverride = descriptionOverride;
    }

    /// <summary>
    /// Builds the 9-byte special reply for Command 134 (Read TMCL Program Memory, FW V1.47 Section 3.2 &amp; Table 14).
    /// Layout: [0]=ReplyAddr, [1]=ModuleAddr, [2]=Opcode, [3]=Type, [4]=MotorOrBank, [5..8]=Value (MSB first, no checksum).
    /// </summary>
    public static TmclReplyPacket CreateMemoryReadReply(
        byte replyAddress,
        byte moduleAddress,
        byte opcode,
        byte type,
        byte motorOrBank,
        int value,
        Func<int, string>? axisLabelResolver = null)
    {
        byte[] raw = new byte[9]
        {
            replyAddress,
            moduleAddress,
            opcode,
            type,
            motorOrBank,
            (byte)((value >> 24) & 0xFF),
            (byte)((value >> 16) & 0xFF),
            (byte)((value >> 8) & 0xFF),
            (byte)(value & 0xFF)
        };

        string mnemonic = new TmclCommandPacket(moduleAddress, opcode, type, motorOrBank, value)
            .ToMnemonicDescription(axisLabelResolver);
        return new TmclReplyPacket(raw, TmclRawReplyKind.ProgramMemorySlot, mnemonic);
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
            switch (RawKind)
            {
                case TmclRawReplyKind.FirmwareVersionString:
                {
                    string verStr = System.Text.Encoding.ASCII.GetString(RawOverrideBytes, 1, 8);
                    return $"FirmwareString=\"{verStr}\"";
                }
                case TmclRawReplyKind.ProgramMemorySlot:
                {
                    if (!string.IsNullOrWhiteSpace(DescriptionOverride))
                    {
                        return DescriptionOverride.StartsWith("MemRead:", StringComparison.Ordinal)
                            ? DescriptionOverride
                            : $"MemRead: {DescriptionOverride}";
                    }

                    int memValue = (RawOverrideBytes[5] << 24)
                                 | (RawOverrideBytes[6] << 16)
                                 | (RawOverrideBytes[7] << 8)
                                 | RawOverrideBytes[8];
                    var cmd = new TmclCommandPacket(
                        RawOverrideBytes[1],
                        RawOverrideBytes[2],
                        RawOverrideBytes[3],
                        RawOverrideBytes[4],
                        memValue);
                    return $"MemRead: {cmd.ToMnemonicDescription()}";
                }
                default:
                    if (!string.IsNullOrWhiteSpace(DescriptionOverride))
                        return DescriptionOverride;
                    break;
            }
        }
        return $"Status={Status} ({StatusEnum}), Cmd={Command} ({(TmclCommand)Command}), Val={Value}";
    }
}
