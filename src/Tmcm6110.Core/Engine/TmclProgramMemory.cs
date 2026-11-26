using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Represents a single 7-byte TMCL standalone program instruction stored in EEPROM program memory.
/// Layout: [0]=Opcode, [1]=Type, [2]=MotorOrBank, [3..6]=Value (32-bit signed big-endian).
/// </summary>
public readonly record struct TmclInstruction(byte Opcode, byte Type, byte MotorOrBank, int Value)
{
    public bool IsEmpty => Opcode == 0 && Type == 0 && MotorOrBank == 0 && Value == 0;
    public bool IsEndMarker => Opcode == 0;

    public byte[] To7Bytes() => new byte[7]
    {
        Opcode,
        Type,
        MotorOrBank,
        (byte)((Value >> 24) & 0xFF),
        (byte)((Value >> 16) & 0xFF),
        (byte)((Value >> 8) & 0xFF),
        (byte)(Value & 0xFF)
    };

    public static TmclInstruction From7Bytes(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < 7)
            throw new ArgumentException("TMCL instruction must be at least 7 bytes.", nameof(bytes));

        int value = (bytes[3] << 24) | (bytes[4] << 16) | (bytes[5] << 8) | bytes[6];
        return new TmclInstruction(bytes[0], bytes[1], bytes[2], value);
    }

    public string ToHex7String() =>
        $"{Opcode:X2} {Type:X2} {MotorOrBank:X2} {(byte)((Value >> 24) & 0xFF):X2} {(byte)((Value >> 16) & 0xFF):X2} {(byte)((Value >> 8) & 0xFF):X2} {(byte)(Value & 0xFF):X2}";

    public string ToMnemonic(Func<int, string>? axisLabelResolver = null) =>
        Opcode == 0
            ? "END (0x00)"
            : new TmclCommandPacket(1, Opcode, Type, MotorOrBank, Value).ToMnemonicDescription(axisLabelResolver);
}

/// <summary>
/// Disassembled view of a single slot in TMCL program memory.
/// </summary>
public readonly record struct TmclProgramLine(
    int Address,
    TmclInstruction Instruction,
    string Hex7,
    string Mnemonic,
    bool IsCurrentPc);

/// <summary>
/// Emulates the TMCM-6110 standalone TMCL EEPROM program memory (default 2048 instruction slots).
/// </summary>
public sealed class TmclProgramMemory
{
    public const int DefaultCapacity = 2048;

    private readonly TmclInstruction[] _slots;

    public int Capacity => _slots.Length;
    public bool IsDownloadMode { get; private set; }
    public int WritePointer { get; private set; }
    public int HighestWrittenAddress { get; private set; } = -1;

    public TmclProgramMemory()
        : this(DefaultCapacity)
    {
    }

    public TmclProgramMemory(int capacity)
    {
        if (capacity <= 0)
            throw new ArgumentOutOfRangeException(nameof(capacity), "Program memory capacity must be positive.");
        _slots = new TmclInstruction[capacity];
    }

    /// <summary>
    /// Number of active instructions starting from address 0 up to the first Opcode == 0 end marker
    /// or HighestWrittenAddress + 1. Returns 0 when empty.
    /// </summary>
    public int InstructionCount
    {
        get
        {
            if (HighestWrittenAddress < 0)
                return 0;

            int limit = Math.Min(HighestWrittenAddress + 1, _slots.Length);
            for (int i = 0; i < limit; i++)
            {
                if (_slots[i].Opcode == 0)
                    return i;
            }
            return limit;
        }
    }

    public void StartDownload(int startAddress)
    {
        if (startAddress < 0 || startAddress >= Capacity)
            throw new ArgumentOutOfRangeException(nameof(startAddress), $"Start address must be in 0..{Capacity - 1}.");

        IsDownloadMode = true;
        WritePointer = startAddress;

        // Clearing on startAddress == 0 prevents stale tail instructions from leaking past a shorter new program.
        if (startAddress == 0)
        {
            Array.Clear(_slots);
            HighestWrittenAddress = -1;
        }
    }

    public bool TryWriteNext(TmclInstruction instruction, out int writtenAddress)
    {
        if (!IsDownloadMode || WritePointer < 0 || WritePointer >= Capacity)
        {
            writtenAddress = 0;
            return false;
        }

        writtenAddress = WritePointer;
        _slots[writtenAddress] = instruction;
        if (writtenAddress > HighestWrittenAddress)
            HighestWrittenAddress = writtenAddress;

        WritePointer++;
        return true;
    }

    public void QuitDownload()
    {
        IsDownloadMode = false;
    }

    public bool TryRead(int address, out TmclInstruction instruction)
    {
        if (address < 0 || address >= Capacity)
        {
            instruction = default;
            return false;
        }

        instruction = _slots[address];
        return true;
    }

    public void Clear()
    {
        Array.Clear(_slots);
        HighestWrittenAddress = -1;
        WritePointer = 0;
        IsDownloadMode = false;
    }

    public IReadOnlyList<TmclProgramLine> GetDisassembly(int currentPc, Func<int, string>? axisLabelResolver = null)
    {
        if (HighestWrittenAddress < 0)
            return Array.Empty<TmclProgramLine>();

        int count = Math.Min(HighestWrittenAddress + 1, _slots.Length);
        var lines = new List<TmclProgramLine>(count);
        for (int addr = 0; addr < count; addr++)
        {
            var instr = _slots[addr];
            lines.Add(new TmclProgramLine(
                addr,
                instr,
                instr.ToHex7String(),
                instr.ToMnemonic(axisLabelResolver),
                addr == currentPc));
        }
        return lines;
    }
}
