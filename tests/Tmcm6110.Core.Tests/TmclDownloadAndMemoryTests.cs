using System.Text;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class TmclDownloadAndMemoryTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value, byte moduleAddr = 1) =>
        new(moduleAddr, (byte)command, type, motorOrBank, value);

    private static void DownloadProgram(Tmcm6110Controller controller, params TmclCommandPacket[] instructions)
    {
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        for (int i = 0; i < instructions.Length; i++)
        {
            var reply = controller.ProcessCommand(instructions[i]);
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, reply.StatusEnum);
            Assert.Equal(i, reply.Value);
        }

        var endReply = controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0));
        Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, endReply.StatusEnum);
        Assert.Equal(instructions.Length, endReply.Value);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);
    }

    [Fact]
    public void FullTmclIdeDownloadFlow_StoresInstructionsWithoutExecutingAndUpdatesGgp129And130()
    {
        var controller = new Tmcm6110Controller();

        // 1. Stop application, then enter download mode at address 0
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 129, 0, 0)).Value);

        long revBefore = controller.ProgramRevision;
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        Assert.True(controller.ProgramRevision > revBefore);

        // Verify Global Param #129 == 1 and #130 (WritePointer) == 0 while in download mode
        // (Note: wire GGP opcode 10 < 128 would itself be stored into EEPROM while in download mode!)
        Assert.Equal(1, controller.GetGlobalParameter(0, 129));
        Assert.Equal(0, controller.GetGlobalParameter(0, 130));
        Assert.True(controller.GetStandaloneSnapshot().IsDownloadMode);

        // Control commands (135, 136) still execute normally during download mode
        var appStatusReply = controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 0, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, appStatusReply.StatusEnum);
        Assert.Equal((int)TmclAppStatus.Stop, appStatusReply.Value);

        var fwBinReply = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 1, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, fwBinReply.StatusEnum);
        Assert.Equal(0x17DE012F, fwBinReply.Value);

        var fwAsciiReply = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 0, 0, 0));
        Assert.Equal("6110V147", Encoding.ASCII.GetString(fwAsciiReply.ToBytes(), 1, 8));

        // 2. Download MVP, WAIT, JA, STOP, and Opcode 0 end marker
        var program = new[]
        {
            Cmd(TmclCommand.MVP, 0, 2, 1500),   // addr 0
            Cmd(TmclCommand.WAIT, 1, 2, 0),     // addr 1
            Cmd(TmclCommand.JA, 0, 0, 3),       // addr 2
            Cmd(TmclCommand.STOP, 0, 0, 0),     // addr 3
            new TmclCommandPacket(1, 0, 0, 0, 0) // addr 4: Opcode 0 end marker
        };

        for (int i = 0; i < program.Length; i++)
        {
            var reply = controller.ProcessCommand(program[i]);
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, reply.StatusEnum);
            Assert.Equal((byte)101, reply.Status);
            Assert.Equal(program[i].Command, reply.Command);
            Assert.Equal(i, reply.Value);
            Assert.Equal(i + 1, controller.GetGlobalParameter(0, 130));
        }

        // Verify MVP was NOT executed during download (TargetPosition and ActualPosition stay 0)
        Assert.Equal(0, controller.GetAxis(2).GetParameter(TmclAxisParam.TargetPosition, clearReadFlags: false));
        Assert.Equal(0, controller.GetAxis(2).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        for (int i = 0; i < 50; i++)
            controller.StepSimulation(0.01);
        Assert.Equal(0, controller.GetAxis(2).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));

        // 3. Quit download mode -> GGP 129 becomes 0 and PC resets to 0
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 129, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 130, 0, 0)).Value);

        var snapshot = controller.GetStandaloneSnapshot();
        Assert.False(snapshot.IsDownloadMode);
        Assert.Equal(4, snapshot.InstructionCount); // slots 0..3 before Opcode 0 at slot 4
        Assert.Equal(0, snapshot.ProgramCounter);
    }

    [Fact]
    public void ReadTmclMemory_Cmd134_ReturnsExact9ByteSpecialReplyAndValidatesRange()
    {
        var controller = new Tmcm6110Controller
        {
            HostAddress = 0x02,
            ModuleAddress = 0x01
        };

        var mvp = Cmd(TmclCommand.MVP, 0, 2, -123456);
        var wait = Cmd(TmclCommand.WAIT, 0, 0, 25);
        var stop = Cmd(TmclCommand.STOP, 0, 0, 0);
        DownloadProgram(controller, mvp, wait, stop);

        // Read slot 0 (MVP ABS, Axis 2, -123456)
        var read0 = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0));
        byte[] raw0 = read0.ToBytes();
        Assert.Equal(9, raw0.Length);
        Assert.Equal(TmclRawReplyKind.ProgramMemorySlot, read0.RawKind);
        Assert.Equal(0x02, raw0[0]); // Host address
        Assert.Equal(0x01, raw0[1]); // Module address
        Assert.Equal((byte)TmclCommand.MVP, raw0[2]);
        Assert.Equal(0x00, raw0[3]);
        Assert.Equal(0x02, raw0[4]);
        Assert.Equal(unchecked((byte)((-123456 >> 24) & 0xFF)), raw0[5]);
        Assert.Equal(unchecked((byte)((-123456 >> 16) & 0xFF)), raw0[6]);
        Assert.Equal(unchecked((byte)((-123456 >> 8) & 0xFF)), raw0[7]);
        Assert.Equal(unchecked((byte)(-123456 & 0xFF)), raw0[8]);
        Assert.Equal(-123456, read0.Value);
        Assert.StartsWith("MemRead:", read0.ToDescription());
        Assert.Contains("MVP ABS", read0.ToDescription());

        // Reconstructing from raw wire bytes with ProgramMemorySlot yields matching MemRead description
        var decoded0 = new TmclReplyPacket(raw0, TmclRawReplyKind.ProgramMemorySlot);
        Assert.Equal(-123456, decoded0.Value);
        Assert.StartsWith("MemRead:", decoded0.ToDescription());

        // Read slot 3 (Opcode 0 end marker)
        var read3 = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 3));
        byte[] raw3 = read3.ToBytes();
        Assert.Equal(0, raw3[2]);
        Assert.StartsWith("MemRead:", read3.ToDescription());

        // Out-of-range addresses return InvalidValue
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, -1)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, TmclProgramMemory.DefaultCapacity)).StatusEnum);
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(1, 2)]
    [InlineData(3, 0)]
    [InlineData(3, 2)]
    public void GlobalParam81_CodeProtection_BlocksReadWriteAndErasesOnReadUnprotect(int protectedMode, int unprotectedMode)
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(
            controller,
            Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 0, 1200),
            Cmd(TmclCommand.MVP, 0, 0, 500),
            Cmd(TmclCommand.STOP, 0, 0, 0));

        // Enable protection mode 1 or 3
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 81, 0, protectedMode)).StatusEnum);

        // Mode 1 blocks Cmd 134 (ReadTmclMemory) but allows Cmd 132 (StartDownload)
        // Mode 3 blocks BOTH Cmd 134 and Cmd 132
        var readBlocked = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0));
        Assert.Equal(TmclStatus.ConfigurationEepromLocked, readBlocked.StatusEnum);

        var startDownloadReply = controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0));
        if ((protectedMode & 2) != 0)
        {
            Assert.Equal(TmclStatus.ConfigurationEepromLocked, startDownloadReply.StatusEnum);
        }
        else
        {
            Assert.Equal(TmclStatus.SuccessOk, startDownloadReply.StatusEnum);
            controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 900));
            controller.ProcessCommand(Cmd(TmclCommand.STOP, 0, 0, 0));
            controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0));
            controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0));
        }

        Assert.True(controller.GetStandaloneSnapshot().InstructionCount > 0);

        // Changing #81 from 1/3 to 0/2 MUST erase downloaded program memory per FW V1.47 Table 18
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 81, 0, unprotectedMode)).StatusEnum);
        Assert.Equal(0, controller.GetStandaloneSnapshot().InstructionCount);
        Assert.Empty(controller.GetDisassembledProgram());

        var readAfterUnprotect = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0));
        Assert.Equal(0, readAfterUnprotect.ToBytes()[2]); // Opcode == 0 (erased)
    }

    [Fact]
    public void GlobalParam81_Mode2BlocksStartDownloadWithoutErasingOnUnprotect()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(
            controller,
            Cmd(TmclCommand.MVP, 0, 1, 750),
            Cmd(TmclCommand.STOP, 0, 0, 0));

        // Mode 2: write-protected only (blocks Cmd 132, allows Cmd 134)
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 81, 0, 2)).StatusEnum);
        Assert.Equal(TmclStatus.ConfigurationEepromLocked, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);

        var read0 = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0));
        Assert.Equal((byte)TmclCommand.MVP, read0.ToBytes()[2]);
        Assert.Equal(750, read0.Value);

        // Clearing mode 2 -> 0 does NOT erase memory because read protection was not set
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 81, 0, 0)).StatusEnum);
        Assert.Equal(2, controller.GetStandaloneSnapshot().InstructionCount);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);
    }

    [Fact]
    public void SoftwareReset255_PreservesProgramMemory_WhileRestoreFactory137_ErasesIt()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(
            controller,
            Cmd(TmclCommand.MVP, 0, 0, 321),
            Cmd(TmclCommand.STOP, 0, 0, 0));

        // Even if download mode was left active, Cmd 255 exits download mode and preserves memory
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 2)).StatusEnum);
        Assert.Equal(1, controller.GetGlobalParameter(0, 129));

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 129, 0, 0)).Value);
        Assert.Equal(2, controller.GetStandaloneSnapshot().InstructionCount);

        var read0 = controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0));
        Assert.Equal((byte)TmclCommand.MVP, read0.ToBytes()[2]);
        Assert.Equal(321, read0.Value);

        // Cmd 137 (RestoreFactorySettings, 1234) erases downloaded program memory and resets #81
        controller.ProcessCommand(Cmd(TmclCommand.SGP, 81, 0, 3));
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RestoreFactorySettings, 0, 0, 1234)).StatusEnum);
        Assert.Equal(0, controller.GetStandaloneSnapshot().InstructionCount);
        Assert.Empty(controller.GetDisassembledProgram());
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 81, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 0)).ToBytes()[2]);
    }

    [Fact]
    public void TmclProtocolAdapter_RawByteStreamDownloadAndCmd134Readback_RoundtripsExactly()
    {
        var controller = new Tmcm6110Controller
        {
            HostAddress = 0x02,
            ModuleAddress = 0x01
        };
        using var adapter = new TmclProtocolAdapter(controller);

        var downloadFrames = new[]
        {
            Cmd(TmclCommand.StopApplication, 0, 0, 0),
            Cmd(TmclCommand.StartDownload, 0, 0, 0),
            Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 0, 1400),
            Cmd(TmclCommand.MVP, 0, 0, 2500),
            Cmd(TmclCommand.WAIT, 1, 0, 0),
            Cmd(TmclCommand.STOP, 0, 0, 0),
            new TmclCommandPacket(1, 0, 0, 0, 0),
            Cmd(TmclCommand.QuitDownload, 0, 0, 0)
        };

        var rxBuffer = new List<byte>();
        foreach (var pkt in downloadFrames)
            rxBuffer.AddRange(pkt.ToBytes());

        var replies = new List<byte[]>();
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));

        Assert.Empty(rxBuffer);
        Assert.Equal(downloadFrames.Length, replies.Count);

        // Replies 0 (Cmd 128), 1 (Cmd 132), 7 (Cmd 133) -> Status=100; Replies 2..6 -> Status=101
        Assert.Equal(TmclStatus.SuccessOk, TmclReplyPacket.FromBytes(replies[0]).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, TmclReplyPacket.FromBytes(replies[1]).StatusEnum);
        for (int i = 2; i <= 6; i++)
        {
            var r = TmclReplyPacket.FromBytes(replies[i]);
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, r.StatusEnum);
            Assert.Equal(i - 2, r.Value);
        }
        Assert.Equal(TmclStatus.SuccessOk, TmclReplyPacket.FromBytes(replies[7]).StatusEnum);

        // Read back slot 1 (MVP ABS, Axis 0, 2500) over raw byte stream
        replies.Clear();
        rxBuffer.AddRange(Cmd(TmclCommand.ReadTmclMemory, 0, 0, 1).ToBytes());
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));

        Assert.Single(replies);
        byte[] memBytes = replies[0];
        Assert.Equal(9, memBytes.Length);
        Assert.Equal(0x02, memBytes[0]);
        Assert.Equal(0x01, memBytes[1]);
        Assert.Equal((byte)TmclCommand.MVP, memBytes[2]);
        Assert.Equal(0x00, memBytes[3]);
        Assert.Equal(0x00, memBytes[4]);
        int decodedVal = (memBytes[5] << 24) | (memBytes[6] << 16) | (memBytes[7] << 8) | memBytes[8];
        Assert.Equal(2500, decodedVal);

        var memReply = new TmclReplyPacket(memBytes, TmclRawReplyKind.ProgramMemorySlot);
        Assert.StartsWith("MemRead:", memReply.ToDescription());
    }

    [Fact]
    public void ProgramMemory_DisassemblyAndTailClearOnRestartAtAddressZero_WorkCorrectly()
    {
        var memory = new TmclProgramMemory();
        Assert.Equal(TmclProgramMemory.DefaultCapacity, memory.Capacity);
        Assert.Equal(0, memory.InstructionCount);
        Assert.False(memory.TryWriteNext(new TmclInstruction(1, 0, 0, 100), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => memory.StartDownload(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => memory.StartDownload(memory.Capacity));

        // Write 4 instructions + end marker
        memory.StartDownload(0);
        for (int i = 0; i < 4; i++)
            Assert.True(memory.TryWriteNext(new TmclInstruction((byte)TmclCommand.MVP, 0, 0, (i + 1) * 100), out _));
        Assert.True(memory.TryWriteNext(default, out _));
        memory.QuitDownload();
        Assert.Equal(4, memory.InstructionCount);

        // Re-downloading from 0 clears previous tail instructions
        memory.StartDownload(0);
        Assert.True(memory.TryWriteNext(new TmclInstruction((byte)TmclCommand.STOP, 0, 0, 0), out int written));
        Assert.Equal(0, written);
        Assert.True(memory.TryWriteNext(default, out _));
        memory.QuitDownload();
        Assert.Equal(1, memory.InstructionCount);

        var disasm = memory.GetDisassembly(currentPc: 0, ax => $"Axis {ax}");
        Assert.Equal(2, disasm.Count);
        Assert.True(disasm[0].IsCurrentPc);
        Assert.Equal("STOP", disasm[0].Mnemonic);
        Assert.Equal("END (0x00)", disasm[1].Mnemonic);
        Assert.Equal("1C 00 00 00 00 00 00", disasm[0].Hex7);

        var roundtrip = TmclInstruction.From7Bytes(disasm[0].Instruction.To7Bytes());
        Assert.Equal(disasm[0].Instruction, roundtrip);
        Assert.True(disasm[1].Instruction.IsEmpty);
        Assert.True(disasm[1].Instruction.IsEndMarker);
    }
}
