using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class TmclAssemblerTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value, byte moduleAddr = 1) =>
        new(moduleAddr, (byte)command, type, motorOrBank, value);

    [Fact]
    public void Assemble_SupportsCommentsConstantsNumericFormatsLabelsAndSymbolicNames()
    {
        string source = """
            // Full-line slash comment
            ; Full-line semicolon comment
            #define AXIS_X      2
            TARGET_POS = 0x1000         // Hex 0x prefix (4096)
            HEX_DOLLAR = $2A            ; Hex $ prefix (42)
            NEG_OFFSET = -500

            Start:
                SAP MaxPositioningSpeed, AXIS_X, 1800
                MVP ABS, AXIS_X, TARGET_POS
                WAIT POS, AXIS_X, 0
                JA ForwardTarget
            BackwardTarget: MVP REL, AXIS_X, NEG_OFFSET ; Prefixed label on instruction line
                CALC LOAD, HEX_DOLLAR
                COMP 42
                JC EQ, Done
            ForwardTarget:
                SGP 0, 2, 1
                JA BackwardTarget
            Done:
                CLE ETO
                STOP
            """;

        var result = TmclAssembler.Assemble(source, moduleAddress: 3);

        Assert.True(result.Success, string.Join(Environment.NewLine, result.Errors));
        Assert.Empty(result.Errors);
        Assert.Equal(12, result.Packets.Count);

        // Verify label addresses (case-insensitive)
        Assert.Equal(0, result.Labels["start"]);
        Assert.Equal(4, result.Labels["BackwardTarget"]);
        Assert.Equal(8, result.Labels["ForwardTarget"]);
        Assert.Equal(10, result.Labels["Done"]);

        // Verify packet contents & module address
        Assert.All(result.Packets, p => Assert.Equal((byte)3, p.ModuleAddress));
        Assert.Equal((byte)TmclCommand.SAP, result.Packets[0].Command);
        Assert.Equal(TmclAxisParam.MaxPositioningSpeed, result.Packets[0].TypeNumber);
        Assert.Equal(2, result.Packets[0].MotorOrBank);
        Assert.Equal(1800, result.Packets[0].Value);

        Assert.Equal(4096, result.Packets[1].Value);   // 0x1000
        Assert.Equal(8, result.Packets[3].Value);      // ForwardTarget
        Assert.Equal(-500, result.Packets[4].Value);   // NEG_OFFSET
        Assert.Equal(42, result.Packets[5].Value);     // $2A
        Assert.Equal((byte)2, result.Packets[7].TypeNumber); // JC EQ
        Assert.Equal(10, result.Packets[7].Value);     // Done
        Assert.Equal(4, result.Packets[9].Value);      // BackwardTarget
        Assert.Equal((byte)1, result.Packets[10].TypeNumber); // CLE ETO
        Assert.Equal((byte)TmclCommand.STOP, result.Packets[11].Command);
    }

    [Fact]
    public void Assemble_SupportsAllFwV147OpcodesIncludingGiv56Aiv57Siv55Call80()
    {
        string source = """
            Entry:
                ROR 0, 100
                ROL 0, 100
                MST 0
                MVP COORD, 0x47, 1
                SAP AP_ActualPosition, 0, 0
                GAP TargetPos, 0
                STAP MaxAccel, 0
                RSAP MaxAccel, 0
                SGP 0, 2, 10
                GGP 0, 2
                STGP 0, 2
                RSGP 0, 2
                RFS START, 0
                SIO 0, 2, 1
                GIO 0, 0
                SAPX MaxSpeed, 1500
                GAPX MaxSpeed
                AAPX MaxSpeed
                CALC ADD, 5
                CALC NOT
                COMP 10
                JC ZE, Entry
                JA Entry
                CSUB Sub1
                EI 255
                DI 0
                WAIT TICKS, 0, 10
                SCO 1, 0, 200
                GCO 1, 0
                CCO 1, 0
                CALCX SWAP
                AAP MaxSpeed, 0
                AGP 1, 2
                CLE ALL
                VECT 0, Isr
                ACO 1, 0
                CALCVV ADD, 0, 1
                CALCVA SUB, 0
                CALCAV MUL, 0
                CALCVX DIV, 0
                CALCXV MOD, 0
                CALCV OR, 0, 3
                CALCV NOT, 0
                MVPA ABS, 0
                MVPXA REL
                ROLA 0
                RORA 0
                ROLXA
                RORXA
                MSTX
                SIV 123
                GIV
                AIV
                CALL NE, Sub1
                DJNZ 0, Entry
                RST Entry
                STOP
            Sub1:
                RSUB
            Isr:
                RETI
            """;

        var result = TmclAssembler.Assemble(source);
        Assert.True(result.Success, string.Join(Environment.NewLine, result.Errors));

        byte[] expectedOpcodes =
        {
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            16, 17, 18, 19, 19, 20, 21, 22, 23, 25, 26, 27, 30, 31, 32,
            33, 34, 35, 36, 37, 39, 40, 41, 42, 43, 44, 45, 45, 46, 47,
            50, 51, 52, 53, 54, 55, 56, 57, 80, 49, 48, 28, 24, 38
        };

        Assert.Equal(expectedOpcodes.Length, result.Packets.Count);
        for (int i = 0; i < expectedOpcodes.Length; i++)
            Assert.Equal(expectedOpcodes[i], result.Packets[i].Command);

        // Explicit check on FW V1.47 manual typo-prone opcodes
        Assert.Equal((byte)55, result.Packets[50].Command); // SIV
        Assert.Equal((byte)56, result.Packets[51].Command); // GIV
        Assert.Equal((byte)57, result.Packets[52].Command); // AIV
        Assert.Equal((byte)80, result.Packets[53].Command); // CALL
    }

    [Fact]
    public void Assemble_ReportsLineNumberedDiagnosticsForErrors()
    {
        var nullResult = TmclAssembler.Assemble(null!);
        Assert.False(nullResult.Success);
        Assert.NotEmpty(nullResult.Errors);

        string badSource = """
            MVP ABS, 0, 100
            DupLabel:
            DupLabel:
            UNKNOWN_CMD 1, 2
            JA MissingTarget
            SAP MaxPositioningSpeed, 300, 1000
            BAD_CONST = not_a_number
            STOP 123
            """;

        var result = TmclAssembler.Assemble(badSource);
        Assert.False(result.Success);
        Assert.Empty(result.Packets);
        Assert.Contains(result.Errors, e => e.Contains("Line 3:") && e.Contains("Duplicate label"));
        Assert.Contains(result.Errors, e => e.Contains("Line 4:") && e.Contains("Unknown TMCL mnemonic"));
        Assert.Contains(result.Errors, e => e.Contains("Line 5:") && e.Contains("MissingTarget"));
        Assert.Contains(result.Errors, e => e.Contains("Line 6:") && e.Contains("byte range"));
        Assert.Contains(result.Errors, e => e.Contains("Line 7:") && e.Contains("BAD_CONST"));
        Assert.Contains(result.Errors, e => e.Contains("Line 8:") && e.Contains("'STOP'"));
    }

    [Fact]
    public void DemoStandaloneTmc_AssemblesDownloadsRunsToCompletionAndTogglesDout0Heartbeat()
    {
        string demoPath = FindRepoFile("samples", "demo_standalone.tmc");
        string source = File.ReadAllText(demoPath);

        Assert.DoesNotContain("#define", source, StringComparison.OrdinalIgnoreCase);
        Assert.DoesNotContain("DJNZ", source, StringComparison.OrdinalIgnoreCase);

        var asm = TmclAssembler.Assemble(source);
        Assert.True(asm.Success, string.Join(Environment.NewLine, asm.Errors));
        Assert.True(asm.Packets.Count > 0);
        Assert.True(asm.Labels.ContainsKey("CycleLoop"));
        Assert.True(asm.Labels.ContainsKey("MoveSafeZ"));
        Assert.True(asm.Labels.ContainsKey("TimerHeartbeat"));

        var controller = new Tmcm6110Controller();

        // Standard TMCL-IDE download flow: 128 -> 132 -> packets -> Opcode 0 -> 133
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        foreach (var pkt in asm.Packets)
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(pkt).StatusEnum);
        Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);

        Assert.Equal(asm.Packets.Count, controller.GetStandaloneSnapshot().InstructionCount);
        var disasm = controller.GetDisassembledProgram();
        Assert.Equal(asm.Packets.Count + 1, disasm.Count);

        // Run from address 0
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);

        bool sawXPositive = false;
        bool sawYPositive = false;
        bool sawZNegative = false;
        bool sawDout0High = false;
        bool sawDout0LowAfterHigh = false;

        for (int i = 0; i < 4000; i++)
        {
            controller.StepSimulation(0.01);

            int posZ = controller.GetAxis(0).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false);
            int posY = controller.GetAxis(1).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false);
            int posX = controller.GetAxis(2).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false);
            bool dout0 = controller.GetDigitalOutput(0);

            if (posX > 10_000) sawXPositive = true;
            if (posY > 10_000) sawYPositive = true;
            if (posZ < -5_000) sawZNegative = true;
            if (dout0) sawDout0High = true;
            else if (sawDout0High) sawDout0LowAfterHigh = true;

            if (controller.GetStandaloneSnapshot().Status == TmclAppStatus.Stop)
                break;
        }

        var finalSnap = controller.GetStandaloneSnapshot();
        Assert.Equal(TmclAppStatus.Stop, finalSnap.Status);
        Assert.True(finalSnap.ExecutedInstructionCount > 0);
        Assert.True(sawXPositive);
        Assert.True(sawYPositive);
        Assert.True(sawZNegative);
        Assert.True(sawDout0High);
        Assert.True(sawDout0LowAfterHigh);

        // Clean shutdown: axes parked at origin, loop counter Var #0 == 0, DOUT 0 off
        Assert.Equal(0, controller.GetAxis(0).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        Assert.Equal(0, controller.GetAxis(1).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        Assert.Equal(0, controller.GetAxis(2).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        Assert.False(controller.GetDigitalOutput(0));
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
    }

    private static string FindRepoFile(params string[] relativeSegments)
    {
        string relativePath = Path.Combine(relativeSegments);
        DirectoryInfo? dir = new(AppContext.BaseDirectory);
        while (dir != null)
        {
            string candidate = Path.Combine(dir.FullName, relativePath);
            if (File.Exists(candidate))
                return candidate;
            dir = dir.Parent;
        }

        if (File.Exists(relativePath))
            return Path.GetFullPath(relativePath);

        throw new FileNotFoundException($"Could not locate repository file '{relativePath}' from '{AppContext.BaseDirectory}'.");
    }
}
