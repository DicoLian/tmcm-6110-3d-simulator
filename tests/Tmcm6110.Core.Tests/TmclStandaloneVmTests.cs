using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class TmclStandaloneVmTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value, byte moduleAddr = 1) =>
        new(moduleAddr, (byte)command, type, motorOrBank, value);

    private static void DownloadAndRun(Tmcm6110Controller controller, string source)
    {
        var asm = TmclAssembler.Assemble(source);
        Assert.True(asm.Success, string.Join(Environment.NewLine, asm.Errors));

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        foreach (var pkt in asm.Packets)
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(pkt).StatusEnum);
        Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);
    }

    private static void StepUntilStopped(Tmcm6110Controller controller, int maxSteps = 1000, double dtSeconds = 0.01)
    {
        for (int i = 0; i < maxSteps; i++)
        {
            controller.StepSimulation(dtSeconds);
            if (controller.GetStandaloneSnapshot().Status == TmclAppStatus.Stop)
                return;
        }
        Assert.Equal(TmclAppStatus.Stop, controller.GetStandaloneSnapshot().Status);
    }

    [Fact]
    public void StandaloneMvpWaitPosAndWaitTicks10msPerTick_ExecuteEndToEndAndStop()
    {
        var controller = new Tmcm6110Controller();
        DownloadAndRun(controller, """
            SAP MaxPositioningSpeed, 0, 2000
            MVP ABS, 0, 600
            WAIT POS, 0, 0
            SGP 0, 2, 111
            MVP ABS, 0, -300
            WAIT POS, 0, 0
            WAIT TICKS, 0, 5
            SGP 0, 2, 222
            STOP
            """);

        // Run until first move completes and program reaches WAIT TICKS, 0, 5 (50 ms)
        bool reachedWaitTicks = false;
        for (int i = 0; i < 300; i++)
        {
            controller.StepSimulation(0.01);
            var snap = controller.GetStandaloneSnapshot();
            if (snap.IsWaiting && snap.WaitDescription.StartsWith("TICKS", StringComparison.Ordinal))
            {
                reachedWaitTicks = true;
                break;
            }
        }

        Assert.True(reachedWaitTicks);
        Assert.Equal(-300, controller.GetAxis(0).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        Assert.Equal(111, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);

        // Verify 10ms/tick: first 4 x 10ms = 40ms keeps waiting; 5th x 10ms = 50ms finishes WAIT & executes STOP
        for (int i = 0; i < 4; i++)
        {
            controller.StepSimulation(0.01);
            var snap = controller.GetStandaloneSnapshot();
            Assert.Equal(TmclAppStatus.Run, snap.Status);
            Assert.True(snap.IsWaiting);
            Assert.Equal(111, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        }

        controller.StepSimulation(0.01);
        var finalSnap = controller.GetStandaloneSnapshot();
        Assert.Equal(TmclAppStatus.Stop, finalSnap.Status);
        Assert.False(finalSnap.IsWaiting);
        Assert.Equal(222, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
    }

    [Fact]
    public void DirectModeGapGgpGioGco_DoNotOverwriteStandaloneAccumulator()
    {
        var controller = new Tmcm6110Controller();
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.ActualPosition, 0, 123));
        controller.ProcessCommand(Cmd(TmclCommand.SGP, 10, 2, 456));
        controller.SetAnalogInput(0, 3333);
        controller.ProcessCommand(Cmd(TmclCommand.SCO, 1, 0, 789));

        DownloadAndRun(controller, """
            CALC LOAD, 4242
            WAIT TICKS, 0, 10
            AGP 11, 2
            STOP
            """);

        // Tick once so CALC LOAD, 4242 runs and VM enters WAIT TICKS (100 ms)
        controller.StepSimulation(0.01);
        Assert.Equal(4242, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);

        // Issue Direct Mode GAP, GGP, GIO, GCO, RFS STATUS while VM is paused in WAIT
        Assert.Equal(123, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value);
        Assert.Equal(456, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0)).Value);
        Assert.Equal(3333, controller.ProcessCommand(Cmd(TmclCommand.GIO, 0, 1, 0)).Value);
        Assert.Equal(789, controller.ProcessCommand(Cmd(TmclCommand.GCO, 1, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.RFS, 2, 0, 0)).Value);

        // Accumulator MUST remain 4242
        Assert.Equal(4242, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);

        StepUntilStopped(controller, 50);
        Assert.Equal(4242, controller.ProcessCommand(Cmd(TmclCommand.GGP, 11, 2, 0)).Value);
    }

    [Fact]
    public void CalcCalcxCompJcAndWaitTimeoutEtoCle_ExecuteAllVariants()
    {
        var controller = new Tmcm6110Controller();
        DownloadAndRun(controller, """
            // CALC 0..9 -> store snapshots into Bank 2 vars 0..9
            CALC LOAD, 10
            AGP 9, 2
            CALC ADD, 5
            AGP 0, 2
            CALC SUB, 3
            AGP 1, 2
            CALC MUL, 4
            AGP 2, 2
            CALC DIV, 6
            AGP 3, 2
            CALC MOD, 5
            AGP 4, 2
            CALC LOAD, 12
            CALC AND, 10
            AGP 5, 2
            CALC OR, 5
            AGP 6, 2
            CALC XOR, 3
            AGP 7, 2
            CALC NOT
            AGP 8, 2

            // CALCX 0..10
            CALC LOAD, 7
            CALCX LOAD
            CALC LOAD, 20
            CALCX ADD
            AGP 10, 2
            CALCX SUB
            AGP 11, 2
            CALCX SWAP
            AGP 12, 2
            CALCX SWAP
            CALC LOAD, 6
            CALCX MUL
            AGP 13, 2
            CALCX DIV
            AGP 14, 2
            CALC LOAD, 17
            CALCX MOD
            AGP 15, 2
            CALC LOAD, 15
            CALCX AND
            AGP 16, 2
            CALCX OR
            AGP 17, 2
            CALCX XOR
            AGP 18, 2
            CALCX NOT
            AGP 19, 2

            // COMP & JC (ZE/NZ/EQ/NE/GT/GE/LT/LE)
            CALC LOAD, 0
            JC ZE, OkZe
            JA Fail
            OkZe:
                CALC LOAD, 5
                JC NZ, OkNz
                JA Fail
            OkNz:
                COMP 5
                JC EQ, OkEq
                JA Fail
            OkEq:
                JC GE, OkGeEq
                JA Fail
            OkGeEq:
                JC LE, OkLeEq
                JA Fail
            OkLeEq:
                COMP 3
                JC NE, OkNe
                JA Fail
            OkNe:
                JC GT, OkGt
                JA Fail
            OkGt:
                COMP 9
                JC LT, OkLt
                JA Fail
            OkLt:
                SGP 20, 2, 1

            // WAIT POS timeout sets ETO -> JC ETO -> CLE ETO -> ETO cleared
            SAP MaxPositioningSpeed, 0, 100
            MVP ABS, 0, 20000
            WAIT POS, 0, 3
            JC ETO, TimeoutCaught
            JA Fail
            TimeoutCaught:
                SGP 21, 2, 1
                CLE ETO
                JC ETO, Fail
                JC EAL, Fail
                JC EDV, Fail
                JC EPO, Fail
                MST 0
                SGP 22, 2, 1
                STOP
            Fail:
                SGP 23, 2, 999
                STOP
            """);

        StepUntilStopped(controller, 200);

        Assert.Equal(10, controller.ProcessCommand(Cmd(TmclCommand.GGP, 9, 2, 0)).Value);
        Assert.Equal(15, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(12, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(48, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);
        Assert.Equal(8, controller.ProcessCommand(Cmd(TmclCommand.GGP, 3, 2, 0)).Value);
        Assert.Equal(3, controller.ProcessCommand(Cmd(TmclCommand.GGP, 4, 2, 0)).Value);
        Assert.Equal(8, controller.ProcessCommand(Cmd(TmclCommand.GGP, 5, 2, 0)).Value);
        Assert.Equal(13, controller.ProcessCommand(Cmd(TmclCommand.GGP, 6, 2, 0)).Value);
        Assert.Equal(14, controller.ProcessCommand(Cmd(TmclCommand.GGP, 7, 2, 0)).Value);
        Assert.Equal(~14, controller.ProcessCommand(Cmd(TmclCommand.GGP, 8, 2, 0)).Value);

        Assert.Equal(27, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0)).Value);
        Assert.Equal(20, controller.ProcessCommand(Cmd(TmclCommand.GGP, 11, 2, 0)).Value);
        Assert.Equal(7, controller.ProcessCommand(Cmd(TmclCommand.GGP, 12, 2, 0)).Value);
        Assert.Equal(42, controller.ProcessCommand(Cmd(TmclCommand.GGP, 13, 2, 0)).Value);
        Assert.Equal(6, controller.ProcessCommand(Cmd(TmclCommand.GGP, 14, 2, 0)).Value);
        Assert.Equal(3, controller.ProcessCommand(Cmd(TmclCommand.GGP, 15, 2, 0)).Value);
        Assert.Equal(7, controller.ProcessCommand(Cmd(TmclCommand.GGP, 16, 2, 0)).Value);
        Assert.Equal(7, controller.ProcessCommand(Cmd(TmclCommand.GGP, 17, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 18, 2, 0)).Value);
        Assert.Equal(~7, controller.ProcessCommand(Cmd(TmclCommand.GGP, 19, 2, 0)).Value);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 20, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 21, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 22, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 23, 2, 0)).Value);
    }

    [Fact]
    public void Subroutines8LevelOverflowGuard_CallDjnzRstAapAgpAco_WorkCorrectly()
    {
        var controller = new Tmcm6110Controller();
        DownloadAndRun(controller, """
            // Test AAP, AGP, ACO
            CALC LOAD, 1234
            AAP MaxPositioningSpeed, 1
            AGP 30, 2
            ACO 2, 1

            // Test DJNZ counting loop (5 iterations)
            SGP 0, 2, 5
            SGP 1, 2, 0
            Loop:
                CALCV ADD, 1, 10
                DJNZ 0, Loop

            // Test conditional CALL
            CALC LOAD, 42
            COMP 42
            CALL EQ, CondSub
            COMP 99
            CALL EQ, CondSub

            // Test 8-level CSUB nesting & 9th-level overflow guard
            CSUB L1

            // Test RST resetting stack from inside a subroutine
            CSUB ResetFromInside
            SGP 5, 2, 999
            STOP

            AfterRst:
                STOP

            CondSub:
                CALCV ADD, 2, 1
                RSUB

            ResetFromInside:
                CSUB InnerBeforeRst
                SGP 6, 2, 888
                RSUB

            InnerBeforeRst:
                RST AfterRst

            L1:
                CSUB L2
                RSUB
            L2:
                CSUB L3
                RSUB
            L3:
                CSUB L4
                RSUB
            L4:
                CSUB L5
                RSUB
            L5:
                CSUB L6
                RSUB
            L6:
                CSUB L7
                RSUB
            L7:
                CSUB L8
                RSUB
            L8:
                // Depth is now 8 (max). Next CSUB (L9) must be ignored without pushing.
                CSUB L9
                CALCV ADD, 3, 1
                RSUB
            L9:
                SGP 4, 2, 777
                RSUB
            """);

        StepUntilStopped(controller, 100);

        var snap = controller.GetStandaloneSnapshot();
        Assert.Equal(TmclAppStatus.Stop, snap.Status);
        Assert.Equal(0, snap.CallStackDepth); // RST cleared stack cleanly

        Assert.Equal(1234, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.MaxPositioningSpeed, 1, 0)).Value);
        Assert.Equal(1234, controller.ProcessCommand(Cmd(TmclCommand.GGP, 30, 2, 0)).Value);
        Assert.Equal(1234, controller.ProcessCommand(Cmd(TmclCommand.GCO, 2, 1, 0)).Value);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(50, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value); // CondSub called once
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 3, 2, 0)).Value); // L8 ran
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 4, 2, 0)).Value); // L9 blocked by 8-level guard
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 5, 2, 0)).Value); // Skipped by RST
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 6, 2, 0)).Value); // Skipped by RST
    }

    [Fact]
    public void FwV145PlusOpcodes_VarCalcMotionAndIndexedOpcodes_ExecuteCorrectly()
    {
        var controller = new Tmcm6110Controller();
        DownloadAndRun(controller, """
            // CALCV, CALCVV, CALCVA, CALCAV, CALCVX, CALCXV
            SGP 0, 2, 10
            SGP 1, 2, 3
            CALCV ADD, 0, 5         // Var0 = 15
            CALCVV SUB, 0, 1        // Var0 = 12
            CALC LOAD, 8
            CALCVA ADD, 0           // Var0 = 20
            CALC LOAD, 2
            CALCAV MUL, 0           // Accu = 40
            AGP 2, 2                // Var2 = 40
            CALCX LOAD              // X = 40
            SGP 3, 2, 7
            CALCVX ADD, 3           // Var3 = 47
            CALCXV SUB, 1           // X = 40 - 3 = 37
            CALCX SWAP              // Accu = 37, X = 40
            AGP 4, 2                // Var4 = 37
            CALCVV COMP, 0, 3       // 20 < 47 -> LT
            JC LT, VarCompOk
            STOP
            VarCompOk:
                SGP 5, 2, 1

            // SIV / GIV / AIV indexed by X
            CALC LOAD, 6
            CALCX LOAD              // X = 6
            SIV 555                 // Var6 = 555
            GIV                     // Accu = 555
            CALC ADD, 111           // Accu = 666
            CALC LOAD, 7
            CALCX LOAD              // X = 7
            CALC LOAD, 666
            AIV                     // Var7 = 666

            // SAPX / GAPX / AAPX on Axis 2 via X = 2
            CALC LOAD, 2
            CALCX LOAD              // X = 2
            SAPX MaxPositioningSpeed, 1600
            GAPX MaxPositioningSpeed // Accu = 1600
            AGP 8, 2                // Var8 = 1600
            CALC LOAD, 900
            AAPX MaxAcceleration    // Axis 2 MaxAccel = 900

            // MVPA ABS & REL on Axis 2
            CALC LOAD, 400
            MVPA ABS, 2
            WAIT POS, 2, 0
            CALC LOAD, 150
            MVPA REL, 2
            WAIT POS, 2, 0

            // MVPXA ABS on Axis 1 (X = 1)
            CALC LOAD, 1
            CALCX LOAD
            CALC LOAD, 350
            MVPXA ABS
            WAIT POS, 1, 0

            // ROLA / RORA / ROLXA / RORXA / MSTX
            CALC LOAD, 500
            RORA 3                  // Axis 3 vel = +500
            CALC LOAD, 3
            CALCX LOAD
            CALC LOAD, 400
            ROLXA                   // Axis 3 vel = -400
            WAIT TICKS, 0, 2
            GAP TargetSpeed, 3
            AGP 9, 2                // Var9 = -400
            MSTX                    // Stop Axis 3
            ROLA 4                  // Axis 4 vel = -400
            MST 4
            CALC LOAD, 4
            CALCX LOAD
            CALC LOAD, 300
            RORXA                   // Axis 4 vel = +300
            MSTX
            STOP
            """);

        StepUntilStopped(controller, 500);

        Assert.Equal(20, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(40, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);
        Assert.Equal(47, controller.ProcessCommand(Cmd(TmclCommand.GGP, 3, 2, 0)).Value);
        Assert.Equal(37, controller.ProcessCommand(Cmd(TmclCommand.GGP, 4, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 5, 2, 0)).Value);
        Assert.Equal(555, controller.ProcessCommand(Cmd(TmclCommand.GGP, 6, 2, 0)).Value);
        Assert.Equal(666, controller.ProcessCommand(Cmd(TmclCommand.GGP, 7, 2, 0)).Value);
        Assert.Equal(1600, controller.ProcessCommand(Cmd(TmclCommand.GGP, 8, 2, 0)).Value);
        Assert.Equal(-400, controller.ProcessCommand(Cmd(TmclCommand.GGP, 9, 2, 0)).Value);
        Assert.Equal(1600, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.MaxPositioningSpeed, 2, 0)).Value);
        Assert.Equal(900, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.MaxAcceleration, 2, 0)).Value);
        Assert.Equal(550, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.Equal(350, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetSpeed, 3, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetSpeed, 4, 0)).Value);
    }

    [Fact]
    public void GlobalParam77_AutoStart_RunsAfterSoftwareResetWhenEnabledAndStaysStoppedWhenDisabled()
    {
        var controller = new Tmcm6110Controller();
        var asm = TmclAssembler.Assemble("""
            SGP 10, 2, 777
            MVP ABS, 0, 200
            WAIT POS, 0, 0
            STOP
            """);
        Assert.True(asm.Success);

        controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0));
        controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0));
        foreach (var pkt in asm.Packets)
            controller.ProcessCommand(pkt);
        controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0));
        controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0));

        // 1. With #77 == 0 (default): Cmd 255 keeps VM stopped
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 77, 0, 0)).Value);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234)).StatusEnum);
        Assert.Equal(TmclAppStatus.Stop, controller.GetStandaloneSnapshot().Status);
        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0)).Value);
        Assert.Equal(0, controller.GetAxis(0).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));

        // 2. With #77 == 1: Cmd 255 auto-starts downloaded program
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 77, 0, 1)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234)).StatusEnum);
        Assert.Equal(TmclAppStatus.Run, controller.GetStandaloneSnapshot().Status);

        StepUntilStopped(controller, 200);
        Assert.Equal(777, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0)).Value);
        Assert.Equal(200, controller.GetAxis(0).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
    }

    [Fact]
    public void UnaryCalcNot_DirectAndStandalone_InvertsAccumulator()
    {
        var controller = new Tmcm6110Controller();

        // Direct Mode: CALC LOAD, 5 -> CALC NOT (operand=0) must invert Accumulator (~5 == -6)
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.CALC, 9, 0, 5)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.CALC, 8, 0, 0)).StatusEnum);
        Assert.Equal(-6, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.CALC, 9, 0, -1)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.CALC, 8, 0, 0)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);

        // Standalone VM: CALC LOAD, 5 -> CALC NOT -> AGP 0, 2 -> STOP
        DownloadAndRun(controller, """
            CALC LOAD, 5
            CALC NOT
            AGP 0, 2
            STOP
            """);
        StepUntilStopped(controller, 20);

        Assert.Equal(~5, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(-6, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);
    }
}
