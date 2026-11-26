using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class TmclInterruptAndDebuggerTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value, byte moduleAddr = 1) =>
        new(moduleAddr, (byte)command, type, motorOrBank, value);

    private static void DownloadProgram(Tmcm6110Controller controller, string source)
    {
        var asm = TmclAssembler.Assemble(source);
        Assert.True(asm.Success, string.Join(Environment.NewLine, asm.Errors));

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 0)).StatusEnum);
        foreach (var pkt in asm.Packets)
            Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(pkt).StatusEnum);
        Assert.Equal(TmclStatus.CommandLoadedIntoEeprom, controller.ProcessCommand(new TmclCommandPacket(1, 0, 0, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0)).StatusEnum);
    }

    [Fact]
    public void Timer012Interrupts_IncrementUserVarsPeriodicallyAndStopOnDi()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(controller, """
            SGP 0, 2, 0
            SGP 1, 2, 0
            SGP 2, 2, 0
            VECT 0, IsrTimer0
            VECT 1, IsrTimer1
            VECT 2, IsrTimer2
            SGP 0, 3, 30
            SGP 1, 3, 50
            SGP 2, 3, 100
            EI 0
            EI 1
            EI 2
            EI 255
            IdleLoop:
                WAIT TICKS, 0, 100
                JA IdleLoop

            IsrTimer0:
                CALCV ADD, 0, 1
                RETI
            IsrTimer1:
                CALCV ADD, 1, 1
                RETI
            IsrTimer2:
                CALCV ADD, 2, 1
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);

        // Step 110 ms (11 x 10 ms): Timer0(30ms)=3, Timer1(50ms)=2, Timer2(100ms)=1
        for (int i = 0; i < 11; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(3, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);

        // Disable Timer 0 individually -> Timer 0 stops while Timers 1 & 2 continue
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.DI, 0, 0, 0)).StatusEnum);
        for (int i = 0; i < 10; i++) // up to 210 ms total
            controller.StepSimulation(0.01);

        Assert.Equal(3, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(4, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);

        // Disable global interrupts via DI 255 -> all timers stop firing
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.DI, 255, 0, 0)).StatusEnum);
        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(3, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(4, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);
    }

    [Fact]
    public void TargetPositionReachedInterrupt_3To8_FiresOnArrival()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(controller, """
            SGP 0, 2, 0
            VECT 5, IsrPosAxis2
            EI 5
            EI 255
            MVP ABS, 2, 600
            WaitArrival:
                WAIT TICKS, 0, 50
                JA WaitArrival

            IsrPosAxis2:
                CALCV ADD, 0, 1
                STOP
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);
        for (int i = 0; i < 200 && controller.GetStandaloneSnapshot().Status != TmclAppStatus.Stop; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(TmclAppStatus.Stop, controller.GetStandaloneSnapshot().Status);
        Assert.Equal(600, controller.GetAxis(2).GetParameter(TmclAxisParam.ActualPosition, clearReadFlags: false));
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
    }

    [Fact]
    public void StallGuardInterrupt_15To20_FiresOnTriggerAlarm()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(controller, """
            SGP 0, 2, 0
            VECT 16, IsrStallAxis1
            EI 16
            EI 255
            Hold:
                WAIT TICKS, 0, 100
                JA Hold

            IsrStallAxis1:
                CALCV ADD, 0, 1
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);
        controller.StepSimulation(0.01);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);

        controller.GetAxis(1).TriggerAlarm();
        controller.StepSimulation(0.01);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
    }

    [Fact]
    public void StopSwitchEdgeInterrupts_27To38_RespectRisingFallingAndBothModes()
    {
        var controller = new Tmcm6110Controller();
        // Axis 0 Left = IRQ 27 (mode 1: low->high)
        // Axis 1 Right = IRQ 30 (mode 2: high->low)
        // Axis 2 Left = IRQ 31 (mode 3: both)
        DownloadProgram(controller, """
            SGP 0, 2, 0
            SGP 1, 2, 0
            SGP 2, 2, 0
            VECT 27, IsrAx0Left
            VECT 30, IsrAx1Right
            VECT 31, IsrAx2Left
            SGP 27, 3, 1
            SGP 30, 3, 2
            SGP 31, 3, 3
            EI 27
            EI 30
            EI 31
            EI 255
            Hold:
                WAIT TICKS, 0, 100
                JA Hold

            IsrAx0Left:
                CALCV ADD, 0, 1
                RETI
            IsrAx1Right:
                CALCV ADD, 1, 1
                RETI
            IsrAx2Left:
                CALCV ADD, 2, 1
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);
        controller.StepSimulation(0.01);

        // Rising edges on all three switches
        controller.GetAxis(0).ManualLeftLimitOverride = true;
        controller.GetAxis(1).ManualRightLimitOverride = true;
        controller.GetAxis(2).ManualLeftLimitOverride = true;
        controller.StepSimulation(0.01);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value); // mode 1 fired
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value); // mode 2 ignored rising
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value); // mode 3 fired

        // Falling edges on all three switches
        controller.GetAxis(0).ManualLeftLimitOverride = false;
        controller.GetAxis(1).ManualRightLimitOverride = false;
        controller.GetAxis(2).ManualLeftLimitOverride = false;
        controller.StepSimulation(0.01);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value); // mode 1 ignored falling
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value); // mode 2 fired
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value); // mode 3 fired again
    }

    [Fact]
    public void DigitalInputEdgeInterrupts_39To46_RespectEdgeModes123()
    {
        var controller = new Tmcm6110Controller();
        // Port 0 = IRQ 39 (mode 1: rising), Port 3 = IRQ 42 (mode 2: falling), Port 7 = IRQ 46 (mode 3: both)
        DownloadProgram(controller, """
            SGP 0, 2, 0
            SGP 1, 2, 0
            SGP 2, 2, 0
            VECT 39, IsrDin0
            VECT 42, IsrDin3
            VECT 46, IsrDin7
            SGP 39, 3, 1
            SGP 42, 3, 2
            SGP 46, 3, 3
            EI 39
            EI 42
            EI 46
            EI 255
            Hold:
                WAIT TICKS, 0, 100
                JA Hold

            IsrDin0:
                CALCV ADD, 0, 1
                RETI
            IsrDin3:
                CALCV ADD, 1, 1
                RETI
            IsrDin7:
                CALCV ADD, 2, 1
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);
        controller.StepSimulation(0.01);

        // Rising edge
        controller.SetDigitalInput(0, true);
        controller.SetDigitalInput(3, true);
        controller.SetDigitalInput(7, true);
        controller.StepSimulation(0.01);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);

        // Falling edge
        controller.SetDigitalInput(0, false);
        controller.SetDigitalInput(3, false);
        controller.SetDigitalInput(7, false);
        controller.StepSimulation(0.01);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 1, 2, 0)).Value);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 2, 2, 0)).Value);
    }

    [Fact]
    public void InterruptPriorityAndWaitContextRestoration_LowerIrqWinsAndRetiRestoresAllState()
    {
        var controller = new Tmcm6110Controller();
        // Configure IRQ 39 (DIN 0) and IRQ 40 (DIN 1) both rising-edge.
        // Main code sets Accu=777, X=888, FlagGreater=true (777 > 500), then enters WAIT TICKS, 0, 5 (50ms).
        // Both IRQs trigger simultaneously; IRQ 39 MUST run before IRQ 40, clobber Accu/X/Flags,
        // and RETI must restore WaitState, PC, Accu=777, X=888, and GT flag so post-WAIT JC GT succeeds!
        DownloadProgram(controller, """
            SGP 10, 2, 0            // Execution order trace: trace = trace * 10 + irqTag
            VECT 39, Isr39
            VECT 40, Isr40
            SGP 39, 3, 1
            SGP 40, 3, 1
            EI 39
            EI 40
            EI 255

            CALC LOAD, 888
            CALCX LOAD              // X = 888
            CALC LOAD, 777          // Accu = 777
            COMP 500                // GT = true, EQ = false, LT = false, ZE = false
            WAIT TICKS, 0, 5        // 50 ms wait (PC = 12)
            JC GT, FlagsOk
            JA Fail
            FlagsOk:
                AGP 11, 2               // Should store restored Accu = 777
                CALCX SWAP
                AGP 12, 2               // Should store restored X = 888
                STOP
            Fail:
                SGP 13, 2, 999
                STOP

            Isr39:
                CALCV MUL, 10, 10
                CALCV ADD, 10, 1        // Tag 1 for IRQ 39
                CALC LOAD, 111
                CALCX LOAD
                CALC LOAD, 222
                COMP 222                // Clobber flags (EQ=true, GT=false)
                RETI

            Isr40:
                CALCV MUL, 10, 10
                CALCV ADD, 10, 2        // Tag 2 for IRQ 40
                CALC LOAD, 333
                COMP 999                // Clobber flags (LT=true, GT=false)
                RETI
            """);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 0)).StatusEnum);

        // Tick 1 (10 ms): sets up Accu/X/Flags and enters WAIT TICKS (elapsed=0/50)
        controller.StepSimulation(0.01);
        var waitSnap = controller.GetStandaloneSnapshot();
        Assert.True(waitSnap.IsWaiting);
        Assert.Equal(777, waitSnap.Accumulator);
        Assert.Equal(888, waitSnap.XRegister);

        // Trigger both DIN 0 (IRQ 39) and DIN 1 (IRQ 40) simultaneously on the next tick
        controller.SetDigitalInput(0, true);
        controller.SetDigitalInput(1, true);
        controller.StepSimulation(0.01); // 10 ms elapsed of 50 ms; both ISRs run & RETI restores WAIT

        // Verify priority: IRQ 39 ran first (tag 1), then IRQ 40 (tag 2) => 12
        Assert.Equal(12, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0)).Value);
        var midSnap = controller.GetStandaloneSnapshot();
        Assert.True(midSnap.IsWaiting);
        Assert.False(midSnap.IsInInterrupt);
        Assert.Equal(-1, midSnap.ActiveInterruptNumber);
        Assert.Equal(777, midSnap.Accumulator);
        Assert.Equal(888, midSnap.XRegister);

        // Remaining 4 ticks (40 ms) finish WAIT TICKS, 0, 5 and verify restored Accu/X/GT flag
        for (int i = 0; i < 4; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(TmclAppStatus.Stop, controller.GetStandaloneSnapshot().Status);
        Assert.Equal(777, controller.ProcessCommand(Cmd(TmclCommand.GGP, 11, 2, 0)).Value);
        Assert.Equal(888, controller.ProcessCommand(Cmd(TmclCommand.GGP, 12, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 13, 2, 0)).Value);
    }

    [Fact]
    public void DebuggerControls_Step130_Stop128Resume129_AndReset131_WorkAccurately()
    {
        var controller = new Tmcm6110Controller();
        DownloadProgram(controller, """
            CALC LOAD, 100          // PC 0
            CALCX LOAD              // PC 1: X = 100
            CALC ADD, 50            // PC 2: Accu = 150
            AGP 0, 2                // PC 3: Var0 = 150
            STOP                    // PC 4
            """);

        // StepApplication is rejected during download mode
        controller.ProcessCommand(Cmd(TmclCommand.StartDownload, 0, 0, 10));
        Assert.Equal(TmclStatus.CommandNotAvailable, controller.ProcessCommand(Cmd(TmclCommand.StepApplication, 0, 0, 0)).StatusEnum);
        controller.ProcessCommand(Cmd(TmclCommand.QuitDownload, 0, 0, 0));

        // 1. Single-step instructions 0, 1, 2 via Cmd 130
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StepApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal((int)TmclAppStatus.Step, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 0, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 1, 0, 0)).Value);
        Assert.Equal(100, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StepApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 1, 0, 0)).Value);
        Assert.Equal(100, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 3, 0, 0)).Value);

        // 2. Pause with Cmd 128 Stop, then resume from paused PC (2) with Cmd 129 Type 0
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.StopApplication, 0, 0, 0)).StatusEnum);
        Assert.Equal((int)TmclAppStatus.Stop, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 0, 0, 0)).Value);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 1, 0, 0)).Value);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 0, 0, 0)).StatusEnum);
        controller.StepSimulation(0.01);

        Assert.Equal(TmclAppStatus.Stop, controller.GetStandaloneSnapshot().Status);
        Assert.Equal(150, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 2, 0)).Value);
        Assert.Equal(4, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 1, 0, 0)).Value);

        // 3. ResetApplication (Cmd 131) resets Status=3, PC=0, Accu=0, X=0, and flags
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.ResetApplication, 0, 0, 0)).StatusEnum);
        var resetSnap = controller.GetStandaloneSnapshot();
        Assert.Equal(TmclAppStatus.Reset, resetSnap.Status);
        Assert.Equal(0, resetSnap.ProgramCounter);
        Assert.Equal(0, resetSnap.Accumulator);
        Assert.Equal(0, resetSnap.XRegister);
        Assert.Equal(0, resetSnap.ErrorFlags);
        Assert.Equal(0, resetSnap.CallStackDepth);
    }
}
