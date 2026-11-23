using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class AxisLimitsAndAlarmsTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value) =>
        new(1, (byte)command, type, motorOrBank, value);

    [Fact]
    public void VirtualLimits_TriggerStatusRegistersAndStopMotionWhenEnabled()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(0);
        axis.VirtualMinStrokeSteps = -500;
        axis.VirtualMaxStrokeSteps = 500;
        axis.AutoTriggerStrokeLimits = true;

        // Enable both +Limit (#12 = 0) and -Limit (#13 = 0)
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, 0, 0));
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, 0, 0));

        // Command motion beyond +Limit
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 2000));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.LeftLimitSwitchStatus, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 0, 0)).Value);
        int stoppedAtRight = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value;
        Assert.True(stoppedAtRight >= 500 && stoppedAtRight < 2000);

        // Command motion beyond -Limit
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, -2000));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.LeftLimitSwitchStatus, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RefSwitchStatus, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 0, 0)).Value);
        int stoppedAtLeft = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value;
        Assert.True(stoppedAtLeft <= -500 && stoppedAtLeft > -2000);
    }

    [Fact]
    public void ForcedLimitOverrides_ReflectInRegistersAndBlockMotionWhenEnabled()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(1);

        // Enable +Limit and -Limit
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, 1, 0));
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, 1, 0));

        // Force +Limit active at position 0
        axis.ManualRightLimitOverride = true;
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 1, 0)).Value);
        Assert.True(axis.GetSnapshot().IsRightLimitTriggered);

        // Attempting ROR should be blocked by forced +Limit
        controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 1, 800));
        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value);

        // Clear +Limit, force -Limit active
        axis.ManualRightLimitOverride = false;
        axis.ManualLeftLimitOverride = true;
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 1, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.LeftLimitSwitchStatus, 1, 0)).Value);
        Assert.True(axis.GetSnapshot().IsLeftLimitTriggered);

        // Attempting ROL should be blocked by forced -Limit
        controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 1, 800));
        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value);
    }

    [Fact]
    public void AlarmFaultInjection_StopsAxisAndBlocksMotionUntilClearedByCle()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(2);

        // Start motion
        controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 2, 1000));
        for (int i = 0; i < 15; i++)
            controller.StepSimulation(0.01);

        Assert.True(controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 2, 0)).Value > 0);

        // Inject StallGuard2 (#207) + TMC262 Driver Error (#208)
        var injectedFlags = Tmc262DriverErrorFlags.StallGuard2Status | Tmc262DriverErrorFlags.OvertemperatureShutdown;
        axis.TriggerAlarm(injectedFlags, stallError: true);

        Assert.True(axis.HasActiveAlarm);
        Assert.Equal(AxisMotionState.AlarmStopped, axis.GetSnapshot().MotionState);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 2, 0)).Value);

        int drvFlags = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.DriverErrorFlags, 2, 0)).Value;
        Assert.Equal((int)injectedFlags, drvFlags & (int)injectedFlags);

        // Motion commands while alarm is latched must not move the axis
        int posAtFault = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value;
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 2, posAtFault + 1000));
        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(posAtFault, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.True(axis.HasActiveAlarm);

        // CLE (Cmd 36) clears alarm flags and restores motion capability
        var cleReply = controller.ProcessCommand(Cmd(TmclCommand.CLE, 0, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, cleReply.StatusEnum);
        Assert.False(axis.HasActiveAlarm);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 2, 0)).Value);
        Assert.Equal(
            (int)Tmc262DriverErrorFlags.StandstillIndicator,
            controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.DriverErrorFlags, 2, 0)).Value);

        // Axis can move again after CLE
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 2, posAtFault + 500));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(posAtFault + 500, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 2, 0)).Value);
    }

    [Fact]
    public void DefaultStrokeLimits_MatchGantryAxisGeometry()
    {
        var controller = new Tmcm6110Controller();

        var snapZ = controller.GetAxis(0).GetSnapshot();
        var snapY = controller.GetAxis(1).GetSnapshot();
        var snapX = controller.GetAxis(2).GetSnapshot();

        Assert.Equal(-45_000, snapZ.VirtualMinStrokeSteps);
        Assert.Equal(45_000, snapZ.VirtualMaxStrokeSteps);
        Assert.Equal(-100_000, snapY.VirtualMinStrokeSteps);
        Assert.Equal(100_000, snapY.VirtualMaxStrokeSteps);
        Assert.Equal(-100_000, snapX.VirtualMinStrokeSteps);
        Assert.Equal(100_000, snapX.VirtualMaxStrokeSteps);
    }

    [Fact]
    public void SetVirtualStrokeLimits_NormalizesOrderAndUpdatesSwitchRegistersImmediately()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(1);

        // Reversed arguments normalize cleanly; at position 0, [0, 5000] activates -Limit immediately
        axis.SetVirtualStrokeLimits(5000, 0);
        var snap = axis.GetSnapshot();

        Assert.Equal(0, snap.VirtualMinStrokeSteps);
        Assert.Equal(5000, snap.VirtualMaxStrokeSteps);
        Assert.True(snap.IsLeftLimitTriggered);
        Assert.False(snap.IsRightLimitTriggered);

        // Expanding min below current position clears -Limit immediately
        axis.SetVirtualStrokeLimits(-5000, 5000);
        snap = axis.GetSnapshot();
        Assert.False(snap.IsLeftLimitTriggered);
        Assert.False(snap.IsRightLimitTriggered);
    }

    [Fact]
    public void VelocityMotion_StopsExactlyAtVirtualStrokeLimitsWhenEnabled()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(2);
        axis.SetVirtualStrokeLimits(-2000, 2000);

        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, 2, 0));
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, 2, 0));

        // Rotate Right past +Limit
        controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 2, 1500));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(2000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 2, 0)).Value);

        // Rotate Left past -Limit
        controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 2, 1500));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(-2000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 2, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.LeftLimitSwitchStatus, 2, 0)).Value);
    }

    [Fact]
    public void ReferenceSearch_AutomaticallyHitsVirtualStrokeLimitAndCompletesHoming()
    {
        var controller = new Tmcm6110Controller();
        var axis = controller.GetAxis(0);
        axis.SetVirtualStrokeLimits(-5000, 5000);

        // Mode 65 searches +Limit automatically without manual override
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.ReferenceSearchMode, 0, 65));
        controller.ProcessCommand(Cmd(TmclCommand.RFS, 0, 0, 0));

        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.RFS, 2, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 0, 0)).Value);
    }

    [Fact]
    public void CornerOriginStrokeLimits_AllowAwayMotionAndHomeCleanly()
    {
        var controller = new Tmcm6110Controller();

        // Case A: -Limit corner origin [0, +5000] on Axis 2 (X), starting at 0, RFS Mode 1 (-Limit)
        var axX = controller.GetAxis(2);
        axX.SetVirtualStrokeLimits(0, 5000);
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, 2, 0));
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, 2, 0));

        // Moving positive away from -Limit at 0 must succeed
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 2, 2500));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(2500, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.LeftLimitSwitchStatus, 2, 0)).Value);

        // Homing toward -Limit (Mode 1) reaches 0, backs off, and completes cleanly
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.ReferenceSearchMode, 2, 1));
        controller.ProcessCommand(Cmd(TmclCommand.RFS, 0, 2, 0));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.RFS, 2, 2, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value);

        // Case B: +Limit corner origin [-5000, 0] on Axis 1 (Y), starting at 0, RFS Mode 65 (+Limit)
        var axY = controller.GetAxis(1);
        axY.SetVirtualStrokeLimits(-5000, 0);
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.RightLimitSwitchDisable, 1, 0));
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.LeftLimitSwitchDisable, 1, 0));

        // Moving negative away from +Limit at 0 must succeed
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 1, -2500));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(-2500, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);

        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.ReferenceSearchMode, 1, 65));
        controller.ProcessCommand(Cmd(TmclCommand.RFS, 0, 1, 0));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.RFS, 2, 1, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);
    }

    [Fact]
    public void SixAxisInterpolatedMove_ReachesAllTargetsAndFiresCmd138()
    {
        var controller = new Tmcm6110Controller();
        var asyncReplies = new List<TmclReplyPacket>();
        controller.AsyncEventReplyGenerated += pkt => asyncReplies.Add(pkt);

        int[] targets = [1200, -1800, 2400, -1500, 2100, -900];
        for (byte i = 0; i < 6; i++)
            controller.ProcessCommand(Cmd(TmclCommand.SCO, 1, i, targets[i]));

        // Bit 6 (0x40) + 6-axis mask (0x3F) = 0x7F
        var mvpReply = controller.ProcessCommand(Cmd(TmclCommand.MVP, 2, 0x7F, 1));
        Assert.Equal(TmclStatus.SuccessOk, mvpReply.StatusEnum);

        controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x3F));

        for (int step = 0; step < 500 && asyncReplies.Count == 0; step++)
            controller.StepSimulation(0.01);

        Assert.Single(asyncReplies);
        Assert.Equal(TmclStatus.TargetPositionReachedEvent, asyncReplies[0].StatusEnum);
        Assert.Equal(0x3F, asyncReplies[0].Value);

        for (byte i = 0; i < 6; i++)
        {
            Assert.Equal(targets[i], controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, i, 0)).Value);
            Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, i, 0)).Value);
        }
    }
}
