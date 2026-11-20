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
}
