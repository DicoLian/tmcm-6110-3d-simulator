using System.Text;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class Tmcm6110ControllerTests
{
    private static TmclCommandPacket Cmd(TmclCommand command, byte type, byte motorOrBank, int value, byte moduleAddr = 1) =>
        new(moduleAddr, (byte)command, type, motorOrBank, value);

    [Fact]
    public void SapAndGap_SetAndReadAxisParameters()
    {
        var controller = new Tmcm6110Controller();

        var sapReply = controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 2, 1750));
        Assert.Equal(TmclStatus.SuccessOk, sapReply.StatusEnum);
        Assert.Equal(1750, sapReply.Value);

        var gapReply = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.MaxPositioningSpeed, 2, 0));
        Assert.Equal(TmclStatus.SuccessOk, gapReply.StatusEnum);
        Assert.Equal(1750, gapReply.Value);
    }

    [Fact]
    public void SgpAndGgp_SetAndReadGlobalParametersAcrossBanks()
    {
        var controller = new Tmcm6110Controller();

        var sgpBank0 = controller.ProcessCommand(Cmd(TmclCommand.SGP, 75, 0, 12));
        Assert.Equal(TmclStatus.SuccessOk, sgpBank0.StatusEnum);

        var ggpBank0 = controller.ProcessCommand(Cmd(TmclCommand.GGP, 75, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, ggpBank0.StatusEnum);
        Assert.Equal(12, ggpBank0.Value);

        var sgpBank2 = controller.ProcessCommand(Cmd(TmclCommand.SGP, 10, 2, -4200));
        Assert.Equal(TmclStatus.SuccessOk, sgpBank2.StatusEnum);

        var ggpBank2 = controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 2, 0));
        Assert.Equal(TmclStatus.SuccessOk, ggpBank2.StatusEnum);
        Assert.Equal(-4200, ggpBank2.Value);

        var invalidBank = controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 3, 0));
        Assert.Equal(TmclStatus.InvalidValue, invalidBank.StatusEnum);
    }

    [Fact]
    public void FirmwareVersionAndAppStatusQueries_ReturnExpectedPayloads()
    {
        var controller = new Tmcm6110Controller();

        // Cmd 135: GetAppStatus reflects Global Parameter (0, 128)
        controller.ProcessCommand(Cmd(TmclCommand.SGP, 128, 0, 1));
        var appStatus = controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 0, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, appStatus.StatusEnum);
        Assert.Equal((byte)TmclCommand.GetAppStatus, appStatus.Command);
        Assert.Equal(1, appStatus.Value);

        // Cmd 136 Type 0: ASCII string format [HostAddr] + "0610V631"
        var fwAscii = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 0, 0, 0));
        byte[] asciiBytes = fwAscii.ToBytes();
        Assert.Equal(9, asciiBytes.Length);
        Assert.Equal(controller.HostAddress, asciiBytes[0]);
        Assert.Equal("0610V631", Encoding.ASCII.GetString(asciiBytes, 1, 8));

        // Cmd 136 Type 1: Binary format 0x06100631
        var fwBin = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 1, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, fwBin.StatusEnum);
        Assert.Equal((byte)TmclCommand.GetFirmwareVersion, fwBin.Command);
        Assert.Equal(0x06100631, fwBin.Value);
    }

    [Fact]
    public void MvpAbsAndRel_MoveAxisToExpectedPositions()
    {
        var controller = new Tmcm6110Controller();

        var absReply = controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 1000));
        Assert.Equal(TmclStatus.SuccessOk, absReply.StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 0, 0)).Value);

        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(1000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 0, 0)).Value);

        var relReply = controller.ProcessCommand(Cmd(TmclCommand.MVP, 1, 0, -400));
        Assert.Equal(TmclStatus.SuccessOk, relReply.StatusEnum);

        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Equal(600, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 0, 0)).Value);
    }

    [Fact]
    public void RorRolAndMst_ControlVelocityMotionAndStop()
    {
        var controller = new Tmcm6110Controller();

        // Rotate Right (positive direction)
        var rorReply = controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 1, 500));
        Assert.Equal(TmclStatus.SuccessOk, rorReply.StatusEnum);

        for (int i = 0; i < 20; i++)
            controller.StepSimulation(0.01);

        int posAfterRor = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        int spdAfterRor = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value;
        Assert.True(posAfterRor > 0);
        Assert.True(spdAfterRor > 0);

        // Stop Motor
        var mstReply = controller.ProcessCommand(Cmd(TmclCommand.MST, 0, 1, 0));
        Assert.Equal(TmclStatus.SuccessOk, mstReply.StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value);

        int stoppedPos = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        controller.StepSimulation(0.05);
        Assert.Equal(stoppedPos, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);

        // Rotate Left (negative direction)
        var rolReply = controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 1, 600));
        Assert.Equal(TmclStatus.SuccessOk, rolReply.StatusEnum);

        for (int i = 0; i < 40; i++)
            controller.StepSimulation(0.01);

        int posAfterRol = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        int spdAfterRol = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value;
        Assert.True(posAfterRol < stoppedPos);
        Assert.True(spdAfterRol < 0);
    }

    [Fact]
    public void MultiAxis_Axes0Through5_RetainIndependentState()
    {
        var controller = new Tmcm6110Controller();

        for (byte axis = 0; axis < Tmcm6110Controller.TotalAxes; axis++)
        {
            int targetPos = (axis + 1) * 250 * (axis % 2 == 0 ? 1 : -1);
            int maxSpeed = 800 + axis * 150;

            Assert.Equal(TmclStatus.SuccessOk,
                controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, axis, maxSpeed)).StatusEnum);
            Assert.Equal(TmclStatus.SuccessOk,
                controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, axis, targetPos)).StatusEnum);
        }

        for (int i = 0; i < 300; i++)
            controller.StepSimulation(0.01);

        for (byte axis = 0; axis < Tmcm6110Controller.TotalAxes; axis++)
        {
            int expectedPos = (axis + 1) * 250 * (axis % 2 == 0 ? 1 : -1);
            int expectedSpeed = 800 + axis * 150;

            Assert.Equal(expectedSpeed,
                controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.MaxPositioningSpeed, axis, 0)).Value);
            Assert.Equal(expectedPos,
                controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, axis, 0)).Value);
            Assert.Equal(1,
                controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, axis, 0)).Value);
        }

        // Axis 6 is out of range
        Assert.Equal(TmclStatus.InvalidValue,
            controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 6, 0)).StatusEnum);
    }

    [Fact]
    public void Cmd138_ReturnsImmediateStatus100Ack_AndPushesStatus128OnArrival()
    {
        var controller = new Tmcm6110Controller();
        var asyncReplies = new List<TmclReplyPacket>();
        controller.AsyncEventReplyGenerated += pkt => asyncReplies.Add(pkt);

        // Start motion on Axis 0 and Axis 2 (bitmask 0b000101 = 0x05)
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 800));
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 2, 1600));

        const int axisMask = (1 << 0) | (1 << 2);
        var ack = controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 0, 0, axisMask));

        Assert.Equal(TmclStatus.SuccessOk, ack.StatusEnum);
        Assert.Equal((byte)TmclCommand.RequestTargetPositionReachedEvent, ack.Command);
        Assert.Equal(axisMask, ack.Value);
        Assert.Empty(asyncReplies);

        // Step once: axes are still moving, so no Status=128 event yet
        controller.StepSimulation(0.01);
        Assert.Empty(asyncReplies);

        // Step until both axes arrive
        for (int i = 0; i < 300; i++)
            controller.StepSimulation(0.01);

        Assert.Single(asyncReplies);
        var asyncEvent = asyncReplies[0];
        Assert.Equal(TmclStatus.TargetPositionReachedEvent, asyncEvent.StatusEnum);
        Assert.Equal((byte)128, asyncEvent.Status);
        Assert.Equal((byte)TmclCommand.RequestTargetPositionReachedEvent, asyncEvent.Command);
        Assert.Equal(axisMask, asyncEvent.Value);
        Assert.Equal(controller.HostAddress, asyncEvent.ReplyAddress);
        Assert.Equal(controller.ModuleAddress, asyncEvent.ModuleAddress);

        // Subsequent simulation steps must not re-fire the one-shot event
        controller.StepSimulation(0.05);
        Assert.Single(asyncReplies);
    }
}
