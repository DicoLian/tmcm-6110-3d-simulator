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

        // Bank 3 interrupt parameters: #0..#2 (any int), #27..#46 (0..3)
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 0, 3, 123)).StatusEnum);
        Assert.Equal(123, controller.ProcessCommand(Cmd(TmclCommand.GGP, 0, 3, 0)).Value);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 27, 3, 2)).StatusEnum);
        Assert.Equal(2, controller.ProcessCommand(Cmd(TmclCommand.GGP, 27, 3, 0)).Value);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SGP, 27, 3, 4)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 3, 0)).StatusEnum);

        var invalidBank = controller.ProcessCommand(Cmd(TmclCommand.GGP, 10, 4, 0));
        Assert.Equal(TmclStatus.InvalidValue, invalidBank.StatusEnum);
    }

    [Fact]
    public void FirmwareVersionAndAppStatusQueries_ReturnExpectedPayloads()
    {
        var controller = new Tmcm6110Controller();

        // Global Param (0, 128) is read-only under FW V1.47
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.SGP, 128, 0, 1)).StatusEnum);

        // Cmd 129 RunApplication sets status=1 & PC=val, Cmd 135 queries Types 0..3
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RunApplication, 1, 0, 42)).StatusEnum);
        var appStatus = controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 0, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, appStatus.StatusEnum);
        Assert.Equal((byte)TmclCommand.GetAppStatus, appStatus.Command);
        Assert.Equal(1, appStatus.Value);

        var appPc = controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 1, 0, 0));
        Assert.Equal(42, appPc.Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 2, 0, 0)).Value);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 3, 0, 0)).Value);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GetAppStatus, 4, 0, 0)).StatusEnum);

        // Cmd 136 Type 0: ASCII string format [HostAddr] + "6110V147"
        var fwAscii = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 0, 0, 0));
        byte[] asciiBytes = fwAscii.ToBytes();
        Assert.Equal(9, asciiBytes.Length);
        Assert.Equal(controller.HostAddress, asciiBytes[0]);
        Assert.Equal("6110V147", Encoding.ASCII.GetString(asciiBytes, 1, 8));

        // Cmd 136 Type 1: Binary format 0x17DE012F (Module 6110, FW V1.47)
        var fwBin = controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 1, 0, 0));
        Assert.Equal(TmclStatus.SuccessOk, fwBin.StatusEnum);
        Assert.Equal((byte)TmclCommand.GetFirmwareVersion, fwBin.Command);
        Assert.Equal(0x17DE012F, fwBin.Value);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GetFirmwareVersion, 2, 0, 0)).StatusEnum);
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

        // FW 1.47 MST: Soft stop (ramping down cleanly using MaxAcceleration, TargetSpeed=0)
        var mstReply = controller.ProcessCommand(Cmd(TmclCommand.MST, 0, 1, 0));
        Assert.Equal(TmclStatus.SuccessOk, mstReply.StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetSpeed, 1, 0)).Value);

        // Allow soft deceleration ramp to bring axis smoothly to a halt
        for (int i = 0; i < 60; i++)
            controller.StepSimulation(0.01);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value);

        int stoppedPos = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        controller.StepSimulation(0.05);
        Assert.Equal(stoppedPos, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value);

        // Rotate Left (negative direction) + signed ROR/ROL validation (-2047..2047)
        var rolReply = controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 1, 600));
        Assert.Equal(TmclStatus.SuccessOk, rolReply.StatusEnum);

        for (int i = 0; i < 40; i++)
            controller.StepSimulation(0.01);

        int posAfterRol = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        int spdAfterRol = controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualSpeed, 1, 0)).Value;
        Assert.True(posAfterRol < stoppedPos);
        Assert.True(spdAfterRol < 0);

        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 1, 2048)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 1, -2048)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.ROR, 0, 1, -400)).StatusEnum);
        Assert.Equal(-400, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetSpeed, 1, 0)).Value);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.ROL, 0, 1, -400)).StatusEnum);
        Assert.Equal(400, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetSpeed, 1, 0)).Value);
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
    public void Cmd138_PreArmedOneShotAndContinuous_ReturnsAckAndPushesStatus128OnArrival()
    {
        var controller = new Tmcm6110Controller();
        var asyncReplies = new List<TmclReplyPacket>();
        controller.AsyncEventReplyGenerated += pkt => asyncReplies.Add(pkt);

        // Type=2 is invalid under FW 1.47
        Assert.Equal(TmclStatus.WrongType,
            controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 2, 0, 0x01)).StatusEnum);

        // 1. Type 0 (one-shot): Must be armed BEFORE the single MVP COORD/ABS command
        const int axisMask = (1 << 0) | (1 << 2);
        var ack = controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 0, 0, axisMask));
        Assert.Equal(TmclStatus.SuccessOk, ack.StatusEnum);
        Assert.Equal((byte)TmclCommand.RequestTargetPositionReachedEvent, ack.Command);
        Assert.Equal(axisMask, ack.Value);

        controller.ProcessCommand(Cmd(TmclCommand.SCO, 1, 0, 800));
        controller.ProcessCommand(Cmd(TmclCommand.SCO, 1, 2, 1600));
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 2, (byte)(0x40 | axisMask), 1));
        Assert.Empty(asyncReplies);

        controller.StepSimulation(0.01);
        Assert.Empty(asyncReplies);

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

        // Subsequent MVP without re-arming must NOT re-fire Type=0
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 0));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);
        Assert.Single(asyncReplies);

        // 2. Type 1 (continuous): Stays armed across multiple separate MVPs until disarmed
        asyncReplies.Clear();
        Assert.Equal(TmclStatus.SuccessOk,
            controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 1, 0, axisMask)).StatusEnum);

        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 500));
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 2, 1200));

        int accumMask = 0;
        for (int i = 0; i < 300 && accumMask != axisMask; i++)
        {
            controller.StepSimulation(0.01);
            foreach (var r in asyncReplies)
                accumMask |= r.Value;
        }
        Assert.Equal(axisMask, accumMask);

        // Disarm with Value=0, Type=0 -> next MVP does not fire
        int countBeforeDisarm = asyncReplies.Count;
        Assert.Equal(TmclStatus.SuccessOk,
            controller.ProcessCommand(Cmd(TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0)).StatusEnum);
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 100));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);
        Assert.Equal(countBeforeDisarm, asyncReplies.Count);
    }

    [Fact]
    public void Position32Bit_SupportsFullSignedInt32Range()
    {
        var controller = new Tmcm6110Controller();
        controller.GetAxis(3).SetVirtualStrokeLimits(-20_000_000, 20_000_000);
        controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 3, 2000));

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 3, 10_000_000)).StatusEnum);
        Assert.Equal(10_000_000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosition, 3, 0)).Value);

        for (int i = 0; i < 4500; i++)
        {
            controller.StepSimulation(0.1);
            if (controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 3, 0)).Value == 1)
                break;
        }
        Assert.Equal(10_000_000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 3, 0)).Value);

        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 3, -10_000_000)).StatusEnum);
        for (int i = 0; i < 9000; i++)
        {
            controller.StepSimulation(0.1);
            if (controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.TargetPosReached, 3, 0)).Value == 1)
                break;
        }
        Assert.Equal(-10_000_000, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.ActualPosition, 3, 0)).Value);
    }

    [Fact]
    public void AxisParameterValidation_EnforcesFw147RangesAndReadOnlyRules()
    {
        var controller = new Tmcm6110Controller();

        // #6/#7 Current 0..255
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.AbsoluteMaxCurrent, 0, 255)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.AbsoluteMaxCurrent, 0, 256)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.StandbyCurrent, 0, 256)).StatusEnum);

        // #4/#5/#130 >= 1
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MaxAcceleration, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MinimumSpeed, 0, 0)).StatusEnum);

        // #140 Microstep resolution 0..8
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MicrostepResolution, 0, 8)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.MicrostepResolution, 0, 9)).StatusEnum);

        // #180 SmartEnergyActualCurrent read-only; removed params -> WrongType; SAC (29) -> InvalidCommand
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.GAP, TmclAxisParam.SmartEnergyActualCurrent, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.SAP, TmclAxisParam.SmartEnergyActualCurrent, 0, 10)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GAP, 142, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GAP, 147, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GAP, 205, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GAP, 209, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidCommand, controller.ProcessCommand(new TmclCommandPacket(1, 29, 0, 0, 0)).StatusEnum);
    }

    [Fact]
    public void Coordinates0To20_ScoGcoCcoAcoBatch255AndSgp84_FollowFw147()
    {
        var controller = new Tmcm6110Controller();

        // Range 0..20 valid; 21 invalid
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SCO, 0, 0, 1111)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SCO, 20, 0, 2222)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SCO, 21, 0, 3333)).StatusEnum);
        Assert.Equal(1111, controller.ProcessCommand(Cmd(TmclCommand.GCO, 0, 0, 0)).Value);
        Assert.Equal(2222, controller.ProcessCommand(Cmd(TmclCommand.GCO, 20, 0, 0)).Value);

        // Batch copy SCO 5, 255 -> EEPROM, overwrite RAM, then GCO 5, 255 -> RAM
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SCO, 5, 1, 5555)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SCO, 5, 255, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SCO, 5, 1, 9999)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.GCO, 5, 255, 0)).StatusEnum);
        Assert.Equal(5555, controller.ProcessCommand(Cmd(TmclCommand.GCO, 5, 1, 0)).Value);

        // CCO captures ActualPosition and ACO copies Accumulator into coordinate
        controller.ProcessCommand(Cmd(TmclCommand.MVP, 0, 0, 450));
        for (int i = 0; i < 150; i++)
            controller.StepSimulation(0.01);
        Assert.Equal(450, controller.ProcessCommand(Cmd(TmclCommand.CCO, 3, 0, 0)).Value);
        Assert.Equal(450, controller.ProcessCommand(Cmd(TmclCommand.GCO, 3, 0, 0)).Value);

        controller.ProcessCommand(Cmd(TmclCommand.CALC, 9, 0, 777)); // Type 9 = LOAD 777 into Accumulator
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.ACO, 4, 0, 0)).StatusEnum);
        Assert.Equal(777, controller.ProcessCommand(Cmd(TmclCommand.GCO, 4, 0, 0)).Value);

        // SGP 84 = 1 persists 1..20 on write while coord 0 stays RAM-only across SoftwareReset
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 84, 0, 1)).StatusEnum);
        controller.ProcessCommand(Cmd(TmclCommand.SCO, 0, 2, 123));
        controller.ProcessCommand(Cmd(TmclCommand.SCO, 6, 2, 456));
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GCO, 0, 2, 0)).Value);
        Assert.Equal(456, controller.ProcessCommand(Cmd(TmclCommand.GCO, 6, 2, 0)).Value);
    }

    [Fact]
    public void SioAndGio_FollowFw147DigitalAndAnalogPorts()
    {
        var controller = new Tmcm6110Controller();

        // Digital outputs (Bank 2, ports 0..7 + 255 bitmask)
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SIO, 0, 2, 1)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SIO, 7, 2, 1)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.SIO, 8, 2, 1)).StatusEnum);
        Assert.Equal(0x81, controller.ProcessCommand(Cmd(TmclCommand.GIO, 255, 2, 0)).Value);

        // Digital inputs (Bank 0, ports 0..7 + 255 bitmask)
        controller.SetDigitalInput(2, true);
        controller.SetDigitalInput(6, true);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GIO, 6, 0, 0)).Value);
        Assert.Equal(0x44, controller.ProcessCommand(Cmd(TmclCommand.GIO, 255, 0, 0)).Value);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GIO, 8, 0, 0)).StatusEnum);

        // Pull-up config SIO 0, 0, 0..3
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SIO, 0, 0, 2)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SIO, 0, 0, 4)).StatusEnum);

        // Analog inputs Bank 1: AIN0(0) & AIN4(4) 12-bit 0..4095, Port 8 supply voltage
        controller.SetAnalogInput(0, 3500);
        controller.SetAnalogInput(4, 1234);
        Assert.Equal(3500, controller.ProcessCommand(Cmd(TmclCommand.GIO, 0, 1, 0)).Value);
        Assert.Equal(1234, controller.ProcessCommand(Cmd(TmclCommand.GIO, 4, 1, 0)).Value);
        Assert.Equal(240, controller.ProcessCommand(Cmd(TmclCommand.GIO, 8, 1, 0)).Value);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GIO, 1, 1, 0)).StatusEnum);
    }

    [Fact]
    public void GlobalParams_Random133_UserVarEeprom_Cmd137AndCmd255_FollowFw147()
    {
        var controller = new Tmcm6110Controller();

        // Obsolete globals #64 and #73 -> WrongType; End switch polarity #79 accepts 0..3
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GGP, 64, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.WrongType, controller.ProcessCommand(Cmd(TmclCommand.GGP, 73, 0, 0)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 79, 0, 3)).StatusEnum);
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.SGP, 79, 0, 4)).StatusEnum);

        // PRNG #133 deterministic given same seed
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 133, 0, 42)).StatusEnum);
        int rnd1 = controller.ProcessCommand(Cmd(TmclCommand.GGP, 133, 0, 0)).Value;
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SGP, 133, 0, 42)).StatusEnum);
        int rnd2 = controller.ProcessCommand(Cmd(TmclCommand.GGP, 133, 0, 0)).Value;
        Assert.True(rnd1 >= 0);
        Assert.Equal(rnd1, rnd2);

        // User variables STGP/RSGP Bank 2 only; persist across Cmd 255 when #85=0, clear when #85=1
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.STGP, 75, 0, 0)).StatusEnum);
        controller.ProcessCommand(Cmd(TmclCommand.SGP, 12, 2, 8888));
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.STGP, 12, 2, 0)).StatusEnum);
        controller.ProcessCommand(Cmd(TmclCommand.SGP, 12, 2, 1111));
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RSGP, 12, 2, 0)).StatusEnum);
        Assert.Equal(8888, controller.ProcessCommand(Cmd(TmclCommand.GGP, 12, 2, 0)).Value);

        controller.ProcessCommand(Cmd(TmclCommand.SGP, 12, 2, 1111));
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234)).StatusEnum);
        Assert.Equal(8888, controller.ProcessCommand(Cmd(TmclCommand.GGP, 12, 2, 0)).Value);

        controller.ProcessCommand(Cmd(TmclCommand.SGP, 85, 0, 1));
        controller.ProcessCommand(Cmd(TmclCommand.SoftwareReset, 0, 0, 1234));
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 12, 2, 0)).Value);

        // Cmd 137 RestoreFactorySettings resets custom params to defaults when value==1234
        Assert.Equal(TmclStatus.InvalidValue, controller.ProcessCommand(Cmd(TmclCommand.RestoreFactorySettings, 0, 0, 1)).StatusEnum);
        Assert.Equal(TmclStatus.SuccessOk, controller.ProcessCommand(Cmd(TmclCommand.RestoreFactorySettings, 0, 0, 1234)).StatusEnum);
        Assert.Equal(0, controller.ProcessCommand(Cmd(TmclCommand.GGP, 85, 0, 0)).Value);
        Assert.Equal(1, controller.ProcessCommand(Cmd(TmclCommand.GGP, 79, 0, 0)).Value);
    }
}
