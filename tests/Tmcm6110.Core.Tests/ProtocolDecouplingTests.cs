using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class ProtocolDecouplingTests
{
    [Fact]
    public void MotionAbstractions_ControlAxisAndExposeTelemetryAndVendorDiagnostics()
    {
        IMotionController controller = new Tmcm6110Controller();
        Assert.Equal("TMCM-6110 (TMCL FW V1.47)", controller.ControllerName);
        Assert.Equal(6, controller.AxisCount);

        IMotionAxis axis = controller.GetMotionAxis(2);
        axis.SetVirtualStrokeLimits(-5000, 5000);
        axis.SetLimitStopEnabled(true, true);
        axis.MoveToAbsolute(1200);

        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        MotionAxisTelemetry telemetry = axis.GetTelemetry();
        Assert.Equal(2, telemetry.AxisIndex);
        Assert.Equal(1200, telemetry.ActualPosition);
        Assert.True(telemetry.IsPositionReached);
        Assert.True(telemetry.IsRightLimitStopEnabled);
        Assert.True(telemetry.IsLeftLimitStopEnabled);

        IReadOnlyList<VendorDiagnosticItem> diags = axis.GetVendorDiagnostics();
        Assert.Contains(diags, d => d.Name.Contains("SG2 Load"));
        Assert.Contains(diags, d => d.Name.Contains("TMC262 Flags"));
    }

    [Fact]
    public void TmclProtocolAdapter_ParsesValidFrames_ResyncsNoiseBytes_AndEmitsTraces()
    {
        var controller = new Tmcm6110Controller();
        using var adapter = new TmclProtocolAdapter(controller);

        var traces = new List<ProtocolTraceEvent>();
        adapter.TraceGenerated += trace => traces.Add(trace);

        var replies = new List<byte[]>();
        var cmd = new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 2, 1500);

        // Prefix 2 noise bytes before a valid 9-byte TMCL frame
        var rxBuffer = new List<byte> { 0xAA, 0x55 };
        rxBuffer.AddRange(cmd.ToBytes());

        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));

        Assert.Empty(rxBuffer);
        Assert.Single(replies);
        var reply = TmclReplyPacket.FromBytes(replies[0]);
        Assert.Equal(TmclStatus.SuccessOk, reply.StatusEnum);
        Assert.Equal(1500, reply.Value);

        Assert.Single(traces);
        Assert.Equal("TMCL 9-Byte Binary", traces[0].ProtocolName);
        Assert.Contains("MVP ABS", traces[0].DecodedRequest);
    }

    [Fact]
    public void TmclProtocolAdapter_ForwardsAsyncCmd138EventBytes()
    {
        var controller = new Tmcm6110Controller();
        using var adapter = new TmclProtocolAdapter(controller);

        var asyncPayloads = new List<byte[]>();
        adapter.AsyncReplyBytesGenerated += bytes => asyncPayloads.Add(bytes);

        var arm138 = new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x01);
        var mvp = new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 500);

        var rxBuffer = new List<byte>();
        rxBuffer.AddRange(arm138.ToBytes());
        rxBuffer.AddRange(mvp.ToBytes());

        var syncReplies = new List<byte[]>();
        adapter.ProcessIncomingBytes(rxBuffer, reply => syncReplies.Add(reply));
        Assert.Equal(2, syncReplies.Count);
        Assert.Empty(asyncPayloads);

        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);

        Assert.Single(asyncPayloads);
        var asyncReply = TmclReplyPacket.FromBytes(asyncPayloads[0]);
        Assert.Equal(TmclStatus.TargetPositionReachedEvent, asyncReply.StatusEnum);
        Assert.Equal(0x01, asyncReply.Value);
    }

    [Fact]
    public void TmclProtocolAdapter_HandlesBadChecksumWithStatus1Reply()
    {
        var controller = new Tmcm6110Controller();
        using var adapter = new TmclProtocolAdapter(controller);

        byte[] badFrame = new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0).ToBytes();
        badFrame[8] ^= 0xFF;

        var rxBuffer = new List<byte>(badFrame);
        var replies = new List<byte[]>();
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));

        Assert.Empty(rxBuffer);
        Assert.Single(replies);
        var errReply = TmclReplyPacket.FromBytes(replies[0]);
        Assert.Equal(TmclStatus.WrongChecksum, errReply.StatusEnum);
    }

    [Fact]
    public void TmclProtocolAdapter_SuppressesRepliesForGlobalParam255_SecondaryAddr87_AndCmd137()
    {
        var controller = new Tmcm6110Controller();
        using var adapter = new TmclProtocolAdapter(controller);

        // 1. Enable Global Parameter #255 (Suppress Reply = 1)
        var sgpSuppressOn = new TmclCommandPacket(1, (byte)TmclCommand.SGP, 255, 0, 1);
        var mvpSuppressed = new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 400);
        var gapAllowed = new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.TargetPosition, 0, 0);

        var replies = new List<byte[]>();
        var rxBuffer = new List<byte>();
        rxBuffer.AddRange(sgpSuppressOn.ToBytes());
        rxBuffer.AddRange(mvpSuppressed.ToBytes());
        rxBuffer.AddRange(gapAllowed.ToBytes());

        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));

        // Only GAP sends a reply when #255 = 1
        Assert.Single(replies);
        var gapReply = TmclReplyPacket.FromBytes(replies[0]);
        Assert.Equal((byte)TmclCommand.GAP, gapReply.Command);
        Assert.Equal(400, gapReply.Value);

        // Disable #255 (once #255 becomes 0 during execution, post-execution reply resumes for SGP and SAP)
        replies.Clear();
        rxBuffer.AddRange(new TmclCommandPacket(1, (byte)TmclCommand.SGP, 255, 0, 0).ToBytes());
        rxBuffer.AddRange(new TmclCommandPacket(1, (byte)TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 0, 1200).ToBytes());
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));
        Assert.Equal(2, replies.Count);
        Assert.Equal((byte)TmclCommand.SGP, TmclReplyPacket.FromBytes(replies[0]).Command);
        Assert.Equal((byte)TmclCommand.SAP, TmclReplyPacket.FromBytes(replies[1]).Command);

        // 2. Secondary Serial Address (#87 = 5): accepted & executed, but reply is suppressed
        replies.Clear();
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.SGP, 87, 0, 5));
        rxBuffer.AddRange(new TmclCommandPacket(5, (byte)TmclCommand.MVP, 0, 1, 900).ToBytes());
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));
        Assert.Empty(replies);
        Assert.Equal(900, controller.GetAxis(1).GetParameter(TmclAxisParam.TargetPosition, clearReadFlags: false));

        // 3. Cmd 137 Restore Factory Settings (1234): resets controller without sending reply
        rxBuffer.AddRange(new TmclCommandPacket(1, (byte)TmclCommand.RestoreFactorySettings, 0, 0, 1234).ToBytes());
        adapter.ProcessIncomingBytes(rxBuffer, reply => replies.Add(reply));
        Assert.Empty(replies);
        Assert.Equal(0, controller.GetAxis(1).GetParameter(TmclAxisParam.TargetPosition, clearReadFlags: false));
    }
}
