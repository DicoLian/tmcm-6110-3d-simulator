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
        Assert.Equal("TMCM-6110 (TMCL)", controller.ControllerName);
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

        var mvp = new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 500);
        var arm138 = new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x01);

        var rxBuffer = new List<byte>();
        rxBuffer.AddRange(mvp.ToBytes());
        rxBuffer.AddRange(arm138.ToBytes());

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
}
