using System.Text;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class TmclPacketTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    [InlineData(123456)]
    [InlineData(-123456)]
    [InlineData(0x12345678)]
    [InlineData(int.MaxValue)]
    [InlineData(int.MinValue)]
    public void CommandPacket_Signed32BitBigEndian_RoundtripsCorrectly(int value)
    {
        var original = new TmclCommandPacket(
            moduleAddress: 0x01,
            command: (byte)TmclCommand.MVP,
            typeNumber: 0x00,
            motorOrBank: 0x02,
            value: value);

        byte[] bytes = original.ToBytes();

        Assert.Equal(9, bytes.Length);
        Assert.Equal((byte)((value >> 24) & 0xFF), bytes[4]);
        Assert.Equal((byte)((value >> 16) & 0xFF), bytes[5]);
        Assert.Equal((byte)((value >> 8) & 0xFF), bytes[6]);
        Assert.Equal((byte)(value & 0xFF), bytes[7]);

        var decoded = TmclCommandPacket.FromBytes(bytes);

        Assert.True(decoded.IsChecksumValid);
        Assert.Equal(original.ModuleAddress, decoded.ModuleAddress);
        Assert.Equal(original.Command, decoded.Command);
        Assert.Equal(original.TypeNumber, decoded.TypeNumber);
        Assert.Equal(original.MotorOrBank, decoded.MotorOrBank);
        Assert.Equal(value, decoded.Value);
        Assert.Equal(original.Checksum, decoded.Checksum);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    [InlineData(-987654321)]
    [InlineData(int.MaxValue)]
    [InlineData(int.MinValue)]
    public void ReplyPacket_Signed32BitBigEndian_RoundtripsCorrectly(int value)
    {
        var original = new TmclReplyPacket(
            replyAddress: 0x02,
            moduleAddress: 0x01,
            status: TmclStatus.SuccessOk,
            command: (byte)TmclCommand.GAP,
            value: value);

        byte[] bytes = original.ToBytes();

        Assert.Equal(9, bytes.Length);
        Assert.Equal((byte)((value >> 24) & 0xFF), bytes[4]);
        Assert.Equal((byte)((value >> 16) & 0xFF), bytes[5]);
        Assert.Equal((byte)((value >> 8) & 0xFF), bytes[6]);
        Assert.Equal((byte)(value & 0xFF), bytes[7]);

        byte expectedChecksum = unchecked((byte)(
            bytes[0] + bytes[1] + bytes[2] + bytes[3] +
            bytes[4] + bytes[5] + bytes[6] + bytes[7]));
        Assert.Equal(expectedChecksum, bytes[8]);

        var decoded = TmclReplyPacket.FromBytes(bytes);

        Assert.Equal(original.ReplyAddress, decoded.ReplyAddress);
        Assert.Equal(original.ModuleAddress, decoded.ModuleAddress);
        Assert.Equal(original.Status, decoded.Status);
        Assert.Equal(original.Command, decoded.Command);
        Assert.Equal(value, decoded.Value);
        Assert.Equal(expectedChecksum, decoded.Checksum);
    }

    [Fact]
    public void CommandPacket_Additive8BitChecksum_WrapsModulo256()
    {
        // Sum of bytes: 0xFF + 0x05 + 0x04 + 0x02 + 0x12 + 0x34 + 0x56 + 0x78
        var pkt = new TmclCommandPacket(0xFF, 0x05, 0x04, 0x02, 0x12345678);
        byte expected = unchecked((byte)(0xFF + 0x05 + 0x04 + 0x02 + 0x12 + 0x34 + 0x56 + 0x78));

        Assert.Equal(expected, pkt.CalculatedChecksum);
        Assert.Equal(expected, pkt.Checksum);
        Assert.True(pkt.IsChecksumValid);
    }

    [Fact]
    public void CommandPacket_BadChecksum_IsDetectedAndRejectedByController()
    {
        var valid = new TmclCommandPacket(1, (byte)TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 0, 1500);
        byte[] corrupted = valid.ToBytes();
        corrupted[8] ^= 0xFF;

        var decoded = TmclCommandPacket.FromBytes(corrupted);

        Assert.False(decoded.IsChecksumValid);
        Assert.NotEqual(decoded.CalculatedChecksum, decoded.Checksum);

        var controller = new Tmcm6110Controller();
        int beforeSpeed = controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed);

        var reply = controller.ProcessCommand(decoded);

        Assert.Equal(TmclStatus.WrongChecksum, reply.StatusEnum);
        Assert.Equal((byte)TmclCommand.SAP, reply.Command);
        Assert.Equal(0, reply.Value);
        Assert.Equal(beforeSpeed, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed));
    }

    [Fact]
    public void CommandPacket_FromBytes_ThrowsWhenTooShort()
    {
        Assert.Throws<ArgumentException>(() => TmclCommandPacket.FromBytes(new byte[8]));
    }

    [Fact]
    public void ReplyPacket_RawOverrideBytes_PreservesExactFirmwareStringPayload()
    {
        byte[] raw = new byte[9];
        raw[0] = 0x02;
        Encoding.ASCII.GetBytes("6110V147").CopyTo(raw, 1);

        var reply = new TmclReplyPacket(raw);

        Assert.Equal(raw, reply.ToBytes());
        Assert.Contains("6110V147", reply.ToDescription());
    }

    [Fact]
    public void ReplyPacket_RawOverrideBytes_ThrowsWhenNot9Bytes()
    {
        Assert.Throws<ArgumentException>(() => new TmclReplyPacket(new byte[5]));
    }
}
