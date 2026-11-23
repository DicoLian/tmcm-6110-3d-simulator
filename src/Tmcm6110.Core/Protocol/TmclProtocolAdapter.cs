using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Engine;

namespace Tmcm6110.Core.Protocol;

/// <summary>
/// Protocol translation adapter for 9-Byte Binary TMCL frames with sliding-window checksum resynchronization.
/// </summary>
public sealed class TmclProtocolAdapter : IProtocolAdapter
{
    private readonly Tmcm6110Controller _controller;

    public string ProtocolName => "TMCL 9-Byte Binary";
    public IMotionController Controller => _controller;
    public Tmcm6110Controller TmcmController => _controller;

    /// <summary>
    /// If true, responds to any incoming Module Address (helpful if Host sends to addr 1 or 0).
    /// If false, strictly checks that byte[0] matches Controller.ModuleAddress.
    /// </summary>
    public bool AcceptAnyModuleAddress { get; set; } = true;

    public event Action<byte[]>? AsyncReplyBytesGenerated;
    public event Action<ProtocolTraceEvent>? TraceGenerated;
    public event Action<string>? StatusMessage;
    public event Action<string, TmclCommandPacket?, TmclReplyPacket?>? RawFrameEvent;

    public TmclProtocolAdapter(Tmcm6110Controller controller)
    {
        _controller = controller ?? throw new ArgumentNullException(nameof(controller));
        _controller.PacketProcessed += OnControllerPacketProcessed;
        _controller.AsyncEventReplyGenerated += OnControllerAsyncEventReplyGenerated;
    }

    /// <summary>
    /// Synchronizes and extracts 9-byte TMCL binary frames from <paramref name="rxBuffer"/>.
    /// Uses sliding-window checksum verification to automatically recover from noise bytes on Serial/RS485.
    /// </summary>
    public void ProcessIncomingBytes(List<byte> rxBuffer, Action<byte[]> sendReply)
    {
        while (rxBuffer.Count >= 9)
        {
            var span = new byte[9];
            rxBuffer.CopyTo(0, span, 0, 9);
            var candidate = TmclCommandPacket.FromBytes(span);

            bool addrMatch = AcceptAnyModuleAddress || candidate.ModuleAddress == _controller.ModuleAddress;
            if (addrMatch && candidate.IsChecksumValid)
            {
                rxBuffer.RemoveRange(0, 9);
                TmclReplyPacket reply = _controller.ProcessCommand(candidate);

                // Cmd 137 (Restore factory settings) does not send back a reply per manual section 3.35
                if (candidate.CommandEnum != TmclCommand.RestoreFactorySettings)
                {
                    sendReply(reply.ToBytes());
                    RawFrameEvent?.Invoke("RX/TX", candidate, reply);
                }
            }
            else
            {
                // Check if another valid 9-byte frame starts within the buffer (sliding resync)
                bool foundSyncLater = false;
                for (int offset = 1; offset <= rxBuffer.Count - 9; offset++)
                {
                    byte sum = 0;
                    for (int k = 0; k < 8; k++) sum = unchecked((byte)(sum + rxBuffer[offset + k]));
                    if (sum == rxBuffer[offset + 8])
                    {
                        rxBuffer.RemoveRange(0, offset);
                        foundSyncLater = true;
                        break;
                    }
                }

                if (!foundSyncLater)
                {
                    // If exactly 9 bytes and address matches, reply WrongChecksum (Status=1) per TMCL spec
                    if (addrMatch)
                    {
                        rxBuffer.RemoveRange(0, 9);
                        TmclReplyPacket errReply = _controller.ProcessCommand(candidate);
                        sendReply(errReply.ToBytes());
                        RawFrameEvent?.Invoke("BAD_CRC", candidate, errReply);
                    }
                    else
                    {
                        rxBuffer.RemoveAt(0);
                    }
                }
            }
        }
    }

    private void OnControllerPacketProcessed(TmclCommandPacket cmd, TmclReplyPacket reply)
    {
        TraceGenerated?.Invoke(new ProtocolTraceEvent(
            ProtocolName,
            "RX -> TX",
            cmd.ToHexString(),
            cmd.ToMnemonicDescription(_controller.GetAxisDisplayLabel),
            reply.ToHexString(),
            reply.ToDescription(),
            cmd.IsPollingQuery));
    }

    private void OnControllerAsyncEventReplyGenerated(TmclReplyPacket asyncEventReply)
    {
        AsyncReplyBytesGenerated?.Invoke(asyncEventReply.ToBytes());
        RawFrameEvent?.Invoke("EVENT_138", null, asyncEventReply);
        StatusMessage?.Invoke(
            $"[Event 138] Sent Async Position Reached Reply (Status=128, Mask=0x{asyncEventReply.Value:X2}): {asyncEventReply.ToHexString()}");
    }

    public void Dispose()
    {
        _controller.PacketProcessed -= OnControllerPacketProcessed;
        _controller.AsyncEventReplyGenerated -= OnControllerAsyncEventReplyGenerated;
    }
}
