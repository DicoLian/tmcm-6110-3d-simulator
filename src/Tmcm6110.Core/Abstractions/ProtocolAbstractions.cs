namespace Tmcm6110.Core.Abstractions;

/// <summary>
/// Protocol-agnostic trace entry consumed by the live Packet Sniffer log.
/// </summary>
public readonly record struct ProtocolTraceEvent(
    string ProtocolName,
    string Direction,
    string RawRequest,
    string DecodedRequest,
    string RawReply,
    string DecodedReply,
    bool IsPollingQuery
);

/// <summary>
/// Translates incoming raw byte streams for a specific motion protocol into commands on <see cref="IMotionController"/>.
/// </summary>
public interface IProtocolAdapter : IDisposable
{
    string ProtocolName { get; }
    IMotionController Controller { get; }

    event Action<byte[]>? AsyncReplyBytesGenerated;
    event Action<ProtocolTraceEvent>? TraceGenerated;
    event Action<string>? StatusMessage;

    void ProcessIncomingBytes(List<byte> rxBuffer, Action<byte[]> sendReply);
}
