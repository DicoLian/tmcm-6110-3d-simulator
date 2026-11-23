using System.IO.Ports;
using Tmcm6110.Core.Abstractions;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Transport;

/// <summary>
/// Backwards-compatible facade combining <see cref="ByteStreamTransportServer"/> and <see cref="TmclProtocolAdapter"/>
/// for Serial Port (COM) and TCP Socket communication with 9-Byte Binary TMCL Packets.
/// </summary>
public sealed class TmclSerialServer : IDisposable
{
    private readonly TmclProtocolAdapter _adapter;
    private readonly ByteStreamTransportServer _transport;

    public IProtocolAdapter ProtocolAdapter => _adapter;
    public ByteStreamTransportServer TransportServer => _transport;

    public bool IsSerialOpen => _transport.IsSerialOpen;
    public string ActivePortName => _transport.ActivePortName;
    public int ActiveBaudRate => _transport.ActiveBaudRate;
    public bool IsTcpListening => _transport.IsTcpListening;
    public int TcpPort => _transport.TcpPort;

    /// <summary>
    /// If true, responds to any incoming Module Address (helpful if Host sends to addr 1 or 0).
    /// If false, strictly checks that byte[0] matches Controller.ModuleAddress.
    /// </summary>
    public bool AcceptAnyModuleAddress
    {
        get => _adapter.AcceptAnyModuleAddress;
        set => _adapter.AcceptAnyModuleAddress = value;
    }

    public event Action<string>? StatusLogMessage;
    public event Action<string, TmclCommandPacket?, TmclReplyPacket?>? RawFrameEvent;

    public TmclSerialServer(Tmcm6110Controller controller)
    {
        _adapter = new TmclProtocolAdapter(controller);
        _transport = new ByteStreamTransportServer(_adapter);
        _transport.StatusLogMessage += OnTransportStatusLogMessage;
        _adapter.RawFrameEvent += OnAdapterRawFrameEvent;
    }

    private void OnTransportStatusLogMessage(string msg) => StatusLogMessage?.Invoke(msg);

    private void OnAdapterRawFrameEvent(string tag, TmclCommandPacket? cmd, TmclReplyPacket? reply) =>
        RawFrameEvent?.Invoke(tag, cmd, reply);

    public static string[] GetAvailableComPorts() => ByteStreamTransportServer.GetAvailableComPorts();

    public void OpenSerialPort(string portName, int baudRate = 9600, Parity parity = Parity.None, int dataBits = 8, StopBits stopBits = StopBits.One) =>
        _transport.OpenSerialPort(portName, baudRate, parity, dataBits, stopBits);

    public void CloseSerialPort() => _transport.CloseSerialPort();

    public void StartTcpServer(int port = 4001) => _transport.StartTcpServer(port);

    public void StopTcpServer() => _transport.StopTcpServer();

    public void Dispose()
    {
        _transport.StatusLogMessage -= OnTransportStatusLogMessage;
        _adapter.RawFrameEvent -= OnAdapterRawFrameEvent;
        _transport.Dispose();
        _adapter.Dispose();
    }
}
