using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Transport;

/// <summary>
/// Provides Serial Port (COM Port) and TCP Socket Server listeners for 9-Byte Binary TMCL Packets.
/// Also handles asynchronous transmission of Command 138 Target Position Reached Event (Status = 128).
/// </summary>
public sealed class TmclSerialServer : IDisposable
{
    private readonly Tmcm6110Controller _controller;
    private readonly object _txLock = new();
    private SerialPort? _serialPort;
    private CancellationTokenSource? _cts;
    private TcpListener? _tcpListener;
    private TcpClient? _connectedTcpClient;

    public bool IsSerialOpen => _serialPort is { IsOpen: true };
    public string ActivePortName => _serialPort?.PortName ?? string.Empty;
    public int ActiveBaudRate => _serialPort?.BaudRate ?? 9600;
    public bool IsTcpListening { get; private set; }
    public int TcpPort { get; private set; } = 4001;

    /// <summary>
    /// If true, responds to any incoming Module Address (helpful if Host sends to addr 1 or 0).
    /// If false, strictly checks that byte[0] matches Controller.ModuleAddress.
    /// </summary>
    public bool AcceptAnyModuleAddress { get; set; } = true;

    public event Action<string>? StatusLogMessage;
    public event Action<string, TmclCommandPacket?, TmclReplyPacket?>? RawFrameEvent;

    public TmclSerialServer(Tmcm6110Controller controller)
    {
        _controller = controller;
        _controller.AsyncEventReplyGenerated += OnAsyncEventReplyGenerated;
    }

    public static string[] GetAvailableComPorts()
    {
        try
        {
            var ports = SerialPort.GetPortNames();
            Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
            return ports;
        }
        catch
        {
            return Array.Empty<string>();
        }
    }

    public void OpenSerialPort(string portName, int baudRate = 9600, Parity parity = Parity.None, int dataBits = 8, StopBits stopBits = StopBits.One)
    {
        CloseSerialPort();

        _serialPort = new SerialPort(portName, baudRate, parity, dataBits, stopBits)
        {
            ReadTimeout = 50,
            WriteTimeout = 500,
            DtrEnable = true,
            RtsEnable = true
        };

        _serialPort.Open();
        _cts = new CancellationTokenSource();
        Task.Run(() => SerialReadLoopAsync(_serialPort, _cts.Token));

        StatusLogMessage?.Invoke($"[Serial] Opened {portName} @ {baudRate} bps ({dataBits}-{parity}-{stopBits}). Ready for 9-Byte Binary TMCL.");
    }

    public void CloseSerialPort()
    {
        try
        {
            _cts?.Cancel();
            if (_serialPort != null)
            {
                string portName = _serialPort.PortName;
                if (_serialPort.IsOpen)
                    _serialPort.Close();
                _serialPort.Dispose();
                _serialPort = null;
                StatusLogMessage?.Invoke($"[Serial] Closed {portName}.");
            }
        }
        catch (Exception ex)
        {
            StatusLogMessage?.Invoke($"[Serial] Error closing port: {ex.Message}");
        }
    }

    public void StartTcpServer(int port = 4001)
    {
        StopTcpServer();
        try
        {
            TcpPort = port;
            _tcpListener = new TcpListener(IPAddress.Any, port);
            _tcpListener.Start();
            IsTcpListening = true;
            _ = Task.Run(() => TcpAcceptLoopAsync(_tcpListener));
            StatusLogMessage?.Invoke($"[TCP] Listening on 127.0.0.1:{port} for 9-Byte Binary TMCL packets.");
        }
        catch (Exception ex)
        {
            IsTcpListening = false;
            StatusLogMessage?.Invoke($"[TCP] Failed to start listener on port {port}: {ex.Message}");
        }
    }

    public void StopTcpServer()
    {
        try
        {
            IsTcpListening = false;
            _connectedTcpClient?.Close();
            _connectedTcpClient = null;
            _tcpListener?.Stop();
            _tcpListener = null;
        }
        catch
        {
            // Ignore shutdown errors
        }
    }

    private async Task SerialReadLoopAsync(SerialPort port, CancellationToken ct)
    {
        var buffer = new List<byte>(64);
        var temp = new byte[64];

        while (!ct.IsCancellationRequested && port.IsOpen)
        {
            try
            {
                int bytesToRead = port.BytesToRead;
                if (bytesToRead > 0)
                {
                    int read = port.Read(temp, 0, Math.Min(temp.Length, bytesToRead));
                    for (int i = 0; i < read; i++)
                        buffer.Add(temp[i]);

                    ProcessIncomingBuffer(buffer, replyBytes =>
                    {
                        lock (_txLock)
                        {
                            if (port.IsOpen)
                                port.Write(replyBytes, 0, replyBytes.Length);
                        }
                    });
                }
                else
                {
                    await Task.Delay(2, ct).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (TimeoutException)
            {
                // Normal when no bytes arrive within ReadTimeout
            }
            catch (Exception ex)
            {
                if (!ct.IsCancellationRequested)
                    StatusLogMessage?.Invoke($"[Serial] Read error: {ex.Message}");
                await Task.Delay(50, ct).ConfigureAwait(false);
            }
        }
    }

    private async Task TcpAcceptLoopAsync(TcpListener listener)
    {
        while (IsTcpListening)
        {
            try
            {
                var client = await listener.AcceptTcpClientAsync().ConfigureAwait(false);
                _connectedTcpClient = client;
                StatusLogMessage?.Invoke($"[TCP] Host connected from {client.Client.RemoteEndPoint}.");

                _ = Task.Run(async () =>
                {
                    using (client)
                    {
                        var stream = client.GetStream();
                        var buffer = new List<byte>(64);
                        var temp = new byte[64];
                        while (client.Connected && IsTcpListening)
                        {
                            int read = await stream.ReadAsync(temp, 0, temp.Length).ConfigureAwait(false);
                            if (read == 0) break;
                            for (int i = 0; i < read; i++)
                                buffer.Add(temp[i]);

                            ProcessIncomingBuffer(buffer, replyBytes =>
                            {
                                lock (_txLock)
                                {
                                    if (client.Connected)
                                        stream.Write(replyBytes, 0, replyBytes.Length);
                                }
                            });
                        }
                    }
                    StatusLogMessage?.Invoke("[TCP] Host disconnected.");
                });
            }
            catch
            {
                if (!IsTcpListening) break;
            }
        }
    }

    /// <summary>
    /// Synchronizes and extracts 9-byte TMCL binary frames from the byte stream.
    /// Uses sliding-window checksum verification to automatically recover from any noise byte on RS485/Serial.
    /// </summary>
    private void ProcessIncomingBuffer(List<byte> buffer, Action<byte[]> sendReplyCallback)
    {
        while (buffer.Count >= 9)
        {
            var span = new byte[9];
            buffer.CopyTo(0, span, 0, 9);
            var candidate = TmclCommandPacket.FromBytes(span);

            bool addrMatch = AcceptAnyModuleAddress || candidate.ModuleAddress == _controller.ModuleAddress;
            if (addrMatch && candidate.IsChecksumValid)
            {
                buffer.RemoveRange(0, 9);
                TmclReplyPacket reply = _controller.ProcessCommand(candidate);

                // Cmd 137 (Restore factory settings) does not send back a reply per manual section 3.35
                if (candidate.CommandEnum != TmclCommand.RestoreFactorySettings)
                {
                    byte[] replyBytes = reply.ToBytes();
                    sendReplyCallback(replyBytes);
                    RawFrameEvent?.Invoke("RX/TX", candidate, reply);
                }
            }
            else
            {
                // Check if another valid 9-byte frame starts within the buffer (sliding resync)
                bool foundSyncLater = false;
                for (int offset = 1; offset <= buffer.Count - 9; offset++)
                {
                    byte sum = 0;
                    for (int k = 0; k < 8; k++) sum = unchecked((byte)(sum + buffer[offset + k]));
                    if (sum == buffer[offset + 8])
                    {
                        buffer.RemoveRange(0, offset);
                        foundSyncLater = true;
                        break;
                    }
                }

                if (!foundSyncLater)
                {
                    // If exactly 9 bytes and address matches, reply WrongChecksum (Status=1) per TMCL spec
                    if (addrMatch)
                    {
                        buffer.RemoveRange(0, 9);
                        TmclReplyPacket errReply = _controller.ProcessCommand(candidate);
                        sendReplyCallback(errReply.ToBytes());
                        RawFrameEvent?.Invoke("BAD_CRC", candidate, errReply);
                    }
                    else
                    {
                        buffer.RemoveAt(0);
                    }
                }
            }
        }
    }

    private void OnAsyncEventReplyGenerated(TmclReplyPacket asyncEventReply)
    {
        byte[] bytes = asyncEventReply.ToBytes();
        lock (_txLock)
        {
            try
            {
                if (_serialPort is { IsOpen: true })
                {
                    _serialPort.Write(bytes, 0, bytes.Length);
                }
                if (_connectedTcpClient is { Connected: true })
                {
                    _connectedTcpClient.GetStream().Write(bytes, 0, bytes.Length);
                }
            }
            catch (Exception ex)
            {
                StatusLogMessage?.Invoke($"[Event 138 TX Error] {ex.Message}");
            }
        }
        RawFrameEvent?.Invoke("EVENT_138", null, asyncEventReply);
        StatusLogMessage?.Invoke($"[Event 138] Sent Async Position Reached Reply (Status=128, Mask=0x{asyncEventReply.Value:X2}): {asyncEventReply.ToHexString()}");
    }

    public void Dispose()
    {
        _controller.AsyncEventReplyGenerated -= OnAsyncEventReplyGenerated;
        CloseSerialPort();
        StopTcpServer();
    }
}
