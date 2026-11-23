using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using Tmcm6110.Core.Abstractions;

namespace Tmcm6110.Core.Transport;

/// <summary>
/// Protocol-agnostic Serial Port (COM) and TCP Socket byte-stream transport server.
/// Routes incoming byte streams into any <see cref="IProtocolAdapter"/> and forwards asynchronous adapter replies.
/// </summary>
public sealed class ByteStreamTransportServer : IDisposable
{
    private readonly object _txLock = new();
    private IProtocolAdapter _adapter;
    private SerialPort? _serialPort;
    private CancellationTokenSource? _cts;
    private TcpListener? _tcpListener;
    private TcpClient? _connectedTcpClient;

    public IProtocolAdapter Adapter => _adapter;
    public bool IsSerialOpen => _serialPort is { IsOpen: true };
    public string ActivePortName => _serialPort?.PortName ?? string.Empty;
    public int ActiveBaudRate => _serialPort?.BaudRate ?? 9600;
    public bool IsTcpListening { get; private set; }
    public int TcpPort { get; private set; } = 4001;

    public event Action<string>? StatusLogMessage;

    public ByteStreamTransportServer(IProtocolAdapter adapter)
    {
        _adapter = adapter ?? throw new ArgumentNullException(nameof(adapter));
        SubscribeAdapter(_adapter);
    }

    public void SetProtocolAdapter(IProtocolAdapter adapter)
    {
        if (adapter == null) throw new ArgumentNullException(nameof(adapter));
        if (ReferenceEquals(_adapter, adapter)) return;

        UnsubscribeAdapter(_adapter);
        _adapter = adapter;
        SubscribeAdapter(_adapter);
        StatusLogMessage?.Invoke($"[Transport] Active protocol switched to {_adapter.ProtocolName}.");
    }

    private void SubscribeAdapter(IProtocolAdapter adapter)
    {
        adapter.AsyncReplyBytesGenerated += OnAdapterAsyncReplyBytesGenerated;
        adapter.StatusMessage += OnAdapterStatusMessage;
    }

    private void UnsubscribeAdapter(IProtocolAdapter adapter)
    {
        adapter.AsyncReplyBytesGenerated -= OnAdapterAsyncReplyBytesGenerated;
        adapter.StatusMessage -= OnAdapterStatusMessage;
    }

    private void OnAdapterStatusMessage(string msg) => StatusLogMessage?.Invoke(msg);

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

        StatusLogMessage?.Invoke($"[Serial] Opened {portName} @ {baudRate} bps ({dataBits}-{parity}-{stopBits}). Ready for {_adapter.ProtocolName}.");
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
            StatusLogMessage?.Invoke($"[TCP] Listening on 127.0.0.1:{port} for {_adapter.ProtocolName}.");
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

                    _adapter.ProcessIncomingBytes(buffer, replyBytes =>
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

                            _adapter.ProcessIncomingBytes(buffer, replyBytes =>
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

    private void OnAdapterAsyncReplyBytesGenerated(byte[] bytes)
    {
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
                StatusLogMessage?.Invoke($"[Async Reply TX Error] {ex.Message}");
            }
        }
    }

    public void Dispose()
    {
        UnsubscribeAdapter(_adapter);
        CloseSerialPort();
        StopTcpServer();
    }
}
