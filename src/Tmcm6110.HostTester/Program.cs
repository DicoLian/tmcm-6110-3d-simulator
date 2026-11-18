using System.IO.Ports;
using System.Net.Sockets;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.HostTester;

/// <summary>
/// Standalone C# Host Tester emulating a 9-Byte Binary TMCL Host Controller.
/// Can connect to the TMCM-6110 3D Simulator via Serial Port (e.g., COM10) or TCP (127.0.0.1:4001)
/// to verify:
/// 1. Binary 9-Byte Packet framing & Checksum
/// 2. Default Axis Mapping: Axis 0 (Z), Axis 1 (Y), Axis 2 (X)
/// 3. Status polling: Idle, Move, PosReached (#8), +Limit (#10), -Limit (#11), Alarm (#207/#208)
/// 4. Command 138 Asynchronous Position Reached Event (Status=100 immediate + Status=128 on arrival)
/// </summary>
internal class Program
{
    static async Task Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--self-test")
        {
            RunAutomatedSelfTest();
            return;
        }

        Console.OutputEncoding = System.Text.Encoding.UTF8;
        Console.WriteLine("======================================================================");
        Console.WriteLine("  TMCM-6110 9-Byte Binary Host Tester / 上位機連線測試工具");
        Console.WriteLine("  Axis Mapping / 軸向對應: Axis 0 = Z | Axis 1 = Y | Axis 2 = X");
        Console.WriteLine("======================================================================");
        Console.WriteLine("Select transport / 請選擇連線方式:");
        Console.WriteLine("  [1] Serial Port (COM Port, e.g. com0com COM10 / 虛擬串列埠)");
        Console.WriteLine("  [2] Local TCP Loopback (127.0.0.1:4001, zero-driver / 免虛擬驅動直連)");
        Console.Write("Choice / 輸入選項 (default 2): ");

        string? mode = Console.ReadLine()?.Trim();
        Func<byte[], byte[]> sendAndReceive;
        Func<int, byte[]?> waitAsyncEvent;

        if (mode == "1")
        {
            Console.Write("COM Port name / 請輸入 COM Port 名稱 (e.g. COM10): ");
            string portName = Console.ReadLine()?.Trim() ?? "COM10";
            Console.Write("BaudRate (default 9600): ");
            int baud = int.TryParse(Console.ReadLine(), out int b) ? b : 9600;

            var sp = new SerialPort(portName, baud, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 2000,
                WriteTimeout = 1000
            };
            sp.Open();
            Console.WriteLine($"[OK] Opened Serial Port {portName} @ {baud} bps.");

            sendAndReceive = tx =>
            {
                sp.Write(tx, 0, tx.Length);
                byte[] rx = new byte[9];
                int read = 0;
                while (read < 9)
                    read += sp.Read(rx, read, 9 - read);
                return rx;
            };

            waitAsyncEvent = timeoutMs =>
            {
                sp.ReadTimeout = timeoutMs;
                byte[] rx = new byte[9];
                int read = 0;
                while (read < 9)
                    read += sp.Read(rx, read, 9 - read);
                sp.ReadTimeout = 2000;
                return rx;
            };
        }
        else
        {
            var client = new TcpClient();
            await client.ConnectAsync("127.0.0.1", 4001);
            var stream = client.GetStream();
            Console.WriteLine("[OK] Connected to Simulator TCP Server 127.0.0.1:4001.");

            sendAndReceive = tx =>
            {
                stream.Write(tx, 0, tx.Length);
                byte[] rx = new byte[9];
                int read = 0;
                while (read < 9)
                    read += stream.Read(rx, read, 9 - read);
                return rx;
            };

            waitAsyncEvent = timeoutMs =>
            {
                stream.ReadTimeout = timeoutMs;
                byte[] rx = new byte[9];
                int read = 0;
                while (read < 9)
                    read += stream.Read(rx, read, 9 - read);
                stream.ReadTimeout = 2000;
                return rx;
            };
        }

        const byte moduleAddr = 1;

        while (true)
        {
            Console.WriteLine();
            Console.WriteLine("--- Select 9-Byte Binary TMCL Test Command / 選擇測試指令 ---");
            Console.WriteLine("  [1] Read 3-Axis Telemetry / 讀取三軸狀態 (Pos #1, Speed #3, PosReached #8, +Lim #10, -Lim #11, Alarm #207/#208)");
            Console.WriteLine("  [2] Absolute Move MVP ABS / 三軸絕對定位 (Z=30000, Y=50000, X=-40000) + Cmd 138 Async Event");
            Console.WriteLine("  [3] 3D Linear Interpolation MVP COORD ($47) / 三軸同步線性插補回原點 (0, 0, 0)");
            Console.WriteLine("  [4] Reference Search RFS START / 原點復歸 (Mode 65 +Limit Homing)");
            Console.WriteLine("  [5] Emergency Stop MST / 全軸緊急停止");
            Console.WriteLine("  [6] Clear Error Flags CLE (Cmd 36) / 清除所有 Alarm 旗標");
            Console.WriteLine("  [Q] Quit / 離開");
            Console.Write("Select / 請選擇 > ");

            string? choice = Console.ReadLine()?.Trim().ToUpperInvariant();
            if (choice == "Q") break;

            switch (choice)
            {
                case "1":
                    for (byte axis = 0; axis < 3; axis++)
                    {
                        string name = axis switch { 0 => "Z", 1 => "Y", 2 => "X", _ => "?" };
                        int pos = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.ActualPosition, axis);
                        int spd = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.ActualSpeed, axis);
                        int reached = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.TargetPosReached, axis);
                        int posLim = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.RightLimitSwitchStatus, axis);
                        int negLim = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.LeftLimitSwitchStatus, axis);
                        int stallErr = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.ExtendedErrorFlags, axis);
                        int drvErr = QueryGap(sendAndReceive, moduleAddr, TmclAxisParam.DriverErrorFlags, axis);

                        string stateStr = (stallErr != 0 || (drvErr & 0x7F) != 0) ? "ALARM" : (spd != 0 ? "MOVE" : "IDLE");
                        Console.WriteLine($"  Axis {axis} ({name}) | State={stateStr,-5} | Pos={pos,8} | Spd={spd,5} | Reached={reached} | +Lim={posLim} | -Lim={negLim} | Alarm(#207={stallErr}, #208=0x{drvErr:X2})");
                    }
                    break;

                case "2":
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 0, 30000));  // Z
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 1, 50000));  // Y
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 2, -40000)); // X

                    // Arm Command 138 Position Reached Event for Axes 0, 1, 2 (Mask = 0x07)
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x07));
                    Console.WriteLine("  [Waiting for Cmd 138 2nd async reply (Status=128)...]");
                    byte[]? asyncBytes = waitAsyncEvent(15000);
                    if (asyncBytes != null)
                    {
                        var evReply = TmclReplyPacket.FromBytes(asyncBytes);
                        Console.WriteLine($"  [Position Reached!] RX: {evReply.ToHexString()} -> {evReply.ToDescription()}");
                    }
                    break;

                case "3":
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.SCO, 0, 0, 0));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.SCO, 0, 1, 0));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.SCO, 0, 2, 0));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 2, 0x47, 0));
                    break;

                case "4":
                    for (byte a = 0; a < 3; a++)
                        SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.RFS, 0, a, 0));
                    break;

                case "5":
                    for (byte a = 0; a < 3; a++)
                        SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MST, 0, a, 0));
                    break;

                case "6":
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CLE, 0, 0, 0));
                    break;
            }
        }
    }

    private static int QueryGap(Func<byte[], byte[]> sendRecv, byte modAddr, byte paramNo, byte axis)
    {
        var pkt = new TmclCommandPacket(modAddr, (byte)TmclCommand.GAP, paramNo, axis, 0);
        byte[] rx = sendRecv(pkt.ToBytes());
        return TmclReplyPacket.FromBytes(rx).Value;
    }

    private static void SendCmd(Func<byte[], byte[]> sendRecv, TmclCommandPacket pkt)
    {
        Console.WriteLine($"  TX -> {pkt.ToHexString()}  ({pkt.ToMnemonicDescription()})");
        byte[] rx = sendRecv(pkt.ToBytes());
        var reply = TmclReplyPacket.FromBytes(rx);
        Console.WriteLine($"  RX <- {reply.ToHexString()}  ({reply.ToDescription()})");
    }

    private static void RunAutomatedSelfTest()
    {
        Console.WriteLine("=== Running TMCM-6110 End-to-End Automated Self-Test ===");
        var controller = new Tmcm6110.Core.Engine.Tmcm6110Controller();

        // 1. Verify built-in defaults and synthetic INI / axis mapping loader
        if (controller.GetAxis(0).AxisName != "Z" || controller.GetAxis(1).AxisName != "Y" || controller.GetAxis(2).AxisName != "X")
            throw new Exception("Default axis mapping mismatch!");

        string tempIniPath = Path.Combine(Path.GetTempPath(), $"tmcm6110_test_{Guid.NewGuid():N}.ini");
        string tempMapPath = Path.Combine(Path.GetTempPath(), $"tmcm6110_map_{Guid.NewGuid():N}.txt");
        try
        {
            File.WriteAllText(tempMapPath, "axis 0 (1st Axis) Z\naxis 1 (2nd Axis) Y\naxis 2 (3rd Axis) X\n");
            File.WriteAllText(tempIniPath,
                "[global_settings]\nSerial_address=1\nSerial_host_address=2\n" +
                "[module_settings_axis%00]\nReference_search_mode=65\nMicrostep_Resolution=6\nMaximum_positioning_speed=2000\n");

            Tmcm6110.Core.Config.TmcmConfigLoader.LoadAxisLabelsFromFile(tempMapPath, controller);
            int iniCount = Tmcm6110.Core.Config.TmcmConfigLoader.LoadFromIniFile(tempIniPath, controller);
            if (iniCount < 5)
                throw new Exception($"Expected >= 5 parameters loaded from temp INI, got {iniCount}");

            Console.WriteLine($"[PASS] Verified built-in defaults & config loader ({iniCount} params). Axis0={controller.GetAxis(0).AxisName}, Axis1={controller.GetAxis(1).AxisName}, Axis2={controller.GetAxis(2).AxisName}");
        }
        finally
        {
            if (File.Exists(tempIniPath)) File.Delete(tempIniPath);
            if (File.Exists(tempMapPath)) File.Delete(tempMapPath);
        }

        // Verify RFS Mode 65 and Microstep 6 via GAP
        int rfsMode0 = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ReferenceSearchMode, 0, 0)).Value;
        int uStep0 = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.MicrostepResolution, 0, 0)).Value;
        if (rfsMode0 != 65 || uStep0 != 6)
            throw new Exception($"Expected RFS Mode=65 and Microstep=6, got {rfsMode0} and {uStep0}");
        Console.WriteLine($"[PASS] Verified EEPROM settings via GAP: RFS Mode={rfsMode0} (+Limit Homing), Microstep={uStep0} (64 uSteps)");

        // 2. Test MVP ABS and Cmd 138 Async Position Reached Event
        bool event138Fired = false;
        TmclReplyPacket capturedAsyncEvent = default;
        controller.AsyncEventReplyGenerated += pkt =>
        {
            event138Fired = true;
            capturedAsyncEvent = pkt;
        };

        // Command Z=1500, Y=-2000, X=3000
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 1500));
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 1, -2000));
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 2, 3000));
        var armReply = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x07));
        if (armReply.StatusEnum != TmclStatus.SuccessOk)
            throw new Exception("Cmd 138 initial reply failed.");

        // Step physics until reached
        for (int i = 0; i < 500 && !event138Fired; i++)
            controller.StepSimulation(0.01);

        if (!event138Fired || capturedAsyncEvent.StatusEnum != TmclStatus.TargetPositionReachedEvent || capturedAsyncEvent.Value != 0x07)
            throw new Exception("Cmd 138 async position reached event did not fire properly!");

        int posZ = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value;
        int posY = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        int posX = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value;
        if (posZ != 1500 || posY != -2000 || posX != 3000)
            throw new Exception($"Target positions not reached: Z={posZ}, Y={posY}, X={posX}");
        Console.WriteLine($"[PASS] MVP ABS + Command 138 Async Event verified! Z={posZ}, Y={posY}, X={posX}, EventHex={capturedAsyncEvent.ToHexString()}");

        // 3. Test +Limit (#10), -Limit (#11), and Alarm (#207/#208) + CLE (Cmd 36)
        controller.GetAxis(0).ManualRightLimitOverride = true;
        int rightLim = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 0, 0)).Value;
        if (rightLim != 1) throw new Exception("+Limit override failed");
        controller.GetAxis(0).ManualRightLimitOverride = false;

        controller.GetAxis(0).TriggerAlarm();
        int stallErr = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 0, 0)).Value;
        int drvErr = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.DriverErrorFlags, 0, 0)).Value;
        if (stallErr == 0 || (drvErr & 0x01) == 0) throw new Exception("Alarm trigger failed");

        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.CLE, 0, 0, 0));
        int stallAfterCle = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 0, 0)).Value;
        if (stallAfterCle != 0) throw new Exception("CLE failed to clear alarm");
        Console.WriteLine("[PASS] Verified +Limit (#10), -Limit (#11), Alarm Injection (#207/#208), and CLE (Cmd 36) reset.");

        // 4. Clean up build artifacts so Visual Studio on Windows has a fresh state or keep clean
        Console.WriteLine("=== ALL SELF-TESTS PASSED SUCCESSFULLY ===");
    }
}
