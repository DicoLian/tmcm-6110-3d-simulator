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
            Console.WriteLine("  [2] Absolute Move MVP ABS / 三軸絕對定位 (Z=30000, Y=50000, X=-40000) + Pre-Armed Cmd 138");
            Console.WriteLine("  [3] 3D Linear Interpolation MVP COORD ($47) / 三軸同步線性插補回原點 (0, 0, 0)");
            Console.WriteLine("  [4] Reference Search RFS START / 原點復歸 (Mode 65 +Limit Homing)");
            Console.WriteLine("  [5] Soft Stop MST / 全軸緩減速停止");
            Console.WriteLine("  [6] FW 1.47 Info (Cmd 136), Clear #207 Auto-Clear, CLE & Software Reset (Cmd 255)");
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
                    // FW 1.47: Arm Cmd 138 (Type=1 continuous for multiple MVPs) BEFORE issuing MVP commands
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.RequestTargetPositionReachedEvent, 1, 0, 0x07));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 0, 30000));  // Z
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 1, 50000));  // Y
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, 0, 2, -40000)); // X

                    Console.WriteLine("  [Waiting for Cmd 138 async reply (Status=128) across axes 0x07...]");
                    int receivedMask = 0;
                    while ((receivedMask & 0x07) != 0x07)
                    {
                        byte[]? asyncBytes = waitAsyncEvent(15000);
                        if (asyncBytes == null)
                        {
                            Console.WriteLine("  [Timeout waiting for remaining Cmd 138 events]");
                            break;
                        }
                        var evReply = TmclReplyPacket.FromBytes(asyncBytes);
                        receivedMask |= evReply.Value;
                        Console.WriteLine($"  [Position Reached!] RX: {evReply.ToHexString()} -> {evReply.ToDescription()} (AccumMask=0x{receivedMask:X2})");
                    }
                    // Disarm continuous Cmd 138 notifications
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0));
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
                    Console.WriteLine("  --- Cmd 136 Firmware Info & FW 1.47 Reset ---");
                    byte[] fwRaw = sendAndReceive(new TmclCommandPacket(moduleAddr, (byte)TmclCommand.GetFirmwareVersion, 0, 0, 0).ToBytes());
                    Console.WriteLine($"  RX (Type=0 ASCII) <- {BitConverter.ToString(fwRaw).Replace("-", " ")} (\"{System.Text.Encoding.ASCII.GetString(fwRaw, 1, 8)}\")");
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.GetFirmwareVersion, 1, 0, 0));
                    for (byte a = 0; a < 3; a++)
                        SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, a, 0));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CLE, 0, 0, 0));
                    SendCmd(sendAndReceive, new TmclCommandPacket(moduleAddr, (byte)TmclCommand.SoftwareReset, 0, 0, 1234));
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
        Console.WriteLine("=== Running TMCM-6110 FW V1.47 End-to-End Automated Self-Test ===");
        var controller = new Tmcm6110.Core.Engine.Tmcm6110Controller();

        // 0. Verify FW V1.47 identity and Cmd 136 firmware version payloads
        if (controller.ControllerName != "TMCM-6110 (TMCL FW V1.47)")
            throw new Exception($"Unexpected controller name: {controller.ControllerName}");

        byte[] fwAsciiBytes = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GetFirmwareVersion, 0, 0, 0)).ToBytes();
        string fwAscii = System.Text.Encoding.ASCII.GetString(fwAsciiBytes, 1, 8);
        int fwBinary = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GetFirmwareVersion, 1, 0, 0)).Value;
        if (fwAscii != "6110V147" || fwBinary != 0x17DE012F)
            throw new Exception($"FW version mismatch: ASCII=\"{fwAscii}\", Binary=0x{fwBinary:X8}");
        Console.WriteLine($"[PASS] Verified Cmd 136 Firmware Version: ASCII=\"{fwAscii}\", Binary=0x{fwBinary:X8}");

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

        // 2. Test Cmd 138 Pre-MVP Type 0 (one-shot) & Type 1 (continuous) + MVP ABS
        var capturedEvents = new List<TmclReplyPacket>();
        controller.AsyncEventReplyGenerated += pkt => capturedEvents.Add(pkt);

        // 2a. Type 0 one-shot: arm BEFORE single MVP on Axis 0
        var oneShotAck = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0x01));
        if (oneShotAck.StatusEnum != TmclStatus.SuccessOk)
            throw new Exception("Cmd 138 Type 0 ack failed.");
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 500));
        for (int i = 0; i < 200 && capturedEvents.Count == 0; i++)
            controller.StepSimulation(0.01);
        if (capturedEvents.Count != 1 || capturedEvents[0].StatusEnum != TmclStatus.TargetPositionReachedEvent || capturedEvents[0].Value != 0x01)
            throw new Exception("Cmd 138 Type 0 one-shot event did not fire properly!");

        // Subsequent MVP without re-arming must NOT fire Type 0 again
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 0));
        for (int i = 0; i < 200; i++)
            controller.StepSimulation(0.01);
        if (capturedEvents.Count != 1)
            throw new Exception("Cmd 138 Type 0 fired more than once!");

        // 2b. Type 1 continuous: arm BEFORE multi-axis MVPs Z=1500, Y=-2000, X=3000
        capturedEvents.Clear();
        var contAck = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 1, 0, 0x07));
        if (contAck.StatusEnum != TmclStatus.SuccessOk)
            throw new Exception("Cmd 138 Type 1 ack failed.");

        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 0, 1500));
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 1, -2000));
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 2, 3000));

        int receivedMask = 0;
        for (int i = 0; i < 500 && (receivedMask & 0x07) != 0x07; i++)
        {
            controller.StepSimulation(0.01);
            foreach (var ev in capturedEvents)
            {
                if (ev.StatusEnum == TmclStatus.TargetPositionReachedEvent)
                    receivedMask |= ev.Value;
            }
        }
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.RequestTargetPositionReachedEvent, 0, 0, 0));

        if ((receivedMask & 0x07) != 0x07)
            throw new Exception($"Cmd 138 Type 1 event mask mismatch: got 0x{receivedMask:X2}");

        int posZ = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 0, 0)).Value;
        int posY = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 1, 0)).Value;
        int posX = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 2, 0)).Value;
        if (posZ != 1500 || posY != -2000 || posX != 3000)
            throw new Exception($"Target positions not reached: Z={posZ}, Y={posY}, X={posX}");
        Console.WriteLine($"[PASS] Pre-armed Cmd 138 (Type 0 & Type 1) + MVP ABS verified! Z={posZ}, Y={posY}, X={posX}, Mask=0x{receivedMask:X2}");

        // 3. Test 32-bit position range beyond +/-8,388,607 on Axis 3
        controller.GetAxis(3).SetVirtualStrokeLimits(-20_000_000, 20_000_000);
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.SAP, TmclAxisParam.MaxPositioningSpeed, 3, 2000));
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.MVP, 0, 3, 10_000_000));
        for (int i = 0; i < 4500; i++)
        {
            controller.StepSimulation(0.1);
            if (controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.TargetPosReached, 3, 0)).Value == 1)
                break;
        }
        int pos32Bit = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ActualPosition, 3, 0)).Value;
        if (pos32Bit != 10_000_000)
            throw new Exception($"32-bit MVP ABS failed: expected 10000000, got {pos32Bit}");
        Console.WriteLine($"[PASS] Verified 32-bit signed positioning on Axis 3: Pos={pos32Bit}");

        // 4. Test +Limit (#10), Alarm (#208/#207 auto-clear on read), and ClearAllFaults + GIO 12-bit/Port 8
        controller.GetAxis(0).ManualRightLimitOverride = true;
        int rightLim = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.RightLimitSwitchStatus, 0, 0)).Value;
        if (rightLim != 1) throw new Exception("+Limit override failed");
        controller.GetAxis(0).ManualRightLimitOverride = false;

        controller.GetAxis(0).TriggerAlarm();
        // Check #208 BEFORE #207 because GAP #207 auto-clears StallGuard2 flags on read in FW 1.47
        int drvErr = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.DriverErrorFlags, 0, 0)).Value;
        int stallErrFirstRead = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 0, 0)).Value;
        int stallErrSecondRead = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.ExtendedErrorFlags, 0, 0)).Value;
        if ((drvErr & 0x01) == 0 || stallErrFirstRead != 1 || stallErrSecondRead != 0)
            throw new Exception($"Alarm/auto-clear mismatch: drvErr=0x{drvErr:X}, first#207={stallErrFirstRead}, second#207={stallErrSecondRead}");

        controller.ClearAllFaults();
        int drvAfterClear = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GAP, TmclAxisParam.DriverErrorFlags, 0, 0)).Value;
        if ((drvAfterClear & ~0x80) != 0)
            throw new Exception($"ClearAllFaults failed to clear TMC262 flags: 0x{drvAfterClear:X}");

        controller.SetAnalogInput(0, 4095);
        int ain0 = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GIO, 0, 1, 0)).Value;
        int supplyVoltage = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GIO, 8, 1, 0)).Value;
        if (ain0 != 4095 || supplyVoltage != 240)
            throw new Exception($"GIO analog/supply mismatch: AIN0={ain0}, Port8={supplyVoltage}");
        Console.WriteLine($"[PASS] Verified +Limit (#10), #207 auto-clear on read, TMC262 fault clear, & GIO (AIN0={ain0}, V={supplyVoltage / 10.0:0.0}V).");

        // 5. Verify Cmd 255 Software Reset (1234)
        var badReset = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.SoftwareReset, 0, 0, 999));
        if (badReset.StatusEnum != TmclStatus.InvalidValue)
            throw new Exception("Cmd 255 should reject value != 1234");
        controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.SCO, 1, 0, 777));
        var resetOk = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.SoftwareReset, 0, 0, 1234));
        if (resetOk.StatusEnum != TmclStatus.SuccessOk)
            throw new Exception("Cmd 255 software reset (1234) failed");
        int ramCoordAfterReset = controller.ProcessCommand(new TmclCommandPacket(1, (byte)TmclCommand.GCO, 1, 0, 0)).Value;
        if (ramCoordAfterReset != 0)
            throw new Exception($"Expected RAM coordinate reset to 0 after Cmd 255, got {ramCoordAfterReset}");
        Console.WriteLine("[PASS] Verified Cmd 255 Software Reset (1234).");

        Console.WriteLine("=== ALL SELF-TESTS PASSED SUCCESSFULLY ===");
    }
}
