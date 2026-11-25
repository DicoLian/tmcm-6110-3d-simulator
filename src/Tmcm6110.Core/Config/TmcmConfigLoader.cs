using System.Text.RegularExpressions;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Config;

/// <summary>
/// Loads TMCM-6110 configuration files:
/// 1. TMCL-IDE Setup Wizard .ini export
/// 2. TMCL .tmc SAP/SGP script export
/// 3. Axis mapping .txt file (e.g., axis 0 = Z, axis 1 = Y, axis 2 = X)
/// </summary>
public static class TmcmConfigLoader
{
    private static readonly Dictionary<string, byte> IniAxisKeyToParam = new(StringComparer.OrdinalIgnoreCase)
    {
        ["Maximum_positioning_speed"] = TmclAxisParam.MaxPositioningSpeed,
        ["Maximum_acceleration"] = TmclAxisParam.MaxAcceleration,
        ["Maximum_current"] = TmclAxisParam.AbsoluteMaxCurrent,
        ["Standby_current"] = TmclAxisParam.StandbyCurrent,
        ["Right_limit_switch_disable"] = TmclAxisParam.RightLimitSwitchDisable,
        ["Left_limit_switch_disable"] = TmclAxisParam.LeftLimitSwitchDisable,
        ["Minimum_speed"] = TmclAxisParam.MinimumSpeed,
        ["Microstep_Resolution"] = TmclAxisParam.MicrostepResolution,
        ["Soft_stop_flag"] = TmclAxisParam.SoftStopFlag,
        ["End_switch_power_down_mode"] = TmclAxisParam.EndSwitchPowerDownMode,
        ["Ramp_divisor"] = TmclAxisParam.RampDivisor,
        ["Pulse_divisor"] = TmclAxisParam.PulseDivisor,
        ["Toggle_step_interpolation"] = TmclAxisParam.StepInterpolation,
        ["Toggle_double_step"] = TmclAxisParam.DoubleStep,
        ["Chopper_blank_time"] = TmclAxisParam.ChopperBlankTime,
        ["Constant_TOff_Mode"] = TmclAxisParam.ConstantTOffMode,
        ["Disable_fast_decay_comparator"] = TmclAxisParam.DisableFastDecayComparator,
        ["Chopper_hysteresis_end_\\_fast_decay_time"] = TmclAxisParam.ChopperHysteresisEnd,
        ["Chopper_hysteresis_start_\\_sine_wave_offset"] = TmclAxisParam.ChopperHysteresisStart,
        ["Chopper_off_time"] = TmclAxisParam.ChopperOffTime,
        ["smartEnergy_current_minimum"] = TmclAxisParam.SmartEnergyCurrentMin,
        ["smartEnergy_current_down_step"] = TmclAxisParam.SmartEnergyCurrentDownStep,
        ["smartEnergy_hysteresis"] = TmclAxisParam.SmartEnergyHysteresis,
        ["smartEnergy_current_up_step"] = TmclAxisParam.SmartEnergyCurrentUpStep,
        ["smartEnergy_hysteresis_start"] = TmclAxisParam.SmartEnergyHysteresisStart,
        ["stallGuard2_filter_enable"] = TmclAxisParam.StallGuard2FilterEnable,
        ["stallGuard2_threshold"] = TmclAxisParam.StallGuard2Threshold,
        ["Slope_control_high_side"] = TmclAxisParam.SlopeControlHighSide,
        ["Slope_control_low_side"] = TmclAxisParam.SlopeControlLowSide,
        ["Short_to_Ground_Protection"] = TmclAxisParam.ShortToGroundProtection,
        ["Short_detection_timer"] = TmclAxisParam.ShortDetectionTimer,
        ["VSense"] = TmclAxisParam.VSense,
        ["smartEnergy_stall_velocity"] = TmclAxisParam.SmartEnergyStallVelocity,
        ["smartEnergy_threshold_speed"] = TmclAxisParam.SmartEnergyThresholdSpeed,
        ["smartEnergy_slow_run_current"] = TmclAxisParam.SmartEnergySlowRunCurrent,
        ["Random_chopper_off_time"] = TmclAxisParam.RandomChopperOffTime,
        ["Reference_search_mode"] = TmclAxisParam.ReferenceSearchMode,
        ["Reference_search_speed"] = TmclAxisParam.ReferenceSearchSpeed,
        ["Reference_switch_speed"] = TmclAxisParam.ReferenceSwitchSpeed,
        ["Boost_current"] = TmclAxisParam.BoostCurrent,
        ["Freewheeling_delay"] = TmclAxisParam.FreewheelingDelay,
        ["Group_index"] = TmclAxisParam.GroupIndex,
        ["Power_down_delay"] = TmclAxisParam.PowerDownDelay
    };

    private static readonly Dictionary<string, byte> IniGlobalKeyToParam = new(StringComparer.OrdinalIgnoreCase)
    {
        ["RS485_baud_rate"] = 65,
        ["Serial_address"] = 66,
        ["ASCII_mode"] = 67,
        ["Serial_heartbeat"] = 68,
        ["CAN_Bitrate"] = 69,
        ["CAN_Send_Id"] = 70,
        ["CAN_Receive_Id"] = 71,
        ["Telegram_pause_time"] = 75,
        ["Serial_host_address"] = 76,
        ["Auto_start_mode"] = 77,
        ["Limit_switch_polarity"] = 79,
        ["TMCL_Protection_Mode"] = 81,
        ["CAN_heartbeat"] = 82,
        ["CAN_secondary_address"] = 83,
        ["EEPROM_Coordinate_Store"] = 84,
        ["Do_not_restore_user_variables"] = 85,
        ["Secondary_serial_address"] = 87,
        ["Reverse_shaft"] = 90
    };

    public static int LoadFromIniFile(string filePath, Tmcm6110Controller controller)
    {
        if (!File.Exists(filePath))
            throw new FileNotFoundException("INI file not found.", filePath);

        int appliedCount = 0;
        int currentAxis = -1;
        bool inGlobalSection = false;

        foreach (string rawLine in File.ReadLines(filePath))
        {
            string line = rawLine.Trim();
            if (string.IsNullOrEmpty(line) || line.StartsWith(';') || line.StartsWith('#'))
                continue;

            if (line.StartsWith('[') && line.EndsWith(']'))
            {
                string section = line[1..^1].Trim();
                var axisMatch = Regex.Match(section, @"module_settings_axis%0*(\d+)", RegexOptions.IgnoreCase);
                if (axisMatch.Success && int.TryParse(axisMatch.Groups[1].Value, out int axIdx))
                {
                    currentAxis = axIdx;
                    inGlobalSection = false;
                }
                else if (section.Contains("global_settings", StringComparison.OrdinalIgnoreCase))
                {
                    currentAxis = -1;
                    inGlobalSection = true;
                }
                else
                {
                    currentAxis = -1;
                    inGlobalSection = false;
                }
                continue;
            }

            int eqPos = line.IndexOf('=');
            if (eqPos <= 0) continue;

            string key = line[..eqPos].Trim();
            string valStr = line[(eqPos + 1)..].Trim();
            if (!int.TryParse(valStr, out int val)) continue;

            if (currentAxis >= 0 && currentAxis < Tmcm6110Controller.TotalAxes)
            {
                if (IniAxisKeyToParam.TryGetValue(key, out byte paramNo))
                {
                    controller.GetAxis(currentAxis).SetParameter(paramNo, val);
                    controller.GetAxis(currentAxis).StoreParameterToEeprom(paramNo);
                    appliedCount++;
                }
            }
            else if (inGlobalSection)
            {
                if (IniGlobalKeyToParam.TryGetValue(key, out byte gParamNo))
                {
                    controller.SetGlobalParameter(0, gParamNo, val);
                    appliedCount++;
                }
            }
        }

        return appliedCount;
    }

    public static int LoadFromTmcFile(string filePath, Tmcm6110Controller controller)
    {
        if (!File.Exists(filePath))
            throw new FileNotFoundException("TMC file not found.", filePath);

        int appliedCount = 0;
        var sapSgpRegex = new Regex(@"^\s*(SAP|SGP)\s+(\d+)\s*,\s*(\d+)\s*,\s*(-?\d+)", RegexOptions.IgnoreCase);

        foreach (string rawLine in File.ReadLines(filePath))
        {
            string line = rawLine.Split("//")[0].Trim();
            if (string.IsNullOrEmpty(line)) continue;

            var match = sapSgpRegex.Match(line);
            if (!match.Success) continue;

            string cmd = match.Groups[1].Value.ToUpperInvariant();
            if (!byte.TryParse(match.Groups[2].Value, out byte typeNo)) continue;
            if (!byte.TryParse(match.Groups[3].Value, out byte motOrBank)) continue;
            if (!int.TryParse(match.Groups[4].Value, out int val)) continue;

            if (cmd == "SAP" && motOrBank < Tmcm6110Controller.TotalAxes && TmclAxisParam.IsValidParameter(typeNo))
            {
                controller.GetAxis(motOrBank).SetParameter(typeNo, val);
                controller.GetAxis(motOrBank).StoreParameterToEeprom(typeNo);
                appliedCount++;
            }
            else if (cmd == "SGP" && motOrBank <= 2)
            {
                // Avoid overwriting ModuleAddress (66) or HostAddress (76) with 0 from incomplete .tmc export
                if (motOrBank == 0 && (typeNo == 66 || typeNo == 76) && val == 0)
                    continue;
                controller.SetGlobalParameter(motOrBank, typeNo, val);
                appliedCount++;
            }
        }

        return appliedCount;
    }

    public static void LoadAxisLabelsFromFile(string filePath, Tmcm6110Controller controller)
    {
        if (!File.Exists(filePath)) return;

        // Matches lines like: "axis 0 (1rd Axis) Z"
        var regex = new Regex(@"axis\s+(\d+).*?\)\s*([A-Za-z0-9_+-]+)", RegexOptions.IgnoreCase);
        foreach (string line in File.ReadLines(filePath))
        {
            var m = regex.Match(line.Trim());
            if (m.Success && int.TryParse(m.Groups[1].Value, out int axisIdx) && axisIdx >= 0 && axisIdx < Tmcm6110Controller.TotalAxes)
            {
                controller.GetAxis(axisIdx).AxisName = m.Groups[2].Value.Trim().ToUpperInvariant();
            }
        }
    }
}
