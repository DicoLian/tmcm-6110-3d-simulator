using System.Collections.ObjectModel;
using System.Globalization;
using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Engine;

/// <summary>
/// Result of assembling a TMCL standalone source program via <see cref="TmclAssembler.Assemble"/>.
/// </summary>
public readonly record struct TmclAssemblyResult(
    bool Success,
    IReadOnlyList<TmclCommandPacket> Packets,
    IReadOnlyDictionary<string, int> Labels,
    IReadOnlyList<string> Errors);

/// <summary>
/// Two-pass TMCL assembler supporting comments, constants (<c>#define</c> and <c>NAME = VALUE</c>),
/// symbolic labels, symbolic axis parameter/condition/operator names, and all TMCM-6110 FW V1.47 commands.
/// </summary>
public static class TmclAssembler
{
    private readonly record struct ParsedInstructionLine(int LineNumber, int Address, string Text);

    private static readonly IReadOnlyDictionary<string, byte> MoveTypeMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["ABS"] = 0,
            ["REL"] = 1,
            ["COORD"] = 2
        };

    private static readonly IReadOnlyDictionary<string, byte> RfsTypeMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["START"] = 0,
            ["STOP"] = 1,
            ["STATUS"] = 2
        };

    private static readonly IReadOnlyDictionary<string, byte> CalcOpMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["ADD"] = 0,
            ["SUB"] = 1,
            ["MUL"] = 2,
            ["DIV"] = 3,
            ["MOD"] = 4,
            ["AND"] = 5,
            ["OR"] = 6,
            ["XOR"] = 7,
            ["NOT"] = 8,
            ["LOAD"] = 9,
            ["SWAP"] = 10,
            ["COMP"] = 11
        };

    private static readonly IReadOnlyDictionary<string, byte> ConditionMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["ZE"] = 0,
            ["NZ"] = 1,
            ["EQ"] = 2,
            ["NE"] = 3,
            ["GT"] = 4,
            ["GE"] = 5,
            ["LT"] = 6,
            ["LE"] = 7,
            ["ETO"] = 8,
            ["EAL"] = 9,
            ["EDV"] = 10,
            ["EPO"] = 11
        };

    private static readonly IReadOnlyDictionary<string, byte> WaitConditionMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["TICKS"] = 0,
            ["POS"] = 1,
            ["REFSW"] = 2,
            ["LIMSW"] = 3,
            ["RFS"] = 4
        };

    private static readonly IReadOnlyDictionary<string, byte> ClearErrorMap =
        new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["ALL"] = 0,
            ["ETO"] = 1,
            ["EAL"] = 2,
            ["EDV"] = 3,
            ["EPO"] = 4,
            ["ESD"] = 5
        };

    private static readonly IReadOnlyDictionary<string, byte> AxisParamExactMap;
    private static readonly IReadOnlyDictionary<string, byte> AxisParamNormalizedMap;

    static TmclAssembler()
    {
        var exact = new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase)
        {
            ["TargetPosition"] = TmclAxisParam.TargetPosition,
            ["TargetPos"] = TmclAxisParam.TargetPosition,
            ["ActualPosition"] = TmclAxisParam.ActualPosition,
            ["ActualPos"] = TmclAxisParam.ActualPosition,
            ["TargetSpeed"] = TmclAxisParam.TargetSpeed,
            ["ActualSpeed"] = TmclAxisParam.ActualSpeed,
            ["MaxPositioningSpeed"] = TmclAxisParam.MaxPositioningSpeed,
            ["MaximumPositioningSpeed"] = TmclAxisParam.MaxPositioningSpeed,
            ["MaxPosSpeed"] = TmclAxisParam.MaxPositioningSpeed,
            ["MaxSpeed"] = TmclAxisParam.MaxPositioningSpeed,
            ["MaxAcceleration"] = TmclAxisParam.MaxAcceleration,
            ["MaximumAcceleration"] = TmclAxisParam.MaxAcceleration,
            ["MaxAccel"] = TmclAxisParam.MaxAcceleration,
            ["AbsoluteMaxCurrent"] = TmclAxisParam.AbsoluteMaxCurrent,
            ["MaximumCurrent"] = TmclAxisParam.AbsoluteMaxCurrent,
            ["MaxCurrent"] = TmclAxisParam.AbsoluteMaxCurrent,
            ["StandbyCurrent"] = TmclAxisParam.StandbyCurrent,
            ["TargetPosReached"] = TmclAxisParam.TargetPosReached,
            ["TargetPositionReached"] = TmclAxisParam.TargetPosReached,
            ["PosReached"] = TmclAxisParam.TargetPosReached,
            ["HomeSwitchState"] = TmclAxisParam.HomeSwitchState,
            ["RefSwitchStatus"] = TmclAxisParam.RefSwitchStatus,
            ["HomeSwitch"] = TmclAxisParam.HomeSwitchState,
            ["RefSwitch"] = TmclAxisParam.HomeSwitchState,
            ["RightLimitSwitchStatus"] = TmclAxisParam.RightLimitSwitchStatus,
            ["RightLimitSwitch"] = TmclAxisParam.RightLimitSwitchStatus,
            ["LeftLimitSwitchStatus"] = TmclAxisParam.LeftLimitSwitchStatus,
            ["LeftLimitSwitch"] = TmclAxisParam.LeftLimitSwitchStatus,
            ["RightLimitSwitchDisable"] = TmclAxisParam.RightLimitSwitchDisable,
            ["LeftLimitSwitchDisable"] = TmclAxisParam.LeftLimitSwitchDisable,
            ["MinimumSpeed"] = TmclAxisParam.MinimumSpeed,
            ["MinSpeed"] = TmclAxisParam.MinimumSpeed,
            ["ActualAcceleration"] = TmclAxisParam.ActualAcceleration,
            ["ActualAccel"] = TmclAxisParam.ActualAcceleration,
            ["RampMode"] = TmclAxisParam.RampMode,
            ["MicrostepResolution"] = TmclAxisParam.MicrostepResolution,
            ["MicrostepRes"] = TmclAxisParam.MicrostepResolution,
            ["SoftStopFlag"] = TmclAxisParam.SoftStopFlag,
            ["SoftStop"] = TmclAxisParam.SoftStopFlag,
            ["EndSwitchPowerDownMode"] = TmclAxisParam.EndSwitchPowerDownMode,
            ["EndSwitchPowerDown"] = TmclAxisParam.EndSwitchPowerDownMode,
            ["RampDivisor"] = TmclAxisParam.RampDivisor,
            ["RampDiv"] = TmclAxisParam.RampDivisor,
            ["PulseDivisor"] = TmclAxisParam.PulseDivisor,
            ["PulseDiv"] = TmclAxisParam.PulseDivisor,
            ["StepInterpolation"] = TmclAxisParam.StepInterpolation,
            ["DoubleStep"] = TmclAxisParam.DoubleStep,
            ["ChopperBlankTime"] = TmclAxisParam.ChopperBlankTime,
            ["ConstantTOffMode"] = TmclAxisParam.ConstantTOffMode,
            ["DisableFastDecayComparator"] = TmclAxisParam.DisableFastDecayComparator,
            ["ChopperHysteresisEnd"] = TmclAxisParam.ChopperHysteresisEnd,
            ["ChopperHysteresisStart"] = TmclAxisParam.ChopperHysteresisStart,
            ["ChopperOffTime"] = TmclAxisParam.ChopperOffTime,
            ["SmartEnergyCurrentMin"] = TmclAxisParam.SmartEnergyCurrentMin,
            ["SmartEnergyCurrentDownStep"] = TmclAxisParam.SmartEnergyCurrentDownStep,
            ["SmartEnergyHysteresis"] = TmclAxisParam.SmartEnergyHysteresis,
            ["SmartEnergyCurrentUpStep"] = TmclAxisParam.SmartEnergyCurrentUpStep,
            ["SmartEnergyHysteresisStart"] = TmclAxisParam.SmartEnergyHysteresisStart,
            ["StallGuard2FilterEnable"] = TmclAxisParam.StallGuard2FilterEnable,
            ["SG2Filter"] = TmclAxisParam.StallGuard2FilterEnable,
            ["StallGuard2Threshold"] = TmclAxisParam.StallGuard2Threshold,
            ["SG2Threshold"] = TmclAxisParam.StallGuard2Threshold,
            ["SlopeControlHighSide"] = TmclAxisParam.SlopeControlHighSide,
            ["SlopeControlLowSide"] = TmclAxisParam.SlopeControlLowSide,
            ["ShortToGroundProtection"] = TmclAxisParam.ShortToGroundProtection,
            ["ShortDetectionTimer"] = TmclAxisParam.ShortDetectionTimer,
            ["VSense"] = TmclAxisParam.VSense,
            ["SmartEnergyActualCurrent"] = TmclAxisParam.SmartEnergyActualCurrent,
            ["SmartEnergyStallVelocity"] = TmclAxisParam.SmartEnergyStallVelocity,
            ["StopOnStallVel"] = TmclAxisParam.SmartEnergyStallVelocity,
            ["SmartEnergyThresholdSpeed"] = TmclAxisParam.SmartEnergyThresholdSpeed,
            ["SmartEnergySlowRunCurrent"] = TmclAxisParam.SmartEnergySlowRunCurrent,
            ["RandomChopperOffTime"] = TmclAxisParam.RandomChopperOffTime,
            ["ReferenceSearchMode"] = TmclAxisParam.ReferenceSearchMode,
            ["RfsMode"] = TmclAxisParam.ReferenceSearchMode,
            ["ReferenceSearchSpeed"] = TmclAxisParam.ReferenceSearchSpeed,
            ["RfsSearchSpeed"] = TmclAxisParam.ReferenceSearchSpeed,
            ["ReferenceSwitchSpeed"] = TmclAxisParam.ReferenceSwitchSpeed,
            ["RfsSwitchSpeed"] = TmclAxisParam.ReferenceSwitchSpeed,
            ["EndSwitchDistance"] = TmclAxisParam.EndSwitchDistance,
            ["EndSwitchDist"] = TmclAxisParam.EndSwitchDistance,
            ["LastReferencePosition"] = TmclAxisParam.LastReferencePosition,
            ["LastRefPos"] = TmclAxisParam.LastReferencePosition,
            ["BoostCurrent"] = TmclAxisParam.BoostCurrent,
            ["FreewheelingDelay"] = TmclAxisParam.FreewheelingDelay,
            ["Freewheeling"] = TmclAxisParam.FreewheelingDelay,
            ["ActualLoadValue"] = TmclAxisParam.ActualLoadValue,
            ["ActualLoad"] = TmclAxisParam.ActualLoadValue,
            ["ExtendedErrorFlags"] = TmclAxisParam.ExtendedErrorFlags,
            ["StallErrorFlag"] = TmclAxisParam.ExtendedErrorFlags,
            ["DriverErrorFlags"] = TmclAxisParam.DriverErrorFlags,
            ["DriverErrorFlag"] = TmclAxisParam.DriverErrorFlags,
            ["GroupIndex"] = TmclAxisParam.GroupIndex,
            ["PowerDownDelay"] = TmclAxisParam.PowerDownDelay
        };

        var normalized = new Dictionary<string, byte>(StringComparer.OrdinalIgnoreCase);
        foreach (var kv in exact)
            normalized[NormalizeSymbolKey(kv.Key)] = kv.Value;

        AxisParamExactMap = exact;
        AxisParamNormalizedMap = normalized;
    }

    /// <summary>
    /// Assembles TMCL source text into binary <see cref="TmclCommandPacket"/> instructions using a two-pass algorithm.
    /// </summary>
    public static TmclAssemblyResult Assemble(string source, byte moduleAddress = 1)
    {
        var labels = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        var readOnlyLabels = new ReadOnlyDictionary<string, int>(labels);

        if (source is null)
        {
            return new TmclAssemblyResult(
                Success: false,
                Packets: Array.Empty<TmclCommandPacket>(),
                Labels: readOnlyLabels,
                Errors: new[] { "Line 0: Source text cannot be null." });
        }

        var constants = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        var errors = new List<string>();
        var instructions = new List<ParsedInstructionLine>();

        // Pass 1: strip comments, collect constants, record labels, and queue instruction lines.
        RunPass1(source, constants, labels, instructions, errors);

        // Pass 2: assemble each instruction line into a TmclCommandPacket.
        var packets = new List<TmclCommandPacket>(instructions.Count);
        foreach (var instr in instructions)
        {
            if (TryAssembleInstruction(instr, moduleAddress, constants, labels, errors, out var packet))
            {
                packets.Add(packet);
            }
        }

        bool success = errors.Count == 0;
        return new TmclAssemblyResult(
            Success: success,
            Packets: success ? packets : Array.Empty<TmclCommandPacket>(),
            Labels: readOnlyLabels,
            Errors: errors);
    }

    private static void RunPass1(
        string source,
        Dictionary<string, int> constants,
        Dictionary<string, int> labels,
        List<ParsedInstructionLine> instructions,
        List<string> errors)
    {
        string normalizedSource = source.Replace("\r\n", "\n").Replace('\r', '\n');
        string[] rawLines = normalizedSource.Split('\n');
        int address = 0;

        for (int i = 0; i < rawLines.Length; i++)
        {
            int lineNumber = i + 1;
            string line = StripComment(rawLines[i]);
            if (string.IsNullOrEmpty(line))
                continue;

            // Handle leading label prefixes (e.g. "Loop:" or "Loop: MVP ABS, 0, 1000").
            bool hadLabel = false;
            while (TrySplitLeadingLabel(line, out string labelName, out string remainder))
            {
                hadLabel = true;
                if (!IsValidIdentifier(labelName))
                {
                    errors.Add($"Line {lineNumber}: Invalid label name '{labelName}'.");
                }
                else if (!labels.TryAdd(labelName, address))
                {
                    errors.Add($"Line {lineNumber}: Duplicate label '{labelName}' (already defined at address {labels[labelName]}).");
                }

                line = remainder.Trim();
                if (string.IsNullOrEmpty(line))
                    break;
            }

            if (string.IsNullOrEmpty(line))
                continue;

            // Preprocessor directive (#define NAME VALUE)
            if (line.StartsWith('#'))
            {
                if (hadLabel)
                {
                    errors.Add($"Line {lineNumber}: Preprocessor directive cannot follow a label on the same line.");
                    continue;
                }

                ParseDefineDirective(lineNumber, line, constants, errors);
                continue;
            }

            // Constant assignment (NAME = VALUE)
            int eqIndex = line.IndexOf('=');
            if (eqIndex >= 0)
            {
                if (hadLabel)
                {
                    errors.Add($"Line {lineNumber}: Constant assignment cannot follow a label on the same line.");
                    continue;
                }

                ParseAssignmentDirective(lineNumber, line, eqIndex, constants, errors);
                continue;
            }

            instructions.Add(new ParsedInstructionLine(lineNumber, address, line));
            address++;
        }
    }

    private static string StripComment(string rawLine)
    {
        int slashIdx = rawLine.IndexOf("//", StringComparison.Ordinal);
        int semiIdx = rawLine.IndexOf(';');

        int cutIdx = -1;
        if (slashIdx >= 0 && semiIdx >= 0)
            cutIdx = Math.Min(slashIdx, semiIdx);
        else if (slashIdx >= 0)
            cutIdx = slashIdx;
        else if (semiIdx >= 0)
            cutIdx = semiIdx;

        string clean = cutIdx >= 0 ? rawLine[..cutIdx] : rawLine;
        return clean.Trim();
    }

    private static bool TrySplitLeadingLabel(string line, out string labelName, out string remainder)
    {
        int colonIdx = line.IndexOf(':');
        if (colonIdx < 0)
        {
            labelName = string.Empty;
            remainder = line;
            return false;
        }

        // Only treat ':' as a label delimiter when the prefix has no whitespace or commas
        // (or at most trailing whitespace before ':').
        string prefix = line[..colonIdx];
        for (int i = 0; i < prefix.Length; i++)
        {
            char c = prefix[i];
            if (char.IsWhiteSpace(c))
            {
                // Allow whitespace only if all remaining chars before ':' are whitespace.
                if (!string.IsNullOrWhiteSpace(prefix[i..]))
                {
                    labelName = string.Empty;
                    remainder = line;
                    return false;
                }
                break;
            }
            if (c == ',')
            {
                labelName = string.Empty;
                remainder = line;
                return false;
            }
        }

        labelName = prefix.Trim();
        remainder = line[(colonIdx + 1)..];
        return true;
    }

    private static void ParseDefineDirective(
        int lineNumber,
        string line,
        Dictionary<string, int> constants,
        List<string> errors)
    {
        string[] tokens = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (tokens.Length == 0 || !tokens[0].Equals("#define", StringComparison.OrdinalIgnoreCase))
        {
            errors.Add($"Line {lineNumber}: Unknown preprocessor directive '{(tokens.Length > 0 ? tokens[0] : line)}'.");
            return;
        }

        if (tokens.Length != 3)
        {
            errors.Add($"Line {lineNumber}: Malformed #define directive; expected '#define NAME VALUE'.");
            return;
        }

        DefineConstant(lineNumber, tokens[1], tokens[2], constants, errors);
    }

    private static void ParseAssignmentDirective(
        int lineNumber,
        string line,
        int eqIndex,
        Dictionary<string, int> constants,
        List<string> errors)
    {
        string name = line[..eqIndex].Trim();
        string valStr = line[(eqIndex + 1)..].Trim();

        if (string.IsNullOrEmpty(name) || string.IsNullOrEmpty(valStr))
        {
            errors.Add($"Line {lineNumber}: Malformed constant assignment; expected 'NAME = VALUE'.");
            return;
        }

        DefineConstant(lineNumber, name, valStr, constants, errors);
    }

    private static void DefineConstant(
        int lineNumber,
        string name,
        string valueToken,
        Dictionary<string, int> constants,
        List<string> errors)
    {
        if (!IsValidIdentifier(name))
        {
            errors.Add($"Line {lineNumber}: Invalid constant name '{name}'.");
            return;
        }

        if (!TryResolveInt(valueToken, constants, out int val, out string? error))
        {
            errors.Add($"Line {lineNumber}: Invalid value '{valueToken}' for constant '{name}': {error}");
            return;
        }

        constants[name] = val;
    }

    private static bool TryAssembleInstruction(
        ParsedInstructionLine instr,
        byte moduleAddress,
        IReadOnlyDictionary<string, int> constants,
        IReadOnlyDictionary<string, int> labels,
        List<string> errors,
        out TmclCommandPacket packet)
    {
        packet = default;
        int line = instr.LineNumber;

        if (!TrySplitMnemonicAndOperands(line, instr.Text, errors, out string mnemonic, out string[] ops))
            return false;

        string upper = mnemonic.ToUpperInvariant();
        switch (upper)
        {
            // Motion
            case "ROR":
                return AssembleAxisValueCommand(line, moduleAddress, TmclCommand.ROR, upper, ops, constants, errors, out packet);
            case "ROL":
                return AssembleAxisValueCommand(line, moduleAddress, TmclCommand.ROL, upper, ops, constants, errors, out packet);
            case "MST":
                return AssembleSingleAxisCommand(line, moduleAddress, TmclCommand.MST, upper, ops, constants, errors, out packet);
            case "MVP":
                return AssembleMvp(line, moduleAddress, ops, constants, errors, out packet);
            case "RFS":
                return AssembleRfs(line, moduleAddress, ops, constants, errors, out packet);

            // Axis & Global Parameters
            case "SAP":
                return AssembleAxisParamCommand(line, moduleAddress, TmclCommand.SAP, upper, valueRequired: true, ops, constants, errors, out packet);
            case "GAP":
                return AssembleAxisParamCommand(line, moduleAddress, TmclCommand.GAP, upper, valueRequired: false, ops, constants, errors, out packet);
            case "STAP":
                return AssembleAxisParamCommand(line, moduleAddress, TmclCommand.STAP, upper, valueRequired: false, ops, constants, errors, out packet);
            case "RSAP":
                return AssembleAxisParamCommand(line, moduleAddress, TmclCommand.RSAP, upper, valueRequired: false, ops, constants, errors, out packet);
            case "SGP":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.SGP, upper, "parameter", "bank", valueRequired: true, ops, constants, errors, out packet);
            case "GGP":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.GGP, upper, "parameter", "bank", valueRequired: false, ops, constants, errors, out packet);
            case "STGP":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.STGP, upper, "parameter", "bank", valueRequired: false, ops, constants, errors, out packet);
            case "RSGP":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.RSGP, upper, "parameter", "bank", valueRequired: false, ops, constants, errors, out packet);

            // I/O & Coordinates
            case "SIO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.SIO, upper, "port", "bank", valueRequired: true, ops, constants, errors, out packet);
            case "GIO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.GIO, upper, "port", "bank", valueRequired: false, ops, constants, errors, out packet);
            case "SCO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.SCO, upper, "coordinate", "axis", valueRequired: true, ops, constants, errors, out packet);
            case "GCO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.GCO, upper, "coordinate", "axis", valueRequired: false, ops, constants, errors, out packet);
            case "CCO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.CCO, upper, "coordinate", "axis", valueRequired: false, ops, constants, errors, out packet);
            case "ACO":
                return AssembleTwoByteOptionalValueCommand(line, moduleAddress, TmclCommand.ACO, upper, "coordinate", "axis", valueRequired: false, ops, constants, errors, out packet);

            // Arithmetic / Logic
            case "CALC":
                return AssembleCalc(line, moduleAddress, ops, constants, errors, out packet);
            case "CALCX":
                return AssembleCalcx(line, moduleAddress, ops, constants, errors, out packet);
            case "AAP":
                return AssembleAccuToAxisParam(line, moduleAddress, ops, constants, errors, out packet);
            case "AGP":
                return AssembleAccuToGlobalParam(line, moduleAddress, ops, constants, errors, out packet);
            case "CALCVV":
                return AssembleCalcvv(line, moduleAddress, ops, constants, errors, out packet);
            case "CALCVA":
                return AssembleCalcVarReg(line, moduleAddress, TmclCommand.CALCVA, upper, ops, constants, errors, out packet);
            case "CALCAV":
                return AssembleCalcVarReg(line, moduleAddress, TmclCommand.CALCAV, upper, ops, constants, errors, out packet);
            case "CALCVX":
                return AssembleCalcVarReg(line, moduleAddress, TmclCommand.CALCVX, upper, ops, constants, errors, out packet);
            case "CALCXV":
                return AssembleCalcVarReg(line, moduleAddress, TmclCommand.CALCXV, upper, ops, constants, errors, out packet);
            case "CALCV":
                return AssembleCalcv(line, moduleAddress, ops, constants, errors, out packet);

            // Branches / Flow
            case "COMP":
                return AssembleSingleIntValueCommand(line, moduleAddress, TmclCommand.COMP, upper, "operand", ops, constants, errors, out packet);
            case "JA":
                return AssembleJumpTargetCommand(line, moduleAddress, TmclCommand.JA, upper, ops, constants, labels, errors, out packet);
            case "CSUB":
                return AssembleJumpTargetCommand(line, moduleAddress, TmclCommand.CSUB, upper, ops, constants, labels, errors, out packet);
            case "RST":
                return AssembleJumpTargetCommand(line, moduleAddress, TmclCommand.RST, upper, ops, constants, labels, errors, out packet);
            case "RSUB":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.RSUB, upper, ops, errors, out packet);
            case "STOP":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.STOP, upper, ops, errors, out packet);
            case "RETI":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.RETI, upper, ops, errors, out packet);
            case "JC":
                return AssembleConditionalJump(line, moduleAddress, TmclCommand.JC, upper, ops, constants, labels, errors, out packet);
            case "CALL":
                return AssembleConditionalJump(line, moduleAddress, TmclCommand.CALL, upper, ops, constants, labels, errors, out packet);
            case "WAIT":
                return AssembleWait(line, moduleAddress, ops, constants, errors, out packet);
            case "CLE":
                return AssembleCle(line, moduleAddress, ops, constants, errors, out packet);
            case "DJNZ":
                return AssembleDjnz(line, moduleAddress, ops, constants, labels, errors, out packet);

            // Interrupts
            case "VECT":
                return AssembleVect(line, moduleAddress, ops, constants, labels, errors, out packet);
            case "EI":
                return AssembleSingleByteTypeCommand(line, moduleAddress, TmclCommand.EI, upper, "interrupt", ops, constants, errors, out packet);
            case "DI":
                return AssembleSingleByteTypeCommand(line, moduleAddress, TmclCommand.DI, upper, "interrupt", ops, constants, errors, out packet);

            // FW V1.45+ Accu / X-Register / Indexed
            case "MVPA":
                return AssembleMvpa(line, moduleAddress, ops, constants, errors, out packet);
            case "MVPXA":
                return AssembleMvpxa(line, moduleAddress, ops, constants, errors, out packet);
            case "ROLA":
                return AssembleSingleAxisCommand(line, moduleAddress, TmclCommand.ROLA, upper, ops, constants, errors, out packet);
            case "RORA":
                return AssembleSingleAxisCommand(line, moduleAddress, TmclCommand.RORA, upper, ops, constants, errors, out packet);
            case "ROLXA":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.ROLXA, upper, ops, errors, out packet);
            case "RORXA":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.RORXA, upper, ops, errors, out packet);
            case "MSTX":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.MSTX, upper, ops, errors, out packet);
            case "SAPX":
                return AssembleSapx(line, moduleAddress, ops, constants, errors, out packet);
            case "GAPX":
                return AssembleAxisParamOnlyCommand(line, moduleAddress, TmclCommand.GAPX, upper, ops, constants, errors, out packet);
            case "AAPX":
                return AssembleAxisParamOnlyCommand(line, moduleAddress, TmclCommand.AAPX, upper, ops, constants, errors, out packet);
            case "SIV":
                return AssembleSingleIntValueCommand(line, moduleAddress, TmclCommand.SIV, upper, "value", ops, constants, errors, out packet);
            case "GIV":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.GIV, upper, ops, errors, out packet);
            case "AIV":
                return AssembleZeroOperandCommand(line, moduleAddress, TmclCommand.AIV, upper, ops, errors, out packet);

            default:
                errors.Add($"Line {line}: Unknown TMCL mnemonic '{mnemonic}'.");
                return false;
        }
    }

    private static bool TrySplitMnemonicAndOperands(
        int lineNumber,
        string line,
        List<string> errors,
        out string mnemonic,
        out string[] operands)
    {
        int splitIdx = -1;
        for (int i = 0; i < line.Length; i++)
        {
            if (char.IsWhiteSpace(line[i]))
            {
                splitIdx = i;
                break;
            }
        }

        if (splitIdx < 0)
        {
            mnemonic = line;
            operands = Array.Empty<string>();
            return true;
        }

        mnemonic = line[..splitIdx];
        string rest = line[(splitIdx + 1)..].Trim();
        if (string.IsNullOrEmpty(rest))
        {
            operands = Array.Empty<string>();
            return true;
        }

        string[] parts = rest.Split(',');
        for (int i = 0; i < parts.Length; i++)
        {
            parts[i] = parts[i].Trim();
            if (string.IsNullOrEmpty(parts[i]))
            {
                errors.Add($"Line {lineNumber}: Empty operand in '{mnemonic}' instruction.");
                operands = Array.Empty<string>();
                return false;
            }
        }

        operands = parts;
        return true;
    }

    private static bool CheckOperandCount(int line, string mnemonic, string[] ops, int min, int max, List<string> errors)
    {
        if (ops.Length >= min && ops.Length <= max)
            return true;

        string expected = min == max ? $"{min}" : $"{min}..{max}";
        errors.Add($"Line {line}: '{mnemonic}' expects {expected} operand(s), got {ops.Length}.");
        return false;
    }

    private static bool AssembleZeroOperandCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 0, 0, errors))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, 0, 0, 0);
        return true;
    }

    private static bool AssembleSingleAxisCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 1, 1, errors))
            return false;
        if (!TryResolveByte(line, ops[0], "axis", constants, errors, out byte axis))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, 0, axis, 0);
        return true;
    }

    private static bool AssembleSingleByteTypeCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string role, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 1, 1, errors))
            return false;
        if (!TryResolveByte(line, ops[0], role, constants, errors, out byte typeVal))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, typeVal, 0, 0);
        return true;
    }

    private static bool AssembleSingleIntValueCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string role, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 1, 1, errors))
            return false;
        if (!TryResolveOperandInt(line, ops[0], role, constants, errors, out int val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, 0, 0, val);
        return true;
    }

    private static bool AssembleAxisValueCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 2, 2, errors))
            return false;
        if (!TryResolveByte(line, ops[0], "axis", constants, errors, out byte axis))
            return false;
        if (!TryResolveOperandInt(line, ops[1], "velocity", constants, errors, out int vel))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, 0, axis, vel);
        return true;
    }

    private static bool AssembleMvp(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "MVP", ops, 3, 3, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "MVP mode", MoveTypeMap, 0, 255, constants, errors, out byte type))
            return false;
        if (!TryResolveByte(line, ops[1], "axis/mask", constants, errors, out byte motOrBank))
            return false;
        if (!TryResolveOperandInt(line, ops[2], "value", constants, errors, out int val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVP, type, motOrBank, val);
        return true;
    }

    private static bool AssembleMvpa(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "MVPA", ops, 2, 2, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "MVPA mode", MoveTypeMap, 0, 255, constants, errors, out byte type))
            return false;
        if (!TryResolveByte(line, ops[1], "axis", constants, errors, out byte axis))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVPA, type, axis, 0);
        return true;
    }

    private static bool AssembleMvpxa(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "MVPXA", ops, 1, 1, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "MVPXA mode", MoveTypeMap, 0, 255, constants, errors, out byte type))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.MVPXA, type, 0, 0);
        return true;
    }

    private static bool AssembleRfs(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "RFS", ops, 2, 2, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "RFS command", RfsTypeMap, 0, 255, constants, errors, out byte type))
            return false;
        if (!TryResolveByte(line, ops[1], "axis", constants, errors, out byte axis))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.RFS, type, axis, 0);
        return true;
    }

    private static bool AssembleAxisParamCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, bool valueRequired, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, valueRequired ? 3 : 2, 3, errors))
            return false;
        if (!TryResolveAxisParam(line, ops[0], constants, errors, out byte param))
            return false;
        if (!TryResolveByte(line, ops[1], "axis", constants, errors, out byte axis))
            return false;

        int val = 0;
        if (ops.Length == 3 && !TryResolveOperandInt(line, ops[2], "value", constants, errors, out val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, param, axis, val);
        return true;
    }

    private static bool AssembleTwoByteOptionalValueCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string typeRole, string motRole, bool valueRequired,
        string[] ops, IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, valueRequired ? 3 : 2, 3, errors))
            return false;
        if (!TryResolveByte(line, ops[0], typeRole, constants, errors, out byte typeVal))
            return false;
        if (!TryResolveByte(line, ops[1], motRole, constants, errors, out byte motVal))
            return false;

        int val = 0;
        if (ops.Length == 3 && !TryResolveOperandInt(line, ops[2], "value", constants, errors, out val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, typeVal, motVal, val);
        return true;
    }

    private static bool AssembleAccuToAxisParam(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "AAP", ops, 2, 2, errors))
            return false;
        if (!TryResolveAxisParam(line, ops[0], constants, errors, out byte param))
            return false;
        if (!TryResolveByte(line, ops[1], "axis", constants, errors, out byte axis))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.AAP, param, axis, 0);
        return true;
    }

    private static bool AssembleAccuToGlobalParam(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "AGP", ops, 2, 2, errors))
            return false;
        if (!TryResolveByte(line, ops[0], "parameter", constants, errors, out byte param))
            return false;
        if (!TryResolveByte(line, ops[1], "bank", constants, errors, out byte bank))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.AGP, param, bank, 0);
        return true;
    }

    private static bool AssembleSapx(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "SAPX", ops, 2, 2, errors))
            return false;
        if (!TryResolveAxisParam(line, ops[0], constants, errors, out byte param))
            return false;
        if (!TryResolveOperandInt(line, ops[1], "value", constants, errors, out int val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.SAPX, param, 0, val);
        return true;
    }

    private static bool AssembleAxisParamOnlyCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 1, 1, errors))
            return false;
        if (!TryResolveAxisParam(line, ops[0], constants, errors, out byte param))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, param, 0, 0);
        return true;
    }

    private static bool AssembleCalc(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "CALC", ops, 1, 2, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "CALC operation", CalcOpMap, 0, 9, constants, errors, out byte op))
            return false;

        if (op != 8 && ops.Length != 2)
        {
            errors.Add($"Line {line}: 'CALC' operation requires 2 operands (<op>, <val>) except for NOT.");
            return false;
        }

        int val = 0;
        if (ops.Length == 2 && !TryResolveOperandInt(line, ops[1], "operand", constants, errors, out val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CALC, op, 0, val);
        return true;
    }

    private static bool AssembleCalcx(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "CALCX", ops, 1, 1, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "CALCX operation", CalcOpMap, 0, 10, constants, errors, out byte op))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CALCX, op, 0, 0);
        return true;
    }

    private static bool AssembleCalcvv(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "CALCVV", ops, 3, 3, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "CALCVV operation", CalcOpMap, 0, 11, constants, errors, out byte op))
            return false;
        if (!TryResolveByte(line, ops[1], "var1", constants, errors, out byte var1))
            return false;
        if (!TryResolveByte(line, ops[2], "var2", constants, errors, out byte var2))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CALCVV, op, var1, var2);
        return true;
    }

    private static bool AssembleCalcVarReg(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 2, 2, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], $"{mnemonic} operation", CalcOpMap, 0, 11, constants, errors, out byte op))
            return false;
        if (!TryResolveByte(line, ops[1], "variable", constants, errors, out byte varIdx))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, op, varIdx, 0);
        return true;
    }

    private static bool AssembleCalcv(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "CALCV", ops, 2, 3, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "CALCV operation", CalcOpMap, 0, 11, constants, errors, out byte op))
            return false;

        if (op == 10)
        {
            errors.Add($"Line {line}: 'CALCV' does not support operation SWAP (10).");
            return false;
        }

        if (!TryResolveByte(line, ops[1], "variable", constants, errors, out byte varIdx))
            return false;

        if (op != 8 && ops.Length != 3)
        {
            errors.Add($"Line {line}: 'CALCV' operation requires 3 operands (<op>, <var>, <val>) except for NOT.");
            return false;
        }

        int val = 0;
        if (ops.Length == 3 && !TryResolveOperandInt(line, ops[2], "value", constants, errors, out val))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CALCV, op, varIdx, val);
        return true;
    }

    private static bool AssembleJumpTargetCommand(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, IReadOnlyDictionary<string, int> labels,
        List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 1, 1, errors))
            return false;
        if (!TryResolveJumpTarget(line, ops[0], constants, labels, errors, out int addr))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, 0, 0, addr);
        return true;
    }

    private static bool AssembleConditionalJump(
        int line, byte moduleAddr, TmclCommand cmd, string mnemonic, string[] ops,
        IReadOnlyDictionary<string, int> constants, IReadOnlyDictionary<string, int> labels,
        List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, mnemonic, ops, 2, 2, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], $"{mnemonic} condition", ConditionMap, 0, 11, constants, errors, out byte cond))
            return false;
        if (!TryResolveJumpTarget(line, ops[1], constants, labels, errors, out int addr))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)cmd, cond, 0, addr);
        return true;
    }

    private static bool AssembleWait(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "WAIT", ops, 3, 3, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "WAIT condition", WaitConditionMap, 0, 4, constants, errors, out byte cond))
            return false;
        if (!TryResolveByte(line, ops[1], "motor", constants, errors, out byte motor))
            return false;
        if (!TryResolveOperandInt(line, ops[2], "ticks", constants, errors, out int ticks))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.WAIT, cond, motor, ticks);
        return true;
    }

    private static bool AssembleCle(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "CLE", ops, 1, 1, errors))
            return false;
        if (!TryResolveKeywordOrByte(line, ops[0], "CLE flag", ClearErrorMap, 0, 5, constants, errors, out byte flag))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.CLE, flag, 0, 0);
        return true;
    }

    private static bool AssembleDjnz(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, IReadOnlyDictionary<string, int> labels,
        List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "DJNZ", ops, 2, 2, errors))
            return false;
        if (!TryResolveByte(line, ops[0], "variable", constants, errors, out byte varIdx))
            return false;
        if (!TryResolveJumpTarget(line, ops[1], constants, labels, errors, out int addr))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.DJNZ, varIdx, 0, addr);
        return true;
    }

    private static bool AssembleVect(
        int line, byte moduleAddr, string[] ops,
        IReadOnlyDictionary<string, int> constants, IReadOnlyDictionary<string, int> labels,
        List<string> errors, out TmclCommandPacket packet)
    {
        packet = default;
        if (!CheckOperandCount(line, "VECT", ops, 2, 2, errors))
            return false;
        if (!TryResolveByte(line, ops[0], "interrupt", constants, errors, out byte irq))
            return false;
        if (!TryResolveJumpTarget(line, ops[1], constants, labels, errors, out int addr))
            return false;

        packet = new TmclCommandPacket(moduleAddr, (byte)TmclCommand.VECT, irq, 0, addr);
        return true;
    }

    private static bool TryResolveByte(
        int lineNumber,
        string token,
        string role,
        IReadOnlyDictionary<string, int> constants,
        List<string> errors,
        out byte result)
    {
        result = 0;
        if (!TryResolveOperandInt(lineNumber, token, role, constants, errors, out int value))
            return false;

        if (value < byte.MinValue || value > byte.MaxValue)
        {
            errors.Add($"Line {lineNumber}: {role} '{token}' ({value}) is out of byte range (0..255).");
            return false;
        }

        result = (byte)value;
        return true;
    }

    private static bool TryResolveAxisParam(
        int lineNumber,
        string token,
        IReadOnlyDictionary<string, int> constants,
        List<string> errors,
        out byte param)
    {
        param = 0;

        if (constants.TryGetValue(token, out int constVal))
        {
            if (constVal < byte.MinValue || constVal > byte.MaxValue)
            {
                errors.Add($"Line {lineNumber}: Axis parameter '{token}' ({constVal}) is out of byte range (0..255).");
                return false;
            }
            param = (byte)constVal;
            return true;
        }

        if (AxisParamExactMap.TryGetValue(token, out param))
            return true;

        string normalized = NormalizeSymbolKey(token);
        if (AxisParamNormalizedMap.TryGetValue(normalized, out param))
            return true;

        return TryResolveByte(lineNumber, token, "axis parameter", constants, errors, out param);
    }

    private static bool TryResolveKeywordOrByte(
        int lineNumber,
        string token,
        string role,
        IReadOnlyDictionary<string, byte> keywords,
        byte minAllowed,
        byte maxAllowed,
        IReadOnlyDictionary<string, int> constants,
        List<string> errors,
        out byte result)
    {
        result = 0;
        int resolved;

        if (keywords.TryGetValue(token, out byte kwValue))
        {
            resolved = kwValue;
        }
        else if (TryResolveInt(token, constants, out int numValue, out string? err))
        {
            resolved = numValue;
        }
        else
        {
            errors.Add($"Line {lineNumber}: Invalid {role} '{token}': {err}");
            return false;
        }

        if (resolved < minAllowed || resolved > maxAllowed)
        {
            errors.Add($"Line {lineNumber}: {role} '{token}' ({resolved}) is out of allowed range ({minAllowed}..{maxAllowed}).");
            return false;
        }

        result = (byte)resolved;
        return true;
    }

    private static bool TryResolveJumpTarget(
        int lineNumber,
        string token,
        IReadOnlyDictionary<string, int> constants,
        IReadOnlyDictionary<string, int> labels,
        List<string> errors,
        out int address)
    {
        if (labels.TryGetValue(token, out address))
            return true;

        if (TryResolveInt(token, constants, out address, out string? err))
        {
            if (address < 0)
            {
                errors.Add($"Line {lineNumber}: Target address '{token}' ({address}) cannot be negative.");
                return false;
            }
            return true;
        }

        errors.Add($"Line {lineNumber}: Unknown target label or address '{token}': {err}");
        return false;
    }

    private static bool TryResolveOperandInt(
        int lineNumber,
        string token,
        string role,
        IReadOnlyDictionary<string, int> constants,
        List<string> errors,
        out int value)
    {
        if (TryResolveInt(token, constants, out value, out string? error))
            return true;

        errors.Add($"Line {lineNumber}: Invalid {role} '{token}': {error}");
        return false;
    }

    private static bool TryResolveInt(
        string token,
        IReadOnlyDictionary<string, int> constants,
        out int value,
        out string? error)
    {
        string trimmed = token.Trim();
        if (constants.TryGetValue(trimmed, out value))
        {
            error = null;
            return true;
        }

        return TryParseIntLiteral(trimmed, out value, out error);
    }

    private static bool TryParseIntLiteral(string text, out int value, out string? error)
    {
        value = 0;
        if (string.IsNullOrEmpty(text))
        {
            error = "Expected an integer literal or defined constant.";
            return false;
        }

        ReadOnlySpan<char> span = text.AsSpan();
        bool negative = false;
        if (span[0] == '-')
        {
            negative = true;
            span = span[1..];
        }
        else if (span[0] == '+')
        {
            span = span[1..];
        }

        if (span.IsEmpty)
        {
            error = $"Invalid integer '{text}'.";
            return false;
        }

        ReadOnlySpan<char> hexDigits = ReadOnlySpan<char>.Empty;
        bool isHex = false;
        if (span.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
        {
            isHex = true;
            hexDigits = span[2..];
        }
        else if (span[0] == '$')
        {
            isHex = true;
            hexDigits = span[1..];
        }

        if (isHex)
        {
            if (hexDigits.IsEmpty)
            {
                error = $"Missing digits in hexadecimal literal '{text}'.";
                return false;
            }

            if (!ulong.TryParse(hexDigits, NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out ulong rawHex))
            {
                error = $"Invalid hexadecimal literal '{text}'.";
                return false;
            }

            if (negative)
            {
                if (rawHex > 0x8000_0000UL)
                {
                    error = $"Hexadecimal literal '{text}' overflows 32-bit signed integer.";
                    return false;
                }

                value = unchecked((int)(-(long)rawHex));
                error = null;
                return true;
            }

            if (rawHex > 0xFFFF_FFFFUL)
            {
                error = $"Hexadecimal literal '{text}' overflows 32-bit integer.";
                return false;
            }

            value = unchecked((int)(uint)rawHex);
            error = null;
            return true;
        }

        if (!long.TryParse(span, NumberStyles.None, CultureInfo.InvariantCulture, out long magnitude))
        {
            bool allAsciiDigits = true;
            for (int i = 0; i < span.Length; i++)
            {
                if (span[i] is < '0' or > '9')
                {
                    allAsciiDigits = false;
                    break;
                }
            }

            error = allAsciiDigits
                ? $"Integer literal '{text}' overflows 32-bit signed integer."
                : $"Unrecognized identifier or malformed integer '{text}'.";
            return false;
        }

        long signed = negative ? -magnitude : magnitude;
        if (signed < int.MinValue || signed > int.MaxValue)
        {
            error = $"Integer literal '{text}' is out of 32-bit signed range ({int.MinValue}..{int.MaxValue}).";
            return false;
        }

        value = (int)signed;
        error = null;
        return true;
    }

    private static bool IsValidIdentifier(string name)
    {
        if (string.IsNullOrEmpty(name))
            return false;

        char first = name[0];
        if (!((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z') || first == '_'))
            return false;

        for (int i = 1; i < name.Length; i++)
        {
            char c = name[i];
            if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
                return false;
        }

        return true;
    }

    private static string NormalizeSymbolKey(string key)
    {
        string trimmed = key.Trim();
        if (trimmed.StartsWith("AP_", StringComparison.OrdinalIgnoreCase))
            trimmed = trimmed[3..];
        return trimmed.Replace("_", string.Empty, StringComparison.Ordinal);
    }
}
