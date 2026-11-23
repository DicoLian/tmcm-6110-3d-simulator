using Tmcm6110.Core.Protocol;

namespace Tmcm6110.Core.Abstractions;

/// <summary>
/// Canonical, vendor-neutral axis telemetry snapshot consumed by UI panels and 3D renderers.
/// </summary>
public readonly record struct MotionAxisTelemetry(
    int AxisIndex,
    string AxisName,
    AxisMotionState MotionState,
    double ExactPosition,
    int ActualPosition,
    int TargetPosition,
    int ActualSpeed,
    int MaxSpeed,
    int MaxAcceleration,
    bool IsPositionReached,
    bool IsRightLimitTriggered,
    bool IsLeftLimitTriggered,
    bool IsRefSwitchTriggered,
    bool IsAlarmActive,
    int HomingPhase,
    int VirtualMinStrokeSteps,
    int VirtualMaxStrokeSteps,
    bool IsRightLimitStopEnabled,
    bool IsLeftLimitStopEnabled
);

/// <summary>
/// Generic key-value diagnostic item exposed by vendor-specific axis drivers/protocols.
/// </summary>
public readonly record struct VendorDiagnosticItem(string Name, string Value);

/// <summary>
/// Vendor-neutral motion axis contract separating physical kinematics from vendor registers.
/// </summary>
public interface IMotionAxis
{
    int AxisIndex { get; }
    string AxisName { get; set; }

    int VirtualMinStrokeSteps { get; set; }
    int VirtualMaxStrokeSteps { get; set; }
    bool AutoTriggerStrokeLimits { get; set; }

    bool ManualRightLimitOverride { get; set; }
    bool ManualLeftLimitOverride { get; set; }
    bool ManualRefSwitchOverride { get; set; }
    bool HasActiveAlarm { get; }

    void SetVirtualStrokeLimits(int minSteps, int maxSteps);
    void MoveToAbsolute(int targetPos, double? interpolatedMaxSpeed = null);
    void MoveRelative(int offset);
    void RotateRight(int velocity);
    void RotateLeft(int velocity);
    void StopMotor();
    void StartHoming();
    void StopHoming();
    void SetActualPosition(int position);
    void SetLimitStopEnabled(bool rightEnabled, bool leftEnabled);
    void TriggerFault();
    void ClearFault();

    MotionAxisTelemetry GetTelemetry();
    IReadOnlyList<VendorDiagnosticItem> GetVendorDiagnostics();
}

/// <summary>
/// Vendor-neutral multi-axis motion controller contract.
/// </summary>
public interface IMotionController
{
    string ControllerName { get; }
    int AxisCount { get; }
    double SimulationSpeedMultiplier { get; set; }

    IMotionAxis GetMotionAxis(int index);
    void StepSimulation(double dtSeconds);
    void StopAll();
    void HomeAll();
    void ClearAllFaults();

    bool GetDigitalInput(int port);
    void SetDigitalInput(int port, bool state);
    bool GetDigitalOutput(int port);
    void SetDigitalOutput(int port, bool state);
}
