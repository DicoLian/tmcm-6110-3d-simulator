using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering;

public readonly record struct SceneDescriptor(
    MachineSceneMode Mode,
    int MinAxes,
    int MaxAxes,
    int DefaultAxes
);

/// <summary>
/// Fixed-input contract for all 3D machine scenes.
/// Receives only canonical <see cref="Axis3DFrame"/> telemetry and digital outputs, completely decoupled from controller vendors.
/// </summary>
public interface IMachine3DScene
{
    SceneDescriptor Descriptor { get; }
    Model3DGroup SceneGroup { get; }
    void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default);
    void ClearTrails();
    void ResetTransforms();
}
