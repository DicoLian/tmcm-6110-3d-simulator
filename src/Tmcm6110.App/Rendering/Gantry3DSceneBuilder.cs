using System.Windows.Media;
using System.Windows.Media.Media3D;
using Tmcm6110.App.Rendering.Scenes;

namespace Tmcm6110.App.Rendering;

/// <summary>
/// Pure native WPF 3D scene host &amp; registry.
/// Manages global lighting and swaps modular <see cref="IMachine3DScene"/> implementations driven by fixed <see cref="Axis3DFrame"/> inputs.
/// </summary>
public class Gantry3DSceneBuilder
{
    public Model3DGroup RootGroup { get; } = new();
    private readonly Model3DGroup _activeSceneSlot = new();
    private readonly Gantry3AxisScene _gantry3AxisScene;
    private readonly Dictionary<MachineSceneMode, IMachine3DScene> _scenes;
    private IMachine3DScene _activeScene;

    // Compatibility accessors for primary XYZ gantry transforms
    public TranslateTransform3D ZSpindleTransform => _gantry3AxisScene.ZSpindleTransform;
    public TranslateTransform3D YBridgeTransform => _gantry3AxisScene.YBridgeTransform;
    public TranslateTransform3D XCarriageTransform => _gantry3AxisScene.XCarriageTransform;

    public MachineSceneMode SceneMode => _activeScene.Descriptor.Mode;
    public SceneDescriptor CurrentDescriptor => _activeScene.Descriptor;

    public Gantry3DSceneBuilder()
    {
        _gantry3AxisScene = new Gantry3AxisScene();
        _scenes = new Dictionary<MachineSceneMode, IMachine3DScene>
        {
            [MachineSceneMode.Gantry3Axis] = _gantry3AxisScene,
            [MachineSceneMode.DualGantry6Axis] = new DualGantry6AxisScene(),
            [MachineSceneMode.GantryRotaryAux6Axis] = new GantryRotaryAux6AxisScene(),
            [MachineSceneMode.RobotArm6Dof] = new RobotArm6DofScene(),
            [MachineSceneMode.LinearBench6Axis] = new LinearBenchScene(),
            [MachineSceneMode.LinearStage1Axis] = new LinearStage1AxisScene(),
            [MachineSceneMode.RotaryTable1Axis] = new RotaryTable1AxisScene(),
            [MachineSceneMode.XyStage2Axis] = new XyStage2AxisScene(),
            [MachineSceneMode.XzGantry2Axis] = new XzGantry2AxisScene(),
            [MachineSceneMode.XyRotary3Axis] = new XyRotary3AxisScene(),
            [MachineSceneMode.GantryRotary4Axis] = new GantryRotary4AxisScene(),
            [MachineSceneMode.GantryDualRotary5Axis] = new GantryDualRotary5AxisScene()
        };

        BuildLighting();
        _activeScene = _gantry3AxisScene;
        _activeSceneSlot.Children.Add(_activeScene.SceneGroup);
        RootGroup.Children.Add(_activeSceneSlot);
    }

    private void BuildLighting()
    {
        RootGroup.Children.Add(new AmbientLight(Color.FromRgb(95, 105, 120)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(220, 225, 235), new Vector3D(-1.2, -2.0, -1.5)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(110, 130, 160), new Vector3D(1.5, 1.0, 1.2)));
    }

    public IReadOnlyList<MachineSceneMode> GetSupportedModesForAxisCount(int axisCount)
    {
        return _scenes.Values
            .Where(s => axisCount >= s.Descriptor.MinAxes && axisCount <= s.Descriptor.MaxAxes)
            .OrderBy(s => (int)s.Descriptor.Mode)
            .Select(s => s.Descriptor.Mode)
            .ToList();
    }

    public void SetSceneMode(MachineSceneMode mode)
    {
        if (_activeScene.Descriptor.Mode == mode) return;
        if (!_scenes.TryGetValue(mode, out var nextScene)) return;

        _activeScene.ClearTrails();
        _activeScene.ResetTransforms();

        _activeScene = nextScene;
        _activeScene.ClearTrails();
        _activeScene.ResetTransforms();

        _activeSceneSlot.Children.Clear();
        _activeSceneSlot.Children.Add(_activeScene.SceneGroup);
    }

    /// <summary>
    /// Updates the active 3D scene from canonical axis frames and optional digital outputs.
    /// </summary>
    public void UpdateScene(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        _activeScene.Update(axes, digitalOutputs);
    }

    public void ClearTrajectoryTrail() => _activeScene.ClearTrails();

    public static GeometryModel3D CreateBoxModel(Point3D center, double sizeX, double sizeY, double sizeZ, Color color) =>
        Mesh3DPrimitives.CreateBoxModel(center, sizeX, sizeY, sizeZ, color);

    public static GeometryModel3D CreateBoxModelWithMaterial(Point3D center, double sizeX, double sizeY, double sizeZ, Material material) =>
        Mesh3DPrimitives.CreateBoxModelWithMaterial(center, sizeX, sizeY, sizeZ, material);

    public static GeometryModel3D CreateCylinderModel(Point3D center, double radius, double height, Color color, int segments = 16, bool alongZ = false) =>
        Mesh3DPrimitives.CreateCylinderModel(center, radius, height, color, segments, alongZ);
}
