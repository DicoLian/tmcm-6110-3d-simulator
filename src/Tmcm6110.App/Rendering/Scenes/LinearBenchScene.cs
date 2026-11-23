using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// Configurable N-rail parallel linear test bench (supports 1..6 active rails; defaults to 6 rails).
/// </summary>
public sealed class LinearBenchScene : IMachine3DScene
{
    private const double BenchHalfTravel = 82.0;

    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D[] _axisTranslate = new TranslateTransform3D[6];
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[6];
    private readonly double[] _minMm = { -45.0, -100.0, -100.0, -100.0, -100.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
    private int _activeRails = 6;

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.LinearBench6Axis, 1, 6, 6);
    public Model3DGroup SceneGroup { get; } = new();

    public int ActiveRails
    {
        get => _activeRails;
        set
        {
            int clamped = Math.Clamp(value, 1, 6);
            if (_activeRails == clamped) return;
            _activeRails = clamped;
            RebuildGeometry();
        }
    }

    public LinearBenchScene()
    {
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i] = new TranslateTransform3D();
            _posLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
            _negLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
        }

        SceneGroup.Children.Add(_machineGroup);
        RebuildGeometry();
    }

    public void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        if (axes.Length == 0) return;

        int desiredRails = Math.Clamp(axes.Length, 1, 6);
        if (_activeRails != desiredRails)
            ActiveRails = desiredRails;

        Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, _activeRails);

        for (int i = 0; i < _activeRails; i++)
        {
            Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
            _axisTranslate[i].OffsetZ = Mesh3DPrimitives.MapPhysicalToVisual(
                axes[i].PositionMm, _minMm[i], _maxMm[i], BenchHalfTravel);
        }
    }

    public void ClearTrails()
    {
        // Linear test bench does not display a tool-tip ribbon trail.
    }

    public void ResetTransforms()
    {
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i].OffsetX = 0;
            _axisTranslate[i].OffsetY = 0;
            _axisTranslate[i].OffsetZ = 0;
        }
    }

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        double spacing = _activeRails <= 3 ? 52.0 : 40.0;
        double startX = -0.5 * (_activeRails - 1) * spacing;
        double benchWidth = Math.Max(110.0, _activeRails * spacing + 25.0);

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -35, 0), benchWidth, 6, 225, Color.FromRgb(32, 38, 48)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -31.5, 0), benchWidth - 20.0, 0.8, 1.5, Color.FromRgb(110, 130, 160)));

        Color[] railColors =
        {
            Color.FromRgb(30, 136, 229),  // Blue
            Color.FromRgb(67, 160, 71),   // Green
            Color.FromRgb(229, 57, 53),   // Red
            Color.FromRgb(0, 172, 193),   // Cyan
            Color.FromRgb(142, 36, 170),  // Purple
            Color.FromRgb(251, 140, 0)    // Amber
        };

        for (int i = 0; i < _activeRails; i++)
        {
            double rx = startX + i * spacing;
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(rx, -29, 0), 10, 6, 196, Color.FromRgb(58, 68, 84)));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(rx, -24, 95), 12, 5, 5, _posLimitMats[i]));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(rx, -24, -95), 12, 5, 5, _negLimitMats[i]));

            var carriage = new Model3DGroup { Transform = _axisTranslate[i] };
            carriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(rx, -21, 0), 22, 12, 20, railColors[i]));
            carriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(rx, -14.5, 0), 16, 2, 12, Color.FromRgb(236, 240, 246)));
            _machineGroup.Children.Add(carriage);
        }
    }
}
