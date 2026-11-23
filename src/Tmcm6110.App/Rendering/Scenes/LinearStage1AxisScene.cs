using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 1-Axis Linear Stage scene (Role 0 = X linear carriage with +/- limit LEDs and dynamic 0 mm origin marker).
/// </summary>
public sealed class LinearStage1AxisScene : IMachine3DScene
{
    private const double HalfTravelX = 105.0;

    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D _translateX = new();
    private readonly DiffuseMaterial _posLimitMat = new(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
    private readonly DiffuseMaterial _negLimitMat = new(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
    private readonly DiffuseMaterial _indicatorMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -100.0 };
    private readonly double[] _maxMm = { 100.0 };

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.LinearStage1Axis, 1, 1, 1);
    public Model3DGroup SceneGroup { get; } = new();

    public LinearStage1AxisScene()
    {
        SceneGroup.Children.Add(_machineGroup);
        SceneGroup.Children.Add(_trailModel);
        RebuildGeometry();
    }

    public void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        if (axes.Length < 1) return;

        if (Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, 1))
        {
            RebuildGeometry();
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToWorld, ref _lastTrailPointWorld);
        }

        Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMat, axes[0].IsRightLimit);
        Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMat, axes[0].IsLeftLimit);
        if (_indicatorMat.Brush is SolidColorBrush tip)
            tip.Color = axes[0].IsAlarm ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);

        double visX = Mesh3DPrimitives.MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], HalfTravelX);
        _translateX.OffsetX = visX;

        var tcp = new Point3D(visX, -6.0, 0.0);
        if (double.IsNaN(_lastTrailPointWorld.X) || (tcp - _lastTrailPointWorld).Length > 0.6)
        {
            _lastTrailPointWorld = tcp;
            _trailSamples.Add(new TrailSample6(axes[0].PositionMm, 0, 0, 0, 0, 0));
            if (_trailSamples.Count > 600)
                _trailSamples.RemoveAt(0);
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToWorld, ref _lastTrailPointWorld);
        }
    }

    public void ClearTrails()
    {
        _trailSamples.Clear();
        _lastTrailPointWorld = new Point3D(double.NaN, double.NaN, double.NaN);
        Mesh3DPrimitives.ClearMesh(_trailModel);
    }

    public void ResetTransforms()
    {
        _translateX.OffsetX = 0;
    }

    private Point3D SampleToWorld(TrailSample6 s) => new(
        Mesh3DPrimitives.MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], HalfTravelX),
        -6.0,
        0.0);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -34, 0), 255, 6, 78, Color.FromRgb(32, 38, 48)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -28, 0), 235, 6, 42, Color.FromRgb(54, 64, 80)));

        // Linear guide rails & scale ticks
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -23.5, -13), 226, 3, 4, Color.FromRgb(120, 134, 156)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -23.5, 13), 226, 3, 4, Color.FromRgb(120, 134, 156)));
        for (int i = -5; i <= 5; i++)
        {
            double tickX = i * (HalfTravelX / 5.0);
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(tickX, -24.5, 25), i == 0 ? 1.8 : 0.9, 0.8, i == 0 ? 8 : 5, Color.FromRgb(140, 160, 185)));
        }

        // Physical 0 mm origin marker when 0 lies inside [min, max]
        if (_minMm[0] <= 0.0 && _maxMm[0] >= 0.0)
        {
            double zeroX = Mesh3DPrimitives.MapPhysicalToVisual(0.0, _minMm[0], _maxMm[0], HalfTravelX);
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroX, -23.0, 29), 4.5, 3.0, 4.5, Color.FromRgb(235, 248, 255)));
        }

        // -Limit and +Limit LED blocks
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-(HalfTravelX + 14.0), -20, 0), 6, 8, 20, _negLimitMat));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(HalfTravelX + 14.0, -20, 0), 6, 8, 20, _posLimitMat));

        // Moving carriage
        var carriage = new Model3DGroup { Transform = _translateX };
        carriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -16, 0), 34, 12, 34, Color.FromRgb(229, 57, 53)));
        carriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -8.5, 0), 26, 3, 26, Color.FromRgb(224, 232, 240)));
        carriage.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(0, -5.0, 0), 5, 5, 5, _indicatorMat));
        _machineGroup.Children.Add(carriage);
    }
}
