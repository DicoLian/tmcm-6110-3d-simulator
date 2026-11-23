using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 1-Axis Rotary Indexing Table scene (Role 0 = R angular table around Y-up axis with +/- limit LEDs).
/// </summary>
public sealed class RotaryTable1AxisScene : IMachine3DScene
{
    private const double MarkerRadius = 46.0;
    private const double TableTopY = -12.0;

    private readonly Model3DGroup _machineGroup = new();
    private readonly AxisAngleRotation3D _rotateR = new(new Vector3D(0, 1, 0), 0);
    private readonly DiffuseMaterial _posLimitMat = new(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
    private readonly DiffuseMaterial _negLimitMat = new(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
    private readonly DiffuseMaterial _indicatorMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -100.0 };
    private readonly double[] _maxMm = { 100.0 };

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.RotaryTable1Axis, 1, 1, 1);
    public Model3DGroup SceneGroup { get; } = new();

    public RotaryTable1AxisScene()
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
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToWorld, ref _lastTrailPointWorld);
        }

        Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMat, axes[0].IsRightLimit);
        Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMat, axes[0].IsLeftLimit);
        if (_indicatorMat.Brush is SolidColorBrush tip)
            tip.Color = axes[0].IsAlarm ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);

        double angleDeg = Mesh3DPrimitives.MapPhysicalToAngle(axes[0].PositionMm, _minMm[0], _maxMm[0], 180.0);
        _rotateR.Angle = angleDeg;

        Point3D tcp = AngleToWorld(angleDeg);
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
        _rotateR.Angle = 0;
    }

    private Point3D SampleToWorld(TrailSample6 s) =>
        AngleToWorld(Mesh3DPrimitives.MapPhysicalToAngle(s.A0, _minMm[0], _maxMm[0], 180.0));

    private static Point3D AngleToWorld(double angleDeg) => Mesh3DPrimitives.RotatePointAroundAxis(
        new Point3D(MarkerRadius, TableTopY + 5.0, 0),
        new Point3D(0, TableTopY, 0),
        new Vector3D(0, 1, 0),
        angleDeg);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -35, 0), 165, 6, 165, Color.FromRgb(32, 38, 48)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, -25, 0), 66, 14, Color.FromRgb(54, 62, 76), segments: 28));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, -17, 0), 60, 4, Color.FromRgb(74, 86, 104), segments: 28));

        // 0 deg fixed reference post & +/- limit LEDs
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(64, -15, 0), 8, 6, 3, Color.FromRgb(235, 248, 255)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(14, -14, 64), 7, 6, 7, _posLimitMat));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-14, -14, 64), 7, 6, 7, _negLimitMat));

        // Rotating platter
        _rotateR.Axis = new Vector3D(0, 1, 0);
        var platter = new Model3DGroup
        {
            Transform = new RotateTransform3D(_rotateR, new Point3D(0, TableTopY, 0))
        };
        platter.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, TableTopY, 0), 54, 6, Color.FromRgb(84, 110, 122), segments: 32));
        platter.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(25, TableTopY + 4.0, 0), 48, 2.5, 4.5, Color.FromRgb(255, 213, 79)));
        platter.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(MarkerRadius, TableTopY + 5.0, 0), 6, 6, 6, _indicatorMat));
        platter.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, TableTopY + 4.0, 34), 6, 3, 6, Color.FromRgb(207, 216, 220)));
        platter.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, TableTopY + 4.0, -34), 6, 3, 6, Color.FromRgb(207, 216, 220)));
        _machineGroup.Children.Add(platter);
    }
}
