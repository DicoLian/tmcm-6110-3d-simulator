using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 2-Axis X-Z Vertical Gantry scene (Role 0 = Z vertical spindle, Role 1 = X crossbeam carriage).
/// </summary>
public sealed class XzGantry2AxisScene : IMachine3DScene
{
    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D _translateZ = new();
    private readonly TranslateTransform3D _translateX = new();
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[2];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[2];
    private readonly DiffuseMaterial _toolTipMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -45.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0 };
    private double _visHalfZ = 45.0;
    private double _visHalfX = 100.0;

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.XzGantry2Axis, 2, 2, 2);
    public Model3DGroup SceneGroup { get; } = new();

    public XzGantry2AxisScene()
    {
        for (int i = 0; i < 2; i++)
        {
            _posLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
            _negLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
        }

        SceneGroup.Children.Add(_machineGroup);
        SceneGroup.Children.Add(_trailModel);
        RebuildGeometry();
    }

    public void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        if (axes.Length < 2) return;

        if (Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, 2))
        {
            (_visHalfX, _, _visHalfZ) = Mesh3DPrimitives.ComputeGantryVisualHalfSpans(
                _minMm[1], _maxMm[1], -100.0, 100.0, _minMm[0], _maxMm[0]);
            RebuildGeometry();
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToWorld, ref _lastTrailPointWorld);
        }

        for (int i = 0; i < 2; i++)
        {
            Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
        }

        if (_toolTipMat.Brush is SolidColorBrush tip)
            tip.Color = (axes[0].IsAlarm || axes[1].IsAlarm) ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);

        double visZ = Mesh3DPrimitives.MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], _visHalfZ);
        double visX = Mesh3DPrimitives.MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], _visHalfX);
        _translateZ.OffsetY = visZ;
        _translateX.OffsetX = visX;

        var tcp = new Point3D(visX, visZ - 8.0, 18.0);
        if (double.IsNaN(_lastTrailPointWorld.X) || (tcp - _lastTrailPointWorld).Length > 0.6)
        {
            _lastTrailPointWorld = tcp;
            _trailSamples.Add(new TrailSample6(axes[0].PositionMm, axes[1].PositionMm, 0, 0, 0, 0));
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
        _translateZ.OffsetY = 0;
        _translateX.OffsetX = 0;
    }

    private Point3D SampleToWorld(TrailSample6 s) => new(
        Mesh3DPrimitives.MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], _visHalfX),
        Mesh3DPrimitives.MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], _visHalfZ) - 8.0,
        18.0);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        double tableTopY = -(_visHalfZ + 10.5);
        double crossbeamY = _visHalfZ + 7.0;
        double pillarOffsetX = _visHalfX + 15.0;

        // Narrow workbed + fixed vertical columns + fixed X crossbeam
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, tableTopY - 6.5, 0), 2.0 * _visHalfX + 60.0, 6, 86, Color.FromRgb(32, 38, 48)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, tableTopY - 1.5, 0), 2.0 * _visHalfX + 12.0, 3, 52, Color.FromRgb(48, 56, 68)));

        if (_minMm[1] <= 0.0 && _maxMm[1] >= 0.0)
        {
            double zeroX = Mesh3DPrimitives.MapPhysicalToVisual(0.0, _minMm[1], _maxMm[1], _visHalfX);
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroX, tableTopY + 1.5, 16), 5, 3, 5, Color.FromRgb(235, 248, 255)));
        }

        double colH = (crossbeamY + 5.0) - (tableTopY + 2.0);
        double colCenterY = (crossbeamY + 5.0 + tableTopY + 2.0) * 0.5;
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(-pillarOffsetX, colCenterY, 0), 12, colH, 18, Color.FromRgb(58, 68, 84)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(pillarOffsetX, colCenterY, 0), 12, colH, 18, Color.FromRgb(58, 68, 84)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, crossbeamY, 0), 2.0 * _visHalfX + 36.0, 14, 14, Color.FromRgb(198, 40, 40)));

        // X limit switches on crossbeam
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(_visHalfX + 4.0, crossbeamY + 9.0, 0), 6, 6, 16, _posLimitMats[1]));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-(_visHalfX + 4.0), crossbeamY + 9.0, 0), 6, 6, 16, _negLimitMats[1]));

        // Moving X carriage + Z rail + moving Z spindle
        var xCarriage = new Model3DGroup { Transform = _translateX };
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, crossbeamY, 8), 24, 22, 12, Color.FromRgb(239, 83, 80)));
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(14.0, crossbeamY, 12), 2.2, 2.0 * _visHalfZ + 8.0, 4, Color.FromRgb(70, 82, 100)));
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(14.0, crossbeamY + _visHalfZ, 12), 5, 5, 8, _posLimitMats[0]));
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(14.0, crossbeamY - _visHalfZ, 12), 5, 5, 8, _negLimitMats[0]));

        var zSpindle = new Model3DGroup { Transform = _translateZ };
        zSpindle.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, _visHalfZ + 1.0, 15), 14, 2.0 * _visHalfZ + 2.0, 12, Color.FromRgb(30, 136, 229)));
        zSpindle.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(9.0, crossbeamY, 12), 4.5, 4.0, 4.0, Color.FromRgb(180, 190, 205)));
        zSpindle.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, 8, 18), 10, 22, 10, Color.FromRgb(180, 190, 205)));
        zSpindle.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(0, -4, 18), 4.5, 8, 4.5, _toolTipMat));

        xCarriage.Children.Add(zSpindle);
        _machineGroup.Children.Add(xCarriage);
    }
}
