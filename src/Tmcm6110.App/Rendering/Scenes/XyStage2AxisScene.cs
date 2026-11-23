using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 2-Axis XY Planar Positioning Stage scene (Role 0 = X carriage, Role 1 = Y gantry bridge).
/// </summary>
public sealed class XyStage2AxisScene : IMachine3DScene
{
    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D _translateX = new();
    private readonly TranslateTransform3D _translateY = new();
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[2];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[2];
    private readonly DiffuseMaterial _toolTipMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -100.0, -100.0 };
    private readonly double[] _maxMm = { 100.0, 100.0 };
    private double _visHalfX = 100.0;
    private double _visHalfY = 100.0;

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.XyStage2Axis, 2, 2, 2);
    public Model3DGroup SceneGroup { get; } = new();

    public XyStage2AxisScene()
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
            (_visHalfX, _visHalfY, _) = Mesh3DPrimitives.ComputeGantryVisualHalfSpans(
                _minMm[0], _maxMm[0], _minMm[1], _maxMm[1], -45.0, 45.0);
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

        double visX = Mesh3DPrimitives.MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], _visHalfX);
        double visY = Mesh3DPrimitives.MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], _visHalfY);
        _translateX.OffsetX = visX;
        _translateY.OffsetZ = visY;

        var tcp = new Point3D(visX, -14.0, visY + 8.0);
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
        _translateX.OffsetX = 0;
        _translateY.OffsetZ = 0;
    }

    private Point3D SampleToWorld(TrailSample6 s) => new(
        Mesh3DPrimitives.MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], _visHalfX),
        -14.0,
        Mesh3DPrimitives.MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], _visHalfY) + 8.0);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        const double tableTopY = -36.0;
        const double bridgeY = 6.0;
        double railOffsetX = _visHalfX + 15.0;

        // Base plate & gridded worktable
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, tableTopY - 6.5, 0), 2.0 * _visHalfX + 60.0, 6, 2.0 * _visHalfY + 60.0, Color.FromRgb(32, 38, 48)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, tableTopY - 1.5, 0), 2.0 * _visHalfX + 10.0, 3, 2.0 * _visHalfY + 10.0, Color.FromRgb(48, 56, 68)));

        for (int i = 0; i <= 8; i++)
        {
            double gx = -_visHalfX + (2.0 * _visHalfX * i / 8.0);
            double gy = -_visHalfY + (2.0 * _visHalfY * i / 8.0);
            bool edge = i == 0 || i == 8;
            Color gridCol = edge ? Color.FromRgb(90, 108, 130) : Color.FromRgb(68, 78, 92);
            double th = edge ? 0.75 : 0.45;
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(gx, tableTopY + 0.3, 0), th, 0.3, 2.0 * _visHalfY, gridCol));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, tableTopY + 0.3, gy), 2.0 * _visHalfX, 0.3, th, gridCol));
        }

        // Dynamic XY origin marker
        bool hasZeroX = _minMm[0] <= 0.0 && _maxMm[0] >= 0.0;
        bool hasZeroY = _minMm[1] <= 0.0 && _maxMm[1] >= 0.0;
        double zeroX = Mesh3DPrimitives.MapPhysicalToVisual(0.0, _minMm[0], _maxMm[0], _visHalfX);
        double zeroY = Mesh3DPrimitives.MapPhysicalToVisual(0.0, _minMm[1], _maxMm[1], _visHalfY);
        if (hasZeroX && hasZeroY)
        {
            double len = Math.Min(26.0, Math.Min(_visHalfX, _visHalfY) * 0.65);
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroX, tableTopY + 1.5, zeroY), 5, 3, 5, Color.FromRgb(235, 248, 255)));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroX + len * 0.5, tableTopY + 1.5, zeroY), len, 2.2, 2.2, Color.FromRgb(255, 82, 82)));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroX, tableTopY + 1.5, zeroY + len * 0.5), 2.2, 2.2, len, Color.FromRgb(105, 240, 174)));
        }

        // Y-axis stationary rails & limit switches
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(-railOffsetX, tableTopY + 5.5, 0), 8, 10, 2.0 * _visHalfY + 30.0, Color.FromRgb(60, 90, 70)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(railOffsetX, tableTopY + 5.5, 0), 8, 10, 2.0 * _visHalfY + 30.0, Color.FromRgb(60, 90, 70)));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-railOffsetX, tableTopY + 13.5, _visHalfY + 12.0), 10, 6, 6, _posLimitMats[1]));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-railOffsetX, tableTopY + 13.5, -(_visHalfY + 12.0)), 10, 6, 6, _negLimitMats[1]));

        // Moving Y bridge + X crossbeam
        var yBridge = new Model3DGroup { Transform = _translateY };
        double colH = bridgeY - (tableTopY + 10.5);
        double colCenterY = (bridgeY + tableTopY + 10.5) * 0.5;
        yBridge.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(-railOffsetX, colCenterY, 0), 12, colH, 16, Color.FromRgb(46, 125, 50)));
        yBridge.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(railOffsetX, colCenterY, 0), 12, colH, 16, Color.FromRgb(46, 125, 50)));
        yBridge.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, bridgeY, 0), 2.0 * _visHalfX + 36.0, 12, 14, Color.FromRgb(198, 40, 40)));
        yBridge.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(_visHalfX + 4.0, bridgeY + 8.0, 0), 6, 6, 14, _posLimitMats[0]));
        yBridge.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-(_visHalfX + 4.0), bridgeY + 8.0, 0), 6, 6, 14, _negLimitMats[0]));

        // Moving X carriage + fixed-height laser/probe head
        var xCarriage = new Model3DGroup { Transform = _translateX };
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, bridgeY, 8), 24, 18, 12, Color.FromRgb(239, 83, 80)));
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -4.0, 8), 10, 14, 10, Color.FromRgb(180, 190, 205)));
        xCarriage.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(0, -13.0, 8), 4.5, 6, 4.5, _toolTipMat));

        yBridge.Children.Add(xCarriage);
        _machineGroup.Children.Add(yBridge);
    }
}
