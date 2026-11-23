using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 4-Axis XYZ Gantry + R Rotary Spindle scene (Role 0=Z, Role 1=Y, Role 2=X, Role 3=R).
/// </summary>
public sealed class GantryRotary4AxisScene : IMachine3DScene
{
    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D[] _axisTranslate = new TranslateTransform3D[6];
    private readonly AxisAngleRotation3D[] _axisRotate = new AxisAngleRotation3D[6];
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial _toolTipMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -45.0, -100.0, -100.0, -100.0, -100.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
    private double _visHalfX = 100.0;
    private double _visHalfY = 100.0;
    private double _visHalfZ = 45.0;

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.GantryRotary4Axis, 4, 4, 4);
    public Model3DGroup SceneGroup { get; } = new();

    public GantryRotary4AxisScene()
    {
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i] = new TranslateTransform3D();
            _axisRotate[i] = new AxisAngleRotation3D();
            _posLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
            _negLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
        }

        SceneGroup.Children.Add(_machineGroup);
        SceneGroup.Children.Add(_trailModel);
        RebuildGeometry();
    }

    public void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        if (axes.Length < 4) return;

        if (Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, 4))
        {
            (_visHalfX, _visHalfY, _visHalfZ) = Mesh3DPrimitives.ComputeGantryVisualHalfSpans(
                _minMm[2], _maxMm[2], _minMm[1], _maxMm[1], _minMm[0], _maxMm[0]);
            RebuildGeometry();
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToTcpWorld, ref _lastTrailPointWorld);
        }

        for (int i = 0; i < 4; i++)
        {
            Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
        }

        bool anyAlarm = axes[0].IsAlarm || axes[1].IsAlarm || axes[2].IsAlarm || axes[3].IsAlarm;
        if (_toolTipMat.Brush is SolidColorBrush tip)
            tip.Color = anyAlarm ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);

        double visZ = Mesh3DPrimitives.MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], _visHalfZ);
        double visY = Mesh3DPrimitives.MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], _visHalfY);
        double visX = Mesh3DPrimitives.MapPhysicalToVisual(axes[2].PositionMm, _minMm[2], _maxMm[2], _visHalfX);
        _axisTranslate[0].OffsetY = visZ;
        _axisTranslate[1].OffsetZ = visY;
        _axisTranslate[2].OffsetX = visX;
        _axisRotate[3].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[3].PositionMm, _minMm[3], _maxMm[3], 180.0);

        var tcp = new Point3D(visX, visZ - 8.0, visY + 18.0);
        if (double.IsNaN(_lastTrailPointWorld.X) || (tcp - _lastTrailPointWorld).Length > 0.6)
        {
            _lastTrailPointWorld = tcp;
            _trailSamples.Add(new TrailSample6(axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm, axes[3].PositionMm, 0, 0));
            if (_trailSamples.Count > 600)
                _trailSamples.RemoveAt(0);
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToTcpWorld, ref _lastTrailPointWorld);
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
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i].OffsetX = 0;
            _axisTranslate[i].OffsetY = 0;
            _axisTranslate[i].OffsetZ = 0;
            _axisRotate[i].Angle = 0;
        }
    }

    private Point3D SampleToTcpWorld(TrailSample6 s) => new(
        Mesh3DPrimitives.MapPhysicalToVisual(s.A2, _minMm[2], _maxMm[2], _visHalfX),
        Mesh3DPrimitives.MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], _visHalfZ) - 8.0,
        Mesh3DPrimitives.MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], _visHalfY) + 18.0);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();
        GantrySceneCore.BuildSingleGantry(
            _machineGroup,
            centerX: 0.0,
            visHalfX: _visHalfX, visHalfY: _visHalfY, visHalfZ: _visHalfZ,
            zIdx: 0, yIdx: 1, xIdx: 2,
            _minMm, _maxMm, _axisTranslate, _axisRotate, _posLimitMats, _negLimitMats,
            tipMat: _toolTipMat,
            includeBasePlate: true,
            includeRotarySpindle: true,
            includeRotaryTable: false,
            includeAuxSlide: false);
    }
}
