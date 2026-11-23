using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 6-Axis Dual-Station Gantry scene (Left Station: Z1/Y1/X1, Right Station: Z2/Y2/X2).
/// </summary>
public sealed class DualGantry6AxisScene : IMachine3DScene
{
    private const double DualHalfX = 62.0;
    private const double DualHalfY = 78.0;
    private const double DualHalfZ = 38.0;
    private const double DualStationOffsetX = 86.0;

    private readonly Model3DGroup _machineGroup = new();
    private readonly TranslateTransform3D[] _axisTranslate = new TranslateTransform3D[6];
    private readonly AxisAngleRotation3D[] _axisRotate = new AxisAngleRotation3D[6];
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial _toolTipMat1 = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly DiffuseMaterial _toolTipMat2 = new(new SolidColorBrush(Color.FromRgb(0, 229, 255)));
    private readonly GeometryModel3D _trailModel1 = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly GeometryModel3D _trailModel2 = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(0, 229, 255));
    private readonly List<TrailSample6> _trailSamples1 = new();
    private readonly List<TrailSample6> _trailSamples2 = new();
    private Point3D _lastTrailPointWorld1 = new(double.NaN, double.NaN, double.NaN);
    private Point3D _lastTrailPointWorld2 = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -45.0, -100.0, -100.0, -100.0, -100.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0, 100.0, 100.0, 100.0, 100.0 };

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.DualGantry6Axis, 6, 6, 6);
    public Model3DGroup SceneGroup { get; } = new();

    public DualGantry6AxisScene()
    {
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i] = new TranslateTransform3D();
            _axisRotate[i] = new AxisAngleRotation3D();
            _posLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
            _negLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
        }

        SceneGroup.Children.Add(_machineGroup);
        SceneGroup.Children.Add(_trailModel1);
        SceneGroup.Children.Add(_trailModel2);
        RebuildGeometry();
    }

    public void Update(ReadOnlySpan<Axis3DFrame> axes, ReadOnlySpan<bool> digitalOutputs = default)
    {
        if (axes.Length < 6) return;

        if (Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, 6))
        {
            RebuildGeometry();
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel1, _trailSamples1, SampleToTcpWorld1, ref _lastTrailPointWorld1);
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel2, _trailSamples2, SampleToTcpWorld2, ref _lastTrailPointWorld2);
        }

        for (int i = 0; i < 6; i++)
        {
            Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
        }

        bool alarm1 = axes[0].IsAlarm || axes[1].IsAlarm || axes[2].IsAlarm;
        bool alarm2 = axes[3].IsAlarm || axes[4].IsAlarm || axes[5].IsAlarm;

        if (_toolTipMat1.Brush is SolidColorBrush tip1)
            tip1.Color = alarm1 ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);
        if (_toolTipMat2.Brush is SolidColorBrush tip2)
            tip2.Color = alarm2 ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 229, 255);

        double visZ1 = Mesh3DPrimitives.MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], DualHalfZ);
        double visY1 = Mesh3DPrimitives.MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], DualHalfY);
        double visX1 = Mesh3DPrimitives.MapPhysicalToVisual(axes[2].PositionMm, _minMm[2], _maxMm[2], DualHalfX);
        _axisTranslate[0].OffsetY = visZ1;
        _axisTranslate[1].OffsetZ = visY1;
        _axisTranslate[2].OffsetX = visX1;

        double visZ2 = Mesh3DPrimitives.MapPhysicalToVisual(axes[3].PositionMm, _minMm[3], _maxMm[3], DualHalfZ);
        double visY2 = Mesh3DPrimitives.MapPhysicalToVisual(axes[4].PositionMm, _minMm[4], _maxMm[4], DualHalfY);
        double visX2 = Mesh3DPrimitives.MapPhysicalToVisual(axes[5].PositionMm, _minMm[5], _maxMm[5], DualHalfX);
        _axisTranslate[3].OffsetY = visZ2;
        _axisTranslate[4].OffsetZ = visY2;
        _axisTranslate[5].OffsetX = visX2;

        var tcp1 = new Point3D(-DualStationOffsetX + visX1, visZ1 - 8.0, visY1 + 18.0);
        if (double.IsNaN(_lastTrailPointWorld1.X) || (tcp1 - _lastTrailPointWorld1).Length > 0.6)
        {
            _lastTrailPointWorld1 = tcp1;
            _trailSamples1.Add(new TrailSample6(
                axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm));
            if (_trailSamples1.Count > 600)
                _trailSamples1.RemoveAt(0);
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel1, _trailSamples1, SampleToTcpWorld1, ref _lastTrailPointWorld1);
        }

        var tcp2 = new Point3D(+DualStationOffsetX + visX2, visZ2 - 8.0, visY2 + 18.0);
        if (double.IsNaN(_lastTrailPointWorld2.X) || (tcp2 - _lastTrailPointWorld2).Length > 0.6)
        {
            _lastTrailPointWorld2 = tcp2;
            _trailSamples2.Add(new TrailSample6(
                axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm));
            if (_trailSamples2.Count > 600)
                _trailSamples2.RemoveAt(0);
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel2, _trailSamples2, SampleToTcpWorld2, ref _lastTrailPointWorld2);
        }
    }

    public void ClearTrails()
    {
        _trailSamples1.Clear();
        _trailSamples2.Clear();
        _lastTrailPointWorld1 = new Point3D(double.NaN, double.NaN, double.NaN);
        _lastTrailPointWorld2 = new Point3D(double.NaN, double.NaN, double.NaN);
        Mesh3DPrimitives.ClearMesh(_trailModel1);
        Mesh3DPrimitives.ClearMesh(_trailModel2);
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

    private Point3D SampleToTcpWorld1(TrailSample6 s) => new(
        -DualStationOffsetX + Mesh3DPrimitives.MapPhysicalToVisual(s.A2, _minMm[2], _maxMm[2], DualHalfX),
        Mesh3DPrimitives.MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], DualHalfZ) - 8.0,
        Mesh3DPrimitives.MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], DualHalfY) + 18.0);

    private Point3D SampleToTcpWorld2(TrailSample6 s) => new(
        +DualStationOffsetX + Mesh3DPrimitives.MapPhysicalToVisual(s.A5, _minMm[5], _maxMm[5], DualHalfX),
        Mesh3DPrimitives.MapPhysicalToVisual(s.A3, _minMm[3], _maxMm[3], DualHalfZ) - 8.0,
        Mesh3DPrimitives.MapPhysicalToVisual(s.A4, _minMm[4], _maxMm[4], DualHalfY) + 18.0);

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();
        double worktableTopY = -(DualHalfZ + 10.5);

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(
            new Point3D(0, worktableTopY - 6.5, 0),
            356, 6, 2.0 * DualHalfY + 56.0,
            Color.FromRgb(32, 38, 48)));

        GantrySceneCore.BuildSingleGantry(
            _machineGroup,
            centerX: -DualStationOffsetX,
            visHalfX: DualHalfX, visHalfY: DualHalfY, visHalfZ: DualHalfZ,
            zIdx: 0, yIdx: 1, xIdx: 2,
            _minMm, _maxMm, _axisTranslate, _axisRotate, _posLimitMats, _negLimitMats,
            tipMat: _toolTipMat1,
            includeBasePlate: false,
            includeRotarySpindle: false,
            includeRotaryTable: false,
            includeAuxSlide: false,
            columnColor: Color.FromRgb(46, 125, 50),
            beamColor: Color.FromRgb(198, 40, 40),
            carriageColor: Color.FromRgb(239, 83, 80),
            slideColor: Color.FromRgb(30, 136, 229));

        GantrySceneCore.BuildSingleGantry(
            _machineGroup,
            centerX: +DualStationOffsetX,
            visHalfX: DualHalfX, visHalfY: DualHalfY, visHalfZ: DualHalfZ,
            zIdx: 3, yIdx: 4, xIdx: 5,
            _minMm, _maxMm, _axisTranslate, _axisRotate, _posLimitMats, _negLimitMats,
            tipMat: _toolTipMat2,
            includeBasePlate: false,
            includeRotarySpindle: false,
            includeRotaryTable: false,
            includeAuxSlide: false,
            columnColor: Color.FromRgb(0, 137, 123),
            beamColor: Color.FromRgb(123, 31, 162),
            carriageColor: Color.FromRgb(171, 71, 188),
            slideColor: Color.FromRgb(0, 172, 193));
    }
}
