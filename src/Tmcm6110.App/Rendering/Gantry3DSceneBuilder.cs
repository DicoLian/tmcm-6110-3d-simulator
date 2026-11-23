using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering;

/// <summary>
/// Pure native WPF 3D scene builder (zero external NuGet dependencies required).
/// Supports 5 selectable 3D machine configurations:
///   0. Gantry3Axis          (Default: Axis 0=Z, 1=Y, 2=X)
///   1. DualGantry6Axis      (Station 1: Z1/Y1/X1 + Station 2: Z2/Y2/X2)
///   2. GantryRotaryAux6Axis (XYZ Gantry + R Spindle + A Rotary Table + B Feeder Slide)
///   3. RobotArm6Dof         (Articulated 6-DOF J1..J6 Industrial Robot Arm)
///   4. LinearBench6Axis     (6 Parallel Linear Test Rails Ch 0..5)
/// </summary>
public class Gantry3DSceneBuilder
{
    private const double DualHalfX = 62.0;
    private const double DualHalfY = 78.0;
    private const double DualHalfZ = 38.0;
    private const double DualStationOffsetX = 86.0;
    private const double BenchHalfTravel = 82.0;

    private readonly record struct TrailSample(double A0, double A1, double A2, double A3, double A4, double A5);

    public Model3DGroup RootGroup { get; } = new();
    private readonly Model3DGroup _machineGroup = new();

    // Shared per-axis dynamic transforms updated at 60 FPS
    private readonly TranslateTransform3D[] _axisTranslate = new TranslateTransform3D[6];
    private readonly AxisAngleRotation3D[] _axisRotate = new AxisAngleRotation3D[6];

    // Compatibility accessors for primary XYZ gantry transforms
    public TranslateTransform3D ZSpindleTransform => _axisTranslate[0];
    public TranslateTransform3D YBridgeTransform => _axisTranslate[1];
    public TranslateTransform3D XCarriageTransform => _axisTranslate[2];

    // Limit switch materials for all 6 axes (+Limit / -Limit)
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial _toolTipMat1 = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly DiffuseMaterial _toolTipMat2 = new(new SolidColorBrush(Color.FromRgb(0, 229, 255)));

    private readonly GeometryModel3D _trailModel1;
    private readonly GeometryModel3D _trailModel2;
    private readonly List<TrailSample> _trailSamples1 = new();
    private readonly List<TrailSample> _trailSamples2 = new();
    private Point3D _lastTrailPointWorld1 = new(double.NaN, double.NaN, double.NaN);
    private Point3D _lastTrailPointWorld2 = new(double.NaN, double.NaN, double.NaN);

    // Physical stroke limits (mm) and proportional 3D visual half-spans
    private readonly double[] _minMm = { -45.0, -100.0, -100.0, -100.0, -100.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
    private double _visHalfX = 100.0;
    private double _visHalfY = 100.0;
    private double _visHalfZ = 45.0;

    public MachineSceneMode SceneMode { get; private set; } = MachineSceneMode.Gantry3Axis;

    public Gantry3DSceneBuilder()
    {
        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i] = new TranslateTransform3D();
            _axisRotate[i] = new AxisAngleRotation3D();
            _posLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
            _negLimitMats[i] = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(45, 110, 65)));
        }

        BuildLighting();
        RootGroup.Children.Add(_machineGroup);
        RebuildMachineGeometry();

        _trailModel1 = new GeometryModel3D
        {
            Geometry = new MeshGeometry3D(),
            Material = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(255, 215, 0))),
            BackMaterial = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(255, 215, 0)))
        };
        _trailModel2 = new GeometryModel3D
        {
            Geometry = new MeshGeometry3D(),
            Material = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(0, 229, 255))),
            BackMaterial = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(0, 229, 255)))
        };
        RootGroup.Children.Add(_trailModel1);
        RootGroup.Children.Add(_trailModel2);
    }

    private void BuildLighting()
    {
        RootGroup.Children.Add(new AmbientLight(Color.FromRgb(95, 105, 120)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(220, 225, 235), new Vector3D(-1.2, -2.0, -1.5)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(110, 130, 160), new Vector3D(1.5, 1.0, 1.2)));
    }

    public void SetSceneMode(MachineSceneMode mode)
    {
        if (SceneMode == mode) return;
        SceneMode = mode;

        for (int i = 0; i < 6; i++)
        {
            _axisTranslate[i].OffsetX = 0;
            _axisTranslate[i].OffsetY = 0;
            _axisTranslate[i].OffsetZ = 0;
            _axisRotate[i].Angle = 0;
        }

        ClearTrajectoryTrail();
        RebuildMachineGeometry();
    }

    /// <summary>
    /// Updates the 3D scene from 6 axis frames (positions, stroke limits, limit switch LEDs, alarms, and TCP trails).
    /// </summary>
    public void UpdateScene(ReadOnlySpan<Axis3DFrame> axes)
    {
        if (axes.Length < 6) return;

        bool boundsChanged = false;
        const double eps = 1e-4;
        for (int i = 0; i < 6; i++)
        {
            double minMm = axes[i].MinMm;
            double maxMm = axes[i].MaxMm <= minMm ? minMm + 1.0 : axes[i].MaxMm;
            if (Math.Abs(_minMm[i] - minMm) >= eps || Math.Abs(_maxMm[i] - maxMm) >= eps)
            {
                _minMm[i] = minMm;
                _maxMm[i] = maxMm;
                boundsChanged = true;
            }
        }

        if (boundsChanged)
        {
            RecomputePrimaryVisualHalfSpans();
            RebuildMachineGeometry();
            RebuildTrailMeshes();
        }

        for (int i = 0; i < 6; i++)
        {
            UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
        }

        bool alarmPrimary = SceneMode is MachineSceneMode.Gantry3Axis or MachineSceneMode.DualGantry6Axis
            ? (axes[0].IsAlarm || axes[1].IsAlarm || axes[2].IsAlarm)
            : (axes[0].IsAlarm || axes[1].IsAlarm || axes[2].IsAlarm || axes[3].IsAlarm || axes[4].IsAlarm || axes[5].IsAlarm);
        bool alarmSecondary = axes[3].IsAlarm || axes[4].IsAlarm || axes[5].IsAlarm;

        if (_toolTipMat1.Brush is SolidColorBrush tip1)
            tip1.Color = alarmPrimary ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);
        if (_toolTipMat2.Brush is SolidColorBrush tip2)
            tip2.Color = alarmSecondary ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 229, 255);

        switch (SceneMode)
        {
            case MachineSceneMode.Gantry3Axis:
            {
                double visZ = MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], _visHalfZ);
                double visY = MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], _visHalfY);
                double visX = MapPhysicalToVisual(axes[2].PositionMm, _minMm[2], _maxMm[2], _visHalfX);
                _axisTranslate[0].OffsetY = visZ;
                _axisTranslate[1].OffsetZ = visY;
                _axisTranslate[2].OffsetX = visX;

                var tcp1 = new Point3D(visX, visZ - 8.0, visY + 18.0);
                AppendTrailSample1(axes, tcp1);
                break;
            }

            case MachineSceneMode.DualGantry6Axis:
            {
                double visZ1 = MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], DualHalfZ);
                double visY1 = MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], DualHalfY);
                double visX1 = MapPhysicalToVisual(axes[2].PositionMm, _minMm[2], _maxMm[2], DualHalfX);
                _axisTranslate[0].OffsetY = visZ1;
                _axisTranslate[1].OffsetZ = visY1;
                _axisTranslate[2].OffsetX = visX1;

                double visZ2 = MapPhysicalToVisual(axes[3].PositionMm, _minMm[3], _maxMm[3], DualHalfZ);
                double visY2 = MapPhysicalToVisual(axes[4].PositionMm, _minMm[4], _maxMm[4], DualHalfY);
                double visX2 = MapPhysicalToVisual(axes[5].PositionMm, _minMm[5], _maxMm[5], DualHalfX);
                _axisTranslate[3].OffsetY = visZ2;
                _axisTranslate[4].OffsetZ = visY2;
                _axisTranslate[5].OffsetX = visX2;

                var tcp1 = new Point3D(-DualStationOffsetX + visX1, visZ1 - 8.0, visY1 + 18.0);
                var tcp2 = new Point3D(+DualStationOffsetX + visX2, visZ2 - 8.0, visY2 + 18.0);
                AppendTrailSample1(axes, tcp1);
                AppendTrailSample2(axes, tcp2);
                break;
            }

            case MachineSceneMode.GantryRotaryAux6Axis:
            {
                double visZ = MapPhysicalToVisual(axes[0].PositionMm, _minMm[0], _maxMm[0], _visHalfZ);
                double visY = MapPhysicalToVisual(axes[1].PositionMm, _minMm[1], _maxMm[1], _visHalfY);
                double visX = MapPhysicalToVisual(axes[2].PositionMm, _minMm[2], _maxMm[2], _visHalfX);
                _axisTranslate[0].OffsetY = visZ;
                _axisTranslate[1].OffsetZ = visY;
                _axisTranslate[2].OffsetX = visX;

                _axisRotate[3].Angle = MapPhysicalToAngle(axes[3].PositionMm, _minMm[3], _maxMm[3], 180.0);
                _axisRotate[4].Angle = MapPhysicalToAngle(axes[4].PositionMm, _minMm[4], _maxMm[4], 180.0);
                _axisTranslate[5].OffsetZ = MapPhysicalToVisual(axes[5].PositionMm, _minMm[5], _maxMm[5], _visHalfY);

                var tcp1 = new Point3D(visX, visZ - 8.0, visY + 18.0);
                AppendTrailSample1(axes, tcp1);
                break;
            }

            case MachineSceneMode.RobotArm6Dof:
            {
                double j1 = MapPhysicalToAngle(axes[0].PositionMm, _minMm[0], _maxMm[0], 170.0);
                double j2 = MapPhysicalToAngle(axes[1].PositionMm, _minMm[1], _maxMm[1], 110.0);
                double j3 = MapPhysicalToAngle(axes[2].PositionMm, _minMm[2], _maxMm[2], 135.0) - 90.0;
                double j4 = MapPhysicalToAngle(axes[3].PositionMm, _minMm[3], _maxMm[3], 180.0);
                double j5 = MapPhysicalToAngle(axes[4].PositionMm, _minMm[4], _maxMm[4], 120.0);
                double j6 = MapPhysicalToAngle(axes[5].PositionMm, _minMm[5], _maxMm[5], 180.0);

                _axisRotate[0].Angle = j1;
                _axisRotate[1].Angle = j2;
                _axisRotate[2].Angle = j3;
                _axisRotate[3].Angle = j4;
                _axisRotate[4].Angle = j5;
                _axisRotate[5].Angle = j6;

                var tcp1 = ComputeRobotArmTcpWorld(
                    axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                    axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm);
                AppendTrailSample1(axes, tcp1);
                break;
            }

            case MachineSceneMode.LinearBench6Axis:
            {
                for (int i = 0; i < 6; i++)
                {
                    _axisTranslate[i].OffsetZ = MapPhysicalToVisual(axes[i].PositionMm, _minMm[i], _maxMm[i], BenchHalfTravel);
                }
                break;
            }
        }
    }

    private void RecomputePrimaryVisualHalfSpans()
    {
        double spanX = Math.Max(1.0, _maxMm[2] - _minMm[2]);
        double spanY = Math.Max(1.0, _maxMm[1] - _minMm[1]);
        double spanZ = Math.Max(1.0, _maxMm[0] - _minMm[0]);

        double scaleXY = Math.Min(1.0, 220.0 / Math.Max(spanX, spanY));
        double scaleZ = Math.Min(1.0, 100.0 / spanZ);

        _visHalfX = Math.Clamp((spanX * 0.5) * scaleXY, 28.0, 120.0);
        _visHalfY = Math.Clamp((spanY * 0.5) * scaleXY, 28.0, 120.0);
        _visHalfZ = Math.Clamp((spanZ * 0.5) * scaleZ, 20.0, 55.0);
    }

    private void RebuildMachineGeometry()
    {
        _machineGroup.Children.Clear();

        switch (SceneMode)
        {
            case MachineSceneMode.Gantry3Axis:
                BuildSingleGantryCore(
                    _machineGroup,
                    centerX: 0.0,
                    visHalfX: _visHalfX, visHalfY: _visHalfY, visHalfZ: _visHalfZ,
                    zIdx: 0, yIdx: 1, xIdx: 2,
                    tipMat: _toolTipMat1,
                    includeBasePlate: true,
                    includeRotarySpindle: false,
                    includeRotaryTable: false,
                    includeAuxSlide: false);
                break;

            case MachineSceneMode.DualGantry6Axis:
                BuildDualGantry6Axis();
                break;

            case MachineSceneMode.GantryRotaryAux6Axis:
                BuildSingleGantryCore(
                    _machineGroup,
                    centerX: 0.0,
                    visHalfX: _visHalfX, visHalfY: _visHalfY, visHalfZ: _visHalfZ,
                    zIdx: 0, yIdx: 1, xIdx: 2,
                    tipMat: _toolTipMat1,
                    includeBasePlate: true,
                    includeRotarySpindle: true,
                    includeRotaryTable: true,
                    includeAuxSlide: true);
                break;

            case MachineSceneMode.RobotArm6Dof:
                BuildRobotArm6Dof();
                break;

            case MachineSceneMode.LinearBench6Axis:
                BuildLinearBench6Axis();
                break;
        }
    }

    private void BuildDualGantry6Axis()
    {
        double worktableTopY = -(DualHalfZ + 10.5);

        // Shared wide metallic baseplate
        _machineGroup.Children.Add(CreateBoxModel(
            new Point3D(0, worktableTopY - 6.5, 0),
            356, 6, 2.0 * DualHalfY + 56.0,
            Color.FromRgb(32, 38, 48)));

        // Station 1 (Left: Axis 0=Z1, Axis 1=Y1, Axis 2=X1)
        BuildSingleGantryCore(
            _machineGroup,
            centerX: -DualStationOffsetX,
            visHalfX: DualHalfX, visHalfY: DualHalfY, visHalfZ: DualHalfZ,
            zIdx: 0, yIdx: 1, xIdx: 2,
            tipMat: _toolTipMat1,
            includeBasePlate: false,
            includeRotarySpindle: false,
            includeRotaryTable: false,
            includeAuxSlide: false,
            columnColor: Color.FromRgb(46, 125, 50),
            beamColor: Color.FromRgb(198, 40, 40),
            carriageColor: Color.FromRgb(239, 83, 80),
            slideColor: Color.FromRgb(30, 136, 229));

        // Station 2 (Right: Axis 3=Z2, Axis 4=Y2, Axis 5=X2)
        BuildSingleGantryCore(
            _machineGroup,
            centerX: +DualStationOffsetX,
            visHalfX: DualHalfX, visHalfY: DualHalfY, visHalfZ: DualHalfZ,
            zIdx: 3, yIdx: 4, xIdx: 5,
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

    private void BuildSingleGantryCore(
        Model3DGroup targetGroup,
        double centerX,
        double visHalfX,
        double visHalfY,
        double visHalfZ,
        int zIdx,
        int yIdx,
        int xIdx,
        Material tipMat,
        bool includeBasePlate,
        bool includeRotarySpindle,
        bool includeRotaryTable,
        bool includeAuxSlide,
        Color? columnColor = null,
        Color? beamColor = null,
        Color? carriageColor = null,
        Color? slideColor = null)
    {
        Color colClr = columnColor ?? Color.FromRgb(46, 125, 50);
        Color bmClr = beamColor ?? Color.FromRgb(198, 40, 40);
        Color carClr = carriageColor ?? Color.FromRgb(239, 83, 80);
        Color sldClr = slideColor ?? Color.FromRgb(30, 136, 229);

        double worktableTopY = -(visHalfZ + (includeRotaryTable ? 15.0 : 10.5));
        double crossbeamY = visHalfZ + 7.0;
        double railOffsetX = visHalfX + (includeBasePlate ? 15.0 : 12.0);
        double minX = _minMm[xIdx];
        double maxX = _maxMm[xIdx];
        double minY = _minMm[yIdx];
        double maxY = _maxMm[yIdx];

        var staticGroup = new Model3DGroup();

        if (includeBasePlate)
        {
            double extraRight = includeAuxSlide ? 34.0 : 0.0;
            staticGroup.Children.Add(CreateBoxModel(
                new Point3D(centerX + extraRight * 0.5, worktableTopY - 6.5, 0),
                2.0 * visHalfX + 60.0 + extraRight, 6, 2.0 * visHalfY + 60.0,
                Color.FromRgb(32, 38, 48)));
        }

        // Worktable surface
        staticGroup.Children.Add(CreateBoxModel(
            new Point3D(centerX, worktableTopY - 1.5, 0),
            2.0 * visHalfX + 10.0, 3, 2.0 * visHalfY + 10.0,
            Color.FromRgb(48, 56, 68)));

        // Active stroke grid lines
        for (int i = 0; i <= 8; i++)
        {
            double gx = centerX - visHalfX + (2.0 * visHalfX * i / 8.0);
            double gy = -visHalfY + (2.0 * visHalfY * i / 8.0);
            bool isBoundary = (i == 0 || i == 8);
            Color gridCol = isBoundary ? Color.FromRgb(90, 108, 130) : Color.FromRgb(68, 78, 92);
            double thickness = isBoundary ? 0.75 : 0.45;

            staticGroup.Children.Add(CreateBoxModel(new Point3D(gx, worktableTopY + 0.3, 0), thickness, 0.3, 2.0 * visHalfY, gridCol));
            staticGroup.Children.Add(CreateBoxModel(new Point3D(centerX, worktableTopY + 0.3, gy), 2.0 * visHalfX, 0.3, thickness, gridCol));
        }

        // Physical 0 mm crosshair lines + dynamic 3D Origin Triad (moves to Center or any of the 4 Corners!)
        bool hasZeroX = minX <= 0.0 && maxX >= 0.0;
        bool hasZeroY = minY <= 0.0 && maxY >= 0.0;
        double zeroVisX = centerX + MapPhysicalToVisual(0.0, minX, maxX, visHalfX);
        double zeroVisY = MapPhysicalToVisual(0.0, minY, maxY, visHalfY);

        if (hasZeroX)
            staticGroup.Children.Add(CreateBoxModel(new Point3D(zeroVisX, worktableTopY + 0.35, 0), 1.0, 0.35, 2.0 * visHalfY, Color.FromRgb(110, 130, 160)));
        if (hasZeroY)
            staticGroup.Children.Add(CreateBoxModel(new Point3D(centerX, worktableTopY + 0.35, zeroVisY), 2.0 * visHalfX, 0.35, 1.0, Color.FromRgb(110, 130, 160)));

        if (hasZeroX && hasZeroY)
        {
            double triadY = worktableTopY + 1.5;
            double triadLen = Math.Min(28.0, Math.Min(visHalfX, visHalfY) * 0.65);
            staticGroup.Children.Add(CreateBoxModel(new Point3D(zeroVisX, triadY, zeroVisY), 5.0, 3.0, 5.0, Color.FromRgb(235, 248, 255)));
            staticGroup.Children.Add(CreateBoxModel(new Point3D(zeroVisX + triadLen * 0.5, triadY, zeroVisY), triadLen, 2.2, 2.2, Color.FromRgb(255, 82, 82)));   // +X Red
            staticGroup.Children.Add(CreateBoxModel(new Point3D(zeroVisX, triadY, zeroVisY + triadLen * 0.5), 2.2, 2.2, triadLen, Color.FromRgb(105, 240, 174))); // +Y Green
            staticGroup.Children.Add(CreateBoxModel(new Point3D(zeroVisX, triadY + triadLen * 0.5, zeroVisY), 2.2, triadLen, 2.2, Color.FromRgb(68, 138, 255)));  // +Z Blue
        }

        // Fixed Y-Axis Side Rails
        staticGroup.Children.Add(CreateBoxModel(new Point3D(centerX - railOffsetX, worktableTopY + 5.5, 0), 8, 10, 2.0 * visHalfY + 30.0, Color.FromRgb(60, 90, 70)));
        staticGroup.Children.Add(CreateBoxModel(new Point3D(centerX + railOffsetX, worktableTopY + 5.5, 0), 8, 10, 2.0 * visHalfY + 30.0, Color.FromRgb(60, 90, 70)));

        // Y-Axis Limit Switch Blocks
        staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX - railOffsetX, worktableTopY + 13.5, visHalfY + 12.0), 10, 6, 6, _posLimitMats[yIdx]));
        staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX - railOffsetX, worktableTopY + 13.5, -(visHalfY + 12.0)), 10, 6, 6, _negLimitMats[yIdx]));

        // Optional Axis 4 (A): Rotary Worktable mounted at center of worktable
        if (includeRotaryTable)
        {
            _axisRotate[4].Axis = new Vector3D(0, 1, 0);
            double platterRadius = Math.Min(42.0, Math.Min(visHalfX, visHalfY) * 0.45);
            staticGroup.Children.Add(CreateCylinderModel(new Point3D(centerX, worktableTopY + 1.0, 0), platterRadius + 4.0, 2.0, Color.FromRgb(60, 68, 80)));
            staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX + 8.0, worktableTopY + 2.5, platterRadius + 4.0), 4, 4, 4, _posLimitMats[4]));
            staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX - 8.0, worktableTopY + 2.5, platterRadius + 4.0), 4, 4, 4, _negLimitMats[4]));

            var rotaryTableGroup = new Model3DGroup
            {
                Transform = new RotateTransform3D(_axisRotate[4], new Point3D(centerX, worktableTopY + 3.5, 0))
            };
            rotaryTableGroup.Children.Add(CreateCylinderModel(new Point3D(centerX, worktableTopY + 3.5, 0), platterRadius, 3.0, Color.FromRgb(84, 110, 122), segments: 24));
            rotaryTableGroup.Children.Add(CreateBoxModel(new Point3D(centerX + platterRadius * 0.45, worktableTopY + 5.5, 0), platterRadius * 0.85, 1.5, 3.0, Color.FromRgb(255, 213, 79)));
            rotaryTableGroup.Children.Add(CreateBoxModel(new Point3D(centerX, worktableTopY + 6.5, platterRadius * 0.65), 4, 3, 4, Color.FromRgb(207, 216, 220)));
            rotaryTableGroup.Children.Add(CreateBoxModel(new Point3D(centerX, worktableTopY + 6.5, -platterRadius * 0.65), 4, 3, 4, Color.FromRgb(207, 216, 220)));
            targetGroup.Children.Add(rotaryTableGroup);
        }

        // Optional Axis 5 (B): Auxiliary Linear Feeder Slide beside right rail
        if (includeAuxSlide)
        {
            double auxX = centerX + railOffsetX + 26.0;
            staticGroup.Children.Add(CreateBoxModel(new Point3D(auxX, worktableTopY + 4.5, 0), 14, 6, 2.0 * visHalfY + 20.0, Color.FromRgb(63, 81, 181)));
            staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(auxX + 9.0, worktableTopY + 9.0, visHalfY + 4.0), 4, 5, 5, _posLimitMats[5]));
            staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(auxX + 9.0, worktableTopY + 9.0, -(visHalfY + 4.0)), 4, 5, 5, _negLimitMats[5]));

            var auxCarriageGroup = new Model3DGroup { Transform = _axisTranslate[5] };
            auxCarriageGroup.Children.Add(CreateBoxModel(new Point3D(auxX, worktableTopY + 12.0, 0), 18, 9, 22, Color.FromRgb(92, 107, 192)));
            auxCarriageGroup.Children.Add(CreateBoxModel(new Point3D(auxX, worktableTopY + 18.0, 0), 12, 4, 14, Color.FromRgb(159, 168, 218)));
            targetGroup.Children.Add(auxCarriageGroup);
        }

        targetGroup.Children.Add(staticGroup);

        // Moving Y-Axis Gantry Bridge
        var yBridgeGroup = new Model3DGroup { Transform = _axisTranslate[yIdx] };
        double colBottom = worktableTopY + 10.5;
        double colTop = crossbeamY + 5.0;
        double colHeight = Math.Max(20.0, colTop - colBottom);
        double colCenterY = (colBottom + colTop) * 0.5;

        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(centerX - railOffsetX, colCenterY, 0), 12, colHeight, 18, colClr));
        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(centerX + railOffsetX, colCenterY, 0), 12, colHeight, 18, colClr));
        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(centerX, crossbeamY, 0), 2.0 * visHalfX + 36.0, 14, 14, bmClr));

        // X-Axis Limit Switch Blocks
        yBridgeGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX + visHalfX + 4.0, crossbeamY + 9.0, 0), 6, 6, 16, _posLimitMats[xIdx]));
        yBridgeGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX - (visHalfX + 4.0), crossbeamY + 9.0, 0), 6, 6, 16, _negLimitMats[xIdx]));

        // Moving X-Axis Carriage
        var xCarriageGroup = new Model3DGroup { Transform = _axisTranslate[xIdx] };
        xCarriageGroup.Children.Add(CreateBoxModel(new Point3D(centerX, crossbeamY, 8), 24, 22, 12, carClr));

        // Z-Axis Guide Rail & Limit Switch Blocks
        xCarriageGroup.Children.Add(CreateBoxModel(new Point3D(centerX + 14.0, crossbeamY, 12), 2.2, 2.0 * visHalfZ + 8.0, 4, Color.FromRgb(70, 82, 100)));
        xCarriageGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX + 14.0, crossbeamY + visHalfZ, 12), 5, 5, 8, _posLimitMats[zIdx]));
        xCarriageGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX + 14.0, crossbeamY - visHalfZ, 12), 5, 5, 8, _negLimitMats[zIdx]));

        // Moving Z-Axis Spindle & Tool Head
        var zSpindleGroup = new Model3DGroup { Transform = _axisTranslate[zIdx] };
        double slideHeight = 2.0 * visHalfZ + 2.0;
        zSpindleGroup.Children.Add(CreateBoxModel(new Point3D(centerX, visHalfZ + 1.0, 15), 14, slideHeight, 12, sldClr));
        zSpindleGroup.Children.Add(CreateBoxModel(new Point3D(centerX + 9.0, crossbeamY, 12), 4.5, 4.0, 4.0, Color.FromRgb(180, 190, 205)));
        zSpindleGroup.Children.Add(CreateBoxModel(new Point3D(centerX, 8, 18), 10, 22, 10, Color.FromRgb(180, 190, 205)));

        if (includeRotarySpindle)
        {
            _axisRotate[3].Axis = new Vector3D(0, 1, 0);
            zSpindleGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX + 6.5, 3, 18), 3, 3, 4, _posLimitMats[3]));
            zSpindleGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX - 6.5, 3, 18), 3, 3, 4, _negLimitMats[3]));

            var chuckGroup = new Model3DGroup
            {
                Transform = new RotateTransform3D(_axisRotate[3], new Point3D(centerX, -4, 18))
            };
            chuckGroup.Children.Add(CreateCylinderModel(new Point3D(centerX, -3, 18), 4.8, 5.0, Color.FromRgb(255, 167, 38), segments: 12));
            chuckGroup.Children.Add(CreateBoxModel(new Point3D(centerX + 5.5, -3, 18), 6.0, 2.5, 2.0, Color.FromRgb(255, 213, 79)));
            chuckGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX, -6, 18), 4.0, 5.0, 4.0, tipMat));
            zSpindleGroup.Children.Add(chuckGroup);
        }
        else
        {
            zSpindleGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(centerX, -4, 18), 4.5, 8, 4.5, tipMat));
        }

        xCarriageGroup.Children.Add(zSpindleGroup);
        yBridgeGroup.Children.Add(xCarriageGroup);
        targetGroup.Children.Add(yBridgeGroup);
    }

    private void BuildRobotArm6Dof()
    {
        // Floor plate & pedestal
        _machineGroup.Children.Add(CreateBoxModel(new Point3D(0, -61, 0), 240, 6, 240, Color.FromRgb(32, 38, 48)));
        for (int g = -100; g <= 100; g += 50)
        {
            _machineGroup.Children.Add(CreateBoxModel(new Point3D(g, -57.8, 0), 0.6, 0.3, 200, Color.FromRgb(64, 74, 88)));
            _machineGroup.Children.Add(CreateBoxModel(new Point3D(0, -57.8, g), 200, 0.3, 0.6, Color.FromRgb(64, 74, 88)));
        }

        _machineGroup.Children.Add(CreateCylinderModel(new Point3D(0, -47, 0), 24, 22, Color.FromRgb(55, 65, 81), segments: 24));
        _machineGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(26, -54, 10), 5, 5, 5, _posLimitMats[0]));
        _machineGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(26, -54, -10), 5, 5, 5, _negLimitMats[0]));

        // J1: Base waist rotation around Y at (0, -36, 0)
        _axisRotate[0].Axis = new Vector3D(0, 1, 0);
        var j1Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[0], new Point3D(0, -36, 0)) };
        j1Group.Children.Add(CreateCylinderModel(new Point3D(0, -23, 0), 18, 26, Color.FromRgb(230, 81, 0), segments: 20));
        j1Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(14, -12, 12), 4, 4, 4, _posLimitMats[1]));
        j1Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(-14, -12, 12), 4, 4, 4, _negLimitMats[1]));

        // J2: Shoulder pitch around Z at (0, -10, 0)
        _axisRotate[1].Axis = new Vector3D(0, 0, 1);
        var j2Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[1], new Point3D(0, -10, 0)) };
        j2Group.Children.Add(CreateCylinderModel(new Point3D(0, -10, 0), 12, 28, Color.FromRgb(69, 90, 100), alongZ: true));
        j2Group.Children.Add(CreateBoxModel(new Point3D(0, 27.5, 0), 14, 75, 14, Color.FromRgb(239, 108, 0)));
        j2Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(9, 60, 8), 4, 4, 4, _posLimitMats[2]));
        j2Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(-9, 60, 8), 4, 4, 4, _negLimitMats[2]));

        // J3: Elbow pitch around Z at (0, 65, 0)
        _axisRotate[2].Axis = new Vector3D(0, 0, 1);
        var j3Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[2], new Point3D(0, 65, 0)) };
        j3Group.Children.Add(CreateCylinderModel(new Point3D(0, 65, 0), 10, 24, Color.FromRgb(69, 90, 100), alongZ: true));
        j3Group.Children.Add(CreateBoxModel(new Point3D(0, 81, 0), 12, 32, 12, Color.FromRgb(251, 140, 0)));
        j3Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(8, 94, 0), 3.5, 3.5, 5, _posLimitMats[3]));
        j3Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(-8, 94, 0), 3.5, 3.5, 5, _negLimitMats[3]));

        // J4: Forearm roll around Y at (0, 97, 0)
        _axisRotate[3].Axis = new Vector3D(0, 1, 0);
        var j4Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[3], new Point3D(0, 97, 0)) };
        j4Group.Children.Add(CreateCylinderModel(new Point3D(0, 113.5, 0), 7.5, 33, Color.FromRgb(255, 167, 38), segments: 16));
        j4Group.Children.Add(CreateBoxModel(new Point3D(6.8, 113.5, 0), 2.5, 28, 3.0, Color.FromRgb(41, 182, 246)));
        j4Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(6.5, 126, 7), 3.5, 3.5, 3.5, _posLimitMats[4]));
        j4Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(-6.5, 126, 7), 3.5, 3.5, 3.5, _negLimitMats[4]));

        // J5: Wrist pitch around Z at (0, 130, 0)
        _axisRotate[4].Axis = new Vector3D(0, 0, 1);
        var j5Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[4], new Point3D(0, 130, 0)) };
        j5Group.Children.Add(CreateCylinderModel(new Point3D(0, 130, 0), 7, 18, Color.FromRgb(69, 90, 100), alongZ: true));
        j5Group.Children.Add(CreateBoxModel(new Point3D(0, 142, 0), 9, 24, 9, Color.FromRgb(120, 144, 156)));
        j5Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(6, 151, 0), 3, 3, 4, _posLimitMats[5]));
        j5Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(-6, 151, 0), 3, 3, 4, _negLimitMats[5]));

        // J6: Tool flange roll around Y at (0, 154, 0)
        _axisRotate[5].Axis = new Vector3D(0, 1, 0);
        var j6Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[5], new Point3D(0, 154, 0)) };
        j6Group.Children.Add(CreateCylinderModel(new Point3D(0, 157, 0), 8.5, 6, Color.FromRgb(176, 190, 197), segments: 16));
        j6Group.Children.Add(CreateBoxModel(new Point3D(5.0, 157, 0), 10, 3.0, 2.5, Color.FromRgb(255, 213, 79)));
        j6Group.Children.Add(CreateBoxModelWithMaterial(new Point3D(0, 164, 0), 4.5, 8, 4.5, _toolTipMat1));

        j5Group.Children.Add(j6Group);
        j4Group.Children.Add(j5Group);
        j3Group.Children.Add(j4Group);
        j2Group.Children.Add(j3Group);
        j1Group.Children.Add(j2Group);
        _machineGroup.Children.Add(j1Group);
    }

    private void BuildLinearBench6Axis()
    {
        // Wide test bench baseplate
        _machineGroup.Children.Add(CreateBoxModel(new Point3D(0, -35, 0), 255, 6, 225, Color.FromRgb(32, 38, 48)));
        // Center 0 mm reference bar across all 6 rails
        _machineGroup.Children.Add(CreateBoxModel(new Point3D(0, -31.5, 0), 225, 0.8, 1.5, Color.FromRgb(110, 130, 160)));

        Color[] railColors =
        {
            Color.FromRgb(30, 136, 229),  // Axis 0: Blue
            Color.FromRgb(67, 160, 71),   // Axis 1: Green
            Color.FromRgb(229, 57, 53),   // Axis 2: Red
            Color.FromRgb(0, 172, 193),   // Axis 3: Cyan
            Color.FromRgb(142, 36, 170),  // Axis 4: Purple
            Color.FromRgb(251, 140, 0)    // Axis 5: Amber
        };

        for (int i = 0; i < 6; i++)
        {
            double rx = -100.0 + i * 40.0;
            _machineGroup.Children.Add(CreateBoxModel(new Point3D(rx, -29, 0), 10, 6, 196, Color.FromRgb(58, 68, 84)));
            _machineGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(rx, -24, 95), 12, 5, 5, _posLimitMats[i]));
            _machineGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(rx, -24, -95), 12, 5, 5, _negLimitMats[i]));

            var carriage = new Model3DGroup { Transform = _axisTranslate[i] };
            carriage.Children.Add(CreateBoxModel(new Point3D(rx, -21, 0), 22, 12, 20, railColors[i]));
            carriage.Children.Add(CreateBoxModel(new Point3D(rx, -14.5, 0), 16, 2, 12, Color.FromRgb(236, 240, 246)));
            _machineGroup.Children.Add(carriage);
        }
    }

    private Point3D ComputeRobotArmTcpWorld(double a0, double a1, double a2, double a3, double a4, double a5)
    {
        double j1 = MapPhysicalToAngle(a0, _minMm[0], _maxMm[0], 170.0);
        double j2 = MapPhysicalToAngle(a1, _minMm[1], _maxMm[1], 110.0);
        double j3 = MapPhysicalToAngle(a2, _minMm[2], _maxMm[2], 135.0) - 90.0;
        double j4 = MapPhysicalToAngle(a3, _minMm[3], _maxMm[3], 180.0);
        double j5 = MapPhysicalToAngle(a4, _minMm[4], _maxMm[4], 120.0);
        double j6 = MapPhysicalToAngle(a5, _minMm[5], _maxMm[5], 180.0);

        Point3D p = new(0, 168, 0);
        p = RotatePointAroundAxis(p, new Point3D(0, 154, 0), new Vector3D(0, 1, 0), j6);
        p = RotatePointAroundAxis(p, new Point3D(0, 130, 0), new Vector3D(0, 0, 1), j5);
        p = RotatePointAroundAxis(p, new Point3D(0, 97, 0), new Vector3D(0, 1, 0), j4);
        p = RotatePointAroundAxis(p, new Point3D(0, 65, 0), new Vector3D(0, 0, 1), j3);
        p = RotatePointAroundAxis(p, new Point3D(0, -10, 0), new Vector3D(0, 0, 1), j2);
        p = RotatePointAroundAxis(p, new Point3D(0, -36, 0), new Vector3D(0, 1, 0), j1);
        return p;
    }

    private static Point3D RotatePointAroundAxis(Point3D p, Point3D pivot, Vector3D axis, double angleDeg)
    {
        var m = Matrix3D.Identity;
        m.RotateAt(new Quaternion(axis, angleDeg), pivot);
        return m.Transform(p);
    }

    private static double MapPhysicalToVisual(double posMm, double minMm, double maxMm, double visHalf)
    {
        double span = Math.Max(1e-6, maxMm - minMm);
        double normalized = ((posMm - minMm) / span) * 2.0 - 1.0;
        return Math.Clamp(normalized, -1.03, 1.03) * visHalf;
    }

    private static double MapPhysicalToAngle(double posMm, double minMm, double maxMm, double halfAngleDeg)
    {
        double span = Math.Max(1e-6, maxMm - minMm);
        double normalized = ((posMm - minMm) / span) * 2.0 - 1.0;
        return Math.Clamp(normalized, -1.0, 1.0) * halfAngleDeg;
    }

    private Point3D SampleToTcpWorld1(TrailSample s)
    {
        return SceneMode switch
        {
            MachineSceneMode.DualGantry6Axis => new Point3D(
                -DualStationOffsetX + MapPhysicalToVisual(s.A2, _minMm[2], _maxMm[2], DualHalfX),
                MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], DualHalfZ) - 8.0,
                MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], DualHalfY) + 18.0),
            MachineSceneMode.RobotArm6Dof => ComputeRobotArmTcpWorld(s.A0, s.A1, s.A2, s.A3, s.A4, s.A5),
            _ => new Point3D(
                MapPhysicalToVisual(s.A2, _minMm[2], _maxMm[2], _visHalfX),
                MapPhysicalToVisual(s.A0, _minMm[0], _maxMm[0], _visHalfZ) - 8.0,
                MapPhysicalToVisual(s.A1, _minMm[1], _maxMm[1], _visHalfY) + 18.0)
        };
    }

    private Point3D SampleToTcpWorld2(TrailSample s)
    {
        return new Point3D(
            +DualStationOffsetX + MapPhysicalToVisual(s.A5, _minMm[5], _maxMm[5], DualHalfX),
            MapPhysicalToVisual(s.A3, _minMm[3], _maxMm[3], DualHalfZ) - 8.0,
            MapPhysicalToVisual(s.A4, _minMm[4], _maxMm[4], DualHalfY) + 18.0);
    }

    private void AppendTrailSample1(ReadOnlySpan<Axis3DFrame> axes, Point3D tcpWorld)
    {
        if (double.IsNaN(_lastTrailPointWorld1.X) || (tcpWorld - _lastTrailPointWorld1).Length > 0.6)
        {
            _lastTrailPointWorld1 = tcpWorld;
            _trailSamples1.Add(new TrailSample(
                axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm));
            if (_trailSamples1.Count > 600)
                _trailSamples1.RemoveAt(0);
            RebuildSingleTrailMesh(_trailModel1, _trailSamples1, SampleToTcpWorld1, ref _lastTrailPointWorld1);
        }
    }

    private void AppendTrailSample2(ReadOnlySpan<Axis3DFrame> axes, Point3D tcpWorld)
    {
        if (double.IsNaN(_lastTrailPointWorld2.X) || (tcpWorld - _lastTrailPointWorld2).Length > 0.6)
        {
            _lastTrailPointWorld2 = tcpWorld;
            _trailSamples2.Add(new TrailSample(
                axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm));
            if (_trailSamples2.Count > 600)
                _trailSamples2.RemoveAt(0);
            RebuildSingleTrailMesh(_trailModel2, _trailSamples2, SampleToTcpWorld2, ref _lastTrailPointWorld2);
        }
    }

    public void ClearTrajectoryTrail()
    {
        _trailSamples1.Clear();
        _trailSamples2.Clear();
        _lastTrailPointWorld1 = new Point3D(double.NaN, double.NaN, double.NaN);
        _lastTrailPointWorld2 = new Point3D(double.NaN, double.NaN, double.NaN);
        ClearMesh(_trailModel1);
        ClearMesh(_trailModel2);
    }

    private static void ClearMesh(GeometryModel3D model)
    {
        if (model.Geometry is MeshGeometry3D mesh)
        {
            mesh.Positions.Clear();
            mesh.TriangleIndices.Clear();
        }
    }

    private void RebuildTrailMeshes()
    {
        RebuildSingleTrailMesh(_trailModel1, _trailSamples1, SampleToTcpWorld1, ref _lastTrailPointWorld1);
        if (SceneMode == MachineSceneMode.DualGantry6Axis)
            RebuildSingleTrailMesh(_trailModel2, _trailSamples2, SampleToTcpWorld2, ref _lastTrailPointWorld2);
        else
            ClearMesh(_trailModel2);
    }

    private static void RebuildSingleTrailMesh(
        GeometryModel3D model,
        List<TrailSample> samples,
        Func<TrailSample, Point3D> toWorld,
        ref Point3D lastWorld)
    {
        if (model.Geometry is not MeshGeometry3D mesh) return;
        mesh.Positions.Clear();
        mesh.TriangleIndices.Clear();

        if (samples.Count > 0)
            lastWorld = toWorld(samples[^1]);

        if (samples.Count < 2) return;

        const double ribbonHalfWidth = 0.85;
        for (int i = 0; i < samples.Count - 1; i++)
        {
            Point3D p0 = toWorld(samples[i]);
            Point3D p1 = toWorld(samples[i + 1]);

            int baseIdx = mesh.Positions.Count;
            mesh.Positions.Add(new Point3D(p0.X - ribbonHalfWidth, p0.Y, p0.Z - ribbonHalfWidth));
            mesh.Positions.Add(new Point3D(p0.X + ribbonHalfWidth, p0.Y, p0.Z + ribbonHalfWidth));
            mesh.Positions.Add(new Point3D(p1.X + ribbonHalfWidth, p1.Y, p1.Z + ribbonHalfWidth));
            mesh.Positions.Add(new Point3D(p1.X - ribbonHalfWidth, p1.Y, p1.Z - ribbonHalfWidth));

            mesh.TriangleIndices.Add(baseIdx + 0);
            mesh.TriangleIndices.Add(baseIdx + 1);
            mesh.TriangleIndices.Add(baseIdx + 2);

            mesh.TriangleIndices.Add(baseIdx + 0);
            mesh.TriangleIndices.Add(baseIdx + 2);
            mesh.TriangleIndices.Add(baseIdx + 3);
        }
    }

    private static void UpdateLimitMaterial(DiffuseMaterial mat, bool triggered)
    {
        if (mat.Brush is SolidColorBrush brush)
        {
            brush.Color = triggered
                ? Color.FromRgb(255, 23, 68)   // Bright Red when limit switch is active
                : Color.FromRgb(45, 110, 65);  // Dim Green when normal
        }
    }

    public static GeometryModel3D CreateBoxModel(Point3D center, double sizeX, double sizeY, double sizeZ, Color color)
    {
        var matGroup = new MaterialGroup();
        matGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
        matGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Color.FromRgb(180, 180, 180)), 35));
        return CreateBoxModelWithMaterial(center, sizeX, sizeY, sizeZ, matGroup);
    }

    public static GeometryModel3D CreateBoxModelWithMaterial(Point3D center, double sizeX, double sizeY, double sizeZ, Material material)
    {
        double hx = sizeX / 2.0;
        double hy = sizeY / 2.0;
        double hz = sizeZ / 2.0;

        Point3D p0 = new(center.X - hx, center.Y - hy, center.Z + hz);
        Point3D p1 = new(center.X + hx, center.Y - hy, center.Z + hz);
        Point3D p2 = new(center.X + hx, center.Y + hy, center.Z + hz);
        Point3D p3 = new(center.X - hx, center.Y + hy, center.Z + hz);
        Point3D p4 = new(center.X - hx, center.Y - hy, center.Z - hz);
        Point3D p5 = new(center.X + hx, center.Y - hy, center.Z - hz);
        Point3D p6 = new(center.X + hx, center.Y + hy, center.Z - hz);
        Point3D p7 = new(center.X - hx, center.Y + hy, center.Z - hz);

        var mesh = new MeshGeometry3D();
        AddQuad(mesh, p0, p1, p2, p3); // Front (+Z)
        AddQuad(mesh, p5, p4, p7, p6); // Back (-Z)
        AddQuad(mesh, p4, p0, p3, p7); // Left (-X)
        AddQuad(mesh, p1, p5, p6, p2); // Right (+X)
        AddQuad(mesh, p3, p2, p6, p7); // Top (+Y)
        AddQuad(mesh, p4, p5, p1, p0); // Bottom (-Y)

        return new GeometryModel3D
        {
            Geometry = mesh,
            Material = material,
            BackMaterial = material
        };
    }

    public static GeometryModel3D CreateCylinderModel(Point3D center, double radius, double height, Color color, int segments = 16, bool alongZ = false)
    {
        var matGroup = new MaterialGroup();
        matGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
        matGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Color.FromRgb(180, 180, 180)), 35));

        var mesh = new MeshGeometry3D();
        double half = height * 0.5;
        Point3D capCenterPos = alongZ ? new Point3D(center.X, center.Y, center.Z + half) : new Point3D(center.X, center.Y + half, center.Z);
        Point3D capCenterNeg = alongZ ? new Point3D(center.X, center.Y, center.Z - half) : new Point3D(center.X, center.Y - half, center.Z);

        for (int i = 0; i < segments; i++)
        {
            double a0 = 2.0 * Math.PI * i / segments;
            double a1 = 2.0 * Math.PI * (i + 1) / segments;
            double c0 = radius * Math.Cos(a0), s0 = radius * Math.Sin(a0);
            double c1 = radius * Math.Cos(a1), s1 = radius * Math.Sin(a1);

            Point3D p0Pos = alongZ ? new Point3D(center.X + c0, center.Y + s0, center.Z + half) : new Point3D(center.X + c0, center.Y + half, center.Z + s0);
            Point3D p1Pos = alongZ ? new Point3D(center.X + c1, center.Y + s1, center.Z + half) : new Point3D(center.X + c1, center.Y + half, center.Z + s1);
            Point3D p0Neg = alongZ ? new Point3D(center.X + c0, center.Y + s0, center.Z - half) : new Point3D(center.X + c0, center.Y - half, center.Z + s0);
            Point3D p1Neg = alongZ ? new Point3D(center.X + c1, center.Y + s1, center.Z - half) : new Point3D(center.X + c1, center.Y - half, center.Z + s1);

            // Side quad
            AddQuad(mesh, p0Neg, p1Neg, p1Pos, p0Pos);

            // Positive end cap triangle
            int t1 = mesh.Positions.Count;
            mesh.Positions.Add(capCenterPos);
            mesh.Positions.Add(p0Pos);
            mesh.Positions.Add(p1Pos);
            mesh.TriangleIndices.Add(t1);
            mesh.TriangleIndices.Add(t1 + 1);
            mesh.TriangleIndices.Add(t1 + 2);

            // Negative end cap triangle
            int t2 = mesh.Positions.Count;
            mesh.Positions.Add(capCenterNeg);
            mesh.Positions.Add(p1Neg);
            mesh.Positions.Add(p0Neg);
            mesh.TriangleIndices.Add(t2);
            mesh.TriangleIndices.Add(t2 + 1);
            mesh.TriangleIndices.Add(t2 + 2);
        }

        return new GeometryModel3D
        {
            Geometry = mesh,
            Material = matGroup,
            BackMaterial = matGroup
        };
    }

    private static void AddQuad(MeshGeometry3D mesh, Point3D a, Point3D b, Point3D c, Point3D d)
    {
        int start = mesh.Positions.Count;
        mesh.Positions.Add(a);
        mesh.Positions.Add(b);
        mesh.Positions.Add(c);
        mesh.Positions.Add(d);

        mesh.TriangleIndices.Add(start + 0);
        mesh.TriangleIndices.Add(start + 1);
        mesh.TriangleIndices.Add(start + 2);

        mesh.TriangleIndices.Add(start + 0);
        mesh.TriangleIndices.Add(start + 2);
        mesh.TriangleIndices.Add(start + 3);
    }
}
