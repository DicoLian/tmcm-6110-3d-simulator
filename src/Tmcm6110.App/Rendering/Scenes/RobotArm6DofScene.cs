using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// 6-DOF Articulated Industrial Robot Arm scene (J1..J6) with exact forward-kinematics TCP trail.
/// </summary>
public sealed class RobotArm6DofScene : IMachine3DScene
{
    private readonly Model3DGroup _machineGroup = new();
    private readonly AxisAngleRotation3D[] _axisRotate = new AxisAngleRotation3D[6];
    private readonly DiffuseMaterial[] _posLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial[] _negLimitMats = new DiffuseMaterial[6];
    private readonly DiffuseMaterial _toolTipMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));
    private readonly GeometryModel3D _trailModel = Mesh3DPrimitives.CreateTrailModel(Color.FromRgb(255, 215, 0));
    private readonly List<TrailSample6> _trailSamples = new();
    private Point3D _lastTrailPointWorld = new(double.NaN, double.NaN, double.NaN);

    private readonly double[] _minMm = { -45.0, -100.0, -100.0, -100.0, -100.0, -100.0 };
    private readonly double[] _maxMm = { 45.0, 100.0, 100.0, 100.0, 100.0, 100.0 };

    public SceneDescriptor Descriptor { get; } = new(MachineSceneMode.RobotArm6Dof, 6, 6, 6);
    public Model3DGroup SceneGroup { get; } = new();

    public RobotArm6DofScene()
    {
        for (int i = 0; i < 6; i++)
        {
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
        if (axes.Length < 6) return;

        if (Mesh3DPrimitives.SyncBoundsIfChanged(axes, _minMm, _maxMm, 6))
        {
            Mesh3DPrimitives.RebuildRibbonTrailMesh(_trailModel, _trailSamples, SampleToTcpWorld, ref _lastTrailPointWorld);
        }

        for (int i = 0; i < 6; i++)
        {
            Mesh3DPrimitives.UpdateLimitMaterial(_posLimitMats[i], axes[i].IsRightLimit);
            Mesh3DPrimitives.UpdateLimitMaterial(_negLimitMats[i], axes[i].IsLeftLimit);
        }

        bool anyAlarm = axes[0].IsAlarm || axes[1].IsAlarm || axes[2].IsAlarm ||
                        axes[3].IsAlarm || axes[4].IsAlarm || axes[5].IsAlarm;
        if (_toolTipMat.Brush is SolidColorBrush tip)
            tip.Color = anyAlarm ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);

        _axisRotate[0].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[0].PositionMm, _minMm[0], _maxMm[0], 170.0);
        _axisRotate[1].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[1].PositionMm, _minMm[1], _maxMm[1], 110.0);
        _axisRotate[2].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[2].PositionMm, _minMm[2], _maxMm[2], 135.0) - 90.0;
        _axisRotate[3].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[3].PositionMm, _minMm[3], _maxMm[3], 180.0);
        _axisRotate[4].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[4].PositionMm, _minMm[4], _maxMm[4], 120.0);
        _axisRotate[5].Angle = Mesh3DPrimitives.MapPhysicalToAngle(axes[5].PositionMm, _minMm[5], _maxMm[5], 180.0);

        var tcp = ComputeTcpWorld(
            axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
            axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm);
        if (double.IsNaN(_lastTrailPointWorld.X) || (tcp - _lastTrailPointWorld).Length > 0.6)
        {
            _lastTrailPointWorld = tcp;
            _trailSamples.Add(new TrailSample6(
                axes[0].PositionMm, axes[1].PositionMm, axes[2].PositionMm,
                axes[3].PositionMm, axes[4].PositionMm, axes[5].PositionMm));
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
            _axisRotate[i].Angle = 0;
    }

    private Point3D SampleToTcpWorld(TrailSample6 s) => ComputeTcpWorld(s.A0, s.A1, s.A2, s.A3, s.A4, s.A5);

    private Point3D ComputeTcpWorld(double a0, double a1, double a2, double a3, double a4, double a5)
    {
        double j1 = Mesh3DPrimitives.MapPhysicalToAngle(a0, _minMm[0], _maxMm[0], 170.0);
        double j2 = Mesh3DPrimitives.MapPhysicalToAngle(a1, _minMm[1], _maxMm[1], 110.0);
        double j3 = Mesh3DPrimitives.MapPhysicalToAngle(a2, _minMm[2], _maxMm[2], 135.0) - 90.0;
        double j4 = Mesh3DPrimitives.MapPhysicalToAngle(a3, _minMm[3], _maxMm[3], 180.0);
        double j5 = Mesh3DPrimitives.MapPhysicalToAngle(a4, _minMm[4], _maxMm[4], 120.0);
        double j6 = Mesh3DPrimitives.MapPhysicalToAngle(a5, _minMm[5], _maxMm[5], 180.0);

        Point3D p = new(0, 168, 0);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, 154, 0), new Vector3D(0, 1, 0), j6);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, 130, 0), new Vector3D(0, 0, 1), j5);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, 97, 0), new Vector3D(0, 1, 0), j4);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, 65, 0), new Vector3D(0, 0, 1), j3);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, -10, 0), new Vector3D(0, 0, 1), j2);
        p = Mesh3DPrimitives.RotatePointAroundAxis(p, new Point3D(0, -36, 0), new Vector3D(0, 1, 0), j1);
        return p;
    }

    private void RebuildGeometry()
    {
        _machineGroup.Children.Clear();

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -61, 0), 240, 6, 240, Color.FromRgb(32, 38, 48)));
        for (int g = -100; g <= 100; g += 50)
        {
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(g, -57.8, 0), 0.6, 0.3, 200, Color.FromRgb(64, 74, 88)));
            _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, -57.8, g), 200, 0.3, 0.6, Color.FromRgb(64, 74, 88)));
        }

        _machineGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, -47, 0), 24, 22, Color.FromRgb(55, 65, 81), segments: 24));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(26, -54, 10), 5, 5, 5, _posLimitMats[0]));
        _machineGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(26, -54, -10), 5, 5, 5, _negLimitMats[0]));

        // J1: Base waist rotation around Y at (0, -36, 0)
        _axisRotate[0].Axis = new Vector3D(0, 1, 0);
        var j1Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[0], new Point3D(0, -36, 0)) };
        j1Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, -23, 0), 18, 26, Color.FromRgb(230, 81, 0), segments: 20));
        j1Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(14, -12, 12), 4, 4, 4, _posLimitMats[1]));
        j1Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-14, -12, 12), 4, 4, 4, _negLimitMats[1]));

        // J2: Shoulder pitch around Z at (0, -10, 0)
        _axisRotate[1].Axis = new Vector3D(0, 0, 1);
        var j2Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[1], new Point3D(0, -10, 0)) };
        j2Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, -10, 0), 12, 28, Color.FromRgb(69, 90, 100), alongZ: true));
        j2Group.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, 27.5, 0), 14, 75, 14, Color.FromRgb(239, 108, 0)));
        j2Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(9, 60, 8), 4, 4, 4, _posLimitMats[2]));
        j2Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-9, 60, 8), 4, 4, 4, _negLimitMats[2]));

        // J3: Elbow pitch around Z at (0, 65, 0)
        _axisRotate[2].Axis = new Vector3D(0, 0, 1);
        var j3Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[2], new Point3D(0, 65, 0)) };
        j3Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, 65, 0), 10, 24, Color.FromRgb(69, 90, 100), alongZ: true));
        j3Group.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, 81, 0), 12, 32, 12, Color.FromRgb(251, 140, 0)));
        j3Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(8, 94, 0), 3.5, 3.5, 5, _posLimitMats[3]));
        j3Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-8, 94, 0), 3.5, 3.5, 5, _negLimitMats[3]));

        // J4: Forearm roll around Y at (0, 97, 0)
        _axisRotate[3].Axis = new Vector3D(0, 1, 0);
        var j4Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[3], new Point3D(0, 97, 0)) };
        j4Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, 113.5, 0), 7.5, 33, Color.FromRgb(255, 167, 38), segments: 16));
        j4Group.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(6.8, 113.5, 0), 2.5, 28, 3.0, Color.FromRgb(41, 182, 246)));
        j4Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(6.5, 126, 7), 3.5, 3.5, 3.5, _posLimitMats[4]));
        j4Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-6.5, 126, 7), 3.5, 3.5, 3.5, _negLimitMats[4]));

        // J5: Wrist pitch around Z at (0, 130, 0)
        _axisRotate[4].Axis = new Vector3D(0, 0, 1);
        var j5Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[4], new Point3D(0, 130, 0)) };
        j5Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, 130, 0), 7, 18, Color.FromRgb(69, 90, 100), alongZ: true));
        j5Group.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(0, 142, 0), 9, 24, 9, Color.FromRgb(120, 144, 156)));
        j5Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(6, 151, 0), 3, 3, 4, _posLimitMats[5]));
        j5Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(-6, 151, 0), 3, 3, 4, _negLimitMats[5]));

        // J6: Tool flange roll around Y at (0, 154, 0)
        _axisRotate[5].Axis = new Vector3D(0, 1, 0);
        var j6Group = new Model3DGroup { Transform = new RotateTransform3D(_axisRotate[5], new Point3D(0, 154, 0)) };
        j6Group.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(0, 157, 0), 8.5, 6, Color.FromRgb(176, 190, 197), segments: 16));
        j6Group.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(5.0, 157, 0), 10, 3.0, 2.5, Color.FromRgb(255, 213, 79)));
        j6Group.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(0, 164, 0), 4.5, 8, 4.5, _toolTipMat));

        j5Group.Children.Add(j6Group);
        j4Group.Children.Add(j5Group);
        j3Group.Children.Add(j4Group);
        j2Group.Children.Add(j3Group);
        j1Group.Children.Add(j2Group);
        _machineGroup.Children.Add(j1Group);
    }
}
