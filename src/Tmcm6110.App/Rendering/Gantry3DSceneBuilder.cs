using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering;

/// <summary>
/// Pure native WPF 3D scene builder (zero external NuGet dependencies required).
/// Constructs the 3D XYZ Gantry Stage, Limit Switch indicators, Coordinate Grid, and 3D Tool Trajectory Trail.
/// Coordinate convention in 3D View:
/// - World X (Left/Right) = Physical X Axis (Motor #2)
/// - World Y (Up/Down)    = Physical Z Axis (Motor #0, vertical elevation)
/// - World Z (Front/Back) = Physical Y Axis (Motor #1, depth travel)
/// </summary>
public class Gantry3DSceneBuilder
{
    public Model3DGroup RootGroup { get; } = new();

    // Dynamic transforms updated at 60 FPS from Axis 0 (Z), Axis 1 (Y), Axis 2 (X) positions
    public TranslateTransform3D XCarriageTransform { get; } = new();
    public TranslateTransform3D YBridgeTransform { get; } = new();
    public TranslateTransform3D ZSpindleTransform { get; } = new();

    // Limit switch materials that turn bright red when +Limit or -Limit is active
    private readonly DiffuseMaterial _xPosLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _xNegLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _yPosLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _yNegLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _zPosLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _zNegLimitMat = new(new SolidColorBrush(Color.FromRgb(40, 120, 60)));
    private readonly DiffuseMaterial _toolTipMat = new(new SolidColorBrush(Color.FromRgb(0, 230, 118)));

    private readonly GeometryModel3D _trailModel;
    private readonly List<Point3D> _trailPoints = new();
    private Point3D _lastTrailPoint = new(double.NaN, double.NaN, double.NaN);

    public Gantry3DSceneBuilder()
    {
        BuildLighting();
        BuildBaseAndGrid();
        BuildGantryHierarchy();

        _trailModel = new GeometryModel3D
        {
            Geometry = new MeshGeometry3D(),
            Material = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(255, 215, 0))),
            BackMaterial = new EmissiveMaterial(new SolidColorBrush(Color.FromRgb(255, 215, 0)))
        };
        RootGroup.Children.Add(_trailModel);
    }

    private void BuildLighting()
    {
        RootGroup.Children.Add(new AmbientLight(Color.FromRgb(95, 105, 120)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(220, 225, 235), new Vector3D(-1.2, -2.0, -1.5)));
        RootGroup.Children.Add(new DirectionalLight(Color.FromRgb(110, 130, 160), new Vector3D(1.5, 1.0, 1.2)));
    }

    private void BuildBaseAndGrid()
    {
        var staticGroup = new Model3DGroup();

        // Dark metallic machine base plate (-130..+130 in X and Y(depth), height -65..-58)
        staticGroup.Children.Add(CreateBoxModel(new Point3D(0, -62, 0), 260, 6, 260, Color.FromRgb(32, 38, 48)));

        // Worktable surface (-105..+105)
        staticGroup.Children.Add(CreateBoxModel(new Point3D(0, -57, 0), 210, 3, 210, Color.FromRgb(48, 56, 68)));

        // Coordinate grid lines on worktable (every 25 mm from -100 to +100)
        for (int g = -100; g <= 100; g += 25)
        {
            Color gridCol = (g == 0) ? Color.FromRgb(110, 130, 160) : Color.FromRgb(68, 78, 92);
            double thickness = (g == 0) ? 1.0 : 0.45;
            staticGroup.Children.Add(CreateBoxModel(new Point3D(g, -55.2, 0), thickness, 0.3, 200, gridCol));
            staticGroup.Children.Add(CreateBoxModel(new Point3D(0, -55.2, g), 200, 0.3, thickness, gridCol));
        }

        // Fixed Y-Axis Side Rails (Greenish rails along World Z at X = -115 and X = +115)
        staticGroup.Children.Add(CreateBoxModel(new Point3D(-115, -50, 0), 8, 10, 230, Color.FromRgb(60, 90, 70)));
        staticGroup.Children.Add(CreateBoxModel(new Point3D(115, -50, 0), 8, 10, 230, Color.FromRgb(60, 90, 70)));

        // Y-Axis Limit Switch Blocks (Motor #1: Front/Back at World Z = +102 and -102)
        staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(-115, -42, 104), 10, 6, 6, _yPosLimitMat));
        staticGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(-115, -42, -104), 10, 6, 6, _yNegLimitMat));

        // 3D Origin Reference Axis Triad at corner (-105, -54, -105)
        staticGroup.Children.Add(CreateBoxModel(new Point3D(-85, -54, -105), 40, 2.2, 2.2, Color.FromRgb(255, 82, 82)));   // +X Red
        staticGroup.Children.Add(CreateBoxModel(new Point3D(-105, -54, -85), 2.2, 2.2, 40, Color.FromRgb(105, 240, 174))); // +Y Green
        staticGroup.Children.Add(CreateBoxModel(new Point3D(-105, -34, -105), 2.2, 40, 2.2, Color.FromRgb(68, 138, 255))); // +Z Blue

        RootGroup.Children.Add(staticGroup);
    }

    private void BuildGantryHierarchy()
    {
        // 1. Y-Axis Moving Gantry Bridge (Motor #1 moves along World Z)
        var yBridgeGroup = new Model3DGroup { Transform = YBridgeTransform };

        // Left & Right Vertical Gantry Columns
        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(-115, 5, 0), 12, 105, 18, Color.FromRgb(46, 125, 50)));
        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(115, 5, 0), 12, 105, 18, Color.FromRgb(46, 125, 50)));

        // Horizontal X-Axis Crossbeam spanning across X (-120..+120) at elevation Y = +52
        yBridgeGroup.Children.Add(CreateBoxModel(new Point3D(0, 52, 0), 242, 14, 14, Color.FromRgb(198, 40, 40)));

        // X-Axis Limit Switch Sensor Blocks on Crossbeam (Motor #2: Left/Right at X = +102 and -102)
        yBridgeGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(104, 61, 0), 6, 6, 16, _xPosLimitMat));
        yBridgeGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(-104, 61, 0), 6, 6, 16, _xNegLimitMat));

        // 2. X-Axis Carriage (Motor #2 moves along World X on the Y-Bridge)
        var xCarriageGroup = new Model3DGroup { Transform = XCarriageTransform };
        xCarriageGroup.Children.Add(CreateBoxModel(new Point3D(0, 52, 8), 24, 22, 12, Color.FromRgb(239, 83, 80)));

        // Z-Axis Guide Plate & Limit Switch Blocks mounted on X-Carriage
        xCarriageGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(14, 85, 12), 5, 5, 8, _zPosLimitMat));
        xCarriageGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(14, 15, 12), 5, 5, 8, _zNegLimitMat));

        // 3. Z-Axis Vertical Spindle & Tool Head (Motor #0 moves up/down along World Y)
        var zSpindleGroup = new Model3DGroup { Transform = ZSpindleTransform };
        // Blue Z-Axis Slide Column
        zSpindleGroup.Children.Add(CreateBoxModel(new Point3D(0, 35, 15), 14, 70, 12, Color.FromRgb(30, 136, 229)));
        // Spindle Motor Body
        zSpindleGroup.Children.Add(CreateBoxModel(new Point3D(0, 8, 18), 10, 22, 10, Color.FromRgb(180, 190, 205)));
        // Glowing Tool Center Point (TCP) Nozzle Tip at (0, 0, 18) when X=0, Y=0, Z=0
        zSpindleGroup.Children.Add(CreateBoxModelWithMaterial(new Point3D(0, -4, 18), 4.5, 8, 4.5, _toolTipMat));

        xCarriageGroup.Children.Add(zSpindleGroup);
        yBridgeGroup.Children.Add(xCarriageGroup);
        RootGroup.Children.Add(yBridgeGroup);
    }

    /// <summary>
    /// Updates the 3D Gantry Stage positions, limit indicator colors, and 3D tool trajectory trail.
    /// </summary>
    public void UpdateScene(
        double zAxisMm,
        double yAxisMm,
        double xAxisMm,
        bool zPosLim, bool zNegLim,
        bool yPosLim, bool yNegLim,
        bool xPosLim, bool xNegLim,
        bool anyAlarm)
    {
        // Clamp visual displacement to physical machine frame (-105mm .. +105mm)
        double clampedX = Math.Clamp(xAxisMm, -102, 102);
        double clampedY = Math.Clamp(yAxisMm, -102, 102);
        double clampedZ = Math.Clamp(zAxisMm, -48, 48);

        XCarriageTransform.OffsetX = clampedX;
        YBridgeTransform.OffsetZ = clampedY;
        ZSpindleTransform.OffsetY = clampedZ;

        // Update Limit Switch LED blocks in 3D view
        UpdateLimitMaterial(_zPosLimitMat, zPosLim);
        UpdateLimitMaterial(_zNegLimitMat, zNegLim);
        UpdateLimitMaterial(_yPosLimitMat, yPosLim);
        UpdateLimitMaterial(_yNegLimitMat, yNegLim);
        UpdateLimitMaterial(_xPosLimitMat, xPosLim);
        UpdateLimitMaterial(_xNegLimitMat, xNegLim);

        // Update Tool Tip Color (Red on Alarm, Cyan/Green on Normal)
        if (_toolTipMat.Brush is SolidColorBrush tipBrush)
        {
            tipBrush.Color = anyAlarm ? Color.FromRgb(255, 23, 68) : Color.FromRgb(0, 230, 118);
        }

        // Compute Tool Center Point (TCP) in World 3D coordinates and append to 3D Trajectory Trail
        var tcpWorld = new Point3D(clampedX, clampedZ - 8.0, clampedY + 18.0);
        if (double.IsNaN(_lastTrailPoint.X) || (tcpWorld - _lastTrailPoint).Length > 0.6)
        {
            _lastTrailPoint = tcpWorld;
            _trailPoints.Add(tcpWorld);
            if (_trailPoints.Count > 600)
                _trailPoints.RemoveAt(0);

            RebuildTrailMesh();
        }
    }

    public void ClearTrajectoryTrail()
    {
        _trailPoints.Clear();
        _lastTrailPoint = new Point3D(double.NaN, double.NaN, double.NaN);
        if (_trailModel.Geometry is MeshGeometry3D mesh)
        {
            mesh.Positions.Clear();
            mesh.TriangleIndices.Clear();
        }
    }

    private void RebuildTrailMesh()
    {
        if (_trailModel.Geometry is not MeshGeometry3D mesh) return;
        mesh.Positions.Clear();
        mesh.TriangleIndices.Clear();

        if (_trailPoints.Count < 2) return;

        const double ribbonHalfWidth = 0.85;
        for (int i = 0; i < _trailPoints.Count - 1; i++)
        {
            Point3D p0 = _trailPoints[i];
            Point3D p1 = _trailPoints[i + 1];

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
