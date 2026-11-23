using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering;

public readonly record struct TrailSample6(
    double A0,
    double A1,
    double A2,
    double A3,
    double A4,
    double A5
);

/// <summary>
/// Shared WPF 3D mesh builders, coordinate mappers, limit LED updaters, and trajectory ribbon helpers.
/// </summary>
public static class Mesh3DPrimitives
{
    public static bool SyncBoundsIfChanged(ReadOnlySpan<Axis3DFrame> axes, double[] minMm, double[] maxMm, int count)
    {
        bool changed = false;
        const double eps = 1e-4;
        for (int i = 0; i < count && i < axes.Length; i++)
        {
            double min = axes[i].MinMm;
            double max = axes[i].MaxMm <= min ? min + 1.0 : axes[i].MaxMm;
            if (Math.Abs(minMm[i] - min) >= eps || Math.Abs(maxMm[i] - max) >= eps)
            {
                minMm[i] = min;
                maxMm[i] = max;
                changed = true;
            }
        }
        return changed;
    }

    public static (double visHalfX, double visHalfY, double visHalfZ) ComputeGantryVisualHalfSpans(
        double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
    {
        double spanX = Math.Max(1.0, maxX - minX);
        double spanY = Math.Max(1.0, maxY - minY);
        double spanZ = Math.Max(1.0, maxZ - minZ);

        double scaleXY = Math.Min(1.0, 220.0 / Math.Max(spanX, spanY));
        double scaleZ = Math.Min(1.0, 100.0 / spanZ);

        return (
            Math.Clamp((spanX * 0.5) * scaleXY, 28.0, 120.0),
            Math.Clamp((spanY * 0.5) * scaleXY, 28.0, 120.0),
            Math.Clamp((spanZ * 0.5) * scaleZ, 20.0, 55.0)
        );
    }

    public static double MapPhysicalToVisual(double posMm, double minMm, double maxMm, double visHalf)
    {
        double span = Math.Max(1e-6, maxMm - minMm);
        double normalized = ((posMm - minMm) / span) * 2.0 - 1.0;
        return Math.Clamp(normalized, -1.03, 1.03) * visHalf;
    }

    public static double MapPhysicalToAngle(double posMm, double minMm, double maxMm, double halfAngleDeg)
    {
        double span = Math.Max(1e-6, maxMm - minMm);
        double normalized = ((posMm - minMm) / span) * 2.0 - 1.0;
        return Math.Clamp(normalized, -1.0, 1.0) * halfAngleDeg;
    }

    public static Point3D RotatePointAroundAxis(Point3D p, Point3D pivot, Vector3D axis, double angleDeg)
    {
        var m = Matrix3D.Identity;
        m.RotateAt(new Quaternion(axis, angleDeg), pivot);
        return m.Transform(p);
    }

    public static void UpdateLimitMaterial(DiffuseMaterial mat, bool triggered)
    {
        if (mat.Brush is SolidColorBrush brush)
        {
            brush.Color = triggered
                ? Color.FromRgb(255, 23, 68)   // Bright Red when limit switch is active
                : Color.FromRgb(45, 110, 65);  // Dim Green when normal
        }
    }

    public static GeometryModel3D CreateTrailModel(Color color) => new()
    {
        Geometry = new MeshGeometry3D(),
        Material = new EmissiveMaterial(new SolidColorBrush(color)),
        BackMaterial = new EmissiveMaterial(new SolidColorBrush(color))
    };

    public static void ClearMesh(GeometryModel3D model)
    {
        if (model.Geometry is MeshGeometry3D mesh)
        {
            mesh.Positions.Clear();
            mesh.TriangleIndices.Clear();
        }
    }

    public static void RebuildRibbonTrailMesh<T>(
        GeometryModel3D model,
        IReadOnlyList<T> samples,
        Func<T, Point3D> toWorld,
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

            AddQuad(mesh, p0Neg, p1Neg, p1Pos, p0Pos);

            int t1 = mesh.Positions.Count;
            mesh.Positions.Add(capCenterPos);
            mesh.Positions.Add(p0Pos);
            mesh.Positions.Add(p1Pos);
            mesh.TriangleIndices.Add(t1);
            mesh.TriangleIndices.Add(t1 + 1);
            mesh.TriangleIndices.Add(t1 + 2);

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

    public static void AddQuad(MeshGeometry3D mesh, Point3D a, Point3D b, Point3D c, Point3D d)
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
