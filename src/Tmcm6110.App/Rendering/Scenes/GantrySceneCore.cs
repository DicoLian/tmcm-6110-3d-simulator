using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace Tmcm6110.App.Rendering.Scenes;

/// <summary>
/// Shared parametric XYZ gantry geometry builder used by 3-Axis Gantry, Dual Gantry, and Gantry + Rotary/Aux scenes.
/// </summary>
internal static class GantrySceneCore
{
    public static void BuildSingleGantry(
        Model3DGroup targetGroup,
        double centerX,
        double visHalfX,
        double visHalfY,
        double visHalfZ,
        int zIdx,
        int yIdx,
        int xIdx,
        double[] minMm,
        double[] maxMm,
        TranslateTransform3D[] axisTranslate,
        AxisAngleRotation3D[] axisRotate,
        DiffuseMaterial[] posLimitMats,
        DiffuseMaterial[] negLimitMats,
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
        double minX = minMm[xIdx];
        double maxX = maxMm[xIdx];
        double minY = minMm[yIdx];
        double maxY = maxMm[yIdx];

        var staticGroup = new Model3DGroup();

        if (includeBasePlate)
        {
            double extraRight = includeAuxSlide ? 34.0 : 0.0;
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(
                new Point3D(centerX + extraRight * 0.5, worktableTopY - 6.5, 0),
                2.0 * visHalfX + 60.0 + extraRight, 6, 2.0 * visHalfY + 60.0,
                Color.FromRgb(32, 38, 48)));
        }

        // Worktable surface
        staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(
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

            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(gx, worktableTopY + 0.3, 0), thickness, 0.3, 2.0 * visHalfY, gridCol));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, worktableTopY + 0.3, gy), 2.0 * visHalfX, 0.3, thickness, gridCol));
        }

        // Physical 0 mm crosshair lines + dynamic 3D Origin Triad (moves to Center or any of the 4 Corners)
        bool hasZeroX = minX <= 0.0 && maxX >= 0.0;
        bool hasZeroY = minY <= 0.0 && maxY >= 0.0;
        double zeroVisX = centerX + Mesh3DPrimitives.MapPhysicalToVisual(0.0, minX, maxX, visHalfX);
        double zeroVisY = Mesh3DPrimitives.MapPhysicalToVisual(0.0, minY, maxY, visHalfY);

        if (hasZeroX)
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroVisX, worktableTopY + 0.35, 0), 1.0, 0.35, 2.0 * visHalfY, Color.FromRgb(110, 130, 160)));
        if (hasZeroY)
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, worktableTopY + 0.35, zeroVisY), 2.0 * visHalfX, 0.35, 1.0, Color.FromRgb(110, 130, 160)));

        if (hasZeroX && hasZeroY)
        {
            double triadY = worktableTopY + 1.5;
            double triadLen = Math.Min(28.0, Math.Min(visHalfX, visHalfY) * 0.65);
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroVisX, triadY, zeroVisY), 5.0, 3.0, 5.0, Color.FromRgb(235, 248, 255)));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroVisX + triadLen * 0.5, triadY, zeroVisY), triadLen, 2.2, 2.2, Color.FromRgb(255, 82, 82)));   // +X Red
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroVisX, triadY, zeroVisY + triadLen * 0.5), 2.2, 2.2, triadLen, Color.FromRgb(105, 240, 174))); // +Y Green
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(zeroVisX, triadY + triadLen * 0.5, zeroVisY), 2.2, triadLen, 2.2, Color.FromRgb(68, 138, 255)));  // +Z Blue
        }

        // Fixed Y-Axis Side Rails
        staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX - railOffsetX, worktableTopY + 5.5, 0), 8, 10, 2.0 * visHalfY + 30.0, Color.FromRgb(60, 90, 70)));
        staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + railOffsetX, worktableTopY + 5.5, 0), 8, 10, 2.0 * visHalfY + 30.0, Color.FromRgb(60, 90, 70)));

        // Y-Axis Limit Switch Blocks
        staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX - railOffsetX, worktableTopY + 13.5, visHalfY + 12.0), 10, 6, 6, posLimitMats[yIdx]));
        staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX - railOffsetX, worktableTopY + 13.5, -(visHalfY + 12.0)), 10, 6, 6, negLimitMats[yIdx]));

        // Optional Axis 4 (A): Rotary Worktable mounted at center of worktable
        if (includeRotaryTable)
        {
            axisRotate[4].Axis = new Vector3D(0, 1, 0);
            double platterRadius = Math.Min(42.0, Math.Min(visHalfX, visHalfY) * 0.45);
            staticGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(centerX, worktableTopY + 1.0, 0), platterRadius + 4.0, 2.0, Color.FromRgb(60, 68, 80)));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX + 8.0, worktableTopY + 2.5, platterRadius + 4.0), 4, 4, 4, posLimitMats[4]));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX - 8.0, worktableTopY + 2.5, platterRadius + 4.0), 4, 4, 4, negLimitMats[4]));

            var rotaryTableGroup = new Model3DGroup
            {
                Transform = new RotateTransform3D(axisRotate[4], new Point3D(centerX, worktableTopY + 3.5, 0))
            };
            rotaryTableGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(centerX, worktableTopY + 3.5, 0), platterRadius, 3.0, Color.FromRgb(84, 110, 122), segments: 24));
            rotaryTableGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + platterRadius * 0.45, worktableTopY + 5.5, 0), platterRadius * 0.85, 1.5, 3.0, Color.FromRgb(255, 213, 79)));
            rotaryTableGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, worktableTopY + 6.5, platterRadius * 0.65), 4, 3, 4, Color.FromRgb(207, 216, 220)));
            rotaryTableGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, worktableTopY + 6.5, -platterRadius * 0.65), 4, 3, 4, Color.FromRgb(207, 216, 220)));
            targetGroup.Children.Add(rotaryTableGroup);
        }

        // Optional Axis 5 (B): Auxiliary Linear Feeder Slide beside right rail
        if (includeAuxSlide)
        {
            double auxX = centerX + railOffsetX + 26.0;
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(auxX, worktableTopY + 4.5, 0), 14, 6, 2.0 * visHalfY + 20.0, Color.FromRgb(63, 81, 181)));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(auxX + 9.0, worktableTopY + 9.0, visHalfY + 4.0), 4, 5, 5, posLimitMats[5]));
            staticGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(auxX + 9.0, worktableTopY + 9.0, -(visHalfY + 4.0)), 4, 5, 5, negLimitMats[5]));

            var auxCarriageGroup = new Model3DGroup { Transform = axisTranslate[5] };
            auxCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(auxX, worktableTopY + 12.0, 0), 18, 9, 22, Color.FromRgb(92, 107, 192)));
            auxCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(auxX, worktableTopY + 18.0, 0), 12, 4, 14, Color.FromRgb(159, 168, 218)));
            targetGroup.Children.Add(auxCarriageGroup);
        }

        targetGroup.Children.Add(staticGroup);

        // Moving Y-Axis Gantry Bridge
        var yBridgeGroup = new Model3DGroup { Transform = axisTranslate[yIdx] };
        double colBottom = worktableTopY + 10.5;
        double colTop = crossbeamY + 5.0;
        double colHeight = Math.Max(20.0, colTop - colBottom);
        double colCenterY = (colBottom + colTop) * 0.5;

        yBridgeGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX - railOffsetX, colCenterY, 0), 12, colHeight, 18, colClr));
        yBridgeGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + railOffsetX, colCenterY, 0), 12, colHeight, 18, colClr));
        yBridgeGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, crossbeamY, 0), 2.0 * visHalfX + 36.0, 14, 14, bmClr));

        // X-Axis Limit Switch Blocks
        yBridgeGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX + visHalfX + 4.0, crossbeamY + 9.0, 0), 6, 6, 16, posLimitMats[xIdx]));
        yBridgeGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX - (visHalfX + 4.0), crossbeamY + 9.0, 0), 6, 6, 16, negLimitMats[xIdx]));

        // Moving X-Axis Carriage
        var xCarriageGroup = new Model3DGroup { Transform = axisTranslate[xIdx] };
        xCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, crossbeamY, 8), 24, 22, 12, carClr));

        // Z-Axis Guide Rail & Limit Switch Blocks
        xCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + 14.0, crossbeamY, 12), 2.2, 2.0 * visHalfZ + 8.0, 4, Color.FromRgb(70, 82, 100)));
        xCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX + 14.0, crossbeamY + visHalfZ, 12), 5, 5, 8, posLimitMats[zIdx]));
        xCarriageGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX + 14.0, crossbeamY - visHalfZ, 12), 5, 5, 8, negLimitMats[zIdx]));

        // Moving Z-Axis Spindle & Tool Head
        var zSpindleGroup = new Model3DGroup { Transform = axisTranslate[zIdx] };
        double slideHeight = 2.0 * visHalfZ + 2.0;
        zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, visHalfZ + 1.0, 15), 14, slideHeight, 12, sldClr));
        zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + 9.0, crossbeamY, 12), 4.5, 4.0, 4.0, Color.FromRgb(180, 190, 205)));
        zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX, 8, 18), 10, 22, 10, Color.FromRgb(180, 190, 205)));

        if (includeRotarySpindle)
        {
            axisRotate[3].Axis = new Vector3D(0, 1, 0);
            zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX + 6.5, 3, 18), 3, 3, 4, posLimitMats[3]));
            zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX - 6.5, 3, 18), 3, 3, 4, negLimitMats[3]));

            var chuckGroup = new Model3DGroup
            {
                Transform = new RotateTransform3D(axisRotate[3], new Point3D(centerX, -4, 18))
            };
            chuckGroup.Children.Add(Mesh3DPrimitives.CreateCylinderModel(new Point3D(centerX, -3, 18), 4.8, 5.0, Color.FromRgb(255, 167, 38), segments: 12));
            chuckGroup.Children.Add(Mesh3DPrimitives.CreateBoxModel(new Point3D(centerX + 5.5, -3, 18), 6.0, 2.5, 2.0, Color.FromRgb(255, 213, 79)));
            chuckGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX, -6, 18), 4.0, 5.0, 4.0, tipMat));
            zSpindleGroup.Children.Add(chuckGroup);
        }
        else
        {
            zSpindleGroup.Children.Add(Mesh3DPrimitives.CreateBoxModelWithMaterial(new Point3D(centerX, -4, 18), 4.5, 8, 4.5, tipMat));
        }

        xCarriageGroup.Children.Add(zSpindleGroup);
        yBridgeGroup.Children.Add(xCarriageGroup);
        targetGroup.Children.Add(yBridgeGroup);
    }
}
