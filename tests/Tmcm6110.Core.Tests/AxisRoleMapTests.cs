using Tmcm6110.Core.Engine;
using Xunit;

namespace Tmcm6110.Core.Tests;

public class AxisRoleMapTests
{
    [Fact]
    public void DefaultMapping_IsIdentityAcrossAllSixAxes()
    {
        var map = new AxisRoleMap();
        for (int i = 0; i < Tmcm6110Controller.TotalAxes; i++)
        {
            Assert.Equal(i, map.GetRoleForMotor(i));
            Assert.Equal(i, map.GetMotorForRole(i));
        }
    }

    [Fact]
    public void TrySwapMotorRole_MaintainsBijectionAndSwapsPartnerMotor()
    {
        var map = new AxisRoleMap();

        // Assign Motor 0 (currently Z=0) to X (slot 2); Motor 2 should move to Z (slot 0)
        bool swapped = map.TrySwapMotorRole(0, 2, activeAxisCount: 3, out int partnerMotor, out int oldRole);

        Assert.True(swapped);
        Assert.Equal(2, partnerMotor);
        Assert.Equal(0, oldRole);
        Assert.Equal(2, map.GetRoleForMotor(0));
        Assert.Equal(1, map.GetRoleForMotor(1));
        Assert.Equal(0, map.GetRoleForMotor(2));
        Assert.Equal(2, map.GetMotorForRole(0));
        Assert.Equal(1, map.GetMotorForRole(1));
        Assert.Equal(0, map.GetMotorForRole(2));

        // Same-role selection is a no-op
        Assert.False(map.TrySwapMotorRole(0, 2, activeAxisCount: 3, out _, out _));

        // Out-of-active-count swap in 3-axis mode is rejected
        Assert.False(map.TrySwapMotorRole(0, 4, activeAxisCount: 3, out _, out _));
    }

    [Fact]
    public void ApplyOrderForFirstThree_AppliesStandardXyzWithoutBreakingUpperAxes()
    {
        var map = new AxisRoleMap();
        map.TrySwapMotorRole(3, 5, activeAxisCount: 6, out _, out _);

        // Apply M0=X(2), M1=Y(1), M2=Z(0)
        map.ApplyOrderForFirstThree(2, 1, 0);

        Assert.Equal(2, map.GetRoleForMotor(0));
        Assert.Equal(1, map.GetRoleForMotor(1));
        Assert.Equal(0, map.GetRoleForMotor(2));
        Assert.Equal(5, map.GetRoleForMotor(3));
        Assert.Equal(4, map.GetRoleForMotor(4));
        Assert.Equal(3, map.GetRoleForMotor(5));
    }

    [Fact]
    public void NormalizeForActiveCount_PreservesValid3AxisPermutation_AndRepairsCrossGroupSwap()
    {
        var map = new AxisRoleMap();
        map.ApplyOrderForFirstThree(2, 1, 0); // Valid 3-axis X-Y-Z

        Assert.False(map.NormalizeForActiveCount(3));
        Assert.Equal(2, map.GetRoleForMotor(0));
        Assert.Equal(1, map.GetRoleForMotor(1));
        Assert.Equal(0, map.GetRoleForMotor(2));

        // Cross-swap Motor 0 with Motor 4 in 6-axis mode, then normalize back to 3 axes
        Assert.True(map.TrySwapMotorRole(0, 4, activeAxisCount: 6, out _, out _));
        Assert.True(map.NormalizeForActiveCount(3));

        bool[] seen = new bool[3];
        for (int m = 0; m < 3; m++)
        {
            int role = map.GetRoleForMotor(m);
            Assert.InRange(role, 0, 2);
            Assert.False(seen[role]);
            seen[role] = true;
            Assert.Equal(m, map.GetMotorForRole(role));
        }

        for (int m = 3; m < 6; m++)
        {
            Assert.Equal(m, map.GetRoleForMotor(m));
            Assert.Equal(m, map.GetMotorForRole(m));
        }
    }

    [Fact]
    public void TryApplyFromXyzLabels_MapsCustomOrderAndRejectsInvalidLabels()
    {
        var map = new AxisRoleMap();

        Assert.True(map.TryApplyFromXyzLabels("x", "y", "z"));
        Assert.Equal(2, map.GetRoleForMotor(0));
        Assert.Equal(1, map.GetRoleForMotor(1));
        Assert.Equal(0, map.GetRoleForMotor(2));

        Assert.False(map.TryApplyFromXyzLabels("X", "X", "Z"));
        Assert.False(map.TryApplyFromXyzLabels("A", "B", "C"));
    }
}
