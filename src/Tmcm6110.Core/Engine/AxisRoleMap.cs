namespace Tmcm6110.Core.Engine;

/// <summary>
/// Maintains a 1-to-1 bijective mapping between physical TMCM-6110 motor channels (0..5)
/// and logical 3D machine axis role slots (e.g., 0=Z, 1=Y, 2=X in 3-axis gantry mode).
/// </summary>
public sealed class AxisRoleMap
{
    private readonly int[] _motorToRole = { 0, 1, 2, 3, 4, 5 };
    private readonly int[] _roleToMotor = { 0, 1, 2, 3, 4, 5 };

    public int GetRoleForMotor(int motorIndex)
    {
        if (motorIndex < 0 || motorIndex >= Tmcm6110Controller.TotalAxes)
            throw new ArgumentOutOfRangeException(nameof(motorIndex));
        return _motorToRole[motorIndex];
    }

    public int GetMotorForRole(int roleSlot)
    {
        if (roleSlot < 0 || roleSlot >= Tmcm6110Controller.TotalAxes)
            throw new ArgumentOutOfRangeException(nameof(roleSlot));
        return _roleToMotor[roleSlot];
    }

    /// <summary>
    /// Assigns <paramref name="targetRoleSlot"/> to <paramref name="motorIndex"/> and swaps
    /// the previous owner of <paramref name="targetRoleSlot"/> onto <paramref name="motorIndex"/>'s old role.
    /// </summary>
    public bool TrySwapMotorRole(
        int motorIndex,
        int targetRoleSlot,
        int activeAxisCount,
        out int swappedMotorIndex,
        out int previousRoleSlot)
    {
        swappedMotorIndex = motorIndex;
        previousRoleSlot = 0;

        if (activeAxisCount < 1 || activeAxisCount > Tmcm6110Controller.TotalAxes)
            return false;
        if (motorIndex < 0 || motorIndex >= Tmcm6110Controller.TotalAxes)
            return false;
        if (targetRoleSlot < 0 || targetRoleSlot >= activeAxisCount)
            return false;

        int oldRole = _motorToRole[motorIndex];
        previousRoleSlot = oldRole;
        if (oldRole < 0 || oldRole >= activeAxisCount)
            return false;
        if (oldRole == targetRoleSlot)
            return false;

        int otherMotor = _roleToMotor[targetRoleSlot];
        _motorToRole[motorIndex] = targetRoleSlot;
        _motorToRole[otherMotor] = oldRole;
        _roleToMotor[targetRoleSlot] = motorIndex;
        _roleToMotor[oldRole] = otherMotor;

        swappedMotorIndex = otherMotor;
        return true;
    }

    public void ResetToIdentity()
    {
        for (int i = 0; i < Tmcm6110Controller.TotalAxes; i++)
        {
            _motorToRole[i] = i;
            _roleToMotor[i] = i;
        }
    }

    public void ApplyOrderForFirstThree(int roleForMotor0, int roleForMotor1, int roleForMotor2)
    {
        if (roleForMotor0 is < 0 or >= 3 ||
            roleForMotor1 is < 0 or >= 3 ||
            roleForMotor2 is < 0 or >= 3 ||
            roleForMotor0 == roleForMotor1 ||
            roleForMotor0 == roleForMotor2 ||
            roleForMotor1 == roleForMotor2)
        {
            throw new ArgumentException("First three role slots must be a permutation of {0, 1, 2}.");
        }

        int[] upperRoles = new int[3];
        int upperCount = 0;
        for (int m = 3; m < Tmcm6110Controller.TotalAxes; m++)
        {
            int r = _motorToRole[m];
            if (r >= 3 && r < Tmcm6110Controller.TotalAxes)
                upperRoles[upperCount++] = r;
        }

        if (upperCount != 3)
        {
            upperRoles[0] = 3;
            upperRoles[1] = 4;
            upperRoles[2] = 5;
        }

        _motorToRole[0] = roleForMotor0;
        _motorToRole[1] = roleForMotor1;
        _motorToRole[2] = roleForMotor2;
        _motorToRole[3] = upperRoles[0];
        _motorToRole[4] = upperRoles[1];
        _motorToRole[5] = upperRoles[2];

        for (int m = 0; m < Tmcm6110Controller.TotalAxes; m++)
            _roleToMotor[_motorToRole[m]] = m;
    }

    /// <summary>
    /// Ensures motors 0..(activeAxisCount-1) map cleanly to roles 0..(activeAxisCount-1) when shrinking from 6 to 3 axes.
    /// Preserves valid 3-axis permutations (such as X-Y-Z) untouched and returns false when no repair was needed.
    /// </summary>
    public bool NormalizeForActiveCount(int activeAxisCount)
    {
        if (activeAxisCount >= Tmcm6110Controller.TotalAxes)
            return false;

        bool isValid = true;
        bool[] seen = new bool[activeAxisCount];
        for (int m = 0; m < activeAxisCount; m++)
        {
            int r = _motorToRole[m];
            if (r < 0 || r >= activeAxisCount || seen[r])
            {
                isValid = false;
                break;
            }
            seen[r] = true;
        }

        if (isValid)
            return false;

        bool[] used = new bool[activeAxisCount];
        for (int m = 0; m < activeAxisCount; m++)
        {
            int r = _motorToRole[m];
            if (r >= 0 && r < activeAxisCount && !used[r])
            {
                used[r] = true;
            }
            else
            {
                _motorToRole[m] = -1;
            }
        }

        int nextRole = 0;
        for (int m = 0; m < activeAxisCount; m++)
        {
            if (_motorToRole[m] == -1)
            {
                while (used[nextRole]) nextRole++;
                _motorToRole[m] = nextRole;
                used[nextRole] = true;
            }
        }

        for (int m = activeAxisCount; m < Tmcm6110Controller.TotalAxes; m++)
            _motorToRole[m] = m;

        for (int m = 0; m < Tmcm6110Controller.TotalAxes; m++)
            _roleToMotor[_motorToRole[m]] = m;

        return true;
    }

    public bool TrySwapMotorChannels(int motorA, int motorB)
    {
        if (motorA < 0 || motorA >= Tmcm6110Controller.TotalAxes)
            return false;
        if (motorB < 0 || motorB >= Tmcm6110Controller.TotalAxes)
            return false;
        if (motorA == motorB)
            return false;

        int roleA = _motorToRole[motorA];
        int roleB = _motorToRole[motorB];

        _motorToRole[motorA] = roleB;
        _motorToRole[motorB] = roleA;
        _roleToMotor[roleB] = motorA;
        _roleToMotor[roleA] = motorB;
        return true;
    }

    public bool TryApplyFromXyzLabels(string label0, string label1, string label2)
    {
        static int ParseXyzRole(string label) => label.Trim().ToUpperInvariant() switch
        {
            "Z" => 0,
            "Y" => 1,
            "X" => 2,
            _ => -1
        };

        int r0 = ParseXyzRole(label0);
        int r1 = ParseXyzRole(label1);
        int r2 = ParseXyzRole(label2);

        if (r0 < 0 || r1 < 0 || r2 < 0 || r0 == r1 || r0 == r2 || r1 == r2)
            return false;

        ApplyOrderForFirstThree(r0, r1, r2);
        return true;
    }
}
