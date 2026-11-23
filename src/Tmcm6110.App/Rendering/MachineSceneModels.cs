using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Tmcm6110.App.Rendering;

public enum MachineSceneMode
{
    Gantry3Axis = 0,
    DualGantry6Axis = 1,
    GantryRotaryAux6Axis = 2,
    RobotArm6Dof = 3,
    LinearBench6Axis = 4,
    LinearStage1Axis = 5,
    RotaryTable1Axis = 6,
    XyStage2Axis = 7,
    XzGantry2Axis = 8,
    XyRotary3Axis = 9,
    GantryRotary4Axis = 10,
    GantryDualRotary5Axis = 11
}

public static class MachineSceneCatalog
{
    public static IReadOnlyList<MachineSceneMode> GetModesForAxisCount(int axisCount) => Math.Clamp(axisCount, 1, 6) switch
    {
        1 => new[] { MachineSceneMode.LinearStage1Axis, MachineSceneMode.RotaryTable1Axis, MachineSceneMode.LinearBench6Axis },
        2 => new[] { MachineSceneMode.XyStage2Axis, MachineSceneMode.XzGantry2Axis, MachineSceneMode.LinearBench6Axis },
        3 => new[] { MachineSceneMode.Gantry3Axis, MachineSceneMode.XyRotary3Axis, MachineSceneMode.LinearBench6Axis },
        4 => new[] { MachineSceneMode.GantryRotary4Axis, MachineSceneMode.LinearBench6Axis },
        5 => new[] { MachineSceneMode.GantryDualRotary5Axis, MachineSceneMode.LinearBench6Axis },
        _ => new[] { MachineSceneMode.DualGantry6Axis, MachineSceneMode.GantryRotaryAux6Axis, MachineSceneMode.RobotArm6Dof, MachineSceneMode.LinearBench6Axis }
    };

    public static MachineSceneMode GetDefaultModeForAxisCount(int axisCount) => GetModesForAxisCount(axisCount)[0];
}

public enum XyOriginPreset
{
    Center = 0,
    FrontLeft = 1,
    BackLeft = 2,
    FrontRight = 3,
    BackRight = 4
}

public enum ZOriginPreset
{
    Center = 0,
    Top = 1,
    Bottom = 2
}

public readonly record struct Axis3DFrame(
    double PositionMm,
    double MinMm,
    double MaxMm,
    bool IsRightLimit,
    bool IsLeftLimit,
    bool IsAlarm
);

public sealed class SelectableOption<T> : INotifyPropertyChanged
{
    private string _label;

    public T Value { get; }

    public string Label
    {
        get => _label;
        set
        {
            if (_label == value) return;
            _label = value;
            OnPropertyChanged();
        }
    }

    public SelectableOption(T value, string label)
    {
        Value = value;
        _label = label;
    }

    public event PropertyChangedEventHandler? PropertyChanged;
    private void OnPropertyChanged([CallerMemberName] string? name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
