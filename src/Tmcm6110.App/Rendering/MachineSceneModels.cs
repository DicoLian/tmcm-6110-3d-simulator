using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Tmcm6110.App.Rendering;

public enum MachineSceneMode
{
    Gantry3Axis = 0,
    DualGantry6Axis = 1,
    GantryRotaryAux6Axis = 2,
    RobotArm6Dof = 3,
    LinearBench6Axis = 4
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
