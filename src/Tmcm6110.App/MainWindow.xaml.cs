using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Microsoft.Win32;
using Tmcm6110.App.Localization;
using Tmcm6110.App.Rendering;
using Tmcm6110.App.ViewModels;

namespace Tmcm6110.App;

public partial class MainWindow : Window
{
    private readonly MainViewModel _viewModel;
    private readonly Gantry3DSceneBuilder _sceneBuilder;
    private readonly Stopwatch _physicsClock = Stopwatch.StartNew();
    private double _lastTickSeconds;

    // 3D Orbit Camera Spherical Coordinates
    private double _camYawDeg = 42.0;
    private double _camPitchDeg = 28.0;
    private double _camDistance = 350.0;
    private bool _isDraggingOrbit;
    private Point _lastMousePos;
    private int _demoStepCounter;
    private readonly Axis3DFrame[] _frameBuffer = new Axis3DFrame[6];
    private readonly double[] _posMmBuffer = new double[6];

    public MainWindow()
    {
        InitializeComponent();

        _viewModel = new MainViewModel();
        DataContext = _viewModel;
        _viewModel.SceneModeChanged += OnViewModelSceneModeChanged;
        _viewModel.AxisMappingChanged += OnViewModelAxisMappingChanged;

        _sceneBuilder = new Gantry3DSceneBuilder();
        MainViewport3D.Children.Add(new ModelVisual3D { Content = _sceneBuilder.RootGroup });

        UpdateOrbitCamera();

        // Hook into WPF CompositionTarget.Rendering (~60 FPS) to step physics and update 3D Gantry & Dashboard
        CompositionTarget.Rendering += OnCompositionTargetRendering;
    }

    private void OnViewModelSceneModeChanged(MachineSceneMode mode) => _sceneBuilder.SetSceneMode(mode);

    private void OnViewModelAxisMappingChanged() => _sceneBuilder.ClearTrajectoryTrail();

    private void OnCompositionTargetRendering(object? sender, EventArgs e)
    {
        double nowSec = _physicsClock.Elapsed.TotalSeconds;
        double dt = nowSec - _lastTickSeconds;
        _lastTickSeconds = nowSec;

        if (dt <= 0 || dt > 0.1)
            dt = 1.0 / 60.0;

        // Sub-step physics 4 times per frame (~240 Hz) for smooth trapezoidal ramping and limit detection
        const int subSteps = 4;
        double subDt = dt / subSteps;
        for (int i = 0; i < subSteps; i++)
        {
            _viewModel.Controller.StepSimulation(subDt);
        }

        double stepsPerMm = Math.Max(10.0, _viewModel.StepsPerMm);

        for (int motorIdx = 0; motorIdx < 6; motorIdx++)
        {
            var snap = _viewModel.Controller.GetAxis(motorIdx).GetSnapshot();
            var axisVm = _viewModel.AllAxes[motorIdx];
            axisVm.RefreshFromSnapshot(snap, stepsPerMm);

            int roleSlot = axisVm.AssignedRoleSlot;
            double logicalPosMm = snap.ExactPosition / stepsPerMm;
            double physicalPosMm = snap.PhysicalPosition / stepsPerMm;
            _posMmBuffer[roleSlot] = logicalPosMm;
            _frameBuffer[roleSlot] = new Axis3DFrame(
                physicalPosMm,
                snap.VirtualMinStrokeSteps / stepsPerMm,
                snap.VirtualMaxStrokeSteps / stepsPerMm,
                snap.IsRightLimitTriggered,
                snap.IsLeftLimitTriggered,
                snap.IsAlarmActive);
        }

        _sceneBuilder.UpdateScene(_frameBuffer.AsSpan(0, _viewModel.SelectedAxisCount));

        string coordPrefix = LanguageManager.IsEnglish ? "Coord:" : "座標:";
        Txt3DCoordOverlay.Text = _viewModel.SelectedSceneMode switch
        {
            MachineSceneMode.LinearStage1Axis =>
                $"{coordPrefix} X(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],7:F2} mm",
            MachineSceneMode.RotaryTable1Axis =>
                $"{coordPrefix} R(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],7:F2} mm",
            MachineSceneMode.XyStage2Axis =>
                $"{coordPrefix} X(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],7:F2} mm | Y(M{_viewModel.GetMotorIndexForRole(1)})={_posMmBuffer[1],7:F2} mm",
            MachineSceneMode.XzGantry2Axis =>
                $"{coordPrefix} X(M{_viewModel.GetMotorIndexForRole(1)})={_posMmBuffer[1],7:F2} mm | Z(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],7:F2} mm",
            MachineSceneMode.XyRotary3Axis =>
                $"{coordPrefix} X(M{_viewModel.GetMotorIndexForRole(2)})={_posMmBuffer[2],6:F1} | Y(M{_viewModel.GetMotorIndexForRole(1)})={_posMmBuffer[1],6:F1} | R(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],5:F1} mm",
            MachineSceneMode.GantryRotary4Axis =>
                $"X(M{_viewModel.GetMotorIndexForRole(2)})={_posMmBuffer[2],6:F1} Y(M{_viewModel.GetMotorIndexForRole(1)})={_posMmBuffer[1],6:F1} Z(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],5:F1} | R(M{_viewModel.GetMotorIndexForRole(3)})={_posMmBuffer[3],5:F1}",
            MachineSceneMode.GantryDualRotary5Axis =>
                $"X={_posMmBuffer[2],6:F1} Y={_posMmBuffer[1],6:F1} Z={_posMmBuffer[0],5:F1} | R={_posMmBuffer[3],5:F1} A={_posMmBuffer[4],5:F1}",
            MachineSceneMode.DualGantry6Axis =>
                $"S1(X={_posMmBuffer[2],6:F1}, Y={_posMmBuffer[1],6:F1}, Z={_posMmBuffer[0],5:F1}) | S2(X={_posMmBuffer[5],6:F1}, Y={_posMmBuffer[4],6:F1}, Z={_posMmBuffer[3],5:F1}) mm",
            MachineSceneMode.GantryRotaryAux6Axis =>
                $"X={_posMmBuffer[2],6:F1} Y={_posMmBuffer[1],6:F1} Z={_posMmBuffer[0],5:F1} | R={_posMmBuffer[3],5:F1} A={_posMmBuffer[4],5:F1} B={_posMmBuffer[5],5:F1}",
            MachineSceneMode.RobotArm6Dof =>
                $"J1={_posMmBuffer[0],5:F1} J2={_posMmBuffer[1],5:F1} J3={_posMmBuffer[2],5:F1} J4={_posMmBuffer[3],5:F1} J5={_posMmBuffer[4],5:F1} J6={_posMmBuffer[5],5:F1}",
            MachineSceneMode.LinearBench6Axis =>
                FormatLinearBenchCoord(_viewModel.SelectedAxisCount),
            _ =>
                $"{coordPrefix} X(M{_viewModel.GetMotorIndexForRole(2)})={_posMmBuffer[2],7:F2} mm | Y(M{_viewModel.GetMotorIndexForRole(1)})={_posMmBuffer[1],7:F2} mm | Z(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],7:F2} mm"
        };
    }

    private string FormatLinearBenchCoord(int axisCount)
    {
        return axisCount switch
        {
            1 => $"Ch0(M{_viewModel.GetMotorIndexForRole(0)})={_posMmBuffer[0],6:F1} mm",
            2 => $"Ch0={_posMmBuffer[0],5:F1} Ch1={_posMmBuffer[1],5:F1} mm",
            3 => $"Ch0={_posMmBuffer[0],5:F1} Ch1={_posMmBuffer[1],5:F1} Ch2={_posMmBuffer[2],5:F1} mm",
            4 => $"Ch0={_posMmBuffer[0],5:F1} Ch1={_posMmBuffer[1],5:F1} Ch2={_posMmBuffer[2],5:F1} Ch3={_posMmBuffer[3],5:F1} mm",
            5 => $"Ch0={_posMmBuffer[0],5:F1} Ch1={_posMmBuffer[1],5:F1} Ch2={_posMmBuffer[2],5:F1} Ch3={_posMmBuffer[3],5:F1} Ch4={_posMmBuffer[4],5:F1} mm",
            _ => $"Ch0={_posMmBuffer[0],5:F1} Ch1={_posMmBuffer[1],5:F1} Ch2={_posMmBuffer[2],5:F1} Ch3={_posMmBuffer[3],5:F1} Ch4={_posMmBuffer[4],5:F1} Ch5={_posMmBuffer[5],5:F1} mm"
        };
    }

    // ==================== 3D ORBIT CAMERA MOUSE HANDLERS ====================
    private void Viewport3D_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (e.LeftButton == MouseButtonState.Pressed || e.RightButton == MouseButtonState.Pressed)
        {
            _isDraggingOrbit = true;
            _lastMousePos = e.GetPosition(this);
            Mouse.Capture((IInputElement)sender);
        }
    }

    private void Viewport3D_MouseMove(object sender, MouseEventArgs e)
    {
        if (!_isDraggingOrbit) return;

        Point currentPos = e.GetPosition(this);
        double dx = currentPos.X - _lastMousePos.X;
        double dy = currentPos.Y - _lastMousePos.Y;
        _lastMousePos = currentPos;

        _camYawDeg -= dx * 0.45;
        _camPitchDeg = Math.Clamp(_camPitchDeg + dy * 0.45, -10.0, 85.0);
        UpdateOrbitCamera();
    }

    private void Viewport3D_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (_isDraggingOrbit)
        {
            _isDraggingOrbit = false;
            Mouse.Capture(null);
        }
    }

    private void Viewport3D_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        _camDistance = Math.Clamp(_camDistance - e.Delta * 0.18, 120.0, 750.0);
        UpdateOrbitCamera();
    }

    private void UpdateOrbitCamera()
    {
        double yawRad = _camYawDeg * Math.PI / 180.0;
        double pitchRad = _camPitchDeg * Math.PI / 180.0;

        double cosPitch = Math.Cos(pitchRad);
        double camX = _camDistance * cosPitch * Math.Sin(yawRad);
        double camY = _camDistance * Math.Sin(pitchRad);
        double camZ = _camDistance * cosPitch * Math.Cos(yawRad);

        Point3D targetCenter = new(0, 0, 0);
        Point3D camPos = new(camX, camY, camZ);
        SceneCamera.Position = camPos;
        SceneCamera.LookDirection = targetCenter - camPos;
    }

    private void BtnResetCamera_Click(object sender, RoutedEventArgs e)
    {
        _camYawDeg = 42.0;
        _camPitchDeg = 28.0;
        _camDistance = 350.0;
        UpdateOrbitCamera();
    }

    private void BtnClearTrail_Click(object sender, RoutedEventArgs e)
    {
        _sceneBuilder.ClearTrajectoryTrail();
    }

    // ==================== TOP BAR BUTTON HANDLERS ====================
    private void BtnRefreshPorts_Click(object sender, RoutedEventArgs e) => _viewModel.RefreshComPorts();

    private void BtnToggleSerial_Click(object sender, RoutedEventArgs e) => _viewModel.ToggleSerialConnection();

    private void BtnToggleLanguage_Click(object sender, RoutedEventArgs e) => LanguageManager.ToggleLanguage();

    private void BtnLoadConfig_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = LanguageManager.Get(
                "選擇 TMCM-6110 設定檔 (.ini / .tmc / 軸定義.txt)",
                "Select TMCM-6110 Config File (.ini / .tmc / Axis Map .txt)"),
            Filter = LanguageManager.Get(
                "TMCM-6110 設定檔 (*.ini;*.tmc;*.txt)|*.ini;*.tmc;*.txt|所有檔案 (*.*)|*.*",
                "TMCM-6110 Config Files (*.ini;*.tmc;*.txt)|*.ini;*.tmc;*.txt|All Files (*.*)|*.*"),
            InitialDirectory = AppDomain.CurrentDomain.BaseDirectory
        };

        if (dlg.ShowDialog() == true)
        {
            _viewModel.LoadConfigFileFromDialog(dlg.FileName);
        }
    }

    private void BtnDemoInterpolation_Click(object sender, RoutedEventArgs e)
    {
        _viewModel.ExecuteDemoInterpolatedStep(_demoStepCounter++);
    }

    private void BtnHomeAll_Click(object sender, RoutedEventArgs e) => _viewModel.HomeAllAxes();

    private void BtnStopAll_Click(object sender, RoutedEventArgs e) => _viewModel.StopAllAxes();

    private void BtnClearAllAlarms_Click(object sender, RoutedEventArgs e) => _viewModel.ClearAllAlarms();

    private void BtnClearLogs_Click(object sender, RoutedEventArgs e) => _viewModel.PacketLogs.Clear();

    // ==================== PER-AXIS CARD BUTTON HANDLERS ====================
    private void BtnAxisMoveAbs_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandMoveAbsolute();
    }

    private void BtnAxisMoveRelPos_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandMoveRelative(+1);
    }

    private void BtnAxisMoveRelNeg_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandMoveRelative(-1);
    }

    private void BtnAxisJogLeft_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandJogLeft();
    }

    private void BtnAxisJogRight_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandJogRight();
    }

    private void BtnAxisStop_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandStop();
    }

    private void BtnAxisHome_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandStartHoming();
    }

    private void BtnAxisZero_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.CommandZeroPosition();
    }

    private void BtnAxisToggleAlarm_Click(object sender, RoutedEventArgs e)
    {
        if (sender is Button { Tag: AxisViewModel vm }) vm.ToggleAlarmInjection();
    }

    private void StrokeLimitTextBox_KeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter && sender is TextBox tb)
        {
            tb.GetBindingExpression(TextBox.TextProperty)?.UpdateSource();
            Keyboard.ClearFocus();
            e.Handled = true;
        }
    }

    private void BtnMappingDefaultZyx_Click(object sender, RoutedEventArgs e) => _viewModel.ResetDefaultAxisMapping();

    private void BtnMappingStandardXyz_Click(object sender, RoutedEventArgs e) => _viewModel.ApplyStandardXyzMapping();

    private void Window_Closed(object sender, EventArgs e)
    {
        CompositionTarget.Rendering -= OnCompositionTargetRendering;
        _viewModel.SceneModeChanged -= OnViewModelSceneModeChanged;
        _viewModel.AxisMappingChanged -= OnViewModelAxisMappingChanged;
        _viewModel.Dispose();
    }
}
