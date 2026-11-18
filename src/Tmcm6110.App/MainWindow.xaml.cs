using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
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

    public MainWindow()
    {
        InitializeComponent();

        _viewModel = new MainViewModel();
        DataContext = _viewModel;

        _sceneBuilder = new Gantry3DSceneBuilder();
        MainViewport3D.Children.Add(new ModelVisual3D { Content = _sceneBuilder.RootGroup });

        UpdateOrbitCamera();

        // Hook into WPF CompositionTarget.Rendering (~60 FPS) to step physics and update 3D Gantry & Dashboard
        CompositionTarget.Rendering += OnCompositionTargetRendering;
    }

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

        // Default Axis Mapping: Axis 0 = Z, Axis 1 = Y, Axis 2 = X
        var snapZ = _viewModel.Controller.GetAxis(0).GetSnapshot();
        var snapY = _viewModel.Controller.GetAxis(1).GetSnapshot();
        var snapX = _viewModel.Controller.GetAxis(2).GetSnapshot();

        _viewModel.ActiveAxes[0].RefreshFromSnapshot(snapZ, stepsPerMm);
        _viewModel.ActiveAxes[1].RefreshFromSnapshot(snapY, stepsPerMm);
        _viewModel.ActiveAxes[2].RefreshFromSnapshot(snapX, stepsPerMm);

        double zMm = snapZ.ExactPosition / stepsPerMm;
        double yMm = snapY.ExactPosition / stepsPerMm;
        double xMm = snapX.ExactPosition / stepsPerMm;

        bool anyAlarm = snapZ.IsAlarmActive || snapY.IsAlarmActive || snapX.IsAlarmActive;

        _sceneBuilder.UpdateScene(
            zMm, yMm, xMm,
            snapZ.IsRightLimitTriggered, snapZ.IsLeftLimitTriggered,
            snapY.IsRightLimitTriggered, snapY.IsLeftLimitTriggered,
            snapX.IsRightLimitTriggered, snapX.IsLeftLimitTriggered,
            anyAlarm);

        string coordPrefix = LanguageManager.IsEnglish ? "TCP Coord:" : "TCP 座標:";
        Txt3DCoordOverlay.Text = $"{coordPrefix} X(Axis2)={xMm,7:F2} mm | Y(Axis1)={yMm,7:F2} mm | Z(Axis0)={zMm,7:F2} mm";
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
        // Cycle through 3D space waypoints to showcase 3-axis coordinated linear interpolation + Cmd 138 event
        (int z, int y, int x)[] waypoints =
        [
            (35000, 65000, 75000),
            (-25000, 65000, -75000),
            (20000, -65000, -75000),
            (-30000, -65000, 75000),
            (0, 0, 0)
        ];

        var wp = waypoints[_demoStepCounter % waypoints.Length];
        _demoStepCounter++;
        _viewModel.ExecuteDemo3DInterpolatedMove(wp.z, wp.y, wp.x);
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

    private void Window_Closed(object sender, EventArgs e)
    {
        CompositionTarget.Rendering -= OnCompositionTargetRendering;
        _viewModel.Dispose();
    }
}
