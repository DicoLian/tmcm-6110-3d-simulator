using Tmcm6110.Core.Config;
using Tmcm6110.Core.Engine;
using Tmcm6110.Core.Protocol;
using Xunit;

namespace Tmcm6110.Core.Tests;

public sealed class TmcmConfigLoaderTests : IDisposable
{
    private readonly string _tempDir;

    public TmcmConfigLoaderTests()
    {
        _tempDir = Path.Combine(Path.GetTempPath(), $"tmcm6110_tests_{Guid.NewGuid():N}");
        Directory.CreateDirectory(_tempDir);
    }

    public void Dispose()
    {
        if (Directory.Exists(_tempDir))
        {
            Directory.Delete(_tempDir, recursive: true);
        }
    }

    [Fact]
    public void LoadFromIniFile_AppliesAxisAndGlobalParameters_AndStoresAxisToEeprom()
    {
        string iniPath = Path.Combine(_tempDir, "test_gantry.ini");
        File.WriteAllText(iniPath, """
            ; Comment line
            # Another comment
            [module_settings_axis%0]
            Maximum_positioning_speed = 1400
            Maximum_acceleration = 750
            Microstep_Resolution = 5
            Unknown_Key = 999
            Bad_Value = not_an_int

            [module_settings_axis%02]
            Reference_search_mode = 65
            Right_limit_switch_disable = 0

            [global_settings]
            Serial_address = 3
            Serial_host_address = 4
            Limit_switch_polarity = 0

            [unrelated_section]
            Maximum_positioning_speed = 111
            """);

        var controller = new Tmcm6110Controller();
        int applied = TmcmConfigLoader.LoadFromIniFile(iniPath, controller);

        Assert.Equal(8, applied);
        Assert.Equal(1400, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed));
        Assert.Equal(750, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxAcceleration));
        Assert.Equal(5, controller.GetAxis(0).GetParameter(TmclAxisParam.MicrostepResolution));
        Assert.Equal(65, controller.GetAxis(2).GetParameter(TmclAxisParam.ReferenceSearchMode));
        Assert.Equal(0, controller.GetAxis(2).GetParameter(TmclAxisParam.RightLimitSwitchDisable));
        Assert.Equal(3, controller.ModuleAddress);
        Assert.Equal(4, controller.HostAddress);
        Assert.Equal(0, controller.GetGlobalParameter(0, 79));

        // Verify axis values were persisted to simulated EEPROM
        controller.GetAxis(0).SetParameter(TmclAxisParam.MaxPositioningSpeed, 999);
        controller.GetAxis(0).RestoreParameterFromEeprom(TmclAxisParam.MaxPositioningSpeed);
        Assert.Equal(1400, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed));
    }

    [Fact]
    public void LoadFromIniFile_ThrowsFileNotFoundException_WhenMissing()
    {
        var controller = new Tmcm6110Controller();
        string missingPath = Path.Combine(_tempDir, "missing.ini");

        Assert.Throws<FileNotFoundException>(() => TmcmConfigLoader.LoadFromIniFile(missingPath, controller));
    }

    [Fact]
    public void LoadFromTmcFile_AppliesSapAndSgpCommands_AndPreservesNonZeroAddresses()
    {
        string tmcPath = Path.Combine(_tempDir, "test_script.tmc");
        File.WriteAllText(tmcPath, """
            // Sample TMCL parameter script
            SAP 4, 0, 1600        // Axis 0 MaxPositioningSpeed
            sap 5, 1, 900         // Axis 1 MaxAcceleration (case-insensitive)
            SAP 174, 2, -3        // Negative StallGuard2 threshold
            SGP 66, 0, 0          // Must be ignored to protect ModuleAddress
            SGP 76, 0, 0          // Must be ignored to protect HostAddress
            SGP 79, 0, 0          // Limit switch polarity
            SGP 12, 2, -250       // Bank 2 user variable
            MVP ABS, 0, 100       // Non SAP/SGP line ignored
            """);

        var controller = new Tmcm6110Controller();
        int applied = TmcmConfigLoader.LoadFromTmcFile(tmcPath, controller);

        Assert.Equal(5, applied);
        Assert.Equal(1600, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed));
        Assert.Equal(900, controller.GetAxis(1).GetParameter(TmclAxisParam.MaxAcceleration));
        Assert.Equal(-3, controller.GetAxis(2).GetParameter(TmclAxisParam.StallGuard2Threshold));
        Assert.Equal(1, controller.ModuleAddress);
        Assert.Equal(2, controller.HostAddress);
        Assert.Equal(0, controller.GetGlobalParameter(0, 79));
        Assert.Equal(-250, controller.GetGlobalParameter(2, 12));

        // Verify SAP stored to EEPROM
        controller.GetAxis(0).SetParameter(TmclAxisParam.MaxPositioningSpeed, 500);
        controller.GetAxis(0).RestoreParameterFromEeprom(TmclAxisParam.MaxPositioningSpeed);
        Assert.Equal(1600, controller.GetAxis(0).GetParameter(TmclAxisParam.MaxPositioningSpeed));
    }

    [Fact]
    public void LoadFromTmcFile_ThrowsFileNotFoundException_WhenMissing()
    {
        var controller = new Tmcm6110Controller();
        string missingPath = Path.Combine(_tempDir, "missing.tmc");

        Assert.Throws<FileNotFoundException>(() => TmcmConfigLoader.LoadFromTmcFile(missingPath, controller));
    }
}
