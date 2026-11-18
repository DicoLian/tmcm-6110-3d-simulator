using System.Windows;
using Tmcm6110.App.Localization;

namespace Tmcm6110.App;

public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);
        LanguageManager.InitializeFromSystemCulture();
    }
}
