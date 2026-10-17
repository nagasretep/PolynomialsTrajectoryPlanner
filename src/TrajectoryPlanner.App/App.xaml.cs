using System.Windows;

namespace TrajectoryPlanner.App;

/// <summary>
/// Interaction logic for App.xaml
/// </summary>
public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        // Before any window exists: bindings must format numbers like the rest of the App (current culture).
        UiCulture.UseCurrentCultureForBindings();
        base.OnStartup(e);
    }
}
