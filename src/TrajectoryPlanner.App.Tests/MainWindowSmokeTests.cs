using System.Windows;

namespace TrajectoryPlanner.App.Tests;

/// <summary>
/// Request 0013: loads the real window once (XAML parsed, bindings created) and switches mode, so a broken
/// resource or binding shows up here and not at the author's first start. It does not click or compute.
/// </summary>
public class MainWindowSmokeTests
{
    [Fact]
    public void TheWindowLoads_AndSwitchesBetweenTheModes_KeepingBothInputs()
    {
        Exception? failure = null;
        var thread = new Thread(() =>
        {
            try
            {
                if (Application.Current is null)
                    _ = new Application { ShutdownMode = ShutdownMode.OnExplicitShutdown };

                var window = new MainWindow();
                window.PassThroughInput.Waypoints[1].Position = "5";
                window.Segments[0].EndPosition = "7";

                window.IsPassThroughMode = true;
                Assert.False(window.IsBlendingMode);
                window.IsBlendingMode = true;
                Assert.True(window.IsBlendingMode);
                window.IsPassThroughMode = true;

                Assert.Equal("5", window.PassThroughInput.Waypoints[1].Position);
                Assert.Equal("7", window.Segments[0].EndPosition);
                window.Close();
            }
            catch (Exception ex)
            {
                failure = ex;
            }
        });
        thread.SetApartmentState(ApartmentState.STA);
        thread.Start();
        thread.Join();

        Assert.Null(failure);
    }
}
