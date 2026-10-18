using System.IO;
using System.Windows;

namespace TrajectoryPlanner.App.Tests;

/// <summary>
/// Request 0013: loads the real window once (XAML parsed, bindings created) and switches mode, so a broken
/// resource or binding shows up here and not at the author's first start. It does not click or compute.
/// </summary>
public class MainWindowSmokeTests
{
    /// <summary>Request 0021: Save then Open in the real window brings the inputs and the mode back; a refused file changes nothing.</summary>
    [Fact]
    public void SaveAndOpen_RestoreTheInputsOfAllModes_AndARefusedFileChangesNothing()
    {
        Exception? failure = null;
        string path = Path.Combine(Path.GetTempPath(), $"ptp-0021-{Guid.NewGuid():N}.ptp.json");
        string badPath = Path.Combine(Path.GetTempPath(), $"ptp-0021-{Guid.NewGuid():N}-bad.ptp.json");
        var thread = new Thread(() =>
        {
            try
            {
                if (Application.Current is null)
                    _ = new Application { ShutdownMode = ShutdownMode.OnExplicitShutdown };

                var window = new MainWindow();
                var errors = new List<string>();
                window.ProjectErrorHandler = errors.Add;

                window.Segments[0].EndPosition = "7";
                window.PassThroughInput.Waypoints[1].Position = "5";
                window.Segments3D[0].EndPosition.Z = "2";
                window.PassThrough3DInput.Waypoints[2].Y = "9";
                window.IsThreeDMode = true;

                Assert.True(window.SaveProjectTo(path));
                Assert.EndsWith(Path.GetFileName(path), window.Title);

                // Change everything and the mode, then open the file: the inputs and the mode come back; nothing is computed.
                window.Segments[0].EndPosition = "70";
                window.PassThroughInput.Waypoints[1].Position = "50";
                window.Segments3D[0].EndPosition.Z = "20";
                window.PassThrough3DInput.Waypoints[2].Y = "90";
                window.IsPassThroughMode = true;

                Assert.True(window.OpenProjectFrom(path));
                Assert.True(window.IsThreeDMode);
                Assert.Equal("7", window.Segments[0].EndPosition);
                Assert.Equal("5", window.PassThroughInput.Waypoints[1].Position);
                Assert.Equal("2", window.Segments3D[0].EndPosition.Z);
                Assert.Equal("9", window.PassThrough3DInput.Waypoints[2].Y);
                Assert.Empty(window.PartRows3D);
                Assert.Empty(errors);

                // A file edited by hand with a wrong value is refused with a message, the inputs stay.
                File.WriteAllText(badPath, File.ReadAllText(path).Replace("\"version\": 1", "\"version\": 7"));
                window.Segments[0].EndPosition = "11";
                Assert.False(window.OpenProjectFrom(badPath));
                Assert.Single(errors);
                Assert.Contains("version 7", errors[0]);
                Assert.Contains(Path.GetFileName(badPath), window.StatusMessage);
                Assert.Equal("11", window.Segments[0].EndPosition);
                Assert.False(window.OpenProjectFrom(badPath + ".missing"));
                Assert.Equal(2, errors.Count);
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
        File.Delete(path);
        File.Delete(badPath);

        Assert.Null(failure);
    }

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

                // Request 0018: the third mode keeps its own data, computes and draws (time plots, path, cursor).
                window.Segments3D[0].EndPosition.Z = "2";
                window.IsThreeDMode = true;
                Assert.True(window.IsThreeDMode);
                Assert.False(window.IsOneDMode);
                var compute = typeof(MainWindow).GetMethod("Compute_Click",
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!;
                compute.Invoke(window, new object?[] { null, new RoutedEventArgs() });
                Assert.StartsWith("Computed 5 plotted quantities", window.StatusMessage);
                Assert.Single(window.PartRows3D);
                Assert.Equal(5, window.Cursor3DRows.Count);
                window.ShowNorm = true; // redraws
                typeof(MainWindow).GetMethod("UpdateCursorReadout",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!
                    .Invoke(window, new object[] { 0.5 });
                Assert.Equal(1.0, window.Cursor3DRows[0].Z, 1e-9); // rest-to-rest 0 -> 2 in 1 s: halfway (z = 1) at t = 0.5

                window.IsBlendingMode = true;
                Assert.Equal("7", window.Segments[0].EndPosition);
                window.IsThreeDMode = true;
                Assert.Equal("2", window.Segments3D[0].EndPosition.Z);
                Assert.Single(window.PartRows3D); // the last 3D result is shown again

                // Request 0020: the fourth mode (default inputs = the reference example of the theory, with its limits).
                window.IsThreeDPassThroughMode = true;
                Assert.True(window.IsAny3DMode);
                Assert.True(window.IsWaypointMode);
                var input = window.PassThrough3DInput;
                input.UseMaxVelocity = input.UseMaxAcceleration = input.UseMaxJerk = input.UseMaxSnap = true;
                compute.Invoke(window, new object?[] { null, new RoutedEventArgs() });
                Assert.StartsWith("Computed 5 plotted quantities across 14.9715", window.StatusMessage.Replace(',', '.'));
                Assert.Contains("binding: velocity on axis x", window.StatusMessage);
                Assert.Equal(3, window.PartRows3D.Count);
                Assert.Equal(20, window.WaypointStateRows3D.Count);
                typeof(MainWindow).GetMethod("UpdateCursorReadout",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!
                    .Invoke(window, new object[] { 3.0 });

                // Round trip over the four modes keeps every result and input.
                window.IsThreeDMode = true;
                Assert.Single(window.PartRows3D);
                window.IsPassThroughMode = true;
                Assert.Equal("5", window.PassThroughInput.Waypoints[1].Position);
                window.IsThreeDPassThroughMode = true;
                Assert.Equal(3, window.PartRows3D.Count);
                Assert.Equal("3", input.Waypoints[2].Y);
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
