using System.IO;
using System.Windows;
using Microsoft.Win32;

namespace TrajectoryPlanner.App;

/// <summary>Save and Open (request 0021): the window only calls <see cref="ProjectSerializer"/> and shows the outcome.</summary>
public partial class MainWindow
{
    private const string BaseTitle = "Polynomial Trajectory Planner - UI First Prototype";
    private const string DialogFilter = "Project files (*.ptp.json)|*.ptp.json|JSON files (*.json)|*.json";

    private void SaveProject_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new SaveFileDialog
        {
            Filter = DialogFilter,
            DefaultExt = ProjectSerializer.FileExtension,
            AddExtension = true,
            FileName = "project" + ProjectSerializer.FileExtension
        };
        if (dialog.ShowDialog(this) == true)
            SaveProjectTo(dialog.FileName);
    }

    private void OpenProject_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new OpenFileDialog { Filter = DialogFilter, DefaultExt = ProjectSerializer.FileExtension };
        if (dialog.ShowDialog(this) == true)
            OpenProjectFrom(dialog.FileName);
    }

    /// <summary>Saves the inputs of the four modes and the selected mode. Returns false (status bar and message box) on failure.</summary>
    public bool SaveProjectTo(string path)
    {
        try
        {
            CommitPendingEdits();
            var project = new ProjectInputs { Mode = _mode, PassThrough = PassThroughInput, PassThrough3D = PassThrough3DInput };
            project.Segments.Clear();
            project.Segments.AddRange(Segments);
            project.Segments3D.Clear();
            project.Segments3D.AddRange(Segments3D);

            ProjectSerializer.Save(project, path);
            SetProjectName(path);
            StatusMessage = $"Saved {Path.GetFileName(path)}.";
            return true;
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            ReportProjectError($"{Path.GetFileName(path)}: the file cannot be written ({ex.Message}).");
            return false;
        }
    }

    /// <summary>
    /// Replaces the inputs of all four modes and selects the saved mode; nothing is computed. If the file is refused the current inputs
    /// stay as they are. Returns false (status bar and message box) on failure.
    /// </summary>
    public bool OpenProjectFrom(string path)
    {
        ProjectInputs project;
        try
        {
            project = ProjectSerializer.Load(path);
        }
        catch (ProjectFileException ex)
        {
            ReportProjectError(ex.Message);
            return false;
        }

        ProjectSerializer.ApplyTo(project, Segments, PassThroughInput, Segments3D, PassThrough3DInput);
        SelectedSegment = Segments[0];
        SelectedSegment3D = Segments3D[0];
        SelectedWaypoint = PassThroughInput.Waypoints[0];
        SelectedWaypoint3D = PassThrough3DInput.Waypoints[0];

        // The results belong to the old inputs: drop them; the author presses Compute.
        _blendingResult = _passThroughResult = null;
        _result3D = _result3DPassThrough = null;
        if (_mode == project.Mode)
        {
            ClearReports();
            ResetPlot();
        }
        else
        {
            SwitchMode(project.Mode);
        }

        SetProjectName(path);
        StatusMessage = $"Opened {Path.GetFileName(path)}; click Compute to see the result.";
        return true;
    }

    private void CommitPendingEdits()
    {
        WaypointGrid.CommitEdit(System.Windows.Controls.DataGridEditingUnit.Row, true);
        WaypointGrid3D.CommitEdit(System.Windows.Controls.DataGridEditingUnit.Row, true);
    }

    private void SetProjectName(string path) => Title = $"{BaseTitle} - {Path.GetFileName(path)}";

    /// <summary>Replaces the message box for a refused or failed project file (the smoke test sets it; null shows the box).</summary>
    public Action<string>? ProjectErrorHandler { get; set; }

    private void ReportProjectError(string message)
    {
        StatusMessage = message;
        if (ProjectErrorHandler is not null)
        {
            ProjectErrorHandler(message);
            return;
        }

        MessageBox.Show(this, message, "Project file", MessageBoxButton.OK, MessageBoxImage.Warning);
    }
}
