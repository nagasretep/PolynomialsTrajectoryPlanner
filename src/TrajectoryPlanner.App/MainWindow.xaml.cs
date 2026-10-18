using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using ScottPlot;
using ScottPlot.Plottables;
using ScottPlot.WPF;

namespace TrajectoryPlanner.App;

public partial class MainWindow : Window, INotifyPropertyChanged
{
    private readonly PrototypeTrajectoryService _trajectoryService = new();
    private readonly List<WpfPlot> _plotControls = [];
    private readonly List<Border> _plotHosts = [];
    private PrototypeComputationResult? _currentResult;
    private PrototypeComputationResult? _blendingResult;
    private PrototypeComputationResult? _passThroughResult;
    private bool _passThroughMode;
    private WaypointInputModel? _selectedWaypoint;
    private readonly List<VerticalLine> _cursorLines = [];
    private SegmentInputModel? _selectedSegment;
    private string _statusMessage = "Configure the prototype inputs, then click Compute.";
    private string _cursorStatusText = "Cursor time: n/a";

    public MainWindow()
    {
        InitializeComponent();
        DataContext = this;

        _plotControls.AddRange([TrajectoryPlot1, TrajectoryPlot2, TrajectoryPlot3, TrajectoryPlot4, TrajectoryPlot5]);
        _plotHosts.AddRange([PlotHost1, PlotHost2, PlotHost3, PlotHost4, PlotHost5]);

        Segments.CollectionChanged += OnSegmentsCollectionChanged;
        PassThroughInput.Waypoints.CollectionChanged += (_, _) => OnPropertyChanged(nameof(CanRemoveWaypoint));
        AddInitialSegment();
        Loaded += (_, _) => ResetPlot();
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    public int[] SupportedDegrees { get; } = [3, 5, 7, 9];

    public ObservableCollection<SegmentInputModel> Segments { get; } = [];

    public ObservableCollection<ExtremaReportRow> ExtremaReportRows { get; } = [];

    public ObservableCollection<CoefficientReportRow> CoefficientReportRows { get; } = [];

    public ObservableCollection<CursorReadoutRow> CursorReadoutRows { get; } = [];

    /// <summary>Inputs of the pass-through mode; kept when the mode is switched.</summary>
    public PassThroughInputModel PassThroughInput { get; } = new();

    public ObservableCollection<SummaryRow> SummaryReportRows { get; } = [];

    public ObservableCollection<LimitPeakRow> LimitPeakRows { get; } = [];

    public bool HasLimitPeaks => LimitPeakRows.Count > 0;

    public ObservableCollection<WaypointStateRow> WaypointStateRows { get; } = [];

    public bool IsBlendingMode
    {
        get => !_passThroughMode;
        set { if (value) SwitchMode(passThrough: false); }
    }

    public bool IsPassThroughMode
    {
        get => _passThroughMode;
        set { if (value) SwitchMode(passThrough: true); }
    }

    public WaypointInputModel? SelectedWaypoint
    {
        get => _selectedWaypoint;
        set
        {
            if (SetProperty(ref _selectedWaypoint, value))
                OnPropertyChanged(nameof(CanRemoveWaypoint));
        }
    }

    public bool CanRemoveWaypoint => PassThroughInput.Waypoints.Count > 2 && SelectedWaypoint is not null;

    public bool HasMultipleSegments => Segments.Count > 1;

    public bool CanRemoveSegment => Segments.Count > 1 && SelectedSegment is not null;

    public SegmentInputModel? SelectedSegment
    {
        get => _selectedSegment;
        set
        {
            if (SetProperty(ref _selectedSegment, value))
                OnPropertyChanged(nameof(CanRemoveSegment));
        }
    }

    public string StatusMessage
    {
        get => _statusMessage;
        set => SetProperty(ref _statusMessage, value);
    }

    public string CursorStatusText
    {
        get => _cursorStatusText;
        set => SetProperty(ref _cursorStatusText, value);
    }

    private void AddInitialSegment()
    {
        var segment = new SegmentInputModel(1);
        Segments.Add(segment);
        SelectedSegment = segment;
    }

    private void OnSegmentsCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        OnPropertyChanged(nameof(HasMultipleSegments));
        OnPropertyChanged(nameof(CanRemoveSegment));
    }

    private void AddSegment_Click(object sender, RoutedEventArgs e)
    {
        int segmentNumber = Segments.Count + 1;
        var segment = new SegmentInputModel(segmentNumber);

        SegmentInputModel? previous = Segments.LastOrDefault();
        if (previous is not null)
        {
            segment.StartPosition = previous.EndPosition;
            if (InputNumber.TryParse(previous.EndPosition, out double previousEnd))
                segment.EndPosition = (previousEnd + 10.0).ToString("F0");
        }

        Segments.Add(segment);
        SelectedSegment = segment;
        StatusMessage = $"Added segment {segmentNumber}.";
    }

    private void RemoveSelectedSegment_Click(object sender, RoutedEventArgs e)
    {
        if (!CanRemoveSegment || SelectedSegment is null)
            return;

        int removedIndex = Segments.IndexOf(SelectedSegment);
        Segments.Remove(SelectedSegment);
        RenumberSegments();

        if (Segments.Count > 0)
        {
            int nextIndex = Math.Clamp(removedIndex, 0, Segments.Count - 1);
            SelectedSegment = Segments[nextIndex];
        }

        StatusMessage = $"Removed segment {removedIndex + 1}.";
    }

    private void SwitchMode(bool passThrough)
    {
        if (_passThroughMode == passThrough)
            return;

        _passThroughMode = passThrough;
        OnPropertyChanged(nameof(IsBlendingMode));
        OnPropertyChanged(nameof(IsPassThroughMode));
        ReportTabs.SelectedIndex = 0;

        // Each mode keeps its inputs (separate models) and the last result it computed.
        PrototypeComputationResult? stored = passThrough ? _passThroughResult : _blendingResult;
        if (stored is null)
        {
            _currentResult = null;
            ExtremaReportRows.Clear();
            CoefficientReportRows.Clear();
            SummaryReportRows.Clear();
            LimitPeakRows.Clear();
            OnPropertyChanged(nameof(HasLimitPeaks));
            WaypointStateRows.Clear();
            CursorReadoutRows.Clear();
            ResetPlot();
            StatusMessage = passThrough
                ? "Pass-through mode: enter the waypoints, then click Compute."
                : "Configure the prototype inputs, then click Compute.";
        }
        else
        {
            ShowResult(stored);
            StatusMessage = "Showing the last result of this mode.";
        }
    }

    private void AddWaypoint_Click(object sender, RoutedEventArgs e)
    {
        var waypoints = PassThroughInput.Waypoints;
        WaypointInputModel? last = waypoints.LastOrDefault();
        string position = last is not null && InputNumber.TryParse(last.Position, out double lastPosition)
            ? (lastPosition + 1.0).ToString("F0") : "0";

        var waypoint = new WaypointInputModel(waypoints.Count + 1, position);
        waypoints.Add(waypoint);
        SelectedWaypoint = waypoint;
        StatusMessage = $"Added waypoint {waypoints.Count}.";
    }

    private void RemoveWaypoint_Click(object sender, RoutedEventArgs e)
    {
        if (!CanRemoveWaypoint || SelectedWaypoint is null)
            return;

        var waypoints = PassThroughInput.Waypoints;
        int index = waypoints.IndexOf(SelectedWaypoint);
        waypoints.Remove(SelectedWaypoint);
        SelectedWaypoint = waypoints[Math.Clamp(index, 0, waypoints.Count - 1)];
        StatusMessage = $"Removed waypoint {index + 1}.";
    }

    private void ShowResult(PrototypeComputationResult result)
    {
        _currentResult = result;

        ReplaceRows(ExtremaReportRows, result.ExtremaRows);
        ReplaceRows(CoefficientReportRows, result.CoefficientRows);
        ReplaceRows(SummaryReportRows, result.SummaryRows ?? []);
        ReplaceRows(LimitPeakRows, result.LimitPeaks ?? []);
        OnPropertyChanged(nameof(HasLimitPeaks));
        ReplaceRows(WaypointStateRows, result.WaypointStates ?? []);

        BuildPlot(result);
        UpdateCursorReadout(0.0);
    }

    private void Compute_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            if (_passThroughMode)
            {
                WaypointGrid.CommitEdit(System.Windows.Controls.DataGridEditingUnit.Row, true);
                _passThroughResult = _trajectoryService.ComputePassThrough(PassThroughInput);
                _currentResult = _passThroughResult;
            }
            else
            {
                _blendingResult = _trajectoryService.Compute(Segments);
                _currentResult = _blendingResult;
            }

            ShowResult(_currentResult);

            StatusMessage = $"Computed {_currentResult.Series.Count} plotted quantities across {_currentResult.TotalDuration:F4} seconds."
                + (_currentResult.Notices.Count > 0 ? " " + string.Join(" ", _currentResult.Notices) : string.Empty);
        }
        catch (Exception ex)
        {
            StatusMessage = ex.Message;
            MessageBox.Show(this, ex.Message, "Computation Error", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private void BuildPlot(PrototypeComputationResult result)
    {
        _cursorLines.Clear();

        for (int i = 0; i < _plotControls.Count; i++)
        {
            WpfPlot plotControl = _plotControls[i];
            plotControl.Plot.Clear();

            bool hasSeries = i < result.Series.Count;
            _plotHosts[i].Visibility = hasSeries ? Visibility.Visible : Visibility.Collapsed;
            if (!hasSeries)
                continue;

            SeriesData series = result.Series[i];
            var scatter = plotControl.Plot.Add.Scatter(series.Times, series.Values);
            scatter.LegendText = series.Name;
            scatter.LineWidth = 2;
            plotControl.Plot.Title(series.Name);
            plotControl.Plot.YLabel(series.Name);
            plotControl.Plot.XLabel("Time");
            if (series.Order == 0 && result.WaypointMarkers is { Count: > 0 } markers)
            {
                // Pass-through: the waypoints, on the position plot.
                var points = plotControl.Plot.Add.Scatter(
                    markers.Select(m => m.Time).ToArray(), markers.Select(m => m.Position).ToArray());
                points.LineWidth = 0;
                points.MarkerSize = 10;
                points.LegendText = "Waypoints";
            }

            plotControl.Plot.Axes.AutoScale();

            var cursorLine = plotControl.Plot.Add.VerticalLine(0);
            cursorLine.LineWidth = 2;
            _cursorLines.Add(cursorLine);

            plotControl.Refresh();
        }
    }

    private void ResetPlot()
    {
        _cursorLines.Clear();

        for (int i = 0; i < _plotControls.Count; i++)
        {
            WpfPlot plotControl = _plotControls[i];
            plotControl.Plot.Clear();
            plotControl.Plot.Title(i == 0 ? "Compute to populate the stacked plots" : string.Empty);
            plotControl.Plot.XLabel("Time");
            plotControl.Plot.YLabel("Value");
            _plotHosts[i].Visibility = i == 0 ? Visibility.Visible : Visibility.Collapsed;
            plotControl.Refresh();
        }
    }

    private void TrajectoryPlot_MouseMove(object sender, MouseEventArgs e)
    {
        if (_currentResult is null)
            return;

        if (sender is not WpfPlot activePlot)
            return;

        Point position = e.GetPosition(activePlot);
        Pixel pixel = new(
            (float)(position.X * activePlot.DisplayScale),
            (float)(position.Y * activePlot.DisplayScale));

        Coordinates coordinates = activePlot.Plot.GetCoordinates(pixel);
        double time = Math.Clamp(coordinates.X, 0.0, _currentResult.TotalDuration);
        UpdateCursorReadout(time);
    }

    private void UpdateCursorReadout(double time)
    {
        if (_currentResult is null)
            return;

        foreach (VerticalLine cursorLine in _cursorLines)
            cursorLine.X = time;

        CursorStatusText = $"Cursor time: {time:F6}";

        for (int i = 0; i < _currentResult.Series.Count; i++)
        {
            double value = _currentResult.Series[i].Order == 0
                ? _currentResult.PositionAt(time)
                : _currentResult.DerivativeAt(time, _currentResult.Series[i].Order);

            if (i < CursorReadoutRows.Count)
            {
                CursorReadoutRows[i].Value = value;
            }
            else
            {
                CursorReadoutRows.Add(new CursorReadoutRow(_currentResult.Series[i].Name, value));
            }
        }

        while (CursorReadoutRows.Count > _currentResult.Series.Count)
        {
            CursorReadoutRows.RemoveAt(CursorReadoutRows.Count - 1);
        }

        foreach (WpfPlot plotControl in _plotControls.Where((_, index) => index < _currentResult.Series.Count))
            plotControl.Refresh();
    }

    private static void ReplaceRows<T>(ObservableCollection<T> target, IReadOnlyList<T> source)
    {
        target.Clear();
        foreach (T item in source)
            target.Add(item);
    }

    private void RenumberSegments()
    {
        for (int i = 0; i < Segments.Count; i++)
            Segments[i].SegmentNumber = i + 1;
    }

    private bool SetProperty<T>(ref T storage, T value, [CallerMemberName] string? propertyName = null)
    {
        if (EqualityComparer<T>.Default.Equals(storage, value))
            return false;

        storage = value;
        OnPropertyChanged(propertyName);
        return true;
    }

    private void OnPropertyChanged([CallerMemberName] string? propertyName = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
