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
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.App;

public enum AppMode
{
    Blending,
    PassThrough,
    ThreeD,
    ThreeDPassThrough
}

public partial class MainWindow : Window, INotifyPropertyChanged
{
    private static readonly ScottPlot.Color[] AxisColors = { Colors.Red, Colors.Green, Colors.Blue };

    private readonly PrototypeTrajectoryService _trajectoryService = new();
    private readonly Prototype3DTrajectoryService _trajectory3DService = new();
    private readonly List<WpfPlot> _plotControls = [];
    private readonly List<Border> _plotHosts = [];
    private readonly List<WpfPlot> _pathPlots = [];
    private PrototypeComputationResult? _currentResult;
    private PrototypeComputationResult? _blendingResult;
    private PrototypeComputationResult? _passThroughResult;
    private Prototype3DComputationResult? _result3D;
    private Prototype3DComputationResult? _result3DPassThrough;
    private Waypoint3DInputModel? _selectedWaypoint3D;
    private AppMode _mode = AppMode.Blending;
    private bool _showNorm;
    private WaypointInputModel? _selectedWaypoint;
    private readonly List<VerticalLine> _cursorLines = [];
    private readonly List<(double[] X, double[] Y)> _pathCursorMarkers = [];
    private SegmentInputModel? _selectedSegment;
    private Segment3DInputModel? _selectedSegment3D;
    private string _statusMessage = "Configure the prototype inputs, then click Compute.";
    private string _cursorStatusText = "Cursor time: n/a";

    public MainWindow()
    {
        InitializeComponent();
        DataContext = this;

        _plotControls.AddRange([TrajectoryPlot1, TrajectoryPlot2, TrajectoryPlot3, TrajectoryPlot4, TrajectoryPlot5]);
        _plotHosts.AddRange([PlotHost1, PlotHost2, PlotHost3, PlotHost4, PlotHost5]);
        _pathPlots.AddRange([PathPlotXY, PathPlotXZ, PathPlotYZ]);

        Segments.CollectionChanged += OnSegmentsCollectionChanged;
        Segments3D.CollectionChanged += OnSegmentsCollectionChanged;
        PassThroughInput.Waypoints.CollectionChanged += (_, _) => OnPropertyChanged(nameof(CanRemoveWaypoint));
        PassThrough3DInput.Waypoints.CollectionChanged += (_, _) => OnPropertyChanged(nameof(CanRemoveWaypoint));
        AddInitialSegment();
        AddInitialSegment3D();
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

    // ---- 3D mode ---------------------------------------------------------------------------------

    /// <summary>Inputs of the 3D mode; kept when the mode is switched.</summary>
    public ObservableCollection<Segment3DInputModel> Segments3D { get; } = [];

    public ObservableCollection<PartReportRow> PartRows3D { get; } = [];

    public ObservableCollection<Extrema3DReportRow> ExtremaRows3D { get; } = [];

    public ObservableCollection<Limit3DReportRow> LimitRows3D { get; } = [];

    public ObservableCollection<Cursor3DRow> Cursor3DRows { get; } = [];

    public bool HasMultipleSegments3D => Segments3D.Count > 1;

    public Segment3DInputModel? SelectedSegment3D
    {
        get => _selectedSegment3D;
        set
        {
            if (SetProperty(ref _selectedSegment3D, value))
                OnPropertyChanged(nameof(CanRemoveSegment));
        }
    }

    /// <summary>3D mode: add the magnitude curve to each quantity.</summary>
    public bool ShowNorm
    {
        get => _showNorm;
        set
        {
            if (SetProperty(ref _showNorm, value) && Current3DResult is { } shown)
                Show3DResult(shown);
        }
    }

    // ---- modes -------------------------------------------------------------------------------------

    public bool IsBlendingMode
    {
        get => _mode == AppMode.Blending;
        set { if (value) SwitchMode(AppMode.Blending); }
    }

    public bool IsPassThroughMode
    {
        get => _mode == AppMode.PassThrough;
        set { if (value) SwitchMode(AppMode.PassThrough); }
    }

    public bool IsThreeDMode
    {
        get => _mode == AppMode.ThreeD;
        set { if (value) SwitchMode(AppMode.ThreeD); }
    }

    public bool IsThreeDPassThroughMode
    {
        get => _mode == AppMode.ThreeDPassThrough;
        set { if (value) SwitchMode(AppMode.ThreeDPassThrough); }
    }

    /// <summary>The two 3D modes share the presentation (three curves, Path tab, 3D report tabs, 3D cursor table).</summary>
    public bool IsAny3DMode => _mode is AppMode.ThreeD or AppMode.ThreeDPassThrough;

    public bool IsOneDMode => !IsAny3DMode;

    /// <summary>Modes whose inputs are segment tabs (add / remove segment buttons).</summary>
    public bool IsSegmentMode => _mode is AppMode.Blending or AppMode.ThreeD;

    /// <summary>Modes whose inputs are a waypoint table (add / remove waypoint buttons).</summary>
    public bool IsWaypointMode => _mode is AppMode.PassThrough or AppMode.ThreeDPassThrough;

    /// <summary>Inputs of the 3D pass-through mode; kept when the mode is switched.</summary>
    public PassThrough3DInputModel PassThrough3DInput { get; } = new();

    public ObservableCollection<WaypointState3DRow> WaypointStateRows3D { get; } = [];

    public Waypoint3DInputModel? SelectedWaypoint3D
    {
        get => _selectedWaypoint3D;
        set
        {
            if (SetProperty(ref _selectedWaypoint3D, value))
                OnPropertyChanged(nameof(CanRemoveWaypoint));
        }
    }

    /// <summary>The 3D result of the current mode (null in the 1D modes or before Compute).</summary>
    private Prototype3DComputationResult? Current3DResult => _mode switch
    {
        AppMode.ThreeD => _result3D,
        AppMode.ThreeDPassThrough => _result3DPassThrough,
        _ => null
    };

    public bool ShowsSummary => _mode != AppMode.Blending;

    public WaypointInputModel? SelectedWaypoint
    {
        get => _selectedWaypoint;
        set
        {
            if (SetProperty(ref _selectedWaypoint, value))
                OnPropertyChanged(nameof(CanRemoveWaypoint));
        }
    }

    public bool CanRemoveWaypoint => _mode == AppMode.ThreeDPassThrough
        ? PassThrough3DInput.Waypoints.Count > 2 && SelectedWaypoint3D is not null
        : PassThroughInput.Waypoints.Count > 2 && SelectedWaypoint is not null;

    public bool HasMultipleSegments => Segments.Count > 1;

    public bool CanRemoveSegment => _mode == AppMode.ThreeD
        ? Segments3D.Count > 1 && SelectedSegment3D is not null
        : Segments.Count > 1 && SelectedSegment is not null;

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

    private void AddInitialSegment3D()
    {
        var segment = new Segment3DInputModel(1);
        Segments3D.Add(segment);
        SelectedSegment3D = segment;
    }

    private void OnSegmentsCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        OnPropertyChanged(nameof(HasMultipleSegments));
        OnPropertyChanged(nameof(HasMultipleSegments3D));
        OnPropertyChanged(nameof(CanRemoveSegment));
    }

    private void AddSegment_Click(object sender, RoutedEventArgs e)
    {
        if (_mode == AppMode.ThreeD)
        {
            AddSegment3D();
            return;
        }

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

    private void AddSegment3D()
    {
        int segmentNumber = Segments3D.Count + 1;
        var segment = new Segment3DInputModel(segmentNumber);

        // The new segment starts where the previous one ends (position and velocity, which a continuous path needs).
        Segment3DInputModel? previous = Segments3D.LastOrDefault();
        if (previous is not null)
        {
            segment.StartPosition.CopyFrom(previous.EndPosition);
            segment.StartVelocity.CopyFrom(previous.EndVelocity);
            segment.StartAcceleration.CopyFrom(previous.EndAcceleration);
            segment.StartJerk.CopyFrom(previous.EndJerk);
            segment.StartSnap.CopyFrom(previous.EndSnap);
            segment.EndPosition.X = Shifted(previous.EndPosition.X, 1);
            segment.EndPosition.Y = previous.EndPosition.Y;
            segment.EndPosition.Z = previous.EndPosition.Z;
        }

        Segments3D.Add(segment);
        SelectedSegment3D = segment;
        StatusMessage = $"Added 3D segment {segmentNumber}.";
    }

    private static string Shifted(string text, double by) =>
        InputNumber.TryParse(text, out double value) ? (value + by).ToString("G") : text;

    private void RemoveSelectedSegment_Click(object sender, RoutedEventArgs e)
    {
        if (!CanRemoveSegment)
            return;

        if (_mode == AppMode.ThreeD)
        {
            int removed3D = Segments3D.IndexOf(SelectedSegment3D!);
            Segments3D.Remove(SelectedSegment3D!);
            for (int i = 0; i < Segments3D.Count; i++)
                Segments3D[i].SegmentNumber = i + 1;
            SelectedSegment3D = Segments3D[Math.Clamp(removed3D, 0, Segments3D.Count - 1)];
            StatusMessage = $"Removed 3D segment {removed3D + 1}.";
            return;
        }

        int removedIndex = Segments.IndexOf(SelectedSegment!);
        Segments.Remove(SelectedSegment!);
        RenumberSegments();

        if (Segments.Count > 0)
        {
            int nextIndex = Math.Clamp(removedIndex, 0, Segments.Count - 1);
            SelectedSegment = Segments[nextIndex];
        }

        StatusMessage = $"Removed segment {removedIndex + 1}.";
    }

    private void SwitchMode(AppMode mode)
    {
        if (_mode == mode)
            return;

        _mode = mode;
        foreach (string name in new[]
                 {
                     nameof(IsBlendingMode), nameof(IsPassThroughMode), nameof(IsThreeDMode), nameof(IsThreeDPassThroughMode),
                     nameof(IsAny3DMode), nameof(IsOneDMode), nameof(IsSegmentMode), nameof(IsWaypointMode),
                     nameof(ShowsSummary), nameof(CanRemoveSegment), nameof(CanRemoveWaypoint)
                 })
            OnPropertyChanged(name);

        // The first report tab that this mode shows: 1D Extrema, 3D Parts, 3D pass-through Summary.
        ReportTabs.SelectedIndex = mode switch { AppMode.ThreeD => 4, AppMode.ThreeDPassThrough => 2, _ => 0 };
        ChartTabs.SelectedIndex = 0;

        ClearReports();

        // Each mode keeps its inputs (separate models) and the last result it computed.
        if (IsAny3DMode)
        {
            if (Current3DResult is null)
            {
                ResetPlot();
                StatusMessage = mode == AppMode.ThreeD
                    ? "3D mode: enter the segments as vectors (X | Y | Z), then click Compute."
                    : "3D pass-through mode: enter the waypoints (X | Y | Z), then click Compute.";
            }
            else
            {
                Show3DResult(Current3DResult);
                StatusMessage = "Showing the last result of this mode.";
            }

            return;
        }

        PrototypeComputationResult? stored = mode == AppMode.PassThrough ? _passThroughResult : _blendingResult;
        if (stored is null)
        {
            _currentResult = null;
            ResetPlot();
            StatusMessage = mode == AppMode.PassThrough
                ? "Pass-through mode: enter the waypoints, then click Compute."
                : "Configure the prototype inputs, then click Compute.";
        }
        else
        {
            ShowResult(stored);
            StatusMessage = "Showing the last result of this mode.";
        }
    }

    private void ClearReports()
    {
        _currentResult = null;
        ExtremaReportRows.Clear();
        CoefficientReportRows.Clear();
        SummaryReportRows.Clear();
        LimitPeakRows.Clear();
        OnPropertyChanged(nameof(HasLimitPeaks));
        WaypointStateRows.Clear();
        CursorReadoutRows.Clear();
        PartRows3D.Clear();
        ExtremaRows3D.Clear();
        LimitRows3D.Clear();
        Cursor3DRows.Clear();
        WaypointStateRows3D.Clear();
    }

    private void AddWaypoint_Click(object sender, RoutedEventArgs e)
    {
        if (_mode == AppMode.ThreeDPassThrough)
        {
            var list = PassThrough3DInput.Waypoints;
            Waypoint3DInputModel? previous = list.LastOrDefault();
            var added = previous is null
                ? new Waypoint3DInputModel(1, "0", "0", "0")
                : new Waypoint3DInputModel(list.Count + 1, Shifted(previous.X, 1), previous.Y, previous.Z);
            list.Add(added);
            SelectedWaypoint3D = added;
            StatusMessage = $"Added waypoint {list.Count}.";
            return;
        }

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
        if (!CanRemoveWaypoint)
            return;

        if (_mode == AppMode.ThreeDPassThrough)
        {
            var list = PassThrough3DInput.Waypoints;
            int removed = list.IndexOf(SelectedWaypoint3D!);
            list.Remove(SelectedWaypoint3D!);
            SelectedWaypoint3D = list[Math.Clamp(removed, 0, list.Count - 1)];
            StatusMessage = $"Removed waypoint {removed + 1}.";
            return;
        }

        if (SelectedWaypoint is null)
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
            if (IsAny3DMode)
            {
                Prototype3DComputationResult computed;
                if (_mode == AppMode.ThreeD)
                {
                    computed = _result3D = _trajectory3DService.Compute(Segments3D);
                }
                else
                {
                    WaypointGrid3D.CommitEdit(System.Windows.Controls.DataGridEditingUnit.Row, true);
                    computed = _result3DPassThrough = _trajectory3DService.ComputePassThrough(PassThrough3DInput);
                }

                Show3DResult(computed);
                StatusMessage = $"Computed {computed.Series.Count} plotted quantities across {computed.TotalDuration:F4} seconds; path length {computed.PathLength:F4}."
                    + (computed.Notices.Count > 0 ? " " + string.Join(" ", computed.Notices) : string.Empty);
                return;
            }

            if (_mode == AppMode.PassThrough)
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

    // ---- 3D display ----------------------------------------------------------------------------------

    private void Show3DResult(Prototype3DComputationResult result)
    {
        ReplaceRows(PartRows3D, result.PartRows);
        ReplaceRows(ExtremaRows3D, result.ExtremaRows);
        ReplaceRows(LimitRows3D, result.LimitRows);
        ReplaceRows(CoefficientReportRows, result.CoefficientRows);
        ReplaceRows(SummaryReportRows, result.SummaryRows);
        ReplaceRows(WaypointStateRows3D, result.WaypointStates ?? []);

        BuildPlot3D(result);
        BuildPathPlots(result);
        UpdateCursorReadout(0.0);
    }

    /// <summary>The same stacked time plots as in 1D, with three curves per quantity (and the norm when asked).</summary>
    private void BuildPlot3D(Prototype3DComputationResult result)
    {
        _cursorLines.Clear();
        string[] axisNames = { "X", "Y", "Z" };

        for (int i = 0; i < _plotControls.Count; i++)
        {
            WpfPlot plotControl = _plotControls[i];
            plotControl.Plot.Clear();

            bool hasSeries = i < result.Series.Count;
            _plotHosts[i].Visibility = hasSeries ? Visibility.Visible : Visibility.Collapsed;
            if (!hasSeries)
                continue;

            Series3D series = result.Series[i];
            double[][] components = { series.X, series.Y, series.Z };
            for (int axis = 0; axis < 3; axis++)
            {
                var curve = plotControl.Plot.Add.Scatter(series.Times, components[axis]);
                curve.LegendText = axisNames[axis];
                curve.Color = AxisColors[axis];
                curve.LineWidth = 2;
                curve.MarkerSize = 0;
            }

            if (ShowNorm)
            {
                var norm = plotControl.Plot.Add.Scatter(series.Times, series.Norm);
                norm.LegendText = "|.|";
                norm.Color = Colors.Black;
                norm.LineWidth = 1.5f;
                norm.MarkerSize = 0;
                norm.LinePattern = LinePattern.Dashed;
            }

            // 3D pass-through: the waypoints on the position plot (one marker per axis, in the axis colour).
            if (series.Order == 0 && result.Waypoints is { Count: > 0 } waypoints)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    var marks = plotControl.Plot.Add.Scatter(
                        waypoints.Select(w => w.Time).ToArray(), waypoints.Select(w => w.Point[axis]).ToArray());
                    marks.LineWidth = 0;
                    marks.MarkerSize = 9;
                    marks.MarkerShape = MarkerShape.OpenCircle;
                    marks.Color = AxisColors[axis];
                }
            }

            // Part boundaries.
            foreach (double boundary in result.PartBoundaries)
            {
                var line = plotControl.Plot.Add.VerticalLine(boundary);
                line.LineWidth = 1;
                line.Color = Colors.Gray.WithAlpha(0.6);
                line.LinePattern = LinePattern.Dotted;
            }

            plotControl.Plot.Title(series.Name);
            plotControl.Plot.YLabel(series.Name);
            plotControl.Plot.XLabel("Time");
            plotControl.Plot.ShowLegend(Alignment.UpperRight);
            plotControl.Plot.Axes.AutoScale();

            var cursorLine = plotControl.Plot.Add.VerticalLine(0);
            cursorLine.LineWidth = 2;
            _cursorLines.Add(cursorLine);

            plotControl.Refresh();
        }
    }

    /// <summary>Projections XY, XZ, YZ with equal scales; the path coloured by part kind, segment ends and a cursor marker.</summary>
    private void BuildPathPlots(Prototype3DComputationResult result)
    {
        _pathCursorMarkers.Clear();
        var projections = new (string Title, Func<PathPiece, double[]> H, Func<PathPiece, double[]> V, Func<Point3D, double> Hp, Func<Point3D, double> Vp)[]
        {
            ("XY", p => p.X, p => p.Y, p => p.X, p => p.Y),
            ("XZ", p => p.X, p => p.Z, p => p.X, p => p.Z),
            ("YZ", p => p.Y, p => p.Z, p => p.Y, p => p.Z)
        };

        for (int k = 0; k < 3; k++)
        {
            var (title, h, v, hp, vp) = projections[k];
            Plot plot = _pathPlots[k].Plot;
            plot.Clear();

            var seen = new HashSet<string>();
            foreach (PathPiece piece in result.PathPieces)
            {
                if (piece.X.Length < 2)
                    continue;

                var line = plot.Add.Scatter(h(piece), v(piece));
                line.MarkerSize = 0;
                line.LineWidth = piece.Kind == "Stretch" ? 3.5f : 2;
                line.Color = PartColor(piece.Kind);
                if (seen.Add(piece.Kind))
                    line.LegendText = piece.Kind;
            }

            var ends = plot.Add.Scatter(
                result.SegmentEndpoints.Select(hp).ToArray(), result.SegmentEndpoints.Select(vp).ToArray());
            ends.LineWidth = 0;
            ends.MarkerSize = 8;
            ends.MarkerShape = MarkerShape.FilledSquare;
            ends.Color = Colors.Black;

            if (result.Waypoints is { Count: > 0 } waypoints)
            {
                var marks = plot.Add.Scatter(
                    waypoints.Select(w => hp(w.Point)).ToArray(), waypoints.Select(w => vp(w.Point)).ToArray());
                marks.LineWidth = 0;
                marks.MarkerSize = 12;
                marks.MarkerShape = MarkerShape.OpenCircle;
                marks.Color = Colors.DarkGreen;
                marks.LegendText = "Waypoints";
            }

            // The cursor marker: a one-point series whose arrays are updated in place.
            var markerX = new double[1];
            var markerY = new double[1];
            var marker = plot.Add.Scatter(markerX, markerY);
            marker.LineWidth = 0;
            marker.MarkerSize = 11;
            marker.MarkerShape = MarkerShape.FilledCircle;
            marker.Color = Colors.Black;
            _pathCursorMarkers.Add((markerX, markerY));

            plot.Title(title);
            plot.XLabel(title[..1]);
            plot.YLabel(title[1..]);
            plot.ShowLegend(Alignment.UpperRight);
            plot.Axes.AutoScale();
            plot.Axes.SquareUnits();
            _pathPlots[k].Refresh();
        }
    }

    private static ScottPlot.Color PartColor(string kind) => kind switch
    {
        "Segment" => Colors.SteelBlue,
        "Blend" => Colors.Orange,
        "Stretch" => Colors.Crimson,
        _ => Colors.DimGray
    };

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

        _pathCursorMarkers.Clear();
        foreach (WpfPlot path in _pathPlots)
        {
            path.Plot.Clear();
            path.Refresh();
        }
    }

    private double CurrentTotalDuration =>
        IsAny3DMode ? Current3DResult?.TotalDuration ?? 0 : _currentResult?.TotalDuration ?? 0;

    private bool HasResult => IsAny3DMode ? Current3DResult is not null : _currentResult is not null;

    private void TrajectoryPlot_MouseMove(object sender, MouseEventArgs e)
    {
        if (!HasResult)
            return;

        if (sender is not WpfPlot activePlot)
            return;

        System.Windows.Point position = e.GetPosition(activePlot);
        Pixel pixel = new(
            (float)(position.X * activePlot.DisplayScale),
            (float)(position.Y * activePlot.DisplayScale));

        Coordinates coordinates = activePlot.Plot.GetCoordinates(pixel);
        double time = Math.Clamp(coordinates.X, 0.0, CurrentTotalDuration);
        UpdateCursorReadout(time);
    }

    private void UpdateCursorReadout(double time)
    {
        if (IsAny3DMode)
        {
            UpdateCursorReadout3D(time);
            return;
        }

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

    private void UpdateCursorReadout3D(double time)
    {
        Prototype3DComputationResult? result = Current3DResult;
        if (result is null)
            return;

        foreach (VerticalLine cursorLine in _cursorLines)
            cursorLine.X = time;

        CursorStatusText = $"Cursor time: {time:F6}";
        State3D state = result.StateAt(time);

        for (int i = 0; i < result.Series.Count; i++)
        {
            Point3D value = state.GetDerivative(result.Series[i].Order);
            if (i >= Cursor3DRows.Count)
                Cursor3DRows.Add(new Cursor3DRow(result.Series[i].Name));

            Cursor3DRows[i].X = value.X;
            Cursor3DRows[i].Y = value.Y;
            Cursor3DRows[i].Z = value.Z;
            Cursor3DRows[i].Norm = value.Length;
        }

        while (Cursor3DRows.Count > result.Series.Count)
            Cursor3DRows.RemoveAt(Cursor3DRows.Count - 1);

        // The marker on the path projections.
        double[] h = { state.Position.X, state.Position.X, state.Position.Y };
        double[] v = { state.Position.Y, state.Position.Z, state.Position.Z };
        for (int k = 0; k < _pathCursorMarkers.Count; k++)
        {
            _pathCursorMarkers[k].X[0] = h[k];
            _pathCursorMarkers[k].Y[0] = v[k];
            _pathPlots[k].Refresh();
        }

        foreach (WpfPlot plotControl in _plotControls.Where((_, index) => index < result.Series.Count))
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
