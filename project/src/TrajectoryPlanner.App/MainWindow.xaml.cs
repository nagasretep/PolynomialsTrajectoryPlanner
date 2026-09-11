using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Input;
using ScottPlot;
using ScottPlot.Plottables;

namespace TrajectoryPlanner.App;

public partial class MainWindow : Window, INotifyPropertyChanged
{
    private readonly PrototypeTrajectoryService _trajectoryService = new();
    private PrototypeComputationResult? _currentResult;
    private VerticalLine? _cursorLine;
    private string _statusMessage = "Configure the prototype inputs, then click Compute.";
    private string _cursorStatusText = "Cursor time: n/a";

    public MainWindow()
    {
        InitializeComponent();
        DataContext = this;

        Segments.CollectionChanged += OnSegmentsCollectionChanged;
        AddInitialSegment();
        Loaded += (_, _) => ResetPlot();
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    public int[] SupportedDegrees { get; } = [3, 5, 7, 9];

    public ObservableCollection<SegmentInputModel> Segments { get; } = [];

    public ObservableCollection<ExtremaReportRow> ExtremaReportRows { get; } = [];

    public ObservableCollection<CoefficientReportRow> CoefficientReportRows { get; } = [];

    public ObservableCollection<CursorReadoutRow> CursorReadoutRows { get; } = [];

    public bool HasMultipleSegments => Segments.Count > 1;

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
    }

    private void OnSegmentsCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        OnPropertyChanged(nameof(HasMultipleSegments));
    }

    private void AddSegment_Click(object sender, RoutedEventArgs e)
    {
        int segmentNumber = Segments.Count + 1;
        var segment = new SegmentInputModel(segmentNumber);

        SegmentInputModel? previous = Segments.LastOrDefault();
        if (previous is not null)
        {
            segment.StartPosition = previous.EndPosition;
            if (double.TryParse(previous.EndPosition, out double previousEnd))
                segment.EndPosition = (previousEnd + 10.0).ToString("F0");
        }

        Segments.Add(segment);
        StatusMessage = $"Added segment {segmentNumber}.";
    }

    private void Compute_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            _currentResult = _trajectoryService.Compute(Segments);

            ReplaceRows(ExtremaReportRows, _currentResult.ExtremaRows);
            ReplaceRows(CoefficientReportRows, _currentResult.CoefficientRows);

            BuildPlot(_currentResult);
            UpdateCursorReadout(0.0);

            StatusMessage = $"Computed {_currentResult.Series.Count} plotted quantities across {_currentResult.TotalDuration:F4} seconds.";
        }
        catch (Exception ex)
        {
            StatusMessage = ex.Message;
            MessageBox.Show(this, ex.Message, "Computation Error", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private void BuildPlot(PrototypeComputationResult result)
    {
        var plot = TrajectoryPlot.Plot;
        plot.Clear();

        foreach (SeriesData series in result.Series)
        {
            var scatter = plot.Add.Scatter(series.Times, series.Values);
            scatter.LegendText = series.Name;
            scatter.LineWidth = 2;
        }

        plot.Title("1D Trajectory Prototype");
        plot.XLabel("Time");
        plot.YLabel("Value");
        plot.ShowLegend(Alignment.UpperRight);

        _cursorLine = plot.Add.VerticalLine(0);
        _cursorLine.LineWidth = 2;

        TrajectoryPlot.Refresh();
    }

    private void ResetPlot()
    {
        var plot = TrajectoryPlot.Plot;
        plot.Clear();
        plot.Title("1D Trajectory Prototype");
        plot.XLabel("Time");
        plot.YLabel("Value");
        plot.ShowLegend(Alignment.UpperRight);
        TrajectoryPlot.Refresh();
    }

    private void TrajectoryPlot_MouseMove(object sender, MouseEventArgs e)
    {
        if (_currentResult is null)
            return;

        Point position = e.GetPosition(TrajectoryPlot);
        Pixel pixel = new(
            (float)(position.X * TrajectoryPlot.DisplayScale),
            (float)(position.Y * TrajectoryPlot.DisplayScale));

        Coordinates coordinates = TrajectoryPlot.Plot.GetCoordinates(pixel);
        double time = Math.Clamp(coordinates.X, 0.0, _currentResult.TotalDuration);
        UpdateCursorReadout(time);
    }

    private void UpdateCursorReadout(double time)
    {
        if (_currentResult is null)
            return;

        if (_cursorLine is not null)
            _cursorLine.X = time;

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

        TrajectoryPlot.Refresh();
    }

    private static void ReplaceRows<T>(ObservableCollection<T> target, IReadOnlyList<T> source)
    {
        target.Clear();
        foreach (T item in source)
            target.Add(item);
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
