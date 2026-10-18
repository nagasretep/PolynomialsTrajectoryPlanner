using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace TrajectoryPlanner.App;

public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected bool SetProperty<T>(ref T storage, T value, [CallerMemberName] string? propertyName = null)
    {
        if (EqualityComparer<T>.Default.Equals(storage, value))
            return false;

        storage = value;
        OnPropertyChanged(propertyName);
        return true;
    }

    protected void OnPropertyChanged([CallerMemberName] string? propertyName = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}

public sealed class SegmentInputModel : ObservableObject
{
    private int _segmentNumber;
    private int _degree = 9;
    private string _startPosition = "0";
    private string _endPosition = "10";
    private string _startVelocity = "0";
    private string _endVelocity = "0";
    private string _startAcceleration = "0";
    private string _endAcceleration = "0";
    private string _startJerk = "0";
    private string _endJerk = "0";
    private string _startSnap = "0";
    private string _endSnap = "0";
    private bool _useDuration = true;
    private string _duration = "1.0";
    private bool _useMaxVelocity;
    private string _maxVelocity = "10";
    private bool _useMaxAcceleration;
    private string _maxAcceleration = "10";
    private bool _useMaxJerk;
    private string _maxJerk = "10";
    private bool _useMaxSnap;
    private string _maxSnap = "10";
    private bool _useConstantVelocity;
    private string _constantVelocityStart = "2";
    private string _constantVelocityEnd = "8";
    private string _constantVelocitySpeed = "2";
    private string _blendToPrevious = string.Empty;
    private string _blendToNext = string.Empty;

    public SegmentInputModel(int segmentNumber)
    {
        _segmentNumber = segmentNumber;
    }

    public int SegmentNumber
    {
        get => _segmentNumber;
        set
        {
            if (SetProperty(ref _segmentNumber, value))
                OnPropertyChanged(nameof(Header));
        }
    }

    public string Header => $"Segment {SegmentNumber}";

    public int Degree
    {
        get => _degree;
        set
        {
            if (SetProperty(ref _degree, value))
            {
                OnPropertyChanged(nameof(SupportsAcceleration));
                OnPropertyChanged(nameof(SupportsJerk));
                OnPropertyChanged(nameof(SupportsSnap));
            }
        }
    }

    public bool SupportsAcceleration => Degree >= 5;

    public bool SupportsJerk => Degree >= 7;

    public bool SupportsSnap => Degree >= 9;

    public string StartPosition
    {
        get => _startPosition;
        set => SetProperty(ref _startPosition, value);
    }

    public string EndPosition
    {
        get => _endPosition;
        set => SetProperty(ref _endPosition, value);
    }

    public string StartVelocity
    {
        get => _startVelocity;
        set => SetProperty(ref _startVelocity, value);
    }

    public string EndVelocity
    {
        get => _endVelocity;
        set => SetProperty(ref _endVelocity, value);
    }

    public string StartAcceleration
    {
        get => _startAcceleration;
        set => SetProperty(ref _startAcceleration, value);
    }

    public string EndAcceleration
    {
        get => _endAcceleration;
        set => SetProperty(ref _endAcceleration, value);
    }

    public string StartJerk
    {
        get => _startJerk;
        set => SetProperty(ref _startJerk, value);
    }

    public string EndJerk
    {
        get => _endJerk;
        set => SetProperty(ref _endJerk, value);
    }

    public string StartSnap
    {
        get => _startSnap;
        set => SetProperty(ref _startSnap, value);
    }

    public string EndSnap
    {
        get => _endSnap;
        set => SetProperty(ref _endSnap, value);
    }

    public bool UseDuration
    {
        get => _useDuration;
        set => SetProperty(ref _useDuration, value);
    }

    public string Duration
    {
        get => _duration;
        set => SetProperty(ref _duration, value);
    }

    public bool UseMaxVelocity
    {
        get => _useMaxVelocity;
        set => SetProperty(ref _useMaxVelocity, value);
    }

    public string MaxVelocity
    {
        get => _maxVelocity;
        set => SetProperty(ref _maxVelocity, value);
    }

    public bool UseMaxAcceleration
    {
        get => _useMaxAcceleration;
        set => SetProperty(ref _useMaxAcceleration, value);
    }

    public string MaxAcceleration
    {
        get => _maxAcceleration;
        set => SetProperty(ref _maxAcceleration, value);
    }

    public bool UseMaxJerk
    {
        get => _useMaxJerk;
        set => SetProperty(ref _useMaxJerk, value);
    }

    public string MaxJerk
    {
        get => _maxJerk;
        set => SetProperty(ref _maxJerk, value);
    }

    public bool UseMaxSnap
    {
        get => _useMaxSnap;
        set => SetProperty(ref _useMaxSnap, value);
    }

    public string MaxSnap
    {
        get => _maxSnap;
        set => SetProperty(ref _maxSnap, value);
    }

    public bool UseConstantVelocity
    {
        get => _useConstantVelocity;
        set => SetProperty(ref _useConstantVelocity, value);
    }

    public string ConstantVelocityStart
    {
        get => _constantVelocityStart;
        set => SetProperty(ref _constantVelocityStart, value);
    }

    public string ConstantVelocityEnd
    {
        get => _constantVelocityEnd;
        set => SetProperty(ref _constantVelocityEnd, value);
    }

    public string ConstantVelocitySpeed
    {
        get => _constantVelocitySpeed;
        set => SetProperty(ref _constantVelocitySpeed, value);
    }

    public string BlendToPrevious
    {
        get => _blendToPrevious;
        set => SetProperty(ref _blendToPrevious, value);
    }

    public string BlendToNext
    {
        get => _blendToNext;
        set => SetProperty(ref _blendToNext, value);
    }
}

public sealed record ExtremaReportRow(
    string Segment,
    string Quantity,
    double Minimum,
    double Maximum,
    double MinTime,
    double MaxTime);

public sealed record CoefficientReportRow(
    string Segment,
    string Coefficient,
    double Value);

public sealed class CursorReadoutRow : ObservableObject
{
    private double _value;

    public CursorReadoutRow(string quantity, double value)
    {
        Quantity = quantity;
        _value = value;
    }

    public string Quantity { get; }

    public double Value
    {
        get => _value;
        set => SetProperty(ref _value, value);
    }
}

public sealed record SeriesData(string Name, int Order, double[] Times, double[] Values);

public sealed record PrototypeComputationResult(
    IReadOnlyList<SeriesData> Series,
    IReadOnlyList<ExtremaReportRow> ExtremaRows,
    IReadOnlyList<CoefficientReportRow> CoefficientRows,
    Func<double, double> PositionAt,
    Func<double, int, double> DerivativeAt,
    double TotalDuration,
    IReadOnlyList<string> Notices,
    IReadOnlyList<WaypointMarker>? WaypointMarkers = null,
    IReadOnlyList<WaypointStateRow>? WaypointStates = null,
    IReadOnlyList<SummaryRow>? SummaryRows = null);

/// <summary>One row of the pass-through waypoint table: a position and, optionally, the duration of the segment that starts there.</summary>
public sealed class WaypointInputModel : ObservableObject
{
    private int _number;
    private bool _hasSegment = true;
    private string _position;
    private string _duration = string.Empty;

    public WaypointInputModel(int number, string position)
    {
        _number = number;
        _position = position;
    }

    public int Number
    {
        get => _number;
        set => SetProperty(ref _number, value);
    }

    /// <summary>False for the last waypoint: no segment starts there, so its duration cell is not used.</summary>
    public bool HasSegment
    {
        get => _hasSegment;
        set => SetProperty(ref _hasSegment, value);
    }

    public string Position
    {
        get => _position;
        set => SetProperty(ref _position, value);
    }

    public string Duration
    {
        get => _duration;
        set => SetProperty(ref _duration, value);
    }
}

/// <summary>Inputs of the pass-through mode (design/08, "Pass-Through Mode"). Kept apart from the blending segments.</summary>
public sealed class PassThroughInputModel : ObservableObject
{
    private int _degree = 9;
    private int _smoothnessOrder = 4;
    private bool _startAtRest = true;
    private bool _endAtRest = true;
    private string _startVelocity = "0", _startAcceleration = "0", _startJerk = "0", _startSnap = "0";
    private string _endVelocity = "0", _endAcceleration = "0", _endJerk = "0", _endSnap = "0";
    private string _totalDuration = string.Empty;
    private bool _useMaxVelocity, _useMaxAcceleration, _useMaxJerk, _useMaxSnap;
    private string _maxVelocity = "10", _maxAcceleration = "10", _maxJerk = "10", _maxSnap = "10";

    public PassThroughInputModel()
    {
        Waypoints.Add(new WaypointInputModel(1, "0"));
        Waypoints.Add(new WaypointInputModel(2, "1"));
        Waypoints.Add(new WaypointInputModel(3, "2"));
        Renumber();
        Waypoints.CollectionChanged += (_, _) => Renumber();
    }

    public ObservableCollection<WaypointInputModel> Waypoints { get; } = [];

    /// <summary>p = (d + 1) / 2: number of shared state components per node (design/08: m runs from 1 to p).</summary>
    public int SharedStateCount => (Degree + 1) / 2;

    public int[] AvailableSmoothnessOrders => Enumerable.Range(1, SharedStateCount).ToArray();

    public int Degree
    {
        get => _degree;
        set
        {
            if (!SetProperty(ref _degree, value))
                return;

            // Project default for the new degree: m = p, except degree 9 (snap, m = 4).
            _smoothnessOrder = value == 9 ? 4 : SharedStateCount;
            OnPropertyChanged(nameof(SmoothnessOrder));
            OnPropertyChanged(nameof(AvailableSmoothnessOrders));
            OnPropertyChanged(nameof(SupportsAcceleration));
            OnPropertyChanged(nameof(SupportsJerk));
            OnPropertyChanged(nameof(SupportsSnap));
        }
    }

    public int SmoothnessOrder
    {
        get => _smoothnessOrder;
        set => SetProperty(ref _smoothnessOrder, value);
    }

    // Derivatives of orders 1 .. p-1 can be prescribed at the ends: velocity (d >= 3), acceleration (d >= 5), jerk (d >= 7), snap (d >= 9).
    public bool SupportsAcceleration => Degree >= 5;
    public bool SupportsJerk => Degree >= 7;
    public bool SupportsSnap => Degree >= 9;

    public bool StartAtRest
    {
        get => _startAtRest;
        set { if (SetProperty(ref _startAtRest, value)) OnPropertyChanged(nameof(StartEditable)); }
    }

    public bool EndAtRest
    {
        get => _endAtRest;
        set { if (SetProperty(ref _endAtRest, value)) OnPropertyChanged(nameof(EndEditable)); }
    }

    public bool StartEditable => !StartAtRest;
    public bool EndEditable => !EndAtRest;

    public string StartVelocity { get => _startVelocity; set => SetProperty(ref _startVelocity, value); }
    public string StartAcceleration { get => _startAcceleration; set => SetProperty(ref _startAcceleration, value); }
    public string StartJerk { get => _startJerk; set => SetProperty(ref _startJerk, value); }
    public string StartSnap { get => _startSnap; set => SetProperty(ref _startSnap, value); }
    public string EndVelocity { get => _endVelocity; set => SetProperty(ref _endVelocity, value); }
    public string EndAcceleration { get => _endAcceleration; set => SetProperty(ref _endAcceleration, value); }
    public string EndJerk { get => _endJerk; set => SetProperty(ref _endJerk, value); }
    public string EndSnap { get => _endSnap; set => SetProperty(ref _endSnap, value); }

    /// <summary>Optional total duration; blank = none.</summary>
    public string TotalDuration { get => _totalDuration; set => SetProperty(ref _totalDuration, value); }

    public bool UseMaxVelocity { get => _useMaxVelocity; set => SetProperty(ref _useMaxVelocity, value); }
    public string MaxVelocity { get => _maxVelocity; set => SetProperty(ref _maxVelocity, value); }
    public bool UseMaxAcceleration { get => _useMaxAcceleration; set => SetProperty(ref _useMaxAcceleration, value); }
    public string MaxAcceleration { get => _maxAcceleration; set => SetProperty(ref _maxAcceleration, value); }
    public bool UseMaxJerk { get => _useMaxJerk; set => SetProperty(ref _useMaxJerk, value); }
    public string MaxJerk { get => _maxJerk; set => SetProperty(ref _maxJerk, value); }
    public bool UseMaxSnap { get => _useMaxSnap; set => SetProperty(ref _useMaxSnap, value); }
    public string MaxSnap { get => _maxSnap; set => SetProperty(ref _maxSnap, value); }

    public void Renumber()
    {
        for (int i = 0; i < Waypoints.Count; i++)
        {
            Waypoints[i].Number = i + 1;
            Waypoints[i].HasSegment = i < Waypoints.Count - 1;
        }
    }
}

/// <summary>A waypoint of the pass-through trajectory: where it is reached and the state there.</summary>
public sealed record WaypointStateRow(
    int Waypoint,
    double Time,
    double Position,
    double Velocity,
    double Acceleration,
    double Jerk,
    double Snap);

/// <summary>A labelled line of the pass-through summary (durations, total, λ, cost); the value is already formatted in the current culture.</summary>
public sealed record SummaryRow(string Item, string Value);

/// <summary>Where a waypoint is reached, for the markers on the position plot.</summary>
public sealed record WaypointMarker(double Time, double Position);
