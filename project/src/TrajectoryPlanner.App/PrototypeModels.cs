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
        SegmentNumber = segmentNumber;
    }

    public int SegmentNumber { get; }

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
    double TotalDuration);
