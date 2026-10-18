using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.App;

/// <summary>One vector quantity of the 3D input: three text boxes X | Y | Z.</summary>
public sealed class VectorInput : ObservableObject
{
    private string _x, _y, _z;

    public VectorInput(string x = "", string y = "", string z = "")
    {
        _x = x;
        _y = y;
        _z = z;
    }

    public string X { get => _x; set => SetProperty(ref _x, value); }
    public string Y { get => _y; set => SetProperty(ref _y, value); }
    public string Z { get => _z; set => SetProperty(ref _z, value); }

    public void CopyFrom(VectorInput other)
    {
        X = other.X;
        Y = other.Y;
        Z = other.Z;
    }
}

/// <summary>How the two ends of a constant-speed stretch are given (the three forms of the Core's <c>StretchSpecification</c>).</summary>
public enum StretchInputKind
{
    Points,
    ParameterFractions,
    ArcLengthFractions
}

/// <summary>How a blend value is given: a fraction of the segment parameter or a point on the segment.</summary>
public enum BlendInputKind
{
    ParameterFractions,
    Points
}

/// <summary>Inputs of one segment of the 3D mode (design/08, "3D Mode").</summary>
public sealed class Segment3DInputModel : ObservableObject
{
    private int _segmentNumber;
    private int _degree = 9;
    private string _duration = string.Empty;
    private bool _useMaxVelocity, _useMaxAcceleration, _useMaxJerk, _useMaxSnap;
    private string _maxVelocity = "10", _maxAcceleration = "10", _maxJerk = "10", _maxSnap = "10";
    private bool _useConstantSpeed;
    private string _speed = "1";
    private StretchInputKind _stretchKind = StretchInputKind.ParameterFractions;
    private string _stretchStartFraction = "0.3", _stretchEndFraction = "0.7";
    private string _entryDuration = string.Empty, _exitDuration = string.Empty, _stretchTotalDuration = string.Empty;
    private BlendInputKind _blendKind = BlendInputKind.ParameterFractions;
    private string _blendToPrevious = string.Empty, _blendToNext = string.Empty;

    public Segment3DInputModel(int segmentNumber)
    {
        _segmentNumber = segmentNumber;
        EndPosition = new VectorInput("1", "1", "0");
    }

    public int SegmentNumber
    {
        get => _segmentNumber;
        set { if (SetProperty(ref _segmentNumber, value)) OnPropertyChanged(nameof(Header)); }
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

    // Boundary conditions: positions are required; empty derivative boxes mean zero.
    public VectorInput StartPosition { get; } = new("0", "0", "0");
    public VectorInput EndPosition { get; }
    public VectorInput StartVelocity { get; } = new();
    public VectorInput EndVelocity { get; } = new();
    public VectorInput StartAcceleration { get; } = new();
    public VectorInput EndAcceleration { get; } = new();
    public VectorInput StartJerk { get; } = new();
    public VectorInput EndJerk { get; } = new();
    public VectorInput StartSnap { get; } = new();
    public VectorInput EndSnap { get; } = new();

    /// <summary>Optional; blank: shortest feasible with limits, 1 s without (Core rule).</summary>
    public string Duration { get => _duration; set => SetProperty(ref _duration, value); }

    public bool UseMaxVelocity { get => _useMaxVelocity; set => SetProperty(ref _useMaxVelocity, value); }
    public string MaxVelocity { get => _maxVelocity; set => SetProperty(ref _maxVelocity, value); }
    public bool UseMaxAcceleration { get => _useMaxAcceleration; set => SetProperty(ref _useMaxAcceleration, value); }
    public string MaxAcceleration { get => _maxAcceleration; set => SetProperty(ref _maxAcceleration, value); }
    public bool UseMaxJerk { get => _useMaxJerk; set => SetProperty(ref _useMaxJerk, value); }
    public string MaxJerk { get => _maxJerk; set => SetProperty(ref _maxJerk, value); }
    public bool UseMaxSnap { get => _useMaxSnap; set => SetProperty(ref _useMaxSnap, value); }
    public string MaxSnap { get => _maxSnap; set => SetProperty(ref _maxSnap, value); }

    // Constant-speed stretch.
    public bool UseConstantSpeed { get => _useConstantSpeed; set => SetProperty(ref _useConstantSpeed, value); }
    public string Speed { get => _speed; set => SetProperty(ref _speed, value); }

    public StretchInputKind StretchKind
    {
        get => _stretchKind;
        set
        {
            if (SetProperty(ref _stretchKind, value))
            {
                OnPropertyChanged(nameof(StretchByPoints));
                OnPropertyChanged(nameof(StretchByFractions));
            }
        }
    }

    public bool StretchByPoints => StretchKind == StretchInputKind.Points;
    public bool StretchByFractions => !StretchByPoints;

    public StretchInputKind[] StretchKinds { get; } = Enum.GetValues<StretchInputKind>();

    public VectorInput StretchStartPoint { get; } = new();
    public VectorInput StretchEndPoint { get; } = new();
    public string StretchStartFraction { get => _stretchStartFraction; set => SetProperty(ref _stretchStartFraction, value); }
    public string StretchEndFraction { get => _stretchEndFraction; set => SetProperty(ref _stretchEndFraction, value); }
    public string EntryDuration { get => _entryDuration; set => SetProperty(ref _entryDuration, value); }
    public string ExitDuration { get => _exitDuration; set => SetProperty(ref _exitDuration, value); }
    public string StretchTotalDuration { get => _stretchTotalDuration; set => SetProperty(ref _stretchTotalDuration, value); }

    // Blend toward the neighbours.
    public BlendInputKind BlendKind
    {
        get => _blendKind;
        set
        {
            if (SetProperty(ref _blendKind, value))
            {
                OnPropertyChanged(nameof(BlendByPoints));
                OnPropertyChanged(nameof(BlendByFractions));
            }
        }
    }

    public bool BlendByPoints => BlendKind == BlendInputKind.Points;
    public bool BlendByFractions => !BlendByPoints;

    public BlendInputKind[] BlendKinds { get; } = Enum.GetValues<BlendInputKind>();

    /// <summary>Fraction of this segment where the blend with the previous segment ends (blank: no blend).</summary>
    public string BlendToPrevious { get => _blendToPrevious; set => SetProperty(ref _blendToPrevious, value); }

    /// <summary>Fraction of this segment where the blend with the next segment starts (blank: no blend).</summary>
    public string BlendToNext { get => _blendToNext; set => SetProperty(ref _blendToNext, value); }

    public VectorInput BlendToPreviousPoint { get; } = new();
    public VectorInput BlendToNextPoint { get; } = new();

    /// <summary>True if the user asked for a blend toward the previous segment (any box filled).</summary>
    public bool HasBlendToPrevious => BlendByPoints ? IsFilled(BlendToPreviousPoint) : BlendToPrevious.Trim().Length > 0;

    public bool HasBlendToNext => BlendByPoints ? IsFilled(BlendToNextPoint) : BlendToNext.Trim().Length > 0;

    private static bool IsFilled(VectorInput v) =>
        v.X.Trim().Length > 0 || v.Y.Trim().Length > 0 || v.Z.Trim().Length > 0;
}

// ---- results of the 3D mode -------------------------------------------------------------------

/// <summary>A plotted quantity: the three components and the magnitude at the sample times.</summary>
public sealed record Series3D(string Name, int Order, double[] Times, double[] X, double[] Y, double[] Z, double[] Norm);

/// <summary>The path of one part (in the samples), for the projections: coloured by kind.</summary>
public sealed record PathPiece(string Label, string Kind, double[] X, double[] Y, double[] Z);

/// <summary>A row of the parts table.</summary>
public sealed record PartReportRow(
    string Part, string Kind, double Start, double Duration,
    double? InheritedDuration, string Adjusted, double? Length, double? Speed, double? AdmissibleSpeed, string BindingLimit);

/// <summary>A row of the 3D extrema table: one quantity on one axis over the whole trajectory.</summary>
public sealed record Extrema3DReportRow(
    string Quantity, string Axis, double Minimum, double MinimumTime, double Maximum, double MaximumTime,
    double MaxAbsolute, double MaxAbsoluteTime, string Part);

/// <summary>A row of the peaks-versus-limits table (the Core's limit report, for the pieces a set of limits applies to).</summary>
public sealed record Limit3DReportRow(
    string LimitsOf, string Quantity, string Axis, double Peak, double Limit, double PercentOfLimit, string Part, double Time);

/// <summary>A row of the 3D cursor table.</summary>
public sealed class Cursor3DRow : ObservableObject
{
    private double _x, _y, _z, _norm;

    public Cursor3DRow(string quantity) => Quantity = quantity;

    public string Quantity { get; }
    public double X { get => _x; set => SetProperty(ref _x, value); }
    public double Y { get => _y; set => SetProperty(ref _y, value); }
    public double Z { get => _z; set => SetProperty(ref _z, value); }
    public double Norm { get => _norm; set => SetProperty(ref _norm, value); }
}

/// <summary>Everything the window shows for a 3D computation.</summary>
public sealed record Prototype3DComputationResult(
    IReadOnlyList<Series3D> Series,
    IReadOnlyList<PathPiece> PathPieces,
    IReadOnlyList<Point3D> SegmentEndpoints,
    IReadOnlyList<double> PartBoundaries,
    IReadOnlyList<PartReportRow> PartRows,
    IReadOnlyList<Extrema3DReportRow> ExtremaRows,
    IReadOnlyList<Limit3DReportRow> LimitRows,
    IReadOnlyList<CoefficientReportRow> CoefficientRows,
    IReadOnlyList<SummaryRow> SummaryRows,
    Func<double, State3D> StateAt,
    double TotalDuration,
    double PathLength,
    IReadOnlyList<string> Notices);
