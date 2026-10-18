using System.Collections.ObjectModel;
using TrajectoryPlanner.Core.Points;

namespace TrajectoryPlanner.App;

/// <summary>One row of the 3D pass-through waypoint table: X, Y, Z and the optional duration of the segment that starts there.</summary>
public sealed class Waypoint3DInputModel : ObservableObject
{
    private int _number;
    private bool _hasSegment = true;
    private string _x, _y, _z;
    private string _duration = string.Empty;

    public Waypoint3DInputModel(int number, string x, string y, string z)
    {
        _number = number;
        _x = x;
        _y = y;
        _z = z;
    }

    public int Number { get => _number; set => SetProperty(ref _number, value); }

    /// <summary>False for the last waypoint: no segment starts there, so its duration cell is not used.</summary>
    public bool HasSegment { get => _hasSegment; set => SetProperty(ref _hasSegment, value); }

    public string X { get => _x; set => SetProperty(ref _x, value); }
    public string Y { get => _y; set => SetProperty(ref _y, value); }
    public string Z { get => _z; set => SetProperty(ref _z, value); }
    public string Duration { get => _duration; set => SetProperty(ref _duration, value); }
}

/// <summary>Inputs of the 3D pass-through mode (request 0020): the 1D pass-through inputs with vectors.</summary>
public sealed class PassThrough3DInputModel : ObservableObject
{
    private int _degree = 9;
    private int _smoothnessOrder = 4;
    private bool _startAtRest = true, _endAtRest = true;
    private string _totalDuration = string.Empty;
    private bool _useMaxVelocity, _useMaxAcceleration, _useMaxJerk, _useMaxSnap;
    private string _maxVelocity = "2", _maxAcceleration = "3", _maxJerk = "10", _maxSnap = "50";

    public PassThrough3DInputModel()
    {
        // The reference example of the theory (Common Durations And Kinematic Limits In 3D).
        Waypoints.Add(new Waypoint3DInputModel(1, "0", "0", "0"));
        Waypoints.Add(new Waypoint3DInputModel(2, "1", "0", "0"));
        Waypoints.Add(new Waypoint3DInputModel(3, "1", "3", "0"));
        Waypoints.Add(new Waypoint3DInputModel(4, "1", "3", "4"));
        Renumber();
        Waypoints.CollectionChanged += (_, _) => Renumber();
    }

    public ObservableCollection<Waypoint3DInputModel> Waypoints { get; } = [];

    public int SharedStateCount => (Degree + 1) / 2;

    public int[] AvailableSmoothnessOrders => Enumerable.Range(1, SharedStateCount).ToArray();

    public int Degree
    {
        get => _degree;
        set
        {
            if (!SetProperty(ref _degree, value))
                return;

            // Project default for the new degree: m = p, except degree 9 (snap, m = 4) — as in 1D.
            _smoothnessOrder = value == 9 ? 4 : SharedStateCount;
            OnPropertyChanged(nameof(SmoothnessOrder));
            OnPropertyChanged(nameof(AvailableSmoothnessOrders));
            OnPropertyChanged(nameof(SupportsAcceleration));
            OnPropertyChanged(nameof(SupportsJerk));
            OnPropertyChanged(nameof(SupportsSnap));
        }
    }

    public int SmoothnessOrder { get => _smoothnessOrder; set => SetProperty(ref _smoothnessOrder, value); }

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

    // End conditions as vectors (empty box = 0).
    public VectorInput StartVelocity { get; } = new();
    public VectorInput StartAcceleration { get; } = new();
    public VectorInput StartJerk { get; } = new();
    public VectorInput StartSnap { get; } = new();
    public VectorInput EndVelocity { get; } = new();
    public VectorInput EndAcceleration { get; } = new();
    public VectorInput EndJerk { get; } = new();
    public VectorInput EndSnap { get; } = new();

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

/// <summary>A waypoint on the time axis and in space, for the markers of the plots.</summary>
public sealed record WaypointMarker3D(int Number, double Time, Point3D Point);

/// <summary>One quantity of the state at a waypoint (position, velocity … as the degree uses), as a vector.</summary>
public sealed record WaypointState3DRow(int Waypoint, double Time, string Quantity, double X, double Y, double Z);
