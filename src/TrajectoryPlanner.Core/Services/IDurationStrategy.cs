using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Services;

public enum DurationMode
{
    UserSpecified,
    AutoDerivedProportional,
    AutoDerivedFromConstraints
}

/// <summary>
/// The single duration mechanism (Decision 1 of <c>design/07-implementation-plan.md</c>).
/// </summary>
public interface IDurationStrategy1D
{
    DurationMode Mode { get; }

    /// <summary>Duration of one segment whose two end states are known.</summary>
    double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments);

    /// <summary>
    /// Durations of the N - 1 segments joining a sequence of N waypoints (exact pass-through, where
    /// the interior states are not known before the solve).
    /// </summary>
    double[] ComputeDurations(IReadOnlyList<double> waypoints, int degree);
}

public class UserSpecifiedDuration : IDurationStrategy1D
{
    private readonly double[] _durations;

    public UserSpecifiedDuration(double[] durations)
    {
        _durations = durations;
    }

    public DurationMode Mode => DurationMode.UserSpecified;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return _durations[segmentIndex];
    }

    public double[] ComputeDurations(IReadOnlyList<double> waypoints, int degree)
    {
        if (_durations.Length != waypoints.Count - 1)
            throw new ArgumentException($"Expected {waypoints.Count - 1} durations, got {_durations.Length}.");

        return (double[])_durations.Clone();
    }
}

/// <summary>
/// Total time split proportionally to the distance between consecutive waypoints (Decision 1).
/// Each segment gets at least <see cref="MinSegmentDuration"/>, so equal consecutive waypoints do
/// not give a zero duration; when the floor is active the sum can exceed the requested total.
/// If all waypoints coincide the total is split equally.
/// </summary>
public class AutoDerivedProportionalDuration : IDurationStrategy1D
{
    public const double DefaultMinSegmentDuration = 0.05;

    private readonly double _totalDuration;

    public double MinSegmentDuration { get; }

    public AutoDerivedProportionalDuration(double totalDuration, double minSegmentDuration = DefaultMinSegmentDuration)
    {
        if (!(totalDuration > 0))
            throw new ArgumentOutOfRangeException(nameof(totalDuration), "Total duration must be positive.");
        if (!(minSegmentDuration > 0))
            throw new ArgumentOutOfRangeException(nameof(minSegmentDuration), "Minimum segment duration must be positive.");

        _totalDuration = totalDuration;
        MinSegmentDuration = minSegmentDuration;
    }

    public DurationMode Mode => DurationMode.AutoDerivedProportional;

    /// <summary>Per-segment form (unchanged legacy behaviour): equal split, since one segment alone knows no distances.</summary>
    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return _totalDuration / totalSegments;
    }

    public double[] ComputeDurations(IReadOnlyList<double> waypoints, int degree)
    {
        var lengths = new double[waypoints.Count - 1];
        for (int k = 0; k < lengths.Length; k++)
            lengths[k] = Math.Abs(waypoints[k + 1] - waypoints[k]);

        return SplitProportionally(lengths);
    }

    public double[] ComputeDurations(IReadOnlyList<Point3D> waypoints)
    {
        var lengths = new double[waypoints.Count - 1];
        for (int k = 0; k < lengths.Length; k++)
            lengths[k] = waypoints[k].DistanceTo(waypoints[k + 1]);

        return SplitProportionally(lengths);
    }

    private double[] SplitProportionally(double[] lengths)
    {
        double totalLength = lengths.Sum();
        var durations = new double[lengths.Length];
        for (int k = 0; k < lengths.Length; k++)
        {
            durations[k] = totalLength > 1e-12
                ? Math.Max(MinSegmentDuration, _totalDuration * lengths[k] / totalLength)
                : Math.Max(MinSegmentDuration, _totalDuration / lengths.Length);
        }

        return durations;
    }
}

public class ConstraintAwareDuration : IDurationStrategy1D
{
    private readonly KinematicLimits _limits;
    private readonly double _initialGuess;

    public ConstraintAwareDuration(KinematicLimits limits, double initialGuess = 1.0)
    {
        _limits = limits;
        _initialGuess = initialGuess;
    }

    public DurationMode Mode => DurationMode.AutoDerivedFromConstraints;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return DurationSearcher.FindValidDuration(degree, startState, endState, _limits, _initialGuess);
    }

    public double[] ComputeDurations(IReadOnlyList<double> waypoints, int degree)
    {
        throw new NotSupportedException(
            "Constraint-aware durations need the segment end states, which exact pass-through determines only after the durations are fixed.");
    }
}
