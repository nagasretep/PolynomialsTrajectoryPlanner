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
/// Total time split proportionally to the distance between consecutive waypoints (Decision 1, rule of
/// <c>design/07</c> A8): weights w_i = max(d_i, phi * mean(d)), durations T_i = T_total * w_i / sum(w).
/// The sum of the durations equals the requested total; a zero-distance segment gets the floor share
/// (phi * mean(d)) instead of a zero duration. If all distances are zero the split is equal.
/// </summary>
public class AutoDerivedProportionalDuration : IDurationStrategy1D
{
    public const double DefaultRelativeFloor = 0.05;

    private readonly double _totalDuration;

    /// <summary>phi: floor of a segment weight, as a fraction of the mean distance.</summary>
    public double RelativeFloor { get; }

    public AutoDerivedProportionalDuration(double totalDuration, double relativeFloor = DefaultRelativeFloor)
    {
        if (!(totalDuration > 0))
            throw new ArgumentOutOfRangeException(nameof(totalDuration), "Total duration must be positive.");
        if (!(relativeFloor >= 0))
            throw new ArgumentOutOfRangeException(nameof(relativeFloor), "Relative floor must not be negative.");

        _totalDuration = totalDuration;
        RelativeFloor = relativeFloor;
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
        var durations = new double[lengths.Length];
        double mean = lengths.Sum() / lengths.Length;

        if (mean <= 1e-12)
        {
            Array.Fill(durations, _totalDuration / lengths.Length);
            return durations;
        }

        double floor = RelativeFloor * mean;
        double weightSum = 0;
        for (int k = 0; k < lengths.Length; k++)
        {
            durations[k] = Math.Max(lengths[k], floor); // weight w_k
            weightSum += durations[k];
        }

        for (int k = 0; k < durations.Length; k++)
            durations[k] = _totalDuration * durations[k] / weightSum;

        return durations;
    }
}

/// <summary>
/// Shortest duration that satisfies all active limits (project decision of 2026-10-17, design/07 A9).
/// When the limits determine no duration (no active limit, or a rest-to-rest segment of zero length) the
/// fallback duration is used instead.
/// </summary>
public class ConstraintAwareDuration : IDurationStrategy1D
{
    private readonly KinematicLimits _limits;

    public double FallbackDuration { get; }

    public ConstraintAwareDuration(KinematicLimits limits, double fallbackDuration = 1.0)
    {
        if (!(fallbackDuration > 0))
            throw new ArgumentOutOfRangeException(nameof(fallbackDuration), "Fallback duration must be positive.");

        _limits = limits;
        FallbackDuration = fallbackDuration;
    }

    public DurationMode Mode => DurationMode.AutoDerivedFromConstraints;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return ShortestFeasibleDuration.Find(degree, startState, endState, _limits) ?? FallbackDuration;
    }

    public double[] ComputeDurations(IReadOnlyList<double> waypoints, int degree)
    {
        throw new NotSupportedException(
            "Constraint-aware durations need the segment end states, which exact pass-through determines only after the durations are fixed.");
    }
}
