using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Services;

/// <summary>
/// The single duration mechanism (Decision 1 of <c>design/07-implementation-plan.md</c>).
/// </summary>
public interface IDurationStrategy1D
{
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
