using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Services;

/// <summary>
/// Builds one ordinary polynomial segment and decides its duration (Decision 1 and the project decision of
/// 2026-10-17 on <c>TrajectoryConstraints.md</c>, "Shortest Feasible Duration"):
/// a prescribed duration is only verified against the active limits; without a prescribed duration the shortest
/// feasible duration is used when limits are active; otherwise (no limit active, or the limits determine
/// nothing) the fallback duration.
/// </summary>
public static class SegmentFactory1D
{
    /// <summary>Duration used when it is neither prescribed nor determined by the limits.</summary>
    public const double DefaultDuration = 1.0;

    /// <exception cref="ArgumentException">A non-positive prescribed duration.</exception>
    /// <exception cref="KinematicLimitsViolatedException">A prescribed duration violates an active limit.</exception>
    /// <exception cref="DurationSearchFailedException">No duration satisfies the active limits.</exception>
    public static PolynomialSegment1D Build(
        int degree,
        State1D startState,
        State1D endState,
        KinematicLimits? limits = null,
        double? duration = null,
        double fallbackDuration = DefaultDuration)
    {
        bool limitsActive = limits is not null && limits.HasAnyLimit;

        double chosen = duration
            ?? (limitsActive ? ShortestFeasibleDuration.Find(degree, startState, endState, limits!) : null)
            ?? fallbackDuration;

        var segment = new PolynomialSegment1D(degree, startState, endState, chosen);

        if (duration is not null && limitsActive && !ExtremaAnalyzer.VerifyLimits(segment, limits!))
        {
            throw new KinematicLimitsViolatedException(
                $"The segment violates the active kinematic limits at the specified duration ({chosen:G6} s).");
        }

        return segment;
    }
}
