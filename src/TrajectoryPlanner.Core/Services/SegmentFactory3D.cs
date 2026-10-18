using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Services;

/// <summary>
/// 3D counterpart of <see cref="SegmentFactory1D"/>: one ordinary polynomial segment and its duration. A prescribed
/// duration is only verified against the active limits (per axis); without one the shortest feasible duration is used
/// when limits are active (the three axes share it: multi-component <see cref="ShortestFeasibleDuration"/>), otherwise
/// the fallback duration, the same <see cref="SegmentFactory1D.DefaultDuration"/> as in 1D.
/// </summary>
public static class SegmentFactory3D
{
    /// <exception cref="ArgumentException">A non-positive prescribed duration.</exception>
    /// <exception cref="KinematicLimitsViolatedException">A prescribed duration violates an active limit.</exception>
    /// <exception cref="DurationSearchFailedException">No duration satisfies the active limits.</exception>
    public static PolynomialSegment3D Build(
        int degree,
        State3D startState,
        State3D endState,
        KinematicLimits? limits = null,
        double? duration = null,
        double fallbackDuration = SegmentFactory1D.DefaultDuration)
    {
        bool limitsActive = limits is not null && limits.HasAnyLimit;

        double chosen = duration
            ?? (limitsActive
                ? ShortestFeasibleDuration.Find(degree, BlendBuilder3D.Components(startState), BlendBuilder3D.Components(endState), limits!)
                : null)
            ?? fallbackDuration;

        var segment = new PolynomialSegment3D(degree, startState, endState, chosen);

        if (duration is not null && limitsActive && !ExtremaAnalyzer.VerifyLimits(segment, limits!))
        {
            throw new KinematicLimitsViolatedException(
                $"The segment violates the active kinematic limits at the specified duration ({chosen:G6} s).");
        }

        return segment;
    }
}
