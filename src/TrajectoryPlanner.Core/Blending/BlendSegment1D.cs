using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Blending;

/// <summary>
/// A blend segment (an ordinary polynomial segment) that also records how its duration was chosen, so that a
/// caller can tell the user when the limits changed it.
/// </summary>
public sealed class BlendSegment1D : PolynomialSegment1D
{
    /// <summary>Duration inherited from the removed portions of the neighbours (with its floor).</summary>
    public double InheritedDuration { get; }

    /// <summary>
    /// True if the active limits forced a duration different from the inherited one (the inherited blend violated
    /// them). False for a prescribed duration, without limits, or when the inherited duration was kept.
    /// </summary>
    public bool AdjustedByLimits { get; }

    public BlendSegment1D(
        int degree, State1D startState, State1D endState, double duration,
        double inheritedDuration, bool adjustedByLimits)
        : base(degree, startState, endState, duration)
    {
        InheritedDuration = inheritedDuration;
        AdjustedByLimits = adjustedByLimits;
    }
}
