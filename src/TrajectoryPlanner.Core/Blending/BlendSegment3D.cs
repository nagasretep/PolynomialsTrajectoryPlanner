using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Blending;

/// <summary>3D counterpart of <see cref="BlendSegment1D"/>: a blend that records how its (common) duration was chosen.</summary>
public sealed class BlendSegment3D : PolynomialSegment3D
{
    /// <summary>Duration inherited from the removed portions of the neighbours (with its floor).</summary>
    public double InheritedDuration { get; }

    /// <summary>True if the active limits forced a duration different from the inherited one.</summary>
    public bool AdjustedByLimits { get; }

    public BlendSegment3D(
        int degree, State3D startState, State3D endState, double duration,
        double inheritedDuration, bool adjustedByLimits)
        : base(degree, startState, endState, duration)
    {
        InheritedDuration = inheritedDuration;
        AdjustedByLimits = adjustedByLimits;
    }
}
