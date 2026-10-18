using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Segments;

/// <summary>
/// A piece of a 3D trajectory, in its own time t ∈ [0, <see cref="Duration"/>]. Implemented by the polynomial segment
/// and by the curved constant-speed stretch (which is not a polynomial), so that a trajectory can hold both.
/// Derivatives of an order the piece does not support evaluate as they do for the piece itself (zero for a polynomial).
/// </summary>
public interface ISegment3D
{
    double Duration { get; }

    State3D EvaluateState(double t);

    Point3D EvaluatePosition(double t);

    Point3D EvaluateDerivative(double t, int order);
}
