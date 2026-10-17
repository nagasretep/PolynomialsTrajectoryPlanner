namespace TrajectoryPlanner.Core.Constraints;

/// <summary>
/// Thrown when a piece of a trajectory that cannot be repaired (a prescribed duration, a prescribed
/// constant speed) violates the active kinematic limits. Such a trajectory is never returned.
/// </summary>
public class KinematicLimitsViolatedException : InvalidOperationException
{
    public KinematicLimitsViolatedException(string message)
        : base(message)
    {
    }
}
