namespace TrajectoryPlanner.Core.Constraints;

/// <summary>
/// Thrown when no duration satisfying the active kinematic limits was found within the search range.
/// An exception (rather than a flag) so that a caller cannot silently use a duration that violates the limits.
/// </summary>
public class DurationSearchFailedException : InvalidOperationException
{
    /// <summary>Longest duration that was tried.</summary>
    public double LastDurationTried { get; }

    public DurationSearchFailedException(string message, double lastDurationTried)
        : base(message)
    {
        LastDurationTried = lastDurationTried;
    }
}
