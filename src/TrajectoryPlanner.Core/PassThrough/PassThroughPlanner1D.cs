using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>Outcome of <see cref="PassThroughPlanner1D.Plan"/>.</summary>
/// <param name="Segments">The solved segments.</param>
/// <param name="Durations">Final segment durations.</param>
/// <param name="Scale">
/// λ, the common factor applied to the reference durations (1 when durations were prescribed or the limits
/// determined nothing).
/// </param>
/// <param name="ScaledByLimits">True if the active limits determined the durations (λ was computed).</param>
public sealed record PassThroughResult(
    PolynomialSegment1D[] Segments,
    double[] Durations,
    double Scale,
    bool ScaledByLimits)
{
    public double TotalDuration => Durations.Sum();
}

/// <summary>
/// Exact pass-through with kinematic limits: <c>theory/TrajectoryPassingThroughConstraintPoints.md</c>,
/// "Pass-Through With Kinematic Limits". The durations are T_k = λ·T̄_k (reference durations T̄_k, one free
/// scalar λ); the shortest feasible sequence is the smallest λ for which the globally optimal trajectory
/// respects every active limit.
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item>Prescribed durations: kept, solved, verified; a violation is reported with the segment index.</item>
/// <item>Automatic, all prescribed boundary derivatives zero: scaling law P_k(λ) = λ^(-k) P_k(1), hence
/// λ* = max_k (P_k(1)/L_k)^(1/k) from one solve; the trajectory is then solved again at λ*·T̄ (check of the law).</item>
/// <item>Automatic, non-zero boundary derivatives: the law is lost; search on λ (geometric steps, bisection),
/// each evaluation one solve, starting from the λ* of the zero-boundary problem.</item>
/// <item>Limits that determine nothing (none active, or every peak zero): the reference durations, unscaled.</item>
/// </list>
/// </remarks>
public static class PassThroughPlanner1D
{
    /// <summary>Default reference duration per segment: the A8 proportional rule is applied to a total of (number of segments) x this value (D23).</summary>
    public const double ReferenceDurationPerSegment = 1.0;

    /// <summary>Geometric ratio of the λ search (non-zero boundary derivatives).</summary>
    public const double GrowthRatio = 1.05;

    /// <summary>The λ search covers [λ0 / SearchRange, λ0 · SearchRange].</summary>
    public const double SearchRange = 1e3;

    /// <param name="waypoints">At least two positions.</param>
    /// <param name="degree">3, 5, 7 or 9.</param>
    /// <param name="criterion">Smoothness order m; null selects the default for the degree.</param>
    /// <param name="limits">Active limits (null or none active: no limit).</param>
    /// <param name="prescribedDurations">One duration per segment (kept and verified), or null for automatic durations.</param>
    /// <param name="referenceDurations">
    /// Automatic durations only: the proportions T̄_k to scale (any positive total); null: the proportional rule of
    /// design/07 A8 with a total of one second per segment. Not allowed together with prescribed durations.
    /// </param>
    /// <param name="startBoundary">Prescribed derivatives (orders 1 .. p-1) at the start; null: rest.</param>
    /// <param name="endBoundary">Same at the end.</param>
    /// <exception cref="KinematicLimitsViolatedException">Prescribed durations violate a limit (message names the segment and the limit).</exception>
    /// <exception cref="DurationSearchFailedException">No scale λ satisfies the limits.</exception>
    public static PassThroughResult Plan(
        IReadOnlyList<double> waypoints,
        int degree,
        SmoothnessCriterion? criterion = null,
        KinematicLimits? limits = null,
        double[]? prescribedDurations = null,
        double[]? referenceDurations = null,
        State1D? startBoundary = null,
        State1D? endBoundary = null)
    {
        double[] wp = waypoints.ToArray();

        if (prescribedDurations is not null && referenceDurations is not null)
            throw new ArgumentException("Prescribed durations and reference durations are mutually exclusive.");

        if (prescribedDurations is not null && prescribedDurations.Length != wp.Length - 1)
            throw new ArgumentException($"Expected {wp.Length - 1} durations, got {prescribedDurations.Length}.", nameof(prescribedDurations));

        double[]? reference = null;
        if (prescribedDurations is null)
        {
            if (referenceDurations is not null && referenceDurations.Length != wp.Length - 1)
                throw new ArgumentException($"Expected {wp.Length - 1} reference durations, got {referenceDurations.Length}.", nameof(referenceDurations));
            if (referenceDurations is not null && referenceDurations.Any(t => !(t > 0)))
                throw new ArgumentException("Reference durations must be positive.", nameof(referenceDurations));

            reference = referenceDurations is not null
                ? (double[])referenceDurations.Clone()
                : new AutoDerivedProportionalDuration((wp.Length - 1) * ReferenceDurationPerSegment).ComputeDurations(wp, degree);
        }

        // The algorithm is shared with 3D (one component here).
        var outcome = PassThroughEngine.Run(
            new[] { wp }, degree, criterion, limits, prescribedDurations, reference,
            new[] { startBoundary }, new[] { endBoundary }, componentNames: null);

        return new PassThroughResult(outcome.Segments[0], outcome.Durations, outcome.Scale, outcome.ScaledByLimits);
    }
}
