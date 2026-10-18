using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>Outcome of <see cref="PassThroughPlanner3D.Plan"/>.</summary>
/// <param name="Segments">The solved 3D segments.</param>
/// <param name="Durations">Final segment durations (common to the three axes).</param>
/// <param name="ReferenceDurations">The reference durations T̄_k that λ scales; null when the durations were prescribed.</param>
/// <param name="Scale">λ when it was computed (the limits determined the durations); null otherwise.</param>
/// <param name="ScaledByLimits">True if the active limits determined the durations.</param>
/// <param name="BindingQuantity">"velocity", "acceleration", "jerk" or "snap": the limit that determines λ (null if not computed).</param>
/// <param name="BindingAxis">"x", "y" or "z": where it binds (null if not computed).</param>
public sealed record PassThroughResult3D(
    PolynomialSegment3D[] Segments,
    double[] Durations,
    double[]? ReferenceDurations,
    double? Scale,
    bool ScaledByLimits,
    string? BindingQuantity,
    string? BindingAxis)
{
    public double TotalDuration => Durations.Sum();
}

/// <summary>
/// Exact pass-through of 3D waypoints with common durations and kinematic limits
/// (<c>theory/TrajectoryPassingThroughConstraintPoints.md</c>, "Common Durations And Kinematic Limits In 3D").
/// For fixed durations the 3D optimum is the three scalar optima of <see cref="PassThroughSolver1D"/> with the same
/// durations; the durations, λ and the limit handling are those of <see cref="PassThroughPlanner1D"/>, with every limit
/// applied to each axis (the same algorithm, <see cref="PassThroughEngine"/>).
/// </summary>
/// <remarks>
/// Reference durations: the proportional rule of design/07 A8 on the <b>Euclidean chords</b> ‖p_k − p_(k−1)‖, with a total of
/// one second per segment; a prescribed total uses the same rule. Along one axis the chords are the 1D increments, so the
/// result equals the 1D planner. Zero chords (consecutive equal waypoints) get the floor share φ·mean(chords) of the rule,
/// all-zero chords an equal split; the limits then determine nothing if every axis is constant.
/// </remarks>
public static class PassThroughPlanner3D
{
    private static readonly string[] AxisNames = { "x", "y", "z" };

    /// <param name="waypoints">At least two points.</param>
    /// <param name="degree">3, 5, 7 or 9.</param>
    /// <param name="criterion">Smoothness order m; null selects the default for the degree.</param>
    /// <param name="limits">One value per quantity, applied to each axis (null or none active: no limit).</param>
    /// <param name="prescribedDurations">One duration per segment, common to the axes (kept and verified), or null.</param>
    /// <param name="totalDuration">A prescribed total, split by the chord rule and only verified; not with prescribed durations.</param>
    /// <param name="referenceDurations">Automatic durations only: the proportions T̄_k to scale; null: the chord rule, 1 s per segment.</param>
    /// <param name="startBoundary">Prescribed derivatives (orders 1 .. p-1) at the start as vectors; null: rest. Its position is ignored (the first waypoint).</param>
    /// <param name="endBoundary">Same at the end.</param>
    /// <exception cref="ArgumentException">Fewer than two waypoints, wrong counts, conflicting duration arguments.</exception>
    /// <exception cref="KinematicLimitsViolatedException">Prescribed durations violate a limit (message names segment, quantity and axis).</exception>
    /// <exception cref="DurationSearchFailedException">No scale λ satisfies the limits.</exception>
    public static PassThroughResult3D Plan(
        IReadOnlyList<Point3D> waypoints,
        int degree,
        SmoothnessCriterion? criterion = null,
        KinematicLimits? limits = null,
        double[]? prescribedDurations = null,
        double? totalDuration = null,
        double[]? referenceDurations = null,
        State3D? startBoundary = null,
        State3D? endBoundary = null)
    {
        if (waypoints.Count < 2)
            throw new ArgumentException("At least two waypoints are required.", nameof(waypoints));

        int segments = waypoints.Count - 1;
        int given = (prescribedDurations is null ? 0 : 1) + (totalDuration is null ? 0 : 1) + (referenceDurations is null ? 0 : 1);
        if (given > 1)
            throw new ArgumentException("Prescribed durations, a prescribed total and reference durations are mutually exclusive.");
        if (prescribedDurations is not null && prescribedDurations.Length != segments)
            throw new ArgumentException($"Expected {segments} durations, got {prescribedDurations.Length}.", nameof(prescribedDurations));
        if (referenceDurations is not null && (referenceDurations.Length != segments || referenceDurations.Any(t => !(t > 0))))
            throw new ArgumentException($"Expected {segments} positive reference durations.", nameof(referenceDurations));

        double[]? prescribed = prescribedDurations is not null
            ? (double[])prescribedDurations.Clone()
            : totalDuration is not null
                ? new AutoDerivedProportionalDuration(totalDuration.Value).ComputeDurations(waypoints)
                : null;

        double[]? reference = prescribed is not null
            ? null
            : referenceDurations is not null
                ? (double[])referenceDurations.Clone()
                : new AutoDerivedProportionalDuration(segments * PassThroughPlanner1D.ReferenceDurationPerSegment).ComputeDurations(waypoints);

        var components = new double[3][];
        var starts = new State1D?[3];
        var ends = new State1D?[3];
        for (int axis = 0; axis < 3; axis++)
        {
            components[axis] = waypoints.Select(p => p[axis]).ToArray();
            starts[axis] = startBoundary?.GetComponent(axis);
            ends[axis] = endBoundary?.GetComponent(axis);
        }

        var outcome = PassThroughEngine.Run(components, degree, criterion, limits, prescribed, reference, starts, ends, AxisNames);

        var result = new PolynomialSegment3D[segments];
        for (int k = 0; k < segments; k++)
        {
            result[k] = new PolynomialSegment3D(
                degree,
                Combine(outcome.Segments, k, start: true),
                Combine(outcome.Segments, k, start: false),
                outcome.Durations[k]);
        }

        return new PassThroughResult3D(
            result,
            outcome.Durations,
            reference,
            outcome.ScaledByLimits ? outcome.Scale : null,
            outcome.ScaledByLimits,
            outcome.ScaledByLimits ? PassThroughEngine.KindName(outcome.BindingOrder) : null,
            outcome.ScaledByLimits ? AxisNames[outcome.BindingComponent] : null);
    }

    /// <summary>The vector state at the start or end of segment k from the three scalar solutions.</summary>
    private static State3D Combine(PolynomialSegment1D[][] axes, int k, bool start)
    {
        State1D x = start ? axes[0][k].StartState : axes[0][k].EndState;
        State1D y = start ? axes[1][k].StartState : axes[1][k].EndState;
        State1D z = start ? axes[2][k].StartState : axes[2][k].EndState;
        return new State3D(
            new Point3D(x.Position, y.Position, z.Position),
            new Point3D(x.Velocity, y.Velocity, z.Velocity),
            new Point3D(x.Acceleration, y.Acceleration, z.Acceleration),
            new Point3D(x.Jerk, y.Jerk, z.Jerk),
            new Point3D(x.Snap, y.Snap, z.Snap));
    }
}
