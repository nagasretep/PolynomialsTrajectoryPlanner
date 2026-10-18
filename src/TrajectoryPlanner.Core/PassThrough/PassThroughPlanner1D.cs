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
    /// <summary>Total of the reference durations (the proportional rule of design/07 A8 is applied to it).</summary>
    public const double ReferenceTotal = 1.0;

    /// <summary>Geometric ratio of the λ search (non-zero boundary derivatives).</summary>
    public const double GrowthRatio = 1.05;

    /// <summary>The λ search covers [λ0 / SearchRange, λ0 · SearchRange].</summary>
    public const double SearchRange = 1e3;

    /// <summary>Relative step used to repair a result that fails the verification by rounding only.</summary>
    private const double SafetyStep = 1e-9;

    private const int MaxSafetySteps = 5;

    /// <param name="waypoints">At least two positions.</param>
    /// <param name="degree">3, 5, 7 or 9.</param>
    /// <param name="criterion">Smoothness order m; null selects the default for the degree.</param>
    /// <param name="limits">Active limits (null or none active: no limit).</param>
    /// <param name="prescribedDurations">One duration per segment (kept and verified), or null for automatic durations.</param>
    /// <param name="referenceDurations">
    /// Automatic durations only: the proportions T̄_k to scale (any positive total); null: the proportional rule of
    /// design/07 A8 with total 1 s. Not allowed together with prescribed durations.
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
        var solver = new PassThroughSolver1D(criterion);
        double[] wp = waypoints.ToArray();
        bool active = limits is not null && limits.HasAnyLimit;

        PolynomialSegment1D[] Solve(double[] durations, bool withBoundary = true) =>
            solver.Solve(wp, durations, degree, withBoundary ? startBoundary : null, withBoundary ? endBoundary : null);

        if (prescribedDurations is not null && referenceDurations is not null)
            throw new ArgumentException("Prescribed durations and reference durations are mutually exclusive.");

        if (prescribedDurations is not null)
        {
            if (prescribedDurations.Length != wp.Length - 1)
                throw new ArgumentException($"Expected {wp.Length - 1} durations, got {prescribedDurations.Length}.", nameof(prescribedDurations));

            var segments = Solve(prescribedDurations);
            if (active)
                VerifyOrThrow(segments, limits!);

            return new PassThroughResult(segments, (double[])prescribedDurations.Clone(), 1.0, false);
        }

        if (referenceDurations is not null && referenceDurations.Length != wp.Length - 1)
            throw new ArgumentException($"Expected {wp.Length - 1} reference durations, got {referenceDurations.Length}.", nameof(referenceDurations));
        if (referenceDurations is not null && referenceDurations.Any(t => !(t > 0)))
            throw new ArgumentException("Reference durations must be positive.", nameof(referenceDurations));

        double[] reference = referenceDurations is not null
            ? (double[])referenceDurations.Clone()
            : new AutoDerivedProportionalDuration(ReferenceTotal).ComputeDurations(wp, degree);

        if (!active)
            return new PassThroughResult(Solve(reference), reference, 1.0, false);

        // Peaks of the reference solution with the boundary derivatives set to zero: the closed form's input.
        bool zeroBoundary = IsRest(degree, startBoundary) && IsRest(degree, endBoundary);
        var restReference = zeroBoundary ? Solve(reference) : Solve(reference, withBoundary: false);
        double? lambdaClosedForm = ClosedFormScale(restReference, limits!);

        if (lambdaClosedForm is null)
            return new PassThroughResult(Solve(reference), reference, 1.0, false); // the limits determine nothing

        double lambda;
        if (zeroBoundary)
        {
            lambda = lambdaClosedForm.Value;
        }
        else
        {
            lambda = SearchScale(lambdaClosedForm.Value, scale => Feasibility(Solve(Scale(reference, scale)), limits!));
        }

        // Second solve at the scaled durations (for zero boundary derivatives it doubles as a check of the scaling law).
        PolynomialSegment1D[] result = Solve(Scale(reference, lambda));
        for (int attempt = 0; attempt < MaxSafetySteps && !VerifiesLimits(result, limits!); attempt++)
        {
            lambda *= 1 + SafetyStep; // the closed form lands exactly on the limit: rounding may exceed it by 1e-10
            result = Solve(Scale(reference, lambda));
        }

        VerifyOrThrow(result, limits!);
        return new PassThroughResult(result, Scale(reference, lambda), lambda, true);
    }

    // ---------------------------------------------------------------------------------------------

    private static bool IsRest(int degree, State1D? boundary)
    {
        if (boundary is null)
            return true;

        int p = (degree + 1) / 2;
        for (int order = 1; order < p; order++)
        {
            if (Math.Abs(boundary.Value.GetDerivative(order)) > 1e-12)
                return false;
        }

        return true;
    }

    private static double[] Scale(double[] reference, double lambda) =>
        reference.Select(t => lambda * t).ToArray();

    private static (int Order, double Limit)[] ActiveLimits(KinematicLimits limits)
    {
        var active = new List<(int, double)>();
        if (limits.HasVelocityLimit) active.Add((1, limits.MaxVelocity!.Value));
        if (limits.HasAccelerationLimit) active.Add((2, limits.MaxAcceleration!.Value));
        if (limits.HasJerkLimit) active.Add((3, limits.MaxJerk!.Value));
        if (limits.HasSnapLimit) active.Add((4, limits.MaxSnap!.Value));

        foreach (var (order, limit) in active)
        {
            if (!(limit > 0) || double.IsInfinity(limit))
                throw new ArgumentException($"The limit of derivative order {order} must be positive and finite, got {limit}.");
        }

        return active.ToArray();
    }

    private static string KindName(int order) => order switch
    {
        1 => "velocity",
        2 => "acceleration",
        3 => "jerk",
        _ => "snap"
    };

    /// <summary>λ* = max over active limits with a non-zero peak of (P_k / L_k)^(1/k); null if nothing is determined.</summary>
    private static double? ClosedFormScale(PolynomialSegment1D[] referenceSolution, KinematicLimits limits)
    {
        double? lambda = null;
        foreach (var (order, limit) in ActiveLimits(limits))
        {
            double peak = ExtremaAnalyzer.PeakAbsolute(referenceSolution, order);
            if (peak <= 0)
                continue; // an order whose peak is zero imposes nothing

            double candidate = Math.Pow(peak / limit, 1.0 / order);
            lambda = lambda is null ? candidate : Math.Max(lambda.Value, candidate);
        }

        return lambda;
    }

    /// <summary>F = max over active limits of P_k / L_k - 1 for a solved trajectory.</summary>
    private static double Feasibility(PolynomialSegment1D[] segments, KinematicLimits limits)
    {
        double worst = double.MinValue;
        foreach (var (order, limit) in ActiveLimits(limits))
            worst = Math.Max(worst, ExtremaAnalyzer.PeakAbsolute(segments, order) / limit);

        return worst - 1.0;
    }

    /// <summary>
    /// Smallest λ with F(λ) ≤ 0, searched around the starting value: geometric steps down while feasible or up while
    /// infeasible, then bisection between the last infeasible and the first feasible value. F need not be monotone
    /// (the first feasible value met from the infeasible side is returned; a window narrower than a step can be missed).
    /// </summary>
    private static double SearchScale(double start, Func<double, double> feasibility)
    {
        double lo = start / SearchRange;
        double hi = start * SearchRange;

        double feasible, infeasible;
        if (feasibility(start) <= 0)
        {
            feasible = start;
            infeasible = double.NaN;
            for (double t = start / GrowthRatio; t >= lo; t /= GrowthRatio)
            {
                if (feasibility(t) <= 0)
                {
                    feasible = t;
                }
                else
                {
                    infeasible = t;
                    break;
                }
            }

            if (double.IsNaN(infeasible))
                return feasible; // feasible down to the lower end of the search range
        }
        else
        {
            infeasible = start;
            feasible = double.NaN;
            for (double t = start * GrowthRatio; t <= hi * GrowthRatio; t *= GrowthRatio)
            {
                if (feasibility(t) <= 0)
                {
                    feasible = t;
                    break;
                }

                infeasible = t;
            }

            if (double.IsNaN(feasible))
            {
                throw new DurationSearchFailedException(
                    $"The pass-through: no common duration scale between {start:G6} and {hi:G6} satisfies the active kinematic limits. " +
                    "The limits may be impossible to meet with these boundary derivatives.",
                    infeasible);
            }
        }

        for (int i = 0; i < 100 && Math.Abs(feasible - infeasible) > 1e-13 * feasible; i++)
        {
            double mid = 0.5 * (infeasible + feasible);
            if (feasibility(mid) <= 0)
                feasible = mid;
            else
                infeasible = mid;
        }

        return feasible;
    }

    private static bool VerifiesLimits(PolynomialSegment1D[] segments, KinematicLimits limits) =>
        segments.All(s => ExtremaAnalyzer.VerifyLimits(s, limits));

    /// <summary>Throws <see cref="KinematicLimitsViolatedException"/> naming the first violating segment and limit.</summary>
    private static void VerifyOrThrow(PolynomialSegment1D[] segments, KinematicLimits limits)
    {
        var active = ActiveLimits(limits);
        for (int k = 0; k < segments.Length; k++)
        {
            foreach (var (order, limit) in active)
            {
                double peak = ExtremaAnalyzer.FindExtrema(segments[k], order).MaxAbsolute;
                if (peak > limit + 1e-10)
                {
                    throw new KinematicLimitsViolatedException(
                        $"Pass-through segment {k + 1} (duration {segments[k].Duration:G6} s) violates the {KindName(order)} limit: " +
                        $"peak {peak:G6} exceeds {limit:G6}.");
                }
            }
        }
    }
}
