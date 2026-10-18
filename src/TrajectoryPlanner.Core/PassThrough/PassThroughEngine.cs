using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>
/// The algorithm of exact pass-through with kinematic limits, for one or several components that share their durations
/// (<c>theory/TrajectoryPassingThroughConstraintPoints.md</c>, "Pass-Through With Kinematic Limits" and "Common Durations
/// And Kinematic Limits In 3D"). 1D is one component, 3D three; for fixed durations the components are independent
/// scalar global solves, and every limit is applied to each component. Used by <see cref="PassThroughPlanner1D"/> and
/// <see cref="PassThroughPlanner3D"/>, so that the closed form, the λ search and the verification exist once.
/// </summary>
internal static class PassThroughEngine
{
    /// <summary>Relative step used to repair a result that fails the verification by rounding only.</summary>
    private const double SafetyStep = 1e-9;

    private const int MaxSafetySteps = 5;

    /// <param name="Segments">[component][segment].</param>
    /// <param name="Scale">λ (1 when not computed).</param>
    /// <param name="BindingOrder">Derivative order of the limit that determines λ (0 when λ was not computed).</param>
    /// <param name="BindingComponent">Component where it binds (-1 when λ was not computed).</param>
    internal sealed record Outcome(
        PolynomialSegment1D[][] Segments, double[] Durations, double Scale, bool ScaledByLimits, int BindingOrder, int BindingComponent);

    /// <param name="waypoints">[component][waypoint].</param>
    /// <param name="prescribed">Durations kept and verified, or null.</param>
    /// <param name="reference">Reference durations T̄_k (used when <paramref name="prescribed"/> is null).</param>
    /// <param name="starts">Per component: prescribed start derivatives (null: rest).</param>
    /// <param name="ends">Per component: prescribed end derivatives (null: rest).</param>
    /// <param name="componentNames">Names used in the violation messages ("x", "y", "z"); null for a single unnamed component.</param>
    internal static Outcome Run(
        double[][] waypoints,
        int degree,
        SmoothnessCriterion? criterion,
        KinematicLimits? limits,
        double[]? prescribed,
        double[]? reference,
        State1D?[] starts,
        State1D?[] ends,
        string[]? componentNames)
    {
        var solver = new PassThroughSolver1D(criterion);
        int n = waypoints.Length;
        bool active = limits is not null && limits.HasAnyLimit;

        PolynomialSegment1D[][] Solve(double[] durations, bool withBoundary = true)
        {
            var result = new PolynomialSegment1D[n][];
            for (int c = 0; c < n; c++)
                result[c] = solver.Solve(waypoints[c], durations, degree, withBoundary ? starts[c] : null, withBoundary ? ends[c] : null);
            return result;
        }

        if (prescribed is not null)
        {
            var segments = Solve(prescribed);
            if (active)
                VerifyOrThrow(segments, limits!, componentNames);

            return new Outcome(segments, (double[])prescribed.Clone(), 1.0, false, 0, -1);
        }

        double[] reference0 = reference ?? throw new ArgumentNullException(nameof(reference));
        if (!active)
            return new Outcome(Solve(reference0), reference0, 1.0, false, 0, -1);

        // Peaks of the reference solution with the boundary derivatives set to zero: the closed form's input.
        bool zeroBoundary = Enumerable.Range(0, n).All(c => IsRest(degree, starts[c]) && IsRest(degree, ends[c]));
        var restReference = zeroBoundary ? Solve(reference0) : Solve(reference0, withBoundary: false);
        var closedForm = ClosedFormScale(restReference, limits!);

        if (closedForm is null)
            return new Outcome(Solve(reference0), reference0, 1.0, false, 0, -1); // the limits determine nothing

        double lambda = zeroBoundary
            ? closedForm.Value.Lambda
            : SearchScale(closedForm.Value.Lambda, scale => Feasibility(Solve(Scale(reference0, scale)), limits!));

        // Second solve at the scaled durations (for zero boundary derivatives it doubles as a check of the scaling law).
        var final = Solve(Scale(reference0, lambda));
        for (int attempt = 0; attempt < MaxSafetySteps && !VerifiesLimits(final, limits!); attempt++)
        {
            lambda *= 1 + SafetyStep; // the closed form lands exactly on the limit: rounding may exceed it by 1e-10
            final = Solve(Scale(reference0, lambda));
        }

        VerifyOrThrow(final, limits!, componentNames);

        // The binding pair: for the closed form the maximiser of (P/L)^(1/k); after a search the largest P/L at λ.
        var (order, component) = zeroBoundary ? (closedForm.Value.Order, closedForm.Value.Component) : WorstRatio(final, limits!);
        return new Outcome(final, Scale(reference0, lambda), lambda, true, order, component);
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

    internal static (int Order, double Limit)[] ActiveLimits(KinematicLimits limits)
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

    internal static string KindName(int order) => order switch
    {
        1 => "velocity",
        2 => "acceleration",
        3 => "jerk",
        _ => "snap"
    };

    /// <summary>
    /// λ* = max over active limits and components with a non-zero peak of (P_k,c / L_k)^(1/k), with the maximiser;
    /// null if nothing is determined (no peak anywhere: constant components impose nothing).
    /// </summary>
    private static (double Lambda, int Order, int Component)? ClosedFormScale(PolynomialSegment1D[][] referenceSolution, KinematicLimits limits)
    {
        (double, int, int)? best = null;
        foreach (var (order, limit) in ActiveLimits(limits))
        {
            for (int c = 0; c < referenceSolution.Length; c++)
            {
                double peak = ExtremaAnalyzer.PeakAbsolute(referenceSolution[c], order);
                if (peak <= 0)
                    continue; // an order whose peak is zero imposes nothing

                double candidate = Math.Pow(peak / limit, 1.0 / order);
                if (best is null || candidate > best.Value.Item1)
                    best = (candidate, order, c);
            }
        }

        return best;
    }

    /// <summary>F = max over active limits and components of P_k / L_k - 1 for a solved trajectory.</summary>
    private static double Feasibility(PolynomialSegment1D[][] segments, KinematicLimits limits)
    {
        double worst = double.MinValue;
        foreach (var (order, limit) in ActiveLimits(limits))
            foreach (var component in segments)
                worst = Math.Max(worst, ExtremaAnalyzer.PeakAbsolute(component, order) / limit);

        return worst - 1.0;
    }

    private static (int Order, int Component) WorstRatio(PolynomialSegment1D[][] segments, KinematicLimits limits)
    {
        double worst = double.MinValue;
        (int, int) at = (0, -1);
        foreach (var (order, limit) in ActiveLimits(limits))
        {
            for (int c = 0; c < segments.Length; c++)
            {
                double ratio = ExtremaAnalyzer.PeakAbsolute(segments[c], order) / limit;
                if (ratio > worst)
                {
                    worst = ratio;
                    at = (order, c);
                }
            }
        }

        return at;
    }

    /// <summary>
    /// Smallest λ with F(λ) ≤ 0, searched around the starting value: geometric steps down while feasible or up while
    /// infeasible, then bisection between the last infeasible and the first feasible value. F need not be monotone
    /// (the first feasible value met from the infeasible side is returned; a window narrower than a step can be missed).
    /// </summary>
    private static double SearchScale(double start, Func<double, double> feasibility)
    {
        double lo = start / PassThroughPlanner1D.SearchRange;
        double hi = start * PassThroughPlanner1D.SearchRange;
        double ratio = PassThroughPlanner1D.GrowthRatio;

        double feasible, infeasible;
        if (feasibility(start) <= 0)
        {
            feasible = start;
            infeasible = double.NaN;
            for (double t = start / ratio; t >= lo; t /= ratio)
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
            for (double t = start * ratio; t <= hi * ratio; t *= ratio)
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

    private static bool VerifiesLimits(PolynomialSegment1D[][] segments, KinematicLimits limits) =>
        segments.All(component => component.All(s => ExtremaAnalyzer.VerifyLimits(s, limits)));

    /// <summary>Throws <see cref="KinematicLimitsViolatedException"/> naming the first violating segment, limit (and axis).</summary>
    private static void VerifyOrThrow(PolynomialSegment1D[][] segments, KinematicLimits limits, string[]? componentNames)
    {
        var active = ActiveLimits(limits);
        int count = segments[0].Length;
        for (int k = 0; k < count; k++)
        {
            foreach (var (order, limit) in active)
            {
                for (int c = 0; c < segments.Length; c++)
                {
                    double peak = ExtremaAnalyzer.FindExtrema(segments[c][k], order).MaxAbsolute;
                    if (peak > limit + 1e-10)
                    {
                        string axis = componentNames is null ? string.Empty : $" on axis {componentNames[c]}";
                        throw new KinematicLimitsViolatedException(
                            $"Pass-through segment {k + 1} (duration {segments[c][k].Duration:G6} s) violates the {KindName(order)} limit{axis}: " +
                            $"peak {peak:G6} exceeds {limit:G6}.");
                    }
                }
            }
        }
    }
}
