using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Constraints;

/// <summary>
/// Shortest duration that satisfies all active kinematic limits of a segment whose duration is not
/// prescribed. Formulation: <c>theory/TrajectoryConstraints.md</c>, "Shortest Feasible Duration"
/// (project decision of 2026-10-17, <c>design/07</c> A9).
/// </summary>
/// <remarks>
/// Feasibility function F(T) = max over active limits k of P_k(T) / L_k - 1, where P_k(T) is the peak of
/// |x^(k)| for the segment recomputed with duration T. A duration is feasible when F(T) &lt;= tolerance.
/// <list type="bullet">
/// <item>Closed form (fast path) when all prescribed boundary derivatives of order 1 .. p-1 are zero:
/// T* = max_k ( |Δ| c_k / L_k )^(1/k), with c_k the peak of |x^(k)| of the unit profile (Δ = 1, T = 1).</item>
/// <item>Otherwise a search: geometric growth from a lower bound, then bisection.</item>
/// </list>
/// Failure (no feasible duration up to the cap) is reported with <see cref="DurationSearchFailedException"/>.
/// </remarks>
public static class ShortestFeasibleDuration
{
    /// <summary>Geometric ratio ρ of the search for the first feasible duration.</summary>
    public const double GrowthRatio = 1.05;

    /// <summary>The search stops at this factor times max(lower bound, 1 s).</summary>
    public const double MaxDurationFactor = 1e6;

    /// <summary>Floor of the search start: protection against degenerate numbers, never an answer by itself.</summary>
    public const double MinimumDuration = 1e-3;

    /// <summary>
    /// A duration is feasible if F(T) is not above this value. Zero (strict) so that the result also passes
    /// <see cref="ExtremaAnalyzer.VerifyLimits"/>, whose absolute slack is 1e-10 (a relative 1e-9 would not).
    /// </summary>
    public const double FeasibilityTolerance = 0.0;

    private const double RestTolerance = 1e-12;

    /// <summary>
    /// The shortest feasible duration: closed form when applicable, otherwise the search. Returns <c>null</c>
    /// when the limits determine no duration (no active limit, or a rest-to-rest segment of zero length, which
    /// satisfies every limit at any duration): the caller then uses the prescribed or default duration.
    /// A nullable result (rather than <c>TryFind</c> with an out parameter) keeps call sites to one expression,
    /// <c>Find(...) ?? fallback</c>. Impossible limits still throw <see cref="DurationSearchFailedException"/>.
    /// </summary>
    public static double? Find(int degree, State1D start, State1D end, KinematicLimits limits)
    {
        if (!limits.HasVelocityLimit && !limits.HasAccelerationLimit && !limits.HasJerkLimit && !limits.HasSnapLimit)
            return null;

        var active = ActiveLimits(limits);

        if (HasRestBoundary(degree, start, end))
            return RestToRest(degree, Math.Abs(end.Position - start.Position), active);

        return Search(degree, start, end, active);
    }

    /// <summary>
    /// True if all prescribed boundary derivatives of order 1 .. p-1 are zero at both ends, so that the
    /// profile is a fixed shape scaled by the distance.
    /// </summary>
    public static bool HasRestBoundary(int degree, State1D start, State1D end)
    {
        int p = (degree + 1) / 2;
        for (int order = 1; order < p; order++)
        {
            if (Math.Abs(start.GetDerivative(order)) > RestTolerance || Math.Abs(end.GetDerivative(order)) > RestTolerance)
                return false;
        }

        return true;
    }

    /// <summary>T* = max_k ( |Δ| c_k / L_k )^(1/k) for a rest-to-rest segment.</summary>
    public static double? RestToRest(int degree, double distance, KinematicLimits limits) =>
        RestToRest(degree, distance, ActiveLimits(limits));

    /// <summary>The general search (also valid for rest-to-rest segments); exposed to compare with the closed form.</summary>
    public static double Search(int degree, State1D start, State1D end, KinematicLimits limits) =>
        Search(degree, start, end, ActiveLimits(limits));

    /// <summary>F(T) of the theory: max over active limits of P_k(T) / L_k - 1.</summary>
    public static double Feasibility(int degree, State1D start, State1D end, KinematicLimits limits, double duration) =>
        Feasibility(degree, start, end, ActiveLimits(limits), duration);

    // ---------------------------------------------------------------------------------------------

    private static (int Order, double Limit)[] ActiveLimits(KinematicLimits limits)
    {
        var active = new List<(int, double)>();
        if (limits.HasVelocityLimit) active.Add((1, limits.MaxVelocity!.Value));
        if (limits.HasAccelerationLimit) active.Add((2, limits.MaxAcceleration!.Value));
        if (limits.HasJerkLimit) active.Add((3, limits.MaxJerk!.Value));
        if (limits.HasSnapLimit) active.Add((4, limits.MaxSnap!.Value));

        if (active.Count == 0)
            throw new InvalidOperationException("No kinematic limit is active: the limits do not determine a duration, which must be prescribed or defaulted.");

        foreach (var (order, limit) in active)
        {
            if (!(limit > 0) || double.IsInfinity(limit))
                throw new ArgumentException($"The limit of derivative order {order} must be positive and finite, got {limit}.");
        }

        return active.ToArray();
    }

    private static double? RestToRest(int degree, double distance, (int Order, double Limit)[] active)
    {
        if (distance <= 0)
            return null; // a constant segment satisfies every limit at any duration: the limits determine nothing

        var unit = new PolynomialSegment1D(degree, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), 1.0);

        double result = 0;
        foreach (var (order, limit) in active)
        {
            double c = ExtremaAnalyzer.FindExtrema(unit, order).MaxAbsolute;
            result = Math.Max(result, Math.Pow(distance * c / limit, 1.0 / order));
        }

        return result;
    }

    private static double Feasibility(int degree, State1D start, State1D end, (int Order, double Limit)[] active, double duration)
    {
        var segment = new PolynomialSegment1D(degree, start, end, duration);

        double worst = double.MinValue;
        foreach (var (order, limit) in active)
            worst = Math.Max(worst, ExtremaAnalyzer.FindExtrema(segment, order).MaxAbsolute / limit);

        return worst - 1.0;
    }

    private static double Search(int degree, State1D start, State1D end, (int Order, double Limit)[] active)
    {
        double lower = LowerBound(degree, start, end, active);

        double feasible = lower;
        if (Feasibility(degree, start, end, active, feasible) <= FeasibilityTolerance)
            return feasible; // nothing shorter can work: lower is a necessary condition

        double cap = MaxDurationFactor * Math.Max(lower, 1.0);
        double infeasible = lower;
        bool found = false;
        for (double t = lower * GrowthRatio; t <= cap * GrowthRatio; t *= GrowthRatio)
        {
            if (Feasibility(degree, start, end, active, t) <= FeasibilityTolerance)
            {
                feasible = t;
                found = true;
                break;
            }

            infeasible = t;
        }

        if (!found)
        {
            throw new DurationSearchFailedException(
                $"No duration between {lower:G6} s and {cap:G6} s satisfies the active kinematic limits. " +
                "The limits may be impossible to meet, for example a boundary velocity above the velocity limit.",
                infeasible);
        }

        // Bisection between the last infeasible and the first feasible duration.
        for (int i = 0; i < 100 && feasible - infeasible > 1e-13 * feasible; i++)
        {
            double mid = 0.5 * (infeasible + feasible);
            if (Feasibility(degree, start, end, active, mid) <= FeasibilityTolerance)
                feasible = mid;
            else
                infeasible = mid;
        }

        return feasible;
    }

    /// <summary>
    /// Necessary conditions: the mean of x^(k) over the segment cannot exceed its peak, so
    /// T >= |x^(k-1)(end) - x^(k-1)(start)| / L_k for every active limit whose boundary value is prescribed.
    /// </summary>
    private static double LowerBound(int degree, State1D start, State1D end, (int Order, double Limit)[] active)
    {
        int p = (degree + 1) / 2;
        double lower = MinimumDuration;
        foreach (var (order, limit) in active)
        {
            if (order - 1 > p - 1)
                continue; // the (order-1)-th boundary derivative is not prescribed for this degree

            double change = Math.Abs(end.GetDerivative(order - 1) - start.GetDerivative(order - 1));
            lower = Math.Max(lower, change / limit);
        }

        return lower;
    }
}
