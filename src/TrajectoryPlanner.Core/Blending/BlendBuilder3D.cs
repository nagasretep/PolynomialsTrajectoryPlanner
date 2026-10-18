using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.Blending;

/// <summary>
/// 3D blending of two consecutive segments (<c>theory/BlendingSegments.md</c>, "3D Blending Problem" and following).
/// The window is chosen geometrically in 3D (local parameter fractions, or absolute points), the boundary states are
/// the vector states of the neighbours there, and the coefficients are determined component-wise with <b>one common
/// duration</b> for the three axes. Rules for degree, duration and limits are those of <see cref="BlendBuilder1D"/>;
/// the limits are per axis, and the shortest feasible duration is the one at which all three axes respect them.
/// </summary>
public static class BlendBuilder3D
{
    /// <summary>
    /// An absolute point is accepted as "on the segment" if its distance from the path is at most this fraction of the
    /// segment's extent (the diagonal of the box that contains the path).
    /// </summary>
    public const double PointDistanceTolerance = 1e-9;

    private const int ScanIntervals = 400;

    /// <summary>Default blend degree: the neighbours' degree if they agree, 9 otherwise (as in 1D).</summary>
    public static int DefaultBlendDegree(PolynomialSegment3D preceding, PolynomialSegment3D following) =>
        preceding.Degree == following.Degree ? preceding.Degree : BlendBuilder1D.MixedNeighbourDegree;

    /// <summary>Blend between two points given as normalized parameters of the neighbours (the primary entry point).</summary>
    /// <exception cref="ArgumentOutOfRangeException">A parameter outside [0, 1].</exception>
    /// <exception cref="ArgumentException">Empty window, unsupported degree, non-positive duration.</exception>
    /// <exception cref="KinematicLimitsViolatedException">A prescribed duration violates a limit.</exception>
    /// <exception cref="DurationSearchFailedException">No duration satisfies the limits.</exception>
    public static BlendSegment3D BuildBlendSegment(
        PolynomialSegment3D precedingSegment,
        PolynomialSegment3D followingSegment,
        double blendStartTauOnPreceding,
        double blendEndTauOnFollowing,
        int? blendDegree = null,
        double? blendDuration = null,
        KinematicLimits? limits = null)
    {
        if (blendStartTauOnPreceding < 0 || blendStartTauOnPreceding > 1)
            throw new ArgumentOutOfRangeException(nameof(blendStartTauOnPreceding));
        if (blendEndTauOnFollowing < 0 || blendEndTauOnFollowing > 1)
            throw new ArgumentOutOfRangeException(nameof(blendEndTauOnFollowing));

        double removed = precedingSegment.Duration * (1 - blendStartTauOnPreceding) +
                         followingSegment.Duration * blendEndTauOnFollowing;
        if (!(removed > 0))
            throw new ArgumentException("Empty blend window: the blend would remove nothing from the neighbouring segments.");

        int degree = blendDegree ?? DefaultBlendDegree(precedingSegment, followingSegment);
        if (!TrajectoryConstants.SupportedDegrees.Contains(degree))
            throw new ArgumentException($"Unsupported blend degree: {degree}", nameof(blendDegree));
        if (blendDuration is <= 0)
            throw new ArgumentException("A prescribed blend duration must be positive.", nameof(blendDuration));

        State3D startState = precedingSegment.EvaluateState(blendStartTauOnPreceding * precedingSegment.Duration);
        State3D endState = followingSegment.EvaluateState(blendEndTauOnFollowing * followingSegment.Duration);

        KinematicLimits? active = limits is not null && limits.HasAnyLimit ? limits : null;
        double inheritedDuration = Math.Max(removed, BlendBuilder1D.MinInheritedDuration);

        double duration;
        if (blendDuration is not null)
            duration = blendDuration.Value;
        else
            duration = active is null ? inheritedDuration : InheritedIfFeasibleElseShortest(degree, startState, endState, active, inheritedDuration);

        bool adjusted = blendDuration is null && active is not null && duration != inheritedDuration;
        var blend = new BlendSegment3D(degree, startState, endState, duration, inheritedDuration, adjusted);

        if (active is not null && !ExtremaAnalyzer.VerifyLimits(blend, active))
        {
            throw new KinematicLimitsViolatedException(
                $"The blend (duration {duration:G6} s) violates the active kinematic limits.");
        }

        return blend;
    }

    /// <summary>Blend between two points given as absolute 3D points on the neighbours.</summary>
    /// <exception cref="ArgumentException">A point that is not on the neighbouring segment (the message gives the distance).</exception>
    public static BlendSegment3D BuildBlendSegment(
        PolynomialSegment3D precedingSegment,
        PolynomialSegment3D followingSegment,
        Point3D blendStartAbsolutePoint,
        Point3D blendEndAbsolutePoint,
        int? blendDegree = null,
        double? blendDuration = null,
        KinematicLimits? limits = null)
    {
        double startTau = TauForPoint(precedingSegment, blendStartAbsolutePoint, "blend start (preceding segment)");
        double endTau = TauForPoint(followingSegment, blendEndAbsolutePoint, "blend end (following segment)");
        return BuildBlendSegment(precedingSegment, followingSegment, startTau, endTau, blendDegree, blendDuration, limits);
    }

    /// <summary>
    /// Blend plus the two truncated neighbours (the preceding keeps [0, τ_start], the following [τ_end, 1]; both must
    /// keep a positive duration).
    /// </summary>
    public static (PolynomialSegment3D TruncatedPreceding, BlendSegment3D BlendSegment, PolynomialSegment3D TruncatedFollowing)
        BuildBlendedSegmentPair(
            PolynomialSegment3D preceding,
            PolynomialSegment3D following,
            double blendStartTauOnPreceding,
            double blendEndTauOnFollowing,
            int? blendDegree = null,
            double? blendDuration = null,
            KinematicLimits? limits = null)
    {
        var blend = BuildBlendSegment(
            preceding, following, blendStartTauOnPreceding, blendEndTauOnFollowing, blendDegree, blendDuration, limits);

        double precedingDuration = blendStartTauOnPreceding * preceding.Duration;
        double followingDuration = (1 - blendEndTauOnFollowing) * following.Duration;
        if (!(precedingDuration > 0) || !(followingDuration > 0))
            throw new ArgumentException("The blend would remove a whole neighbouring segment: each neighbour must keep a positive duration.");

        var truncatedPreceding = new PolynomialSegment3D(
            preceding.Degree, preceding.StartState, preceding.EvaluateState(precedingDuration), precedingDuration);
        var truncatedFollowing = new PolynomialSegment3D(
            following.Degree, following.EvaluateState(blendEndTauOnFollowing * following.Duration), following.EndState, followingDuration);

        return (truncatedPreceding, blend, truncatedFollowing);
    }

    /// <summary>Same, with the blend points given as absolute 3D points.</summary>
    public static (PolynomialSegment3D TruncatedPreceding, BlendSegment3D BlendSegment, PolynomialSegment3D TruncatedFollowing)
        BuildBlendedSegmentPair(
            PolynomialSegment3D preceding,
            PolynomialSegment3D following,
            Point3D blendStartAbsolutePoint,
            Point3D blendEndAbsolutePoint,
            int? blendDegree = null,
            double? blendDuration = null,
            KinematicLimits? limits = null)
    {
        double startTau = TauForPoint(preceding, blendStartAbsolutePoint, "blend start (preceding segment)");
        double endTau = TauForPoint(following, blendEndAbsolutePoint, "blend end (following segment)");
        return BuildBlendedSegmentPair(preceding, following, startTau, endTau, blendDegree, blendDuration, limits);
    }

    /// <summary>
    /// Normalized parameter of the point of the segment's path closest to <paramref name="target"/>, clamped to
    /// [<see cref="BlendBuilder1D.TauMin"/>, <see cref="BlendBuilder1D.TauMax"/>] like the 1D parameter. A scan of the
    /// distance finds the best sample, and the root of d/dτ |p(τ) − q|² = 0 in the neighbouring interval refines it.
    /// </summary>
    /// <exception cref="ArgumentException">The closest point is farther than <see cref="PointDistanceTolerance"/> times the segment's extent.</exception>
    public static double TauForPoint(PolynomialSegment3D segment, Point3D target, string label = "blend point")
    {
        double T = segment.Duration;
        double Distance2(double tau) => (segment.EvaluatePosition(tau * T) - target).LengthSquared;

        // Scan.
        int best = 0;
        double bestValue = Distance2(0);
        for (int i = 1; i <= ScanIntervals; i++)
        {
            double value = Distance2((double)i / ScanIntervals);
            if (value < bestValue)
            {
                bestValue = value;
                best = i;
            }
        }

        // Refinement: root of the derivative (p(τ) − q)·p'(τ) in the interval around the best sample.
        double G(double tau) => (segment.EvaluatePosition(tau * T) - target).Dot(segment.EvaluateDerivative(tau * T, 1));

        double lo = Math.Max(0.0, (best - 1.0) / ScanIntervals);
        double hi = Math.Min(1.0, (best + 1.0) / ScanIntervals);
        double tauBest = (double)best / ScanIntervals;
        double glo = G(lo), ghi = G(hi);
        if (glo * ghi < 0 || glo == 0 || ghi == 0)
        {
            if (glo == 0) tauBest = lo;
            else if (ghi == 0) tauBest = hi;
            else
            {
                for (int k = 0; k < 100; k++)
                {
                    double mid = 0.5 * (lo + hi);
                    double gm = G(mid);
                    if (gm == 0) { lo = hi = mid; break; }
                    if (glo * gm < 0) hi = mid; else { lo = mid; glo = gm; }
                }

                tauBest = 0.5 * (lo + hi);
            }
        }

        double distance = Math.Sqrt(Distance2(tauBest));
        double extent = Extent(segment);
        if (distance > PointDistanceTolerance * Math.Max(extent, 1e-300))
        {
            throw new ArgumentException(
                $"The {label} {target} is not on the segment: its distance from the path is {distance:G6} " +
                $"(extent of the segment {extent:G6}, tolerance {PointDistanceTolerance * extent:G3}).");
        }

        return Math.Clamp(tauBest, BlendBuilder1D.TauMin, BlendBuilder1D.TauMax);
    }

    /// <summary>Diagonal of the axis-aligned box that contains the path of the segment.</summary>
    public static double Extent(PolynomialSegment3D segment)
    {
        double Range(PolynomialSegment1D axis)
        {
            var e = ExtremaAnalyzer.PositionExtrema(axis);
            return e.Maximum - e.Minimum;
        }

        double x = Range(segment.XAxis), y = Range(segment.YAxis), z = Range(segment.ZAxis);
        return Math.Sqrt(x * x + y * y + z * z);
    }

    private static double InheritedIfFeasibleElseShortest(
        int degree, State3D start, State3D end, KinematicLimits limits, double inherited)
    {
        if (ExtremaAnalyzer.VerifyLimits(new PolynomialSegment3D(degree, start, end, inherited), limits))
            return inherited;

        try
        {
            return ShortestFeasibleDuration.Find(degree, Components(start), Components(end), limits) ?? inherited;
        }
        catch (DurationSearchFailedException ex)
        {
            throw new DurationSearchFailedException($"The blend: {ex.Message}", ex.LastDurationTried);
        }
    }

    /// <summary>The three axis states of a vector state, in the order x, y, z.</summary>
    public static State1D[] Components(State3D state) =>
        [state.GetComponent(0), state.GetComponent(1), state.GetComponent(2)];
}
