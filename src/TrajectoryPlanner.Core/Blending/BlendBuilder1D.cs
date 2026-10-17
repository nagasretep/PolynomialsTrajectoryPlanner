using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.Blending;

/// <summary>
/// 1D blending of two consecutive segments: a dedicated segment replaces a window around the nominal join
/// (<c>theory/BlendingSegments.md</c>, baseline algorithm).
/// </summary>
/// <remarks>
/// Degree: selectable; default <see cref="DefaultBlendDegree"/>. Duration: prescribed (only verified against the
/// limits); otherwise the duration inherited from the removed portions of the neighbours (floor 0.01 s), kept
/// whenever it satisfies the active limits, and replaced by the shortest feasible duration only when it does not
/// (decision of 2026-10-17, D19). Failures are explicit; nothing is silently clamped or repaired
/// except the documented clamp of the normalized parameter for absolute positions.
/// </remarks>
public static class BlendBuilder1D
{
    /// <summary>Floor of the inherited blend duration (protection against a vanishing window).</summary>
    public const double MinInheritedDuration = 0.01;

    /// <summary>Absolute positions are converted to a normalized parameter clamped to [TauMin, TauMax].</summary>
    public const double TauMin = 0.001;

    public const double TauMax = 0.999;

    /// <summary>Blend degree 9 for neighbours of different degrees.</summary>
    public const int MixedNeighbourDegree = 9;

    /// <summary>
    /// Default blend degree: the degree of the neighbouring segments if they agree, 9 otherwise.
    /// (A blend between two cubics is a cubic; a blend between segments of different degrees takes the richest
    /// reference degree.)
    /// </summary>
    public static int DefaultBlendDegree(PolynomialSegment1D preceding, PolynomialSegment1D following) =>
        preceding.Degree == following.Degree ? preceding.Degree : MixedNeighbourDegree;

    /// <summary>Blend between two points given as normalized parameters of the neighbours.</summary>
    /// <param name="blendDegree">3, 5, 7 or 9; null selects <see cref="DefaultBlendDegree"/>.</param>
    /// <param name="blendDuration">Prescribed duration (kept, only verified against the limits); null: see the class remarks.</param>
    /// <param name="limits">Active limits for the blend (null or none active: no limit).</param>
    /// <exception cref="ArgumentOutOfRangeException">A parameter outside [0, 1].</exception>
    /// <exception cref="ArgumentException">Empty window (nothing removed), unsupported degree, non-positive duration.</exception>
    /// <exception cref="KinematicLimitsViolatedException">A prescribed duration violates a limit.</exception>
    /// <exception cref="DurationSearchFailedException">No duration satisfies the limits.</exception>
    public static BlendSegment1D BuildBlendSegment(
        PolynomialSegment1D precedingSegment,
        PolynomialSegment1D followingSegment,
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

        State1D blendStartState = precedingSegment.EvaluateState(blendStartTauOnPreceding * precedingSegment.Duration);
        State1D blendEndState = followingSegment.EvaluateState(blendEndTauOnFollowing * followingSegment.Duration);

        KinematicLimits? active = ActiveOrNull(limits);

        // The inherited duration (sum of the removed portions, floored) is always computed: it is what the blend gets
        // unless limits force another value, and it is reported on the result.
        double inheritedDuration = Math.Max(removed, MinInheritedDuration);

        double duration;
        if (blendDuration is not null)
        {
            duration = blendDuration.Value;
        }
        else
        {
            duration = active is null
                ? inheritedDuration
                : InheritedIfFeasibleElseShortest(degree, blendStartState, blendEndState, active, inheritedDuration);
        }

        bool adjusted = blendDuration is null && active is not null && duration != inheritedDuration;
        var blend = new BlendSegment1D(degree, blendStartState, blendEndState, duration, inheritedDuration, adjusted);

        if (active is not null && !ExtremaAnalyzer.VerifyLimits(blend, active))
        {
            throw new KinematicLimitsViolatedException(
                $"The blend (duration {duration:G6} s) violates the active kinematic limits.");
        }

        return blend;
    }

    /// <summary>Blend between two points given as absolute positions on the neighbours.</summary>
    /// <exception cref="ArgumentException">A position that the neighbouring segment never reaches.</exception>
    public static BlendSegment1D BuildBlendSegment(
        PolynomialSegment1D precedingSegment,
        PolynomialSegment1D followingSegment,
        Point1D blendStartAbsolutePosition,
        Point1D blendEndAbsolutePosition,
        int? blendDegree = null,
        double? blendDuration = null,
        KinematicLimits? limits = null)
    {
        double startTau = TauForPosition(precedingSegment, blendStartAbsolutePosition.X, "blend start (preceding segment)");
        double endTau = TauForPosition(followingSegment, blendEndAbsolutePosition.X, "blend end (following segment)");
        return BuildBlendSegment(precedingSegment, followingSegment, startTau, endTau, blendDegree, blendDuration, limits);
    }

    /// <summary>
    /// Normalized parameter at which the segment has the given position, clamped to [TauMin, TauMax].
    /// If the position is reached more than once the first (earliest) crossing is taken.
    /// </summary>
    /// <exception cref="ArgumentException">The position is outside the range the segment covers.</exception>
    public static double TauForPosition(PolynomialSegment1D segment, double targetPosition, string label = "blend point")
    {
        var range = ExtremaAnalyzer.PositionExtrema(segment);
        double slack = 1e-9 * (1.0 + Math.Abs(range.Maximum - range.Minimum));
        if (targetPosition < range.Minimum - slack || targetPosition > range.Maximum + slack)
        {
            throw new ArgumentException(
                $"The {label} position {targetPosition:G6} is outside the segment, which covers " +
                $"[{range.Minimum:G6}, {range.Maximum:G6}].");
        }

        double F(double tau) => segment.EvaluatePosition(tau * segment.Duration) - targetPosition;

        const int intervals = 400;
        double previousTau = 0, previousValue = F(0);
        double bestTau = 0, bestAbs = Math.Abs(previousValue);
        double? found = previousValue == 0 ? 0.0 : null;

        for (int i = 1; i <= intervals && found is null; i++)
        {
            double tau = (double)i / intervals;
            double value = F(tau);
            if (Math.Abs(value) < bestAbs)
            {
                bestAbs = Math.Abs(value);
                bestTau = tau;
            }

            if (value == 0)
            {
                found = tau;
            }
            else if (previousValue * value < 0)
            {
                double lo = previousTau, hi = tau, flo = previousValue;
                for (int k = 0; k < 100; k++)
                {
                    double mid = 0.5 * (lo + hi);
                    double fm = F(mid);
                    if (fm == 0) { lo = hi = mid; break; }
                    if (flo * fm < 0) hi = mid; else { lo = mid; flo = fm; }
                }

                found = 0.5 * (lo + hi);
            }

            previousTau = tau;
            previousValue = value;
        }

        // No sign change (the target touches an extremum): take the closest sample.
        return Math.Clamp(found ?? bestTau, TauMin, TauMax);
    }

    /// <summary>
    /// Blend plus the two truncated neighbours (the kept portions), points given as normalized parameters.
    /// The preceding segment keeps [0, τ_start], the following one [τ_end, 1]; both must keep a positive duration.
    /// </summary>
    public static (PolynomialSegment1D TruncatedPreceding,
                    BlendSegment1D BlendSegment,
                    PolynomialSegment1D TruncatedFollowing)
        BuildBlendedSegmentPair(
            PolynomialSegment1D preceding,
            PolynomialSegment1D following,
            double blendStartTauOnPreceding,
            double blendEndTauOnFollowing,
            int? blendDegree = null,
            double? blendDuration = null,
            KinematicLimits? limits = null)
    {
        var blendSeg = BuildBlendSegment(
            preceding, following, blendStartTauOnPreceding, blendEndTauOnFollowing, blendDegree, blendDuration, limits);

        double precedingDuration = blendStartTauOnPreceding * preceding.Duration;
        double followingDuration = (1 - blendEndTauOnFollowing) * following.Duration;
        if (!(precedingDuration > 0) || !(followingDuration > 0))
            throw new ArgumentException("The blend would remove a whole neighbouring segment: each neighbour must keep a positive duration.");

        State1D truncatedPrecedingEnd = preceding.EvaluateState(precedingDuration);
        var truncatedPreceding = new PolynomialSegment1D(preceding.Degree, preceding.StartState, truncatedPrecedingEnd, precedingDuration);

        double followingStartTime = blendEndTauOnFollowing * following.Duration;
        State1D truncatedFollowingStart = following.EvaluateState(followingStartTime);
        var truncatedFollowing = new PolynomialSegment1D(following.Degree, truncatedFollowingStart, following.EndState, followingDuration);

        return (truncatedPreceding, blendSeg, truncatedFollowing);
    }

    /// <summary>Same, with the blend points given as absolute positions on the neighbours.</summary>
    public static (PolynomialSegment1D TruncatedPreceding,
                    BlendSegment1D BlendSegment,
                    PolynomialSegment1D TruncatedFollowing)
        BuildBlendedSegmentPair(
            PolynomialSegment1D preceding,
            PolynomialSegment1D following,
            Point1D blendStartAbsolutePosition,
            Point1D blendEndAbsolutePosition,
            int? blendDegree = null,
            double? blendDuration = null,
            KinematicLimits? limits = null)
    {
        double startTau = TauForPosition(preceding, blendStartAbsolutePosition.X, "blend start (preceding segment)");
        double endTau = TauForPosition(following, blendEndAbsolutePosition.X, "blend end (following segment)");
        return BuildBlendedSegmentPair(preceding, following, startTau, endTau, blendDegree, blendDuration, limits);
    }

    private static KinematicLimits? ActiveOrNull(KinematicLimits? limits) =>
        limits is not null && (limits.HasVelocityLimit || limits.HasAccelerationLimit || limits.HasJerkLimit || limits.HasSnapLimit)
            ? limits
            : null;

    private static double InheritedIfFeasibleElseShortest(int degree, State1D start, State1D end, KinematicLimits limits, double inherited)
    {
        // Keep the inherited duration whenever the blend with that duration respects the limits.
        if (ExtremaAnalyzer.VerifyLimits(new PolynomialSegment1D(degree, start, end, inherited), limits))
            return inherited;

        try
        {
            return ShortestFeasibleDuration.Find(degree, start, end, limits) ?? inherited;
        }
        catch (DurationSearchFailedException ex)
        {
            throw new DurationSearchFailedException($"The blend: {ex.Message}", ex.LastDurationTried);
        }
    }
}
