using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Blending;

public static class BlendBuilder1D
{
    public static PolynomialSegment1D BuildBlendSegment(
        PolynomialSegment1D precedingSegment,
        PolynomialSegment1D followingSegment,
        double blendStartTauOnPreceding,
        double blendEndTauOnFollowing,
        int blendDegree = 9,
        double? blendDuration = null)
    {
        if (blendStartTauOnPreceding < 0 || blendStartTauOnPreceding > 1)
            throw new ArgumentOutOfRangeException(nameof(blendStartTauOnPreceding));
        if (blendEndTauOnFollowing < 0 || blendEndTauOnFollowing > 1)
            throw new ArgumentOutOfRangeException(nameof(blendEndTauOnFollowing));

        double blendStartTime = blendStartTauOnPreceding * precedingSegment.Duration;
        double blendEndTime = blendEndTauOnFollowing * followingSegment.Duration;

        State1D blendStartState = precedingSegment.EvaluateState(blendStartTime);
        State1D blendEndState = followingSegment.EvaluateState(blendEndTime);

        double T = blendDuration ?? (precedingSegment.Duration * (1 - blendStartTauOnPreceding) +
                                      followingSegment.Duration * blendEndTauOnFollowing);
        T = Math.Max(T, 0.01);

        return new PolynomialSegment1D(blendDegree, blendStartState, blendEndState, T);
    }

    public static PolynomialSegment1D BuildBlendSegment(
        PolynomialSegment1D precedingSegment,
        PolynomialSegment1D followingSegment,
        Point1D blendStartAbsolutePosition,
        Point1D blendEndAbsolutePosition,
        int blendDegree = 9,
        double? blendDuration = null)
    {
        double startTau = FindTauForPosition(precedingSegment, blendStartAbsolutePosition.X);
        double endTau = FindTauForPosition(followingSegment, blendEndAbsolutePosition.X);
        return BuildBlendSegment(precedingSegment, followingSegment, startTau, endTau, blendDegree, blendDuration);
    }

    private static double FindTauForPosition(PolynomialSegment1D segment, double targetPosition)
    {
        double startPos = segment.StartState.Position;
        double endPos = segment.EndState.Position;
        double range = endPos - startPos;

        if (Math.Abs(range) < 1e-12)
            return 0.5;

        double tauEstimate = (targetPosition - startPos) / range;
        tauEstimate = Math.Clamp(tauEstimate, 0.01, 0.99);

        for (int iter = 0; iter < 50; iter++)
        {
            double currentPos = segment.EvaluatePosition(tauEstimate * segment.Duration);
            double currentVel = segment.EvaluateDerivative(tauEstimate * segment.Duration, 1);

            if (Math.Abs(currentVel) < 1e-12)
                break;

            double error = targetPosition - currentPos;
            double deltaTau = error / (currentVel * segment.Duration);
            tauEstimate += deltaTau;
            tauEstimate = Math.Clamp(tauEstimate, 0.001, 0.999);

            if (Math.Abs(error) < 1e-8)
                break;
        }

        return tauEstimate;
    }

    public static (PolynomialSegment1D TruncatedPreceding,
                    PolynomialSegment1D BlendSegment,
                    PolynomialSegment1D TruncatedFollowing)
        BuildBlendedSegmentPair(
            PolynomialSegment1D preceding,
            PolynomialSegment1D following,
            double blendStartTauOnPreceding,
            double blendEndTauOnFollowing,
            int blendDegree = 9)
    {
        var blendSeg = BuildBlendSegment(preceding, following, blendStartTauOnPreceding, blendEndTauOnFollowing, blendDegree);

        double precedingDuration = blendStartTauOnPreceding * preceding.Duration;
        State1D truncatedPrecedingEnd = preceding.EvaluateState(precedingDuration);
        var truncatedPreceding = new PolynomialSegment1D(preceding.Degree, preceding.StartState, truncatedPrecedingEnd, precedingDuration);

        double followingDuration = (1 - blendEndTauOnFollowing) * following.Duration;
        double followingStartTime = blendEndTauOnFollowing * following.Duration;
        State1D truncatedFollowingStart = following.EvaluateState(followingStartTime);
        var truncatedFollowing = new PolynomialSegment1D(following.Degree, truncatedFollowingStart, following.EndState, followingDuration);

        return (truncatedPreceding, blendSeg, truncatedFollowing);
    }
}
