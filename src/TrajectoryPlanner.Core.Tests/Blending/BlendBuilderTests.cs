using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Blending;

/// <summary>
/// Request 0004, item 1. Tests named "Today_…" pin the behaviour of the present implementation, which is
/// not (or not fully) prescribed by theory/BlendingSegments.md (discrepancy D7): they are expected to
/// change knowingly in the consolidation.
/// </summary>
public class BlendBuilderTests
{
    // Two degree-9 segments meeting at x = 2 with a velocity reversal (a corner in 1D).
    private static (PolynomialSegment1D Pre, PolynomialSegment1D Post) Corner()
    {
        var pre = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), new State1D(2, 2, 0, 0, 0), 1.0);
        var post = new PolynomialSegment1D(9, new State1D(2, -2, 0, 0, 0), State1D.FromPositionOnly(0.5), 1.5);
        return (pre, post);
    }

    private static void AssertSameState(State1D expected, State1D actual, double tolerance)
    {
        for (int order = 0; order <= 4; order++)
            Assert.Equal(expected.GetDerivative(order), actual.GetDerivative(order), tolerance);
    }

    [Fact]
    public void BlendStartsAndEndsWithTheStatesSampledFromTheNeighbours()
    {
        var (pre, post) = Corner();
        double tauPre = 0.7, tauPost = 0.3;

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, tauPre, tauPost);

        State1D expectedStart = pre.EvaluateState(tauPre * pre.Duration);
        State1D expectedEnd = post.EvaluateState(tauPost * post.Duration);

        AssertSameState(expectedStart, blend.StartState, 1e-12);
        AssertSameState(expectedEnd, blend.EndState, 1e-12);

        // ... and the polynomial really reproduces them (position ... snap).
        AssertSameState(expectedStart, blend.EvaluateState(0), 1e-6);
        AssertSameState(expectedEnd, blend.EvaluateState(blend.Duration), 1e-6);
    }

    [Fact]
    public void TruncatedNeighboursJoinTheBlendWithContinuityUpToSnap()
    {
        var (pre, post) = Corner();

        var (truncatedPre, blend, truncatedPost) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3);

        AssertSameState(truncatedPre.EvaluateState(truncatedPre.Duration), blend.EvaluateState(0), 1e-6);
        AssertSameState(blend.EvaluateState(blend.Duration), truncatedPost.EvaluateState(0), 1e-6);

        // The outer ends are untouched.
        AssertSameState(pre.StartState, truncatedPre.EvaluateState(0), 1e-9);
        AssertSameState(post.EndState, truncatedPost.EvaluateState(truncatedPost.Duration), 1e-9);
    }

    [Fact]
    public void TruncatedNeighbours_KeepTheOriginalShapeOnTheKeptPortions()
    {
        var (pre, post) = Corner();
        var (truncatedPre, _, truncatedPost) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3);

        // Same polynomial path, only cut: compare positions at corresponding times.
        Assert.Equal(0.7, truncatedPre.Duration, 12);
        Assert.Equal(pre.EvaluatePosition(0.35), truncatedPre.EvaluatePosition(0.35), 1e-6);

        double offset = 0.3 * post.Duration;
        Assert.Equal(post.Duration - offset, truncatedPost.Duration, 12);
        Assert.Equal(post.EvaluatePosition(offset + 0.4), truncatedPost.EvaluatePosition(0.4), 1e-6);
    }

    [Fact]
    public void InheritedDuration_IsTheSumOfTheRemovedPortions()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3);

        // removed from the preceding segment: (1 - 0.7) * 1.0; from the following: 0.3 * 1.5
        Assert.Equal(0.3 * 1.0 + 0.3 * 1.5, blend.Duration, 12);
    }

    [Fact]
    public void ExplicitBlendDuration_IsUsed()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 2.0);

        Assert.Equal(2.0, blend.Duration, 12);
    }

    [Fact]
    public void Today_InheritedDurationHasAFloorOfOneHundredthOfASecond()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 1.0, 0.0);

        Assert.Equal(0.01, blend.Duration, 12);
    }

    [Fact]
    public void AbsolutePositions_MapToTheTauWhereTheNeighbourHasThatPosition()
    {
        var (pre, post) = Corner();
        // The preceding segment is monotone up to x = 2; the following one is monotone down from 2.
        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, new Point1D(1.5), new Point1D(1.2));

        Assert.Equal(1.5, blend.StartState.Position, 1e-6);
        Assert.Equal(1.2, blend.EndState.Position, 1e-6);

        // The sampled state is that of the neighbour at the corresponding time.
        AssertSameState(pre.EvaluateState(FindTime(pre, 1.5)), blend.StartState, 1e-4);
        AssertSameState(post.EvaluateState(FindTime(post, 1.2)), blend.EndState, 1e-4);
    }

    private static double FindTime(PolynomialSegment1D segment, double position)
    {
        double lo = 0, hi = segment.Duration;
        bool increasing = segment.EndState.Position > segment.StartState.Position;
        for (int i = 0; i < 200; i++)
        {
            double mid = (lo + hi) / 2;
            bool below = segment.EvaluatePosition(mid) < position;
            if (below == increasing) lo = mid; else hi = mid;
        }

        return (lo + hi) / 2;
    }

    [Fact]
    public void BlendAtACorner_DoesNotPassThroughTheNominalJoin()
    {
        var (pre, post) = Corner();
        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.8, 0.2);

        double join = 2.0;
        double closest = double.MaxValue;
        for (int i = 0; i <= 1000; i++)
        {
            double t = blend.Duration * i / 1000;
            closest = Math.Min(closest, Math.Abs(blend.EvaluatePosition(t) - join));
        }

        Assert.True(closest > 1e-3, $"blend comes within {closest} of the nominal join");
    }

    [Theory]
    [InlineData(-0.1, 0.5)]
    [InlineData(1.1, 0.5)]
    [InlineData(0.5, -0.1)]
    [InlineData(0.5, 1.1)]
    public void TauOutsideZeroOne_Rejected(double tauPre, double tauPost)
    {
        var (pre, post) = Corner();

        Assert.Throws<ArgumentOutOfRangeException>(() => BlendBuilder1D.BuildBlendSegment(pre, post, tauPre, tauPost));
    }

    [Fact]
    public void Today_PositionOutsideTheSegmentIsClampedNotRejected()
    {
        var (pre, post) = Corner();

        // x = 5 is beyond the end (2) of the preceding segment: no exception, the blend starts near its end.
        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, new Point1D(5.0), new Point1D(1.2));

        Assert.True(blend.StartState.Position <= 2.0 + 1e-9);
        Assert.True(blend.StartState.Position > 1.9);
    }

    [Fact]
    public void Today_EmptyWindow_IsNotRejected()
    {
        var (pre, post) = Corner();

        // tau = 1 on the preceding and tau = 0 on the following segment: nothing is removed.
        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 1.0, 0.0);

        Assert.True(blend.Duration > 0);
    }

    [Fact]
    public void Today_BlendDegreeIsNineByDefault()
    {
        var pre = PolynomialSegment1D.CreateDegree3(0, 2, 0, 1, 1.0);
        var post = PolynomialSegment1D.CreateDegree3(2, 4, 1, 0, 1.0);

        Assert.Equal(9, BlendBuilder1D.BuildBlendSegment(pre, post, 0.5, 0.5).Degree);
    }

    [Fact]
    public void Today_AbsolutePositionTauIsClampedAwayFromTheEndpoints()
    {
        var (pre, post) = Corner();

        // Asking for exactly the start position of the following segment cannot give tau = 0 (clamp at 0.001).
        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, new Point1D(1.5), new Point1D(2.0));

        Assert.True(blend.EndState.Position < 2.0);
    }
}
