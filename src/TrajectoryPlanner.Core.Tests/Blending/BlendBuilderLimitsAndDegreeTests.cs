using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Blending;

/// <summary>Request 0009, Part A: selectable degree, limit verification, input validation.</summary>
public class BlendBuilderLimitsAndDegreeTests
{
    private static (PolynomialSegment1D Pre, PolynomialSegment1D Post) Corner(int degree = 9)
    {
        var pre = new PolynomialSegment1D(degree, State1D.FromPositionOnly(0), new State1D(2, 2, 0, 0, 0), 1.0);
        var post = new PolynomialSegment1D(degree, new State1D(2, -2, 0, 0, 0), State1D.FromPositionOnly(0.5), 1.5);
        return (pre, post);
    }

    private static void AssertRelative(double expected, double actual, double tolerance, string what) =>
        Assert.True(Math.Abs(expected - actual) <= tolerance * Math.Abs(expected), $"{what}: expected {expected:R}, got {actual:R}");

    // ---- degree ----------------------------------------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void ExplicitBlendDegree_IsUsed(int degree)
    {
        var (pre, post) = Corner();

        Assert.Equal(degree, BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, degree).Degree);
    }

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void DefaultDegree_IsTheNeighbourDegreeWhenTheyAgree(int degree)
    {
        var (pre, post) = Corner(degree);

        Assert.Equal(degree, BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3).Degree);
        Assert.Equal(degree, BlendBuilder1D.DefaultBlendDegree(pre, post));
    }

    [Fact]
    public void DefaultDegree_IsNineWhenTheNeighboursDisagree()
    {
        var pre = PolynomialSegment1D.CreateDegree3(0, 2, 0, 1, 1.0);
        var post = PolynomialSegment1D.CreateDegree5(2, 4, 1, 0, 0, 0, 1.0);

        Assert.Equal(9, BlendBuilder1D.BuildBlendSegment(pre, post, 0.5, 0.5).Degree);
    }

    [Fact]
    public void UnsupportedDegree_IsRejected()
    {
        var (pre, post) = Corner();

        Assert.Throws<ArgumentException>(() => BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, 4));
    }

    [Fact]
    public void Degree5Blend_BetweenDegree5Neighbours_IsContinuousUpToAcceleration()
    {
        var (pre, post) = Corner(5);

        var (truncatedPre, blend, truncatedPost) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3);

        Assert.Equal(5, blend.Degree);
        for (int order = 0; order <= 2; order++)
        {
            Assert.Equal(truncatedPre.EvaluateDerivative(truncatedPre.Duration, order), blend.EvaluateDerivative(0, order), 1e-6);
            Assert.Equal(blend.EvaluateDerivative(blend.Duration, order), truncatedPost.EvaluateDerivative(0, order), 1e-6);
        }
    }

    // ---- limits ----------------------------------------------------------------------------

    private static KinematicLimits LimitsOfKind(string kind, double value) => kind switch
    {
        "v" => KinematicLimits.FromValues(vMax: value),
        "a" => KinematicLimits.FromValues(aMax: value),
        "j" => KinematicLimits.FromValues(jMax: value),
        _ => KinematicLimits.FromValues(sMax: value)
    };

    [Theory]
    [InlineData("v", 4.0)]
    [InlineData("a", 40.0)]
    [InlineData("j", 400.0)]
    [InlineData("s", 5000.0)]
    public void WithAnActiveLimit_AnInheritedDurationThatIsFeasibleIsKept_AndPassesVerifyLimits(string kind, double value)
    {
        // Request 0010 (D19): rewritten from "takes the shortest feasible duration" (0009). With these loose
        // limits the inherited blend (0.3 + 0.3 * 1.5 = 0.75 s) is already feasible, so it is kept.
        var (pre, post) = Corner();
        var limits = LimitsOfKind(kind, value);

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, limits: limits);

        Assert.Equal(0.3 * 1.0 + 0.3 * 1.5, blend.Duration, 12);
        Assert.True(ExtremaAnalyzer.VerifyLimits(blend, limits));

        // The shortest feasible duration would have been shorter: it is not what was used.
        double shortest = ShortestFeasibleDuration.Find(9, blend.StartState, blend.EndState, limits)!.Value;
        Assert.True(shortest < blend.Duration);
    }

    [Theory]
    [InlineData("v")]
    [InlineData("a")]
    [InlineData("j")]
    [InlineData("s")]
    public void WithALimitThatTheInheritedBlendViolates_TheShortestFeasibleDurationIsUsed(string kind)
    {
        var (pre, post) = Corner();
        int order = kind switch { "v" => 1, "a" => 2, "j" => 3, _ => 4 };

        // A window whose inherited blend has a peak clearly above its boundary values (which no duration can change),
        // so that a limit 10 % under the peak is reachable. The peak sits at a boundary for some windows.
        (double Start, double End)? window = null;
        BlendBuilderResult? chosen = null;
        foreach (var candidate in new[] { (0.3, 0.7), (0.7, 0.3), (0.5, 0.5), (0.2, 0.8), (0.8, 0.2) })
        {
            var inherited = BlendBuilder1D.BuildBlendSegment(pre, post, candidate.Item1, candidate.Item2);
            double peak = ExtremaAnalyzer.FindExtrema(inherited, order).MaxAbsolute;
            double boundary = Math.Max(Math.Abs(inherited.StartState.GetDerivative(order)), Math.Abs(inherited.EndState.GetDerivative(order)));
            if (0.9 * peak > 1.05 * boundary)
            {
                window = candidate;
                chosen = new BlendBuilderResult(inherited, 0.9 * peak);
                break;
            }
        }

        Assert.NotNull(window);
        var limits = LimitsOfKind(kind, chosen!.Limit);
        Assert.False(ExtremaAnalyzer.VerifyLimits(chosen.Inherited, limits)); // precondition: the inherited blend is infeasible

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, window!.Value.Start, window.Value.End, limits: limits);

        double expected = ShortestFeasibleDuration.Find(9, blend.StartState, blend.EndState, limits)!.Value;
        AssertRelative(expected, blend.Duration, 1e-9, "blend duration");
        Assert.NotEqual(chosen.Inherited.Duration, blend.Duration, 6); // not the inherited one (may be shorter or longer)
        Assert.True(ExtremaAnalyzer.VerifyLimits(blend, limits));

        // The limit is binding: the shortest feasible duration sits at F = 0.
        double f = ShortestFeasibleDuration.Feasibility(9, blend.StartState, blend.EndState, limits, blend.Duration);
        Assert.InRange(f, -1e-6, 0.0);
    }

    private sealed record BlendBuilderResult(PolynomialSegment1D Inherited, double Limit);

    [Fact]
    public void KeepingTheInheritedDuration_LeavesTheTotalTrajectoryDurationUnchanged()
    {
        var (pre, post) = Corner();
        var limits = KinematicLimits.FromValues(vMax: 8.0);

        var (truncatedPre, blend, truncatedPost) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3, limits: limits);

        Assert.Equal(pre.Duration + post.Duration, truncatedPre.Duration + blend.Duration + truncatedPost.Duration, 12);
    }

    [Fact]
    public void WithLimits_TheWholeBlendedTrajectoryPassesVerifyLimits()
    {
        var (pre, post) = Corner();
        var limits = KinematicLimits.FromValues(vMax: 6.0, aMax: 60.0);

        var (truncatedPre, blend, truncatedPost) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3, limits: limits);

        foreach (var segment in new[] { truncatedPre, blend, truncatedPost })
            Assert.True(ExtremaAnalyzer.VerifyLimits(segment, limits));
    }

    [Fact]
    public void PrescribedBlendDuration_IsKept_WhenItRespectsTheLimits()
    {
        var (pre, post) = Corner();
        var limits = KinematicLimits.FromValues(vMax: 100.0);

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 1.7, limits: limits);

        Assert.Equal(1.7, blend.Duration, 12);
    }

    [Fact]
    public void PrescribedBlendDuration_ViolatingALimit_IsReportedNotRepaired()
    {
        var (pre, post) = Corner();
        var limits = KinematicLimits.FromValues(aMax: 5.0);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(
            () => BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 0.05, limits: limits));

        Assert.Contains("blend", ex.Message);
    }

    [Fact]
    public void InfeasibleLimits_GiveAnExplicitErrorNamingTheBlend()
    {
        var (pre, post) = Corner();
        // The blend starts at a speed of about 2, above this limit: no duration can satisfy it.
        var limits = KinematicLimits.FromValues(vMax: 0.1);

        var ex = Assert.Throws<DurationSearchFailedException>(
            () => BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, limits: limits));

        Assert.StartsWith("The blend:", ex.Message);
    }

    [Fact]
    public void LimitsWithNothingActive_BehaveLikeNoLimits()
    {
        var (pre, post) = Corner();

        var plain = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3);
        var none = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, limits: KinematicLimits.None);

        Assert.Equal(plain.Duration, none.Duration, 12);
    }

    // ---- positions and windows -------------------------------------------------------------

    [Fact]
    public void TauForPosition_FindsTheCrossing_AndIsConsistentWithTheSegment()
    {
        var (pre, _) = Corner();

        double tau = BlendBuilder1D.TauForPosition(pre, 1.3);

        Assert.Equal(1.3, pre.EvaluatePosition(tau * pre.Duration), 1e-9);
    }

    [Fact]
    public void TauForPosition_PositionAtTheSegmentEnd_IsClampedToTheUpperBound()
    {
        var (pre, _) = Corner();

        Assert.Equal(BlendBuilder1D.TauMax, BlendBuilder1D.TauForPosition(pre, 2.0), 12);
        Assert.Equal(BlendBuilder1D.TauMin, BlendBuilder1D.TauForPosition(pre, 0.0), 12);
    }

    [Fact]
    public void TauForPosition_NonMonotoneSegment_AcceptsThePeakAndRejectsBeyondIt()
    {
        // Overshoot: goes up to about 2.4 before coming back to 2.
        var overshoot = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), new State1D(2, -3, 0, 0, 0), 1.0);
        double peak = ExtremaAnalyzer.PositionExtrema(overshoot).Maximum;
        Assert.True(peak > 2.0);

        double tau = BlendBuilder1D.TauForPosition(overshoot, 2.0 + 0.5 * (peak - 2.0));
        Assert.InRange(tau, BlendBuilder1D.TauMin, BlendBuilder1D.TauMax);

        Assert.Throws<ArgumentException>(() => BlendBuilder1D.TauForPosition(overshoot, peak + 0.1));
    }

    [Fact]
    public void PairWithAbsolutePositions_BuildsTheSamePairAsWithTheCorrespondingTaus()
    {
        var (pre, post) = Corner();
        double tauPre = BlendBuilder1D.TauForPosition(pre, 1.5);
        double tauPost = BlendBuilder1D.TauForPosition(post, 1.2);

        var byPosition = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, new Point1D(1.5), new Point1D(1.2));
        var byTau = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, tauPre, tauPost);

        Assert.Equal(byTau.BlendSegment.Duration, byPosition.BlendSegment.Duration, 12);
        Assert.Equal(byTau.TruncatedPreceding.Duration, byPosition.TruncatedPreceding.Duration, 12);
    }

    [Fact]
    public void Pair_RejectsAWindowThatRemovesAWholeNeighbour()
    {
        var (pre, post) = Corner();

        Assert.Throws<ArgumentException>(() => BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.0, 0.3));
        Assert.Throws<ArgumentException>(() => BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 1.0));
    }
}
