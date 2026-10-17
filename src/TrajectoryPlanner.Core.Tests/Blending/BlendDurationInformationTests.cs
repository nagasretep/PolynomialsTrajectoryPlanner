using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Blending;

/// <summary>Request 0012: the blend reports how its duration was chosen.</summary>
public class BlendDurationInformationTests
{
    private static (PolynomialSegment1D Pre, PolynomialSegment1D Post) Corner()
    {
        var pre = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), new State1D(2, 2, 0, 0, 0), 1.0);
        var post = new PolynomialSegment1D(9, new State1D(2, -2, 0, 0, 0), State1D.FromPositionOnly(0.5), 1.5);
        return (pre, post);
    }

    [Fact]
    public void WithoutLimits_TheInheritedDurationIsUsed_AndNotAdjusted()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3);

        Assert.Equal(0.75, blend.InheritedDuration, 12);
        Assert.Equal(0.75, blend.Duration, 12);
        Assert.False(blend.AdjustedByLimits);
    }

    [Fact]
    public void WithLimitsThatTheInheritedBlendRespects_NotAdjusted()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, limits: KinematicLimits.FromValues(vMax: 8));

        Assert.False(blend.AdjustedByLimits);
        Assert.Equal(blend.InheritedDuration, blend.Duration, 12);
    }

    [Fact]
    public void WithALimitThatTheInheritedBlendViolates_AdjustedAndBothDurationsAreExposed()
    {
        var (pre, post) = Corner();
        var inherited = BlendBuilder1D.BuildBlendSegment(pre, post, 0.3, 0.7);
        double peak = ExtremaAnalyzer.VelocityExtrema(inherited).MaxAbsolute;
        var limits = KinematicLimits.FromValues(vMax: 0.9 * peak);

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.3, 0.7, limits: limits);

        Assert.True(blend.AdjustedByLimits);
        Assert.Equal(inherited.Duration, blend.InheritedDuration, 12);
        Assert.NotEqual(blend.InheritedDuration, blend.Duration);
    }

    [Fact]
    public void PrescribedDuration_IsNeverReportedAsAdjusted()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder1D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 1.7, limits: KinematicLimits.FromValues(vMax: 100));

        Assert.False(blend.AdjustedByLimits);
        Assert.Equal(1.7, blend.Duration, 12);
        Assert.Equal(0.75, blend.InheritedDuration, 12);
    }

    [Fact]
    public void ThePairMethodsReturnTheSameInformation()
    {
        var (pre, post) = Corner();

        var (_, blend, _) = BlendBuilder1D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3);

        Assert.Equal(0.75, blend.InheritedDuration, 12);
    }
}
