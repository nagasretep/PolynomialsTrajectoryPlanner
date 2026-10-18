using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Constraints;

/// <summary>Request 0014: the peak of a whole trajectory and the segment where it occurs.</summary>
public class PeakLocationTests
{
    private static PolynomialSegment1D Rest(double x0, double x1, double duration) =>
        new(9, State1D.FromPositionOnly(x0), State1D.FromPositionOnly(x1), duration);

    [Fact]
    public void PeakIsTheSameAsPeakAbsolute_AndTheSegmentIsTheFasterOne()
    {
        // Same distance, second segment twice as short: its velocity peak is twice as large.
        var segments = new[] { Rest(0, 1, 2.0), Rest(1, 2, 1.0) };

        var (peak, index, time) = ExtremaAnalyzer.PeakAbsoluteWithLocation(segments, 1);

        Assert.Equal(ExtremaAnalyzer.PeakAbsolute(segments, 1), peak, 12);
        Assert.Equal(1, index);
        Assert.Equal(2.4609375, peak, 1e-6); // 315/128 over a 1 s rest-to-rest move of distance 1
        Assert.InRange(time, 0.0, 1.0);
        Assert.Equal(0.5, time, 1e-2);
    }

    [Fact]
    public void ThePeakOfANegativeExcursion_IsReportedAsAbsoluteValue()
    {
        var segments = new[] { Rest(0, -1, 1.0), Rest(-1, -1.5, 1.0) };

        var (peak, index, _) = ExtremaAnalyzer.PeakAbsoluteWithLocation(segments, 1);

        Assert.Equal(0, index);
        Assert.Equal(2.4609375, peak, 1e-6);
    }
}
