using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Constraints;

/// <summary>Request 0004, items 4 and 5.</summary>
public class ExtremaAnalyticTests
{
    // Rest-to-rest peak velocity is c * distance / T with c = 3/2, 15/8, 35/16, 315/128
    // (x'(tau) is proportional to tau^(p-1) (1-tau)^(p-1); for d = 9: 630 tau^4 (1-tau)^4 distance / T).
    [Theory]
    [InlineData(3, 3.0 / 2)]
    [InlineData(5, 15.0 / 8)]
    [InlineData(7, 35.0 / 16)]
    [InlineData(9, 315.0 / 128)]
    public void RestToRest_PeakVelocity_ValueAndLocation(int degree, double c)
    {
        double distance = 7.0, duration = 3.0;
        var segment = new PolynomialSegment1D(degree, State1D.FromPositionOnly(0), State1D.FromPositionOnly(distance), duration);

        var velocity = ExtremaAnalyzer.VelocityExtrema(segment);

        Assert.Equal(c * distance / duration, velocity.Maximum, c * distance / duration * 1e-9);
        Assert.Equal(duration / 2, velocity.MaxTime, 1e-6);
        Assert.Equal(0.0, velocity.Minimum, 1e-9); // velocity is zero at both ends and never negative
    }

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void RestToRest_AccelerationPeak_ScalesAsDistanceOverDurationSquared(int degree)
    {
        double PeakAcceleration(double distance, double duration)
        {
            var segment = new PolynomialSegment1D(degree, State1D.FromPositionOnly(0), State1D.FromPositionOnly(distance), duration);
            return ExtremaAnalyzer.AccelerationExtrema(segment).MaxAbsolute;
        }

        double reference = PeakAcceleration(2.0, 1.0);

        Assert.Equal(reference / 4, PeakAcceleration(2.0, 2.0), reference * 1e-9); // doubling T divides by 4
        Assert.Equal(reference * 3, PeakAcceleration(6.0, 1.0), reference * 3e-9); // tripling the distance multiplies by 3
    }

    [Fact]
    public void RestToRest_PositionExtrema_AreTheEndPositions()
    {
        var segment = new PolynomialSegment1D(9, State1D.FromPositionOnly(1), State1D.FromPositionOnly(5), 2.0);

        var position = ExtremaAnalyzer.PositionExtrema(segment);

        Assert.Equal(1.0, position.Minimum, 1e-9);
        Assert.Equal(5.0, position.Maximum, 1e-9);
    }

    [Fact]
    public void VerifyLimits_ViolatedVelocityLimit_Fails()
    {
        var segment = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(10), 5.0);
        // The peak velocity is 315/128 * 10 / 5 = 4.92

        Assert.False(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(vMax: 4.0)));
        Assert.True(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(vMax: 5.0)));
    }

    [Fact]
    public void VerifyLimits_EachLimitKindIsChecked()
    {
        var segment = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(10), 5.0);

        double peakA = ExtremaAnalyzer.AccelerationExtrema(segment).MaxAbsolute;
        double peakJ = ExtremaAnalyzer.JerkExtrema(segment).MaxAbsolute;
        double peakS = ExtremaAnalyzer.SnapExtrema(segment).MaxAbsolute;

        Assert.False(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(aMax: peakA * 0.99)));
        Assert.False(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(jMax: peakJ * 0.99)));
        Assert.False(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(sMax: peakS * 0.99)));
        Assert.True(ExtremaAnalyzer.VerifyLimits(segment, KinematicLimits.FromValues(aMax: peakA * 1.01, jMax: peakJ * 1.01, sMax: peakS * 1.01)));
    }
}

public class DurationStrategyTests
{
    [Fact]
    public void UserSpecified_CountMismatch_Rejected()
    {
        var strategy = new UserSpecifiedDuration(new[] { 1.0, 2.0 });

        Assert.Throws<ArgumentException>(() => strategy.ComputeDurations(new[] { 0.0, 1, 2, 3 }, 9));
    }

    [Fact]
    public void UserSpecified_ReturnsACopyOfTheGivenDurations()
    {
        var given = new[] { 1.0, 2.0 };
        var strategy = new UserSpecifiedDuration(given);

        double[] result = strategy.ComputeDurations(new[] { 0.0, 1, 2 }, 9);
        result[0] = 99;

        Assert.Equal(1.0, strategy.ComputeDurations(new[] { 0.0, 1, 2 }, 9)[0]);
    }
}
