using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using Xunit;

namespace TrajectoryPlanner.Core.Tests.Segments;

public class PolynomialSegment1DTests
{
    [Fact]
    public void RestToRest_Degree9_EndpointsCorrect()
    {
        var start = State1D.FromPositionOnly(0.0);
        var end = State1D.FromPositionOnly(10.0);
        double T = 2.0;

        var seg = new PolynomialSegment1D(9, start, end, T);

        Assert.Equal(start, seg.EvaluateState(0.0));
        Assert.Equal(end, seg.EvaluateState(T));
    }

    [Fact]
    public void Degree3_LinearRamp_EvaluatesCorrectly()
    {
        double x0 = 0, xf = 10, v0 = 5, vf = 5, T = 2.0;
        var seg = PolynomialSegment1D.CreateDegree3(x0, xf, v0, vf, T);

        double midPos = seg.EvaluatePosition(T / 2);
        double midVel = seg.EvaluateDerivative(T / 2, 1);

        Assert.Equal(5.0, midPos, 1e-9);
        Assert.Equal(5.0, midVel, 1e-9);
    }

    [Fact]
    public void FindExtrema_Degree3RestToRest_VelocityPeakAtMidpoint()
    {
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(10);
        double T = 2.0;
        var seg = new PolynomialSegment1D(3, start, end, T);

        var vExtrema = ExtremaAnalyzer.VelocityExtrema(seg);

        Assert.True(vExtrema.Maximum > 0);
        Assert.Equal(T / 2, vExtrema.MaxTime, tolerance: 0.1);
    }

    [Fact]
    public void VerifyLimits_ValidLimits_Passes()
    {
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(10);
        double T = 5.0;
        var seg = new PolynomialSegment1D(9, start, end, T);

        var limits = KinematicLimits.FromValues(vMax: 10, aMax: 10, jMax: 100);
        bool valid = ExtremaAnalyzer.VerifyLimits(seg, limits);

        Assert.True(valid);
    }
}

public class ConstraintsTests
{
    [Fact]
    public void DurationSearcher_FindsValidDurationForDegree3()
    {
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(10);
        var limits = KinematicLimits.FromValues(vMax: 4, aMax: 5);

        double T = DurationSearcher.FindValidDuration(3, start, end, limits, 0.1);

        var seg = new PolynomialSegment1D(3, start, end, T);
        bool valid = ExtremaAnalyzer.VerifyLimits(seg, limits);

        Assert.True(T > 0);
        Assert.True(valid);
    }
}
