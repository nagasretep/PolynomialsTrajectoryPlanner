using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.Services;

/// <summary>Request 0009, Part B: logic moved from the App into the Core.</summary>
public class SegmentFactoryTests
{
    private static readonly State1D A = State1D.FromPositionOnly(0);
    private static readonly State1D B = State1D.FromPositionOnly(1);

    [Fact]
    public void NoLimitsNoDuration_UsesTheFallback()
    {
        Assert.Equal(SegmentFactory1D.DefaultDuration, SegmentFactory1D.Build(9, A, B).Duration, 12);
        Assert.Equal(2.5, SegmentFactory1D.Build(9, A, B, fallbackDuration: 2.5).Duration, 12);
    }

    [Fact]
    public void LimitsNoDuration_UsesTheShortestFeasibleDuration()
    {
        var segment = SegmentFactory1D.Build(9, A, B, KinematicLimits.FromValues(vMax: 1));

        Assert.Equal(315.0 / 128, segment.Duration, 1e-6);
    }

    [Fact]
    public void PrescribedDuration_IsKept_WithOrWithoutSatisfiedLimits()
    {
        Assert.Equal(3.0, SegmentFactory1D.Build(9, A, B, duration: 3.0).Duration, 12);
        Assert.Equal(3.0, SegmentFactory1D.Build(9, A, B, KinematicLimits.FromValues(vMax: 5), 3.0).Duration, 12);
    }

    [Fact]
    public void PrescribedDurationViolatingTheLimits_IsReportedNotRepaired()
    {
        Assert.Throws<KinematicLimitsViolatedException>(
            () => SegmentFactory1D.Build(9, A, B, KinematicLimits.FromValues(vMax: 1), 1.0));
    }

    [Fact]
    public void LimitsThatDetermineNothing_UseTheFallback()
    {
        var segment = SegmentFactory1D.Build(9, B, B, KinematicLimits.FromValues(vMax: 1), fallbackDuration: 1.5);

        Assert.Equal(1.5, segment.Duration, 12);
    }

    [Fact]
    public void ImpossibleLimits_ReportTheSearchFailure()
    {
        Assert.Throws<DurationSearchFailedException>(
            () => SegmentFactory1D.Build(5, new State1D(0, 10, 0, 0, 0), B, KinematicLimits.FromValues(vMax: 1)));
    }

    [Fact]
    public void NonPositivePrescribedDuration_Rejected()
    {
        Assert.Throws<ArgumentException>(() => SegmentFactory1D.Build(9, A, B, duration: 0.0));
    }
}

public class KinematicLimitsCombinationTests
{
    [Fact]
    public void MostRestrictive_TakesTheSmallerOfEachKind_AndTheOneThatIsSet()
    {
        var a = KinematicLimits.FromValues(vMax: 5, aMax: 10);
        var b = KinematicLimits.FromValues(vMax: 3, jMax: 7);

        var c = KinematicLimits.MostRestrictive(a, b);

        Assert.Equal(3.0, c.MaxVelocity);
        Assert.Equal(10.0, c.MaxAcceleration);
        Assert.Equal(7.0, c.MaxJerk);
        Assert.Null(c.MaxSnap);
        Assert.True(c.HasAnyLimit);
    }

    [Fact]
    public void HasAnyLimit_IsFalseForNone()
    {
        Assert.False(KinematicLimits.None.HasAnyLimit);
        Assert.False(KinematicLimits.MostRestrictive(KinematicLimits.None, KinematicLimits.None).HasAnyLimit);
    }
}

public class TrajectorySamplingTests
{
    private static Trajectory1D TwoSegments() => new(new[]
    {
        PolynomialSegment1D.CreateDegree3(0, 1, 0, 0, 1.0),
        PolynomialSegment1D.CreateDegree3(1, 3, 0, 0, 3.0)
    });

    [Fact]
    public void Sample_IsUniformFromZeroToTheTotalDuration_WithPositionAndDerivatives()
    {
        var trajectory = TwoSegments();

        var samples = trajectory.Sample(5, 2);

        Assert.Equal(new[] { 0.0, 1.0, 2.0, 3.0, 4.0 }, samples.Times);
        Assert.Equal(3, samples.Values.Length); // position, velocity, acceleration
        Assert.Equal(0.0, samples.Values[0][0], 12);
        Assert.Equal(3.0, samples.Values[0][4], 12);
        for (int i = 0; i < 5; i++)
        {
            Assert.Equal(trajectory.EvaluatePosition(samples.Times[i]), samples.Values[0][i], 12);
            Assert.Equal(trajectory.EvaluateDerivative(samples.Times[i], 1), samples.Values[1][i], 12);
            Assert.Equal(trajectory.EvaluateDerivative(samples.Times[i], 2), samples.Values[2][i], 12);
        }
    }

    [Fact]
    public void Sample_RejectsTooFewSamplesAndNegativeOrder()
    {
        var trajectory = TwoSegments();

        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.Sample(1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.Sample(10, -1));
    }
}
