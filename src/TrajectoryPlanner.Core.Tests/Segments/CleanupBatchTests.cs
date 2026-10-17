using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.Segments;

/// <summary>Request 0003: SampleTimes (D12), DurationSearcher failure (D5, bug part), proportional durations (D14).</summary>
public class SampleTimesTests
{
    [Fact]
    public void SameSegmentObjectTwice_GivesTwoCorrectTimeRanges()
    {
        var segment = PolynomialSegment1D.CreateDegree3(0, 1, 0, 0, 2.0);
        var trajectory = new Trajectory1D(new[] { segment, segment });

        double[] times = trajectory.SampleTimes(5).ToArray();

        Assert.Equal(10, times.Length);
        Assert.Equal(new[] { 0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 2.5, 3.0, 3.5, 4.0 }, times);
    }

    [Fact]
    public void DifferentDurations_UseTheirOwnStartTimes()
    {
        var first = PolynomialSegment1D.CreateDegree3(0, 1, 0, 0, 1.0);
        var second = PolynomialSegment1D.CreateDegree3(1, 3, 0, 0, 3.0);

        double[] times = new Trajectory1D(new[] { first, second }).SampleTimes(3).ToArray();

        Assert.Equal(new[] { 0.0, 0.5, 1.0, 1.0, 2.5, 4.0 }, times);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(0)]
    [InlineData(-3)]
    public void FewerThanTwoSamples_RejectedImmediately(int samples)
    {
        var trajectory = new Trajectory1D(new[] { PolynomialSegment1D.CreateDegree3(0, 1, 0, 0, 1.0) });

        // Rejected when called, not only when the sequence is enumerated.
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.SampleTimes(samples));
    }
}

public class DurationSearchFailureTests
{
    [Fact]
    public void VelocityLimitFarTooSmallForTheDistance_ReportsFailure()
    {
        // Needs T of order 1.5e18 s; the search stops after 50 doublings (about 1e15 s).
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(1e9);
        var limits = KinematicLimits.FromValues(vMax: 1e-9);

        var ex = Assert.Throws<DurationSearchFailedException>(
            () => DurationSearcher.FindValidDuration(3, start, end, limits));

        Assert.True(ex.LastDurationTried > 1e14);
        Assert.Contains("limits", ex.Message);
    }

    [Fact]
    public void BoundaryVelocityAboveTheVelocityLimit_ReportsFailure()
    {
        // A segment that starts at speed 10 can never keep |v| <= 1, whatever its duration.
        var start = new State1D(0, 10, 0, 0, 0);
        var end = new State1D(1, 0, 0, 0, 0);
        var limits = KinematicLimits.FromValues(vMax: 1.0);

        Assert.Throws<DurationSearchFailedException>(
            () => DurationSearcher.FindValidDuration(3, start, end, limits));
    }

    [Fact]
    public void ConstraintAwareDuration_PropagatesTheFailure()
    {
        var strategy = new ConstraintAwareDuration(KinematicLimits.FromValues(vMax: 1.0));

        Assert.Throws<DurationSearchFailedException>(
            () => strategy.ComputeDuration(3, new State1D(0, 10, 0, 0, 0), State1D.FromPositionOnly(1), 0, 1));
    }

    [Fact]
    public void AchievableLimits_StillReturnADuration()
    {
        var limits = KinematicLimits.FromValues(vMax: 1.0);

        double duration = DurationSearcher.FindValidDuration(
            3, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), limits);

        Assert.True(duration >= 1.5 - 1e-6); // degree-3 peak velocity is 1.5 * distance / T
    }
}

public class ProportionalDurationTests
{
    private static void AssertSequence(double[] expected, double[] actual, double tolerance = 1e-12)
    {
        Assert.Equal(expected.Length, actual.Length);
        for (int i = 0; i < expected.Length; i++)
            Assert.Equal(expected[i], actual[i], tolerance);
    }

    [Fact]
    public void SumEqualsTheTotalAndDurationsAreProportionalToDistance()
    {
        double[] durations = new AutoDerivedProportionalDuration(10.0).ComputeDurations(new[] { 0.0, 1, 4, 6, 6.5 }, 9);

        // distances 1, 3, 2, 0.5: all above the floor 0.05 * mean = 0.0875
        AssertSequence(new[] { 10 * 1 / 6.5, 10 * 3 / 6.5, 10 * 2 / 6.5, 10 * 0.5 / 6.5 }, durations);
        Assert.Equal(10.0, durations.Sum(), 12);
    }

    [Fact]
    public void ZeroDistanceSegment_GetsTheFloorShare_AndTheTotalIsStillExact()
    {
        double[] durations = new AutoDerivedProportionalDuration(4.0).ComputeDurations(new[] { 0.0, 2, 2, 4 }, 9);

        // distances 2, 0, 2; mean 4/3; floor weight 0.05 * 4/3; weights sum 4 + 0.05 * 4/3
        double floor = 0.05 * 4.0 / 3.0;
        double sum = 4.0 + floor;
        AssertSequence(new[] { 4 * 2 / sum, 4 * floor / sum, 4 * 2 / sum }, durations);
        Assert.True(durations[1] > 0);
        Assert.Equal(4.0, durations.Sum(), 12);
    }

    [Fact]
    public void FloorIsAConstructorParameter()
    {
        double[] durations = new AutoDerivedProportionalDuration(3.0, relativeFloor: 0.5).ComputeDurations(new[] { 0.0, 3, 3, 6 }, 9);

        // distances 3, 0, 3; mean 2; floor weight 1; weights 3, 1, 3
        AssertSequence(new[] { 9.0 / 7, 3.0 / 7, 9.0 / 7 }, durations);
    }

    [Fact]
    public void TinyTotal_IsRespected()
    {
        // The old absolute 0.05 s floor would have given a sum of at least 0.15 s here.
        double[] durations = new AutoDerivedProportionalDuration(0.03).ComputeDurations(new[] { 0.0, 1, 1, 2 }, 9);

        Assert.Equal(0.03, durations.Sum(), 12);
    }

    [Fact]
    public void AllDistancesZero_GivesAnEqualSplit()
    {
        double[] durations = new AutoDerivedProportionalDuration(6.0).ComputeDurations(new[] { 1.0, 1, 1, 1 }, 9);

        AssertSequence(new[] { 2.0, 2.0, 2.0 }, durations);
    }

    [Fact]
    public void ThreeDimensions_UsesTheEuclideanDistance()
    {
        var waypoints = new[] { new Point3D(0, 0, 0), new Point3D(3, 4, 0), new Point3D(3, 4, 12) };

        double[] durations = new AutoDerivedProportionalDuration(17.0).ComputeDurations(waypoints);

        // distances 5 and 12
        AssertSequence(new[] { 5.0, 12.0 }, durations);
    }
}
