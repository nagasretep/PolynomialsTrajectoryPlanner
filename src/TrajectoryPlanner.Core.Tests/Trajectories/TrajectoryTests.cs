using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.Trajectories;

public class Trajectory1DTests
{
    // Three degree-9 segments chained through shared states: durations 1.0, 0.001, 2.0.
    private static Trajectory1D Chain()
    {
        var s0 = State1D.FromPositionOnly(0);
        var s1 = new State1D(1, 0.5, 0, 0, 0);
        var s2 = new State1D(1.0005, 0.5, 0, 0, 0);
        var s3 = State1D.FromPositionOnly(4);
        return new Trajectory1D(new[]
        {
            new PolynomialSegment1D(9, s0, s1, 1.0),
            new PolynomialSegment1D(9, s1, s2, 0.001),
            new PolynomialSegment1D(9, s2, s3, 2.0)
        });
    }

    [Fact]
    public void TotalDuration_IsTheSumOfTheSegmentDurations()
    {
        Assert.Equal(3.001, Chain().TotalDuration, 12);
    }

    [Fact]
    public void Evaluation_AtStartAndAtTotalDuration_GivesTheBoundaryStates()
    {
        var trajectory = Chain();

        Assert.Equal(0, trajectory.EvaluatePosition(0), 1e-9);
        Assert.Equal(4, trajectory.EvaluatePosition(trajectory.TotalDuration), 1e-9);
        Assert.Equal(0, trajectory.EvaluateDerivative(trajectory.TotalDuration, 1), 1e-9);
    }

    [Fact]
    public void Evaluation_AtAnInteriorNode_BelongsToTheFollowingSegmentAndIsConsistent()
    {
        var trajectory = Chain();

        var (index, localTime) = trajectory.FindSegment(1.0);
        Assert.Equal(1, index);
        Assert.Equal(0.0, localTime, 12);

        // The value at the node equals the limit from either side.
        double at = trajectory.EvaluatePosition(1.0);
        double before = trajectory.Segments[0].EvaluatePosition(trajectory.Segments[0].Duration);
        double after = trajectory.Segments[1].EvaluatePosition(0);
        Assert.Equal(before, at, 1e-9);
        Assert.Equal(after, at, 1e-9);
    }

    [Fact]
    public void VeryShortSegment_IsFoundAndNotSkipped()
    {
        var trajectory = Chain();

        var (index, localTime) = trajectory.FindSegment(1.0005);

        Assert.Equal(1, index);
        Assert.Equal(0.0005, localTime, 1e-12);
    }

    [Fact]
    public void FindSegment_AtTotalDuration_ReturnsTheEndOfTheLastSegment()
    {
        var trajectory = Chain();

        var (index, localTime) = trajectory.FindSegment(trajectory.TotalDuration);

        Assert.Equal(2, index);
        Assert.Equal(2.0, localTime, 12);
    }

    [Fact]
    public void OutsideTheRange_ThrowsArgumentOutOfRange_ExceptForAToleranceOfOneE12AtTheEnd()
    {
        var trajectory = Chain();

        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.EvaluatePosition(-1e-9));
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.EvaluatePosition(trajectory.TotalDuration + 1e-6));
        Assert.Equal(4, trajectory.EvaluatePosition(trajectory.TotalDuration + 1e-13), 1e-9);
    }

    [Fact]
    public void Position_AndVelocity_AreContinuousAcrossAllNodes()
    {
        var trajectory = Chain();
        double time = 0;
        for (int k = 0; k < trajectory.Segments.Count - 1; k++)
        {
            time += trajectory.Segments[k].Duration;
            double eps = 1e-9;
            Assert.Equal(trajectory.EvaluatePosition(time - eps), trajectory.EvaluatePosition(time + eps), 1e-6);
            Assert.Equal(trajectory.EvaluateDerivative(time - eps, 1), trajectory.EvaluateDerivative(time + eps, 1), 1e-6);
        }
    }

    [Fact]
    public void EvaluateState_MatchesThePositionAndDerivativeEvaluations()
    {
        var trajectory = Chain();
        double t = 1.7;

        var state = trajectory.EvaluateState(t);

        Assert.Equal(trajectory.EvaluatePosition(t), state.Position, 1e-9);
        Assert.Equal(trajectory.EvaluateDerivative(t, 1), state.Velocity, 1e-9);
        Assert.Equal(trajectory.EvaluateDerivative(t, 2), state.Acceleration, 1e-9);
    }
}

public class Trajectory3DTests
{
    private static Trajectory3D Chain()
    {
        var a = State3D.FromPositionOnly(new Point3D(0, 0, 0));
        var b = new State3D(new Point3D(1, 2, 3), new Point3D(0.5, 0.5, 0.5), Point3D.Zero, Point3D.Zero, Point3D.Zero);
        var c = State3D.FromPositionOnly(new Point3D(4, 0, -1));
        return new Trajectory3D(new[]
        {
            new PolynomialSegment3D(9, a, b, 1.0),
            new PolynomialSegment3D(9, b, c, 2.0)
        });
    }

    [Fact]
    public void TotalDuration_AndBoundaryEvaluation()
    {
        var trajectory = Chain();

        Assert.Equal(3.0, trajectory.TotalDuration, 12);
        var start = trajectory.EvaluatePosition(0);
        var end = trajectory.EvaluatePosition(trajectory.TotalDuration);
        Assert.Equal(0, start.X, 1e-9);
        Assert.Equal(4, end.X, 1e-9);
        Assert.Equal(-1, end.Z, 1e-9);
    }

    [Fact]
    public void InteriorNode_IsConsistentFromBothSides_AndFindSegmentPicksTheFollowingSegment()
    {
        var trajectory = Chain();

        var (index, localTime) = trajectory.FindSegment(1.0);
        Assert.Equal(1, index);
        Assert.Equal(0.0, localTime, 12);

        var atNode = trajectory.EvaluatePosition(1.0);
        var before = trajectory.Segments[0].EvaluatePosition(1.0);
        Assert.Equal(before.X, atNode.X, 1e-9);
        Assert.Equal(before.Y, atNode.Y, 1e-9);
        Assert.Equal(before.Z, atNode.Z, 1e-9);
    }

    [Fact]
    public void Velocity_IsContinuousAcrossTheNode()
    {
        var trajectory = Chain();

        var left = trajectory.EvaluateDerivative(1.0 - 1e-9, 1);
        var right = trajectory.EvaluateDerivative(1.0 + 1e-9, 1);

        Assert.Equal(left.X, right.X, 1e-6);
        Assert.Equal(left.Y, right.Y, 1e-6);
        Assert.Equal(left.Z, right.Z, 1e-6);
    }

    [Fact]
    public void OutsideTheRange_Throws()
    {
        var trajectory = Chain();

        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.EvaluatePosition(-0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.EvaluatePosition(3.1));
    }
}
