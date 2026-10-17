using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.Trajectories;
using Xunit;

namespace TrajectoryPlanner.Core.Tests.Integration;

public class TrajectoryBuilderTests
{
    [Fact]
    public void Builder_CreatesTrajectory1D_WithMultipleSegments()
    {
        var trajectory = new TrajectoryBuilder1D()
            .WithDefaultDegree(5)
            .AddRestToRest(0, 10, 2.0)
            .AddRestToRest(10, 20, 2.0)
            .Build();

        Assert.Equal(2, trajectory.Segments.Count);
        Assert.Equal(4.0, trajectory.TotalDuration, 1e-12);
        Assert.Equal(0, trajectory.EvaluatePosition(0), 1e-9);
        Assert.Equal(10, trajectory.EvaluatePosition(2.0), 1e-9);
        Assert.Equal(20, trajectory.EvaluatePosition(4.0), 1e-9);
    }

    [Fact]
    public void Builder_PassThroughWaypoints_CoversAllPoints()
    {
        double[] waypoints = { 0.0, 5.0, 15.0, 30.0 };

        var trajectory = new TrajectoryBuilder1D()
            .WithDefaultDegree(9)
            .AddSegmentsFromWaypointsPassThrough(waypoints, totalDuration: 5.0)
            .Build();

        Assert.Equal(3, trajectory.Segments.Count);
        Assert.Equal(0, trajectory.EvaluatePosition(0), 1e-6);
        Assert.Equal(30, trajectory.EvaluatePosition(trajectory.TotalDuration), 1e-6);
    }

    [Fact]
    public void Builder3D_RestToRest_MovesBetweenPoints()
    {
        var p0 = new Point3D(0, 0, 0);
        var p1 = new Point3D(10, 20, 30);

        var trajectory = new TrajectoryBuilder3D()
            .AddRestToRest(p0, p1, 3.0)
            .Build();

        Assert.Single(trajectory.Segments);
        var posStart = trajectory.EvaluatePosition(0);
        var posEnd = trajectory.EvaluatePosition(3.0);

        Assert.Equal(p0.X, posStart.X, 1e-6);
        Assert.Equal(p0.Y, posStart.Y, 1e-6);
        Assert.Equal(p0.Z, posStart.Z, 1e-6);

        Assert.Equal(p1.X, posEnd.X, 1e-6);
        Assert.Equal(p1.Y, posEnd.Y, 1e-6);
        Assert.Equal(p1.Z, posEnd.Z, 1e-6);
    }
}

public class PassThroughSolverTests
{
    [Fact]
    public void Solve_ThreeWaypoints_CreatesValidSegments()
    {
        double[] waypoints = { 0.0, 10.0, 20.0 };
        double[] durations = { 2.0, 2.0 };

        var solver = new PassThroughSolver1D(new MinimumSnapCriterion());
        var segments = solver.Solve(waypoints, durations, 9);

        Assert.Equal(2, segments.Length);

        Assert.Equal(0, segments[0].StartState.Position, 1e-9);
        Assert.Equal(10, segments[0].EndState.Position, 1e-6);
        Assert.Equal(10, segments[1].StartState.Position, 1e-6);
        Assert.Equal(20, segments[1].EndState.Position, 1e-9);

        Assert.Equal(0, segments[0].StartState.Velocity, 1e-9);
        Assert.Equal(0, segments[1].EndState.Velocity, 1e-9);
    }
}

public class ConstantVelocityTests
{
    [Fact]
    public void ConstantVelocitySegment_MaintainsSpeed()
    {
        double x0 = 0, xf = 100;
        double xcvStart = 20, xcvEnd = 80;
        double vcv = 15;

        var cvTraj = new TrajectoryPlanner.Core.ConstantVelocity.ConstantVelocityTrajectory1D(
            x0, xf, 0, 0, xcvStart, xcvEnd, vcv, polynomialDegree: 9);

        double tCVStart = cvTraj.EntryTransition.Duration;
        double tCVEnd = cvTraj.EntryTransition.Duration + cvTraj.ConstantVelocity.Duration;
        double tCVMid = (tCVStart + tCVEnd) / 2;

        var midState = cvTraj.EvaluateState(tCVMid);

        Assert.Equal(vcv, Math.Abs(midState.Velocity), 1e-3);
        Assert.Equal(0, midState.Acceleration, 1e-3);
    }
}
