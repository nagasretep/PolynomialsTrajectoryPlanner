using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Services;

public class TrajectoryBuilder3D
{
    private readonly List<PolynomialSegment3D> _segments = new();
    private int _defaultDegree = 9;

    public TrajectoryBuilder3D WithDefaultDegree(int degree)
    {
        _defaultDegree = degree;
        return this;
    }

    public TrajectoryBuilder3D AddSegment(State3D startState, State3D endState, double duration, int? degree = null)
    {
        int d = degree ?? _defaultDegree;
        _segments.Add(new PolynomialSegment3D(d, startState, endState, duration));
        return this;
    }

    public TrajectoryBuilder3D AddRestToRest(Point3D p0, Point3D pf, double duration, int? degree = null)
    {
        var start = State3D.FromPositionOnly(p0);
        var end = State3D.FromPositionOnly(pf);
        return AddSegment(start, end, duration, degree);
    }

    public TrajectoryBuilder3D AddSegmentsFromWaypointsPassThrough(
        Point3D[] waypoints,
        double[]? durations = null,
        double totalDuration = 1.0,
        int? degree = null)
    {
        int d = degree ?? _defaultDegree;
        int M = waypoints.Length - 1;
        double[] segDurations = durations ?? DurationStrategies.ProportionalToDistance3D(waypoints, totalDuration);

        double[] xWaypoints = waypoints.Select(p => p.X).ToArray();
        double[] yWaypoints = waypoints.Select(p => p.Y).ToArray();
        double[] zWaypoints = waypoints.Select(p => p.Z).ToArray();

        var solver = new PassThroughSolver1D(new MinimumSnapCriterion());
        var xSegs = solver.Solve(xWaypoints, segDurations, d);
        var ySegs = solver.Solve(yWaypoints, segDurations, d);
        var zSegs = solver.Solve(zWaypoints, segDurations, d);

        for (int k = 0; k < M; k++)
        {
            var startState = new State3D(
                new Point3D(xSegs[k].StartState.Position, ySegs[k].StartState.Position, zSegs[k].StartState.Position),
                new Point3D(xSegs[k].StartState.Velocity, ySegs[k].StartState.Velocity, zSegs[k].StartState.Velocity),
                new Point3D(xSegs[k].StartState.Acceleration, ySegs[k].StartState.Acceleration, zSegs[k].StartState.Acceleration),
                new Point3D(xSegs[k].StartState.Jerk, ySegs[k].StartState.Jerk, zSegs[k].StartState.Jerk),
                new Point3D(xSegs[k].StartState.Snap, ySegs[k].StartState.Snap, zSegs[k].StartState.Snap));

            var endState = new State3D(
                new Point3D(xSegs[k].EndState.Position, ySegs[k].EndState.Position, zSegs[k].EndState.Position),
                new Point3D(xSegs[k].EndState.Velocity, ySegs[k].EndState.Velocity, zSegs[k].EndState.Velocity),
                new Point3D(xSegs[k].EndState.Acceleration, ySegs[k].EndState.Acceleration, zSegs[k].EndState.Acceleration),
                new Point3D(xSegs[k].EndState.Jerk, ySegs[k].EndState.Jerk, zSegs[k].EndState.Jerk),
                new Point3D(xSegs[k].EndState.Snap, ySegs[k].EndState.Snap, zSegs[k].EndState.Snap));

            _segments.Add(new PolynomialSegment3D(d, startState, endState, segDurations[k]));
        }

        return this;
    }

    public Trajectory3D Build()
    {
        if (_segments.Count == 0)
            throw new InvalidOperationException("No segments have been added to the trajectory builder");

        return new Trajectory3D(_segments);
    }
}
