using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Services;

public class TrajectoryBuilder1D
{
    private readonly List<PolynomialSegment1D> _segments = new();
    private IDurationStrategy1D? _durationStrategy;
    private int _defaultDegree = 9;

    public TrajectoryBuilder1D WithDefaultDegree(int degree)
    {
        _defaultDegree = degree;
        return this;
    }

    public TrajectoryBuilder1D WithDurationStrategy(IDurationStrategy1D strategy)
    {
        _durationStrategy = strategy;
        return this;
    }

    public TrajectoryBuilder1D AddSegment(State1D startState, State1D endState, double duration, int? degree = null)
    {
        int d = degree ?? _defaultDegree;
        _segments.Add(new PolynomialSegment1D(d, startState, endState, duration));
        return this;
    }

    public TrajectoryBuilder1D AddSegmentSimple(double x0, double xf, double v0, double vf, double duration, int? degree = null)
    {
        var start = new State1D(x0, v0, 0, 0, 0);
        var end = new State1D(xf, vf, 0, 0, 0);
        return AddSegment(start, end, duration, degree);
    }

    public TrajectoryBuilder1D AddRestToRest(double x0, double xf, double duration, int? degree = null)
    {
        var start = State1D.FromPositionOnly(x0);
        var end = State1D.FromPositionOnly(xf);
        return AddSegment(start, end, duration, degree);
    }

    public TrajectoryBuilder1D AddSegmentsFromWaypointsPassThrough(
        double[] waypoints,
        double[]? durations = null,
        double totalDuration = 1.0,
        int? degree = null)
    {
        int d = degree ?? _defaultDegree;
        double[] segDurations = durations ?? DurationStrategies.ProportionalToDistance(waypoints, totalDuration);

        var solver = new PassThroughSolver1D(new MinimumSnapCriterion());
        var segments = solver.Solve(waypoints, segDurations, d);
        _segments.AddRange(segments);

        return this;
    }

    public TrajectoryBuilder1D AddBlendBetween(
        int precedingIndex,
        int followingIndex,
        double blendStartTau,
        double blendEndTau,
        int blendDegree = 9)
    {
        if (precedingIndex < 0 || followingIndex != precedingIndex + 1)
            throw new ArgumentException("Blend can only be between consecutive segments");

        var preceding = _segments[precedingIndex];
        var following = _segments[followingIndex];

        var (truncatedPre, blendSeg, truncatedPost) =
            BlendBuilder1D.BuildBlendedSegmentPair(preceding, following, blendStartTau, blendEndTau, blendDegree);

        _segments.RemoveAt(followingIndex);
        _segments.RemoveAt(precedingIndex);

        _segments.Insert(precedingIndex, truncatedPre);
        _segments.Insert(precedingIndex + 1, blendSeg);
        _segments.Insert(precedingIndex + 2, truncatedPost);

        return this;
    }

    public Trajectory1D Build()
    {
        if (_segments.Count == 0)
            throw new InvalidOperationException("No segments have been added to the trajectory builder");

        return new Trajectory1D(_segments);
    }
}
