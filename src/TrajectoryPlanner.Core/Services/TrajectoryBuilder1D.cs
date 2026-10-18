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
    private SmoothnessCriterion? _criterion;

    public TrajectoryBuilder1D WithDefaultDegree(int degree)
    {
        _defaultDegree = degree;
        return this;
    }

    /// <summary>Smoothness order of exact pass-through; default is chosen per degree (snap for degree 9).</summary>
    public TrajectoryBuilder1D WithSmoothnessCriterion(SmoothnessCriterion criterion)
    {
        _criterion = criterion;
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

    /// <summary>Outcome of the last pass-through call: final durations and the scale λ chosen by the limits.</summary>
    public PassThroughResult? LastPassThroughResult { get; private set; }

    /// <summary>
    /// Exact pass-through of the waypoints.
    /// Durations are <b>prescribed</b> when <paramref name="durations"/> is given, when a duration strategy was set, or
    /// when <paramref name="totalDuration"/> is given (proportional rule): they are kept and, if limits are active,
    /// verified (<see cref="KinematicLimitsViolatedException"/> naming the segment). Otherwise they are <b>automatic</b>:
    /// with active limits the shortest feasible sequence (one common scale λ on the proportional reference durations,
    /// see <see cref="PassThroughPlanner1D"/>), without limits the reference durations (A8 proportions, one second per segment in total).
    /// </summary>
    /// <param name="startBoundary">Prescribed derivatives (orders 1 .. p-1) at the start; null: rest.</param>
    /// <param name="endBoundary">Same at the end.</param>
    public TrajectoryBuilder1D AddSegmentsFromWaypointsPassThrough(
        double[] waypoints,
        double[]? durations = null,
        double? totalDuration = null,
        int? degree = null,
        KinematicLimits? limits = null,
        State1D? startBoundary = null,
        State1D? endBoundary = null)
    {
        int d = degree ?? _defaultDegree;

        // One duration mechanism: explicit durations win, then the configured strategy, then a prescribed total.
        IDurationStrategy1D? strategy = durations is not null
            ? new UserSpecifiedDuration(durations)
            : _durationStrategy ?? (totalDuration is not null ? new AutoDerivedProportionalDuration(totalDuration.Value) : null);
        double[]? prescribed = strategy?.ComputeDurations(waypoints, d);

        var result = PassThroughPlanner1D.Plan(
            waypoints, d, _criterion, limits, prescribed, startBoundary: startBoundary, endBoundary: endBoundary);
        LastPassThroughResult = result;
        _segments.AddRange(result.Segments);

        return this;
    }

    public TrajectoryBuilder1D AddBlendBetween(
        int precedingIndex,
        int followingIndex,
        double blendStartTau,
        double blendEndTau,
        int? blendDegree = null,
        double? blendDuration = null,
        KinematicLimits? limits = null)
    {
        if (precedingIndex < 0 || followingIndex != precedingIndex + 1)
            throw new ArgumentException("Blend can only be between consecutive segments");

        var preceding = _segments[precedingIndex];
        var following = _segments[followingIndex];

        var (truncatedPre, blendSeg, truncatedPost) =
            BlendBuilder1D.BuildBlendedSegmentPair(preceding, following, blendStartTau, blendEndTau, blendDegree, blendDuration, limits);

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
