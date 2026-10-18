using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Services;

/// <summary>
/// Builds a <see cref="Trajectory3D"/> from segments, blends and constant-speed stretches, mirroring
/// <see cref="TrajectoryBuilder1D"/>. Indices passed to <see cref="AddBlendBetween(int,int,double,double,int?,double?,KinematicLimits?)"/>
/// and <see cref="AddConstantSpeed"/> are positions in the current list of pieces (a blend or a stretch replaces its
/// neighbours by three pieces, as in 1D); <see cref="Parts"/> describes the pieces in order. One
/// <see cref="KinematicLimits"/> applies to every axis.
/// </summary>
public class TrajectoryBuilder3D
{
    private sealed record Piece(
        PartKind Kind, ISegment3D Segment, int Original, int? Second,
        double? InheritedDuration = null, bool Adjusted = false,
        double? StretchLength = null, double? StretchSpeed = null, double? AdmissibleSpeed = null, string? BindingLimit = null);

    private readonly List<Piece> _pieces = new();
    private int _nextOriginal;
    private int _defaultDegree = 9;
    private SmoothnessCriterion? _criterion;

    public TrajectoryBuilder3D WithDefaultDegree(int degree)
    {
        _defaultDegree = degree;
        return this;
    }

    public TrajectoryBuilder3D WithSmoothnessCriterion(SmoothnessCriterion criterion)
    {
        _criterion = criterion;
        return this;
    }

    /// <summary>The pieces so far, in order, with start times, durations and the blend / stretch information.</summary>
    public IReadOnlyList<TrajectoryPart3D> Parts
    {
        get
        {
            var parts = new List<TrajectoryPart3D>();
            double start = 0;
            foreach (Piece p in _pieces)
            {
                parts.Add(new TrajectoryPart3D(
                    p.Kind, p.Original, p.Second, start, p.Segment.Duration, p.Segment,
                    p.InheritedDuration, p.Adjusted, p.StretchLength, p.StretchSpeed, p.AdmissibleSpeed, p.BindingLimit));
                start += p.Segment.Duration;
            }

            return parts;
        }
    }

    /// <summary>
    /// Adds a polynomial segment. A given duration is only verified against the limits; a null duration means the shortest
    /// feasible duration when limits are active (all three axes share it), and the 1D fallback
    /// (<see cref="SegmentFactory1D.DefaultDuration"/>, 1 s) otherwise.
    /// </summary>
    public TrajectoryBuilder3D AddSegment(
        State3D startState, State3D endState, double? duration = null, int? degree = null, KinematicLimits? limits = null)
    {
        var segment = SegmentFactory3D.Build(degree ?? _defaultDegree, startState, endState, limits, duration);
        _pieces.Add(new Piece(PartKind.Segment, segment, _nextOriginal++, null));
        return this;
    }

    public TrajectoryBuilder3D AddRestToRest(
        Point3D p0, Point3D pf, double? duration = null, int? degree = null, KinematicLimits? limits = null) =>
        AddSegment(State3D.FromPositionOnly(p0), State3D.FromPositionOnly(pf), duration, degree, limits);

    public TrajectoryBuilder3D AddSegmentsFromWaypointsPassThrough(
        Point3D[] waypoints,
        double[]? durations = null,
        double totalDuration = 1.0,
        int? degree = null)
    {
        int d = degree ?? _defaultDegree;
        int M = waypoints.Length - 1;
        double[] segDurations = durations ?? new AutoDerivedProportionalDuration(totalDuration).ComputeDurations(waypoints);

        double[] xWaypoints = waypoints.Select(p => p.X).ToArray();
        double[] yWaypoints = waypoints.Select(p => p.Y).ToArray();
        double[] zWaypoints = waypoints.Select(p => p.Z).ToArray();

        // Common durations on all axes: the 3D cost is the sum of the three axis costs, so the
        // component-wise solve is exactly optimal.
        var solver = new PassThroughSolver1D(_criterion);
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

            _pieces.Add(new Piece(PartKind.Segment, new PolynomialSegment3D(d, startState, endState, segDurations[k]), _nextOriginal++, null));
        }

        return this;
    }

    /// <summary>Blends two consecutive plain segments; the blend window is given by parameter fractions of the neighbours.</summary>
    public TrajectoryBuilder3D AddBlendBetween(
        int precedingIndex, int followingIndex, double blendStartTau, double blendEndTau,
        int? blendDegree = null, double? blendDuration = null, KinematicLimits? limits = null)
    {
        var (preceding, following) = BlendNeighbours(precedingIndex, followingIndex);

        var (truncatedPre, blend, truncatedPost) = BlendBuilder3D.BuildBlendedSegmentPair(
            preceding.Segment, following.Segment, blendStartTau, blendEndTau, blendDegree, blendDuration, limits);

        Replace(precedingIndex, truncatedPre, blend, truncatedPost, preceding, following);
        return this;
    }

    /// <summary>Same, with the blend window given by absolute points on the neighbours.</summary>
    public TrajectoryBuilder3D AddBlendBetween(
        int precedingIndex, int followingIndex, Point3D blendStartPoint, Point3D blendEndPoint,
        int? blendDegree = null, double? blendDuration = null, KinematicLimits? limits = null)
    {
        var (preceding, following) = BlendNeighbours(precedingIndex, followingIndex);

        var (truncatedPre, blend, truncatedPost) = BlendBuilder3D.BuildBlendedSegmentPair(
            preceding.Segment, following.Segment, blendStartPoint, blendEndPoint, blendDegree, blendDuration, limits);

        Replace(precedingIndex, truncatedPre, blend, truncatedPost, preceding, following);
        return this;
    }

    private (BlendNeighbour Preceding, BlendNeighbour Following) BlendNeighbours(int precedingIndex, int followingIndex)
    {
        if (precedingIndex < 0 || followingIndex != precedingIndex + 1 || followingIndex >= _pieces.Count)
            throw new ArgumentException("Blend can only be between consecutive segments");

        return (Neighbour(precedingIndex), Neighbour(followingIndex));
    }

    private BlendNeighbour Neighbour(int index)
    {
        Piece piece = _pieces[index];
        if (piece.Kind != PartKind.Segment || piece.Segment is not PolynomialSegment3D polynomial)
            throw new InvalidOperationException($"Piece {index + 1} is a {piece.Kind}: only plain segments can be blended.");
        return new BlendNeighbour(polynomial, piece.Original);
    }

    private readonly record struct BlendNeighbour(PolynomialSegment3D Segment, int Original);

    private void Replace(
        int precedingIndex, PolynomialSegment3D truncatedPre, BlendSegment3D blend, PolynomialSegment3D truncatedPost,
        BlendNeighbour preceding, BlendNeighbour following)
    {
        _pieces.RemoveRange(precedingIndex, 2);
        _pieces.Insert(precedingIndex, new Piece(PartKind.Segment, truncatedPre, preceding.Original, null));
        _pieces.Insert(precedingIndex + 1, new Piece(
            PartKind.Blend, blend, preceding.Original, following.Original, blend.InheritedDuration, blend.AdjustedByLimits));
        _pieces.Insert(precedingIndex + 2, new Piece(PartKind.Segment, truncatedPost, following.Original, null));
    }

    /// <summary>
    /// Replaces a plain segment by entry transition + constant-speed stretch + exit transition (<see cref="ConstantSpeedTrajectory3D"/>,
    /// also for a straight stretch). The transitions run from the segment's start state to the stretch and from the stretch to
    /// the segment's end state; their degree is <paramref name="degree"/>, by default that of the segment. The part of the
    /// segment outside the stretch is replaced by the transitions (they are new pieces).
    /// </summary>
    public TrajectoryBuilder3D AddConstantSpeed(
        int segmentIndex, StretchSpecification specification, double speed, int? degree = null, KinematicLimits? limits = null,
        double? entryDuration = null, double? exitDuration = null, double? totalDuration = null)
    {
        if (segmentIndex < 0 || segmentIndex >= _pieces.Count)
            throw new ArgumentOutOfRangeException(nameof(segmentIndex));

        Piece piece = _pieces[segmentIndex];
        if (piece.Kind != PartKind.Segment || piece.Segment is not PolynomialSegment3D segment)
            throw new InvalidOperationException($"Piece {segmentIndex + 1} is a {piece.Kind}: a constant-speed stretch needs a plain segment.");

        int d = degree ?? segment.Degree;
        State3D start = segment.StartState, end = segment.EndState;

        ConstantSpeedTrajectory3D trajectory = specification.Kind switch
        {
            StretchSpecificationKind.ParameterFractions => ConstantSpeedTrajectory3D.FromParameterFractions(
                segment, specification.Start, specification.End, speed, start, end, d, limits, entryDuration, exitDuration, totalDuration),
            StretchSpecificationKind.Points => ConstantSpeedTrajectory3D.FromPoints(
                segment, specification.StartPoint, specification.EndPoint, speed, start, end, d, limits, entryDuration, exitDuration, totalDuration),
            _ => ConstantSpeedTrajectory3D.FromArcLengthFractions(
                segment, specification.Start, specification.End, speed, start, end, d, limits, entryDuration, exitDuration, totalDuration)
        };

        var (admissible, binding) = trajectory.Stretch.AdmissibleSpeed(limits);

        _pieces.RemoveAt(segmentIndex);
        _pieces.Insert(segmentIndex, new Piece(PartKind.CvEntry, trajectory.EntryTransition, piece.Original, null));
        _pieces.Insert(segmentIndex + 1, new Piece(
            PartKind.CvStretch, trajectory.Stretch, piece.Original, null,
            StretchLength: trajectory.Stretch.Subpath.Length, StretchSpeed: speed,
            AdmissibleSpeed: double.IsPositiveInfinity(admissible) ? null : admissible, BindingLimit: binding));
        _pieces.Insert(segmentIndex + 2, new Piece(PartKind.CvExit, trajectory.ExitTransition, piece.Original, null));
        return this;
    }

    public Trajectory3D Build()
    {
        if (_pieces.Count == 0)
            throw new InvalidOperationException("No segments have been added to the trajectory builder");

        return new Trajectory3D(_pieces.Select(p => p.Segment).ToList(), Parts);
    }
}
