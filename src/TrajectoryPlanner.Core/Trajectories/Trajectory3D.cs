using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Trajectories;

/// <summary>
/// A 3D trajectory: consecutive pieces (<see cref="ISegment3D"/>: polynomial segments and curved constant-speed
/// stretches) in one time axis. <see cref="Parts"/> describes the pieces in order.
/// </summary>
public class Trajectory3D
{
    private readonly List<ISegment3D> _segments = new();
    private readonly double[] _segmentStartTimes;

    public IReadOnlyList<ISegment3D> Segments => _segments;
    public IReadOnlyList<TrajectoryPart3D> Parts { get; }
    public double TotalDuration { get; private set; }

    /// <param name="segments">The pieces, in order.</param>
    /// <param name="parts">Their descriptors (same count and order); null: every piece is a plain segment numbered in order.</param>
    public Trajectory3D(IEnumerable<ISegment3D> segments, IReadOnlyList<TrajectoryPart3D>? parts = null)
    {
        _segments.AddRange(segments);
        _segmentStartTimes = new double[_segments.Count + 1];
        ComputeTimes();

        if (parts is not null)
        {
            if (parts.Count != _segments.Count)
                throw new ArgumentException("There must be one part descriptor per segment.", nameof(parts));
            Parts = parts;
        }
        else
        {
            Parts = _segments
                .Select((s, i) => new TrajectoryPart3D(PartKind.Segment, i, null, _segmentStartTimes[i], s.Duration, s))
                .ToList();
        }
    }

    private void ComputeTimes()
    {
        TotalDuration = 0;
        _segmentStartTimes[0] = 0;
        for (int i = 0; i < _segments.Count; i++)
        {
            TotalDuration += _segments[i].Duration;
            _segmentStartTimes[i + 1] = TotalDuration;
        }
    }

    /// <summary>Start time of piece i (i = Segments.Count: the end of the trajectory).</summary>
    public double StartTimeOf(int index) => _segmentStartTimes[index];

    public (int SegmentIndex, double LocalTime) FindSegment(double globalTime)
    {
        if (globalTime < 0 || globalTime > TotalDuration + 1e-12)
            throw new ArgumentOutOfRangeException(nameof(globalTime), $"Time {globalTime} outside trajectory [0, {TotalDuration}]");

        if (globalTime >= TotalDuration)
        {
            int lastIdx = _segments.Count - 1;
            return (lastIdx, _segments[lastIdx].Duration);
        }

        for (int i = 0; i < _segments.Count; i++)
        {
            if (globalTime < _segmentStartTimes[i + 1])
            {
                double localT = globalTime - _segmentStartTimes[i];
                return (i, localT);
            }
        }

        return (_segments.Count - 1, _segments[^1].Duration);
    }

    public Point3D EvaluatePosition(double t)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluatePosition(localT);
    }

    public Point3D EvaluateDerivative(double t, int order)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluateDerivative(localT, order);
    }

    public State3D EvaluateState(double t)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluateState(localT);
    }

    /// <summary>
    /// Samples of the position and its derivatives up to <paramref name="maxDerivativeOrder"/> (0..4): <paramref name="count"/>
    /// uniform times over [0, TotalDuration] (ends included) <b>plus</b> the times of the junctions between pieces that the
    /// uniform times do not already contain, in increasing order, so that a junction is never skipped (a sample exactly at
    /// a junction belongs to the piece that starts there). The number of samples is therefore count plus at most (pieces - 1).
    /// </summary>
    public TrajectorySamples3D Sample(int count, int maxDerivativeOrder)
    {
        if (count < 2)
            throw new ArgumentOutOfRangeException(nameof(count), "At least 2 samples are required (start and end).");
        if (maxDerivativeOrder < 0 || maxDerivativeOrder > 4)
            throw new ArgumentOutOfRangeException(nameof(maxDerivativeOrder));

        var times = new List<double>();
        for (int i = 0; i < count; i++)
            times.Add(TotalDuration * i / (count - 1));
        for (int k = 1; k < _segments.Count; k++)
            times.Add(_segmentStartTimes[k]);

        times.Sort();
        var unique = new List<double>();
        foreach (double t in times)
        {
            if (unique.Count == 0 || t - unique[^1] > 1e-12 * Math.Max(1.0, TotalDuration))
                unique.Add(t);
        }

        var values = new Point3D[maxDerivativeOrder + 1][];
        for (int order = 0; order <= maxDerivativeOrder; order++)
            values[order] = new Point3D[unique.Count];

        for (int i = 0; i < unique.Count; i++)
        {
            State3D state = EvaluateState(unique[i]);
            for (int order = 0; order <= maxDerivativeOrder; order++)
                values[order][i] = state.GetDerivative(order);
        }

        return new TrajectorySamples3D(unique.ToArray(), values);
    }
}
