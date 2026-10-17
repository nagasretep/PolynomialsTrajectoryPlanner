using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Trajectories;

public class Trajectory1D
{
    private readonly List<PolynomialSegment1D> _segments = new();
    private readonly double[] _segmentStartTimes;

    public IReadOnlyList<PolynomialSegment1D> Segments => _segments;
    public double TotalDuration { get; private set; }
    public bool RespectEndpointsExactly { get; set; } = true;

    public Trajectory1D(IEnumerable<PolynomialSegment1D> segments)
    {
        _segments.AddRange(segments);
        _segmentStartTimes = new double[_segments.Count + 1];
        ComputeTimes();
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

    public double EvaluatePosition(double t)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluatePosition(localT);
    }

    public double EvaluateDerivative(double t, int order)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluateDerivative(localT, order);
    }

    public State1D EvaluateState(double t)
    {
        var (idx, localT) = FindSegment(t);
        return _segments[idx].EvaluateState(localT);
    }

    public Point1D EvaluatePoint(double t) => new(EvaluatePosition(t));

    public IEnumerable<double> SampleTimes(int samplesPerSegment = 50)
    {
        foreach (var seg in _segments)
        {
            for (int i = 0; i < samplesPerSegment; i++)
            {
                double localT = (i * seg.Duration) / (samplesPerSegment - 1);
                double globalT = _segmentStartTimes[_segments.IndexOf(seg)] + localT;
                yield return globalT;
            }
        }
    }
}
