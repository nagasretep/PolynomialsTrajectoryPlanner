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

    /// <summary>
    /// Uniform samples over [0, TotalDuration] (first and last included) of position and its derivatives
    /// up to <paramref name="maxDerivativeOrder"/>.
    /// </summary>
    public TrajectorySamples Sample(int sampleCount, int maxDerivativeOrder)
    {
        if (sampleCount < 2)
            throw new ArgumentOutOfRangeException(nameof(sampleCount), "At least 2 samples are required (start and end).");
        if (maxDerivativeOrder < 0)
            throw new ArgumentOutOfRangeException(nameof(maxDerivativeOrder));

        var times = new double[sampleCount];
        for (int i = 0; i < sampleCount; i++)
            times[i] = TotalDuration * i / (sampleCount - 1);

        var values = new double[maxDerivativeOrder + 1][];
        for (int order = 0; order <= maxDerivativeOrder; order++)
        {
            values[order] = times
                .Select(t => order == 0 ? EvaluatePosition(t) : EvaluateDerivative(t, order))
                .ToArray();
        }

        return new TrajectorySamples(times, values);
    }

    public IEnumerable<double> SampleTimes(int samplesPerSegment = 50)
    {
        if (samplesPerSegment < 2)
            throw new ArgumentOutOfRangeException(nameof(samplesPerSegment), "At least 2 samples per segment are required (start and end).");

        return SampleTimesCore(samplesPerSegment);
    }

    private IEnumerable<double> SampleTimesCore(int samplesPerSegment)
    {
        for (int s = 0; s < _segments.Count; s++)
        {
            double duration = _segments[s].Duration;
            for (int i = 0; i < samplesPerSegment; i++)
            {
                double localT = (i * duration) / (samplesPerSegment - 1);
                yield return _segmentStartTimes[s] + localT;
            }
        }
    }
}
