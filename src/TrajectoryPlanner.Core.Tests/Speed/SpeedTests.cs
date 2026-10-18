using System.Diagnostics;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;
using Xunit.Abstractions;

namespace TrajectoryPlanner.Core.Tests.Speed;

/// <summary>
/// Request 0021 (R15): timing tests for the speed criteria S1-S3 (<c>spec/05-success-criteria.md</c>). One warm-up run (JIT compilation
/// and caches), then the median of five timed runs; the test fails if the median exceeds the criterion. Run alone with
/// <c>dotnet test --filter Category=Speed</c>; the measured medians are written to the test output.
/// </summary>
[Trait("Category", "Speed")]
public class SpeedTests(ITestOutputHelper output)
{
    private const int TimedRuns = 5;

    /// <summary>Median of five timed runs (milliseconds) after one warm-up run; <paramref name="work"/> returns a value so the call is not optimised away.</summary>
    private double MedianMilliseconds(string label, Func<object> work, double criterionMilliseconds)
    {
        GC.KeepAlive(work()); // warm-up: the first run pays for JIT compilation

        var times = new double[TimedRuns];
        for (int i = 0; i < TimedRuns; i++)
        {
            long start = Stopwatch.GetTimestamp();
            GC.KeepAlive(work());
            times[i] = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
        }

        Array.Sort(times);
        double median = times[TimedRuns / 2];
        output.WriteLine($"{label}: median {median:F2} ms (criterion {criterionMilliseconds:F0} ms; runs {string.Join(", ", times.Select(t => t.ToString("F2")))} ms)");
        return median;
    }

    // ---- S1: one 1D segment, each degree, limits active (the shortest-feasible-duration search included) <= 100 ms ----------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void S1_OneSegmentWithLimits_IsComputedInUnder100Milliseconds(int degree)
    {
        // Moving boundary states within the degree's capability make the search non-trivial; the duration is left to the search.
        var start = new State1D(0, 0, 0, 0, 0);
        var end = new State1D(10, 1, 0, 0, 0);
        var limits = KinematicLimits.FromValues(vMax: 4, aMax: degree >= 5 ? 20 : null, jMax: degree >= 7 ? 200 : null, sMax: degree >= 9 ? 2000 : null);

        double median = MedianMilliseconds($"S1 degree {degree}", () =>
            SegmentFactory1D.Build(degree, start, end, limits, duration: null), 100);

        var built = SegmentFactory1D.Build(degree, start, end, limits, duration: null);
        Assert.True(ExtremaAnalyzer.VerifyLimits(built, limits)); // the search ran and found a feasible duration
        Assert.NotEqual(SegmentFactory1D.DefaultDuration, built.Duration);
        Assert.True(median <= 100, $"S1 degree {degree}: median {median:F2} ms exceeds 100 ms");
    }

    // ---- S2: pass-through with 30 waypoints, degree 9, default order, limits active, 1D and 3D <= 1 s ------------------------

    private static readonly KinematicLimits PassThroughLimits = KinematicLimits.FromValues(vMax: 5, aMax: 20, jMax: 200, sMax: 2000);

    [Fact]
    public void S2_PassThrough30Waypoints1D_WithLimits_IsComputedInUnder1Second()
    {
        double[] waypoints = Enumerable.Range(0, 30).Select(i => 3.0 * Math.Sin(0.7 * i) + 0.1 * i).ToArray();

        double median = MedianMilliseconds("S2 1D, 30 waypoints, degree 9", () =>
            PassThroughPlanner1D.Plan(waypoints, 9, limits: PassThroughLimits), 1000);

        PassThroughResult once = PassThroughPlanner1D.Plan(waypoints, 9, limits: PassThroughLimits);
        Assert.Equal(29, once.Segments.Length); // the plan is real: 29 segments, the limits bind (λ applied)
        Assert.True(once.ScaledByLimits);
        Assert.True(median <= 1000, $"S2 1D: median {median:F2} ms exceeds 1000 ms");
    }

    [Fact]
    public void S2_PassThrough30Waypoints3D_WithLimits_IsComputedInUnder1Second()
    {
        Point3D[] waypoints = Enumerable.Range(0, 30)
            .Select(i => new Point3D(3.0 * Math.Sin(0.7 * i) + 0.1 * i, 2.0 * Math.Cos(0.5 * i), 0.4 * i + Math.Sin(0.3 * i)))
            .ToArray();

        double median = MedianMilliseconds("S2 3D, 30 waypoints, degree 9", () =>
            PassThroughPlanner3D.Plan(waypoints, 9, limits: PassThroughLimits), 1000);

        PassThroughResult3D once = PassThroughPlanner3D.Plan(waypoints, 9, limits: PassThroughLimits);
        Assert.Equal(29, once.Segments.Length);
        Assert.True(once.ScaledByLimits);
        Assert.True(median <= 1000, $"S2 3D: median {median:F2} ms exceeds 1000 ms");
    }

    // ---- S3: 3D trajectory with a curved constant-speed stretch, analysis and report included <= 1 s -----------------------

    /// <summary>The reference example of the theory (Curved 3D Constant-Speed Stretch): the parabola p(t) = (t, t², 0), here as a degree-9 segment, with a stretch on its middle part.</summary>
    private static Trajectory3D ReferenceCurvedTrajectory(KinematicLimits limits)
    {
        var start = new State3D(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero);
        var end = new State3D(new Point3D(1, 1, 0), new Point3D(1, 2, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero);

        var builder = new TrajectoryBuilder3D().AddSegment(start, end, 1.0);
        builder.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.3, 0.7), 1.0, limits: limits);
        return builder.Build();
    }

    [Fact]
    public void S3_CurvedConstantSpeedTrajectory_WithAnalysisAndReport_IsComputedInUnder1Second()
    {
        var limits = KinematicLimits.FromValues(vMax: 5, aMax: 50, jMax: 500, sMax: 5000);

        double median = MedianMilliseconds("S3 curved constant-speed stretch + analysis + report", () =>
        {
            Trajectory3D trajectory = ReferenceCurvedTrajectory(limits);
            var extrema = TrajectoryAnalyzer3D.Extrema(trajectory);
            var report = TrajectoryAnalyzer3D.LimitReport(trajectory, limits);
            double length = TrajectoryAnalyzer3D.PathLength(trajectory);
            var samples = trajectory.Sample(400, 4);
            return (extrema, report, length, samples);
        }, 1000);

        Trajectory3D check = ReferenceCurvedTrajectory(limits);
        Assert.Equal(3, check.Parts.Count); // entry, stretch, exit
        Assert.True(TrajectoryAnalyzer3D.PathLength(check) > 1.4);
        Assert.True(median <= 1000, $"S3: median {median:F2} ms exceeds 1000 ms");
    }
}
