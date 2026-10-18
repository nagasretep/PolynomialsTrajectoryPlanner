using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.PassThrough;

/// <summary>Request 0019: exact pass-through of 3D waypoints with common durations and kinematic limits.</summary>
public class PassThroughPlanner3DTests
{
    private static readonly Point3D[] Reference =
    {
        new(0, 0, 0), new(1, 0, 0), new(1, 3, 0), new(1, 3, 4)
    };

    private static readonly KinematicLimits ReferenceLimits = KinematicLimits.FromValues(vMax: 2, aMax: 3, jMax: 10, sMax: 50);

    private static PolynomialSegment1D[] Axis(PolynomialSegment3D[] segments, int axis) =>
        segments.Select(s => axis switch { 0 => s.XAxis, 1 => s.YAxis, _ => s.ZAxis }).ToArray();

    private static double Peak(PolynomialSegment3D[] segments, int axis, int order) =>
        ExtremaAnalyzer.PeakAbsolute(Axis(segments, axis), order);

    private static void AssertRelative(double expected, double actual, double tolerance) =>
        Assert.True(Math.Abs(actual - expected) <= tolerance * Math.Abs(expected), $"expected {expected}, got {actual}");

    // ---- the reference example of the theory --------------------------------------------------

    [Fact]
    public void ReferenceExample_ReferenceDurationsAndPeaksAtLambdaOne()
    {
        var unscaled = PassThroughPlanner3D.Plan(Reference, 9);

        Assert.Equal(new[] { 0.375, 1.125, 1.5 }, unscaled.ReferenceDurations!, new Tolerance(1e-12));
        Assert.Equal(new[] { 0.375, 1.125, 1.5 }, unscaled.Durations, new Tolerance(1e-12));
        Assert.Null(unscaled.Scale);
        Assert.False(unscaled.ScaledByLimits);

        double[,] table =
        {
            { 9.981004, 38.207532, 207.293297, 2904.831335 },
            { 4.044221, 7.592746, 25.276926, 127.142593 },
            { 4.895422, 8.795230, 28.459199, 123.104392 }
        };
        for (int axis = 0; axis < 3; axis++)
            for (int order = 1; order <= 4; order++)
                AssertRelative(table[axis, order - 1], Peak(unscaled.Segments, axis, order), 1e-5);
    }

    [Fact]
    public void ReferenceExample_WithLimits_LambdaTotalBindingAndPeaks()
    {
        var r = PassThroughPlanner3D.Plan(Reference, 9, limits: ReferenceLimits);

        Assert.True(r.ScaledByLimits);
        AssertRelative(4.990502, r.Scale!.Value, 1e-6);
        AssertRelative(14.971506, r.TotalDuration, 1e-6);
        Assert.Equal(new[] { 1.871438, 5.614315, 7.485753 }, r.Durations, new Tolerance(1e-5));
        Assert.Equal("velocity", r.BindingQuantity);
        Assert.Equal("x", r.BindingAxis);

        double[] peaks = Enumerable.Range(1, 4).Select(order => Enumerable.Range(0, 3).Max(axis => Peak(r.Segments, axis, order))).ToArray();
        AssertRelative(2.0, peaks[0], 1e-9); // binding
        AssertRelative(1.534124, peaks[1], 1e-5);
        AssertRelative(1.667833, peaks[2], 1e-5);
        AssertRelative(4.683214, peaks[3], 1e-5);
        Assert.Equal(r.ReferenceDurations!.Select(t => t * r.Scale!.Value).ToArray(), r.Durations, new Tolerance(1e-12));
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(2.0)]
    public void ScalingLaw_PerAxis(double lambda)
    {
        var at1 = PassThroughPlanner3D.Plan(Reference, 9).Segments;
        var atLambda = PassThroughPlanner3D.Plan(Reference, 9, prescribedDurations: new[] { 0.375, 1.125, 1.5 }.Select(t => t * lambda).ToArray()).Segments;

        for (int axis = 0; axis < 3; axis++)
            for (int order = 1; order <= 4; order++)
                AssertRelative(Math.Pow(lambda, -order) * Peak(at1, axis, order), Peak(atLambda, axis, order), 1e-8);
    }

    // ---- along one axis = 1D ---------------------------------------------------------------------

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    [InlineData(false, true)]
    public void AlongOneAxis_EqualsThe1DPlanner(bool withLimits, bool withStartVelocity)
    {
        double[] x = { 0, 1, 3, 4 };
        var points = x.Select(v => new Point3D(v, 2, -1)).ToArray();
        var limits = withLimits ? KinematicLimits.FromValues(vMax: 2, aMax: 3) : null;
        State1D? start1 = withStartVelocity ? new State1D(0, 1.5, 0, 0, 0) : null;
        State3D? start3 = withStartVelocity
            ? new State3D(Point3D.Zero, new Point3D(1.5, 0, 0), Point3D.Zero, Point3D.Zero, Point3D.Zero)
            : null;

        var oneD = PassThroughPlanner1D.Plan(x, 9, null, limits, startBoundary: start1);
        var threeD = PassThroughPlanner3D.Plan(points, 9, null, limits, startBoundary: start3);

        Assert.Equal(oneD.Durations, threeD.Durations, new Tolerance(1e-9));
        Assert.Equal(oneD.ScaledByLimits, threeD.ScaledByLimits);
        if (oneD.ScaledByLimits)
        {
            Assert.Equal(oneD.Scale, threeD.Scale!.Value, 1e-9);
            Assert.Equal("x", threeD.BindingAxis);
        }

        for (int k = 0; k < x.Length - 1; k++)
        {
            for (int i = 0; i <= 9; i++)
                Assert.Equal(oneD.Segments[k].Coefficients[i], threeD.Segments[k].XAxis.Coefficients[i], 1e-9);
            Assert.Equal(2.0, threeD.Segments[k].EvaluatePosition(threeD.Segments[k].Duration / 3).Y, 12);
            Assert.Equal(-1.0, threeD.Segments[k].EvaluatePosition(threeD.Segments[k].Duration / 3).Z, 12);
        }
    }

    // ---- continuity and degrees --------------------------------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void EveryWaypointIsHit_AndTheStateIsContinuous_UpToTheOrderOfTheDegree(int degree)
    {
        var points = new[] { new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 1), new Point3D(4, 0, 3), new Point3D(2, -1, 2) };

        var r = PassThroughPlanner3D.Plan(points, degree, limits: KinematicLimits.FromValues(vMax: 3));

        int p = (degree + 1) / 2;
        for (int k = 0; k < r.Segments.Length; k++)
        {
            Point3D start = r.Segments[k].EvaluatePosition(0);
            Assert.Equal(0.0, (start - points[k]).Length, 1e-9);
            if (k + 1 < r.Segments.Length)
            {
                State3D end = r.Segments[k].EvaluateState(r.Segments[k].Duration);
                State3D next = r.Segments[k + 1].EvaluateState(0);
                for (int order = 0; order < p; order++)
                    Assert.Equal(0.0, (end.GetDerivative(order) - next.GetDerivative(order)).Length, 1e-9 * (1 + end.GetDerivative(order).Length));
            }
        }

        Assert.Equal(0.0, (r.Segments[^1].EvaluatePosition(r.Segments[^1].Duration) - points[^1]).Length, 1e-9);
        Assert.All(r.Segments, s => Assert.True(ExtremaAnalyzer.VerifyLimits(s, KinematicLimits.FromValues(vMax: 3))));
    }

    // ---- durations -------------------------------------------------------------------------------

    [Fact]
    public void PrescribedDurations_AreKept_AndAViolationNamesSegmentQuantityAndAxis()
    {
        var kept = PassThroughPlanner3D.Plan(Reference, 9, prescribedDurations: new[] { 4.0, 5.0, 6.0 }, limits: ReferenceLimits);
        Assert.Equal(new[] { 4.0, 5.0, 6.0 }, kept.Durations);
        Assert.Null(kept.ReferenceDurations);
        Assert.Null(kept.BindingAxis);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(() =>
            PassThroughPlanner3D.Plan(Reference, 9, prescribedDurations: new[] { 0.5, 6.0, 8.0 }, limits: ReferenceLimits));

        Assert.Contains("Pass-through segment 1", ex.Message);
        Assert.Contains("on axis x", ex.Message);
        Assert.Contains("limit", ex.Message);
    }

    [Fact]
    public void APrescribedTotal_IsSplitByTheChords_AndOnlyVerified()
    {
        var r = PassThroughPlanner3D.Plan(Reference, 9, totalDuration: 16.0, limits: ReferenceLimits);

        Assert.Equal(new[] { 2.0, 6.0, 8.0 }, r.Durations, new Tolerance(1e-12)); // chords 1 : 3 : 4
        Assert.False(r.ScaledByLimits);

        Assert.Throws<KinematicLimitsViolatedException>(() => PassThroughPlanner3D.Plan(Reference, 9, totalDuration: 6.0, limits: ReferenceLimits));
        Assert.Throws<ArgumentException>(() => PassThroughPlanner3D.Plan(Reference, 9, prescribedDurations: new[] { 1.0, 1, 1 }, totalDuration: 3));
    }

    [Fact]
    public void AllWaypointsEqual_EqualSplit_AndTheLimitsDetermineNothing()
    {
        var same = Enumerable.Repeat(new Point3D(1, 2, 3), 4).ToArray();

        var r = PassThroughPlanner3D.Plan(same, 9, limits: ReferenceLimits);

        Assert.Equal(new[] { 1.0, 1.0, 1.0 }, r.Durations, new Tolerance(1e-12));
        Assert.False(r.ScaledByLimits);
        Assert.Null(r.Scale);
        Assert.All(r.Segments, s => Assert.Equal(0.0, (s.EvaluatePosition(s.Duration / 2) - new Point3D(1, 2, 3)).Length, 1e-12));
    }

    [Fact]
    public void AZeroChord_GetsTheFloorShareOfTheRule()
    {
        var points = new[] { new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 1, 0) };

        var r = PassThroughPlanner3D.Plan(points, 9);

        double[] expected = new AutoDerivedProportionalDuration(3.0).ComputeDurations(points);
        Assert.Equal(expected, r.Durations, new Tolerance(1e-12));
        Assert.True(r.Durations[1] > 0 && r.Durations[1] < 0.1 * r.Durations[0]);
    }

    [Fact]
    public void AConstantAxis_ImposesNothing()
    {
        var planar = new[] { new Point3D(0, 0, 5), new Point3D(1, 2, 5), new Point3D(3, 1, 5) };

        var r = PassThroughPlanner3D.Plan(planar, 9, limits: KinematicLimits.FromValues(vMax: 1, aMax: 2));

        Assert.NotEqual("z", r.BindingAxis);
        Assert.Equal(0.0, Peak(r.Segments, 2, 1), 12);
        Assert.All(r.Segments, s => Assert.Equal(5.0, s.EvaluatePosition(s.Duration / 2).Z, 12));
    }

    [Fact]
    public void ANonZeroBoundaryVelocityVector_IsSolvedBySearch_AndVerifiedByTheLimitReport()
    {
        var start = new State3D(Point3D.Zero, new Point3D(0.5, -0.4, 0.3), Point3D.Zero, Point3D.Zero, Point3D.Zero);
        var end = new State3D(Point3D.Zero, new Point3D(0, 0.2, -0.3), Point3D.Zero, Point3D.Zero, Point3D.Zero);

        var r = PassThroughPlanner3D.Plan(Reference, 9, limits: ReferenceLimits, startBoundary: start, endBoundary: end);

        Assert.True(r.ScaledByLimits);
        Assert.NotNull(r.BindingQuantity);
        Assert.Equal(0.0, (r.Segments[0].EvaluateDerivative(0, 1) - start.Velocity).Length, 1e-9);
        Assert.Equal(0.0, (r.Segments[^1].EvaluateDerivative(r.Segments[^1].Duration, 1) - end.Velocity).Length, 1e-9);

        var report = TrajectoryAnalyzer3D.LimitReport(new Trajectory3D(r.Segments), ReferenceLimits);
        Assert.All(report, row => Assert.False(row.Violated));
        var top = report.OrderByDescending(row => row.PercentOfLimit).First();
        Assert.Equal(100.0, top.PercentOfLimit, 1e-6); // the feasibility boundary
        Assert.Equal(r.BindingAxis, top.Axis);
        Assert.Equal(r.BindingQuantity, top.Quantity.ToLowerInvariant());

        // 0.01 % shorter violates a limit.
        var shorter = PassThroughPlanner3D.Plan(Reference, 9, prescribedDurations: r.Durations.Select(t => t * 0.9999).ToArray(),
            startBoundary: start, endBoundary: end);
        Assert.Contains(TrajectoryAnalyzer3D.LimitReport(new Trajectory3D(shorter.Segments), ReferenceLimits), row => row.Violated);
    }

    [Fact]
    public void ABoundaryVelocityAboveTheLimit_FailsExplicitly()
    {
        var start = new State3D(Point3D.Zero, new Point3D(0, 0, 5), Point3D.Zero, Point3D.Zero, Point3D.Zero);

        Assert.Throws<DurationSearchFailedException>(() =>
            PassThroughPlanner3D.Plan(Reference, 9, limits: KinematicLimits.FromValues(vMax: 2), startBoundary: start));
    }

    [Fact]
    public void FewerThanTwoWaypoints_IsRejected()
    {
        Assert.Throws<ArgumentException>(() => PassThroughPlanner3D.Plan(new[] { new Point3D(0, 0, 0) }, 9));
    }

    // ---- builder -------------------------------------------------------------------------------------

    [Fact]
    public void Builder_UsesThePlanner_DefaultOneSecondPerSegment_AndExposesTheResult()
    {
        var builder = new TrajectoryBuilder3D();
        var trajectory = builder.AddSegmentsFromWaypointsPassThrough(Reference).Build();

        Assert.Equal(3.0, trajectory.TotalDuration, 12);
        Assert.Equal(new[] { 0.375, 1.125, 1.5 }, builder.LastPassThroughResult!.Durations, new Tolerance(1e-12));

        var limited = new TrajectoryBuilder3D();
        limited.AddSegmentsFromWaypointsPassThrough(Reference, limits: ReferenceLimits);
        AssertRelative(14.971506, limited.Build().TotalDuration, 1e-6);
        Assert.Equal("x", limited.LastPassThroughResult!.BindingAxis);
        Assert.Equal(3, limited.Parts.Count);
    }

    private sealed class Tolerance : IEqualityComparer<double>
    {
        private readonly double _tolerance;
        public Tolerance(double tolerance) => _tolerance = tolerance;
        public bool Equals(double x, double y) => Math.Abs(x - y) <= _tolerance;
        public int GetHashCode(double obj) => 0;
    }
}
