using TrajectoryPlanner.Core.ArcLength;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.Trajectories;

/// <summary>Request 0017: sampling, extrema, limit report and path length of a whole 3D trajectory.</summary>
public class TrajectoryAnalyzer3DTests
{
    private static State3D S(double x, double y, double z, double vx = 0, double vy = 0, double vz = 0) =>
        new(new Point3D(x, y, z), new Point3D(vx, vy, vz), Point3D.Zero, Point3D.Zero, Point3D.Zero);

    private static TrajectoryBuilder3D MixedBuilder()
    {
        var builder = new TrajectoryBuilder3D()
            .AddSegment(S(0, 0, 0), S(4, 2, 1, 1, 0.5, 0.25), 2.0)
            .AddSegment(S(4, 2, 1, 1, 0.5, 0.25), S(6, 5, 3, 0, 1, 1), 2.5)
            .AddSegment(S(6, 5, 3, 0, 1, 1), S(9, 5, 3), 2.0);
        builder.AddBlendBetween(0, 1, 0.7, 0.3);
        builder.AddConstantSpeed(3, StretchSpecification.FromParameterFractions(0.3, 0.7), 0.8);
        return builder;
    }

    // ---- sampling ------------------------------------------------------------------------------

    [Fact]
    public void SampleTimes_ContainEveryJunction_AreIncreasing_AndIncludeTheEnds()
    {
        var builder = MixedBuilder();
        Trajectory3D trajectory = builder.Build();

        TrajectorySamples3D samples = trajectory.Sample(301, 4);

        Assert.Equal(0.0, samples.Times[0]);
        Assert.Equal(trajectory.TotalDuration, samples.Times[^1], 1e-12);
        for (int i = 1; i < samples.Times.Length; i++)
            Assert.True(samples.Times[i] > samples.Times[i - 1]);
        foreach (var part in builder.Parts.Skip(1))
            Assert.Contains(samples.Times, t => Math.Abs(t - part.StartTime) <= 1e-12);

        Assert.InRange(samples.Times.Length, 301, 301 + builder.Parts.Count - 1);
        Assert.Equal(5, samples.Values.Length);
        Assert.All(samples.Values, v => Assert.Equal(samples.Times.Length, v.Length));
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.Sample(1, 2));
        Assert.Throws<ArgumentOutOfRangeException>(() => trajectory.Sample(10, 5));
    }

    [Fact]
    public void ASampleExactlyAtAJunction_IsNotDuplicated()
    {
        // Two segments of 1 s: with 3 uniform samples the junction (t = 1) is already among them.
        Trajectory3D trajectory = new TrajectoryBuilder3D()
            .AddRestToRest(new Point3D(0, 0, 0), new Point3D(1, 0, 0), 1.0)
            .AddRestToRest(new Point3D(1, 0, 0), new Point3D(2, 0, 0), 1.0)
            .Build();

        Assert.Equal(3, trajectory.Sample(3, 1).Times.Length);
        Assert.Equal(5, trajectory.Sample(4, 1).Times.Length); // 0, 2/3, 4/3, 2 and the junction at 1
    }

    // ---- extrema --------------------------------------------------------------------------------

    [Fact]
    public void Extrema_AgreeWithADenseSampling()
    {
        Trajectory3D trajectory = MixedBuilder().Build();
        TrajectoryExtrema3D extrema = TrajectoryAnalyzer3D.Extrema(trajectory);

        TrajectorySamples3D dense = trajectory.Sample(30000, 4);
        double[] tolerance = { 1e-6, 1e-6, 1e-6, 1e-4, 1e-3 }; // the higher the order, the coarser the sampling error
        for (int axis = 0; axis < 3; axis++)
        {
            for (int order = 0; order <= 4; order++)
            {
                double[] values = dense.Values[order].Select(p => p[axis]).ToArray();
                AxisExtremum e = extrema.Of(axis, order);
                double scale = 1 + Math.Abs(e.MaxAbsolute);

                Assert.True(e.Minimum <= values.Min() + 1e-9 * scale, $"min {axis}/{order}");
                Assert.True(e.Maximum >= values.Max() - 1e-9 * scale, $"max {axis}/{order}");
                Assert.Equal(values.Min(), e.Minimum, tolerance[order] * scale);
                Assert.Equal(values.Max(), e.Maximum, tolerance[order] * scale);
                Assert.Equal(values.Max(Math.Abs), e.MaxAbsolute, tolerance[order] * scale);

                // The reported time reproduces the value.
                Assert.Equal(e.Maximum, trajectory.EvaluateDerivative(Math.Min(e.MaximumTime, trajectory.TotalDuration), order)[axis], tolerance[order] * scale);
                Assert.Equal(e.Minimum, trajectory.EvaluateDerivative(Math.Min(e.MinimumTime, trajectory.TotalDuration), order)[axis], tolerance[order] * scale);
            }
        }
    }

    [Fact]
    public void MaxSpeed_AgreesWithADenseSampling_AndTheStretchSpeedIsConstant()
    {
        Trajectory3D trajectory = MixedBuilder().Build();
        TrajectoryExtrema3D extrema = TrajectoryAnalyzer3D.Extrema(trajectory);

        TrajectorySamples3D dense = trajectory.Sample(30000, 1);
        double sampled = dense.Values[1].Max(v => v.Length);

        Assert.True(extrema.MaxSpeed >= sampled - 1e-9);
        Assert.Equal(sampled, extrema.MaxSpeed, 1e-6);
        Assert.Equal(extrema.MaxSpeed, trajectory.EvaluateDerivative(extrema.MaxSpeedTime, 1).Length, 1e-6);
    }

    [Fact]
    public void ThePartOfTheMaximum_IsReported()
    {
        // The second segment is much faster: its velocity peak is the trajectory's.
        var builder = new TrajectoryBuilder3D()
            .AddRestToRest(new Point3D(0, 0, 0), new Point3D(1, 0, 0), 2.0)
            .AddRestToRest(new Point3D(1, 0, 0), new Point3D(6, 0, 0), 1.0);

        AxisExtremum vx = TrajectoryAnalyzer3D.Extrema(builder.Build()).Of(0, 1);

        Assert.Equal(1, vx.MaxAbsolutePart);
        Assert.InRange(vx.MaxAbsoluteTime, 2.0, 3.0);
    }

    // ---- limits --------------------------------------------------------------------------------------

    [Fact]
    public void LimitReport_ShowsTheBindingAxisAndPart()
    {
        // Motion mostly along x in the second segment.
        Trajectory3D trajectory = new TrajectoryBuilder3D()
            .AddRestToRest(new Point3D(0, 0, 0), new Point3D(1, 1, 0), 2.0)
            .AddRestToRest(new Point3D(1, 1, 0), new Point3D(7, 1.5, 0), 2.0)
            .Build();
        var limits = KinematicLimits.FromValues(vMax: 10, aMax: 20);

        var rows = TrajectoryAnalyzer3D.LimitReport(trajectory, limits);

        Assert.Equal(6, rows.Count); // two limits x three axes
        var velocity = rows.Where(r => r.Quantity == "Velocity").ToList();
        var worst = velocity.OrderByDescending(r => r.PercentOfLimit).First();
        Assert.Equal("x", worst.Axis);
        Assert.Equal(1, worst.PartIndex);
        Assert.Equal("Segment 2", worst.PartLabel);
        Assert.Equal(100.0 * worst.Peak / worst.Limit, worst.PercentOfLimit, 1e-9);
        Assert.All(rows, r => Assert.False(r.Violated));
        Assert.Equal(0.0, rows.Single(r => r.Quantity == "Velocity" && r.Axis == "z").Peak, 1e-12);
        Assert.Equal(rows, TrajectoryAnalyzer3D.VerifyLimits(trajectory, limits));
    }

    [Fact]
    public void AViolatedLimit_NamesPartAxisAndQuantity()
    {
        Trajectory3D trajectory = new TrajectoryBuilder3D()
            .AddRestToRest(new Point3D(0, 0, 0), new Point3D(1, 1, 0), 2.0)
            .AddRestToRest(new Point3D(1, 1, 0), new Point3D(7, 1.5, 0), 2.0)
            .Build();
        var limits = KinematicLimits.FromValues(vMax: 2);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(() => TrajectoryAnalyzer3D.VerifyLimits(trajectory, limits));

        Assert.Contains("Segment 2", ex.Message);
        Assert.Contains("axis x", ex.Message);
        Assert.Contains("velocity", ex.Message);
        Assert.Contains(TrajectoryAnalyzer3D.LimitReport(trajectory, limits), r => r.Violated && r.Axis == "x" && r.PercentOfLimit > 100);
    }

    [Fact]
    public void ALimitedMixedTrajectory_PassesTheWholeVerification()
    {
        var limits = KinematicLimits.FromValues(vMax: 6, aMax: 40, jMax: 500, sMax: 20000);
        var builder = new TrajectoryBuilder3D()
            .AddSegment(S(0, 0, 0), S(4, 2, 1, 1, 0.5, 0.25), null, null, limits)
            .AddSegment(S(4, 2, 1, 1, 0.5, 0.25), S(6, 5, 3, 0, 1, 1), null, null, limits)
            .AddSegment(S(6, 5, 3, 0, 1, 1), S(9, 5, 3), null, null, limits);
        builder.AddBlendBetween(0, 1, 0.7, 0.3, limits: limits);
        builder.AddConstantSpeed(3, StretchSpecification.FromParameterFractions(0.3, 0.7), 0.8, limits: limits);

        var rows = TrajectoryAnalyzer3D.VerifyLimits(builder.Build(), limits);

        Assert.All(rows, r => Assert.True(r.PercentOfLimit <= 100.0 + 1e-6));
        // The stretch part is the one that holds the velocity peak of nothing larger than its speed.
        Assert.Contains(rows, r => r.Quantity == "Velocity");
    }

    [Fact]
    public void LimitPeaksOfTheStretch_AreItsOwn()
    {
        // A curved stretch alone: the acceleration peak per axis is v^2 times the geometric peak.
        var parabola = new PolynomialSegment3D(
            9,
            new State3D(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
            new State3D(new Point3D(1, 1, 0), new Point3D(1, 2, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
            1.0);
        var stretch = new CurvedConstantSpeedStretch3D(new ArcLengthPath(parabola, 0, 1), 1.0);
        var trajectory = new Trajectory3D(new ISegment3D[] { stretch });

        AxisExtremum ay = TrajectoryAnalyzer3D.Extrema(trajectory).Of(1, 2);

        Assert.Equal(2.0, ay.MaxAbsolute, 1e-5);     // P_2 of the theory (y component)
        Assert.Equal(0.0, ay.MaxAbsoluteTime, 1e-5); // at the start of the parabola
        Assert.Equal(0.0, ay.MaxAbsolutePart);
    }

    // ---- path length ------------------------------------------------------------------------------------

    [Fact]
    public void PathLength_OfAStraightSegment_IsTheChord()
    {
        var chord = new Point3D(3, 4, 12).Length;
        Trajectory3D trajectory = new TrajectoryBuilder3D().AddRestToRest(new Point3D(0, 0, 0), new Point3D(3, 4, 12), 2.0).Build();

        Assert.Equal(chord, TrajectoryAnalyzer3D.PathLength(trajectory), 1e-10);
    }

    [Fact]
    public void PathLength_OfTheTheoryParabola_Is1_478943()
    {
        var parabola = new PolynomialSegment3D(
            9,
            new State3D(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
            new State3D(new Point3D(1, 1, 0), new Point3D(1, 2, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
            1.0);
        Trajectory3D trajectory = new(new ISegment3D[] { parabola });

        double length = TrajectoryAnalyzer3D.PathLength(trajectory);

        Assert.Equal(1.478943, length, 6);
        Assert.True(Math.Abs(length - (2 * Math.Sqrt(5) + Math.Asinh(2)) / 4) <= 1e-12 * length);
    }

    [Fact]
    public void PathLength_OfAMixedTrajectory_IsTheSumOfItsPieces()
    {
        var builder = MixedBuilder();
        Trajectory3D trajectory = builder.Build();

        double expected = 0;
        foreach (var part in builder.Parts)
        {
            expected += part.Segment switch
            {
                PolynomialSegment3D p => new ArcLengthPath(p, 0, p.Duration, requireRegular: false).Length,
                CurvedConstantSpeedStretch3D s => s.Subpath.Length,
                _ => throw new InvalidOperationException()
            };
        }

        Assert.Equal(expected, TrajectoryAnalyzer3D.PathLength(trajectory), 1e-12);
        // Independent check: a fine polyline through dense samples.
        var samples = trajectory.Sample(60000, 0).Values[0];
        double polyline = 0;
        for (int i = 1; i < samples.Length; i++)
            polyline += (samples[i] - samples[i - 1]).Length;
        Assert.Equal(polyline, TrajectoryAnalyzer3D.PathLength(trajectory), 1e-4);
    }
}
