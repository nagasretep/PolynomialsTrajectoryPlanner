using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.PassThrough;

/// <summary>
/// Request 0011: exact pass-through with kinematic limits (theory/TrajectoryPassingThroughConstraintPoints.md,
/// "Pass-Through With Kinematic Limits"). Reference values come from Cowork's NumPy prototype (peaks by 4001-point
/// sampling, hence 1e-5 relative); the binding peaks are exact (exact extrema) and compared at 1e-9.
/// </summary>
public class PassThroughWithLimitsTests
{
    private static readonly double[] Data1 = { 0, 1, 2 };
    private static readonly double[] Data2 = { 0, 1, 3, 2 };

    private static void AssertRelative(double expected, double actual, double tolerance, string what) =>
        Assert.True(
            Math.Abs(expected - actual) <= tolerance * Math.Abs(expected),
            $"{what}: expected {expected:R}, got {actual:R}");

    private static double[] Peaks(IEnumerable<PolynomialSegment1D> segments) =>
        Enumerable.Range(1, 4).Select(k => ExtremaAnalyzer.PeakAbsolute(segments, k)).ToArray();

    private static PolynomialSegment1D[] Solve(double[] x, double[] t, int degree, int? order = null) =>
        new PassThroughSolver1D(order is null ? null : new SmoothnessCriterion(order.Value)).Solve(x, t, degree);

    private static bool Respects(PassThroughResult r, KinematicLimits limits) =>
        r.Segments.All(s => ExtremaAnalyzer.VerifyLimits(s, limits));

    // ---- reference values of the theory ----------------------------------------------------

    [Fact]
    public void ReferenceData1_PeaksOfTheReferenceSolution()
    {
        double[] p = Peaks(Solve(Data1, new[] { 1.0, 1 }, 9, 4));

        AssertRelative(2.275, p[0], 1e-5, "P1");
        AssertRelative(4.094571, p[1], 1e-5, "P2");
        AssertRelative(15.4, p[2], 1e-5, "P3");
        AssertRelative(74.005522, p[3], 1e-5, "P4");
    }

    [Fact]
    public void ReferenceData1_AccelerationBinds_LambdaDurationsTotalAndPeaks()
    {
        var limits = KinematicLimits.FromValues(vMax: 2, aMax: 3, jMax: 10, sMax: 50);

        var r = PassThroughPlanner1D.Plan(Data1, 9, new SmoothnessCriterion(4), limits, referenceDurations: new[] { 1.0, 1 });

        Assert.True(r.ScaledByLimits);
        AssertRelative(1.168271, r.Scale, 1e-6, "lambda*");
        AssertRelative(1.168271, r.Durations[0], 1e-6, "T0");
        AssertRelative(1.168271, r.Durations[1], 1e-6, "T1");
        AssertRelative(2.336542, r.TotalDuration, 1e-6, "total");

        double[] p = Peaks(r.Segments);
        AssertRelative(1.947322, p[0], 1e-5, "v at lambda*");
        AssertRelative(3.0, p[1], 1e-9, "a at lambda* (binding, exact)");
        AssertRelative(9.658063, p[2], 1e-5, "j at lambda*");
        AssertRelative(39.727378, p[3], 1e-5, "s at lambda*");
        Assert.True(Respects(r, limits));
    }

    [Fact]
    public void ReferenceData2_PeaksOfTheReferenceSolutions()
    {
        double[] a = Peaks(Solve(Data2, new[] { 0.7, 1.3, 0.9 }, 9, 4));
        AssertRelative(4.47151, a[0], 1e-5, "P1");
        AssertRelative(9.626191, a[1], 1e-5, "P2");
        AssertRelative(33.013296, a[2], 1e-5, "P3");
        AssertRelative(255.962162, a[3], 1e-5, "P4");

        double[] b = Peaks(Solve(Data2, new[] { 1.0, 1, 1 }, 9, 4));
        AssertRelative(3.490773, b[0], 1e-5, "P1 equal durations");
        AssertRelative(8.369728, b[1], 1e-5, "P2 equal durations");
        AssertRelative(24.329164, b[2], 1e-5, "P3 equal durations");
        AssertRelative(129.585159, b[3], 1e-5, "P4 equal durations");
    }

    [Fact]
    public void ReferenceData2_VelocityBinds_LambdaDurationsTotalAndPeaks()
    {
        var limits = KinematicLimits.FromValues(vMax: 3, aMax: 6);

        var r = PassThroughPlanner1D.Plan(Data2, 9, new SmoothnessCriterion(4), limits, referenceDurations: new[] { 0.7, 1.3, 0.9 });

        AssertRelative(1.490503, r.Scale, 1e-6, "lambda*");
        AssertRelative(1.043352, r.Durations[0], 1e-6, "T0");
        AssertRelative(1.937654, r.Durations[1], 1e-6, "T1");
        AssertRelative(1.341453, r.Durations[2], 1e-6, "T2");
        AssertRelative(4.32246, r.TotalDuration, 1e-6, "total");

        double[] p = Peaks(r.Segments);
        AssertRelative(3.0, p[0], 1e-9, "v at lambda* (binding, exact)");
        AssertRelative(4.332999, p[1], 1e-5, "a at lambda*");
        Assert.True(Respects(r, limits));
    }

    // ---- each limit binding in turn --------------------------------------------------------

    [Theory]
    [InlineData(1, 2.0)]
    [InlineData(2, 3.0)]
    [InlineData(3, 10.0)]
    [InlineData(4, 50.0)]
    public void EachLimitBindingInTurn_ItsPeakEqualsTheLimit_AndTheOthersAreRespected(int order, double limit)
    {
        var limits = order switch
        {
            1 => KinematicLimits.FromValues(vMax: limit),
            2 => KinematicLimits.FromValues(aMax: limit),
            3 => KinematicLimits.FromValues(jMax: limit),
            _ => KinematicLimits.FromValues(sMax: limit)
        };

        var r = PassThroughPlanner1D.Plan(Data1, 9, new SmoothnessCriterion(4), limits, referenceDurations: new[] { 1.0, 1 });

        AssertRelative(limit, Peaks(r.Segments)[order - 1], 1e-9, $"peak of order {order}");
        Assert.True(Respects(r, limits));
        Assert.True(r.ScaledByLimits);
    }

    [Fact]
    public void AutomaticDurations_UseTheProportionalReference_AndAreScaledByTheLimits()
    {
        var limits = KinematicLimits.FromValues(vMax: 3);

        var r = PassThroughPlanner1D.Plan(Data2, 9, null, limits);

        // Reference: A8 with a total of 1 s, distances 1, 2, 1 -> (0.25, 0.5, 0.25), then scaled by lambda.
        Assert.Equal(r.Durations[0] / r.Scale, 0.25, 9);
        Assert.Equal(r.Durations[1] / r.Scale, 0.5, 9);
        Assert.Equal(r.Durations[2] / r.Scale, 0.25, 9);
        AssertRelative(3.0, Peaks(r.Segments)[0], 1e-9, "binding velocity");
    }

    // ---- scaling law -----------------------------------------------------------------------

    [Theory]
    [InlineData(5, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 4)]
    [InlineData(9, 5)]
    public void ScalingLaw_PeaksAtHalfAndDoubleDurationsEqualLambdaToTheMinusKTimesTheReferencePeaks(int degree, int order)
    {
        foreach (var (x, t) in new[]
                 {
                     (Data1, new[] { 1.0, 1 }),
                     (Data2, new[] { 0.7, 1.3, 0.9 }),
                     (new[] { 0.0, 2, -1, 3, 3.5, 1 }, new[] { 0.5, 1.0, 0.8, 1.2, 0.6 })
                 })
        {
            double[] reference = Peaks(Solve(x, t, degree, order));
            foreach (double lambda in new[] { 0.5, 2.0 })
            {
                double[] scaled = Peaks(Solve(x, t.Select(v => lambda * v).ToArray(), degree, order));
                for (int k = 1; k <= 4; k++)
                {
                    if (reference[k - 1] == 0) continue;
                    AssertRelative(Math.Pow(lambda, -k) * reference[k - 1], scaled[k - 1], 1e-9, $"degree {degree}, m {order}, lambda {lambda}, order {k}");
                }
            }
        }
    }

    // ---- lower degrees, degenerate limits --------------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    public void LowerDegrees_WithTheDefaultOrder_AreScaledToTheLimit(int degree)
    {
        var limits = KinematicLimits.FromValues(vMax: 2.5);

        var r = PassThroughPlanner1D.Plan(Data2, degree, null, limits);

        AssertRelative(2.5, Peaks(r.Segments)[0], 1e-9, "binding velocity");
        Assert.True(Respects(r, limits));
    }

    [Fact]
    public void Cubic_SnapLimitAlone_ImposesNothing_BecauseTheCubicHasNoSnap()
    {
        var r = PassThroughPlanner1D.Plan(Data2, 3, null, KinematicLimits.FromValues(sMax: 1.0));

        Assert.False(r.ScaledByLimits);
        Assert.Equal(1.0, r.Scale);
    }

    [Fact]
    public void Cubic_SnapLimitTogetherWithVelocity_DoesNotBreakTheFormula()
    {
        var limits = KinematicLimits.FromValues(vMax: 2.5, sMax: 1.0);

        var r = PassThroughPlanner1D.Plan(Data2, 3, null, limits);

        Assert.True(r.ScaledByLimits);
        AssertRelative(2.5, Peaks(r.Segments)[0], 1e-9, "binding velocity");
    }

    [Fact]
    public void AllWaypointsEqual_LimitsDetermineNothing_ReferenceDurationsAreUsed()
    {
        var r = PassThroughPlanner1D.Plan(new[] { 3.0, 3, 3, 3 }, 9, null, KinematicLimits.FromValues(vMax: 1, aMax: 1));

        Assert.False(r.ScaledByLimits);
        Assert.Equal(1.0, r.Scale);
        Assert.Equal(new[] { 1.0 / 3, 1.0 / 3, 1.0 / 3 }, r.Durations, new Tolerance(1e-12));
    }

    [Fact]
    public void NoActiveLimit_ReferenceDurationsUnscaled()
    {
        foreach (var limits in new KinematicLimits?[] { null, KinematicLimits.None })
        {
            var r = PassThroughPlanner1D.Plan(Data2, 9, null, limits);

            Assert.False(r.ScaledByLimits);
            Assert.Equal(1.0, r.TotalDuration, 12);
        }
    }

    [Fact]
    public void ASingleLimit_IsEnough()
    {
        var r = PassThroughPlanner1D.Plan(Data2, 9, null, KinematicLimits.FromValues(jMax: 20));

        AssertRelative(20, Peaks(r.Segments)[2], 1e-9, "binding jerk");
    }

    // ---- a larger case and overshoot -------------------------------------------------------

    [Fact]
    public void ThirtySegmentsWithADurationRatioOf100_Runs_IsFeasible_AndLambdaIsFinite()
    {
        var random = new Random(2026);
        var t = new double[30];
        for (int k = 0; k < 30; k++) t[k] = 0.05 * Math.Pow(100, random.NextDouble());
        t[3] = 0.05; t[17] = 5.0;
        var x = new double[31];
        for (int i = 1; i <= 30; i++) x[i] = x[i - 1] + (random.NextDouble() - 0.4) * 4;
        var limits = KinematicLimits.FromValues(vMax: 5, aMax: 20, jMax: 200);

        var r = PassThroughPlanner1D.Plan(x, 9, null, limits, referenceDurations: t);

        Assert.True(double.IsFinite(r.Scale) && r.Scale > 0);
        Assert.True(Respects(r, limits));
        Assert.Equal(30, r.Segments.Length);
        Assert.Equal(t.Select(v => v * r.Scale), r.Durations, new Tolerance(1e-12));
    }

    [Fact]
    public void WhenThePeakIsBetweenNodes_TheLimitBindsOnTheTruePeak()
    {
        double[] x = { 0, 2, 2.2, 5, 4.9, 6 };
        var limits = KinematicLimits.FromValues(vMax: 4, aMax: 20);

        var r = PassThroughPlanner1D.Plan(x, 9, null, limits, referenceDurations: new[] { 1.0, 1, 1, 1, 1 });

        double nodePeak = r.Segments.Max(s => Math.Max(Math.Abs(s.StartState.Velocity), Math.Abs(s.EndState.Velocity)));
        double truePeak = ExtremaAnalyzer.PeakAbsolute(r.Segments, 1);

        Assert.True(truePeak > nodePeak * 1.01, $"true peak {truePeak}, node peak {nodePeak}");
        AssertRelative(4.0, truePeak, 1e-9, "velocity peak");
        Assert.True(Respects(r, limits));
    }

    // ---- non-zero boundary derivatives -----------------------------------------------------

    [Fact]
    public void BoundaryDerivatives_AreHonouredBySolverAndPlanner()
    {
        var start = new State1D(0, 1.5, 0.5, 0, 0);
        var end = new State1D(2, -1, 0, 0, 0);

        var segments = new PassThroughSolver1D().Solve(Data1, new[] { 1.0, 1 }, 9, start, end);

        Assert.Equal(1.5, segments[0].StartState.Velocity, 12);
        Assert.Equal(0.5, segments[0].StartState.Acceleration, 12);
        Assert.Equal(-1.0, segments[^1].EndState.Velocity, 12);
        Assert.Equal(0.0, segments[^1].EndState.Acceleration, 12);
        // continuity at the interior node
        for (int order = 0; order <= 4; order++)
            Assert.Equal(segments[0].EvaluateDerivative(1.0, order), segments[1].EvaluateDerivative(0, order), 1e-6);
    }

    [Fact]
    public void NonZeroStartVelocity_ResultIsFeasible_ByTheSearch_AndLambdaDiffersFromTheZeroBoundaryValue()
    {
        var limits = KinematicLimits.FromValues(vMax: 3, aMax: 6);
        double[] reference = { 0.7, 1.3, 0.9 };
        var zero = PassThroughPlanner1D.Plan(Data2, 9, new SmoothnessCriterion(4), limits, referenceDurations: reference);

        var along = PassThroughPlanner1D.Plan(Data2, 9, new SmoothnessCriterion(4), limits, referenceDurations: reference,
            startBoundary: new State1D(0, 1.5, 0, 0, 0));
        var against = PassThroughPlanner1D.Plan(Data2, 9, new SmoothnessCriterion(4), limits, referenceDurations: reference,
            startBoundary: new State1D(0, -1.5, 0, 0, 0));

        Assert.True(Respects(along, limits));
        Assert.True(Respects(against, limits));
        Assert.Equal(1.5, along.Segments[0].StartState.Velocity, 12);

        // Documented comparison (measured 2026-10-17): zero-boundary lambda* = 1.4905; starting with a velocity
        // along the motion helps (1.1908), starting against it hurts a lot (5.9005). The closed form does not apply
        // here: boundary velocities do not scale with lambda.
        AssertRelative(1.490503, zero.Scale, 1e-6, "zero-boundary lambda*");
        AssertRelative(1.190828, along.Scale, 1e-4, "start velocity along the motion");
        AssertRelative(5.900517, against.Scale, 1e-4, "start velocity against the motion");
        Assert.True(along.Scale < zero.Scale);
        Assert.True(against.Scale > zero.Scale);

        // The result is the boundary of feasibility: slightly shorter durations violate a limit.
        var shorter = new PassThroughSolver1D(new SmoothnessCriterion(4)).Solve(
            Data2, reference.Select(v => v * along.Scale * (1 - 1e-4)).ToArray(), 9, new State1D(0, 1.5, 0, 0, 0));
        Assert.False(shorter.All(s => ExtremaAnalyzer.VerifyLimits(s, limits)));
    }

    [Fact]
    public void BoundaryVelocityAboveTheVelocityLimit_ReportsASearchFailure()
    {
        var ex = Assert.Throws<DurationSearchFailedException>(() => PassThroughPlanner1D.Plan(
            Data2, 9, null, KinematicLimits.FromValues(vMax: 3), startBoundary: new State1D(0, 10, 0, 0, 0)));

        Assert.StartsWith("The pass-through:", ex.Message);
    }

    // ---- prescribed durations --------------------------------------------------------------

    [Fact]
    public void PrescribedDurations_AreKeptAndVerified()
    {
        var limits = KinematicLimits.FromValues(vMax: 10, aMax: 100);

        var r = PassThroughPlanner1D.Plan(Data2, 9, null, limits, prescribedDurations: new[] { 0.7, 1.3, 0.9 });

        Assert.Equal(new[] { 0.7, 1.3, 0.9 }, r.Durations);
        Assert.Equal(1.0, r.Scale);
        Assert.False(r.ScaledByLimits);
    }

    [Fact]
    public void PrescribedDurationsViolatingALimit_AreReportedWithTheSegmentIndexAndTheLimit()
    {
        var limits = KinematicLimits.FromValues(vMax: 3);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(
            () => PassThroughPlanner1D.Plan(Data2, 9, new SmoothnessCriterion(4), limits, prescribedDurations: new[] { 0.7, 1.3, 0.9 }));

        Assert.Contains("segment", ex.Message);
        Assert.Contains("velocity limit", ex.Message);
        Assert.Matches(@"segment [1-3] ", ex.Message);
    }

    [Fact]
    public void PrescribedAndReferenceDurationsTogether_Rejected()
    {
        Assert.Throws<ArgumentException>(() => PassThroughPlanner1D.Plan(
            Data2, 9, null, null, prescribedDurations: new[] { 1.0, 1, 1 }, referenceDurations: new[] { 1.0, 1, 1 }));
    }

    [Fact]
    public void WrongNumberOfDurations_Rejected()
    {
        Assert.Throws<ArgumentException>(() => PassThroughPlanner1D.Plan(Data2, 9, null, null, prescribedDurations: new[] { 1.0 }));
        Assert.Throws<ArgumentException>(() => PassThroughPlanner1D.Plan(Data2, 9, null, null, referenceDurations: new[] { 1.0 }));
        Assert.Throws<ArgumentException>(() => PassThroughPlanner1D.Plan(Data2, 9, null, null, referenceDurations: new[] { 1.0, 0, 1 }));
    }

    // ---- builder ---------------------------------------------------------------------------

    [Fact]
    public void Builder_AutomaticDurationsWithLimits_ExposeLambdaAndTheFinalDurations()
    {
        var limits = KinematicLimits.FromValues(vMax: 3);
        var builder = new TrajectoryBuilder1D();

        var trajectory = builder.AddSegmentsFromWaypointsPassThrough(Data2, limits: limits).Build();

        var result = builder.LastPassThroughResult!;
        Assert.True(result.ScaledByLimits);
        Assert.Equal(result.TotalDuration, trajectory.TotalDuration, 12);
        AssertRelative(3.0, ExtremaAnalyzer.PeakAbsolute(trajectory.Segments, 1), 1e-9, "binding velocity");
    }

    [Fact]
    public void Builder_PrescribedDurationsWithLimits_AreVerifiedNotScaled()
    {
        var limits = KinematicLimits.FromValues(vMax: 3);

        Assert.Throws<KinematicLimitsViolatedException>(() => new TrajectoryBuilder1D()
            .AddSegmentsFromWaypointsPassThrough(Data2, durations: new[] { 0.7, 1.3, 0.9 }, limits: limits));
        Assert.Throws<KinematicLimitsViolatedException>(() => new TrajectoryBuilder1D()
            .AddSegmentsFromWaypointsPassThrough(Data2, totalDuration: 2.0, limits: limits));

        var ok = new TrajectoryBuilder1D().AddSegmentsFromWaypointsPassThrough(Data2, totalDuration: 20.0, limits: limits).Build();
        Assert.Equal(20.0, ok.TotalDuration, 12);
    }

    [Fact]
    public void Builder_WithoutLimits_KeepsTheEarlierBehaviour_TotalOneSecondByDefault()
    {
        var trajectory = new TrajectoryBuilder1D().AddSegmentsFromWaypointsPassThrough(Data2).Build();

        Assert.Equal(1.0, trajectory.TotalDuration, 12);
    }

    [Fact]
    public void Builder_NonZeroBoundary_IsPassedThrough()
    {
        var trajectory = new TrajectoryBuilder1D()
            .AddSegmentsFromWaypointsPassThrough(Data2, startBoundary: new State1D(0, 1.5, 0, 0, 0))
            .Build();

        Assert.Equal(1.5, trajectory.Segments[0].StartState.Velocity, 12);
    }

    // ---- helper ----------------------------------------------------------------------------

    private sealed class Tolerance : IEqualityComparer<double>
    {
        private readonly double _tolerance;
        public Tolerance(double tolerance) => _tolerance = tolerance;
        public bool Equals(double a, double b) => Math.Abs(a - b) <= _tolerance;
        public int GetHashCode(double value) => 0;
    }
}

public class PeakAbsoluteTests
{
    [Fact]
    public void PeakAbsolute_IsTheLargestPeakOverTheSegments()
    {
        var a = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), 1.0);
        var b = new PolynomialSegment1D(9, State1D.FromPositionOnly(1), State1D.FromPositionOnly(4), 1.0);

        double peak = ExtremaAnalyzer.PeakAbsolute(new[] { a, b }, 1);

        Assert.Equal(3 * 315.0 / 128, peak, 1e-9);
        Assert.Equal(0.0, ExtremaAnalyzer.PeakAbsolute(Array.Empty<PolynomialSegment1D>(), 1));
    }
}
