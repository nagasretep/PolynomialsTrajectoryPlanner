using TrajectoryPlanner.Core.ArcLength;
using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.ConstantVelocity;

/// <summary>Request 0016: constant speed along a curved subpath (arc-length derivatives, inversion, stretch, transitions, limits).</summary>
public class ConstantSpeedTrajectory3DTests
{
    private static State3D S(double x, double y, double z, double vx = 0, double vy = 0, double vz = 0) =>
        new(new Point3D(x, y, z), new Point3D(vx, vy, vz), Point3D.Zero, Point3D.Zero, Point3D.Zero);

    /// <summary>The path p(t) = (t, t², 0), t in [0, 1], as an exact polynomial segment (boundary states of the parabola).</summary>
    private static PolynomialSegment3D Parabola(int degree = 9) => new(
        degree,
        new State3D(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
        new State3D(new Point3D(1, 1, 0), new Point3D(1, 2, 0), new Point3D(0, 2, 0), Point3D.Zero, Point3D.Zero),
        1.0);

    /// <summary>A genuinely 3D curve of degree 5 (curvature and torsion), regular on its whole length.</summary>
    private static PolynomialSegment3D SpaceCurve5() => new(
        5,
        new State3D(new Point3D(0, 0, 0), new Point3D(2, 1, 0.5), new Point3D(0, 3, 1), Point3D.Zero, Point3D.Zero),
        new State3D(new Point3D(3, 2, 1.5), new Point3D(0.5, 2, 2), new Point3D(-1, 0, 2), Point3D.Zero, Point3D.Zero),
        2.0);

    private static PolynomialSegment3D SpaceCurve9() => new(
        9,
        new State3D(new Point3D(0, 0, 0), new Point3D(2, 1, 0.5), new Point3D(0, 3, 1), new Point3D(1, 0, 0), new Point3D(0, 0, 1)),
        new State3D(new Point3D(3, 2, 1.5), new Point3D(0.5, 2, 2), new Point3D(-1, 0, 2), new Point3D(0, 1, 0), new Point3D(1, 0, 0)),
        2.0);

    private static void AssertPoint(Point3D expected, Point3D actual, double tolerance)
    {
        for (int i = 0; i < 3; i++)
            Assert.Equal(expected[i], actual[i], tolerance);
    }

    // ---- series arithmetic and arc-length derivatives ---------------------------------------

    [Fact]
    public void TaylorSeries_ReciprocalAndSquareRoot_AreInverseOperations()
    {
        double[] a = { 4.0, 1.5, -0.7, 0.3, 0.2 };

        double[] root = TaylorSeries.SquareRoot(a);
        double[] back = TaylorSeries.Multiply(root, root);
        double[] unit = TaylorSeries.Multiply(a, TaylorSeries.Reciprocal(a));

        for (int k = 0; k < a.Length; k++)
        {
            Assert.Equal(a[k], back[k], 1e-12);
            Assert.Equal(k == 0 ? 1.0 : 0.0, unit[k], 1e-12);
        }
    }

    [Fact]
    public void ParabolaDerivatives_AtTheStart_MatchTheTheoryReference()
    {
        var (sigma, d) = ArcLengthDerivatives.Compute(Parabola(), 0.0);

        Assert.Equal(1.0, sigma, 1e-12);
        AssertPoint(new Point3D(1, 0, 0), d[0], 1e-9);
        AssertPoint(new Point3D(0, 2, 0), d[1], 1e-9);
        AssertPoint(new Point3D(-4, 0, 0), d[2], 1e-9);
        AssertPoint(new Point3D(0, -32, 0), d[3], 1e-9);
    }

    [Fact]
    public void ParabolaDerivatives_AtTheEnd_MatchTheTheoryReference()
    {
        var (sigma, d) = ArcLengthDerivatives.Compute(Parabola(), 1.0);

        Assert.Equal(Math.Sqrt(5), sigma, 1e-12);
        AssertPoint(new Point3D(1 / Math.Sqrt(5), 2 / Math.Sqrt(5), 0), d[0], 1e-9);
        AssertPoint(new Point3D(-0.16, 0.08, 0), d[1], 1e-9);              // exact: (-4/25, 2/25)
        AssertPoint(new Point3D(0.157419, -0.114487, 0), d[2], 1e-6);      // the theory prints six digits
        AssertPoint(new Point3D(-0.24064, 0.23552, 0), d[3], 1e-5);
    }

    [Theory]
    [InlineData(0.3)]
    [InlineData(1.1)]
    [InlineData(1.8)]
    public void SpaceCurve_DerivativeOfOrderK_IsTheArcLengthDerivativeOfOrderKMinus1(double t)
    {
        // Independent check: D^(k+1) p = d/dt(D^k p) / sigma, by central differences of the lower order.
        var curve = SpaceCurve5();
        const double h = 1e-5;

        var (sigma, d) = ArcLengthDerivatives.Compute(curve, t);
        var (_, up) = ArcLengthDerivatives.Compute(curve, t + h);
        var (_, down) = ArcLengthDerivatives.Compute(curve, t - h);

        for (int k = 0; k < 3; k++)
        {
            Point3D numeric = (up[k] - down[k]) / (2 * h * sigma);
            AssertPoint(d[k + 1], numeric, 1e-6 * (1 + d[k + 1].Length));
        }

        // First order: the unit tangent.
        AssertPoint(curve.EvaluateDerivative(t, 1) / sigma, d[0], 1e-12);
        Assert.Equal(1.0, d[0].Length, 1e-12);
        // Second order: closed form (1/σ²)(p̈ − (ṗ·p̈/σ²) ṗ).
        Point3D pd = curve.EvaluateDerivative(t, 1), pdd = curve.EvaluateDerivative(t, 2);
        AssertPoint((pdd - pd * (pd.Dot(pdd) / (sigma * sigma))) / (sigma * sigma), d[1], 1e-10);
    }

    [Fact]
    public void ARestPoint_IsRejectedExplicitly()
    {
        var restToRest = new PolynomialSegment3D(9, S(0, 0, 0), S(2, 1, 0), 1.0);

        Assert.Throws<PathNotRegularException>(() => ArcLengthDerivatives.Compute(restToRest, 0.0));
        Assert.Throws<PathNotRegularException>(() => new ArcLengthPath(restToRest, 0.0, 0.5));
        Assert.Throws<PathNotRegularException>(() => new ArcLengthPath(restToRest, 0.5, 1.0));
        _ = new ArcLengthPath(restToRest, 0.05, 0.95); // regular inside
        _ = new ArcLengthPath(restToRest, 0.0, 1.0, requireRegular: false); // only its length
    }

    // ---- arc length and inversion --------------------------------------------------------------

    [Fact]
    public void ParabolaLength_MatchesTheClosedForm()
    {
        var path = new ArcLengthPath(Parabola(), 0, 1);

        double exact = (2 * Math.Sqrt(5) + Math.Asinh(2)) / 4;
        Assert.Equal(1.478943, path.Length, 6);
        Assert.True(Math.Abs(path.Length - exact) <= 1e-12 * exact);
    }

    [Fact]
    public void Inversion_RoundTripsRandomParameters_ToTwelveDigits()
    {
        var path = new ArcLengthPath(Parabola(), 0, 1);
        var random = new Random(12345);

        for (int i = 0; i < 200; i++)
        {
            double t = random.NextDouble();
            double back = path.TimeAtLength(path.LengthAt(t));
            Assert.Equal(t, back, 1e-12);
        }

        Assert.Equal(1.0, path.TimeAtLength(path.Length));
        Assert.Equal(0.0, path.TimeAtLength(0));
    }

    [Fact]
    public void Inversion_OnAPathWithStronglyVaryingSpeed_RoundTripsTheLength()
    {
        // Rest-to-rest segment: the speed goes from ~0 at the ends to its maximum in the middle.
        var strong = new PolynomialSegment3D(9, S(0, 0, 0), S(4, 2, 1), 1.0);
        var path = new ArcLengthPath(strong, 0.04, 0.96);
        double sigmaMin = Math.Min(path.Speed(0.04), path.Speed(0.96));
        double sigmaMax = path.Speed(0.5);
        Assert.True(sigmaMax / sigmaMin > 1e3);

        var random = new Random(7);
        for (int i = 0; i < 200; i++)
        {
            double ell = random.NextDouble() * path.Length;
            double t = path.TimeAtLength(ell);
            Assert.InRange(t, 0.04, 0.96);
            Assert.True(Math.Abs(path.LengthAt(t) - ell) <= 1e-12 * path.Length);
        }

        // The length of the straight path is the chord.
        Assert.Equal(strong.EvaluatePosition(0.96).DistanceTo(strong.EvaluatePosition(0.04)), path.Length, 1e-10);
    }

    // ---- peaks and admissible speed ------------------------------------------------------------

    [Fact]
    public void ParabolaPeaks_MatchTheTheoryReference()
    {
        var stretch = new CurvedConstantSpeedStretch3D(new ArcLengthPath(Parabola(), 0, 1), 1.0);

        double[][] p = stretch.ComponentPeaks();

        Assert.Equal(1.0, p[0][0], 1e-5);
        Assert.Equal(0.894427, p[0][1], 1e-5);
        Assert.Equal(0.649519, p[1][0], 1e-5);
        Assert.Equal(2.0, p[1][1], 1e-5);
        Assert.Equal(4.0, p[2][0], 1e-5);
        Assert.Equal(3.808294, p[2][1], 1e-5);
        // The theory samples 2001 points (18.187075); the refined maximum is the true one, 3.5e-5 higher.
        Assert.Equal(18.187075, p[3][0], 1e-4);
        Assert.True(p[3][0] >= 18.187075);
        double sampled = Enumerable.Range(0, 2001)
            .Max(i => Math.Abs(ArcLengthDerivatives.Compute(Parabola(), i / 2000.0).Derivatives[3].X));
        Assert.Equal(18.187075, sampled, 1e-5);
        Assert.Equal(32.0, p[3][1], 1e-5);
        Assert.Equal(new[] { 1.0, 2.0, 4.0, 32.0 }, stretch.Peaks(), new Tolerance(1e-5));
        Assert.All(p, comps => Assert.Equal(0.0, comps[2], 12)); // z does not move
    }

    private sealed class Tolerance : IEqualityComparer<double>
    {
        private readonly double _tol;
        public Tolerance(double tol) => _tol = tol;
        public bool Equals(double x, double y) => Math.Abs(x - y) <= _tol;
        public int GetHashCode(double obj) => 0;
    }

    [Fact]
    public void ParabolaAdmissibleSpeed_IsBoundByTheJerk()
    {
        var stretch = new CurvedConstantSpeedStretch3D(new ArcLengthPath(Parabola(), 0, 1), 1.0);

        var (speed, binding) = stretch.AdmissibleSpeed(KinematicLimits.FromValues(aMax: 8, jMax: 20));

        Assert.Equal(1.709976, speed, 6);
        Assert.Equal("jerk", binding);
        Assert.Equal(double.PositiveInfinity, stretch.AdmissibleSpeed(null).Speed);
    }

    // ---- the stretch ---------------------------------------------------------------------------

    [Fact]
    public void TheStretch_HasConstantSpeed_AndStaysOnThePath_AndTimeIsLengthOverSpeed()
    {
        var curve = SpaceCurve9();
        var t = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.7, S(-1, -1, 0), S(4, 3, 2), 9);
        var stretch = t.Stretch;

        Assert.Equal(stretch.Subpath.Length / 1.7, stretch.Duration, 1e-12);
        for (int i = 0; i <= 60; i++)
        {
            double time = stretch.Duration * i / 60;
            Assert.Equal(1.7, stretch.EvaluateDerivative(time, 1).Length, 1e-9);

            Point3D position = stretch.EvaluatePosition(time);
            double tau = BlendBuilder3D.TauForPoint(curve, position); // throws if the point is farther than 1e-9 of the extent
            Assert.InRange(tau, 0.001, 0.999);
            // Distance travelled along the path equals v t.
            Assert.Equal(1.7 * time, stretch.Subpath.LengthAt(tau * curve.Duration), 1e-7);
        }

        // At T_cv the inversion returns t_e.
        Assert.Equal(0.85 * curve.Duration, stretch.Subpath.TimeAtLength(1.7 * stretch.Duration), 1e-12);
    }

    [Fact]
    public void TheStretchDerivatives_AreVToTheKTimesTheArcLengthDerivatives()
    {
        var curve = SpaceCurve9();
        var t = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.7, S(-1, -1, 0), S(4, 3, 2));
        var stretch = t.Stretch;
        double time = 0.37 * stretch.Duration;

        double tStar = stretch.Subpath.TimeAtLength(1.7 * time);
        var (_, d) = ArcLengthDerivatives.Compute(curve, tStar);
        State3D state = stretch.EvaluateState(time);

        for (int k = 1; k <= 4; k++)
            AssertPoint(d[k - 1] * Math.Pow(1.7, k), state.GetDerivative(k), 1e-9);

        // Centripetal acceleration: perpendicular to the velocity, magnitude v² κ.
        Assert.Equal(0.0, state.Velocity.Dot(state.Acceleration), 1e-9);
    }

    [Fact]
    public void TheTimeDerivativesOfTheStretch_AreConsistentByFiniteDifferences()
    {
        var stretch = ConstantSpeedTrajectory3D.FromParameterFractions(SpaceCurve9(), 0.2, 0.85, 1.7, S(-1, -1, 0), S(4, 3, 2)).Stretch;
        double time = 0.4 * stretch.Duration;
        const double h = 1e-5;

        for (int k = 0; k < 4; k++)
        {
            Point3D numeric = (stretch.EvaluateDerivative(time + h, k) - stretch.EvaluateDerivative(time - h, k)) / (2 * h);
            AssertPoint(stretch.EvaluateDerivative(time, k + 1), numeric, 2e-5 * (1 + numeric.Length));
        }
    }

    // ---- junctions ---------------------------------------------------------------------------------

    [Theory]
    [InlineData(5, 2)]
    [InlineData(7, 3)]
    [InlineData(9, 4)]
    public void TheJunctions_AreContinuousUpToTheOrderOfTheDegree(int degree, int maxOrder)
    {
        var curve = SpaceCurve9();
        var t = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.7, S(-1, -1, 0), S(4, 3, 2), degree);

        State3D entryEnd = t.EntryTransition.EvaluateState(t.EntryTransition.Duration);
        State3D stretchStart = t.Stretch.EvaluateState(0);
        State3D stretchEnd = t.Stretch.EvaluateState(t.Stretch.Duration);
        State3D exitStart = t.ExitTransition.EvaluateState(0);

        for (int order = 0; order <= maxOrder; order++)
        {
            AssertPoint(stretchStart.GetDerivative(order), entryEnd.GetDerivative(order), 1e-9);
            AssertPoint(stretchEnd.GetDerivative(order), exitStart.GetDerivative(order), 1e-9);
        }

        // The whole trajectory is continuous in time as well.
        double junction = t.EntryTransition.Duration;
        AssertPoint(t.EvaluatePosition(junction - 1e-9), t.EvaluatePosition(junction + 1e-9), 1e-7);
        // And the curved stretch really has non-zero acceleration at the junction.
        Assert.True(stretchStart.Acceleration.Length > 1e-3);
    }

    [Fact]
    public void ADegree3Transition_MatchesOnlyThePositionAndTheVelocity()
    {
        var t = ConstantSpeedTrajectory3D.FromParameterFractions(SpaceCurve9(), 0.2, 0.85, 1.7, S(-1, -1, 0), S(4, 3, 2), 3);

        State3D entryEnd = t.EntryTransition.EvaluateState(t.EntryTransition.Duration);
        State3D stretchStart = t.Stretch.EvaluateState(0);

        AssertPoint(stretchStart.Position, entryEnd.Position, 1e-9);
        AssertPoint(stretchStart.Velocity, entryEnd.Velocity, 1e-9);
    }

    // ---- the straight case through the general code ---------------------------------------------

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AStraightSegment_ThroughTheGeneralCode_GivesTheStraightConstruction(bool withLimits)
    {
        var line = new PolynomialSegment3D(9, S(0, 0, 0), S(8, 4, 2), 4.0);
        var start = S(-1, 0, 0);
        var end = S(9, 5, 3);
        var limits = withLimits ? KinematicLimits.FromValues(vMax: 6, aMax: 20, jMax: 200) : null;

        var general = ConstantSpeedTrajectory3D.FromParameterFractions(line, 0.3, 0.7, 1.5, start, end, 9, limits);
        var straight = ConstantVelocityTrajectory3D.OnSegment(line, 0.3, 0.7, 1.5, start, end, 9, limits);

        Assert.Equal(straight.TotalDuration, general.TotalDuration, 1e-9);
        Assert.Equal(straight.ConstantVelocity.Duration, general.Stretch.Duration, 1e-9);
        Assert.Equal(straight.EntryTransition.Duration, general.EntryTransition.Duration, 1e-9);
        Assert.Equal(straight.ExitTransition.Duration, general.ExitTransition.Duration, 1e-9);

        State3D junction = general.Stretch.EvaluateState(0);
        AssertPoint(Point3D.Zero, junction.Acceleration, 1e-9);
        AssertPoint(Point3D.Zero, junction.Jerk, 1e-9);
        AssertPoint(Point3D.Zero, junction.Snap, 1e-9);

        for (double t = 0; t <= general.TotalDuration; t += general.TotalDuration / 40)
            for (int order = 0; order <= 4; order++)
                AssertPoint(straight.EvaluateState(t).GetDerivative(order), general.EvaluateState(t).GetDerivative(order), 1e-8);
    }

    // ---- the three specifications -------------------------------------------------------------------

    [Fact]
    public void ParametersPointsAndLengthFractions_GiveTheSameStretch()
    {
        var curve = SpaceCurve9();
        var start = S(-1, -1, 0);
        var end = S(4, 3, 2);
        var byTau = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.25, 0.8, 1.2, start, end);

        var byPoints = ConstantSpeedTrajectory3D.FromPoints(
            curve, curve.EvaluatePosition(0.25 * curve.Duration), curve.EvaluatePosition(0.8 * curve.Duration), 1.2, start, end);

        var whole = new ArcLengthPath(curve, 0, curve.Duration, requireRegular: false);
        var byLength = ConstantSpeedTrajectory3D.FromArcLengthFractions(
            curve,
            whole.LengthAt(0.25 * curve.Duration) / whole.Length,
            whole.LengthAt(0.8 * curve.Duration) / whole.Length, 1.2, start, end);

        foreach (var other in new[] { byPoints, byLength })
        {
            Assert.Equal(byTau.TotalDuration, other.TotalDuration, 1e-8);
            Assert.Equal(byTau.Stretch.Duration, other.Stretch.Duration, 1e-8);
            for (int order = 0; order <= 4; order++)
                AssertPoint(byTau.Stretch.EvaluateState(0).GetDerivative(order), other.Stretch.EvaluateState(0).GetDerivative(order), 1e-7);
        }

        Assert.Throws<ArgumentException>(() => ConstantSpeedTrajectory3D.FromPoints(
            curve, curve.EvaluatePosition(0.25 * curve.Duration) + new Point3D(0, 0, 0.01), curve.EvaluatePosition(0.8 * curve.Duration), 1.2, start, end));
        Assert.Throws<ArgumentException>(() => ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.8, 0.25, 1.2, start, end));
        Assert.Throws<ArgumentException>(() => ConstantSpeedTrajectory3D.FromArcLengthFractions(curve, -0.1, 0.5, 1.2, start, end));
    }

    // ---- limits ----------------------------------------------------------------------------------------

    [Fact]
    public void ASpeedAboveTheAdmissibleSpeed_FailsExplicitly_NamingVAdmAndTheBindingLimit()
    {
        var parabola = Parabola();
        var limits = KinematicLimits.FromValues(aMax: 8, jMax: 20);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(() =>
            ConstantSpeedTrajectory3D.FromParameterFractions(parabola, 0.0, 1.0, 1.8, S(-1, 0, 0), S(2, 1, 0), 9, limits));

        Assert.Contains("1.70998", ex.Message);
        Assert.Contains("jerk", ex.Message);

        // Just below v_adm the stretch itself is accepted; the transitions may then be impossible for such tight limits
        // (the boundary jerk is almost the limit), which is reported by the duration search, not by the stretch.
        var stretch = new CurvedConstantSpeedStretch3D(new ArcLengthPath(parabola, 0, 1), 1.7);
        Assert.True(1.7 < stretch.AdmissibleSpeed(limits).Speed);
        Assert.Equal(1.7, stretch.EvaluateDerivative(0.3, 1).Length, 1e-9);
    }

    [Fact]
    public void ASubpathThroughARestPoint_FailsExplicitly()
    {
        var restToRest = new PolynomialSegment3D(9, S(0, 0, 0), S(2, 1, 0), 1.0);

        Assert.Throws<PathNotRegularException>(() =>
            ConstantSpeedTrajectory3D.FromParameterFractions(restToRest, 0.0, 0.5, 1.0, S(-1, 0, 0), S(3, 1, 0)));
    }

    [Fact]
    public void Rule1_TheTransitionsTakeTheirShortestFeasibleDurations_AndEveryPieceRespectsTheLimits()
    {
        var curve = SpaceCurve9();
        var start = S(-1, -1, 0);
        var end = S(4, 3, 2);
        var limits = KinematicLimits.FromValues(vMax: 6, aMax: 30, jMax: 400, sMax: 8000);

        var t = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.0, start, end, 9, limits);

        double expectedEntry = ShortestFeasibleDuration.Find(
            9, BlendBuilder3D.Components(start), BlendBuilder3D.Components(t.Stretch.StartState), limits)!.Value;
        Assert.Equal(expectedEntry, t.EntryTransition.Duration, 1e-9);
        Assert.True(ExtremaAnalyzer.VerifyLimits(t.EntryTransition, limits));
        Assert.True(ExtremaAnalyzer.VerifyLimits(t.ExitTransition, limits));

        // The stretch itself, sampled, respects the per-axis limits (v_cv below v_adm).
        var samples = t.Sample(400, 4);
        double[] caps = { 6, 30, 400, 8000 };
        for (int order = 1; order <= 4; order++)
            foreach (var value in samples.Values[order].Skip(0))
                Assert.True(Math.Max(Math.Abs(value.X), Math.Max(Math.Abs(value.Y), Math.Abs(value.Z))) <= caps[order - 1] * (1 + 1e-6));
    }

    [Fact]
    public void Rule3_APrescribedTotal_IsSplitInProportionToTheShortestFeasibleDurations()
    {
        var curve = SpaceCurve9();
        var start = S(-1, -1, 0);
        var end = S(4, 3, 2);
        var limits = KinematicLimits.FromValues(vMax: 6, aMax: 30, jMax: 400);
        var free = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.0, start, end, 9, limits);

        var roomy = ConstantSpeedTrajectory3D.FromParameterFractions(
            curve, 0.2, 0.85, 1.0, start, end, 9, limits, totalDuration: free.TotalDuration + 2);

        Assert.Equal(free.TotalDuration + 2, roomy.TotalDuration, 1e-9);
        Assert.Equal(free.EntryTransition.Duration / free.ExitTransition.Duration,
            roomy.EntryTransition.Duration / roomy.ExitTransition.Duration, 1e-9);
        Assert.Throws<KinematicLimitsViolatedException>(() => ConstantSpeedTrajectory3D.FromParameterFractions(
            curve, 0.2, 0.85, 1.0, start, end, 9, limits, totalDuration: free.TotalDuration - 0.2));
    }

    [Fact]
    public void Rule2_WithoutLimits_UsesTheEuclideanDistanceAndTheSpeedNorms()
    {
        var curve = SpaceCurve9();
        var start = S(-1, -1, 0, 0, 2, 0);
        var end = S(4, 3, 2);

        var t = ConstantSpeedTrajectory3D.FromParameterFractions(curve, 0.2, 0.85, 1.7, start, end);

        double expectedEntry = 2.0 * (t.Stretch.StartState.Position - start.Position).Length / (2.0 + 1.7);
        double expectedExit = 2.0 * (end.Position - t.Stretch.EndState.Position).Length / (0.0 + 1.7);
        Assert.Equal(expectedEntry, t.EntryTransition.Duration, 1e-12);
        Assert.Equal(expectedExit, t.ExitTransition.Duration, 1e-12);
    }

    // ---- sampling --------------------------------------------------------------------------------------------

    [Fact]
    public void Sampling_IsUniformIncludesTheEnds_AndReachesTheEndState()
    {
        var end = S(4, 3, 2);
        var t = ConstantSpeedTrajectory3D.FromParameterFractions(SpaceCurve9(), 0.2, 0.85, 1.7, S(-1, -1, 0), end);

        var samples = t.Sample(301, 4);

        Assert.Equal(0.0, samples.Times[0]);
        Assert.Equal(t.TotalDuration, samples.Times[^1], 1e-12);
        Assert.Equal(5, samples.Values.Length);
        Assert.Equal(301, samples.Values[2].Length);
        AssertPoint(end.Position, samples.Values[0][^1], 1e-9);
        AssertPoint(Point3D.Zero, samples.Values[1][0] - S(-1, -1, 0).Velocity, 1e-9);
        Assert.Throws<ArgumentOutOfRangeException>(() => t.Sample(1, 2));
    }
}
