using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.ConstantVelocity;

/// <summary>Request 0015, Part B: 3D constant velocity on a straight stretch (rules 1-3, per-axis limits, straight-subpath detection).</summary>
public class ConstantVelocityTrajectory3DTests
{
    private static State3D S(double x, double y, double z, double vx = 0, double vy = 0, double vz = 0) =>
        new(new Point3D(x, y, z), new Point3D(vx, vy, vz), Point3D.Zero, Point3D.Zero, Point3D.Zero);

    private static void AssertSegmentsEqual1D(PolynomialSegment1D expected, PolynomialSegment1D actual, double tolerance = 1e-9)
    {
        Assert.Equal(expected.Duration, actual.Duration, tolerance);
        for (int i = 0; i < expected.Coefficients.NormalizedCoefficients.Length; i++)
            Assert.Equal(expected.Coefficients.NormalizedCoefficients[i], actual.Coefficients.NormalizedCoefficients[i], tolerance);
    }

    private static bool Respects(ConstantVelocityTrajectory3D t, KinematicLimits limits) =>
        t.GetAllSegments().All(s => ExtremaAnalyzer.VerifyLimits(s, limits));

    // ---- axis-aligned = 1D -------------------------------------------------------------------

    [Theory]
    [InlineData(false, false)] // Rule 2
    [InlineData(true, false)]  // Rule 1
    [InlineData(false, true)]  // Rule 3 without limits
    [InlineData(true, true)]   // Rule 3 with limits
    public void AlongAnAxis_EqualsTheOneDimensionalConstruction(bool withLimits, bool withTotal)
    {
        var limits = withLimits ? KinematicLimits.FromValues(vMax: 20, aMax: 40, jMax: 400) : null;
        double? total = withTotal ? 12.0 : null;

        var oneD = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 10, 60, 15, 9, limits, totalDuration: total);
        var threeD = new ConstantVelocityTrajectory3D(
            S(0, 1, 2), S(100, 1, 2), new Point3D(10, 1, 2), new Point3D(60, 1, 2), 15, 9, limits, totalDuration: total);

        Assert.Equal(oneD.TotalDuration, threeD.TotalDuration, 1e-9);
        AssertSegmentsEqual1D(oneD.EntryTransition, threeD.EntryTransition.XAxis);
        AssertSegmentsEqual1D(oneD.ExitTransition, threeD.ExitTransition.XAxis);
        Assert.Equal(oneD.ConstantVelocity.Duration, threeD.ConstantVelocity.Duration, 1e-12);
        // The quiet axes do not move.
        Assert.All(threeD.GetAllSegments(), s =>
        {
            Assert.Equal(1.0, s.EvaluatePosition(s.Duration / 2).Y, 12);
            Assert.Equal(2.0, s.EvaluatePosition(s.Duration / 3).Z, 12);
        });
    }

    [Fact]
    public void AlongANegativeAxis_EqualsTheOneDimensionalConstruction()
    {
        var oneD = new ConstantVelocityTrajectory1D(100, 0, 0, 0, 60, 10, 15, 9);
        var threeD = new ConstantVelocityTrajectory3D(S(0, 100, 0), S(0, 0, 0), new Point3D(0, 60, 0), new Point3D(0, 10, 0), 15);

        AssertSegmentsEqual1D(oneD.EntryTransition, threeD.EntryTransition.YAxis);
        AssertSegmentsEqual1D(oneD.ExitTransition, threeD.ExitTransition.YAxis);
    }

    // ---- diagonal ------------------------------------------------------------------------------

    [Fact]
    public void AlongADiagonal_TheStretchHasExactlyTheSpeed_AndTheTransitionsAreContinuous()
    {
        var start = S(0, 0, 0);
        var end = S(12, 16, 0);
        var cvStart = new Point3D(3, 4, 0);
        var cvEnd = new Point3D(9, 12, 0);

        var t = new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 2.5);

        Assert.Equal(2.5, t.ConstantVelocity.StartState.Velocity.Length, 1e-12);
        Assert.Equal(10.0 / 2.5, t.ConstantVelocity.Duration, 1e-12); // |Δp| = 10
        Assert.Equal(0.6 * 2.5, t.ConstantVelocity.StartState.Velocity.X, 1e-12);
        Assert.Equal(0.8 * 2.5, t.ConstantVelocity.StartState.Velocity.Y, 1e-12);
        Assert.Equal(1.0, t.Direction.Length, 1e-12);

        // Speed along the stretch, sampled.
        var cv = t.ConstantVelocity.ToPolynomialSegments().Single();
        for (double s = 0; s <= cv.Duration; s += cv.Duration / 7)
            Assert.Equal(2.5, cv.EvaluateDerivative(s, 1).Length, 1e-9);

        // Continuity at both junctions up to the snap of the degree.
        for (int order = 0; order <= 4; order++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                Assert.Equal(
                    t.EntryTransition.EvaluateState(t.EntryTransition.Duration).GetDerivative(order)[axis],
                    cv.EvaluateState(0).GetDerivative(order)[axis], 1e-9);
                Assert.Equal(
                    cv.EvaluateState(cv.Duration).GetDerivative(order)[axis],
                    t.ExitTransition.EvaluateState(0).GetDerivative(order)[axis], 1e-9);
            }
        }

        Assert.Equal(end.Position.X, t.EvaluateState(t.TotalDuration).Position.X, 1e-9);
        Assert.Equal(end.Position.Y, t.EvaluateState(t.TotalDuration).Position.Y, 1e-9);
    }

    // ---- Rules 1-3 with Euclidean distances ---------------------------------------------------

    [Fact]
    public void Rule2_UsesTheEuclideanDistanceAndTheSpeedNorms()
    {
        // Entry covers (3, 4, 0): distance 5; the outer velocity is (0, 3, 4): norm 5; speed 5 -> T = 2*5/(5+5).
        var t = new ConstantVelocityTrajectory3D(
            S(0, 0, 0, 0, 3, 4), S(30, 40, 0), new Point3D(3, 4, 0), new Point3D(18, 24, 0), 5);

        Assert.Equal(2.0 * 5 / (5 + 5), t.EntryTransition.Duration, 1e-12);
        // Exit covers (12, 16, 0): distance 20, outer velocity 0 -> T = 2*20/(0+5).
        Assert.Equal(2.0 * 20 / 5, t.ExitTransition.Duration, 1e-12);
    }

    [Fact]
    public void Rule1_WithLimits_EachTransitionTakesItsShortestFeasibleDuration()
    {
        var limits = KinematicLimits.FromValues(vMax: 10, aMax: 8, jMax: 100);
        var start = S(0, 0, 0);
        var end = S(10, 10, 6);
        var cvStart = new Point3D(2, 2, 1);
        var cvEnd = new Point3D(8, 8, 5);

        var t = new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 4, 9, limits);

        var v = t.Direction * 4;
        var cvStartState = new State3D(cvStart, v, Point3D.Zero, Point3D.Zero, Point3D.Zero);
        double expectedEntry = ShortestFeasibleDuration.Find(
            9, Blending3D(start), Blending3D(cvStartState), limits)!.Value;
        Assert.Equal(expectedEntry, t.EntryTransition.Duration, 1e-9);
        Assert.True(Respects(t, limits));
        // The shorter entry would violate a limit.
        var shorter = new PolynomialSegment3D(9, start, cvStartState, t.EntryTransition.Duration * (1 - 1e-6));
        Assert.False(ExtremaAnalyzer.VerifyLimits(shorter, limits));
    }

    private static State1D[] Blending3D(State3D s) => TrajectoryPlanner.Core.Blending.BlendBuilder3D.Components(s);

    [Fact]
    public void Rule3_APrescribedTotal_IsSplitInProportionToTheWeights_AndIsExact()
    {
        var start = S(0, 0, 0);
        var end = S(30, 40, 0);
        var cvStart = new Point3D(3, 4, 0);
        var cvEnd = new Point3D(18, 24, 0);
        double tcv = 25.0 / 5;

        var t = new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 5, totalDuration: 30);

        Assert.Equal(30.0, t.TotalDuration, 1e-12);
        double ruleIn = 2.0 * 5 / 5, ruleOut = 2.0 * 20 / 5; // distances 5 and 20, speed 5, outer velocity 0
        Assert.Equal(ruleIn / ruleOut, t.EntryTransition.Duration / t.ExitTransition.Duration, 1e-12);
        Assert.Equal((30 - tcv) * ruleIn / (ruleIn + ruleOut), t.EntryTransition.Duration, 1e-12);
    }

    [Fact]
    public void Rule3_WithLimits_SplitsInProportionToTheShortestFeasibleDurations_OrFailsWhenTooShort()
    {
        var limits = KinematicLimits.FromValues(vMax: 8, aMax: 10);
        var start = S(0, 0, 0);
        var end = S(30, 40, 0);
        var cvStart = new Point3D(3, 4, 0);
        var cvEnd = new Point3D(18, 24, 0);
        var free = new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 5, 9, limits);

        double minimum = free.TotalDuration;
        var roomy = new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 5, 9, limits, totalDuration: minimum + 3);
        Assert.Equal(minimum + 3, roomy.TotalDuration, 1e-9);
        Assert.Equal(free.EntryTransition.Duration / free.ExitTransition.Duration,
            roomy.EntryTransition.Duration / roomy.ExitTransition.Duration, 1e-9);
        Assert.True(Respects(roomy, limits));

        Assert.Throws<KinematicLimitsViolatedException>(() =>
            new ConstantVelocityTrajectory3D(start, end, cvStart, cvEnd, 5, 9, limits, totalDuration: minimum - 0.5));
    }

    [Fact]
    public void PrescribedTransitionDurations_AreKept_AndOnlyVerified()
    {
        var limits = KinematicLimits.FromValues(vMax: 50);
        var args = (S(0, 0, 0), S(30, 40, 0), new Point3D(3, 4, 0), new Point3D(18, 24, 0));

        var t = new ConstantVelocityTrajectory3D(args.Item1, args.Item2, args.Item3, args.Item4, 5, 9, limits, entryDuration: 2.2, exitDuration: 7.7);
        Assert.Equal(2.2, t.EntryTransition.Duration, 12);
        Assert.Equal(7.7, t.ExitTransition.Duration, 12);

        Assert.Throws<KinematicLimitsViolatedException>(() => new ConstantVelocityTrajectory3D(
            args.Item1, args.Item2, args.Item3, args.Item4, 5, 9, KinematicLimits.FromValues(vMax: 6), entryDuration: 0.1));
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory3D(
            args.Item1, args.Item2, args.Item3, args.Item4, 5, 9, null, entryDuration: 1, exitDuration: 1, totalDuration: 20));
    }

    // ---- per-axis speed limit ------------------------------------------------------------------

    [Fact]
    public void ASpeedWhoseComponentExceedsThePerAxisVelocityLimit_FailsExplicitly()
    {
        var start = S(0, 0, 0);
        var end = S(10, 10, 0);

        // speed 3 along (1,1,0)/√2: components 2.121
        Assert.Throws<KinematicLimitsViolatedException>(() => new ConstantVelocityTrajectory3D(
            start, end, new Point3D(2, 2, 0), new Point3D(8, 8, 0), 3, 9, KinematicLimits.FromValues(vMax: 2)));

        var ok = new ConstantVelocityTrajectory3D(
            start, end, new Point3D(2, 2, 0), new Point3D(8, 8, 0), 3, 9, KinematicLimits.FromValues(vMax: 2.2));
        Assert.Equal(3.0, ok.ConstantVelocity.StartState.Velocity.Length, 1e-12); // speed above vMax, per-axis within it
    }

    // ---- degenerate input ------------------------------------------------------------------------

    [Fact]
    public void DegenerateInput_IsRejected()
    {
        var s = S(0, 0, 0);
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory3D(s, s, new Point3D(1, 1, 1), new Point3D(1, 1, 1), 1));
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory3D(s, s, new Point3D(1, 1, 1), new Point3D(2, 2, 2), 0));
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory3D(s, s, new Point3D(1, 1, 1), new Point3D(2, 2, 2), -1));
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory3D(
            s, S(5, 5, 5), new Point3D(1, 1, 1), new Point3D(2, 2, 2), 1, totalDuration: 0.5)); // total below T_cv
    }

    // ---- straight subpath of a given segment -----------------------------------------------------

    [Fact]
    public void OnAStraightSegment_TheStretchIsBuilt_FromParametersOrPoints()
    {
        var line = new PolynomialSegment3D(9, S(0, 0, 0), S(8, 4, 2), 4.0); // rest to rest along a line
        var start = S(0, 0, 0);
        var end = S(8, 4, 2);

        var byTau = ConstantVelocityTrajectory3D.OnSegment(line, 0.3, 0.7, 1.5, start, end);
        var byPoints = ConstantVelocityTrajectory3D.OnSegment(
            line, line.EvaluatePosition(0.3 * 4), line.EvaluatePosition(0.7 * 4), 1.5, start, end);

        Assert.Equal(1.5, byTau.ConstantVelocity.StartState.Velocity.Length, 1e-12);
        Assert.Equal(byTau.TotalDuration, byPoints.TotalDuration, 1e-9);
        Assert.True(StraightSubpath.IsStraight(line, 0.0, 1.0, StraightSubpath.DefaultRelativeTolerance, out _));
    }

    [Fact]
    public void OnACurvedSegment_NotSupportedExceptionSaysItIsNotImplementedYet()
    {
        // The velocities are not parallel to the chord: the path is a curve.
        var curved = new PolynomialSegment3D(9, S(0, 0, 0, 0, 3, 0), S(8, 4, 2, 0, 0, 3), 4.0);

        var ex = Assert.Throws<NotSupportedException>(() =>
            ConstantVelocityTrajectory3D.OnSegment(curved, 0.2, 0.8, 1.5, S(0, 0, 0), S(8, 4, 2)));

        Assert.Contains("not implemented yet", ex.Message);
        Assert.Contains("curved", ex.Message);
        Assert.Throws<NotSupportedException>(() => ConstantVelocityTrajectory3D.OnSegment(
            curved, curved.EvaluatePosition(0.8), curved.EvaluatePosition(3.2), 1.5, S(0, 0, 0), S(8, 4, 2)));
    }

    [Fact]
    public void AStraightPathThatDoublesBack_IsNotAStraightStretch()
    {
        // Along x only, but the path overshoots the end and comes back: the chord is not what is travelled.
        var overshoot = new PolynomialSegment3D(9, S(0, 0, 0, 6, 0, 0), S(2, 0, 0, -6, 0, 0), 2.0);

        Assert.False(StraightSubpath.IsStraight(overshoot, 0.05, 0.95, StraightSubpath.DefaultRelativeTolerance, out string reason));
        Assert.NotEmpty(reason);
        Assert.Throws<NotSupportedException>(() => StraightSubpath.Require(overshoot, 0.05, 0.95));
    }

    [Fact]
    public void ThePointsConstructor_NeedsNoUnderlyingSegment()
    {
        var t = new ConstantVelocityTrajectory3D(S(0, 0, 0), S(5, 5, 5), new Point3D(1, 1, 1), new Point3D(4, 4, 4), 2);

        Assert.True(t.TotalDuration > 0);
        Assert.Equal(3 * Math.Sqrt(3) / 2, t.ConstantVelocity.Duration, 1e-12);
    }
}
