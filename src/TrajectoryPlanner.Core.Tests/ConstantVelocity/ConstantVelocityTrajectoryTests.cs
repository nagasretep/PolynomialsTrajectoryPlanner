using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.ConstantVelocity;

/// <summary>Request 0007: constant-velocity trajectory (theory/ConstantVelocitySegment.md, transition durations).</summary>
public class ConstantVelocityTrajectoryTests
{
    private static void AssertRelative(double expected, double actual, double relativeTolerance, string what)
    {
        Assert.True(
            Math.Abs(expected - actual) <= relativeTolerance * Math.Max(Math.Abs(expected), 1e-300),
            $"{what}: expected {expected:R}, got {actual:R}");
    }

    /// <summary>Derivatives of order 0 .. p-1 are shared by the two pieces at a junction; they must match.</summary>
    private static void AssertJunction(int degree, State1D leftEnd, State1D rightStart, string what)
    {
        int p = (degree + 1) / 2;
        for (int order = 0; order < p; order++)
        {
            double a = leftEnd.GetDerivative(order);
            double b = rightStart.GetDerivative(order);
            Assert.True(Math.Abs(a - b) <= 1e-9 * Math.Max(1.0, Math.Abs(a)), $"{what}, order {order}: {a:R} vs {b:R}");
        }
    }

    private static void AssertContinuity(ConstantVelocityTrajectory1D t, int degree)
    {
        AssertJunction(degree, t.EntryTransition.EvaluateState(t.EntryTransition.Duration), t.ConstantVelocity.StartState, "entry -> constant velocity");
        AssertJunction(degree, t.ConstantVelocity.EndState, t.ExitTransition.EvaluateState(0), "constant velocity -> exit");

        // The constant-velocity states themselves have zero acceleration, jerk and snap.
        Assert.Equal(0.0, t.ConstantVelocity.StartState.Acceleration);
        Assert.Equal(0.0, t.ConstantVelocity.EndState.Snap);
    }

    private static bool Respects(ConstantVelocityTrajectory1D t, KinematicLimits limits) =>
        t.GetAllSegments().All(s => ExtremaAnalyzer.VerifyLimits(s, limits));

    // ---- rest-to-rest ends -----------------------------------------------------------------

    [Theory]
    [InlineData(3, false)]
    [InlineData(5, false)]
    [InlineData(7, false)]
    [InlineData(9, false)]
    [InlineData(3, true)]
    [InlineData(5, true)]
    [InlineData(7, true)]
    [InlineData(9, true)]
    public void RestToRest_ContinuousAtBothJunctions_AndTotalDurationIsTheSum(int degree, bool negativeDirection)
    {
        double sign = negativeDirection ? -1 : 1;
        double x0 = negativeDirection ? 100 : 0, xf = negativeDirection ? 0 : 100;
        double cvStart = 50 - sign * 30, cvEnd = 50 + sign * 30;

        var t = new ConstantVelocityTrajectory1D(x0, xf, 0, 0, cvStart, cvEnd, 15.0, degree);

        AssertContinuity(t, degree);
        Assert.Equal(60.0 / 15.0, t.ConstantVelocity.Duration, 12);
        Assert.Equal(t.EntryTransition.Duration + t.ConstantVelocity.Duration + t.ExitTransition.Duration, t.TotalDuration, 12);

        // Velocity in the middle of the stretch is the signed constant velocity.
        double mid = t.EntryTransition.Duration + t.ConstantVelocity.Duration / 2;
        Assert.Equal(sign * 15.0, t.EvaluateState(mid).Velocity, 1e-9);
        Assert.Equal(0.0, t.EvaluateState(mid).Acceleration, 1e-9);

        // Whole-trajectory ends.
        Assert.Equal(x0, t.EvaluatePosition(0), 1e-8);
        Assert.Equal(xf, t.EvaluatePosition(t.TotalDuration), 1e-6);
    }

    [Fact]
    public void ConstantSpeedGivenAsNegativeNumber_IsTakenAsMagnitude()
    {
        var positive = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0);
        var negative = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, -15.0);

        Assert.Equal(positive.ConstantVelocity.StartState.Velocity, negative.ConstantVelocity.StartState.Velocity, 12);
        Assert.Equal(15.0, negative.ConstantSpeed, 12);
    }

    // ---- non-zero boundary velocities -----------------------------------------------------

    [Theory]
    [InlineData(5, -4.0, 3.0)] // starts moving away from the stretch, ends moving forward
    [InlineData(7, 6.0, -2.0)] // ends moving against the direction of travel
    [InlineData(9, -4.0, -2.0)]
    public void NonZeroBoundaryVelocities_IncludingOppositeToTheStretch(int degree, double v0, double vf)
    {
        var t = new ConstantVelocityTrajectory1D(0, 100, v0, vf, 20, 80, 15.0, degree);

        AssertContinuity(t, degree);
        Assert.Equal(v0, t.EvaluateState(0).Velocity, 1e-8);
        Assert.Equal(vf, t.EvaluateState(t.TotalDuration).Velocity, 1e-6);
    }

    [Fact]
    public void FullBoundaryStates_AreHonoured()
    {
        var start = new State1D(0, 1, 0.5, 0.2, 0.1);
        var end = new State1D(100, -1, -0.5, 0, 0);

        var t = new ConstantVelocityTrajectory1D(start, end, 20, 80, 15.0, 9);

        var s = t.EvaluateState(0);
        Assert.Equal(0.5, s.Acceleration, 1e-8);
        Assert.Equal(0.2, s.Jerk, 1e-7);
        AssertContinuity(t, 9);
    }

    // ---- limits: each one binding in turn --------------------------------------------------

    [Theory]
    [InlineData(9, "v")]
    [InlineData(9, "a")]
    [InlineData(9, "j")]
    [InlineData(9, "s")]
    [InlineData(7, "a")]
    [InlineData(5, "a")]
    [InlineData(5, "v")]
    public void WithLimits_TransitionsTakeTheirShortestFeasibleDuration_AndTheWholeTrajectoryRespectsTheLimits(int degree, string kind)
    {
        var limits = kind switch
        {
            "v" => KinematicLimits.FromValues(vMax: 16.0),
            "a" => KinematicLimits.FromValues(aMax: 6.0),
            "j" => KinematicLimits.FromValues(jMax: 12.0),
            _ => KinematicLimits.FromValues(sMax: 25.0)
        };

        double v0 = 2.0, vf = -1.0;
        var t = new ConstantVelocityTrajectory1D(0, 100, v0, vf, 20, 80, 15.0, degree, limits);

        var start = new State1D(0, v0, 0, 0, 0);
        var end = new State1D(100, vf, 0, 0, 0);
        double expectedEntry = ShortestFeasibleDuration.Find(degree, start, t.ConstantVelocity.StartState, limits)!.Value;
        double expectedExit = ShortestFeasibleDuration.Find(degree, t.ConstantVelocity.EndState, end, limits)!.Value;

        AssertRelative(expectedEntry, t.EntryTransition.Duration, 1e-9, "entry duration");
        AssertRelative(expectedExit, t.ExitTransition.Duration, 1e-9, "exit duration");
        Assert.True(Respects(t, limits));
        AssertContinuity(t, degree);

        // Binding: the transitions use the limit up to the search tolerance (F = 0 at the shortest duration).
        double fEntry = ShortestFeasibleDuration.Feasibility(degree, start, t.ConstantVelocity.StartState, limits, t.EntryTransition.Duration);
        Assert.InRange(fEntry, -1e-6, 0.0);
    }

    [Fact]
    public void EntryAndExit_AreSizedIndependently_SwappingTheOuterStatesChangesTheDurations()
    {
        var limits = KinematicLimits.FromValues(aMax: 4.0);
        // Each transition depends only on its own two boundary states: swapping the outer velocities changes the entry duration.
        var a = new ConstantVelocityTrajectory1D(0, 100, 0, 5, 20, 80, 15.0, 9, limits);
        var b = new ConstantVelocityTrajectory1D(0, 100, 5, 0, 20, 80, 15.0, 9, limits);

        Assert.NotEqual(a.EntryTransition.Duration, b.EntryTransition.Duration);
        Assert.True(Respects(a, limits));
        Assert.True(Respects(b, limits));
    }

    [Fact]
    public void Limits_CanBeCombined_AllRespected()
    {
        var limits = KinematicLimits.FromValues(vMax: 18, aMax: 5, jMax: 15, sMax: 60);

        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits);

        Assert.True(Respects(t, limits));
    }

    [Fact]
    public void LimitsWithNothingActive_BehaveLikeNoLimits()
    {
        var plain = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0);
        var none = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, KinematicLimits.None);

        Assert.Equal(plain.EntryTransition.Duration, none.EntryTransition.Duration, 12);
        Assert.Equal(plain.ExitTransition.Duration, none.ExitTransition.Duration, 12);
    }

    // ---- explicit failures ----------------------------------------------------------------

    [Fact]
    public void ConstantSpeedAboveTheVelocityLimit_RaisesAnExplicitException()
    {
        var limits = KinematicLimits.FromValues(vMax: 10.0);

        var ex = Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits));

        Assert.Contains("constant speed", ex.Message);
    }

    [Fact]
    public void ConstantSpeedAboveTheVelocityLimit_IsReportedEvenWithPrescribedTransitionDurations()
    {
        var limits = KinematicLimits.FromValues(vMax: 10.0);

        Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits, entryDuration: 5, exitDuration: 5));
    }

    [Fact]
    public void ImpossibleLimitsForATransition_ReportTheSearchFailure()
    {
        // The trajectory starts at speed 50, above the velocity limit 20: no entry duration can satisfy it.
        var limits = KinematicLimits.FromValues(vMax: 20.0);

        var ex = Assert.Throws<DurationSearchFailedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 50, 0, 20, 80, 15.0, 9, limits));

        Assert.Contains("entry transition", ex.Message);
    }

    [Fact]
    public void ImpossibleLimitsForTheExitTransition_AreAttributedToTheExit()
    {
        var limits = KinematicLimits.FromValues(vMax: 20.0);

        var ex = Assert.Throws<DurationSearchFailedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, -50, 20, 80, 15.0, 9, limits));

        Assert.Contains("exit transition", ex.Message);
    }

    // ---- prescribed durations --------------------------------------------------------------

    [Fact]
    public void PrescribedDurations_AreKeptAsGiven_WithoutLimits()
    {
        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, null, entryDuration: 3.0, exitDuration: 4.5);

        Assert.Equal(3.0, t.EntryTransition.Duration, 12);
        Assert.Equal(4.5, t.ExitTransition.Duration, 12);
        Assert.Equal(3.0 + 4.0 + 4.5, t.TotalDuration, 12);
    }

    [Fact]
    public void PrescribedDurations_AreKeptAsGiven_WhenTheyRespectTheLimits()
    {
        var limits = KinematicLimits.FromValues(vMax: 40, aMax: 50);

        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits, entryDuration: 3.0, exitDuration: 4.5);

        Assert.Equal(3.0, t.EntryTransition.Duration, 12);
        Assert.Equal(4.5, t.ExitTransition.Duration, 12);
    }

    [Fact]
    public void PrescribedEntryDurationViolatingALimit_IsReportedNotRepaired()
    {
        var limits = KinematicLimits.FromValues(aMax: 2.0);

        // 0.5 s to reach 15 units/s needs far more than 2 units/s^2.
        var ex = Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits, entryDuration: 0.5));

        Assert.Contains("entry transition", ex.Message);
    }

    [Fact]
    public void OnePrescribedAndOneComputedDuration()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);

        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, limits, exitDuration: 20.0);

        Assert.Equal(20.0, t.ExitTransition.Duration, 12);
        double expected = ShortestFeasibleDuration.Find(9, new State1D(0, 0, 0, 0, 0), t.ConstantVelocity.StartState, limits)!.Value;
        AssertRelative(expected, t.EntryTransition.Duration, 1e-9, "computed entry");
    }

    [Theory]
    [InlineData(0.0, 1.0)]
    [InlineData(1.0, -1.0)]
    public void NonPositivePrescribedDuration_Rejected(double entry, double exit)
    {
        Assert.Throws<ArgumentException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9, null, entry, exit));
    }

    // ---- today's rule without limits (provisional, theory marks it open) -------------------

    [Theory]
    [InlineData(20.0, 80.0, 15.0)] // T_cv = 4 -> 2 s
    [InlineData(20.0, 21.0, 15.0)] // T_cv = 1/15 -> floor 0.1 s
    [InlineData(80.0, 20.0, 5.0)] // T_cv = 12 -> 6 s, negative direction
    public void Today_WithoutLimitsEachTransitionTakesMaxOfHalfTheStretchAndTenthOfASecond(double cvStart, double cvEnd, double speed)
    {
        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, cvStart, cvEnd, speed);

        double expected = Math.Max(0.5 * Math.Abs(cvEnd - cvStart) / speed, 0.1);
        Assert.Equal(expected, t.EntryTransition.Duration, 12);
        Assert.Equal(expected, t.ExitTransition.Duration, 12);
        Assert.Equal(expected, ConstantVelocityTrajectory1D.DefaultTransitionDuration(Math.Abs(cvEnd - cvStart) / speed), 12);
    }

    // ---- degenerate input ------------------------------------------------------------------

    [Fact]
    public void EqualStretchPositions_Rejected()
    {
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 50, 50, 15.0));
    }

    [Fact]
    public void ZeroSpeed_Rejected()
    {
        Assert.Throws<ArgumentException>(() => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 0.0));
    }

    // ---- evaluation ------------------------------------------------------------------------

    [Fact]
    public void Evaluation_AtTheVeryEnd_DoesNotThrow_AndEqualsTheEndState()
    {
        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9);

        var end = t.EvaluateState(t.TotalDuration);

        Assert.Equal(100.0, end.Position, 1e-6);
        Assert.Equal(0.0, end.Velocity, 1e-6);
    }

    [Fact]
    public void GetAllSegments_ReturnsEntryStretchExit_InOrder()
    {
        var t = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15.0, 9);

        var segments = t.GetAllSegments().ToArray();

        Assert.Equal(3, segments.Length);
        Assert.Same(t.EntryTransition, segments[0]);
        Assert.Same(t.ExitTransition, segments[2]);
        Assert.Equal(3, segments[1].Degree); // the stretch is a degree-3 polynomial with equal end velocities
    }
}
