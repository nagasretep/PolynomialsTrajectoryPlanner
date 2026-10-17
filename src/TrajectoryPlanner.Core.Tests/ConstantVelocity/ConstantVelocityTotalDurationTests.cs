using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.ConstantVelocity;

/// <summary>Request 0008, Rule 3: prescribed total duration split in proportion to the transition weights.</summary>
public class ConstantVelocityTotalDurationTests
{
    // Asymmetric case: entry covers 10 units, exit 40 units; stretch 10 -> 60 at speed 15 (T_cv = 10/3).
    private const double X0 = 0, Xf = 100, CvStart = 10, CvEnd = 60, Speed = 15;
    private const double Tcv = (CvEnd - CvStart) / Speed;

    private static readonly State1D Start = State1D.FromPositionOnly(X0);
    private static readonly State1D End = State1D.FromPositionOnly(Xf);
    private static readonly State1D CvStartState = new(CvStart, Speed, 0, 0, 0);
    private static readonly State1D CvEndState = new(CvEnd, Speed, 0, 0, 0);

    private static (double In, double Out) ShortestDurations(int degree, KinematicLimits limits) =>
        (ShortestFeasibleDuration.Find(degree, Start, CvStartState, limits)!.Value,
         ShortestFeasibleDuration.Find(degree, CvEndState, End, limits)!.Value);

    private static bool Respects(ConstantVelocityTrajectory1D t, KinematicLimits limits) =>
        t.GetAllSegments().All(s => ExtremaAnalyzer.VerifyLimits(s, limits));

    // ---- without limits: weights are the Rule 2 durations ----------------------------------

    [Fact]
    public void NoLimits_TotalIsExact_AndTheRatioIsTheRatioOfTheRule2Durations()
    {
        double ruleIn = 2.0 * 10 / Speed, ruleOut = 2.0 * 40 / Speed;
        double total = 20.0;

        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, totalDuration: total);

        Assert.Equal(total, t.TotalDuration, 12);
        Assert.Equal(ruleIn / ruleOut, t.EntryTransition.Duration / t.ExitTransition.Duration, 12);
        Assert.Equal((total - Tcv) * ruleIn / (ruleIn + ruleOut), t.EntryTransition.Duration, 12);
    }

    [Fact]
    public void NoLimits_TotalEqualToTheRule2Total_ReproducesTheRule2Durations()
    {
        double ruleIn = 2.0 * 10 / Speed, ruleOut = 2.0 * 40 / Speed;

        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, totalDuration: Tcv + ruleIn + ruleOut);

        Assert.Equal(ruleIn, t.EntryTransition.Duration, 12);
        Assert.Equal(ruleOut, t.ExitTransition.Duration, 12);
    }

    // ---- with limits: weights are the shortest feasible durations --------------------------

    [Theory]
    [InlineData(9, 1.5)]
    [InlineData(9, 1.0001)]
    [InlineData(7, 3.0)]
    [InlineData(5, 2.0)]
    public void WithLimits_TotalIsExact_RatioIsTheRatioOfTheShortestDurations_AndLimitsAreRespected(int degree, double slack)
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);
        var (sIn, sOut) = ShortestDurations(degree, limits);
        double total = Tcv + slack * (sIn + sOut);

        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, degree, limits, totalDuration: total);

        Assert.Equal(total, t.TotalDuration, 12);
        Assert.Equal(sIn / sOut, t.EntryTransition.Duration / t.ExitTransition.Duration, 10);
        Assert.True(t.EntryTransition.Duration >= sIn * (1 - 1e-12));
        Assert.True(t.ExitTransition.Duration >= sOut * (1 - 1e-12));
        Assert.True(Respects(t, limits));
    }

    [Fact]
    public void WithLimits_TimeLeftExactlyEqualToTheSumOfTheMinima_IsFeasible_BothTransitionsAtTheirMinimum()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0, vMax: 20.0);
        var (sIn, sOut) = ShortestDurations(9, limits);

        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, limits, totalDuration: Tcv + sIn + sOut);

        Assert.Equal(sIn, t.EntryTransition.Duration, 9);
        Assert.Equal(sOut, t.ExitTransition.Duration, 9);
        Assert.True(Respects(t, limits));
    }

    [Fact]
    public void WithLimits_TimeLeftSlightlyBelowTheSumOfTheMinima_FailsAndNamesTheMinimumFeasibleTotal()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);
        var (sIn, sOut) = ShortestDurations(9, limits);
        double minimumTotal = Tcv + sIn + sOut;

        var ex = Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, limits, totalDuration: Tcv + (sIn + sOut) * (1 - 1e-6)));

        Assert.Contains(minimumTotal.ToString("G6", System.Globalization.CultureInfo.CurrentCulture), ex.Message);
    }

    [Fact]
    public void WithLimits_ProportionalSplitSucceedsWhereTheEqualSplitViolatesTheLimit()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);
        var (sIn, sOut) = ShortestDurations(9, limits);
        double remaining = 1.2 * (sIn + sOut);
        double total = Tcv + remaining;

        // Precondition of the scenario: the equal split gives the more demanding transition less than its minimum.
        Assert.True(remaining / 2 < Math.Max(sIn, sOut), $"sIn = {sIn}, sOut = {sOut}");

        // The old App rule (equal split, prescribed to Core) is rejected ...
        Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, limits, remaining / 2, remaining / 2));

        // ... the proportional split of the same total is feasible.
        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, limits, totalDuration: total);
        Assert.True(Respects(t, limits));
        Assert.Equal(total, t.TotalDuration, 12);
    }

    [Fact]
    public void WithLimits_SearchFailureOfATransition_StillReportsTheSearchFailure()
    {
        var limits = KinematicLimits.FromValues(vMax: 20.0);

        Assert.Throws<DurationSearchFailedException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 50, 0, CvStart, CvEnd, Speed, 9, limits, totalDuration: 100));
    }

    // Finding of request 0008: feasibility does not always improve with a longer transition. For degree 3 with
    // a_max = 6, a rest start and 20 units to cover while arriving at speed 15, the feasible set of the entry
    // duration is [2.6235, 2.7635] together with a second range from about 7.5 s on; in between the peak
    // acceleration exceeds the limit. The proportional split of Rule 3 assumes T >= s is feasible, which fails here.
    [Fact]
    public void Today_Degree3_FeasibleDurationsAreNotAnUpwardClosedSet_SoALargerTotalCanFailWhereASmallerOneWorks()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);
        var rest = State1D.FromPositionOnly(0);
        double s = ShortestFeasibleDuration.Find(3, rest, new State1D(20, 15, 0, 0, 0), limits)!.Value;

        // Window: feasible just above s, infeasible a bit further, feasible again for long durations.
        var target = new State1D(20, 15, 0, 0, 0);
        Assert.True(ShortestFeasibleDuration.Feasibility(3, rest, target, limits, 2.7) <= 0);
        Assert.True(ShortestFeasibleDuration.Feasibility(3, rest, target, limits, 4.0) > 0);
        Assert.True(ShortestFeasibleDuration.Feasibility(3, rest, target, limits, 10.0) <= 0);

        double tcv = 60.0 / 15;
        var small = new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15, 3, limits, totalDuration: tcv + 2 * s * 1.05);
        Assert.True(Respects(small, limits));

        // More time, same rule: the equal weights land in the infeasible middle range and the result is rejected.
        Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(0, 100, 0, 0, 20, 80, 15, 3, limits, totalDuration: tcv + 2 * s * 2.0));
    }

    // ---- invalid totals --------------------------------------------------------------------

    [Theory]
    [InlineData(Tcv)]
    [InlineData(Tcv / 2)]
    [InlineData(0.0)]
    [InlineData(-5.0)]
    public void TotalNotAboveTheStretchDuration_Rejected_WithTcvInTheMessage(double total)
    {
        var ex = Assert.Throws<ArgumentException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, totalDuration: total));

        Assert.Contains("T_cv", ex.Message);
    }

    // ---- combinations with prescribed transition durations ---------------------------------

    [Fact]
    public void TotalWithPrescribedEntry_ExitTakesTheRemainder()
    {
        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, entryDuration: 2.0, totalDuration: 12.0);

        Assert.Equal(2.0, t.EntryTransition.Duration, 12);
        Assert.Equal(12.0 - Tcv - 2.0, t.ExitTransition.Duration, 12);
        Assert.Equal(12.0, t.TotalDuration, 12);
    }

    [Fact]
    public void TotalWithPrescribedExit_EntryTakesTheRemainder()
    {
        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, exitDuration: 3.0, totalDuration: 12.0);

        Assert.Equal(3.0, t.ExitTransition.Duration, 12);
        Assert.Equal(12.0 - Tcv - 3.0, t.EntryTransition.Duration, 12);
    }

    [Fact]
    public void TotalWithPrescribedTransition_RemainderMustBePositive()
    {
        // R = 12 - T_cv = 8.667; a prescribed entry of 9 leaves nothing.
        Assert.Throws<ArgumentException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, entryDuration: 9.0, totalDuration: 12.0));
    }

    [Fact]
    public void TotalWithPrescribedTransition_TheRemainderIsVerifiedAgainstTheLimits()
    {
        var limits = KinematicLimits.FromValues(aMax: 6.0);
        var (sIn, sOut) = ShortestDurations(9, limits);

        // Entry generous, exit gets a remainder below its minimum.
        double entry = 2 * sIn;
        double total = Tcv + entry + 0.5 * sOut;

        Assert.Throws<KinematicLimitsViolatedException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, limits, entryDuration: entry, totalDuration: total));
    }

    [Fact]
    public void TotalWithBothTransitionDurations_IsAConflict()
    {
        Assert.Throws<ArgumentException>(
            () => new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, entryDuration: 2.0, exitDuration: 3.0, totalDuration: 12.0));
    }

    [Fact]
    public void PrescribedSingleDurationWithoutTotal_StillWorksAsBefore()
    {
        var t = new ConstantVelocityTrajectory1D(X0, Xf, 0, 0, CvStart, CvEnd, Speed, 9, null, entryDuration: 2.5);

        Assert.Equal(2.5, t.EntryTransition.Duration, 12);
        Assert.Equal(2.0 * 40 / Speed, t.ExitTransition.Duration, 12); // Rule 2 for the other one
    }

    [Fact]
    public void FullBoundaryStatesConstructor_AlsoTakesATotal()
    {
        var start = new State1D(0, 0, 0.5, 0, 0);
        var end = new State1D(100, 0, -0.5, 0, 0);

        var t = new ConstantVelocityTrajectory1D(start, end, CvStart, CvEnd, Speed, 9, null, totalDuration: 15.0);

        Assert.Equal(15.0, t.TotalDuration, 12);
    }
}
