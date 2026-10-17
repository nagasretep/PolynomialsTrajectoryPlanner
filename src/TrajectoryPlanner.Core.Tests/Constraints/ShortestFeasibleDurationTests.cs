using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Constraints;

/// <summary>Request 0005: shortest feasible duration (theory/TrajectoryConstraints.md, "Shortest Feasible Duration").</summary>
public class ShortestFeasibleDurationTests
{
    private static readonly int[] Degrees = { 3, 5, 7, 9 };

    private static double C1(int degree) => degree switch { 3 => 3.0 / 2, 5 => 15.0 / 8, 7 => 35.0 / 16, _ => 315.0 / 128 };

    private static double FindValue(int degree, State1D start, State1D end, KinematicLimits limits) =>
        ShortestFeasibleDuration.Find(degree, start, end, limits)
            ?? throw new InvalidOperationException("The limits determine no duration.");

    private static double RestToRestValue(int degree, double distance, KinematicLimits limits) =>
        ShortestFeasibleDuration.RestToRest(degree, distance, limits)
            ?? throw new InvalidOperationException("The limits determine no duration.");

    private static void AssertRelative(double expected, double actual, double relativeTolerance, string? what = null)
    {
        Assert.True(
            Math.Abs(expected - actual) <= relativeTolerance * Math.Abs(expected),
            $"{what}expected {expected:R}, got {actual:R}");
    }

    // ---- rest-to-rest: closed form and search ----------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void RestToRest_VelocityLimit_IsC1TimesDistanceOverVmax_FromBothPaths(int degree)
    {
        double distance = 4.0, vMax = 2.5;
        var start = State1D.FromPositionOnly(1.0);
        var end = State1D.FromPositionOnly(1.0 + distance);
        var limits = KinematicLimits.FromValues(vMax: vMax);
        double expected = C1(degree) * distance / vMax;

        double closedForm = RestToRestValue(degree, distance, limits);
        double search = ShortestFeasibleDuration.Search(degree, start, end, limits);
        double dispatched = FindValue(degree, start, end, limits);

        AssertRelative(expected, closedForm, 1e-6, "closed form: ");
        AssertRelative(expected, search, 1e-6, "search: ");
        AssertRelative(expected, dispatched, 1e-6, "Find: ");
    }

    [Fact]
    public void Degree9_UnitDistanceUnitVelocity_Is315Over128()
    {
        double t = FindValue(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), KinematicLimits.FromValues(vMax: 1));

        AssertRelative(315.0 / 128, t, 1e-6);
    }

    [Theory]
    [InlineData(3, 2)]
    [InlineData(5, 2)]
    [InlineData(7, 2)]
    [InlineData(9, 2)]
    [InlineData(5, 3)]
    [InlineData(7, 3)]
    [InlineData(9, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 4)]
    public void RestToRest_SingleHigherOrderLimit_ClosedFormAndSearchAgree(int degree, int order)
    {
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(3.0);
        var limits = order switch
        {
            2 => KinematicLimits.FromValues(aMax: 5.0),
            3 => KinematicLimits.FromValues(jMax: 20.0),
            _ => KinematicLimits.FromValues(sMax: 100.0)
        };

        double closedForm = RestToRestValue(degree, 3.0, limits);
        double search = ShortestFeasibleDuration.Search(degree, start, end, limits);

        AssertRelative(closedForm, search, 1e-6);
    }

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void RestToRest_AccelerationOnly_ScalesAsSqrtOfDistance(int degree)
    {
        var limits = KinematicLimits.FromValues(aMax: 3.0);

        double t1 = RestToRestValue(degree, 1.0, limits);
        double t4 = RestToRestValue(degree, 4.0, limits);

        AssertRelative(2.0, t4 / t1, 1e-9);
    }

    [Theory]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void RestToRest_JerkOnly_ScalesAsCubeRootOfDistance(int degree)
    {
        var limits = KinematicLimits.FromValues(jMax: 7.0);

        double t1 = RestToRestValue(degree, 1.0, limits);
        double t8 = RestToRestValue(degree, 8.0, limits);

        AssertRelative(2.0, t8 / t1, 1e-9);
    }

    [Fact]
    public void RestToRest_SeveralLimits_TheLargestIndividualDurationWins()
    {
        const int degree = 9;
        double distance = 5.0;
        var v = KinematicLimits.FromValues(vMax: 3.0);
        var a = KinematicLimits.FromValues(aMax: 2.0);
        var j = KinematicLimits.FromValues(jMax: 4.0);
        var all = KinematicLimits.FromValues(vMax: 3.0, aMax: 2.0, jMax: 4.0);

        double tv = RestToRestValue(degree, distance, v);
        double ta = RestToRestValue(degree, distance, a);
        double tj = RestToRestValue(degree, distance, j);

        double combined = RestToRestValue(degree, distance, all);
        Assert.Equal(Math.Max(tv, Math.Max(ta, tj)), combined, 12);

        // The search agrees.
        double search = ShortestFeasibleDuration.Search(degree, State1D.FromPositionOnly(0), State1D.FromPositionOnly(distance), all);
        AssertRelative(combined, search, 1e-6);
    }

    [Fact]
    public void RestToRest_ResultIsActuallyFeasible_AndAnyShorterDurationIsNot()
    {
        var limits = KinematicLimits.FromValues(vMax: 2.0, aMax: 3.0);
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(6);

        double t = FindValue(9, start, end, limits);

        Assert.True(ExtremaAnalyzer.VerifyLimits(new PolynomialSegment1D(9, start, end, t * (1 + 1e-7)), limits));
        Assert.False(ExtremaAnalyzer.VerifyLimits(new PolynomialSegment1D(9, start, end, t * (1 - 1e-4)), limits));
    }

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void RestToRest_ZeroDistance_IsNotDeterminedByTheLimits(int degree)
    {
        var limits = KinematicLimits.FromValues(vMax: 1, aMax: 2);

        Assert.Null(ShortestFeasibleDuration.Find(degree, State1D.FromPositionOnly(2), State1D.FromPositionOnly(2), limits));
        Assert.Null(ShortestFeasibleDuration.RestToRest(degree, 0.0, limits));
    }

    [Fact]
    public void RestToRest_VeryShortDistance_IsStillAnAnswerBelowOneMillisecond()
    {
        // No floor is applied to the closed form: T* = c1 * distance / vMax = 1.5 * 1e-6 for degree 3.
        double t = FindValue(3, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1e-6), KinematicLimits.FromValues(vMax: 1));

        AssertRelative(1.5e-6, t, 1e-9);
    }

    [Fact]
    public void ZeroLengthSegmentWithNonZeroBoundaryVelocity_IsStillDetermined()
    {
        // Not rest-to-rest: leaving and returning with a velocity needs time, so the search answers.
        var start = new State1D(0, 1.0, 0, 0, 0);
        var end = new State1D(0, -1.0, 0, 0, 0);

        double? t = ShortestFeasibleDuration.Find(5, start, end, KinematicLimits.FromValues(vMax: 2, aMax: 4));

        Assert.NotNull(t);
        Assert.True(t > 0);
    }

    [Fact]
    public void RestToRest_NegativeDirection_UsesTheAbsoluteDistance()
    {
        var limits = KinematicLimits.FromValues(vMax: 2.0);

        double up = FindValue(7, State1D.FromPositionOnly(0), State1D.FromPositionOnly(4), limits);
        double down = FindValue(7, State1D.FromPositionOnly(4), State1D.FromPositionOnly(0), limits);

        Assert.Equal(up, down, 12);
    }

    // ---- general case: non-zero boundary velocity ------------------------------------------

    [Theory]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void NonZeroBoundaryVelocity_ResultIsTheBoundaryBetweenInfeasibleAndFeasible(int degree)
    {
        var start = new State1D(0, 1.0, 0, 0, 0);
        var end = new State1D(3.0, -0.5, 0, 0, 0);
        var limits = KinematicLimits.FromValues(vMax: 2.0, aMax: 3.0);

        double t = FindValue(degree, start, end, limits);

        double f = ShortestFeasibleDuration.Feasibility(degree, start, end, limits, t);
        Assert.InRange(f, -1e-6, ShortestFeasibleDuration.FeasibilityTolerance + 1e-12);

        // Slightly below T*: infeasible.
        Assert.True(ShortestFeasibleDuration.Feasibility(degree, start, end, limits, t * (1 - 1e-4)) > 0);

        // On a fine geometric grid, no feasible duration is shorter than T*.
        for (double grid = 0.02; grid < t * (1 - 1e-6); grid *= 1.01)
        {
            Assert.True(
                ShortestFeasibleDuration.Feasibility(degree, start, end, limits, grid) > 0,
                $"T = {grid:G6} is feasible but T* = {t:G6}");
        }

        // ... and the durations above it that are feasible exist (T* itself and slightly above).
        Assert.True(ShortestFeasibleDuration.Feasibility(degree, start, end, limits, t * (1 + 1e-6)) <= ShortestFeasibleDuration.FeasibilityTolerance);
    }

    [Fact]
    public void NonZeroBoundaryVelocity_IsNotLongerThanAnyFeasibleDurationOnTheGrid()
    {
        var start = new State1D(0, 1.0, 0, 0, 0);
        var end = new State1D(3.0, -0.5, 0, 0, 0);
        var limits = KinematicLimits.FromValues(vMax: 2.0, aMax: 3.0);

        double t = FindValue(7, start, end, limits);

        for (double grid = 0.02; grid < 30; grid *= 1.02)
        {
            if (ShortestFeasibleDuration.Feasibility(7, start, end, limits, grid) <= 0)
                Assert.True(t <= grid * (1 + 1e-9), $"T* = {t:G6} exceeds the feasible T = {grid:G6}");
        }
    }

    [Fact]
    public void BoundaryVelocityAboveTheVelocityLimit_ReportsFailure()
    {
        var start = new State1D(0, 10, 0, 0, 0);
        var end = new State1D(1, 0, 0, 0, 0);

        Assert.Throws<DurationSearchFailedException>(
            () => FindValue(5, start, end, KinematicLimits.FromValues(vMax: 1.0)));
    }

    [Fact]
    public void BoundaryAccelerationAboveTheAccelerationLimit_ReportsFailure()
    {
        var start = new State1D(0, 0, 100, 0, 0);
        var end = State1D.FromPositionOnly(1);

        Assert.Throws<DurationSearchFailedException>(
            () => FindValue(5, start, end, KinematicLimits.FromValues(aMax: 1.0)));
    }

    // ---- input validation and strategy -----------------------------------------------------

    [Fact]
    public void NoActiveLimit_IsNotDetermined_BecauseNothingFixesTheDuration()
    {
        Assert.Null(ShortestFeasibleDuration.Find(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), KinematicLimits.None));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    public void NonPositiveLimit_IsRejected(double limit)
    {
        Assert.Throws<ArgumentException>(
            () => FindValue(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), KinematicLimits.FromValues(vMax: limit)));
    }

    [Fact]
    public void ConstraintAwareDuration_ReturnsTheShortestFeasibleDuration()
    {
        var limits = KinematicLimits.FromValues(vMax: 1.0);
        var strategy = new ConstraintAwareDuration(limits);

        double t = strategy.ComputeDuration(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1));

        AssertRelative(315.0 / 128, t, 1e-6);
    }

    [Fact]
    public void LooseLimit_GivesAShortDurationNotTheDefaultSecond()
    {
        // A first guess of 1 s would be feasible here, but the shortest feasible duration is much shorter:
        // closed form 315/128 * 1 / 10, with the limit binding (F = 0) and anything shorter infeasible.
        var start = State1D.FromPositionOnly(0);
        var end = State1D.FromPositionOnly(1);
        var limits = KinematicLimits.FromValues(vMax: 10.0);

        double shortest = FindValue(9, start, end, limits);

        AssertRelative(315.0 / 128 / 10.0, shortest, 1e-6);
        Assert.InRange(ShortestFeasibleDuration.Feasibility(9, start, end, limits, shortest), -1e-6, 0.0);
        Assert.True(ShortestFeasibleDuration.Feasibility(9, start, end, limits, shortest * (1 - 1e-4)) > 0);
    }
}
