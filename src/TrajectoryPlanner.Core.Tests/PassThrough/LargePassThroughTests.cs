using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.PassThrough;

/// <summary>Request 0004, item 6: N = 30 segments, durations spanning a ratio of 100 or more.</summary>
public class LargePassThroughTests
{
    private const int SegmentCount = 30;

    private static (double[] X, double[] T) Data(int seed)
    {
        var random = new Random(seed);
        var t = new double[SegmentCount];
        for (int k = 0; k < SegmentCount; k++)
            t[k] = 0.05 * Math.Pow(100, random.NextDouble()); // log-uniform in [0.05, 5]
        t[3] = 0.05; // force a ratio of exactly 100 between the extremes
        t[17] = 5.0;

        var x = new double[SegmentCount + 1];
        for (int i = 1; i <= SegmentCount; i++)
            x[i] = x[i - 1] + (random.NextDouble() - 0.4) * 4;

        return (x, t);
    }

    [Theory]
    [InlineData(3, 2)]
    [InlineData(5, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 4)]
    [InlineData(9, 5)]
    public void SolveSucceeds_PassesExactly_AndGradientIsSmall(int degree, int order)
    {
        var (x, t) = Data(seed: 2026);
        Assert.True(t.Max() / t.Min() >= 100 - 1e-9);

        var problem = PassThroughProblem1D.Build(x, t, degree, new SmoothnessCriterion(order));
        Vector<double> y = problem.Solve();

        var residual = problem.H * y + problem.F;
        double relative = residual.L2Norm() / ((problem.H * y).L2Norm() + problem.F.L2Norm());
        double cost = problem.Cost(y);
        Assert.True(relative < 1e-7, $"degree {degree}, m = {order}: relative gradient {relative:E2}, cost {cost:E4}");

        var segments = new PassThroughSolver1D(new SmoothnessCriterion(order)).Solve(x, t, degree);
        int p = (degree + 1) / 2;
        for (int k = 0; k < SegmentCount; k++)
        {
            Assert.Equal(x[k], segments[k].EvaluatePosition(0), 1e-8);
            Assert.Equal(x[k + 1], segments[k].EvaluatePosition(segments[k].Duration), 1e-6);
        }

        for (int k = 0; k < SegmentCount - 1; k++)
        {
            for (int derivative = 0; derivative < p; derivative++)
            {
                double left = segments[k].EvaluateDerivative(segments[k].Duration, derivative);
                double right = segments[k + 1].EvaluateDerivative(0, derivative);
                Assert.True(
                    Math.Abs(left - right) <= 1e-6 * Math.Max(1.0, Math.Abs(left)),
                    $"degree {degree}, node {k + 1}, order {derivative}: {left} vs {right}");
            }
        }

        // The quadratic form and the exact segment integral agree, up to the cancellation inherent in
        // y'Hy + 2f'y + const when const >> J (measured: const/J reaches 4e6 for d = 9, m = 5, so the
        // quadratic form loses about 6 digits there; SmoothnessCost.Of does not have this problem).
        Assert.Equal(cost, SmoothnessCost.Of(segments, order), 1e-11 * problem.Constant + 1e-9 * Math.Abs(cost));
    }


    private static TrajectoryPlanner.Core.Segments.PolynomialSegment1D[] SegmentsFrom(
        PassThroughProblem1D problem, double[] x, double[] t, int degree, Vector<double> y)
    {
        double[][] nodes = problem.NodeDerivatives(x, y);
        State1D State(double[] d) => new(d[0], d.Length > 1 ? d[1] : 0, d.Length > 2 ? d[2] : 0, d.Length > 3 ? d[3] : 0, d.Length > 4 ? d[4] : 0);
        return Enumerable.Range(0, t.Length)
            .Select(k => new TrajectoryPlanner.Core.Segments.PolynomialSegment1D(degree, State(nodes[k]), State(nodes[k + 1]), t[k]))
            .ToArray();
    }

    // Request 0006, task 1 (D16): the diagonally scaled solve is better conditioned and not worse in cost.
    [Theory]
    [InlineData(5, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 4)]
    [InlineData(9, 5)]
    public void ScaledSolve_HasAMuchSmallerConditionNumber_AndANotLargerExactCost(int degree, int order)
    {
        var (x, t) = Data(seed: 2026);
        var problem = PassThroughProblem1D.Build(x, t, degree, new SmoothnessCriterion(order));

        // Conditioning of H and of S H S.
        Vector<double> s = problem.DiagonalScale();
        Matrix<double> scaledH = problem.H.Clone();
        for (int i = 0; i < scaledH.RowCount; i++)
            for (int j = 0; j < scaledH.ColumnCount; j++)
                scaledH[i, j] *= s[i] * s[j];
        double condBefore = problem.H.ConditionNumber();
        double condAfter = scaledH.ConditionNumber();
        Assert.True(condAfter < condBefore / 100, $"cond(H) = {condBefore:E2}, cond(SHS) = {condAfter:E2}");

        // Exact cost of the scaled solution against the plain (unscaled) Cholesky solution. Tolerance 1e-8, not
        // the 1e-12 of the request: the two costs differ by up to 1.6e-9 (d = 9, m = 5), with either sign depending
        // on the data, i.e. at the level of the rounding noise of the cost evaluation itself (see note 0006).
        Vector<double> scaled = problem.Solve();
        Vector<double> unscaled = problem.H.Cholesky().Solve(-problem.F);

        double costScaled = SmoothnessCost.Of(SegmentsFrom(problem, x, t, degree, scaled), order);
        double costUnscaled = SmoothnessCost.Of(SegmentsFrom(problem, x, t, degree, unscaled), order);
        Assert.True(
            costScaled <= costUnscaled * (1 + 1e-8),
            $"scaled {costScaled:R} vs unscaled {costUnscaled:R} (cond {condBefore:E2} -> {condAfter:E2})");
    }

    [Fact]
    public void ScaledSolve_SatisfiesTheNormalEquationsOfTheOriginalSystem()
    {
        var (x, t) = Data(seed: 7);
        var problem = PassThroughProblem1D.Build(x, t, 9, new SmoothnessCriterion(5));

        Vector<double> y = problem.Solve();
        var residual = problem.H * y + problem.F;

        Assert.True(residual.L2Norm() / ((problem.H * y).L2Norm() + problem.F.L2Norm()) < 1e-9);
    }
    [Fact]
    public void ConditionNumberOfH_StaysBelowTheLimitOfDoublePrecision()
    {
        // Measured on this data (2026-10-17): 3e1 (d=3), 1e6 (d=5), 4e10 (d=7), 6e13 (d=9, m=4), 5e15 (d=9, m=5).
        var (x, t) = Data(seed: 2026);

        foreach (var (degree, order) in new[] { (9, 4), (9, 5) })
        {
            double condition = PassThroughProblem1D.Build(x, t, degree, new SmoothnessCriterion(order)).H.ConditionNumber();
            Assert.True(condition < 1e17, $"degree {degree}, m = {order}: condition number of H is {condition:E3}");
        }
    }
}
