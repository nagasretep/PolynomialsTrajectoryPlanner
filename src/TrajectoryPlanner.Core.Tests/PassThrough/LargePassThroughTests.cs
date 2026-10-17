using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.PassThrough;

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
