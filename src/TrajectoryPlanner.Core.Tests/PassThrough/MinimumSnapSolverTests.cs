using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.PassThrough;

/// <summary>
/// Exact minimum-snap pass-through (theory/TrajectoryPassingThroughConstraintPoints.md,
/// "Global Minimum-Snap Formulation In 1D"). Reference values come from Cowork's NumPy prototype.
/// </summary>
public class MinimumSnapSolverTests
{
    private static readonly int[] Degrees = { 3, 5, 7, 9 };

    private static PolynomialSegment1D[] Solve(double[] x, double[] t, int degree, int? order = null)
    {
        var criterion = order is null ? (SmoothnessCriterion?)null : new SmoothnessCriterion(order.Value);
        return new PassThroughSolver1D(criterion).Solve(x, t, degree);
    }

    private static double[] Equal(int count, double h) => Enumerable.Repeat(h, count).ToArray();

    // ---- reference values ------------------------------------------------------------------

    [Theory]
    [InlineData(3, 1.5)]
    [InlineData(5, 1.875)]
    [InlineData(7, 2.1875)]
    [InlineData(9, 2.275)]
    public void ThreeWaypoints_InteriorVelocityMatchesReference(int degree, double expectedVelocity)
    {
        var segments = Solve(new[] { 0.0, 1, 2 }, Equal(2, 1.0), degree);

        var node = segments[0].EndState;
        Assert.Equal(expectedVelocity, node.Velocity, 6);
        Assert.Equal(0.0, node.Acceleration, 6); // symmetry
        if (degree == 9)
            Assert.Equal(0.0, node.Snap, 6);
    }

    [Fact]
    public void ThreeWaypoints_Degree9Snap_JerkAndCostMatchReference()
    {
        var segments = Solve(new[] { 0.0, 1, 2 }, Equal(2, 1.0), 9, order: 4);

        Assert.Equal(-15.4, segments[0].EndState.Jerk, 6);
        Assert.Equal(4032.0, SmoothnessCost.Of(segments, 4), 4032.0 * 1e-9);
    }

    [Fact]
    public void ThreeWaypoints_Degree3_CostIsSix()
    {
        var segments = Solve(new[] { 0.0, 1, 2 }, Equal(2, 1.0), 3);
        Assert.Equal(6.0, SmoothnessCost.Of(segments, 2), 6.0 * 1e-9);
    }

    [Fact]
    public void FourWaypoints_Degree9Snap_NodeStatesMatchReference()
    {
        var segments = Solve(new[] { 0.0, 1, 3, 2 }, Equal(3, 1.0), 9, order: 4);

        AssertDerivatives(segments[0].EndState, 3.038429, 3.850718, -12.978544, -51.947904);
        AssertDerivatives(segments[1].EndState, -1.142124, -5.848363, 20.390968, 86.148599);

        // The reference gives J to 7 significant digits only.
        Assert.Equal(10977.31, SmoothnessCost.Of(segments, 4), 0.01);
    }

    [Fact]
    public void FourWaypoints_UnequalDurations_CostIsFarBelowTheOldHeuristic()
    {
        var segments = Solve(new[] { 0.0, 1, 3, 2 }, new[] { 0.7, 1.3, 0.9 }, 9, order: 4);

        double cost = SmoothnessCost.Of(segments, 4);

        // The finite-difference heuristic removed by request 0002 gave 981993.99 on this data.
        Assert.InRange(cost, 1.5e4, 3.0e4);
    }

    private static void AssertDerivatives(State1D state, double v, double a, double j, double s)
    {
        Assert.Equal(v, state.Velocity, 6);
        Assert.Equal(a, state.Acceleration, 6);
        Assert.Equal(j, state.Jerk, 6);
        Assert.Equal(s, state.Snap, 6);
    }

    // ---- clamped cubic spline --------------------------------------------------------------

    [Fact]
    public void Degree3_Order2_SatisfiesClampedSplineRelation()
    {
        double[] x = { 0, 2, 1, 4, 3, 5, 4.5 };
        double h = 0.8;
        var segments = Solve(x, Equal(x.Length - 1, h), 3, order: 2);

        // v_0 = v_N = 0 (clamped); v_{i-1} + 4 v_i + v_{i+1} = 3 (x_{i+1} - x_{i-1}) / h.
        for (int i = 1; i < x.Length - 1; i++)
        {
            double vPrev = segments[i - 1].StartState.Velocity;
            double v = segments[i].StartState.Velocity;
            double vNext = segments[i].EndState.Velocity;
            Assert.Equal(3 * (x[i + 1] - x[i - 1]) / h, vPrev + 4 * v + vNext, 9);
        }
    }

    // ---- structure of the solution ---------------------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void Solution_PassesExactlyStartsAndEndsAtRest_AndIsContinuous(int degree)
    {
        double[] x = { 0, 1.5, -0.5, 2, 3.5, 3 };
        double[] t = { 0.6, 1.1, 0.9, 1.4, 0.8 };
        int p = (degree + 1) / 2;
        var segments = Solve(x, t, degree);

        for (int k = 0; k < segments.Length; k++)
        {
            Assert.Equal(x[k], segments[k].EvaluatePosition(0), 9);
            Assert.Equal(x[k + 1], segments[k].EvaluatePosition(segments[k].Duration), 9);
        }

        for (int order = 1; order < p; order++)
        {
            Assert.Equal(0.0, segments[0].EvaluateDerivative(0, order), 9);
            Assert.Equal(0.0, segments[^1].EvaluateDerivative(segments[^1].Duration, order), 9);
        }

        for (int k = 0; k < segments.Length - 1; k++)
        {
            for (int order = 0; order < p; order++)
            {
                double left = segments[k].EvaluateDerivative(segments[k].Duration, order);
                double right = segments[k + 1].EvaluateDerivative(0, order);
                Assert.Equal(left, right, 1e-7 * Math.Max(1.0, Math.Abs(left)));
            }
        }
    }

    [Fact]
    public void Degree9_Order5_IsContinuousUpToTheEighthDerivative()
    {
        double[] x = { 0, 1, 3, 2, 4 };
        double[] t = { 0.9, 1.2, 1.0, 0.8 };
        var segments = Solve(x, t, 9, order: 5);

        // m = p: the minimizer is the classical spline of degree 2p - 1, C^(2p-2) = C^8.
        // Tolerance: relative 1e-6 (the 8th derivative is of order 1e4..1e5 and H has condition ~1e7).
        for (int k = 0; k < segments.Length - 1; k++)
        {
            for (int order = 0; order <= 8; order++)
            {
                double left = segments[k].EvaluateDerivative(segments[k].Duration, order);
                double right = segments[k + 1].EvaluateDerivative(0, order);
                Assert.Equal(left, right, 1e-6 * Math.Max(1.0, Math.Abs(left)));
            }
        }
    }

    [Fact]
    public void Degree9_Snap_HasJumpsInTheFifthDerivative()
    {
        // Documents the contrast with m = 5: for m = 4 only the shared snap (order 4) is continuous.
        var segments = Solve(new[] { 0.0, 1, 3, 2 }, Equal(3, 1.0), 9, order: 4);

        double left = segments[0].EvaluateDerivative(1.0, 5);
        double right = segments[1].EvaluateDerivative(0, 5);
        Assert.True(Math.Abs(left - right) > 1.0);
    }

    // ---- optimality ------------------------------------------------------------------------

    [Theory]
    [InlineData(3, 2)]
    [InlineData(5, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 4)]
    [InlineData(9, 5)]
    public void Solution_IsTheMinimizer(int degree, int order)
    {
        double[] x = { 0, 1, 3, 2, 2.5 };
        double[] t = { 0.7, 1.3, 0.9, 1.1 };
        var problem = PassThroughProblem1D.Build(x, t, degree, new SmoothnessCriterion(order));
        Vector<double> y = problem.Solve();

        // Gradient H y + f is zero in relative terms.
        var residual = problem.H * y + problem.F;
        double relative = residual.L2Norm() / ((problem.H * y).L2Norm() + problem.F.L2Norm());
        Assert.True(relative < 1e-7, $"relative gradient {relative}");

        // Small random perturbations of the interior derivatives always increase J.
        double best = problem.Cost(y);
        var random = new Random(12345);
        for (int trial = 0; trial < 100; trial++)
        {
            var perturbation = Vector<double>.Build.Dense(y.Count, _ => (random.NextDouble() - 0.5) * 2e-2 * (1 + y.L2Norm()));
            Assert.True(problem.Cost(y + perturbation) > best);
        }

        // The quadratic form agrees with the exact segment integral.
        var segments = new PassThroughSolver1D(new SmoothnessCriterion(order)).Solve(x, t, degree);
        Assert.Equal(best, SmoothnessCost.Of(segments, order), best * 1e-8);
    }

    [Fact]
    public void Scaling_AllPositionsByLambda_ScalesAllNodeDerivatives()
    {
        double[] x = { 0, 1, 3, 2 };
        double[] t = { 0.7, 1.3, 0.9 };
        double lambda = 3.7;
        var baseSegments = Solve(x, t, 9);
        var scaled = Solve(x.Select(v => v * lambda).ToArray(), t, 9);

        for (int k = 0; k < baseSegments.Length; k++)
        {
            for (int order = 0; order <= 4; order++)
            {
                Assert.Equal(lambda * baseSegments[k].StartState.GetDerivative(order), scaled[k].StartState.GetDerivative(order), 1e-8);
                Assert.Equal(lambda * baseSegments[k].EndState.GetDerivative(order), scaled[k].EndState.GetDerivative(order), 1e-8);
            }
        }
    }

    // ---- edge cases ------------------------------------------------------------------------

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    [InlineData(7)]
    [InlineData(9)]
    public void OneSegment_IsTheRestToRestPolynomial(int degree)
    {
        var segments = Solve(new[] { 1.0, 4.0 }, new[] { 2.0 }, degree);
        var reference = new PolynomialSegment1D(degree, State1D.FromPositionOnly(1.0), State1D.FromPositionOnly(4.0), 2.0);

        Assert.Single(segments);
        for (int i = 0; i <= degree; i++)
            Assert.Equal(reference.Coefficients[i], segments[0].Coefficients[i], 12);
    }

    [Fact]
    public void RepeatedWaypoint_StillSolves()
    {
        var segments = Solve(new[] { 0.0, 2.0, 2.0, 5.0 }, new[] { 1.0, 0.5, 1.0 }, 9);
        Assert.Equal(2.0, segments[1].EvaluatePosition(0), 9);
        Assert.Equal(2.0, segments[1].EvaluatePosition(0.5), 9);
    }

    [Fact]
    public void FewerThanTwoWaypoints_Rejected()
    {
        Assert.Throws<ArgumentException>(() => Solve(new[] { 1.0 }, Array.Empty<double>(), 9));
    }

    [Fact]
    public void DurationCountMismatch_Rejected()
    {
        Assert.Throws<ArgumentException>(() => Solve(new[] { 0.0, 1, 2 }, new[] { 1.0 }, 9));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    public void NonPositiveDuration_Rejected(double bad)
    {
        Assert.Throws<ArgumentException>(() => Solve(new[] { 0.0, 1, 2 }, new[] { 1.0, bad }, 9));
    }

    [Theory]
    [InlineData(3, 3)]
    [InlineData(5, 4)]
    [InlineData(7, 5)]
    [InlineData(9, 6)]
    [InlineData(9, 0)]
    public void CriterionOrderOutsideOneToP_Rejected(int degree, int order)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Solve(new[] { 0.0, 1, 2 }, Equal(2, 1.0), degree, order));
    }

    [Fact]
    public void UnsupportedDegree_Rejected()
    {
        Assert.Throws<ArgumentException>(() => Solve(new[] { 0.0, 1, 2 }, Equal(2, 1.0), 4));
    }

    [Fact]
    public void DefaultCriterion_IsMpForLowDegreesAndSnapForDegreeNine()
    {
        Assert.Equal(2, SmoothnessCriterion.DefaultForDegree(3).DerivativeOrder);
        Assert.Equal(3, SmoothnessCriterion.DefaultForDegree(5).DerivativeOrder);
        Assert.Equal(4, SmoothnessCriterion.DefaultForDegree(7).DerivativeOrder);
        Assert.Equal(4, SmoothnessCriterion.DefaultForDegree(9).DerivativeOrder);
    }

    // ---- builders: durations and 3D --------------------------------------------------------

    [Fact]
    public void AutoDerivedProportional_SplitsByDistance()
    {
        var durations = new AutoDerivedProportionalDuration(10.0).ComputeDurations(new[] { 0.0, 1, 4, 6 }, 9);

        Assert.Equal(new[] { 10.0 / 6, 30.0 / 6, 20.0 / 6 }, durations, new DoubleComparer());
    }

    [Fact]
    public void Builder1D_UsesConfiguredDurationStrategy()
    {
        var trajectory = new TrajectoryBuilder1D()
            .WithDurationStrategy(new AutoDerivedProportionalDuration(6.0))
            .AddSegmentsFromWaypointsPassThrough(new[] { 0.0, 1, 4 })
            .Build();

        Assert.Equal(6.0, trajectory.TotalDuration, 9);
        Assert.Equal(1.5, trajectory.Segments[0].Duration, 9);
        Assert.Equal(4.5, trajectory.Segments[1].Duration, 9);
    }

    [Fact]
    public void Builder1D_ExplicitDurationsWinOverTheStrategy()
    {
        var trajectory = new TrajectoryBuilder1D()
            .WithDurationStrategy(new AutoDerivedProportionalDuration(6.0))
            .AddSegmentsFromWaypointsPassThrough(new[] { 0.0, 1, 4 }, durations: new[] { 2.0, 3.0 })
            .Build();

        Assert.Equal(5.0, trajectory.TotalDuration, 9);
    }

    [Fact]
    public void Builder3D_EachComponentEqualsThe1DSolveWithTheSameDurations()
    {
        var waypoints = new[]
        {
            new Point3D(0, 0, 0), new Point3D(1, 2, -1), new Point3D(3, 1, 0.5), new Point3D(2, 4, 2)
        };
        double[] durations = { 0.7, 1.3, 0.9 };

        var trajectory = new TrajectoryBuilder3D().AddSegmentsFromWaypointsPassThrough(waypoints, durations).Build();

        var solver = new PassThroughSolver1D();
        var expected = new[]
        {
            solver.Solve(waypoints.Select(p => p.X).ToArray(), durations, 9),
            solver.Solve(waypoints.Select(p => p.Y).ToArray(), durations, 9),
            solver.Solve(waypoints.Select(p => p.Z).ToArray(), durations, 9)
        };

        for (int k = 0; k < durations.Length; k++)
        {
            var segment = (TrajectoryPlanner.Core.Segments.PolynomialSegment3D)trajectory.Segments[k];
            var axes = new[] { segment.XAxis, segment.YAxis, segment.ZAxis };
            for (int axis = 0; axis < 3; axis++)
                for (int i = 0; i <= 9; i++)
                    Assert.Equal(expected[axis][k].Coefficients[i], axes[axis].Coefficients[i], 9);
        }
    }

    private sealed class DoubleComparer : IEqualityComparer<double>
    {
        public bool Equals(double a, double b) => Math.Abs(a - b) < 1e-12;
        public int GetHashCode(double value) => 0;
    }
}
