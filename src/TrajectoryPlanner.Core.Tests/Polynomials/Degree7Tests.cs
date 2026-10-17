using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Polynomials;

public class Degree7Tests
{
    [Fact]
    public void Degree7_Solve_EightConditionsSatisfied()
    {
        double T = 2.5;
        var coeffs = PolynomialCoefficientSolver.SolveDegree7(1, 9, -2, 3, 4, -5, 6, -7, T);

        var start = PolynomialEvaluator.EvaluateState1D(0.0, coeffs, T);
        var end = PolynomialEvaluator.EvaluateState1D(1.0, coeffs, T);

        Assert.Equal(1, start.Position, 1e-8);
        Assert.Equal(-2, start.Velocity, 1e-8);
        Assert.Equal(4, start.Acceleration, 1e-8);
        Assert.Equal(6, start.Jerk, 1e-7);

        Assert.Equal(9, end.Position, 1e-8);
        Assert.Equal(3, end.Velocity, 1e-8);
        Assert.Equal(-5, end.Acceleration, 1e-8);
        Assert.Equal(-7, end.Jerk, 1e-7);
    }

    [Fact]
    public void Degree7_RestToRest_IsSymmetricAndHasZeroDerivativesAtBothEnds()
    {
        double T = 2.0, distance = 10;
        var seg = new PolynomialSegment1D(7, State1D.FromPositionOnly(0), State1D.FromPositionOnly(distance), T);

        Assert.Equal(distance / 2, seg.EvaluatePosition(T / 2), 1e-9);
        foreach (int order in new[] { 1, 2, 3 })
        {
            Assert.Equal(0, seg.EvaluateDerivative(0, order), 1e-8);
            Assert.Equal(0, seg.EvaluateDerivative(T, order), 1e-8);
        }

        // Odd symmetry about the midpoint: x(T/2 + s) + x(T/2 - s) = distance.
        Assert.Equal(distance, seg.EvaluatePosition(0.3 * T) + seg.EvaluatePosition(0.7 * T), 1e-9);
    }

    [Fact]
    public void Degree7_Segment_IgnoresSnapInTheStates()
    {
        var withSnap = new PolynomialSegment1D(7, new State1D(0, 0, 0, 0, 99), new State1D(1, 0, 0, 0, -99), 1.0);
        var withoutSnap = new PolynomialSegment1D(7, State1D.FromPositionOnly(0), State1D.FromPositionOnly(1), 1.0);

        for (int i = 0; i <= 7; i++)
            Assert.Equal(withoutSnap.Coefficients[i], withSnap.Coefficients[i], 12);
    }
}
