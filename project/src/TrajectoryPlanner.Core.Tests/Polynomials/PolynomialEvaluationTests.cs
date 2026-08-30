using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.States;
using Xunit;

namespace TrajectoryPlanner.Core.Tests.Polynomials;

public class PolynomialEvaluationTests
{
    [Fact]
    public void Horner_EqualsCanonical_ForSimplePolynomial()
    {
        double[] coeffs = { 1.0, 2.0, 3.0, 4.0 };
        double tau = 0.3;

        double canonical = PolynomialEvaluator.EvaluateCanonical(tau, coeffs);
        double horner = PolynomialEvaluator.EvaluateHorner(tau, coeffs);

        Assert.Equal(canonical, horner, 1e-12);
    }

    [Fact]
    public void EvaluateDerivativeCoefficients_CorrectForFirstDerivative()
    {
        double[] coeffs = { 1, 4, 3, 2 };
        double[] deriv = PolynomialEvaluator.EvaluateDerivativeCoefficients(coeffs, 1);

        Assert.Equal(3, deriv.Length);
        Assert.Equal(4, deriv[0], 1e-12);
        Assert.Equal(6, deriv[1], 1e-12);
        Assert.Equal(6, deriv[2], 1e-12);
    }

    [Fact]
    public void DerivativeBeyondDegree_ReturnsZero()
    {
        double[] coeffs = { 1, 2, 3 };
        double[] deriv = PolynomialEvaluator.EvaluateDerivativeCoefficients(coeffs, 5);

        Assert.Single(deriv);
        Assert.Equal(0, deriv[0], 1e-12);
    }
}

public class CoefficientSolverTests
{
    [Fact]
    public void Degree3_Solve_EndpointConditionsSatisfied()
    {
        double x0 = 0, xf = 10;
        double v0 = 0, vf = 0;
        double T = 2.0;

        var coeffs = PolynomialCoefficientSolver.SolveDegree3(x0, xf, v0, vf, T);
        var start = PolynomialEvaluator.EvaluateState1D(0.0, coeffs, T);
        var end = PolynomialEvaluator.EvaluateState1D(1.0, coeffs, T);

        Assert.Equal(x0, start.Position, 1e-9);
        Assert.Equal(v0, start.Velocity, 1e-9);
        Assert.Equal(xf, end.Position, 1e-9);
        Assert.Equal(vf, end.Velocity, 1e-9);
    }

    [Fact]
    public void Degree5_Solve_EndpointConditionsSatisfied()
    {
        double x0 = 0, xf = 5;
        double v0 = 0, vf = 1;
        double a0 = 0, af = 0;
        double T = 1.0;

        var coeffs = PolynomialCoefficientSolver.SolveDegree5(x0, xf, v0, vf, a0, af, T);
        var start = PolynomialEvaluator.EvaluateState1D(0.0, coeffs, T);
        var end = PolynomialEvaluator.EvaluateState1D(1.0, coeffs, T);

        Assert.Equal(x0, start.Position, 1e-9);
        Assert.Equal(v0, start.Velocity, 1e-9);
        Assert.Equal(a0, start.Acceleration, 1e-9);
        Assert.Equal(xf, end.Position, 1e-9);
        Assert.Equal(vf, end.Velocity, 1e-9);
        Assert.Equal(af, end.Acceleration, 1e-9);
    }

    [Fact]
    public void Degree9_Solve_AllTenConditionsSatisfied()
    {
        double x0 = 0, xf = 100;
        double v0 = 0, vf = 0;
        double a0 = 0, af = 0;
        double j0 = 0, jf = 0;
        double s0 = 0, sf = 0;
        double T = 5.0;

        var startState = new State1D(x0, v0, a0, j0, s0);
        var endState = new State1D(xf, vf, af, jf, sf);
        var coeffs = PolynomialCoefficientSolver.SolveNormalized(9, startState, endState, T);

        var startEval = PolynomialEvaluator.EvaluateState1D(0.0, coeffs, T);
        var endEval = PolynomialEvaluator.EvaluateState1D(1.0, coeffs, T);

        Assert.Equal(x0, startEval.Position, 1e-8);
        Assert.Equal(v0, startEval.Velocity, 1e-8);
        Assert.Equal(a0, startEval.Acceleration, 1e-8);
        Assert.Equal(j0, startEval.Jerk, 1e-8);
        Assert.Equal(s0, startEval.Snap, 1e-8);

        Assert.Equal(xf, endEval.Position, 1e-8);
        Assert.Equal(vf, endEval.Velocity, 1e-8);
        Assert.Equal(af, endEval.Acceleration, 1e-8);
        Assert.Equal(jf, endEval.Jerk, 1e-8);
        Assert.Equal(sf, endEval.Snap, 1e-8);
    }

    [Fact]
    public void Degree3_RestToRest_Symmetric()
    {
        double x0 = 0, xf = 10;
        double T = 2.0;
        var coeffs = PolynomialCoefficientSolver.SolveDegree3(x0, xf, 0, 0, T);

        double midPos = PolynomialEvaluator.EvaluateHorner(0.5, coeffs.NormalizedCoefficients);
        double midVel = PolynomialEvaluator.EvaluateDerivative(0.5, coeffs, 1, T);

        Assert.Equal(xf / 2.0, midPos, 1e-9);
        Assert.True(midVel > 0);
    }
}
