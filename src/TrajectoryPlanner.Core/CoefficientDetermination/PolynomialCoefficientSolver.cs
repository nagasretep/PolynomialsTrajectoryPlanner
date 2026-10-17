using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.CoefficientDetermination;

public static class PolynomialCoefficientSolver
{
    public static PolynomialCoefficients SolveNormalized(int degree, State1D startState, State1D endState, double durationT)
    {
        if (durationT <= 0)
            throw new ArgumentException("Duration must be positive", nameof(durationT));

        double[,] matrixA = NormalizedTimeMatrixFactory.BuildNormalizedMatrix(degree);
        double[] vectorB = NormalizedTimeMatrixFactory.BuildRightHandSide(degree, startState, endState, durationT);

        double[] solution = SolveLinearSystem(matrixA, vectorB);
        return new PolynomialCoefficients(degree, solution);
    }

    public static double[] SolveLinearSystem(double[,] A, double[] b)
    {
        int n = b.Length;
        if (A.GetLength(0) != n || A.GetLength(1) != n)
            throw new ArgumentException("Matrix dimensions must match vector length");

        var matrix = Matrix<double>.Build.DenseOfArray(A);
        var vector = Vector<double>.Build.Dense(b);

        var result = matrix.Solve(vector);
        return result.ToArray();
    }

    public static PolynomialCoefficients SolveFromBoundaryValues(
        int degree,
        double x0, double xf,
        double v0, double vf,
        double a0 = 0, double af = 0,
        double j0 = 0, double jf = 0,
        double s0 = 0, double sf = 0,
        double durationT = 1.0)
    {
        var startState = new State1D(x0, v0, a0, j0, s0);
        var endState = new State1D(xf, vf, af, jf, sf);
        return SolveNormalized(degree, startState, endState, durationT);
    }

    public static PolynomialCoefficients SolveDegree3(double x0, double xf, double v0, double vf, double T)
    {
        return SolveFromBoundaryValues(3, x0, xf, v0, vf, 0, 0, 0, 0, 0, 0, T);
    }

    public static PolynomialCoefficients SolveDegree5(double x0, double xf, double v0, double vf, double a0, double af, double T)
    {
        return SolveFromBoundaryValues(5, x0, xf, v0, vf, a0, af, 0, 0, 0, 0, T);
    }

    public static PolynomialCoefficients SolveDegree7(double x0, double xf, double v0, double vf, double a0, double af, double j0, double jf, double T)
    {
        return SolveFromBoundaryValues(7, x0, xf, v0, vf, a0, af, j0, jf, 0, 0, T);
    }

    public static PolynomialCoefficients SolveDegree9(double x0, double xf, double v0, double vf, double a0, double af, double j0, double jf, double s0, double sf, double T)
    {
        return SolveFromBoundaryValues(9, x0, xf, v0, vf, a0, af, j0, jf, s0, sf, T);
    }
}
