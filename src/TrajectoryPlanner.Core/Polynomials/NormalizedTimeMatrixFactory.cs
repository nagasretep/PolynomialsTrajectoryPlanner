using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Polynomials;

public static class NormalizedTimeMatrixFactory
{
    public static double[,] BuildNormalizedMatrix(int degree)
    {
        return degree switch
        {
            3 => BuildDegree3Matrix(),
            5 => BuildDegree5Matrix(),
            7 => BuildDegree7Matrix(),
            9 => BuildDegree9Matrix(),
            _ => throw new ArgumentException($"Unsupported polynomial degree: {degree}")
        };
    }

    private static double[,] BuildDegree3Matrix()
    {
        return new double[,]
        {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 1, 1, 1, 1 },
            { 0, 1, 2, 3 }
        };
    }

    private static double[,] BuildDegree5Matrix()
    {
        return new double[,]
        {
            { 1, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0 },
            { 0, 0, 2, 0, 0, 0 },
            { 1, 1, 1, 1, 1, 1 },
            { 0, 1, 2, 3, 4, 5 },
            { 0, 0, 2, 6, 12, 20 }
        };
    }

    private static double[,] BuildDegree7Matrix()
    {
        return new double[,]
        {
            { 1, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0, 0, 0 },
            { 0, 0, 2, 0, 0, 0, 0, 0 },
            { 0, 0, 0, 6, 0, 0, 0, 0 },
            { 1, 1, 1, 1, 1, 1, 1, 1 },
            { 0, 1, 2, 3, 4, 5, 6, 7 },
            { 0, 0, 2, 6, 12, 20, 30, 42 },
            { 0, 0, 0, 6, 24, 60, 120, 210 }
        };
    }

    private static double[,] BuildDegree9Matrix()
    {
        return new double[,]
        {
            { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 0, 0, 6, 0, 0, 0, 0, 0, 0 },
            { 0, 0, 0, 0, 24, 0, 0, 0, 0, 0 },
            { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
            { 0, 0, 2, 6, 12, 20, 30, 42, 56, 72 },
            { 0, 0, 0, 6, 24, 60, 120, 210, 336, 504 },
            { 0, 0, 0, 0, 24, 120, 360, 840, 1680, 3024 }
        };
    }

    public static double[] BuildRightHandSide(int degree, State1D startState, State1D endState, double durationT)
    {
        int size = degree + 1;
        double[] b = new double[size];
        int levels = (degree + 1) / 2;
        double T = durationT;

        double[] startVals = { startState.Position, startState.Velocity, startState.Acceleration, startState.Jerk, startState.Snap };
        double[] endVals = { endState.Position, endState.Velocity, endState.Acceleration, endState.Jerk, endState.Snap };

        for (int k = 0; k < levels; k++)
        {
            double tPowK = 1.0;
            for (int i = 0; i < k; i++) tPowK *= T;
            b[k] = startVals[k] * tPowK;
            b[k + levels] = endVals[k] * tPowK;
        }

        return b;
    }
}
