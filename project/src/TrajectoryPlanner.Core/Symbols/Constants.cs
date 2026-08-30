namespace TrajectoryPlanner.Core.Symbols;

public static class TrajectoryConstants
{
    public const double Epsilon = 1e-10;
    public const int MaxPolynomialDegree = 9;
    public const int MinPolynomialDegree = 3;
    public static readonly int[] SupportedDegrees = { 3, 5, 7, 9 };
}
