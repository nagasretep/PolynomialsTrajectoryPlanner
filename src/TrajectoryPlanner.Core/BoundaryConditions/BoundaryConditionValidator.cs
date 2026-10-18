using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.BoundaryConditions;

public static class BoundaryConditionValidator
{
    internal static int RequiredConditionCount(int degree)
    {
        if (!TrajectoryConstants.SupportedDegrees.Contains(degree))
            throw new ArgumentException($"Unsupported polynomial degree: {degree}. Supported: {string.Join(", ", TrajectoryConstants.SupportedDegrees)}");
        return degree + 1;
    }

    public static int DerivativeLevelsForDegree(int degree) => (degree + 1) / 2;
}
