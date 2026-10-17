using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.BoundaryConditions;

public static class BoundaryConditionValidator
{
    public static int RequiredConditionCount(int degree)
    {
        if (!TrajectoryConstants.SupportedDegrees.Contains(degree))
            throw new ArgumentException($"Unsupported polynomial degree: {degree}. Supported: {string.Join(", ", TrajectoryConstants.SupportedDegrees)}");
        return degree + 1;
    }

    public static int DerivativeLevelsForDegree(int degree) => (degree + 1) / 2;

    public static DerivativeOrder[] RequiredDerivativeOrders(int degree)
    {
        int levels = DerivativeLevelsForDegree(degree);
        var orders = new DerivativeOrder[levels];
        for (int i = 0; i < levels; i++)
            orders[i] = (DerivativeOrder)i;
        return orders;
    }

    public static void ValidateConditions1D(int degree, IReadOnlyList<BoundaryCondition1D> conditions)
    {
        int required = RequiredConditionCount(degree);
        if (conditions.Count != required)
            throw new ArgumentException($"Degree {degree} requires {required} boundary conditions, got {conditions.Count}");

        var requiredOrders = RequiredDerivativeOrders(degree);
        foreach (var order in requiredOrders)
        {
            bool hasStart = conditions.Any(c => c.Order == order && c.Endpoint == BoundaryEndpoint.Start);
            bool hasEnd = conditions.Any(c => c.Order == order && c.Endpoint == BoundaryEndpoint.End);
            if (!hasStart || !hasEnd)
                throw new ArgumentException($"Missing {order} conditions for start/end endpoints");
        }
    }

    public static void ValidateConditions3D(int degree, IReadOnlyList<BoundaryCondition3D> conditions)
    {
        int required = RequiredConditionCount(degree);
        if (conditions.Count != required)
            throw new ArgumentException($"Degree {degree} requires {required} boundary conditions, got {conditions.Count}");

        var requiredOrders = RequiredDerivativeOrders(degree);
        foreach (var order in requiredOrders)
        {
            bool hasStart = conditions.Any(c => c.Order == order && c.Endpoint == BoundaryEndpoint.Start);
            bool hasEnd = conditions.Any(c => c.Order == order && c.Endpoint == BoundaryEndpoint.End);
            if (!hasStart || !hasEnd)
                throw new ArgumentException($"Missing {order} conditions for start/end endpoints");
        }
    }
}
