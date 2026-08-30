using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.BoundaryConditions;

public enum BoundaryEndpoint
{
    Start,
    End
}

public readonly record struct BoundaryCondition1D(
    DerivativeOrder Order,
    BoundaryEndpoint Endpoint,
    double Value)
{
    public static BoundaryCondition1D Position(BoundaryEndpoint end, double value) =>
        new(DerivativeOrder.Position, end, value);

    public static BoundaryCondition1D Velocity(BoundaryEndpoint end, double value) =>
        new(DerivativeOrder.Velocity, end, value);

    public static BoundaryCondition1D Acceleration(BoundaryEndpoint end, double value) =>
        new(DerivativeOrder.Acceleration, end, value);

    public static BoundaryCondition1D Jerk(BoundaryEndpoint end, double value) =>
        new(DerivativeOrder.Jerk, end, value);

    public static BoundaryCondition1D Snap(BoundaryEndpoint end, double value) =>
        new(DerivativeOrder.Snap, end, value);
}
