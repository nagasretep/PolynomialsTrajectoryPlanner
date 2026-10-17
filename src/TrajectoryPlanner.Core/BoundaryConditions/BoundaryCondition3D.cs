using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.BoundaryConditions;

public readonly record struct BoundaryCondition3D(
    DerivativeOrder Order,
    BoundaryEndpoint Endpoint,
    Point3D Value)
{
    public static BoundaryCondition3D Position(BoundaryEndpoint end, Point3D value) =>
        new(DerivativeOrder.Position, end, value);

    public static BoundaryCondition3D Velocity(BoundaryEndpoint end, Point3D value) =>
        new(DerivativeOrder.Velocity, end, value);

    public static BoundaryCondition3D Acceleration(BoundaryEndpoint end, Point3D value) =>
        new(DerivativeOrder.Acceleration, end, value);

    public static BoundaryCondition3D Jerk(BoundaryEndpoint end, Point3D value) =>
        new(DerivativeOrder.Jerk, end, value);

    public static BoundaryCondition3D Snap(BoundaryEndpoint end, Point3D value) =>
        new(DerivativeOrder.Snap, end, value);

    public BoundaryCondition1D GetComponent(int axis) => new(
        Order,
        Endpoint,
        Value[axis]);
}
