using TrajectoryPlanner.Core.Points;

namespace TrajectoryPlanner.Core.States;

public readonly struct State3D : IEquatable<State3D>
{
    public Point3D Position { get; }
    public Point3D Velocity { get; }
    public Point3D Acceleration { get; }
    public Point3D Jerk { get; }
    public Point3D Snap { get; }

    public State3D(Point3D position, Point3D velocity, Point3D acceleration, Point3D jerk, Point3D snap)
    {
        Position = position;
        Velocity = velocity;
        Acceleration = acceleration;
        Jerk = jerk;
        Snap = snap;
    }

    public static State3D Zero => new(Point3D.Zero, Point3D.Zero, Point3D.Zero, Point3D.Zero, Point3D.Zero);

    public static State3D FromPositionOnly(Point3D position) =>
        new(position, Point3D.Zero, Point3D.Zero, Point3D.Zero, Point3D.Zero);

    public State1D GetComponent(int axis) => new(
        Position[axis],
        Velocity[axis],
        Acceleration[axis],
        Jerk[axis],
        Snap[axis]);

    public Point3D GetDerivative(int order) => order switch
    {
        0 => Position,
        1 => Velocity,
        2 => Acceleration,
        3 => Jerk,
        4 => Snap,
        _ => Point3D.Zero
    };

    public bool Equals(State3D other) =>
        Position == other.Position &&
        Velocity == other.Velocity &&
        Acceleration == other.Acceleration &&
        Jerk == other.Jerk &&
        Snap == other.Snap;

    public override bool Equals(object? obj) => obj is State3D s && Equals(s);
    public override int GetHashCode() => HashCode.Combine(Position, Velocity, Acceleration, Jerk, Snap);
    public override string ToString() => $"State3D(p={Position}, v={Velocity}, a={Acceleration}, j={Jerk}, s={Snap})";

    public static bool operator ==(State3D left, State3D right) => left.Equals(right);
    public static bool operator !=(State3D left, State3D right) => !left.Equals(right);
}
