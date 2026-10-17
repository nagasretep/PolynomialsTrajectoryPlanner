namespace TrajectoryPlanner.Core.States;

public readonly struct State1D : IEquatable<State1D>
{
    public double Position { get; }
    public double Velocity { get; }
    public double Acceleration { get; }
    public double Jerk { get; }
    public double Snap { get; }

    public State1D(double position, double velocity, double acceleration, double jerk, double snap)
    {
        Position = position;
        Velocity = velocity;
        Acceleration = acceleration;
        Jerk = jerk;
        Snap = snap;
    }

    public static State1D Zero => new(0.0, 0.0, 0.0, 0.0, 0.0);

    public static State1D FromPositionOnly(double position) =>
        new(position, 0.0, 0.0, 0.0, 0.0);

    public double GetDerivative(int order) => order switch
    {
        0 => Position,
        1 => Velocity,
        2 => Acceleration,
        3 => Jerk,
        4 => Snap,
        _ => 0.0
    };

    public bool Equals(State1D other) =>
        Math.Abs(Position - other.Position) < 1e-10 &&
        Math.Abs(Velocity - other.Velocity) < 1e-10 &&
        Math.Abs(Acceleration - other.Acceleration) < 1e-10 &&
        Math.Abs(Jerk - other.Jerk) < 1e-10 &&
        Math.Abs(Snap - other.Snap) < 1e-10;

    public override bool Equals(object? obj) => obj is State1D s && Equals(s);
    public override int GetHashCode() => HashCode.Combine(Position, Velocity, Acceleration, Jerk, Snap);
    public override string ToString() => $"State1D(x={Position:G4}, v={Velocity:G4}, a={Acceleration:G4}, j={Jerk:G4}, s={Snap:G4})";

    public static bool operator ==(State1D left, State1D right) => left.Equals(right);
    public static bool operator !=(State1D left, State1D right) => !left.Equals(right);
}
