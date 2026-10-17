namespace TrajectoryPlanner.Core.Points;

public readonly struct Point1D : IEquatable<Point1D>
{
    public double X { get; }

    public Point1D(double x)
    {
        X = x;
    }

    public static Point1D Zero => new(0.0);

    public static Point1D operator +(Point1D a, Point1D b) => new(a.X + b.X);
    public static Point1D operator -(Point1D a, Point1D b) => new(a.X - b.X);
    public static Point1D operator *(Point1D p, double s) => new(p.X * s);
    public static Point1D operator *(double s, Point1D p) => new(p.X * s);
    public static Point1D operator /(Point1D p, double s) => new(p.X / s);

    public static implicit operator Point1D(double x) => new(x);
    public static implicit operator double(Point1D p) => p.X;

    public double DistanceTo(Point1D other) => Math.Abs(X - other.X);

    public bool Equals(Point1D other) => Math.Abs(X - other.X) < 1e-12;
    public override bool Equals(object? obj) => obj is Point1D p && Equals(p);
    public override int GetHashCode() => X.GetHashCode();
    public override string ToString() => $"Point1D({X:G6})";

    public static bool operator ==(Point1D left, Point1D right) => left.Equals(right);
    public static bool operator !=(Point1D left, Point1D right) => !left.Equals(right);
}
