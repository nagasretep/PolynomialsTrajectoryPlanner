using System.Numerics;

namespace TrajectoryPlanner.Core.Points;

public readonly struct Point3D : IEquatable<Point3D>
{
    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public Point3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public static Point3D Zero => new(0.0, 0.0, 0.0);

    public double this[int index] => index switch
    {
        0 => X,
        1 => Y,
        2 => Z,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };

    public static Point3D operator +(Point3D a, Point3D b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Point3D operator -(Point3D a, Point3D b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Point3D operator *(Point3D p, double s) => new(p.X * s, p.Y * s, p.Z * s);
    public static Point3D operator *(double s, Point3D p) => new(p.X * s, p.Y * s, p.Z * s);
    public static Point3D operator /(Point3D p, double s) => new(p.X / s, p.Y / s, p.Z / s);
    public static Point3D operator -(Point3D p) => new(-p.X, -p.Y, -p.Z);

    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    public double LengthSquared => X * X + Y * Y + Z * Z;

    public double DistanceTo(Point3D other)
    {
        double dx = X - other.X;
        double dy = Y - other.Y;
        double dz = Z - other.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    public Point3D Normalize()
    {
        double len = Length;
        if (len < 1e-15) return Zero;
        return this / len;
    }

    public double Dot(Point3D other) => X * other.X + Y * other.Y + Z * other.Z;
    public Point3D Cross(Point3D other) => new(
        Y * other.Z - Z * other.Y,
        Z * other.X - X * other.Z,
        X * other.Y - Y * other.X);

    public bool Equals(Point3D other) =>
        Math.Abs(X - other.X) < 1e-12 &&
        Math.Abs(Y - other.Y) < 1e-12 &&
        Math.Abs(Z - other.Z) < 1e-12;

    public override bool Equals(object? obj) => obj is Point3D p && Equals(p);
    public override int GetHashCode() => HashCode.Combine(X, Y, Z);
    public override string ToString() => $"Point3D({X:G6}, {Y:G6}, {Z:G6})";

    public static bool operator ==(Point3D left, Point3D right) => left.Equals(right);
    public static bool operator !=(Point3D left, Point3D right) => !left.Equals(right);
}
