using TrajectoryPlanner.Core.Points;

namespace TrajectoryPlanner.Core.ConstantVelocity;

public enum StretchSpecificationKind
{
    /// <summary>Two fractions τ of the normalized parameter of the segment.</summary>
    ParameterFractions,

    /// <summary>Two points on the path (the closest point of the path is used).</summary>
    Points,

    /// <summary>Two fractions of the total arc length of the segment.</summary>
    ArcLengthFractions
}

/// <summary>
/// How the subpath of a constant-speed stretch is delimited on its segment (<c>theory/ConstantVelocitySegment.md</c>,
/// "Specifying The Stretch"); the three forms are equivalent.
/// </summary>
public sealed record StretchSpecification
{
    public StretchSpecificationKind Kind { get; }
    public double Start { get; }
    public double End { get; }
    public Point3D StartPoint { get; }
    public Point3D EndPoint { get; }

    private StretchSpecification(StretchSpecificationKind kind, double start, double end, Point3D startPoint, Point3D endPoint)
    {
        Kind = kind;
        Start = start;
        End = end;
        StartPoint = startPoint;
        EndPoint = endPoint;
    }

    public static StretchSpecification FromParameterFractions(double tauStart, double tauEnd) =>
        new(StretchSpecificationKind.ParameterFractions, tauStart, tauEnd, Point3D.Zero, Point3D.Zero);

    public static StretchSpecification FromPoints(Point3D start, Point3D end) =>
        new(StretchSpecificationKind.Points, 0, 0, start, end);

    public static StretchSpecification FromArcLengthFractions(double lengthFractionStart, double lengthFractionEnd) =>
        new(StretchSpecificationKind.ArcLengthFractions, lengthFractionStart, lengthFractionEnd, Point3D.Zero, Point3D.Zero);
}
