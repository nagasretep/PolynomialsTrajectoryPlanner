using TrajectoryPlanner.Core.ArcLength;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.Trajectories;

/// <summary>Minimum, maximum and largest absolute value of one quantity along one axis over a whole trajectory, with where they occur.</summary>
/// <param name="MaxAbsolutePart">Zero-based index of the piece where |value| is largest.</param>
public sealed record AxisExtremum(
    double Minimum, double MinimumTime,
    double Maximum, double MaximumTime,
    double MaxAbsolute, double MaxAbsoluteTime, int MaxAbsolutePart);

/// <summary>Extrema of a 3D trajectory: <c>Of(axis, order)</c> for axis 0..2 (x, y, z) and order 0..4 (position … snap), and the largest speed ‖v‖.</summary>
public sealed record TrajectoryExtrema3D(AxisExtremum[][] ByAxisAndOrder, double MaxSpeed, double MaxSpeedTime)
{
    public AxisExtremum Of(int axis, int order) => ByAxisAndOrder[axis][order];
}

/// <summary>One row of the limit report: the peak of a quantity on an axis against its limit.</summary>
public sealed record LimitReportRow3D(
    string Quantity, string Axis, double Peak, double Limit, double PercentOfLimit, int PartIndex, string PartLabel, double Time)
{
    public bool Violated => Peak > Limit * (1 + TrajectoryAnalyzer3D.ViolationRelativeSlack) + TrajectoryAnalyzer3D.ViolationAbsoluteSlack;
}

/// <summary>
/// Whole-trajectory analysis of a <see cref="Trajectory3D"/>: extrema per axis and order, limit report and verification,
/// path length. Polynomial pieces are analysed exactly (<see cref="ExtremaAnalyzer"/>: the real roots of the derivative
/// polynomial, plus the ends); the curved stretch by its own routine (grid + refinement, because it is not polynomial).
/// </summary>
public static class TrajectoryAnalyzer3D
{
    /// <summary>A peak counts as above its limit only beyond this relative slack plus <see cref="ViolationAbsoluteSlack"/>.</summary>
    public const double ViolationRelativeSlack = 1e-9;

    public const double ViolationAbsoluteSlack = 1e-10;

    private static readonly string[] Axes = { "x", "y", "z" };
    private static readonly string[] Quantities = { "Position", "Velocity", "Acceleration", "Jerk", "Snap" };

    public static TrajectoryExtrema3D Extrema(Trajectory3D trajectory)
    {
        var axes = new AxisExtremum[3][];
        for (int axis = 0; axis < 3; axis++)
        {
            axes[axis] = new AxisExtremum[5];
            for (int order = 0; order <= 4; order++)
                axes[axis][order] = ExtremumOf(trajectory, axis, order);
        }

        var (speed, time) = MaxSpeed(trajectory);
        return new TrajectoryExtrema3D(axes, speed, time);
    }

    /// <summary>The peak of every active limit on every axis, with the piece and time where it occurs (never throws for a violation).</summary>
    public static IReadOnlyList<LimitReportRow3D> LimitReport(Trajectory3D trajectory, KinematicLimits limits)
    {
        var rows = new List<LimitReportRow3D>();
        var active = new (int Order, double? Limit)[]
        {
            (1, limits.MaxVelocity), (2, limits.MaxAcceleration), (3, limits.MaxJerk), (4, limits.MaxSnap)
        };

        foreach (var (order, limit) in active)
        {
            if (limit is null)
                continue;

            for (int axis = 0; axis < 3; axis++)
            {
                AxisExtremum e = ExtremumOf(trajectory, axis, order);
                rows.Add(new LimitReportRow3D(
                    Quantities[order], Axes[axis], e.MaxAbsolute, limit.Value, 100.0 * e.MaxAbsolute / limit.Value,
                    e.MaxAbsolutePart, trajectory.Parts[e.MaxAbsolutePart].Label, e.MaxAbsoluteTime));
            }
        }

        return rows;
    }

    /// <summary>The report of <see cref="LimitReport"/>, throwing for the worst violation.</summary>
    /// <exception cref="KinematicLimitsViolatedException">A peak exceeds its limit; the message names the piece, axis and quantity.</exception>
    public static IReadOnlyList<LimitReportRow3D> VerifyLimits(Trajectory3D trajectory, KinematicLimits limits)
    {
        var rows = LimitReport(trajectory, limits);
        LimitReportRow3D? worst = rows.Where(r => r.Violated).OrderByDescending(r => r.PercentOfLimit).FirstOrDefault();
        if (worst is not null)
        {
            throw new KinematicLimitsViolatedException(
                $"{worst.PartLabel} violates the {worst.Quantity.ToLowerInvariant()} limit on axis {worst.Axis}: " +
                $"peak {worst.Peak:G6} exceeds {worst.Limit:G6} ({worst.PercentOfLimit:F2} % of the limit) at t = {worst.Time:G6} s.");
        }

        return rows;
    }

    /// <summary>Total length of the path: polynomial pieces by the quadrature of the arc length, a stretch by its own length.</summary>
    public static double PathLength(Trajectory3D trajectory)
    {
        double total = 0;
        foreach (ISegment3D segment in trajectory.Segments)
        {
            total += segment switch
            {
                PolynomialSegment3D polynomial => new ArcLengthPath(polynomial, 0, polynomial.Duration, requireRegular: false).Length,
                CurvedConstantSpeedStretch3D stretch => stretch.Subpath.Length,
                _ => SampledLength(segment)
            };
        }

        return total;
    }

    // ---- extrema -------------------------------------------------------------------------------

    private static AxisExtremum ExtremumOf(Trajectory3D trajectory, int axis, int order)
    {
        double min = double.MaxValue, max = double.MinValue, maxAbs = -1;
        double tMin = 0, tMax = 0, tAbs = 0;
        int partAbs = 0;

        for (int i = 0; i < trajectory.Segments.Count; i++)
        {
            double start = trajectory.StartTimeOf(i);
            var (pMin, pMinTime, pMax, pMaxTime) = PieceExtremum(trajectory.Segments[i], axis, order);

            if (pMin < min) { min = pMin; tMin = start + pMinTime; }
            if (pMax > max) { max = pMax; tMax = start + pMaxTime; }

            if (Math.Abs(pMin) > maxAbs) { maxAbs = Math.Abs(pMin); tAbs = start + pMinTime; partAbs = i; }
            if (Math.Abs(pMax) > maxAbs) { maxAbs = Math.Abs(pMax); tAbs = start + pMaxTime; partAbs = i; }
        }

        return new AxisExtremum(min, tMin, max, tMax, maxAbs, tAbs, partAbs);
    }

    private static (double Min, double MinTime, double Max, double MaxTime) PieceExtremum(ISegment3D piece, int axis, int order)
    {
        switch (piece)
        {
            case PolynomialSegment3D polynomial:
            {
                PolynomialSegment1D axisSegment = axis switch { 0 => polynomial.XAxis, 1 => polynomial.YAxis, _ => polynomial.ZAxis };
                ExtremaResult e = ExtremaAnalyzer.FindExtrema(axisSegment, order);
                return (e.Minimum, e.MinTime, e.Maximum, e.MaxTime);
            }
            case CurvedConstantSpeedStretch3D stretch:
                return stretch.ComponentExtrema(order)[axis];
            default:
            {
                double min = double.MaxValue, max = double.MinValue, tMin = 0, tMax = 0;
                for (int i = 0; i <= 400; i++)
                {
                    double t = piece.Duration * i / 400;
                    double v = piece.EvaluateDerivative(t, order)[axis];
                    if (v < min) { min = v; tMin = t; }
                    if (v > max) { max = v; tMax = t; }
                }

                return (min, tMin, max, tMax);
            }
        }
    }

    private static (double Speed, double Time) MaxSpeed(Trajectory3D trajectory)
    {
        double best = 0, bestTime = 0;
        for (int i = 0; i < trajectory.Segments.Count; i++)
        {
            double start = trajectory.StartTimeOf(i);
            ISegment3D piece = trajectory.Segments[i];

            if (piece is CurvedConstantSpeedStretch3D stretch)
            {
                if (stretch.Speed > best) { best = stretch.Speed; bestTime = start; }
                continue;
            }

            // |v| of a polynomial piece: scan, then golden-section refinement around the best sample.
            double Speed(double t) => piece.EvaluateDerivative(t, 1).Length;
            const int grid = 400;
            int bestIndex = 0;
            double bestValue = -1;
            for (int k = 0; k <= grid; k++)
            {
                double v = Speed(piece.Duration * k / grid);
                if (v > bestValue) { bestValue = v; bestIndex = k; }
            }

            const double invPhi = 0.6180339887498949;
            double lo = piece.Duration * Math.Max(bestIndex - 1, 0) / grid;
            double hi = piece.Duration * Math.Min(bestIndex + 1, grid) / grid;
            double bestT = piece.Duration * bestIndex / grid;
            for (int k = 0; k < 60; k++)
            {
                double c = hi - invPhi * (hi - lo), d = lo + invPhi * (hi - lo);
                double fc = Speed(c), fd = Speed(d);
                if (fc > bestValue) { bestValue = fc; bestT = c; }
                if (fd > bestValue) { bestValue = fd; bestT = d; }
                if (fc > fd) hi = d; else lo = c;
            }

            if (bestValue > best) { best = bestValue; bestTime = start + bestT; }
        }

        return (best, bestTime);
    }

    private static double SampledLength(ISegment3D segment)
    {
        const int n = 2000;
        double length = 0;
        Point3D previous = segment.EvaluatePosition(0);
        for (int i = 1; i <= n; i++)
        {
            Point3D p = segment.EvaluatePosition(segment.Duration * i / n);
            length += (p - previous).Length;
            previous = p;
        }

        return length;
    }
}
