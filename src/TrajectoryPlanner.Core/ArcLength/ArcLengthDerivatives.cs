using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.ArcLength;

/// <summary>The path has a vanishing tangent (σ = 0) where the arc-length derivatives are asked for.</summary>
public sealed class PathNotRegularException : ArgumentException
{
    public PathNotRegularException(string message) : base(message)
    {
    }
}

/// <summary>
/// Derivatives of a polynomial 3D path with respect to arc length (<c>theory/ConstantVelocitySegment.md</c>,
/// "Arc-Length Derivatives Of The Path"): with σ = ‖ṗ‖ and D f = ḟ/σ, the vectors p_s = Dp, p_ss = D²p, p_sss = D³p,
/// p_ssss = D⁴p. They are computed at one parameter by series arithmetic: expand each component to order 4, form the
/// series of σ² = ṗ·ṗ, of σ and of 1/σ, then apply D four times (differentiate, multiply by 1/σ). Each application costs
/// one term of the series, so order 4 is exactly enough for D⁴p.
/// </summary>
public static class ArcLengthDerivatives
{
    /// <summary>Highest derivative order supported (snap needs D⁴).</summary>
    public const int MaxOrder = 4;

    /// <summary>Below this speed the tangent is considered undefined.</summary>
    public const double MinimumSpeed = 1e-12;

    /// <summary>
    /// σ and D^k p (k = 1 .. <paramref name="maxOrder"/>; element k-1) of the path at the segment time <paramref name="t0"/>.
    /// </summary>
    /// <exception cref="PathNotRegularException">σ(t0) is (numerically) zero.</exception>
    public static (double Speed, Point3D[] Derivatives) Compute(PolynomialSegment3D path, double t0, int maxOrder = MaxOrder)
    {
        if (maxOrder < 1 || maxOrder > MaxOrder)
            throw new ArgumentOutOfRangeException(nameof(maxOrder));

        // Taylor coefficients of every component around t0, up to maxOrder terms beyond the constant.
        // Grids computed as t0 + (t1 - t0) i / n can overshoot the end of the segment by one ulp.
        t0 = Math.Clamp(t0, 0, path.Duration);

        var axes = new[] { path.XAxis, path.YAxis, path.ZAxis };
        var series = new double[3][];
        for (int i = 0; i < 3; i++)
        {
            series[i] = new double[maxOrder + 1];
            double factorial = 1;
            for (int k = 0; k <= maxOrder; k++)
            {
                if (k > 1)
                    factorial *= k;
                double derivative = k == 0 ? axes[i].EvaluatePosition(t0) : axes[i].EvaluateDerivative(t0, k);
                series[i][k] = derivative / factorial;
            }
        }

        // σ² = ṗ·ṗ, σ, 1/σ (maxOrder terms: the derivative of the position series has one term less).
        double[] speedSquared = new double[maxOrder];
        for (int i = 0; i < 3; i++)
        {
            double[] d = TaylorSeries.Derivative(series[i]);
            double[] sq = TaylorSeries.Multiply(d, d);
            for (int k = 0; k < maxOrder; k++)
                speedSquared[k] += sq[k];
        }

        if (!(speedSquared[0] > MinimumSpeed * MinimumSpeed))
        {
            throw new PathNotRegularException(
                $"The path has no defined tangent at segment time {t0:G6}: its speed is {Math.Sqrt(Math.Max(speedSquared[0], 0)):G3}. " +
                "A subpath through a rest point of the segment cannot be traversed at constant speed.");
        }

        double[] speed = TaylorSeries.SquareRoot(speedSquared);
        double[] inverseSpeed = TaylorSeries.Reciprocal(speed);

        var result = new Point3D[maxOrder];
        var current = new double[3][];
        for (int i = 0; i < 3; i++)
            current[i] = series[i];

        for (int order = 1; order <= maxOrder; order++)
        {
            for (int i = 0; i < 3; i++)
                current[i] = TaylorSeries.Multiply(TaylorSeries.Derivative(current[i]), inverseSpeed);

            result[order - 1] = new Point3D(current[0][0], current[1][0], current[2][0]);
        }

        return (speed[0], result);
    }
}
