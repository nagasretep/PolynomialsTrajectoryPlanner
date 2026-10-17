using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>
/// Exact value of the smoothness cost J = sum_k integral_0^T_k ( x_k^(m)(t) )^2 dt
/// of <c>theory/TrajectoryPassingThroughConstraintPoints.md</c> (section "The Cost").
/// </summary>
public static class SmoothnessCost
{
    /// <summary>
    /// Matrix Q of the quadratic form c' Q c = integral_0^1 ( d^m x / d tau^m )^2 d tau,
    /// with Q_ij = i^(m) j^(m) / (i + j - 2m + 1) for i, j >= m (falling factorials), 0 otherwise.
    /// </summary>
    public static double[,] BuildQ(int degree, int order)
    {
        if (order < 1 || order > degree)
            throw new ArgumentOutOfRangeException(nameof(order), $"Derivative order must be between 1 and {degree}.");

        int size = degree + 1;
        var q = new double[size, size];
        for (int i = order; i < size; i++)
        {
            for (int j = order; j < size; j++)
            {
                q[i, j] = FallingFactorial(i, order) * FallingFactorial(j, order) / (i + j - 2 * order + 1);
            }
        }

        return q;
    }

    /// <summary>J_k = T^(1-2m) c' Q c for one segment.</summary>
    public static double OfSegment(PolynomialSegment1D segment, int order)
    {
        double[] c = segment.Coefficients.NormalizedCoefficients;
        double[,] q = BuildQ(segment.Degree, order);

        double sum = 0;
        for (int i = order; i < c.Length; i++)
            for (int j = order; j < c.Length; j++)
                sum += c[i] * q[i, j] * c[j];

        return Math.Pow(segment.Duration, 1 - 2 * order) * sum;
    }

    /// <summary>J = sum_k J_k over all segments.</summary>
    public static double Of(IEnumerable<PolynomialSegment1D> segments, int order)
    {
        double total = 0;
        foreach (var segment in segments)
            total += OfSegment(segment, order);
        return total;
    }

    private static double FallingFactorial(int n, int m)
    {
        double result = 1;
        for (int k = 0; k < m; k++)
            result *= n - k;
        return result;
    }
}
