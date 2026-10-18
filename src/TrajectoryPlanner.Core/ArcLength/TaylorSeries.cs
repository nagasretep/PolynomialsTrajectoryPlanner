namespace TrajectoryPlanner.Core.ArcLength;

/// <summary>
/// Truncated Taylor-series arithmetic: a series is the array <c>c</c> of the coefficients of
/// f(t0 + h) = c[0] + c[1] h + c[2] h² + ...; the length of the array is the (number of known terms). Operations on two
/// series keep the shorter length. Used to obtain the derivatives of a path with respect to arc length without long
/// symbolic formulas (<c>theory/ConstantVelocitySegment.md</c>, "Arc-Length Derivatives Of The Path").
/// </summary>
public static class TaylorSeries
{
    /// <summary>Product, truncated to the shorter length.</summary>
    public static double[] Multiply(double[] a, double[] b)
    {
        int n = Math.Min(a.Length, b.Length);
        var c = new double[n];
        for (int k = 0; k < n; k++)
        {
            double sum = 0;
            for (int i = 0; i <= k; i++)
                sum += a[i] * b[k - i];
            c[k] = sum;
        }

        return c;
    }

    /// <summary>d/dh: the coefficients (k+1) c[k+1]; one term is lost.</summary>
    public static double[] Derivative(double[] a)
    {
        if (a.Length < 2)
            throw new ArgumentException("A series with a single term cannot be differentiated.", nameof(a));

        var d = new double[a.Length - 1];
        for (int k = 0; k < d.Length; k++)
            d[k] = (k + 1) * a[k + 1];
        return d;
    }

    /// <summary>Square root of a series whose constant term is positive.</summary>
    public static double[] SquareRoot(double[] a)
    {
        if (!(a[0] > 0))
            throw new ArgumentException("The square root of a series needs a positive constant term.", nameof(a));

        var s = new double[a.Length];
        s[0] = Math.Sqrt(a[0]);
        for (int k = 1; k < a.Length; k++)
        {
            double sum = 0;
            for (int i = 1; i < k; i++)
                sum += s[i] * s[k - i];
            s[k] = (a[k] - sum) / (2 * s[0]);
        }

        return s;
    }

    /// <summary>Reciprocal 1/a of a series with a non-zero constant term.</summary>
    public static double[] Reciprocal(double[] a)
    {
        if (a[0] == 0)
            throw new ArgumentException("The reciprocal of a series needs a non-zero constant term.", nameof(a));

        var r = new double[a.Length];
        r[0] = 1.0 / a[0];
        for (int k = 1; k < a.Length; k++)
        {
            double sum = 0;
            for (int i = 1; i <= k; i++)
                sum += a[i] * r[k - i];
            r[k] = -sum / a[0];
        }

        return r;
    }
}
