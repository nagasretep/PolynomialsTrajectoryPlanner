using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.ArcLength;

/// <summary>
/// Arc length ℓ(t) = ∫ σ(u) du of the subpath [t_s, t_e] of a polynomial 3D segment (t is the time of the segment) and
/// its inverse (<c>theory/ConstantVelocitySegment.md</c>, "Evaluation Of The Stretch").
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item>Arc length: composite 16-point Gauss–Legendre; the number of panels is doubled until two successive values agree
/// to a relative <see cref="QuadratureTolerance"/>.</item>
/// <item>Inversion: safeguarded Newton (bracket [t_s, t_e], derivative σ), bisection whenever a step leaves the bracket,
/// until |ℓ(t*) − ℓ*| ≤ <see cref="InversionTolerance"/> · L; ℓ* ≤ 0 returns t_s and ℓ* ≥ L returns t_e exactly.</item>
/// <item>Regularity: σ must stay above <see cref="RegularityTolerance"/> times its largest value on the subpath (checked
/// on a grid with a local refinement of the minimum); otherwise <see cref="PathNotRegularException"/>.</item>
/// </list>
/// </remarks>
public sealed class ArcLengthPath
{
    public const double QuadratureTolerance = 1e-13;
    public const double InversionTolerance = 1e-13;

    /// <summary>σ_min / σ_max below this value is a rest point for the purpose of constant-speed motion.</summary>
    public const double RegularityTolerance = 1e-9;

    private const int GaussPoints = 16;
    private const int MaxPanels = 1 << 13;
    private const int RegularityGrid = 400;

    private static readonly (double[] Nodes, double[] Weights) Rule = GaussLegendre(GaussPoints);

    public PolynomialSegment3D Path { get; }

    /// <summary>Start and end of the subpath, as times of the segment.</summary>
    public double StartTime { get; }

    public double EndTime { get; }

    /// <summary>Total length of the subpath.</summary>
    public double Length { get; }

    /// <param name="requireRegular">False only for measuring a whole segment that may touch a rest point (σ = 0 at its ends).</param>
    /// <exception cref="ArgumentException">Not 0 ≤ t_s &lt; t_e ≤ duration.</exception>
    /// <exception cref="PathNotRegularException">σ vanishes on the subpath.</exception>
    public ArcLengthPath(PolynomialSegment3D path, double startTime, double endTime, bool requireRegular = true)
    {
        if (!(startTime >= 0) || !(endTime <= path.Duration) || !(startTime < endTime))
        {
            throw new ArgumentException(
                $"The subpath must satisfy 0 <= t_s < t_e <= {path.Duration:G6}, got {startTime:G6} and {endTime:G6}.");
        }

        Path = path;
        StartTime = startTime;
        EndTime = endTime;

        if (requireRegular)
            RequireRegular();

        Length = Integrate(startTime, endTime);
    }

    /// <summary>σ(t) = ‖ṗ(t)‖, the speed of the original parametrization.</summary>
    public double Speed(double t) => Path.EvaluateDerivative(Math.Clamp(t, 0, Path.Duration), 1).Length;

    /// <summary>ℓ(t), the arc length from the start of the subpath to the segment time t.</summary>
    public double LengthAt(double t)
    {
        if (t <= StartTime)
            return 0;
        return t >= EndTime ? Length : Integrate(StartTime, t);
    }

    /// <summary>The segment time t* with ℓ(t*) = <paramref name="length"/> (0 → t_s, L → t_e).</summary>
    public double TimeAtLength(double length)
    {
        if (length <= 0)
            return StartTime;
        if (length >= Length)
            return EndTime;

        double lo = StartTime, hi = EndTime;
        double t = StartTime + (EndTime - StartTime) * length / Length;
        double tolerance = InversionTolerance * Length;

        for (int i = 0; i < 200; i++)
        {
            double f = LengthAt(t) - length;
            if (Math.Abs(f) <= tolerance)
                return t;

            if (f > 0) hi = t; else lo = t;

            double sigma = Speed(t);
            double next = sigma > 0 ? t - f / sigma : double.NaN;
            if (!(next > lo) || !(next < hi))
                next = 0.5 * (lo + hi);

            if (Math.Abs(next - t) <= 1e-16 * Math.Max(1.0, Math.Abs(t)))
                return next;

            t = next;
        }

        return t;
    }

    private double Integrate(double a, double b)
    {
        if (!(b > a))
            return 0;

        double previous = Panels(a, b, 1);
        for (int panels = 2; panels <= MaxPanels; panels *= 2)
        {
            double current = Panels(a, b, panels);
            if (Math.Abs(current - previous) <= QuadratureTolerance * Math.Abs(current))
                return current;
            previous = current;
        }

        return previous;
    }

    private double Panels(double a, double b, int panels)
    {
        double h = (b - a) / panels;
        double sum = 0;
        for (int p = 0; p < panels; p++)
        {
            double mid = a + (p + 0.5) * h;
            for (int i = 0; i < GaussPoints; i++)
                sum += Rule.Weights[i] * Speed(mid + 0.5 * h * Rule.Nodes[i]);
        }

        return 0.5 * h * sum;
    }

    private void RequireRegular()
    {
        double min = double.MaxValue, max = 0, tMin = StartTime;
        for (int i = 0; i <= RegularityGrid; i++)
        {
            double t = StartTime + (EndTime - StartTime) * i / RegularityGrid;
            double sigma = Speed(t);
            max = Math.Max(max, sigma);
            if (sigma < min)
            {
                min = sigma;
                tMin = t;
            }
        }

        // Refine the smallest sample inside its two neighbouring intervals (golden section on σ).
        double step = (EndTime - StartTime) / RegularityGrid;
        double lo = Math.Max(StartTime, tMin - step), hi = Math.Min(EndTime, tMin + step);
        const double invPhi = 0.6180339887498949;
        for (int k = 0; k < 80; k++)
        {
            double c = hi - invPhi * (hi - lo), d = lo + invPhi * (hi - lo);
            if (Speed(c) < Speed(d)) hi = d; else lo = c;
        }

        min = Math.Min(min, Speed(0.5 * (lo + hi)));

        if (!(min > RegularityTolerance * max))
        {
            throw new PathNotRegularException(
                $"The path is not regular on the subpath: its speed falls to {min:G3} (largest {max:G3}) near segment time {tMin:G6}. " +
                "A subpath through a rest point of the segment cannot be traversed at constant speed.");
        }
    }

    /// <summary>Nodes and weights of the n-point Gauss–Legendre rule on [-1, 1] (Newton on the Legendre polynomial).</summary>
    private static (double[] Nodes, double[] Weights) GaussLegendre(int n)
    {
        var nodes = new double[n];
        var weights = new double[n];
        for (int i = 0; i < (n + 1) / 2; i++)
        {
            double x = Math.Cos(Math.PI * (i + 0.75) / (n + 0.5));
            double derivative = 1;
            for (int iteration = 0; iteration < 100; iteration++)
            {
                double p0 = 1, p1 = x;
                for (int k = 2; k <= n; k++)
                {
                    double p2 = ((2 * k - 1) * x * p1 - (k - 1) * p0) / k;
                    p0 = p1;
                    p1 = p2;
                }

                derivative = n * (x * p1 - p0) / (x * x - 1);
                double dx = p1 / derivative;
                x -= dx;
                if (Math.Abs(dx) < 1e-16)
                    break;
            }

            nodes[i] = -x;
            nodes[n - 1 - i] = x;
            weights[i] = weights[n - 1 - i] = 2.0 / ((1 - x * x) * derivative * derivative);
        }

        return (nodes, weights);
    }
}
