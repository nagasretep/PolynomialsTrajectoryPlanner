using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>
/// The unconstrained quadratic problem J(y) = y' H y + 2 f' y + const of
/// <c>theory/TrajectoryPassingThroughConstraintPoints.md</c> ("Reduction To One Linear System").
/// y collects the derivatives of order 1 .. p-1 at the interior waypoints; the exact passage,
/// the continuity and the zero end derivatives are built into the parametrization.
/// </summary>
public sealed class PassThroughProblem1D
{
    private readonly int _nodeCount;
    private readonly int _sharedCount;

    public int Degree { get; }
    public int DerivativeOrder { get; }

    /// <summary>H = sum_k G_k' R_k G_k (symmetric positive definite).</summary>
    public Matrix<double> H { get; }

    /// <summary>f = sum_k G_k' R_k g_k.</summary>
    public Vector<double> F { get; }

    /// <summary>Constant term g_k' R_k g_k summed over the segments (cost when y = 0).</summary>
    public double Constant { get; }

    /// <summary>Number of unknowns, (p - 1)(N - 1).</summary>
    public int UnknownCount => H.RowCount;

    private PassThroughProblem1D(int degree, int order, int nodeCount, Matrix<double> h, Vector<double> f, double constant)
    {
        Degree = degree;
        DerivativeOrder = order;
        _nodeCount = nodeCount;
        _sharedCount = SmoothnessCriterion.SharedStateCount(degree);
        H = h;
        F = f;
        Constant = constant;
    }

    public static PassThroughProblem1D Build(
        IReadOnlyList<double> waypoints,
        IReadOnlyList<double> durations,
        int degree,
        SmoothnessCriterion criterion)
    {
        if (!TrajectoryConstants.SupportedDegrees.Contains(degree))
            throw new ArgumentException($"Unsupported polynomial degree: {degree}", nameof(degree));
        if (waypoints.Count < 2)
            throw new ArgumentException("At least 2 waypoints are required.", nameof(waypoints));
        if (durations.Count != waypoints.Count - 1)
            throw new ArgumentException("durations length must be waypoints.Count - 1.", nameof(durations));
        foreach (double duration in durations)
        {
            if (!(duration > 0) || double.IsInfinity(duration))
                throw new ArgumentException("Every segment duration must be positive and finite.", nameof(durations));
        }

        criterion.Validate(degree);

        int m = criterion.DerivativeOrder;
        int p = SmoothnessCriterion.SharedStateCount(degree);
        int segmentCount = waypoints.Count - 1;
        int unknownCount = (p - 1) * (segmentCount - 1);

        // B = inverse of the normalized-time matrix (depends on the degree only); the right-hand side
        // of that system is ordered [start orders 0..p-1, end orders 0..p-1], which is the order of z_k.
        Matrix<double> b = Matrix<double>.Build.DenseOfArray(NormalizedTimeMatrixFactory.BuildNormalizedMatrix(degree)).Inverse();
        Matrix<double> q = Matrix<double>.Build.DenseOfArray(SmoothnessCost.BuildQ(degree, m));
        Matrix<double> bQb = b.TransposeThisAndMultiply(q * b);

        var h = Matrix<double>.Build.Dense(unknownCount, unknownCount);
        var f = Vector<double>.Build.Dense(unknownCount);
        double constant = 0;

        int size = 2 * p;
        var unknownIndex = new int[size];
        var knownValue = new double[size];

        for (int k = 0; k < segmentCount; k++)
        {
            double t = durations[k];

            // Delta_k = diag(1, T, ..., T^(p-1), 1, T, ..., T^(p-1)); R_k = T^(1-2m) Delta B' Q B Delta.
            var delta = new double[size];
            for (int r = 0; r < p; r++)
            {
                delta[r] = Math.Pow(t, r);
                delta[p + r] = delta[r];
            }

            double scale = Math.Pow(t, 1 - 2 * m);

            // z_k = G_k y + g_k: each entry is either a known number or one component of y.
            for (int s = 0; s < size; s++)
            {
                int node = s < p ? k : k + 1;
                int order = s % p;
                bool globalEnd = node == 0 || node == segmentCount;

                if (order == 0)
                {
                    unknownIndex[s] = -1;
                    knownValue[s] = waypoints[node];
                }
                else if (globalEnd)
                {
                    unknownIndex[s] = -1;
                    knownValue[s] = 0.0;
                }
                else
                {
                    unknownIndex[s] = (node - 1) * (p - 1) + (order - 1);
                    knownValue[s] = 0.0;
                }
            }

            for (int row = 0; row < size; row++)
            {
                for (int col = 0; col < size; col++)
                {
                    double r = scale * delta[row] * bQb[row, col] * delta[col];
                    int ir = unknownIndex[row];
                    int ic = unknownIndex[col];

                    if (ir >= 0 && ic >= 0)
                        h[ir, ic] += r;
                    else if (ir >= 0)
                        f[ir] += r * knownValue[col];
                    else if (ic < 0)
                        constant += knownValue[row] * r * knownValue[col];
                }
            }
        }

        return new PassThroughProblem1D(degree, m, waypoints.Count, h, f, constant);
    }

    /// <summary>
    /// Diagonal scaling s_i = 1 / sqrt(H_ii) (Jacobi / symmetric diagonal equilibration). The unknowns
    /// have very different physical units (velocity ... snap) and durations can differ by orders of
    /// magnitude, so H is badly scaled; S H S has a unit diagonal and a much smaller condition number.
    /// </summary>
    public Vector<double> DiagonalScale()
    {
        return Vector<double>.Build.Dense(UnknownCount, i => 1.0 / Math.Sqrt(H[i, i]));
    }

    /// <summary>
    /// Solves H y = -f. The system is first scaled, S H S z = -S f, factorized with Cholesky
    /// (no explicit inverse) and mapped back with y = S z. The same minimizer, better conditioned.
    /// </summary>
    public Vector<double> Solve()
    {
        if (UnknownCount == 0)
            return Vector<double>.Build.Dense(0);

        Vector<double> s = DiagonalScale();
        Matrix<double> scaledH = H.Clone();
        for (int i = 0; i < UnknownCount; i++)
            for (int j = 0; j < UnknownCount; j++)
                scaledH[i, j] *= s[i] * s[j];

        Vector<double> scaledF = F.PointwiseMultiply(s);
        Vector<double> z = scaledH.Cholesky().Solve(-scaledF);
        return z.PointwiseMultiply(s);
    }

    /// <summary>
    /// J(y) = y' H y + 2 f' y + const.
    /// Unreliable for large constants: when the constant term is many orders of magnitude larger than J
    /// (measured up to 4e6 times, d = 9, m = 5, 30 segments), the sum cancels and loses digits.
    /// Meant for the optimality tests (comparing J at nearby points). For a cost to report use
    /// <see cref="SmoothnessCost.Of"/>, which integrates the segments directly.
    /// </summary>
    public double Cost(Vector<double> y)
    {
        if (UnknownCount == 0)
            return Constant;

        return y * (H * y) + 2.0 * (F * y) + Constant;
    }

    /// <summary>Gradient of J, 2 (H y + f).</summary>
    public Vector<double> Gradient(Vector<double> y) => 2.0 * (H * y + F);

    /// <summary>
    /// Derivatives of order 0 .. p-1 at every waypoint: [node][order]. Position is taken from
    /// <paramref name="waypoints"/>, global ends are zero, interior values come from y.
    /// </summary>
    public double[][] NodeDerivatives(IReadOnlyList<double> waypoints, Vector<double> y)
    {
        var nodes = new double[_nodeCount][];
        for (int i = 0; i < _nodeCount; i++)
        {
            nodes[i] = new double[_sharedCount];
            nodes[i][0] = waypoints[i];

            if (i == 0 || i == _nodeCount - 1)
                continue;

            for (int order = 1; order < _sharedCount; order++)
                nodes[i][order] = y[(i - 1) * (_sharedCount - 1) + (order - 1)];
        }

        return nodes;
    }
}
