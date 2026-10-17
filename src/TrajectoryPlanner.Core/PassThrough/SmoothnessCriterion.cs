namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>
/// The smoothness criterion of exact pass-through: minimize the integral of the square of the
/// derivative of order <see cref="DerivativeOrder"/> (m). See
/// <c>theory/TrajectoryPassingThroughConstraintPoints.md</c>, "Which Order m".
/// </summary>
/// <param name="DerivativeOrder">m: 2 = acceleration, 3 = jerk, 4 = snap, 5 = crackle.</param>
public readonly record struct SmoothnessCriterion(int DerivativeOrder)
{
    public static SmoothnessCriterion MinimumSnap => new(4);

    /// <summary>Number of shared state components per node, p = (d + 1) / 2.</summary>
    public static int SharedStateCount(int degree) => (degree + 1) / 2;

    /// <summary>Project default: m = p for degrees 3, 5, 7 and snap (m = 4) for degree 9.</summary>
    public static SmoothnessCriterion DefaultForDegree(int degree)
    {
        int p = SharedStateCount(degree);
        return new SmoothnessCriterion(degree == 9 ? 4 : p);
    }

    /// <summary>Checks 1 &lt;= m &lt;= p for the given degree.</summary>
    public void Validate(int degree)
    {
        int p = SharedStateCount(degree);
        if (DerivativeOrder < 1 || DerivativeOrder > p)
            throw new ArgumentOutOfRangeException(
                nameof(DerivativeOrder),
                $"Derivative order must satisfy 1 <= m <= {p} for degree {degree}, got {DerivativeOrder}.");
    }
}
