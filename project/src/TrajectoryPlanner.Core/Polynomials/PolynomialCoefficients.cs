namespace TrajectoryPlanner.Core.Polynomials;

public readonly struct PolynomialCoefficients
{
    public int Degree { get; }
    public double[] NormalizedCoefficients { get; }

    public PolynomialCoefficients(int degree, double[] normalizedCoefficients)
    {
        if (normalizedCoefficients.Length != degree + 1)
            throw new ArgumentException($"Coefficient count mismatch for degree {degree}: expected {degree + 1}, got {normalizedCoefficients.Length}");

        Degree = degree;
        NormalizedCoefficients = (double[])normalizedCoefficients.Clone();
    }

    public double this[int index]
    {
        get
        {
            if (index < 0 || index > Degree)
                throw new ArgumentOutOfRangeException(nameof(index));
            return NormalizedCoefficients[index];
        }
    }

    public double[] GetCoefficientsCopy() => (double[])NormalizedCoefficients.Clone();
}
