using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.Polynomials;

public static class PolynomialEvaluator
{
    public static double EvaluateCanonical(double tau, double[] coefficients)
    {
        double result = 0.0;
        double power = 1.0;
        for (int i = 0; i < coefficients.Length; i++)
        {
            result += coefficients[i] * power;
            power *= tau;
        }
        return result;
    }

    public static double EvaluateHorner(double tau, double[] coefficients)
    {
        int n = coefficients.Length - 1;
        double result = coefficients[n];
        for (int i = n - 1; i >= 0; i--)
        {
            result = result * tau + coefficients[i];
        }
        return result;
    }

    public static double Evaluate(double tau, PolynomialCoefficients coeffs, bool useHorner = true)
    {
        return useHorner
            ? EvaluateHorner(tau, coeffs.NormalizedCoefficients)
            : EvaluateCanonical(tau, coeffs.NormalizedCoefficients);
    }

    public static double[] EvaluateDerivativeCoefficients(double[] coefficients, int derivativeOrder)
    {
        if (derivativeOrder < 0)
            throw new ArgumentOutOfRangeException(nameof(derivativeOrder));

        if (derivativeOrder == 0)
            return (double[])coefficients.Clone();

        int originalDegree = coefficients.Length - 1;
        if (derivativeOrder > originalDegree)
            return new double[] { 0.0 };

        int newDegree = originalDegree - derivativeOrder;
        double[] result = new double[newDegree + 1];

        for (int i = derivativeOrder; i <= originalDegree; i++)
        {
            int factorial = 1;
            for (int k = 0; k < derivativeOrder; k++)
            {
                factorial *= (i - k);
            }
            result[i - derivativeOrder] = coefficients[i] * factorial;
        }

        return result;
    }

    public static double EvaluateDerivative(double tau, PolynomialCoefficients coeffs, int derivativeOrder, double durationT)
    {
        double[] derivCoeffs = EvaluateDerivativeCoefficients(coeffs.NormalizedCoefficients, derivativeOrder);
        double derivativeInTau = EvaluateHorner(tau, derivCoeffs);
        double durationFactor = Math.Pow(durationT, derivativeOrder);
        return derivativeInTau / durationFactor;
    }

    public static double EvaluateDerivative(double tau, double[] coefficients, int derivativeOrder, double durationT)
    {
        double[] derivCoeffs = EvaluateDerivativeCoefficients(coefficients, derivativeOrder);
        double derivativeInTau = EvaluateHorner(tau, derivCoeffs);
        double durationFactor = Math.Pow(durationT, derivativeOrder);
        return derivativeInTau / durationFactor;
    }

    public static (double Position, double Velocity, double Acceleration, double Jerk, double Snap)
        EvaluateState1D(double tau, PolynomialCoefficients coeffs, double durationT)
    {
        double[] c = coeffs.NormalizedCoefficients;
        double T = durationT;

        double pos = EvaluateHorner(tau, c);

        double[] vCoeffs = EvaluateDerivativeCoefficients(c, 1);
        double vel = EvaluateHorner(tau, vCoeffs) / T;

        double[] aCoeffs = EvaluateDerivativeCoefficients(c, 2);
        double acc = EvaluateHorner(tau, aCoeffs) / (T * T);

        double[] jCoeffs = EvaluateDerivativeCoefficients(c, 3);
        double jerk = EvaluateHorner(tau, jCoeffs) / (T * T * T);

        double[] sCoeffs = EvaluateDerivativeCoefficients(c, 4);
        double snap = EvaluateHorner(tau, sCoeffs) / (T * T * T * T);

        return (pos, vel, acc, jerk, snap);
    }

    public static DerivativeOrder[] DerivativeOrdersFromIndex(int[] indices)
    {
        return indices.Select(i => (DerivativeOrder)i).ToArray();
    }
}
