using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.Constraints;

public static class ExtremaAnalyzer
{
    public static ExtremaResult FindExtrema(PolynomialSegment1D segment, int derivativeOrder, int samplePoints = 200)
    {
        double[] coeffs = PolynomialEvaluator.EvaluateDerivativeCoefficients(
            segment.Coefficients.NormalizedCoefficients, derivativeOrder);

        double[] stationarityCoeffs = PolynomialEvaluator.EvaluateDerivativeCoefficients(coeffs, 1);
        double durationFactor = Math.Pow(segment.Duration, derivativeOrder);

        double globalMin = double.MaxValue;
        double globalMax = double.MinValue;
        double tMin = 0, tMax = 0;

        var candidateTaus = new List<double> { 0.0, 1.0 };

        if (stationarityCoeffs.Length > 1)
        {
            candidateTaus.AddRange(FindStationaryPoints(stationarityCoeffs));
        }

        for (int i = 0; i <= samplePoints; i++)
        {
            candidateTaus.Add((double)i / samplePoints);
        }

        foreach (double tau in candidateTaus)
        {
            if (tau < -1e-12 || tau > 1.0 + 1e-12)
                continue;

            double tauClamped = Math.Clamp(tau, 0.0, 1.0);
            double value = PolynomialEvaluator.EvaluateHorner(tauClamped, coeffs) / durationFactor;
            double absT = tauClamped * segment.Duration;

            if (value < globalMin)
            {
                globalMin = value;
                tMin = absT;
            }
            if (value > globalMax)
            {
                globalMax = value;
                tMax = absT;
            }
        }

        return new ExtremaResult(globalMin, globalMax, tMin, tMax);
    }

    private static List<double> FindStationaryPoints(double[] polynomialCoeffs)
    {
        var results = new List<double>();
        int degree = polynomialCoeffs.Length - 1;

        if (degree <= 0)
            return results;

        if (degree == 1)
        {
            if (Math.Abs(polynomialCoeffs[1]) > 1e-15)
            {
                results.Add(-polynomialCoeffs[0] / polynomialCoeffs[1]);
            }
            return results;
        }

        Func<double, double> f = x => PolynomialEvaluator.EvaluateHorner(x, polynomialCoeffs);

        double[] scanPoints = new double[100];
        for (int i = 0; i < scanPoints.Length; i++)
            scanPoints[i] = i / 99.0;

        for (int i = 0; i < scanPoints.Length - 1; i++)
        {
            double a = scanPoints[i];
            double b = scanPoints[i + 1];
            double fa = f(a);
            double fb = f(b);

            if (fa * fb <= 0)
            {
                double? root = TryBisection(f, a, b);
                if (root.HasValue)
                {
                    results.Add(root.Value);
                }
            }
        }

        return results;
    }

    private static double? TryBisection(Func<double, double> f, double a, double b, int maxIter = 100, double tol = 1e-12)
    {
        double fa = f(a);
        double fb = f(b);
        if (fa * fb > 0) return null;

        for (int i = 0; i < maxIter; i++)
        {
            double mid = (a + b) / 2.0;
            double fm = f(mid);
            if (Math.Abs(fm) < tol || (b - a) / 2 < tol)
                return mid;

            if (fa * fm <= 0)
            {
                b = mid;
                fb = fm;
            }
            else
            {
                a = mid;
                fa = fm;
            }
        }

        return (a + b) / 2.0;
    }

    public static ExtremaResult PositionExtrema(PolynomialSegment1D segment)
    {
        return FindExtrema(segment, 0);
    }

    public static ExtremaResult VelocityExtrema(PolynomialSegment1D segment)
    {
        return FindExtrema(segment, 1);
    }

    public static ExtremaResult AccelerationExtrema(PolynomialSegment1D segment)
    {
        return FindExtrema(segment, 2);
    }

    public static ExtremaResult JerkExtrema(PolynomialSegment1D segment)
    {
        return FindExtrema(segment, 3);
    }

    public static ExtremaResult SnapExtrema(PolynomialSegment1D segment)
    {
        return FindExtrema(segment, 4);
    }

    public static bool VerifyLimits(PolynomialSegment1D segment, KinematicLimits limits)
    {
        if (limits.HasVelocityLimit)
        {
            var vExtrema = VelocityExtrema(segment);
            if (vExtrema.MaxAbsolute > limits.MaxVelocity!.Value + 1e-10)
                return false;
        }

        if (limits.HasAccelerationLimit)
        {
            var aExtrema = AccelerationExtrema(segment);
            if (aExtrema.MaxAbsolute > limits.MaxAcceleration!.Value + 1e-10)
                return false;
        }

        if (limits.HasJerkLimit)
        {
            var jExtrema = JerkExtrema(segment);
            if (jExtrema.MaxAbsolute > limits.MaxJerk!.Value + 1e-10)
                return false;
        }

        if (limits.HasSnapLimit)
        {
            var sExtrema = SnapExtrema(segment);
            if (sExtrema.MaxAbsolute > limits.MaxSnap!.Value + 1e-10)
                return false;
        }

        return true;
    }
}
