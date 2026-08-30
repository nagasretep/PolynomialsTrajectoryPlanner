using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Constraints;

public static class DurationSearcher
{
    public static double FindValidDuration(
        int degree,
        State1D startState,
        State1D endState,
        KinematicLimits limits,
        double initialGuess = 1.0,
        double maxIterations = 50,
        double tolerance = 1e-6)
    {
        double duration = Math.Max(initialGuess, 1e-3);
        double stepFactor = 2.0;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            try
            {
                var segment = new PolynomialSegment1D(degree, startState, endState, duration);
                bool valid = ExtremaAnalyzer.VerifyLimits(segment, limits);

                if (valid)
                {
                    if (iter == 0)
                        return duration;

                    double lowerBound = duration / stepFactor;
                    double upperBound = duration;

                    for (int binaryIter = 0; binaryIter < 30; binaryIter++)
                    {
                        double mid = (lowerBound + upperBound) / 2.0;
                        var midSeg = new PolynomialSegment1D(degree, startState, endState, mid);
                        if (ExtremaAnalyzer.VerifyLimits(midSeg, limits))
                        {
                            upperBound = mid;
                        }
                        else
                        {
                            lowerBound = mid;
                        }
                    }

                    return upperBound;
                }

                duration *= stepFactor;
            }
            catch
            {
                duration *= stepFactor;
            }
        }

        return duration;
    }

    public static PolynomialSegment1D CreateSegmentWithAutoDuration(
        int degree,
        State1D startState,
        State1D endState,
        KinematicLimits limits,
        double initialGuess = 1.0)
    {
        double T = FindValidDuration(degree, startState, endState, limits, initialGuess);
        return new PolynomialSegment1D(degree, startState, endState, T);
    }
}
