using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Segments;

public class PolynomialSegment1D
{
    public int Degree { get; }
    public double Duration { get; }
    public State1D StartState { get; }
    public State1D EndState { get; }
    public PolynomialCoefficients Coefficients { get; }

    public PolynomialSegment1D(int degree, State1D startState, State1D endState, double duration)
    {
        if (duration <= 0)
            throw new ArgumentException("Duration must be positive", nameof(duration));

        Degree = degree;
        Duration = duration;
        StartState = startState;
        EndState = endState;
        Coefficients = PolynomialCoefficientSolver.SolveNormalized(degree, startState, endState, duration);
    }

    public double TauFromAbsoluteTime(double t)
    {
        if (t < 0 || t > Duration)
            throw new ArgumentOutOfRangeException(nameof(t), $"Time {t} is outside segment duration [0, {Duration}]");
        return t / Duration;
    }

    public double EvaluatePosition(double t)
    {
        double tau = TauFromAbsoluteTime(t);
        return PolynomialEvaluator.Evaluate(tau, Coefficients);
    }

    public double EvaluateDerivative(double t, int order)
    {
        double tau = TauFromAbsoluteTime(t);
        return PolynomialEvaluator.EvaluateDerivative(tau, Coefficients, order, Duration);
    }

    public State1D EvaluateState(double t)
    {
        double tau = TauFromAbsoluteTime(t);
        var (pos, vel, acc, jerk, snap) = PolynomialEvaluator.EvaluateState1D(tau, Coefficients, Duration);
        return new State1D(pos, vel, acc, jerk, snap);
    }

    public Point1D StartPoint => new(StartState.Position);
    public Point1D EndPoint => new(EndState.Position);

    public static PolynomialSegment1D CreateDegree3(double x0, double xf, double v0, double vf, double T)
    {
        var start = new State1D(x0, v0, 0, 0, 0);
        var end = new State1D(xf, vf, 0, 0, 0);
        return new PolynomialSegment1D(3, start, end, T);
    }

    public static PolynomialSegment1D CreateDegree5(double x0, double xf, double v0, double vf, double a0, double af, double T)
    {
        var start = new State1D(x0, v0, a0, 0, 0);
        var end = new State1D(xf, vf, af, 0, 0);
        return new PolynomialSegment1D(5, start, end, T);
    }

    public static PolynomialSegment1D CreateDegree7(double x0, double xf, double v0, double vf, double a0, double af, double j0, double jf, double T)
    {
        var start = new State1D(x0, v0, a0, j0, 0);
        var end = new State1D(xf, vf, af, jf, 0);
        return new PolynomialSegment1D(7, start, end, T);
    }

    public static PolynomialSegment1D CreateDegree9(double x0, double xf, double v0, double vf, double a0, double af, double j0, double jf, double s0, double sf, double T)
    {
        var start = new State1D(x0, v0, a0, j0, s0);
        var end = new State1D(xf, vf, af, jf, sf);
        return new PolynomialSegment1D(9, start, end, T);
    }
}
