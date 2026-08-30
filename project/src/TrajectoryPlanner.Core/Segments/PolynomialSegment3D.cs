using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Polynomials;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Segments;

public class PolynomialSegment3D
{
    public int Degree { get; }
    public double Duration { get; }
    public State3D StartState { get; }
    public State3D EndState { get; }

    public PolynomialSegment1D XAxis { get; }
    public PolynomialSegment1D YAxis { get; }
    public PolynomialSegment1D ZAxis { get; }

    public PolynomialSegment3D(int degree, State3D startState, State3D endState, double duration)
    {
        if (duration <= 0)
            throw new ArgumentException("Duration must be positive", nameof(duration));

        Degree = degree;
        Duration = duration;
        StartState = startState;
        EndState = endState;

        XAxis = new PolynomialSegment1D(degree, startState.GetComponent(0), endState.GetComponent(0), duration);
        YAxis = new PolynomialSegment1D(degree, startState.GetComponent(1), endState.GetComponent(1), duration);
        ZAxis = new PolynomialSegment1D(degree, startState.GetComponent(2), endState.GetComponent(2), duration);
    }

    public double TauFromAbsoluteTime(double t)
    {
        if (t < 0 || t > Duration)
            throw new ArgumentOutOfRangeException(nameof(t), $"Time {t} is outside segment duration [0, {Duration}]");
        return t / Duration;
    }

    public Point3D EvaluatePosition(double t)
    {
        return new Point3D(
            XAxis.EvaluatePosition(t),
            YAxis.EvaluatePosition(t),
            ZAxis.EvaluatePosition(t));
    }

    public Point3D EvaluateDerivative(double t, int order)
    {
        return new Point3D(
            XAxis.EvaluateDerivative(t, order),
            YAxis.EvaluateDerivative(t, order),
            ZAxis.EvaluateDerivative(t, order));
    }

    public State3D EvaluateState(double t)
    {
        var xState = XAxis.EvaluateState(t);
        var yState = YAxis.EvaluateState(t);
        var zState = ZAxis.EvaluateState(t);

        return new State3D(
            new Point3D(xState.Position, yState.Position, zState.Position),
            new Point3D(xState.Velocity, yState.Velocity, zState.Velocity),
            new Point3D(xState.Acceleration, yState.Acceleration, zState.Acceleration),
            new Point3D(xState.Jerk, yState.Jerk, zState.Jerk),
            new Point3D(xState.Snap, yState.Snap, zState.Snap));
    }

    public Point3D StartPoint => StartState.Position;
    public Point3D EndPoint => EndState.Position;
}
