using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.ConstantVelocity;

public class ConstantVelocityTrajectory1D
{
    public PolynomialSegment1D EntryTransition { get; }
    public LinearConstantVelocitySegment ConstantVelocity { get; }
    public PolynomialSegment1D ExitTransition { get; }

    public double TotalDuration => EntryTransition.Duration + ConstantVelocity.Duration + ExitTransition.Duration;
    public double ConstantSpeed { get; }

    public ConstantVelocityTrajectory1D(
        double x0, double xf,
        double v0, double vf,
        double xCvStart, double xCvEnd,
        double vCv,
        int polynomialDegree = 9,
        KinematicLimits? limits = null)
    {
        ConstantSpeed = vCv;
        double directionSign = xCvEnd >= xCvStart ? 1.0 : -1.0;
        double cvDuration = Math.Abs(xCvEnd - xCvStart) / Math.Abs(vCv);

        if (cvDuration <= 0)
            throw new ArgumentException("Invalid constant velocity interval: start and end positions are equal or speed is zero");

        var cvStartState = new State1D(xCvStart, directionSign * vCv, 0, 0, 0);
        var cvEndState = new State1D(xCvEnd, directionSign * vCv, 0, 0, 0);

        ConstantVelocity = new LinearConstantVelocitySegment(cvStartState, cvEndState, cvDuration);

        var initState = new State1D(x0, v0, 0, 0, 0);
        limits ??= KinematicLimits.None;

        if (limits != KinematicLimits.None)
        {
            double entryGuess = Math.Max(cvDuration * 0.5, Math.Abs(xCvStart - x0) / (Math.Abs(v0) + Math.Abs(vCv) + 1e-6) * 2.0);
            EntryTransition = DurationSearcher.CreateSegmentWithAutoDuration(
                polynomialDegree, initState, cvStartState, limits, entryGuess);

            double exitGuess = Math.Max(cvDuration * 0.5, Math.Abs(xf - xCvEnd) / (Math.Abs(vCv) + Math.Abs(vf) + 1e-6) * 2.0);
            ExitTransition = DurationSearcher.CreateSegmentWithAutoDuration(
                polynomialDegree, cvEndState, new State1D(xf, vf, 0, 0, 0), limits, exitGuess);
        }
        else
        {
            double entryDur = Math.Max(0.5 * cvDuration, 0.1);
            EntryTransition = new PolynomialSegment1D(polynomialDegree, initState, cvStartState, entryDur);

            double exitDur = Math.Max(0.5 * cvDuration, 0.1);
            ExitTransition = new PolynomialSegment1D(polynomialDegree, cvEndState, new State1D(xf, vf, 0, 0, 0), exitDur);
        }
    }

    public double EvaluatePosition(double t)
    {
        if (t < EntryTransition.Duration)
            return EntryTransition.EvaluatePosition(t);

        t -= EntryTransition.Duration;
        if (t < ConstantVelocity.Duration)
            return ConstantVelocity.EvaluatePosition(t);

        t -= ConstantVelocity.Duration;
        return ExitTransition.EvaluatePosition(t);
    }

    public State1D EvaluateState(double t)
    {
        if (t < EntryTransition.Duration)
            return EntryTransition.EvaluateState(t);

        t -= EntryTransition.Duration;
        if (t < ConstantVelocity.Duration)
            return ConstantVelocity.EvaluateState(t);

        t -= ConstantVelocity.Duration;
        return ExitTransition.EvaluateState(t);
    }

    public IEnumerable<PolynomialSegment1D> GetAllSegments()
    {
        yield return EntryTransition;
        foreach (var seg in ConstantVelocity.ToPolynomialSegments())
            yield return seg;
        yield return ExitTransition;
    }
}

public class LinearConstantVelocitySegment
{
    public State1D StartState { get; }
    public State1D EndState { get; }
    public double Duration { get; }

    public LinearConstantVelocitySegment(State1D startState, State1D endState, double duration)
    {
        StartState = startState;
        EndState = endState;
        Duration = duration;
    }

    public double EvaluatePosition(double t)
    {
        double tau = t / Duration;
        return StartState.Position + tau * (EndState.Position - StartState.Position);
    }

    public State1D EvaluateState(double t)
    {
        double pos = EvaluatePosition(t);
        return new State1D(pos, StartState.Velocity, 0, 0, 0);
    }

    public IEnumerable<PolynomialSegment1D> ToPolynomialSegments()
    {
        yield return PolynomialSegment1D.CreateDegree3(
            StartState.Position, EndState.Position,
            StartState.Velocity, StartState.Velocity,
            Duration);
    }
}
