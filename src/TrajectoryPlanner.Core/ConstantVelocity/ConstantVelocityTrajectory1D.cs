using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.ConstantVelocity;

/// <summary>
/// 1D constant-velocity trajectory: entry transition (polynomial) + constant-speed stretch + exit
/// transition (polynomial). See <c>theory/ConstantVelocitySegment.md</c>,
/// "Durations Of The Entry And Exit Transitions In 1D".
/// </summary>
/// <remarks>
/// The two transitions have both boundary states fixed (outer end given, inner end = constant-velocity
/// state), so each is an ordinary segment whose only free parameter is its duration; the two durations do not
/// influence each other and are determined independently. The constant-speed stretch is fixed by the
/// distance and speed, so limits cannot change it: a speed above the velocity limit is reported, never repaired.
/// </remarks>
public class ConstantVelocityTrajectory1D
{
    public PolynomialSegment1D EntryTransition { get; }
    public LinearConstantVelocitySegment ConstantVelocity { get; }
    public PolynomialSegment1D ExitTransition { get; }

    public double TotalDuration => EntryTransition.Duration + ConstantVelocity.Duration + ExitTransition.Duration;

    /// <summary>The constant speed (magnitude); the direction follows xCvStart -> xCvEnd.</summary>
    public double ConstantSpeed { get; }

    /// <summary>
    /// Provisional rule (theory marks it open): transition duration when no limit is active and none is
    /// prescribed, and fallback when the limits do not determine a transition (coincident boundary states):
    /// max(0.5 * T_cv, 0.1 s). Kept as it was before request 0007.
    /// </summary>
    public static double DefaultTransitionDuration(double constantVelocityDuration) =>
        Math.Max(0.5 * constantVelocityDuration, 0.1);

    /// <summary>Convenience: the outer states carry only position and velocity (zero acceleration, jerk, snap).</summary>
    public ConstantVelocityTrajectory1D(
        double x0, double xf,
        double v0, double vf,
        double xCvStart, double xCvEnd,
        double vCv,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null)
        : this(new State1D(x0, v0, 0, 0, 0), new State1D(xf, vf, 0, 0, 0),
               xCvStart, xCvEnd, vCv, polynomialDegree, limits, entryDuration, exitDuration)
    {
    }

    /// <param name="startState">State at the start of the whole trajectory (outer end of the entry transition).</param>
    /// <param name="endState">State at the end of the whole trajectory (outer end of the exit transition).</param>
    /// <param name="limits">Active limits (null or none active: no limit is imposed).</param>
    /// <param name="entryDuration">Prescribed entry duration: kept as given, only verified against the limits.</param>
    /// <param name="exitDuration">Prescribed exit duration: kept as given, only verified against the limits.</param>
    /// <exception cref="ArgumentException">Degenerate stretch (equal positions, zero speed) or non-positive prescribed duration.</exception>
    /// <exception cref="KinematicLimitsViolatedException">Speed above the velocity limit, or a limit violated by a transition.</exception>
    /// <exception cref="DurationSearchFailedException">No duration of a transition satisfies the limits.</exception>
    public ConstantVelocityTrajectory1D(
        State1D startState,
        State1D endState,
        double xCvStart, double xCvEnd,
        double vCv,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null)
    {
        double speed = Math.Abs(vCv);
        if (!(speed > 0) || double.IsInfinity(speed))
            throw new ArgumentException("Invalid constant velocity interval: the speed must be non-zero and finite", nameof(vCv));

        double directionSign = xCvEnd >= xCvStart ? 1.0 : -1.0;
        double cvDuration = Math.Abs(xCvEnd - xCvStart) / speed;

        if (!(cvDuration > 0))
            throw new ArgumentException("Invalid constant velocity interval: start and end positions are equal or speed is zero");
        if (entryDuration is <= 0 || exitDuration is <= 0)
            throw new ArgumentException("Prescribed transition durations must be positive");

        ConstantSpeed = speed;
        var cvStartState = new State1D(xCvStart, directionSign * speed, 0, 0, 0);
        var cvEndState = new State1D(xCvEnd, directionSign * speed, 0, 0, 0);
        ConstantVelocity = new LinearConstantVelocitySegment(cvStartState, cvEndState, cvDuration);

        KinematicLimits? active = limits is not null &&
            (limits.HasVelocityLimit || limits.HasAccelerationLimit || limits.HasJerkLimit || limits.HasSnapLimit)
            ? limits
            : null;

        // The stretch cannot be changed by the limits: report it, never repair it.
        if (active is not null && active.HasVelocityLimit && speed > active.MaxVelocity!.Value + 1e-10)
        {
            throw new KinematicLimitsViolatedException(
                $"The constant speed {speed:G6} exceeds the velocity limit {active.MaxVelocity.Value:G6}: " +
                "no transition can repair it.");
        }

        double entry = entryDuration ?? TransitionDuration(
            "entry transition", polynomialDegree, startState, cvStartState, active, cvDuration);
        double exit = exitDuration ?? TransitionDuration(
            "exit transition", polynomialDegree, cvEndState, endState, active, cvDuration);

        EntryTransition = new PolynomialSegment1D(polynomialDegree, startState, cvStartState, entry);
        ExitTransition = new PolynomialSegment1D(polynomialDegree, cvEndState, endState, exit);

        if (active is not null)
        {
            // Prescribed durations are only verified; computed ones are re-checked as a safety net.
            Verify(EntryTransition, active, "entry transition");
            Verify(ExitTransition, active, "exit transition");
        }
    }

    private static double TransitionDuration(
        string label, int degree, State1D from, State1D to, KinematicLimits? limits, double cvDuration)
    {
        if (limits is null)
            return DefaultTransitionDuration(cvDuration);

        try
        {
            return ShortestFeasibleDuration.Find(degree, from, to, limits) ?? DefaultTransitionDuration(cvDuration);
        }
        catch (DurationSearchFailedException ex)
        {
            throw new DurationSearchFailedException($"The {label}: {ex.Message}", ex.LastDurationTried);
        }
    }

    private static void Verify(PolynomialSegment1D segment, KinematicLimits limits, string label)
    {
        if (!ExtremaAnalyzer.VerifyLimits(segment, limits))
        {
            throw new KinematicLimitsViolatedException(
                $"The {label} (duration {segment.Duration:G6} s) violates the active kinematic limits.");
        }
    }

    public double EvaluatePosition(double t) => EvaluateState(t).Position;

    public State1D EvaluateState(double t)
    {
        if (t < EntryTransition.Duration)
            return EntryTransition.EvaluateState(t);

        t -= EntryTransition.Duration;
        if (t < ConstantVelocity.Duration)
            return ConstantVelocity.EvaluateState(t);

        t -= ConstantVelocity.Duration;
        // Rounding of the subtractions can leave t a few ulps beyond the exit duration at the very end.
        return ExitTransition.EvaluateState(Math.Min(t, ExitTransition.Duration));
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
