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
/// state), so each is an ordinary segment whose only free parameter is its duration. Durations:
/// <list type="bullet">
/// <item>Rule 1, limits active: each transition takes its shortest feasible duration (independently).</item>
/// <item>Rule 2, no limits, nothing prescribed: mean-velocity rule, see <see cref="MeanVelocityDuration"/>.</item>
/// <item>Rule 3, total duration prescribed: the time left after the stretch is split in proportion to the
/// weights of Rule 1 (limits active) or Rule 2 (no limits).</item>
/// <item>A prescribed transition duration always wins and is only verified against the limits.</item>
/// </list>
/// The constant-speed stretch is fixed by the distance and speed, so limits cannot change it: a speed above the
/// velocity limit is reported, never repaired.
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
    /// Rule 2 (decided 2026-10-17): duration of a transition without limits and without prescribed duration,
    /// T = 2 |Δx| / (|v_b| + v_cv), with Δx the distance covered by the transition and v_b the velocity at its
    /// outer end. If Δx = 0 the rule gives 0, which is not a valid segment: the fallback max(0.5 T_cv, 0.1 s)
    /// is returned instead.
    /// </summary>
    /// <param name="distance">Distance covered by the transition (absolute value is used).</param>
    /// <param name="outerVelocity">Velocity at the outer end of the transition (absolute value is used).</param>
    /// <param name="speed">Constant speed v_cv (magnitude).</param>
    /// <param name="constantVelocityDuration">T_cv, only used by the fallback.</param>
    public static double MeanVelocityDuration(double distance, double outerVelocity, double speed, double constantVelocityDuration)
    {
        double absDistance = Math.Abs(distance);
        if (absDistance <= 0)
            return Math.Max(0.5 * constantVelocityDuration, 0.1);

        return 2.0 * absDistance / (Math.Abs(outerVelocity) + Math.Abs(speed));
    }

    /// <summary>Convenience: the outer states carry only position and velocity (zero acceleration, jerk, snap).</summary>
    public ConstantVelocityTrajectory1D(
        double x0, double xf,
        double v0, double vf,
        double xCvStart, double xCvEnd,
        double vCv,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null)
        : this(new State1D(x0, v0, 0, 0, 0), new State1D(xf, vf, 0, 0, 0),
               xCvStart, xCvEnd, vCv, polynomialDegree, limits, entryDuration, exitDuration, totalDuration)
    {
    }

    /// <param name="startState">State at the start of the whole trajectory (outer end of the entry transition).</param>
    /// <param name="endState">State at the end of the whole trajectory (outer end of the exit transition).</param>
    /// <param name="limits">Active limits (null or none active: no limit is imposed).</param>
    /// <param name="entryDuration">Prescribed entry duration: kept as given, only verified against the limits.</param>
    /// <param name="exitDuration">Prescribed exit duration: kept as given, only verified against the limits.</param>
    /// <param name="totalDuration">
    /// Prescribed total duration (Rule 3). The time left after the stretch, R = total - T_cv, must be positive;
    /// it is split between the transitions in proportion to their shortest feasible durations (limits active) or
    /// their Rule 2 durations (no limits). With one prescribed transition duration the other takes the remainder.
    /// Together with both transition durations it is a conflict.
    /// </param>
    /// <exception cref="ArgumentException">
    /// Degenerate stretch, non-positive prescribed duration, total not above T_cv, a remainder that is not positive,
    /// or a total together with both transition durations.
    /// </exception>
    /// <exception cref="KinematicLimitsViolatedException">
    /// Speed above the velocity limit; a limit violated by a transition; or a total below the minimum feasible one.
    /// </exception>
    /// <exception cref="DurationSearchFailedException">No duration of a transition satisfies the limits.</exception>
    public ConstantVelocityTrajectory1D(
        State1D startState,
        State1D endState,
        double xCvStart, double xCvEnd,
        double vCv,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null)
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
        if (totalDuration is not null && entryDuration is not null && exitDuration is not null)
            throw new ArgumentException("A total duration together with both transition durations is a conflict: prescribe at most two of the three");

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

        double entryRule2 = MeanVelocityDuration(
            cvStartState.Position - startState.Position, startState.Velocity, speed, cvDuration);
        double exitRule2 = MeanVelocityDuration(
            endState.Position - cvEndState.Position, endState.Velocity, speed, cvDuration);

        // Rules 1-3 are shared with the 3D construction (TransitionDurationRules).
        var (entry, exit) = TransitionDurationRules.Resolve(
            cvDuration, active is not null, entryDuration, exitDuration, totalDuration,
            () => Weight("entry transition", polynomialDegree, startState, cvStartState, active, entryRule2).Duration,
            () => Weight("exit transition", polynomialDegree, cvEndState, endState, active, exitRule2).Duration);

        EntryTransition = new PolynomialSegment1D(polynomialDegree, startState, cvStartState, entry);
        ExitTransition = new PolynomialSegment1D(polynomialDegree, cvEndState, endState, exit);

        if (active is not null)
        {
            // Prescribed durations are only verified; computed ones are re-checked as a safety net.
            Verify(EntryTransition, active, "entry transition");
            Verify(ExitTransition, active, "exit transition");
        }
    }


    /// <summary>
    /// The Rule 1 duration of a transition (shortest feasible) when limits are active, or the Rule 2 duration
    /// when they are not or when the limits do not determine the transition.
    /// </summary>
    private static (double Duration, bool Determined) Weight(
        string label, int degree, State1D from, State1D to, KinematicLimits? limits, double rule2)
    {
        if (limits is null)
            return (rule2, false);

        try
        {
            double? shortest = ShortestFeasibleDuration.Find(degree, from, to, limits);
            return (shortest ?? rule2, shortest is not null);
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
