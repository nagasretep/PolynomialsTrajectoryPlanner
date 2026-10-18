using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.ConstantVelocity;

/// <summary>
/// 3D constant-velocity trajectory on a <b>straight</b> stretch: entry transition (polynomial) + stretch at constant
/// velocity vector + exit transition (polynomial). Straight-chord construction of
/// <c>theory/ConstantVelocitySegment.md</c> ("3D Straight-Chord Alternative", "3D Transition Targets"), which coincides
/// exactly with the path-preserving construction whenever the selected subpath is a straight line.
/// </summary>
/// <remarks>
/// The stretch goes from <c>p_cv_start</c> to <c>p_cv_end</c> at speed v_cv: T_cv = ‖Δp‖ / v_cv and the velocity vector is
/// v_cv·n, n = Δp/‖Δp‖. The transitions are ordinary 3D segments with vector boundary states (outer end given, inner end
/// (p, v_cv·n, 0, 0, 0)) and <b>one common duration</b> for the three axes. Durations are those of 1D
/// (<see cref="TransitionDurationRules"/>): Rule 1 with limits (shortest feasible, the maximum over the axes), Rule 2
/// without (mean-velocity rule with Euclidean distance and speeds), Rule 3 for a prescribed total; a prescribed transition
/// duration is only verified. The limits are per axis, and the constant speed is checked per axis too: every
/// |v_cv·n_i| must be within the velocity limit, otherwise the failure is explicit (the stretch cannot be repaired).
/// A curved subpath is not supported yet: see <see cref="OnSegment"/>.
/// </remarks>
public class ConstantVelocityTrajectory3D
{
    public PolynomialSegment3D EntryTransition { get; }
    public LinearConstantVelocitySegment3D ConstantVelocity { get; }
    public PolynomialSegment3D ExitTransition { get; }

    public double TotalDuration => EntryTransition.Duration + ConstantVelocity.Duration + ExitTransition.Duration;

    /// <summary>The constant speed (magnitude of the velocity vector).</summary>
    public double ConstantSpeed { get; }

    /// <summary>Unit vector from the start to the end of the stretch.</summary>
    public Point3D Direction { get; }

    /// <param name="startState">State at the start of the whole trajectory (outer end of the entry transition).</param>
    /// <param name="endState">State at the end of the whole trajectory (outer end of the exit transition).</param>
    /// <param name="cvStart">First point of the constant-velocity stretch.</param>
    /// <param name="cvEnd">Last point of the stretch (the direction is cvStart → cvEnd).</param>
    /// <param name="speed">Speed v_cv &gt; 0.</param>
    /// <param name="limits">Per-axis limits (null or none active: no limit is imposed).</param>
    /// <param name="entryDuration">Prescribed entry duration (kept, only verified).</param>
    /// <param name="exitDuration">Prescribed exit duration (kept, only verified).</param>
    /// <param name="totalDuration">Prescribed total duration (Rule 3), as in 1D.</param>
    /// <exception cref="ArgumentException">Degenerate stretch or speed, non-positive prescribed duration, total not above T_cv, a remainder that is not positive, or a total together with both transition durations.</exception>
    /// <exception cref="KinematicLimitsViolatedException">Speed above the per-axis velocity limit, a limit violated by a transition, or a total below the minimum feasible one.</exception>
    /// <exception cref="DurationSearchFailedException">No duration of a transition satisfies the limits.</exception>
    public ConstantVelocityTrajectory3D(
        State3D startState,
        State3D endState,
        Point3D cvStart,
        Point3D cvEnd,
        double speed,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null)
    {
        if (!(speed > 0) || double.IsInfinity(speed))
            throw new ArgumentException("Invalid constant velocity: the speed must be positive and finite", nameof(speed));

        Point3D delta = cvEnd - cvStart;
        double length = delta.Length;
        if (!(length > 0) || double.IsInfinity(length))
            throw new ArgumentException("Invalid constant velocity stretch: start and end points are equal");
        if (entryDuration is <= 0 || exitDuration is <= 0)
            throw new ArgumentException("Prescribed transition durations must be positive");
        if (totalDuration is not null && entryDuration is not null && exitDuration is not null)
            throw new ArgumentException("A total duration together with both transition durations is a conflict: prescribe at most two of the three");

        double cvDuration = length / speed;
        ConstantSpeed = speed;
        Direction = delta / length;
        Point3D velocity = Direction * speed;

        var cvStartState = new State3D(cvStart, velocity, Point3D.Zero, Point3D.Zero, Point3D.Zero);
        var cvEndState = new State3D(cvEnd, velocity, Point3D.Zero, Point3D.Zero, Point3D.Zero);
        ConstantVelocity = new LinearConstantVelocitySegment3D(cvStartState, cvEndState, cvDuration);

        KinematicLimits? active = limits is not null && limits.HasAnyLimit ? limits : null;

        // The stretch cannot be changed by the limits: report it, never repair it. Per-axis check.
        if (active is not null && active.HasVelocityLimit)
        {
            string[] axes = ["x", "y", "z"];
            for (int i = 0; i < 3; i++)
            {
                if (Math.Abs(velocity[i]) > active.MaxVelocity!.Value + 1e-10)
                {
                    throw new KinematicLimitsViolatedException(
                        $"The constant velocity component along {axes[i]} ({Math.Abs(velocity[i]):G6}, speed {speed:G6}) exceeds the " +
                        $"velocity limit {active.MaxVelocity.Value:G6}: no transition can repair it.");
                }
            }
        }

        double entryRule2 = ConstantVelocityTrajectory1D.MeanVelocityDuration(
            (cvStart - startState.Position).Length, startState.Velocity.Length, speed, cvDuration);
        double exitRule2 = ConstantVelocityTrajectory1D.MeanVelocityDuration(
            (endState.Position - cvEnd).Length, endState.Velocity.Length, speed, cvDuration);

        var (entry, exit) = TransitionDurationRules.Resolve(
            cvDuration, active is not null, entryDuration, exitDuration, totalDuration,
            () => Weight("entry transition", polynomialDegree, startState, cvStartState, active, entryRule2),
            () => Weight("exit transition", polynomialDegree, cvEndState, endState, active, exitRule2));

        EntryTransition = new PolynomialSegment3D(polynomialDegree, startState, cvStartState, entry);
        ExitTransition = new PolynomialSegment3D(polynomialDegree, cvEndState, endState, exit);

        if (active is not null)
        {
            Verify(EntryTransition, active, "entry transition");
            Verify(ExitTransition, active, "exit transition");
        }
    }

    /// <summary>
    /// Constant velocity along the part of a given 3D segment between two of its parameters (τ_start &lt; τ_end). The
    /// subpath must be a straight line: if the segment's path between the two points is not collinear (within
    /// <paramref name="relativeTolerance"/> times the chord length) a <see cref="NotSupportedException"/> is raised, because
    /// the curved multi-axis case (arc-length reparameterization, <c>theory/ConstantVelocitySegment.md</c>) is not
    /// implemented yet.
    /// </summary>
    /// <exception cref="NotSupportedException">The subpath is curved (or doubles back along the line).</exception>
    public static ConstantVelocityTrajectory3D OnSegment(
        PolynomialSegment3D segment,
        double tauStart,
        double tauEnd,
        double speed,
        State3D startState,
        State3D endState,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null,
        double relativeTolerance = StraightSubpath.DefaultRelativeTolerance)
    {
        StraightSubpath.Require(segment, tauStart, tauEnd, relativeTolerance);

        Point3D start = segment.EvaluatePosition(tauStart * segment.Duration);
        Point3D end = segment.EvaluatePosition(tauEnd * segment.Duration);
        return new ConstantVelocityTrajectory3D(
            startState, endState, start, end, speed, polynomialDegree, limits, entryDuration, exitDuration, totalDuration);
    }

    /// <summary>Same, with the two stretch points given as absolute points that must lie on the segment (see <see cref="BlendBuilder3D.TauForPoint"/>).</summary>
    public static ConstantVelocityTrajectory3D OnSegment(
        PolynomialSegment3D segment,
        Point3D stretchStart,
        Point3D stretchEnd,
        double speed,
        State3D startState,
        State3D endState,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null,
        double relativeTolerance = StraightSubpath.DefaultRelativeTolerance)
    {
        // The parameters are used only to check the subpath; the stretch itself starts and ends at the given points.
        double tauStart = BlendBuilder3D.TauForPoint(segment, stretchStart, "constant-velocity start");
        double tauEnd = BlendBuilder3D.TauForPoint(segment, stretchEnd, "constant-velocity end");
        StraightSubpath.Require(segment, tauStart, tauEnd, relativeTolerance);

        return new ConstantVelocityTrajectory3D(
            startState, endState, stretchStart, stretchEnd, speed, polynomialDegree, limits, entryDuration, exitDuration, totalDuration);
    }

    private static double Weight(
        string label, int degree, State3D from, State3D to, KinematicLimits? limits, double rule2)
    {
        if (limits is null)
            return rule2;

        try
        {
            return ShortestFeasibleDuration.Find(
                degree, BlendBuilder3D.Components(from), BlendBuilder3D.Components(to), limits) ?? rule2;
        }
        catch (DurationSearchFailedException ex)
        {
            throw new DurationSearchFailedException($"The {label}: {ex.Message}", ex.LastDurationTried);
        }
    }

    private static void Verify(PolynomialSegment3D segment, KinematicLimits limits, string label)
    {
        if (!ExtremaAnalyzer.VerifyLimits(segment, limits))
        {
            throw new KinematicLimitsViolatedException(
                $"The {label} (duration {segment.Duration:G6} s) violates the active kinematic limits.");
        }
    }

    public Point3D EvaluatePosition(double t) => EvaluateState(t).Position;

    public State3D EvaluateState(double t)
    {
        if (t < EntryTransition.Duration)
            return EntryTransition.EvaluateState(t);

        t -= EntryTransition.Duration;
        if (t < ConstantVelocity.Duration)
            return ConstantVelocity.EvaluateState(t);

        t -= ConstantVelocity.Duration;
        return ExitTransition.EvaluateState(Math.Min(t, ExitTransition.Duration));
    }

    public IEnumerable<PolynomialSegment3D> GetAllSegments()
    {
        yield return EntryTransition;
        foreach (var segment in ConstantVelocity.ToPolynomialSegments())
            yield return segment;
        yield return ExitTransition;
    }
}

/// <summary>The stretch at constant velocity vector: position linear in time.</summary>
public class LinearConstantVelocitySegment3D
{
    public State3D StartState { get; }
    public State3D EndState { get; }
    public double Duration { get; }

    public LinearConstantVelocitySegment3D(State3D startState, State3D endState, double duration)
    {
        StartState = startState;
        EndState = endState;
        Duration = duration;
    }

    public State3D EvaluateState(double t)
    {
        double tau = t / Duration;
        Point3D position = StartState.Position + (EndState.Position - StartState.Position) * tau;
        return new State3D(position, StartState.Velocity, Point3D.Zero, Point3D.Zero, Point3D.Zero);
    }

    /// <summary>The stretch as a cubic (exactly linear: equal end velocities and the chord as displacement).</summary>
    public IEnumerable<PolynomialSegment3D> ToPolynomialSegments()
    {
        yield return new PolynomialSegment3D(3, StartState, EndState, Duration);
    }
}

/// <summary>Detection of a straight subpath of a 3D segment.</summary>
public static class StraightSubpath
{
    /// <summary>Maximum deviation from the chord, relative to the chord length.</summary>
    public const double DefaultRelativeTolerance = 1e-9;

    private const int Samples = 200;

    /// <summary>
    /// True if the path of <paramref name="segment"/> between the two parameters is a straight line traversed from the first
    /// point to the second without doubling back: every sampled point (200 uniform parameters, ends included) is within
    /// <paramref name="relativeTolerance"/> × chord length of the chord and its projection stays inside it.
    /// </summary>
    public static bool IsStraight(PolynomialSegment3D segment, double tauStart, double tauEnd, double relativeTolerance, out string reason)
    {
        Validate(tauStart, tauEnd);

        double T = segment.Duration;
        Point3D p0 = segment.EvaluatePosition(tauStart * T);
        Point3D chord = segment.EvaluatePosition(tauEnd * T) - p0;
        double length = chord.Length;
        if (!(length > 0))
            throw new ArgumentException("Degenerate stretch: the two points of the subpath coincide.");

        double tolerance = relativeTolerance * length;
        double worstDeviation = 0;
        double worstOvershoot = 0;
        for (int i = 0; i <= Samples; i++)
        {
            double tau = tauStart + (tauEnd - tauStart) * i / Samples;
            Point3D w = segment.EvaluatePosition(tau * T) - p0;
            double s = w.Dot(chord) / (length * length);
            double deviation = (w - chord * s).Length;
            worstDeviation = Math.Max(worstDeviation, deviation);
            worstOvershoot = Math.Max(worstOvershoot, Math.Max(-s, s - 1.0) * length);
        }

        if (worstDeviation > tolerance)
        {
            reason = $"the path deviates from the chord by up to {worstDeviation:G6} (chord length {length:G6}, tolerance {tolerance:G3})";
            return false;
        }

        if (worstOvershoot > tolerance)
        {
            reason = $"the path leaves the chord by {worstOvershoot:G6} along the line (it does not run monotonically between the two points)";
            return false;
        }

        reason = string.Empty;
        return true;
    }

    /// <exception cref="NotSupportedException">The subpath is not straight.</exception>
    public static void Require(PolynomialSegment3D segment, double tauStart, double tauEnd, double relativeTolerance = DefaultRelativeTolerance)
    {
        if (!IsStraight(segment, tauStart, tauEnd, relativeTolerance, out string reason))
        {
            throw new NotSupportedException(
                "Constant velocity along a curved subpath is not implemented yet: only a straight subpath is supported " +
                $"(arc-length reparameterization is still to be written). Here {reason}.");
        }
    }

    private static void Validate(double tauStart, double tauEnd)
    {
        if (tauStart < 0 || tauEnd > 1 || !(tauStart < tauEnd))
            throw new ArgumentException($"The stretch parameters must satisfy 0 <= tau_start < tau_end <= 1, got {tauStart:G6} and {tauEnd:G6}.");
    }
}
