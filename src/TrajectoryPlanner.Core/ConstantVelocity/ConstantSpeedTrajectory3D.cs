using TrajectoryPlanner.Core.ArcLength;
using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.ConstantVelocity;

/// <summary>
/// The constant-speed stretch along a (possibly curved) subpath of a polynomial 3D segment: path-preserving, speed
/// magnitude exactly v_cv (<c>theory/ConstantVelocitySegment.md</c>, "Curved 3D Constant-Speed Stretch"). It is not a
/// polynomial in time: the position is p(t*) with t* = ℓ⁻¹(v_cv t), and the time derivative of order k is v_cv^k D^k p(t*).
/// </summary>
public sealed class CurvedConstantSpeedStretch3D : ISegment3D
{
    /// <summary>Intervals of the grid on which the peaks of D^k p are searched.</summary>
    private const int PeakGrid = 1000;

    private double[][]? _componentPeaks;

    public ArcLengthPath Subpath { get; }

    public double Speed { get; }

    /// <summary>T_cv = L / v_cv.</summary>
    public double Duration => Subpath.Length / Speed;

    /// <summary>The full boundary state (p, v D p, v² D²p, v³ D³p, v⁴ D⁴p) at the start of the stretch.</summary>
    public State3D StartState { get; }

    public State3D EndState { get; }

    public CurvedConstantSpeedStretch3D(ArcLengthPath subpath, double speed)
    {
        if (!(speed > 0) || double.IsInfinity(speed))
            throw new ArgumentException("Invalid constant speed: it must be positive and finite", nameof(speed));

        Subpath = subpath;
        Speed = speed;
        StartState = StateAtParameter(subpath.StartTime);
        EndState = StateAtParameter(subpath.EndTime);
    }

    /// <summary>The state of the stretch at the time t ∈ [0, T_cv] (clamped at the ends).</summary>
    public State3D EvaluateState(double t) => StateAtParameter(Subpath.TimeAtLength(Speed * Math.Clamp(t, 0, Duration)));

    /// <summary>Position (order 0) or time derivative of the given order (1 .. 4) at the time t.</summary>
    public Point3D EvaluateDerivative(double t, int order) => EvaluateState(t).GetDerivative(order);

    public Point3D EvaluatePosition(double t) => EvaluateState(t).Position;

    private State3D StateAtParameter(double segmentTime)
    {
        var (_, d) = ArcLengthDerivatives.Compute(Subpath.Path, segmentTime);
        return new State3D(
            Subpath.Path.EvaluatePosition(segmentTime),
            d[0] * Speed,
            d[1] * (Speed * Speed),
            d[2] * (Speed * Speed * Speed),
            d[3] * (Speed * Speed * Speed * Speed));
    }

    /// <summary>
    /// Peak over the subpath of |D^k p| for each component: <c>[k - 1][axis]</c>, k = 1 .. 4 (the theory's P_k per
    /// component). A grid of 1000 intervals, then golden-section refinement of the best sample of each component.
    /// </summary>
    public double[][] ComponentPeaks()
    {
        if (_componentPeaks is not null)
            return _componentPeaks;

        double t0 = Subpath.StartTime, t1 = Subpath.EndTime;
        var peaks = new double[ArcLengthDerivatives.MaxOrder][];
        var best = new double[ArcLengthDerivatives.MaxOrder][];
        var bestIndex = new int[ArcLengthDerivatives.MaxOrder][];
        for (int k = 0; k < peaks.Length; k++)
        {
            peaks[k] = new double[3];
            best[k] = new double[3];
            bestIndex[k] = new int[3];
        }

        for (int i = 0; i <= PeakGrid; i++)
        {
            double t = t0 + (t1 - t0) * i / PeakGrid;
            var (_, d) = ArcLengthDerivatives.Compute(Subpath.Path, t);
            for (int k = 0; k < peaks.Length; k++)
                for (int axis = 0; axis < 3; axis++)
                {
                    double value = Math.Abs(d[k][axis]);
                    if (value > best[k][axis])
                    {
                        best[k][axis] = value;
                        bestIndex[k][axis] = i;
                    }
                }
        }

        const double invPhi = 0.6180339887498949;
        for (int k = 0; k < peaks.Length; k++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                double Value(double t) => Math.Abs(ArcLengthDerivatives.Compute(Subpath.Path, t).Derivatives[k][axis]);

                int index = bestIndex[k][axis];
                double lo = t0 + (t1 - t0) * Math.Max(index - 1, 0) / PeakGrid;
                double hi = t0 + (t1 - t0) * Math.Min(index + 1, PeakGrid) / PeakGrid;
                double peak = best[k][axis];
                for (int iteration = 0; iteration < 60; iteration++)
                {
                    double c = hi - invPhi * (hi - lo), d = lo + invPhi * (hi - lo);
                    double fc = Value(c), fd = Value(d);
                    peak = Math.Max(peak, Math.Max(fc, fd));
                    if (fc > fd) hi = d; else lo = c;
                }

                peaks[k][axis] = peak;
            }
        }

        _componentPeaks = peaks;
        return peaks;
    }

    /// <summary>
    /// Minimum and maximum, per axis, of the position (order 0) or of the time derivative of the given order (1 .. 4)
    /// over the stretch, each with the time (from the start of the stretch) where it occurs. The extrema are not
    /// polynomial: a grid of 1000 intervals of the path parameter, then golden-section refinement around the best sample.
    /// </summary>
    public (double Minimum, double MinimumTime, double Maximum, double MaximumTime)[] ComponentExtrema(int order)
    {
        if (order < 0 || order > ArcLengthDerivatives.MaxOrder)
            throw new ArgumentOutOfRangeException(nameof(order));

        double t0 = Subpath.StartTime, t1 = Subpath.EndTime;
        double scale = Math.Pow(Speed, order);

        double[] Values(double parameter)
        {
            if (order == 0)
            {
                Point3D p = Subpath.Path.EvaluatePosition(parameter);
                return [p.X, p.Y, p.Z];
            }

            Point3D d = ArcLengthDerivatives.Compute(Subpath.Path, parameter).Derivatives[order - 1];
            return [d.X * scale, d.Y * scale, d.Z * scale];
        }

        var minValue = new double[] { double.MaxValue, double.MaxValue, double.MaxValue };
        var maxValue = new double[] { double.MinValue, double.MinValue, double.MinValue };
        var minIndex = new int[3];
        var maxIndex = new int[3];
        for (int i = 0; i <= PeakGrid; i++)
        {
            double[] v = Values(t0 + (t1 - t0) * i / PeakGrid);
            for (int axis = 0; axis < 3; axis++)
            {
                if (v[axis] < minValue[axis]) { minValue[axis] = v[axis]; minIndex[axis] = i; }
                if (v[axis] > maxValue[axis]) { maxValue[axis] = v[axis]; maxIndex[axis] = i; }
            }
        }

        // Golden-section search of the best parameter near a grid index, for the function sign * value.
        (double Value, double Parameter) Refine(int axis, int index, double sign)
        {
            const double invPhi = 0.6180339887498949;
            double lo = t0 + (t1 - t0) * Math.Max(index - 1, 0) / PeakGrid;
            double hi = t0 + (t1 - t0) * Math.Min(index + 1, PeakGrid) / PeakGrid;
            double bestParameter = t0 + (t1 - t0) * index / PeakGrid;
            double bestValue = sign * Values(bestParameter)[axis];
            for (int k = 0; k < 60; k++)
            {
                double c = hi - invPhi * (hi - lo), d = lo + invPhi * (hi - lo);
                double fc = sign * Values(c)[axis], fd = sign * Values(d)[axis];
                if (fc > bestValue) { bestValue = fc; bestParameter = c; }
                if (fd > bestValue) { bestValue = fd; bestParameter = d; }
                if (fc > fd) hi = d; else lo = c;
            }

            return (sign * bestValue, bestParameter);
        }

        var result = new (double, double, double, double)[3];
        for (int axis = 0; axis < 3; axis++)
        {
            var (minimum, minParameter) = Refine(axis, minIndex[axis], -1.0);
            var (maximum, maxParameter) = Refine(axis, maxIndex[axis], +1.0);
            result[axis] = (minimum, Subpath.LengthAt(minParameter) / Speed, maximum, Subpath.LengthAt(maxParameter) / Speed);
        }

        return result;
    }

    /// <summary>P_k = the largest component peak of D^k p, k = 1 .. 4 (element k - 1).</summary>
    public double[] Peaks() => ComponentPeaks().Select(p => p.Max()).ToArray();

    /// <summary>
    /// v_adm = min over the active limits of (L_k / P_k)^(1/k) and the limit that binds (null if no limit is active or
    /// no active limit is ever reached: then every speed is admissible).
    /// </summary>
    public (double Speed, string? BindingLimit) AdmissibleSpeed(KinematicLimits? limits)
    {
        if (limits is null || !limits.HasAnyLimit)
            return (double.PositiveInfinity, null);

        double[] p = Peaks();
        var candidates = new (int Order, string Name, double? Limit)[]
        {
            (1, "velocity", limits.MaxVelocity), (2, "acceleration", limits.MaxAcceleration),
            (3, "jerk", limits.MaxJerk), (4, "snap", limits.MaxSnap)
        };

        double admissible = double.PositiveInfinity;
        string? binding = null;
        foreach (var (order, name, limit) in candidates)
        {
            if (limit is null || !(p[order - 1] > 0))
                continue;

            double v = Math.Pow(limit.Value / p[order - 1], 1.0 / order);
            if (v < admissible)
            {
                admissible = v;
                binding = name;
            }
        }

        return (admissible, binding);
    }
}

/// <summary>
/// Entry transition + constant-speed stretch along a given subpath of a 3D segment + exit transition. For a straight
/// subpath it coincides with <see cref="ConstantVelocityTrajectory3D"/>; for a curved one the stretch keeps the path
/// and the speed magnitude and has the (centripetal …) derivatives of the curve.
/// </summary>
/// <remarks>
/// The transitions are ordinary 3D polynomial segments (common duration, component-wise coefficients) from the outer
/// state to the stretch's full boundary state (not zero acceleration, jerk, snap: the arc-length derivatives), so that
/// position … snap are continuous at the junctions up to the order of the degree. Durations: Rules 1–3 of 1D with the
/// feasibility as the maximum over the axes and the Euclidean distance in Rule 2. The stretch cannot adapt to limits:
/// v_cv above the admissible speed fails explicitly, naming the binding limit.
/// </remarks>
public class ConstantSpeedTrajectory3D
{
    public PolynomialSegment3D EntryTransition { get; }
    public CurvedConstantSpeedStretch3D Stretch { get; }
    public PolynomialSegment3D ExitTransition { get; }

    public double TotalDuration => EntryTransition.Duration + Stretch.Duration + ExitTransition.Duration;

    /// <param name="path">The segment whose subpath is traversed at constant speed.</param>
    /// <param name="startTime">Start of the subpath, as a time of the segment.</param>
    /// <param name="endTime">End of the subpath (greater than the start).</param>
    /// <param name="speed">v_cv &gt; 0.</param>
    /// <param name="startState">State at the start of the whole trajectory (outer end of the entry transition).</param>
    /// <param name="endState">State at the end of the whole trajectory.</param>
    /// <param name="limits">Per-axis limits (null or none active: no limit is imposed).</param>
    /// <exception cref="PathNotRegularException">The subpath goes through a rest point (σ = 0).</exception>
    /// <exception cref="KinematicLimitsViolatedException">v_cv above v_adm (message gives v_adm and the binding limit), a transition violating a limit, or a total below the minimum feasible one.</exception>
    /// <exception cref="DurationSearchFailedException">No duration of a transition satisfies the limits.</exception>
    public ConstantSpeedTrajectory3D(
        PolynomialSegment3D path,
        double startTime,
        double endTime,
        double speed,
        State3D startState,
        State3D endState,
        int polynomialDegree = 9,
        KinematicLimits? limits = null,
        double? entryDuration = null,
        double? exitDuration = null,
        double? totalDuration = null)
    {
        if (entryDuration is <= 0 || exitDuration is <= 0)
            throw new ArgumentException("Prescribed transition durations must be positive");
        if (totalDuration is not null && entryDuration is not null && exitDuration is not null)
            throw new ArgumentException("A total duration together with both transition durations is a conflict: prescribe at most two of the three");

        var subpath = new ArcLengthPath(path, startTime, endTime);
        Stretch = new CurvedConstantSpeedStretch3D(subpath, speed);

        KinematicLimits? active = limits is not null && limits.HasAnyLimit ? limits : null;
        if (active is not null)
        {
            var (admissible, binding) = Stretch.AdmissibleSpeed(active);
            if (speed > admissible * (1 + 1e-12))
            {
                throw new KinematicLimitsViolatedException(
                    $"The constant speed {speed:G6} exceeds the admissible speed {admissible:G6} of this stretch: " +
                    $"the {binding} limit binds. The time law of the stretch is fixed by the speed and the geometry, so no transition can repair it.");
            }
        }

        double cvDuration = Stretch.Duration;
        State3D cvStart = Stretch.StartState, cvEnd = Stretch.EndState;

        double entryRule2 = ConstantVelocityTrajectory1D.MeanVelocityDuration(
            (cvStart.Position - startState.Position).Length, startState.Velocity.Length, speed, cvDuration);
        double exitRule2 = ConstantVelocityTrajectory1D.MeanVelocityDuration(
            (endState.Position - cvEnd.Position).Length, endState.Velocity.Length, speed, cvDuration);

        var (entry, exit) = TransitionDurationRules.Resolve(
            cvDuration, active is not null, entryDuration, exitDuration, totalDuration,
            () => ConstantVelocityTrajectory3D.Weight("entry transition", polynomialDegree, startState, cvStart, active, entryRule2),
            () => ConstantVelocityTrajectory3D.Weight("exit transition", polynomialDegree, cvEnd, endState, active, exitRule2));

        EntryTransition = new PolynomialSegment3D(polynomialDegree, startState, cvStart, entry);
        ExitTransition = new PolynomialSegment3D(polynomialDegree, cvEnd, endState, exit);

        if (active is not null)
        {
            ConstantVelocityTrajectory3D.Verify(EntryTransition, active, "entry transition");
            ConstantVelocityTrajectory3D.Verify(ExitTransition, active, "exit transition");
        }
    }

    // ---- the three specifications of the subpath (theory: "Specifying The Stretch") --------------

    /// <summary>(b) Two fractions τ of the normalized parameter of the segment, 0 ≤ τ_start &lt; τ_end ≤ 1.</summary>
    public static ConstantSpeedTrajectory3D FromParameterFractions(
        PolynomialSegment3D path, double tauStart, double tauEnd, double speed, State3D startState, State3D endState,
        int polynomialDegree = 9, KinematicLimits? limits = null,
        double? entryDuration = null, double? exitDuration = null, double? totalDuration = null)
    {
        if (tauStart < 0 || tauEnd > 1 || !(tauStart < tauEnd))
            throw new ArgumentException($"The parameter fractions must satisfy 0 <= tau_start < tau_end <= 1, got {tauStart:G6} and {tauEnd:G6}.");

        return new ConstantSpeedTrajectory3D(
            path, tauStart * path.Duration, tauEnd * path.Duration, speed, startState, endState,
            polynomialDegree, limits, entryDuration, exitDuration, totalDuration);
    }

    /// <summary>(a) Two points on the path: the parameter of the closest point is used (rejected beyond the tolerance of <see cref="BlendBuilder3D.TauForPoint"/>).</summary>
    public static ConstantSpeedTrajectory3D FromPoints(
        PolynomialSegment3D path, Point3D start, Point3D end, double speed, State3D startState, State3D endState,
        int polynomialDegree = 9, KinematicLimits? limits = null,
        double? entryDuration = null, double? exitDuration = null, double? totalDuration = null)
    {
        double tauStart = BlendBuilder3D.TauForPoint(path, start, "constant-speed start");
        double tauEnd = BlendBuilder3D.TauForPoint(path, end, "constant-speed end");
        return FromParameterFractions(
            path, tauStart, tauEnd, speed, startState, endState, polynomialDegree, limits, entryDuration, exitDuration, totalDuration);
    }

    /// <summary>(c) Two fractions of the total arc length of the segment (by the inversion of ℓ).</summary>
    public static ConstantSpeedTrajectory3D FromArcLengthFractions(
        PolynomialSegment3D path, double lengthFractionStart, double lengthFractionEnd, double speed, State3D startState, State3D endState,
        int polynomialDegree = 9, KinematicLimits? limits = null,
        double? entryDuration = null, double? exitDuration = null, double? totalDuration = null)
    {
        if (lengthFractionStart < 0 || lengthFractionEnd > 1 || !(lengthFractionStart < lengthFractionEnd))
            throw new ArgumentException(
                $"The length fractions must satisfy 0 <= start < end <= 1, got {lengthFractionStart:G6} and {lengthFractionEnd:G6}.");

        // The whole segment may touch rest points (σ = 0 at its ends): only its length is needed here.
        var whole = new ArcLengthPath(path, 0, path.Duration, requireRegular: false);
        double tStart = whole.TimeAtLength(lengthFractionStart * whole.Length);
        double tEnd = whole.TimeAtLength(lengthFractionEnd * whole.Length);
        return new ConstantSpeedTrajectory3D(
            path, tStart, tEnd, speed, startState, endState, polynomialDegree, limits, entryDuration, exitDuration, totalDuration);
    }

    // ---- evaluation and sampling ------------------------------------------------------------------

    public State3D EvaluateState(double t)
    {
        if (t < EntryTransition.Duration)
            return EntryTransition.EvaluateState(t);

        t -= EntryTransition.Duration;
        if (t < Stretch.Duration)
            return Stretch.EvaluateState(t);

        t -= Stretch.Duration;
        return ExitTransition.EvaluateState(Math.Min(t, ExitTransition.Duration));
    }

    public Point3D EvaluatePosition(double t) => EvaluateState(t).Position;

    public Point3D EvaluateDerivative(double t, int order) => EvaluateState(t).GetDerivative(order);

    /// <summary>
    /// Uniform samples over [0, TotalDuration] (ends included) of the position and its derivatives up to
    /// <paramref name="maxDerivativeOrder"/>, like <see cref="Trajectory1D.Sample"/>: <c>Values[order][i]</c> is a vector.
    /// </summary>
    public TrajectorySamples3D Sample(int sampleCount, int maxDerivativeOrder)
    {
        if (sampleCount < 2)
            throw new ArgumentOutOfRangeException(nameof(sampleCount), "At least 2 samples are required (start and end).");
        if (maxDerivativeOrder < 0 || maxDerivativeOrder > 4)
            throw new ArgumentOutOfRangeException(nameof(maxDerivativeOrder));

        var times = new double[sampleCount];
        var values = new Point3D[maxDerivativeOrder + 1][];
        for (int order = 0; order <= maxDerivativeOrder; order++)
            values[order] = new Point3D[sampleCount];

        for (int i = 0; i < sampleCount; i++)
        {
            times[i] = TotalDuration * i / (sampleCount - 1);
            State3D state = EvaluateState(times[i]);
            for (int order = 0; order <= maxDerivativeOrder; order++)
                values[order][i] = state.GetDerivative(order);
        }

        return new TrajectorySamples3D(times, values);
    }
}
