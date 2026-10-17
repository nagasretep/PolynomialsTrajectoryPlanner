using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.BoundaryConditions;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App;

public sealed class PrototypeTrajectoryService
{
    /// <summary>Duration used when it is neither prescribed nor determined by active limits.</summary>
    private const double DefaultDuration = 1.0;

    public PrototypeComputationResult Compute(IReadOnlyList<SegmentInputModel> inputs)
    {
        if (inputs.Count == 0)
            throw new InvalidOperationException("At least one segment is required.");

        var representations = inputs.Select(BuildRepresentation).ToList();
        ApplyBlends(inputs, representations);

        var flattenedSegments = new List<LabeledSegment>();
        foreach (var representation in representations)
        {
            flattenedSegments.AddRange(representation.Segments);
            if (representation.BlendAfter is not null)
                flattenedSegments.Add(representation.BlendAfter);
        }

        if (flattenedSegments.Count == 0)
            throw new InvalidOperationException("No trajectory segments were generated.");

        var trajectory = new Trajectory1D(flattenedSegments.Select(x => x.Segment));
        int maxDerivativeOrder = inputs
            .Select(x => BoundaryConditionValidator.DerivativeLevelsForDegree(x.Degree) - 1)
            .DefaultIfEmpty(0)
            .Max();

        double[] sampleTimes = SampleGlobalTimes(trajectory.TotalDuration, Math.Max(300, flattenedSegments.Count * 120));
        var series = new List<SeriesData>();
        for (int order = 0; order <= maxDerivativeOrder; order++)
        {
            double[] values = sampleTimes
                .Select(t => order == 0 ? trajectory.EvaluatePosition(t) : trajectory.EvaluateDerivative(t, order))
                .ToArray();
            series.Add(new SeriesData(GetQuantityName(order), order, sampleTimes, values));
        }

        var extremaRows = new List<ExtremaReportRow>();
        var coefficientRows = new List<CoefficientReportRow>();
        foreach (var labeled in flattenedSegments)
        {
            int supportedOrder = BoundaryConditionValidator.DerivativeLevelsForDegree(labeled.Segment.Degree) - 1;
            // Duration of every piece (prescribed, or the shortest feasible one when limits are active).
            coefficientRows.Add(new CoefficientReportRow(labeled.Label, "T [s]", labeled.Segment.Duration));
            for (int i = 0; i <= labeled.Segment.Degree; i++)
            {
                coefficientRows.Add(new CoefficientReportRow(labeled.Label, $"c{i}", labeled.Segment.Coefficients[i]));
            }

            for (int order = 0; order <= supportedOrder; order++)
            {
                ExtremaResult extrema = order switch
                {
                    0 => ExtremaAnalyzer.PositionExtrema(labeled.Segment),
                    1 => ExtremaAnalyzer.VelocityExtrema(labeled.Segment),
                    2 => ExtremaAnalyzer.AccelerationExtrema(labeled.Segment),
                    3 => ExtremaAnalyzer.JerkExtrema(labeled.Segment),
                    4 => ExtremaAnalyzer.SnapExtrema(labeled.Segment),
                    _ => throw new InvalidOperationException("Unsupported derivative order.")
                };

                extremaRows.Add(new ExtremaReportRow(
                    labeled.Label,
                    GetQuantityName(order),
                    extrema.Minimum,
                    extrema.Maximum,
                    extrema.MinTime,
                    extrema.MaxTime));
            }
        }

        return new PrototypeComputationResult(
            series,
            extremaRows,
            coefficientRows,
            trajectory.EvaluatePosition,
            trajectory.EvaluateDerivative,
            trajectory.TotalDuration);
    }

    private static SegmentRepresentation BuildRepresentation(SegmentInputModel input)
    {
        State1D startState = BuildState(input, isStart: true);
        State1D endState = BuildState(input, isStart: false);
        KinematicLimits limits = BuildLimits(input);

        if (input.UseConstantVelocity)
        {
            var segments = BuildConstantVelocitySegments(input, startState, endState, limits);
            return new SegmentRepresentation(input.SegmentNumber, supportsBlending: false, segments);
        }

        PolynomialSegment1D segment = BuildPolynomialSegment(input, startState, endState, limits);
        return new SegmentRepresentation(
            input.SegmentNumber,
            supportsBlending: true,
            new List<LabeledSegment> { new($"Segment {input.SegmentNumber}", segment) });
    }

    private static void ApplyBlends(IReadOnlyList<SegmentInputModel> inputs, IList<SegmentRepresentation> representations)
    {
        for (int i = 0; i < inputs.Count - 1; i++)
        {
            string outgoingBlend = inputs[i].BlendToNext.Trim();
            string incomingBlend = inputs[i + 1].BlendToPrevious.Trim();

            bool outgoingActive = outgoingBlend.Length > 0;
            bool incomingActive = incomingBlend.Length > 0;

            if (!outgoingActive && !incomingActive)
                continue;

            if (outgoingActive != incomingActive)
            {
                throw new InvalidOperationException(
                    $"Blend between segment {inputs[i].SegmentNumber} and segment {inputs[i + 1].SegmentNumber} requires both values.");
            }

            if (!representations[i].SupportsBlending || !representations[i + 1].SupportsBlending)
            {
                throw new InvalidOperationException(
                    $"Blending is currently supported only for plain polynomial segments. Remove constant-velocity mode from segments {inputs[i].SegmentNumber} or {inputs[i + 1].SegmentNumber} to use blending.");
            }

            var preceding = representations[i].Segments.Single().Segment;
            var following = representations[i + 1].Segments.Single().Segment;

            double precedingBlendPosition = ParseRequiredDouble(outgoingBlend, $"Segment {inputs[i].SegmentNumber} final blend value");
            double followingBlendPosition = ParseRequiredDouble(incomingBlend, $"Segment {inputs[i + 1].SegmentNumber} initial blend value");

            double blendStartTau = FindTauForPosition(preceding, precedingBlendPosition);
            double blendEndTau = FindTauForPosition(following, followingBlendPosition);

            var (truncatedPreceding, blendSegment, truncatedFollowing) =
                BlendBuilder1D.BuildBlendedSegmentPair(preceding, following, blendStartTau, blendEndTau, 9);

            representations[i].Segments[0] = new LabeledSegment($"Segment {inputs[i].SegmentNumber}", truncatedPreceding);
            representations[i + 1].Segments[0] = new LabeledSegment($"Segment {inputs[i + 1].SegmentNumber}", truncatedFollowing);
            representations[i].BlendAfter = new LabeledSegment(
                $"Blend {inputs[i].SegmentNumber}-{inputs[i + 1].SegmentNumber}",
                blendSegment);
        }
    }

    private static State1D BuildState(SegmentInputModel input, bool isStart)
    {
        string side = isStart ? "start" : "end";
        double position = ParseRequiredDouble(isStart ? input.StartPosition : input.EndPosition, $"Segment {input.SegmentNumber} {side} position");
        double velocity = ParseRequiredDouble(isStart ? input.StartVelocity : input.EndVelocity, $"Segment {input.SegmentNumber} {side} velocity");
        double acceleration = input.SupportsAcceleration
            ? ParseRequiredDouble(isStart ? input.StartAcceleration : input.EndAcceleration, $"Segment {input.SegmentNumber} {side} acceleration")
            : 0.0;
        double jerk = input.SupportsJerk
            ? ParseRequiredDouble(isStart ? input.StartJerk : input.EndJerk, $"Segment {input.SegmentNumber} {side} jerk")
            : 0.0;
        double snap = input.SupportsSnap
            ? ParseRequiredDouble(isStart ? input.StartSnap : input.EndSnap, $"Segment {input.SegmentNumber} {side} snap")
            : 0.0;

        return new State1D(position, velocity, acceleration, jerk, snap);
    }

    private static KinematicLimits BuildLimits(SegmentInputModel input)
    {
        double? maxVelocity = input.UseMaxVelocity
            ? ParseRequiredDouble(input.MaxVelocity, $"Segment {input.SegmentNumber} maximum velocity")
            : null;
        double? maxAcceleration = input.UseMaxAcceleration && input.SupportsAcceleration
            ? ParseRequiredDouble(input.MaxAcceleration, $"Segment {input.SegmentNumber} maximum acceleration")
            : null;
        double? maxJerk = input.UseMaxJerk && input.SupportsJerk
            ? ParseRequiredDouble(input.MaxJerk, $"Segment {input.SegmentNumber} maximum jerk")
            : null;
        double? maxSnap = input.UseMaxSnap && input.SupportsSnap
            ? ParseRequiredDouble(input.MaxSnap, $"Segment {input.SegmentNumber} maximum snap")
            : null;

        return KinematicLimits.FromValues(maxVelocity, maxAcceleration, maxJerk, maxSnap);
    }

    private static PolynomialSegment1D BuildPolynomialSegment(
        SegmentInputModel input,
        State1D startState,
        State1D endState,
        KinematicLimits limits)
    {
        bool hasLimits = HasAnyKinematicLimit(limits);
        double duration;

        if (input.UseDuration)
        {
            duration = ParsePositiveDouble(input.Duration, $"Segment {input.SegmentNumber} duration");
        }
        else if (hasLimits)
        {
            duration = ShortestDurationOrExplain(input.Degree, startState, endState, limits, $"Segment {input.SegmentNumber}") ?? DefaultDuration;
        }
        else
        {
            duration = DefaultDuration;
        }

        var segment = new PolynomialSegment1D(input.Degree, startState, endState, duration);

        if (input.UseDuration && hasLimits)
        {
            ValidateSegmentAgainstLimits(segment, limits, $"Segment {input.SegmentNumber}");
        }

        return segment;
    }

    private static List<LabeledSegment> BuildConstantVelocitySegments(
        SegmentInputModel input,
        State1D startState,
        State1D endState,
        KinematicLimits limits)
    {
        string segmentLabel = $"Segment {input.SegmentNumber}";
        double xCvStart = ParseRequiredDouble(input.ConstantVelocityStart, $"{segmentLabel} constant-velocity start");
        double xCvEnd = ParseRequiredDouble(input.ConstantVelocityEnd, $"{segmentLabel} constant-velocity end");
        double speed = ParsePositiveDouble(input.ConstantVelocitySpeed, $"{segmentLabel} constant speed");

        if (xCvStart == xCvEnd)
            throw new InvalidOperationException($"{segmentLabel} constant-velocity stretch must have different start and end positions.");

        // Durations: Core decides (theory/ConstantVelocitySegment.md, Rules 1-3). A prescribed total duration is passed
        // on as is; Core splits the time left after the stretch between the transitions.
        double? totalDuration = input.UseDuration
            ? ParsePositiveDouble(input.Duration, $"{segmentLabel} duration")
            : null;

        ConstantVelocityTrajectory1D trajectory;
        try
        {
            trajectory = new ConstantVelocityTrajectory1D(
                startState, endState, xCvStart, xCvEnd, speed, input.Degree,
                HasAnyKinematicLimit(limits) ? limits : null, totalDuration: totalDuration);
        }
        catch (Exception ex) when (ex is DurationSearchFailedException or KinematicLimitsViolatedException or ArgumentException)
        {
            throw new InvalidOperationException($"{segmentLabel}: {ex.Message}", ex);
        }

        return
        [
            new LabeledSegment($"{segmentLabel} Entry", trajectory.EntryTransition),
            new LabeledSegment($"{segmentLabel} Constant-Velocity", trajectory.ConstantVelocity.ToPolynomialSegments().Single()),
            new LabeledSegment($"{segmentLabel} Exit", trajectory.ExitTransition)
        ];
    }

    private static double? ShortestDurationOrExplain(int degree, State1D start, State1D end, KinematicLimits limits, string label)
    {
        try
        {
            return ShortestFeasibleDuration.Find(degree, start, end, limits);
        }
        catch (DurationSearchFailedException ex)
        {
            throw new InvalidOperationException($"{label}: {ex.Message}", ex);
        }
    }

    private static void ValidateSegmentAgainstLimits(PolynomialSegment1D segment, KinematicLimits limits, string label)
    {
        if (!ExtremaAnalyzer.VerifyLimits(segment, limits))
            throw new InvalidOperationException($"{label} violates the active kinematic limits at the specified duration.");
    }

    private static bool HasAnyKinematicLimit(KinematicLimits limits)
    {
        return limits.HasVelocityLimit || limits.HasAccelerationLimit || limits.HasJerkLimit || limits.HasSnapLimit;
    }

    private static double[] SampleGlobalTimes(double totalDuration, int sampleCount)
    {
        if (sampleCount < 2)
            sampleCount = 2;

        double[] times = new double[sampleCount];
        for (int i = 0; i < sampleCount; i++)
        {
            times[i] = totalDuration * i / (sampleCount - 1);
        }

        return times;
    }

    private static double ParseRequiredDouble(string value, string label)
    {
        if (!double.TryParse(value, out double parsed))
            throw new InvalidOperationException($"{label} must be a valid number.");

        return parsed;
    }

    private static double ParsePositiveDouble(string value, string label)
    {
        double parsed = ParseRequiredDouble(value, label);
        if (parsed <= 0)
            throw new InvalidOperationException($"{label} must be greater than zero.");
        return parsed;
    }

    private static double FindTauForPosition(PolynomialSegment1D segment, double targetPosition)
    {
        double startPos = segment.StartState.Position;
        double endPos = segment.EndState.Position;
        double range = endPos - startPos;

        if (Math.Abs(range) < 1e-12)
            return 0.5;

        double tauEstimate = (targetPosition - startPos) / range;
        tauEstimate = Math.Clamp(tauEstimate, 0.01, 0.99);

        for (int iter = 0; iter < 50; iter++)
        {
            double time = tauEstimate * segment.Duration;
            double currentPos = segment.EvaluatePosition(time);
            double currentVel = segment.EvaluateDerivative(time, 1);

            if (Math.Abs(currentVel) < 1e-12)
                break;

            double error = targetPosition - currentPos;
            double deltaTau = error / (currentVel * segment.Duration);
            tauEstimate = Math.Clamp(tauEstimate + deltaTau, 0.001, 0.999);

            if (Math.Abs(error) < 1e-8)
                break;
        }

        return tauEstimate;
    }

    private static string GetQuantityName(int order) => order switch
    {
        0 => "Position",
        1 => "Velocity",
        2 => "Acceleration",
        3 => "Jerk",
        4 => "Snap",
        _ => $"Derivative {order}"
    };

    private sealed class SegmentRepresentation
    {
        public SegmentRepresentation(int segmentNumber, bool supportsBlending, List<LabeledSegment> segments)
        {
            SegmentNumber = segmentNumber;
            SupportsBlending = supportsBlending;
            Segments = segments;
        }

        public int SegmentNumber { get; }

        public bool SupportsBlending { get; }

        public List<LabeledSegment> Segments { get; }

        public LabeledSegment? BlendAfter { get; set; }
    }

    private sealed record LabeledSegment(string Label, PolynomialSegment1D Segment);
}
