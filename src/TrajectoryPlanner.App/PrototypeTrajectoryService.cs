using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.BoundaryConditions;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App;

/// <summary>
/// Thin layer between the UI input rows and the Core: parses the text fields, calls the Core builders (segment
/// factory, blending, constant velocity), and formats the results for the report tables and the plots.
/// Durations, limit checks, blend point search, evaluation and sampling are all done by the Core.
/// </summary>
public sealed class PrototypeTrajectoryService
{
    /// <summary>Samples per plotted curve: at least this many, or this many per segment, whichever is larger.</summary>
    private const int MinimumSamples = 300;

    private const int SamplesPerSegment = 120;

    public PrototypeComputationResult Compute(IReadOnlyList<SegmentInputModel> inputs)
    {
        if (inputs.Count == 0)
            throw new InvalidOperationException("At least one segment is required.");

        var representations = inputs.Select(BuildRepresentation).ToList();
        var notices = new List<string>();
        ApplyBlends(inputs, representations, notices);

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

        TrajectorySamples samples = trajectory.Sample(
            Math.Max(MinimumSamples, flattenedSegments.Count * SamplesPerSegment), maxDerivativeOrder);
        var series = new List<SeriesData>();
        for (int order = 0; order <= maxDerivativeOrder; order++)
            series.Add(new SeriesData(GetQuantityName(order), order, samples.Times, samples.Values[order]));

        var extremaRows = new List<ExtremaReportRow>();
        var coefficientRows = new List<CoefficientReportRow>();
        foreach (var labeled in flattenedSegments)
        {
            int supportedOrder = BoundaryConditionValidator.DerivativeLevelsForDegree(labeled.Segment.Degree) - 1;
            // Duration of every piece (prescribed, or the shortest feasible one when limits are active).
            coefficientRows.Add(new CoefficientReportRow(labeled.Label, "T [s]", labeled.Segment.Duration));
            for (int i = 0; i <= labeled.Segment.Degree; i++)
                coefficientRows.Add(new CoefficientReportRow(labeled.Label, $"c{i}", labeled.Segment.Coefficients[i]));

            for (int order = 0; order <= supportedOrder; order++)
            {
                ExtremaResult extrema = ExtremaAnalyzer.FindExtrema(labeled.Segment, order);
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
            trajectory.TotalDuration,
            notices);
    }

    private static SegmentRepresentation BuildRepresentation(SegmentInputModel input)
    {
        string label = $"Segment {input.SegmentNumber}";
        State1D startState = BuildState(input, isStart: true);
        State1D endState = BuildState(input, isStart: false);
        KinematicLimits limits = BuildLimits(input);

        if (input.UseConstantVelocity)
        {
            var segments = BuildConstantVelocitySegments(input, startState, endState, limits);
            return new SegmentRepresentation(limits, supportsBlending: false, segments);
        }

        double? duration = input.UseDuration ? ParsePositiveDouble(input.Duration, $"{label} duration") : null;
        PolynomialSegment1D segment = CallCore(label, () =>
            SegmentFactory1D.Build(input.Degree, startState, endState, limits, duration));

        return new SegmentRepresentation(
            limits,
            supportsBlending: true,
            new List<LabeledSegment> { new(label, segment) });
    }

    private static void ApplyBlends(
        IReadOnlyList<SegmentInputModel> inputs, IList<SegmentRepresentation> representations, List<string> notices)
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

            // The blend must respect the limits of both neighbours: the more restrictive one for each kind.
            KinematicLimits blendLimits = KinematicLimits.MostRestrictive(representations[i].Limits, representations[i + 1].Limits);
            string blendLabel = $"Blend {inputs[i].SegmentNumber}-{inputs[i + 1].SegmentNumber}";

            var (truncatedPreceding, blendSegment, truncatedFollowing) = CallCore(blendLabel, () =>
                BlendBuilder1D.BuildBlendedSegmentPair(
                    preceding, following, new Point1D(precedingBlendPosition), new Point1D(followingBlendPosition),
                    limits: blendLimits));

            representations[i].Segments[0] = new LabeledSegment($"Segment {inputs[i].SegmentNumber}", truncatedPreceding);
            representations[i + 1].Segments[0] = new LabeledSegment($"Segment {inputs[i + 1].SegmentNumber}", truncatedFollowing);
            representations[i].BlendAfter = new LabeledSegment(blendLabel, blendSegment);

            // Tell the user when the limits forced a blend duration other than the inherited one (numbers in the current culture).
            if (blendSegment.AdjustedByLimits)
            {
                notices.Add(
                    $"{blendLabel}: duration adjusted from {blendSegment.InheritedDuration:F4} s to {blendSegment.Duration:F4} s to respect the limits.");
            }
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

        ConstantVelocityTrajectory1D trajectory = CallCore(segmentLabel, () =>
            new ConstantVelocityTrajectory1D(
                startState, endState, xCvStart, xCvEnd, speed, input.Degree,
                limits.HasAnyLimit ? limits : null, totalDuration: totalDuration));

        return
        [
            new LabeledSegment($"{segmentLabel} Entry", trajectory.EntryTransition),
            new LabeledSegment($"{segmentLabel} Constant-Velocity", trajectory.ConstantVelocity.ToPolynomialSegments().Single()),
            new LabeledSegment($"{segmentLabel} Exit", trajectory.ExitTransition)
        ];
    }

    /// <summary>
    /// Runs a Core call and turns the Core's explicit failures (limits, impossible searches, invalid arguments)
    /// into one exception type whose message names the segment or blend, for the status bar and the message box.
    /// </summary>
    private static T CallCore<T>(string label, Func<T> call)
    {
        try
        {
            return call();
        }
        catch (Exception ex) when (ex is DurationSearchFailedException or KinematicLimitsViolatedException or ArgumentException)
        {
            throw new InvalidOperationException($"{label}: {ex.Message}", ex);
        }
    }

    private static double ParseRequiredDouble(string value, string label)
    {
        if (!InputNumber.TryParse(value, out double parsed))
            throw new InvalidOperationException($"{label} must be a valid number (decimal point or comma, no thousands separators): \"{value}\".");

        return parsed;
    }

    private static double ParsePositiveDouble(string value, string label)
    {
        double parsed = ParseRequiredDouble(value, label);
        if (parsed <= 0)
            throw new InvalidOperationException($"{label} must be greater than zero.");
        return parsed;
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
        public SegmentRepresentation(KinematicLimits limits, bool supportsBlending, List<LabeledSegment> segments)
        {
            Limits = limits;
            SupportsBlending = supportsBlending;
            Segments = segments;
        }

        public KinematicLimits Limits { get; }

        public bool SupportsBlending { get; }

        public List<LabeledSegment> Segments { get; }

        public LabeledSegment? BlendAfter { get; set; }
    }

    private sealed record LabeledSegment(string Label, PolynomialSegment1D Segment);
}
