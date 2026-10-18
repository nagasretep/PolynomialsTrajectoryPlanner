using TrajectoryPlanner.Core.BoundaryConditions;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App;

/// <summary>
/// Pass-through mode (request 0013): parses the waypoint table, the end conditions and the limits, asks the Core
/// planner (<see cref="PassThroughPlanner1D"/>) for the trajectory and formats the report. The durations, the
/// common factor λ and the limit checks are all computed by the Core.
/// </summary>
public sealed partial class PrototypeTrajectoryService
{
    /// <summary>Reference total per segment when no duration and no total are given: A8 proportions of N x 1 s (design/08).</summary>
    public const double ReferenceDurationPerSegment = 1.0;

    public PrototypeComputationResult ComputePassThrough(PassThroughInputModel input)
    {
        int degree = input.Degree;
        double[] waypoints = ParseWaypoints(input);
        int segmentCount = waypoints.Length - 1;

        var criterion = new SmoothnessCriterion(input.SmoothnessOrder);
        try
        {
            criterion.Validate(degree);
        }
        catch (ArgumentOutOfRangeException ex)
        {
            throw new InvalidOperationException($"Smoothness order: {ex.Message}", ex);
        }

        State1D? startBoundary = input.StartAtRest ? null : BuildBoundary(input, isStart: true, waypoints[0]);
        State1D? endBoundary = input.EndAtRest ? null : BuildBoundary(input, isStart: false, waypoints[^1]);
        KinematicLimits limits = BuildLimits(input);

        double[]? segmentDurations = ParseSegmentDurations(input, segmentCount);
        double? totalDuration = ParseOptionalPositive(input.TotalDuration, "Total duration");
        if (segmentDurations is not null && totalDuration is not null)
            throw new InvalidOperationException("Segment durations and total duration are both given: fill either the durations or the total, not both.");

        // Which durations are prescribed (kept and only verified) and which are scaled by the limits is the Core's job;
        // here only the user's inputs are turned into the planner's arguments.
        double[]? prescribed = segmentDurations;
        double[]? reference = null;
        if (prescribed is null && totalDuration is not null)
        {
            prescribed = new AutoDerivedProportionalDuration(totalDuration.Value).ComputeDurations(waypoints, degree);
        }
        else if (prescribed is null)
        {
            // One rule with and without limits: A8 proportions, one second per segment in total. Without limits the
            // planner uses them as they are; with limits it scales them by λ.
            reference = new AutoDerivedProportionalDuration(segmentCount * ReferenceDurationPerSegment).ComputeDurations(waypoints, degree);
        }

        PassThroughResult plan = CallCore("Cannot build the trajectory", () =>
            PassThroughPlanner1D.Plan(
                waypoints, degree, criterion, limits.HasAnyLimit ? limits : null,
                prescribed, reference, startBoundary, endBoundary));

        var notices = new List<string>();
        if (plan.ScaledByLimits)
        {
            notices.Add(
                $"Limits applied: all segment durations were multiplied by λ = {plan.Scale:F6}; total duration {plan.TotalDuration:F4} s.");
        }

        return BuildPassThroughResult(degree, criterion, waypoints, plan, limits, notices);
    }

    private static PrototypeComputationResult BuildPassThroughResult(
        int degree, SmoothnessCriterion criterion, double[] waypoints, PassThroughResult plan, KinematicLimits limits, List<string> notices)
    {
        var trajectory = new Trajectory1D(plan.Segments);
        int maxDerivativeOrder = BoundaryConditionValidator.DerivativeLevelsForDegree(degree) - 1;
        List<SeriesData> series = SampleSeries(trajectory, maxDerivativeOrder);

        // Segment start times, the waypoints on the time axis and the state at each of them.
        var startTimes = new double[plan.Segments.Length + 1];
        for (int k = 0; k < plan.Segments.Length; k++)
            startTimes[k + 1] = startTimes[k] + plan.Segments[k].Duration;

        var markers = new List<WaypointMarker>();
        var states = new List<WaypointStateRow>();
        for (int i = 0; i < waypoints.Length; i++)
        {
            State1D state = trajectory.EvaluateState(startTimes[i]);
            markers.Add(new WaypointMarker(startTimes[i], waypoints[i]));
            states.Add(new WaypointStateRow(
                i + 1, startTimes[i], state.Position, state.Velocity, state.Acceleration, state.Jerk, state.Snap));
        }

        // Coefficients: one block per segment, as in blending mode.
        var coefficientRows = new List<CoefficientReportRow>();
        for (int k = 0; k < plan.Segments.Length; k++)
        {
            string label = $"Segment {k + 1}";
            coefficientRows.Add(new CoefficientReportRow(label, "T [s]", plan.Segments[k].Duration));
            for (int i = 0; i <= degree; i++)
                coefficientRows.Add(new CoefficientReportRow(label, $"c{i}", plan.Segments[k].Coefficients[i]));
        }

        // Extrema over the whole trajectory: best of the per-segment extrema, times on the global axis.
        var extremaRows = new List<ExtremaReportRow>();
        for (int order = 0; order <= maxDerivativeOrder; order++)
        {
            double min = double.MaxValue, max = double.MinValue, tMin = 0, tMax = 0;
            for (int k = 0; k < plan.Segments.Length; k++)
            {
                ExtremaResult e = ExtremaAnalyzer.FindExtrema(plan.Segments[k], order);
                if (e.Minimum < min) { min = e.Minimum; tMin = startTimes[k] + e.MinTime; }
                if (e.Maximum > max) { max = e.Maximum; tMax = startTimes[k] + e.MaxTime; }
            }

            extremaRows.Add(new ExtremaReportRow("Trajectory", GetQuantityName(order), min, max, tMin, tMax));
        }

        var summary = new List<SummaryRow>();
        for (int k = 0; k < plan.Segments.Length; k++)
            summary.Add(new SummaryRow($"Segment {k + 1} duration [s]", $"{plan.Durations[k]:F6}"));
        summary.Add(new SummaryRow("Total duration [s]", $"{plan.TotalDuration:F6}"));
        if (plan.ScaledByLimits)
            summary.Add(new SummaryRow("Common factor λ", $"{plan.Scale:F6}"));
        summary.Add(new SummaryRow("Smoothness order m", $"{criterion.DerivativeOrder}"));
        summary.Add(new SummaryRow(
            $"Smoothness cost J (order {criterion.DerivativeOrder})",
            $"{SmoothnessCost.Of(plan.Segments, criterion.DerivativeOrder):G8}"));

        // Peak against each active limit over the whole trajectory (the binding limit of a λ-scaled result is at 100 %).
        var peaks = new List<LimitPeakRow>();
        foreach (var (order, limit) in new[] { (1, limits.MaxVelocity), (2, limits.MaxAcceleration), (3, limits.MaxJerk), (4, limits.MaxSnap) })
        {
            if (limit is null || order > maxDerivativeOrder)
                continue;

            var (peak, segmentIndex, _) = ExtremaAnalyzer.PeakAbsoluteWithLocation(plan.Segments, order);
            peaks.Add(new LimitPeakRow(GetQuantityName(order), peak, limit.Value, 100.0 * peak / limit.Value, segmentIndex + 1));
        }

        return new PrototypeComputationResult(
            series,
            extremaRows,
            coefficientRows,
            trajectory.EvaluatePosition,
            trajectory.EvaluateDerivative,
            trajectory.TotalDuration,
            notices,
            markers,
            states,
            summary,
            peaks);
    }

    private static double[] ParseWaypoints(PassThroughInputModel input)
    {
        if (input.Waypoints.Count < 2)
            throw new InvalidOperationException("At least two waypoints are required.");

        return input.Waypoints
            .Select(w => ParseRequiredDouble(w.Position, $"Waypoint {w.Number} position"))
            .ToArray();
    }

    /// <summary>All segment durations filled (prescribed) or all blank (null); a partly filled column is an input error.</summary>
    private static double[]? ParseSegmentDurations(PassThroughInputModel input, int segmentCount)
    {
        var cells = input.Waypoints.Take(segmentCount).Select(w => w.Duration?.Trim() ?? string.Empty).ToList();
        int filled = cells.Count(c => c.Length > 0);
        if (filled == 0)
            return null;

        if (filled != segmentCount)
        {
            int firstBlank = cells.FindIndex(c => c.Length == 0) + 1;
            throw new InvalidOperationException(
                $"Segment durations are partly filled ({filled} of {segmentCount}; waypoint {firstBlank} is blank): fill all of them or leave all blank.");
        }

        return cells
            .Select((c, i) => ParsePositiveDouble(c, $"Segment {i + 1} duration (waypoint {i + 1} row)"))
            .ToArray();
    }

    private static double? ParseOptionalPositive(string value, string label) =>
        string.IsNullOrWhiteSpace(value) ? null : ParsePositiveDouble(value, label);

    private static State1D BuildBoundary(PassThroughInputModel input, bool isStart, double position)
    {
        string side = isStart ? "start" : "end";
        double v = ParseRequiredDouble(isStart ? input.StartVelocity : input.EndVelocity, $"{side} velocity");
        double a = input.SupportsAcceleration
            ? ParseRequiredDouble(isStart ? input.StartAcceleration : input.EndAcceleration, $"{side} acceleration") : 0.0;
        double j = input.SupportsJerk
            ? ParseRequiredDouble(isStart ? input.StartJerk : input.EndJerk, $"{side} jerk") : 0.0;
        double s = input.SupportsSnap
            ? ParseRequiredDouble(isStart ? input.StartSnap : input.EndSnap, $"{side} snap") : 0.0;
        return new State1D(position, v, a, j, s);
    }

    private static KinematicLimits BuildLimits(PassThroughInputModel input)
    {
        double? vMax = input.UseMaxVelocity ? ParsePositiveDouble(input.MaxVelocity, "Maximum velocity") : null;
        double? aMax = input.UseMaxAcceleration && input.SupportsAcceleration
            ? ParsePositiveDouble(input.MaxAcceleration, "Maximum acceleration") : null;
        double? jMax = input.UseMaxJerk && input.SupportsJerk
            ? ParsePositiveDouble(input.MaxJerk, "Maximum jerk") : null;
        double? sMax = input.UseMaxSnap && input.SupportsSnap
            ? ParsePositiveDouble(input.MaxSnap, "Maximum snap") : null;
        return KinematicLimits.FromValues(vMax, aMax, jMax, sMax);
    }
}
