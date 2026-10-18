using TrajectoryPlanner.Core.BoundaryConditions;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App;

/// <summary>
/// 3D pass-through mode (request 0020): parses the waypoint table, the end conditions and the limits, asks the Core
/// (<see cref="TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough"/>, i.e. <see cref="PassThroughPlanner3D"/>) for the
/// trajectory and presents it like the 3D mode. Durations, the common factor λ, the binding pair and the limit checks are
/// all computed by the Core.
/// </summary>
public sealed partial class Prototype3DTrajectoryService
{
    public Prototype3DComputationResult ComputePassThrough(PassThrough3DInputModel input)
    {
        int degree = input.Degree;
        Point3D[] waypoints = ParseWaypoints(input);
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

        State3D? start = input.StartAtRest ? null : Boundary(input, isStart: true);
        State3D? end = input.EndAtRest ? null : Boundary(input, isStart: false);
        KinematicLimits limits = Limits(input);

        double[]? durations = ParseDurationColumn(input, segmentCount);
        double? total = ParseOptionalPositive(input.TotalDuration, "Total duration");
        if (durations is not null && total is not null)
            throw new InvalidOperationException("Segment durations and total duration are both given: fill either the durations or the total, not both.");

        var builder = new TrajectoryBuilder3D().WithSmoothnessCriterion(criterion);
        CallCore("Cannot build the trajectory", () => builder.AddSegmentsFromWaypointsPassThrough(
            waypoints, durations, total, degree, limits.HasAnyLimit ? limits : null, start, end));
        PassThroughResult3D plan = builder.LastPassThroughResult!;
        Trajectory3D trajectory = builder.Build();

        int maxOrder = BoundaryConditionValidator.DerivativeLevelsForDegree(degree) - 1;
        int p = (degree + 1) / 2;

        // Waypoints on the time axis and the state there (the orders the waypoint states share: 0 .. p-1).
        var markers = new List<WaypointMarker3D>();
        var states = new List<WaypointState3DRow>();
        double time = 0;
        for (int i = 0; i < waypoints.Length; i++)
        {
            markers.Add(new WaypointMarker3D(i + 1, time, waypoints[i]));
            State3D state = trajectory.EvaluateState(Math.Min(time, trajectory.TotalDuration));
            for (int order = 0; order < p; order++)
            {
                Point3D v = state.GetDerivative(order);
                states.Add(new WaypointState3DRow(i + 1, time, QuantityName(order), v.X, v.Y, v.Z));
            }

            if (i < segmentCount)
                time += plan.Durations[i];
        }

        // Smoothness cost: the sum over the axes of the Core's exact cost (the 3D cost is that sum).
        double cost = 0;
        var polynomials = plan.Segments;
        cost += SmoothnessCost.Of(polynomials.Select(s => s.XAxis), criterion.DerivativeOrder);
        cost += SmoothnessCost.Of(polynomials.Select(s => s.YAxis), criterion.DerivativeOrder);
        cost += SmoothnessCost.Of(polynomials.Select(s => s.ZAxis), criterion.DerivativeOrder);

        var leading = new List<SummaryRow>();
        for (int k = 0; k < segmentCount; k++)
            leading.Add(new SummaryRow($"Segment {k + 1} duration [s]", $"{plan.Durations[k]:F6}"));
        if (plan.ReferenceDurations is not null && plan.ScaledByLimits)
            leading.Add(new SummaryRow("Reference durations [s]", string.Join("  ", plan.ReferenceDurations.Select(t => $"{t:F6}"))));
        if (plan.Scale is not null)
        {
            leading.Add(new SummaryRow("Common factor λ", $"{plan.Scale:F6}"));
            leading.Add(new SummaryRow("Binding limit", $"{plan.BindingQuantity} on axis {plan.BindingAxis}"));
        }

        leading.Add(new SummaryRow("Smoothness order m", $"{criterion.DerivativeOrder}"));
        leading.Add(new SummaryRow($"Smoothness cost J (order {criterion.DerivativeOrder}, sum over the axes)", $"{cost:G8}"));

        var limitRows = new List<Limit3DReportRow>();
        if (limits.HasAnyLimit)
        {
            limitRows = TrajectoryAnalyzer3D.LimitReport(trajectory, limits)
                .Select(r => new Limit3DReportRow("All segments", r.Quantity, r.Axis.ToUpperInvariant(), r.Peak, r.Limit, r.PercentOfLimit, r.PartLabel, r.Time))
                .OrderByDescending(r => r.PercentOfLimit)
                .ToList();
        }

        var notices = new List<string>();
        if (plan.ScaledByLimits)
        {
            notices.Add(
                $"Limits applied: all segment durations were multiplied by λ = {plan.Scale:F6} " +
                $"(binding: {plan.BindingQuantity} on axis {plan.BindingAxis}); total duration {plan.TotalDuration:F4} s.");
        }

        return Report(trajectory, maxOrder, limitRows, notices, leading, markers, states);
    }

    private static Point3D[] ParseWaypoints(PassThrough3DInputModel input)
    {
        if (input.Waypoints.Count < 2)
            throw new InvalidOperationException("At least two waypoints are required.");

        return input.Waypoints
            .Select(w => ParseVector(new VectorInput(w.X, w.Y, w.Z), $"Waypoint {w.Number}", required: true))
            .ToArray();
    }

    /// <summary>All segment durations filled (prescribed) or all blank (null); a partly filled column is an input error.</summary>
    private static double[]? ParseDurationColumn(PassThrough3DInputModel input, int segmentCount)
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

        return cells.Select((c, i) => ParsePositive(c, $"Segment {i + 1} duration (waypoint {i + 1} row)")).ToArray();
    }

    private static State3D Boundary(PassThrough3DInputModel input, bool isStart)
    {
        string side = isStart ? "start" : "end";
        Point3D v = ParseVector(isStart ? input.StartVelocity : input.EndVelocity, $"{side} velocity", required: false);
        Point3D a = input.SupportsAcceleration
            ? ParseVector(isStart ? input.StartAcceleration : input.EndAcceleration, $"{side} acceleration", required: false) : Point3D.Zero;
        Point3D j = input.SupportsJerk
            ? ParseVector(isStart ? input.StartJerk : input.EndJerk, $"{side} jerk", required: false) : Point3D.Zero;
        Point3D s = input.SupportsSnap
            ? ParseVector(isStart ? input.StartSnap : input.EndSnap, $"{side} snap", required: false) : Point3D.Zero;
        return new State3D(Point3D.Zero, v, a, j, s); // the position is the waypoint's
    }

    private static KinematicLimits Limits(PassThrough3DInputModel input)
    {
        double? Limit(bool use, bool supported, string text, string name) =>
            use && supported ? ParsePositive(text, $"Maximum {name}") : null;

        return KinematicLimits.FromValues(
            Limit(input.UseMaxVelocity, true, input.MaxVelocity, "velocity"),
            Limit(input.UseMaxAcceleration, input.SupportsAcceleration, input.MaxAcceleration, "acceleration"),
            Limit(input.UseMaxJerk, input.SupportsJerk, input.MaxJerk, "jerk"),
            Limit(input.UseMaxSnap, input.SupportsSnap, input.MaxSnap, "snap"));
    }
}
