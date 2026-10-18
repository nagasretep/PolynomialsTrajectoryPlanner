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
/// Thin layer of the 3D mode (request 0018): parses the vector inputs, builds the trajectory with
/// <see cref="TrajectoryBuilder3D"/>, analyses it with <see cref="TrajectoryAnalyzer3D"/> and formats the results.
/// Durations, limits, blend windows, constant-speed geometry and every check of them are done by the Core; the App only
/// turns text into the builder's arguments and the Core's answers into tables, curves and messages.
/// </summary>
public sealed partial class Prototype3DTrajectoryService
{
    /// <summary>Samples per plotted curve: at least this many, or this many per part, whichever is larger.</summary>
    private const int MinimumSamples = 600;

    private const int SamplesPerPart = 150;

    private static readonly string[] AxisNames = { "X", "Y", "Z" };

    public Prototype3DComputationResult Compute(IReadOnlyList<Segment3DInputModel> inputs)
    {
        if (inputs.Count == 0)
            throw new InvalidOperationException("At least one segment is required.");

        var parsed = inputs.Select(Parse).ToList();
        CheckBlendsAndStretches(inputs);

        var builder = new TrajectoryBuilder3D();
        foreach (var p in parsed)
            CallCore(p.Label, () => builder.AddSegment(p.Start, p.End, p.Duration, p.Degree, p.Limits));

        var originals = builder.Parts.Select(part => (PolynomialSegment3D)part.Segment).ToList();

        // Blends, from the first pair to the last. A blend value given as a fraction is turned into the point of the
        // original segment at that fraction, and the Core's point form is used: a segment blended on both sides is then
        // blended toward the next one on what is left of it, with the value still meaning a place on the original segment.
        var blendsBefore = new int[inputs.Count + 1];
        for (int i = 0; i < inputs.Count - 1; i++)
        {
            blendsBefore[i + 1] = blendsBefore[i];
            if (!inputs[i].HasBlendToNext)
                continue;

            string blendLabel = $"Blend {i + 1}-{i + 2}";
            Point3D start = BlendPoint(inputs[i], originals[i], next: true);
            Point3D end = BlendPoint(inputs[i + 1], originals[i + 1], next: false);
            KinematicLimits limits = KinematicLimits.MostRestrictive(parsed[i].Limits, parsed[i + 1].Limits);

            int precedingIndex = i + blendsBefore[i];
            CallCore(blendLabel, () => builder.AddBlendBetween(precedingIndex, precedingIndex + 1, start, end, limits: limits));
            blendsBefore[i + 1]++;
        }

        // Constant-speed stretches, from the last segment to the first, so that the earlier positions do not move.
        for (int i = inputs.Count - 1; i >= 0; i--)
        {
            if (!inputs[i].UseConstantSpeed)
                continue;

            var input = inputs[i];
            string label = parsed[i].Label;
            double speed = ParsePositive(input.Speed, $"{label} constant speed");
            StretchSpecification specification = StretchOf(input, label);
            double? entry = ParseOptionalPositive(input.EntryDuration, $"{label} entry duration");
            double? exit = ParseOptionalPositive(input.ExitDuration, $"{label} exit duration");
            double? total = ParseOptionalPositive(input.StretchTotalDuration, $"{label} total duration of the stretch");

            int pieceIndex = i + blendsBefore[i];
            CallCore(label, () => builder.AddConstantSpeed(
                pieceIndex, specification, speed, limits: parsed[i].Limits,
                entryDuration: entry, exitDuration: exit, totalDuration: total));
        }

        Trajectory3D trajectory = builder.Build();

        int maxOrder = parsed.Max(p => BoundaryConditionValidator.DerivativeLevelsForDegree(p.Degree) - 1);
        if (inputs.Any(x => x.UseConstantSpeed))
            maxOrder = 4; // the stretch supplies every order up to snap

        var notices = new List<string>();
        foreach (var part in trajectory.Parts.Where(p => p.Kind == PartKind.Blend && p.AdjustedByLimits))
        {
            notices.Add(
                $"{part.Label}: duration adjusted from {part.InheritedDuration:F4} s to {part.Duration:F4} s to respect the limits.");
        }

        return Report(trajectory, maxOrder, LimitRows(trajectory, parsed), notices, []);
    }

    // ---- report (shared by the 3D mode and the 3D pass-through mode) ------------------------------

    private static Prototype3DComputationResult Report(
        Trajectory3D trajectory, int maxOrder, List<Limit3DReportRow> limitRows, List<string> notices,
        List<SummaryRow> leadingSummary,
        IReadOnlyList<WaypointMarker3D>? waypoints = null, IReadOnlyList<WaypointState3DRow>? waypointStates = null)
    {
        var parts = trajectory.Parts;

        TrajectorySamples3D samples = trajectory.Sample(Math.Max(MinimumSamples, parts.Count * SamplesPerPart), maxOrder);
        var series = new List<Series3D>();
        for (int order = 0; order <= maxOrder; order++)
        {
            Point3D[] v = samples.Values[order];
            series.Add(new Series3D(
                QuantityName(order), order, samples.Times,
                v.Select(p => p.X).ToArray(), v.Select(p => p.Y).ToArray(), v.Select(p => p.Z).ToArray(),
                v.Select(p => p.Length).ToArray()));
        }

        // The path, piece by piece (the junction times are among the samples, so neighbouring pieces meet).
        var pieces = new List<PathPiece>();
        foreach (var part in parts)
        {
            double eps = 1e-12 * Math.Max(1.0, trajectory.TotalDuration);
            var indices = Enumerable.Range(0, samples.Times.Length)
                .Where(k => samples.Times[k] >= part.StartTime - eps && samples.Times[k] <= part.EndTime + eps).ToArray();
            Point3D[] p = indices.Select(k => samples.Values[0][k]).ToArray();
            pieces.Add(new PathPiece(part.Label, KindName(part.Kind), p.Select(q => q.X).ToArray(), p.Select(q => q.Y).ToArray(), p.Select(q => q.Z).ToArray()));
        }

        var endpoints = new List<Point3D>();
        foreach (var group in parts.GroupBy(part => part.OriginalSegmentIndex))
        {
            endpoints.Add(trajectory.EvaluatePosition(group.Min(g => g.StartTime)));
            endpoints.Add(trajectory.EvaluatePosition(Math.Min(group.Max(g => g.EndTime), trajectory.TotalDuration)));
        }

        var partRows = parts.Select(part => new PartReportRow(
            part.Label, KindName(part.Kind), part.StartTime, part.Duration,
            part.InheritedDuration,
            part.Kind == PartKind.Blend ? (part.AdjustedByLimits ? "yes" : "no") : string.Empty,
            part.StretchLength, part.StretchSpeed, part.AdmissibleSpeed, part.BindingLimit ?? string.Empty)).ToList();

        TrajectoryExtrema3D extrema = TrajectoryAnalyzer3D.Extrema(trajectory);
        var extremaRows = new List<Extrema3DReportRow>();
        for (int order = 0; order <= maxOrder; order++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                AxisExtremum e = extrema.Of(axis, order);
                extremaRows.Add(new Extrema3DReportRow(
                    QuantityName(order), AxisNames[axis], e.Minimum, e.MinimumTime, e.Maximum, e.MaximumTime,
                    e.MaxAbsolute, e.MaxAbsoluteTime, parts[e.MaxAbsolutePart].Label));
            }
        }

        var coefficientRows = new List<CoefficientReportRow>();
        foreach (var part in parts)
        {
            if (part.Segment is not PolynomialSegment3D polynomial)
                continue;

            var axes = new[] { polynomial.XAxis, polynomial.YAxis, polynomial.ZAxis };
            for (int axis = 0; axis < 3; axis++)
            {
                string label = $"{part.Label} {AxisNames[axis]}";
                coefficientRows.Add(new CoefficientReportRow(label, "T [s]", polynomial.Duration));
                for (int i = 0; i <= polynomial.Degree; i++)
                    coefficientRows.Add(new CoefficientReportRow(label, $"c{i}", axes[axis].Coefficients[i]));
            }
        }

        double pathLength = TrajectoryAnalyzer3D.PathLength(trajectory);
        var summary = new List<SummaryRow>(leadingSummary)
        {
            new("Total duration [s]", $"{trajectory.TotalDuration:F6}"),
            new("Path length", $"{pathLength:F6}"),
            new("Parts", $"{parts.Count}"),
            new("Largest speed |v|", $"{extrema.MaxSpeed:F6} at t = {extrema.MaxSpeedTime:F4} s")
        };

        return new Prototype3DComputationResult(
            series, pieces, endpoints, parts.Skip(1).Select(p => p.StartTime).ToList(),
            partRows, extremaRows, limitRows, coefficientRows, summary,
            t => trajectory.EvaluateState(Math.Clamp(t, 0.0, trajectory.TotalDuration)),
            trajectory.TotalDuration, pathLength, notices, waypoints, waypointStates);
    }

    /// <summary>
    /// The Core's limit report for each run of pieces that share one set of limits: the pieces of a segment have that
    /// segment's limits, a blend the most restrictive of its neighbours. Rows sorted with the highest percentage first.
    /// </summary>
    private static List<Limit3DReportRow> LimitRows(Trajectory3D trajectory, List<ParsedSegment> parsed)
    {
        var parts = trajectory.Parts;
        var rows = new List<Limit3DReportRow>();

        int start = 0;
        while (start < parts.Count)
        {
            var (owner, limits) = LimitsOf(parts[start], parsed);
            int end = start;
            while (end + 1 < parts.Count && LimitsOf(parts[end + 1], parsed).Owner == owner)
                end++;

            if (limits.HasAnyLimit)
            {
                var group = parts.Skip(start).Take(end - start + 1).ToList();
                var sub = new Trajectory3D(group.Select(g => g.Segment), group);
                double offset = parts[start].StartTime;
                foreach (var r in TrajectoryAnalyzer3D.LimitReport(sub, limits))
                {
                    rows.Add(new Limit3DReportRow(
                        owner, r.Quantity, r.Axis.ToUpperInvariant(), r.Peak, r.Limit, r.PercentOfLimit, r.PartLabel, offset + r.Time));
                }
            }

            start = end + 1;
        }

        return rows.OrderByDescending(r => r.PercentOfLimit).ToList();
    }

    private static (string Owner, KinematicLimits Limits) LimitsOf(TrajectoryPart3D part, List<ParsedSegment> parsed) =>
        part.Kind == PartKind.Blend
            ? (part.Label, KinematicLimits.MostRestrictive(parsed[part.OriginalSegmentIndex].Limits, parsed[part.SecondSegmentIndex!.Value].Limits))
            : (parsed[part.OriginalSegmentIndex].Label, parsed[part.OriginalSegmentIndex].Limits);

    // ---- input ------------------------------------------------------------------------------------

    private sealed record ParsedSegment(string Label, int Degree, State3D Start, State3D End, double? Duration, KinematicLimits Limits);

    private static ParsedSegment Parse(Segment3DInputModel input)
    {
        string label = $"Segment {input.SegmentNumber}";

        State3D State(bool isStart)
        {
            string side = isStart ? "start" : "end";
            Point3D position = ParseVector(isStart ? input.StartPosition : input.EndPosition, $"{label} {side} position", required: true);
            Point3D velocity = ParseVector(isStart ? input.StartVelocity : input.EndVelocity, $"{label} {side} velocity", required: false);
            Point3D acceleration = input.SupportsAcceleration
                ? ParseVector(isStart ? input.StartAcceleration : input.EndAcceleration, $"{label} {side} acceleration", required: false)
                : Point3D.Zero;
            Point3D jerk = input.SupportsJerk
                ? ParseVector(isStart ? input.StartJerk : input.EndJerk, $"{label} {side} jerk", required: false)
                : Point3D.Zero;
            Point3D snap = input.SupportsSnap
                ? ParseVector(isStart ? input.StartSnap : input.EndSnap, $"{label} {side} snap", required: false)
                : Point3D.Zero;
            return new State3D(position, velocity, acceleration, jerk, snap);
        }

        double? Limit(bool use, bool supported, string text, string name) =>
            use && supported ? ParsePositive(text, $"{label} maximum {name}") : null;

        var limits = KinematicLimits.FromValues(
            Limit(input.UseMaxVelocity, true, input.MaxVelocity, "velocity"),
            Limit(input.UseMaxAcceleration, input.SupportsAcceleration, input.MaxAcceleration, "acceleration"),
            Limit(input.UseMaxJerk, input.SupportsJerk, input.MaxJerk, "jerk"),
            Limit(input.UseMaxSnap, input.SupportsSnap, input.MaxSnap, "snap"));

        return new ParsedSegment(
            label, input.Degree, State(true), State(false), ParseOptionalPositive(input.Duration, $"{label} duration"), limits);
    }

    /// <summary>The rule of this step (the Core blends plain segments only) and the pairing of blend values.</summary>
    private static void CheckBlendsAndStretches(IReadOnlyList<Segment3DInputModel> inputs)
    {
        for (int i = 0; i < inputs.Count; i++)
        {
            if (!inputs[i].UseConstantSpeed)
                continue;

            var involved = new List<int>();
            if (inputs[i].HasBlendToPrevious || inputs[i].HasBlendToNext) involved.Add(i + 1);
            if (i > 0 && inputs[i - 1].HasBlendToNext) involved.Add(i);
            if (i + 1 < inputs.Count && inputs[i + 1].HasBlendToPrevious) involved.Add(i + 2);
            if (involved.Count > 0)
            {
                throw new InvalidOperationException(
                    $"Segment {i + 1} has a constant-speed stretch, so it cannot be blended with a neighbour: remove the blend values " +
                    $"of segment(s) {string.Join(", ", involved.Distinct().OrderBy(n => n))} toward it, or the stretch.");
            }
        }

        for (int i = 0; i < inputs.Count - 1; i++)
        {
            if (inputs[i].HasBlendToNext != inputs[i + 1].HasBlendToPrevious)
                throw new InvalidOperationException($"Blend between segment {i + 1} and segment {i + 2} requires both values.");
        }
    }

    private static Point3D BlendPoint(Segment3DInputModel input, PolynomialSegment3D original, bool next)
    {
        string label = $"Segment {input.SegmentNumber} {(next ? "final" : "initial")} blend value";
        if (input.BlendByPoints)
            return ParseVector(next ? input.BlendToNextPoint : input.BlendToPreviousPoint, label, required: true);

        double tau = ParseFraction(next ? input.BlendToNext : input.BlendToPrevious, label);
        return original.EvaluatePosition(tau * original.Duration);
    }

    private static StretchSpecification StretchOf(Segment3DInputModel input, string label)
    {
        if (input.StretchKind == StretchInputKind.Points)
        {
            return StretchSpecification.FromPoints(
                ParseVector(input.StretchStartPoint, $"{label} stretch start point", required: true),
                ParseVector(input.StretchEndPoint, $"{label} stretch end point", required: true));
        }

        double a = ParseFraction(input.StretchStartFraction, $"{label} stretch start");
        double b = ParseFraction(input.StretchEndFraction, $"{label} stretch end");
        return input.StretchKind == StretchInputKind.ParameterFractions
            ? StretchSpecification.FromParameterFractions(a, b)
            : StretchSpecification.FromArcLengthFractions(a, b);
    }

    private static Point3D ParseVector(VectorInput vector, string label, bool required)
    {
        var text = new[] { vector.X, vector.Y, vector.Z };
        var values = new double[3];
        for (int axis = 0; axis < 3; axis++)
        {
            string box = text[axis]?.Trim() ?? string.Empty;
            if (box.Length == 0)
            {
                if (required)
                    throw new InvalidOperationException($"{label}: the {AxisNames[axis]} component is empty.");
                values[axis] = 0;
                continue;
            }

            if (!InputNumber.TryParse(box, out values[axis]))
            {
                throw new InvalidOperationException(
                    $"{label}, {AxisNames[axis]} component, must be a valid number (decimal point or comma, no thousands separators): \"{box}\".");
            }
        }

        return new Point3D(values[0], values[1], values[2]);
    }

    private static double ParseFraction(string text, string label)
    {
        double value = ParseRequired(text, label);
        if (value < 0 || value > 1)
            throw new InvalidOperationException($"{label} must be a fraction between 0 and 1, got {value}.");
        return value;
    }

    private static double ParseRequired(string text, string label)
    {
        if (!InputNumber.TryParse(text, out double value))
            throw new InvalidOperationException($"{label} must be a valid number (decimal point or comma, no thousands separators): \"{text}\".");
        return value;
    }

    private static double ParsePositive(string text, string label)
    {
        double value = ParseRequired(text, label);
        if (value <= 0)
            throw new InvalidOperationException($"{label} must be greater than zero.");
        return value;
    }

    private static double? ParseOptionalPositive(string text, string label) =>
        string.IsNullOrWhiteSpace(text) ? null : ParsePositive(text, label);

    /// <summary>Runs a Core call and gives its explicit failures one exception type whose message names the segment or blend.</summary>
    private static void CallCore(string label, Action call)
    {
        try
        {
            call();
        }
        catch (Exception ex) when (ex is DurationSearchFailedException or KinematicLimitsViolatedException
                                       or ArgumentException or NotSupportedException)
        {
            throw new InvalidOperationException($"{label}: {ex.Message}", ex);
        }
    }

    private static string KindName(PartKind kind) => kind switch
    {
        PartKind.Segment => "Segment",
        PartKind.Blend => "Blend",
        PartKind.CvEntry => "Entry",
        PartKind.CvStretch => "Stretch",
        _ => "Exit"
    };

    private static string QuantityName(int order) => order switch
    {
        0 => "Position",
        1 => "Velocity",
        2 => "Acceleration",
        3 => "Jerk",
        _ => "Snap"
    };
}
