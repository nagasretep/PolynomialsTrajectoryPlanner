using TrajectoryPlanner.App;

namespace TrajectoryPlanner.App.Tests;

/// <summary>
/// Service-level tests (request 0009, Part B.4): the cases that were checked by hand with a throw-away console
/// project in requests 0006-0008, plus the blending cases of 0009. The service is a plain class; no window is opened.
/// </summary>
public class PrototypeTrajectoryServiceTests
{
    private readonly PrototypeTrajectoryService _service = new();

    private static SegmentInputModel Segment(
        int number, int degree, string x0, string x1, string v0 = "0", string v1 = "0",
        string? duration = null, string? vMax = null, string? aMax = null)
    {
        var m = new SegmentInputModel(number)
        {
            Degree = degree,
            StartPosition = x0, EndPosition = x1,
            StartVelocity = v0, EndVelocity = v1,
            StartAcceleration = "0", EndAcceleration = "0",
            StartJerk = "0", EndJerk = "0",
            StartSnap = "0", EndSnap = "0",
            UseDuration = duration is not null
        };
        if (duration is not null) m.Duration = duration;
        if (vMax is not null) { m.UseMaxVelocity = true; m.MaxVelocity = vMax; }
        if (aMax is not null) { m.UseMaxAcceleration = true; m.MaxAcceleration = aMax; }
        return m;
    }

    private static SegmentInputModel WithConstantVelocity(SegmentInputModel m, string start, string end, string speed)
    {
        m.UseConstantVelocity = true;
        m.ConstantVelocityStart = start;
        m.ConstantVelocityEnd = end;
        m.ConstantVelocitySpeed = speed;
        return m;
    }

    private static double[] Durations(PrototypeComputationResult r) =>
        r.CoefficientRows.Where(c => c.Coefficient == "T [s]").Select(c => c.Value).ToArray();

    private static double PeakVelocity(PrototypeComputationResult r) =>
        r.ExtremaRows.Where(e => e.Quantity == "Velocity").Max(e => Math.Max(Math.Abs(e.Minimum), Math.Abs(e.Maximum)));

    // ---- plain segments --------------------------------------------------------------------

    [Fact]
    public void NoSegments_Rejected()
    {
        Assert.Throws<InvalidOperationException>(() => _service.Compute(Array.Empty<SegmentInputModel>()));
    }

    [Fact]
    public void SingleSegmentWithoutLimits_UsesTheDefaultDuration()
    {
        var r = _service.Compute(new[] { Segment(1, 9, "0", "1") });

        Assert.Equal(1.0, r.TotalDuration, 12);
        Assert.Equal(new[] { 1.0 }, Durations(r));
        Assert.Equal(2.4609375, PeakVelocity(r), 1e-6); // 315/128
        Assert.Equal(300, r.Series[0].Times.Length);
        Assert.Equal(5, r.Series.Count); // position ... snap
    }

    [Fact]
    public void CoefficientRows_StartWithTheDuration_AndUseCiLabels()
    {
        var r = _service.Compute(new[] { Segment(1, 5, "0", "10", duration: "4") });

        var labels = r.CoefficientRows.Select(c => c.Coefficient).ToArray();
        Assert.Equal(new[] { "T [s]", "c0", "c1", "c2", "c3", "c4", "c5" }, labels);
        Assert.DoesNotContain(labels, l => l.StartsWith('a'));
    }

    [Fact]
    public void AutoDurationWithAVelocityLimit_IsTheShortestFeasibleDuration()
    {
        var r = _service.Compute(new[] { Segment(1, 9, "0", "1", vMax: "1") });

        Assert.Equal(315.0 / 128, r.TotalDuration, 1e-6);
        Assert.Equal(1.0, PeakVelocity(r), 1e-6);
    }

    [Fact]
    public void AutoDurationWithALooseLimit_IsShorterThanTheDefaultSecond()
    {
        var r = _service.Compute(new[] { Segment(1, 9, "0", "1", vMax: "10") });

        Assert.Equal(315.0 / 1280, r.TotalDuration, 1e-6);
    }

    [Fact]
    public void AutoDurationWithAnAccelerationLimit_Degree7()
    {
        var r = _service.Compute(new[] { Segment(1, 7, "0", "6", aMax: "3") });

        Assert.Equal(3.876387, r.TotalDuration, 1e-5);
    }

    [Fact]
    public void PrescribedDurationViolatingALimit_IsReportedNotRepaired()
    {
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { Segment(1, 9, "0", "1", duration: "1", vMax: "1") }));

        Assert.StartsWith("Segment 1:", ex.Message);
        Assert.Contains("violates the active kinematic limits", ex.Message);
    }

    [Fact]
    public void ImpossibleLimits_AreReportedWithTheSegmentNumber()
    {
        // The segment starts at speed 10, above the velocity limit 1: no duration can satisfy it.
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { Segment(1, 5, "0", "1", v0: "10", vMax: "1") }));

        Assert.StartsWith("Segment 1:", ex.Message);
    }

    [Fact]
    public void ZeroLengthSegmentWithLimits_UsesTheDefaultDuration()
    {
        var r = _service.Compute(new[] { Segment(1, 9, "3", "3", vMax: "1") });

        Assert.Equal(1.0, r.TotalDuration, 12);
    }

    [Fact]
    public void NonNumericInput_IsReported()
    {
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { Segment(1, 9, "abc", "1") }));

        Assert.Contains("must be a valid number", ex.Message);
    }

    [Fact]
    public void ExtremaRows_ExistForEverySupportedOrder()
    {
        var r = _service.Compute(new[] { Segment(1, 7, "0", "1") });

        Assert.Equal(new[] { "Position", "Velocity", "Acceleration", "Jerk" }, r.ExtremaRows.Select(e => e.Quantity).ToArray());
    }

    [Fact]
    public void ResultFunctions_EvaluateTheTrajectory()
    {
        var r = _service.Compute(new[] { Segment(1, 9, "0", "1") });

        Assert.Equal(0.0, r.PositionAt(0), 1e-9);
        Assert.Equal(1.0, r.PositionAt(r.TotalDuration), 1e-9);
        Assert.Equal(0.5, r.PositionAt(0.5), 1e-9);
        Assert.Equal(315.0 / 128, r.DerivativeAt(0.5, 1), 1e-9);
    }

    // ---- constant velocity -----------------------------------------------------------------

    [Fact]
    public void ConstantVelocity_WithoutLimits_UsesTheMeanVelocityRule()
    {
        var r = _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100"), "20", "80", "15") });

        Assert.Equal(new[] { 2.0 * 20 / 15, 4.0, 2.0 * 20 / 15 }, Durations(r).Select(d => Math.Round(d, 9)).ToArray(),
            new ToleranceComparer(1e-6));
        Assert.Equal(15.0, PeakVelocity(r), 1e-6);
    }

    [Fact]
    public void ConstantVelocity_PrescribedTotal_IsSplitByCore()
    {
        var r = _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100", duration: "20"), "20", "80", "15") });

        Assert.Equal(20.0, r.TotalDuration, 1e-9);
        Assert.Equal(new[] { 8.0, 4.0, 8.0 }, Durations(r), new ToleranceComparer(1e-9));
    }

    [Fact]
    public void ConstantVelocity_TotalNotAboveTheStretch_IsReported()
    {
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100", duration: "3"), "20", "80", "15") }));

        Assert.Contains("T_cv", ex.Message);
        Assert.StartsWith("Segment 1:", ex.Message);
    }

    [Fact]
    public void ConstantVelocity_AccelerationLimit_TransitionsTakeTheirShortestFeasibleDuration()
    {
        var r = _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100", aMax: "6"), "20", "80", "15") });

        Assert.Equal(new[] { 13.011293, 4.0, 13.011293 }, Durations(r), new ToleranceComparer(1e-5));
    }

    [Fact]
    public void ConstantVelocity_SpeedAboveTheVelocityLimit_IsReported()
    {
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100", vMax: "10"), "20", "80", "15") }));

        Assert.Contains("constant speed", ex.Message);
    }

    [Fact]
    public void ConstantVelocity_PrescribedTotalTooShortForTheLimits_NamesTheMinimumTotal()
    {
        var ex = Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100", duration: "8", aMax: "2"), "20", "80", "15") }));

        Assert.Contains("minimum feasible total", ex.Message);
    }

    [Fact]
    public void ConstantVelocity_EqualStretchPositions_IsReported()
    {
        Assert.Throws<InvalidOperationException>(
            () => _service.Compute(new[] { WithConstantVelocity(Segment(1, 9, "0", "100"), "50", "50", "15") }));
    }

    // ---- blending --------------------------------------------------------------------------

    private static SegmentInputModel[] Corner(int degree, string? vMax = null)
    {
        var a = Segment(1, degree, "0", "2", v1: "2", duration: "1", vMax: vMax);
        var b = Segment(2, degree, "2", "0.5", v0: "-2", duration: "1.5", vMax: vMax);
        a.BlendToNext = "1.5";
        b.BlendToPrevious = "1.2";
        return new[] { a, b };
    }

    [Fact]
    public void Blend_Degree9_KeepsTheTotalAndInheritsTheDurationWithoutLimits()
    {
        var r = _service.Compute(Corner(9));

        Assert.Equal(2.5, r.TotalDuration, 1e-9);
        Assert.Equal(new[] { 0.765826, 0.642911, 1.091263 }, Durations(r), new ToleranceComparer(1e-5));
        Assert.Contains(r.CoefficientRows, c => c.Segment == "Blend 1-2");
    }

    [Fact]
    public void Blend_BetweenDegree5Segments_IsADegree5Polynomial()
    {
        var r = _service.Compute(Corner(5));

        var blendRows = r.CoefficientRows.Where(c => c.Segment == "Blend 1-2" && c.Coefficient.StartsWith('c')).ToArray();
        Assert.Equal(6, blendRows.Length); // c0 ... c5
    }

    [Fact]
    public void Blend_WithALooseLimit_KeepsTheInheritedDuration_AndTheTotal()
    {
        // Request 0010 (D19): in 0009 this case shortened the blend to 0.100 s (peak v = 8); the inherited blend
        // (0.643 s) is feasible, so it is now kept.
        var r = _service.Compute(Corner(9, vMax: "8"));

        Assert.Equal(new[] { 0.765826, 0.642911, 1.091263 }, Durations(r), new ToleranceComparer(1e-5));
        Assert.Equal(2.5, r.TotalDuration, 1e-9);
        Assert.True(PeakVelocity(r) < 8.0);
    }

    [Fact]
    public void Blend_PositionOutsideTheSegment_IsReportedNamingTheBlend()
    {
        var segments = Corner(9);
        segments[0].BlendToNext = "5";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(segments));

        Assert.StartsWith("Blend 1-2:", ex.Message);
        Assert.Contains("outside the segment", ex.Message);
    }

    [Fact]
    public void Blend_NeedsBothValues()
    {
        var segments = Corner(9);
        segments[1].BlendToPrevious = "";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(segments));

        Assert.Contains("requires both values", ex.Message);
    }

    [Fact]
    public void Blend_WithAConstantVelocitySegment_IsNotSupported()
    {
        var segments = Corner(9);
        WithConstantVelocity(segments[1], "1", "1.5", "1");

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(segments));

        Assert.Contains("only for plain polynomial segments", ex.Message);
    }

    [Fact]
    public void TwoSegmentsWithoutBlend_AreConcatenated()
    {
        var r = _service.Compute(new[]
        {
            Segment(1, 9, "0", "2", duration: "1"),
            Segment(2, 9, "2", "0.5", duration: "1.5")
        });

        Assert.Equal(2.5, r.TotalDuration, 1e-9);
        Assert.Equal(4.921875, PeakVelocity(r), 1e-6);
    }

    private sealed class ToleranceComparer : IEqualityComparer<double>
    {
        private readonly double _tolerance;
        public ToleranceComparer(double tolerance) => _tolerance = tolerance;
        public bool Equals(double a, double b) => Math.Abs(a - b) <= _tolerance;
        public int GetHashCode(double value) => 0;
    }
}
