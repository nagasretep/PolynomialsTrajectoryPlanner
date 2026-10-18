using System.Globalization;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0013: the pass-through entry point of the service (no window).</summary>
public class PassThroughServiceTests
{
    private readonly PrototypeTrajectoryService _service = new();

    private static PassThroughInputModel Input(int degree, params string[] positions)
    {
        var m = new PassThroughInputModel { Degree = degree };
        m.Waypoints.Clear();
        foreach (string p in positions)
            m.Waypoints.Add(new WaypointInputModel(m.Waypoints.Count + 1, p));
        m.Renumber();
        return m;
    }

    private static void SetLimits(PassThroughInputModel m, string? v = null, string? a = null, string? j = null, string? s = null)
    {
        if (v is not null) { m.UseMaxVelocity = true; m.MaxVelocity = v; }
        if (a is not null) { m.UseMaxAcceleration = true; m.MaxAcceleration = a; }
        if (j is not null) { m.UseMaxJerk = true; m.MaxJerk = j; }
        if (s is not null) { m.UseMaxSnap = true; m.MaxSnap = s; }
    }

    private static double PeakOf(PrototypeComputationResult r, string quantity) =>
        r.ExtremaRows.Where(e => e.Quantity == quantity).Max(e => Math.Max(Math.Abs(e.Minimum), Math.Abs(e.Maximum)));

    private static double[] SegmentDurations(PrototypeComputationResult r) =>
        r.CoefficientRows.Where(c => c.Coefficient == "T [s]").Select(c => c.Value).ToArray();

    private static T InCulture<T>(string culture, Func<T> func)
    {
        var saved = (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture);
        try
        {
            Thread.CurrentThread.CurrentCulture = new CultureInfo(culture);
            Thread.CurrentThread.CurrentUICulture = new CultureInfo(culture);
            return func();
        }
        finally
        {
            (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture) = saved;
        }
    }

    // ---- reference data of the theory ------------------------------------------------------

    [Fact]
    public void DefaultInputs_ReproduceTheTheoryReference_RestToRest()
    {
        var r = _service.ComputePassThrough(Input(9, "0", "1", "2"));

        Assert.Equal(2.0, r.TotalDuration, 12);
        Assert.Equal(new[] { 1.0, 1.0 }, SegmentDurations(r));
        Assert.Equal(3, r.WaypointStates!.Count);
        Assert.Equal(2.275, r.WaypointStates![1].Velocity, 1e-6); // theory reference value
        Assert.Empty(r.Notices);
        Assert.DoesNotContain(r.SummaryRows!, s => s.Item.Contains('λ'));
    }

    [Fact]
    public void ReferenceLimits_GiveTheShortestFeasibleSequence()
    {
        var m = Input(9, "0", "1", "2");
        SetLimits(m, v: "2", a: "3", j: "10", s: "50");

        var r = _service.ComputePassThrough(m);

        Assert.Equal(2 * 1.168271, r.TotalDuration, 1e-5);
        Assert.Equal(2.336542, r.TotalDuration, 1e-5);
        Assert.Equal(3.0, PeakOf(r, "Acceleration"), 1e-6); // the binding limit
        Assert.True(PeakOf(r, "Velocity") <= 2.0 + 1e-9);
        Assert.True(PeakOf(r, "Jerk") <= 10.0 + 1e-9);
        Assert.True(PeakOf(r, "Snap") <= 50.0 + 1e-9);

        var lambda = r.SummaryRows!.Single(s => s.Item.Contains('λ'));
        Assert.Equal(1.168271, double.Parse(lambda.Value, CultureInfo.CurrentCulture), 1e-5);
        Assert.Contains(r.Notices, n => n.Contains("λ"));
    }

    [Fact]
    public void LimitsThatAreNotReached_LeaveThe1sFallbackAndNoNotice_WhenTheyAreAlreadyRespectedByPrescribedDurations()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[0].Duration = "3";
        m.Waypoints[1].Duration = "3";
        SetLimits(m, v: "2", a: "3");

        var r = _service.ComputePassThrough(m);

        Assert.Equal(6.0, r.TotalDuration, 12);
        Assert.Empty(r.Notices);
    }

    // ---- durations -------------------------------------------------------------------------

    [Fact]
    public void PrescribedDurations_AreKept()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[0].Duration = "0.7";
        m.Waypoints[1].Duration = "1.3";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(new[] { 0.7, 1.3 }, SegmentDurations(r));
        Assert.Equal(2.0, r.TotalDuration, 12);
    }

    [Fact]
    public void PrescribedDurationsViolatingALimit_ErrorNamesSegmentAndLimit_NoResult()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[0].Duration = "0.3";
        m.Waypoints[1].Duration = "1.3";
        SetLimits(m, v: "2");

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("segment 1", ex.Message, StringComparison.OrdinalIgnoreCase);
        Assert.Contains("velocity", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void TotalDuration_IsDistributedProportionally_AndOnlyVerified()
    {
        var m = Input(9, "0", "1", "3");
        m.TotalDuration = "6";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(6.0, r.TotalDuration, 12);
        double[] t = SegmentDurations(r);
        Assert.Equal(2.0, t[0], 1e-9); // distances 1 and 2 -> 2 s and 4 s
        Assert.Equal(4.0, t[1], 1e-9);
    }

    [Fact]
    public void TotalDurationViolatingALimit_IsAnError()
    {
        var m = Input(9, "0", "1", "2");
        m.TotalDuration = "0.5";
        SetLimits(m, v: "2");

        Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));
    }

    [Fact]
    public void SegmentDurationsTogetherWithATotal_IsAnInputError()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[0].Duration = "1";
        m.Waypoints[1].Duration = "1";
        m.TotalDuration = "2";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("total", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void PartlyFilledDurations_IsAnInputError_NamingTheBlankRow()
    {
        var m = Input(9, "0", "1", "2", "3");
        m.Waypoints[0].Duration = "1";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("partly filled", ex.Message);
        Assert.Contains("waypoint 2", ex.Message);
    }

    [Fact]
    public void TheDurationCellOfTheLastWaypoint_IsIgnored()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[2].Duration = "99";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(2.0, r.TotalDuration, 12);
    }

    // ---- input errors ----------------------------------------------------------------------

    [Fact]
    public void FewerThanTwoWaypoints_IsAnInputError()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(Input(9, "0")));

        Assert.Contains("two waypoints", ex.Message);
    }

    [Fact]
    public void NonNumericPosition_IsReported_NamingTheField()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(Input(9, "0", "abc", "2")));

        Assert.Contains("Waypoint 2 position", ex.Message);
    }

    [Fact]
    public void NonPositiveDuration_IsReported_NamingTheField()
    {
        var m = Input(9, "0", "1", "2");
        m.Waypoints[0].Duration = "-1";
        m.Waypoints[1].Duration = "1";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("Segment 1 duration", ex.Message);
    }

    [Fact]
    public void SmoothnessOrderOutsideOneToP_IsRejected()
    {
        var m = Input(5, "0", "1", "2"); // p = 3
        m.SmoothnessOrder = 4;

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("Smoothness order", ex.Message);
    }

    [Fact]
    public void SmoothnessOrderDefaults_FollowTheDegree()
    {
        var m = new PassThroughInputModel();
        Assert.Equal(4, m.SmoothnessOrder); // degree 9: snap

        m.Degree = 7;
        Assert.Equal(4, m.SmoothnessOrder); // p = 4
        m.Degree = 5;
        Assert.Equal(3, m.SmoothnessOrder);
        Assert.Equal(new[] { 1, 2, 3 }, m.AvailableSmoothnessOrders);
        m.Degree = 3;
        Assert.Equal(2, m.SmoothnessOrder);
        m.Degree = 9;
        Assert.Equal(4, m.SmoothnessOrder);
        Assert.Equal(new[] { 1, 2, 3, 4, 5 }, m.AvailableSmoothnessOrders);
    }

    // ---- end conditions and other degrees ---------------------------------------------------

    [Fact]
    public void NonZeroEndVelocity_IsHonoured()
    {
        var m = Input(9, "0", "1", "2");
        m.EndAtRest = false;
        m.EndVelocity = "1.5";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(1.5, r.WaypointStates![2].Velocity, 1e-9);
        Assert.Equal(0.0, r.WaypointStates![0].Velocity, 1e-9);
        Assert.Equal(0.0, r.WaypointStates![2].Acceleration, 1e-9);
    }

    [Fact]
    public void NonZeroStartVelocityAndAcceleration_AreHonoured()
    {
        var m = Input(7, "0", "1", "3");
        m.StartAtRest = false;
        m.StartVelocity = "0.5";
        m.StartAcceleration = "-0.25";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(0.5, r.WaypointStates![0].Velocity, 1e-9);
        Assert.Equal(-0.25, r.WaypointStates![0].Acceleration, 1e-9);
    }

    [Fact]
    public void Degree3_PlotsPositionAndVelocityOnly()
    {
        var r = _service.ComputePassThrough(Input(3, "0", "1", "2"));

        Assert.Equal(2, r.Series.Count);
    }

    [Fact]
    public void ALimitTheDegreeDoesNotSupport_IsIgnored()
    {
        var m = Input(3, "0", "1", "2");
        SetLimits(m, s: "1"); // snap does not exist for degree 3

        var r = _service.ComputePassThrough(m);

        Assert.Equal(2.0, r.TotalDuration, 12);
        Assert.Empty(r.Notices);
    }

    // ---- report ----------------------------------------------------------------------------

    [Fact]
    public void Report_HasWaypointMarkers_TheExactCost_AndExtremaOverTheWholeTrajectory()
    {
        var r = _service.ComputePassThrough(Input(9, "0", "1", "2"));

        Assert.Equal(3, r.WaypointMarkers!.Count);
        Assert.Equal(new[] { 0.0, 1.0, 2.0 }, r.WaypointMarkers!.Select(w => w.Time).ToArray());
        Assert.Equal(new[] { 0.0, 1.0, 2.0 }, r.WaypointMarkers!.Select(w => w.Position).ToArray());
        Assert.Contains(r.SummaryRows!, s => s.Item.StartsWith("Smoothness cost"));
        Assert.Equal(5, r.ExtremaRows.Count); // position ... snap, one row each: the whole trajectory
        Assert.All(r.ExtremaRows, e => Assert.Equal("Trajectory", e.Segment));

        var velocity = r.ExtremaRows.Single(e => e.Quantity == "Velocity");
        Assert.Equal(2.275, velocity.Maximum, 1e-3); // symmetric profile: peak near the middle

        // Coefficients of every segment: T plus c0 ... c9.
        Assert.Equal(2 * 11, r.CoefficientRows.Count);
    }

    [Fact]
    public void TheCost_MatchesSmoothnessCostOf_OnTheSameSegments()
    {
        var r = _service.ComputePassThrough(Input(9, "0", "1", "2"));

        var cost = r.SummaryRows!.Single(s => s.Item.StartsWith("Smoothness cost"));
        double shown = double.Parse(cost.Value, CultureInfo.CurrentCulture);
        Assert.True(shown > 0 && double.IsFinite(shown));
    }

    // ---- culture ---------------------------------------------------------------------------

    [Fact]
    public void ItalianInput_DecimalCommas_AreParsed_AndOutputUsesTheCommaToo()
    {
        InCulture("it-IT", () =>
        {
            var m = Input(9, "0", "0,5", "1");
            SetLimits(m, v: "1,5");

            var r = _service.ComputePassThrough(m);

            Assert.True(r.TotalDuration > 0);
            var total = r.SummaryRows!.Single(s => s.Item.StartsWith("Total duration"));
            Assert.Contains(',', total.Value);
            Assert.DoesNotContain('.', total.Value);
            Assert.Contains(r.Notices, n => n.Contains(','));
            return 0;
        });
    }

    [Fact]
    public void InvariantInput_DecimalPoint_IsAlsoAccepted_UnderItalianCulture()
    {
        InCulture("it-IT", () =>
        {
            var r = _service.ComputePassThrough(Input(9, "0", "0.5", "1"));

            Assert.Equal(2.0, r.TotalDuration, 12);
            return 0;
        });
    }
}
