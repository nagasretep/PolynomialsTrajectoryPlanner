using System.Globalization;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.PassThrough;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0020: the 3D pass-through entry point of the 3D service (no window).</summary>
public class PassThrough3DServiceTests
{
    private readonly Prototype3DTrajectoryService _service = new();

    /// <summary>The default inputs are the reference example of the theory: (0,0,0), (1,0,0), (1,3,0), (1,3,4).</summary>
    private static PassThrough3DInputModel ReferenceExample(bool withLimits)
    {
        var m = new PassThrough3DInputModel();
        if (withLimits)
        {
            m.UseMaxVelocity = true; m.MaxVelocity = "2";
            m.UseMaxAcceleration = true; m.MaxAcceleration = "3";
            m.UseMaxJerk = true; m.MaxJerk = "10";
            m.UseMaxSnap = true; m.MaxSnap = "50";
        }

        return m;
    }

    private static PassThrough3DInputModel Input(params (string X, string Y, string Z)[] points)
    {
        var m = new PassThrough3DInputModel();
        m.Waypoints.Clear();
        foreach (var (x, y, z) in points)
            m.Waypoints.Add(new Waypoint3DInputModel(m.Waypoints.Count + 1, x, y, z));
        m.Renumber();
        return m;
    }

    private static string Summary(Prototype3DComputationResult r, string item) =>
        r.SummaryRows.Single(s => s.Item.StartsWith(item)).Value;

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

    // ---- the reference example ---------------------------------------------------------------

    [Fact]
    public void ReferenceExample_WithLimits_LambdaTotalAndBindingPair()
    {
        var r = InCulture("en-US", () => _service.ComputePassThrough(ReferenceExample(withLimits: true)));

        Assert.Equal(14.971506, r.TotalDuration, 5);
        Assert.Equal("4.990502", Summary(r, "Common factor λ"));
        Assert.Equal("velocity on axis x", Summary(r, "Binding limit"));
        var notice = Assert.Single(r.Notices);
        Assert.Contains("λ = 4.990502", notice);
        Assert.Contains("binding: velocity on axis x", notice);

        // The peaks table is the Core's LimitReport, binding first.
        Assert.Equal(12, r.LimitRows.Count);
        Assert.Equal("Velocity", r.LimitRows[0].Quantity);
        Assert.Equal("X", r.LimitRows[0].Axis);
        Assert.Equal(100.0, r.LimitRows[0].PercentOfLimit, 1e-6);
        Assert.All(r.LimitRows, row => Assert.True(row.PercentOfLimit <= 100 + 1e-6));
    }

    [Fact]
    public void ReferenceExample_WithoutLimits_ThreeSecondsByTheChordRule()
    {
        var r = InCulture("en-US", () => _service.ComputePassThrough(ReferenceExample(withLimits: false)));

        Assert.Equal(3.0, r.TotalDuration, 12);
        Assert.Equal("0.375000", Summary(r, "Segment 1 duration"));
        Assert.Equal("1.125000", Summary(r, "Segment 2 duration"));
        Assert.Equal("1.500000", Summary(r, "Segment 3 duration"));
        Assert.Empty(r.Notices);
        Assert.Empty(r.LimitRows);
        Assert.DoesNotContain(r.SummaryRows, s => s.Item.StartsWith("Common factor"));
    }

    [Fact]
    public void TheWaypointsAreMarked_AndThePathPassesThroughThem()
    {
        var r = _service.ComputePassThrough(ReferenceExample(withLimits: true));

        Assert.Equal(4, r.Waypoints!.Count);
        Assert.Equal(new[] { 0.0, 1.871438, 1.871438 + 5.614315, 14.971506 }, r.Waypoints.Select(w => w.Time).ToArray(), new Tolerance(1e-5));
        foreach (var w in r.Waypoints)
            Assert.Equal(0.0, (r.StateAt(w.Time).Position - w.Point).Length, 1e-9);

        Assert.Equal(new[] { "Segment", "Segment", "Segment" }, r.PartRows.Select(p => p.Kind).ToArray());
        Assert.Equal(2, r.PartBoundaries.Count); // two interior junctions
    }

    [Fact]
    public void TheWaypointStates_AreReported_ForTheOrdersTheDegreeShares()
    {
        var r = _service.ComputePassThrough(ReferenceExample(withLimits: false));

        Assert.Equal(4 * 5, r.WaypointStates!.Count); // degree 9: position .. snap at every waypoint
        var v2 = r.WaypointStates.Single(s => s.Waypoint == 2 && s.Quantity == "Velocity");
        Assert.Equal(r.StateAt(0.375).Velocity.X, v2.X, 12);
        Assert.Equal(0.0, r.WaypointStates.Single(s => s.Waypoint == 1 && s.Quantity == "Velocity").X, 12); // rest start
    }

    [Fact]
    public void TheCost_IsTheSumOverTheAxes()
    {
        var r = InCulture("en-US", () => _service.ComputePassThrough(ReferenceExample(withLimits: false)));
        var plan = PassThroughPlanner3D.Plan(new[] { new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 3, 0), new Point3D(1, 3, 4) }, 9);

        double expected = SmoothnessCost.Of(plan.Segments.Select(s => s.XAxis), 4)
                          + SmoothnessCost.Of(plan.Segments.Select(s => s.YAxis), 4)
                          + SmoothnessCost.Of(plan.Segments.Select(s => s.ZAxis), 4);
        Assert.Equal(expected, double.Parse(Summary(r, "Smoothness cost"), CultureInfo.GetCultureInfo("en-US")), 1e-6 * expected);
    }

    // ---- durations -----------------------------------------------------------------------------

    [Fact]
    public void PrescribedDurationsViolatingALimit_NameSegmentAxisAndQuantity()
    {
        var m = ReferenceExample(withLimits: true);
        m.Waypoints[0].Duration = "0.5";
        m.Waypoints[1].Duration = "6";
        m.Waypoints[2].Duration = "8";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m));

        Assert.Contains("segment 1", ex.Message);
        Assert.Contains("on axis x", ex.Message);
        Assert.Contains("limit", ex.Message);
    }

    [Fact]
    public void ATotalDuration_IsSplitByTheChords()
    {
        var m = ReferenceExample(withLimits: true);
        m.TotalDuration = "16";

        var r = InCulture("en-US", () => _service.ComputePassThrough(m));

        Assert.Equal(16.0, r.TotalDuration, 12);
        Assert.Equal("2.000000", Summary(r, "Segment 1 duration"));
        Assert.Equal("8.000000", Summary(r, "Segment 3 duration"));
        Assert.Empty(r.Notices);
    }

    [Fact]
    public void DurationsAndTotalTogether_AndPartlyFilledDurations_AreInputErrors()
    {
        var both = ReferenceExample(false);
        foreach (var w in both.Waypoints.Take(3)) w.Duration = "1";
        both.TotalDuration = "3";
        Assert.Contains("both given", Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(both)).Message);

        var partly = ReferenceExample(false);
        partly.Waypoints[0].Duration = "1";
        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(partly));
        Assert.Contains("partly filled", ex.Message);
        Assert.Contains("waypoint 2", ex.Message);
    }

    // ---- input errors ------------------------------------------------------------------------------

    [Fact]
    public void FewerThanTwoWaypoints_IsAnInputError()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(Input(("0", "0", "0"))));

        Assert.Contains("two waypoints", ex.Message);
    }

    [Fact]
    public void AnEmptyOrNonNumericCoordinate_NamesTheRowAndTheAxis()
    {
        var empty = Input(("0", "0", "0"), ("1", "", "0"));
        Assert.Equal("Waypoint 2: the Y component is empty.",
            Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(empty)).Message);

        var text = Input(("0", "0", "0"), ("1", "2", "x"));
        Assert.Contains("Waypoint 2, Z component",
            Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(text)).Message);
    }

    [Fact]
    public void ASmoothnessOrderAboveP_IsRejected()
    {
        var m = ReferenceExample(false);
        m.Degree = 5;
        m.SmoothnessOrder = 4;

        Assert.Contains("Smoothness order", Assert.Throws<InvalidOperationException>(() => _service.ComputePassThrough(m)).Message);
    }

    // ---- end conditions, 1D agreement, culture -----------------------------------------------------

    [Fact]
    public void ANonZeroEndVelocityVector_IsHonoured()
    {
        var m = ReferenceExample(withLimits: true);
        m.EndAtRest = false;
        m.EndVelocity.X = "0";
        m.EndVelocity.Y = "0.2";
        m.EndVelocity.Z = "-0.3";

        var r = _service.ComputePassThrough(m);

        var end = r.StateAt(r.TotalDuration);
        Assert.Equal(0.2, end.Velocity.Y, 1e-9);
        Assert.Equal(-0.3, end.Velocity.Z, 1e-9);
        Assert.All(r.LimitRows, row => Assert.True(row.PercentOfLimit <= 100 + 1e-6));
        Assert.Contains(r.Notices, n => n.StartsWith("Limits applied"));
    }

    [Fact]
    public void AlongOneAxis_AgreesWithThe1DService()
    {
        var m3 = Input(("0", "2", "-1"), ("1", "2", "-1"), ("3", "2", "-1"), ("4", "2", "-1"));
        m3.UseMaxVelocity = true; m3.MaxVelocity = "2";
        m3.UseMaxAcceleration = true; m3.MaxAcceleration = "3";

        var m1 = new PassThroughInputModel();
        m1.Waypoints.Clear();
        foreach (string x in new[] { "0", "1", "3", "4" })
            m1.Waypoints.Add(new WaypointInputModel(m1.Waypoints.Count + 1, x));
        m1.Renumber();
        m1.UseMaxVelocity = true; m1.MaxVelocity = "2";
        m1.UseMaxAcceleration = true; m1.MaxAcceleration = "3";

        var r3 = _service.ComputePassThrough(m3);
        var r1 = new PrototypeTrajectoryService().ComputePassThrough(m1);

        Assert.Equal(r1.TotalDuration, r3.TotalDuration, 1e-9);
        var c1 = r1.CoefficientRows.ToList();
        var c3x = r3.CoefficientRows.Where(c => c.Segment.EndsWith(" X")).ToList();
        Assert.Equal(c1.Count, c3x.Count);
        for (int i = 0; i < c1.Count; i++)
            Assert.Equal(c1[i].Value, c3x[i].Value, 1e-9);
        Assert.Equal(r1.SummaryRows!.Single(s => s.Item.Contains('λ')).Value, Summary(r3, "Common factor λ"));
    }

    [Fact]
    public void ItalianInput_DecimalCommas()
    {
        InCulture("it-IT", () =>
        {
            var m = Input(("0", "0", "0"), ("0,5", "1,5", "0"), ("1", "1", "0,25"));
            m.UseMaxVelocity = true; m.MaxVelocity = "1,5";

            var r = _service.ComputePassThrough(m);

            Assert.Equal(0.0, (r.Waypoints![1].Point - new Point3D(0.5, 1.5, 0)).Length, 12);
            Assert.Contains(',', Summary(r, "Common factor λ"));
            Assert.Contains(r.Notices, n => n.Contains(','));
            return 0;
        });
    }

    private sealed class Tolerance : IEqualityComparer<double>
    {
        private readonly double _tolerance;
        public Tolerance(double tolerance) => _tolerance = tolerance;
        public bool Equals(double x, double y) => Math.Abs(x - y) <= _tolerance;
        public int GetHashCode(double obj) => 0;
    }
}
