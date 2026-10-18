using System.Globalization;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0014: one default rule for the durations, and the peaks against the limits.</summary>
public class PassThroughDefaultsAndPeaksTests
{
    private readonly PrototypeTrajectoryService _service = new();

    private static PassThroughInputModel Input(params string[] positions)
    {
        var m = new PassThroughInputModel();
        m.Waypoints.Clear();
        foreach (string p in positions)
            m.Waypoints.Add(new WaypointInputModel(m.Waypoints.Count + 1, p));
        m.Renumber();
        return m;
    }

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

    // ---- uniform default -------------------------------------------------------------------

    [Fact]
    public void UnequalSpacing_NoLimits_DurationsAreProportionalToDistance_TotalIsOneSecondPerSegment()
    {
        var r = _service.ComputePassThrough(Input("0", "1", "3"));

        double[] t = SegmentDurations(r);
        Assert.Equal(2.0, r.TotalDuration, 12);
        Assert.Equal(1.0 / 3 * 2, t[0], 1e-9); // distances 1 : 2 -> 2/3 s and 4/3 s
        Assert.Equal(2.0, t[1] / t[0], 1e-9);
    }

    [Fact]
    public void EqualSpacing_NoLimits_ReferenceDataUnchanged()
    {
        var r = _service.ComputePassThrough(Input("0", "1", "2"));

        Assert.Equal(2.0, r.TotalDuration, 12);
        Assert.Equal(2.275, r.WaypointStates![1].Velocity, 1e-6);
    }

    [Fact]
    public void WithLimits_TheReferenceProportionsAreTheSame_BeforeScaling()
    {
        var m = Input("0", "1", "3");
        m.UseMaxVelocity = true; m.MaxVelocity = "1";

        var scaled = _service.ComputePassThrough(m);
        var plain = _service.ComputePassThrough(Input("0", "1", "3"));

        double[] ts = SegmentDurations(scaled), tp = SegmentDurations(plain);
        Assert.Equal(tp[1] / tp[0], ts[1] / ts[0], 1e-9); // same proportions
        double lambda = double.Parse(scaled.SummaryRows!.Single(s => s.Item.Contains('λ')).Value, CultureInfo.CurrentCulture);
        Assert.Equal(lambda * tp[0], ts[0], 1e-5);
    }

    // ---- peaks against the limits ----------------------------------------------------------

    [Fact]
    public void NoLimit_NoPeaksTable()
    {
        var r = _service.ComputePassThrough(Input("0", "1", "2"));

        Assert.NotNull(r.LimitPeaks);
        Assert.Empty(r.LimitPeaks!);
    }

    [Fact]
    public void LimitedCase_TheBindingLimitIsAt100Percent_TheOthersBelow_SegmentsFrom1()
    {
        var m = Input("0", "1", "2");
        m.UseMaxVelocity = true; m.MaxVelocity = "2";
        m.UseMaxAcceleration = true; m.MaxAcceleration = "3";
        m.UseMaxJerk = true; m.MaxJerk = "10";
        m.UseMaxSnap = true; m.MaxSnap = "50";

        var r = _service.ComputePassThrough(m);

        var peaks = r.LimitPeaks!;
        Assert.Equal(new[] { "Velocity", "Acceleration", "Jerk", "Snap" }, peaks.Select(p => p.Quantity).ToArray());
        var acceleration = peaks.Single(p => p.Quantity == "Acceleration");
        Assert.Equal(100.0, acceleration.PercentOfLimit, 1e-4);
        Assert.Equal(3.0, acceleration.Limit);
        Assert.All(peaks.Where(p => p.Quantity != "Acceleration"), p => Assert.True(p.PercentOfLimit < 100.0 - 1e-3));
        Assert.All(peaks, p => Assert.InRange(p.Segment, 1, 2));
        Assert.All(peaks, p => Assert.Equal(100.0 * p.Peak / p.Limit, p.PercentOfLimit, 9));
    }

    [Fact]
    public void ThePeakSegment_IsTheFasterOne_WhenTheDistanceIsMuchLarger()
    {
        var m = Input("0", "0.1", "2.1");
        m.Waypoints[0].Duration = "1"; m.Waypoints[1].Duration = "1";
        m.UseMaxVelocity = true; m.MaxVelocity = "100";

        var r = _service.ComputePassThrough(m);

        Assert.Equal(2, r.LimitPeaks!.Single().Segment);
        Assert.True(r.LimitPeaks!.Single().PercentOfLimit < 100.0);
    }

    [Fact]
    public void LimitsTheDegreeDoesNotSupport_AreNotListed()
    {
        var m = Input("0", "1", "2");
        m.Degree = 3;
        m.UseMaxSnap = true; m.MaxSnap = "1";
        m.UseMaxVelocity = true; m.MaxVelocity = "5";

        var r = _service.ComputePassThrough(m);

        Assert.Equal("Velocity", r.LimitPeaks!.Single().Quantity);
    }

    [Fact]
    public void UnderItalianCulture_SummaryAndNoticeUseTheComma_PeaksAreNumbers()
    {
        InCulture("it-IT", () =>
        {
            var m = Input("0", "0,5", "1,5");
            m.UseMaxVelocity = true; m.MaxVelocity = "1,5";

            var r = _service.ComputePassThrough(m);

            Assert.Contains(',', r.SummaryRows!.Single(s => s.Item.StartsWith("Total duration")).Value);
            Assert.Equal(100.0, r.LimitPeaks!.Single().PercentOfLimit, 1e-4); // numbers: the window formats them with the culture
            return 0;
        });
    }
}
