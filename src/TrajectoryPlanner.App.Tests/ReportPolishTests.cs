using System.Globalization;
using System.Windows.Controls;
using System.Windows.Data;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0012: blend adjustment notice, and one output culture.</summary>
public class ReportPolishTests
{
    private static readonly CultureInfo Italian = new("it-IT");
    private static readonly CultureInfo English = new("en-US");

    private static SegmentInputModel Segment(int number, string x0, string x1, string v0, string v1, string duration, string? aMax)
    {
        var m = new SegmentInputModel(number)
        {
            Degree = 9,
            StartPosition = x0, EndPosition = x1,
            StartVelocity = v0, EndVelocity = v1,
            StartAcceleration = "0", EndAcceleration = "0",
            StartJerk = "0", EndJerk = "0",
            StartSnap = "0", EndSnap = "0",
            UseDuration = true, Duration = duration
        };
        if (aMax is not null) { m.UseMaxAcceleration = true; m.MaxAcceleration = aMax; }
        return m;
    }

    private static SegmentInputModel[] Corner(string? aMax)
    {
        var a = Segment(1, "0", "2", "0", "2", "1", aMax);
        var b = Segment(2, "2", "0.5", "-2", "0", "1.5", aMax);
        a.BlendToNext = "1.5";
        b.BlendToPrevious = "1.2";
        return new[] { a, b };
    }

    private static void WithCurrentCulture(CultureInfo culture, Action action)
    {
        var saved = (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture);
        try
        {
            Thread.CurrentThread.CurrentCulture = culture;
            Thread.CurrentThread.CurrentUICulture = culture;
            action();
        }
        finally
        {
            (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture) = saved;
        }
    }

    private static T WithCultureResult<T>(CultureInfo culture, Func<T> func)
    {
        T result = default!;
        WithCurrentCulture(culture, () => result = func());
        return result;
    }

    // ---- blend adjustment notice -----------------------------------------------------------

    [Fact]
    public void BlendWithoutLimits_ProducesNoNotice()
    {
        var r = new PrototypeTrajectoryService().Compute(Corner(null));

        Assert.Empty(r.Notices);
    }

    [Fact]
    public void BlendWhoseInheritedDurationIsFeasible_ProducesNoNotice()
    {
        // a_max = 30: the inherited blend (0.643 s) is feasible and kept.
        var r = new PrototypeTrajectoryService().Compute(Corner("30"));

        Assert.Empty(r.Notices);
        double blendDuration = r.CoefficientRows.Where(c => c.Segment == "Blend 1-2" && c.Coefficient == "T [s]").Single().Value;
        Assert.Equal(0.642911, blendDuration, 1e-5);
    }

    [Fact]
    public void BlendAdjustedByTheLimits_ProducesOneNoticeWithBothDurations_EnglishFormat()
    {
        WithCurrentCulture(English, () =>
        {
            // a_max = 15: the inherited blend (0.6429 s) violates it; the shortest feasible one is 0.7215 s.
            var r = new PrototypeTrajectoryService().Compute(Corner("15"));

            var notice = Assert.Single(r.Notices);
            Assert.Equal("Blend 1-2: duration adjusted from 0.6429 s to 0.7215 s to respect the limits.", notice);

            // The numbers of the notice are the ones of the report.
            double blendDuration = r.CoefficientRows.Where(c => c.Segment == "Blend 1-2" && c.Coefficient == "T [s]").Single().Value;
            Assert.Equal(0.7215, blendDuration, 1e-4);
        });
    }

    [Fact]
    public void TheNotice_UsesTheCurrentCulture_ItalianDecimalComma()
    {
        WithCurrentCulture(Italian, () =>
        {
            var r = new PrototypeTrajectoryService().Compute(Corner("15"));

            Assert.Equal("Blend 1-2: duration adjusted from 0,6429 s to 0,7215 s to respect the limits.", Assert.Single(r.Notices));
        });
    }

    // ---- one output culture ----------------------------------------------------------------

    /// <summary>Evaluates a binding with StringFormat=F6 on a TextBlock (needs an STA thread), like the XAML tables do.</summary>
    private static string FormatThroughABinding(double value)
    {
        string? result = null;
        Exception? failure = null;
        var thread = new Thread(() =>
        {
            try
            {
                var text = new TextBlock { DataContext = new { Value = value } };
                BindingOperations.SetBinding(text, TextBlock.TextProperty, new Binding("Value") { StringFormat = "F6" });
                result = text.Text;
            }
            catch (Exception ex)
            {
                failure = ex;
            }
        });
        thread.SetApartmentState(ApartmentState.STA);
        thread.Start();
        thread.Join();

        if (failure is not null)
            throw new InvalidOperationException("Binding evaluation failed.", failure);

        return result!;
    }

    [Fact]
    public void AfterUiCultureIsApplied_BindingsFormatWithTheCurrentCulture_NotWithEnUs()
    {
        // The override is process-wide and happens once (WPF allows it once): run it under Italian settings here.
        WithCurrentCulture(Italian, () => UiCulture.UseCurrentCultureForBindings());

        // The binding (StringFormat) and the code-built text now agree on the decimal separator.
        string viaBinding = FormatThroughABinding(2.460938);
        string viaCode = WithCultureResult(Italian, () => $"{2.460938:F6}");

        Assert.Equal("2,460938", viaCode);
        Assert.Equal(viaCode, viaBinding);
    }
}
