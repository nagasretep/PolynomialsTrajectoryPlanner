using System.Globalization;

namespace TrajectoryPlanner.App.Tests;

/// <summary>
/// Request 0010 (D20): number parsing of the user's text. The cultures are always set explicitly (and the
/// previous one restored), so the results do not depend on the machine's regional settings.
/// </summary>
public class InputNumberTests
{
    private static readonly CultureInfo Italian = new("it-IT");
    private static readonly CultureInfo English = new("en-US");

    private static (bool Ok, double Value) Parse(string text, CultureInfo culture)
    {
        bool ok = InputNumber.TryParse(text, culture, out double value);
        return (ok, value);
    }

    /// <summary>Runs the action with the thread's current culture set, then restores it.</summary>
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

    // ---- the matrix of the request ---------------------------------------------------------

    [Theory]
    [InlineData("1,5", true, 1.5)]      // Italian decimal comma
    [InlineData("1.5", true, 1.5)]      // dot: falls back to the invariant culture (it is the Italian thousands separator!)
    [InlineData("-2,25", true, -2.25)]
    [InlineData("0.001", true, 0.001)]
    [InlineData("1e3", true, 1000.0)]
    [InlineData("  7,5  ", true, 7.5)]
    [InlineData("1.234,5", false, 0)]   // thousands separator + decimal comma: rejected
    [InlineData("1,234.5", false, 0)]   // English-style thousands: rejected
    [InlineData("1.234", true, 1.234)]  // a dot with three digits is a decimal, never "one thousand two hundred thirty-four"
    [InlineData("1,234", true, 1.234)]
    [InlineData("abc", false, 0)]
    [InlineData("", false, 0)]
    [InlineData("   ", false, 0)]
    [InlineData("1,5,5", false, 0)]
    [InlineData("NaN", false, 0)]
    [InlineData("Infinity", false, 0)]
    [InlineData("1.5x", false, 0)]
    public void ItalianCulture(string text, bool ok, double expected)
    {
        var (parsed, value) = Parse(text, Italian);

        Assert.Equal(ok, parsed);
        if (ok) Assert.Equal(expected, value, 12);
    }

    [Theory]
    [InlineData("1.5", true, 1.5)]
    [InlineData("-2.25", true, -2.25)]
    [InlineData("1,5", false, 0)]       // comma is not a decimal separator in English and thousands are not accepted
    [InlineData("1,234.5", false, 0)]
    [InlineData("1.234,5", false, 0)]
    [InlineData("1,234", false, 0)]     // would be 1234 with thousands: rejected, not guessed
    [InlineData("1e-3", true, 0.001)]
    [InlineData("abc", false, 0)]
    [InlineData("", false, 0)]
    public void EnglishCulture(string text, bool ok, double expected)
    {
        var (parsed, value) = Parse(text, English);

        Assert.Equal(ok, parsed);
        if (ok) Assert.Equal(expected, value, 12);
    }

    [Theory]
    [InlineData("1.5", true, 1.5)]
    [InlineData("1,5", false, 0)]
    [InlineData("1,234.5", false, 0)]
    [InlineData("1.234,5", false, 0)]
    public void InvariantCulture(string text, bool ok, double expected)
    {
        var (parsed, value) = Parse(text, CultureInfo.InvariantCulture);

        Assert.Equal(ok, parsed);
        if (ok) Assert.Equal(expected, value, 12);
    }

    [Fact]
    public void TheOverloadWithoutCulture_UsesTheCurrentCulture()
    {
        WithCurrentCulture(Italian, () =>
        {
            Assert.True(InputNumber.TryParse("2,5", out double a));
            Assert.Equal(2.5, a, 12);
        });
        WithCurrentCulture(English, () =>
        {
            Assert.False(InputNumber.TryParse("2,5", out _));
            Assert.True(InputNumber.TryParse("2.5", out double b));
            Assert.Equal(2.5, b, 12);
        });
    }

    // ---- through the service ---------------------------------------------------------------

    private static SegmentInputModel Segment(string x0, string x1) => new(1)
    {
        Degree = 9,
        StartPosition = x0, EndPosition = x1,
        StartVelocity = "0", EndVelocity = "0",
        StartAcceleration = "0", EndAcceleration = "0",
        StartJerk = "0", EndJerk = "0",
        StartSnap = "0", EndSnap = "0",
        UseDuration = true, Duration = "2"
    };

    [Fact]
    public void Service_OnAnItalianPc_AcceptsDecimalCommaAndDotAlike()
    {
        WithCurrentCulture(Italian, () =>
        {
            var service = new PrototypeTrajectoryService();

            var withComma = service.Compute(new[] { Segment("0", "1,5") });
            var withDot = service.Compute(new[] { Segment("0", "1.5") });

            Assert.Equal(1.5, withComma.PositionAt(withComma.TotalDuration), 1e-9);
            Assert.Equal(1.5, withDot.PositionAt(withDot.TotalDuration), 1e-9);
        });
    }

    [Fact]
    public void Service_OnAnEnglishPc_AcceptsTheDot_AndRejectsTheComma()
    {
        WithCurrentCulture(English, () =>
        {
            var service = new PrototypeTrajectoryService();

            var ok = service.Compute(new[] { Segment("0", "1.5") });
            Assert.Equal(1.5, ok.PositionAt(ok.TotalDuration), 1e-9);

            var ex = Assert.Throws<InvalidOperationException>(() => service.Compute(new[] { Segment("0", "1,5") }));
            Assert.Contains("must be a valid number", ex.Message);
        });
    }

    [Theory]
    [InlineData("it-IT", "1.234,5")]
    [InlineData("it-IT", "1,234.5")]
    [InlineData("en-US", "1,234.5")]
    [InlineData("en-US", "1.234,5")]
    public void Service_RejectsThousandsSeparators_WithAMessageThatNamesTheField(string culture, string text)
    {
        WithCurrentCulture(new CultureInfo(culture), () =>
        {
            var ex = Assert.Throws<InvalidOperationException>(
                () => new PrototypeTrajectoryService().Compute(new[] { Segment("0", text) }));

            Assert.Contains("Segment 1 end position must be a valid number", ex.Message);
            Assert.Contains("no thousands separators", ex.Message);
            Assert.Contains(text, ex.Message);
        });
    }
}
