using System.Globalization;

namespace TrajectoryPlanner.App;

/// <summary>
/// Parses numbers typed by the user. The same text means different numbers in different cultures ("1.5" is
/// one and a half in English but fifteen where the dot is a thousands separator, as in Italian), so the rules
/// are fixed here: no thousands separators are accepted at all, and the decimal separator may be the one of
/// the current culture or the invariant dot. Examples: with Italian settings "1,5" and "1.5" both give 1.5 while
/// "1.234,5" and "1,234.5" are rejected; with English settings "1.5" works and "1,5" is rejected.
/// </summary>
public static class InputNumber
{
    /// <summary>Sign, decimal point and exponent only: <c>NumberStyles.Float</c> has no <c>AllowThousands</c>.</summary>
    private const NumberStyles Styles = NumberStyles.Float;

    public static bool TryParse(string? text, out double value) =>
        TryParse(text, CultureInfo.CurrentCulture, out value);

    /// <summary>Same with an explicit culture (tests, or a UI with its own culture).</summary>
    public static bool TryParse(string? text, CultureInfo culture, out double value)
    {
        value = 0;
        if (string.IsNullOrWhiteSpace(text))
            return false;

        if (double.TryParse(text, Styles, culture, out double parsed) ||
            double.TryParse(text, Styles, CultureInfo.InvariantCulture, out parsed))
        {
            // "NaN" and "Infinity" are accepted by the framework as text; they are not usable inputs.
            if (!double.IsFinite(parsed))
                return false;

            value = parsed;
            return true;
        }

        return false;
    }
}
