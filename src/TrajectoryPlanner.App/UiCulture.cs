using System.Globalization;
using System.Windows;
using System.Windows.Markup;

namespace TrajectoryPlanner.App;

/// <summary>
/// WPF formats the numbers of a data binding (<c>StringFormat=F6</c>) with the culture of the target element's
/// <c>Language</c> property, not with the Windows setting. That property defaults to en-US, so tables showed a
/// decimal point while text built in code (<c>$"{x:F4}"</c>, which uses the current culture) showed the
/// regional separator. Setting the default Language to the current culture makes both agree.
/// </summary>
public static class UiCulture
{
    private static readonly object Gate = new();
    private static bool _applied;

    /// <summary>
    /// Makes every <see cref="FrameworkElement"/> created from now on default to the current culture. Must run
    /// before the first window is created; WPF allows the override once per process, so repeated calls do nothing.
    /// </summary>
    public static void UseCurrentCultureForBindings()
    {
        lock (Gate)
        {
            if (_applied)
                return;

            FrameworkElement.LanguageProperty.OverrideMetadata(
                typeof(FrameworkElement),
                new FrameworkPropertyMetadata(XmlLanguage.GetLanguage(CultureInfo.CurrentCulture.IetfLanguageTag)));
            _applied = true;
        }
    }
}
