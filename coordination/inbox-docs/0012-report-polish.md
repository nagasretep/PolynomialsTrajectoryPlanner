# 0012 — Report polish (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0012-report-polish.md`

## Summary

Both points done. Tests: Core.Tests 331 → **336**, App.Tests 63 → **68** (404 total, was 394), all passing. `dotnet build`: 0 errors; warnings only the known NU1701. No change to any computed duration.

## 1. Blend adjustment notice (D21)

- **Core**: `BlendBuilder1D` now returns a `BlendSegment1D` (new small class deriving from `PolynomialSegment1D`, so every existing use keeps compiling) with `InheritedDuration` (sum of the removed portions, floored at 0.01 s, always computed), `Duration` (the chosen one, as before) and `AdjustedByLimits` (true only when limits are active, no duration was prescribed, and the chosen duration differs from the inherited one, i.e. the inherited blend violated the limits). The pair methods return the blend as `BlendSegment1D` in their tuple. A prescribed duration is never "adjusted".
- **Service**: `PrototypeComputationResult` has a new last member `Notices` (list of strings). For an adjusted blend it holds `Blend 1-2: duration adjusted from 0.6429 s to 0.7215 s to respect the limits.` (`F4`, the same format as the total duration in the status bar, in the current culture); nothing is added when the blend was not adjusted or has no limits.
- **UI**: `MainWindow` appends the notices to the status-bar text after "Computed … seconds." (one line, notices separated by a space). The duration itself is in the Coefficients table (`T [s]` row of the blend), as before; I did not add a second report line.
- Example (App service, degree 9 corner, both neighbours with `a_max`): 30 → inherited blend 0.6429 s feasible, kept, **no notice**; 15 → inherited infeasible, adjusted to 0.7215 s, **notice**; 12 → the neighbours themselves violate the limit at their prescribed durations (error as before, naming "Segment 1").

## 2. One output culture (D22)

- New `App/UiCulture.cs` (public static) with `UseCurrentCultureForBindings()`: `FrameworkElement.LanguageProperty.OverrideMetadata(typeof(FrameworkElement), new FrameworkPropertyMetadata(XmlLanguage.GetLanguage(CultureInfo.CurrentCulture.IetfLanguageTag)))`, guarded to run once (WPF allows the override once per process). `App.OnStartup` calls it before `base.OnStartup`, i.e. before the first window exists. (`App.xaml.cs` also lost its unused `using System.Configuration; using System.Data;` and its BOM; line endings kept CRLF.)
- Checked: all 59 bindings in `MainWindow.xaml` are plain `{Binding … StringFormat=…}` and none sets `ConverterCulture` or `Language`, so they follow the default Language. Everything built in code already used the current culture (`$"{…:F4}"`, `ToString("F0")`).
- **Measured, without the window** (throw-away console under `it-IT`): before the override the same number 2.460938 formatted through a `StringFormat=F6` binding gives `2.460938` and through code `2,460938`; **after** the override both give `2,460938`. The unit test `AfterUiCultureIsApplied_BindingsFormatWithTheCurrentCulture_NotWithEnUs` repeats that on an STA thread (a `TextBlock` with the same kind of binding) and asserts that binding and code text are equal and use the comma.
- **Not covered, to be looked at when running the App** (the author, on an Italian setup): (a) the report tables (Extrema, Coefficients) and the cursor table show the decimal **comma**, like the status bar and the cursor text; (b) the **axis tick labels of the ScottPlot charts**: I did not check which culture ScottPlot uses for them and did not touch it; if they still show a dot, that is the plot library's setting (a larger change, reported rather than done); (c) the editable input cells keep showing the text exactly as typed.

## Tests

- `Core.Tests/Blending/BlendDurationInformationTests.cs` (5): no limits (0.75 s inherited, not adjusted); feasible inherited kept (not adjusted); violated inherited (adjusted, both durations exposed); prescribed never adjusted; pair methods.
- `App.Tests/ReportPolishTests.cs` (5): no limits → no notice; `a_max = 30` → no notice and the blend keeps 0.642911 s; `a_max = 15` → exactly one notice with the right values (en-US); the same under `it-IT` with decimal commas; the culture/binding test above. All cultures are set explicitly on the thread and restored in `finally`.

## For the author (educational)

WPF does not format the numbers of a data binding with the Windows regional setting. When a binding has a `StringFormat` (`F6`) and no `ConverterCulture`, WPF uses the culture named by the **`Language`** property of the element that shows the value (inherited down the visual tree), and that property **defaults to `en-US`** on every WPF element, whatever the machine is set to. Text built in code (`$"{x:F4}"`, `ToString("F0")`) goes through .NET, which uses the thread's current culture, i.e. the Windows setting. So on an Italian PC the tables showed `2.460938` (binding → en-US) while the status bar showed `2,4609` (code → it-IT): two parts of the same window formatting with two cultures. The fix is to change the default of `Language` once, at start-up, to the current culture, which is what `UiCulture` does; the bindings themselves are not touched.

## Discrepancies between theory and code

None. (`BlendingSegments.md`, bullet "Visibility": implemented as written.)

## Open questions

1. The ScottPlot tick labels (see above): to be seen when running the App; if they differ, do you want them aligned (a plot-library setting) or left alone?
2. Should the notice also be shown in the report table (a row in the Coefficients tab), or is the status bar enough? The status bar line is replaced by the next action ("Added segment…"), while the table row is permanent.

## Suggested commits

1. `code: blend segment reports its inherited duration and whether the limits adjusted it` — `src/TrajectoryPlanner.Core/Blending/BlendBuilder1D.cs`, `src/TrajectoryPlanner.Core/Blending/BlendSegment1D.cs`, `src/TrajectoryPlanner.Core.Tests/Blending/BlendDurationInformationTests.cs`
2. `code: adjustment notice in the status bar; one output culture (current culture) for bindings` — `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs`, `src/TrajectoryPlanner.App/PrototypeModels.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`, `src/TrajectoryPlanner.App/UiCulture.cs`, `src/TrajectoryPlanner.App/App.xaml.cs`, `src/TrajectoryPlanner.App.Tests/ReportPolishTests.cs`
3. `coord: close request 0012` — `coordination/inbox-code/0012-report-polish.md`, `coordination/inbox-docs/0012-report-polish.md`, `coordination/STATUS.md`
