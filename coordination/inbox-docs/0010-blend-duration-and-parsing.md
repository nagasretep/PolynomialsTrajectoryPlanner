# 0010 — Blend duration rule and number parsing (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0010-blend-duration-and-parsing.md`

## Summary

Both changes done. Tests: Core.Tests 291 → **296**, App.Tests 26 → **63** (359 total, was 317), all passing. `dotnet build`: 0 errors; warnings only the known NU1701.

## 1. Blend duration (D19)

`BlendBuilder1D`, no prescribed duration and limits active: the inherited duration (sum of the removed portions, floor 0.01 s) is built first; **if that blend passes `VerifyLimits` it is kept**, otherwise `ShortestFeasibleDuration.Find ?? inherited` is used; the final `VerifyLimits` and the errors are as before (explicit error starting "The blend:" if no duration works; prescribed duration only verified; no limits: unchanged). Implemented in the private `InheritedIfFeasibleElseShortest` (renamed from `ShortestOrInherited`); the class remarks state the rule.

Effect on the 0009 example (degree 9 corner, neighbours with v_max = 8): the blend is again **0.642911 s**, total **2.5 s**, peak velocity 3.58 (in 0009 it was 0.100120 s, total 1.957, peak 8.000). All other numbers of the 0009 before/after table are unchanged (checked with the same console harness: only that line differs).

Tests rewritten because the rule changed (0009 tests that expected the shortest feasible blend when the inherited one was feasible):
- `BlendBuilderLimitsAndDegreeTests.WithAnActiveLimit_TheBlendTakesItsShortestFeasibleDuration_AndPassesVerifyLimits` (4 cases v/a/j/s) → `WithAnActiveLimit_AnInheritedDurationThatIsFeasibleIsKept_AndPassesVerifyLimits`: the inherited 0.75 s is kept, passes the limits, and the shortest feasible would have been shorter.
- `PrototypeTrajectoryServiceTests.Blend_WithALimit_TakesItsShortestFeasibleDuration_AndStaysWithinTheLimit` (App.Tests) → `Blend_WithALooseLimit_KeepsTheInheritedDuration_AndTheTotal`.
New: `WithALimitThatTheInheritedBlendViolates_TheShortestFeasibleDurationIsUsed` (v, a, j, s: limit 10 % below the inherited peak; precondition asserted: inherited infeasible; result = `ShortestFeasibleDuration`, F ≈ 0, passes `VerifyLimits`), `KeepingTheInheritedDuration_LeavesTheTotalTrajectoryDurationUnchanged`. Unchanged and still passing: infeasible limits → explicit error naming the blend (`InfeasibleLimits_GiveAnExplicitErrorNamingTheBlend`), prescribed duration kept/violating, limits with nothing active.

**An observation worth knowing**: when the inherited blend is infeasible, the shortest feasible duration is **not always longer** than the inherited one. For a velocity limit on a long blend (window 0.3/0.7 of the test corner) the inherited blend overshoots its boundary speeds, and a *shorter* blend overshoots less; so "infeasible → replaced by the shortest feasible" can shorten the blend (and the total trajectory). The test therefore asserts "not the inherited duration" rather than "longer". It is the direct consequence of the rule as decided (and of the non-monotone feasibility seen in 0008), not a defect.

## 2. Number parsing (D20)

New `App/InputNumber.cs` (public static, documented): `TryParse(text)` / `TryParse(text, culture)` with `NumberStyles.Float` (sign, decimal point, exponent, surrounding spaces; **no `AllowThousands`**), first with the given/current culture, then with the invariant culture; empty/whitespace and the non-finite texts "NaN"/"Infinity" (which the framework accepts as text) are rejected. Used by `PrototypeTrajectoryService.ParseRequiredDouble` (and so `ParsePositiveDouble`) and by `MainWindow` (the only other `double.TryParse` of user text: the default end position of a new segment). The error message is now e.g. `Segment 1 end position must be a valid number (decimal point or comma, no thousands separators): "1.234,5".`

Results (all tested with explicit cultures; the machine's culture is never relied on):

| text | it-IT | en-US | invariant |
|---|---|---|---|
| `1,5` | 1.5 | rejected | rejected |
| `1.5` | 1.5 (via invariant) | 1.5 | 1.5 |
| `1.234,5` | rejected | rejected | rejected |
| `1,234.5` | rejected | rejected | rejected |
| `1.234` | **1.234** | 1.234 | 1.234 |
| `1,234` | 1.234 | rejected | rejected |
| `1e3`, ` 7,5 ` | 1000, 7.5 | 1000 (`1e3`) | – |
| `NaN`, `Infinity`, `abc`, empty | rejected | rejected | rejected |

Note the change for Italian users: the old default parse (`double.TryParse(text)`, thousands allowed) read `1.234` as **1234** under it-IT; it now reads **1.234**, and `1.234,5` is refused instead of being read as 1234.5. That is the point of the decision (no silent reinterpretation), but it is a behaviour change worth knowing.

Tests in `App.Tests/InputNumberTests.cs` (37 cases): the matrix per culture, the overload without culture (current culture set explicitly and restored in `finally`), and through the service (it-IT accepts both `1,5` and `1.5`; en-US accepts `1.5`, rejects `1,5`; the four thousands-separator texts give a message naming the field and quoting the text). `CultureInfo` is per thread, so the tests do not interfere when xUnit runs classes in parallel.

**Outputs use a different culture than inputs (reported, not changed)**: the report tables and cursor table in `MainWindow.xaml` use `StringFormat=F6/F8` in bindings, and WPF bindings format with the element's `Language` (default **en-US**), so they always show a decimal **dot**; the status bar and cursor label (`$"{...:F4}"`, `:F6`) and the default end position (`ToString("F0")`) use the **current culture** (decimal comma on an Italian PC). So on an Italian PC the tables show `2.460938` while the status bar shows `2,4609`. Inputs now accept both, so a value copied from a table into an input field works in either culture. If you want one rule for outputs, the simplest is to set `Language`/`ConverterCulture` to the current culture in XAML (or format the status texts with the invariant culture); not done.

## For the author (educational)

**Why the culture of `double.TryParse` matters.** A string of characters has no number in it until a culture says how to read it. With English settings `.` is the decimal point and `,` groups thousands; with Italian settings it is the other way round. The old call allowed thousands separators, so under it-IT the text `1.5` could be read as fifteen (the dot is a thousands separator that "groups" a one and a five), and the same text on an English PC means one and a half: same input file, different trajectories on two machines, with no error. Rejecting thousands separators removes the ambiguity because then each separator has one possible meaning: it is either the decimal mark or an error. Accepting both `,` and `.` as decimal mark (current culture first, invariant second) is safe once thousands are impossible, and it avoids annoying a user who types `1.5` on an Italian keyboard layout; texts that mix both (`1.234,5`) are refused rather than guessed.

## Discrepancies between theory and code

None. `BlendingSegments.md` *Decisions For The 1D Baseline* is implemented as written.

## Open questions

1. Output culture (see above): leave the mixed behaviour, or unify?
2. Should a blend whose inherited duration is infeasible but whose shortest feasible duration is *shorter* be allowed to change the total trajectory duration silently (it does, by design of the rule)? The Total column of the report shows it.

## Suggested commits

1. `code: blend keeps the inherited duration when feasible, shortest feasible otherwise` — `src/TrajectoryPlanner.Core/Blending/BlendBuilder1D.cs`, `src/TrajectoryPlanner.Core.Tests/Blending/BlendBuilderLimitsAndDegreeTests.cs`
2. `code: culture-safe parsing of user numbers` — `src/TrajectoryPlanner.App/InputNumber.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`, `src/TrajectoryPlanner.App.Tests/InputNumberTests.cs`, `src/TrajectoryPlanner.App.Tests/PrototypeTrajectoryServiceTests.cs` (this file also holds the blend test rewrite of part 1)
3. `coord: close request 0010` — `coordination/inbox-code/0010-blend-duration-and-parsing.md`, `coordination/inbox-docs/0010-blend-duration-and-parsing.md`, `coordination/STATUS.md`
