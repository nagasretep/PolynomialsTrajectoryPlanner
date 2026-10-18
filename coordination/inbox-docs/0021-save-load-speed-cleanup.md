# 0021 — Save and load, speed tests, unused public members (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0021-save-load-speed-cleanup.md`

## Summary

All three items are done. **A3**: the inputs of the four modes and the selected mode are saved to a `.ptp.json` file and loaded back, exactly as typed; the results computed from the loaded inputs equal the originals (compared to 1e-12 in the tests). **S1–S3**: the speed tests exist and pass with a very large margin (the slowest, S3, takes 165 ms against 1 s). **Q4**: the unused public members were removed (or made internal where only tests use them); 8 test methods (12 test cases) that only exercised a removed class were removed.

Tests: Core.Tests **445** (450 − 12 removed + 7 new speed tests), App.Tests **155** (136 + 18 project-file tests + 1 window test), **600 in total**, all passing. `dotnet build`: 0 errors, only the known NU1701.

## 1. Save and load (R14, criterion A3)

**Files** (all in `src/TrajectoryPlanner.App`): `ProjectFile.cs` (`ProjectInputs`, `ProjectSerializer`, `ProjectFileException`), `MainWindow.Project.cs` (Save/Open handlers: dialogs, title, status bar), two buttons in `MainWindow.xaml`.

**Format** (`format` = `PolynomialsTrajectoryPlanner.Project`, `version` = 1). Indented JSON with `mode` and one object per mode:

```json
{ "format": "PolynomialsTrajectoryPlanner.Project", "version": 1, "mode": "ThreeDPassThrough",
  "blending": { "segments": [ { "degree": 7, "startPosition": "0", "endPosition": "4.5", "duration": "1.5", "useMaxVelocity": true, ... } ] },
  "passThrough": { "degree": 7, "smoothnessOrder": 3, ..., "waypoints": [ { "position": "0", "duration": "" }, ... ] },
  "threeD": { "segments": [ { ..., "endPosition": { "x": "1", "y": "1", "z": "0.5" }, ... } ] },
  "threeDPassThrough": { ..., "waypoints": [ { "x": "0", "y": "0", "z": "0", "duration": "" }, ... ] } }
```

- **Text as typed.** Every box is saved as its text (`"7,5"` stays `"7,5"`, `"abc"` stays `"abc"`), flags as booleans, the degree and the choices (blend kind, stretch kind) as a number and a name. No number is formatted or parsed, so the culture never matters (tested with it-IT/en-US both ways).
- **Fields found by reflection.** The serializer saves every public settable text/flag/number/choice property of the input models (and the X | Y | Z vectors), except the derived ones (`SegmentNumber`, waypoint `Number` and `HasSegment`). Consequence: a new input added to a model is saved without touching the serializer, and cannot be forgotten. `Degree` is written first so that loading it (which resets the smoothness order to the degree's default) is followed by the saved order.
- **Tolerant reading.** Unknown fields are ignored; a missing field keeps the default of the model; a missing mode object loads as the default of that mode (the mode's default segment/waypoints); a missing `mode` selects Blending.
- **Refusals** (message always starts with the file name and gives the reason; nothing is touched): file missing; not JSON ("not a valid JSON file (…)"); JSON that is not an object; wrong `format`; `version` missing, not an integer or newer than the program ("version 2 is newer than this program supports (up to 1); update the program"); unknown `mode` or choice name; a field of the wrong type (names the field); a list of segments/waypoints that is empty (or fewer than two waypoints) or not a list.
- **Loading is two-phase.** The file is read completely into new models (`ProjectSerializer.Load`); only if that succeeds are the live models of the window replaced (`ApplyTo`). A refusal therefore cannot leave the inputs half changed. Loading does **not** compute; the stored results of the four modes are dropped (they belong to the old inputs) and the status bar says "Opened x; click Compute to see the result."
- **Window.** Buttons `Open` and `Save` after Compute, standard Windows dialogs, filter `*.ptp.json` / `*.json`; the window title becomes "… - name.ptp.json"; pending edits in the waypoint tables are committed before saving; errors go to the status bar and a message box. For the smoke test the message box can be replaced (`ProjectErrorHandler`).

**Tests** (`App.Tests/ProjectFileTests.cs`, 18, and one in `MainWindowSmokeTests`):

| Criterion | Test |
|---|---|
| Per mode: non-default input saved, loaded into a fresh model, compared field by field; results equal to 1e-12 | `BlendingMode_…`, `PassThroughMode_…`, `ThreeDMode_…`, `ThreeDPassThroughMode_RoundTrips…` (degree 7, a saved smoothness order of 3, non-zero end velocity vector, limits; coefficients, extrema, durations, path length) |
| Every field of every model, also non-numeric text | `EveryInputOfEveryModel_Survives_EvenWhenItIsNotANumber` (all properties changed by reflection to "text n, é", flags flipped, choices changed; the file text of the loaded project is identical) |
| Culture it-IT → en-US and en-US → it-IT | `TheCultureDoesNotMatter_…` (2 cases; results compared too); `ADecimalCommaTypedByTheAuthor_IsKeptAsTyped` |
| Unknown field; missing mode object; missing field | `AnUnknownField_IsIgnored`, `AMissingModeObject_…`, `AMissingField_…` |
| Version 2; not JSON; wrong format/version/mode/type/list | `ANewerVersion_IsRefused_…`, `AFileThatIsNotJson_…`, `WrongFormat…_AreRefused`, `AMissingFile_…` (messages checked for the file name and the reason) |
| Through a real file; applying to the live models | `SavingAndLoadingThroughAFile_Works`, `ApplyingALoadedProject_ReplacesTheLiveModels` |
| In the window: save, change, open → inputs and mode return; a refused file leaves the inputs unchanged and reports the reason | `SaveAndOpen_RestoreTheInputsOfAllModes_AndARefusedFileChangesNothing` |

### Hand checks for the author

1. Change some inputs in each mode, pick a mode (e.g. 3D pass-through), press **Save**, give a name; the title shows the name.
2. Change several inputs and switch mode; press **Open**, choose the file: all inputs return in all four modes, the saved mode is selected, plots and tables are empty until Compute (status bar says so).
3. Open the file in a text editor, change `"version": 1` to `2` (or break the JSON, or write `"degree": "seven"`), Open it: a message names the file and the reason; the inputs on screen do not change.
4. Type a decimal comma and a word in a box, Save, Open: the boxes show the same text.

## 2. Speed tests (R15, S1–S3)

`src/TrajectoryPlanner.Core.Tests/Speed/SpeedTests.cs`, `[Trait("Category", "Speed")]`, run alone with `dotnet test --filter Category=Speed`. Method: one warm-up run, then the **median of 5** `Stopwatch` runs; the test fails if the median exceeds the criterion; each test writes its median and the five runs to the test output (`--logger "console;verbosity=detailed"` shows them). The tests also check that the work is real (the S1 search finds a feasible, non-default duration; the S2 plans have 29 segments and λ applied; S3 has three parts and a path longer than the chord).

**Machine**: AMD Ryzen 9 8940HX, 31 GB, Windows 11, .NET 8, test run in Debug (Release in brackets).

| Criterion | What is timed | Median (ms) | Limit |
|---|---|---|---|
| S1 degree 3 | one segment, limits active, duration by the shortest-feasible search | 0.37 (0.33) | 100 |
| S1 degree 5 | same | 1.30 (1.68) | 100 |
| S1 degree 7 | same | 2.59 (2.94) | 100 |
| S1 degree 9 | same | 4.37 (4.74) | 100 |
| S2 1D | pass-through, 30 waypoints, degree 9, default order, limits (λ applied) | 7.2 (6.6) | 1000 |
| S2 3D | same in 3D | 25.2 (23.0) | 1000 |
| S3 | parabola of the theory as a degree-9 segment + curved constant-speed stretch (entry, stretch, exit) with limits, built, then `Extrema`, `LimitReport`, `PathLength`, 400 samples | 165.8 (193.4) | 1000 |

All criteria pass with a margin of at least 5×; nothing to fix. S3 dominates (the curved stretch is evaluated by series arithmetic and Newton inversion, and the analysis and sampling evaluate it many times); at 6× under the limit it needs no optimisation now. Debug and Release are alike because the work is in numerical loops over small arrays, not in code the optimiser changes much; Release is not faster for S1/S3 (differences are within noise).

Note on the data: the S1 limits are 4 / 20 / 200 / 2000 (as the degree allows) with a boundary velocity at the end so that the search is not just the closed form; the S2 limits are 5 / 20 / 200 / 2000; the S3 limits are 5 / 50 / 500 / 5000. The criteria do not fix the data, so these are my choice; they can be changed in the test.

## 3. Unused public members (Q4)

Method: every `public` member and type of `Core` and `App` was looked up by name in the production code (Core + App sources, XAML bindings counted as use), tests not counted. The ones with no use were then decided one by one.

**Removed** (no production use; no test, or only tests of the removed thing):

| Member | Reason |
|---|---|
| `TrajectoryBuilder1D.AddSegmentSimple` | duplicate of `AddSegment`; no test |
| `PolynomialSegment1D.CreateDegree7`, `CreateDegree9` | unused, no test |
| `PolynomialCoefficientSolver.SolveDegree9` | unused, no test |
| `Point3D.Cross`, `Point3D.Normalize` | unused, no test |
| `Trajectory1D.EvaluatePoint` | unused, no test |
| `TrajectoryConstants.MaxPolynomialDegree`, `MinPolynomialDegree` | unused anywhere (`SupportedDegrees` is the one in use) |
| `BoundaryConditionValidator.ValidateConditions1D`, `ValidateConditions3D`, `RequiredDerivativeOrders` | unused, no test |
| types `BoundaryCondition1D`, `BoundaryCondition3D`, enum `BoundaryEndpoint` (files `BoundaryCondition1D.cs`, `BoundaryCondition3D.cs`) | only used by the removed validators |
| class `ConstraintAwareDuration` (`ComputeDuration`) | a thin wrapper over `ShortestFeasibleDuration.Find` plus a fallback, which the builders and `SegmentFactory1D` do directly; used only by its own tests |

**Removed tests** (all of them only exercised `ConstraintAwareDuration`; the search itself keeps its own tests in `ShortestFeasibleDurationTests`, the fallback is covered through `SegmentFactory1D`): in `ExtremaAndDurationTests.cs`: `ConstraintAware_PerSegmentResult_SatisfiesTheLimits` (4 cases), `ConstraintAware_ImpossibleLimits_ReportFailure`, and the class `ConstraintAwareFallbackTests` (`ZeroLengthRestToRest_UsesTheFallbackDuration`, `NoActiveLimit_UsesTheFallbackDuration_AndDefaultsToOneSecond`, `DeterminedCase_IgnoresTheFallback`, `NonPositiveFallback_Rejected` ×2); in `ShortestFeasibleDurationTests.cs`: `ConstraintAwareDuration_ReturnsTheShortestFeasibleDuration`; in `CleanupBatchTests.cs`: `ConstraintAwareDuration_PropagatesTheFailure`. Together **8 test methods, 12 test cases**.

**Made `internal`** (only the tests use them; `InternalsVisibleTo` for `TrajectoryPlanner.Core.Tests` added to `TrajectoryPlanner.Core.csproj`, the one new project-file line): `PolynomialCoefficientSolver.SolveDegree3/5/7`, `SolveFromBoundaryValues` (the tests of the polynomial evaluation use them as the closed-form oracle), `PolynomialSegment1D.CreateDegree5` (test fixture), `Trajectory1D.SampleTimes`, `PassThroughProblem1D.Cost` and `Gradient` (optimality tests), `BoundaryConditionValidator.RequiredConditionCount`. No test changed.

**Kept, with the reason**:
- `SmoothnessCriterion.MinimumSnap`: the named criterion of the project's namesake method (default of degree 9); used by a test.
- `TrajectoryBuilder1D` (whole class), `WithDurationStrategy`, `IDurationStrategy1D`, `UserSpecifiedDuration`: the 1D counterpart of `TrajectoryBuilder3D` and the documented extension point of Decision 1 of `design/07`; the App calls the services directly, so no production caller exists today, but the class has its own tests and a future caller (R16 manual, scripting) needs it.
- enum `DerivativeOrder` (`Symbols`): the code form of `theory/SymbolsAndNomenclature.md`; tested.
- `BlendKinds`, `StretchKinds` (App): used by XAML bindings (a search of the `.cs` files alone does not see them).

**No removal needed in `App`**: the other public members of the App are bound in XAML or called by `MainWindow` or by the tests.

## For the author (educational)

- **Why the inputs are saved as text and the results are recomputed.** The file records what *you* decided (the boxes), not what the program derived from it. A saved result could disagree with a later, better algorithm (a shorter duration, a corrected peak), and a file that carried both would contradict itself; recomputing guarantees that the plots always come from the current code. The file also stays small and readable, so it can be edited by hand or kept in git. Text rather than numbers keeps a half-typed or invalid box exactly as it was, and avoids every locale problem ("1,5" and "1.5" are never reinterpreted by the file).
- **What `version` is for.** It is the contract of the file layout. If a later release changes the layout (a new required field, a renamed one), it writes version 2 and can still read version 1 files (by upgrading them); an old program that meets version 2 refuses it with a clear message instead of silently reading something wrong. Adding a new optional field does *not* need a new version: old files lack it and get the default, old programs ignore it.
- **How to read the timing numbers.** One run of a short computation is dominated by noise (the operating system, the processor's clock, the garbage collector), so the test takes the **median** of five runs: the middle value ignores one lucky and one unlucky run, unlike the mean. The **warm-up** run is thrown away because the first call pays for the JIT compiler, which turns the program's intermediate code into machine code the first time each method runs, and fills caches; the user would only meet that cost once, at the first Compute, but the criterion is about the steady state. If the five runs of a test differ widely, the machine was busy and the number should not be trusted. S1–S3 are far below their limits, which is the point of a timing criterion: it is a guard against a future change that makes the program 10× slower, not a target to squeeze.

## Discrepancies between theory and code

None.

## Open questions

1. The speed criteria do not fix the data (waypoints, limits, boundary states). I chose the ones in the table above; if the criteria should refer to specific data, say which.
2. `TrajectoryBuilder1D` has no production caller (the App uses the services). Decide whether it stays the library's 1D entry point (kept now) or goes in a later cleanup together with `IDurationStrategy1D`.
3. For the user manual (R16): the file format is documented here only; if `spec/` or the manual should carry it, the example above can be the source.

## Suggested commits

1. `code: App — save and open projects (.ptp.json) for the four modes` — `src/TrajectoryPlanner.App/ProjectFile.cs`, `src/TrajectoryPlanner.App/MainWindow.Project.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml`
2. `code: App.Tests — project file tests and window save/open test` — `src/TrajectoryPlanner.App.Tests/ProjectFileTests.cs`, `src/TrajectoryPlanner.App.Tests/MainWindowSmokeTests.cs`
3. `code: Core.Tests — speed tests S1-S3 (Category=Speed)` — `src/TrajectoryPlanner.Core.Tests/Speed/SpeedTests.cs`
4. `code: Core — remove unused public members, internal for test-only ones` — `src/TrajectoryPlanner.Core/` (`BoundaryConditions/`, `CoefficientDetermination/PolynomialCoefficientSolver.cs`, `PassThrough/PassThroughProblem1D.cs`, `Points/Point3D.cs`, `Segments/PolynomialSegment1D.cs`, `Services/IDurationStrategy.cs`, `Services/TrajectoryBuilder1D.cs`, `Symbols/Constants.cs`, `Trajectories/Trajectory1D.cs`, `TrajectoryPlanner.Core.csproj`) and the edited tests `src/TrajectoryPlanner.Core.Tests/Constraints/ExtremaAndDurationTests.cs`, `…/Constraints/ShortestFeasibleDurationTests.cs`, `…/Segments/CleanupBatchTests.cs`
5. `coord: close request 0021` — `coordination/inbox-code/0021-save-load-speed-cleanup.md`, `coordination/inbox-docs/0021-save-load-speed-cleanup.md`, `coordination/STATUS.md`

(`roadmap/`, `theory/`, `design/`, `spec/` untouched. Deleted files: `src/TrajectoryPlanner.Core/BoundaryConditions/BoundaryCondition1D.cs` and `BoundaryCondition3D.cs`, to be staged with the Core commit.)
