# 0021 — Save and load projects, speed tests, unused public members

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

## Goal

Three work items of `roadmap/ROADMAP.md`, all needed for version 1.0 (`spec/05-success-criteria.md`): **R14** the author can save the inputs of all four modes to a file and load them again (criterion A3); **R15** timing tests that prove the speed criteria S1-S3; **Q4** the public members left unused after the consolidation are removed.

## Reference documents

- `spec/05-success-criteria.md` (A3, S1, S2, S3, Q4, A2)
- `design/08-ui-first-prototype.md` (the four modes and their inputs)
- Notes `0009` (consolidation, unused public members kept "until R9"), `0013`, `0018`, `0020` (the input models of the four modes)

## Scope

### 1. Save and load (R14, criterion A3)

1. **What is saved**: the **inputs** of the four modes (the input models of the App: degree, smoothness order, end conditions, tables of waypoints and segments, durations, limits, blend and constant-speed settings, whatever each mode holds) and which mode is selected. Results are never saved: they are recomputed after loading.
2. **Inputs are saved as the author typed them** (the text of each box, not a parsed number), so that loading reproduces exactly the same boxes, and a field that did not parse is saved and restored too. Saving must not depend on the current culture.
3. **Format**: one human-readable JSON file (`System.Text.Json`, indented), extension `.ptp.json`, with a `format` name, an integer `version` (start at 1) and one object per mode. Numbers that are stored as numbers (if any) use the round-trip form and the invariant culture. Unknown fields are ignored; a missing field takes the default of the mode.
4. **Code location**: the file model and the serialization belong to the App layer, as a service testable without a window (like the other services); no logic in code-behind except calling it. Suggested names: `ProjectFile` (model), `ProjectSerializer` (`Save(path)`, `Load(path)` or stream versions for the tests).
5. **User interface**: a `Save` and an `Open` button (or File menu, if it is simpler) with the standard Windows dialogs; the window title shows the file name; loading replaces the inputs of all four modes and selects the saved mode; it does **not** compute by itself. Errors (file missing, not JSON, wrong `format`, `version` newer than supported, unreadable content) are shown in the status bar or a message box with the file name and the reason; the current inputs are left untouched when loading fails.
6. **Tests** (App.Tests, service level): for each of the four modes, a non-default input is saved, loaded into a fresh model and compared field by field (identical text); then the **result** computed from the loaded inputs equals the result of the original inputs to $10^{-12}$ (coefficients, durations, peaks); the same with the current culture set to `it-IT` while saving and to `en-US` while loading and the other way round; a field with non-numeric text round-trips unchanged; a file with an extra unknown field loads; a file with a missing mode object loads with the default of that mode; a file with `version` 2 and a file that is not JSON are refused with a message that names the reason, leaving the target model unchanged.

### 2. Speed tests (R15, criteria S1-S3)

Timing tests in the test projects, written so that they can be run on the author's laptop and report the measured times:

- **S1**: compute of one 1D segment, each degree 3, 5, 7, 9, **with** limits active (the shortest-feasible-duration search included) $\le 100$ ms.
- **S2**: pass-through with 30 waypoints, 1D and 3D, **with** limits active (degree 9, default order) $\le 1$ s.
- **S3**: a 3D trajectory with a curved constant-speed stretch, **including** the analysis and the report (extrema, limit report, path length), the reference example of the theory (*Curved 3D Constant-Speed Stretch*) $\le 1$ s.

Method: one warm-up run, then the **median of 5** timed runs with `Stopwatch`; the test fails if the median exceeds the criterion. Put the tests in their own class (`SpeedTests`) with a `Trait("Category", "Speed")` so that they can be run alone (`dotnet test --filter Category=Speed`). Each test writes the measured median to the test output. If a criterion fails on your machine, **do not relax it**: report the measured times in the note and the profile (what dominates), and propose the fix; fix it only if it is small and local.

### 3. Unused public members (Q4)

List the `public` members of the Core (and App) that no production code references (references from tests alone do not count). For each: if it is part of the intended API of the library (a documented entry point, a type a future caller needs) keep it and say why; otherwise remove it, or make it `internal` if the tests need it (`InternalsVisibleTo` already exists or add it for the test projects). Remove tests that only exercised removed members; **the note lists every removed member and every removed test**. Do not remove behaviour: after the cleanup all other tests pass unchanged.

## Out of scope

New modes or features, a project-wide settings dialog, recent-files list, auto-save, saving the results or the plots, changes to `roadmap/`, `theory/`, `design/` (a missing design statement is reported in the note, not written).

## Acceptance criteria

- [ ] A3: for each mode, save then load reproduces the inputs exactly and the results to $10^{-12}$; the culture does not matter; invalid files are refused with a clear message and leave the inputs unchanged.
- [ ] Save and Open work in the window (hand check listed in the note: save, change something, open, the inputs return; open a file edited by hand with a wrong value and see the error).
- [ ] S1, S2, S3 tests exist, pass, and the measured medians are in the note (with the machine used).
- [ ] Q4: the list of unused public members, with the decision for each, is in the note; they are removed or made internal.
- [ ] `dotnet build` free of new warnings; all tests pass; the note states the test count (it may drop because of removed tests: give the number removed).

## Notes for the author (educational)

Explain in the return note: why the inputs are saved as text and the results are recomputed (a saved result could disagree with a later version of the algorithm; the file stays small and readable), what the `version` field is for, and how to read the timing numbers (median, warm-up, why the first run is slower: JIT compilation).

Return note: `coordination/inbox-docs/0021-save-load-speed-cleanup.md`, ending with a "Suggested commits" section. Do not touch `roadmap/`, `theory/` or `design/`; targeted edits only in `coordination/STATUS.md`.
