# 0001 — Baseline review of the code

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Give Cowork an accurate, read-only picture of what the code actually does today, so that the theory library (`theory/`) can be checked against it and the next requests (exact minimum-snap solver, 3D extensions) can be written precisely. This request produces a **report**, not features.

## Context

- Build: `dotnet build PolynomialsTrajectoryPlanner.slnx` succeeds (verified by the author on 2026-10-17). Tests: `dotnet test src\TrajectoryPlanner.Core.Tests` → 24 total, 24 passed.
- The repository was just restructured (ADR-0001); read `CLAUDE.md` first.
- `LearningCsWPF/` is out of scope: do not read or touch it.

## Reference documents

- `design/07-implementation-plan.md` (decisions, milestone sequence, appendix A1–A5)
- `design/08-ui-first-prototype.md` (UI scope and the note on the provisional pass-through solver)
- `roadmap/06-theory-library-roadmap.md` (theory ↔ requirement map)

## Tasks

1. **Warning NU1701.** The build warns that `SkiaSharp.Views.WPF 3.119.0` is restored for .NET Framework instead of `net8.0-windows`. Find where it comes from (the App project references only `ScottPlot.WPF 5.1.59`, so probably transitive) and say whether it is harmless or needs action. Change code or package versions **only** if the fix is trivial and the app behaves the same; otherwise just report options.
2. **Core inventory.** For each folder of `TrajectoryPlanner.Core` list the public types and their one-line purpose, and the theory document it implements (table). Flag any folder with no clear theory counterpart.
3. **Test coverage by area.** For each Core folder, which tests exercise it (24 tests in total: say which areas have none or only smoke tests). Do not add tests in this request.
4. **Pass-through solver.** Describe precisely what `PassThroughSolver1D` computes today (inputs, how interior waypoint derivatives are chosen, what is a heuristic) and how it differs from the minimum-snap formulation of `theory/TrajectoryPassingThroughConstraintPoints.md` and Decision 5.
5. **Where logic lives.** `PrototypeTrajectoryService` (App) orchestrates computation while `TrajectoryBuilder1D/3D` live in Core. List which orchestration steps in the App could be done by the builders, and anything calculation-related that currently sits in the UI layer.
6. **Theory/code discrepancies.** Anything you notice where code and a theory document disagree (document, section, what differs). Do not resolve; just list.
7. **Math.NET Numerics.** Is it actually used (Decision 4)? If not, which root-finding/integration approach is used instead?

## Acceptance criteria

- [ ] A return note `coordination/inbox-docs/0001-baseline-review.md` (template `_TEMPLATE-note.md`) answers items 1–7.
- [ ] No behavioral change to the app; if any file is edited, say which and why.
- [ ] `dotnet build` and `dotnet test` still succeed.
- [ ] `coordination/STATUS.md` updated (request 0001 closed or blocked).

## Notes for the author (educational)

Item 4 is the most useful for you: it shows in plain terms what "minimum snap" means in practice and how far the current solver is from it.
