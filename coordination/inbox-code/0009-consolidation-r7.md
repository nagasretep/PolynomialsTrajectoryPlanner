# 0009 — Consolidation R7: the App uses the Core, legacy removal

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

The App is a thin layer over the Core: parsing the inputs, calling Core builders, formatting results. No trajectory logic (duration choice, blend point search, limit validation, sampling) is duplicated in the App. The Core blending follows the theory (selectable degree, limit verification, input validation). Legacy duration code is gone. Do the work in the three parts below, in this order, running the full test suite after each part.

## Reference documents

- `theory/BlendingSegments.md` (Baseline steps 7 and 10), `theory/TrajectoryConstraints.md` (*Shortest Feasible Duration*)
- `design/07-implementation-plan.md` (A6–A9), `design/08-ui-first-prototype.md`
- `roadmap/discrepancies.md`: D7, D11, D15 (and D4 only for awareness)

## Scope

### Part A — Core blending (D7)

1. **Selectable blend degree** (3/5/7/9) as a parameter of the Core blend builder. Default: the degree of the neighbouring segments if they agree, 9 otherwise; state the rule you choose in the code and the note.
2. **Limit verification**: with limits active the blend segment takes its shortest feasible duration (`ShortestFeasibleDuration`, as for ordinary segments); a prescribed blend duration is only verified. Failure is explicit (reuse `KinematicLimitsViolatedException` / `DurationSearchFailedException`, message naming the blend).
3. **Input validation**: an absolute blend position outside the segment is rejected with `ArgumentException` (today it is silently clamped); an empty window is rejected (today accepted). The $\tau \in [0.001, 0.999]$ clamp for $\tau$-based entry points stays.
4. The `Today_…` blending tests pin the old behaviour: update exactly those that now change, and say which in the note. Do not weaken other tests.

### Part B — App consolidation (D11, D15)

1. `PrototypeTrajectoryService` calls the Core for: pass-through (already), constant velocity (already, 0007/0008), blending (Part A), limit verification and sampling. Remove its own blend point search, limit validation and sampling code. Constant-velocity code paths stay as they are.
2. Remove the per-segment `ComputeDuration` of the duration strategies and clean the interface (D15); keep behaviour of the three strategies (user-specified, proportional A8, constraint-aware).
3. If a piece of logic in the service is not specific to the UI, move it to the Core rather than keeping it in the App.
4. **Tests for the service.** Check whether `PrototypeTrajectoryService` can be tested from a test project without WPF (it is a plain class). If yes, add tests for the cases you exercised by hand in 0006–0008 (the console runs); if the WPF target prevents it, move the testable logic to the Core and test it there. Explain the choice in the note.

### Part C — Legacy removal

1. Delete `DurationSearcher.FindValidDuration`, `CreateSegmentWithAutoDuration` and any other code no production path uses.
2. Their tests: delete those that only exercise the legacy code. The "legacy versus new" comparison in `ShortestFeasibleDurationTests` is replaced by checks that do not need the legacy search (closed form, feasibility of the result, F(T) ≈ 0). Keep everything else that tests behaviour.
3. Search for remaining unused public members left by the earlier requests and list them in the note (delete only the clearly dead ones).

- Out of scope: 3D constant velocity and blending (R9), new UI features (R10), pass-through with limits (R11), the open question about finer scans for degree 3 (D18).

## Acceptance criteria

- [ ] The App project contains no duration search, no blend point search, no limit verification, no polynomial evaluation or sampling of its own (list in the note what remains and why).
- [ ] Blending: a degree-5 blend between degree-5 neighbours works; a position outside the segment and an empty window are rejected; with an active limit the blend duration is the shortest feasible one and the whole trajectory passes `VerifyLimits`; infeasible limits give an explicit error naming the blend.
- [ ] `FindValidDuration` and `CreateSegmentWithAutoDuration` no longer exist.
- [ ] `dotnet build` free of new warnings; all remaining tests pass; the note states the test count before and after and which tests were removed or rewritten, with the reason.
- [ ] The App compiles and, as far as can be checked without the window, the service gives the same results as before for unchanged cases (list a few before/after numbers).

## Expected tests

- Blend degree selection and the default rule; each limit binding in turn (v, a, j, s); prescribed blend duration kept and verified; out-of-range position; empty window; infeasible limits.
- Service-level tests per Part B.4.
- Rewritten comparison tests per Part C.2.

## Notes for the author (educational)

Explain in the return note: what "thin layer over the Core" means here and how to recognise logic that belongs in the Core; why the blend now needs a shortest feasible duration like any other segment; what the user sees differently (error instead of silent clamp). Report anything you find odd in the App code while moving it.

Return note: `coordination/inbox-docs/0009-consolidation-r7.md`, ending with a "Suggested commits" section (separate commits for Part A, Part B and Part C if their files allow it).
