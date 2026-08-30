# Implementation Plan

## Intro And Scope

This document is the bridge between the theory library (`SymbolsAndNomenclature` through `TrajectoryPassingThroughConstraintPoints`, tracked in `06-theory-library-roadmap.md`) and the software implementation phase in `src/` (Requirement 15).

Its purpose is to consolidate the implementation-oriented Open Questions scattered across the eight theory documents — many of which turned out to be the same underlying decision restated in different words — into a single set of resolved (or deliberately deferred) decisions, before any C#/WPF code is written. This mirrors, for the theory→code transition, the same discipline already applied when the theory library itself was planned in `06-theory-library-roadmap.md`.

This document does not re-derive any theory. It only decides how the already-validated theory gets translated into software structure.

## Consolidated Decisions

Each decision below traces back to one or more Open Questions in the theory library, listed for reference.

### Decision 1 — Segment/Blend/Transition Duration Strategy

*Open Questions consolidated from*: `TrajectoryConstraints` (Q2), `BlendingSegments` (Q1), `TrajectoryPassingThroughConstraintPoints` (Q2).

There is no single duration strategy for the whole project — the right default differs by context:

- **General kinematic limits** (`TrajectoryConstraints`): already decided in that document — **recompute-and-verify**. Choose an initial duration, verify the active constraint, increase duration and recompute if violated, repeat.
- **Constant-velocity segment** (`ConstantVelocitySegment`): not a duration-strategy case at all — $T_{cv}$ is fully determined by the distance/speed formula already established there. No decision needed.
- **Blending** (`BlendingSegments`): **default = inherited from the removed original portions** of the neighboring segments. Simplest to implement first, geometrically intuitive.
- **Exact pass-through** (`TrajectoryPassingThroughConstraintPoints`): **default = derived from waypoint spacing** (segment duration proportional to the distance between consecutive waypoints).

**Architecture note**: implement duration selection behind a single small abstraction (e.g. an `IDurationStrategy` per context, or a shared enum `{ UserSpecified, AutoDerived }`) reused across all four contexts, rather than four separate ad-hoc mechanisms. All defaults above remain replaceable later by a constrained search (duration as an optimization variable) without changing calling code.

### Decision 2 — Boundary Condition / Waypoint State Representation

*Open Questions consolidated from*: `BoundaryConditions`, `PolynomialCoefficientDetermination` (Q3), `TrajectoryPassingThroughConstraintPoints` (Q3).

A single, degree-agnostic representation is adopted instead of per-degree typed structures:

```
enum DerivativeOrder { Position, Velocity, Acceleration, Jerk, Snap }

record BoundaryCondition {
    DerivativeOrder Order;
    double Value;              // Point3D / Vector3D in 3D, per Decision-by-context
}
```

`necessaryConditions[]`, `possibleConditions[]`, and `redundantConditions[]` (per `02-data-model.md`) are plain lists of `BoundaryCondition`. Which derivative orders are required for a given degree (per `BoundaryConditions.md`'s classification table) is implemented as a separate validation lookup, not as separate classes per degree — one representation for degrees 3, 5, 7, and 9 alike.

The same `BoundaryCondition` record is reused for shared interior waypoint states in `TrajectoryPassingThroughConstraintPoints` — a waypoint state is simply a small collection of `BoundaryCondition` values at one point.

This is a deliberately minimal starting representation — see *Open Points Carried Forward* below for its planned refinement trigger.

### Decision 3 — Absolute Vs Percentage Specification

*Open Questions consolidated from*: `ConstantVelocitySegment`, `BlendingSegments` (Q2), `BoundaryConditions` (UI open question).

**DECISION**: positions and reference points (blend points, constant-velocity sub-segment endpoints, etc.) are specified in **absolute** terms relative to the segment endpoints. Percentage-based specification is not implemented in this baseline.

**Accepted trade-off**: if a segment's original endpoints are later moved, absolute reference points do not automatically follow — they would need to be re-entered. This is accepted given the project's scope (segments are not expected to be moved frequently once set up).

### Decision 4 — Numerical Strategy For Non-Closed-Form Cases

*Open Questions consolidated from*: `TrajectoryConstraints` (Q1), `ConstantVelocitySegment`'s implementation note.

**DECISION**: implement the strategy fully — both branches, not a single simplified path:

- **Closed-form branch**: used whenever the theory guarantees it (single-axis constant-velocity segments, all Horner/canonical polynomial evaluation, degree ≤ 4 stationarity equations where applicable).
- **Numerical branch**: used only where the theory shows no closed form exists (root extraction for stationarity polynomials of degree ≥ 5 in `TrajectoryConstraints`; arc-length integration and inversion for genuinely curved multi-axis constant-velocity segments in `ConstantVelocitySegment`).

**Library choice**: **Math.NET Numerics** (mature, free, well-documented .NET library) for root-finding (e.g. Brent's method) and numerical integration, instead of a hand-written solver. This keeps the project's educational focus on trajectory planning rather than on numerical-methods implementation details, consistent with the constraint in `01-requirements.md` item 16 against learning new tools/techniques not strictly necessary to the project's actual goal.

Implementation should branch on the single-axis/straight-subpath detection established in `ConstantVelocitySegment` to avoid paying the numerical cost when the closed form applies.

### Decision 5 — Smoothness Criterion For Exact Pass-Through

*Open Question consolidated from*: `TrajectoryPassingThroughConstraintPoints` (Q1).

**DECISION**: **minimum-snap** is adopted as the sole smoothness criterion for determining shared interior waypoint derivatives, consistent with the special structural role of snap already established in `NinthDegreePolynomialTheory`.

**Architecture note**: implement the criterion behind a small interface (e.g. `ISmoothnessCriterion`) rather than a hard-coded cost function, so an alternative (e.g. minimum-jerk, computationally cheaper but less smooth) could be substituted later without restructuring the surrounding waypoint-solving code.

### Decision 6 — Polynomial Evaluation Form

*Open Question consolidated from*: `NinthDegreePolynomialTheory` (Q1).

**DECISION**: **Horner form** is used for all repeated numerical evaluation (sampling a segment at many time instants for plotting). Canonical form remains the reference form for symbolic derivation and documentation, per the distinction already established in `NinthDegreePolynomialTheory`.

## Deferred Decisions

These remain open on purpose — deciding them now, without the concrete context they depend on, would risk a premature and possibly wrong choice.

### Deferred — UI Exposure Choices

*Open Questions consolidated from*: `NinthDegreePolynomialTheory` (Q3), `BoundaryConditions` (UI open question), `BlendingSegments` (Q3).

Deferred to the UI design phase itself:

- which derivative levels are shown by default in the charts, and which are optional;
- whether 3D boundary conditions are entered axis-by-axis or in vector form;
- whether Requirement 13 and Requirement 14 are exposed as two separate modes or as one continuity tool with a pass-through toggle.

### Deferred — Charting Library

*Carried over from* `01-requirements.md` item 16, which explicitly left this choice for implementation time.

Candidates already identified: LiveCharts2, OxyPlot, ScottPlot. Decision deferred to the point where the first chart is actually needed (Milestone M2 below), when a short practical comparison will be made against the concrete requirement at hand rather than in the abstract.

## Open Points Carried Forward

- **Decision 2's representation is a deliberate starting point, not a final design.** It should be revisited once real GUI and data-persistence requirements exist — the trigger for revisiting is the start of UI design work, not a fixed milestone number.
- **Decision 1's defaults are replaceable** by a constrained-search strategy (duration as an optimization variable under active constraints) if the fixed defaults prove insufficient once constraint verification (`TrajectoryConstraints`) is exercised against real blend/pass-through segments.

## Project Structure

Two projects, to keep calculation logic testable independently of the UI, consistent with the educational goal of understanding the theory rather than the framework:

```
src/
  TrajectoryPlanner.Core/         # pure calculation library, no UI
    Symbols/                      # DerivativeOrder, shared constants
    Polynomials/                  # coefficients, canonical + Horner
    BoundaryConditions/           # BoundaryCondition record
    Constraints/                  # extrema, limits, duration search
    ConstantVelocity/             # closed-form + numerical branches
    Blending/                     # blend segment construction
    PassThrough/                  # global minimum-snap solve
  TrajectoryPlanner.App/          # WPF project, references Core
  TrajectoryPlanner.Core.Tests/   # unit tests against Core only
```

This structure lets each theory document map to one folder/namespace in `Core`, keeping the traceability the project has maintained throughout the theory library.

## Suggested Milestone Sequence

Following `03-assumptions.md` item 4 (single-axis analysis before multi-axis analysis), 1D is implemented and validated end-to-end before the 3D extension is added as a wrapping layer that reuses the same component-wise logic three times (per the baseline already established in `TrajectoryConstraints`).

1. **M1 — Core degree-9 engine (1D)** (`PolynomialCoefficientDetermination`, `NinthDegreePolynomialTheory`): `BoundaryCondition` model, coefficient determination, canonical and Horner evaluation, derivatives up to the 6th.
2. **M2 — Minimal WPF shell**: boundary-condition input, single-segment plot. This is where the charting library decision is made.
3. **M3 — Lower-degree support** (`BoundaryConditions`): extend M1 to degrees 3, 5, 7 per the per-degree table.
4. **M4 — Constraints** (`TrajectoryConstraints`): extrema analysis and limit verification, recompute-and-verify duration search.
5. **M5 — Constant-velocity segment** (`ConstantVelocitySegment`): closed-form branch first, then the numerical branch.
6. **M6 — Blending** (`BlendingSegments`).
7. **M7 — Exact pass-through** (`TrajectoryPassingThroughConstraintPoints`): global minimum-snap waypoint solve.
8. **M8 — 3D extension**: component-wise wrapping of M1–M7 into `Segment3D`/`Trajectory3D`.

## Relation To The Data Model

`02-data-model.md` remains the conceptual reference; this document adds the concrete representation decided in *Decision 2* for `BoundaryConditions`' array contents, and confirms `Constraints`/`Blend` map directly to the `Constraints/` and `Blending/` folders above. No further data-model revision is required to start M1.

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `NinthDegreePolynomialTheory.md`
- `BoundaryConditions.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `ConstantVelocitySegment.md`
- `BlendingSegments.md`
- `TrajectoryPassingThroughConstraintPoints.md`
- `02-data-model.md`
- `01-requirements.md` (item 16)
