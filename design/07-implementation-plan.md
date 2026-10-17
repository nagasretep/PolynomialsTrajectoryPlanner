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

Candidates identified at planning time were: LiveCharts2, OxyPlot, ScottPlot.

**Status update after M2 implementation**: this decision has now been resolved by `08-ui-first-prototype.md` and the implemented WPF prototype. The chosen charting library is **ScottPlot**.

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

## Appendix — Stable Facts Emerged During Initial Implementation

The following points were decided implicitly during the first implementation pass in `src/` and are now recorded here so they do not remain only in chat history or in code:

### A1 — Decision 2 Was Refined Into Typed 1D/3D Boundary Conditions

Decision 2 originally proposed one minimal, degree-agnostic `BoundaryCondition` sketch as a starting point.

The implemented baseline refines that sketch into two typed records:

- `BoundaryCondition1D`
- `BoundaryCondition3D`

Both carry:

- `DerivativeOrder`
- `BoundaryEndpoint` (`Start` / `End`)

and then:

- a scalar `double` value in `1D`;
- a `Point3D` value in `3D`.

This is a post-plan implementation refinement, not a theory change. Its purpose is to keep dimensional typing explicit while also making endpoint identity part of the condition itself rather than leaving it implicit.

### A2 — The Duration Abstraction Was Concretized

Decision 1 proposed a small shared abstraction for duration handling. The initial implementation has concretized that idea as:

```
DurationMode { UserSpecified, AutoDerivedProportional, AutoDerivedFromConstraints }
```

with strategy implementations behind `IDurationStrategy1D`.

This makes one earlier chat-level idea explicit: the project does not use a single monolithic duration rule, but a small strategy family that already separates:

- directly entered durations;
- automatically distributed durations;
- constraint-aware duration search.

### A3 — Trajectory Construction Uses Builder-Style Services

The initial implementation has adopted builder-style orchestration services as the main application-facing assembly layer:

- `TrajectoryBuilder1D`
- `TrajectoryBuilder3D`

These builders centralize the composition steps that otherwise would be scattered across the UI layer:

- add single segments;
- create rest-to-rest segments;
- build pass-through trajectories from waypoint arrays;
- insert blends between adjacent segments.

This is now a stable architectural fact of the implementation baseline.

### A4 — The Test Stack Is xUnit And Remains Core-Only

The testing project `TrajectoryPlanner.Core.Tests` has been implemented with **xUnit**.

This records two practical implementation choices that were previously only implicit in `src/`:

- **test framework**: xUnit;
- **test scope**: tests target `TrajectoryPlanner.Core` only, not the WPF layer.

This remains fully consistent with the original separation principle of keeping mathematical logic testable independently from the UI.

### A5 — Milestone M2 Evolved Into A Chart-First Prototype Layout

The original milestone sketch described a minimal shell with boundary-condition input and a single-segment plot.

The implemented and now stabilized M2 baseline is more specific:

- the application uses a **chart-first layout**, with inputs and reports in a left control rail;
- the visualization area is the dominant region of the window;
- the plotted quantities are displayed as **stacked plots with independent Y scaling** rather than one compressed shared-axis plot;
- the window uses the project icon `assets/calculus.png`;
- segment management includes both **add** and **remove selected segment**, while preserving at least one segment at all times.

These are implementation-level refinements of M2, not changes to the underlying theory scope.

### A6 — Math.NET Numerics: Actual Usage (as built, reviewed 2026-10-17)

Decision 4 planned Math.NET for root-finding and numerical integration. The implemented code uses it **only for dense linear algebra** (the LU solve of the boundary-condition system in `PolynomialCoefficientSolver`). Root finding is hand-written (100-point sign-change scan plus bisection in `ExtremaAnalyzer`; a Newton iteration for the blend point), and no numerical integration exists yet (the case reserved for it, arc-length integration for 3D constant velocity, is not implemented).

Status: accepted as is. The hand-written root finders work on the polynomials involved and keep the algorithm visible, which suits the educational goal; moving to `MathNet.Numerics.RootFinding` is deferred until robustness problems appear. The exact minimum-snap solve (A7) adds a second use of the dense linear algebra (Cholesky).

### A7 — Decision 5 Made Concrete

`theory/TrajectoryPassingThroughConstraintPoints.md` (section *Global Minimum-Snap Formulation In 1D*) now states the minimum-snap problem explicitly: interior derivatives as unknowns, quadratic cost, one linear system. Consequence for the architecture note of Decision 5: the "smoothness criterion" is fully described by its **derivative order $m$** (snap: $m = 4$), not by a pluggable cost function; the solver builds the quadratic form from $m$. The current finite-difference heuristic in `PassThroughSolver1D` is superseded (request 0002). The choice of $m$ for degree 9 (snap versus crackle) is an open decision of the author, see that document's Open Questions.

### A8 — Proportional Durations: Relative Floor, Total Preserved

Decision 1 sets the pass-through default duration "proportional to the distance between consecutive waypoints". The first implementation used an absolute floor of 0.05 s per segment, which can make the sum exceed the requested total (for a small total and many segments). Rule adopted (2026-10-17): with $\Delta_i = \lvert x_{i+1} - x_i \rvert$ and $\bar{\Delta}$ their mean, use the weights $w_i = \max(\Delta_i,\; \varphi\,\bar{\Delta})$ and durations $T_i = T_{tot}\, w_i / \sum_j w_j$, with $\varphi = 0.05$ as a constructor parameter. The sum of the durations is then always exactly $T_{tot}$; if all $\Delta_i = 0$ the split is equal. In 3D, $\Delta_i$ is the Euclidean distance.

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
