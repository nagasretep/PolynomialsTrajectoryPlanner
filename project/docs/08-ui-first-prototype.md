# UI First Prototype

## Intro And Scope

This document specifies the minimal graphical interface needed to exercise the `TrajectoryPlanner.Core` engine end to end, concretizing Milestone M2 of `07-implementation-plan.md`.

This is a **validation testbed**, not the final application UI. Its only purpose is to let the project author input segment data, apply constraints/constant-velocity/blend settings, and see numeric and graphical results — so the boundaries stay deliberately tight, to avoid features creeping in beyond what is needed to validate the Core.

**Scope for this prototype**: 1D only, consistent with `03-assumptions.md` item 4 (single-axis analysis before multi-axis analysis). 3D UI is explicitly out of scope here (see *Resolution Of Open Points* below).

**Requirement coverage**: this prototype covers Requirement 13 (`BlendingSegments`) only. Requirement 14 (`TrajectoryPassingThroughConstraintPoints`, exact pass-through) is deliberately excluded — see *Resolution Of Open Points* for the rationale.

## Operational Flow

1. Create a segment (segment count starts at one, increments via a button once the current segment's data is complete, and the currently selected segment tab can be removed through a dedicated button).
2. For the active segment, choose the polynomial degree and enter the boundary conditions required for that degree.
3. Optionally select one or more active constraints and enter their values.
4. Optionally enable a constant-velocity stretch within the segment.
5. If at least two segments are defined, optionally set blend values connecting to the previous/next segment.
6. Compute.
7. Visualize: numeric report and chart.

## Input Requirements

A single tab-based panel per segment, with a segment-number indicator, a button to add a new segment, and a button to remove the currently selected segment. The remove button is disabled when only one segment exists, so the prototype always keeps at least one editable segment. After a removal, segment tabs are renumbered sequentially. Each segment tab contains:

- **Degree**: dropdown, values 3 / 5 / 7 / 9.
- **Boundary conditions**: the necessary conditions for the selected degree (per `BoundaryConditions.md`'s classification table), each with its value field. The set of fields shown updates when the degree changes.
- **Constraints**: the full constraint list is always shown; each entry is individually selectable (checkbox) with its own value field, and **multiple constraints may be active at once** on the same segment, per the multi-constraint baseline already established in `TrajectoryConstraints.md`:
  - period / duration;
  - maximum velocity;
  - maximum acceleration (shown only if the selected degree supports it);
  - maximum jerk (shown only if the selected degree supports it);
  - maximum snap (shown only if the selected degree supports it).
- **Constant-velocity stretch**: a toggle, and when enabled, the start and end of the stretch, expressed relative to the whole segment (the interval between the segment's start and end position from its boundary conditions) — absolute values, per Decision 3 in `07-implementation-plan.md`.
- **Constant speed value**: when the constant-velocity stretch is enabled, the user also enters the speed value for that stretch explicitly.
- **Blend values**: shown only when at least two segments are defined — initial blend value (toward the previous segment) and final blend value (toward the next segment), relative to the whole segment.

## Output Requirements

Two distinct outputs, produced after Compute:

- **Numeric report**: a table containing the polynomial coefficients and the maximum/minimum values of position, velocity, acceleration, jerk, and snap — limited to the quantities actually computable for the chosen degree.
- **Chart area**: a large visualization area occupying the dominant portion of the application window.
- **Per-quantity plots**: instead of compressing all curves into a single shared Y-axis plot, each computable quantity (position, velocity, acceleration, jerk, snap) is shown in its own vertically stacked plot with an independent Y scale, so each plot fully represents its own minimum and maximum values.
- **Cursor readout**: a shared time cursor across the stacked plots, with a numeric table beside the plots showing the value of each visible quantity at the cursor's current position.

## Explicitly Out Of Scope For Now

- Project save/load.
- Refined chart styling.
- Advanced analysis tools (e.g. differential cursors comparing two points) — only the single value-readout cursor described above is included.
- 3D input and visualization.
- Exact pass-through (Requirement 14) — see rationale below.

## Resolution Of Open Points From 07-implementation-plan.md

- **Default derivatives shown in charts**: **RESOLVED** — all derivatives applicable to the chosen degree are shown. The implemented prototype presents them as stacked plots with shared time inspection rather than compressing them into one shared Y-axis plot (see *Output Requirements*).
- **3D boundary condition input (axis-by-axis vs. vector)**: **remains open**, deferred to when 3D UI work begins (Milestone M8) — out of scope for this 1D-only prototype.
- **Unified vs. separate tool for Blending (Req. 13) and Exact Pass-Through (Req. 14)**: **remains open**, and for a reason beyond UI design: the current pass-through solver (`PassThroughSolver1D.cs`) is still a provisional finite-difference-style heuristic, not the exact global minimum-snap formulation promised by `TrajectoryPassingThroughConstraintPoints.md`. Designing UI for that requirement now would risk having to redesign it once the solver reaches its final form. This prototype therefore covers Blending only; Exact Pass-Through is added to the UI once the underlying solver is completed.

## Charting Library

**DECISION**: **ScottPlot**.

ScottPlot was confirmed and adopted for the prototype because:

- it supports interactive time inspection cleanly;
- it is lightweight and free;
- it allows a large stacked-plot layout without additional framework complexity.

This decision is no longer provisional.

## Implemented Clarifications

The following UI details were decided during the prototype implementation and are now fixed in this document:

- **Layout priority**: the chart area is the primary workspace. Inputs and reports are placed in a narrower left-side control/report column so visualization keeps most of the window width and height.
- **Optional controls**: constant-velocity and blend inputs are grouped as optional shape controls and may be collapsed when not needed, reducing wasted space.
- **Segment management**: the prototype exposes both add and remove actions at window level; removal always applies to the currently selected tab and is disabled when only one segment remains.
- **Window icon**: the application window uses `IconsAndGraphics/calculus.png`.

## Relation To 07-implementation-plan.md

This document is the concrete specification for Milestone M2. It resolves one of the two items under *Deferred — UI Exposure Choices* (default derivatives shown) and narrows the other two to an explicit scope boundary (3D deferred to M8; Exact Pass-Through deferred to when its solver is complete) rather than leaving them open-ended. It also records that the *Deferred — Charting Library* decision has now been finalized as ScottPlot.

## Related Documents

- `07-implementation-plan.md`
- `BoundaryConditions.md`
- `TrajectoryConstraints.md`
- `ConstantVelocitySegment.md`
- `BlendingSegments.md`
- `TrajectoryPassingThroughConstraintPoints.md`
- `06-theory-library-roadmap.md`
