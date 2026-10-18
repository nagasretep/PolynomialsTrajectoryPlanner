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
- Exact pass-through (Requirement 14) was out of scope for the first prototype; it is now specified in *Pass-Through Mode*.

## Resolution Of Open Points From 07-implementation-plan.md

- **Default derivatives shown in charts**: **RESOLVED** — all derivatives applicable to the chosen degree are shown. The implemented prototype presents them as stacked plots with shared time inspection rather than compressing them into one shared Y-axis plot (see *Output Requirements*).
- **3D boundary condition input (axis-by-axis vs. vector)**: **RESOLVED 2026-10-18** — by vector; see *3D Mode*.
- **Unified vs. separate tool for Blending (Req. 13) and Exact Pass-Through (Req. 14)**: **remains open**, and for a reason beyond UI design: the current pass-through solver (`PassThroughSolver1D.cs`) is still a provisional finite-difference-style heuristic, not the exact global minimum-snap formulation promised by `TrajectoryPassingThroughConstraintPoints.md`. Designing UI for that requirement now would risk having to redesign it once the solver reaches its final form. This prototype therefore covers Blending only; Exact Pass-Through is added to the UI once the underlying solver is completed.

## Pass-Through Mode (decided 2026-10-17)

Exact pass-through (Requirement 14) is exposed as a **separate mode** selected at the top of the window: `Blending | Pass-through`. The two modes keep their own inputs; switching mode does not delete the other mode's data. Blending and constant velocity are not available in pass-through mode (a combination would need theory that does not exist yet).

**Inputs in pass-through mode**

- **Degree** (3 / 5 / 7 / 9) and **smoothness order** $m$ (dropdown, from $1$ to $p = (d+1)/2$, default per degree: $m = p$, except degree $9$: snap, $m = 4$).
- **Global end conditions**: rest (default) or explicit values of the derivatives $v, a, j, s$ at the first and last waypoint, as many as the degree uses.
- **Waypoint table**: position (required) and duration of the segment that starts at that waypoint (optional). Add / remove row buttons; at least two waypoints.
- **Durations**: either all segment durations are filled (prescribed: kept and only verified against the limits) or all blank. A partly filled column is reported as an input error.
- **Total duration** (optional): if given with blank segment durations, the proportional rule of A8 distributes it and it is only verified against the limits.
- **Limits** (global, optional): maximum velocity, acceleration, jerk, snap (those the degree supports), as in blending mode.
- With blank durations and no total, the reference durations are always the **proportional rule of A8 with a total of one second per segment** (decided 2026-10-18; it avoids equal durations on unequally spaced waypoints). Without active limits these are the durations used; with active limits, they are scaled to the shortest feasible sequence with the common scale factor $\lambda$ (*Pass-Through With Kinematic Limits* in `TrajectoryPassingThroughConstraintPoints.md`).

**Outputs in pass-through mode**

- One continuous trajectory over all segments in the same stacked plots as blending mode, with the waypoints marked on the position plot.
- Numeric report: segment durations $T_k$ and the total; when limits are active, for each active limit the peak over the whole trajectory, the limit and the segment where the peak occurs (the binding limit is the one at 100 %); the factor $\lambda$ when it was computed, the states $(v, a, j, s)$ at every waypoint, the smoothness cost (exact integral, `SmoothnessCost.Of`), the extrema of every quantity over the whole trajectory, and the coefficients of every segment.
- Notices (status bar) for: $\lambda$ applied, a duration or total only verified and violating a limit (error), infeasible limits (error).

**Out of scope for this step**: saving and loading, editing waypoints on the plot, 3D.

## 3D Mode (decided 2026-10-18)

The first 3D increment is a third entry of the mode selector: `Blending | Pass-through | 3D`. The two 1D modes are unchanged and keep their data. 3D pass-through is **not** part of this increment (the Core planner is 1D; see `roadmap/ROADMAP.md`, R9c).

**Input form (decided by the author, 2026-10-18)**: by **vector**. Every quantity with three components is one row with three boxes `X | Y | Z` (position, velocity, acceleration, jerk, snap boundary conditions of the segment; blend points; stretch points). Numbers are parsed with `InputNumber`.

**Segment tabs** as in blending mode (add, remove, degree 3 / 5 / 7 / 9, boundary conditions the degree needs), with:
- **Constraints**: duration (optional), maximum velocity, acceleration, jerk, snap (those the degree supports). One value per quantity, applied **component-wise** to each of the three axes (the limits are per axis, equal for all axes).
- **Constant-speed stretch** (optional, curved or straight, one code path): speed value; start and end of the stretch given in one of three ways chosen by a selector: *points* (absolute `X|Y|Z`, accepted if on the segment's path), *path-parameter fractions* (0–1) or *arc-length fractions* (0–1). Optional durations of the entry and exit transitions, or a total duration, as in 1D.
- **Blend**: initial and final blend values toward the previous / next segment, as fractions of the segment parameter, or as points.

**Outputs**
- Stacked plots per quantity (position, velocity, acceleration, jerk, snap) as in 1D, each with **three curves** (X, Y, Z) and a checkbox to add the **magnitude** (norm); shared time cursor; cursor table with X, Y, Z (and norm).
- A **Path** area with the three projections XY, XZ, YZ (equal axes), the path coloured by part (segment, blend, entry / stretch / exit), waypoint markers, a marker at the cursor time. A true 3D view is not part of this increment.
- Numeric report: for each part its kind, duration and coefficients (polynomial parts); for a curved stretch its length, speed, duration and the admissible speed $v_{adm}$ with the binding limit; per quantity and per axis the maximum absolute value over the whole trajectory against the limit (same table form as the pass-through peaks); total duration and path length.
- Notices (status bar): blend duration adjusted by limits, transition search failures, speed above $v_{adm}$, stretch not on the path, singular path point (zero speed on the stretch).

**Out of scope**: saving, editing on the plot, 3D pass-through, per-axis limits, rotating 3D view.

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
- **Window icon**: the application window uses `assets/calculus.png`.

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
