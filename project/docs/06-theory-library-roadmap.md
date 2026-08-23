# Theory Library Roadmap

This is the working checklist for the "personal library" of theory/technical documents to be produced with the AI IDE, one requirement-cluster at a time. Each entry maps to the requirement(s) it covers in `01-requirements.md`, so coverage can be checked off as documents are produced.

Suggested per-document template (for consistency across the library): short intro / scope → symbols used (referencing `SymbolsAndNomenclature`) → theory / derivation → algorithm or procedure → worked considerations (symbolic, per `03-assumptions.md`) → open questions / links to related documents.

| # | Document | Covers (requirements) | Status |
|---|---|---|---|
| 1 | `SymbolsAndNomenclature` | Req. 1 — symbology for 1D/2D/3D polynomial paths and solid orientation (Euler angles, quaternions, rotation matrices — notation reserved now, full treatment later per `04-future-topics.md`) | Not started |
| 2 | `NinthDegreePolynomialTheory` | Req. 2 — canonical & Horner form, derivatives up to the 6th (velocity → pop), pros/cons discussion | Not started |
| 3 | `BoundaryConditions` | Req. 3 (learning part) — full list of boundary conditions needed to determine the polynomial | Not started |
| 4 | `PolynomialCoefficientDetermination` | Req. 3 (technical part) + Req. 4 — coefficient-calculation procedure; absolute vs. normalised period analysis | Not started |
| 5 | `TrajectoryConstraints` | Req. 7–10 — max/min position/velocity/acceleration/jerk/snap analysis; imposing velocity/acceleration/jerk limits | Not started |
| 6 | `ConstantVelocitySegment` | **Req. 12** — constant-speed segment within a degree-9 path (start/end position, absolute or percentage) | Not started |
| 7 | `BlendingSegments` | Req. 13 — connecting consecutive segments without a pass-through constraint at the join | Not started |
| 8 | `TrajectoryPassingThroughConstraintPoints` | Req. 14 — smooth connection with precise transition points (zero velocity/acceleration/jerk/snap at path start/end) | Not started |

> **Note on item 6**: this document was not in the original list of documents discussed — it covers requirement 12 (constant-velocity segment), which is distinct both from `TrajectoryConstraints` (general kinematic limits) and from `BlendingSegments`/`TrajectoryPassingThroughConstraintPoints` (continuity *between* segments, per the `Constraints` vs. `Blend` distinction in `02-data-model.md`). It is placed here because it logically follows the general constraints treatment and precedes the inter-segment blending topics.

## Out of scope for this roadmap (tracked elsewhere)

- Requirement 15 (the application itself, graphical interface, extension to degrees 3/5/7) is not a theory document — it is the implementation phase in `src/`, to start once the library above (or at least items 1–5) is in reasonably solid shape.
- Requirements 6 and 11 (1D → 3D progression) are not separate documents — per the general rule in `00-vision.md`, each document above covers 1D first and notes 3D differences inline.
- Future Topics (orientation in full, time scaling, multi-axis sync, numerical stability) remain in `04-future-topics.md` until promoted to their own roadmap entries.
