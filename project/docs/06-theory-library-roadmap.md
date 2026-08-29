# Theory Library Roadmap

This is the working checklist for the "personal library" of theory/technical documents to be produced with the AI IDE, one requirement-cluster at a time. Each entry maps to the requirement(s) it covers in `01-requirements.md`, so coverage can be checked off as documents are produced.

Suggested per-document template (for consistency across the library): short intro / scope → symbols used (referencing `SymbolsAndNomenclature`) → theory / derivation → algorithm or procedure → worked considerations (symbolic, per `03-assumptions.md`) → open questions / links to related documents.

## Format convention

- **Source format**: every document in this library is written in **Markdown** (`.md`), consistent with the specification files (`00`–`05`).
- **Mathematical notation**: formulas and equations are written as LaTeX syntax within the Markdown: inline math uses single delimiters like `$x(t)$`, while display equations on their own line use double delimiters like `$$x(t) = \sum_{i=0}^{9} a_i t^i$$`. Math delimiters are never wrapped in backticks. This keeps documents renderable in-editor (MathJax/KaTeX) and diffable in version control.
- **Reading / archival output**: when a readable/printable version of the library is needed, it is generated from the Markdown sources via `pandoc` (LaTeX engine) into PDF — the Markdown+LaTeX-inline files remain the single source of truth; PDF is a generated artifact, never edited directly.
- **Not used as a working/source format**: LaTeX (`.tex`) as primary source, Word (`.docx`), ODT, plain `.txt`. See discussion in chat for the rationale.

## Pre-publication verification checklist

Before considering any document in this library "finished", verify it with the following steps:

1. Compile it: `pandoc <file>.md -o <file>.pdf --pdf-engine=xelatex -V mainfont="<a system font>" -V geometry:margin=2.5cm`
2. **Check inline math specifically** (formulas embedded in running text, not standalone display equations on their own line) — this is where syntax errors are most likely to hide, since display-equation blocks tend to be visibly broken (and so get caught immediately) while inline math can silently render as literal text instead of a formula.
3. Confirm backticks are only used around genuine code/filename references, never around `$...$` math delimiters — inline math must be single `$...$`, with no surrounding backticks (backtick-wrapped `$$...$$` renders as literal text instead of a formula).
4. If the PDF build fails with a missing LaTeX package (e.g. `lmodern.sty` not found), install the missing package via the local TeX distribution (MiKTeX installs it automatically on Windows; `tlmgr install <package>` on TeX Live) — or work around it by forcing a system font with `--pdf-engine=xelatex -V mainfont="<font name>"`.
5. **In tables, never write a multi-symbol list as a single inline math block** (e.g. `$x_0, x_f, v_0, v_f$`) — LaTeX cannot line-wrap inside one math box, so it silently overflows into the next column instead of wrapping. Write each symbol as its own inline math, comma-separated in plain text (e.g. `$x_0$, $x_f$, $v_0$, $v_f$`), so the cell can wrap normally.
6. **Respect heading hierarchy — never place two consecutive headings at the same level when the second is logically a subsection of the first** (e.g. a `##` "1D Blending Problem" section immediately followed by a `##` "Nominal Consecutive Segments" that is actually part of it). The second heading should be one level deeper (`###`).

| # | Document | Covers (requirements) | Status |
|---|---|---|---|
| 1 | `SymbolsAndNomenclature` | Req. 1 — symbology for 1D/3D polynomial paths and solid orientation (Euler angles, quaternions, rotation matrices — notation reserved now, full treatment later per `04-future-topics.md`) | Draft v1 |
| 2 | `NinthDegreePolynomialTheory` | Req. 2 — canonical & Horner form, derivatives up to the 6th (velocity → pop), pros/cons discussion | Draft v1 |
| 3 | `BoundaryConditions` | Req. 3 (learning part) — full list of boundary conditions needed to determine the polynomial | Draft v1 |
| 4 | `PolynomialCoefficientDetermination` | Req. 3 (technical part) + Req. 4 — coefficient-calculation procedure; absolute vs. normalised period analysis | Draft v1 |
| 5 | `TrajectoryConstraints` | Req. 7–10 — max/min position/velocity/acceleration/jerk/snap analysis; imposing velocity/acceleration/jerk limits | Draft v1 |
| 6 | `ConstantVelocitySegment` | **Req. 12** — constant-speed segment within a degree-9 path (start/end position, absolute or percentage) | Draft v1 |
| 7 | `BlendingSegments` | Req. 13 — connecting consecutive segments without a pass-through constraint at the join | Draft v1 |
| 8 | `TrajectoryPassingThroughConstraintPoints` | Req. 14 — smooth connection with precise transition points (zero velocity/acceleration/jerk/snap at path start/end) | Draft v1 |

> **Note on item 6**: this document was not in the original list of documents discussed — it covers requirement 12 (constant-velocity segment), which is distinct both from `TrajectoryConstraints` (general kinematic limits) and from `BlendingSegments`/`TrajectoryPassingThroughConstraintPoints` (continuity *between* segments, per the `Constraints` vs. `Blend` distinction in `02-data-model.md`). It is placed here because it logically follows the general constraints treatment and precedes the inter-segment blending topics.

## Out of scope for this roadmap (tracked elsewhere)

- Requirement 15 (the application itself, graphical interface, extension to degrees 3/5/7) is not a theory document — it is the implementation phase in `src/`, to start once the library above (or at least items 1–5) is in reasonably solid shape.
- Requirements 6 and 11 (1D → 3D progression) are not separate documents — per the general rule in `00-vision.md`, each document above covers 1D first and notes 3D differences inline.
- Future Topics (orientation in full, time scaling, multi-axis sync, numerical stability) remain in `04-future-topics.md` until promoted to their own roadmap entries.
