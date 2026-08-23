# Requirements

Tags: `[Learning]`, `[Technical]`, `[Learning & Technical]` — see `00-vision.md` for definitions. Where a requirement's dimensionality progression (1D → 3D) or symbolic-vs-numeric preference is not restated, the general rules in `00-vision.md` apply.

1. **`[Learning & Technical]`** Define the symbology to be adopted to clearly and correctly represent polynomial paths in one, two, and three dimensions, including symbols for orienting a solid in three-dimensional space (the latter to be detailed in a later step — see `04-future-topics.md` → Orientation). This is a starting point to be established before other points.

2. Regarding the polynomial of degree 9 (the first to be considered):
   - **`[Learning]`** Report the canonical form and the Horner form in the time domain; list pros and cons of the two forms.
   - **`[Learning]`** Consider the derivatives up to the sixth, starting from the position algorithm (velocity, acceleration, jerk, snap, crackle, pop); discuss the usefulness of using more or fewer derivatives for trajectory planning.

3. - **`[Technical]`** Report the mathematical procedure to calculate the coefficients of the polynomial, assuming all necessary boundary data has been specified.
   - **`[Learning]`** List all boundary conditions that must be specified to correctly determine the polynomial.

4. **`[Learning]`** Report the analysis (pros and cons) of using an absolute period vs. a normalised period.

5. **`[Learning & Technical]`** Symbolic-vs-numeric preference — see general rule in `00-vision.md`.

6. **`[Learning & Technical]`** Dimensionality progression for items 2–5 — see general rule in `00-vision.md`.

7. **`[Technical]`** For a polynomial of degree 9, carry out the following analysis where relevant: procedure and algorithms to determine max/min position, max/min velocity, acceleration, jerk, and snap.

8. **`[Technical]`** Procedure and algorithms necessary to impose a velocity limit on a polynomial of degree nine.

9. **`[Technical]`** As item 8, but imposing a maximum acceleration.

10. **`[Technical]`** As item 8, but imposing a maximum jerk.

11. **`[Learning & Technical]`** Dimensionality progression for items 7–10 — see general rule in `00-vision.md`.

12. **`[Learning & Technical]`** If a constant-speed segment is required within a path originally defined as a degree-nine polynomial, describe the considerations and algorithms needed to satisfy this. The constant-speed segment is specified by its start and end positions within the original path, in absolute terms or percentages (arbitrary choice of convenience). Cover 1D first, then 3D (per general rule).

13. **`[Technical]`** Complex trajectories are normally composed of more than one segment (from two to several dozen). The connection between consecutive segments should be specified by: the starting point of the connection (in the segment preceding the common connection point) and the ending point of the connection (in the segment following it) — in absolute terms or percentages. Report the strategy and algorithms involved. Cover 1D first, then 3D (per general rule).

14. **`[Learning & Technical]`** The smooth-connection approach from item 13 works correctly except when the path must pass exactly through the endpoints of the specified segments, avoiding a "zero velocity" event. A smooth connection with precise transition points (the segment endpoints) is required instead. Assuming velocity, acceleration, jerk, and snap are zero at the start of the first segment and the end of the last segment, report the considerations and solution strategy for this case, in both 1D and 3D (per general rule).

15. **`[Learning & Technical]`** The output resulting from the above points should be summarized in a software application including a graphical representation of the calculated profiles, based on data acquired from an input interface (details to be provided during development). At the moment only the degree-nine polynomial has been considered, but the final version will extend the same considerations/analysis/calculations to degrees seven, five, and three (only the applicable ones per degree). The second output is the project log — a single or multiple readable document(s) reporting notes, considerations, strategies, and theoretical treatments involved in the application.

16. **`[Technical]`** — **DECISION: desktop application, C#, WPF.**
    - **Platform**: desktop application (not web).
    - **Language / toolchain**: C#, WPF for the UI, a charting library (e.g. LiveCharts2 / OxyPlot / ScottPlot) for graphical visualization of the computed profiles.
    - **Rationale**: no requirement in this document calls for multi-user access, remote sharing, or cross-device execution — the tool is a single-user local analysis application, so the web's main advantages don't apply here. A web stack would additionally require frontend/backend tooling unrelated to trajectory planning, conflicting with the constraint below. A desktop app keeps all calculation logic (coefficients, derivatives, constraints, blending) in one project and one language, which best matches the educational goal of understanding the theory rather than a client/server architecture.
    - Constraint (kept from the original brief): the purpose of the project is primarily to deepen knowledge of trajectory planning, not to learn new tools/languages unless strictly necessary for that goal.
    - Constraint (kept from the original brief): the project author must remain involved in and aware of the specific content of the project software — the educational aspect must not be sacrificed for development speed, even if this slows progress.
    - At this stage no hardware/actuator/kinematics application is considered — theory only (see `03-assumptions.md`), even though the natural extension is toward robotic manipulators.
    - **Revisit if**: a future requirement emerges for multi-user access, remote/cross-device use, or integration with an online service — none of these apply today.
