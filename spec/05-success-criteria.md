# Success Criteria

A project is considered successful if:

- The mathematical theory is completely documented.
- Every algorithm is explained.
- Every software decision is justified.
- The generated application can calculate polynomial trajectories.
- The generated application can visualize the results graphically.
- A complete educational project log is produced.
- The author can understand the theory and implementation process.

## Measurable Criteria (decided 2026-10-18)

The qualitative criteria above are made checkable as follows. Each criterion has an identifier, a measurement and a verifier (automated test, hand check by the author, or sign-off by the author).

### Numerical correctness

| Id | Criterion | Measurement | Verifier |
|---|---|---|---|
| N1 | Boundary conditions are reached | Every polynomial segment reproduces its prescribed start and end states (position and the derivatives its degree uses) to within $10^{-9}$ (absolute, values of order 1) | automated tests |
| N2 | Junctions are continuous | At every junction between pieces of a trajectory (segment, blend, entry, stretch, exit, pass-through segment) the state is continuous up to the order the degrees allow, to within $10^{-9}$ | automated tests |
| N3 | Limits are respected | With active limits, no extremum exceeds its limit by more than $10^{-9}$ relative; a request that cannot meet the limits is reported explicitly (never silently altered); the path and the speed are never changed silently | automated tests |
| N4 | Reference values of the theory are reproduced | Every reference table in `theory/` (pass-through peaks and $\lambda$, parabola derivatives and $v_{adm}$, 13 s example, ...) is reproduced by an automated test at the precision stated in the theory | automated tests |
| N5 | Pass-through is the exact optimum | The pass-through solution has the minimum smoothness cost: the cost equals an independent evaluation to $10^{-6}$ relative, and the 3D result along one axis equals the 1D result to $10^{-9}$ | automated tests |

### Speed

Measured on the author's laptop, Release build, after the first run (warm-up), for the stated input sizes.

| Id | Criterion | Limit |
|---|---|---|
| S1 | Compute of one 1D segment (any degree, with limits) | $\le 100$ ms |
| S2 | Compute of a pass-through with up to 30 waypoints (1D or 3D, with limits) | $\le 1$ s |
| S3 | Compute of a 3D trajectory containing a curved constant-speed stretch (analysis and report included) | $\le 1$ s |

Verifier: automated timing tests that report the measured times (they may be marked as performance tests and run separately), confirmed once by the author on the laptop.

### Software quality

| Id | Criterion | Measurement | Verifier |
|---|---|---|---|
| Q1 | All automated tests pass | `dotnet test` green; the count is recorded in `coordination/STATUS.md` | automated |
| Q2 | Build without warnings | `dotnet build` has no warnings other than the known NU1701 | automated |
| Q3 | Every public algorithm of the Core is tested against the theory | each algorithm named in `theory/` has at least one test that cites a reference value or an independent check | Cowork review at R13 |
| Q4 | No unused public API | the unused public members left after the consolidation are removed | Code request |

### Application

| Id | Criterion | Measurement | Verifier |
|---|---|---|---|
| A1 | All four modes work | 1D blending, 1D pass-through, 3D, 3D pass-through: Compute, stacked plots, cursor, reports, notices; the hand-check list of the return note of the request that introduced each mode passes | author hand check |
| A2 | Errors are explicit | every input error and every infeasible request produces a message naming the field, segment, axis or quantity | automated tests and hand check |
| A3 | Projects can be saved and loaded | saving a project (all modes' inputs) and loading it again reproduces the same inputs exactly and the same results to $10^{-12}$ | automated tests |
| A4 | Short user manual | one page per mode, with a worked example the reader can reproduce and its expected numbers | author reads and follows it |

### Documentation and education

| Id | Criterion | Measurement | Verifier |
|---|---|---|---|
| D1 | Theory library reviewed | every document in `theory/` has status *Reviewed* in `roadmap/06-theory-library-roadmap.md` and agrees with the code (no open item in `roadmap/discrepancies.md` of kind theory) | Cowork review (R13) |
| D2 | Theory book builds | the PDF of the theory builds into `build/` without errors, with working cross references | build script |
| D3 | Decisions are justified | every decision of `roadmap/decisions/` and every closed discrepancy has its reason recorded | Cowork review |
| D4 | The author understands | the checklist below is completed and signed by the author | author sign-off |

#### Understanding checklist (to be completed by the author)

One row per module. The author marks each column when it is true for him: *Theory* (the document explains the module completely), *Algorithm* (the algorithm is explained step by step), *Decisions* (the decisions are recorded with reasons), *Understood* (the author can explain the module in his own words).

| Module | Theory document | Theory | Algorithm | Decisions | Understood |
|---|---|---|---|---|---|
| Symbols and nomenclature | `SymbolsAndNomenclature.md` | | | | |
| Ninth-degree polynomial and lower degrees | `NinthDegreePolynomialTheory.md`, `PolynomialCoefficientDetermination.md` | | | | |
| Boundary conditions | `BoundaryConditions.md` | | | | |
| Constraints: extrema, shortest feasible duration | `TrajectoryConstraints.md` | | | | |
| Constant velocity, 1D and 3D (straight and curved) | `ConstantVelocitySegment.md` | | | | |
| Blending | `BlendingSegments.md` | | | | |
| Exact pass-through (1D, 3D, with limits) | `TrajectoryPassingThroughConstraintPoints.md` | | | | |
| Application and charts | `design/08-ui-first-prototype.md` | | | | |

### Definition of version 1.0

Version 1.0 is reached when N1-N5, S1-S3, Q1-Q4, A1-A4 and D1-D4 are all satisfied, with the work items of `roadmap/ROADMAP.md` R13 (theory review and PDF), R14 (save and load), R15 (speed tests), R16 (user manual) closed.
