# Understanding Questions (criterion D4)

Purpose: help the author complete and sign the understanding checklist in `05-success-criteria.md`. One block per module, in the order of the checklist. For each module:

1. read the theory document named in the block (the headings to look at are given);
2. answer the questions **in your own words, without looking** (aloud, on paper, or in a note: any language);
3. then compare with `06-understanding-answers.md` (read it only after answering that module);
4. tick the four columns of the checklist for that module only if true: *Theory* (the document explains the module completely), *Algorithm* (step by step), *Decisions* (recorded with reasons), *Understood* (you could explain the module to a colleague).

A question you cannot answer is not a failure of the author: it is either a gap in the document (tell Cowork, the document gets fixed) or something to re-read. Do not tick *Understood* for a module with questions you could not answer.

## 1. Symbols and nomenclature

Document: `SymbolsAndNomenclature.md` (including *Symbols Introduced By Later Documents*).

1. Why are the polynomial coefficients called $c_i$ and not $a_i$?
2. List the derivatives of position in order, with their names (up to the sixth).
3. What is the difference between $t$, $T$ and $\tau$?
4. The letters $\sigma$, $\nu$, $\eta$, $\lambda$ each appear in more than one document. What does each one mean, and why were they chosen to be different from one another?
5. How is a 3D quantity written differently from a 1D one?

## 2. Ninth-degree polynomial and lower degrees

Documents: `NinthDegreePolynomialTheory.md`, `PolynomialCoefficientDetermination.md`.

1. How many coefficients has a polynomial of degree $d$, and how many conditions fix it?
2. Which conditions does each of the degrees 3, 5, 7, 9 use at each end?
3. What is the Horner form and why is it preferred for evaluation?
4. Why is the system solved in normalized time $\tau \in [0,1]$ and then scaled, instead of solving it in absolute time?
5. How does the $k$-th derivative of a segment of duration $T$ relate to the $k$-th derivative of the same segment with duration 1?
6. Why do higher derivatives matter for finding extrema of position?

## 3. Boundary conditions

Document: `BoundaryConditions.md`.

1. What is a *necessary* condition, a *possible* one, a *redundant* one? Give an example of each for degree 5.
2. Why does degree 9 need exactly snap at both ends, and what happens if you leave it out?
3. What does "rest to rest" mean in terms of the conditions?
4. What changes when the segment is 3D?

## 4. Constraints: extrema, shortest feasible duration

Document: `TrajectoryConstraints.md`.

1. Where can the maximum of $|v(t)|$ on a segment occur? How are the candidate points found?
2. How is a maximum acceleration limit checked on a segment?
3. For a rest-to-rest segment, how does the peak of the $k$-th derivative change with the duration $T$? What closed-form duration follows from a limit $L_k$?
4. What is the *shortest feasible duration* and how does the program find it when boundary derivatives are not zero?
5. Why is the duration search for degree 3 not simply "make it longer until it fits"?
6. In 3D, are limits applied to the vector length or to each axis? Why?

## 5. Constant velocity, 1D and 3D

Document: `ConstantVelocitySegment.md`.

1. In 1D, how is the constant-velocity stretch assembled from the original segment? What are the three pieces?
2. State the three duration rules for the transitions in your own words (no limits, limits, prescribed total).
3. What does "constant speed" mean on a curved 3D path, and how is it different from a constant velocity vector?
4. Why is arc length $\ell$ needed, and what is computed from it (list the quantities)?
5. What is the admissible speed $v_{adm}$ and how is it computed? What does the program do if the requested speed is higher?
6. For the parabola $(t, t^2, 0)$ on $[0,1]$ with speed 1, what is the duration of the stretch and why?

## 6. Blending

Document: `BlendingSegments.md`.

1. What is removed from the two neighbouring segments, and what takes its place?
2. Up to which derivative must the blend match the neighbours at each junction?
3. How is the duration of the blend chosen, with and without limits?
4. What happens to the original join point?
5. In 1D a blend position is an absolute position; in 3D a point. How is it converted to a parameter on the segment, and what can go wrong?
6. How is a blend different from an exact pass-through, in one sentence?

## 7. Exact pass-through (1D, 3D, with limits)

Document: `TrajectoryPassingThroughConstraintPoints.md`.

1. What are the unknowns of the pass-through problem, and what is fixed?
2. What does "minimum snap" mean as a formula, and why is the result called exact?
3. How does the smoothness order $m$ relate to the degree, and what is the default for degree 9?
4. When the segment durations are scaled by $\lambda$, how do the peaks of velocity, acceleration, jerk and snap scale?
5. How is $\lambda^\star$ found with limits active? What is the *binding* limit?
6. In 3D, why are the three axes three independent problems, and what ties them together? Why must the durations be common?
7. Why is one common $\lambda$ used for all axes and not one per axis?
8. Why is a pass-through more constrained than a blend (Requirement 14 against 13)?

## 8. Application and charts

Document: `design/08-ui-first-prototype.md` and the user manual `manual/UserManual.md`.

1. Why is the calculation logic in the Core and not in the window?
2. What are the four modes and what does each keep when you switch?
3. What does the project file contain, and why are results not saved?
4. Why does the program take the *shortest feasible* duration when you do not give one?
5. Which numbers on the screen would you check first if you suspected a mistake, and against what?
