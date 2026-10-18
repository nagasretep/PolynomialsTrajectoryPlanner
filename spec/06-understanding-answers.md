# Understanding Questions: Reference Answers

Read a block only **after** answering the same block of `06-understanding-questions.md`. The answers are short; the document named in each block has the full reasoning.

## 1. Symbols and nomenclature

1. $a_i$ would collide with acceleration ($a(t)$, $a_0$, $a_f$), so coefficients are $c_i$.
2. Position, velocity, acceleration, jerk, snap, crackle, pop (derivatives 0 to 6).
3. $t$ is absolute time, $T$ the duration of a segment, $\tau = t/T$ the normalized time in $[0,1]$.
4. $\sigma = \pm 1$ is the 1D direction sign; $\nu(t) = \lVert \dot{\mathbf{p}} \rVert$ is the parametric speed of a path; $\eta \in [0,1]$ is a fraction of a segment; $\lambda$ is the common scale factor of the durations. Each letter has one meaning in the whole library, so formulas from different documents can be combined without ambiguity.
5. Bold letters ($\mathbf{p}$, $\mathbf{v}$, ...) with components $X, Y, Z$; 3D follows 1D component by component unless a document says otherwise.

## 2. Ninth-degree polynomial and lower degrees

1. A polynomial of degree $d$ has $d+1$ coefficients, fixed by $d+1$ conditions. Degree 9: ten conditions.
2. $(d+1)/2$ conditions per end: degree 3 uses position and velocity; 5 adds acceleration; 7 adds jerk; 9 adds snap.
3. Nested multiplication, $c_0 + \tau(c_1 + \tau(c_2 + \dots))$: fewer operations and better numerical behaviour than computing powers.
4. In normalized time the matrix is the same for every segment and only depends on the degree; the duration enters only through the scaling of the derivatives. It is better conditioned and is computed once.
5. The derivative of order $k$ with respect to $t$ equals the derivative with respect to $\tau$ divided by $T^k$.
6. The extrema of position are where velocity vanishes, the extrema of velocity where acceleration vanishes, and so on: each quantity's candidates come from the roots of the next derivative.

## 3. Boundary conditions

1. Necessary: the $d+1$ endpoint conditions, symmetric at the two ends, that determine the polynomial uniquely (degree 5: position, velocity and acceleration at both ends). Possible: conditions admissible in an extended formulation but not needed in the default one (for degree 5: a condition at an interior point in place of the acceleration at one end, or a free duration). Redundant: extra independent conditions beyond $d+1$ (for degree 5: a seventh condition such as jerk at the start): the problem becomes overconstrained, with no exact solution unless a condition is not really independent.
2. Ten coefficients need ten conditions; leaving snap out would leave two unknowns and the problem would not have one solution.
3. Position given at both ends and all the other conditions zero (velocity, acceleration, ... at both ends).
4. The conditions are vectors with $X, Y, Z$ components; each axis is solved independently with the same duration.

## 4. Constraints: extrema, shortest feasible duration

1. At the endpoints or at the roots of the acceleration inside the interval; the roots of the derivative polynomial are found by a scan for sign changes and bisection, checked against a sampling.
2. Find the extrema of acceleration (roots of jerk plus endpoints) and compare the largest absolute value with the limit.
3. $P_k(T) = P_k(1)\,T^{-k}$. From $P_k(T) \le L_k$: $T \ge (P_k(1)/L_k)^{1/k}$; the largest over the active limits is the shortest duration.
4. The smallest duration for which no limit is exceeded. With zero boundary derivatives it is the closed form; with non-zero ones the peaks no longer scale in a simple way, so the program grows the duration geometrically until it is feasible and then narrows the interval (a search, with an explicit failure message if none is found).
5. For degree 3 feasibility is not monotone in the duration (found in request 0008): a longer duration can be refused where a shorter one passed, so the search cannot rely on monotonicity and reports explicitly.
6. Each axis separately (component-wise), with the same value for all axes: simpler, predictable, and matches how axes of a machine have their own limits. It was decided on 2026-10-18.

## 5. Constant velocity, 1D and 3D

1. The segment is split into an entry transition (from the original start to the stretch), a middle affine piece with constant velocity, and an exit transition; the transitions are polynomial segments that match the boundary states.
2. No limits: the transition duration comes from a mean-velocity rule. Limits active: each transition takes its shortest feasible duration. Prescribed total: the total minus the stretch time is split in proportion to the shortest feasible durations.
3. The length of the velocity vector is constant while its direction changes with the path; the vector itself is not constant on a curve.
4. Constant speed means $\ell(t)$ grows linearly. The arc length gives the stretch length and duration ($L/v$), the position at any time (by inverting $\ell$), and, with the jet arithmetic, the derivatives with respect to arc length used for the boundary states of the transitions.
5. $v_{adm} = \min_k (L_k/P_k)^{1/k}$ over the active limits, where $P_k$ are the per-axis peaks of the arc-length derivatives. A higher requested speed is refused with an error that names the binding limit; the program never changes the path or the speed silently.
6. $L/v = 1.478943$ s, because the length of the parabola is $L = (2\sqrt{5} + \operatorname{asinh} 2)/4 = 1.478943$ and the speed is 1.

## 6. Blending

1. The portions of the two segments around the join (the blend window) are removed; a single blend polynomial takes their place.
2. The blend matches the full state at both junctions (position and every derivative the degree uses), so the result is continuous up to snap for degree 9.
3. The inherited duration is the sum of the removed portions (with a small floor); with limits it is kept if the blend respects them on every axis, otherwise it is replaced by the shortest feasible duration.
4. It generally disappears: the blend passes near it but not through it, because the part of the path around it is removed.
5. The segment is searched for the position: first crossing in 1D (error if the position is outside the covered range), closest point in 3D (error if the distance is more than $10^{-9}$ times the extent). A self-crossing path makes a point ambiguous, so fractions are safer.
6. A blend smooths a join and may miss the original point; a pass-through is forced through every point.

## 7. Exact pass-through (1D, 3D, with limits)

1. Fixed: waypoint positions, degree, order $m$, durations, end conditions. Unknown: the derivatives (velocity, acceleration, jerk, snap) at every interior waypoint.
2. Minimise $\sum_k \int_0^{T_k} (x^{(m)})^2 \, dt$ with $m = 4$ for snap. Exact because the problem is solved globally as a quadratic problem (a linear system), not by an iterative heuristic; the solution is the true minimum.
3. $m$ goes from 1 to $p = (d+1)/2$; default $m = p$, except degree 9 where it is snap, $m = 4$.
4. With zero boundary derivatives the peak of the $k$-th derivative scales as $\lambda^{-k}$.
5. Compute the peaks at $\lambda = 1$, then $\lambda^\star = \max_k (P_k/L_k)^{1/k}$; with non-zero boundary derivatives a search over $\lambda$. The binding limit is the one at 100 %, the one that stops $\lambda$ from being smaller.
6. The cost is a sum over the axes and each axis only has its own unknowns, so the 3D optimum is three 1D optima; what ties them is the time axis: all axes must reach the waypoint at the same instants, hence common durations.
7. A common $\lambda$ keeps the shape of the motion (proportions between segments and the path in space) and only changes the time scale; per-axis factors would distort the path.
8. In a blend the join point may be left; a pass-through must hit every point exactly, which fixes the positions and leaves less freedom, hence a tighter problem to solve (see the section *Why Requirement 14 Is More Constrained Than Requirement 13*).

## 8. Application and charts

1. So that it is tested once, shared by every client, and cannot disagree with the application; the window only translates text and draws.
2. Blending, pass-through, 3D, 3D pass-through; each keeps its own inputs and last result.
3. The text of every input box of all four modes and the selected mode; results are recomputed so they always come from the current algorithm and the file stays small and readable.
4. Because it is the fastest motion that respects the limits; a longer one is always possible by giving a duration, a shorter one is not allowed.
5. For example the end states (they must equal what you typed), the peaks against the limits (the binding one at 100 %), and the reference values of the manual and the theory.
