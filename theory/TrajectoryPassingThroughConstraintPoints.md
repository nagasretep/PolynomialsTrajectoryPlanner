# Trajectory Passing Through Constraint Points

## Intro And Scope

This document studies the multi-segment trajectory case in which the path must pass **exactly** through the specified segment endpoints while still remaining smooth across the transitions.

It covers Requirement 14 of `01-requirements.md`:

- the ordinary smooth-connection approach of Requirement 13 is not sufficient when the path must pass exactly through the segment endpoints;
- the transition points must coincide with the specified segment endpoints;
- the trajectory must avoid a stop-and-go behavior at those interior points;
- velocity, acceleration, jerk, and snap are assumed to be zero at the start of the first segment and at the end of the last segment;
- the treatment starts in 1D and then extends to 3D.

This document is therefore the natural counterpart of `BlendingSegments.md`:

- `BlendingSegments.md` studies the case where the nominal join may be bypassed;
- the present document studies the case where the nominal join is itself a mandatory interpolation point.

The discussion remains symbolic-first and theoretical, consistent with `03-assumptions.md`.

## Symbols Used

The notation follows `SymbolsAndNomenclature.md`. The main symbols used here are:

- $x_k(t)$: the $k$-th scalar trajectory segment in 1D;
- $\mathbf{p}_k(t)$: the $k$-th vector trajectory segment in 3D;
- $x_i$: the $i$-th required waypoint in 1D;
- $\mathbf{p}_i$: the $i$-th required waypoint in 3D;
- $\mathcal{S}_i$: state attached to the $i$-th waypoint;
- $v_i$, $a_i$, $j_i$, $s_i$: velocity, acceleration, jerk, and snap assigned to waypoint $i$ in 1D;
- $\mathbf{v}_i$, $\mathbf{a}_i$, $\mathbf{j}_i$, $\mathbf{p}^{(4)}_i$: waypoint derivatives in 3D;
- $T_k$: duration of segment $k$;
- $\tau_k \in [0,1]$: local normalized time of segment $k$;
- $N$: number of segments;
- $M = N+1$: number of exact waypoints.

For the degree-$9$ reference case, the state attached to waypoint $i$ in 1D is:

$$
\mathcal{S}_i = \left(x_i, v_i, a_i, j_i, s_i\right)
$$

and, in 3D:

$$
\mathcal{S}_i =
\left(
\mathbf{p}_i,
\mathbf{v}_i,
\mathbf{a}_i,
\mathbf{j}_i,
\mathbf{p}^{(4)}_i
\right)
$$

## Why Requirement 13 Is Not Enough

The ordinary blending strategy of Requirement 13 removes a neighborhood around the nominal join and replaces it with a dedicated connection segment.

That is useful when:

- exact passage through the nominal join is not required;
- geometric and kinematic smoothing are allowed to shift the effective transition away from the original endpoint.

Requirement 14 changes the problem completely.

Here:

- the specified segment endpoints are mandatory interpolation points;
- the final trajectory must pass exactly through them;
- yet the motion should not collapse into a zero-velocity stop at every interior point.

So the key challenge is:

- exact waypoint passage;
- smooth multi-segment continuity;
- nontrivial motion through the interior waypoints.

This is no longer a local blend-replacement problem. It becomes a **global piecewise trajectory construction** problem.

## Exact Pass-Through In 1D

### Waypoint Sequence

Suppose the desired scalar trajectory must pass through the ordered waypoints:

$$
x_0,\; x_1,\; x_2,\; \dots,\; x_N
$$

with one segment between each consecutive pair.

Then segment $k$ connects:

$$
x_k \to x_{k+1}
$$

for:

$$
k = 0,1,\dots,N-1
$$

The final trajectory must satisfy:

$$
x_k(t_k^{end}) = x_{k+1}
$$

and:

$$
x_{k+1}(t_{k+1}^{start}) = x_{k+1}
$$

at the shared interior waypoint.

So the waypoint is not bypassed. It is interpolated exactly by both neighboring segments.

### Why A Stop-And-Go Solution Is Undesirable

An easy but poor solution would be:

- make each segment begin and end with zero velocity;
- do the same for acceleration, jerk, and possibly snap.

That guarantees exact passage through every waypoint, but it also creates a stop event at each one.

In symbolic form, this undesirable interior choice would be:

$$
v_i = 0
$$

at every interior waypoint $i$.

Requirement 14 explicitly aims to avoid this behavior.

So the project needs a formulation in which:

- the trajectory passes exactly through every waypoint;
- the shared interior state is smooth;
- interior velocity is generally nonzero unless the geometry or timing truly forces it to vanish.

### Boundary Conditions At The Global Ends

The requirement fixes the boundary behavior at the beginning and at the end of the whole trajectory:

$$
v_0 = a_0 = j_0 = s_0 = 0
$$

$$
v_N = a_N = j_N = s_N = 0
$$

for the degree-$9$ reference case.

These are global start/end conditions, not conditions to be imposed at every interior waypoint.

### Interior Waypoint States

At each interior waypoint:

$$
x_i, \qquad i = 1,\dots,N-1
$$

the position is known because the trajectory must pass through that point exactly.

The derivatives:

$$
v_i,\qquad a_i,\qquad j_i,\qquad s_i
$$

are not, in general, known in advance.

They must be treated as:

- either design variables specified by the user or by a higher-level rule;
- or unknown internal variables determined by a global solution strategy.

This is the central conceptual difference from ordinary endpoint interpolation of a single segment.

## Segment Representation In 1D

### One Segment Per Consecutive Pair

For the degree-$9$ reference case, segment $k$ can be written as:

$$
x_k(\tau_k), \qquad \tau_k \in [0,1]
$$

with duration:

$$
T_k
$$

and boundary states:

$$
\mathcal{S}_k^{start} = \left(x_k, v_k, a_k, j_k, s_k\right)
$$

$$
\mathcal{S}_k^{end} = \left(x_{k+1}, v_{k+1}, a_{k+1}, j_{k+1}, s_{k+1}\right)
$$

Once those two states and the duration $T_k$ are known, the coefficients of segment $k$ are determined exactly by the standard degree-$9$ coefficient procedure from `PolynomialCoefficientDetermination.md`.

### Automatic Continuity Through Shared States

If consecutive segments are built with the same interior waypoint state, continuity is automatic.

For example, at waypoint $x_i$:

$$
\mathcal{S}_{i}^{shared} =
\left(x_i, v_i, a_i, j_i, s_i\right)
$$

If segment $i-1$ ends with that state and segment $i$ starts with that same state, then the full trajectory is continuous up to snap at that waypoint.

So the continuity requirement does not need an extra correction step. It is built into the shared-state formulation itself.

## Why The Full Problem Is Global

### Local Segment Solves Are Easy

Once the waypoint states and segment durations are known, each segment is easy:

- it is just a standard endpoint interpolation problem;
- it can be solved independently from the others.

### Determining The Interior States Is The Hard Part

The actual difficulty is to determine:

$$
v_i,\quad a_i,\quad j_i,\quad s_i
$$

for every interior waypoint.

If only the positions:

$$
x_0,\; x_1,\; \dots,\; x_N
$$

and the endpoint zero-derivative conditions are prescribed, then the problem is not uniquely determined yet.

This is because:

- the segment coefficients depend on the waypoint states;
- the waypoint states are shared across segments;
- the exact passage constraints alone do not uniquely fix all those interior derivatives.

Therefore Requirement 14 requires a **global rule** in addition to exact interpolation.

## Three Families Of Global Strategy

The interior waypoint states may be determined in three broad ways.

### Prescribed Interior Derivatives

One may decide explicitly:

$$
v_i,\quad a_i,\quad j_i,\quad s_i
$$

at each interior waypoint.

This is the most direct strategy, but it requires the user or a higher-level planner to know those values.

### Heuristic Interior Derivatives

One may compute interior derivatives from neighboring waypoint positions and durations by a deterministic rule, for example:

- finite-difference-inspired velocity estimates;
- smoothed slope estimates;
- recursively derived higher derivatives.

This is simpler than full optimization, but it is still a design choice rather than a mathematically unique consequence of the interpolation constraints.

### Optimization-Based Global Determination

One may treat the interior waypoint derivatives as unknowns and determine them by minimizing a global smoothness cost while preserving exact waypoint passage.

Typical examples are:

- minimum-jerk-type criteria;
- minimum-snap-type criteria;
- weighted combinations of derivative costs.

This is the most systematic strategy and is the cleanest theoretical baseline for the present requirement.

## Recommended 1D Baseline For This Project

The recommended baseline is:

- exact pass-through at every waypoint;
- zero velocity, acceleration, jerk, and snap only at the global start and end;
- interior waypoint derivatives treated as unknown shared state variables;
- those unknowns determined by a global smoothness criterion;
- each segment then reconstructed from the resulting shared states.

For the degree-$9$ reference case, a natural smoothness criterion is to minimize a functional related to snap, because:

- degree $9$ naturally matches snap at both ends;
- snap already has a privileged role in the project's degree-$9$ theory;
- this criterion strongly penalizes overly abrupt changes in the motion law.

So the practical baseline picture is:

1. choose the exact waypoints;
2. choose the segment durations;
3. treat interior derivative values as global unknowns;
4. impose the global start/end derivative zeros;
5. solve for the interior states through a smoothness criterion;
6. reconstruct every segment using standard coefficient determination.

## Global Minimum-Snap Formulation In 1D

The previous sections say that the interior derivatives are "determined by a global smoothness criterion", without writing that criterion down. This section states it as an explicit optimization problem and shows that it reduces to **one linear system**. It is the formulation implemented by the software (milestone M7).

### Data And Unknowns

- The waypoints $x_0, x_1, \dots, x_N$ and the durations $T_0, \dots, T_{N-1}$ are given (durations are chosen as discussed in *Segment Durations In 1D*). Segment $k$ joins waypoint $k$ to waypoint $k+1$.
- The segment degree is $d = 2p - 1$, so that each node carries a shared state of $p$ components: the derivatives of order $0, 1, \dots, p-1$ (for $d = 9$: $p = 5$, from position up to snap).
- At the two global ends all derivatives of order $1, \dots, p-1$ are zero.
- The **unknowns** are the derivatives of order $1, \dots, p-1$ at the $N-1$ interior waypoints, collected in a vector $\mathbf{y}$ with $(p-1)(N-1)$ components (for $d = 9$: $4(N-1)$).

### Segment Coefficients From The Node States

Let $\tau = (t - t_k)/T_k \in [0,1]$ be the normalized time of segment $k$, and $c_{k,0}, \dots, c_{k,d}$ its coefficients. Stack the physical states of its two end nodes in the vector $\mathbf{z}_k$ of $2p$ components. Since $\mathrm{d}^i x / \mathrm{d}\tau^i = T_k^{\,i}\, x^{(i)}$, the coefficients are

$$
\mathbf{c}_k = \mathbf{B}\,\boldsymbol{\Delta}_k\,\mathbf{z}_k,
\qquad
\boldsymbol{\Delta}_k = \mathrm{diag}\left(1, T_k, \dots, T_k^{p-1},\; 1, T_k, \dots, T_k^{p-1}\right)
$$

where $\mathbf{B}$ is the inverse of the normalized-time matrix of `PolynomialCoefficientDetermination.md` (it depends only on the degree $d$, not on the data).

### The Cost

The smoothness criterion of order $m$ is

$$
J = \sum_{k=0}^{N-1} \int_0^{T_k} \left( x_k^{(m)}(t) \right)^2 \mathrm{d}t
$$

with $m = 4$ for minimum snap. Changing variable to $\tau$ (with $x^{(m)} = T_k^{-m}\, \mathrm{d}^m x / \mathrm{d}\tau^m$ and $\mathrm{d}t = T_k \,\mathrm{d}\tau$) gives

$$
J_k = T_k^{\,1-2m}\; \mathbf{c}_k^{\top} \mathbf{Q}\, \mathbf{c}_k,
\qquad
Q_{ij} = \frac{i^{\underline{m}}\; j^{\underline{m}}}{i + j - 2m + 1} \quad (i, j \ge m),
\qquad Q_{ij} = 0 \text{ otherwise}
$$

where $i^{\underline{m}} = i!/(i-m)!$ is the falling factorial. (The $m$-th derivative of $\tau^i$ is $i^{\underline{m}}\,\tau^{i-m}$, and the integral of $\tau^{i+j-2m}$ over $[0,1]$ is $1/(i+j-2m+1)$.) Combining with the previous formula:

$$
J_k = \mathbf{z}_k^{\top} \mathbf{R}_k\, \mathbf{z}_k,
\qquad
\mathbf{R}_k = T_k^{\,1-2m}\; \boldsymbol{\Delta}_k\, \mathbf{B}^{\top} \mathbf{Q}\, \mathbf{B}\, \boldsymbol{\Delta}_k
$$

$\mathbf{R}_k$ is symmetric positive semi-definite, of size $2p \times 2p$.

### Reduction To One Linear System

Every entry of $\mathbf{z}_k$ is either a known number (a waypoint position, or a zero at a global end) or a component of $\mathbf{y}$. Hence $\mathbf{z}_k = \mathbf{G}_k \mathbf{y} + \mathbf{g}_k$, where $\mathbf{G}_k$ is a 0/1 selection matrix and $\mathbf{g}_k$ holds the known entries. Then

$$
J(\mathbf{y}) = \mathbf{y}^{\top} \mathbf{H}\, \mathbf{y} + 2\, \mathbf{f}^{\top} \mathbf{y} + \text{const},
\qquad
\mathbf{H} = \sum_k \mathbf{G}_k^{\top} \mathbf{R}_k \mathbf{G}_k,
\qquad
\mathbf{f} = \sum_k \mathbf{G}_k^{\top} \mathbf{R}_k \mathbf{g}_k
$$

The exact passage through the waypoints, the continuity (shared states) and the zero end values are already built into this parametrization, so $J$ is minimized **without constraints**:

$$
\mathbf{H}\, \mathbf{y}^{\star} = -\mathbf{f}
$$

The segments are then reconstructed from the resulting node states with the formula for $\mathbf{c}_k$ above. This is the "global solve" of the Baseline Algorithm, step 6.

### Existence And Uniqueness

For $1 \le m \le p$ the matrix $\mathbf{H}$ is positive definite, so $\mathbf{y}^{\star}$ exists and is unique. Sketch: $\mathbf{y}^{\top}\mathbf{H}\,\mathbf{y}$ is the cost of the *homogeneous* problem (all positions zero, zero end values). If it vanishes, $x^{(m)} \equiv 0$ on every segment; since the derivatives up to order $p-1 \ge m-1$ are continuous at the nodes, $x$ is one polynomial of degree at most $m-1$ on the whole interval, and it has zero value and zero derivatives up to order $m-1$ at the start, so it is identically zero, that is $\mathbf{y} = \mathbf{0}$.

### Which Order $m$

| Degree | $p$ | Natural order $m = p$ | Resulting classical curve |
|---|---|---|---|
| $3$ | $2$ | acceleration | clamped cubic spline |
| $5$ | $3$ | jerk | minimum-jerk spline |
| $7$ | $4$ | snap | minimum-snap spline |
| $9$ | $5$ | crackle | minimum-crackle spline |

When $m = p$ the minimizer is the classical spline of degree $2p-1$, whose derivatives are continuous up to order $2p-2$ at the waypoints, far beyond the order $p-1$ imposed by the shared states. Numerical check (Cowork prototype, 2026-10-17): for $d = 9$ and $m = 5$ all derivative jumps at an interior waypoint vanish up to the 8th derivative.

For degree $9$ the project default is nevertheless **snap** ($m = 4$, Decision 5 of `07-implementation-plan.md`): the snap is continuous (order $p-1 = 4$) but the crackle and higher derivatives may jump at the waypoints (same numerical check: the jumps for $m = 4$ are non-zero from the 5th derivative on). The formulation treats $m$ as a parameter, so the choice is a one-line change. For degrees $3$, $5$ and $7$ the natural order $m = p$ is used.

### Properties Worth Knowing

- **Global coupling**: moving one waypoint, or changing one duration, changes all interior states. $\mathbf{H}$ is block-tridiagonal (a node is coupled only with its neighbours), so the cost of the solve grows only linearly with $N$; for the project's sizes (up to a few dozen segments) a dense solve is entirely adequate.
- **Overshoot**: minimizing a smoothness cost does not bound the position between waypoints. Kinematic limits must still be verified afterwards (recompute-and-verify of `TrajectoryConstraints.md`).
- **Conditioning**: $\mathbf{H}$ can be ill-conditioned: about $5 \cdot 10^{6}$ in the small example below, but up to $5 \cdot 10^{15}$ for degree $9$ with $m = 5$ and $30$ segments whose durations differ by a factor $100$ (measured by Code, request 0004). The cause is mostly the different physical scales of the unknowns (velocity, acceleration, jerk, snap and segments of very different durations). A symmetric diagonal scaling $\mathbf{S} = \mathrm{diag}(1/\sqrt{H_{ii}})$, solving $(\mathbf{S}\mathbf{H}\mathbf{S})\,\mathbf{z} = -\mathbf{S}\mathbf{f}$ and $\mathbf{y} = \mathbf{S}\mathbf{z}$, brought the condition number of a comparable test case from $3 \cdot 10^{15}$ down to $2 \cdot 10^{9}$ (and from $4 \cdot 10^{13}$ to $1 \cdot 10^{6}$ for $m = 4$) in Cowork's prototype. Use a Cholesky solve on the scaled matrix, never an explicit inverse. Evaluate the cost of a solution from the segment coefficients, not from the quadratic form $\mathbf{y}^{\top}\mathbf{H}\mathbf{y} + 2\mathbf{f}^{\top}\mathbf{y} + \text{const}$, whose large constant term cancels badly.

### Reference Values For Verification

Computed independently by Cowork with a small NumPy prototype (not the application code), for rest-to-rest ends, equal durations $T_k = 1$:

- **Cubic case** ($d = 3$, $m = 2$): the interior velocities obey the clamped-spline relation $v_{i-1} + 4 v_i + v_{i+1} = 3 (x_{i+1} - x_{i-1}) / h$ for equal durations $h$.
- **Waypoints** $(0, 1, 2)$, **two segments**: the interior velocity is $1.5$ ($d=3$, $m=2$), $1.875$ ($d=5$, $m=3$), $2.1875$ ($d=7$, $m=4$), $2.275$ ($d=9$, $m=4$). By symmetry the interior acceleration is $0$ in all cases and, for $d=9$, the interior snap is also $0$. For $d=9$, $m=4$ the interior jerk is $-15.4$ and the cost is $J = 4032$; for $d = 3$, $m = 2$ the cost is $J = 6$.
- **Waypoints** $(0, 1, 3, 2)$, **three segments**, $d = 9$, $m = 4$ (derivatives $v, a, j, s$):
  - node 1: $3.038429,\; 3.850718,\; -12.978544,\; -51.947904$
  - node 2: $-1.142124,\; -5.848363,\; 20.390968,\; 86.148599$
  - cost $J \approx 10977.31$.
- **Optimality**: at $\mathbf{y}^{\star}$ the gradient $\mathbf{H}\mathbf{y} + \mathbf{f}$ vanishes, and any small perturbation of $\mathbf{y}^{\star}$ increases $J$.

## Pass-Through With Kinematic Limits

Exact pass-through does not bound the velocity, acceleration, jerk or snap between the waypoints, so active limits must be handled. The project decision for ordinary segments (*Shortest Feasible Duration* in `TrajectoryConstraints.md`) is extended here to a whole waypoint sequence.

### What Is Free And What Is Fixed

The waypoint positions are fixed by the user. The smoothness order $m$ and the polynomial degree are fixed. The durations are $T_k = \lambda\,\bar T_k$, where the **reference durations** $\bar T_k$ fix the proportions (user-specified durations, or the proportional rule of `design/07-implementation-plan.md` A8 with any reference total) and the single scalar $\lambda > 0$ is the only free parameter. The shortest feasible duration of the sequence is therefore the smallest $\lambda$ for which the globally optimal trajectory respects every active limit. Choosing the proportions themselves under limits is a different, nonlinear problem and is not part of the baseline (see Open Questions).

### Scaling Law

Let $x_{\bar T}(t)$ be the optimal trajectory for the reference durations, with all prescribed boundary derivatives (orders $1,\dots,p-1$ at the two global ends) equal to zero. For durations $\lambda \bar T_k$, the trajectory

$$
x_\lambda(t) = x_{\bar T}(t/\lambda)
$$

satisfies the same waypoints and the same (zero) boundary derivatives, and its cost is

$$
J_\lambda = \sum_k \int \left(x_\lambda^{(m)}\right)^2 dt = \lambda^{\,1-2m}\, J_{\bar T}.
$$

Since $J_\lambda$ is a constant multiple of $J_{\bar T}$ for every admissible trajectory of the scaled problem, $x_\lambda$ is the optimum of the scaled problem. By uniqueness (*Existence And Uniqueness*) it is **the** solution. The derivative of order $k$ scales as

$$
x_\lambda^{(k)}(t) = \lambda^{-k}\, x_{\bar T}^{(k)}(t/\lambda),
\qquad
P_k(\lambda) = \lambda^{-k} P_k(1),
$$

where $P_k(\lambda)$ is the peak of $\lvert x^{(k)} \rvert$ over the whole trajectory. No new solve is needed when $\lambda$ changes. (Numerical check by Cowork: for the data of *Reference Values*, the peaks at $\lambda = 0.5$ and $\lambda = 2$ equal $\lambda^{-k} P_k(1)$ to nine digits.)

### Closed Form For Zero Boundary Derivatives

With limits $L_k$ on the orders $k = 1,\dots,4$ (velocity, acceleration, jerk, snap; only the active ones), the feasibility condition $P_k(1)\,\lambda^{-k} \le L_k$ gives

$$
\lambda^{\star} = \max_{k\ \text{active}} \left( \frac{P_k(1)}{L_k} \right)^{1/k},
\qquad
T_k^{\star} = \lambda^{\star}\,\bar T_k .
$$

It is the same structure as the rest-to-rest closed form for a single segment, with the peaks $P_k(1)$ of the reference solution in place of $\lvert\Delta\rvert\,c_k$. Notes:

- Orders with $P_k(1) = 0$ (for example snap for a cubic) impose nothing.
- If no limit is active, or all $P_k(1)$ are zero (all waypoints equal), the limits determine nothing: the prescribed or default durations are used.
- $\lambda^{\star}$ may be smaller than $1$: the reference durations were too slow, and the sequence is sped up.
- The peaks are found with the extrema machinery of `TrajectoryConstraints.md`; the same routine that verifies limits supplies $P_k(1)$.

### Non-Zero Boundary Derivatives

Prescribed non-zero boundary derivatives are physical values that do not scale with $\lambda$, so the scaling law is lost and $F(\lambda) = \max_k P_k(\lambda)/L_k - 1$ is not necessarily monotone. The general duration search of `TrajectoryConstraints.md` (geometric growth from a lower bound, bisection, explicit failure, the caveat on narrow windows) applies with $\lambda$ in place of $T$; each evaluation of $F$ is one global solve, which is cheap because $\mathbf{H}$ is block-tridiagonal. A reasonable start of the search is the value $\lambda^{\star}$ of the closed form computed with the boundary derivatives set to zero; the lower bound from the necessary conditions of *General Case: Duration Search* applies to the boundary derivatives of the first and last segment.

### Prescribed Durations

If the user prescribes all durations, they are kept and only verified (extrema of every segment against every active limit); a violation is reported, never repaired.

### Reference Values For The Closed Form

Computed by Cowork with the NumPy prototype of *Reference Values For Verification* (rest ends, $d = 9$, $m = 4$).

- Waypoints $(0, 1, 2)$, reference durations $(1, 1)$: peaks $P_1 = 2.275$, $P_2 = 4.094571$, $P_3 = 15.4$, $P_4 = 74.005522$. With limits $v \le 2$, $a \le 3$, $j \le 10$, $s \le 50$: the ratios $(P_k/L_k)^{1/k}$ are $1.1375,\ 1.168271,\ 1.1548,\ 1.102995$, so the acceleration limit binds, $\lambda^{\star} = 1.168271$, durations $(1.168271, 1.168271)$, total $2.336542$. At $\lambda^{\star}$ the peaks are $1.947322,\ 3.0,\ 9.658063,\ 39.727378$.
- Waypoints $(0, 1, 3, 2)$, reference durations $(0.7, 1.3, 0.9)$: peaks $P_1 = 4.47151$, $P_2 = 9.626191$, $P_3 = 33.013296$, $P_4 = 255.962162$. With limits $v \le 3$, $a \le 6$: $\lambda^{\star} = 1.490503$ (velocity binds), durations $(1.043352, 1.937654, 1.341453)$, total $4.32246$; the peaks at $\lambda^{\star}$ are $v = 3.0$, $a = 4.332999$.
- Same waypoints, durations $(1, 1, 1)$: $P_1 = 3.490773$, $P_2 = 8.369728$, $P_3 = 24.329164$, $P_4 = 129.585159$.

Peaks are maxima over fine sampling (4001 points per segment), which slightly underestimates them; the exact value of the first snap peak at $\lambda^{\star}$ is $39.727413$ (the sampled $39.727378$). Verify to about $10^{-6}$ relative.

- **Non-zero boundary velocity** (waypoints $(0, 1, 3, 2)$, reference durations $(0.7, 1.3, 0.9)$, limits $v \le 3$, $a \le 6$, initial velocity $b$, other boundary derivatives zero): $\lambda^{\star} = 1.4905$ for $b = 0$, $1.1908$ for $b = +1.5$ (along the motion), $5.9005$ for $b = -1.5$ (against it, the total time almost quadruples). Confirmed by an independent grid scan of the feasibility function (first feasible $\lambda$ at $1.195$ and $5.905$ on a step of $0.005$).

## Exact Pass-Through Versus Blending

The difference from Requirement 13 can now be stated sharply.

In ordinary blending:

- the original join point may disappear from the final trajectory;
- the transition is distributed over a local blend window.

In exact pass-through mode:

- every specified waypoint remains a mandatory interpolation point;
- no local bypass is allowed;
- continuity is achieved by shared states at the waypoint itself.

So the two methods are not small variations of the same thing. They are genuinely different trajectory-construction modes.

## Segment Durations In 1D

The durations:

$$
T_0,\; T_1,\; \dots,\; T_{N-1}
$$

remain important design parameters.

As in the previous documents:

- changing a duration changes the segment coefficients;
- changing durations also changes the globally optimal or heuristic interior derivatives;
- duration selection and exact pass-through smoothness are therefore coupled.

Possible strategies are:

- user-specified durations;
- durations derived from nominal geometric spacing;
- durations scaled by a common factor to satisfy active constraints (*Pass-Through With Kinematic Limits*).

This keeps Requirement 14 naturally connected to `TrajectoryConstraints.md`.

## Lower-Degree Versions

The same exact-pass-through logic applies to degrees lower than $9$, but with a reduced shared state.

| Degree | Shared waypoint state in the default symmetric formulation |
|---|---|
| $3$ | $x_i$, $v_i$ |
| $5$ | $x_i$, $v_i$, $a_i$ |
| $7$ | $x_i$, $v_i$, $a_i$, $j_i$ |
| $9$ | $x_i$, $v_i$, $a_i$, $j_i$, $s_i$ |

So Requirement 14 is not conceptually specific to degree $9$, but degree $9$ is the richest reference case because it supports shared continuity up to snap.

The criterion order $m$ to use for each degree is given in *Global Minimum-Snap Formulation In 1D* (table *Which Order $m$*).

## 3D Exact Pass-Through

### Exact Waypoints In 3D

Now suppose the trajectory must pass exactly through:

$$
\mathbf{p}_0,\; \mathbf{p}_1,\; \mathbf{p}_2,\; \dots,\; \mathbf{p}_N
$$

with one segment between consecutive vector waypoints.

The same conceptual structure remains:

- every waypoint is mandatory;
- interior stop events are not desired;
- continuity is enforced by shared waypoint states.

### Shared Waypoint States In 3D

For the degree-$9$ reference case, waypoint $i$ carries:

$$
\mathcal{S}_i =
\left(
\mathbf{p}_i,
\mathbf{v}_i,
\mathbf{a}_i,
\mathbf{j}_i,
\mathbf{p}^{(4)}_i
\right)
$$

The global end conditions become:

$$
\mathbf{v}_0 = \mathbf{a}_0 = \mathbf{j}_0 = \mathbf{p}^{(4)}_0 = \mathbf{0}
$$

$$
\mathbf{v}_N = \mathbf{a}_N = \mathbf{j}_N = \mathbf{p}^{(4)}_N = \mathbf{0}
$$

and the interior waypoint derivatives are shared unknown vectors.

### How The 3D Segments Are Computed

As in the previous documents, the 3D coefficient computation is still most naturally carried out component-wise.

That is:

- solve one scalar pass-through problem for the $x$ axis;
- solve one scalar pass-through problem for the $y$ axis;
- solve one scalar pass-through problem for the $z$ axis;
- keep the segment durations synchronized across the three axes.

So the correct project-level interpretation is:

- exact waypoint geometry is handled at the vector level;
- coefficient determination is reused from the 1D theory, component by component.

### Avoiding Zero Velocity In 3D

The stop-and-go issue in 3D is not only:

$$
\mathbf{v}_i = \mathbf{0}
$$

at an interior waypoint, but also the broader case in which the shared interior state is chosen so poorly that the trajectory effectively stalls or changes direction unnaturally.

So the same principle applies:

- do not force interior waypoint derivatives to zero unless the geometry or timing genuinely requires it;
- determine them globally so that exact passage and smooth through-motion coexist.

## Recommended 3D Baseline

The recommended baseline in 3D is:

- exact interpolation of all specified waypoints;
- zero derivative vectors only at the global start and end;
- shared interior vector states across neighboring segments;
- component-wise segment reconstruction once those states are known;
- one synchronized set of segment durations across all axes.

This is fully consistent with the overall project philosophy:

- start from the 1D theory;
- extend to 3D through structured vector composition.

### Common Durations And Kinematic Limits In 3D (decided 2026-10-18)

**Decoupling.** For fixed durations the smoothness cost of the 3D problem is the sum of the three scalar costs, and the waypoint conditions and the (zero or prescribed) end conditions act on each component separately. The 3D optimum is therefore the three scalar optima, computed with the **same durations** $T_k$; the durations are the only coupling.

**Reference durations.** The proportional rule of A8 uses the Euclidean chords $\Delta_k = \lVert \mathbf{p}_{k} - \mathbf{p}_{k-1} \rVert$ in place of $\lvert\Delta x\rvert$:

$$
\bar T_k = N\,\frac{\Delta_k}{\sum_j \Delta_j}\quad(\text{total } N \text{ seconds, one per segment}).
$$

Along a single axis the chords are the absolute increments and the rule reduces to the 1D rule exactly.

**Scaling law.** For zero boundary derivatives the scalar law $x_\lambda(t) = x_{\bar T}(t/\lambda)$ holds on every axis with the same $\lambda$, so the vector trajectory scales as a whole and

$$
P_k(\lambda) = \lambda^{-k}\,\max_{a\in\{x,y,z\}} P_{k,a}(1),
$$

where $P_{k,a}(1)$ is the peak of $\lvert x_a^{(k)}\rvert$ over the whole trajectory for the reference durations. Limits are applied **component-wise** (one value $L_k$ per quantity, equal for the three axes). The shortest feasible common scale is

$$
\lambda^{\star} = \max_{k\ \text{active}}\ \max_{a}\ \left(\frac{P_{k,a}(1)}{L_k}\right)^{1/k},\qquad T_k^{\star} = \lambda^{\star}\bar T_k .
$$

The pair $(k, a)$ that attains the maximum is the **binding limit and axis** (to be reported). An axis whose waypoints are all equal is constant and imposes nothing; if every axis is constant, or no limit is active, the limits determine nothing.

**Non-zero boundary derivatives.** The boundary derivatives are given as vectors. As in 1D the scaling law is lost, $F(\lambda) = \max_{k,a} P_{k,a}(\lambda)/L_k - 1$ (maximum over all axes) is searched with the general duration search (geometric growth from the zero-boundary closed form, bisection, explicit failure, caveat on non-monotone windows). Each evaluation is three scalar global solves.

**Prescribed durations.** The common durations are kept and verified on every axis; a violation names the segment, the axis and the quantity.

**Reference values** (NumPy prototype of *Reference Values For Verification*, rest ends, $d = 9$, $m = 4$, peaks on 20001 points per segment, verify to about $10^{-5}$ relative). Waypoints $(0,0,0),\ (1,0,0),\ (1,3,0),\ (1,3,4)$: chords $(1, 3, 4)$, reference durations $(0.375,\ 1.125,\ 1.5)$ (total $3$).

| Axis | $P_1$ | $P_2$ | $P_3$ | $P_4$ |
|---|---|---|---|---|
| $x$ | 9.981004 | 38.207532 | 207.293297 | 2904.831335 |
| $y$ | 4.044221 | 7.592746 | 25.276926 | 127.142593 |
| $z$ | 4.895422 | 8.795230 | 28.459199 | 123.104392 |

With limits $v \le 2$, $a \le 3$, $j \le 10$, $s \le 50$ per axis: the ratios $(P_{k,x}/L_k)^{1/k}$ of the $x$ axis are $4.990502,\ 3.568731,\ 2.747020,\ 2.760818$ and dominate the other axes, so $\lambda^{\star} = 4.990502$ (velocity on $x$ binds), durations $(1.871438,\ 5.614315,\ 7.485753)$, total $14.971506$; at $\lambda^{\star}$ the peaks over the axes are $2.0,\ 1.534124,\ 1.667833,\ 4.683214$ (below the other limits). The example shows the weakness of the chord rule: the $x$ axis makes its whole motion in the short first segment, and only the common scale can repair it.

**Consistency.** With waypoints that differ along one axis only, the 3D result equals the 1D result (durations, coefficients, $\lambda^{\star}$).

**Open point.** Choosing the proportions $\bar T_k$ so that all axes are well balanced is a nonlinear problem and is not part of the baseline (as in 1D).

## Why Requirement 14 Is More Constrained Than Requirement 13

Requirement 13 gives geometric freedom because the transition may avoid the nominal endpoint.

Requirement 14 removes that freedom because:

- the waypoint must be hit exactly;
- the same point belongs to both neighboring segments;
- smoothness must be achieved with no geometric bypass.

So Requirement 14 is intrinsically more constrained.

This is why a local blend insertion is no longer sufficient and a global waypoint-state strategy becomes necessary.

## Baseline Algorithm

The following procedure is the recommended baseline for Requirement 14.

1. Define the exact waypoint sequence.
2. Choose the polynomial degree for the segments.
3. Choose or initialize the segment durations.
4. Impose zero velocity, acceleration, jerk, and snap at the global start and end for the degree-$9$ reference case.
5. Introduce shared unknown derivative states at the interior waypoints.
6. Determine those interior states by a global rule:
   - direct prescription,
   - heuristic estimation,
   - or, preferably, a smoothness-based global solve.
7. Reconstruct each segment from its start and end waypoint states using `PolynomialCoefficientDetermination.md`.
8. Verify continuity and exact waypoint interpolation.
9. Verify active kinematic constraints and, if needed, update durations and recompute.

This algorithm applies directly in 1D and extends to 3D by vector waypoint states and component-wise segment reconstruction.

## Relation To The Data Model

`02-data-model.md` currently contains:

$$
\mathrm{Trajectory1D}\{\mathrm{Segment1D}[nn],\; \mathrm{segmentEndpointRespect}\}
$$

$$
\mathrm{Trajectory3D}\{\mathrm{Segment3D}[nn],\; \mathrm{segmentEndpointRespect}\}
$$

This document gives a precise theoretical interpretation to:

$$
\mathrm{segmentEndpointRespect}
$$

For Requirement 14, the natural interpretation is:

- `segmentEndpointRespect = yes`;
- exact passage through the specified segment endpoints is mandatory;
- ordinary blending settings are therefore not the primary mechanism.

This complements the interpretation already given in `BlendingSegments.md`:

- `segmentEndpointRespect = no` activates ordinary blending logic;
- `segmentEndpointRespect = yes` activates exact-pass-through logic.

At the state level the implementation does not store the shared interior waypoint derivatives separately: they are the unknowns of the global solve, and the waypoint states are recovered by evaluating the segments (Resolved 2026-10-18).

## Worked Symbolic Considerations

### Example 1: Three Waypoints In 1D

Suppose the trajectory must pass exactly through:

$$
x_0,\; x_1,\; x_2
$$

with two segments.

The global endpoint conditions are:

$$
v_0 = a_0 = j_0 = s_0 = 0
$$

$$
v_2 = a_2 = j_2 = s_2 = 0
$$

The interior waypoint state is:

$$
\mathcal{S}_1 = \left(x_1, v_1, a_1, j_1, s_1\right)
$$

If:

$$
v_1 \ne 0
$$

then the trajectory can pass through $x_1$ without stopping there.

### Example 2: Difference From Ordinary Blending

In `BlendingSegments.md`, two neighboring segments may be truncated before and after their nominal join, and the final blended curve may avoid the original join completely.

Here, that is not allowed.

The two neighboring segments must both interpolate the exact shared waypoint:

$$
x_i
$$

or:

$$
\mathbf{p}_i
$$

depending on dimension.

So the geometric freedom is smaller, but the waypoint fidelity is exact.

### Example 3: 3D Exact Pass-Through

Suppose the 3D trajectory must interpolate:

$$
\mathbf{p}_0,\; \mathbf{p}_1,\; \mathbf{p}_2
$$

with zero derivative vectors at the global start and end.

Then the interior shared state is:

$$
\left(
\mathbf{p}_1,
\mathbf{v}_1,
\mathbf{a}_1,
\mathbf{j}_1,
\mathbf{p}^{(4)}_1
\right)
$$

Once that interior state and the segment durations are fixed, both segments are determined by standard endpoint interpolation applied component-wise.

## Open Questions

- **Resolved**: the default exact-pass-through rule is a minimum-snap global solve (Decision 5 of `07-implementation-plan.md`), formulated in *Global Minimum-Snap Formulation In 1D*; the criterion order $m$ is a parameter, so alternatives can be exposed without restructuring.
- **Resolved (author, 2026-10-17)**: for degree $9$ the default is snap ($m = 4$: derivatives continuous up to snap, crackle may jump at the waypoints). The natural order crackle ($m = 5$, derivatives continuous up to the 8th) remains available as a parameter.
- **Resolved (2026-10-17)**: durations in exact-pass-through mode are user-specified or derived by the proportional rule; with active limits a common scale factor $\lambda$ gives the shortest feasible sequence (*Pass-Through With Kinematic Limits*).
- **Open — later, non-blocking**: allocating the durations unequally to reduce the total time under limits (a nonlinear joint search over the individual $T_k$) is not part of the baseline; is it worth the complexity?
- **Resolved (2026-10-18)**: the shared interior derivatives are the unknowns of the global solve and are not stored separately; the state at every waypoint is obtained by evaluating the segments and is reported to the user.

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `BoundaryConditions.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `BlendingSegments.md`
- `02-data-model.md`
