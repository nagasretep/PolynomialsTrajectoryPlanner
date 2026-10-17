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
- durations refined later to satisfy active constraints.

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

At the state level, the current data model does not yet explicitly store the shared interior waypoint derivatives needed by this document. So one future refinement will likely need a more explicit representation of:

- waypoint state variables;
- segment durations;
- global continuity mode.

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

- **Open — to resolve during later software design**: should the project adopt one explicit global smoothness criterion as the default exact-pass-through rule, such as a minimum-snap cost, or should multiple selectable criteria be exposed?
- **Open — to resolve together with duration handling**: should segment durations in exact-pass-through mode be specified directly by the user, derived automatically from waypoint spacing, or refined through a joint duration-and-state search?
- **Open — to resolve during data-structure design**: should shared interior waypoint derivatives be represented explicitly as trajectory-level state objects, or should they remain implicit variables reconstructed only during computation?

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `BoundaryConditions.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `BlendingSegments.md`
- `02-data-model.md`
