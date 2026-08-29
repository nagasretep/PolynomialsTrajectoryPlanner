# Blending Segments

## Intro And Scope

This document studies the smooth connection between consecutive trajectory segments when the final trajectory is **not** required to pass exactly through the nominal common endpoint of the original segments.

It covers Requirement 13 of `01-requirements.md`:

- complex trajectories are made of multiple segments;
- the connection between two consecutive segments is defined by:
  - the start of the connection inside the preceding segment;
  - the end of the connection inside the following segment;
- those two points may be specified in absolute form or by percentages;
- the treatment starts in 1D and then extends to 3D.

This document therefore addresses the classical "blend" situation:

- the original neighboring segments exist as nominal reference pieces;
- a local neighborhood around their nominal join is removed;
- a dedicated connecting segment is inserted instead;
- the final blended trajectory no longer needs to pass through the original common endpoint.

This topic is distinct from:

- `TrajectoryConstraints.md`, which studies limits inside a single segment;
- `ConstantVelocitySegment.md`, which studies a special subsegment with imposed constant speed;
- `TrajectoryPassingThroughConstraintPoints`, which will study the opposite case in which exact passage through the transition point must be preserved.

The discussion remains symbolic-first and theoretical, consistent with `03-assumptions.md`.

## Symbols Used

The notation follows `SymbolsAndNomenclature.md`. The main symbols used here are:

- $x_k(t)$: preceding scalar segment in 1D;
- $x_{k+1}(t)$: following scalar segment in 1D;
- $\mathbf{p}_k(t)$: preceding vector segment in 3D;
- $\mathbf{p}_{k+1}(t)$: following vector segment in 3D;
- $x_J$: nominal common endpoint of two consecutive 1D segments before blending;
- $\mathbf{p}_J$: nominal common endpoint of two consecutive 3D segments before blending;
- $x_{b,k}^{start}$: point where blending starts inside segment $k$;
- $x_{b,k+1}^{end}$: point where blending ends inside segment $k+1$;
- $\mathbf{p}_{b,k}^{start}$: point where blending starts inside segment $k$ in 3D;
- $\mathbf{p}_{b,k+1}^{end}$: point where blending ends inside segment $k+1$ in 3D;
- $t_{b,k}^{start}$: parameter value on segment $k$ corresponding to the chosen blend-start point;
- $t_{b,k+1}^{end}$: parameter value on segment $k+1$ corresponding to the chosen blend-end point;
- $T_b$: duration of the blend segment;
- $\mathcal{S}_{b}^{-}$: state sampled from the preceding segment at the blend-start point;
- $\mathcal{S}_{b}^{+}$: state sampled from the following segment at the blend-end point.

When degree $9$ is the main reference case, the two sampled 1D states are:

$$
\mathcal{S}_{b}^{-} = \left(x, v, a, j, s\right)_{b}^{-}, \qquad
\mathcal{S}_{b}^{+} = \left(x, v, a, j, s\right)_{b}^{+}
$$

where each component is obtained by evaluating the original neighboring segments and their derivatives at the selected blend points.

## Why Blending Is Needed

A multi-segment trajectory defined only by nominal segment endpoints may suffer from one or more of the following issues at a join:

- a visible corner in the path;
- discontinuity in velocity;
- discontinuity in acceleration or higher derivatives;
- an overly abrupt change in kinematic demand.

Blending replaces the direct transition with a dedicated local connection that is smoother.

The key idea is:

- do not connect segment $k$ directly to segment $k+1$ at the nominal join;
- truncate segment $k$ before the join;
- truncate segment $k+1$ after the join;
- insert a new segment between the two truncation points.

So the final trajectory around the join becomes:

$$
\{\text{truncated segment }k,\; \text{blend segment},\; \text{truncated segment }k+1\}
$$

## The Meaning Of "Without A Pass-Through Constraint"

Requirement 13 is fundamentally different from Requirement 14.

In the present document:

- the nominal join point is only a reference inherited from the original segmentation;
- the final blended trajectory is **not** forced to pass through that join;
- the true transition is spread over an interval.

This means that the original common endpoint loses its role as a mandatory interpolation point.

That relaxation is exactly what gives blending its geometric and kinematic flexibility.

## Local Blend Window

Consider two nominal neighboring segments:

$$
x_k(t), \qquad x_{k+1}(t)
$$

in 1D, or:

$$
\mathbf{p}_k(t), \qquad \mathbf{p}_{k+1}(t)
$$

in 3D.

The blend is defined by two local points:

- one point inside the preceding segment;
- one point inside the following segment.

Those two points delimit the blend window.

In 1D:

$$
x_{b,k}^{start}, \qquad x_{b,k+1}^{end}
$$

In 3D:

$$
\mathbf{p}_{b,k}^{start}, \qquad \mathbf{p}_{b,k+1}^{end}
$$

The blend segment must connect the two sampled states attached to those points.

## 1D Blending Problem

### Nominal Consecutive Segments

In 1D, let:

$$
x_k(t), \qquad x_{k+1}(t)
$$

be two consecutive nominal segments, with common nominal join value:

$$
x_J
$$

before blending.

Choose:

$$
x_{b,k}^{start}
$$

inside segment $k$, and:

$$
x_{b,k+1}^{end}
$$

inside segment $k+1$.

These two values identify the portion to be replaced.

## Recovering The Corresponding Parameters

The selected blending points may be given as positions, but the original segments are parameterized by time or by a local normalized parameter. Therefore the first practical step is to recover:

$$
t_{b,k}^{start}, \qquad t_{b,k+1}^{end}
$$

such that:

$$
x_k\!\left(t_{b,k}^{start}\right) = x_{b,k}^{start}
$$

$$
x_{k+1}\!\left(t_{b,k+1}^{end}\right) = x_{b,k+1}^{end}
$$

If the original segments are monotonic over the relevant local portions, this recovery is conceptually simple.

If they are not monotonic, the same position value may correspond to multiple parameter values. In that case the intended branch must be selected explicitly.

This is one reason why percentage-based selection can be operationally simpler than absolute-position selection.

## Sampling The Boundary States For The Blend

Once the two parameter values are known, the blending states are obtained directly from the original segments.

For the preceding segment:

$$
\mathcal{S}_{b}^{-} =
\left(
x_k\!\left(t_{b,k}^{start}\right),
\dot{x}_k\!\left(t_{b,k}^{start}\right),
\ddot{x}_k\!\left(t_{b,k}^{start}\right),
x_k^{(3)}\!\left(t_{b,k}^{start}\right),
x_k^{(4)}\!\left(t_{b,k}^{start}\right)
\right)
$$

For the following segment:

$$
\mathcal{S}_{b}^{+} =
\left(
x_{k+1}\!\left(t_{b,k+1}^{end}\right),
\dot{x}_{k+1}\!\left(t_{b,k+1}^{end}\right),
\ddot{x}_{k+1}\!\left(t_{b,k+1}^{end}\right),
x_{k+1}^{(3)}\!\left(t_{b,k+1}^{end}\right),
x_{k+1}^{(4)}\!\left(t_{b,k+1}^{end}\right)
\right)
$$

for the degree-$9$ reference case.

These are not guessed or imposed independently. They are inherited from the neighboring segments.

## Building The Blend Segment In 1D

The blend segment is a new local polynomial:

$$
x_b(t)
$$

with duration:

$$
T_b
$$

chosen for the blend interval.

In the degree-$9$ reference case, the blend segment is naturally determined by enforcing:

- position, velocity, acceleration, jerk, and snap at its start;
- position, velocity, acceleration, jerk, and snap at its end.

So the boundary data for the blend are exactly:

$$
\mathcal{S}_{b}^{-}, \qquad \mathcal{S}_{b}^{+}
$$

and the coefficient-determination problem is the same one already studied in `PolynomialCoefficientDetermination.md`.

This gives a very clean theoretical structure:

1. sample the two states from the original segments;
2. choose a blend duration $T_b$;
3. solve one standard degree-$9$ endpoint-interpolation problem.

## Why Degree 9 Is A Natural Blend Reference

Degree $9$ is especially convenient for blending because it can match:

- position;
- velocity;
- acceleration;
- jerk;
- snap;

at both ends of the blend.

This means that, in the reference case, the transition from the truncated original segment to the blend, and then from the blend to the following truncated segment, is smooth up to the fourth derivative.

That is a strong continuity level for a theoretical baseline.

## Lower-Degree Analogues

The same blending logic applies to lower polynomial degrees, but with a reduced matched state.

| Blend degree | Matched quantities at each end |
|---|---|
| $3$ | $x$, $v$ |
| $5$ | $x$, $v$, $a$ |
| $7$ | $x$, $v$, $a$, $j$ |
| $9$ | $x$, $v$, $a$, $j$, $s$ |

So the general rule is:

- choose the blend degree;
- sample from the neighboring segments only the derivative orders that that degree can support at both ends;
- determine the blend coefficients accordingly.

## Choosing The Blend Duration

The blend duration $T_b$ is an actual design variable.

Several strategies are possible:

- inherit a duration from the removed original portions;
- specify $T_b$ directly;
- search for $T_b$ so that kinematic limits are respected.

The most important theoretical point is that changing $T_b$ changes the blend coefficients.

So, exactly as in `TrajectoryConstraints.md`, duration and profile shape are coupled through coefficient recomputation.

## Recommended 1D Baseline

The recommended baseline for this project is:

1. select the two blending points;
2. recover the corresponding local parameters on the neighboring segments;
3. sample the endpoint states from those segments;
4. choose a blend duration;
5. compute the blend coefficients in normalized time;
6. verify the resulting blend against any active constraints;
7. if needed, adjust $T_b$ and recompute.

This keeps the whole procedure aligned with the rest of the theory library.

## Absolute Vs Percentage Specification In 1D

Requirement 13 allows the blend points to be given either in absolute form or by percentages.

### Absolute Specification

The user gives:

$$
x_{b,k}^{start}, \qquad x_{b,k+1}^{end}
$$

directly.

This is intuitive when the scalar positions themselves are meaningful and the corresponding branches on the original segments are unambiguous.

### Percentage Specification

A convenient local parameter-based form is:

$$
\lambda_k^{start}, \qquad \lambda_{k+1}^{end} \in [0,1]
$$

with:

$$
\tau_{b,k}^{start} = \lambda_k^{start}
$$

$$
\tau_{b,k+1}^{end} = \lambda_{k+1}^{end}
$$

where each $\tau$ is the local normalized coordinate of its own segment.

This is often operationally cleaner than absolute-position selection because:

- no inverse position lookup is needed;
- the choice is unambiguous even when a segment is not monotonic in position.

For this reason, local percentage selection is a particularly natural baseline for blending.

## 1D Blend Does Not Need To Pass Through The Nominal Join

Once the blend segment is built, the final trajectory around the connection is:

$$
\{
x_k \text{ up to } t_{b,k}^{start},
\;
x_b,
\;
x_{k+1} \text{ from } t_{b,k+1}^{end}
\}
$$

The nominal common point between the original segments is no longer enforced.

This is exactly the behavior intended in Requirement 13.

## 3D Blending Problem

### Geometric Meaning In 3D

In 3D, blending is first a geometric operation and only then a coefficient-determination problem.

Let:

$$
\mathbf{p}_k(t), \qquad \mathbf{p}_{k+1}(t)
$$

be two nominal consecutive 3D segments.

Choose:

$$
\mathbf{p}_{b,k}^{start}, \qquad \mathbf{p}_{b,k+1}^{end}
$$

as the two vector points delimiting the blend window.

These are points on the original neighboring trajectories, not arbitrary free-space points detached from them.

## Recovering The Corresponding Parameters In 3D

The practical goal is again to recover:

$$
t_{b,k}^{start}, \qquad t_{b,k+1}^{end}
$$

such that:

$$
\mathbf{p}_k\!\left(t_{b,k}^{start}\right) = \mathbf{p}_{b,k}^{start}
$$

$$
\mathbf{p}_{k+1}\!\left(t_{b,k+1}^{end}\right) = \mathbf{p}_{b,k+1}^{end}
$$

If the blend points are specified through local percentages, these parameters are obtained directly.

If they are specified as absolute 3D points, one must identify the corresponding parameter values on the original segments. This is straightforward only when the intended point-to-parameter correspondence is unique.

So, just as in 1D, percentage-based selection is often cleaner operationally.

## Sampling The Boundary States In 3D

Once the two parameter values are known, the vector states are sampled from the original segments.

For the preceding segment:

$$
\mathcal{S}_{b}^{-} =
\left(
\mathbf{p}_k,
\dot{\mathbf{p}}_k,
\ddot{\mathbf{p}}_k,
\mathbf{p}_k^{(3)},
\mathbf{p}_k^{(4)}
\right)_{t=t_{b,k}^{start}}
$$

For the following segment:

$$
\mathcal{S}_{b}^{+} =
\left(
\mathbf{p}_{k+1},
\dot{\mathbf{p}}_{k+1},
\ddot{\mathbf{p}}_{k+1},
\mathbf{p}_{k+1}^{(3)},
\mathbf{p}_{k+1}^{(4)}
\right)_{t=t_{b,k+1}^{end}}
$$

in the degree-$9$ reference case.

These vector states define the blend boundary conditions.

## How The 3D Blend Is Actually Computed

Even though the blend is defined geometrically in 3D, the coefficient calculation is still most naturally performed component-wise.

That is:

- one scalar blend is computed for the $x$ component;
- one scalar blend is computed for the $y$ component;
- one scalar blend is computed for the $z$ component;
- all three share the same blend duration $T_b$.

So the recommended interpretation is:

- **geometric selection** of the blend window in 3D;
- **component-wise coefficient determination** once the endpoint vector states are known.

This is consistent with the overall project architecture:

- geometry is handled at the vector level;
- polynomial determination is reused from the scalar theory.

## Why The 3D Blend Is Still Different From Purely Independent Axis Work

Although the actual coefficient determination is component-wise, the 3D blending problem is not just "three unrelated 1D blends".

The coupling comes from:

- the shared geometric meaning of the chosen blend points;
- the shared blend duration;
- the fact that the same truncation event affects all axes simultaneously.

So the correct project-level picture is:

- geometric synchronization in 3D;
- scalar coefficient solving underneath.

## Straight-Line Shortcut Vs Genuine 3D Blend

A tempting simplification in 3D would be to connect:

$$
\mathbf{p}_{b,k}^{start}
\quad \text{to} \quad
\mathbf{p}_{b,k+1}^{end}
$$

with a straight line.

That is not the general blending strategy adopted here.

The reason is that Requirement 13 is about smooth connection between neighboring segments, not about replacing the local transition with an arbitrary chord disconnected from the inherited kinematic state.

Therefore the baseline strategy is:

- sample the full states from the original segments;
- determine a dedicated blend segment consistent with those states.

This preserves the local motion information carried by the nominal segments.

## Absolute Vs Percentage Specification In 3D

### Absolute Specification

The user gives:

$$
\mathbf{p}_{b,k}^{start}, \qquad \mathbf{p}_{b,k+1}^{end}
$$

directly.

This is meaningful when the intended points on the original segments are already known unambiguously.

### Percentage Specification

A particularly practical choice is local parameter percentage:

$$
\lambda_k^{start}, \qquad \lambda_{k+1}^{end} \in [0,1]
$$

applied to the normalized parameter of each segment.

Then:

$$
\tau_{b,k}^{start} = \lambda_k^{start}, \qquad
\tau_{b,k+1}^{end} = \lambda_{k+1}^{end}
$$

and the corresponding points are obtained by direct evaluation of the original segments.

This avoids point-to-parameter inversion and is therefore a strong practical baseline for 3D as well.

## Relation Between Blending And Constraints

A blend is primarily a continuity device, but it may also have to satisfy active constraints.

Once a blend segment has been determined, one may still need to verify:

- velocity limits;
- acceleration limits;
- jerk limits;
- other project-specific restrictions.

So blending and constraints are not separate universes:

- blending decides how the local connection is shaped;
- constraints decide whether that shape is admissible.

If the blend violates active limits, one baseline remedy is to modify the blend duration $T_b$ and recompute.

## Baseline Blending Algorithm

The following procedure is the recommended baseline for Requirement 13.

1. Start from two nominal consecutive segments.
2. Select the blend-start point inside the preceding segment.
3. Select the blend-end point inside the following segment.
4. Recover the corresponding local parameters.
5. Evaluate the neighboring segments and their derivatives at those parameters.
6. Form the two endpoint states of the blend.
7. Choose a blend degree and a blend duration.
8. Determine the blend coefficients using `PolynomialCoefficientDetermination.md`.
9. Replace the removed local portions of the original segments with the new blend segment.
10. Verify the resulting blend against active constraints and adjust duration if needed.

This procedure applies directly in 1D and extends to 3D by vector-state sampling plus component-wise coefficient determination.

## Relation To The Data Model

`02-data-model.md` defines:

Blend =

- `nextSegmentBlendingPos1D`,
- `previousSegmentBlendingPos1D`,
- `nextSegmentBlendingPos`,
- `previousSegmentBlendingPos`

This document gives the theoretical meaning of those fields:

- `nextSegmentBlendingPos1D` identifies where blending toward the next segment starts in 1D;
- `previousSegmentBlendingPos1D` identifies where blending from the previous segment ends in 1D;
- `nextSegmentBlendingPos` identifies where blending toward the next segment starts in 3D;
- `previousSegmentBlendingPos` identifies where blending from the previous segment ends in 3D.

At a higher level, `Trajectory1D` and `Trajectory3D` also contain:

$$
\mathrm{segmentEndpointRespect}
$$

whose meaning becomes especially relevant here.

For Requirement 13, the natural interpretation is:

- if exact respect of nominal segment endpoints is required, then ordinary blending is not the right mechanism;
- if exact endpoint passage is **not** required, then the blend settings become active and meaningful.

So this document is the theoretical counterpart of the branch:

- `segmentEndpointRespect = no` for ordinary blending;
- `segmentEndpointRespect = yes` for the later exact-pass-through topic of Requirement 14.

## Worked Symbolic Considerations

### Example 1: 1D Blend Around A Nominal Join

Suppose two nominal scalar segments meet at a nominal point $x_J$.

Choose:

$$
x_{b,k}^{start}
$$

before the nominal join, and:

$$
x_{b,k+1}^{end}
$$

after the nominal join.

The blend segment is then computed from the states sampled at those two points, not from the nominal join itself.

So the final trajectory is smoother, but it no longer has to interpolate $x_J$.

### Example 2: Degree-9 Blend

In the degree-$9$ case, the sampled start and end states are:

$$
\left(x, v, a, j, s\right)_{b}^{-}
\quad \text{and} \quad
\left(x, v, a, j, s\right)_{b}^{+}
$$

These ten scalar values are exactly the right number to determine one degree-$9$ blend segment uniquely once $T_b$ is fixed.

### Example 3: 3D Blend

Suppose two 3D segments are truncated at:

$$
\mathbf{p}_{b,k}^{start}, \qquad \mathbf{p}_{b,k+1}^{end}
$$

The blend boundary data are the two vector states sampled there.

The blend is then computed by solving:

- one scalar polynomial problem for $x$;
- one scalar polynomial problem for $y$;
- one scalar polynomial problem for $z$;

with one shared duration $T_b$.

This is the cleanest way to combine 3D geometry with scalar polynomial theory.

## Open Questions

- **Open — to resolve during later software design**: should the blend duration $T_b$ be user-specified, inherited automatically from the removed original portions, or optimized under active constraints?
- **Open — to resolve during data-format design**: should blend points be stored primarily as absolute points, as local percentages, or as both representations?
- **Open — to resolve together with `TrajectoryPassingThroughConstraintPoints`**: should the application expose Requirement 13 and Requirement 14 as two clearly separated modes, or as one continuity tool with a mandatory pass-through toggle?

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `ConstantVelocitySegment.md`
- `TrajectoryPassingThroughConstraintPoints`
- `02-data-model.md`
