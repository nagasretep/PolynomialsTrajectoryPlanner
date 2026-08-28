# Constant Velocity Segment

## Intro And Scope

This document studies how to introduce a constant-velocity segment inside a trajectory originally described through a ninth-degree polynomial.

It covers Requirement 12 of `01-requirements.md`:

- a constant-speed segment is required inside a path originally defined as a degree-9 polynomial;
- the segment is specified by its start and end positions, either in absolute form or as percentages;
- the treatment starts in 1D and then extends to 3D.

This document builds on:

- `PolynomialCoefficientDetermination.md`, for segment recomputation from boundary data;
- `TrajectoryConstraints.md`, for the interpretation of velocity limits and duration search;
- `02-data-model.md`, for the `Constraints` fields that already reserve `constantVelocitySelection`, `constantVelocityValue`, and the start/end positions of the constant-velocity part.

The discussion remains symbolic-first and theoretical, consistent with `03-assumptions.md`.

## Symbols Used

The notation follows `SymbolsAndNomenclature.md`. The main symbols used here are:

- $t$: absolute time;
- $t_0$: initial time;
- $t_f$: final time;
- $T = t_f - t_0$: duration of a generic segment;
- $\tau = \frac{t - t_0}{T}$: normalized time;
- $x(t)$: scalar position law in 1D;
- $\mathbf{p}(t)$: position vector in 3D;
- $v(t) = \dot{x}(t)$: scalar velocity in 1D;
- $\mathbf{v}(t) = \dot{\mathbf{p}}(t)$: velocity vector in 3D;
- $v_{cv}$: requested constant speed magnitude inside the constant-velocity segment;
- $x_{cv}^{start}$, $x_{cv}^{end}$: start and end positions of the constant-velocity subsegment in 1D;
- $\mathbf{p}_{cv}^{start}$, $\mathbf{p}_{cv}^{end}$: start and end positions of the constant-velocity subsegment in 3D;
- $t_{cv}^{start}$, $t_{cv}^{end}$: start and end times of the constant-velocity subsegment;
- $T_{cv} = t_{cv}^{end} - t_{cv}^{start}$: duration of the constant-velocity subsegment.

When the constant-velocity segment is embedded between transition segments, the endpoint states of that middle segment are written as:

$$
\mathcal{S}_{cv}^{start}, \qquad \mathcal{S}_{cv}^{end}
$$

when compact notation is useful.

## Path Vs Trajectory: Why This Topic Is Special

This requirement is easier to understand if the distinction from `04-future-topics.md` is made explicit:

- a **path** is the geometric locus to be followed;
- a **trajectory** is that path plus a time law.

A constant-velocity segment changes the time law in a very specific way:

- in 1D, it imposes constant signed velocity or constant speed magnitude over a selected interval;
- in 3D, it raises a choice between constant component-wise velocity and constant speed along the geometric path.

So the central question is not only "how do we keep velocity constant?", but also:

- what exactly must remain unchanged from the original polynomial path;
- what is allowed to be reformulated;
- what continuity level is expected at the boundaries of the constant-velocity part.

## First Key Fact: Exact Constant Velocity Is Very Restrictive

Suppose a single polynomial segment contains a nontrivial interval on which velocity is exactly constant.

In 1D, that means:

$$
\dot{x}(t) = v_{cv}
$$

for every $t$ in some interval of nonzero length.

Equivalently:

$$
\ddot{x}(t) = 0
$$

throughout that interval.

But $\ddot{x}(t)$ is itself a polynomial. A polynomial that is zero on a nontrivial interval is zero everywhere. Therefore:

$$
\ddot{x}(t) \equiv 0
$$

for the whole segment, and the segment must be globally affine:

$$
x(t) = c_0 + c_1 t
$$

So, in general:

- an arbitrary ninth-degree polynomial cannot contain an exact constant-velocity subinterval while remaining the same single nontrivial ninth-degree polynomial;
- an exact constant-velocity part therefore requires a **reformulation** of the trajectory into multiple segments, unless the whole original segment is already linear.

This fact is foundational for the rest of the document.

## 1D Case: Exact Constant Velocity Segment

## Geometric Interpretation

In 1D, the path is simply a line on the scalar axis. So "constant speed along the path" and "constant velocity magnitude" are almost the same notion, apart from sign.

If the motion direction over the selected interval is fixed, then the exact constant-velocity segment is:

$$
x_{cv}(t) = x_{cv}^{start} + \sigma v_{cv}(t - t_{cv}^{start})
$$

where:

- $\sigma \in \{+1,-1\}$ encodes the travel direction;
- $v_{cv} > 0$ is the requested speed magnitude.

The corresponding derivatives are:

$$
\dot{x}_{cv}(t) = \sigma v_{cv}, \qquad
\ddot{x}_{cv}(t) = 0, \qquad
x_{cv}^{(3)}(t) = 0, \qquad
x_{cv}^{(4)}(t) = 0
$$

for the whole constant-velocity interval.

## Duration Of The Constant-Velocity Part

Once the selected positions are known, the duration of the middle constant-velocity part is:

$$
T_{cv} = \frac{|x_{cv}^{end} - x_{cv}^{start}|}{v_{cv}}
$$

provided:

$$
x_{cv}^{end} \ne x_{cv}^{start}, \qquad v_{cv} > 0
$$

If the two positions coincide, the constant-velocity subsegment degenerates and the requirement loses meaning.

## Why A Multi-Segment Reformulation Is Needed

If the original trajectory outside the selected interval is a degree-9 polynomial, then an exact constant-velocity part is normally introduced as a three-part structure:

1. entry transition segment;
2. constant-velocity middle segment;
3. exit transition segment.

In symbolic form:

$$
\mathcal{T} =
\{
\text{entry transition},
\text{constant-velocity segment},
\text{exit transition}
\}
$$

The role of the two transition segments is to connect the surrounding motion to a middle segment whose higher derivatives are all zero.

## Entry And Exit States In 1D

The middle segment has the natural endpoint states:

$$
\mathcal{S}_{cv}^{start} =
\left(
x_{cv}^{start},
\sigma v_{cv},
0,
0,
0
\right)
$$

$$
\mathcal{S}_{cv}^{end} =
\left(
x_{cv}^{end},
\sigma v_{cv},
0,
0,
0
\right)
$$

These states make the nature of the middle segment explicit:

- prescribed position at both ends;
- same velocity at both ends;
- zero acceleration, jerk, and snap at both ends.

The entry transition must end at $\mathcal{S}_{cv}^{start}$.

The exit transition must begin at $\mathcal{S}_{cv}^{end}$.

## Baseline 1D Construction Strategy

The cleanest exact strategy in the current project is:

1. choose the two positions that delimit the constant-velocity interval;
2. define the target speed magnitude $v_{cv}$;
3. compute the constant-velocity duration $T_{cv}$;
4. create a middle affine segment with velocity $\sigma v_{cv}$;
5. construct an entry degree-9 transition that reaches $\mathcal{S}_{cv}^{start}$;
6. construct an exit degree-9 transition that leaves $\mathcal{S}_{cv}^{end}$ toward the later trajectory conditions;
7. determine the polynomial coefficients of the transition segments using `PolynomialCoefficientDetermination.md`.

This is the main exact 1D strategy recommended by this document.

## How The Start And End Positions May Be Specified In 1D

The requirement allows the interval to be specified either in absolute form or by percentages.

### Absolute Specification

The user gives:

$$
x_{cv}^{start}, \qquad x_{cv}^{end}
$$

directly.

This is the clearest formulation whenever the scalar coordinate range is already meaningful.

### Percentage Specification

A simple percentage-based choice is:

$$
\lambda_{start}, \lambda_{end} \in [0,1]
$$

with:

$$
x_{cv}^{start} = x_0 + \lambda_{start}(x_f - x_0)
$$

$$
x_{cv}^{end} = x_0 + \lambda_{end}(x_f - x_0)
$$

This is convenient, but it should be interpreted carefully:

- it is natural when the 1D motion is monotonic;
- it becomes less meaningful if the original polynomial overshoots or reverses direction.

So, in 1D, percentage specification is best treated as a convenience rule for monotonic path portions.

## 1D Consistency Checks

Before building the constant-velocity part, the following conditions should be checked.

1. The selected positions must lie on the intended path portion.
2. The ordering of the positions must be compatible with the chosen travel direction.
3. The requested speed must be positive:

$$
v_{cv} > 0
$$

4. The surrounding transition segments must have enough room to connect smoothly to zero higher derivatives at the boundaries of the constant-velocity segment.

The fourth point is especially important: a constant-velocity middle segment is easy to define, but smooth entry and exit may force the surrounding intervals to be long enough.

## A Simpler But Less Smooth 1D Alternative

There is also a simpler formulation:

- keep only one polynomial segment before the constant part;
- insert the affine constant-velocity part;
- switch directly to the following part.

This is easy to define, but generally poor in smoothness because direct switching may produce discontinuities in acceleration, jerk, or snap.

So, unless low continuity is explicitly acceptable, the project baseline should prefer transition segments that match zero acceleration, jerk, and snap at the entry and exit of the constant-velocity part.

## 3D Case: Two Different Meanings Of "Constant Velocity"

In 3D, the topic becomes richer because two meanings must be separated.

### Constant Velocity Vector

One may require:

$$
\mathbf{v}(t) = \mathbf{v}_{cv}
$$

constant over the selected interval.

Then the position law is:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{cv}^{start} +
\mathbf{v}_{cv}(t - t_{cv}^{start})
$$

This implies:

- straight-line motion along the chord joining the two selected points;
- constant speed magnitude;
- zero acceleration, jerk, and snap.

But this does **not** generally preserve the original curved polynomial path between the two selected positions.

### Constant Speed Along The Original 3D Path

Alternatively, one may require that the motion follows the same geometric curve portion while travelling with constant speed magnitude:

$$
\|\mathbf{v}(t)\| = v_{cv}
$$

This preserves the path geometry, but the component velocities are no longer constant in general.

These two meanings coincide only in the special case where the selected path portion is already a straight line.

## Recommended 3D Interpretation For This Requirement

For this specific requirement, the recommended primary interpretation is:

- preserve the selected geometric subpath of the original polynomial curve;
- impose constant **speed magnitude** along that preserved subpath.

This recommendation is appropriate because the requirement explicitly speaks about a segment "within a path" and locates it through start/end positions on that path.

So, unlike the component-wise baseline adopted in `TrajectoryConstraints.md` for general constraint analysis, this document is primarily path-oriented:

- for general limits, component-wise treatment remains the default baseline;
- for this specific requirement, the natural meaning is constant speed along the selected path portion.

## 3D Path-Preserving Strategy: Arc-Length Reparameterization

Let the original 3D polynomial path be:

$$
\mathbf{p}_{orig}(t)
$$

and let the selected subpath correspond to original parameter values:

$$
t_s^{orig}, \qquad t_e^{orig}
$$

with:

$$
\mathbf{p}_{cv}^{start} = \mathbf{p}_{orig}(t_s^{orig}), \qquad
\mathbf{p}_{cv}^{end} = \mathbf{p}_{orig}(t_e^{orig})
$$

Define the arc-length function from the start of the selected interval:

$$
\ell(t) = \int_{t_s^{orig}}^{t} \left\| \dot{\mathbf{p}}_{orig}(u) \right\|\,du
$$

Then the total length of the selected subpath is:

$$
L_{cv} = \ell(t_e^{orig})
$$

If the requested constant speed magnitude is $v_{cv}$, the duration of the constant-speed traversal becomes:

$$
T_{cv} = \frac{L_{cv}}{v_{cv}}
$$

Now define a new local time variable over the constant-speed part:

$$
\sigma = \frac{t - t_{cv}^{start}}{T_{cv}} \in [0,1]
$$

and the corresponding traveled arc length:

$$
s(\sigma) = \sigma L_{cv}
$$

The path-preserving constant-speed motion is then obtained by:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{orig}\!\left(\ell^{-1}(s(\sigma))\right)
$$

This construction preserves the original curve and enforces constant speed magnitude along it.

## Important Consequence In 3D

The arc-length-reparameterized constant-speed segment is generally **not polynomial in time anymore**.

This matters because it means:

- the original geometric path can be preserved;
- exact constant speed can be preserved;
- but the resulting time law is no longer, in general, a degree-9 polynomial in the new time variable.

So the 3D problem naturally separates into two layers:

- geometric path preservation;
- time-law reformulation.

This is not a defect. It is the mathematically natural outcome of the requirement.

## 3D Straight-Chord Alternative

If path preservation is not required, then the simpler exact construction is a straight-line constant-velocity segment:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{cv}^{start} +
\frac{t - t_{cv}^{start}}{T_{cv}}
\left(
\mathbf{p}_{cv}^{end} - \mathbf{p}_{cv}^{start}
\right)
$$

with:

$$
T_{cv} = \frac{\left\|\mathbf{p}_{cv}^{end} - \mathbf{p}_{cv}^{start}\right\|}{v_{cv}}
$$

This gives:

- constant velocity vector;
- constant speed magnitude;
- straight motion between the selected points.

Its advantage is simplicity.

Its limitation is that the curved portion of the original polynomial path is replaced by its chord.

## How The Start And End Positions May Be Specified In 3D

### Absolute Specification

The user gives:

$$
\mathbf{p}_{cv}^{start}, \qquad \mathbf{p}_{cv}^{end}
$$

directly.

This is simple only if those points are already known on the original path.

### Percentage Specification Along The Path

For path-oriented use, the most meaningful percentage convention is based on the path progression parameter over the selected original interval.

One convenient choice is to define:

$$
\lambda_{start}, \lambda_{end} \in [0,1]
$$

on the original segment parameter interval:

$$
t_s^{orig} = t_0 + \lambda_{start}(t_f - t_0)
$$

$$
t_e^{orig} = t_0 + \lambda_{end}(t_f - t_0)
$$

and then:

$$
\mathbf{p}_{cv}^{start} = \mathbf{p}_{orig}(t_s^{orig}), \qquad
\mathbf{p}_{cv}^{end} = \mathbf{p}_{orig}(t_e^{orig})
$$

This is parameter-percentage, not true arc-length percentage.

If a geometric percentage of traveled distance is preferred, then the percentage should be defined on arc length instead:

$$
\ell(t_s^{orig}) = \lambda_{start} L_{tot}, \qquad
\ell(t_e^{orig}) = \lambda_{end} L_{tot}
$$

where $L_{tot}$ is the total arc length of the original segment.

For this project, the cleanest recommendation is:

- in 1D, percentage by scalar position interval is acceptable on monotonic portions;
- in 3D, percentage by original parameter is the simplest practical convention;
- if precise geometric meaning is important in 3D, arc-length percentage is the more rigorous convention.

## Transition Segments Around The Constant-Speed Part

Whether the constant-speed segment is 1D or 3D, smooth entry and exit normally require dedicated transition segments.

### 1D Transition Targets

In 1D, the target state at the start of the constant-speed interval is:

$$
\left(
x_{cv}^{start},
\sigma v_{cv},
0,
0,
0
\right)
$$

and at the end:

$$
\left(
x_{cv}^{end},
\sigma v_{cv},
0,
0,
0
\right)
$$

### 3D Transition Targets

For the straight-chord constant-velocity-vector option:

$$
\mathbf{v}_{cv} =
\frac{\mathbf{p}_{cv}^{end} - \mathbf{p}_{cv}^{start}}{T_{cv}}
$$

and the transitions should match:

- position;
- velocity vector $\mathbf{v}_{cv}$;
- zero acceleration;
- zero jerk;
- zero snap.

For the arc-length-reparameterized path-preserving option, the velocity direction varies along the segment, so transition design is less trivial. In that case, the main requirement is smooth matching between:

- the preceding motion state;
- the reparameterized curve state at the beginning of the constant-speed interval;
- the corresponding state at the end.

This may lead to dedicated blending/reparameterization design choices in later documents.

## Baseline Algorithm For Requirement 12

The following algorithm is the recommended baseline for the project.

1. Start from the original degree-9 trajectory segment.
2. Select the start and end of the desired constant-speed interval:
   - in absolute coordinates, or
   - through a chosen percentage convention.
3. Determine the corresponding positions:
   - $x_{cv}^{start}$, $x_{cv}^{end}$ in 1D, or
   - $\mathbf{p}_{cv}^{start}$, $\mathbf{p}_{cv}^{end}$ in 3D.
4. Choose the requested constant speed magnitude $v_{cv}$.
5. Build the middle constant-speed segment:
   - affine in 1D;
   - arc-length-reparameterized in 3D if path preservation is required;
   - straight-chord affine in 3D if path preservation is not required.
6. Compute the duration $T_{cv}$ of the middle segment.
7. Construct entry and exit transitions that connect smoothly to the middle segment.
8. Compute the transition coefficients with `PolynomialCoefficientDetermination.md`.
9. Verify continuity and duration consistency for the full three-part construction.

## Relation To The Data Model

`02-data-model.md` already defines the following fields inside `Constraints`:

$$
\mathrm{constantVelocitySelection},\quad
\mathrm{constantVelocityValue},\quad
\mathrm{constantVelocityInitialPos1D},\quad
\mathrm{constantVelocityEndPos1D},\quad
\mathrm{constantVelocityInitialPos},\quad
\mathrm{constantVelocityEndPos}
$$

This document provides the theoretical meaning of those fields:

- `constantVelocitySelection` activates the special subsegment logic;
- `constantVelocityValue` stores the requested speed magnitude $v_{cv}$;
- the `1D` and `3D` position fields locate the start and end of the constant-speed part.

What the data model does **not** yet decide explicitly is:

- whether the 3D interpretation is path-preserving or straight-chord;
- which percentage convention should be used when positions are not given absolutely;
- how transition segments are represented as part of the higher-level trajectory structure.

Those are theory-to-design bridges, not contradictions.

## Worked Symbolic Considerations

### Example 1: 1D Exact Constant-Velocity Middle Segment

Suppose the selected interval is:

$$
x_{cv}^{start} \to x_{cv}^{end}
$$

with requested speed magnitude $v_{cv}$.

Then the middle segment is:

$$
x_{cv}(t) = x_{cv}^{start} + \sigma v_{cv}(t - t_{cv}^{start})
$$

with duration:

$$
T_{cv} = \frac{|x_{cv}^{end} - x_{cv}^{start}|}{v_{cv}}
$$

The surrounding degree-9 transition segments must reach and leave this affine law with zero acceleration, jerk, and snap at the junctions.

### Example 2: 3D Path-Preserving Constant-Speed Segment

Suppose the original polynomial path is:

$$
\mathbf{p}_{orig}(t)
$$

and two points on it are selected.

The geometric subpath between them is preserved, but the time law is replaced by an arc-length-based one:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{orig}\!\left(\ell^{-1}(s(\sigma))\right)
$$

This preserves the original curve while enforcing constant speed magnitude.

### Example 3: 3D Straight-Chord Alternative

If one instead chooses:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{cv}^{start} +
\frac{t - t_{cv}^{start}}{T_{cv}}
\left(
\mathbf{p}_{cv}^{end} - \mathbf{p}_{cv}^{start}
\right)
$$

then the motion is exactly constant-velocity in the vector sense, but the original curved subpath is replaced by a straight chord.

This makes the trade-off explicit:

- preserve path geometry, or
- preserve a simpler affine time law.

## Open Questions

- **Open — to resolve during later software design**: in 3D, should the application expose the path-preserving arc-length option as the default, or also offer the straight-chord constant-velocity alternative explicitly?
- **Open — to resolve during data-format design**: when users specify the interval in percentages, which convention should be standardized in 3D: parameter percentage, arc-length percentage, or both?
- **Open — to resolve together with the later blending documents**: should the transition segments around the constant-speed part always enforce zero acceleration, jerk, and snap at the junctions, or should the continuity order be configurable?

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `BlendingSegments`
- `TrajectoryPassingThroughConstraintPoints`
- `02-data-model.md`
