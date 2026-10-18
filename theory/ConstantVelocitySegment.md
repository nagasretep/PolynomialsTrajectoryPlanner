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
- $T_{cv} = t_{cv}^{end} - t_{cv}^{start}$: duration of the constant-velocity subsegment;
- $\sigma \in \{+1,-1\}$: 1D travel-direction sign;
- $\ell(t)$: arc-length function along the original 3D path, measured from the start of the selected interval;
- $\tau_{cv} \in [0,1]$: local normalized time within the 3D constant-speed part (distinct from the direction sign $\sigma$ above — see the notation note where it is introduced);
- $\ell_{cv}(\tau_{cv})$: arc length traveled within the constant-speed part, as a function of $\tau_{cv}$.

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

### Geometric Interpretation

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
\eta_{start}, \eta_{end} \in [0,1]
$$

with:

$$
x_{cv}^{start} = x_0 + \eta_{start}(x_f - x_0)
$$

$$
x_{cv}^{end} = x_0 + \eta_{end}(x_f - x_0)
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

## Decided 3D Interpretation For This Requirement

**DECISION**: for this requirement, the project adopts arc-length reparameterization (path-preserving, constant speed magnitude) as the baseline 3D interpretation, not just as a recommendation.

- preserve the selected geometric subpath of the original polynomial curve;
- impose constant **speed magnitude** along that preserved subpath.

**Rationale**:

- this interpretation offers the widest applicative range: it correctly handles a constant-speed requirement along a genuinely curved, multi-axis path, which the straight-chord alternative cannot do without deviating from the original path;
- in the practical case where the constant-speed requirement applies to motion along a single Cartesian axis — the most common real-world case for this kind of process constraint (e.g. avoiding acceleration-induced stress on the transported/machined body along one direction) — the original path in that interval is already a straight line, and the general arc-length construction reduces exactly (not approximately) to the same affine law used in the 1D case. No separate code path is needed for that case;
- the straight-chord alternative remains valid and simpler, but only coincides with path preservation when the selected subpath is already straight; for a genuinely curved multi-axis subpath it silently departs from the original geometry;
- the straight-chord-on-a-genuinely-curved-multi-axis-path case is deliberately not adopted as the project baseline now — it is deferred and will be added only if a concrete need for it arises (see Open Questions).

This decision was reached after clarifying two points: (a) only the straight-chord option keeps the velocity *vector* constant in both magnitude and direction — the arc-length option keeps only the magnitude constant, since direction varies continuously along a curve; (b) the two options coincide exactly, not just approximately, whenever the selected subpath is a straight line — which was verified algebraically by reducing the general arc-length construction to the 1D affine formula for that special case.

So, unlike the component-wise baseline adopted in `TrajectoryConstraints.md` for general constraint analysis, this document is primarily path-oriented:

- for general limits, component-wise treatment remains the default baseline;
- for this specific requirement, the decided baseline is constant speed along the selected path portion, via arc-length reparameterization.

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
\tau_{cv} = \frac{t - t_{cv}^{start}}{T_{cv}} \in [0,1]
$$

**Notation note**: this local normalized time is called $\tau_{cv}$, not $\sigma$, even though $\sigma$ was already used earlier in this document. That earlier $\sigma \in \{+1,-1\}$ is the 1D travel-direction sign (see *1D Case: Exact Constant Velocity Segment*) — an unrelated quantity that happens to share the same letter. Keeping both as $\sigma$ would make this section read as if the direction sign were somehow becoming a normalized time variable. $\tau_{cv}$ also matches the project-wide convention that normalized time is written $\tau$ (see `SymbolsAndNomenclature`).

The corresponding traveled arc length is:

$$
\ell_{cv}(\tau_{cv}) = \tau_{cv} L_{cv}
$$

(named $\ell_{cv}$, not $s$, to avoid colliding with $s$/$\mathbf{s}$, which are reserved exclusively for snap).

The path-preserving constant-speed motion is then obtained by:

$$
\mathbf{p}_{cv}(t) =
\mathbf{p}_{orig}\!\left(\ell^{-1}\!\left(\ell_{cv}(\tau_{cv})\right)\right)
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

### Implementation Note: Where Numerical Methods Are Actually Needed

This non-polynomial consequence only bites in the genuinely curved, multi-axis case:

- **single-axis case** (the original subpath is already a straight line): as shown above, the arc-length construction reduces exactly to the closed-form affine law — evaluation stays polynomial, no numerical methods needed;
- **genuinely curved multi-axis case**: $\ell(t)$ generally has no closed form for a degree-9 polynomial path and must be computed via **numerical integration**; recovering $\ell^{-1}(\ell_{cv})$ then requires **numerical root-finding**. This is the only point in the theory library so far where evaluation departs from direct coefficient/Horner-based computation.

Implementation should branch on this distinction rather than always paying the numerical cost: detect the single-axis (straight-subpath) case and use the closed-form affine law directly, reserving numerical integration and root-finding for the genuinely curved multi-axis case.

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
\eta_{start}, \eta_{end} \in [0,1]
$$

on the original segment parameter interval:

$$
t_s^{orig} = t_0 + \eta_{start}(t_f - t_0)
$$

$$
t_e^{orig} = t_0 + \eta_{end}(t_f - t_0)
$$

and then:

$$
\mathbf{p}_{cv}^{start} = \mathbf{p}_{orig}(t_s^{orig}), \qquad
\mathbf{p}_{cv}^{end} = \mathbf{p}_{orig}(t_e^{orig})
$$

This is parameter-percentage, not true arc-length percentage.

If a geometric percentage of traveled distance is preferred, then the percentage should be defined on arc length instead:

$$
\ell(t_s^{orig}) = \eta_{start} L_{tot}, \qquad
\ell(t_e^{orig}) = \eta_{end} L_{tot}
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

### Durations Of The Entry And Exit Transitions In 1D

The states at both ends of each transition are fixed: the outer end is the state of the neighbouring trajectory (or a rest state), the inner end is $\mathcal{S}_{cv}^{start}$ or $\mathcal{S}_{cv}^{end}$. Each transition is therefore an ordinary single polynomial segment with prescribed boundary states, and only its duration is still free. The two durations do not influence each other, so they are determined independently.

**Rule 1 — limits active (decided by the author, 2026-10-17).** Each transition takes its **shortest feasible duration**, exactly as an ordinary segment (see *Shortest Feasible Duration* in `TrajectoryConstraints.md`): the smallest $T$ for which the transition respects every active limit, found with the closed form when it applies (rest-to-rest) or with the duration search otherwise. The total duration is then

$$
T_{tot} = T_{in} + T_{cv} + T_{out}.
$$

Consequences:

- The constant-velocity part is not changed by the limits: if $v_{cv}$ itself exceeds the velocity limit, no transition can repair it, and the construction must be reported as infeasible (explicit failure, not a silently wrong trajectory).
- If the search for one transition fails (limits too tight for the prescribed boundary states), the failure is reported, as for ordinary segments.
- A transition whose boundary states coincide (no distance to cover, same velocity) is not determined by the limits; the fallback duration below applies.
- A **user-specified** transition duration always wins; it is then only verified against the limits.

**A consequence worth knowing.** A transition must arrive with zero acceleration, jerk and snap at the speed $v_{cv}$, while covering a fixed distance. With a tight acceleration limit this can force a long transition even when the constant-speed stretch is short. Example (degree 9, rest start, 20 units to cover, $v_{cv}=15$, $a_{max}=6$): the shortest feasible transition lasts about $13$ s although the stretch may last only $4$ s. This is the shortest duration that satisfies the limit with that boundary condition, not an error. A lower degree, or a longer lead-in distance, shortens it.

**Rule 2 — no limits active, no prescribed duration (decided by the author, 2026-10-17).** The duration follows the mean-velocity rule

$$
T = \frac{2\,\lvert \Delta x \rvert}{\lvert v_b \rvert + v_{cv}},
$$

where $\Delta x$ is the distance covered by the transition and $v_b$ the velocity at its outer end ($0$ at a rest end). It is the duration for which a smooth ramp between $v_b$ and $v_{cv}$ has about the right mean velocity. If $\Delta x = 0$ the rule gives $T = 0$, which is not a valid segment; in that case the fallback $\max(0.5\,T_{cv},\ 0.1\ \text{s})$ applies. The rule is crude when $v_b$ points against the travel direction; it is accepted as a baseline and the result is still verified when limits exist.

**Rule 3 — total duration prescribed (decided by the author, 2026-10-17).** The user fixes $T_{tot}$. The time left for the transitions is $R = T_{tot} - T_{cv}$, and it must be positive. It is split **in proportion to the shortest feasible durations** $s_{in}, s_{out}$ of the two transitions:

$$
T_{in} = R\,\frac{s_{in}}{s_{in}+s_{out}}, \qquad
T_{out} = R\,\frac{s_{out}}{s_{in}+s_{out}}.
$$

With limits active, $s$ comes from *Shortest Feasible Duration*. Without limits, the weights $s$ are the durations of Rule 2. Under the usual assumption that feasibility is not lost by lengthening a transition, $T_{in} \ge s_{in}$ and $T_{out} \ge s_{out}$ hold exactly when $R \ge s_{in}+s_{out}$, so the proportional split respects the limits whenever any split can. If $R < s_{in}+s_{out}$ with limits active, the request is infeasible and is reported explicitly (with the minimum total $T_{cv}+s_{in}+s_{out}$ in the message). The assumption does not always hold: with a non-zero boundary velocity the feasible durations of a transition may form separate windows. Example (degree 3, acceleration limit $6$, rest start, $20$ units to cover, arrival at $15$): the entry is feasible for $T \in [2.62,\ 2.76]\ \text{s}$, infeasible up to about $7.5$ s, and feasible again beyond. For a large total duration the proportional split can then fall in the infeasible stretch although another split would be feasible. The request is then rejected with an explicit error and never silently repaired; choosing, among the feasible splits, the one closest to the proportional one is a possible refinement (register D18). A scan of degrees 5, 7 and 9 with the same data found no such case, which is not a proof. If only one transition has a prescribed duration, the other takes the remainder $R$ minus that duration. Prescribing both durations and the total is a conflict and is rejected.

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

The curved case is completed in *Curved 3D Constant-Speed Stretch: Boundary States, Evaluation And Limits*.

## Curved 3D Constant-Speed Stretch: Boundary States, Evaluation And Limits

**Notation note**: in this section the parametric speed of the path is $\nu(t) = \lVert \dot{\mathbf{p}}(t) \rVert$ (not $\sigma$, which is the 1D travel-direction sign), the fractions of the segment are $\eta$ (not $\lambda$, which is the common duration scale factor of the pass-through documents), and the arc length is $\ell$.

This section completes the path-preserving 3D option (*3D Path-Preserving Strategy: Arc-Length Reparameterization*) for a genuinely curved subpath: what states the stretch has at its two ends, how the transitions connect to it, how it is evaluated, and what the limits imply. For a straight subpath everything below reduces to the straight-chord construction (see *Consistency With The Straight Case*).

### Regularity

Let $\mathbf{p}(t)$ be the polynomial path of the original segment and $\nu(t) = \lVert \dot{\mathbf{p}}(t) \rVert$ its speed in the original time. The selected subpath $[t_s, t_e]$ must satisfy $\nu(t) > 0$ on the whole closed interval, so that the arc length $\ell$ is a strictly increasing function of $t$. A subpath that touches a point where $\dot{\mathbf{p}} = \mathbf{0}$ (for example the rest state at the very start of a segment) is rejected with an explicit error: the tangent is not defined there. The direction of travel is the direction of increasing $t$ ($t_e > t_s$).

### Arc-Length Derivatives Of The Path

Let $D$ be the operator "derivative with respect to arc length", $D f = \dot f / \nu$. The geometric derivatives of the path are

$$
\mathbf{p}_s = D\mathbf{p}, \quad \mathbf{p}_{ss} = D^2\mathbf{p}, \quad \mathbf{p}_{sss} = D^3\mathbf{p}, \quad \mathbf{p}_{ssss} = D^4\mathbf{p}.
$$

The first two have closed forms:

$$
\mathbf{p}_s = \hat{\mathbf{T}} = \frac{\dot{\mathbf{p}}}{\nu},
\qquad
\mathbf{p}_{ss} = \kappa\,\hat{\mathbf{N}} = \frac{1}{\nu^{2}}\left( \ddot{\mathbf{p}} - \frac{\dot{\mathbf{p}}\cdot\ddot{\mathbf{p}}}{\nu^{2}}\,\dot{\mathbf{p}} \right),
$$

with $\hat{\mathbf{T}}$ the unit tangent, $\kappa$ the curvature and $\hat{\mathbf{N}}$ the unit principal normal. $\mathbf{p}_{sss}$ and $\mathbf{p}_{ssss}$ have no short closed form and are obtained by **truncated Taylor-series (jet) arithmetic** at the point of interest: expand each component of $\mathbf{p}$ to order $4$ around $t_0$; compute the series of $\nu^2 = \dot{\mathbf{p}}\cdot\dot{\mathbf{p}}$, of $\nu$ (series square root) and of $1/\nu$; then apply $D$ four times, each time differentiating the series and multiplying by the series of $1/\nu$ (the series of $\nu$ is computed once from the original path and not recomputed). Each application costs one order of accuracy of the series, so a series of order $4$ is exactly enough to reach $D^4\mathbf{p}$ at order $0$. Only the derivatives up to order $4$ of $\mathbf{p}$ at $t_0$ are used; for a degree below $9$ only the orders the degree needs are required ($p - 1$ with $p = (d+1)/2$).

### States Of The Stretch In Time

On the stretch the speed magnitude is the constant $v_{cv}$, so the arc length is $\ell = v_{cv}\,(t - t_{cv}^{start})$ and $d\ell/dt = v_{cv}$ exactly. Therefore the derivative of order $k$ in time is

$$
\frac{d^{k}\mathbf{p}_{cv}}{dt^{k}} = v_{cv}^{\,k}\; D^{k}\mathbf{p}\big|_{t^{*}},
\qquad t^{*} = \ell^{-1}\!\left(v_{cv}\,(t - t_{cv}^{start})\right).
$$

Unlike the 1D (or straight) case, the acceleration, jerk and snap of the stretch are **not zero** on a curved path: for example the acceleration is $v_{cv}^2\,\kappa\,\hat{\mathbf{N}}$ (centripetal), of constant magnitude only if the curvature is constant. The boundary states of the stretch are

$$
\mathcal{S}_{cv}^{start} = \left( \mathbf{p}(t_s),\; v_{cv}\mathbf{p}_s,\; v_{cv}^2\mathbf{p}_{ss},\; v_{cv}^3\mathbf{p}_{sss},\; v_{cv}^4\mathbf{p}_{ssss} \right)_{t_s},
$$

and the same at $t_e$ for $\mathcal{S}_{cv}^{end}$ (as many entries as the degree of the transitions uses).

### Transitions

The entry transition is an ordinary 3D polynomial segment (component-wise coefficients, one common duration) from the state at the start of the trajectory to $\mathcal{S}_{cv}^{start}$; the exit transition from $\mathcal{S}_{cv}^{end}$ to the state at the end. Because their end states are exactly the derivatives of the stretch, position, velocity, acceleration, jerk and snap are continuous at both junctions. The path is preserved **on the stretch only**; the transitions are new polynomial pieces that connect the same end states and do not follow the original path. Their durations follow Rules 1–3 of *Durations Of The Entry And Exit Transitions In 1D* with two changes: feasibility is the maximum over the three components and the active limits, and the distance in Rule 2 is the Euclidean distance between the end positions of the transition.

### Evaluation Of The Stretch

- **Arc length**: $\ell(t) = \int_{t_s}^{t}\nu(u)\,du$ by composite Gauss–Legendre quadrature (for example $16$ nodes per panel), doubling the number of panels until two successive values agree to a relative $10^{-13}$. $\nu$ is smooth and bounded away from zero on the subpath, so convergence is fast. The total length is $L_{cv} = \ell(t_e)$ and $T_{cv} = L_{cv}/v_{cv}$.
- **Inversion**: given the target length $\ell_\star = v_{cv}(t - t_{cv}^{start})$, solve $\ell(t^*) = \ell_\star$ by a safeguarded Newton iteration (bracket $[t_s, t_e]$, derivative $\nu > 0$), falling back to bisection when a Newton step leaves the bracket, until $\lvert\ell(t^*) - \ell_\star\rvert \le 10^{-13} L_{cv}$.
- **State at time $t$**: $t^*$ from the inversion, then $v_{cv}^{k} D^{k}\mathbf{p}(t^*)$ for the needed orders (the jets of the previous paragraph at $t^*$).
- **Consistency check**: at $t = T_{cv}$ the inversion returns $t_e$.

### Limits On The Stretch

The transitions can adapt their durations to the limits; the stretch cannot, because its time law is fixed by $v_{cv}$ and the geometry. Writing $P_k$ for the peak over the subpath of a component of $D^{k}\mathbf{p}$ (component-wise treatment, as in `TrajectoryConstraints.md`), the peak of the $k$-th time derivative on the stretch is $v_{cv}^{k} P_k$. With the limits $L_k$ ($k = 1,\dots,4$ for velocity, acceleration, jerk, snap):

$$
v_{cv} \le v_{adm} = \min_{k\ \text{active}} \left( \frac{L_k}{P_k} \right)^{1/k}.
$$

This is the same structure as the scaling laws of the other chapters. $P_1$ is at most $1$ (a component of the unit tangent), so the velocity condition is $v_{cv}\max_i\lvert T_i\rvert \le v_{max}$ per axis. If $v_{cv} > v_{adm}$ the construction fails explicitly and the message states $v_{adm}$ and the limit that binds. The peaks $P_k$ are found by sampling $t$ on a fine grid over $[t_s, t_e]$ followed by local refinement (golden-section or Newton on the derivative of the component), because the components of $D^k\mathbf{p}$ are not polynomial.

### Specifying The Stretch

The Core accepts any of three equivalent specifications of $t_s$ and $t_e$ on a given segment: (a) two points on the path, recovered as the parameter of the closest point (rejected if the distance exceeds a tolerance, as for blending); (b) two fractions of the normalized parameter $\tau \in (0,1)$; (c) two fractions of the total arc length of the segment (by the inversion above). Which of them the user interface offers is a decision for the UI work.

### Consistency With The Straight Case

If the subpath is a straight line, then $\kappa = 0$ and $\mathbf{p}_{ss} = \mathbf{p}_{sss} = \mathbf{p}_{ssss} = \mathbf{0}$ on it, $\hat{\mathbf{T}}$ is constant, and the construction coincides exactly with the straight-chord one: constant velocity vector, zero acceleration, jerk and snap at the junctions. This is the test that the general code must pass on a straight segment, and the reason a single code path is enough.

### Reference Values

The path $\mathbf{p}(t) = (t,\ t^2,\ 0)$ on $t \in [0, 1]$ (computed by Cowork with series arithmetic; independent of the application code):

- Length $L = (2\sqrt{5} + \operatorname{asinh} 2)/4 = 1.478943$.
- At $t = 0$: $\mathbf{p}_s = (1, 0, 0)$, $\mathbf{p}_{ss} = (0, 2, 0)$, $\mathbf{p}_{sss} = (-4, 0, 0)$, $\mathbf{p}_{ssss} = (0, -32, 0)$.
- At $t = 1$: $\mathbf{p}_s = (0.447214, 0.894427, 0)$, $\mathbf{p}_{ss} = (-0.16, 0.08, 0)$, $\mathbf{p}_{sss} = (0.157419, -0.114487, 0)$, $\mathbf{p}_{ssss} = (-0.24064, 0.23552, 0)$.
- Component peaks over $[0,1]$ (true maxima, refined; a $2001$-point sampling gives $18.187075$ for the first value of $P_4$ because the maximum lies between samples): $P_1 = (1, 0.894427)$, $P_2 = (0.649519, 2)$, $P_3 = (4, 3.808294)$, $P_4 = (18.187110, 32)$ for the $(x, y)$ components; the maxima over components are $1, 2, 4, 32$.
- Admissible speed with $a_{max} = 8$ and $j_{max} = 20$ per axis: $\min\{(8/2)^{1/2}, (20/4)^{1/3}\} = \min\{2, 1.709976\} = 1.709976$ (jerk binds).
- With $v_{cv} = 1$ on this path the stretch lasts $T_{cv} = 1.478943$ s, and at $t = T_{cv}/2$ the parameter is the solution of $\ell(t^*) = L/2$.

### Baseline Algorithm For The Curved Case

1. Check $\nu > 0$ on $[t_s, t_e]$; recover $t_s, t_e$ from the specification.
2. Compute $L_{cv}$ and $T_{cv} = L_{cv}/v_{cv}$.
3. Compute $P_k$ and $v_{adm}$ if limits are active; fail explicitly if $v_{cv} > v_{adm}$.
4. Compute the boundary states $\mathcal{S}_{cv}^{start}$ and $\mathcal{S}_{cv}^{end}$ from the jets.
5. Build the entry and exit transitions as 3D polynomial segments with durations from Rules 1–3.
6. Verify continuity at the junctions and the limits on the three pieces; the stretch is verified by sampling.

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

- `constantVelocitySelection`
- `constantVelocityValue`
- `constantVelocityInitialPos1D`
- `constantVelocityEndPos1D`
- `constantVelocityInitialPos`
- `constantVelocityEndPos`

This document provides the theoretical meaning of those fields:

- `constantVelocitySelection` activates the special subsegment logic;
- `constantVelocityValue` stores the requested speed magnitude $v_{cv}$;
- the `1D` and `3D` position fields locate the start and end of the constant-speed part.

What the data model does **not** yet decide explicitly is:

- which percentage convention should be used when positions are not given absolutely;
- how transition segments are represented as part of the higher-level trajectory structure.

(Whether the 3D interpretation is path-preserving or straight-chord is no longer open — see *Decided 3D Interpretation For This Requirement* above.)

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
\mathbf{p}_{orig}\!\left(\ell^{-1}\!\left(\ell_{cv}(\tau_{cv})\right)\right)
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

- **Open — to resolve if/when a concrete need arises**: the straight-chord alternative for a genuinely curved multi-axis subpath is deliberately not part of the current baseline (see *Decided 3D Interpretation*); should it be added later as an explicit user-facing option, and what would trigger that need?
- **Resolved (2026-10-18)**: the Core supports points, parameter fractions and arc-length fractions (see *Specifying The Stretch*); the choice offered by the user interface is a UI decision.
- **Resolved (2026-10-18)**: the transitions match the full boundary state of the stretch (zero acceleration, jerk and snap on a straight stretch, the arc-length derivatives on a curved one) up to the order of the polynomial degree; the continuity order is the degree's, not configurable.
- **Resolved (2026-10-17)**: durations of the entry and exit transitions: Rule 1 (limits active), Rule 2 (mean-velocity rule without limits) and Rule 3 (prescribed total, proportional split); see *Durations Of The Entry And Exit Transitions In 1D*.
- **Resolved**: the 3D arc-length section reused $\sigma$ for the local normalized time, colliding with the 1D travel-direction sign $\sigma$ defined earlier in this same document, and named the traveled-arc-length function $s(\cdot)$, colliding with $s$/$\mathbf{s}$ (reserved for snap). The local normalized time is now $\tau_{cv}$ and the traveled-arc-length function is now $\ell_{cv}(\cdot)$; see *3D Path-Preserving Strategy: Arc-Length Reparameterization*.

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `PolynomialCoefficientDetermination.md`
- `TrajectoryConstraints.md`
- `BlendingSegments`
- `TrajectoryPassingThroughConstraintPoints`
- `02-data-model.md`
