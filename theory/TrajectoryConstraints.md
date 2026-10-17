# Trajectory Constraints

## Intro And Scope

This document studies trajectory constraints for the polynomial trajectories currently in scope, with the ninth-degree polynomial as the main technical reference case.

It covers the requirement cluster formed by items 7, 8, 9, and 10 in `01-requirements.md`:

- determination of maxima and minima for position, velocity, acceleration, jerk, and snap;
- procedure to impose a velocity limit;
- procedure to impose an acceleration limit;
- procedure to impose a jerk limit.

The treatment starts in 1D and then extends to 3D. In keeping with `03-assumptions.md`, the discussion remains theoretical, symbolic-first, and independent of actuator dynamics or robot kinematics.

## Symbols Used

The notation follows `SymbolsAndNomenclature.md`. The main symbols used here are:

- $t$: absolute time;
- $t_0$: initial time;
- $t_f$: final time;
- $T = t_f - t_0$: segment duration;
- $\tau = \frac{t - t_0}{T}$: normalized time;
- $x(t)$: scalar position law in 1D;
- $v(t) = \dot{x}(t)$: velocity;
- $a(t) = \ddot{x}(t)$: acceleration;
- $j(t) = x^{(3)}(t)$: jerk;
- $s(t) = x^{(4)}(t)$: snap;
- $x^{(5)}(t)$: crackle;
- $\mathbf{p}(t)$, $\mathbf{v}(t)$, $\mathbf{a}(t)$, $\mathbf{j}(t)$, $\mathbf{p}^{(4)}(t)$: 3D position and derivative vectors;
- $v_{max}$: admissible velocity limit;
- $a_{max}$: admissible acceleration limit;
- $j_{max}$: admissible jerk limit.

Unless stated otherwise, all extrema are searched over the closed interval:

$$
t \in [t_0, t_f]
$$

or, equivalently:

$$
\tau \in [0,1]
$$

## Constraint Context

In this project, a constraint is a condition imposed on the trajectory after the polynomial structure has been defined.

The most important kinematic constraints considered here are:

- bounds on velocity;
- bounds on acceleration;
- bounds on jerk.

Snap is included in the extrema analysis because Requirement 7 asks for it explicitly, even though there is no separate limit-imposition requirement for snap at this stage.

Position is also included in the extrema analysis, not because it is normally a "limit" in the same sense as velocity or acceleration, but because understanding where position reaches local or global extrema is part of understanding the trajectory profile.

## Core Principle: Candidate Points For Extrema

For any scalar function $f(t)$ on a closed interval, the global maximum and minimum can only occur at:

- the interval endpoints;
- interior stationary points where $\frac{df}{dt} = 0$;
- interior points where the function is not differentiable.

Polynomial trajectories and their derivatives are smooth, so the third item does not apply here.

Therefore, the general constraint-analysis rule in this project is:

1. identify the scalar quantity of interest;
2. differentiate it;
3. solve the stationarity equation inside the interval;
4. evaluate the original quantity at all candidate points;
5. compare those values to obtain maxima and minima.

This rule is simple, but it drives almost the whole document.

## Why Higher Derivatives Matter For Extrema

The derivative chain studied in `NinthDegreePolynomialTheory.md` now becomes operational.

For a degree-9 polynomial:

- extrema of position depend on velocity;
- extrema of velocity depend on acceleration;
- extrema of acceleration depend on jerk;
- extrema of jerk depend on snap;
- extrema of snap depend on crackle.

So the higher derivatives are not only abstract quantities. They are exactly the tools needed to locate the candidate extrema of the lower-order profiles.

## Position Extrema In 1D

### General Rule

To find maxima and minima of position $x(t)$ over $[t_0, t_f]$, solve:

$$
v(t) = \dot{x}(t) = 0
$$

for all roots inside the interval, then evaluate:

- $x(t_0)$;
- $x(t_f)$;
- $x(t_i)$ for every interior root $t_i$ of $v(t)$.

The largest of these values is the maximum position, and the smallest is the minimum position.

### Degree-9 Specialization

For a degree-9 polynomial:

$$
x(t) = \sum_{i=0}^{9} a_i t^i
$$

the velocity is degree 8:

$$
v(t) = \dot{x}(t)
$$

So the position-extrema problem reduces to solving a degree-8 polynomial equation.

### Important Interpretation

The second derivative test may help classify local extrema, but for global max/min over a finite interval it is not strictly necessary. The safest method is direct comparison of all candidate values.

## Velocity Extrema In 1D

### General Rule

To find maxima and minima of velocity $v(t)$ over $[t_0, t_f]$, solve:

$$
a(t) = \ddot{x}(t) = 0
$$

for all roots inside the interval, then evaluate:

- $v(t_0)$;
- $v(t_f)$;
- $v(t_i)$ for every interior root $t_i$ of $a(t)$.

### Degree-9 Specialization

For a degree-9 polynomial, acceleration is degree 7:

$$
a(t) = \ddot{x}(t)
$$

So the velocity-extrema problem reduces to solving a degree-7 polynomial equation.

### Absolute Velocity

When a velocity limit is expressed as a magnitude bound:

$$
|v(t)| \le v_{max}
$$

the relevant quantity is not the signed maximum alone, but:

$$
\max_{t \in [t_0,t_f]} |v(t)|
$$

This can be obtained by evaluating $|v(t)|$ at the same candidate points used for the signed extrema, or equivalently by checking both the maximum and minimum signed values.

## Acceleration Extrema In 1D

### General Rule

To find maxima and minima of acceleration $a(t)$, solve:

$$
j(t) = x^{(3)}(t) = 0
$$

inside the interval, then compare:

- $a(t_0)$;
- $a(t_f)$;
- $a(t_i)$ for each interior root $t_i$ of $j(t)$.

### Degree-9 Specialization

For the degree-9 polynomial, jerk is degree 6:

$$
j(t) = x^{(3)}(t)
$$

So acceleration-extrema analysis reduces to solving a degree-6 polynomial equation.

### Absolute Acceleration

For an acceleration bound:

$$
|a(t)| \le a_{max}
$$

the quantity to monitor is:

$$
\max_{t \in [t_0,t_f]} |a(t)|
$$

again obtained by evaluating the acceleration at all candidate points and taking absolute values.

## Jerk Extrema In 1D

### General Rule

To find maxima and minima of jerk $j(t)$, solve:

$$
s(t) = x^{(4)}(t) = 0
$$

inside the interval, then compare:

- $j(t_0)$;
- $j(t_f)$;
- $j(t_i)$ for each interior root $t_i$ of $s(t)$.

### Degree-9 Specialization

For the degree-9 polynomial, snap is degree 5:

$$
s(t) = x^{(4)}(t)
$$

So jerk-extrema analysis reduces to solving a degree-5 polynomial equation.

### Absolute Jerk

For a jerk limit:

$$
|j(t)| \le j_{max}
$$

the relevant quantity is:

$$
\max_{t \in [t_0,t_f]} |j(t)|
$$

computed from the same candidate-point set.

## Snap Extrema In 1D

### General Rule

To find maxima and minima of snap $s(t)$, solve:

$$
x^{(5)}(t) = 0
$$

inside the interval, then compare:

- $s(t_0)$;
- $s(t_f)$;
- $s(t_i)$ for each interior root $t_i$ of $x^{(5)}(t)$.

### Degree-9 Specialization

For the degree-9 polynomial, crackle is degree 4:

$$
x^{(5)}(t)
$$

So snap-extrema analysis reduces to solving a quartic equation. This is the highest-order extrema search in this document whose stationarity equation still falls inside the classical closed-form range.

## Summary Of Stationarity Equations

The extrema-search structure for the degree-9 case can be summarized as:

| Quantity analysed | Candidate interior points come from |
|---|---|
| position $x(t)$ | roots of $v(t) = 0$ |
| velocity $v(t)$ | roots of $a(t) = 0$ |
| acceleration $a(t)$ | roots of $j(t) = 0$ |
| jerk $j(t)$ | roots of $s(t) = 0$ |
| snap $s(t)$ | roots of $x^{(5)}(t) = 0$ |

This table is operationally important: it tells which polynomial must be solved for each profile.

## Symbolic Setup Vs Numerical Root Extraction

The project prefers symbolic treatment, but extrema detection introduces an unavoidable practical distinction:

- the polynomial expressions and the stationarity equations can be written symbolically;
- the actual extraction of all real roots in the interval may require numerical methods.

This happens because:

- quartic equations still admit closed-form symbolic solutions in principle;
- quintic and higher-degree polynomial equations do not, in general, admit a universal closed-form solution by radicals.

Therefore, for the degree-9 case:

- snap extrema can still be tied to a quartic stationarity equation;
- jerk, acceleration, velocity, and position extrema typically require numerical root extraction for the stationarity polynomial.

This is not a contradiction with the symbolic-first goal. The theory remains symbolic; the final root-finding step becomes numerical when algebra no longer admits a general closed form.

## Generic 1D Extrema Algorithm

The following algorithm applies to any scalar quantity among:

$$
x(t),\qquad v(t),\qquad a(t),\qquad j(t),\qquad s(t)
$$

1. Select the target profile $f(t)$.
2. Compute its derivative $f'(t)$.
3. Solve $f'(t) = 0$ and retain only the real roots inside $[t_0, t_f]$.
4. Build the candidate set:

$$
\{t_0,\; t_f,\; t_1,\; t_2,\; \dots,\; t_m\}
$$

5. Evaluate $f(t)$ at every candidate point.
6. Compare those values to obtain:
   - global maximum;
   - global minimum;
   - if needed, maximum absolute value.

This is the baseline algorithm for Requirement 7.

## Practical Root-Finding Note

For software implementation, the root-finding step should be understood as:

- formulate the stationarity polynomial exactly from the coefficients;
- compute all its roots numerically;
- discard complex roots;
- discard real roots lying outside the interval;
- keep the remaining roots as extrema candidates.

Possible numerical strategies include:

- companion-matrix eigenvalue extraction;
- robust polynomial root solvers;
- interval-based refinement after coarse root localization.

The choice of numerical solver belongs to implementation, not to the theory baseline. The theory only requires that all real interval roots be found reliably enough for extrema detection.

## Imposing A Velocity Limit In 1D

### Problem Form

The constraint is:

$$
|v(t)| \le v_{max}\qquad \forall t \in [t_0,t_f]
$$

The question is not merely whether the current segment satisfies the limit, but how to modify the formulation so that it does.

### Verification Step

The first step is always verification:

1. determine the polynomial coefficients;
2. compute $\max |v(t)|$ over the interval using the extrema algorithm;
3. compare the result with $v_{max}$.

If:

$$
\max |v(t)| \le v_{max}
$$

then the constraint is already satisfied.

### Constraint-Imposition Strategy

If the limit is violated, the standard project-level remedy is to modify the segment duration and recompute the coefficients.

The reason is structural:

- the endpoint conditions remain the same;
- the polynomial degree remains the same;
- the time law changes because the duration changes;
- the coefficients are then recomputed from the updated duration.

So the practical velocity-limit procedure is:

1. choose a candidate duration $T$;
2. determine the coefficients using `PolynomialCoefficientDetermination.md`;
3. compute $\max |v(t)|$;
4. if the bound is violated, increase $T$ and repeat;
5. stop when the limit is satisfied.

### Duration Search

A robust algorithmic form is:

1. start from an initial duration $T^{(0)}$;
2. build a feasibility test:

$$
F_v(T) = \max_{t \in [t_0,t_f]} |v(t;T)| - v_{max}
$$

3. if $F_v(T) \le 0$, the duration is feasible;
4. otherwise enlarge $T$ and test again;
5. once a feasible interval is bracketed, optionally refine the smallest acceptable duration by bisection or another one-dimensional search method.

This turns velocity-limit imposition into a duration-search problem.

## Imposing An Acceleration Limit In 1D

### Problem Form

The constraint is:

$$
|a(t)| \le a_{max}\qquad \forall t \in [t_0,t_f]
$$

### Verification Step

1. determine the polynomial coefficients;
2. compute $\max |a(t)|$ using the extrema algorithm driven by $j(t)=0$;
3. compare the result with $a_{max}$.

### Constraint-Imposition Strategy

If the bound is violated, the same logic applies:

- modify the duration;
- recompute the polynomial;
- recompute the acceleration extrema;
- repeat until the bound is satisfied.

Define:

$$
F_a(T) = \max_{t \in [t_0,t_f]} |a(t;T)| - a_{max}
$$

and search for a duration such that:

$$
F_a(T) \le 0
$$

The method is the same as for velocity, but the monitored profile is acceleration instead of velocity.

## Imposing A Jerk Limit In 1D

### Problem Form

The constraint is:

$$
|j(t)| \le j_{max}\qquad \forall t \in [t_0,t_f]
$$

### Verification Step

1. determine the polynomial coefficients;
2. compute $\max |j(t)|$ using the extrema algorithm driven by $s(t)=0$;
3. compare the result with $j_{max}$.

### Constraint-Imposition Strategy

If the bound is violated, define:

$$
F_j(T) = \max_{t \in [t_0,t_f]} |j(t;T)| - j_{max}
$$

and search for a duration satisfying:

$$
F_j(T) \le 0
$$

Again, the general procedure is:

1. modify duration;
2. recompute coefficients;
3. recompute jerk extrema;
4. iterate until feasible.

## A Caution About Time Scaling

The previous sections intentionally describe limit imposition through recomputation, not through a universal closed-form scaling formula.

That caution matters because, in the general project setting:

- physical endpoint derivative values may be prescribed;
- the coefficient solution depends on the chosen duration;
- changing $T$ therefore changes the coefficient problem itself.

So, in the most general case treated here, it is safest to:

- recompute the polynomial for each tested duration;
- then re-evaluate the constrained profile.

### Special Normalized-Profile Case

There is, however, an important special case.

If the normalized profile shape is already fixed and one only rescales time, then derivative magnitudes scale as:

$$
v \sim \frac{1}{T}, \qquad
a \sim \frac{1}{T^2}, \qquad
j \sim \frac{1}{T^3}
$$

This explains why increasing duration often helps reduce peak derivatives.

But in the baseline formulation of this project, the more general recompute-and-verify strategy is the safer one to adopt for requirements 8, 9, and 10.

## Position, Velocity, Acceleration, Jerk, And Snap In Normalized Time

Using normalized time can simplify extrema analysis because every segment is studied on the same interval:

$$
\tau \in [0,1]
$$

The candidate-point logic remains identical:

- extrema of $x(\tau)$ come from $\frac{dx}{d\tau}=0$;
- extrema of $\frac{dx}{d\tau}$ come from $\frac{d^2x}{d\tau^2}=0$;
- and so on.

The benefit is mainly organizational:

- one local interval for all segments;
- one uniform search domain;
- easier comparison across segments.

The cost is that physical derivative limits must still be interpreted through the scaling relations between $t$ and $\tau$.

## What Changes In 3D

The scalar extrema logic does not disappear in 3D. It reappears in two distinct ways:

- component-wise analysis;
- vector-magnitude analysis.

### Component-Wise Analysis

Each Cartesian component is treated as its own scalar polynomial:

$$
x(t),\qquad y(t),\qquad z(t)
$$

Then each component has its own profiles:

- $v_x(t)$, $v_y(t)$, $v_z(t)$;
- $a_x(t)$, $a_y(t)$, $a_z(t)$;
- $j_x(t)$, $j_y(t)$, $j_z(t)$;
- $p_x^{(4)}(t)$, $p_y^{(4)}(t)$, $p_z^{(4)}(t)$.

The scalar extrema algorithm can then be applied independently to each component.

### Vector-Magnitude Analysis

Alternatively, one may analyse magnitudes such as:

$$
\|\mathbf{v}(t)\|,\qquad
\|\mathbf{a}(t)\|,\qquad
\|\mathbf{j}(t)\|,\qquad
\|\mathbf{p}^{(4)}(t)\|
$$

This is often more physically meaningful when the concern is the total kinematic demand rather than the per-axis demand.

For example:

$$
\|\mathbf{v}(t)\| = \sqrt{v_x(t)^2 + v_y(t)^2 + v_z(t)^2}
$$

To find extrema of velocity magnitude, one can instead study:

$$
\|\mathbf{v}(t)\|^2 = v_x(t)^2 + v_y(t)^2 + v_z(t)^2
$$

because squaring preserves the location of maxima and minima for nonnegative magnitudes.

Its stationarity condition is:

$$
\frac{d}{dt}\left(\|\mathbf{v}(t)\|^2\right)
= 2\,\mathbf{v}(t)\cdot\mathbf{a}(t) = 0
$$

Similarly:

- extrema of $\|\mathbf{a}(t)\|^2$ come from $\mathbf{a}(t)\cdot\mathbf{j}(t)=0$;
- extrema of $\|\mathbf{j}(t)\|^2$ come from $\mathbf{j}(t)\cdot\mathbf{p}^{(4)}(t)=0$.

## Recommended 3D Interpretation For This Project

This document resolves the earlier open question deferred from `SymbolsAndNomenclature.md` and `NinthDegreePolynomialTheory.md`.

The recommended project baseline is:

- use **component-wise analysis as the primary theory and implementation baseline**;
- use **vector-magnitude analysis as an additional derived check when the total 3D kinematic demand matters**.

This recommendation is consistent with the project assumptions because:

- the project is explicitly built from 1D to 3D;
- component-wise treatment is the direct extension of the scalar theory;
- it is simpler to derive, verify, and implement;
- it avoids introducing unnecessary coupling too early.

So, for the continuation of this project:

- the default interpretation of 3D constraints is component-wise;
- vector-magnitude analysis remains valid and useful, but secondary.

## Relation To The Data Model

`02-data-model.md` defines:

$$
\mathrm{Constraints} = \{
\mathrm{constraintType},
\mathrm{constraintValue},
\mathrm{constantVelocitySelection},
\dots
\}
$$

From the viewpoint of this document:

- `constraintType` can represent velocity, acceleration, jerk, snap, time, or related segment-level restrictions;
- `constraintValue` is the bound associated with the selected type;
- `constraints[]` inside each segment is the place where one or more such restrictions are attached to that segment.

This document gives the theoretical meaning of those entries:

- analysis of maxima/minima tells whether a given profile violates a bound;
- duration search provides one baseline strategy to impose the bound;
- the same structure later supports the constant-velocity requirement addressed separately in `ConstantVelocitySegment`.

One implementation-oriented point remains open: if multiple limits must be imposed simultaneously on the same segment, the future software design may need a clearer representation than a single type/value pair viewed in isolation, even though the current `constraints[]` container already points in that direction.

## Generic Constraint-Verification Algorithm

For a selected derivative order and a selected bound, the following procedure can be used.

1. Determine the segment coefficients.
2. Build the target profile:
   - $v(t)$ for velocity;
   - $a(t)$ for acceleration;
   - $j(t)$ for jerk.
3. Compute the candidate extrema points from the next derivative.
4. Evaluate the target profile at all candidates.
5. Compute the maximum absolute value.
6. Compare that result with the admissible limit.
7. Mark the segment as feasible or infeasible.

This algorithm is the verification core shared by requirements 8, 9, and 10.

## Generic Constraint-Imposition Algorithm

The baseline limit-imposition strategy for the current project is:

1. choose the segment degree and boundary conditions;
2. choose an initial duration;
3. determine the polynomial coefficients;
4. verify the selected constraint;
5. if violated, increase duration and recompute;
6. repeat until the constraint is satisfied;
7. if desired, refine the smallest feasible duration.

This is conceptually simple, consistent with the current theory baseline, and directly compatible with the future desktop application.

## Worked Symbolic Considerations

### Example 1: Position Extrema

Suppose a degree-9 segment is written as:

$$
x(t) = \sum_{i=0}^{9} a_i t^i
$$

To determine the maximum and minimum position:

1. compute $v(t)$;
2. solve $v(t)=0$ in $[t_0,t_f]$;
3. evaluate $x(t)$ at the interval endpoints and at the retained interior roots;
4. compare the values.

The result is exact at the level of the candidate-point principle, even if the interior roots themselves are obtained numerically.

### Example 2: Velocity Limit

Suppose the segment must satisfy:

$$
|v(t)| \le v_{max}
$$

The procedure is:

1. compute the coefficients for the current duration;
2. solve $a(t)=0$ inside the interval;
3. evaluate $|v(t)|$ at all candidate points;
4. if the largest value exceeds $v_{max}$, enlarge duration and recompute.

This gives a clean theoretical basis for the future implementation of a velocity-limit tool.

### Example 3: 3D Constraint Interpretation

Suppose a 3D trajectory has velocity vector:

$$
\mathbf{v}(t) =
\begin{bmatrix}
v_x(t) \\
v_y(t) \\
v_z(t)
\end{bmatrix}
$$

The project baseline first checks:

$$
|v_x(t)|,\qquad |v_y(t)|,\qquad |v_z(t)|
$$

component by component.

If needed, a secondary check may also inspect:

$$
\|\mathbf{v}(t)\|
$$

to understand the total 3D kinematic demand.

## Open Questions

- **Open — to resolve during software implementation** (`src/`): which numerical root solver should be adopted for interval root extraction of the stationarity polynomials?
- **Open — to resolve during software design**: when several limits are imposed simultaneously on the same segment, should the application search duration only, or also allow reformulations that change additional parameters?
- **Open — to resolve during later project phases**: should snap also receive an explicit limit-imposition requirement, beyond the extrema analysis already covered here?

## Related Documents

- `06-theory-library-roadmap.md`
- `SymbolsAndNomenclature.md`
- `NinthDegreePolynomialTheory.md`
- `BoundaryConditions.md`
- `PolynomialCoefficientDetermination.md`
- `ConstantVelocitySegment`
- `02-data-model.md`
