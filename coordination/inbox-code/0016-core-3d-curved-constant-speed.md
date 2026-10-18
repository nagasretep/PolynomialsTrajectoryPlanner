# 0016 — Core 3D: constant speed along a curved subpath

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

Run after request 0015 (it replaces the `NotSupportedException` of its straight-subpath check).

## Goal

The Core builds a 3D constant-speed stretch along a genuinely curved subpath of a polynomial segment, preserving the path and the speed magnitude, with entry and exit transitions that match the stretch's full boundary state, and with explicit limit handling.

## Reference documents

- `theory/ConstantVelocitySegment.md`, *Curved 3D Constant-Speed Stretch: Boundary States, Evaluation And Limits* (read fully; it contains the formulas, the algorithm and reference values), *3D Path-Preserving Strategy: Arc-Length Reparameterization*, *Durations Of The Entry And Exit Transitions In 1D* (rules 1–3)
- Request 0015 and its return note (3D transitions, multi-component duration search)

## Scope

- In scope:
  1. **Arc-length derivatives**: a component that, for a polynomial path and a parameter $t_0$, returns $\mathbf{p}_s, \mathbf{p}_{ss}, \mathbf{p}_{sss}, \mathbf{p}_{ssss}$ by truncated Taylor-series arithmetic as described (order 4; fewer for lower degrees). Check regularity ($\sigma > 0$ on the subpath) and reject otherwise with an explicit exception.
  2. **Arc length and inversion**: composite Gauss–Legendre with refinement, safeguarded Newton inversion, tolerances as in the theory.
  3. **Stretch specification**: points (closest point, rejected beyond a tolerance), parameter fractions, arc-length fractions.
  4. **The 3D constant-speed trajectory** for a curved subpath: entry transition, stretch, exit transition. The stretch exposes evaluation of position and derivatives up to the needed order at any time (via inversion), and `ToPolynomialSegments`-like sampling is not required (it is not polynomial); give it a sampling method consistent with what the App already uses for trajectories. Transitions as in 0015 with the boundary states of the theory (full jets, not zero a/j/s). Straight subpaths keep using the 0015 path or the general one; the general one must reproduce the straight result (see tests).
  5. **Limits**: peaks $P_k$ over the subpath (grid + local refinement), admissible speed $v_{adm}$, explicit failure when $v_{cv} > v_{adm}$ with $v_{adm}$ and the binding limit in the message; transitions via the multi-component shortest-feasible search; whole trajectory verified.
  6. Tests (below). No App/UI change.
- Out of scope: UI, 3D pass-through, several consecutive stretches, any change to 1D.

## Acceptance criteria

- [ ] Parabola reference values of the theory (derivatives at $t = 0$ and $t = 1$, length, component peaks, $v_{adm} = 1.709976$) reproduced: derivatives to 1e-9, length to 1e-12 relative, peaks to 1e-5 (sampling).
- [ ] Inversion: for random $t^*$, $\ell^{-1}(\ell(t^*)) = t^*$ to 1e-12; at $T_{cv}$ the inversion returns $t_e$.
- [ ] Speed along the stretch is $v_{cv}$ to 1e-9 at many sampled times; the stretch stays on the original path (distance to the polynomial path below 1e-9).
- [ ] Continuity at both junctions: position, velocity, acceleration, jerk, snap between transition and stretch to 1e-9 (degree 9; fewer orders for lower degrees).
- [ ] A straight segment through the general code gives the same trajectory as the 0015 straight construction (1e-9): zero a/j/s at the junctions.
- [ ] $v_{cv} > v_{adm}$ fails explicitly; a subpath through a rest point ($\sigma = 0$) fails explicitly; limits on transitions as in 0015.
- [ ] `dotnet build` free of new warnings; all tests pass; the note states the count.

## Expected tests

- Derivatives against the reference values and against a finite-difference check on a second path (a degree-5 polynomial space curve).
- Quadrature and inversion tests (including a path with strongly varying $\sigma$).
- Junction continuity for degrees 5, 7, 9; the three specification forms giving the same stretch; limits per axis; rule 3 with a prescribed total.

## Notes for the author (educational)

Explain in the return note, in plain words: what the derivative with respect to arc length is, why a curved constant-speed motion still has acceleration (centripetal), why the speed limit on such a stretch cannot be repaired by the transitions, and how the series arithmetic avoids long symbolic formulas.

Return note: `coordination/inbox-docs/0016-core-3d-curved-constant-speed.md`, ending with "Suggested commits". Small targeted edits to `STATUS.md` only; do not touch `roadmap/`, `theory/`, `design/`.
