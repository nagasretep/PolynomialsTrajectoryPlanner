# Data Model

This chapter summarizes the data entities involved in the project. Orientation is **not** considered yet in these models (see `04-future-topics.md` → Orientation); it will be added later.

This is a reference example for the data potentially involved in the project — it will be revised as the project progresses, as needed.

## Points

```
Point1D {
  x;   // mono-dimensional position value of a single point
}

Point3D {
  x;   // three-dimensional x position value of a single point
  y;   // three-dimensional y position value of a single point
  z;   // three-dimensional z position value of a single point
}
```

## Segments and Trajectories

A segment of a complex trajectory is itself a trajectory:

```
Segment1D {
  startPoint;          // Point1D
  endPoint;            // Point1D
  polynomialDegree;    // chosen among 3, 5, 7, 9 (and eventually other values)
  constraints[];        // settings of constraints applied to the segment
}

Segment3D {
  startPoint;          // Point3D
  endPoint;            // Point3D
  polynomialDegree;    // chosen among 3, 5, 7, 9 (and eventually other values)
  constraints[];        // settings of constraints applied to the segment
}

Trajectory1D {
  Segment1D[nn];              // array of Segment1D elements
  segmentEndpointRespect;     // yes/no — if yes, segment blending settings become irrelevant
}

Trajectory3D {
  Segment3D[nn];              // array of Segment3D elements
  segmentEndpointRespect;     // yes/no — if yes, segment blending settings become irrelevant
}
```

## Polynomial and boundary conditions

```
Polynomial {
  degree;              // chosen among three, five, seven, nine, etc.
  coefficients;
  boundaryConditions;
}

BoundaryConditions {
  startPosition;   endPosition;
  startVelocity;   endVelocity;
  startAcceleration; endAcceleration;
  startJerk;       endJerk;
  startSnap;       endSnap;
}
```

`BoundaryConditions` is crucial to the calculation of polynomial profiles of any degree — it must always be fully defined, without exceptions.

## Constraints and Blend

```
Constraints {
  constraintType;               // time, velocity, acceleration, jerk, snap, etc. (if applicable)
  constraintValue;               // value referred to the selected constraintType
  constantVelocitySelection;     // yes/no — activates the constant-velocity part of the segment
  constantVelocityValue;         // applied if constantVelocitySelection = yes
  constantVelocityInitialPos;    // Point3D — start of the constant-velocity part of the segment
  constantVelocityEndPos;        // Point3D — end of the constant-velocity part of the segment
}

Blend {
  nextSegmentBlendingPos;        // Point3D — where blending with the next segment starts
  previousSegmentBlendingPos;    // Point3D — where blending with the previous segment ends
}
```

> **Design note (confirmed distinction)**: `Constraints` and `Blend` are deliberately distinct concepts, kept separate:
> - **`Constraints`** describes kinematic limits/settings applied *within* a single segment (velocity/acceleration/jerk/snap limits, and the optional constant-velocity sub-part of the segment, via `constantVelocityInitialPos`/`constantVelocityEndPos`).
> - **`Blend`** describes the geometric continuity *between* adjacent segments — the raccordo (smoothing) at the junction points, via `nextSegmentBlendingPos`/`previousSegmentBlendingPos`.
>
> Relevant to requirements 12–14 in `01-requirements.md`.

## State (conceptual view)

The `State` element is an elementary entity of the trajectory concept: it specifies an endpoint (initial or final, relative to a single segment) and its relevant details.

```
State1D {
  position; velocity; acceleration; jerk; snap;
}
```

The same concept applies to `State3D`. `State` may be more relevant than `Point` when approaching the framework's motion-planning task.

## How the entities relate

- **`Constraints`** and **`Blend`** are project-specific requirements — their settings/values are internal parts of the whole project (see open note above).
- **`Trajectory`** is the top-level container that encompasses the entire data structure (all elements above, in terms of selections and values) leading to the determination of the overall movement profile — the purpose of the project.
