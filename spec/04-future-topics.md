# Future Topics

Not in the current scope, but tracked for later consideration.

## Orientation

- Euler angles.
- Quaternions.
- Rotation matrices.

## Time scaling

- Trajectory stretching.
- Trajectory compression.

## Multi-axis synchronization

- Synchronous end-time.
- Master axis.
- Slave axes.

## Path vs Trajectory

- Path = geometric definition.
- Trajectory = path + time law.

## Numerical Stability

- Conditioning.
- Stability.
- Floating point errors.

## Implementation notes carried over from the original brief

- Polynomial degrees other than three, five, seven and nine may be added if deemed useful during project development.
- Application type decided: desktop application in C# with WPF (see `01-requirements.md`, item 16).
- Readable documentation for learning is a hard requirement (the author's own stated need), not optional polish.
- Open question: derivatives usefulness — up to which degree is it actually useful to compute/use them?
- Orientation of a solid in three-dimensional space (see Orientation above).
- Technical tools for app development decided: C# for the calculation/application code, WPF for the UI, and a charting library chosen during implementation — see `01-requirements.md`, item 16.
