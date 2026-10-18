---
title: "Polynomial Trajectory Planner"
subtitle: "Short User Manual"
author: "Gianluca Castellari"
date: "October 2026"
---

# What The Program Does

The program builds polynomial motion profiles (degrees 3, 5, 7 and 9) and shows them as stacked plots of position, velocity, acceleration, jerk and snap, together with a report of the numbers behind them. It has **four modes**, chosen at the top of the window:

| Mode | Use it when |
|---|---|
| Blending | you describe one or more 1D segments one by one and want smooth joins between them (optionally with a constant-velocity stretch) |
| Pass-through | you give a list of 1D points and the path must pass exactly through all of them, as smoothly as possible |
| 3D | the same as Blending, with positions as `X, Y, Z` vectors, and a constant-speed stretch that may follow a curved path |
| 3D pass-through | the same as Pass-through, with `X, Y, Z` points |

Each mode keeps its own inputs and its last result: switching mode does not delete anything. The theory behind every number is in the theory library (`theory/`, or the PDF in `build/`).

# Starting And General Use

Start the program from Visual Studio or with `dotnet run --project src\TrajectoryPlanner.App`. The window has the inputs and the report in a narrow column on the left and the plots on the right.

1. Choose the mode.
2. Fill in the inputs. A box left empty means zero (for boundary derivatives) or "automatic" (for durations).
3. Press **Compute**. Nothing is computed while you type.
4. Read the status bar (notices and errors), the plots and the report tabs.
5. Move the mouse over a plot: a vertical cursor follows on all plots and the table beside them shows the values at that time.

**Numbers** are accepted with the decimal point or the decimal comma, never with a thousands separator. Tables and the status bar use the format of the computer.

**Errors** never produce a half-result: you get a message that names the segment, the axis, the field or the limit, and no trajectory. Fix the field and press Compute again.

**Degrees.** Degree 3 fixes position and velocity at both ends, degree 5 adds acceleration, degree 7 adds jerk, degree 9 adds snap. The boundary boxes shown depend on the degree; quantities that the degree does not support are not shown.

## Limits

Maximum velocity, acceleration, jerk and snap are optional. When a limit is active the program never silently exceeds it: either the result respects it, or you get an error. If you do not give a duration, the program looks for the **shortest duration that respects all active limits**. If you do give a duration, it is kept and only checked.

# Mode 1: Blending (1D)

**Inputs.** One tab per segment (**Add Segment** adds one that starts where the previous ends, **Remove Segment** removes the selected tab; at least one remains). Each segment has the degree, the start and end position, the boundary derivatives the degree needs, an optional duration and the optional limits.

**Constant-velocity stretch** (optional): switch it on, give the position where the stretch starts and ends and the speed. The program builds an entry transition, the stretch at exactly that speed, and an exit transition.

**Blend** (optional, needs two segments): *final blend value* on a segment and *initial blend value* on the next are **positions on those segments** where the smooth join starts and ends. The part of each segment beyond those positions is replaced by one blend polynomial that is continuous with both neighbours up to the highest derivative of the degree. A position outside the range covered by the segment is an error.

**Worked example.** One segment, degree 9, from 0 to 1, duration 1, all boundary derivatives empty (rest to rest). Expected maximum absolute values in the Extrema table:

| Quantity | Value |
|---|---|
| velocity | 2.4609 |
| acceleration | 9.3720 |
| jerk | 78.75 |
| snap | 622.53 |

Add a second segment from 1 to 3, give the first a *final blend value* of 0.7 and the second an *initial blend value* of 1.3: the plots show one smooth curve without a stop at 1, and the report shows the blend as a separate part.

# Mode 2: Pass-Through (1D)

Use this mode when the path **must** go exactly through every point.

**Inputs.**

- *Degree* and *smoothness order*: the order says which derivative the program makes as small as possible. The default is the right choice (for degree 9 it is snap); change it only to experiment.
- *End conditions*: rest (default) or values of velocity, acceleration, jerk and snap at the first and the last point.
- *Waypoint table*: the position of each point and, optionally, the duration of the segment that starts there (the last row's duration is greyed out). **Add Waypoint** and **Remove Waypoint** change the table; at least two points. Fill in either all durations or none.
- *Total duration* (optional, with blank durations): split among the segments in proportion to their distances.
- *Limits* (optional).

**What happens to durations.** All durations given: kept, only checked against the limits. Total given: split by distance, only checked. Nothing given and no limits: 1 second per segment. Nothing given and limits active: the program starts from the proportions of the distances and finds one scale factor $\lambda$ so that the whole timetable is as short as possible while respecting every limit; the proportions never change.

**Report tabs.** *Summary*: segment durations and total, $\lambda$, every active limit with its peak and the segment where the peak occurs (the limit at 100 % is the one that stops the timetable from being shorter), the smoothness cost. *Waypoints*: velocity, acceleration, jerk and snap at every point.

**Worked example.** Points 0, 1, 2, degree 9, no durations.

- No limits: total 2 s; the velocity at the middle point is 2.275.
- Limits velocity 2, acceleration 3, jerk 10, snap 50: $\lambda = 1.168271$, total 2.336542 s, the acceleration peak is exactly 3 (it is the limit that binds), the status bar reports $\lambda$.

# Mode 3: 3D

Inputs are **vectors**: every quantity that has three components is a row of three boxes `X | Y | Z`.

**Segments** work as in Blending: tabs, degree, start and end position, boundary vectors (position, velocity, acceleration, jerk, snap as the degree needs), optional duration. **Limits are per axis**: one value per quantity, applied to each of X, Y and Z separately.

**Constant-speed stretch** (optional). It keeps the **speed**, the length of the velocity vector, constant while the path may curve. Choose how to give its start and end:

- *Points*: absolute `X | Y | Z`, accepted if they lie on the path of the segment;
- *Parameter fractions*: from 0 to 1 along the segment's own time;
- *Arc-length fractions*: from 0 to 1 along the length of the path.

Entry and exit transitions are computed like in 1D. If the requested speed is higher than the **admissible speed** (the highest speed at which the curve respects the limits) you get an error that names the limit that binds.

**Blend** between two segments: fractions of the segment (default) or points.

**Output.** Stacked plots with three curves (X, Y, Z) and a *norm* checkbox for the length of the vector; a **Path** tab with the projections XY, XZ and YZ (equal scales, coloured by part); report tabs *Parts* (each piece with kind, duration, and for a curved stretch its length, speed and admissible speed), *Extrema 3D*, *Peaks vs limits* (per quantity and per axis, highest percentage first).

**Worked example (curved stretch).** One segment, degree 9, duration 1; start position (0, 0, 0), start velocity (1, 0, 0), start acceleration (0, 2, 0); end position (1, 1, 0), end velocity (1, 2, 0), end acceleration (0, 2, 0): this is the parabola (t, t², 0). Switch the stretch on, speed 1, *parameter fractions* 0 and 1. Expected: the stretch is 1.478943 s long and 1.478943 long (speed 1); entry and exit cover no distance and take 0.739471 s each; total 2.957886 s. With maximum acceleration 8 and maximum jerk 20, a speed of 1.8 is refused: the admissible speed is 1.70998 and the jerk limit binds.

# Mode 4: 3D Pass-Through

The same as Mode 2, with `X | Y | Z` points and vector end conditions. Durations are **common to the three axes** (they are the same time axis), distributed by the lengths of the straight lines between the points. When limits are active, **one** factor $\lambda$ is applied to all segments and all axes, so the shape of the motion is kept and only the timing changes. The report names the **binding quantity and axis**: the one pair, such as "velocity on axis x", that stops the whole motion from being faster. To speed up the motion, relieve that pair: a higher limit on that axis, or more distance in time for the segment where that axis moves.

**Worked example.** Points (0, 0, 0), (1, 0, 0), (1, 3, 0), (1, 3, 4) are already in the table. With limits velocity 2, acceleration 3, jerk 10, snap 50: $\lambda = 4.990502$, segment durations 1.871438, 5.614315 and 7.485753, total 14.971506 s, binding: velocity on axis x. Without limits: total 3 s, durations 0.375, 1.125 and 1.5.

# Saving And Opening Projects

**Save** writes the inputs of **all four modes** and the selected mode to a file with extension `.ptp.json`; **Open** loads one. The file contains what you typed, box by box, not the results: after opening, press **Compute** again. The title of the window shows the file name.

- The file is plain text (JSON); you can read it in any editor or keep it in git.
- Boxes are stored as typed, so a decimal comma stays a decimal comma and the computer's language does not matter.
- Fields that are missing in a file take their default; unknown fields are ignored.
- A file that cannot be read (missing, not JSON, wrong format, a `version` newer than the program supports, a field of the wrong type, no segments) is refused with a message that names the file and the reason, and your current inputs are left untouched.

A minimal example of the layout (shortened):

```
{ "format": "PolynomialsTrajectoryPlanner.Project", "version": 1, "mode": "ThreeDPassThrough",
  "blending": { ... }, "passThrough": { ... }, "threeD": { ... },
  "threeDPassThrough": { "degree": 9, "waypoints": [ { "x": "0", "y": "0", "z": "0", "duration": "" }, ... ] } }
```

# Common Messages

| Message says | What to do |
|---|---|
| a duration column partly filled | fill all durations or clear them all |
| durations and total both given | clear one of them |
| a prescribed duration violates a limit (names segment, axis, quantity) | give a longer duration, a higher limit, or clear the duration to let the program find it |
| blend position outside the segment | choose a position in the range the segment covers (the message states it) |
| stretch point not on the path | use fractions instead of points, or correct the point (the message gives the distance) |
| constant speed exceeds the admissible speed | lower the speed, or relax the limit that the message names |
| (3D) a segment with a stretch cannot be blended | remove the stretch or the blend values |
| $\lambda$ applied | information: the shortest timetable that respects the limits was used |
