using TrajectoryPlanner.Core.Constraints;

namespace TrajectoryPlanner.Core.ConstantVelocity;

/// <summary>
/// Rules 1-3 for the durations of the entry and exit transitions of a constant-velocity trajectory
/// (<c>theory/ConstantVelocitySegment.md</c>, "Durations Of The Entry And Exit Transitions In 1D"), shared by the 1D and
/// the 3D construction so that the logic exists once. The caller supplies how the "weight" of each transition is computed
/// (Rule 1: its shortest feasible duration, with limits; Rule 2: the mean-velocity duration), and the rest is common.
/// </summary>
internal static class TransitionDurationRules
{
    /// <summary>Relative tolerance when comparing the time left (Rule 3) with the minimum feasible one.</summary>
    private const double TotalTolerance = 1e-12;

    /// <param name="cvDuration">Duration T_cv of the constant-velocity stretch.</param>
    /// <param name="limitsActive">True if any limit is active (the weights are then shortest feasible durations).</param>
    /// <param name="entryWeight">Rule 1 / Rule 2 duration of the entry transition; evaluated only when needed.</param>
    /// <param name="exitWeight">Same for the exit transition.</param>
    public static (double Entry, double Exit) Resolve(
        double cvDuration,
        bool limitsActive,
        double? entryDuration,
        double? exitDuration,
        double? totalDuration,
        Func<double> entryWeight,
        Func<double> exitWeight)
    {
        if (totalDuration is null)
            return (entryDuration ?? entryWeight(), exitDuration ?? exitWeight());

        double remaining = totalDuration.Value - cvDuration;
        if (!(remaining > 0))
        {
            throw new ArgumentException(
                $"The total duration {totalDuration.Value:G6} must exceed the constant-velocity stretch duration T_cv = {cvDuration:G6}");
        }

        if (entryDuration is not null)
        {
            double exit = remaining - entryDuration.Value;
            RequireRemainderPositive(exit, "exit transition", entryDuration.Value, remaining);
            return (entryDuration.Value, exit);
        }

        if (exitDuration is not null)
        {
            double entry = remaining - exitDuration.Value;
            RequireRemainderPositive(entry, "entry transition", exitDuration.Value, remaining);
            return (entry, exitDuration.Value);
        }

        double sIn = entryWeight();
        double sOut = exitWeight();
        double weightSum = sIn + sOut;

        // With limits the weights are the shortest feasible durations: the split is feasible exactly
        // when the time left is at least their sum.
        if (limitsActive && remaining < weightSum * (1 - TotalTolerance))
        {
            throw new KinematicLimitsViolatedException(
                $"The total duration {totalDuration.Value:G6} s is too short for the active kinematic limits: " +
                $"the minimum feasible total is {cvDuration + weightSum:G6} s " +
                $"(constant-velocity stretch {cvDuration:G6} s + entry {sIn:G6} s + exit {sOut:G6} s).");
        }

        double splitEntry = remaining * sIn / weightSum;
        return (splitEntry, remaining - splitEntry);
    }

    private static void RequireRemainderPositive(double remainder, string label, double prescribed, double remaining)
    {
        if (!(remainder > 0))
        {
            throw new ArgumentException(
                $"The prescribed transition duration {prescribed:G6} s leaves no time for the {label}: " +
                $"the time left after the stretch is {remaining:G6} s");
        }
    }
}
