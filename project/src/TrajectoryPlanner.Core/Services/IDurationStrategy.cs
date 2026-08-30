using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Services;

public enum DurationMode
{
    UserSpecified,
    AutoDerivedProportional,
    AutoDerivedFromConstraints
}

public interface IDurationStrategy1D
{
    DurationMode Mode { get; }
    double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments);
}

public class UserSpecifiedDuration : IDurationStrategy1D
{
    private readonly double[] _durations;

    public UserSpecifiedDuration(double[] durations)
    {
        _durations = durations;
    }

    public DurationMode Mode => DurationMode.UserSpecified;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return _durations[segmentIndex];
    }
}

public class AutoDerivedProportionalDuration : IDurationStrategy1D
{
    private readonly double _totalDuration;

    public AutoDerivedProportionalDuration(double totalDuration)
    {
        _totalDuration = totalDuration;
    }

    public DurationMode Mode => DurationMode.AutoDerivedProportional;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return _totalDuration / totalSegments;
    }
}

public class ConstraintAwareDuration : IDurationStrategy1D
{
    private readonly KinematicLimits _limits;
    private readonly double _initialGuess;

    public ConstraintAwareDuration(KinematicLimits limits, double initialGuess = 1.0)
    {
        _limits = limits;
        _initialGuess = initialGuess;
    }

    public DurationMode Mode => DurationMode.AutoDerivedFromConstraints;

    public double ComputeDuration(int degree, State1D startState, State1D endState, int segmentIndex, int totalSegments)
    {
        return DurationSearcher.FindValidDuration(degree, startState, endState, _limits, _initialGuess);
    }
}
