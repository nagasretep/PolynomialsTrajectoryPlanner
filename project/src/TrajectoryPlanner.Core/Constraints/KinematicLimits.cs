using TrajectoryPlanner.Core.Symbols;

namespace TrajectoryPlanner.Core.Constraints;

public class KinematicLimits
{
    public double? MaxVelocity { get; set; }
    public double? MaxAcceleration { get; set; }
    public double? MaxJerk { get; set; }
    public double? MaxSnap { get; set; }

    public bool HasVelocityLimit => MaxVelocity.HasValue;
    public bool HasAccelerationLimit => MaxAcceleration.HasValue;
    public bool HasJerkLimit => MaxJerk.HasValue;
    public bool HasSnapLimit => MaxSnap.HasValue;

    public static KinematicLimits None => new();

    public static KinematicLimits FromValues(double? vMax = null, double? aMax = null, double? jMax = null, double? sMax = null)
    {
        return new KinematicLimits
        {
            MaxVelocity = vMax,
            MaxAcceleration = aMax,
            MaxJerk = jMax,
            MaxSnap = sMax
        };
    }
}

public readonly struct ExtremaResult
{
    public double Minimum { get; }
    public double Maximum { get; }
    public double MinTime { get; }
    public double MaxTime { get; }

    public double MaxAbsolute => Math.Max(Math.Abs(Minimum), Math.Abs(Maximum));

    public ExtremaResult(double min, double max, double minTime, double maxTime)
    {
        Minimum = min;
        Maximum = max;
        MinTime = minTime;
        MaxTime = maxTime;
    }
}
