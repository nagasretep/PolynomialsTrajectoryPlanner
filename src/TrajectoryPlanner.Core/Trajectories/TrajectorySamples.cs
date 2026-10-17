namespace TrajectoryPlanner.Core.Trajectories;

/// <summary>Uniformly spaced samples of a trajectory: <c>Values[order][i]</c> is the derivative of order <c>order</c> (0 = position) at <c>Times[i]</c>.</summary>
public sealed record TrajectorySamples(double[] Times, double[][] Values);
