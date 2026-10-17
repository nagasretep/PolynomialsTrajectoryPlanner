using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanner.Core.CoefficientDetermination;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.PassThrough;

public interface ISmoothnessCriterion
{
    string Name { get; }
    double ComputeCost(PolynomialSegment1D[] segments);
    int DerivativeWeight { get; }
}

public class MinimumSnapCriterion : ISmoothnessCriterion
{
    public string Name => "Minimum-Snap";
    public int DerivativeWeight => 4;

    public double ComputeCost(PolynomialSegment1D[] segments)
    {
        double cost = 0;
        foreach (var seg in segments)
        {
            int samples = 100;
            double integral = 0;
            double dt = seg.Duration / samples;
            for (int i = 0; i <= samples; i++)
            {
                double t = i * dt;
                double snap = seg.EvaluateDerivative(t, 4);
                double w = (i == 0 || i == samples) ? 0.5 : 1.0;
                integral += w * snap * snap;
            }
            cost += integral * dt;
        }
        return cost;
    }
}

public class MinimumJerkCriterion : ISmoothnessCriterion
{
    public string Name => "Minimum-Jerk";
    public int DerivativeWeight => 3;

    public double ComputeCost(PolynomialSegment1D[] segments)
    {
        double cost = 0;
        foreach (var seg in segments)
        {
            int samples = 100;
            double integral = 0;
            double dt = seg.Duration / samples;
            for (int i = 0; i <= samples; i++)
            {
                double t = i * dt;
                double jerk = seg.EvaluateDerivative(t, 3);
                double w = (i == 0 || i == samples) ? 0.5 : 1.0;
                integral += w * jerk * jerk;
            }
            cost += integral * dt;
        }
        return cost;
    }
}

public class PassThroughSolver1D
{
    private readonly ISmoothnessCriterion _criterion;

    public PassThroughSolver1D(ISmoothnessCriterion? criterion = null)
    {
        _criterion = criterion ?? new MinimumSnapCriterion();
    }

    public PolynomialSegment1D[] Solve(double[] waypoints, double[] segmentDurations, int degree = 9)
    {
        if (waypoints.Length < 2)
            throw new ArgumentException("At least 2 waypoints required", nameof(waypoints));
        if (segmentDurations.Length != waypoints.Length - 1)
            throw new ArgumentException("segmentDurations length must be waypoints.Length - 1");

        int N = waypoints.Length;
        int M = N - 1;

        double[] velocities = new double[N];
        double[] accelerations = new double[N];
        double[] jerks = new double[N];
        double[] snaps = new double[N];

        velocities[0] = 0; velocities[N - 1] = 0;
        accelerations[0] = 0; accelerations[N - 1] = 0;
        jerks[0] = 0; jerks[N - 1] = 0;
        snaps[0] = 0; snaps[N - 1] = 0;

        SolveInteriorStates(waypoints, segmentDurations, degree,
            ref velocities, ref accelerations, ref jerks, ref snaps);

        var segments = new PolynomialSegment1D[M];
        for (int k = 0; k < M; k++)
        {
            var startState = new State1D(waypoints[k], velocities[k], accelerations[k], jerks[k], snaps[k]);
            var endState = new State1D(waypoints[k + 1], velocities[k + 1], accelerations[k + 1], jerks[k + 1], snaps[k + 1]);
            segments[k] = new PolynomialSegment1D(degree, startState, endState, segmentDurations[k]);
        }

        return segments;
    }

    public PolynomialSegment1D[] SolveWithAutoDurations(double[] waypoints, int degree = 9, double? globalDuration = null)
    {
        int M = waypoints.Length - 1;
        double[] durations = new double[M];

        double totalDistance = 0;
        double[] segDistances = new double[M];
        for (int k = 0; k < M; k++)
        {
            segDistances[k] = Math.Abs(waypoints[k + 1] - waypoints[k]);
            totalDistance += segDistances[k];
        }

        double totalT = globalDuration ?? Math.Max(1.0, totalDistance);
        for (int k = 0; k < M; k++)
        {
            durations[k] = totalDistance > 0
                ? Math.Max(0.05, totalT * segDistances[k] / totalDistance)
                : totalT / M;
        }

        return Solve(waypoints, durations, degree);
    }

    private void SolveInteriorStates(
        double[] waypoints, double[] durations, int degree,
        ref double[] v, ref double[] a, ref double[] j, ref double[] s)
    {
        int N = waypoints.Length;

        if (_criterion.DerivativeWeight == 4)
        {
            SolveMinimumSnapInterior(waypoints, durations, ref v, ref a, ref j, ref s);
        }
        else if (_criterion.DerivativeWeight == 3)
        {
            SolveMinimumJerkInterior(waypoints, durations, ref v, ref a, ref j, ref s);
        }
        else
        {
            SolveFiniteDifferenceHeuristic(waypoints, durations, ref v, ref a, ref j, ref s);
        }
    }

    private void SolveMinimumSnapInterior(
        double[] waypoints, double[] durations,
        ref double[] v, ref double[] a, ref double[] j, ref double[] s)
    {
        int N = waypoints.Length;
        if (N <= 2) return;
        int interiorCount = N - 2;
        int interiorVars = interiorCount * 4;

        if (interiorVars <= 0) return;

        double[] finiteV = new double[N];
        double[] finiteA = new double[N];
        double[] finiteJ = new double[N];

        SolveFiniteDifferenceHeuristic(waypoints, durations, ref v, ref finiteA, ref finiteJ, ref s);
    }

    private void SolveMinimumJerkInterior(
        double[] waypoints, double[] durations,
        ref double[] v, ref double[] a, ref double[] j, ref double[] s)
    {
        SolveFiniteDifferenceHeuristic(waypoints, durations, ref v, ref a, ref j, ref s);
    }

    private void SolveFiniteDifferenceHeuristic(
        double[] waypoints, double[] durations,
        ref double[] v, ref double[] a, ref double[] j, ref double[] s)
    {
        int N = waypoints.Length;
        double[] times = new double[N];
        for (int i = 1; i < N; i++)
            times[i] = times[i - 1] + durations[i - 1];

        for (int i = 1; i < N - 1; i++)
        {
            double dx = waypoints[i + 1] - waypoints[i - 1];
            double dt = times[i + 1] - times[i - 1];
            if (dt > 1e-12) v[i] = dx / dt;
        }

        for (int i = 1; i < N - 1; i++)
        {
            double dv = v[i + 1] - v[i - 1];
            double dt = times[i + 1] - times[i - 1];
            if (dt > 1e-12) a[i] = dv / dt;
        }

        for (int i = 1; i < N - 1; i++)
        {
            double da = a[i + 1] - a[i - 1];
            double dt = times[i + 1] - times[i - 1];
            if (dt > 1e-12) j[i] = da / dt;
        }

        for (int i = 1; i < N - 1; i++)
        {
            double dj = j[i + 1] - j[i - 1];
            double dt = times[i + 1] - times[i - 1];
            if (dt > 1e-12) s[i] = dj / dt;
        }
    }
}

public static class DurationStrategies
{
    public static double[] ProportionalToDistance(double[] waypoints, double totalDuration)
    {
        int M = waypoints.Length - 1;
        double[] durations = new double[M];
        double[] dists = new double[M];
        double totalDist = 0;

        for (int k = 0; k < M; k++)
        {
            dists[k] = Math.Abs(waypoints[k + 1] - waypoints[k]);
            totalDist += dists[k];
        }

        for (int k = 0; k < M; k++)
        {
            durations[k] = totalDist > 1e-12
                ? Math.Max(0.05, totalDuration * dists[k] / totalDist)
                : totalDuration / M;
        }

        return durations;
    }

    public static double[] ProportionalToDistance3D(Point3D[] waypoints, double totalDuration)
    {
        int M = waypoints.Length - 1;
        double[] durations = new double[M];
        double[] dists = new double[M];
        double totalDist = 0;

        for (int k = 0; k < M; k++)
        {
            dists[k] = waypoints[k].DistanceTo(waypoints[k + 1]);
            totalDist += dists[k];
        }

        for (int k = 0; k < M; k++)
        {
            durations[k] = totalDist > 1e-12
                ? Math.Max(0.05, totalDuration * dists[k] / totalDist)
                : totalDuration / M;
        }

        return durations;
    }
}
