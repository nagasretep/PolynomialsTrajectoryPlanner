using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.PassThrough;

/// <summary>
/// Exact pass-through of 1D waypoints: the interior derivatives are the minimizer of the smoothness
/// cost of order m (default: snap for degree 9), found by one linear solve. Formulation:
/// <c>theory/TrajectoryPassingThroughConstraintPoints.md</c>, "Global Minimum-Snap Formulation In 1D".
/// Rest at the two global ends (all derivatives of order 1 .. p-1 are zero there).
/// </summary>
public class PassThroughSolver1D
{
    private readonly SmoothnessCriterion? _criterion;

    /// <param name="criterion">Smoothness order m; null selects <see cref="SmoothnessCriterion.DefaultForDegree"/>.</param>
    public PassThroughSolver1D(SmoothnessCriterion? criterion = null)
    {
        _criterion = criterion;
    }

    public SmoothnessCriterion CriterionFor(int degree) => _criterion ?? SmoothnessCriterion.DefaultForDegree(degree);

    public PolynomialSegment1D[] Solve(double[] waypoints, double[] segmentDurations, int degree = 9)
    {
        var problem = PassThroughProblem1D.Build(waypoints, segmentDurations, degree, CriterionFor(degree));
        var y = problem.Solve();
        double[][] nodes = problem.NodeDerivatives(waypoints, y);

        int segmentCount = waypoints.Length - 1;
        var segments = new PolynomialSegment1D[segmentCount];
        for (int k = 0; k < segmentCount; k++)
        {
            segments[k] = new PolynomialSegment1D(
                degree,
                ToState(nodes[k]),
                ToState(nodes[k + 1]),
                segmentDurations[k]);
        }

        return segments;
    }

    private static State1D ToState(double[] derivatives)
    {
        double At(int order) => order < derivatives.Length ? derivatives[order] : 0.0;
        return new State1D(At(0), At(1), At(2), At(3), At(4));
    }
}
