using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.States;

namespace TrajectoryPlanner.Core.Tests.Blending;

/// <summary>Request 0015, Part A: 3D blending (common duration, component-wise coefficients, per-axis limits).</summary>
public class BlendBuilder3DTests
{
    private static State3D S(double x, double y, double z, double vx = 0, double vy = 0, double vz = 0) =>
        new(new Point3D(x, y, z), new Point3D(vx, vy, vz), Point3D.Zero, Point3D.Zero, Point3D.Zero);

    /// <summary>Two neighbours along different directions that meet at (2, 1, 0).</summary>
    private static (PolynomialSegment3D Pre, PolynomialSegment3D Post) Corner() =>
        (new PolynomialSegment3D(9, S(0, 0, 0), S(2, 1, 0, 2, 1, 0), 1.0),
         new PolynomialSegment3D(9, S(2, 1, 0, 0, 2, 1), S(2, 3, 2), 1.5));

    private static void AssertStatesEqual(State3D expected, State3D actual, double tolerance = 1e-9)
    {
        for (int order = 0; order <= 4; order++)
            for (int axis = 0; axis < 3; axis++)
                Assert.Equal(expected.GetDerivative(order)[axis], actual.GetDerivative(order)[axis], tolerance);
    }

    // ---- continuity and equivalence with 1D -------------------------------------------------

    [Fact]
    public void TheBlend_IsContinuousWithBothNeighbours_UpToTheDegreeOrder()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3);

        AssertStatesEqual(pre.EvaluateState(0.7 * pre.Duration), blend.EvaluateState(0));
        AssertStatesEqual(post.EvaluateState(0.3 * post.Duration), blend.EvaluateState(blend.Duration));
        Assert.Equal(0.3 * 1.0 + 0.3 * 1.5, blend.InheritedDuration, 12);
        Assert.Equal(blend.InheritedDuration, blend.Duration, 12);
        Assert.False(blend.AdjustedByLimits);
    }

    [Fact]
    public void EachComponent_EqualsTheOneDimensionalBlendWithTheSameDuration()
    {
        var (pre, post) = Corner();
        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3);

        var axes = new (PolynomialSegment1D Pre, PolynomialSegment1D Post, PolynomialSegment1D Of3D)[]
        {
            (pre.XAxis, post.XAxis, blend.XAxis), (pre.YAxis, post.YAxis, blend.YAxis), (pre.ZAxis, post.ZAxis, blend.ZAxis)
        };
        foreach (var (p1, q1, b3) in axes)
        {
            var oneD = BlendBuilder1D.BuildBlendSegment(p1, q1, 0.7, 0.3, blendDuration: blend.Duration);
            for (int i = 0; i < b3.Coefficients.NormalizedCoefficients.Length; i++)
                Assert.Equal(oneD.Coefficients.NormalizedCoefficients[i], b3.Coefficients.NormalizedCoefficients[i], 1e-9);
        }
    }

    [Fact]
    public void AxisAlignedCase_ReproducesTheOneDimensionalNumbers()
    {
        // The 1D case of BlendDurationInformationTests, on the x axis; y and z do not move.
        var pre = new PolynomialSegment3D(9, S(0, 5, -1), S(2, 5, -1, 2, 0, 0), 1.0);
        var post = new PolynomialSegment3D(9, S(2, 5, -1, -2, 0, 0), S(0.5, 5, -1), 1.5);
        var pre1 = new PolynomialSegment1D(9, State1D.FromPositionOnly(0), new State1D(2, 2, 0, 0, 0), 1.0);
        var post1 = new PolynomialSegment1D(9, new State1D(2, -2, 0, 0, 0), State1D.FromPositionOnly(0.5), 1.5);

        var blend3 = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3);
        var blend1 = BlendBuilder1D.BuildBlendSegment(pre1, post1, 0.7, 0.3);

        Assert.Equal(blend1.Duration, blend3.Duration, 12);
        Assert.Equal(0.75, blend3.Duration, 12);
        for (int i = 0; i <= 9; i++)
            Assert.Equal(blend1.Coefficients.NormalizedCoefficients[i], blend3.XAxis.Coefficients.NormalizedCoefficients[i], 1e-9);
        Assert.Equal(5.0, blend3.EvaluatePosition(0.3).Y, 12); // the quiet axes stay where they are
        Assert.Equal(-1.0, blend3.EvaluatePosition(0.3).Z, 12);
    }

    [Fact]
    public void OppositeDirections_AreBlended()
    {
        var pre = new PolynomialSegment3D(9, S(0, 0, 0), S(2, 0, 0, 2, 0, 0), 1.0);
        var post = new PolynomialSegment3D(9, S(2, 0, 0, -2, 0, 0), S(0, 0, 0), 1.0);

        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.6, 0.4);

        AssertStatesEqual(pre.EvaluateState(0.6), blend.EvaluateState(0));
        AssertStatesEqual(post.EvaluateState(0.4), blend.EvaluateState(blend.Duration));
    }

    [Fact]
    public void DefaultDegree_FollowsTheNeighbours_NineWhenTheyDiffer()
    {
        var (pre, post) = Corner();
        Assert.Equal(9, BlendBuilder3D.DefaultBlendDegree(pre, post));

        var cubic = new PolynomialSegment3D(3, S(0, 0, 0), S(1, 1, 1, 1, 0, 0), 1.0);
        var cubic2 = new PolynomialSegment3D(3, S(1, 1, 1, 0, 1, 0), S(2, 2, 2), 1.0);
        Assert.Equal(3, BlendBuilder3D.DefaultBlendDegree(cubic, cubic2));
        Assert.Equal(9, BlendBuilder3D.DefaultBlendDegree(cubic, post));
        Assert.Equal(3, BlendBuilder3D.BuildBlendSegment(cubic, cubic2, 0.5, 0.5).Degree);
        Assert.Equal(5, BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDegree: 5).Degree);
    }

    // ---- limits -----------------------------------------------------------------------------

    private static double PeakVelocity(PolynomialSegment3D s) =>
        new[] { s.XAxis, s.YAxis, s.ZAxis }.Max(a => ExtremaAnalyzer.VelocityExtrema(a).MaxAbsolute);

    [Fact]
    public void LimitsTheInheritedBlendRespects_KeepTheInheritedDuration()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3, limits: KinematicLimits.FromValues(vMax: 8));

        Assert.False(blend.AdjustedByLimits);
        Assert.Equal(blend.InheritedDuration, blend.Duration, 12);
    }

    [Fact]
    public void ALimitTheInheritedBlendViolates_ShortestFeasibleDurationIsUsed_AndEveryAxisRespectsIt()
    {
        var (pre, post) = Corner();

        // A velocity limit just below the peak of the inherited blend. Not every window admits one (the boundary velocities
        // are fixed, so a limit that is too low is impossible for any duration): take the first (window, limit) that does.
        double ts = 0, te = 0;
        BlendSegment3D? inherited = null;
        KinematicLimits? limits = null;
        foreach (var (a, b) in new[] { (0.3, 0.7), (0.5, 0.5), (0.2, 0.2), (0.6, 0.3), (0.1, 0.9) })
        {
            var candidate = BlendBuilder3D.BuildBlendSegment(pre, post, a, b);
            foreach (double fraction in new[] { 0.95, 0.9, 0.8 })
            {
                var trial = KinematicLimits.FromValues(vMax: fraction * PeakVelocity(candidate));
                try
                {
                    var attempt = BlendBuilder3D.BuildBlendSegment(pre, post, a, b, limits: trial);
                    if (!attempt.AdjustedByLimits)
                        continue;
                }
                catch (DurationSearchFailedException)
                {
                    continue;
                }

                (ts, te, inherited, limits) = (a, b, candidate, trial);
                break;
            }

            if (inherited is not null)
                break;
        }

        Assert.NotNull(inherited);
        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, ts, te, limits: limits);

        Assert.True(blend.AdjustedByLimits);
        Assert.Equal(inherited.Duration, blend.InheritedDuration, 12);
        Assert.NotEqual(blend.InheritedDuration, blend.Duration);
        Assert.True(ExtremaAnalyzer.VerifyLimits(blend, limits!));
        // Shortest: a hair shorter violates.
        var shorter = new PolynomialSegment3D(9, blend.StartState, blend.EndState, blend.Duration * (1 - 1e-6));
        Assert.False(ExtremaAnalyzer.VerifyLimits(shorter, limits!));
    }

    [Fact]
    public void ALimitBindingOnOneAxisOnly_GivesTheDurationOfThatAxis()
    {
        // Only x moves fast: its velocity decides, y and z have a lot of margin.
        var pre = new PolynomialSegment3D(9, S(0, 0, 0), S(2, 0.1, 0.1, 2, 0.1, 0.1), 1.0);
        var post = new PolynomialSegment3D(9, S(2, 0.1, 0.1, -2, 0.1, 0.1), S(0.5, 0.2, 0.2), 1.5);
        var inherited = BlendBuilder3D.BuildBlendSegment(pre, post, 0.3, 0.7);
        double xPeak = ExtremaAnalyzer.VelocityExtrema(inherited.XAxis).MaxAbsolute;
        double yPeak = ExtremaAnalyzer.VelocityExtrema(inherited.YAxis).MaxAbsolute;
        Assert.True(xPeak > 5 * yPeak);
        var limits = KinematicLimits.FromValues(vMax: 0.85 * xPeak);

        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.3, 0.7, limits: limits);

        double xOnly = ShortestFeasibleDuration.Find(9, inherited.StartState.GetComponent(0), inherited.EndState.GetComponent(0), limits)!.Value;
        Assert.Equal(xOnly, blend.Duration, 1e-9);
        Assert.True(blend.AdjustedByLimits);
    }

    [Fact]
    public void PrescribedDuration_IsKeptAndOnlyVerified()
    {
        var (pre, post) = Corner();

        var blend = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 1.7, limits: KinematicLimits.FromValues(vMax: 100));
        Assert.Equal(1.7, blend.Duration, 12);
        Assert.False(blend.AdjustedByLimits);

        Assert.Throws<KinematicLimitsViolatedException>(() =>
            BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3, blendDuration: 0.05, limits: KinematicLimits.FromValues(vMax: 1)));
    }

    // ---- shortest feasible duration for several components -----------------------------------

    [Fact]
    public void MultiComponentRestToRest_UsesTheLargestDistance()
    {
        var limits = KinematicLimits.FromValues(vMax: 2, aMax: 5);
        var zero = State1D.FromPositionOnly(0);

        double? three = ShortestFeasibleDuration.Find(
            9, new[] { zero, zero, zero }, new[] { State1D.FromPositionOnly(1), State1D.FromPositionOnly(3), State1D.FromPositionOnly(0.5) }, limits);
        double? single = ShortestFeasibleDuration.Find(9, zero, State1D.FromPositionOnly(3), limits);

        Assert.Equal(single!.Value, three!.Value, 12);
    }

    [Fact]
    public void MultiComponent_AllWithoutMotion_DeterminesNothing()
    {
        var zero = State1D.FromPositionOnly(0);

        Assert.Null(ShortestFeasibleDuration.Find(9, new[] { zero, zero }, new[] { zero, zero }, KinematicLimits.FromValues(vMax: 1)));
    }

    // ---- absolute points ---------------------------------------------------------------------

    [Fact]
    public void AbsolutePointsOnTheSegments_GiveTheSameBlendAsTheParameters()
    {
        var (pre, post) = Corner();
        var byTau = BlendBuilder3D.BuildBlendSegment(pre, post, 0.7, 0.3);

        var byPoint = BlendBuilder3D.BuildBlendSegment(
            pre, post, pre.EvaluatePosition(0.7 * pre.Duration), post.EvaluatePosition(0.3 * post.Duration));

        Assert.Equal(byTau.Duration, byPoint.Duration, 1e-9);
        for (int axis = 0; axis < 3; axis++)
            for (int order = 0; order <= 4; order++)
                for (double t = 0; t <= byTau.Duration; t += byTau.Duration / 4)
                    Assert.Equal(byTau.EvaluateDerivative(t, order)[axis], byPoint.EvaluateDerivative(t, order)[axis], 1e-9);
    }

    [Fact]
    public void TauForPoint_RecoversTheParameter_ToMachinePrecision()
    {
        var (pre, _) = Corner();
        foreach (double tau in new[] { 0.05, 0.3, 0.5, 0.77, 0.95 })
        {
            double found = BlendBuilder3D.TauForPoint(pre, pre.EvaluatePosition(tau * pre.Duration));
            Assert.Equal(tau, found, 1e-9);
        }
    }

    [Fact]
    public void AnAbsolutePointOffTheSegment_IsRejected_NamingTheDistance()
    {
        var (pre, post) = Corner();
        Point3D off = pre.EvaluatePosition(0.7 * pre.Duration) + new Point3D(0, 0, 0.01);

        var ex = Assert.Throws<ArgumentException>(() =>
            BlendBuilder3D.BuildBlendSegment(pre, post, off, post.EvaluatePosition(0.3 * post.Duration)));

        Assert.Contains("blend start", ex.Message);
        Assert.Contains("distance", ex.Message);
    }

    // ---- degenerate windows -------------------------------------------------------------------

    [Fact]
    public void EmptyWindow_IsRejected()
    {
        var (pre, post) = Corner();

        Assert.Throws<ArgumentException>(() => BlendBuilder3D.BuildBlendSegment(pre, post, 1.0, 0.0));
    }

    [Fact]
    public void RemovingAWholeNeighbour_IsRejected_ByThePairMethod()
    {
        var (pre, post) = Corner();

        Assert.Throws<ArgumentException>(() => BlendBuilder3D.BuildBlendedSegmentPair(pre, post, 0.0, 0.3));
        Assert.Throws<ArgumentException>(() => BlendBuilder3D.BuildBlendedSegmentPair(pre, post, 0.7, 1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => BlendBuilder3D.BuildBlendSegment(pre, post, -0.1, 0.3));
    }

    [Fact]
    public void ThePair_HasTruncatedNeighboursThatMeetTheBlend()
    {
        var (pre, post) = Corner();

        var (truncPre, blend, truncPost) = BlendBuilder3D.BuildBlendedSegmentPair(pre, post, 0.7, 0.3);

        Assert.Equal(0.7, truncPre.Duration, 12);
        Assert.Equal(1.05, truncPost.Duration, 12);
        AssertStatesEqual(truncPre.EvaluateState(truncPre.Duration), blend.EvaluateState(0));
        AssertStatesEqual(blend.EvaluateState(blend.Duration), truncPost.EvaluateState(0));
        AssertStatesEqual(post.EndState, truncPost.EvaluateState(truncPost.Duration));
    }
}
