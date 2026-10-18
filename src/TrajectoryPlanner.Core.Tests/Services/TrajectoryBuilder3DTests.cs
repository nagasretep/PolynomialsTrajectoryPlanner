using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.ConstantVelocity;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.Core.Tests.Services;

/// <summary>Request 0017: the 3D trajectory builder (segments, blends, constant-speed stretches, part descriptors).</summary>
public class TrajectoryBuilder3DTests
{
    private static State3D S(double x, double y, double z, double vx = 0, double vy = 0, double vz = 0) =>
        new(new Point3D(x, y, z), new Point3D(vx, vy, vz), Point3D.Zero, Point3D.Zero, Point3D.Zero);

    private static void AssertPoint(Point3D expected, Point3D actual, double tolerance)
    {
        for (int i = 0; i < 3; i++)
            Assert.Equal(expected[i], actual[i], tolerance);
    }

    /// <summary>Three segments, a blend between the first two and a constant-speed stretch on the third: six pieces.</summary>
    private static TrajectoryBuilder3D MixedBuilder(KinematicLimits? limits = null)
    {
        var builder = new TrajectoryBuilder3D()
            .AddSegment(S(0, 0, 0), S(4, 2, 1, 1, 0.5, 0.25), 2.0)
            .AddSegment(S(4, 2, 1, 1, 0.5, 0.25), S(6, 5, 3, 0, 1, 1), 2.5)
            .AddSegment(S(6, 5, 3, 0, 1, 1), S(9, 5, 3), 2.0);
        builder.AddBlendBetween(0, 1, 0.7, 0.3, limits: limits);
        builder.AddConstantSpeed(3, StretchSpecification.FromParameterFractions(0.3, 0.7), 0.8, limits: limits);
        return builder;
    }

    // ---- mixed trajectory --------------------------------------------------------------------

    [Fact]
    public void MixedTrajectory_IsContinuousAtEveryJunction_UpToTheOrderOfTheDegree()
    {
        Trajectory3D trajectory = MixedBuilder().Build();

        Assert.Equal(6, trajectory.Segments.Count);
        for (int i = 1; i < trajectory.Segments.Count; i++)
        {
            State3D end = trajectory.Segments[i - 1].EvaluateState(trajectory.Segments[i - 1].Duration);
            State3D start = trajectory.Segments[i].EvaluateState(0);
            for (int order = 0; order <= 4; order++)
                AssertPoint(end.GetDerivative(order), start.GetDerivative(order), 1e-9);
        }
    }

    [Fact]
    public void Parts_DescribeThePiecesInOrder()
    {
        var builder = MixedBuilder();
        var parts = builder.Parts;

        Assert.Equal(
            new[] { PartKind.Segment, PartKind.Blend, PartKind.Segment, PartKind.CvEntry, PartKind.CvStretch, PartKind.CvExit },
            parts.Select(p => p.Kind).ToArray());
        Assert.Equal(new[] { 0, 0, 1, 2, 2, 2 }, parts.Select(p => p.OriginalSegmentIndex).ToArray());
        Assert.Equal(1, parts[1].SecondSegmentIndex);
        Assert.Equal("Blend 1-2", parts[1].Label);
        Assert.Equal("Stretch 3", parts[4].Label);

        double start = 0;
        foreach (var part in parts)
        {
            Assert.Equal(start, part.StartTime, 1e-12);
            start += part.Duration;
        }

        Assert.Equal(0.3 * 2.0 + 0.3 * 2.5, parts[1].InheritedDuration!.Value, 1e-12);
        Assert.False(parts[1].AdjustedByLimits);
        Assert.Equal(0.8, parts[4].StretchSpeed!.Value);
        Assert.True(parts[4].StretchLength > 0);
        Assert.Null(parts[4].AdmissibleSpeed); // no limits
        Assert.Equal(parts.Count, builder.Build().Parts.Count);
        Assert.Same(parts[0].Segment, builder.Build().Segments[0]);
    }

    [Fact]
    public void StretchPartCarriesTheAdmissibleSpeedAndTheBindingLimit()
    {
        var limits = KinematicLimits.FromValues(vMax: 5, aMax: 40, jMax: 400);

        var parts = MixedBuilder(limits).Parts;

        var stretch = parts[4];
        Assert.NotNull(stretch.AdmissibleSpeed);
        Assert.True(stretch.AdmissibleSpeed > 0.8);
        Assert.Contains(stretch.BindingLimit, new[] { "velocity", "acceleration", "jerk" });
    }

    [Fact]
    public void TheTrajectoryWithTheCurvedStretch_EvaluatesLikeTheStretchItself()
    {
        var builder = MixedBuilder();
        Trajectory3D trajectory = builder.Build();
        var stretchPart = builder.Parts[4];
        var stretch = (CurvedConstantSpeedStretch3D)stretchPart.Segment;

        for (int i = 0; i <= 20; i++)
        {
            double local = stretch.Duration * i / 20 * 0.999999;
            State3D inTrajectory = trajectory.EvaluateState(stretchPart.StartTime + local);
            State3D alone = stretch.EvaluateState(local);
            for (int order = 0; order <= 4; order++)
                AssertPoint(alone.GetDerivative(order), inTrajectory.GetDerivative(order), 1e-9);
        }

        AssertPoint(stretch.EvaluateDerivative(1.0, 2), trajectory.EvaluateDerivative(stretchPart.StartTime + 1.0, 2), 1e-9);
        AssertPoint(stretch.EvaluatePosition(1.0), trajectory.EvaluatePosition(stretchPart.StartTime + 1.0), 1e-9);
    }

    // ---- segments ----------------------------------------------------------------------------

    [Fact]
    public void NullDurationWithLimits_IsTheMultiComponentShortestFeasibleDuration()
    {
        var limits = KinematicLimits.FromValues(vMax: 2, aMax: 5);
        var start = S(0, 0, 0);
        var end = S(3, 1, 2);

        var trajectory = new TrajectoryBuilder3D().AddSegment(start, end, null, null, limits).Build();

        double expected = ShortestFeasibleDuration.Find(
            9, BlendBuilder3D.Components(start), BlendBuilder3D.Components(end), limits)!.Value;
        Assert.Equal(expected, trajectory.TotalDuration, 1e-12);
        Assert.True(ExtremaAnalyzer.VerifyLimits((PolynomialSegment3D)trajectory.Segments[0], limits));
        // The largest distance (3, on x) decides: the same as the 1D segment.
        Assert.Equal(ShortestFeasibleDuration.Find(9, State1D.FromPositionOnly(0), State1D.FromPositionOnly(3), limits)!.Value,
            trajectory.TotalDuration, 1e-12);
    }

    [Fact]
    public void NullDurationWithoutLimits_FollowsTheOneDimensionalRule_OneSecond()
    {
        var trajectory = new TrajectoryBuilder3D().AddSegment(S(0, 0, 0), S(3, 1, 2)).Build();

        Assert.Equal(SegmentFactory1D.DefaultDuration, trajectory.TotalDuration, 12);
        Assert.Equal(1.0, new TrajectoryBuilder3D().AddRestToRest(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Build().TotalDuration, 12);
    }

    [Fact]
    public void AGivenDuration_IsOnlyVerified()
    {
        var limits = KinematicLimits.FromValues(vMax: 2);

        var ok = new TrajectoryBuilder3D().AddSegment(S(0, 0, 0), S(3, 1, 2), 5.0, null, limits).Build();
        Assert.Equal(5.0, ok.TotalDuration, 12);

        Assert.Throws<KinematicLimitsViolatedException>(() =>
            new TrajectoryBuilder3D().AddSegment(S(0, 0, 0), S(3, 1, 2), 0.5, null, limits));
    }

    [Fact]
    public void AlongOneAxis_TheSegmentEqualsTheOneDimensionalBuilder()
    {
        var limits = KinematicLimits.FromValues(vMax: 2, aMax: 5, jMax: 40);
        var oneD = new TrajectoryBuilder1D().AddSegment(new State1D(0, 0.5, 0, 0, 0), new State1D(6, 0, 0, 0, 0), 2.0).Build();
        var threeD = new TrajectoryBuilder3D().AddSegment(S(0, 7, -1, 0.5), S(6, 7, -1), 2.0).Build();

        AssertSegmentsEqual(oneD.Segments[0], ((PolynomialSegment3D)threeD.Segments[0]).XAxis);

        // Shortest feasible duration too.
        var oneDAuto = SegmentFactory1D.Build(9, new State1D(0, 0, 0, 0, 0), new State1D(6, 0, 0, 0, 0), limits);
        var threeDAuto = (PolynomialSegment3D)new TrajectoryBuilder3D().AddSegment(S(0, 7, -1), S(6, 7, -1), null, null, limits).Build().Segments[0];
        AssertSegmentsEqual(oneDAuto, threeDAuto.XAxis);
    }

    private static void AssertSegmentsEqual(PolynomialSegment1D expected, PolynomialSegment1D actual)
    {
        Assert.Equal(expected.Duration, actual.Duration, 1e-9);
        for (int i = 0; i < expected.Coefficients.NormalizedCoefficients.Length; i++)
            Assert.Equal(expected.Coefficients.NormalizedCoefficients[i], actual.Coefficients.NormalizedCoefficients[i], 1e-9);
    }

    // ---- blends ----------------------------------------------------------------------------------

    [Fact]
    public void AlongOneAxis_TheBlendEqualsTheOneDimensionalBuilder()
    {
        var limits = KinematicLimits.FromValues(vMax: 3);
        var oneD = new TrajectoryBuilder1D()
            .AddSegment(new State1D(0, 0, 0, 0, 0), new State1D(2, 2, 0, 0, 0), 1.0)
            .AddSegment(new State1D(2, -2, 0, 0, 0), new State1D(0.5, 0, 0, 0, 0), 1.5);
        oneD.AddBlendBetween(0, 1, 0.7, 0.3, limits: limits);
        var threeD = new TrajectoryBuilder3D()
            .AddSegment(S(0, 3, 4), S(2, 3, 4, 2), 1.0)
            .AddSegment(S(2, 3, 4, -2), S(0.5, 3, 4), 1.5);
        threeD.AddBlendBetween(0, 1, 0.7, 0.3, limits: limits);

        var t1 = oneD.Build();
        var t3 = threeD.Build();

        Assert.Equal(3, t3.Segments.Count);
        for (int i = 0; i < 3; i++)
            AssertSegmentsEqual(t1.Segments[i], ((PolynomialSegment3D)t3.Segments[i]).XAxis);
    }

    [Fact]
    public void BlendByPoints_EqualsBlendByParameters_AndReportsTheAdjustment()
    {
        // A tight acceleration limit that the inherited blend violates but a longer blend can respect.
        var pre = S(0, 0, 0);
        var limits = KinematicLimits.FromValues(aMax: 12);
        TrajectoryBuilder3D Make() => new TrajectoryBuilder3D()
            .AddSegment(pre, S(2, 1, 0, 2, 1, 0), 1.0)
            .AddSegment(S(2, 1, 0, 0, 2, 1), S(2, 3, 2), 1.5);

        var byTau = Make();
        byTau.AddBlendBetween(0, 1, 0.7, 0.3, limits: limits);

        var first = new PolynomialSegment3D(9, pre, S(2, 1, 0, 2, 1, 0), 1.0);
        var second = new PolynomialSegment3D(9, S(2, 1, 0, 0, 2, 1), S(2, 3, 2), 1.5);
        var byPoint = Make();
        byPoint.AddBlendBetween(0, 1, first.EvaluatePosition(0.7), second.EvaluatePosition(0.3 * 1.5), limits: limits);

        Assert.Equal(byTau.Parts[1].Duration, byPoint.Parts[1].Duration, 1e-9);
        Assert.Equal(byTau.Parts[1].AdjustedByLimits, byPoint.Parts[1].AdjustedByLimits);
        Assert.True(ExtremaAnalyzer.VerifyLimits((PolynomialSegment3D)byTau.Parts[1].Segment, limits));
    }

    [Fact]
    public void BlendErrors_AreExplicit()
    {
        var b = MixedBuilder(); // pieces: Segment, Blend, Segment, CvEntry, CvStretch, CvExit

        Assert.Throws<ArgumentException>(() => b.AddBlendBetween(0, 2, 0.5, 0.5));            // not consecutive
        Assert.Throws<InvalidOperationException>(() => b.AddBlendBetween(0, 1, 0.5, 0.5));    // the second is a blend
        Assert.Throws<InvalidOperationException>(() => b.AddBlendBetween(2, 3, 0.5, 0.5));    // the second is an entry transition
        Assert.Throws<ArgumentException>(() => new TrajectoryBuilder3D().AddBlendBetween(0, 1, 0.5, 0.5));
    }

    // ---- constant speed --------------------------------------------------------------------------------

    [Fact]
    public void AlongOneAxis_TheConstantSpeedStretchEqualsTheOneDimensionalConstruction()
    {
        var limits = KinematicLimits.FromValues(vMax: 20, aMax: 40, jMax: 400);
        var line3 = new PolynomialSegment3D(9, S(0, 5, 6), S(100, 5, 6), 8.0);
        double xs = line3.EvaluatePosition(0.3 * 8).X, xe = line3.EvaluatePosition(0.7 * 8).X;
        var oneD = new ConstantVelocityTrajectory1D(new State1D(0, 0, 0, 0, 0), new State1D(100, 0, 0, 0, 0), xs, xe, 15, 9, limits);

        var builder = new TrajectoryBuilder3D().AddSegment(S(0, 5, 6), S(100, 5, 6), 8.0);
        builder.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.3, 0.7), 15, limits: limits);
        var parts = builder.Parts;

        Assert.Equal(oneD.TotalDuration, builder.Build().TotalDuration, 1e-9);
        AssertSegmentsEqual(oneD.EntryTransition, ((PolynomialSegment3D)parts[0].Segment).XAxis);
        AssertSegmentsEqual(oneD.ExitTransition, ((PolynomialSegment3D)parts[2].Segment).XAxis);
        Assert.Equal(oneD.ConstantVelocity.Duration, parts[1].Duration, 1e-9);
    }

    [Fact]
    public void TheThreeSpecifications_GiveTheSameConstantSpeedPieces()
    {
        var segment = new PolynomialSegment3D(9, S(0, 0, 0, 2, 1, 0.5), S(3, 2, 1.5, 0.5, 2, 2), 2.0);
        var whole = new TrajectoryPlanner.Core.ArcLength.ArcLengthPath(segment, 0, segment.Duration, requireRegular: false);

        TrajectoryBuilder3D Make(StretchSpecification spec)
        {
            var b = new TrajectoryBuilder3D().AddSegment(segment.StartState, segment.EndState, segment.Duration, 9);
            b.AddConstantSpeed(0, spec, 1.2);
            return b;
        }

        var byTau = Make(StretchSpecification.FromParameterFractions(0.25, 0.8));
        var byPoints = Make(StretchSpecification.FromPoints(segment.EvaluatePosition(0.5), segment.EvaluatePosition(1.6)));
        var byLength = Make(StretchSpecification.FromArcLengthFractions(
            whole.LengthAt(0.5) / whole.Length, whole.LengthAt(1.6) / whole.Length));

        foreach (var other in new[] { byPoints, byLength })
        {
            Assert.Equal(byTau.Build().TotalDuration, other.Build().TotalDuration, 1e-8);
            Assert.Equal(byTau.Parts[1].StretchLength!.Value, other.Parts[1].StretchLength!.Value, 1e-8);
        }
    }

    [Fact]
    public void ConstantSpeedErrors_AreExplicit()
    {
        var limits = KinematicLimits.FromValues(aMax: 8, jMax: 20);
        var b = new TrajectoryBuilder3D().AddSegment(S(0, 0, 0), S(8, 4, 2), 4.0);

        Assert.Throws<ArgumentOutOfRangeException>(() => b.AddConstantSpeed(3, StretchSpecification.FromParameterFractions(0.3, 0.7), 1));
        Assert.Throws<ArgumentException>(() => b.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.7, 0.3), 1));
        Assert.Throws<KinematicLimitsViolatedException>(() =>
            b.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.3, 0.7), 50, limits: KinematicLimits.FromValues(vMax: 2)));

        b.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.3, 0.7), 1.0);
        Assert.Throws<InvalidOperationException>(() => b.AddConstantSpeed(1, StretchSpecification.FromParameterFractions(0.3, 0.7), 1));
        Assert.Throws<InvalidOperationException>(() => new TrajectoryBuilder3D().Build());
    }

    [Fact]
    public void ConstantSpeedWithAPrescribedTotal_SplitsTheTimeLeft()
    {
        var b = new TrajectoryBuilder3D().AddSegment(S(0, 0, 0), S(8, 4, 2), 4.0);
        b.AddConstantSpeed(0, StretchSpecification.FromParameterFractions(0.3, 0.7), 1.0, totalDuration: 20.0);

        Assert.Equal(20.0, b.Build().TotalDuration, 1e-9);
        Assert.Equal(PartKind.CvStretch, b.Parts[1].Kind);
    }

    [Fact]
    public void PassThroughSegments_AreNumberedAsOriginalSegments()
    {
        var b = new TrajectoryBuilder3D().AddSegmentsFromWaypointsPassThrough(
            new[] { new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 1) }, durations: new[] { 1.0, 1.5 });

        Assert.Equal(new[] { 0, 1 }, b.Parts.Select(p => p.OriginalSegmentIndex).ToArray());
        Assert.Equal(2.5, b.Build().TotalDuration, 12);
    }
}
