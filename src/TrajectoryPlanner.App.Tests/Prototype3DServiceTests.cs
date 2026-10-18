using System.Globalization;
using TrajectoryPlanner.Core.ArcLength;
using TrajectoryPlanner.Core.Blending;
using TrajectoryPlanner.Core.Constraints;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.Segments;
using TrajectoryPlanner.Core.Services;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Trajectories;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0018: the 3D service (vector inputs, builder, analysis), without a window.</summary>
public class Prototype3DServiceTests
{
    private readonly Prototype3DTrajectoryService _service = new();

    private static string N(double v) => v.ToString("R", CultureInfo.CurrentCulture);

    private static void Set(VectorInput v, double x, double y, double z)
    {
        v.X = N(x);
        v.Y = N(y);
        v.Z = N(z);
    }

    private static Segment3DInputModel Segment(int number, int degree, (double, double, double) start, (double, double, double) end, string duration = "")
    {
        var m = new Segment3DInputModel(number) { Degree = degree, Duration = duration };
        Set(m.StartPosition, start.Item1, start.Item2, start.Item3);
        Set(m.EndPosition, end.Item1, end.Item2, end.Item3);
        return m;
    }

    /// <summary>A genuinely curved degree-9 segment (velocities and accelerations not along the chord).</summary>
    private static Segment3DInputModel Curve(int number = 1)
    {
        var m = Segment(number, 9, (0, 0, 0), (3, 2, 1.5), "2");
        Set(m.StartVelocity, 2, 1, 0.5);
        Set(m.StartAcceleration, 0, 3, 1);
        Set(m.EndVelocity, 0.5, 2, 2);
        Set(m.EndAcceleration, -1, 0, 2);
        return m;
    }

    private static PolynomialSegment3D CurveSegment() => new(
        9,
        new State3D(new Point3D(0, 0, 0), new Point3D(2, 1, 0.5), new Point3D(0, 3, 1), Point3D.Zero, Point3D.Zero),
        new State3D(new Point3D(3, 2, 1.5), new Point3D(0.5, 2, 2), new Point3D(-1, 0, 2), Point3D.Zero, Point3D.Zero),
        2.0);

    /// <summary>The theory parabola (t, t², 0), t in [0, 1], entered as a degree-9 segment.</summary>
    private static Segment3DInputModel Parabola()
    {
        var m = Segment(1, 9, (0, 0, 0), (1, 1, 0), "1");
        Set(m.StartVelocity, 1, 0, 0);
        Set(m.StartAcceleration, 0, 2, 0);
        Set(m.EndVelocity, 1, 2, 0);
        Set(m.EndAcceleration, 0, 2, 0);
        return m;
    }

    private static T InCulture<T>(string culture, Func<T> func)
    {
        var saved = (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture);
        try
        {
            Thread.CurrentThread.CurrentCulture = new CultureInfo(culture);
            Thread.CurrentThread.CurrentUICulture = new CultureInfo(culture);
            return func();
        }
        finally
        {
            (Thread.CurrentThread.CurrentCulture, Thread.CurrentThread.CurrentUICulture) = saved;
        }
    }

    // ---- segments ---------------------------------------------------------------------------

    [Theory]
    [InlineData(3, 2)]
    [InlineData(5, 3)]
    [InlineData(7, 4)]
    [InlineData(9, 5)]
    public void OneSegmentOfEachDegree_WithVectorInput(int degree, int quantities)
    {
        var m = Segment(1, degree, (0, 0, 0), (1, 2, 3), "2");
        Set(m.StartVelocity, 0.5, 0, -0.5);

        var r = _service.Compute(new[] { m });

        Assert.Equal(2.0, r.TotalDuration, 12);
        Assert.Equal(quantities, r.Series.Count);
        Assert.Equal(0.5, r.Series[1].X[0], 9);
        Assert.Equal(-0.5, r.Series[1].Z[0], 9);
        Assert.Equal(3.0, r.Series[0].Z[^1], 9);
        Assert.Equal(new Point3D(1, 2, 3).Length, r.Series[0].Norm[^1], 9);
        Assert.Single(r.PartRows);
        Assert.Equal("Segment", r.PartRows[0].Kind);
        Assert.Equal(3 * (degree + 2), r.CoefficientRows.Count); // T and c0..cd per axis
    }

    [Fact]
    public void BlankDuration_WithLimits_IsTheShortestFeasible_WithoutLimits_OneSecond()
    {
        var limited = Segment(1, 9, (0, 0, 0), (3, 1, 2));
        limited.UseMaxVelocity = true;
        limited.MaxVelocity = "2";

        var r = _service.Compute(new[] { limited });
        double expected = ShortestFeasibleDuration.Find(
            9, new[] { State1D.FromPositionOnly(0), State1D.FromPositionOnly(0), State1D.FromPositionOnly(0) },
            new[] { State1D.FromPositionOnly(3), State1D.FromPositionOnly(1), State1D.FromPositionOnly(2) },
            KinematicLimits.FromValues(vMax: 2))!.Value;

        Assert.Equal(expected, r.TotalDuration, 12);
        Assert.Equal(1.0, _service.Compute(new[] { Segment(1, 9, (0, 0, 0), (3, 1, 2)) }).TotalDuration, 12);
    }

    [Fact]
    public void APrescribedDurationViolatingALimit_NamesSegmentAxisAndQuantity()
    {
        var m = Segment(1, 9, (0, 0, 0), (3, 1, 2), "0.5");
        m.UseMaxVelocity = true;
        m.MaxVelocity = "2";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));

        Assert.StartsWith("Segment 1:", ex.Message);
        Assert.Contains("velocity limit on axis x", ex.Message);
    }

    // ---- blends -------------------------------------------------------------------------------------

    [Fact]
    public void TwoSegmentsWithABlend_PartsAreSegmentBlendSegment_AndTheLimitAdjustmentIsReported()
    {
        // Both segments take their shortest feasible duration (acceleration at the limit); the inherited blend then
        // tends to exceed it. Look for a window where the Core adjusts the blend.
        Prototype3DComputationResult? adjusted = null;
        foreach (var (a, b) in new[] { ("0.7", "0.3"), ("0.5", "0.5"), ("0.8", "0.2"), ("0.6", "0.4"), ("0.9", "0.1") })
        {
            var first = Segment(1, 9, (0, 0, 0), (2, 1, 0));
            Set(first.EndVelocity, 1, 0.5, 0);
            var second = Segment(2, 9, (2, 1, 0), (2, 3, 2));
            Set(second.StartVelocity, 0, 1, 0.5);
            foreach (var s in new[] { first, second })
            {
                s.UseMaxAcceleration = true;
                s.MaxAcceleration = "3";
            }

            first.BlendToNext = a;
            second.BlendToPrevious = b;

            var r = _service.Compute(new[] { first, second });
            Assert.Equal(new[] { "Segment", "Blend", "Segment" }, r.PartRows.Select(p => p.Kind).ToArray());
            Assert.Equal("Blend 1-2", r.PartRows[1].Part);
            if (r.PartRows[1].Adjusted == "yes")
            {
                adjusted = r;
                break;
            }
        }

        Assert.NotNull(adjusted);
        var notice = Assert.Single(adjusted.Notices);
        Assert.StartsWith("Blend 1-2: duration adjusted from ", notice);
        Assert.True(adjusted.LimitRows.All(row => row.PercentOfLimit <= 100 + 1e-6));
    }

    [Fact]
    public void ABlendIsContinuous_AndByPointsEqualsByFractions()
    {
        Segment3DInputModel[] Make(bool points)
        {
            var first = Segment(1, 9, (0, 0, 0), (2, 1, 0), "1");
            Set(first.EndVelocity, 2, 1, 0);
            var second = Segment(2, 9, (2, 1, 0), (2, 3, 2), "1.5");
            Set(second.StartVelocity, 0, 2, 1);
            if (!points)
            {
                first.BlendToNext = "0.7";
                second.BlendToPrevious = "0.3";
                return new[] { first, second };
            }

            var s1 = new PolynomialSegment3D(9, new State3D(new Point3D(0, 0, 0), Point3D.Zero, Point3D.Zero, Point3D.Zero, Point3D.Zero),
                new State3D(new Point3D(2, 1, 0), new Point3D(2, 1, 0), Point3D.Zero, Point3D.Zero, Point3D.Zero), 1.0);
            var s2 = new PolynomialSegment3D(9, new State3D(new Point3D(2, 1, 0), new Point3D(0, 2, 1), Point3D.Zero, Point3D.Zero, Point3D.Zero),
                new State3D(new Point3D(2, 3, 2), Point3D.Zero, Point3D.Zero, Point3D.Zero, Point3D.Zero), 1.5);
            first.BlendKind = BlendInputKind.Points;
            second.BlendKind = BlendInputKind.Points;
            Point3D p = s1.EvaluatePosition(0.7), q = s2.EvaluatePosition(0.45);
            Set(first.BlendToNextPoint, p.X, p.Y, p.Z);
            Set(second.BlendToPreviousPoint, q.X, q.Y, q.Z);
            return new[] { first, second };
        }

        var byFraction = _service.Compute(Make(points: false));
        var byPoint = _service.Compute(Make(points: true));

        Assert.Equal(byFraction.TotalDuration, byPoint.TotalDuration, 1e-9);
        Assert.Equal(0.3 * 1.0 + 0.3 * 1.5, byFraction.PartRows[1].InheritedDuration!.Value, 1e-9);
        foreach (double boundary in byFraction.PartBoundaries)
        {
            State3D before = byFraction.StateAt(boundary - 1e-9), after = byFraction.StateAt(boundary + 1e-9);
            Assert.Equal(before.Velocity.X, after.Velocity.X, 1e-6);
            Assert.Equal(before.Acceleration.Y, after.Acceleration.Y, 1e-5);
        }
    }

    [Fact]
    public void ABlendWithOnlyOneValue_IsAnInputError()
    {
        var first = Segment(1, 9, (0, 0, 0), (2, 1, 0), "1");
        var second = Segment(2, 9, (2, 1, 0), (2, 3, 2), "1");
        first.BlendToNext = "0.7";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { first, second }));

        Assert.Contains("requires both values", ex.Message);
    }

    // ---- constant-speed stretch ------------------------------------------------------------------------

    [Fact]
    public void ACurvedStretch_ByPointsByFractionsAndByArcLength_GivesTheSameTrajectory()
    {
        var curve = CurveSegment();
        var whole = new ArcLengthPath(curve, 0, curve.Duration, requireRegular: false);

        var byTau = Curve();
        byTau.UseConstantSpeed = true;
        byTau.Speed = "1.2";
        byTau.StretchKind = StretchInputKind.ParameterFractions;
        byTau.StretchStartFraction = "0.25";
        byTau.StretchEndFraction = "0.8";

        var byPoints = Curve();
        byPoints.UseConstantSpeed = true;
        byPoints.Speed = "1.2";
        byPoints.StretchKind = StretchInputKind.Points;
        Point3D a = curve.EvaluatePosition(0.5), b = curve.EvaluatePosition(1.6);
        Set(byPoints.StretchStartPoint, a.X, a.Y, a.Z);
        Set(byPoints.StretchEndPoint, b.X, b.Y, b.Z);

        var byLength = Curve();
        byLength.UseConstantSpeed = true;
        byLength.Speed = "1.2";
        byLength.StretchKind = StretchInputKind.ArcLengthFractions;
        byLength.StretchStartFraction = N(whole.LengthAt(0.5) / whole.Length);
        byLength.StretchEndFraction = N(whole.LengthAt(1.6) / whole.Length);

        var r1 = _service.Compute(new[] { byTau });
        var r2 = _service.Compute(new[] { byPoints });
        var r3 = _service.Compute(new[] { byLength });

        Assert.Equal(new[] { "Entry", "Stretch", "Exit" }, r1.PartRows.Select(p => p.Kind).ToArray());
        foreach (var other in new[] { r2, r3 })
        {
            Assert.Equal(r1.TotalDuration, other.TotalDuration, 1e-8);
            Assert.Equal(r1.PathLength, other.PathLength, 1e-8);
            Assert.Equal(r1.PartRows[1].Length!.Value, other.PartRows[1].Length!.Value, 1e-8);
        }

        Assert.Equal(5, r1.Series.Count); // the stretch supplies up to snap
        var stretchPiece = r1.PathPieces.Single(p => p.Kind == "Stretch");
        Assert.True(stretchPiece.X.Length > 10);
    }

    [Fact]
    public void TheParabola_StretchAtSpeedOne_LastsTheLengthOverTheSpeed()
    {
        var m = Parabola();
        m.UseConstantSpeed = true;
        m.Speed = "1";
        m.StretchStartFraction = "0";
        m.StretchEndFraction = "1";

        var r = _service.Compute(new[] { m });

        var stretch = r.PartRows.Single(p => p.Kind == "Stretch");
        Assert.Equal(1.478943, stretch.Duration, 6);
        Assert.Equal(1.478943, stretch.Length!.Value, 6);
        Assert.Equal(1.0, stretch.Speed!.Value);
        // Entry and exit cover no distance: Rule 2 fallback max(T_cv / 2, 0.1 s) each (hand-check number of the note).
        Assert.Equal(2 * 1.478943, r.TotalDuration, 5);
    }

    [Fact]
    public void ASpeedAboveTheAdmissibleSpeed_IsRefusedWithTheCoreMessage()
    {
        var m = Parabola();
        m.UseConstantSpeed = true;
        m.Speed = "1.8";
        m.StretchStartFraction = "0";
        m.StretchEndFraction = "1";
        m.UseMaxAcceleration = true;
        m.MaxAcceleration = "8";
        m.UseMaxJerk = true;
        m.MaxJerk = "20";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));

        Assert.StartsWith("Segment 1:", ex.Message);
        Assert.Contains("admissible speed 1.70998", ex.Message);
        Assert.Contains("jerk", ex.Message);
    }

    [Fact]
    public void AStretchPointOffThePath_IsAnError()
    {
        var m = Curve();
        m.UseConstantSpeed = true;
        m.StretchKind = StretchInputKind.Points;
        Point3D a = CurveSegment().EvaluatePosition(0.5), b = CurveSegment().EvaluatePosition(1.6);
        Set(m.StretchStartPoint, a.X, a.Y, a.Z + 0.05);
        Set(m.StretchEndPoint, b.X, b.Y, b.Z);

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));

        Assert.Contains("not on the segment", ex.Message);
        Assert.Contains("distance", ex.Message);
    }

    [Fact]
    public void AStretchThroughARestPoint_IsAnError()
    {
        var m = Segment(1, 9, (0, 0, 0), (2, 1, 0), "1"); // rest to rest: speed zero at the start
        m.UseConstantSpeed = true;
        m.StretchStartFraction = "0";
        m.StretchEndFraction = "0.5";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));

        Assert.Contains("rest point", ex.Message);
    }

    [Fact]
    public void BlendAndStretchOnTheSameSegment_IsAnInputErrorNamingTheSegments()
    {
        var first = Segment(1, 9, (0, 0, 0), (2, 1, 0), "1");
        var second = Curve(2);
        Set(second.StartPosition, 2, 1, 0);
        second.UseConstantSpeed = true;
        first.BlendToNext = "0.7";
        second.BlendToPrevious = "0.3";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { first, second }));

        Assert.Contains("Segment 2 has a constant-speed stretch", ex.Message);
        Assert.Contains("segment(s) 1, 2", ex.Message);
    }

    // ---- input errors and culture ------------------------------------------------------------------------

    [Fact]
    public void APartlyFilledPositionRow_NamesSegmentQuantityAndAxis()
    {
        var m = Segment(1, 9, (0, 0, 0), (1, 1, 1), "1");
        m.StartPosition.Y = "";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));

        Assert.Equal("Segment 1 start position: the Y component is empty.", ex.Message);
    }

    [Fact]
    public void ANonNumericBox_NamesSegmentQuantityAndAxis_AndEmptyDerivativeBoxesAreZero()
    {
        var m = Segment(1, 9, (0, 0, 0), (1, 1, 1), "1");
        m.EndVelocity.Z = "abc";

        var ex = Assert.Throws<InvalidOperationException>(() => _service.Compute(new[] { m }));
        Assert.Contains("Segment 1 end velocity, Z component", ex.Message);

        m.EndVelocity.Z = "";
        m.EndVelocity.X = "0.25"; // the other boxes stay empty: zero
        var r = _service.Compute(new[] { m });
        Assert.Equal(0.25, r.Series[1].X[^1], 9);
        Assert.Equal(0.0, r.Series[1].Y[^1], 9);
    }

    [Fact]
    public void ItalianInput_DecimalCommasInTheVectors()
    {
        InCulture("it-IT", () =>
        {
            var m = new Segment3DInputModel(1) { Degree = 9, Duration = "1,5" };
            m.StartPosition.X = "0"; m.StartPosition.Y = "0"; m.StartPosition.Z = "0";
            m.EndPosition.X = "0,5"; m.EndPosition.Y = "1,25"; m.EndPosition.Z = "2";

            var r = _service.Compute(new[] { m });

            Assert.Equal(1.5, r.TotalDuration, 12);
            Assert.Equal(1.25, r.Series[0].Y[^1], 9);
            Assert.Contains(',', r.SummaryRows.Single(s => s.Item.StartsWith("Total duration")).Value);
            return 0;
        });
    }

    // ---- report ---------------------------------------------------------------------------------------------

    [Fact]
    public void ThePeaksTable_IsTheCoreLimitReport()
    {
        var m = Segment(1, 9, (0, 0, 0), (3, 1, 2));
        m.UseMaxVelocity = true; m.MaxVelocity = "2";
        m.UseMaxAcceleration = true; m.MaxAcceleration = "5";

        var r = _service.Compute(new[] { m });

        var limits = KinematicLimits.FromValues(vMax: 2, aMax: 5);
        var core = TrajectoryAnalyzer3D.LimitReport(
            new TrajectoryBuilder3D().AddSegment(
                State3D.FromPositionOnly(new Point3D(0, 0, 0)), State3D.FromPositionOnly(new Point3D(3, 1, 2)), null, 9, limits).Build(),
            limits);

        Assert.Equal(core.Count, r.LimitRows.Count);
        foreach (var row in core)
        {
            var shown = r.LimitRows.Single(x => x.Quantity == row.Quantity && x.Axis == row.Axis.ToUpperInvariant());
            Assert.Equal(row.Peak, shown.Peak, 12);
            Assert.Equal(row.PercentOfLimit, shown.PercentOfLimit, 12);
            Assert.Equal(row.PartLabel, shown.Part);
        }

        Assert.Equal(r.LimitRows.Max(x => x.PercentOfLimit), r.LimitRows[0].PercentOfLimit); // binding first
        Assert.Equal(100.0, r.LimitRows[0].PercentOfLimit, 1e-6);
    }

    [Fact]
    public void TheReport_HasExtremaPathLengthAndEndpointMarkers()
    {
        var r = _service.Compute(new[] { Segment(1, 9, (0, 0, 0), (3, 4, 12), "2") });

        Assert.Equal(13.0, r.PathLength, 1e-9);
        Assert.Equal(5 * 3, r.ExtremaRows.Count);
        var zPosition = r.ExtremaRows.Single(e => e.Quantity == "Position" && e.Axis == "Z");
        Assert.Equal(12.0, zPosition.Maximum, 9);
        Assert.Equal(2.0, zPosition.MaximumTime, 9);
        Assert.Equal(2, r.SegmentEndpoints.Count);
        Assert.Empty(r.PartBoundaries);
        Assert.Contains(r.SummaryRows, s => s.Item == "Path length");
    }

    [Fact]
    public void NoSegments_IsRejected()
    {
        Assert.Throws<InvalidOperationException>(() => _service.Compute(Array.Empty<Segment3DInputModel>()));
    }
}
