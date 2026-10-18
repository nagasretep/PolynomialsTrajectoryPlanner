using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;

namespace TrajectoryPlanner.App.Tests;

/// <summary>Request 0021 (criterion A3): the project file keeps the inputs of the four modes exactly; no window involved.</summary>
public class ProjectFileTests
{
    private readonly PrototypeTrajectoryService _service1D = new();
    private readonly Prototype3DTrajectoryService _service3D = new();

    // ---- helpers ---------------------------------------------------------------------------------

    /// <summary>All four modes with non-default, valid inputs.</summary>
    private static ProjectInputs NonDefaultProject()
    {
        var p = new ProjectInputs { Mode = AppMode.ThreeDPassThrough };

        // Blending: two segments, degree 7, a limit.
        p.Segments.Clear();
        p.Segments.Add(new SegmentInputModel(1)
        {
            Degree = 7, StartPosition = "0", EndPosition = "4.5", Duration = "1.5",
            UseMaxVelocity = true, MaxVelocity = "8"
        });
        p.Segments.Add(new SegmentInputModel(2)
        {
            Degree = 7, StartPosition = "4.5", EndPosition = "9", Duration = "2.25"
        });

        // 1D pass-through: degree 7, four waypoints, an optional duration, limits.
        p.PassThrough = new PassThroughInputModel { Degree = 7 };
        p.PassThrough.SmoothnessOrder = 3;
        p.PassThrough.Waypoints.Clear();
        foreach (var (position, number) in new[] { ("0", 1), ("1", 2), ("3.5", 3), ("2", 4) })
            p.PassThrough.Waypoints.Add(new WaypointInputModel(number, position));
        p.PassThrough.UseMaxVelocity = true;
        p.PassThrough.MaxVelocity = "2.5";
        p.PassThrough.UseMaxAcceleration = true;
        p.PassThrough.MaxAcceleration = "8";

        // 3D: two straight segments with a limit.
        p.Segments3D.Clear();
        var s1 = new Segment3DInputModel(1) { Degree = 7, Duration = "1.25", UseMaxVelocity = true, MaxVelocity = "5" };
        s1.EndPosition.X = "1"; s1.EndPosition.Y = "1"; s1.EndPosition.Z = "0.5";
        var s2 = new Segment3DInputModel(2) { Degree = 5 };
        s2.StartPosition.X = "1"; s2.StartPosition.Y = "1"; s2.StartPosition.Z = "0.5";
        s2.EndPosition.X = "2"; s2.EndPosition.Y = "0"; s2.EndPosition.Z = "1.5";
        p.Segments3D.Add(s1);
        p.Segments3D.Add(s2);

        // 3D pass-through: degree 7, a non-zero end velocity, limits.
        p.PassThrough3D = new PassThrough3DInputModel { Degree = 7 };
        p.PassThrough3D.EndAtRest = false;
        p.PassThrough3D.EndVelocity.X = "0.5";
        p.PassThrough3D.EndVelocity.Z = "-0.25";
        p.PassThrough3D.UseMaxVelocity = true;
        p.PassThrough3D.MaxVelocity = "4";
        p.PassThrough3D.UseMaxAcceleration = true;
        p.PassThrough3D.MaxAcceleration = "9";
        return p;
    }

    private static string Text(ProjectInputs project)
    {
        using var stream = new MemoryStream();
        ProjectSerializer.Save(project, stream);
        return Encoding.UTF8.GetString(stream.ToArray());
    }

    private static ProjectInputs Load(string json)
    {
        using var stream = new MemoryStream(Encoding.UTF8.GetBytes(json));
        return ProjectSerializer.Load(stream, "test.ptp.json");
    }

    private static ProjectInputs RoundTrip(ProjectInputs project) => Load(Text(project));

    /// <summary>Sets every saved text/flag/choice field of a model (and its vectors) to a distinctive non-default value, including text that is not a number.</summary>
    private static void Disturb(object model, ref int counter)
    {
        foreach (PropertyInfo property in model.GetType().GetProperties(BindingFlags.Public | BindingFlags.Instance))
        {
            if (property.GetIndexParameters().Length > 0)
                continue;

            Type type = property.PropertyType;
            if (type == typeof(string) && property.CanWrite)
                property.SetValue(model, $"text {++counter}, é");
            else if (type == typeof(bool) && property.CanWrite && property.Name is not ("HasSegment"))
                property.SetValue(model, !(bool)property.GetValue(model)!);
            else if (type.IsEnum && property.CanWrite)
            {
                var values = Enum.GetValues(type);
                property.SetValue(model, values.GetValue(((int)property.GetValue(model)! + 1) % values.Length));
            }
            else if (type == typeof(VectorInput))
                Disturb(property.GetValue(model)!, ref counter);
        }
    }

    private static void WithCulture(string name, Action action)
    {
        CultureInfo saved = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo(name);
            action();
        }
        finally
        {
            CultureInfo.CurrentCulture = saved;
        }
    }

    private void AssertSameResults(ProjectInputs a, ProjectInputs b)
    {
        var r1 = _service1D.Compute(a.Segments);
        var r2 = _service1D.Compute(b.Segments);
        AssertSame1D(r1, r2);
        AssertSame1D(_service1D.ComputePassThrough(a.PassThrough), _service1D.ComputePassThrough(b.PassThrough));

        AssertSame3D(_service3D.Compute(a.Segments3D), _service3D.Compute(b.Segments3D));
        AssertSame3D(_service3D.ComputePassThrough(a.PassThrough3D), _service3D.ComputePassThrough(b.PassThrough3D));
    }

    private static void AssertSame1D(PrototypeComputationResult x, PrototypeComputationResult y)
    {
        Assert.Equal(x.TotalDuration, y.TotalDuration, 12);
        Assert.Equal(x.CoefficientRows.Count, y.CoefficientRows.Count);
        Assert.NotEmpty(x.CoefficientRows);
        for (int i = 0; i < x.CoefficientRows.Count; i++)
            Assert.Equal(x.CoefficientRows[i].Value, y.CoefficientRows[i].Value, 12);
        for (int i = 0; i < x.ExtremaRows.Count; i++)
        {
            Assert.Equal(x.ExtremaRows[i].Minimum, y.ExtremaRows[i].Minimum, 12);
            Assert.Equal(x.ExtremaRows[i].Maximum, y.ExtremaRows[i].Maximum, 12);
        }
    }

    private static void AssertSame3D(Prototype3DComputationResult x, Prototype3DComputationResult y)
    {
        Assert.Equal(x.TotalDuration, y.TotalDuration, 12);
        Assert.Equal(x.PathLength, y.PathLength, 12);
        Assert.NotEmpty(x.CoefficientRows);
        for (int i = 0; i < x.CoefficientRows.Count; i++)
            Assert.Equal(x.CoefficientRows[i].Value, y.CoefficientRows[i].Value, 12);
        for (int i = 0; i < x.ExtremaRows.Count; i++)
        {
            Assert.Equal(x.ExtremaRows[i].Minimum, y.ExtremaRows[i].Minimum, 12);
            Assert.Equal(x.ExtremaRows[i].Maximum, y.ExtremaRows[i].Maximum, 12);
        }
    }

    // ---- save then load ----------------------------------------------------------------------------

    [Fact]
    public void TheFileHasTheFormatNameVersionMode_AndOneObjectPerMode()
    {
        string json = Text(NonDefaultProject());
        Assert.Contains("\"format\": \"PolynomialsTrajectoryPlanner.Project\"", json);
        Assert.Contains("\"version\": 1", json);
        Assert.Contains("\"mode\": \"ThreeDPassThrough\"", json);
        foreach (string section in new[] { "\"blending\"", "\"passThrough\"", "\"threeD\"", "\"threeDPassThrough\"" })
            Assert.Contains(section, json);
        Assert.Contains("\n", json); // indented, human readable
    }

    [Fact]
    public void BlendingMode_RoundTripsField_ByField_AndTheResultIsTheSame()
    {
        var original = NonDefaultProject();
        var loaded = RoundTrip(original);

        Assert.Equal(2, loaded.Segments.Count);
        Assert.Equal(7, loaded.Segments[0].Degree);
        Assert.Equal("4.5", loaded.Segments[0].EndPosition);
        Assert.Equal("1.5", loaded.Segments[0].Duration);
        Assert.True(loaded.Segments[0].UseMaxVelocity);
        Assert.Equal("8", loaded.Segments[0].MaxVelocity);
        Assert.Equal(2, loaded.Segments[1].SegmentNumber);
        Assert.Equal(Text(original), Text(loaded));
        AssertSame1D(_service1D.Compute(original.Segments), _service1D.Compute(loaded.Segments));
    }

    [Fact]
    public void PassThroughMode_RoundTripsField_ByField_AndTheResultIsTheSame()
    {
        var original = NonDefaultProject();
        var loaded = RoundTrip(original);

        Assert.Equal(7, loaded.PassThrough.Degree);
        Assert.Equal(["0", "1", "3.5", "2"], loaded.PassThrough.Waypoints.Select(w => w.Position));
        Assert.Equal([1, 2, 3, 4], loaded.PassThrough.Waypoints.Select(w => w.Number));
        Assert.False(loaded.PassThrough.Waypoints[3].HasSegment);
        Assert.Equal("2.5", loaded.PassThrough.MaxVelocity);
        Assert.True(loaded.PassThrough.UseMaxAcceleration);
        AssertSame1D(_service1D.ComputePassThrough(original.PassThrough), _service1D.ComputePassThrough(loaded.PassThrough));
    }

    [Fact]
    public void ThreeDMode_RoundTripsField_ByField_AndTheResultIsTheSame()
    {
        var original = NonDefaultProject();
        var loaded = RoundTrip(original);

        Assert.Equal(2, loaded.Segments3D.Count);
        Assert.Equal("0.5", loaded.Segments3D[0].EndPosition.Z);
        Assert.Equal("1.5", loaded.Segments3D[1].EndPosition.Z);
        Assert.Equal(5, loaded.Segments3D[1].Degree);
        Assert.Equal("1.25", loaded.Segments3D[0].Duration);
        AssertSame3D(_service3D.Compute(original.Segments3D), _service3D.Compute(loaded.Segments3D));
    }

    [Fact]
    public void ThreeDPassThroughMode_RoundTripsField_ByField_AndTheResultIsTheSame()
    {
        var original = NonDefaultProject();
        var loaded = RoundTrip(original);

        Assert.Equal(AppMode.ThreeDPassThrough, loaded.Mode);
        Assert.Equal(7, loaded.PassThrough3D.Degree);
        Assert.False(loaded.PassThrough3D.EndAtRest);
        Assert.Equal("0.5", loaded.PassThrough3D.EndVelocity.X);
        Assert.Equal("-0.25", loaded.PassThrough3D.EndVelocity.Z);
        Assert.Equal(4, loaded.PassThrough3D.Waypoints.Count);
        Assert.Equal("3", loaded.PassThrough3D.Waypoints[2].Y);
        Assert.Equal("4", loaded.PassThrough3D.MaxVelocity);
        AssertSame3D(_service3D.ComputePassThrough(original.PassThrough3D), _service3D.ComputePassThrough(loaded.PassThrough3D));
    }

    [Fact]
    public void EveryInputOfEveryModel_Survives_EvenWhenItIsNotANumber()
    {
        // Every text/flag/choice field of every model is changed to something distinctive; the file must carry all of them.
        var original = new ProjectInputs { Mode = AppMode.ThreeD };
        int counter = 0;
        Disturb(original.Segments[0], ref counter);
        Disturb(original.PassThrough, ref counter);
        Disturb(original.PassThrough.Waypoints[1], ref counter);
        Disturb(original.Segments3D[0], ref counter);
        Disturb(original.PassThrough3D, ref counter);
        Disturb(original.PassThrough3D.Waypoints[2], ref counter);

        string before = Text(original);
        ProjectInputs loaded = RoundTrip(original);
        Assert.Equal(before, Text(loaded));
        Assert.NotEqual(Text(new ProjectInputs { Mode = AppMode.ThreeD }), before); // the test changes something

        Assert.StartsWith("text ", loaded.Segments[0].StartPosition);
        Assert.Equal(original.Segments3D[0].BlendKind, loaded.Segments3D[0].BlendKind);
        Assert.Equal(original.Segments3D[0].StretchKind, loaded.Segments3D[0].StretchKind);
        Assert.Equal(original.PassThrough3D.Waypoints[2].X, loaded.PassThrough3D.Waypoints[2].X);
        Assert.Equal(original.PassThrough3D.StartJerk.Y, loaded.PassThrough3D.StartJerk.Y);
        Assert.Equal(original.PassThrough.TotalDuration, loaded.PassThrough.TotalDuration);
        Assert.Equal("text é".Length > 0, loaded.Segments[0].StartPosition.Contains('é'));
    }

    [Theory]
    [InlineData("it-IT", "en-US")]
    [InlineData("en-US", "it-IT")]
    public void TheCultureDoesNotMatter_WhenSavingOrLoading(string savingCulture, string loadingCulture)
    {
        ProjectInputs original = NonDefaultProject();
        string json = string.Empty;
        WithCulture(savingCulture, () => json = Text(original));

        Assert.DoesNotContain(",5", json.Replace("é", "")); // no culture-formatted number was written (numbers stay as typed)
        ProjectInputs loaded = null!;
        WithCulture(loadingCulture, () => loaded = Load(json));

        Assert.Equal(Text(original), Text(loaded));
        WithCulture(loadingCulture, () => AssertSameResults(original, loaded));
    }

    [Fact]
    public void ADecimalCommaTypedByTheAuthor_IsKeptAsTyped()
    {
        var original = new ProjectInputs();
        original.Segments[0].EndPosition = "7,5";
        original.PassThrough.Waypoints[1].Position = "1,25";
        ProjectInputs loaded = RoundTrip(original);
        Assert.Equal("7,5", loaded.Segments[0].EndPosition);
        Assert.Equal("1,25", loaded.PassThrough.Waypoints[1].Position);
    }

    // ---- tolerance of the format -----------------------------------------------------------------

    [Fact]
    public void AnUnknownField_IsIgnored()
    {
        string json = Text(NonDefaultProject())
            .Replace("\"version\": 1,", "\"version\": 1,\n  \"comment\": \"written by hand\",")
            .Replace("\"degree\": 7,", "\"degree\": 7,\n \"somethingNew\": [1, 2, 3],");
        Assert.Contains("somethingNew", json);

        ProjectInputs loaded = Load(json);
        Assert.Equal(Text(NonDefaultProject()), Text(loaded));
    }

    [Fact]
    public void AMissingModeObject_LoadsAsTheDefaultOfThatMode()
    {
        var node = System.Text.Json.Nodes.JsonNode.Parse(Text(NonDefaultProject()))!.AsObject();
        node.Remove("passThrough");
        node.Remove("threeD");

        ProjectInputs loaded = Load(node.ToJsonString());
        Assert.Equal(["0", "1", "2"], loaded.PassThrough.Waypoints.Select(w => w.Position));
        Assert.Single(loaded.Segments3D);
        Assert.Equal(2, loaded.Segments.Count); // the modes present are loaded
        Assert.Equal(7, loaded.PassThrough3D.Degree);
    }

    [Fact]
    public void AMissingField_TakesTheDefaultOfTheMode()
    {
        var node = System.Text.Json.Nodes.JsonNode.Parse(Text(NonDefaultProject()))!.AsObject();
        node["threeDPassThrough"]!.AsObject().Remove("maxSnap");
        node["threeDPassThrough"]!.AsObject().Remove("totalDuration");
        node.Remove("mode");

        ProjectInputs loaded = Load(node.ToJsonString());
        Assert.Equal("50", loaded.PassThrough3D.MaxSnap);
        Assert.Equal(string.Empty, loaded.PassThrough3D.TotalDuration);
        Assert.Equal(AppMode.Blending, loaded.Mode);
    }

    // ---- refusals ----------------------------------------------------------------------------------

    private static string ReasonOf(string json)
    {
        var ex = Assert.Throws<ProjectFileException>(() => Load(json));
        Assert.Contains("test.ptp.json", ex.Message);
        return ex.Message;
    }

    [Fact]
    public void ANewerVersion_IsRefused_WithItsNumber()
    {
        string json = Text(NonDefaultProject()).Replace("\"version\": 1", "\"version\": 2");
        string message = ReasonOf(json);
        Assert.Contains("version 2", message);
        Assert.Contains("newer", message);
    }

    [Fact]
    public void AFileThatIsNotJson_IsRefused_WithTheReason()
    {
        Assert.Contains("not a valid JSON", ReasonOf("this is { not json"));
        Assert.Contains("not a project file", ReasonOf("[1, 2, 3]"));
    }

    [Fact]
    public void WrongFormatMissingVersionWrongTypesAndUnknownChoices_AreRefused()
    {
        string good = Text(NonDefaultProject());
        Assert.Contains("wrong format", ReasonOf(good.Replace("PolynomialsTrajectoryPlanner.Project", "Something.Else")));
        Assert.Contains("version", ReasonOf(good.Replace("\"version\": 1,", "")));
        Assert.Contains("unknown mode", ReasonOf(good.Replace("ThreeDPassThrough\",", "Quantum\",")));
        Assert.Contains("\"degree\"", ReasonOf(good.Replace("\"degree\": 7,", "\"degree\": \"seven\",")));
        Assert.Contains("\"maxVelocity\"", ReasonOf(good.Replace("\"maxVelocity\": \"2.5\"", "\"maxVelocity\": 2.5")));
        var wrongList = System.Text.Json.Nodes.JsonNode.Parse(good)!.AsObject();
        wrongList["threeD"]!["segments"] = "none";
        Assert.Contains("\"segments\"", ReasonOf(wrongList.ToJsonString()));
        var emptyList = System.Text.Json.Nodes.JsonNode.Parse(good)!.AsObject();
        emptyList["blending"]!["segments"] = new System.Text.Json.Nodes.JsonArray();
        Assert.Contains("at least", ReasonOf(emptyList.ToJsonString()));
    }

    [Fact]
    public void AMissingFile_IsRefused_WithItsName()
    {
        var ex = Assert.Throws<ProjectFileException>(() =>
            ProjectSerializer.Load(Path.Combine(Path.GetTempPath(), "no-such-folder-0021", "absent.ptp.json")));
        Assert.Contains("absent.ptp.json", ex.Message);
    }

    [Fact]
    public void SavingAndLoadingThroughAFile_Works()
    {
        string path = Path.Combine(Path.GetTempPath(), $"ptp-0021-{Guid.NewGuid():N}.ptp.json");
        try
        {
            ProjectSerializer.Save(NonDefaultProject(), path);
            Assert.Equal(Text(NonDefaultProject()), Text(ProjectSerializer.Load(path)));
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void ApplyingALoadedProject_ReplacesTheLiveModels()
    {
        var segments = new System.Collections.ObjectModel.ObservableCollection<SegmentInputModel> { new(1) };
        var pass = new PassThroughInputModel();
        var segments3D = new System.Collections.ObjectModel.ObservableCollection<Segment3DInputModel> { new(1) };
        var pass3D = new PassThrough3DInputModel();

        ProjectSerializer.ApplyTo(RoundTrip(NonDefaultProject()), segments, pass, segments3D, pass3D);

        Assert.Equal(2, segments.Count);
        Assert.Equal(7, pass.Degree);
        Assert.Equal(3, pass.SmoothnessOrder); // the saved order wins over the degree default (m = p = 4)
        Assert.Equal(4, pass.Waypoints.Count);
        Assert.Equal([1, 2, 3, 4], pass.Waypoints.Select(w => w.Number));
        Assert.Equal(2, segments3D.Count);
        Assert.Equal("-0.25", pass3D.EndVelocity.Z);
        Assert.False(pass3D.EndAtRest);
        Assert.True(pass3D.EndEditable);
    }
}
