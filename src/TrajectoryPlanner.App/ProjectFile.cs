using System.Collections;
using System.Collections.ObjectModel;
using System.IO;
using System.Reflection;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace TrajectoryPlanner.App;

/// <summary>The refusal of a project file: the message names the file (when known) and the reason.</summary>
public sealed class ProjectFileException(string message) : Exception(message);

/// <summary>
/// The inputs of the four modes plus the selected mode (request 0021, criterion A3). Every value is the text the author typed;
/// results are never part of a project.
/// </summary>
public sealed class ProjectInputs
{
    public AppMode Mode { get; set; } = AppMode.Blending;
    public List<SegmentInputModel> Segments { get; } = [new SegmentInputModel(1)];
    public PassThroughInputModel PassThrough { get; set; } = new();
    public List<Segment3DInputModel> Segments3D { get; } = [new Segment3DInputModel(1)];
    public PassThrough3DInputModel PassThrough3D { get; set; } = new();
}

/// <summary>
/// Saves and loads a project as one indented JSON file (<c>.ptp.json</c>). The file has a <c>format</c> name, an integer
/// <c>version</c> and one object per mode. Fields are the public settable text/flag/choice properties of the input models, found by
/// reflection, so a new input added to a model is saved without touching this class. No number is formatted: the culture never matters.
/// </summary>
public static class ProjectSerializer
{
    public const string FormatName = "PolynomialsTrajectoryPlanner.Project";
    public const int CurrentVersion = 1;
    public const string FileExtension = ".ptp.json";

    // Derived or bookkeeping properties that are recomputed from the others, never saved.
    private static readonly HashSet<string> Derived = ["SegmentNumber", "Number", "HasSegment"];

    private static readonly JsonSerializerOptions Options = new() { WriteIndented = true };

    // ---- save ------------------------------------------------------------------------------------

    public static void Save(ProjectInputs project, string path)
    {
        using var stream = File.Create(path);
        Save(project, stream);
    }

    public static void Save(ProjectInputs project, Stream stream)
    {
        var root = new JsonObject
        {
            ["format"] = FormatName,
            ["version"] = CurrentVersion,
            ["mode"] = project.Mode.ToString(),
            ["blending"] = new JsonObject { ["segments"] = ArrayOf(project.Segments) },
            ["passThrough"] = Pack(project.PassThrough, ("waypoints", ArrayOf(project.PassThrough.Waypoints))),
            ["threeD"] = new JsonObject { ["segments"] = ArrayOf(project.Segments3D) },
            ["threeDPassThrough"] = Pack(project.PassThrough3D, ("waypoints", ArrayOf(project.PassThrough3D.Waypoints)))
        };

        using var writer = new Utf8JsonWriter(stream, new JsonWriterOptions { Indented = true });
        root.WriteTo(writer, Options);
    }

    private static JsonArray ArrayOf(IEnumerable items)
    {
        var array = new JsonArray();
        foreach (object item in items)
            array.Add(Pack(item));
        return array;
    }

    private static JsonObject Pack(object model, params (string Name, JsonNode Value)[] extra)
    {
        var node = new JsonObject();
        foreach (PropertyInfo property in Fields(model.GetType()))
        {
            object value = property.GetValue(model)!;
            node[CamelCase(property.Name)] = value switch
            {
                string text => text,
                bool flag => flag,
                int number => number,
                Enum choice => choice.ToString(),
                _ => Pack(value) // a VectorInput: X | Y | Z
            };
        }

        foreach (var (name, value) in extra)
            node[name] = value;
        return node;
    }

    // ---- load ------------------------------------------------------------------------------------

    public static ProjectInputs Load(string path)
    {
        string name = Path.GetFileName(path);
        try
        {
            using var stream = File.OpenRead(path);
            return Load(stream, name);
        }
        catch (FileNotFoundException)
        {
            throw new ProjectFileException($"{name}: the file does not exist.");
        }
        catch (DirectoryNotFoundException)
        {
            throw new ProjectFileException($"{name}: the folder does not exist.");
        }
        catch (IOException ex)
        {
            throw new ProjectFileException($"{name}: the file cannot be read ({ex.Message}).");
        }
        catch (UnauthorizedAccessException ex)
        {
            throw new ProjectFileException($"{name}: access to the file is denied ({ex.Message}).");
        }
    }

    /// <summary>Reads a project completely into new models; throws <see cref="ProjectFileException"/> (nothing else changes) if anything is wrong.</summary>
    public static ProjectInputs Load(Stream stream, string fileName = "project")
    {
        JsonNode? parsed;
        try
        {
            using var reader = new StreamReader(stream, Encoding.UTF8);
            parsed = JsonNode.Parse(reader.ReadToEnd());
        }
        catch (JsonException ex)
        {
            throw new ProjectFileException($"{fileName}: not a valid JSON file ({ex.Message}).");
        }
        catch (DecoderFallbackException)
        {
            throw new ProjectFileException($"{fileName}: not a valid text file (UTF-8 expected).");
        }

        if (parsed is not JsonObject root)
            throw new ProjectFileException($"{fileName}: not a project file (the content is not a JSON object).");

        try
        {
            return Unpack(root, fileName);
        }
        catch (ProjectFileException)
        {
            throw;
        }
        catch (Exception ex) when (ex is InvalidOperationException or FormatException or ArgumentException or InvalidCastException)
        {
            throw new ProjectFileException($"{fileName}: unreadable content ({ex.Message}).");
        }
    }

    private static ProjectInputs Unpack(JsonObject root, string fileName)
    {
        if (Read<string>(root, "format", fileName) != FormatName)
            throw new ProjectFileException($"{fileName}: wrong format (expected \"{FormatName}\").");

        int? version = Read<int?>(root, "version", fileName);
        if (version is null or < 1)
            throw new ProjectFileException($"{fileName}: the version is missing or not a positive integer.");
        if (version > CurrentVersion)
            throw new ProjectFileException(
                $"{fileName}: version {version} is newer than this program supports (up to {CurrentVersion}); update the program.");

        var project = new ProjectInputs();

        if (Read<string>(root, "mode", fileName) is { } mode)
        {
            if (!Enum.TryParse(mode, out AppMode parsedMode) || !Enum.IsDefined(parsedMode))
                throw new ProjectFileException($"{fileName}: unknown mode \"{mode}\".");
            project.Mode = parsedMode;
        }

        if (Section(root, "blending", fileName) is { } blending && ListOf(blending, "segments", fileName) is { } blendingList)
            Fill(project.Segments, blendingList, n => new SegmentInputModel(n), fileName);

        if (Section(root, "passThrough", fileName) is { } passThrough)
        {
            Unpack(passThrough, project.PassThrough, fileName);
            if (ListOf(passThrough, "waypoints", fileName) is { } waypoints)
            {
                var list = new List<WaypointInputModel>();
                Fill(list, waypoints, n => new WaypointInputModel(n, "0"), fileName, minimum: 2);
                project.PassThrough.Waypoints.Clear();
                foreach (var item in list)
                    project.PassThrough.Waypoints.Add(item);
            }
        }

        if (Section(root, "threeD", fileName) is { } threeD && ListOf(threeD, "segments", fileName) is { } threeDList)
            Fill(project.Segments3D, threeDList, n => new Segment3DInputModel(n), fileName);

        if (Section(root, "threeDPassThrough", fileName) is { } threeDPass)
        {
            Unpack(threeDPass, project.PassThrough3D, fileName);
            if (ListOf(threeDPass, "waypoints", fileName) is { } waypoints)
            {
                var list = new List<Waypoint3DInputModel>();
                Fill(list, waypoints, n => new Waypoint3DInputModel(n, "0", "0", "0"), fileName, minimum: 2);
                project.PassThrough3D.Waypoints.Clear();
                foreach (var item in list)
                    project.PassThrough3D.Waypoints.Add(item);
            }
        }

        return project;
    }

    private static void Fill<T>(List<T> target, JsonArray source, Func<int, T> create, string fileName, int minimum = 1)
        where T : class
    {
        if (source.Count < minimum)
            throw new ProjectFileException($"{fileName}: a list needs at least {minimum} entr{(minimum == 1 ? "y" : "ies")}.");

        var built = new List<T>();
        for (int i = 0; i < source.Count; i++)
        {
            if (source[i] is not JsonObject node)
                throw new ProjectFileException($"{fileName}: entry {i + 1} of a list is not an object.");
            T model = create(i + 1);
            Unpack(node, model, fileName);
            built.Add(model);
        }

        target.Clear();
        target.AddRange(built);
    }

    /// <summary>Copies the fields present in the node into the model; a missing field keeps the model's default, an unknown field is ignored.</summary>
    private static void Unpack(JsonObject node, object model, string fileName)
    {
        foreach (PropertyInfo property in Fields(model.GetType()))
        {
            string key = CamelCase(property.Name);
            if (!node.TryGetPropertyValue(key, out JsonNode? value) || value is null)
                continue;

            if (property.PropertyType == typeof(string))
                property.SetValue(model, ExpectValue<string>(value, key, fileName));
            else if (property.PropertyType == typeof(bool))
                property.SetValue(model, ExpectValue<bool>(value, key, fileName));
            else if (property.PropertyType == typeof(int))
                property.SetValue(model, ExpectValue<int>(value, key, fileName));
            else if (property.PropertyType.IsEnum)
            {
                string name = ExpectValue<string>(value, key, fileName);
                if (!Enum.TryParse(property.PropertyType, name, out object? choice) || !Enum.IsDefined(property.PropertyType, choice!))
                    throw new ProjectFileException($"{fileName}: \"{name}\" is not a valid value of \"{key}\".");
                property.SetValue(model, choice);
            }
            else if (value is JsonObject nested)
                Unpack(nested, property.GetValue(model)!, fileName);
            else
                throw new ProjectFileException($"{fileName}: field \"{key}\" must be an object.");
        }
    }

    private static T ExpectValue<T>(JsonNode value, string key, string fileName)
    {
        try
        {
            return value.GetValue<T>();
        }
        catch (Exception ex) when (ex is InvalidOperationException or FormatException)
        {
            throw new ProjectFileException($"{fileName}: field \"{key}\" has the wrong type (expected {typeof(T).Name}).");
        }
    }

    private static T? Read<T>(JsonObject node, string key, string fileName)
    {
        if (!node.TryGetPropertyValue(key, out JsonNode? value) || value is null)
            return default;
        return ExpectValue<T>(value, key, fileName);
    }

    private static JsonObject? Section(JsonObject root, string key, string fileName) =>
        !root.TryGetPropertyValue(key, out JsonNode? node) || node is null
            ? null
            : node as JsonObject ?? throw new ProjectFileException($"{fileName}: \"{key}\" must be an object.");

    private static JsonArray? ListOf(JsonObject section, string key, string fileName) =>
        !section.TryGetPropertyValue(key, out JsonNode? node) || node is null
            ? null
            : node as JsonArray ?? throw new ProjectFileException($"{fileName}: \"{key}\" must be a list.");

    // ---- applying a loaded project to the live models of the window -----------------------------------

    /// <summary>Copies every input of <paramref name="source"/> into the live models. Cannot fail: the source was built and validated by Load.</summary>
    public static void ApplyTo(
        ProjectInputs source,
        ObservableCollection<SegmentInputModel> segments,
        PassThroughInputModel passThrough,
        ObservableCollection<Segment3DInputModel> segments3D,
        PassThrough3DInputModel passThrough3D)
    {
        segments.Clear();
        foreach (var segment in source.Segments)
            segments.Add(segment);

        segments3D.Clear();
        foreach (var segment in source.Segments3D)
            segments3D.Add(segment);

        CopyFields(source.PassThrough, passThrough);
        passThrough.Waypoints.Clear();
        foreach (var waypoint in source.PassThrough.Waypoints.ToList())
            passThrough.Waypoints.Add(waypoint);

        CopyFields(source.PassThrough3D, passThrough3D);
        passThrough3D.Waypoints.Clear();
        foreach (var waypoint in source.PassThrough3D.Waypoints.ToList())
            passThrough3D.Waypoints.Add(waypoint);
    }

    /// <summary>Copies all saved fields (also nested vectors) from one model to another of the same type.</summary>
    public static void CopyFields(object source, object target)
    {
        foreach (PropertyInfo property in Fields(source.GetType()))
        {
            object value = property.GetValue(source)!;
            if (property.CanWrite)
                property.SetValue(target, value);
            else
                CopyFields(value, property.GetValue(target)!);
        }
    }

    /// <summary>
    /// The saved fields of a model in a stable order: <c>Degree</c> first (setting it resets dependent defaults), then the others in
    /// declaration order. Text, flag, whole-number and choice properties with a public setter; vectors are get-only objects.
    /// </summary>
    private static IEnumerable<PropertyInfo> Fields(Type type) =>
        type.GetProperties(BindingFlags.Public | BindingFlags.Instance)
            .Where(p => p.GetIndexParameters().Length == 0 && p.CanRead && !Derived.Contains(p.Name) && IsField(p))
            .OrderBy(p => p.Name == "Degree" ? 0 : 1);

    private static bool IsField(PropertyInfo p)
    {
        Type t = p.PropertyType;
        if (t == typeof(string) || t == typeof(bool) || t == typeof(int) || t.IsEnum)
            return p.CanWrite && p.SetMethod!.IsPublic;
        return t == typeof(VectorInput) && !p.CanWrite;
    }

    private static string CamelCase(string name) => char.ToLowerInvariant(name[0]) + name[1..];
}
