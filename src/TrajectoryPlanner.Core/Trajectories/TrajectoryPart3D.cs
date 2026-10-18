using TrajectoryPlanner.Core.Segments;

namespace TrajectoryPlanner.Core.Trajectories;

public enum PartKind
{
    /// <summary>An ordinary polynomial segment (possibly the truncated rest of one that was blended).</summary>
    Segment,

    /// <summary>A blend between two consecutive segments.</summary>
    Blend,

    /// <summary>Entry transition of a constant-speed stretch.</summary>
    CvEntry,

    /// <summary>The constant-speed stretch itself.</summary>
    CvStretch,

    /// <summary>Exit transition of a constant-speed stretch.</summary>
    CvExit
}

/// <summary>
/// Descriptor of one piece of a 3D trajectory, in order: what it is, which original segment(s) it comes from, when it
/// starts and how long it lasts, and the extra information the user interface shows for blends and stretches.
/// </summary>
/// <param name="Kind">What the piece is.</param>
/// <param name="OriginalSegmentIndex">Zero-based index, in the order the segments were added, of the original segment (for a blend: the preceding one).</param>
/// <param name="SecondSegmentIndex">For a blend: the following original segment; otherwise null.</param>
/// <param name="StartTime">Start of the piece on the time axis of the trajectory.</param>
/// <param name="Duration">Duration of the piece.</param>
/// <param name="Segment">The piece itself.</param>
/// <param name="InheritedDuration">Blend only: the duration inherited from the removed portions of the neighbours.</param>
/// <param name="AdjustedByLimits">Blend only: the limits forced a duration other than the inherited one.</param>
/// <param name="StretchLength">Stretch only: arc length of the subpath.</param>
/// <param name="StretchSpeed">Stretch only: the constant speed.</param>
/// <param name="AdmissibleSpeed">Stretch only: the largest speed the limits admit (null: no active limit constrains it).</param>
/// <param name="BindingLimit">Stretch only: the limit that determines the admissible speed.</param>
public sealed record TrajectoryPart3D(
    PartKind Kind,
    int OriginalSegmentIndex,
    int? SecondSegmentIndex,
    double StartTime,
    double Duration,
    ISegment3D Segment,
    double? InheritedDuration = null,
    bool AdjustedByLimits = false,
    double? StretchLength = null,
    double? StretchSpeed = null,
    double? AdmissibleSpeed = null,
    string? BindingLimit = null)
{
    public double EndTime => StartTime + Duration;

    /// <summary>A short name with numbers from 1, for messages and tables: "Segment 2", "Blend 1-2", "Entry 3", "Stretch 3", "Exit 3".</summary>
    public string Label => Kind switch
    {
        PartKind.Segment => $"Segment {OriginalSegmentIndex + 1}",
        PartKind.Blend => $"Blend {OriginalSegmentIndex + 1}-{(SecondSegmentIndex ?? OriginalSegmentIndex + 1) + 1}",
        PartKind.CvEntry => $"Entry {OriginalSegmentIndex + 1}",
        PartKind.CvStretch => $"Stretch {OriginalSegmentIndex + 1}",
        _ => $"Exit {OriginalSegmentIndex + 1}"
    };
}
