using System;
using System.Collections.Generic;

namespace PlanarLineGeometry
{
    public readonly struct Point2
    {
        public Point2(double x, double y) { X = x; Y = y; }
        public double X { get; }
        public double Y { get; }
        internal Point2 Add(Vector2 v) => new Point2(X + v.X, Y + v.Y);
        internal Vector2 Subtract(Point2 other) => new Vector2(X - other.X, Y - other.Y);
    }

    internal readonly struct Vector2
    {
        public Vector2(double x, double y) { X = x; Y = y; }
        public double X { get; }
        public double Y { get; }
        public double Length => Math.Sqrt(X * X + Y * Y);
        public double Dot(Vector2 other) => X * other.X + Y * other.Y;
        public static Vector2 operator *(double k, Vector2 v) => new Vector2(k * v.X, k * v.Y);
    }

    public sealed class Segment2
    {
        public Segment2(Point2 start, Point2 end, string sourceId = null)
        { Start = start; End = end; SourceId = sourceId; }
        public Point2 Start { get; }
        public Point2 End { get; }
        public string SourceId { get; }
        public double Length => End.Subtract(Start).Length;
    }

    public sealed class NormalizationSettings
    {
        public double LinearTolerance { get; set; } = 0.00001;
        public double AngularToleranceDegrees { get; set; } = 0.1;
        public bool MergeOverlapping { get; set; } = true;
        public bool MergeAdjacent { get; set; } = true;
    }

    public sealed class GroupDiagnostic
    {
        public int SourceCount { get; internal set; }
        public int ResultCount { get; internal set; }
        public int CorrectedCount { get; internal set; }
        public double MeanOffset { get; internal set; }
        public double MaxOffset { get; internal set; }
        public double MaxAngleDegrees { get; internal set; }
        public IReadOnlyList<string> SourceIds { get; internal set; }
        internal IReadOnlyList<Segment2> ResultSegments { get; set; }
    }

    public sealed class NormalizationResult
    {
        internal NormalizationResult(List<Segment2> segments, List<GroupDiagnostic> groups, List<Segment2> unchanged, int invalid, int rejected)
        { Segments = segments; Groups = groups; Unchanged = unchanged; InvalidCount = invalid; RejectedGroupCount = rejected; }
        public IReadOnlyList<Segment2> Segments { get; }
        public IReadOnlyList<GroupDiagnostic> Groups { get; }
        public IReadOnlyList<Segment2> Unchanged { get; }
        public int InvalidCount { get; }
        public int RejectedGroupCount { get; }
    }
}
