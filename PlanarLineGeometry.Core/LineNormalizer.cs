using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public static class LineNormalizer
    {
        private const double Epsilon = 1e-10;

        public static NormalizationResult Normalize(IEnumerable<Segment2> source, NormalizationSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            if (settings.LinearTolerance < 0 || settings.AngularToleranceDegrees < 0)
                throw new ArgumentOutOfRangeException(nameof(settings));

            var input = source.ToList();
            var unchanged = new List<Segment2>();
            var described = new List<LineInfo>();
            var ox = input.Count == 0 ? 0 : input.SelectMany(s => new[] { s.Start.X, s.End.X }).Average();
            var oy = input.Count == 0 ? 0 : input.SelectMany(s => new[] { s.Start.Y, s.End.Y }).Average();
            var origin = new Point2(ox, oy);
            foreach (var segment in input)
            {
                if (segment == null || !Finite(segment.Start) || !Finite(segment.End) || segment.Length <= Epsilon)
                { if (segment != null) unchanged.Add(segment); continue; }
                described.Add(new LineInfo(segment, origin));
            }

            var angleTolerance = settings.AngularToleranceDegrees * Math.PI / 180.0;
            described.Sort(LineInfo.Compare);
            var groups = new List<List<LineInfo>>();
            foreach (var line in described)
            {
                var group = groups.Count == 0 ? null : groups[groups.Count - 1];
                if (group == null || AxisAngle(line.Theta, group[0].Theta) > angleTolerance || Math.Abs(line.Rho - group[0].Rho) > settings.LinearTolerance)
                { group = new List<LineInfo>(); groups.Add(group); }
                group.Add(line);
            }
            if (groups.Count > 1 && Compatible(groups[groups.Count - 1][0], groups[0][0], angleTolerance, settings.LinearTolerance))
            {
                groups[0].InsertRange(0, groups[groups.Count - 1]);
                groups.RemoveAt(groups.Count - 1);
            }

            var output = new List<Segment2>();
            var diagnostics = new List<GroupDiagnostic>();
            var rejected = 0;
            foreach (var candidates in groups)
            {
                if (!TryNormalize(candidates, origin, settings, angleTolerance, out var normalized, out var diagnostic))
                { output.AddRange(candidates.Select(x => x.Source)); unchanged.AddRange(candidates.Select(x => x.Source)); rejected++; continue; }
                if (candidates.Count == 1 && normalized.Count == 1)
                { output.Add(candidates[0].Source); unchanged.Add(candidates[0].Source); continue; }
                output.AddRange(normalized);
                diagnostics.Add(diagnostic);
            }
            var describedSources = new HashSet<Segment2>(described.Select(x => x.Source));
            output.AddRange(unchanged.Where(x => x == null || x.Length <= Epsilon || !describedSources.Contains(x)));
            output.Sort(CompareSegments);
            return new NormalizationResult(output, diagnostics, unchanged.Distinct().ToList(), input.Count - described.Count, rejected);
        }

        private static bool TryNormalize(List<LineInfo> lines, Point2 origin, NormalizationSettings settings, double angleTolerance, out List<Segment2> result, out GroupDiagnostic diagnostic)
        {
            var c = lines.Sum(x => x.Length * Math.Cos(2 * x.Theta));
            var s = lines.Sum(x => x.Length * Math.Sin(2 * x.Theta));
            var theta = NormalizeTheta(0.5 * Math.Atan2(s, c));
            var d = new Vector2(Math.Cos(theta), Math.Sin(theta));
            var n = new Vector2(-d.Y, d.X);
            var totalLength = lines.Sum(x => x.Length);
            var rho = lines.Sum(x => x.Length * n.Dot(x.Midpoint.Subtract(origin))) / totalLength;
            var intervals = lines.Select(x => new Interval(Math.Min(d.Dot(x.Source.Start.Subtract(origin)), d.Dot(x.Source.End.Subtract(origin))), Math.Max(d.Dot(x.Source.Start.Subtract(origin)), d.Dot(x.Source.End.Subtract(origin))))).OrderBy(x => x.Min).ThenBy(x => x.Max).ToList();
            var merged = new List<Interval>();
            foreach (var interval in intervals)
            {
                if (merged.Count == 0 || interval.Min > merged[merged.Count - 1].Max + settings.LinearTolerance) merged.Add(interval);
                else if (interval.Max > merged[merged.Count - 1].Max) merged[merged.Count - 1] = new Interval(merged[merged.Count - 1].Min, interval.Max);
            }
            var offsets = new List<double>(); var maxAngle = 0.0;
            foreach (var line in lines)
            {
                maxAngle = Math.Max(maxAngle, AxisAngle(line.Theta, theta));
                offsets.Add(Math.Abs(n.Dot(line.Source.Start.Subtract(origin)) - rho));
                offsets.Add(Math.Abs(n.Dot(line.Source.End.Subtract(origin)) - rho));
            }
            if (maxAngle > angleTolerance + Epsilon || offsets.Max() > settings.LinearTolerance + Epsilon)
            { result = null; diagnostic = null; return false; }
            var carrier = lines.OrderByDescending(x => x.Length).ThenBy(x => x.Source.SourceId, StringComparer.Ordinal).First().Source;
            result = merged.Select(i => new Segment2(origin.Add(rho * n).Add(i.Min * d), origin.Add(rho * n).Add(i.Max * d), carrier.SourceId)).ToList();
            diagnostic = new GroupDiagnostic { SourceCount = lines.Count, ResultCount = result.Count, CorrectedCount = lines.Count(x => x.MaxOffset(origin, n, rho) > Epsilon || AxisAngle(x.Theta, theta) > Epsilon), MeanOffset = offsets.Average(), MaxOffset = offsets.Max(), MaxAngleDegrees = maxAngle * 180 / Math.PI, SourceIds = lines.Select(x => x.Source.SourceId).ToList() };
            return true;
        }

        private static bool Finite(Point2 p) => !double.IsNaN(p.X) && !double.IsInfinity(p.X) && !double.IsNaN(p.Y) && !double.IsInfinity(p.Y);
        private static double NormalizeTheta(double a) { a %= Math.PI; if (a < 0) a += Math.PI; return a; }
        private static double AxisAngle(double a, double b) { var x = Math.Abs(a - b) % Math.PI; return Math.Min(x, Math.PI - x); }
        private static bool Compatible(LineInfo a, LineInfo b, double angleTolerance, double linearTolerance)
        {
            if (AxisAngle(a.Theta, b.Theta) > angleTolerance) return false;
            var sameNormal = Math.Cos(a.Theta - b.Theta) >= 0;
            return Math.Abs(a.Rho - (sameNormal ? b.Rho : -b.Rho)) <= linearTolerance;
        }
        private static int CompareSegments(Segment2 a, Segment2 b) { int c = a.Start.X.CompareTo(b.Start.X); if (c != 0) return c; c = a.Start.Y.CompareTo(b.Start.Y); if (c != 0) return c; c = a.End.X.CompareTo(b.End.X); return c != 0 ? c : a.End.Y.CompareTo(b.End.Y); }
        private struct Interval { public Interval(double min, double max) { Min = min; Max = max; } public double Min; public double Max; }
        private sealed class LineInfo
        {
            public LineInfo(Segment2 source, Point2 origin) { Source = source; var v = source.End.Subtract(source.Start); Length = v.Length; Theta = NormalizeTheta(Math.Atan2(v.Y, v.X)); var n = new Vector2(-Math.Sin(Theta), Math.Cos(Theta)); Rho = n.Dot(source.Start.Subtract(origin)); Midpoint = new Point2((source.Start.X + source.End.X) / 2, (source.Start.Y + source.End.Y) / 2); }
            public Segment2 Source; public double Length; public double Theta; public double Rho; public Point2 Midpoint;
            public double MaxOffset(Point2 o, Vector2 n, double rho) => Math.Max(Math.Abs(n.Dot(Source.Start.Subtract(o)) - rho), Math.Abs(n.Dot(Source.End.Subtract(o)) - rho));
            public static int Compare(LineInfo a, LineInfo b) { var c = a.Theta.CompareTo(b.Theta); if (c != 0) return c; c = a.Rho.CompareTo(b.Rho); if (c != 0) return c; c = a.Source.Start.X.CompareTo(b.Source.Start.X); return c != 0 ? c : a.Source.Start.Y.CompareTo(b.Source.Start.Y); }
        }
    }
}
