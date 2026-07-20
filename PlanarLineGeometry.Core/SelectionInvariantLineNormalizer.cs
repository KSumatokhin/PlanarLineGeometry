using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    /// <summary>
    /// Experimental normalizer whose groups are independent of unrelated input.
    /// V0 remains unchanged until this model is validated on real DWG files.
    /// </summary>
    public static class SelectionInvariantLineNormalizer
    {
        private const double Epsilon = 1e-10;

        public static NormalizationResult Normalize(
            IEnumerable<Segment2> source,
            NormalizationSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            if (settings.LinearTolerance < 0 || settings.AngularToleranceDegrees < 0)
                throw new ArgumentOutOfRangeException(nameof(settings));

            var input = source.ToList();
            var valid = input
                .Where(IsValid)
                .Select(segment => new LineInfo(segment))
                .OrderBy(line => line, LineInfoComparer.Instance)
                .ToList();
            var invalid = input.Where(segment => !IsValid(segment)).ToList();
            var angleTolerance = settings.AngularToleranceDegrees * Math.PI / 180.0;
            var groups = BuildGroups(valid, angleTolerance, settings.LinearTolerance);

            var output = new List<Segment2>();
            var unchanged = new List<Segment2>(invalid);
            var diagnostics = new List<GroupDiagnostic>();
            var rejected = 0;

            foreach (List<LineInfo> candidates in groups)
            {
                if (!TryNormalize(
                    candidates,
                    settings,
                    angleTolerance,
                    out List<Segment2> normalized,
                    out GroupDiagnostic diagnostic))
                {
                    output.AddRange(candidates.Select(line => line.Source));
                    unchanged.AddRange(candidates.Select(line => line.Source));
                    rejected++;
                    continue;
                }

                if (candidates.Count == 1 && normalized.Count == 1)
                {
                    output.Add(candidates[0].Source);
                    unchanged.Add(candidates[0].Source);
                    continue;
                }

                output.AddRange(normalized);
                diagnostics.Add(diagnostic);
            }

            output.AddRange(invalid);
            output.Sort(CompareSegments);
            return new NormalizationResult(
                output,
                diagnostics,
                unchanged.Distinct().ToList(),
                invalid.Count,
                rejected);
        }

        private static List<List<LineInfo>> BuildGroups(
            List<LineInfo> lines,
            double angleTolerance,
            double linearTolerance)
        {
            var groups = new List<List<LineInfo>>();
            foreach (LineInfo line in lines)
            {
                List<LineInfo> best = null;
                double bestDistance = double.MaxValue;

                foreach (List<LineInfo> group in groups)
                {
                    double distance = 0;
                    bool compatible = true;
                    foreach (LineInfo member in group)
                    {
                        if (!PairCompatible(
                            line,
                            member,
                            angleTolerance,
                            linearTolerance,
                            out double pairDistance))
                        {
                            compatible = false;
                            break;
                        }

                        distance = Math.Max(distance, pairDistance);
                    }

                    if (compatible && distance < bestDistance)
                    {
                        best = group;
                        bestDistance = distance;
                    }
                }

                if (best == null)
                {
                    best = new List<LineInfo>();
                    groups.Add(best);
                }

                best.Add(line);
            }

            return groups;
        }

        private static bool PairCompatible(
            LineInfo left,
            LineInfo right,
            double angleTolerance,
            double linearTolerance,
            out double distance)
        {
            distance = double.MaxValue;
            if (AxisAngle(left.Theta, right.Theta) > angleTolerance + Epsilon)
                return false;

            Vector2 delta = right.Midpoint.Subtract(left.Midpoint);
            double fromLeft = Math.Abs(left.Normal.Dot(delta));
            double fromRight = Math.Abs(right.Normal.Dot(delta));
            distance = Math.Max(fromLeft, fromRight);
            return distance <= linearTolerance + Epsilon;
        }

        private static bool TryNormalize(
            List<LineInfo> lines,
            NormalizationSettings settings,
            double angleTolerance,
            out List<Segment2> result,
            out GroupDiagnostic diagnostic)
        {
            Point2 origin = LocalOrigin(lines);
            double c = lines.Sum(line => line.Length * Math.Cos(2 * line.Theta));
            double s = lines.Sum(line => line.Length * Math.Sin(2 * line.Theta));
            double theta = NormalizeTheta(0.5 * Math.Atan2(s, c));
            var direction = new Vector2(Math.Cos(theta), Math.Sin(theta));
            var normal = new Vector2(-direction.Y, direction.X);
            double totalLength = lines.Sum(line => line.Length);
            double rho = lines.Sum(line =>
                line.Length * normal.Dot(line.Midpoint.Subtract(origin))) / totalLength;

            var intervals = lines
                .Select(line => CreateInterval(line.Source, origin, direction))
                .OrderBy(interval => interval.Min)
                .ThenBy(interval => interval.Max)
                .ToList();
            var merged = MergeIntervals(intervals, settings);
            var offsets = new List<double>();
            double maxAngle = 0;

            foreach (LineInfo line in lines)
            {
                maxAngle = Math.Max(maxAngle, AxisAngle(line.Theta, theta));
                offsets.Add(Math.Abs(normal.Dot(line.Source.Start.Subtract(origin)) - rho));
                offsets.Add(Math.Abs(normal.Dot(line.Source.End.Subtract(origin)) - rho));
            }

            if (maxAngle > angleTolerance + Epsilon ||
                offsets.Max() > settings.LinearTolerance + Epsilon)
            {
                result = null;
                diagnostic = null;
                return false;
            }

            Segment2 carrier = lines
                .OrderByDescending(line => line.Length)
                .ThenBy(line => line.Source.SourceId, StringComparer.Ordinal)
                .First().Source;
            result = merged.Select(interval => new Segment2(
                origin.Add(rho * normal).Add(interval.Min * direction),
                origin.Add(rho * normal).Add(interval.Max * direction),
                carrier.SourceId)).ToList();
            diagnostic = new GroupDiagnostic
            {
                SourceCount = lines.Count,
                ResultCount = result.Count,
                CorrectedCount = lines.Count(line =>
                    line.MaxOffset(origin, normal, rho) > Epsilon ||
                    AxisAngle(line.Theta, theta) > Epsilon),
                MeanOffset = offsets.Average(),
                MaxOffset = offsets.Max(),
                MaxAngleDegrees = maxAngle * 180 / Math.PI,
                SourceIds = lines.Select(line => line.Source.SourceId).ToList()
            };
            return true;
        }

        private static List<Interval> MergeIntervals(
            List<Interval> intervals,
            NormalizationSettings settings)
        {
            var result = new List<Interval>();
            foreach (Interval interval in intervals)
            {
                if (result.Count == 0 || !ShouldMerge(result[result.Count - 1], interval, settings))
                    result.Add(interval);
                else if (interval.Max > result[result.Count - 1].Max)
                    result[result.Count - 1] = new Interval(result[result.Count - 1].Min, interval.Max);
            }

            return result;
        }

        private static bool ShouldMerge(
            Interval current,
            Interval next,
            NormalizationSettings settings)
        {
            if (Math.Abs(current.Min - next.Min) <= Epsilon &&
                Math.Abs(current.Max - next.Max) <= Epsilon)
                return true;

            double gap = next.Min - current.Max;
            if (gap < -Epsilon) return settings.MergeOverlapping;
            return settings.MergeAdjacent && gap <= settings.LinearTolerance + Epsilon;
        }

        private static Interval CreateInterval(
            Segment2 segment,
            Point2 origin,
            Vector2 direction)
        {
            double start = direction.Dot(segment.Start.Subtract(origin));
            double end = direction.Dot(segment.End.Subtract(origin));
            return new Interval(Math.Min(start, end), Math.Max(start, end));
        }

        private static Point2 LocalOrigin(List<LineInfo> lines)
        {
            return new Point2(
                lines.SelectMany(line => new[] { line.Source.Start.X, line.Source.End.X }).Average(),
                lines.SelectMany(line => new[] { line.Source.Start.Y, line.Source.End.Y }).Average());
        }

        private static bool IsValid(Segment2 segment)
        {
            return segment != null &&
                   Finite(segment.Start) &&
                   Finite(segment.End) &&
                   segment.Length > Epsilon;
        }

        private static bool Finite(Point2 point)
        {
            return !double.IsNaN(point.X) && !double.IsInfinity(point.X) &&
                   !double.IsNaN(point.Y) && !double.IsInfinity(point.Y);
        }

        private static double NormalizeTheta(double angle)
        {
            angle %= Math.PI;
            if (angle < 0) angle += Math.PI;
            return angle;
        }

        private static double AxisAngle(double left, double right)
        {
            double angle = Math.Abs(left - right) % Math.PI;
            return Math.Min(angle, Math.PI - angle);
        }

        private static int CompareSegments(Segment2 left, Segment2 right)
        {
            int result = left.Start.X.CompareTo(right.Start.X);
            if (result != 0) return result;
            result = left.Start.Y.CompareTo(right.Start.Y);
            if (result != 0) return result;
            result = left.End.X.CompareTo(right.End.X);
            return result != 0 ? result : left.End.Y.CompareTo(right.End.Y);
        }

        private struct Interval
        {
            public Interval(double min, double max) { Min = min; Max = max; }
            public double Min;
            public double Max;
        }

        private sealed class LineInfo
        {
            public LineInfo(Segment2 source)
            {
                Source = source;
                Vector2 vector = source.End.Subtract(source.Start);
                Length = vector.Length;
                Theta = NormalizeTheta(Math.Atan2(vector.Y, vector.X));
                Direction = new Vector2(Math.Cos(Theta), Math.Sin(Theta));
                Normal = new Vector2(-Direction.Y, Direction.X);
                Midpoint = new Point2(
                    (source.Start.X + source.End.X) / 2,
                    (source.Start.Y + source.End.Y) / 2);
            }

            public Segment2 Source { get; }
            public double Length { get; }
            public double Theta { get; }
            public Vector2 Direction { get; }
            public Vector2 Normal { get; }
            public Point2 Midpoint { get; }

            public double MaxOffset(Point2 origin, Vector2 normal, double rho)
            {
                return Math.Max(
                    Math.Abs(normal.Dot(Source.Start.Subtract(origin)) - rho),
                    Math.Abs(normal.Dot(Source.End.Subtract(origin)) - rho));
            }
        }

        private sealed class LineInfoComparer : IComparer<LineInfo>
        {
            public static readonly LineInfoComparer Instance = new LineInfoComparer();

            public int Compare(LineInfo left, LineInfo right)
            {
                int result = left.Source.Start.X.CompareTo(right.Source.Start.X);
                if (result != 0) return result;
                result = left.Source.Start.Y.CompareTo(right.Source.Start.Y);
                if (result != 0) return result;
                result = left.Source.End.X.CompareTo(right.Source.End.X);
                if (result != 0) return result;
                result = left.Source.End.Y.CompareTo(right.Source.End.Y);
                if (result != 0) return result;
                return string.Compare(
                    left.Source.SourceId,
                    right.Source.SourceId,
                    StringComparison.Ordinal);
            }
        }
    }
}
