using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public sealed class AxisGroup
    {
        internal AxisGroup(string axisId, Segment2 representative, List<string> sourceIds)
        {
            AxisId = axisId;
            Representative = representative;
            SourceIds = sourceIds;
        }

        public string AxisId { get; }
        public Segment2 Representative { get; }
        public IReadOnlyList<string> SourceIds { get; }
    }

    public static class AxisGroupBuilder
    {
        private const double Epsilon = 1e-10;

        public static IReadOnlyList<AxisGroup> Build(
            IEnumerable<Segment2> source,
            double linearTolerance,
            double angularToleranceDegrees)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (linearTolerance < 0) throw new ArgumentOutOfRangeException(nameof(linearTolerance));
            if (angularToleranceDegrees < 0) throw new ArgumentOutOfRangeException(nameof(angularToleranceDegrees));

            List<LineInfo> lines = source
                .Where(segment => segment != null && segment.Length > Epsilon)
                .Select(segment => new LineInfo(segment))
                .OrderBy(line => line.Source.Start.X)
                .ThenBy(line => line.Source.Start.Y)
                .ThenBy(line => line.Source.End.X)
                .ThenBy(line => line.Source.End.Y)
                .ThenBy(line => line.Source.SourceId, StringComparer.Ordinal)
                .ToList();
            double angleTolerance = angularToleranceDegrees * Math.PI / 180.0;
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
                        if (!PairCompatible(line, member, angleTolerance, linearTolerance, out double pairDistance))
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

            return groups.Select(CreateGroup).ToList();
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
            {
                return false;
            }

            Vector2 delta = right.Midpoint.Subtract(left.Midpoint);
            double fromLeft = Math.Abs(left.Normal.Dot(delta));
            double fromRight = Math.Abs(right.Normal.Dot(delta));
            distance = Math.Max(fromLeft, fromRight);
            return distance <= linearTolerance + Epsilon;
        }

        private static AxisGroup CreateGroup(List<LineInfo> lines)
        {
            Point2 origin = new Point2(
                lines.SelectMany(line => new[] { line.Source.Start.X, line.Source.End.X }).Average(),
                lines.SelectMany(line => new[] { line.Source.Start.Y, line.Source.End.Y }).Average());
            double c = lines.Sum(line => line.Length * Math.Cos(2 * line.Theta));
            double s = lines.Sum(line => line.Length * Math.Sin(2 * line.Theta));
            double theta = NormalizeTheta(0.5 * Math.Atan2(s, c));
            var direction = new Vector2(Math.Cos(theta), Math.Sin(theta));
            var normal = new Vector2(-direction.Y, direction.X);
            double totalLength = lines.Sum(line => line.Length);
            double rho = lines.Sum(line =>
                line.Length * normal.Dot(line.Midpoint.Subtract(origin))) / totalLength;
            double min = lines.SelectMany(line => new[]
            {
                direction.Dot(line.Source.Start.Subtract(origin)),
                direction.Dot(line.Source.End.Subtract(origin))
            }).Min();
            double max = lines.SelectMany(line => new[]
            {
                direction.Dot(line.Source.Start.Subtract(origin)),
                direction.Dot(line.Source.End.Subtract(origin))
            }).Max();
            List<string> sourceIds = lines
                .Select(line => line.Source.SourceId)
                .OrderBy(id => id, StringComparer.Ordinal)
                .ToList();
            string axisId = string.Join("|", sourceIds.Select(id => id ?? string.Empty));
            Point2 axisPoint = origin.Add(rho * normal);
            var representative = new Segment2(
                axisPoint.Add(min * direction),
                axisPoint.Add(max * direction),
                axisId);
            return new AxisGroup(axisId, representative, sourceIds);
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

        private sealed class LineInfo
        {
            public LineInfo(Segment2 source)
            {
                Source = source;
                Vector2 vector = source.End.Subtract(source.Start);
                Length = vector.Length;
                Theta = NormalizeTheta(Math.Atan2(vector.Y, vector.X));
                Normal = new Vector2(-Math.Sin(Theta), Math.Cos(Theta));
                Midpoint = new Point2(
                    (source.Start.X + source.End.X) / 2.0,
                    (source.Start.Y + source.End.Y) / 2.0);
            }

            public Segment2 Source { get; }
            public double Length { get; }
            public double Theta { get; }
            public Vector2 Normal { get; }
            public Point2 Midpoint { get; }
        }
    }
}
