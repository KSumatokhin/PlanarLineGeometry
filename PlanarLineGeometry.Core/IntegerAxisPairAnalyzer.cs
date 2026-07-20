using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public enum IntegerAxisPairClass
    {
        Strict,
        Candidate
    }

    public sealed class IntegerAxisPairSettings
    {
        public double MaximumDistance { get; set; } = 3000.0;
        public double AngularToleranceDegrees { get; set; } = 0.1;
        public double StrictIntegerTolerance { get; set; } = 0.00000001;
        public double CandidateIntegerTolerance { get; set; } = 0.001;
    }

    public sealed class IntegerAxisPair
    {
        internal IntegerAxisPair(
            string sourceIdA,
            string sourceIdB,
            double distance,
            double nearestInteger,
            double deviation,
            double angleDifferenceDegrees,
            double overlapLength,
            IntegerAxisPairClass classification)
        {
            SourceIdA = sourceIdA;
            SourceIdB = sourceIdB;
            Distance = distance;
            NearestInteger = nearestInteger;
            Deviation = deviation;
            AngleDifferenceDegrees = angleDifferenceDegrees;
            OverlapLength = overlapLength;
            Classification = classification;
        }

        public string SourceIdA { get; }
        public string SourceIdB { get; }
        public double Distance { get; }
        public double NearestInteger { get; }
        public double Deviation { get; }
        public double AngleDifferenceDegrees { get; }
        public double OverlapLength { get; }
        public IntegerAxisPairClass Classification { get; }
    }

    public sealed class IntegerAxisPairAnalysis
    {
        internal IntegerAxisPairAnalysis(
            int segmentCount,
            long checkedPairCount,
            int parallelOverlappingPairCount,
            List<IntegerAxisPair> pairs)
        {
            SegmentCount = segmentCount;
            CheckedPairCount = checkedPairCount;
            ParallelOverlappingPairCount = parallelOverlappingPairCount;
            Pairs = pairs;
        }

        public int SegmentCount { get; }
        public long CheckedPairCount { get; }
        public int ParallelOverlappingPairCount { get; }
        public IReadOnlyList<IntegerAxisPair> Pairs { get; }
        public int StrictPairCount => Pairs.Count(pair => pair.Classification == IntegerAxisPairClass.Strict);
        public int CandidatePairCount => Pairs.Count(pair => pair.Classification == IntegerAxisPairClass.Candidate);
    }

    public static class IntegerAxisPairAnalyzer
    {
        public static IntegerAxisPairAnalysis Analyze(
            IEnumerable<Segment2> source,
            IntegerAxisPairSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            Validate(settings);

            List<AxisSegment> segments = source
                .Where(segment => segment != null && segment.Length > 1e-12)
                .Select(AxisSegment.Create)
                .OrderBy(segment => segment.SourceId, StringComparer.Ordinal)
                .ThenBy(segment => segment.Start.X)
                .ThenBy(segment => segment.Start.Y)
                .ThenBy(segment => segment.End.X)
                .ThenBy(segment => segment.End.Y)
                .ToList();

            long checkedPairs = 0;
            int parallelOverlapping = 0;
            var pairs = new List<IntegerAxisPair>();

            for (int i = 0; i < segments.Count; i++)
            {
                for (int j = i + 1; j < segments.Count; j++)
                {
                    checkedPairs++;
                    AxisSegment a = segments[i];
                    AxisSegment b = segments[j];
                    double angle = AngleDifferenceDegrees(a.Direction, b.Direction);
                    if (angle > settings.AngularToleranceDegrees)
                    {
                        continue;
                    }

                    double overlap = LongitudinalOverlap(a, b);
                    if (overlap <= 0)
                    {
                        continue;
                    }

                    parallelOverlapping++;
                    double distance = SymmetricPerpendicularDistance(a, b);
                    if (distance <= 1e-12 || distance > settings.MaximumDistance)
                    {
                        continue;
                    }

                    double nearestInteger = Math.Round(distance, MidpointRounding.AwayFromZero);
                    if (nearestInteger < 1.0)
                    {
                        continue;
                    }

                    double deviation = Math.Abs(distance - nearestInteger);
                    IntegerAxisPairClass classification;
                    if (deviation <= settings.StrictIntegerTolerance)
                    {
                        classification = IntegerAxisPairClass.Strict;
                    }
                    else if (deviation <= settings.CandidateIntegerTolerance)
                    {
                        classification = IntegerAxisPairClass.Candidate;
                    }
                    else
                    {
                        continue;
                    }

                    pairs.Add(new IntegerAxisPair(
                        a.SourceId,
                        b.SourceId,
                        distance,
                        nearestInteger,
                        deviation,
                        angle,
                        overlap,
                        classification));
                }
            }

            return new IntegerAxisPairAnalysis(
                segments.Count,
                checkedPairs,
                parallelOverlapping,
                pairs);
        }

        private static void Validate(IntegerAxisPairSettings settings)
        {
            if (settings.MaximumDistance <= 0) throw new ArgumentOutOfRangeException(nameof(settings.MaximumDistance));
            if (settings.AngularToleranceDegrees < 0 || settings.AngularToleranceDegrees >= 90) throw new ArgumentOutOfRangeException(nameof(settings.AngularToleranceDegrees));
            if (settings.StrictIntegerTolerance < 0) throw new ArgumentOutOfRangeException(nameof(settings.StrictIntegerTolerance));
            if (settings.CandidateIntegerTolerance < settings.StrictIntegerTolerance) throw new ArgumentOutOfRangeException(nameof(settings.CandidateIntegerTolerance));
        }

        private static double LongitudinalOverlap(AxisSegment a, AxisSegment b)
        {
            Vector2 direction = CanonicalAverageDirection(a.Direction, b.Direction);
            double a0 = Dot(a.Start, direction);
            double a1 = Dot(a.End, direction);
            double b0 = Dot(b.Start, direction);
            double b1 = Dot(b.End, direction);
            return Math.Min(Math.Max(a0, a1), Math.Max(b0, b1)) -
                   Math.Max(Math.Min(a0, a1), Math.Min(b0, b1));
        }

        private static double SymmetricPerpendicularDistance(AxisSegment a, AxisSegment b)
        {
            double fromA = DistanceToCarrier(a.Midpoint, b);
            double fromB = DistanceToCarrier(b.Midpoint, a);
            return (fromA + fromB) / 2.0;
        }

        private static double DistanceToCarrier(Point2 point, AxisSegment carrier)
        {
            Vector2 delta = point.Subtract(carrier.Start);
            return Math.Abs(delta.X * carrier.Direction.Y - delta.Y * carrier.Direction.X);
        }

        private static Vector2 CanonicalAverageDirection(Vector2 a, Vector2 b)
        {
            if (a.Dot(b) < 0) b = -1 * b;
            var sum = new Vector2(a.X + b.X, a.Y + b.Y);
            double length = sum.Length;
            return new Vector2(sum.X / length, sum.Y / length);
        }

        private static double AngleDifferenceDegrees(Vector2 a, Vector2 b)
        {
            double dot = Math.Abs(a.Dot(b));
            dot = Math.Max(-1.0, Math.Min(1.0, dot));
            return Math.Acos(dot) * 180.0 / Math.PI;
        }

        private static double Dot(Point2 point, Vector2 direction) =>
            point.X * direction.X + point.Y * direction.Y;

        private sealed class AxisSegment
        {
            private AxisSegment(Point2 start, Point2 end, string sourceId, Vector2 direction)
            {
                Start = start;
                End = end;
                SourceId = sourceId;
                Direction = direction;
                Midpoint = new Point2((start.X + end.X) / 2.0, (start.Y + end.Y) / 2.0);
            }

            public Point2 Start { get; }
            public Point2 End { get; }
            public Point2 Midpoint { get; }
            public string SourceId { get; }
            public Vector2 Direction { get; }

            public static AxisSegment Create(Segment2 segment)
            {
                Vector2 vector = segment.End.Subtract(segment.Start);
                double length = vector.Length;
                return new AxisSegment(
                    segment.Start,
                    segment.End,
                    segment.SourceId,
                    new Vector2(vector.X / length, vector.Y / length));
            }
        }
    }
}
