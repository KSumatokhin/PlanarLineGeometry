using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public enum AxisCorrectionStatus
    {
        Anchor,
        Consistent,
        Conflict,
        Unsupported
    }

    public sealed class AxisCorrectionProposal
    {
        internal AxisCorrectionProposal(
            string sourceId,
            IReadOnlyList<string> sourceIds,
            Segment2 representative,
            AxisCorrectionStatus status,
            double shiftX,
            double shiftY,
            int supportingPairCount,
            int conflictingPairCount,
            double maximumProposalSpread)
        {
            SourceId = sourceId;
            SourceIds = sourceIds;
            Representative = representative;
            Status = status;
            ShiftX = shiftX;
            ShiftY = shiftY;
            SupportingPairCount = supportingPairCount;
            ConflictingPairCount = conflictingPairCount;
            MaximumProposalSpread = maximumProposalSpread;
        }

        public string SourceId { get; }
        public IReadOnlyList<string> SourceIds { get; }
        public Segment2 Representative { get; }
        public AxisCorrectionStatus Status { get; }
        public double ShiftX { get; }
        public double ShiftY { get; }
        public double ShiftLength => Math.Sqrt(ShiftX * ShiftX + ShiftY * ShiftY);
        public int SupportingPairCount { get; }
        public int ConflictingPairCount { get; }
        public double MaximumProposalSpread { get; }

        public Segment2 ProjectToProposedAxis(Segment2 source)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            Vector2 vector = Representative.End.Subtract(Representative.Start);
            double length = vector.Length;
            var direction = new Vector2(vector.X / length, vector.Y / length);
            Point2 origin = new Point2(
                Representative.Start.X + ShiftX,
                Representative.Start.Y + ShiftY);
            return new Segment2(
                Project(source.Start, origin, direction),
                Project(source.End, origin, direction),
                source.SourceId);
        }

        private static Point2 Project(Point2 point, Point2 origin, Vector2 direction)
        {
            double position = direction.Dot(point.Subtract(origin));
            return origin.Add(position * direction);
        }
    }

    public sealed class AxisCorrectionPlan
    {
        internal AxisCorrectionPlan(
            IntegerAxisPairAnalysis pairAnalysis,
            List<AxisCorrectionProposal> proposals,
            int conflictingAnchorPairCount)
        {
            PairAnalysis = pairAnalysis;
            Proposals = proposals;
            ConflictingAnchorPairCount = conflictingAnchorPairCount;
        }

        public IntegerAxisPairAnalysis PairAnalysis { get; }
        public IReadOnlyList<AxisCorrectionProposal> Proposals { get; }
        public int ConflictingAnchorPairCount { get; }
        public int AnchorCount => Proposals.Count(item => item.Status == AxisCorrectionStatus.Anchor);
        public int ConsistentCount => Proposals.Count(item => item.Status == AxisCorrectionStatus.Consistent);
        public int ConflictCount => Proposals.Count(item => item.Status == AxisCorrectionStatus.Conflict);
        public int UnsupportedCount => Proposals.Count(item => item.Status == AxisCorrectionStatus.Unsupported);
    }

    public static class IntegerAxisCorrectionPlanner
    {
        public static AxisCorrectionPlan Plan(
            IEnumerable<Segment2> source,
            IntegerAxisPairSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));

            List<Segment2> segments = source
                .Where(segment => segment != null && segment.Length > 1e-12)
                .OrderBy(segment => segment.SourceId, StringComparer.Ordinal)
                .ToList();
            IReadOnlyList<AxisGroup> groups = AxisGroupBuilder.Build(
                segments,
                settings.CandidateIntegerTolerance,
                settings.AngularToleranceDegrees);
            Dictionary<string, AxisGroup> byId = groups.ToDictionary(
                group => group.AxisId,
                StringComparer.Ordinal);
            IntegerAxisPairAnalysis analysis = IntegerAxisPairAnalyzer.Analyze(
                groups.Select(group => group.Representative),
                settings);
            var anchors = new HashSet<string>(StringComparer.Ordinal);
            foreach (IntegerAxisPair pair in analysis.Pairs.Where(
                pair => pair.Classification == IntegerAxisPairClass.Strict))
            {
                anchors.Add(pair.SourceIdA);
                anchors.Add(pair.SourceIdB);
            }

            var proposed = groups.ToDictionary(
                group => group.AxisId,
                group => new List<Shift>(),
                StringComparer.Ordinal);
            int conflictingAnchorPairs = 0;

            foreach (IntegerAxisPair pair in analysis.Pairs.Where(
                pair => pair.Classification == IntegerAxisPairClass.Candidate))
            {
                bool aAnchor = anchors.Contains(pair.SourceIdA);
                bool bAnchor = anchors.Contains(pair.SourceIdB);
                if (aAnchor && bAnchor)
                {
                    conflictingAnchorPairs++;
                    continue;
                }

                if (aAnchor)
                {
                    proposed[pair.SourceIdB].Add(Correction(
                        byId[pair.SourceIdA].Representative,
                        byId[pair.SourceIdB].Representative,
                        pair.NearestInteger));
                }
                else if (bAnchor)
                {
                    proposed[pair.SourceIdA].Add(Correction(
                        byId[pair.SourceIdB].Representative,
                        byId[pair.SourceIdA].Representative,
                        pair.NearestInteger));
                }
            }

            var result = new List<AxisCorrectionProposal>();
            foreach (AxisGroup group in groups)
            {
                if (anchors.Contains(group.AxisId))
                {
                    result.Add(new AxisCorrectionProposal(
                        group.AxisId,
                        group.SourceIds,
                        group.Representative,
                        AxisCorrectionStatus.Anchor,
                        0,
                        0,
                        0,
                        0,
                        0));
                    continue;
                }

                List<Shift> shifts = proposed[group.AxisId];
                if (shifts.Count == 0)
                {
                    result.Add(new AxisCorrectionProposal(
                        group.AxisId,
                        group.SourceIds,
                        group.Representative,
                        AxisCorrectionStatus.Unsupported,
                        0,
                        0,
                        0,
                        0,
                        0));
                    continue;
                }

                double meanX = shifts.Average(shift => shift.X);
                double meanY = shifts.Average(shift => shift.Y);
                double maxSpread = shifts.Max(shift => Distance(shift.X, shift.Y, meanX, meanY));
                int conflicts = shifts.Count(shift =>
                    Distance(shift.X, shift.Y, meanX, meanY) > settings.StrictIntegerTolerance);
                AxisCorrectionStatus status = conflicts == 0
                    ? AxisCorrectionStatus.Consistent
                    : AxisCorrectionStatus.Conflict;
                result.Add(new AxisCorrectionProposal(
                    group.AxisId,
                    group.SourceIds,
                    group.Representative,
                    status,
                    meanX,
                    meanY,
                    shifts.Count - conflicts,
                    conflicts,
                    maxSpread));
            }

            return new AxisCorrectionPlan(analysis, result, conflictingAnchorPairs);
        }

        private static Shift Correction(
            Segment2 anchor,
            Segment2 target,
            double nearestInteger)
        {
            Vector2 anchorDirection = CanonicalDirection(anchor);
            Vector2 targetDirection = CanonicalDirection(target);
            var anchorNormal = new Vector2(-anchorDirection.Y, anchorDirection.X);
            var targetNormal = new Vector2(-targetDirection.Y, targetDirection.X);
            Point2 anchorMidpoint = Midpoint(anchor);
            Point2 targetMidpoint = Midpoint(target);
            double signedDistance = anchorNormal.Dot(targetMidpoint.Subtract(anchorMidpoint));
            double desiredDistance = signedDistance < 0 ? -nearestInteger : nearestInteger;
            double coefficient = anchorNormal.Dot(targetNormal);
            double shift = (desiredDistance - signedDistance) / coefficient;
            return new Shift(shift * targetNormal.X, shift * targetNormal.Y);
        }

        private static Vector2 CanonicalDirection(Segment2 segment)
        {
            Vector2 vector = segment.End.Subtract(segment.Start);
            double length = vector.Length;
            double x = vector.X / length;
            double y = vector.Y / length;
            if (x < 0 || (Math.Abs(x) <= 1e-12 && y < 0))
            {
                x = -x;
                y = -y;
            }

            return new Vector2(x, y);
        }

        private static Point2 Midpoint(Segment2 segment) =>
            new Point2(
                (segment.Start.X + segment.End.X) / 2.0,
                (segment.Start.Y + segment.End.Y) / 2.0);

        private static double Distance(double x1, double y1, double x2, double y2)
        {
            double dx = x1 - x2;
            double dy = y1 - y2;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private readonly struct Shift
        {
            public Shift(double x, double y) { X = x; Y = y; }
            public double X { get; }
            public double Y { get; }
        }
    }
}
