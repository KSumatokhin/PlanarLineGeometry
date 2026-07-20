using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public sealed class GroupedAxisEvidenceSettings
    {
        public NormalizationSettings Normalization { get; set; } = new NormalizationSettings
        {
            LinearTolerance = 0.001,
            AngularToleranceDegrees = 0.1,
            MergeOverlapping = true,
            MergeAdjacent = true
        };

        public double MaximumDistance { get; set; } = 3000.0;
        public double ParallelToleranceDegrees { get; set; } = 0.000001;
    }

    public sealed class GroupedAxis
    {
        internal GroupedAxis(string id, Segment2 segment, IReadOnlyList<string> sourceIds)
        {
            Id = id;
            Segment = segment;
            SourceIds = sourceIds;
        }

        public string Id { get; }
        public Segment2 Segment { get; }
        public IReadOnlyList<string> SourceIds { get; }
    }

    public sealed class GroupedAxisEvidence
    {
        internal GroupedAxisEvidence(GroupedAxis axis, GroupedAxis partner, IntegerAxisPair pair)
        {
            Axis = axis;
            Partner = partner;
            Pair = pair;
        }

        public GroupedAxis Axis { get; }
        public GroupedAxis Partner { get; }
        public IntegerAxisPair Pair { get; }
    }

    public sealed class GroupedAxisEvidenceAnalysis
    {
        internal GroupedAxisEvidenceAnalysis(
            NormalizationResult normalization,
            List<GroupedAxis> axes,
            IntegerAxisPairAnalysis pairAnalysis,
            List<GroupedAxisEvidence> bestByAxis)
        {
            Normalization = normalization;
            Axes = axes;
            PairAnalysis = pairAnalysis;
            BestByAxis = bestByAxis;
        }

        public NormalizationResult Normalization { get; }
        public IReadOnlyList<GroupedAxis> Axes { get; }
        public IntegerAxisPairAnalysis PairAnalysis { get; }
        public IReadOnlyList<GroupedAxisEvidence> BestByAxis { get; }
        public int SupportedAxisCount => BestByAxis.Count;
    }

    /// <summary>
    /// Builds virtual ADU1 axes, then chooses the parallel partner whose
    /// perpendicular distance is closest to an integer. Overlap length only
    /// establishes that two finite axes face each other; it is not a weight.
    /// </summary>
    public static class GroupedAxisEvidenceAnalyzer
    {
        public static GroupedAxisEvidenceAnalysis Analyze(
            IEnumerable<Segment2> source,
            GroupedAxisEvidenceSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            if (settings.Normalization == null) throw new ArgumentNullException(nameof(settings.Normalization));

            NormalizationResult normalization = SelectionInvariantLineNormalizer.Normalize(
                source,
                settings.Normalization);
            List<GroupedAxis> axes = BuildAxes(normalization);
            IntegerAxisPairAnalysis pairs = IntegerAxisPairAnalyzer.Analyze(
                axes.Select(axis => new Segment2(axis.Segment.Start, axis.Segment.End, axis.Id)),
                new IntegerAxisPairSettings
                {
                    MaximumDistance = settings.MaximumDistance,
                    AngularToleranceDegrees = settings.ParallelToleranceDegrees,
                    StrictIntegerTolerance = 0.00000001,
                    CandidateIntegerTolerance = 0.5000000001
                });

            Dictionary<string, GroupedAxis> byId = axes.ToDictionary(axis => axis.Id, StringComparer.Ordinal);
            var bestByAxis = new List<GroupedAxisEvidence>();
            foreach (GroupedAxis axis in axes)
            {
                IntegerAxisPair best = pairs.Pairs
                    .Where(pair => pair.SourceIdA == axis.Id || pair.SourceIdB == axis.Id)
                    .OrderBy(pair => pair.Deviation)
                    .ThenBy(pair => OtherId(pair, axis.Id), StringComparer.Ordinal)
                    .FirstOrDefault();
                if (best == null) continue;
                bestByAxis.Add(new GroupedAxisEvidence(
                    axis,
                    byId[OtherId(best, axis.Id)],
                    best));
            }

            return new GroupedAxisEvidenceAnalysis(normalization, axes, pairs, bestByAxis);
        }

        private static List<GroupedAxis> BuildAxes(NormalizationResult normalization)
        {
            var membersByResult = new Dictionary<Segment2, IReadOnlyList<string>>();
            foreach (GroupDiagnostic group in normalization.Groups)
            {
                foreach (Segment2 result in group.ResultSegments)
                    membersByResult[result] = group.SourceIds;
            }

            var axes = new List<GroupedAxis>();
            int index = 1;
            foreach (Segment2 segment in normalization.Segments)
            {
                IReadOnlyList<string> members;
                if (!membersByResult.TryGetValue(segment, out members))
                    members = new[] { segment.SourceId };
                axes.Add(new GroupedAxis(
                    "G" + index.ToString("D6"),
                    segment,
                    members.Where(id => id != null).Distinct(StringComparer.Ordinal).OrderBy(id => id, StringComparer.Ordinal).ToList()));
                index++;
            }

            return axes;
        }

        private static string OtherId(IntegerAxisPair pair, string axisId) =>
            pair.SourceIdA == axisId ? pair.SourceIdB : pair.SourceIdA;
    }
}
