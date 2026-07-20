using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public sealed class AxisAlignedLineUnionSettings
    {
        public double AngularStepDegrees { get; set; } = 1.0;
        public double MaximumEndpointShift { get; set; } = 0.025;
        public GroupedAxisEvidenceSettings AxisEvidence { get; set; } = new GroupedAxisEvidenceSettings();
    }

    public sealed class AxisAlignedLineUnionGroup
    {
        internal AxisAlignedLineUnionGroup(
            IReadOnlyList<string> sourceIds,
            Segment2 result,
            GroupedAxisSystemRole role,
            double axisShift)
        {
            SourceIds = sourceIds;
            Result = result;
            Role = role;
            AxisShift = axisShift;
        }

        public IReadOnlyList<string> SourceIds { get; }
        public Segment2 Result { get; }
        public GroupedAxisSystemRole Role { get; }
        public double AxisShift { get; }
    }

    public sealed class AxisAlignedLineUnionResult
    {
        internal AxisAlignedLineUnionResult(
            List<AxisAlignedLineUnionGroup> groups,
            int angularCorrected,
            int angularUnchanged,
            int angularRejected,
            int invalid,
            GroupedAxisCorrectionPlan correctionPlan)
        {
            Groups = groups;
            AngularCorrectedCount = angularCorrected;
            AngularUnchangedCount = angularUnchanged;
            AngularRejectedCount = angularRejected;
            InvalidCount = invalid;
            CorrectionPlan = correctionPlan;
        }

        public IReadOnlyList<AxisAlignedLineUnionGroup> Groups { get; }
        public int AngularCorrectedCount { get; }
        public int AngularUnchangedCount { get; }
        public int AngularRejectedCount { get; }
        public int InvalidCount { get; }
        public GroupedAxisCorrectionPlan CorrectionPlan { get; }
    }

    /// <summary>
    /// Complete V2 pipeline: angular snapping, selection-invariant grouping,
    /// mutual axis systems, symmetric integer correction and final axes.
    /// </summary>
    public static class AxisAlignedLineUnionPipeline
    {
        private const double Epsilon = 1e-12;

        public static AxisAlignedLineUnionResult Run(
            IEnumerable<Segment2> source,
            AxisAlignedLineUnionSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            if (settings.MaximumEndpointShift < 0) throw new ArgumentOutOfRangeException(nameof(settings.MaximumEndpointShift));
            if (settings.AxisEvidence == null) throw new ArgumentNullException(nameof(settings.AxisEvidence));

            AngularStepAnalysis angular = AngularStepAnalyzer.Analyze(
                source,
                new AngularStepSettings { StepDegrees = settings.AngularStepDegrees });
            var aligned = new List<Segment2>();
            int corrected = 0;
            int unchanged = 0;
            int rejected = 0;
            foreach (AngularStepDiagnostic line in angular.Lines)
            {
                if (line.EndpointShift > settings.MaximumEndpointShift + Epsilon)
                {
                    aligned.Add(line.Source);
                    rejected++;
                }
                else if (line.EndpointShift <= Epsilon)
                {
                    aligned.Add(line.Source);
                    unchanged++;
                }
                else
                {
                    aligned.Add(line.Proposed);
                    corrected++;
                }
            }

            GroupedAxisCorrectionPlan plan = GroupedAxisCorrectionPlanAnalyzer.Plan(
                aligned,
                settings.AxisEvidence);
            Dictionary<string, GroupedAxisCorrectionProposal> carrierProposals = plan.Proposals
                .GroupBy(proposal => CarrierKey(proposal.Member.Axis.SourceIds), StringComparer.Ordinal)
                .ToDictionary(
                    group => group.Key,
                    group => group
                        .OrderByDescending(proposal => proposal.Member.Axis.Segment.Length)
                        .ThenBy(proposal => proposal.Member.Axis.Id, StringComparer.Ordinal)
                        .First(),
                    StringComparer.Ordinal);
            var correctedGroups = plan.Proposals.Select((proposal, index) =>
            {
                GroupedAxisCorrectionProposal carrier = carrierProposals[CarrierKey(proposal.Member.Axis.SourceIds)];
                Segment2 sourceAxis = proposal.Member.Axis.Segment;
                var result = new Segment2(
                    new Point2(sourceAxis.Start.X + carrier.ShiftX, sourceAxis.Start.Y + carrier.ShiftY),
                    new Point2(sourceAxis.End.X + carrier.ShiftX, sourceAxis.End.Y + carrier.ShiftY),
                    "P" + (index + 1).ToString("D6"));
                return new AxisAlignedLineUnionGroup(
                    proposal.Member.Axis.SourceIds,
                    result,
                    proposal.Member.Role,
                    carrier.ShiftLength);
            }).ToList();
            List<AxisAlignedLineUnionGroup> groups = FinalizeSetUnion(correctedGroups, settings.AxisEvidence.Normalization);
            return new AxisAlignedLineUnionResult(
                groups,
                corrected,
                unchanged,
                rejected,
                angular.InvalidCount,
                plan);
        }

        private static string CarrierKey(IReadOnlyList<string> sourceIds) =>
            string.Join("\u001f", sourceIds.OrderBy(id => id, StringComparer.Ordinal));

        private static List<AxisAlignedLineUnionGroup> FinalizeSetUnion(
            IReadOnlyList<AxisAlignedLineUnionGroup> corrected,
            NormalizationSettings settings)
        {
            Dictionary<string, AxisAlignedLineUnionGroup> byId = corrected.ToDictionary(group => group.Result.SourceId, StringComparer.Ordinal);
            NormalizationResult union = SelectionInvariantLineNormalizer.Normalize(corrected.Select(group => group.Result), settings);
            var membersByResult = new Dictionary<Segment2, IReadOnlyList<string>>();
            foreach (GroupDiagnostic diagnostic in union.Groups)
                foreach (Segment2 result in diagnostic.ResultSegments)
                    membersByResult[result] = diagnostic.SourceIds;

            var final = new List<AxisAlignedLineUnionGroup>();
            foreach (Segment2 result in union.Segments)
            {
                IReadOnlyList<string> memberIds;
                if (!membersByResult.TryGetValue(result, out memberIds)) memberIds = new[] { result.SourceId };
                List<AxisAlignedLineUnionGroup> members = memberIds.Select(id => byId[id]).ToList();
                AxisAlignedLineUnionGroup representative = members
                    .OrderByDescending(group => group.Result.Length)
                    .ThenBy(group => group.Result.SourceId, StringComparer.Ordinal)
                    .First();
                IReadOnlyList<string> originalIds = members
                    .SelectMany(group => group.SourceIds)
                    .Distinct(StringComparer.Ordinal)
                    .OrderBy(id => id, StringComparer.Ordinal)
                    .ToList();
                final.Add(new AxisAlignedLineUnionGroup(
                    originalIds,
                    result,
                    representative.Role,
                    members.Max(group => group.AxisShift)));
            }
            return final;
        }
    }
}
