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
            var groups = plan.Proposals.Select(proposal => new AxisAlignedLineUnionGroup(
                proposal.Member.Axis.SourceIds,
                proposal.Proposed,
                proposal.Member.Role,
                proposal.ShiftLength)).ToList();
            return new AxisAlignedLineUnionResult(
                groups,
                corrected,
                unchanged,
                rejected,
                angular.InvalidCount,
                plan);
        }
    }
}
