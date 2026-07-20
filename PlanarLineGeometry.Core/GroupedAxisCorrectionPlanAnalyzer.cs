using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public sealed class GroupedAxisCorrectionProposal
    {
        internal GroupedAxisCorrectionProposal(
            GroupedAxisSystemMember member,
            Segment2 proposed,
            double shiftX,
            double shiftY)
        {
            Member = member;
            Proposed = proposed;
            ShiftX = shiftX;
            ShiftY = shiftY;
        }

        public GroupedAxisSystemMember Member { get; }
        public Segment2 Proposed { get; }
        public double ShiftX { get; }
        public double ShiftY { get; }
        public double ShiftLength => Math.Sqrt(ShiftX * ShiftX + ShiftY * ShiftY);
    }

    public sealed class GroupedAxisCorrectionPlan
    {
        internal GroupedAxisCorrectionPlan(
            GroupedAxisSystemAnalysis systems,
            List<GroupedAxisCorrectionProposal> proposals)
        {
            Systems = systems;
            Proposals = proposals;
        }

        public GroupedAxisSystemAnalysis Systems { get; }
        public IReadOnlyList<GroupedAxisCorrectionProposal> Proposals { get; }
        public double MaximumShift => Proposals.Count == 0 ? 0 : Proposals.Max(proposal => proposal.ShiftLength);
    }

    /// <summary>
    /// Corrects each mutual seed symmetrically around its common middle carrier,
    /// then places attached axes at exact signed integer distances from their
    /// already corrected parent. Unresolved axes remain unchanged.
    /// </summary>
    public static class GroupedAxisCorrectionPlanAnalyzer
    {
        public static GroupedAxisCorrectionPlan Plan(
            IEnumerable<Segment2> source,
            GroupedAxisEvidenceSettings settings)
        {
            GroupedAxisSystemAnalysis systems = GroupedAxisSystemAnalyzer.Analyze(source, settings);
            var proposedByAxis = new Dictionary<string, Segment2>(StringComparer.Ordinal);
            var proposals = new List<GroupedAxisCorrectionProposal>();

            foreach (GroupedAxisSystem system in systems.Systems.OrderBy(item => item.Id, StringComparer.Ordinal))
            {
                Vector2 direction = CanonicalDirection(system.SeedA.Segment);
                Vector2 normal = new Vector2(-direction.Y, direction.X);
                GroupedAxisSystemMember seedA = system.Members.Single(member => member.Axis.Id == system.SeedA.Id);
                GroupedAxisSystemMember seedB = system.Members.Single(member => member.Axis.Id == system.SeedB.Id);
                double rhoA = CarrierCoordinate(seedA.Axis.Segment, normal);
                double rhoB = CarrierCoordinate(seedB.Axis.Segment, normal);
                double signedDistance = rhoB - rhoA;
                double target = SignedTarget(signedDistance, seedA.Evidence.Pair.NearestInteger);
                double correction = target - signedDistance;
                AddProposal(seedA, -correction / 2.0, normal, proposedByAxis, proposals);
                AddProposal(seedB, correction / 2.0, normal, proposedByAxis, proposals);

                foreach (GroupedAxisSystemMember member in system.Members
                    .Where(item => item.Role == GroupedAxisSystemRole.Attached)
                    .OrderBy(item => item.Depth)
                    .ThenBy(item => item.Axis.Id, StringComparer.Ordinal))
                {
                    Segment2 parent = proposedByAxis[member.Evidence.Partner.Id];
                    double parentRho = CarrierCoordinate(parent, normal);
                    double currentRho = CarrierCoordinate(member.Axis.Segment, normal);
                    double parentOriginalRho = CarrierCoordinate(member.Evidence.Partner.Segment, normal);
                    double signedOriginalDistance = currentRho - parentOriginalRho;
                    double desiredRho = parentRho + SignedTarget(
                        signedOriginalDistance,
                        member.Evidence.Pair.NearestInteger);
                    AddProposal(member, desiredRho - currentRho, normal, proposedByAxis, proposals);
                }
            }

            foreach (GroupedAxisSystemMember member in systems.Members
                .Where(item => item.Role == GroupedAxisSystemRole.Unresolved))
            {
                AddProposal(member, 0, new Vector2(0, 0), proposedByAxis, proposals);
            }

            proposals.Sort((left, right) => string.CompareOrdinal(left.Member.Axis.Id, right.Member.Axis.Id));
            return new GroupedAxisCorrectionPlan(systems, proposals);
        }

        private static void AddProposal(
            GroupedAxisSystemMember member,
            double scalarShift,
            Vector2 normal,
            Dictionary<string, Segment2> proposedByAxis,
            List<GroupedAxisCorrectionProposal> proposals)
        {
            double shiftX = scalarShift * normal.X;
            double shiftY = scalarShift * normal.Y;
            Segment2 source = member.Axis.Segment;
            var proposed = new Segment2(
                new Point2(source.Start.X + shiftX, source.Start.Y + shiftY),
                new Point2(source.End.X + shiftX, source.End.Y + shiftY),
                source.SourceId);
            proposedByAxis[member.Axis.Id] = proposed;
            proposals.Add(new GroupedAxisCorrectionProposal(member, proposed, shiftX, shiftY));
        }

        private static Vector2 CanonicalDirection(Segment2 segment)
        {
            Vector2 vector = segment.End.Subtract(segment.Start);
            double length = vector.Length;
            var direction = new Vector2(vector.X / length, vector.Y / length);
            if (direction.X < 0 || (Math.Abs(direction.X) <= 1e-12 && direction.Y < 0))
                direction = -1 * direction;
            return direction;
        }

        private static double CarrierCoordinate(Segment2 segment, Vector2 normal)
        {
            double x = (segment.Start.X + segment.End.X) / 2.0;
            double y = (segment.Start.Y + segment.End.Y) / 2.0;
            return x * normal.X + y * normal.Y;
        }

        private static double SignedTarget(double signedDistance, double magnitude) =>
            signedDistance < 0 ? -magnitude : magnitude;
    }
}
