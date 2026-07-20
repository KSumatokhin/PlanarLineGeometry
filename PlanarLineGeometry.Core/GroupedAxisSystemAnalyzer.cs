using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public enum GroupedAxisSystemRole
    {
        Seed,
        Attached,
        Unresolved
    }

    public sealed class GroupedAxisSystemMember
    {
        internal GroupedAxisSystemMember(
            GroupedAxis axis,
            GroupedAxisSystemRole role,
            string systemId,
            int depth,
            GroupedAxisEvidence evidence)
        {
            Axis = axis;
            Role = role;
            SystemId = systemId;
            Depth = depth;
            Evidence = evidence;
        }

        public GroupedAxis Axis { get; }
        public GroupedAxisSystemRole Role { get; }
        public string SystemId { get; }
        public int Depth { get; }
        public GroupedAxisEvidence Evidence { get; }
    }

    public sealed class GroupedAxisSystem
    {
        internal GroupedAxisSystem(
            string id,
            GroupedAxis seedA,
            GroupedAxis seedB,
            List<GroupedAxisSystemMember> members)
        {
            Id = id;
            SeedA = seedA;
            SeedB = seedB;
            Members = members;
        }

        public string Id { get; }
        public GroupedAxis SeedA { get; }
        public GroupedAxis SeedB { get; }
        public IReadOnlyList<GroupedAxisSystemMember> Members { get; }
        public int MaximumDepth => Members.Count == 0 ? 0 : Members.Max(member => member.Depth);
    }

    public sealed class GroupedAxisSystemAnalysis
    {
        internal GroupedAxisSystemAnalysis(
            GroupedAxisEvidenceAnalysis evidence,
            List<GroupedAxisSystem> systems,
            List<GroupedAxisSystemMember> members)
        {
            Evidence = evidence;
            Systems = systems;
            Members = members;
        }

        public GroupedAxisEvidenceAnalysis Evidence { get; }
        public IReadOnlyList<GroupedAxisSystem> Systems { get; }
        public IReadOnlyList<GroupedAxisSystemMember> Members { get; }
        public int SeedAxisCount => Members.Count(member => member.Role == GroupedAxisSystemRole.Seed);
        public int AttachedAxisCount => Members.Count(member => member.Role == GroupedAxisSystemRole.Attached);
        public int UnresolvedAxisCount => Members.Count(member => member.Role == GroupedAxisSystemRole.Unresolved);
    }

    /// <summary>
    /// Treats mutual best partners as trusted two-axis seeds. Directed best-partner
    /// chains may attach to a seed, but never create a new seed themselves.
    /// </summary>
    public static class GroupedAxisSystemAnalyzer
    {
        public static GroupedAxisSystemAnalysis Analyze(
            IEnumerable<Segment2> source,
            GroupedAxisEvidenceSettings settings)
        {
            GroupedAxisEvidenceAnalysis evidence = GroupedAxisEvidenceAnalyzer.Analyze(source, settings);
            Dictionary<string, GroupedAxisEvidence> best = evidence.BestByAxis.ToDictionary(
                item => item.Axis.Id,
                StringComparer.Ordinal);
            var seedPairs = new List<Tuple<string, string>>();
            foreach (GroupedAxisEvidence item in evidence.BestByAxis)
            {
                GroupedAxisEvidence reverse;
                if (!best.TryGetValue(item.Partner.Id, out reverse) || reverse.Partner.Id != item.Axis.Id)
                    continue;
                string left = string.CompareOrdinal(item.Axis.Id, item.Partner.Id) < 0 ? item.Axis.Id : item.Partner.Id;
                string right = left == item.Axis.Id ? item.Partner.Id : item.Axis.Id;
                if (!seedPairs.Any(pair => pair.Item1 == left && pair.Item2 == right))
                    seedPairs.Add(Tuple.Create(left, right));
            }
            seedPairs = seedPairs.OrderBy(pair => pair.Item1, StringComparer.Ordinal).ThenBy(pair => pair.Item2, StringComparer.Ordinal).ToList();

            Dictionary<string, GroupedAxis> axes = evidence.Axes.ToDictionary(axis => axis.Id, StringComparer.Ordinal);
            var assigned = new Dictionary<string, GroupedAxisSystemMember>(StringComparer.Ordinal);
            var systems = new List<GroupedAxisSystem>();
            int systemNumber = 1;
            foreach (Tuple<string, string> pair in seedPairs)
            {
                string systemId = "S" + systemNumber.ToString("D6");
                systemNumber++;
                var members = new List<GroupedAxisSystemMember>();
                AddSeed(pair.Item1, systemId, axes, best, assigned, members);
                AddSeed(pair.Item2, systemId, axes, best, assigned, members);
                systems.Add(new GroupedAxisSystem(systemId, axes[pair.Item1], axes[pair.Item2], members));
            }

            bool changed;
            do
            {
                changed = false;
                foreach (GroupedAxisEvidence item in evidence.BestByAxis.OrderBy(value => value.Axis.Id, StringComparer.Ordinal))
                {
                    if (assigned.ContainsKey(item.Axis.Id)) continue;
                    GroupedAxisSystemMember parent;
                    if (!assigned.TryGetValue(item.Partner.Id, out parent)) continue;
                    var member = new GroupedAxisSystemMember(
                        item.Axis,
                        GroupedAxisSystemRole.Attached,
                        parent.SystemId,
                        parent.Depth + 1,
                        item);
                    assigned[item.Axis.Id] = member;
                    systems.Single(system => system.Id == parent.SystemId).MembersInternal().Add(member);
                    changed = true;
                }
            } while (changed);

            var allMembers = new List<GroupedAxisSystemMember>(assigned.Values);
            foreach (GroupedAxis axis in evidence.Axes.Where(axis => !assigned.ContainsKey(axis.Id)))
            {
                GroupedAxisEvidence item;
                best.TryGetValue(axis.Id, out item);
                allMembers.Add(new GroupedAxisSystemMember(
                    axis,
                    GroupedAxisSystemRole.Unresolved,
                    null,
                    -1,
                    item));
            }

            allMembers.Sort((left, right) => string.CompareOrdinal(left.Axis.Id, right.Axis.Id));
            return new GroupedAxisSystemAnalysis(evidence, systems, allMembers);
        }

        private static void AddSeed(
            string axisId,
            string systemId,
            Dictionary<string, GroupedAxis> axes,
            Dictionary<string, GroupedAxisEvidence> best,
            Dictionary<string, GroupedAxisSystemMember> assigned,
            List<GroupedAxisSystemMember> members)
        {
            var member = new GroupedAxisSystemMember(
                axes[axisId],
                GroupedAxisSystemRole.Seed,
                systemId,
                0,
                best[axisId]);
            assigned[axisId] = member;
            members.Add(member);
        }

        private static List<GroupedAxisSystemMember> MembersInternal(this GroupedAxisSystem system) =>
            (List<GroupedAxisSystemMember>)system.Members;
    }
}
