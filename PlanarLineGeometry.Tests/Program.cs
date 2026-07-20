using System;
using System.Collections.Generic;
using System.Linq;
using PlanarLineGeometry;

namespace PlanarLineGeometry.Tests
{
    internal static class Program
    {
        private static int passed;
        private static readonly NormalizationSettings Default = new NormalizationSettings
        {
            LinearTolerance = 1.0,
            AngularToleranceDegrees = 0.1
        };
        private static int Main()
        {
            try
            {
                Run("conservative default tolerance", () => Near(.00001, new NormalizationSettings().LinearTolerance, 1e-12));
                Run("overlap option", OverlapOption);
                Run("adjacent option", AdjacentOption);
                Run("duplicates remain normalized", DuplicateOption);
                Run("shared endpoint", () => AssertSpan(N(S(0,0,10,0), S(10,0,20,0)), 0, 20));
                Run("reversed endpoints", () => AssertSpan(N(S(10,0,0,0), S(20,0,10,0)), 0, 20));
                Run("duplicates", () => Eq(1, N(S(0,0,10,0), S(0,0,10,0)).Segments.Count));
                Run("partial overlap", () => AssertSpan(N(S(0,0,10,0), S(5,0,15,0)), 0, 15));
                Run("nested", () => AssertSpan(N(S(0,0,100,0), S(20,0,30,0)), 0, 100));
                Run("gap within tolerance", () => Eq(1, N(S(0,0,10,0), S(10.8,0,20,0)).Segments.Count));
                Run("gap beyond tolerance", () => Eq(2, N(S(0,0,10,0), S(11.2,0,20,0)).Segments.Count));
                Run("mean axis", () => Near(.3, N(S(0,0,10,0), S(0,.6,10,.6)).Segments[0].Start.Y));
                Run("offset rejected", () => Eq(2, N(S(0,0,10,0), S(0,1.1,10,1.1)).Segments.Count));
                Run("angle accepted", () => Eq(1, N(S(0,0,10,0), Angle(0.05, 10)).Segments.Count));
                Run("angle rejected", () => Eq(2, N(S(0,0,10,0), Angle(0.2, 10)).Segments.Count));
                Run("parallel walls", () => Eq(2, N(S(0,0,10,0), S(0,3,10,3)).Segments.Count));
                Run("chain drift rejected", ChainDriftRejected);
                Run("large gaps", () => Eq(3, N(S(0,0,10,0), S(20,0,30,0), S(40,0,50,0)).Segments.Count));
                Run("0 and 180", () => Eq(1, N(S(0,0,10,0), S(20,0,10,0)).Segments.Count));
                Run("angle wrap", AngleWrap);
                Run("zero length", () => Eq(1, N(S(1,1,1,1)).InvalidCount));
                Run("nested without endpoint search", () => AssertSpan(N(S(0,0,1000,0), S(400,0,401,0)), 0, 1000));
                Run("shuffle deterministic", ShuffleDeterministic);
                Run("coordinates remain exact", () => Near(10.123456789, N(S(.123456789,0,10.123456789,0), S(5,0,6,0)).Segments[0].End.X, 1e-12));
                Run("V1 selection extension invariant", SelectionExtensionInvariant);
                Run("integer axis strict pair", IntegerAxisStrictPair);
                Run("integer axis practical candidate", IntegerAxisPracticalCandidate);
                Run("integer axis requires overlap", IntegerAxisRequiresOverlap);
                Run("integer axis limits distance", IntegerAxisLimitsDistance);
                Run("integer axis excludes zero width", IntegerAxisExcludesZeroWidth);
                Run("axis plan anchors strict pair", AxisPlanAnchorsStrictPair);
                Run("axis plan proposes supported correction", AxisPlanProposesSupportedCorrection);
                Run("axis plan leaves unsupported line", AxisPlanLeavesUnsupportedLine);
                Run("axis plan reports conflicting anchors", AxisPlanReportsConflictingAnchors);
                Run("angular step keeps exact angle", AngularStepKeepsExactAngle);
                Run("angular step reports endpoint displacement", AngularStepReportsEndpointDisplacement);
                Run("angular step wraps around 180", AngularStepWrapsAround180);
                Run("group axes retain source fragments", GroupAxesRetainSourceFragments);
                Run("group axis chooses nearest integer evidence", GroupAxisChoosesNearestIntegerEvidence);
                Run("mutual group axes seed a system", MutualGroupAxesSeedSystem);
                Run("one-way evidence attaches to seed", OneWayEvidenceAttachesToSeed);
                Run("seed correction is symmetric", SeedCorrectionIsSymmetric);
                Run("attached correction follows corrected parent", AttachedCorrectionFollowsCorrectedParent);
                Console.WriteLine("PlanarLineGeometry.Tests: " + passed + " tests passed."); return 0;
            }
            catch(Exception e) { Console.Error.WriteLine(e.Message); return 1; }
        }
        private static NormalizationResult N(params Segment2[] s) => LineNormalizer.Normalize(s, Default);
        private static Segment2 S(double x1,double y1,double x2,double y2,string id=null) => new Segment2(new Point2(x1,y1),new Point2(x2,y2),id);
        private static Segment2 Angle(double degrees,double length) { var a=degrees*Math.PI/180; return S(0,0,length*Math.Cos(a),length*Math.Sin(a)); }
        private static void ChainDriftRejected() { var settings=new NormalizationSettings{LinearTolerance=1,AngularToleranceDegrees=.1}; var r=LineNormalizer.Normalize(new[]{S(0,0,10,0),S(0,.9,10,.9),S(0,1.8,10,1.8)},settings); Eq(2,r.Segments.Count); }
        private static void AngleWrap() { var a=Angle(.05,10); var b=S(0,0,-10*Math.Cos(.05*Math.PI/180),10*Math.Sin(.05*Math.PI/180)); Eq(1,N(a,b).Segments.Count); }
        private static void OverlapOption() { var settings=Settings(false,true); Eq(2,LineNormalizer.Normalize(new[]{S(0,0,10,0),S(5,0,15,0)},settings).Segments.Count); }
        private static void AdjacentOption() { var settings=Settings(true,false); Eq(2,LineNormalizer.Normalize(new[]{S(0,0,10,0),S(10,0,20,0)},settings).Segments.Count); }
        private static void DuplicateOption() { var settings=Settings(false,false); Eq(1,LineNormalizer.Normalize(new[]{S(0,0,10,0),S(0,0,10,0)},settings).Segments.Count); }
        private static NormalizationSettings Settings(bool overlap,bool adjacent) => new NormalizationSettings { LinearTolerance=1, AngularToleranceDegrees=.1, MergeOverlapping=overlap, MergeAdjacent=adjacent };
        private static void ShuffleDeterministic() { var a=new[]{S(10,0,20,0),S(0,0,10,0),S(5,0,15,0)}; var b=a.Reverse().ToArray(); var x=N(a).Segments[0]; var y=N(b).Segments[0]; Near(x.Start.X,y.Start.X); Near(x.End.X,y.End.X); Near(x.Start.Y,y.Start.Y); }
        private static void SelectionExtensionInvariant()
        {
            var settings = new NormalizationSettings { LinearTolerance=.01, AngularToleranceDegrees=.1 };
            var target = new[] {
                S(24910.05167284,50840,24910.05168122,50855,"a"),
                S(24910.05168872,50855,24910.05245762,52234.99634271,"b"),
                S(24910.05245762,52234.99634271,24910.05322652,53614.99268542,"c"),
                S(24910.05322652,53614.99268542,24910.05399542,54989.98902813,"d")
            };
            var distractors = Enumerable.Range(0,3192).Select(i =>
            {
                var angle = (89.99996 + i * .0000002) * Math.PI / 180;
                var x = 100000 + i * 100;
                return S(x,0,x + 100*Math.Cos(angle),100*Math.Sin(angle),"x"+i);
            });
            var local = SelectionInvariantLineNormalizer.Normalize(target,settings);
            var expanded = SelectionInvariantLineNormalizer.Normalize(target.Concat(distractors),settings);
            Eq(1, local.Groups.Count);
            Eq(1, local.Segments.Count);
            var targetGroup = expanded.Groups.Single(g => target.All(t => g.SourceIds.Contains(t.SourceId)));
            Eq(4,targetGroup.SourceCount);
            Eq(1,targetGroup.ResultCount);
        }
        private static void IntegerAxisStrictPair()
        {
            var result = IntegerAxisPairAnalyzer.Analyze(
                new[] { S(0,0,1000,0,"A"), S(0,250,1000,250,"B") },
                new IntegerAxisPairSettings());
            Eq(1,result.StrictPairCount);
            Eq(0,result.CandidatePairCount);
            Near(250,result.Pairs[0].Distance,1e-12);
        }
        private static void IntegerAxisPracticalCandidate()
        {
            var result = IntegerAxisPairAnalyzer.Analyze(
                new[] { S(0,0,1000,0,"A"), S(0,249.9999925,1000,249.9999925,"B") },
                new IntegerAxisPairSettings());
            Eq(0,result.StrictPairCount);
            Eq(1,result.CandidatePairCount);
            Near(.0000075,result.Pairs[0].Deviation,1e-10);
        }
        private static void IntegerAxisRequiresOverlap()
        {
            var result = IntegerAxisPairAnalyzer.Analyze(
                new[] { S(0,0,100,0,"A"), S(200,250,300,250,"B") },
                new IntegerAxisPairSettings());
            Eq(0,result.Pairs.Count);
        }
        private static void IntegerAxisLimitsDistance()
        {
            var result = IntegerAxisPairAnalyzer.Analyze(
                new[] { S(0,0,100,0,"A"), S(0,3001,100,3001,"B") },
                new IntegerAxisPairSettings());
            Eq(0,result.Pairs.Count);
        }
        private static void IntegerAxisExcludesZeroWidth()
        {
            var result = IntegerAxisPairAnalyzer.Analyze(
                new[] { S(0,0,100,0,"A"), S(0,.00008453,100,.00008453,"B") },
                new IntegerAxisPairSettings());
            Eq(0,result.Pairs.Count);
        }
        private static void AxisPlanAnchorsStrictPair()
        {
            var plan = IntegerAxisCorrectionPlanner.Plan(
                new[] { S(0,0,1000,0,"A"), S(0,250,1000,250,"B") },
                new IntegerAxisPairSettings());
            Eq(2,plan.AnchorCount);
        }
        private static void AxisPlanProposesSupportedCorrection()
        {
            var plan = IntegerAxisCorrectionPlanner.Plan(
                new[] {
                    S(0,0,1000,0,"A"),
                    S(0,250,1000,250,"B"),
                    S(0,380.0005,1000,380.0005,"T")
                },
                new IntegerAxisPairSettings());
            AxisCorrectionProposal target = plan.Proposals.Single(item => item.SourceId == "T");
            if (target.Status != AxisCorrectionStatus.Consistent) throw new Exception("expected consistent proposal");
            Near(-.0005,target.ShiftY,1e-10);
            Eq(2,target.SupportingPairCount);
        }
        private static void AxisPlanLeavesUnsupportedLine()
        {
            var plan = IntegerAxisCorrectionPlanner.Plan(
                new[] {
                    S(0,0,1000,0,"A"),
                    S(0,250,1000,250,"B"),
                    S(0,501.25,1000,501.25,"T")
                },
                new IntegerAxisPairSettings());
            AxisCorrectionProposal target = plan.Proposals.Single(item => item.SourceId == "T");
            if (target.Status != AxisCorrectionStatus.Unsupported) throw new Exception("expected unsupported proposal");
        }
        private static void AxisPlanReportsConflictingAnchors()
        {
            var plan = IntegerAxisCorrectionPlanner.Plan(
                new[] {
                    S(0,0,1000,0,"A0"),
                    S(0,100,1000,100,"A1"),
                    S(0,250.0005,1000,250.0005,"B0"),
                    S(0,350.0005,1000,350.0005,"B1"),
                    S(0,500.0002,1000,500.0002,"T")
                },
                new IntegerAxisPairSettings());
            AxisCorrectionProposal target = plan.Proposals.Single(item => item.SourceId == "T");
            if (target.Status != AxisCorrectionStatus.Conflict) throw new Exception("expected conflict");
            if (target.ConflictingPairCount == 0) throw new Exception("expected conflicting support");
        }
        private static void AngularStepKeepsExactAngle()
        {
            double radians = 30 * Math.PI / 180;
            AngularStepDiagnostic line = AngularStepAnalyzer.Analyze(
                new[] { S(0,0,100*Math.Cos(radians),100*Math.Sin(radians),"A") },
                new AngularStepSettings()).Lines[0];
            Near(30,line.NearestAngleDegrees);
            Near(0,line.EndpointShift,1e-10);
        }
        private static void AngularStepReportsEndpointDisplacement()
        {
            double radians = 5.001 * Math.PI / 180;
            AngularStepDiagnostic line = AngularStepAnalyzer.Analyze(
                new[] { S(0,0,10000*Math.Cos(radians),10000*Math.Sin(radians),"A") },
                new AngularStepSettings()).Lines[0];
            Near(5,line.NearestAngleDegrees);
            Near(0.001,line.AngleDeviationDegrees,1e-10);
            Near(10000*Math.Sin(.001*Math.PI/360),line.EndpointShift,1e-10);
        }
        private static void AngularStepWrapsAround180()
        {
            double radians = 179.8 * Math.PI / 180;
            AngularStepDiagnostic line = AngularStepAnalyzer.Analyze(
                new[] { S(0,0,100*Math.Cos(radians),100*Math.Sin(radians),"A") },
                new AngularStepSettings()).Lines[0];
            Near(0,line.NearestAngleDegrees);
            Near(.2,line.AngleDeviationDegrees,1e-10);
        }
        private static void GroupAxesRetainSourceFragments()
        {
            GroupedAxisEvidenceAnalysis analysis = GroupedAxisEvidenceAnalyzer.Analyze(
                new[] { S(0,0,50,0,"A1"), S(50,0,100,0,"A2"), S(0,249.9999,100,249.9999,"B") },
                new GroupedAxisEvidenceSettings());
            Eq(2, analysis.Axes.Count);
            Eq(2, analysis.Axes.Single(axis => axis.SourceIds.Contains("A1")).SourceIds.Count);
            Eq(1, analysis.PairAnalysis.Pairs.Count);
        }
        private static void GroupAxisChoosesNearestIntegerEvidence()
        {
            GroupedAxisEvidenceAnalysis analysis = GroupedAxisEvidenceAnalyzer.Analyze(
                new[] {
                    S(0,0,1000,0,"A"),
                    S(0,250.2,1000,250.2,"B"),
                    S(100,500.01,200,500.01,"C")
                },
                new GroupedAxisEvidenceSettings());
            GroupedAxis a = analysis.Axes.Single(axis => axis.SourceIds.Contains("A"));
            GroupedAxisEvidence best = analysis.BestByAxis.Single(item => item.Axis.Id == a.Id);
            if (!best.Partner.SourceIds.Contains("C")) throw new Exception("shorter overlap with better integer evidence must win");
            Near(.01, best.Pair.Deviation, 1e-10);
        }
        private static void MutualGroupAxesSeedSystem()
        {
            GroupedAxisSystemAnalysis analysis = GroupedAxisSystemAnalyzer.Analyze(
                new[] { S(0,0,1000,0,"A"), S(0,100.001,1000,100.001,"B") },
                new GroupedAxisEvidenceSettings());
            Eq(1, analysis.Systems.Count);
            Eq(2, analysis.SeedAxisCount);
            Eq(0, analysis.UnresolvedAxisCount);
        }
        private static void OneWayEvidenceAttachesToSeed()
        {
            GroupedAxisSystemAnalysis analysis = GroupedAxisSystemAnalyzer.Analyze(
                new[] {
                    S(0,0,1000,0,"A"),
                    S(0,100.001,1000,100.001,"B"),
                    S(0,250.01,1000,250.01,"C")
                },
                new GroupedAxisEvidenceSettings());
            Eq(1, analysis.Systems.Count);
            Eq(2, analysis.SeedAxisCount);
            Eq(1, analysis.AttachedAxisCount);
            GroupedAxisSystemMember attached = analysis.Members.Single(member => member.Role == GroupedAxisSystemRole.Attached);
            Eq(1, attached.Depth);
        }
        private static void SeedCorrectionIsSymmetric()
        {
            GroupedAxisCorrectionPlan plan = GroupedAxisCorrectionPlanAnalyzer.Plan(
                new[] { S(0,0,1000,0,"A"), S(0,100.002,1000,100.002,"B") },
                new GroupedAxisEvidenceSettings());
            GroupedAxisCorrectionProposal a = plan.Proposals.Single(item => item.Member.Axis.SourceIds.Contains("A"));
            GroupedAxisCorrectionProposal b = plan.Proposals.Single(item => item.Member.Axis.SourceIds.Contains("B"));
            Near(.001, a.ShiftY, 1e-10);
            Near(-.001, b.ShiftY, 1e-10);
            Near(100, Math.Abs(b.Proposed.Start.Y - a.Proposed.Start.Y), 1e-10);
        }
        private static void AttachedCorrectionFollowsCorrectedParent()
        {
            GroupedAxisCorrectionPlan plan = GroupedAxisCorrectionPlanAnalyzer.Plan(
                new[] {
                    S(0,0,1000,0,"A"),
                    S(0,100.002,1000,100.002,"B"),
                    S(0,250.01,1000,250.01,"C")
                },
                new GroupedAxisEvidenceSettings());
            GroupedAxisCorrectionProposal b = plan.Proposals.Single(item => item.Member.Axis.SourceIds.Contains("B"));
            GroupedAxisCorrectionProposal c = plan.Proposals.Single(item => item.Member.Axis.SourceIds.Contains("C"));
            Near(150, Math.Abs(c.Proposed.Start.Y - b.Proposed.Start.Y), 1e-10);
            Near(-.009, c.ShiftY, 1e-10);
        }
        private static void AssertSpan(NormalizationResult r,double min,double max) { Eq(1,r.Segments.Count); var s=r.Segments[0]; Near(min,Math.Min(s.Start.X,s.End.X)); Near(max,Math.Max(s.Start.X,s.End.X)); }
        private static void Run(string name,Action action) { try { action(); passed++; } catch(Exception e) { throw new Exception(name+": "+e.Message,e); } }
        private static void Eq(int expected,int actual) { if(expected!=actual) throw new Exception("expected "+expected+", actual "+actual); }
        private static void Near(double expected,double actual,double tolerance=1e-8) { if(Math.Abs(expected-actual)>tolerance) throw new Exception("expected "+expected+", actual "+actual); }
    }
}
