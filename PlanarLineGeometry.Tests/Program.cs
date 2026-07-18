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
        private static void AssertSpan(NormalizationResult r,double min,double max) { Eq(1,r.Segments.Count); var s=r.Segments[0]; Near(min,Math.Min(s.Start.X,s.End.X)); Near(max,Math.Max(s.Start.X,s.End.X)); }
        private static void Run(string name,Action action) { try { action(); passed++; } catch(Exception e) { throw new Exception(name+": "+e.Message,e); } }
        private static void Eq(int expected,int actual) { if(expected!=actual) throw new Exception("expected "+expected+", actual "+actual); }
        private static void Near(double expected,double actual,double tolerance=1e-8) { if(Math.Abs(expected-actual)>tolerance) throw new Exception("expected "+expected+", actual "+actual); }
    }
}
