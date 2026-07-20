using System;
using System.Collections.Generic;
using System.Linq;

namespace PlanarLineGeometry
{
    public sealed class AngularStepSettings
    {
        public double StepDegrees { get; set; } = 1.0;
    }

    public sealed class AngularStepDiagnostic
    {
        internal AngularStepDiagnostic(
            Segment2 source,
            Segment2 proposed,
            double currentAngleDegrees,
            double nearestAngleDegrees,
            double angleDeviationDegrees,
            double endpointShift)
        {
            Source = source;
            Proposed = proposed;
            CurrentAngleDegrees = currentAngleDegrees;
            NearestAngleDegrees = nearestAngleDegrees;
            AngleDeviationDegrees = angleDeviationDegrees;
            EndpointShift = endpointShift;
        }

        public Segment2 Source { get; }
        public Segment2 Proposed { get; }
        public double CurrentAngleDegrees { get; }
        public double NearestAngleDegrees { get; }
        public double AngleDeviationDegrees { get; }
        public double EndpointShift { get; }
    }

    public sealed class AngularStepAnalysis
    {
        internal AngularStepAnalysis(List<AngularStepDiagnostic> lines, int invalidCount)
        {
            Lines = lines;
            InvalidCount = invalidCount;
        }

        public IReadOnlyList<AngularStepDiagnostic> Lines { get; }
        public int InvalidCount { get; }
    }

    public static class AngularStepAnalyzer
    {
        private const double Epsilon = 1e-12;

        public static AngularStepAnalysis Analyze(
            IEnumerable<Segment2> source,
            AngularStepSettings settings)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));
            if (settings == null) throw new ArgumentNullException(nameof(settings));
            if (settings.StepDegrees <= 0 || settings.StepDegrees > 90)
                throw new ArgumentOutOfRangeException(nameof(settings.StepDegrees));

            var result = new List<AngularStepDiagnostic>();
            int invalid = 0;
            foreach (Segment2 segment in source)
            {
                if (segment == null || segment.Length <= Epsilon)
                {
                    invalid++;
                    continue;
                }

                double rawAngle = Normalize360(Math.Atan2(
                    segment.End.Y - segment.Start.Y,
                    segment.End.X - segment.Start.X) * 180.0 / Math.PI);
                double currentAxisAngle = Normalize180(rawAngle);
                double nearest = Math.Round(
                    currentAxisAngle / settings.StepDegrees,
                    MidpointRounding.AwayFromZero) * settings.StepDegrees;
                nearest = Normalize180(nearest);
                double signedDeviation = SignedAxisDifference(nearest, currentAxisAngle);
                double targetRawAngle = rawAngle + signedDeviation;
                double radians = targetRawAngle * Math.PI / 180.0;
                double halfLength = segment.Length / 2.0;
                var midpoint = new Point2(
                    (segment.Start.X + segment.End.X) / 2.0,
                    (segment.Start.Y + segment.End.Y) / 2.0);
                var half = new Vector2(
                    halfLength * Math.Cos(radians),
                    halfLength * Math.Sin(radians));
                var proposed = new Segment2(
                    midpoint.Add(-1 * half),
                    midpoint.Add(half),
                    segment.SourceId);
                double endpointShift = segment.Length * Math.Sin(
                    Math.Abs(signedDeviation) * Math.PI / 360.0);

                result.Add(new AngularStepDiagnostic(
                    segment,
                    proposed,
                    currentAxisAngle,
                    nearest,
                    Math.Abs(signedDeviation),
                    endpointShift));
            }

            return new AngularStepAnalysis(
                result.OrderBy(item => item.Source.SourceId, StringComparer.Ordinal).ToList(),
                invalid);
        }

        private static double Normalize360(double angle)
        {
            angle %= 360.0;
            if (angle < 0) angle += 360.0;
            return angle;
        }

        private static double Normalize180(double angle)
        {
            angle %= 180.0;
            if (angle < 0) angle += 180.0;
            return angle;
        }

        private static double SignedAxisDifference(double target, double source)
        {
            double difference = target - source;
            while (difference >= 90.0) difference -= 180.0;
            while (difference < -90.0) difference += 180.0;
            return difference;
        }
    }
}
