using Common;

namespace SpaceTravelVisualization;


internal struct TrajectoryLeg
{
    public Vector3d Force { get; }
    public double Duration { get; }

    public TrajectoryLeg(Vector3d thrust, double duration)
    {
        Force = thrust;
        Duration = duration;
    }
}

internal class TrajectoryParameters
{
    public Vector3d DeltaPosition { get; }
    public Vector3d DeltaVelocity { get; }
    public double MaxForce { get; }
    public double Mass { get; }

    public TrajectoryParameters(Vector3d p0, Vector3d u, Vector3d q0, Vector3d v, double maxForce, double mass)
    {
        DeltaPosition = q0 - p0;
        DeltaVelocity = v - u;
        MaxForce = maxForce;
        Mass = mass;
    }
}


internal static class TrajectoryGenerator
{
    private const double TimeTolerance = 1e-6;
    private const double GeometricTolerance = 1e-6;
    private const double AngleTolerance = 1e-5;
    private const double ResultForceTolerance = 1e-6;
    private const int MaxIterations = 128;

    // Implementation from https://mmoarch.blogspot.com/

    private static double SimpleRatio(TrajectoryParameters tp)
    {
        // compute the f_v ratio of the simplest trajectory where we accelerate in two steps:
        // 1) reduce the velocity difference with the destination to 0 (time needed: t_v)
        // 2) traverse the distance to arrive at the destination (time needed: t_p)
        // (This usually takes longer than the optimal trajectory, since the distance traversal
        // does not utilize the full travel time period. (The greater travel period that
        // the distance traversal can use, the less impulse (acceleration) it needs.))

        var inv_acc = tp.Mass / tp.MaxForce;
        var t_v = inv_acc * tp.DeltaVelocity.Norm;

        var newDeltaPos = tp.DeltaVelocity * (t_v / 2) + (tp.DeltaPosition);
        var distance = newDeltaPos.Norm;
        var t_p = 2.0 * Math.Sqrt(inv_acc * distance);

        double t_tot = t_v + t_p;
        double f_v_ratio = t_v / t_tot;
        return f_v_ratio;
    }
    
    public static TrajectoryLeg[]? ComputeRendezvousTrajectory(TrajectoryParameters tp)
    {
        var dv = tp.DeltaVelocity.Norm;
        var distance = tp.DeltaPosition.Norm;

        // check special case 1, dV = 0:
        if (dv < GeometricTolerance)
        {
            if (distance < GeometricTolerance)
            {
                return null;
            }

            var time = Math.Sqrt(4.0 * distance / (tp.MaxForce / tp.Mass));
            var thrust = tp.DeltaPosition * (tp.MaxForce / distance);

            return new[]
            {
                new TrajectoryLeg(+thrust, time / 2.0),
                new TrajectoryLeg(-thrust, time / 2.0)
            };
        }

        // pick f_v in (0, tp.maxForce) via f_v_ratio in (0, 1):
        var bestRatio = -1.0;
        var maxRatio = 1.0;
        var simpleRatio = SimpleRatio(tp);
        var ratio = simpleRatio;  // start value
        var minRatio = simpleRatio * 0.99; // (account for rounding error)
       
        var iterations = 0;
        var best1 = new TrajectoryLeg();
        var best2 = new TrajectoryLeg();

        do
        {
            iterations++;
            var f_v = ratio * tp.MaxForce;
            var t_tot = tp.Mass / f_v * dv;

            var F_v = (tp.DeltaVelocity) * (tp.Mass / t_tot);
            var D_ttot = (tp.DeltaVelocity) * (t_tot / 2) + (tp.DeltaPosition);

            double dist_ttot = D_ttot.Norm;

            // check special case 2, dP_ttot = 0:
            if (dist_ttot < GeometricTolerance)
            {
                return new[]
                {
                    new TrajectoryLeg(F_v, t_tot)
                };
            }

            var R_d = (D_ttot) * (1 / dist_ttot);

            var alpha = Math.PI - Vector3d.Angle(F_v, R_d);  // angle between F_v and F_p1
           
            double f_d;
            if (Math.PI - alpha < AngleTolerance)
            {
                // special case 3a, F_v and F_p1 are parallel in same direction
                f_d = tp.MaxForce - f_v;
            }
            else if (alpha < AngleTolerance)
            {
                // special case 3b, F_v and F_p1 are parallel in opposite directions
                f_d = tp.MaxForce + f_v;
            }
            else
            {
                var sinAlpha = Math.Sin(alpha);
                f_d = tp.MaxForce / sinAlpha * Math.Sin(Math.PI - alpha - Math.Asin(Math.Clamp(f_v / tp.MaxForce * sinAlpha, -1, +1)));
            }

            var t_1 = 2 * tp.Mass * dist_ttot / (t_tot * f_d);
            var t_2 = t_tot - t_1;
            if (t_2 < TimeTolerance)
            {
                // pick smaller f_v
                maxRatio = ratio;
                ratio += (minRatio - ratio) / 2;  // (divisor experimentally calibrated)
                continue;
            }

            var F_d = (R_d) * (f_d);
            var F_d2 = (F_d) * (-t_1 / t_2);

            F_d += F_v;
            F_d2 += F_v;

            var F_1 = F_d;
            var F_2 = F_d2;

            var f_2 = F_2.Norm;
            if (f_2 > tp.MaxForce)
            {
                // pick smaller f_v
                //LOG.debug(String.format("Iteration %2d:             f_v_ratio %f; f_2 diff %e", nofIters, f_v_ratio, (tp.maxForce-f_2)));
                maxRatio = ratio;
                ratio += (minRatio - ratio) / 1.25;  // (divisor experimentally calibrated)
            }
            else
            {
                // best so far
                bestRatio = ratio;
                best1 = new TrajectoryLeg(F_1, t_1);
                best2 = new TrajectoryLeg(F_2, t_2);

                if (f_2 < (tp.MaxForce * (1 - ResultForceTolerance)))
                {
                    // pick greater f_v
                    minRatio = ratio;
                    ratio += (maxRatio - ratio) / 4;  // (divisor experimentally calibrated)
                }
                else
                {
                    break;  // done!
                }
            }
        } while (iterations < MaxIterations);

        if (bestRatio >= 0)
        {
            return new[]
            {
                best1, 
                best2
            };
        }

        return new[]
        {
            new TrajectoryLeg((tp.DeltaPosition + tp.DeltaVelocity).Normalized() * tp.MaxForce, dv * tp.Mass / tp.MaxForce)
        };
    }
}