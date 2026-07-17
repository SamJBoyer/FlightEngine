using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Units;

namespace FlightEngine.Aircraft;

/// <summary>
/// Tuned arcade fighter matching the behavioral targets in hDocs/test.md,
/// using the base-fighter airframe geometry for spatial aero integration.
/// </summary>
public static class DefaultAircraft
{
    public static FlightProperties CreateProperties()
    {
        AirframeGeometry geometry = AirframeGeometry.BaseFighter;
        const float mass = 3500f;
        float wingArea = geometry.WingAreaSquareMeters;
        // Stronger parasite drag so idle coasts bleed energy in tens of seconds, not minutes.
        const float cd0 = 0.11f;
        const float airDensity = 1.225f;

        // CLmax chosen so level stall speed ≈ 150 km/h:
        // Vs = sqrt(2mg / (ρ S CLmax))
        float stallSpeed = Speed.KmhToMetersPerSecond(150f);
        float clMax = 2f * mass * 9.81f /
                      (airDensity * wingArea * stallSpeed * stallSpeed);

        float climbSpeed = Speed.KmhToMetersPerSecond(280f);
        float qClimb = 0.5f * airDensity * climbSpeed * climbSpeed;
        // Include a representative induced-drag term for a 20° climb.
        float clClimb = (mass * 9.81f * MathF.Cos(20f * MathF.PI / 180f)) / (qClimb * wingArea);
        float cdClimb = cd0 + 0.08f * clClimb * clClimb;
        float dragClimb = qClimb * wingArea * cdClimb;
        float weight = mass * 9.81f;
        float climbAngle = 20f * MathF.PI / 180f;
        float maxThrust = dragClimb + weight * MathF.Sin(climbAngle) * 0.88f;

        return new FlightProperties
        {
            MassKg = mass,
            InertiaTensor = new Vector3(12000f, 18000f, 8000f),
            Engines =
            [
                new EngineThrust(maxThrust, Vector3.UnitZ, Vector3.Zero)
            ],
            Geometry = geometry,
            WingArea = wingArea,
            // Lower slope → higher critical AoA (~30°) so loops/pull-ups stay attached.
            LiftSlope = 2.8f,
            MaxLiftCoefficient = clMax,
            ParasiteDragCoefficient = cd0,
            InducedDragFactor = 0.08f,
            StallAoAWidth = 0.5f,
            StallLiftRetention = 0.15f,
            PostStallControlRetention = 0.2f,
            ReferenceSpeedKmh = 400f,
            MinimumControlSpeedKmh = 40f,
            MaxRollRate = MathF.PI / 3f,
            // Sized so a full-stick loop from 400 km/h lands near 15s after q-authority bleed.
            MaxPitchRate = MathF.PI * 2f / 11f,
            MaxYawRate = MathF.PI * 2f / 15f,
            AileronClGain = 0.9f,
            ElevatorClGain = 0.55f,
            RudderClGain = 0.85f,
            ElevatorAreaFraction = 0.16f,
            RudderAreaFraction = 0.1f,
            // CoG ahead of / below the aero center — mild static margin, natural stall tip.
            CenterOfGravityLocal = new Vector3(0f, -0.1f, 0.28f),
            AeroCenterLocal = new Vector3(0f, 0f, 0.2f),
            VelocityAlignRate = 3.2f,
            AeroTorqueCoupling = 1f,
            ControlRateAssist = 1f,
            ControlSurfaceMomentBlend = 0.08f,
            AirDensity = airDensity
        };
    }

    public static FlightState CreateLevelFlight(float speedKmh, float altitudeMeters = 2000f)
    {
        float speed = Speed.KmhToMetersPerSecond(speedKmh);
        return new FlightState(
            new Vector3(0f, altitudeMeters, 0f),
            Quaternion.Identity,
            new Vector3(0f, 0f, speed),
            Vector3.Zero);
    }
}
