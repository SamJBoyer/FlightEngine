using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Units;

namespace FlightEngine.Aircraft;

/// <summary>
/// Tuned arcade fighter matching the behavioral targets in hDocs/test.md.
/// </summary>
public static class DefaultAircraft
{
    public static FlightProperties CreateProperties()
    {
        const float mass = 3500f;
        const float wingArea = 22f;
        const float cd0 = 0.028f;
        const float airDensity = 1.225f;

        // Size thrust so a 20° climb near 280 km/h is sustainable at full throttle,
        // while level flight settles lower without needing continuous dive.
        float climbSpeed = Speed.KmhToMetersPerSecond(280f);
        float qClimb = 0.5f * airDensity * climbSpeed * climbSpeed;
        float dragClimb = qClimb * wingArea * cd0;
        float weight = mass * 9.81f;
        float climbAngle = 20f * MathF.PI / 180f;
        float maxThrust = dragClimb + weight * MathF.Sin(climbAngle) * 1.05f;

        return new FlightProperties
        {
            MassKg = mass,
            InertiaTensor = new Vector3(12000f, 18000f, 8000f),
            Engines =
            [
                new EngineThrust(maxThrust, Vector3.UnitZ, Vector3.Zero)
            ],
            WingArea = wingArea,
            LiftSlope = 5.2f,
            MaxLiftCoefficient = 1.6f,
            ParasiteDragCoefficient = cd0,
            InducedDragFactor = 0.05f,
            MaxRollRate = MathF.PI / 3f,
            MaxPitchRate = MathF.PI * 2f / 15f,
            MaxYawRate = MathF.PI * 2f / 15f,
            LevelStallSpeedKmh = 150f,
            VerticalStallSpeedKmh = 40f,
            CompressionStartKmh = 550f,
            CompressionSpanKmh = 250f,
            AngularDamping = new Vector3(4f, 4f, 5f),
            VelocityAlignRate = 2.0f,
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
