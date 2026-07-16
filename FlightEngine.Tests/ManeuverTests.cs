using System.Numerics;
using FlightEngine.Aircraft;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Units;

namespace FlightEngine.Tests;

public class ManeuverTests
{
    private const float Dt = 1f / 60f;
    private const float TimeTolerance = 1.5f;

    [Fact]
    public void VerticalLoop_From400Kmh_CompletesInAbout15Seconds()
    {
        FlightSimulator sim = CreateSim();
        FlightState state = DefaultAircraft.CreateLevelFlight(400f);
        float pitchIntegrated = 0f;
        float time = 0f;
        const float target = MathF.PI * 2f;

        while (pitchIntegrated < target && time < 30f)
        {
            state = sim.Tick(state, new Fci(0f, 1f, 0f), Dt);
            pitchIntegrated += MathF.Abs(state.AngularVelocity.X) * Dt;
            time += Dt;
        }

        Assert.True(pitchIntegrated >= target, $"Loop incomplete after {time:F1}s");
        Assert.InRange(time, 15f - TimeTolerance, 15f + TimeTolerance);
    }

    [Fact]
    public void FlatCircle_From400Kmh_CompletesInAbout15Seconds_AndHoldsSpeed()
    {
        FlightSimulator sim = CreateSim();
        FlightState state = DefaultAircraft.CreateLevelFlight(400f);

        float heading = HeadingRadians(state);
        float headingTravel = 0f;
        float time = 0f;
        float minSpeed = float.MaxValue;
        float maxSpeed = 0f;
        float startAltitude = state.AltitudeMeters;
        const float target = MathF.PI * 2f;

        while (headingTravel < target && time < 30f)
        {
            // Hold ~400 km/h with throttle (climb-capable thrust exceeds level drag at 400).
            float speedKmhNow = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
            sim.Throttle = Math.Clamp(sim.Throttle + (400f - speedKmhNow) * 0.002f, 0.15f, 1f);

            // Wings-level yaw circle: full rudder at MaxYawRate (~15s), light elevator for altitude.
            float pitch = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));
            float elevator = Math.Clamp(-pitch * 3f, -1f, 1f);
            state = sim.Tick(state, new Fci(0f, elevator, 1f), Dt);

            float nextHeading = HeadingRadians(state);
            float delta = NormalizeAngle(nextHeading - heading);
            headingTravel += MathF.Abs(delta);
            heading = nextHeading;

            float speedKmh = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
            minSpeed = MathF.Min(minSpeed, speedKmh);
            maxSpeed = MathF.Max(maxSpeed, speedKmh);
            time += Dt;
        }

        Assert.True(headingTravel >= target, $"Circle incomplete after {time:F1}s");
        Assert.InRange(time, 15f - TimeTolerance, 15f + TimeTolerance);
        Assert.InRange(minSpeed, 400f * 0.85f, 400f * 1.15f);
        Assert.InRange(maxSpeed, 400f * 0.85f, 400f * 1.15f);
        Assert.InRange(state.AltitudeMeters, startAltitude - 150f, startAltitude + 150f);
    }

    [Fact]
    public void FullRoll_CompletesInAbout6Seconds()
    {
        FlightSimulator sim = CreateSim();
        FlightState state = DefaultAircraft.CreateLevelFlight(400f);
        float rollIntegrated = 0f;
        float time = 0f;
        const float target = MathF.PI * 2f;

        while (rollIntegrated < target && time < 20f)
        {
            state = sim.Tick(state, new Fci(1f, 0f, 0f), Dt);
            rollIntegrated += MathF.Abs(state.AngularVelocity.Z) * Dt;
            time += Dt;
        }

        Assert.True(rollIntegrated >= target, $"Roll incomplete after {time:F1}s");
        Assert.InRange(time, 6f - TimeTolerance, 6f + TimeTolerance);
    }

    [Fact]
    public void StallSpeed_LevelIs150Kmh_VerticalIs40Kmh()
    {
        FlightSimulator sim = CreateSim();

        FlightState level = DefaultAircraft.CreateLevelFlight(150f);
        Assert.InRange(sim.StallSpeedKmh(level), 148f, 152f);

        // Nose straight up — load factor → 0, diagnostic floor is minimum control speed.
        Quaternion vertical = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathF.PI / 2f);
        FlightState up = new(
            level.Position,
            vertical,
            Vector3.Transform(new Vector3(0f, 0f, Speed.KmhToMetersPerSecond(40f)), vertical),
            Vector3.Zero);

        Assert.InRange(sim.StallSpeedKmh(up), 38f, 42f);
    }

    [Fact]
    public void Climb_20Degrees_CanMaintain280Kmh()
    {
        FlightSimulator sim = CreateSim();
        float speed = Speed.KmhToMetersPerSecond(280f);
        float pitch = 20f * MathF.PI / 180f;
        Quaternion climbAttitude = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitch);

        FlightState state = new(
            new Vector3(0f, 2000f, 0f),
            climbAttitude,
            Vector3.Transform(new Vector3(0f, 0f, speed), climbAttitude),
            Vector3.Zero);

        sim.Throttle = 1f;

        // Hold attitude with light elevator; measure speed after settling.
        for (int i = 0; i < 600; i++)
        {
            float pitchError = pitch - MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));
            float elevator = Math.Clamp(pitchError * 2.5f, -1f, 1f);
            state = sim.Tick(state, new Fci(0f, elevator, 0f), Dt);
        }

        float speedKmh = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
        float climbAngleDeg = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f)) * 180f / MathF.PI;

        Assert.InRange(climbAngleDeg, 15f, 25f);
        Assert.InRange(speedKmh, 280f * 0.9f, 280f * 1.1f);
        Assert.True(state.LinearVelocity.Y > 0f, "Should still be climbing");
    }

    private static FlightSimulator CreateSim() => new(DefaultAircraft.CreateProperties());

    private static float HeadingRadians(in FlightState state)
    {
        Vector3 flat = new(state.NoseVector.X, 0f, state.NoseVector.Z);
        if (flat.LengthSquared() < 1e-8f)
        {
            flat = new(state.LinearVelocity.X, 0f, state.LinearVelocity.Z);
        }

        flat = Vector3.Normalize(flat);
        return MathF.Atan2(flat.X, flat.Z);
    }

    private static float NormalizeAngle(float radians)
    {
        while (radians > MathF.PI)
        {
            radians -= MathF.PI * 2f;
        }

        while (radians < -MathF.PI)
        {
            radians += MathF.PI * 2f;
        }

        return radians;
    }
}
