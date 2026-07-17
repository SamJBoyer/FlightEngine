using System.Numerics;
using FlightEngine.Aircraft;
using FlightEngine.Control;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Units;

namespace FlightEngine.Tests;

public class PhysicsContractTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void Climbing_TradesSpeedForAltitude_WithoutThrottle()
    {
        FlightProperties props = DefaultAircraft.CreateProperties();
        // Zero thrust so energy exchange is visible.
        props = CloneWithEngines(props, [new EngineThrust(0f, Vector3.UnitZ, Vector3.Zero)]);
        FlightSimulator sim = new(props) { Throttle = 0f };

        FlightState state = DefaultAircraft.CreateLevelFlight(400f);
        float startEnergy = MechanicalEnergy(state, props.MassKg, props.Gravity);

        for (int i = 0; i < 180; i++)
        {
            state = sim.Tick(state, new Fci(0f, 1f, 0f), Dt);
        }

        float endEnergy = MechanicalEnergy(state, props.MassKg, props.Gravity);
        Assert.True(state.AltitudeMeters > 2000f, "Should gain altitude in a pull-up");
        Assert.True(
            Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond) < 400f,
            "Should lose speed while climbing without thrust");
        // Drag dissipates some energy; allow bounded loss, not gain from nowhere.
        Assert.True(endEnergy <= startEnergy + 1f);
    }

    [Fact]
    public void ForceVector_OffsetFromCom_ProducesRotation()
    {
        FlightSimulator sim = new(DefaultAircraft.CreateProperties()) { Throttle = 0f };
        FlightState state = DefaultAircraft.CreateLevelFlight(200f);

        // Offset local force produces torque on the rigid body.
        ForceVector impact = ForceVector.LocalThrust(new Vector3(0f, 50000f, 0f), new Vector3(5f, 0f, 0f));

        FlightState next = sim.Tick(state, Fci.Neutral, Dt, [impact]);
        Assert.True(MathF.Abs(next.AngularVelocity.Z) > 0.01f || MathF.Abs(next.AngularVelocity.X) > 0.01f
            || MathF.Abs(next.AngularVelocity.Y) > 0.01f);
    }

    [Fact]
    public void Gravity_PullsAltitudeDown_WhenCoasting()
    {
        FlightProperties props = DefaultAircraft.CreateProperties();
        props = CloneWithEngines(props, [new EngineThrust(0f, Vector3.UnitZ, Vector3.Zero)]);
        FlightSimulator sim = new(props) { Throttle = 0f };

        FlightState state = new(
            new Vector3(0f, 3000f, 0f),
            Quaternion.Identity,
            Vector3.Zero,
            Vector3.Zero);

        for (int i = 0; i < 120; i++)
        {
            state = sim.Tick(state, Fci.Neutral, Dt);
        }

        Assert.True(state.AltitudeMeters < 3000f);
        Assert.True(state.LinearVelocity.Y < 0f);
    }

    [Fact]
    public void Compression_ReducesControlEffectiveness_AtHighSpeed()
    {
        FlightSimulator sim = new(DefaultAircraft.CreateProperties());

        FlightState slow = DefaultAircraft.CreateLevelFlight(400f);
        FlightState fast = DefaultAircraft.CreateLevelFlight(800f);

        FlightState slowNext = sim.Tick(slow, new Fci(1f, 0f, 0f), Dt);
        FlightState fastNext = sim.Tick(fast, new Fci(1f, 0f, 0f), Dt);

        Assert.True(
            MathF.Abs(fastNext.AngularVelocity.Z) < MathF.Abs(slowNext.AngularVelocity.Z),
            "High-speed compression should stiffen ailerons");
    }

    [Fact]
    public void StalledPlane_IsLessResponsiveThanFlyingPlane()
    {
        FlightSimulator sim = new(DefaultAircraft.CreateProperties());

        FlightState flying = DefaultAircraft.CreateLevelFlight(300f);
        FlightState stalled = DefaultAircraft.CreateLevelFlight(100f);

        Assert.False(sim.IsStalled(flying));
        Assert.True(sim.IsStalled(stalled));

        FlightState flyNext = sim.Tick(flying, new Fci(1f, 0f, 0f), Dt);
        FlightState stallNext = sim.Tick(stalled, new Fci(1f, 0f, 0f), Dt);

        Assert.True(MathF.Abs(stallNext.AngularVelocity.Z) < MathF.Abs(flyNext.AngularVelocity.Z));
    }

    [Fact]
    public void Stall_DropsNoseAndRecoversAirspeed()
    {
        FlightSimulator sim = new(DefaultAircraft.CreateProperties()) { Throttle = 0.2f };

        // Deep stall: below vertical stall speed (40 km/h) with nose straight up.
        float speed = Speed.KmhToMetersPerSecond(30f);
        Quaternion noseUp = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathF.PI * 0.5f);
        FlightState state = new(
            new Vector3(0f, 2500f, 0f),
            noseUp,
            Vector3.Transform(new Vector3(0f, 0f, speed), noseUp),
            Vector3.Zero);

        Assert.True(sim.IsStalled(state));
        float startPitch = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));

        // Full back stick: with no stall assist, forward/low CoG should still tip the nose.
        float minPitch = startPitch;
        bool sawFall = false;
        for (int i = 0; i < 240; i++)
        {
            state = sim.Tick(state, new Fci(0f, 1f, 0f), Dt);
            minPitch = MathF.Min(minPitch, MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f)));
            sawFall |= state.LinearVelocity.Y < 0f;
        }

        Assert.True(minPitch < startPitch - 0.25f, $"Nose should drop from CoG (start {startPitch:F2}, min {minPitch:F2})");
        Assert.True(sawFall, "Should enter a falling trajectory");

        // Let it dive and regain flying speed.
        for (int i = 0; i < 540; i++)
        {
            state = sim.Tick(state, Fci.Neutral, Dt);
        }

        float endSpeedKmh = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
        Assert.True(endSpeedKmh > 150f, $"Should regain flying speed after dive, got {endSpeedKmh:F0} km/h");
        Assert.False(sim.IsStalled(state));
    }

    [Fact]
    public void ManualPathHold_NeutralStick_HoldsNoseBetterThanZeroElevator()
    {
        FlightProperties props = DefaultAircraft.CreateProperties();
        FlightSimulator heldSim = new(props) { Throttle = 0.75f };
        FlightSimulator rawSim = new(props) { Throttle = 0.75f };
        ManualPathHoldController pathHold = new();

        FlightState held = DefaultAircraft.CreateLevelFlight(320f);
        FlightState raw = held;
        pathHold.Reset(held);

        Vector3 startNose = held.NoseVector;
        for (int i = 0; i < 240; i++)
        {
            Fci holdFci = pathHold.ComputeFci(held, aileron: 0f, elevatorStick: 0f, rudder: 0f, Dt);
            held = heldSim.Tick(held, holdFci, Dt);
            raw = rawSim.Tick(raw, Fci.Neutral, Dt);
        }

        float heldAlign = Vector3.Dot(held.NoseVector, startNose);
        float rawAlign = Vector3.Dot(raw.NoseVector, startNose);
        float heldAltitudeLoss = MathF.Abs(held.AltitudeMeters - 2000f);
        float rawAltitudeLoss = MathF.Abs(raw.AltitudeMeters - 2000f);

        Assert.True(heldAlign > rawAlign, $"Path-hold should keep nose ({heldAlign:F3} vs raw {rawAlign:F3})");
        Assert.True(heldAltitudeLoss < rawAltitudeLoss, $"Path-hold should lose less altitude ({heldAltitudeLoss:F1}m vs {rawAltitudeLoss:F1}m)");
    }

    [Fact]
    public void Vpc_PointsTranslationTowardFlightCursor()
    {
        FlightProperties props = DefaultAircraft.CreateProperties();
        FlightSimulator sim = new(props);
        VirtualPilotController vpc = new(props);

        FlightState state = DefaultAircraft.CreateLevelFlight(350f);
        Vector3 cursor = state.Position + new Vector3(500f, 100f, 800f);

        float startAlign = Alignment(state, cursor);
        for (int i = 0; i < 300; i++)
        {
            Fci fci = vpc.ComputeFci(state, cursor);
            state = sim.Tick(state, fci, Dt);
        }

        float endAlign = Alignment(state, cursor);
        Assert.True(endAlign > startAlign, $"Expected improved alignment ({startAlign:F3} -> {endAlign:F3})");
    }

    [Fact]
    public void Vpc_CompensatesUnbalancedEngines()
    {
        FlightProperties balanced = DefaultAircraft.CreateProperties();
        FlightProperties unbalanced = CloneWithEngines(balanced,
        [
            new EngineThrust(balanced.Engines[0].MaxThrustNewtons * 0.55f, Vector3.UnitZ, new Vector3(-3f, 0f, 0f)),
            new EngineThrust(balanced.Engines[0].MaxThrustNewtons * 0.45f, Vector3.UnitZ, new Vector3(3f, 0f, 0f))
        ]);

        VirtualPilotController vpc = new(unbalanced);
        FlightState state = DefaultAircraft.CreateLevelFlight(300f);
        Fci fci = vpc.ComputeFci(state, state.Position + new Vector3(0f, 0f, 1000f));

        // Neutral cursor ahead should still produce trim opposing the torque bias.
        Assert.True(MathF.Abs(fci.Aileron) > 0.001f || MathF.Abs(fci.Rudder) > 0.001f);
    }

    private static float Alignment(in FlightState state, Vector3 cursor)
    {
        Vector3 desired = Vector3.Normalize(cursor - state.Position);
        Vector3 translation = state.SpeedMetersPerSecond > 1f
            ? Vector3.Normalize(state.LinearVelocity)
            : state.NoseVector;
        return Vector3.Dot(translation, desired);
    }

    private static float MechanicalEnergy(in FlightState state, float mass, float gravity) =>
        0.5f * mass * state.LinearVelocity.LengthSquared() + mass * gravity * state.AltitudeMeters;

    private static FlightProperties CloneWithEngines(FlightProperties source, EngineThrust[] engines) =>
        new()
        {
            MassKg = source.MassKg,
            InertiaTensor = source.InertiaTensor,
            Engines = engines,
            Geometry = source.Geometry,
            WingArea = source.WingArea,
            LiftSlope = source.LiftSlope,
            MaxLiftCoefficient = source.MaxLiftCoefficient,
            ParasiteDragCoefficient = source.ParasiteDragCoefficient,
            InducedDragFactor = source.InducedDragFactor,
            StallAoAWidth = source.StallAoAWidth,
            StallLiftRetention = source.StallLiftRetention,
            PostStallControlRetention = source.PostStallControlRetention,
            ReferenceSpeedKmh = source.ReferenceSpeedKmh,
            MinimumControlSpeedKmh = source.MinimumControlSpeedKmh,
            MaxRollRate = source.MaxRollRate,
            MaxPitchRate = source.MaxPitchRate,
            MaxYawRate = source.MaxYawRate,
            AileronClGain = source.AileronClGain,
            ElevatorClGain = source.ElevatorClGain,
            RudderClGain = source.RudderClGain,
            ElevatorAreaFraction = source.ElevatorAreaFraction,
            RudderAreaFraction = source.RudderAreaFraction,
            CenterOfGravityLocal = source.CenterOfGravityLocal,
            AeroCenterLocal = source.AeroCenterLocal,
            VelocityAlignRate = source.VelocityAlignRate,
            AeroTorqueCoupling = source.AeroTorqueCoupling,
            ControlRateAssist = source.ControlRateAssist,
            ControlSurfaceMomentBlend = source.ControlSurfaceMomentBlend,
            AirDensity = source.AirDensity,
            Gravity = source.Gravity
        };
}
