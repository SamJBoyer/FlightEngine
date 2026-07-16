using System.Numerics;
using FlightEngine.Aircraft;
using FlightEngine.Control;
using FlightEngine.Core;
using FlightEngine.Physics;
using Raylib_cs;

namespace FlightEngine.Visualizer;

internal static class Program
{
    private const int Width = 1280;
    private const int Height = 720;

    [STAThread]
    public static void Main()
    {
        Raylib.SetConfigFlags(ConfigFlags.Msaa4xHint | ConfigFlags.ResizableWindow);
        Raylib.InitWindow(Width, Height, "FlightEngine Visualizer");
        Raylib.SetTargetFPS(60);
        Raylib.DisableCursor();

        int aircraftIndex = 0;
        AircraftDefinition aircraft = AircraftRoster.ByIndex(aircraftIndex);
        FlightSimulator sim = new(aircraft.Properties) { Throttle = 0.75f };
        VirtualPilotController vpc = new(aircraft.Properties);
        ManualPathHoldController pathHold = new();
        FlightState state = DefaultAircraft.CreateLevelFlight(320f, altitudeMeters: 800f);
        pathHold.Reset(state);
        using CloudField clouds = new();

        bool vpcMode = false;
        float chaseDistance = 35f;
        float cameraYawOffset = 0f;
        float cameraPitchOffset = 0.25f;
        float elapsed = 0f;
        float cruiseThrottle = 0.75f;
        ForceVector? pendingDebugForce = null;

        while (!Raylib.WindowShouldClose())
        {
            float dt = Math.Clamp(Raylib.GetFrameTime(), 1f / 240f, 1f / 20f);
            elapsed += dt;

            HandleMetaInput(
                ref vpcMode,
                ref chaseDistance,
                ref state,
                ref aircraftIndex,
                ref aircraft,
                ref sim,
                ref vpc,
                ref pathHold,
                ref cruiseThrottle,
                ref pendingDebugForce);

            if (!vpcMode)
            {
                Vector2 mouseDelta = Raylib.GetMouseDelta();
                cameraYawOffset -= mouseDelta.X * 0.0025f;
                cameraPitchOffset = Math.Clamp(cameraPitchOffset + mouseDelta.Y * 0.0025f, -0.6f, 0.8f);
            }
            else
            {
                // Locked chase cam while aiming with the free mouse cursor.
                cameraYawOffset = 0f;
                cameraPitchOffset = 0.18f;
            }

            Camera3D camera = BuildChaseCamera(state, chaseDistance, cameraYawOffset, cameraPitchOffset);

            Vector3 flightCursor = default;
            Fci fci;
            if (vpcMode)
            {
                flightCursor = FlightCursor.ProjectFromMouse(state, camera);
                fci = vpc.ComputeFci(state, flightCursor);
                // Manual roll still available in VPC mode.
                float manualAileron = ReadManualAileron();
                if (MathF.Abs(manualAileron) > 0.01f)
                {
                    fci = new Fci(manualAileron, fci.Elevator, fci.Rudder);
                }
            }
            else
            {
                fci = ReadManualFci(pathHold, state, dt);
            }

            AdjustThrottle(sim, ref cruiseThrottle, dt);
            if (pendingDebugForce is ForceVector debugForce)
            {
                state = sim.Tick(state, fci, dt, [debugForce]);
                pendingDebugForce = null;
            }
            else
            {
                state = sim.Tick(state, fci, dt);
            }

            // Camera tracks the post-tick pose for rendering.
            camera = BuildChaseCamera(state, chaseDistance, cameraYawOffset, cameraPitchOffset);
            if (vpcMode)
            {
                flightCursor = FlightCursor.ProjectFromMouse(state, camera);
            }

            Raylib.BeginDrawing();
            Raylib.ClearBackground(new Color(45, 95, 170, 255));

            Raylib.BeginMode3D(camera);
            WorldRenderer.Draw(camera, state.Position.X, state.Position.Z);
            clouds.Draw(camera, elapsed);
            DummyPlaneRenderer.Draw(state, aircraft.Visual, sim.EngineOnline);

            if (vpcMode)
            {
                FlightCursor.DrawWorldMarkers(state, flightCursor);
            }

            Raylib.EndMode3D();

            if (vpcMode)
            {
                FlightCursor.DrawScreenReticle();
            }

            HudOverlay.Draw(
                state,
                fci,
                sim,
                aircraft,
                aircraftIndex,
                AircraftRoster.All.Count,
                vpcMode,
                chaseDistance);
            Raylib.DrawFPS(Raylib.GetScreenWidth() - 100, 16);
            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }

    private static void HandleMetaInput(
        ref bool vpcMode,
        ref float chaseDistance,
        ref FlightState state,
        ref int aircraftIndex,
        ref AircraftDefinition aircraft,
        ref FlightSimulator sim,
        ref VirtualPilotController vpc,
        ref ManualPathHoldController pathHold,
        ref float cruiseThrottle,
        ref ForceVector? pendingDebugForce)
    {
        if (Raylib.IsKeyPressed(KeyboardKey.V))
        {
            vpcMode = !vpcMode;
            if (vpcMode)
            {
                Raylib.EnableCursor();
                Raylib.SetMousePosition(Raylib.GetScreenWidth() / 2, Raylib.GetScreenHeight() / 2);
            }
            else
            {
                Raylib.DisableCursor();
                pathHold.Reset(state);
            }
        }

        if (Raylib.IsKeyPressed(KeyboardKey.C))
        {
            chaseDistance = chaseDistance switch
            {
                <= 25f => 35f,
                <= 40f => 55f,
                _ => 22f
            };
        }

        if (Raylib.IsKeyPressed(KeyboardKey.P))
        {
            aircraftIndex = (aircraftIndex + 1) % AircraftRoster.All.Count;
            LoadAircraft(aircraftIndex, ref aircraft, ref sim, ref vpc, ref pathHold, ref state);
        }

        if (Raylib.IsKeyPressed(KeyboardKey.Y))
        {
            int killed = sim.KillRandomEngine();
            if (killed >= 0)
            {
                // Refresh VPC trim so it compensates for the new imbalance.
                FlightProperties masked = AircraftRoster.WithEngineMask(aircraft.Properties, sim.EngineOnline);
                vpc = new VirtualPilotController(masked, sim.Throttle);
            }
        }

        if (Raylib.IsKeyPressed(KeyboardKey.R))
        {
            state = DefaultAircraft.CreateLevelFlight(320f, altitudeMeters: 800f);
            cruiseThrottle = 0.75f;
            sim.Throttle = cruiseThrottle;
            sim.RestoreEngines();
            vpc = new VirtualPilotController(aircraft.Properties);
            pathHold.Reset(state);
        }

        if (Raylib.IsKeyPressed(KeyboardKey.Space))
        {
            // ~5–8× weight: clear jolt.
            pendingDebugForce = CreateRandomAirframeForce(sim.Properties.MassKg, 5f, 8f);
        }

        if (Raylib.IsKeyPressed(KeyboardKey.T))
        {
            // ~40–60× weight: huge kick.
            pendingDebugForce = CreateRandomAirframeForce(sim.Properties.MassKg, 40f, 60f);
        }
    }

    private static void LoadAircraft(
        int index,
        ref AircraftDefinition aircraft,
        ref FlightSimulator sim,
        ref VirtualPilotController vpc,
        ref ManualPathHoldController pathHold,
        ref FlightState state)
    {
        aircraft = AircraftRoster.ByIndex(index);
        float throttle = Math.Clamp(sim.Throttle > 1f ? 1f : sim.Throttle, 0f, 1f);
        sim = new FlightSimulator(aircraft.Properties) { Throttle = throttle };
        vpc = new VirtualPilotController(aircraft.Properties);
        state = DefaultAircraft.CreateLevelFlight(320f, altitudeMeters: 800f);
        pathHold.Reset(state);
    }

    /// <summary>
    /// Debug jolt: world-space force at a random body point on the dummy airframe.
    /// </summary>
    private static ForceVector CreateRandomAirframeForce(float massKg, float minWeightMult, float maxWeightMult)
    {
        // Rough bounds matching DummyPlaneRenderer (wingspan ~12 m, fuselage ~8 m).
        Vector3 localPoint = new(
            Random.Shared.NextSingle() * 12f - 6f,
            Random.Shared.NextSingle() * 2f - 0.5f,
            Random.Shared.NextSingle() * 8f - 4f);

        Vector3 direction = RandomUnitVector();
        float weightMult = minWeightMult + Random.Shared.NextSingle() * (maxWeightMult - minWeightMult);
        float magnitude = massKg * 9.81f * weightMult;
        return ForceVector.World(direction * magnitude, localPoint);
    }

    private static Vector3 RandomUnitVector()
    {
        // Uniform direction on the sphere via Gaussian components.
        Vector3 v = new(
            NextGaussian(),
            NextGaussian(),
            NextGaussian());
        float lenSq = v.LengthSquared();
        if (lenSq < 1e-8f)
        {
            return Vector3.UnitY;
        }

        return v / MathF.Sqrt(lenSq);
    }

    private static float NextGaussian()
    {
        float u1 = MathF.Max(Random.Shared.NextSingle(), 1e-6f);
        float u2 = Random.Shared.NextSingle();
        return MathF.Sqrt(-2f * MathF.Log(u1)) * MathF.Cos(2f * MathF.PI * u2);
    }

    private static float ReadManualAileron()
    {
        float aileron = 0f;
        if (Raylib.IsKeyDown(KeyboardKey.D) || Raylib.IsKeyDown(KeyboardKey.Right))
        {
            aileron += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.A) || Raylib.IsKeyDown(KeyboardKey.Left))
        {
            aileron -= 1f;
        }

        return aileron;
    }

    private static Fci ReadManualFci(ManualPathHoldController pathHold, in FlightState state, float dt)
    {
        float elevatorStick = 0f;
        float rudder = 0f;

        if (Raylib.IsKeyDown(KeyboardKey.W) || Raylib.IsKeyDown(KeyboardKey.Up))
        {
            elevatorStick += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.S) || Raylib.IsKeyDown(KeyboardKey.Down))
        {
            elevatorStick -= 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.E))
        {
            rudder += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.Q))
        {
            rudder -= 1f;
        }

        // Elevators always trim to the last nose-vector; stick retargets that attitude.
        return pathHold.ComputeFci(state, ReadManualAileron(), elevatorStick, rudder, dt);
    }

    /// <summary>
    /// Hold Shift for 5× afterburner (compression testing). Ctrl / = adjust cruise 0–100%.
    /// </summary>
    private static void AdjustThrottle(FlightSimulator sim, ref float cruiseThrottle, float dt)
    {
        const float afterburnerThrottle = 5f;

        float delta = 0f;
        if (Raylib.IsKeyDown(KeyboardKey.Equal) || Raylib.IsKeyDown(KeyboardKey.KpAdd))
        {
            delta += 0.55f * dt;
        }

        if (Raylib.IsKeyDown(KeyboardKey.LeftControl) || Raylib.IsKeyDown(KeyboardKey.RightControl)
            || Raylib.IsKeyDown(KeyboardKey.Minus) || Raylib.IsKeyDown(KeyboardKey.KpSubtract))
        {
            delta -= 0.55f * dt;
        }

        cruiseThrottle = Math.Clamp(cruiseThrottle + delta, 0f, 1f);

        bool boost = Raylib.IsKeyDown(KeyboardKey.LeftShift) || Raylib.IsKeyDown(KeyboardKey.RightShift);
        sim.Throttle = boost ? afterburnerThrottle : cruiseThrottle;
    }

    private static Camera3D BuildChaseCamera(
        in FlightState state,
        float distance,
        float yawOffset,
        float pitchOffset)
    {
        Quaternion orbit =
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawOffset) *
            Quaternion.CreateFromAxisAngle(state.RightVector, pitchOffset);

        Vector3 back = Vector3.Transform(-state.NoseVector, orbit);
        if (back.LengthSquared() < 1e-6f)
        {
            back = -state.NoseVector;
        }

        back = Vector3.Normalize(back);
        Vector3 camPos = state.Position + back * distance + state.UpVector * (distance * 0.22f);
        camPos.Y = Math.Max(camPos.Y, 2f);

        return new Camera3D
        {
            Position = camPos,
            Target = state.Position + state.NoseVector * 8f,
            Up = Vector3.UnitY,
            FovY = 55f,
            Projection = CameraProjection.Perspective
        };
    }
}
