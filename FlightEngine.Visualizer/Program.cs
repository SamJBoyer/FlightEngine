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

        FlightProperties props = DefaultAircraft.CreateProperties();
        FlightSimulator sim = new(props) { Throttle = 0.75f };
        VirtualPilotController vpc = new(props);
        FlightState state = DefaultAircraft.CreateLevelFlight(320f, altitudeMeters: 800f);

        bool vpcMode = false;
        float chaseDistance = 35f;
        float cameraYawOffset = 0f;
        float cameraPitchOffset = 0.25f;

        while (!Raylib.WindowShouldClose())
        {
            float dt = Math.Clamp(Raylib.GetFrameTime(), 1f / 240f, 1f / 20f);

            HandleMetaInput(ref vpcMode, ref chaseDistance, ref state, sim);

            Vector2 mouseDelta = Raylib.GetMouseDelta();
            cameraYawOffset -= mouseDelta.X * 0.0025f;
            cameraPitchOffset = Math.Clamp(cameraPitchOffset + mouseDelta.Y * 0.0025f, -0.6f, 0.8f);

            Fci fci = vpcMode
                ? vpc.ComputeFci(state, ComputeFlightCursor(state, cameraYawOffset, cameraPitchOffset))
                : ReadManualFci();

            AdjustThrottle(sim, dt);
            state = sim.Tick(state, fci, dt);

            Camera3D camera = BuildChaseCamera(state, chaseDistance, cameraYawOffset, cameraPitchOffset);

            Raylib.BeginDrawing();
            Raylib.ClearBackground(new Color(135, 185, 220, 255));

            Raylib.BeginMode3D(camera);
            WorldRenderer.Draw(state.Position.X, state.Position.Z);
            DummyPlaneRenderer.Draw(state);

            if (vpcMode)
            {
                Vector3 cursor = ComputeFlightCursor(state, cameraYawOffset, cameraPitchOffset);
                Raylib.DrawSphere(cursor, 4f, new Color(255, 200, 60, 220));
                Raylib.DrawLine3D(state.Position, cursor, new Color(255, 200, 60, 160));
            }

            Raylib.EndMode3D();

            HudOverlay.Draw(state, fci, sim, vpcMode, chaseDistance);
            Raylib.DrawFPS(Raylib.GetScreenWidth() - 100, 16);
            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }

    private static void HandleMetaInput(
        ref bool vpcMode,
        ref float chaseDistance,
        ref FlightState state,
        FlightSimulator sim)
    {
        if (Raylib.IsKeyPressed(KeyboardKey.V))
        {
            vpcMode = !vpcMode;
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

        if (Raylib.IsKeyPressed(KeyboardKey.R))
        {
            state = DefaultAircraft.CreateLevelFlight(320f, altitudeMeters: 800f);
            sim.Throttle = 0.75f;
        }
    }

    private static Fci ReadManualFci()
    {
        float elevator = 0f;
        float aileron = 0f;
        float rudder = 0f;

        if (Raylib.IsKeyDown(KeyboardKey.W) || Raylib.IsKeyDown(KeyboardKey.Up))
        {
            elevator += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.S) || Raylib.IsKeyDown(KeyboardKey.Down))
        {
            elevator -= 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.D) || Raylib.IsKeyDown(KeyboardKey.Right))
        {
            aileron += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.A) || Raylib.IsKeyDown(KeyboardKey.Left))
        {
            aileron -= 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.E))
        {
            rudder += 1f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.Q))
        {
            rudder -= 1f;
        }

        return new Fci(aileron, elevator, rudder);
    }

    private static void AdjustThrottle(FlightSimulator sim, float dt)
    {
        float delta = 0f;
        if (Raylib.IsKeyDown(KeyboardKey.LeftShift) || Raylib.IsKeyDown(KeyboardKey.RightShift))
        {
            delta += 0.55f * dt;
        }

        if (Raylib.IsKeyDown(KeyboardKey.LeftControl) || Raylib.IsKeyDown(KeyboardKey.RightControl))
        {
            delta -= 0.55f * dt;
        }

        sim.Throttle = Math.Clamp(sim.Throttle + delta, 0f, 1f);
    }

    private static Vector3 ComputeFlightCursor(in FlightState state, float yawOffset, float pitchOffset)
    {
        Quaternion look =
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawOffset) *
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchOffset);

        Vector3 ahead = Vector3.Transform(Vector3.UnitZ, look * state.Rotation);
        return state.Position + ahead * 400f + Vector3.UnitY * 20f;
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
