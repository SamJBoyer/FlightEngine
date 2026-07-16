using System.Numerics;
using FlightEngine.Core;
using Raylib_cs;

namespace FlightEngine.Visualizer;

/// <summary>
/// Projects the screen mouse onto an aim plane fixed in front of the aircraft in world space.
/// </summary>
internal static class FlightCursor
{
    public const float DefaultAimDistanceMeters = 350f;

    public static Vector3 AimPlaneCenter(in FlightState state, float aimDistanceMeters = DefaultAimDistanceMeters) =>
        state.Position + state.NoseVector * aimDistanceMeters;

    public static Vector3 ProjectFromMouse(
        in FlightState state,
        Camera3D camera,
        float aimDistanceMeters = DefaultAimDistanceMeters)
    {
        Vector3 planePoint = AimPlaneCenter(state, aimDistanceMeters);
        Vector3 planeNormal = state.NoseVector;

        Ray ray = Raylib.GetScreenToWorldRay(Raylib.GetMousePosition(), camera);
        if (TryIntersectPlane(ray, planePoint, planeNormal, out Vector3 hit))
        {
            return hit;
        }

        return planePoint;
    }

    public static void DrawWorldMarkers(in FlightState state, Vector3 cursorWorld, float aimDistanceMeters = DefaultAimDistanceMeters)
    {
        Vector3 planeCenter = AimPlaneCenter(state, aimDistanceMeters);
        Raylib.DrawLine3D(state.Position, cursorWorld, new Color(255, 200, 60, 160));
        Raylib.DrawSphere(cursorWorld, 3.5f, new Color(255, 200, 60, 230));
        Raylib.DrawCircle3D(planeCenter, 40f, state.RightVector, 90f, new Color(255, 200, 60, 90));
    }

    public static void DrawScreenReticle()
    {
        Vector2 mouse = Raylib.GetMousePosition();
        int x = (int)mouse.X;
        int y = (int)mouse.Y;
        Color reticle = new(255, 220, 80, 255);
        Raylib.DrawLine(x - 14, y, x - 4, y, reticle);
        Raylib.DrawLine(x + 4, y, x + 14, y, reticle);
        Raylib.DrawLine(x, y - 14, x, y - 4, reticle);
        Raylib.DrawLine(x, y + 4, x, y + 14, reticle);
        Raylib.DrawCircleLines(x, y, 10, reticle);
    }

    private static bool TryIntersectPlane(Ray ray, Vector3 planePoint, Vector3 planeNormal, out Vector3 hit)
    {
        hit = planePoint;
        float denom = Vector3.Dot(planeNormal, ray.Direction);
        if (MathF.Abs(denom) < 1e-5f)
        {
            return false;
        }

        float t = Vector3.Dot(planePoint - ray.Position, planeNormal) / denom;
        if (t < 0.01f)
        {
            return false;
        }

        hit = ray.Position + ray.Direction * t;
        return true;
    }
}
