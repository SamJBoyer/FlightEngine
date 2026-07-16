using System.Numerics;
using FlightEngine.Core;
using Raylib_cs;

namespace FlightEngine.Visualizer;

/// <summary>
/// Simple rigid dummy aircraft built from oriented boxes.
/// Body frame: +Z nose, +Y up, +X right.
/// </summary>
internal static class DummyPlaneRenderer
{
    public static void Draw(in FlightState state)
    {
        Quaternion rot = state.Rotation;
        Vector3 pos = state.Position;

        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.2f), new Vector3(1.2f, 1.0f, 8.0f), new Color(210, 70, 55, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.1f, 0f), new Vector3(12.0f, 0.25f, 1.8f), new Color(230, 210, 170, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.9f, -3.2f), new Vector3(0.25f, 2.2f, 1.4f), new Color(230, 210, 170, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.15f, -3.4f), new Vector3(3.6f, 0.2f, 1.2f), new Color(200, 180, 140, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.35f, 3.4f), new Vector3(1.0f, 0.7f, 1.4f), new Color(40, 48, 58, 255));

        Vector3 noseTip = pos + state.NoseVector * 10f;
        Raylib.DrawLine3D(pos, noseTip, new Color(80, 220, 120, 255));

        if (state.SpeedMetersPerSecond > 1f)
        {
            Vector3 velTip = pos + Vector3.Normalize(state.LinearVelocity) * 10f;
            Raylib.DrawLine3D(pos, velTip, new Color(80, 180, 255, 255));
        }
    }

    private static void DrawOrientedBox(
        Vector3 worldPos,
        Quaternion worldRot,
        Vector3 localCenter,
        Vector3 size,
        Color color)
    {
        Vector3 center = worldPos + Vector3.Transform(localCenter, worldRot);
        Vector3 h = size * 0.5f;

        Vector3[] c =
        [
            center + Vector3.Transform(new Vector3(-h.X, -h.Y, -h.Z), worldRot),
            center + Vector3.Transform(new Vector3(h.X, -h.Y, -h.Z), worldRot),
            center + Vector3.Transform(new Vector3(h.X, h.Y, -h.Z), worldRot),
            center + Vector3.Transform(new Vector3(-h.X, h.Y, -h.Z), worldRot),
            center + Vector3.Transform(new Vector3(-h.X, -h.Y, h.Z), worldRot),
            center + Vector3.Transform(new Vector3(h.X, -h.Y, h.Z), worldRot),
            center + Vector3.Transform(new Vector3(h.X, h.Y, h.Z), worldRot),
            center + Vector3.Transform(new Vector3(-h.X, h.Y, h.Z), worldRot)
        ];

        Quad(c[0], c[1], c[2], c[3], color);
        Quad(c[5], c[4], c[7], c[6], color);
        Quad(c[4], c[0], c[3], c[7], color);
        Quad(c[1], c[5], c[6], c[2], color);
        Quad(c[3], c[2], c[6], c[7], color);
        Quad(c[4], c[5], c[1], c[0], color);
    }

    private static void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color color)
    {
        Raylib.DrawTriangle3D(a, b, c, color);
        Raylib.DrawTriangle3D(a, c, d, color);
    }
}
