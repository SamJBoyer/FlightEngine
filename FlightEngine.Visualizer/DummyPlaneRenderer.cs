using System.Numerics;
using FlightEngine.Aircraft;
using FlightEngine.Core;
using Raylib_cs;

namespace FlightEngine.Visualizer;

/// <summary>
/// Simple rigid dummy aircraft built from oriented boxes.
/// Body frame: +Z nose, +Y up, +X right.
/// </summary>
internal static class DummyPlaneRenderer
{
    public static void Draw(in FlightState state, PlaneVisualStyle style, ReadOnlySpan<bool> engineOnline = default)
    {
        Quaternion rot = state.Rotation;
        Vector3 pos = state.Position;

        switch (style)
        {
            case PlaneVisualStyle.FastRoll:
                DrawFastRoll(pos, rot);
                break;
            case PlaneVisualStyle.SlowRoll:
                DrawSlowRoll(pos, rot);
                break;
            case PlaneVisualStyle.TightTurn:
                DrawTightTurn(pos, rot);
                break;
            case PlaneVisualStyle.WideTurn:
                DrawWideTurn(pos, rot);
                break;
            case PlaneVisualStyle.TwinEngine:
                DrawTwinEngine(pos, rot, engineOnline);
                break;
            case PlaneVisualStyle.FastCruise:
                DrawFastCruise(pos, rot);
                break;
            case PlaneVisualStyle.SlowCruise:
                DrawSlowCruise(pos, rot);
                break;
            case PlaneVisualStyle.LateCompression:
                DrawLateCompression(pos, rot);
                break;
            case PlaneVisualStyle.EarlyCompression:
                DrawEarlyCompression(pos, rot);
                break;
            default:
                DrawBaseline(pos, rot);
                break;
        }

        Vector3 noseTip = pos + state.NoseVector * 10f;
        Raylib.DrawLine3D(pos, noseTip, new Color(80, 220, 120, 255));

        if (state.SpeedMetersPerSecond > 1f)
        {
            Vector3 velTip = pos + Vector3.Normalize(state.LinearVelocity) * 10f;
            Raylib.DrawLine3D(pos, velTip, new Color(80, 180, 255, 255));
        }
    }

    private static void DrawBaseline(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.2f), new Vector3(1.2f, 1.0f, 8.0f), new Color(210, 70, 55, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.1f, 0f), new Vector3(12.0f, 0.25f, 1.8f), new Color(230, 210, 170, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.9f, -3.2f), new Vector3(0.25f, 2.2f, 1.4f), new Color(230, 210, 170, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.15f, -3.4f), new Vector3(3.6f, 0.2f, 1.2f), new Color(200, 180, 140, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.35f, 3.4f), new Vector3(1.0f, 0.7f, 1.4f), new Color(40, 48, 58, 255));
    }

    /// <summary>Short stubby wings, high-vis yellow — snappy ailerons.</summary>
    private static void DrawFastRoll(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.1f), new Vector3(1.1f, 0.95f, 7.2f), new Color(35, 38, 42, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.08f, 0.2f), new Vector3(7.5f, 0.22f, 2.4f), new Color(245, 200, 40, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.85f, -2.8f), new Vector3(0.22f, 1.9f, 1.2f), new Color(245, 200, 40, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.12f, -3.0f), new Vector3(2.8f, 0.18f, 1.0f), new Color(220, 170, 30, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.3f, 3.1f), new Vector3(0.9f, 0.6f, 1.2f), new Color(20, 22, 26, 255));
        // Tip stripes
        DrawOrientedBox(pos, rot, new Vector3(-3.5f, 0.12f, 0.2f), new Vector3(0.5f, 0.28f, 1.6f), new Color(20, 22, 26, 255));
        DrawOrientedBox(pos, rot, new Vector3(3.5f, 0.12f, 0.2f), new Vector3(0.5f, 0.28f, 1.6f), new Color(20, 22, 26, 255));
    }

    /// <summary>Very long wings, pale blue — slow to bank.</summary>
    private static void DrawSlowRoll(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.3f), new Vector3(1.3f, 1.05f, 8.5f), new Color(90, 130, 160, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.1f, -0.2f), new Vector3(18.0f, 0.2f, 1.5f), new Color(180, 210, 230, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 1.0f, -3.4f), new Vector3(0.22f, 2.4f, 1.3f), new Color(160, 190, 210, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.15f, -3.6f), new Vector3(4.2f, 0.18f, 1.1f), new Color(150, 180, 200, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.35f, 3.6f), new Vector3(1.1f, 0.7f, 1.5f), new Color(50, 70, 90, 255));
    }

    /// <summary>Swept delta, green — tight turner.</summary>
    private static void DrawTightTurn(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.4f), new Vector3(1.0f, 0.9f, 7.5f), new Color(40, 110, 70, 255));
        // Swept main plane (two half-wings angled forward visually via offset)
        DrawOrientedBox(pos, rot, new Vector3(-3.2f, 0.08f, -0.4f), new Vector3(6.4f, 0.2f, 2.8f), new Color(70, 180, 110, 255));
        DrawOrientedBox(pos, rot, new Vector3(3.2f, 0.08f, -0.4f), new Vector3(6.4f, 0.2f, 2.8f), new Color(70, 180, 110, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.75f, -2.6f), new Vector3(0.2f, 1.7f, 1.8f), new Color(55, 150, 95, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.28f, 3.2f), new Vector3(0.85f, 0.55f, 1.3f), new Color(25, 50, 35, 255));
        // Canards
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.15f, 2.6f), new Vector3(3.2f, 0.14f, 0.7f), new Color(90, 200, 130, 255));
    }

    /// <summary>Bulky bomber silhouette, dark gray — poor turner.</summary>
    private static void DrawWideTurn(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0f), new Vector3(1.8f, 1.4f, 10.0f), new Color(70, 72, 78, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.05f, -0.5f), new Vector3(14.0f, 0.3f, 2.2f), new Color(95, 98, 105, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 1.1f, -4.0f), new Vector3(0.3f, 2.6f, 1.6f), new Color(85, 88, 94, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.2f, -4.2f), new Vector3(5.0f, 0.25f, 1.4f), new Color(80, 82, 88, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.4f, 4.0f), new Vector3(1.3f, 0.9f, 1.8f), new Color(30, 32, 36, 255));
        // Belly bulge
        DrawOrientedBox(pos, rot, new Vector3(0f, -0.55f, 0.2f), new Vector3(1.4f, 0.7f, 5.5f), new Color(55, 57, 62, 255));
    }

    /// <summary>Long needle fuselage, silver — built to go fast.</summary>
    private static void DrawFastCruise(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.6f), new Vector3(0.95f, 0.85f, 10.5f), new Color(190, 195, 205, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.06f, -0.8f), new Vector3(9.5f, 0.18f, 1.4f), new Color(150, 160, 175, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.7f, -4.2f), new Vector3(0.18f, 1.6f, 1.8f), new Color(140, 150, 165, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.12f, -4.4f), new Vector3(2.6f, 0.15f, 1.0f), new Color(130, 140, 155, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.28f, 4.6f), new Vector3(0.75f, 0.5f, 1.6f), new Color(30, 34, 40, 255));
        // Afterburner glow
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, -5.0f), new Vector3(0.55f, 0.55f, 0.4f), new Color(255, 140, 50, 255));
    }

    /// <summary>Fat short fuselage, olive — lives in the slow lane.</summary>
    private static void DrawSlowCruise(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0f), new Vector3(1.6f, 1.35f, 7.0f), new Color(95, 110, 70, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.12f, 0.2f), new Vector3(15.0f, 0.28f, 2.4f), new Color(120, 140, 85, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 1.15f, -2.6f), new Vector3(0.28f, 2.5f, 1.3f), new Color(110, 130, 75, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.18f, -2.8f), new Vector3(4.5f, 0.22f, 1.3f), new Color(100, 120, 70, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.4f, 2.8f), new Vector3(1.2f, 0.85f, 1.5f), new Color(45, 55, 35, 255));
        // Fixed gear stubs
        DrawOrientedBox(pos, rot, new Vector3(-1.2f, -0.9f, 0.5f), new Vector3(0.2f, 0.9f, 0.2f), new Color(40, 42, 38, 255));
        DrawOrientedBox(pos, rot, new Vector3(1.2f, -0.9f, 0.5f), new Vector3(0.2f, 0.9f, 0.2f), new Color(40, 42, 38, 255));
    }

    /// <summary>Arrowhead, white — stays crisp deep into high speed.</summary>
    private static void DrawLateCompression(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.5f), new Vector3(1.05f, 0.9f, 9.0f), new Color(235, 238, 242, 255));
        DrawOrientedBox(pos, rot, new Vector3(-2.8f, 0.08f, -0.6f), new Vector3(5.6f, 0.18f, 2.6f), new Color(70, 150, 190, 255));
        DrawOrientedBox(pos, rot, new Vector3(2.8f, 0.08f, -0.6f), new Vector3(5.6f, 0.18f, 2.6f), new Color(70, 150, 190, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.85f, -3.6f), new Vector3(0.2f, 2.0f, 1.5f), new Color(55, 130, 170, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.14f, -3.8f), new Vector3(3.0f, 0.16f, 1.1f), new Color(50, 120, 160, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.3f, 3.8f), new Vector3(0.85f, 0.55f, 1.4f), new Color(25, 45, 60, 255));
        // Tip markers
        DrawOrientedBox(pos, rot, new Vector3(-5.2f, 0.1f, -1.4f), new Vector3(0.35f, 0.22f, 0.9f), new Color(220, 80, 50, 255));
        DrawOrientedBox(pos, rot, new Vector3(5.2f, 0.1f, -1.4f), new Vector3(0.35f, 0.22f, 0.9f), new Color(220, 80, 50, 255));
    }

    /// <summary>Blunt trainer, copper — controls stiffen early.</summary>
    private static void DrawEarlyCompression(Vector3 pos, Quaternion rot)
    {
        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.1f), new Vector3(1.4f, 1.15f, 7.5f), new Color(170, 105, 65, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.1f, 0f), new Vector3(11.5f, 0.26f, 2.0f), new Color(195, 130, 80, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 1.0f, -3.0f), new Vector3(0.26f, 2.2f, 1.3f), new Color(180, 115, 70, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.16f, -3.2f), new Vector3(3.8f, 0.2f, 1.2f), new Color(160, 100, 60, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.38f, 3.0f), new Vector3(1.15f, 0.75f, 1.5f), new Color(55, 40, 30, 255));
        // Soft-stick cue: oversized elevators
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.2f, -3.5f), new Vector3(4.6f, 0.14f, 0.55f), new Color(90, 55, 35, 255));
    }

    /// <summary>Twin nacelles on the wings; dim a nacelle when that engine is offline.</summary>
    private static void DrawTwinEngine(Vector3 pos, Quaternion rot, ReadOnlySpan<bool> engineOnline)
    {
        bool leftOn = engineOnline.Length < 1 || engineOnline[0];
        bool rightOn = engineOnline.Length < 2 || engineOnline[1];

        DrawOrientedBox(pos, rot, new Vector3(0f, 0f, 0.2f), new Vector3(1.25f, 1.05f, 8.2f), new Color(160, 55, 45, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.1f, 0f), new Vector3(13.0f, 0.24f, 1.9f), new Color(220, 140, 90, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.95f, -3.3f), new Vector3(0.24f, 2.3f, 1.4f), new Color(200, 120, 70, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.15f, -3.5f), new Vector3(3.8f, 0.2f, 1.2f), new Color(190, 110, 65, 255));
        DrawOrientedBox(pos, rot, new Vector3(0f, 0.35f, 3.5f), new Vector3(1.05f, 0.7f, 1.4f), new Color(35, 30, 28, 255));

        Color leftNacelle = leftOn ? new Color(40, 42, 48, 255) : new Color(90, 40, 40, 255);
        Color rightNacelle = rightOn ? new Color(40, 42, 48, 255) : new Color(90, 40, 40, 255);
        Color leftGlow = leftOn ? new Color(255, 180, 60, 255) : new Color(40, 20, 20, 255);
        Color rightGlow = rightOn ? new Color(255, 180, 60, 255) : new Color(40, 20, 20, 255);

        DrawOrientedBox(pos, rot, new Vector3(-3.2f, -0.15f, 0.5f), new Vector3(1.1f, 1.0f, 3.2f), leftNacelle);
        DrawOrientedBox(pos, rot, new Vector3(3.2f, -0.15f, 0.5f), new Vector3(1.1f, 1.0f, 3.2f), rightNacelle);
        DrawOrientedBox(pos, rot, new Vector3(-3.2f, -0.15f, 2.0f), new Vector3(0.7f, 0.7f, 0.35f), leftGlow);
        DrawOrientedBox(pos, rot, new Vector3(3.2f, -0.15f, 2.0f), new Vector3(0.7f, 0.7f, 0.35f), rightGlow);
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
