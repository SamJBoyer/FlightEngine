using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Visualizer;

internal static class WorldRenderer
{
    private static readonly Color[] FieldPalette =
    [
        new(74, 118, 64, 255),
        new(92, 138, 72, 255),
        new(110, 128, 68, 255),
        new(128, 118, 70, 255),
        new(86, 104, 58, 255),
        new(148, 132, 78, 255),
        new(70, 112, 88, 255),
        new(118, 96, 62, 255)
    ];

    public static void Draw(in Camera3D camera, float focusX, float focusZ)
    {
        DrawSky(camera);
        DrawGround(focusX, focusZ);
        DrawGrids(focusX, focusZ);
    }

    private static void DrawSky(in Camera3D camera)
    {
        Vector3 origin = new(camera.Position.X, 0f, camera.Position.Z);
        const float radius = 6000f;
        const int rings = 12;
        const int segments = 32;

        // Dome is viewed from the inside.
        Rlgl.DisableBackfaceCulling();

        for (int r = 0; r < rings; r++)
        {
            float t0 = r / (float)rings;
            float t1 = (r + 1) / (float)rings;
            float elev0 = t0 * MathF.PI * 0.5f;
            float elev1 = t1 * MathF.PI * 0.5f;

            Color c0 = SkyColor(t0);
            Color c1 = SkyColor(t1);

            for (int s = 0; s < segments; s++)
            {
                float a0 = s * MathF.Tau / segments;
                float a1 = (s + 1) * MathF.Tau / segments;

                Vector3 p00 = SkyPoint(origin, radius, elev0, a0);
                Vector3 p01 = SkyPoint(origin, radius, elev0, a1);
                Vector3 p10 = SkyPoint(origin, radius, elev1, a0);
                Vector3 p11 = SkyPoint(origin, radius, elev1, a1);

                Color mid = LerpColor(c0, c1, 0.5f);
                Raylib.DrawTriangle3D(p00, p11, p10, mid);
                Raylib.DrawTriangle3D(p00, p01, p11, mid);
            }
        }

        Rlgl.EnableBackfaceCulling();

        // Soft sun disc for a heading cue.
        Vector3 sunDir = Vector3.Normalize(new Vector3(0.55f, 0.65f, 0.35f));
        Vector3 sunPos = origin + sunDir * (radius * 0.92f);
        Raylib.DrawSphere(sunPos, 120f, new Color(255, 236, 170, 255));
        Raylib.DrawSphere(sunPos, 180f, new Color(255, 220, 140, 60));
    }

    private static void DrawGround(float focusX, float focusZ)
    {
        const float tile = 250f;
        const int half = 18;
        int ox = (int)MathF.Floor(focusX / tile);
        int oz = (int)MathF.Floor(focusZ / tile);

        for (int iz = -half; iz <= half; iz++)
        {
            for (int ix = -half; ix <= half; ix++)
            {
                int tx = ox + ix;
                int tz = oz + iz;
                float x = tx * tile + tile * 0.5f;
                float z = tz * tile + tile * 0.5f;

                Color color = FieldColor(tx, tz);
                Raylib.DrawPlane(new Vector3(x, 0f, z), new Vector2(tile * 0.98f, tile * 0.98f), color);

                // Occasional lighter "crop row" strip for extra motion parallax.
                if (((tx * 17 + tz * 31) & 7) == 0)
                {
                    Raylib.DrawCubeV(
                        new Vector3(x, 0.02f, z),
                        new Vector3(tile * 0.9f, 0.02f, tile * 0.08f),
                        Tint(color, 1.15f));
                }
            }
        }
    }

    private static void DrawGrids(float focusX, float focusZ)
    {
        DrawGrid(focusX, focusZ, spacing: 100f, half: 40, color: new Color(40, 70, 40, 120));
        DrawGrid(focusX, focusZ, spacing: 500f, half: 10, color: new Color(30, 50, 30, 200));

        // Cardinal axes through the focus for orientation.
        Color east = new(180, 70, 60, 180);
        Color north = new(60, 90, 180, 180);
        Raylib.DrawLine3D(new Vector3(focusX - 2000f, 0.2f, focusZ), new Vector3(focusX + 2000f, 0.2f, focusZ), east);
        Raylib.DrawLine3D(new Vector3(focusX, 0.2f, focusZ - 2000f), new Vector3(focusX, 0.2f, focusZ + 2000f), north);
    }

    private static void DrawGrid(float focusX, float focusZ, float spacing, int half, Color color)
    {
        int originX = (int)MathF.Floor(focusX / spacing);
        int originZ = (int)MathF.Floor(focusZ / spacing);

        for (int i = -half; i <= half; i++)
        {
            float x = (originX + i) * spacing;
            float z0 = (originZ - half) * spacing;
            float z1 = (originZ + half) * spacing;
            Raylib.DrawLine3D(new Vector3(x, 0.08f, z0), new Vector3(x, 0.08f, z1), color);

            float z = (originZ + i) * spacing;
            float x0 = (originX - half) * spacing;
            float x1 = (originX + half) * spacing;
            Raylib.DrawLine3D(new Vector3(x0, 0.08f, z), new Vector3(x1, 0.08f, z), color);
        }
    }

    private static Color FieldColor(int tx, int tz)
    {
        int h = Hash(tx, tz);
        Color baseColor = FieldPalette[h & 7];

        // Checker bias so neighboring tiles read clearly in motion.
        if (((tx + tz) & 1) == 0)
        {
            baseColor = Tint(baseColor, 0.88f);
        }

        return baseColor;
    }

    private static Color SkyColor(float t)
    {
        // t: 0 horizon → 1 zenith
        Color horizon = new(210, 170, 130, 255);
        Color mid = new(135, 185, 220, 255);
        Color zenith = new(45, 95, 170, 255);

        if (t < 0.2f)
        {
            return LerpColor(horizon, mid, t / 0.2f);
        }

        return LerpColor(mid, zenith, (t - 0.2f) / 0.8f);
    }

    private static Vector3 SkyPoint(Vector3 origin, float radius, float elevation, float azimuth) =>
        origin + new Vector3(
            MathF.Cos(elevation) * MathF.Sin(azimuth) * radius,
            MathF.Sin(elevation) * radius,
            MathF.Cos(elevation) * MathF.Cos(azimuth) * radius);

    private static Color LerpColor(Color a, Color b, float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return new Color(
            (int)float.Lerp(a.R, b.R, t),
            (int)float.Lerp(a.G, b.G, t),
            (int)float.Lerp(a.B, b.B, t),
            255);
    }

    private static Color Tint(Color c, float factor) =>
        new(
            Math.Clamp((int)(c.R * factor), 0, 255),
            Math.Clamp((int)(c.G * factor), 0, 255),
            Math.Clamp((int)(c.B * factor), 0, 255),
            c.A);

    private static int Hash(int x, int z)
    {
        unchecked
        {
            int h = x * 374761393 + z * 668265263;
            h = (h ^ (h >> 13)) * 1274126177;
            return h ^ (h >> 16);
        }
    }
}
