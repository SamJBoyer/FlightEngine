using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Visualizer;

internal static class WorldRenderer
{
    public static void Draw(float focusX, float focusZ)
    {
        Raylib.DrawPlane(new Vector3(focusX, 0f, focusZ), new Vector2(8000f, 8000f), new Color(72, 110, 72, 255));

        const float spacing = 100f;
        const int half = 40;
        int originX = (int)MathF.Floor(focusX / spacing);
        int originZ = (int)MathF.Floor(focusZ / spacing);

        Color grid = new(55, 90, 55, 255);
        for (int i = -half; i <= half; i++)
        {
            float x = (originX + i) * spacing;
            float z0 = (originZ - half) * spacing;
            float z1 = (originZ + half) * spacing;
            Raylib.DrawLine3D(new Vector3(x, 0.05f, z0), new Vector3(x, 0.05f, z1), grid);

            float z = (originZ + i) * spacing;
            float x0 = (originX - half) * spacing;
            float x1 = (originX + half) * spacing;
            Raylib.DrawLine3D(new Vector3(x0, 0.05f, z), new Vector3(x1, 0.05f, z), grid);
        }
    }
}
