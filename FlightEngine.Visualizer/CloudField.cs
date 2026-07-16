using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Visualizer;

/// <summary>
/// Procedural cloud textures billboarded around the camera for sky reference.
/// </summary>
internal sealed class CloudField : IDisposable
{
    private readonly Texture2D _softCloud;
    private readonly Texture2D _wispyCloud;
    private readonly Cloud[] _clouds;
    private bool _disposed;

    private readonly struct Cloud
    {
        public Cloud(Vector3 localOffset, float size, int textureIndex, float drift)
        {
            LocalOffset = localOffset;
            Size = size;
            TextureIndex = textureIndex;
            Drift = drift;
        }

        public Vector3 LocalOffset { get; }
        public float Size { get; }
        public int TextureIndex { get; }
        public float Drift { get; }
    }

    public CloudField(int count = 48)
    {
        _softCloud = BuildSoftCloudTexture(256);
        _wispyCloud = BuildWispyCloudTexture(256);

        var rng = new Random(42);
        _clouds = new Cloud[count];
        for (int i = 0; i < count; i++)
        {
            float angle = (float)(rng.NextDouble() * Math.Tau);
            float radius = 400f + (float)rng.NextDouble() * 2200f;
            float height = 350f + (float)rng.NextDouble() * 900f;
            float size = 180f + (float)rng.NextDouble() * 420f;
            _clouds[i] = new Cloud(
                new Vector3(MathF.Cos(angle) * radius, height, MathF.Sin(angle) * radius),
                size,
                rng.Next(0, 2),
                (float)(rng.NextDouble() * Math.Tau));
        }
    }

    public void Draw(in Camera3D camera, float timeSeconds)
    {
        Vector3 origin = new(camera.Position.X, 0f, camera.Position.Z);
        Raylib.BeginBlendMode(BlendMode.Alpha);

        for (int i = 0; i < _clouds.Length; i++)
        {
            Cloud cloud = _clouds[i];
            float sway = MathF.Sin(timeSeconds * 0.03f + cloud.Drift) * 40f;
            Vector3 pos = origin + cloud.LocalOffset + new Vector3(sway, 0f, sway * 0.35f);

            // Keep clouds from sitting on top of the camera.
            Vector3 toCam = pos - camera.Position;
            if (toCam.LengthSquared() < 120f * 120f)
            {
                continue;
            }

            Texture2D tex = cloud.TextureIndex == 0 ? _softCloud : _wispyCloud;
            Color tint = cloud.TextureIndex == 0
                ? new Color(255, 255, 255, 210)
                : new Color(245, 248, 255, 170);

            Raylib.DrawBillboard(camera, tex, pos, cloud.Size, tint);
        }

        Raylib.EndBlendMode();
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        Raylib.UnloadTexture(_softCloud);
        Raylib.UnloadTexture(_wispyCloud);
        _disposed = true;
    }

    private static Texture2D BuildSoftCloudTexture(int size)
    {
        Image img = Raylib.GenImageColor(size, size, new Color(0, 0, 0, 0));
        var rng = new Random(7);
        int cx = size / 2;
        int cy = size / 2;

        // Layered soft blobs → puffy cumulus look.
        for (int i = 0; i < 14; i++)
        {
            int ox = cx + rng.Next(-size / 5, size / 5);
            int oy = cy + rng.Next(-size / 6, size / 6);
            int radius = size / 6 + rng.Next(0, size / 8);
            for (int r = radius; r > 0; r -= 3)
            {
                int a = Math.Clamp(18 + r * 2, 0, 90);
                Raylib.ImageDrawCircle(ref img, ox, oy, r, new Color(255, 255, 255, a));
            }
        }

        // Soft edge falloff.
        Image falloff = Raylib.GenImageGradientRadial(size, size, 0.2f, new Color(255, 255, 255, 255), new Color(0, 0, 0, 0));
        Raylib.ImageDraw(
            ref img,
            falloff,
            new Rectangle(0, 0, size, size),
            new Rectangle(0, 0, size, size),
            new Color(255, 255, 255, 90));
        Raylib.UnloadImage(falloff);

        Texture2D tex = Raylib.LoadTextureFromImage(img);
        Raylib.UnloadImage(img);
        Raylib.SetTextureFilter(tex, TextureFilter.Bilinear);
        return tex;
    }

    private static Texture2D BuildWispyCloudTexture(int size)
    {
        Image img = Raylib.GenImageColor(size, size, new Color(0, 0, 0, 0));
        var rng = new Random(99);

        for (int i = 0; i < 10; i++)
        {
            int ox = size / 2 + rng.Next(-size / 3, size / 3);
            int oy = size / 2 + rng.Next(-size / 5, size / 5);
            int rx = size / 3 + rng.Next(0, size / 6);
            int ry = size / 10 + rng.Next(0, size / 14);

            for (int y = -ry; y <= ry; y++)
            {
                for (int x = -rx; x <= rx; x++)
                {
                    float nx = x / (float)rx;
                    float ny = y / (float)Math.Max(ry, 1);
                    float d = nx * nx + ny * ny * 2.8f;
                    if (d > 1f)
                    {
                        continue;
                    }

                    int a = (int)Math.Clamp((1f - d) * (1f - d) * 110f, 0f, 110f);
                    int px = ox + x;
                    int py = oy + y;
                    if (px < 0 || py < 0 || px >= size || py >= size)
                    {
                        continue;
                    }

                    Raylib.ImageDrawPixel(ref img, px, py, new Color(255, 255, 255, a));
                }
            }
        }

        Texture2D tex = Raylib.LoadTextureFromImage(img);
        Raylib.UnloadImage(img);
        Raylib.SetTextureFilter(tex, TextureFilter.Bilinear);
        return tex;
    }
}
