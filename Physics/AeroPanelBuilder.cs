using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

/// <summary>
/// Builds a spanwise wing-strip layout from airframe geometry.
/// Strip areas sum to <see cref="FlightProperties.WingArea"/>.
/// </summary>
internal static class AeroPanelBuilder
{
    public const int WingStripCount = 12;

    public static AeroPanel[] Build(FlightProperties props)
    {
        AirframeGeometry geo = props.Geometry;
        float wingArea = props.WingArea > 1e-6f ? props.WingArea : geo.WingAreaSquareMeters;
        float span = Math.Max(geo.WingspanMeters, 1f);
        Vector3 wingRoot = props.AeroCenterLocal;

        float stripArea = wingArea / WingStripCount;
        float stripWidth = span / WingStripCount;
        AeroPanel[] panels = new AeroPanel[WingStripCount];

        for (int i = 0; i < WingStripCount; i++)
        {
            float y = -0.5f * span + (i + 0.5f) * stripWidth;
            float z = wingRoot.Z + 0.02f * MathF.Abs(y) / (0.5f * span);
            Vector3 centroid = new(y, wingRoot.Y, z);
            panels[i] = new AeroPanel(centroid, stripArea, AeroPanelKind.Wing, 1f, Vector3.UnitX);
        }

        return panels;
    }
}
