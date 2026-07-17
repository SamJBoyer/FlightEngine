using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

/// <summary>
/// Builds wing strips plus hinged control surfaces from airframe geometry.
/// Wing strip areas sum to <see cref="FlightProperties.WingArea"/>.
/// </summary>
internal static class AeroPanelBuilder
{
    public const int WingStripCount = 12;

    /// <summary>Outer fraction of semi-span used as aileron (per side).</summary>
    public const float AileronSpanFraction = 0.34f;

    public static AeroPanel[] Build(FlightProperties props)
    {
        AirframeGeometry geo = props.Geometry;
        float wingArea = props.WingArea > 1e-6f ? props.WingArea : geo.WingAreaSquareMeters;
        float span = Math.Max(geo.WingspanMeters, 1f);
        float length = Math.Max(geo.LengthMeters, 1f);
        float height = Math.Max(geo.HeightMeters, 1f);
        float mac = Math.Max(geo.MeanAerodynamicChordMeters, wingArea / span);
        Vector3 wingRoot = props.AeroCenterLocal;

        float aileronGain = props.AileronClGain;
        float elevatorGain = props.ElevatorClGain;
        float rudderGain = props.RudderClGain;

        float stripArea = wingArea / WingStripCount;
        float stripWidth = span / WingStripCount;
        float aileronYStart = 0.5f * span * (1f - AileronSpanFraction);

        // Wing strips + 2 elevator halves + 1 rudder.
        AeroPanel[] panels = new AeroPanel[WingStripCount + 3];
        int n = 0;

        for (int i = 0; i < WingStripCount; i++)
        {
            float y = -0.5f * span + (i + 0.5f) * stripWidth;
            float z = wingRoot.Z + 0.02f * MathF.Abs(y) / (0.5f * span);
            Vector3 centroid = new(y, wingRoot.Y, z);

            bool isAileron = MathF.Abs(y) >= aileronYStart;
            if (isAileron)
            {
                // +aileron → more lift on left / less on right → right-wing-down (ωz < 0).
                float sign = y >= 0f ? -1f : 1f;
                panels[n++] = new AeroPanel(
                    centroid,
                    stripArea,
                    AeroPanelKind.Wing,
                    liftEffectiveness: 1f,
                    spanAxisBody: Vector3.UnitX,
                    controlRole: ControlRole.Aileron,
                    controlSign: sign,
                    controlClGain: aileronGain);
            }
            else
            {
                panels[n++] = new AeroPanel(centroid, stripArea, AeroPanelKind.Wing, 1f, Vector3.UnitX);
            }
        }

        float tailZ = -0.58f * length + 0.35f * mac;
        float hTailArea = wingArea * props.ElevatorAreaFraction;
        float vTailArea = wingArea * props.RudderAreaFraction;

        // Elevator: +elevator stick → downforce on tail → nose up.
        // Low fixed-camber effectiveness so the tail doesn't overpower pitch authority.
        panels[n++] = new AeroPanel(
            new Vector3(-0.04f * span, 0.04f, tailZ),
            hTailArea * 0.5f,
            AeroPanelKind.HorizontalStabilizer,
            liftEffectiveness: 0.12f,
            spanAxisBody: Vector3.UnitX,
            controlRole: ControlRole.Elevator,
            controlSign: -1f,
            controlClGain: elevatorGain);
        panels[n++] = new AeroPanel(
            new Vector3(0.04f * span, 0.04f, tailZ),
            hTailArea * 0.5f,
            AeroPanelKind.HorizontalStabilizer,
            liftEffectiveness: 0.12f,
            spanAxisBody: Vector3.UnitX,
            controlRole: ControlRole.Elevator,
            controlSign: -1f,
            controlClGain: elevatorGain);

        // Rudder: +rudder → fin force to -X → nose-right yaw (+Y).
        panels[n++] = new AeroPanel(
            new Vector3(0f, 0.32f * height, tailZ),
            vTailArea,
            AeroPanelKind.VerticalStabilizer,
            liftEffectiveness: 0.1f,
            spanAxisBody: Vector3.UnitY,
            controlRole: ControlRole.Rudder,
            controlSign: -1f,
            controlClGain: rudderGain);

        return panels;
    }
}
