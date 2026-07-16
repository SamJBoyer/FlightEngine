using FlightEngine.Core;
using FlightEngine.Units;

namespace FlightEngine.Physics;

internal static class ControlEffectiveness
{
    /// <summary>
    /// Stall speed interpolates from level to vertical based on nose pitch (radians from horizon).
    /// </summary>
    public static float StallSpeedKmh(FlightProperties props, float pitchRadians)
    {
        float t = Math.Clamp(MathF.Abs(pitchRadians) / (MathF.PI * 0.5f), 0f, 1f);
        return props.LevelStallSpeedKmh + (props.VerticalStallSpeedKmh - props.LevelStallSpeedKmh) * t;
    }

    public static float StallFactor(float speedKmh, float stallSpeedKmh)
    {
        if (stallSpeedKmh <= 1f)
        {
            return 1f;
        }

        // Soft floor so the aircraft is nearly dead below stall, lively above.
        float ratio = speedKmh / stallSpeedKmh;
        if (ratio >= 1.15f)
        {
            return 1f;
        }

        if (ratio <= 0.85f)
        {
            return 0.05f;
        }

        return 0.05f + 0.95f * ((ratio - 0.85f) / 0.3f);
    }

    public static float CompressionFactor(FlightProperties props, float speedKmh)
    {
        if (speedKmh <= props.CompressionStartKmh)
        {
            return 1f;
        }

        float x = (speedKmh - props.CompressionStartKmh) / Math.Max(1f, props.CompressionSpanKmh);
        return 1f / (1f + x * x);
    }

    public static float Evaluate(FlightProperties props, float speedMetersPerSecond, float pitchRadians)
    {
        float speedKmh = Speed.MetersPerSecondToKmh(speedMetersPerSecond);
        float stall = StallFactor(speedKmh, StallSpeedKmh(props, pitchRadians));
        float compression = CompressionFactor(props, speedKmh);
        return stall * compression;
    }
}
