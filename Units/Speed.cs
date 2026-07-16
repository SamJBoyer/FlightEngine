namespace FlightEngine.Units;

/// <summary>
/// Metric speed helpers. Simulation uses m/s internally; km/h is the display/test standard.
/// </summary>
public static class Speed
{
    public const float MetersPerSecondPerKmh = 1000f / 3600f;
    public const float KmhPerMetersPerSecond = 3600f / 1000f;

    public static float KmhToMetersPerSecond(float kmh) => kmh * MetersPerSecondPerKmh;

    public static float MetersPerSecondToKmh(float metersPerSecond) => metersPerSecond * KmhPerMetersPerSecond;
}
