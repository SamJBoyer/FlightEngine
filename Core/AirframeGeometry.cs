namespace FlightEngine.Core;

/// <summary>
/// Rigid-airframe envelope used to build the spatially integrated aero panel layout.
/// </summary>
public readonly record struct AirframeGeometry(
    float LengthMeters,
    float WingspanMeters,
    float HeightMeters,
    float WingAreaSquareMeters)
{
    /// <summary>
    /// Hypothetical base fighter: length 36'1", span 40'9", height 14'2", wing area 300 sq ft.
    /// </summary>
    public static AirframeGeometry BaseFighter { get; } = new(
        LengthMeters: 11.0f,
        WingspanMeters: 12.4f,
        HeightMeters: 4.3f,
        WingAreaSquareMeters: 300f * 0.09290304f);

    public float MeanAerodynamicChordMeters =>
        WingspanMeters > 1e-6f ? WingAreaSquareMeters / WingspanMeters : 0f;
}
