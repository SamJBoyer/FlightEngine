using System.Numerics;

namespace FlightEngine.Physics;

internal enum AeroPanelKind : byte
{
    Wing = 0,
    HorizontalStabilizer = 1,
    VerticalStabilizer = 2,
    Fuselage = 3
}

/// <summary>
/// Discrete surface element for spatial force integration. Centroids are body-local (meters, CoM origin).
/// </summary>
internal readonly struct AeroPanel
{
    public AeroPanel(
        Vector3 localCentroid,
        float areaSquareMeters,
        AeroPanelKind kind,
        float liftEffectiveness,
        Vector3 spanAxisBody)
    {
        LocalCentroid = localCentroid;
        AreaSquareMeters = Math.Max(0f, areaSquareMeters);
        Kind = kind;
        LiftEffectiveness = Math.Clamp(liftEffectiveness, 0f, 1f);
        SpanAxisBody = spanAxisBody.LengthSquared() > 1e-8f
            ? Vector3.Normalize(spanAxisBody)
            : Vector3.UnitX;
    }

    public Vector3 LocalCentroid { get; }

    public float AreaSquareMeters { get; }

    public AeroPanelKind Kind { get; }

    /// <summary>1 = full wing polar, lower for tails, 0 = drag-only (fuselage).</summary>
    public float LiftEffectiveness { get; }

    /// <summary>Body-space span axis used to build the local lift direction.</summary>
    public Vector3 SpanAxisBody { get; }
}
