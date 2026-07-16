using FlightEngine.Core;

namespace FlightEngine.Aircraft;

/// <summary>Named aircraft preset for demos and the visualizer roster.</summary>
public sealed class AircraftDefinition
{
    public required string Id { get; init; }

    public required string DisplayName { get; init; }

    /// <summary>Short trait label shown in the HUD (e.g. "good roll").</summary>
    public required string Trait { get; init; }

    public required FlightProperties Properties { get; init; }

    public required PlaneVisualStyle Visual { get; init; }
}

/// <summary>Visualizer silhouette / paint scheme key.</summary>
public enum PlaneVisualStyle
{
    Baseline,
    FastRoll,
    SlowRoll,
    TightTurn,
    WideTurn,
    TwinEngine,
    FastCruise,
    SlowCruise,
    LateCompression,
    EarlyCompression
}
