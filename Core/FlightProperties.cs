using System.Numerics;

namespace FlightEngine.Core;

/// <summary>
/// Aerodynamic and mass properties of a single rigid-body aircraft.
/// </summary>
public sealed class FlightProperties
{
    public required float MassKg { get; init; }

    /// <summary>Principal moments of inertia about body X/Y/Z (kg·m²).</summary>
    public required Vector3 InertiaTensor { get; init; }

    public required EngineThrust[] Engines { get; init; }

    /// <summary>Reference wing area for lift/drag (m²).</summary>
    public float WingArea { get; init; } = 20f;

    /// <summary>Lift coefficient slope per radian of angle of attack.</summary>
    public float LiftSlope { get; init; } = 4.5f;

    public float MaxLiftCoefficient { get; init; } = 1.4f;

    public float ParasiteDragCoefficient { get; init; } = 0.025f;

    public float InducedDragFactor { get; init; } = 0.08f;

    /// <summary>Peak control rates at design dynamic pressure (rad/s).</summary>
    public float MaxRollRate { get; init; } = MathF.PI / 3f; // 60 deg/s → 6s roll

    public float MaxPitchRate { get; init; } = MathF.PI * 2f / 15f; // 24 deg/s → 15s loop

    public float MaxYawRate { get; init; } = MathF.PI * 2f / 15f;

    /// <summary>Level-flight stall speed (km/h) at 0° pitch.</summary>
    public float LevelStallSpeedKmh { get; init; } = 150f;

    /// <summary>Stall speed (km/h) when nose is pointed straight up (90°).</summary>
    public float VerticalStallSpeedKmh { get; init; } = 40f;

    /// <summary>
    /// Body-space center of gravity relative to the aero reference (meters).
    /// Forward (+Z) CoG produces nose-down pitch when wing lift collapses.
    /// </summary>
    public Vector3 CenterOfGravityLocal { get; init; } = new(0f, 0f, 1.2f);

    /// <summary>Fraction of lift retained in a deep stall (0–1).</summary>
    public float StallLiftRetention { get; init; } = 0.08f;

    /// <summary>
    /// Extra body-torque gain that weathervanes the nose into the velocity vector while stalled.
    /// </summary>
    public float StallWeathercockGain { get; init; } = 14000f;

    /// <summary>
    /// Direct nose-down body torque (N·m scale) applied while stalled, strongest when nose-high.
    /// Covers the vertical case where CoG lever arm and weathercock both go to zero.
    /// </summary>
    public float StallNoseDownTorque { get; init; } = 22000f;

    /// <summary>Extra angular damping while stalled to prevent end-over-end tumbling.</summary>
    public float StallAngularDamping { get; init; } = 9000f;

    /// <summary>Speed (km/h) where compression begins reducing control effectiveness.</summary>
    public float CompressionStartKmh { get; init; } = 550f;

    /// <summary>Additional speed (km/h) above start for near-full compression.</summary>
    public float CompressionSpanKmh { get; init; } = 250f;

    /// <summary>Angular damping coefficients (body axes).</summary>
    public Vector3 AngularDamping { get; init; } = new(3.5f, 3.5f, 4f);

    /// <summary>How strongly velocity aligns with the nose (1/s).</summary>
    public float VelocityAlignRate { get; init; } = 1.2f;

    public float AirDensity { get; init; } = 1.225f;

    public float Gravity { get; init; } = 9.81f;
}
