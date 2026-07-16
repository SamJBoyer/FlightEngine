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

    /// <summary>AoA span (radians) over which CL falls from peak to deep-stall remnant.</summary>
    public float StallAoAWidth { get; init; } = 0.35f;

    /// <summary>Fraction of CLmax retained in deep stall.</summary>
    public float StallLiftRetention { get; init; } = 0.12f;

    /// <summary>Control-surface retention once flow is fully separated.</summary>
    public float PostStallControlRetention { get; init; } = 0.08f;

    /// <summary>
    /// Speed (km/h) where dynamic-pressure control authority peaks (design maneuver speed).
    /// </summary>
    public float ReferenceSpeedKmh { get; init; } = 400f;

    /// <summary>
    /// Minimum useful flying speed (km/h) used when reporting stall speed at extreme pitch.
    /// Emerges as the floor of attitude-dependent stall diagnostics.
    /// </summary>
    public float MinimumControlSpeedKmh { get; init; } = 40f;

    /// <summary>Peak control rates at reference dynamic pressure (rad/s).</summary>
    public float MaxRollRate { get; init; } = MathF.PI / 3f;

    public float MaxPitchRate { get; init; } = MathF.PI * 2f / 15f;

    public float MaxYawRate { get; init; } = MathF.PI * 2f / 15f;

    /// <summary>
    /// Body-space center of gravity relative to the aero reference (meters).
    /// Forward (+Z) CoG produces nose-down pitch when wing lift collapses.
    /// </summary>
    public Vector3 CenterOfGravityLocal { get; init; } = new(0f, 0f, 1.2f);

    /// <summary>
    /// Extra body-torque gain that weathervanes the nose into the velocity vector while stalled.
    /// </summary>
    public float StallWeathercockGain { get; init; } = 14000f;

    /// <summary>
    /// Direct nose-down body torque (N·m scale) applied while stalled, strongest when nose-high.
    /// </summary>
    public float StallNoseDownTorque { get; init; } = 22000f;

    /// <summary>Extra angular damping while stalled to prevent end-over-end tumbling.</summary>
    public float StallAngularDamping { get; init; } = 9000f;

    /// <summary>How strongly velocity aligns with the nose when lift is producing (1/s).</summary>
    public float VelocityAlignRate { get; init; } = 1.2f;

    public float AirDensity { get; init; } = 1.225f;

    public float Gravity { get; init; } = 9.81f;
}
