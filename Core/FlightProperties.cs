using System.Numerics;

namespace FlightEngine.Core;

/// <summary>
/// Aerodynamic and mass properties of a single rigid-body aircraft.
/// </summary>
public sealed record FlightProperties
{
    public required float MassKg { get; init; }

    /// <summary>Principal moments of inertia about body X/Y/Z (kg·m²).</summary>
    public required Vector3 InertiaTensor { get; init; }

    public required EngineThrust[] Engines { get; init; }

    /// <summary>
    /// Airframe envelope used to place aero panels for spatial force integration.
    /// </summary>
    public AirframeGeometry Geometry { get; init; } = AirframeGeometry.BaseFighter;

    /// <summary>Reference wing area for lift/drag (m²). Distributed across wing strips.</summary>
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

    /// <summary>
    /// Target peak body rates at reference dynamic pressure (rad/s).
    /// Size rate-damping against hinged-surface torque so steady rates land here.
    /// </summary>
    public float MaxRollRate { get; init; } = MathF.PI / 3f;

    public float MaxPitchRate { get; init; } = MathF.PI * 2f / 15f;

    public float MaxYawRate { get; init; } = MathF.PI * 2f / 15f;

    /// <summary>Peak |ΔCL| on aileron strips at full deflection.</summary>
    public float AileronClGain { get; init; } = 0.85f;

    /// <summary>Peak |ΔCL| on elevator panels at full deflection.</summary>
    public float ElevatorClGain { get; init; } = 0.9f;

    /// <summary>Peak |ΔCY| on the rudder at full deflection.</summary>
    public float RudderClGain { get; init; } = 0.75f;

    /// <summary>Horizontal-tail / elevator area as a fraction of wing area.</summary>
    public float ElevatorAreaFraction { get; init; } = 0.16f;

    /// <summary>Vertical-tail / rudder area as a fraction of wing area.</summary>
    public float RudderAreaFraction { get; init; } = 0.1f;

    /// <summary>
    /// Body-space center of gravity relative to the CoM origin (meters).
    /// Forward (+Z) of the wing so weight pitches the nose down when lift collapses.
    /// </summary>
    public Vector3 CenterOfGravityLocal { get; init; } = new(0f, -0.2f, 0.55f);

    /// <summary>
    /// Body-space wing aero-center / MAC reference used when laying out wing strips.
    /// Slightly behind the CoG so level flight is near trim, while lost lift leaves
    /// a CoG nose-down moment.
    /// </summary>
    public Vector3 AeroCenterLocal { get; init; } = new(0f, 0f, 0.4f);

    /// <summary>How strongly velocity aligns with the nose when lift is producing (1/s).</summary>
    public float VelocityAlignRate { get; init; } = 1.2f;

    /// <summary>
    /// Scales moments from spatially integrated aero panels (including hinged surfaces).
    /// </summary>
    public float AeroTorqueCoupling { get; init; } = 1f;

    /// <summary>
    /// How strongly body rates track Max*Rate targets (1 = full arcade rate ceiling).
    /// </summary>
    public float ControlRateAssist { get; init; } = 1f;

    /// <summary>
    /// How much hinged-surface / strip aero torque couples into body rates.
    /// Keep moderate so surface geometry matters without breaking loop continuity.
    /// </summary>
    public float ControlSurfaceMomentBlend { get; init; } = 0.08f;

    public float AirDensity { get; init; } = 1.225f;

    public float Gravity { get; init; } = 9.81f;
}
