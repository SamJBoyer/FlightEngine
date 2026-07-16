using System.Numerics;

namespace FlightEngine.Core;

/// <summary>
/// Cardinal aerodynamic state that crosses the boundary layer.
/// Altitude is Y position (meters); speed is |LinearVelocity| (m/s).
/// </summary>
public readonly struct FlightState
{
    public FlightState(
        Vector3 position,
        Quaternion rotation,
        Vector3 linearVelocity,
        Vector3 angularVelocity)
    {
        Position = position;
        Rotation = Quaternion.Normalize(rotation);
        LinearVelocity = linearVelocity;
        AngularVelocity = angularVelocity;
    }

    /// <summary>World position in meters. Y is altitude.</summary>
    public Vector3 Position { get; }

    /// <summary>Body orientation. Local +Z is the nose-vector.</summary>
    public Quaternion Rotation { get; }

    /// <summary>World-space linear velocity in m/s (translation-vector direction when normalized).</summary>
    public Vector3 LinearVelocity { get; }

    /// <summary>Body-space angular velocity in rad/s (X=pitch, Y=yaw, Z=roll).</summary>
    public Vector3 AngularVelocity { get; }

    public Vector3 NoseVector => Vector3.Transform(Vector3.UnitZ, Rotation);

    public Vector3 RightVector => Vector3.Transform(Vector3.UnitX, Rotation);

    public Vector3 UpVector => Vector3.Transform(Vector3.UnitY, Rotation);

    public float AltitudeMeters => Position.Y;

    public float SpeedMetersPerSecond => LinearVelocity.Length();

    public FlightState With(
        Vector3? position = null,
        Quaternion? rotation = null,
        Vector3? linearVelocity = null,
        Vector3? angularVelocity = null) =>
        new(
            position ?? Position,
            rotation ?? Rotation,
            linearVelocity ?? LinearVelocity,
            angularVelocity ?? AngularVelocity);
}
