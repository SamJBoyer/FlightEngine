using System.Numerics;

namespace FlightEngine.Core;

/// <summary>
/// Simple thrust source modeled as a force vector (no fuel, damage, or complex engine dynamics).
/// </summary>
public readonly struct EngineThrust
{
    public EngineThrust(float maxThrustNewtons, Vector3 localDirection, Vector3 localApplicationPoint)
    {
        MaxThrustNewtons = Math.Max(0f, maxThrustNewtons);
        LocalDirection = localDirection.LengthSquared() > 1e-8f
            ? Vector3.Normalize(localDirection)
            : Vector3.UnitZ;
        LocalApplicationPoint = localApplicationPoint;
    }

    public float MaxThrustNewtons { get; }

    public Vector3 LocalDirection { get; }

    public Vector3 LocalApplicationPoint { get; }

    public ForceVector ToForceVector(float throttle) =>
        ForceVector.LocalThrust(LocalDirection * (MaxThrustNewtons * Math.Max(0f, throttle)), LocalApplicationPoint);
}
