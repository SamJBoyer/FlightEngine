using System.Numerics;

namespace FlightEngine.Core;

/// <summary>
/// A force applied at a point on the rigid body. Produces translation and, when offset from CoM, rotation.
/// </summary>
public readonly struct ForceVector
{
    public ForceVector(Vector3 forceNewtons, Vector3 localApplicationPoint, bool forceInLocalSpace = false)
    {
        ForceNewtons = forceNewtons;
        LocalApplicationPoint = localApplicationPoint;
        ForceInLocalSpace = forceInLocalSpace;
    }

    public Vector3 ForceNewtons { get; }

    /// <summary>Application point in body space relative to center of mass (meters).</summary>
    public Vector3 LocalApplicationPoint { get; }

    /// <summary>When true, <see cref="ForceNewtons"/> is expressed in body axes.</summary>
    public bool ForceInLocalSpace { get; }

    public static ForceVector LocalThrust(Vector3 localForceNewtons, Vector3 localApplicationPoint) =>
        new(localForceNewtons, localApplicationPoint, forceInLocalSpace: true);

    public static ForceVector World(Vector3 worldForceNewtons, Vector3 localApplicationPoint) =>
        new(worldForceNewtons, localApplicationPoint, forceInLocalSpace: false);
}
