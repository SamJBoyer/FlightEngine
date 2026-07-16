using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

internal static class ForceAccumulator
{
    public static void Accumulate(
        in FlightState state,
        ReadOnlySpan<ForceVector> forces,
        out Vector3 worldForce,
        out Vector3 bodyTorque)
    {
        worldForce = Vector3.Zero;
        bodyTorque = Vector3.Zero;

        for (int i = 0; i < forces.Length; i++)
        {
            ForceVector fv = forces[i];
            Vector3 world = fv.ForceInLocalSpace
                ? Vector3.Transform(fv.ForceNewtons, state.Rotation)
                : fv.ForceNewtons;

            worldForce += world;

            Vector3 bodyForce = fv.ForceInLocalSpace
                ? fv.ForceNewtons
                : Vector3.Transform(fv.ForceNewtons, Quaternion.Inverse(state.Rotation));

            bodyTorque += Vector3.Cross(fv.LocalApplicationPoint, bodyForce);
        }
    }
}
