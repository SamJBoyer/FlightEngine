using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

/// <summary>
/// Spatially integrates aerodynamic forces over discrete airframe panels.
/// Dynamic pressure uses local flow including rigid-body rotation (v = V + ω × r);
/// wing AoA uses the aircraft-level diagnostic so strip sums match the prior lumped polar.
/// </summary>
internal static class SpatialAerodynamics
{
    public static void Integrate(
        in FlightState state,
        FlightProperties props,
        ReadOnlySpan<AeroPanel> panels,
        out Vector3 worldForce,
        out Vector3 bodyTorque)
    {
        worldForce = Vector3.Zero;
        bodyTorque = Vector3.Zero;

        if (panels.Length == 0)
        {
            return;
        }

        Quaternion rotation = state.Rotation;
        Quaternion invRotation = Quaternion.Inverse(rotation);
        Vector3 linearVelocity = state.LinearVelocity;
        Vector3 bodyOmega = state.AngularVelocity;
        Vector3 worldUp = state.UpVector;
        float aircraftAoA = Aerodynamics.AngleOfAttack(state, props);

        Vector3 comVelDir = Vector3.Zero;
        float comSpeed = linearVelocity.Length();
        if (comSpeed > 1e-4f)
        {
            comVelDir = linearVelocity / comSpeed;
        }

        for (int i = 0; i < panels.Length; i++)
        {
            AeroPanel panel = panels[i];
            if (panel.AreaSquareMeters < 1e-8f)
            {
                continue;
            }

            Vector3 r = panel.LocalCentroid;
            Vector3 localFlowBody = Vector3.Cross(bodyOmega, r);
            Vector3 localVelWorld = linearVelocity + Vector3.Transform(localFlowBody, rotation);
            float speed = localVelWorld.Length();
            if (speed < 0.5f)
            {
                continue;
            }

            // Direction from CoM flow (stable polar); magnitude from local flow (spatial q).
            Vector3 velDir = comSpeed > 0.5f ? comVelDir : localVelWorld / speed;
            float q = 0.5f * props.AirDensity * speed * speed;

            Vector3 spanWorld = Vector3.Transform(panel.SpanAxisBody, rotation);
            float aoa = panel.Kind == AeroPanelKind.Wing || panel.Kind == AeroPanelKind.HorizontalStabilizer
                ? aircraftAoA
                : LocalFinAoA(velDir, state.NoseVector, worldUp);

            float cl = 0f;
            Vector3 liftDir = worldUp;
            if (panel.LiftEffectiveness > 1e-4f)
            {
                cl = Aerodynamics.LiftCoefficient(aoa, props) * panel.LiftEffectiveness;
                liftDir = BuildLiftDirection(velDir, spanWorld, worldUp, panel.Kind);
            }

            float stallSeverity = Aerodynamics.StallSeverity(props, speed, aoa);
            float cd;
            if (panel.Kind == AeroPanelKind.Wing)
            {
                cd = props.ParasiteDragCoefficient + props.InducedDragFactor * cl * cl;
                cd += stallSeverity * 0.9f;
            }
            else
            {
                // Non-wing stations (if added later): keep out of the wing Cd0 budget.
                cd = 0.015f + 0.5f * props.InducedDragFactor * cl * cl;
                cd += stallSeverity * 0.25f;
            }

            // Match prior lumped convention: magnitude from |CL|, direction flipped when CL < 0.
            float liftMag = q * panel.AreaSquareMeters * MathF.Abs(cl);
            if (cl < 0f)
            {
                liftDir = -liftDir;
            }

            Vector3 panelForce = liftDir * liftMag - velDir * (q * panel.AreaSquareMeters * cd);

            if (panel.Kind == AeroPanelKind.Wing)
            {
                Vector3 side = state.RightVector;
                float sideSlip = Vector3.Dot(velDir, side);
                panelForce += -side * (sideSlip * q * panel.AreaSquareMeters * 0.4f);
            }

            worldForce += panelForce;

            Vector3 bodyForce = Vector3.Transform(panelForce, invRotation);
            bodyTorque += Vector3.Cross(r, bodyForce);
        }
    }

    private static Vector3 BuildLiftDirection(
        Vector3 velDir,
        Vector3 spanWorld,
        Vector3 worldUp,
        AeroPanelKind kind)
    {
        Vector3 liftDir = Vector3.Cross(velDir, spanWorld);
        if (liftDir.LengthSquared() < 1e-8f)
        {
            liftDir = kind == AeroPanelKind.VerticalStabilizer
                ? Vector3.Cross(velDir, worldUp)
                : worldUp;
        }

        liftDir = Vector3.Normalize(liftDir);
        Vector3 prefer = kind == AeroPanelKind.VerticalStabilizer
            ? Vector3.Cross(Vector3.Normalize(spanWorld), velDir)
            : worldUp;
        if (prefer.LengthSquared() > 1e-8f && Vector3.Dot(liftDir, Vector3.Normalize(prefer)) < 0f)
        {
            liftDir = -liftDir;
        }

        return liftDir;
    }

    private static float LocalFinAoA(Vector3 velDir, Vector3 nose, Vector3 worldUp)
    {
        Vector3 lateral = Vector3.Cross(worldUp, nose);
        if (lateral.LengthSquared() < 1e-8f)
        {
            return 0f;
        }

        return SignedAngle(velDir, nose, Vector3.Normalize(lateral));
    }

    private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
    {
        Vector3 fromN = Vector3.Normalize(from);
        Vector3 toN = Vector3.Normalize(to);
        float sin = Vector3.Dot(axis, Vector3.Cross(fromN, toN));
        float cos = Vector3.Dot(fromN, toN);
        return MathF.Atan2(sin, cos);
    }
}
