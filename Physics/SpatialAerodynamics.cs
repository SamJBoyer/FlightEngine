using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

/// <summary>
/// Spatially integrates aerodynamic forces over discrete airframe panels.
/// Dynamic pressure uses local flow including rigid-body rotation (v = V + ω × r).
/// Hinged-surface deflections add moments (net force canceled at CoM so maneuvers keep energy).
/// </summary>
internal static class SpatialAerodynamics
{
    public static void Integrate(
        in FlightState state,
        FlightProperties props,
        ReadOnlySpan<AeroPanel> panels,
        in Fci fci,
        float controlAuthority,
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
        float authority = Math.Clamp(controlAuthority, 0f, 1f);
        float qRef = Math.Max(Aerodynamics.ReferenceDynamicPressure(props), 1f);

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

            Vector3 velDir = comSpeed > 0.5f ? comVelDir : localVelWorld / speed;
            float q = 0.5f * props.AirDensity * speed * speed;
            float controlQScale = qRef / Math.Max(q, 1f);

            Vector3 spanWorld = Vector3.Transform(panel.SpanAxisBody, rotation);
            float aoa = panel.Kind == AeroPanelKind.Wing || panel.Kind == AeroPanelKind.HorizontalStabilizer
                ? aircraftAoA
                : LocalFinAoA(velDir, state.NoseVector, worldUp);

            float baselineCl = 0f;
            Vector3 liftDir = worldUp;
            if (panel.LiftEffectiveness > 1e-4f)
            {
                baselineCl = Aerodynamics.LiftCoefficient(aoa, props) * panel.LiftEffectiveness;
                liftDir = BuildLiftDirection(velDir, spanWorld, worldUp, panel.Kind);
            }

            float controlCl = ControlDeltaCl(panel, fci, authority, controlQScale);
            float cl = baselineCl + controlCl;

            float stallSeverity = Aerodynamics.StallSeverity(props, speed, aoa);
            float cd = panel.Kind == AeroPanelKind.Wing
                ? props.ParasiteDragCoefficient + props.InducedDragFactor * cl * cl + stallSeverity * 0.9f
                : 0.015f + 0.5f * props.InducedDragFactor * cl * cl + stallSeverity * 0.25f;

            Vector3 baselineLift = LiftForce(liftDir, q, panel.AreaSquareMeters, baselineCl);
            Vector3 controlLift = LiftForce(liftDir, q, panel.AreaSquareMeters, controlCl);
            Vector3 dragForce = -velDir * (q * panel.AreaSquareMeters * cd);

            Vector3 panelForce = baselineLift + controlLift + dragForce;
            Vector3 controlOnlyForce = controlLift;

            if (panel.Kind == AeroPanelKind.Wing)
            {
                Vector3 side = state.RightVector;
                float sideSlip = Vector3.Dot(velDir, side);
                panelForce += -side * (sideSlip * q * panel.AreaSquareMeters * 0.4f);
            }

            if (panel.ControlRole == ControlRole.Rudder)
            {
                float deflection = Math.Clamp(fci.Rudder, -1f, 1f);
                float cy = panel.ControlSign * deflection * panel.ControlClGain * authority * controlQScale;
                Vector3 rudderForce = Vector3.Transform(
                    new Vector3(cy * q * panel.AreaSquareMeters, 0f, 0f),
                    rotation);
                panelForce += rudderForce;
                controlOnlyForce += rudderForce;
            }

            // Baseline aero + drag translate; control deflection is applied as a pure moment.
            worldForce += panelForce - controlOnlyForce;

            Vector3 bodyForce = Vector3.Transform(panelForce, invRotation);
            bodyTorque += Vector3.Cross(r, bodyForce);
        }
    }

    /// <summary>
    /// Body torque from full deflection on each axis at reference dynamic pressure (authority = 1).
    /// </summary>
    public static Vector3 EstimateFullControlTorqueAtReference(
        FlightProperties props,
        ReadOnlySpan<AeroPanel> panels)
    {
        float qRef = Math.Max(Aerodynamics.ReferenceDynamicPressure(props), 1f);
        Vector3 torqueAileron = Vector3.Zero;
        Vector3 torqueElevator = Vector3.Zero;
        Vector3 torqueRudder = Vector3.Zero;

        for (int i = 0; i < panels.Length; i++)
        {
            AeroPanel panel = panels[i];
            if (panel.ControlRole == ControlRole.None || panel.ControlClGain <= 1e-6f)
            {
                continue;
            }

            float delta = panel.ControlSign * panel.ControlClGain;
            Vector3 bodyForce = panel.ControlRole == ControlRole.Rudder
                ? new Vector3(delta * qRef * panel.AreaSquareMeters, 0f, 0f)
                : new Vector3(0f, delta * qRef * panel.AreaSquareMeters, 0f);

            Vector3 contribution = Vector3.Cross(panel.LocalCentroid, bodyForce);
            switch (panel.ControlRole)
            {
                case ControlRole.Aileron:
                    torqueAileron += contribution;
                    break;
                case ControlRole.Elevator:
                    torqueElevator += contribution;
                    break;
                case ControlRole.Rudder:
                    torqueRudder += contribution;
                    break;
            }
        }

        return new Vector3(torqueElevator.X, torqueRudder.Y, torqueAileron.Z);
    }

    private static Vector3 LiftForce(Vector3 liftDir, float q, float area, float cl)
    {
        float mag = q * area * MathF.Abs(cl);
        if (mag < 1e-8f)
        {
            return Vector3.Zero;
        }

        return (cl < 0f ? -liftDir : liftDir) * mag;
    }

    private static float ControlDeltaCl(in AeroPanel panel, in Fci fci, float authority, float controlQScale)
    {
        if (panel.ControlRole is ControlRole.None or ControlRole.Rudder || panel.ControlClGain <= 1e-6f)
        {
            return 0f;
        }

        float stick = panel.ControlRole switch
        {
            ControlRole.Aileron => Math.Clamp(fci.Aileron, -1f, 1f),
            ControlRole.Elevator => Math.Clamp(fci.Elevator, -1f, 1f),
            _ => 0f
        };

        return panel.ControlSign * stick * panel.ControlClGain * authority * controlQScale;
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
