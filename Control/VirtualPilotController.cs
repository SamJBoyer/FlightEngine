using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Control;

/// <summary>
/// VPC: translates a flight-cursor point into an FCI that steers the translation-vector toward it,
/// compensating for unbalanced engine force vectors.
/// </summary>
public sealed class VirtualPilotController
{
    private readonly FlightProperties _props;
    private readonly Vector3 _engineTorqueBias;

    public VirtualPilotController(FlightProperties properties, float throttleForBias = 1f)
    {
        _props = properties ?? throw new ArgumentNullException(nameof(properties));
        _engineTorqueBias = EstimateEngineTorqueBias(properties, throttleForBias);
    }

    public float ProportionalGain { get; set; } = 2.8f;

    public float RollGain { get; set; } = 2.2f;

    public Fci ComputeFci(in FlightState state, Vector3 flightCursorWorld)
    {
        Vector3 toTarget = flightCursorWorld - state.Position;
        if (toTarget.LengthSquared() < 1e-4f)
        {
            return CompensateImbalance(Fci.Neutral);
        }

        Vector3 desiredDir = Vector3.Normalize(toTarget);

        // Steer primarily with the nose; translation follows via aero alignment.
        Vector3 bodyDesired = Vector3.Transform(desiredDir, Quaternion.Inverse(state.Rotation));

        // Pitch/yaw error in body space: desired direction relative to nose (+Z).
        float elevator = Math.Clamp(bodyDesired.Y * ProportionalGain, -1f, 1f);
        float rudder = Math.Clamp(bodyDesired.X * ProportionalGain, -1f, 1f);

        // Bank into horizontal turn demand.
        Vector3 flatDesired = new(desiredDir.X, 0f, desiredDir.Z);
        float aileron = 0f;
        if (flatDesired.LengthSquared() > 1e-4f)
        {
            flatDesired = Vector3.Normalize(flatDesired);
            Vector3 flatNose = new(state.NoseVector.X, 0f, state.NoseVector.Z);
            if (flatNose.LengthSquared() > 1e-4f)
            {
                flatNose = Vector3.Normalize(flatNose);
                float yawErr = MathF.Atan2(
                    Vector3.Dot(Vector3.Cross(flatNose, flatDesired), Vector3.UnitY),
                    Vector3.Dot(flatNose, flatDesired));

                float desiredBank = Math.Clamp(yawErr * 1.25f, -1.2f, 1.2f);
                float currentBank = MathF.Atan2(-state.RightVector.Y, state.UpVector.Y);
                aileron = Math.Clamp((desiredBank - currentBank) * RollGain, -1f, 1f);
            }
        }

        // Prefer roll+elevator for large lateral errors; bleed rudder when banked.
        float bankAbs = MathF.Abs(MathF.Atan2(-state.RightVector.Y, state.UpVector.Y));
        if (bankAbs > 0.4f)
        {
            rudder *= 0.35f;
            elevator = Math.Clamp(elevator + MathF.Abs(aileron) * 0.25f, -1f, 1f);
        }

        return CompensateImbalance(new Fci(aileron, elevator, rudder));
    }

    private Fci CompensateImbalance(in Fci raw)
    {
        Vector3 inertia = _props.InertiaTensor;
        float aileronTrim = _engineTorqueBias.Z / Math.Max(inertia.Z, 1f) * 0.2f;
        float rudderTrim = -_engineTorqueBias.Y / Math.Max(inertia.Y, 1f) * 0.2f;
        float elevatorTrim = _engineTorqueBias.X / Math.Max(inertia.X, 1f) * 0.2f;

        return new Fci(
            raw.Aileron + aileronTrim,
            raw.Elevator + elevatorTrim,
            raw.Rudder + rudderTrim);
    }

    private static Vector3 EstimateEngineTorqueBias(FlightProperties props, float throttle)
    {
        Vector3 torque = Vector3.Zero;
        foreach (EngineThrust engine in props.Engines)
        {
            ForceVector fv = engine.ToForceVector(throttle);
            torque += Vector3.Cross(fv.LocalApplicationPoint, fv.ForceNewtons);
        }

        return torque;
    }
}
