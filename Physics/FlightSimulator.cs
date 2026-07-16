using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Units;

namespace FlightEngine.Physics;

/// <summary>
/// Flight-engine: applies an FCI (and optional force vectors) to a flight-state and advances one tick.
/// </summary>
public sealed class FlightSimulator
{
    private readonly FlightProperties _props;
    private readonly ForceVector[] _scratch;

    public FlightSimulator(FlightProperties properties, int maxExternalForces = 16)
    {
        _props = properties ?? throw new ArgumentNullException(nameof(properties));
        _scratch = new ForceVector[properties.Engines.Length + maxExternalForces + 4];
        Throttle = 1f;
    }

    public FlightProperties Properties => _props;

    /// <summary>Engine throttle in [0, 1]. Not part of FCI (control surfaces only).</summary>
    public float Throttle { get; set; }

    /// <summary>How quickly body rates track control-commanded rates (1/s).</summary>
    public float AngularResponse { get; set; } = 6f;

    public FlightState Tick(in FlightState state, in Fci fci, float deltaTime) =>
        Tick(state, fci, deltaTime, ReadOnlySpan<ForceVector>.Empty);

    public FlightState Tick(
        in FlightState state,
        in Fci fci,
        float deltaTime,
        ReadOnlySpan<ForceVector> externalForces)
    {
        if (deltaTime <= 0f)
        {
            return state;
        }

        float dt = Math.Min(deltaTime, 0.05f);
        float speed = state.SpeedMetersPerSecond;
        Vector3 nose = state.NoseVector;
        Vector3 velocity = state.LinearVelocity;

        float aoa = Aerodynamics.AngleOfAttack(state, _props);
        float authority = Aerodynamics.ControlAuthority(_props, speed, aoa);
        float stallSeverity = Aerodynamics.StallSeverity(_props, speed, aoa);

        int count = 0;
        CollectEngineForces(ref count);
        CollectAeroForces(state, velocity, speed, nose, aoa, stallSeverity, ref count);
        CollectGravity(ref count);

        for (int i = 0; i < externalForces.Length; i++)
        {
            _scratch[count++] = externalForces[i];
        }

        ForceAccumulator.Accumulate(
            state,
            _scratch.AsSpan(0, count),
            out Vector3 worldForce,
            out Vector3 bodyTorqueFromForces);

        ApplyStallRecoveryTorques(state, velocity, speed, nose, stallSeverity, ref bodyTorqueFromForces);

        Vector3 linearAccel = worldForce / _props.MassKg;
        Vector3 newVelocity = velocity + linearAccel * dt;

        // Path follows the nose with attached flow; keeps AoA from spiking in maneuvers.
        float qAuth = Aerodynamics.DynamicPressureAuthority(_props, speed);
        float flow = Aerodynamics.FlowAttachment(aoa, _props);
        float alignStrength = qAuth * flow * Math.Clamp(_props.VelocityAlignRate, 0f, 6f);
        if (speed > 1f && alignStrength > 0.05f)
        {
            float align = 1f - MathF.Exp(-alignStrength * dt);
            Vector3 velDir = Vector3.Normalize(newVelocity);
            Vector3 blended = Vector3.Normalize(Vector3.Lerp(velDir, nose, Math.Clamp(0.75f * align, 0f, 0.95f)));
            newVelocity = blended * newVelocity.Length();
        }

        // Body rates scale with emergent control authority (q × flow attachment).
        Vector3 desiredBodyOmega = new(
            -fci.Elevator * _props.MaxPitchRate * authority,
            fci.Rudder * _props.MaxYawRate * authority,
            -fci.Aileron * _props.MaxRollRate * authority);

        Vector3 inertia = _props.InertiaTensor;
        Vector3 torqueAlpha = new(
            bodyTorqueFromForces.X / Math.Max(inertia.X, 1f),
            bodyTorqueFromForces.Y / Math.Max(inertia.Y, 1f),
            bodyTorqueFromForces.Z / Math.Max(inertia.Z, 1f));

        float rateBlend = 1f - MathF.Exp(-AngularResponse * dt);
        float controlBlend = rateBlend * Math.Clamp(authority + 0.02f, 0.02f, 1f);
        Vector3 newBodyOmega = Vector3.Lerp(state.AngularVelocity, desiredBodyOmega, controlBlend);
        newBodyOmega += torqueAlpha * dt;
        newBodyOmega *= MathF.Exp(-0.15f * dt);

        Quaternion rotationDelta = IntegrateAngularVelocity(newBodyOmega, dt);
        Quaternion newRotation = Quaternion.Normalize(state.Rotation * rotationDelta);
        Vector3 newPosition = state.Position + newVelocity * dt;

        return new FlightState(newPosition, newRotation, newVelocity, newBodyOmega);
    }

    /// <summary>
    /// Stalled when airspeed cannot support flight for this attitude, or when flow is
    /// separated and dynamic pressure is too low to fly out.
    /// </summary>
    public bool IsStalled(in FlightState state)
    {
        float speedKmh = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
        float stallKmh = StallSpeedKmh(state);
        if (speedKmh < stallKmh)
        {
            return true;
        }

        float aoa = Aerodynamics.AngleOfAttack(state, _props);
        float qAuth = Aerodynamics.DynamicPressureAuthority(_props, state.SpeedMetersPerSecond);
        return MathF.Abs(aoa) >= Aerodynamics.CriticalAoA(_props) && qAuth < 0.55f;
    }

    /// <summary>
    /// Diagnostic stall speed for the current attitude (emergent from CLmax / weight,
    /// blended with minimum control speed at extreme pitch).
    /// </summary>
    public float StallSpeedKmh(in FlightState state)
    {
        float pitch = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));
        return Aerodynamics.StallSpeedKmh(_props, pitch);
    }

    private void ApplyStallRecoveryTorques(
        in FlightState state,
        Vector3 velocity,
        float speed,
        Vector3 nose,
        float stallSeverity,
        ref Vector3 bodyTorqueFromForces)
    {
        if (stallSeverity <= 0.05f)
        {
            return;
        }

        Vector3 weightBody = Vector3.Transform(
            new Vector3(0f, -_props.MassKg * _props.Gravity, 0f),
            Quaternion.Inverse(state.Rotation));
        bodyTorqueFromForces += Vector3.Cross(_props.CenterOfGravityLocal, weightBody) * stallSeverity;

        float noseHigh = Math.Clamp(nose.Y, 0f, 1f);
        if (noseHigh > 0f)
        {
            bodyTorqueFromForces.X += _props.StallNoseDownTorque * stallSeverity * (0.35f + 0.65f * noseHigh);
        }

        Vector3 targetDir = speed > 2f && velocity.Y <= 0f
            ? velocity / speed
            : Vector3.Normalize(new Vector3(nose.X, -1f, nose.Z));
        float align = Vector3.Dot(nose, targetDir);
        if (align > -0.92f)
        {
            Vector3 alignAxis = Vector3.Cross(nose, targetDir);
            if (alignAxis.LengthSquared() > 1e-8f)
            {
                float catchUp = Math.Clamp(1f - align, 0f, 1.5f);
                Vector3 bodyAxis = Vector3.Transform(alignAxis, Quaternion.Inverse(state.Rotation));
                bodyTorqueFromForces += bodyAxis * (_props.StallWeathercockGain * stallSeverity * catchUp);
            }
        }

        bodyTorqueFromForces += -state.AngularVelocity * (_props.StallAngularDamping * stallSeverity);
    }

    private void CollectEngineForces(ref int count)
    {
        float throttle = Math.Clamp(Throttle, 0f, 1f);
        for (int i = 0; i < _props.Engines.Length; i++)
        {
            _scratch[count++] = _props.Engines[i].ToForceVector(throttle);
        }
    }

    private void CollectGravity(ref int count)
    {
        _scratch[count++] = ForceVector.World(
            new Vector3(0f, -_props.MassKg * _props.Gravity, 0f),
            Vector3.Zero);
    }

    private void CollectAeroForces(
        in FlightState state,
        Vector3 velocity,
        float speed,
        Vector3 nose,
        float aoa,
        float stallSeverity,
        ref int count)
    {
        if (speed < 0.5f)
        {
            return;
        }

        Vector3 velDir = velocity / speed;
        float q = Aerodynamics.DynamicPressure(_props, speed);
        float cl = Aerodynamics.LiftCoefficient(aoa, _props);

        Vector3 liftDir = Vector3.Cross(velDir, state.RightVector);
        if (liftDir.LengthSquared() < 1e-6f)
        {
            liftDir = state.UpVector;
        }

        liftDir = Vector3.Normalize(liftDir);
        if (Vector3.Dot(liftDir, state.UpVector) < 0f)
        {
            liftDir = -liftDir;
        }

        float liftMag = q * _props.WingArea * MathF.Abs(cl);
        if (cl < 0f)
        {
            liftDir = -liftDir;
        }

        _scratch[count++] = ForceVector.World(liftDir * liftMag, Vector3.Zero);

        float cd = _props.ParasiteDragCoefficient + _props.InducedDragFactor * cl * cl;
        // Separated flow adds bluff-body drag.
        cd += stallSeverity * 0.9f;
        float dragMag = q * _props.WingArea * cd;
        _scratch[count++] = ForceVector.World(-velDir * dragMag, Vector3.Zero);

        Vector3 side = state.RightVector;
        float sideSlip = Vector3.Dot(velDir, side);
        _scratch[count++] = ForceVector.World(-side * (sideSlip * q * _props.WingArea * 0.4f), Vector3.Zero);
    }

    private static Quaternion IntegrateAngularVelocity(Vector3 bodyOmega, float dt)
    {
        float angle = bodyOmega.Length() * dt;
        if (angle < 1e-8f)
        {
            return Quaternion.Identity;
        }

        Vector3 axis = bodyOmega / bodyOmega.Length();
        return Quaternion.CreateFromAxisAngle(axis, angle);
    }
}
