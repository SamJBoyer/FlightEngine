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

        float pitchFromHorizon = MathF.Asin(Math.Clamp(nose.Y, -1f, 1f));
        float speedKmh = Speed.MetersPerSecondToKmh(speed);
        float stallSpeedKmh = ControlEffectiveness.StallSpeedKmh(_props, pitchFromHorizon);
        float stallFactor = ControlEffectiveness.StallFactor(speedKmh, stallSpeedKmh);
        float compression = ControlEffectiveness.CompressionFactor(_props, speedKmh);
        float effectiveness = stallFactor * compression;
        float stallSeverity = 1f - stallFactor;

        int count = 0;
        CollectEngineForces(ref count);
        CollectAeroForces(state, velocity, speed, nose, stallFactor, stallSeverity, ref count);
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

        if (stallSeverity > 0.05f)
        {
            // Forward-CoG moment only engages in stall (keeps normal flight trim clean).
            Vector3 weightBody = Vector3.Transform(
                new Vector3(0f, -_props.MassKg * _props.Gravity, 0f),
                Quaternion.Inverse(state.Rotation));
            bodyTorqueFromForces += Vector3.Cross(_props.CenterOfGravityLocal, weightBody) * stallSeverity;

            // Nose-down while above the horizon; fade out once pitched down into the dive.
            float noseHigh = Math.Clamp(nose.Y, 0f, 1f);
            if (noseHigh > 0f)
            {
                bodyTorqueFromForces.X += _props.StallNoseDownTorque * stallSeverity * (0.35f + 0.65f * noseHigh);
            }

            // Weathervane into the dive once falling. While still climbing, tip toward world-down.
            Vector3 targetDir = speed > 2f && velocity.Y <= 0f
                ? velocity / speed
                : Vector3.Normalize(new Vector3(nose.X, -1f, nose.Z));
            float align = Vector3.Dot(nose, targetDir);
            if (align > -0.92f)
            {
                Vector3 alignAxis = Vector3.Cross(nose, targetDir);
                if (alignAxis.LengthSquared() > 1e-8f)
                {
                    // Ease weathercock as we approach the target to avoid flip-through.
                    float catchUp = Math.Clamp(1f - align, 0f, 1.5f);
                    Vector3 bodyAxis = Vector3.Transform(alignAxis, Quaternion.Inverse(state.Rotation));
                    bodyTorqueFromForces += bodyAxis * (_props.StallWeathercockGain * stallSeverity * catchUp);
                }
            }

            bodyTorqueFromForces += -state.AngularVelocity * (_props.StallAngularDamping * stallSeverity);
        }

        Vector3 linearAccel = worldForce / _props.MassKg;
        Vector3 newVelocity = velocity + linearAccel * dt;

        // Path follows the nose only while flying; in a stall, gravity owns the trajectory.
        if (speed > 1f && stallFactor > 0.2f)
        {
            Vector3 velDir = Vector3.Normalize(newVelocity);
            Vector3 blended = Vector3.Normalize(Vector3.Lerp(velDir, nose, 0.55f * effectiveness));
            newVelocity = blended * newVelocity.Length();
        }

        // Body rates: +elevator => nose up => negative omega.X in right-handed Z-forward frame.
        // +aileron => roll right => negative omega.Z. +rudder => yaw right => positive omega.Y.
        Vector3 desiredBodyOmega = new(
            -fci.Elevator * _props.MaxPitchRate * effectiveness,
            fci.Rudder * _props.MaxYawRate * effectiveness,
            -fci.Aileron * _props.MaxRollRate * effectiveness);

        Vector3 inertia = _props.InertiaTensor;
        Vector3 torqueAlpha = new(
            bodyTorqueFromForces.X / Math.Max(inertia.X, 1f),
            bodyTorqueFromForces.Y / Math.Max(inertia.Y, 1f),
            bodyTorqueFromForces.Z / Math.Max(inertia.Z, 1f));

        float rateBlend = 1f - MathF.Exp(-AngularResponse * dt);
        // Deep stall: let recovery torques dominate over stick commands.
        float controlAuthority = Math.Clamp(effectiveness, 0.05f, 1f);
        Vector3 newBodyOmega = Vector3.Lerp(state.AngularVelocity, desiredBodyOmega, rateBlend * controlAuthority);
        newBodyOmega += torqueAlpha * dt;
        newBodyOmega *= MathF.Exp(-0.15f * dt);

        Quaternion rotationDelta = IntegrateAngularVelocity(newBodyOmega, dt);
        Quaternion newRotation = Quaternion.Normalize(state.Rotation * rotationDelta);
        Vector3 newPosition = state.Position + newVelocity * dt;

        return new FlightState(newPosition, newRotation, newVelocity, newBodyOmega);
    }

    /// <summary>True when airspeed is below the attitude-dependent stall threshold.</summary>
    public bool IsStalled(in FlightState state)
    {
        float pitch = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));
        float stallKmh = ControlEffectiveness.StallSpeedKmh(_props, pitch);
        return Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond) < stallKmh;
    }

    public float StallSpeedKmh(in FlightState state)
    {
        float pitch = MathF.Asin(Math.Clamp(state.NoseVector.Y, -1f, 1f));
        return ControlEffectiveness.StallSpeedKmh(_props, pitch);
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
        float stallFactor,
        float stallSeverity,
        ref int count)
    {
        if (speed < 0.5f)
        {
            return;
        }

        Vector3 velDir = velocity / speed;
        float dynamicPressure = 0.5f * _props.AirDensity * speed * speed;

        float aoa = SignedAngle(velDir, nose, state.RightVector);
        float cl = Math.Clamp(_props.LiftSlope * aoa, -_props.MaxLiftCoefficient, _props.MaxLiftCoefficient);

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

        float liftScale = _props.StallLiftRetention + (1f - _props.StallLiftRetention) * stallFactor;
        float liftMag = dynamicPressure * _props.WingArea * MathF.Abs(cl) * liftScale;
        if (cl < 0f)
        {
            liftDir = -liftDir;
        }

        // Lift acts at the aero reference (origin). With a forward CoG, lost lift => nose drop.
        _scratch[count++] = ForceVector.World(liftDir * liftMag, Vector3.Zero);

        float cd = _props.ParasiteDragCoefficient + _props.InducedDragFactor * cl * cl;
        cd *= 1f + stallSeverity * 1.1f;
        float dragMag = dynamicPressure * _props.WingArea * cd;
        _scratch[count++] = ForceVector.World(-velDir * dragMag, Vector3.Zero);

        Vector3 side = state.RightVector;
        float sideSlip = Vector3.Dot(velDir, side);
        _scratch[count++] = ForceVector.World(-side * (sideSlip * dynamicPressure * _props.WingArea * 0.4f), Vector3.Zero);
    }

    private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
    {
        Vector3 fromN = Vector3.Normalize(from);
        Vector3 toN = Vector3.Normalize(to);
        float sin = Vector3.Dot(axis, Vector3.Cross(fromN, toN));
        float cos = Vector3.Dot(fromN, toN);
        return MathF.Atan2(sin, cos);
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
