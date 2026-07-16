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
    private readonly bool[] _engineOnline;

    public FlightSimulator(FlightProperties properties, int maxExternalForces = 16)
    {
        _props = properties ?? throw new ArgumentNullException(nameof(properties));
        _scratch = new ForceVector[properties.Engines.Length + maxExternalForces + 4];
        _engineOnline = new bool[properties.Engines.Length];
        Array.Fill(_engineOnline, true);
        Throttle = 1f;
    }

    public FlightProperties Properties => _props;

    /// <summary>Per-engine run state. Offline engines produce no thrust.</summary>
    public ReadOnlySpan<bool> EngineOnline => _engineOnline;

    public int OnlineEngineCount
    {
        get
        {
            int n = 0;
            for (int i = 0; i < _engineOnline.Length; i++)
            {
                if (_engineOnline[i])
                {
                    n++;
                }
            }

            return n;
        }
    }

    /// <summary>
    /// Engine throttle. 1 = rated thrust; values above 1 are overthrust / afterburner.
    /// Not part of FCI (control surfaces only).
    /// </summary>
    public float Throttle { get; set; }

    /// <summary>How quickly body rates track control-commanded rates (1/s).</summary>
    public float AngularResponse { get; set; } = 10f;

    /// <summary>
    /// Disables a random online engine. Returns the engine index, or -1 if none were online.
    /// </summary>
    public int KillRandomEngine()
    {
        int online = OnlineEngineCount;
        if (online == 0)
        {
            return -1;
        }

        int pick = Random.Shared.Next(online);
        for (int i = 0; i < _engineOnline.Length; i++)
        {
            if (!_engineOnline[i])
            {
                continue;
            }

            if (pick == 0)
            {
                _engineOnline[i] = false;
                return i;
            }

            pick--;
        }

        return -1;
    }

    public void RestoreEngines() => Array.Fill(_engineOnline, true);

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
        CollectAeroForces(state, velocity, speed, aoa, stallSeverity, ref count);
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

        // Control accel fights physical moments (forward CoG, aero, impacts).
        float controlStrength = AngularResponse * Math.Clamp(authority + 0.02f, 0.02f, 1f);
        Vector3 controlAlpha = (desiredBodyOmega - state.AngularVelocity) * controlStrength;
        Vector3 newBodyOmega = state.AngularVelocity + (controlAlpha + torqueAlpha) * dt;
        newBodyOmega *= MathF.Exp(-0.12f * dt);

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

    private void CollectEngineForces(ref int count)
    {
        float throttle = Math.Max(0f, Throttle);
        for (int i = 0; i < _props.Engines.Length; i++)
        {
            if (!_engineOnline[i])
            {
                continue;
            }

            _scratch[count++] = _props.Engines[i].ToForceVector(throttle);
        }
    }

    private void CollectGravity(ref int count)
    {
        // Weight at the forward CoG — nose-down moment appears naturally when wing lift fades.
        _scratch[count++] = ForceVector.World(
            new Vector3(0f, -_props.MassKg * _props.Gravity, 0f),
            _props.CenterOfGravityLocal);
    }

    private void CollectAeroForces(
        in FlightState state,
        Vector3 velocity,
        float speed,
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
        Vector3 aeroPoint = _props.AeroCenterLocal;

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

        _scratch[count++] = ForceVector.World(liftDir * liftMag, aeroPoint);

        float cd = _props.ParasiteDragCoefficient + _props.InducedDragFactor * cl * cl;
        cd += stallSeverity * 0.9f;
        float dragMag = q * _props.WingArea * cd;
        _scratch[count++] = ForceVector.World(-velDir * dragMag, aeroPoint);

        Vector3 side = state.RightVector;
        float sideSlip = Vector3.Dot(velDir, side);
        _scratch[count++] = ForceVector.World(-side * (sideSlip * q * _props.WingArea * 0.4f), aeroPoint);
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
