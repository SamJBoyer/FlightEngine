using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Control;

/// <summary>
/// Manual pitch helper: elevators always trim to hold a commanded nose-vector.
/// Stick pitch changes that held attitude; release holds it without constant stick pressure.
/// </summary>
public sealed class ManualPathHoldController
{
    private Vector3 _heldNose = Vector3.UnitZ;
    private bool _hasHold;

    /// <summary>How strongly elevators chase the held nose direction.</summary>
    public float ProportionalGain { get; set; } = 3.2f;

    /// <summary>Damps body pitch rate so the hold settles instead of oscillating.</summary>
    public float RateDamping { get; set; } = 0.45f;

    /// <summary>How fast full stick rotates the held nose-vector (rad/s).</summary>
    public float StickPathRate { get; set; } = 0.95f;

    public Vector3 HeldNose => _heldNose;

    public void Reset(in FlightState state)
    {
        _heldNose = state.NoseVector;
        _hasHold = true;
    }

    public void Clear()
    {
        _hasHold = false;
        _heldNose = Vector3.UnitZ;
    }

    /// <summary>
    /// Builds an FCI whose elevator maintains <see cref="HeldNose"/>.
    /// Aileron/rudder pass through; elevator stick retargets the held nose.
    /// </summary>
    public Fci ComputeFci(
        in FlightState state,
        float aileron,
        float elevatorStick,
        float rudder,
        float deltaTime)
    {
        if (!_hasHold)
        {
            _heldNose = state.NoseVector;
            _hasHold = true;
        }

        float stick = Math.Clamp(elevatorStick, -1f, 1f);
        float dt = Math.Max(0f, deltaTime);

        // Stick commands a change to the held nose attitude (around the wing axis).
        if (MathF.Abs(stick) > 0.01f && dt > 0f)
        {
            float pitchDelta = stick * StickPathRate * dt;
            // +X right: positive angle pitches nose down; invert so +stick pitches up.
            Quaternion delta = Quaternion.CreateFromAxisAngle(state.RightVector, -pitchDelta);
            _heldNose = Vector3.Normalize(Vector3.Transform(state.NoseVector, delta));
        }

        float elevator = ElevatorToHold(state, _heldNose);

        // While commanding, blend stick for snappy pitch response on top of the hold.
        if (MathF.Abs(stick) > 0.01f)
        {
            elevator = Math.Clamp(elevator * 0.4f + stick * 0.75f, -1f, 1f);
        }

        return new Fci(aileron, elevator, rudder);
    }

    private float ElevatorToHold(in FlightState state, Vector3 heldWorld)
    {
        Vector3 bodyDesired = Vector3.Transform(heldWorld, Quaternion.Inverse(state.Rotation));
        // Same convention as VPC: +body Y demand → +elevator → nose up.
        float elevator = bodyDesired.Y * ProportionalGain;
        // AngularVelocity.X > 0 is nose-down; damp so hold doesn't overshoot.
        elevator -= state.AngularVelocity.X * RateDamping;
        return Math.Clamp(elevator, -1f, 1f);
    }
}
