namespace FlightEngine.Core;

/// <summary>
/// Flight controller input: normalized control-surface intent in [-1, 1].
/// </summary>
public readonly struct Fci
{
    public Fci(float aileron, float elevator, float rudder)
    {
        Aileron = Math.Clamp(aileron, -1f, 1f);
        Elevator = Math.Clamp(elevator, -1f, 1f);
        Rudder = Math.Clamp(rudder, -1f, 1f);
    }

    /// <summary>Roll control via ailerons.</summary>
    public float Aileron { get; }

    /// <summary>Pitch control via elevators.</summary>
    public float Elevator { get; }

    /// <summary>Yaw control via rudder.</summary>
    public float Rudder { get; }

    public static Fci Neutral => new(0f, 0f, 0f);
}
