using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Units;

namespace FlightEngine.Physics;

/// <summary>
/// Dynamic-pressure / angle-of-attack aerodynamics. Stall and control fade emerge from
/// the CL polar and q, not from authored speed gates.
/// </summary>
internal static class Aerodynamics
{
    public static float DynamicPressure(FlightProperties props, float speedMetersPerSecond) =>
        0.5f * props.AirDensity * speedMetersPerSecond * speedMetersPerSecond;

    public static float ReferenceDynamicPressure(FlightProperties props)
    {
        float v = Speed.KmhToMetersPerSecond(props.ReferenceSpeedKmh);
        return DynamicPressure(props, v);
    }

    /// <summary>Critical AoA where CL peaks (radians).</summary>
    public static float CriticalAoA(FlightProperties props) =>
        props.MaxLiftCoefficient / Math.Max(props.LiftSlope, 0.1f);

    /// <summary>
    /// CL polar: linear up to critical AoA, then drops toward a deep-stall remnant.
    /// </summary>
    public static float LiftCoefficient(float aoaRadians, FlightProperties props)
    {
        float aCrit = CriticalAoA(props);
        float abs = MathF.Abs(aoaRadians);
        float sign = MathF.Sign(aoaRadians);
        if (sign == 0f)
        {
            return 0f;
        }

        if (abs <= aCrit)
        {
            return props.LiftSlope * aoaRadians;
        }

        float t = Math.Clamp((abs - aCrit) / Math.Max(props.StallAoAWidth, 1e-3f), 0f, 1f);
        t = t * t * (3f - 2f * t);
        float clDeep = props.StallLiftRetention * props.MaxLiftCoefficient;
        return sign * Lerp(props.MaxLiftCoefficient, clDeep, t);
    }

    /// <summary>
    /// Attached-flow factor for control surfaces (1 = clean, low = separated / stalled).
    /// </summary>
    public static float FlowAttachment(float aoaRadians, FlightProperties props)
    {
        float aCrit = CriticalAoA(props);
        float abs = MathF.Abs(aoaRadians);
        if (abs <= aCrit)
        {
            return 1f;
        }

        float t = Math.Clamp((abs - aCrit) / Math.Max(props.StallAoAWidth, 1e-3f), 0f, 1f);
        return Lerp(1f, props.PostStallControlRetention, t);
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    /// <summary>
    /// Control authority from dynamic pressure. Peaks near reference speed, fades at low q
    /// (natural soft controls) and at high q (compression / hinge stiffening).
    /// Shape: 2x/(1+x²) with x = q/q_ref — max of 1 at x = 1.
    /// </summary>
    public static float DynamicPressureAuthority(FlightProperties props, float speedMetersPerSecond)
    {
        float q = DynamicPressure(props, speedMetersPerSecond);
        float qRef = Math.Max(ReferenceDynamicPressure(props), 1f);
        float x = q / qRef;
        return 2f * x / (1f + x * x);
    }

    public static float ControlAuthority(FlightProperties props, float speedMetersPerSecond, float aoaRadians) =>
        DynamicPressureAuthority(props, speedMetersPerSecond) * FlowAttachment(aoaRadians, props);

    /// <summary>
    /// Level-flight stall speed from lift = weight at CLmax.
    /// </summary>
    public static float LevelStallSpeedKmh(FlightProperties props)
    {
        float weight = props.MassKg * props.Gravity;
        float denom = props.AirDensity * props.WingArea * props.MaxLiftCoefficient;
        if (denom <= 1e-6f)
        {
            return 0f;
        }

        float vs = MathF.Sqrt(2f * weight / denom);
        return Speed.MetersPerSecondToKmh(vs);
    }

    /// <summary>
    /// Attitude-dependent stall speed: level Vs supports weight, blending toward minimum
    /// control speed as the nose goes vertical (cos-load → 0).
    /// </summary>
    public static float StallSpeedKmh(FlightProperties props, float pitchRadians)
    {
        float vsLevel = LevelStallSpeedKmh(props);
        float vsMin = props.MinimumControlSpeedKmh;
        float c = MathF.Cos(pitchRadians);
        float s = MathF.Sin(pitchRadians);
        return MathF.Sqrt(vsLevel * vsLevel * c * c + vsMin * vsMin * s * s);
    }

    public static float AngleOfAttack(in FlightState state, FlightProperties props)
    {
        float speed = state.SpeedMetersPerSecond;
        if (speed < 0.5f)
        {
            return CriticalAoA(props) * 2f;
        }

        Vector3 velDir = state.LinearVelocity / speed;
        return SignedAngle(velDir, state.NoseVector, state.RightVector);
    }

    /// <summary>
    /// How deeply stalled we are from AoA separation and/or vanishing dynamic pressure.
    /// </summary>
    public static float StallSeverity(FlightProperties props, float speedMetersPerSecond, float aoaRadians)
    {
        float aCrit = CriticalAoA(props);
        float aoaSeverity = 0f;
        float abs = MathF.Abs(aoaRadians);
        if (abs > aCrit)
        {
            aoaSeverity = Math.Clamp((abs - aCrit) / Math.Max(props.StallAoAWidth, 1e-3f), 0f, 1f);
        }

        float qAuth = DynamicPressureAuthority(props, speedMetersPerSecond);
        float qSeverity = 1f - Math.Clamp(qAuth / 0.35f, 0f, 1f);

        return Math.Max(aoaSeverity, qSeverity);
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
