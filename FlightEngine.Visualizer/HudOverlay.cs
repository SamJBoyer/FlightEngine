using FlightEngine.Aircraft;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Units;
using Raylib_cs;

namespace FlightEngine.Visualizer;

internal static class HudOverlay
{
    public static void Draw(
        in FlightState state,
        in Fci fci,
        FlightSimulator sim,
        AircraftDefinition aircraft,
        int aircraftIndex,
        int aircraftCount,
        bool vpcMode,
        float chaseDistance)
    {
        float speedKmh = Speed.MetersPerSecondToKmh(state.SpeedMetersPerSecond);
        float stallKmh = sim.StallSpeedKmh(state);
        bool stalled = sim.IsStalled(state);

        Raylib.DrawRectangle(12, 12, 420, 255, new Color(12, 16, 22, 170));
        Raylib.DrawText("FlightEngine Visualizer", 24, 24, 20, new Color(240, 220, 180, 255));

        int y = 54;
        Line(ref y, $"Aircraft   {aircraft.DisplayName}  ({aircraftIndex + 1}/{aircraftCount})");
        Line(ref y, $"Trait      {aircraft.Trait}", new Color(200, 190, 140, 255));
        Line(ref y, $"Speed      {speedKmh,7:0.0} km/h");
        Line(ref y, $"Altitude   {state.AltitudeMeters,7:0.0} m");
        Line(ref y, $"Throttle   {sim.Throttle * 100f,7:0}%");
        Line(ref y, EngineStatus(sim));
        Line(ref y, $"Stall @    {stallKmh,7:0.0} km/h");
        Line(ref y, stalled ? "STATE      STALL" : "STATE      Flying", stalled ? new Color(255, 90, 70, 255) : new Color(120, 220, 140, 255));
        Line(ref y, vpcMode ? "Control    VPC (aim plane)" : "Control    Manual FCI");
        Line(ref y, $"Ail {fci.Aileron,5:0.00}  Elv {fci.Elevator,5:0.00}  Rdr {fci.Rudder,5:0.00}");

        Raylib.DrawRectangle(12, Raylib.GetScreenHeight() - 210, 580, 198, new Color(12, 16, 22, 170));
        int hy = Raylib.GetScreenHeight() - 198;
        if (vpcMode)
        {
            Help(ref hy, "Mouse            aim on plane ahead (world cursor)");
            Help(ref hy, "A / D            manual roll (overrides VPC bank)");
            Help(ref hy, "Shift / Ctrl     throttle up / down");
            Help(ref hy, "P  next plane   Y kill random engine");
            Help(ref hy, "V  exit VPC   R reset   C camera zoom");
            Help(ref hy, "Space / T       debug force / huge kick");
            Help(ref hy, $"Chase distance: {chaseDistance:0} m");
        }
        else
        {
            Help(ref hy, "W/S or Up/Down   elevator (pitch)");
            Help(ref hy, "A/D or Left/Right aileron (roll)");
            Help(ref hy, "Q / E            rudder (yaw)");
            Help(ref hy, "Shift / Ctrl     throttle up / down");
            Help(ref hy, "P  next plane   Y kill random engine");
            Help(ref hy, "V  VPC mode   R reset   C camera zoom");
            Help(ref hy, "Space / T       debug force / huge kick");
            Help(ref hy, $"Chase distance: {chaseDistance:0} m   | green=nose  blue=velocity");
        }
    }

    private static string EngineStatus(FlightSimulator sim)
    {
        int total = sim.Properties.Engines.Length;
        int online = sim.OnlineEngineCount;
        if (total <= 1)
        {
            return online == 1 ? "Engines    1 online" : "Engines    OFFLINE";
        }

        return $"Engines    {online}/{total} online";
    }

    private static void Line(ref int y, string text, Color? color = null)
    {
        Raylib.DrawText(text, 24, y, 18, color ?? new Color(230, 230, 230, 255));
        y += 22;
    }

    private static void Help(ref int y, string text)
    {
        Raylib.DrawText(text, 24, y, 16, new Color(200, 200, 200, 255));
        y += 20;
    }
}
