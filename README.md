# FlightEngine

Headless C# flight physics for arcade-style War Thunder–like flying. The library owns simulation truth; a separate front end (Unity, a custom client, or the bundled visualizer) owns input and presentation.

**Requires:** [.NET 9 SDK](https://dotnet.microsoft.com/download)

---

## Headless mode (front end drives the engine)

`FlightEngine` is a class library, not a windowed app. There is no built-in input loop. Your front end (for example a Unity game) builds an **FCI** each frame, ticks the simulator, and applies the returned **flight-state** to visuals / networking.

### Boundary contract

Each tick:

1. Front end sends the last **flight-state** and the current **FCI** (aileron, elevator, rudder in `[-1, 1]`).
2. Backend (`FlightSimulator`) returns a new **flight-state** (position, rotation, linear velocity, angular velocity).

Only those cardinal values cross the boundary. Derived quantities (altitude = `Position.Y`, speed = `|LinearVelocity|`, nose vector, etc.) stay on the front end.

Throttle is **not** part of FCI — set `FlightSimulator.Throttle` on the simulator instance.

Optional: pass external `ForceVector`s into `Tick` for impacts or other novel forces.

### Reference the library

From your front-end project (Unity plugin, server, or another C# app):

```xml
<ItemGroup>
  <ProjectReference Include="path\to\FlightEngine\FlightEngine.csproj" />
</ItemGroup>
```

Or build and reference the DLL:

```bash
dotnet build FlightEngine.csproj -c Release
# → bin/Release/net9.0/FlightEngine.dll
```

### Minimal tick loop

```csharp
using System.Numerics;
using FlightEngine.Aircraft;
using FlightEngine.Core;
using FlightEngine.Physics;

var props = DefaultAircraft.CreateProperties(); // or AircraftRoster / custom FlightProperties
var sim = new FlightSimulator(props) { Throttle = 0.75f };

FlightState state = DefaultAircraft.CreateLevelFlight(speedKmh: 320f, altitudeMeters: 800f);

// Each frame / fixed timestep from the front end:
float dt = /* your delta time in seconds */;
Fci fci = new Fci(aileron, elevator, rudder); // from keys, gamepad, or network

state = sim.Tick(state, fci, dt);

// Present state.Position / state.Rotation (and velocities) in Unity or your renderer.
// Altitude meters: state.AltitudeMeters  |  Speed m/s: state.SpeedMetersPerSecond
```

### Control helpers (optional)

| Helper | Role |
|--------|------|
| `ManualPathHoldController` | Manual stick: elevators trim to hold the last commanded nose-vector when the stick is released. |
| `VirtualPilotController` | VPC: turn a world-space flight-cursor point into an FCI that steers toward it. |

Example VPC tick:

```csharp
using FlightEngine.Control;

var vpc = new VirtualPilotController(props);
Vector3 flightCursor = /* aim point in world space */;
Fci fci = vpc.ComputeFci(state, flightCursor);
state = sim.Tick(state, fci, dt);
```

### Units

- Position / altitude: **meters** (Y up)
- Linear velocity: **m/s**
- Angular velocity: **rad/s** (body space)
- Display speed helpers: `FlightEngine.Units.Speed` (`KmhToMetersPerSecond` / `MetersPerSecondToKmh`)

See `hDocs/boundry.md` for the full front/back policy.

---

## Visualizer

A Raylib window that runs the same headless simulator with keyboard / mouse as the front end. Useful for tuning feel without Unity.

### Run

From the `FlightEngine` folder:

```bash
dotnet run --project FlightEngine.Visualizer
```

Or open `FlightEngine.sln` and run the **FlightEngine.Visualizer** project.

### Controls

**Manual FCI** (default)

| Input | Action |
|-------|--------|
| W/S or ↑/↓ | Pitch (release holds nose) |
| A/D or ←/→ | Aileron (roll) |
| Q / E | Rudder (yaw) |
| Shift | Afterburner (5× thrust) |
| = / Ctrl | Cruise throttle up / down |
| V | Toggle VPC mode |
| P | Next aircraft preset |
| Y | Kill a random engine |
| R | Reset flight |
| C | Cycle chase-camera distance |
| Space / T | Debug force / huge kick |
| Mouse | Orbit chase camera |

**VPC mode** (press `V`)

| Input | Action |
|-------|--------|
| Mouse | Aim flight-cursor ahead of the plane |
| A / D | Manual roll override |
| V | Exit VPC |

HUD shows speed (km/h), altitude (m), throttle, stall state, and current FCI.

---

## Tests

```bash
dotnet test FlightEngine.Tests
```
