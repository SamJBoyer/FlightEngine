# FlightState

**FlightState** is the authoritative **6-DOF physics snapshot** for one aircraft at one simulation instant. A flight engine reads the previous **FlightState**, applies forces and control input, integrates, and writes an updated **FlightState**. It is **not** player input.

See [terms.md](../terms.md), [fci.md](./fci.md) (commands), and [flight-system.md](./flight-system.md).

---

## Purpose

- Hold the integrated pose and motion the sim actually uses each **60 Hz** tick.
- Decouple **physics result** from **control requests** (FCI).
- Give clients a single authoritative snapshot for display and reconciliation.

---

## Struct (prototype C#)

Implementation: `[Assets/Scripts/FlightState.cs](../../Assets/Scripts/FlightState.cs)`.


| Field             | Type         | Description                           |
| ----------------- | ------------ | ------------------------------------- |
| `Position`        | `Vector3`    | World-space position (m).             |
| `Rotation`        | `Quaternion` | World-space body attitude.            |
| `Velocity`        | `Vector3`    | World-space linear velocity (m/s).    |
| `AngularVelocity` | `Vector3`    | World-space angular velocity (rad/s). |


### Factory


| Method                                                | Use                                                                    |
| ----------------------------------------------------- | ---------------------------------------------------------------------- |
| `FromTransform(transform, velocity, angularVelocity)` | Seed an engine from a scene `Transform` at spawn or reset (e.g. TestFlight). |


## Lifecycle

```text
FlightState_prev + control input + forces → FlightEngine.Simulate → FlightState_next
```

- **Authoritative:** the server flight engine owns `FlightState` for every aircraft in a match.
- **Prototype:** the selected physics engine owns integration and drives the aircraft transform.
- **Optional predict:** a client engine may integrate locally with the same control input; reconcile to server **FlightState** when it arrives.

---

## Network (production, Alpha)


| Direction        | Payload                                        |
| ---------------- | ---------------------------------------------- |
| Client → server  | **FCI** only (see [fci.md](./fci.md))          |
| Server → clients | **FlightState** fields (+ damage mask, events) |


Wire names and types (target): [FlightState.json](../TestFlight/formats/FlightState.json) — `position`, `rotation` (quaternion), `velocity`, `angular_velocity`.

---

## See also


| Document                     | Focus                                                    |
| ---------------------------- | -------------------------------------------------------- |
| [fci.md](./fci.md)           | Client control commands                                  |
| [flight-system.md](./flight-system.md) | Flight-engine boundaries and responsibilities |


