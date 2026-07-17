using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Aircraft;

/// <summary>
/// Demo fleet: roll / turn / speed / compression contrasts plus a twin-engine airframe.
/// Compression onset is <see cref="FlightProperties.ReferenceSpeedKmh"/> — authority peaks there,
/// then fades as dynamic pressure climbs past design speed.
/// </summary>
public static class AircraftRoster
{
    private static readonly AircraftDefinition[] Roster =
    [
        Create(
            id: "baseline",
            name: "Standard Fighter",
            trait: "balanced baseline",
            visual: PlaneVisualStyle.Baseline,
            mutate: null),
        Create(
            id: "good-roll",
            name: "Snap Roller",
            trait: "good roll rate",
            visual: PlaneVisualStyle.FastRoll,
            mutate: p => p with { MaxRollRate = p.MaxRollRate * 2.4f, InertiaTensor = p.InertiaTensor * new Vector3(0.85f, 0.9f, 0.55f) }),
        Create(
            id: "bad-roll",
            name: "Heavy Wings",
            trait: "bad roll rate",
            visual: PlaneVisualStyle.SlowRoll,
            mutate: p => p with { MaxRollRate = p.MaxRollRate * 0.28f, InertiaTensor = p.InertiaTensor * new Vector3(1.1f, 1.05f, 1.9f) }),
        Create(
            id: "good-turn",
            name: "Corner Fighter",
            trait: "good turn rate",
            visual: PlaneVisualStyle.TightTurn,
            mutate: p => p with
            {
                MaxPitchRate = p.MaxPitchRate * 1.85f,
                MaxYawRate = p.MaxYawRate * 1.55f,
                MaxLiftCoefficient = p.MaxLiftCoefficient * 1.15f,
                InducedDragFactor = p.InducedDragFactor * 0.85f
            }),
        Create(
            id: "bad-turn",
            name: "Boom Cruiser",
            trait: "bad turn rate",
            visual: PlaneVisualStyle.WideTurn,
            mutate: p => p with
            {
                MaxPitchRate = p.MaxPitchRate * 0.38f,
                MaxYawRate = p.MaxYawRate * 0.45f,
                MaxLiftCoefficient = p.MaxLiftCoefficient * 0.85f,
                MassKg = p.MassKg * 1.25f,
                InertiaTensor = p.InertiaTensor * 1.35f
            }),
        Create(
            id: "fast",
            name: "Needle Jet",
            trait: "fast cruise",
            visual: PlaneVisualStyle.FastCruise,
            mutate: MakeFastCruise),
        Create(
            id: "slow",
            name: "Sky Barge",
            trait: "slow cruise",
            visual: PlaneVisualStyle.SlowCruise,
            mutate: MakeSlowCruise),
        Create(
            id: "late-comp",
            name: "Hard Stick",
            trait: "late compression",
            visual: PlaneVisualStyle.LateCompression,
            mutate: p => p with { ReferenceSpeedKmh = 720f }),
        Create(
            id: "early-comp",
            name: "Soft Stick",
            trait: "early compression",
            visual: PlaneVisualStyle.EarlyCompression,
            mutate: p => p with { ReferenceSpeedKmh = 240f }),
        Create(
            id: "twin",
            name: "Twin Boom",
            trait: "two engines (Y kills one)",
            visual: PlaneVisualStyle.TwinEngine,
            mutate: MakeTwinEngine)
    ];

    public static IReadOnlyList<AircraftDefinition> All => Roster;

    public static AircraftDefinition ByIndex(int index)
    {
        int i = ((index % Roster.Length) + Roster.Length) % Roster.Length;
        return Roster[i];
    }

    /// <summary>Copy of properties with offline engines zeroed for VPC bias estimation.</summary>
    public static FlightProperties WithEngineMask(FlightProperties source, ReadOnlySpan<bool> engineOnline)
    {
        EngineThrust[] engines = new EngineThrust[source.Engines.Length];
        for (int i = 0; i < engines.Length; i++)
        {
            EngineThrust e = source.Engines[i];
            bool online = i < engineOnline.Length && engineOnline[i];
            engines[i] = online
                ? e
                : new EngineThrust(0f, e.LocalDirection, e.LocalApplicationPoint);
        }

        return Clone(source, engines);
    }

    private static AircraftDefinition Create(
        string id,
        string name,
        string trait,
        PlaneVisualStyle visual,
        Func<FlightProperties, FlightProperties>? mutate)
    {
        FlightProperties baseProps = DefaultAircraft.CreateProperties();
        FlightProperties props = mutate is null ? baseProps : mutate(Clone(baseProps, baseProps.Engines));
        return new AircraftDefinition
        {
            Id = id,
            DisplayName = name,
            Trait = trait,
            Properties = props,
            Visual = visual
        };
    }

    /// <summary>High thrust, slick airframe — builds and holds speed easily.</summary>
    private static FlightProperties MakeFastCruise(FlightProperties p) =>
        ScaleThrust(p, 1.75f) with
        {
            ParasiteDragCoefficient = p.ParasiteDragCoefficient * 0.55f,
            MassKg = p.MassKg * 0.92f,
            WingArea = p.WingArea * 0.9f,
            InducedDragFactor = p.InducedDragFactor * 1.1f,
            MaxLiftCoefficient = p.MaxLiftCoefficient * 0.92f
        };

    /// <summary>Weak thrust, draggy airframe — lives at lower cruise speeds.</summary>
    private static FlightProperties MakeSlowCruise(FlightProperties p) =>
        ScaleThrust(p, 0.55f) with
        {
            ParasiteDragCoefficient = p.ParasiteDragCoefficient * 1.85f,
            MassKg = p.MassKg * 1.15f,
            WingArea = p.WingArea * 1.25f,
            InducedDragFactor = p.InducedDragFactor * 0.9f,
            MaxLiftCoefficient = p.MaxLiftCoefficient * 1.1f,
            MinimumControlSpeedKmh = p.MinimumControlSpeedKmh * 0.85f
        };

    private static FlightProperties MakeTwinEngine(FlightProperties p)
    {
        float half = p.Engines[0].MaxThrustNewtons * 0.5f;
        return p with
        {
            Engines =
            [
                new EngineThrust(half, Vector3.UnitZ, new Vector3(-3.2f, 0f, 0.4f)),
                new EngineThrust(half, Vector3.UnitZ, new Vector3(3.2f, 0f, 0.4f))
            ],
            MassKg = p.MassKg * 1.1f,
            InertiaTensor = p.InertiaTensor * new Vector3(1.15f, 1.25f, 1.1f)
        };
    }

    private static FlightProperties ScaleThrust(FlightProperties p, float factor)
    {
        EngineThrust[] engines = new EngineThrust[p.Engines.Length];
        for (int i = 0; i < engines.Length; i++)
        {
            EngineThrust e = p.Engines[i];
            engines[i] = new EngineThrust(e.MaxThrustNewtons * factor, e.LocalDirection, e.LocalApplicationPoint);
        }

        return p with { Engines = engines };
    }

    private static FlightProperties Clone(FlightProperties source, EngineThrust[] engines) =>
        new()
        {
            MassKg = source.MassKg,
            InertiaTensor = source.InertiaTensor,
            Engines = engines,
            Geometry = source.Geometry,
            WingArea = source.WingArea,
            LiftSlope = source.LiftSlope,
            MaxLiftCoefficient = source.MaxLiftCoefficient,
            ParasiteDragCoefficient = source.ParasiteDragCoefficient,
            InducedDragFactor = source.InducedDragFactor,
            StallAoAWidth = source.StallAoAWidth,
            StallLiftRetention = source.StallLiftRetention,
            PostStallControlRetention = source.PostStallControlRetention,
            ReferenceSpeedKmh = source.ReferenceSpeedKmh,
            MinimumControlSpeedKmh = source.MinimumControlSpeedKmh,
            MaxRollRate = source.MaxRollRate,
            MaxPitchRate = source.MaxPitchRate,
            MaxYawRate = source.MaxYawRate,
            CenterOfGravityLocal = source.CenterOfGravityLocal,
            AeroCenterLocal = source.AeroCenterLocal,
            VelocityAlignRate = source.VelocityAlignRate,
            AeroTorqueCoupling = source.AeroTorqueCoupling,
            AirDensity = source.AirDensity,
            Gravity = source.Gravity
        };
}
