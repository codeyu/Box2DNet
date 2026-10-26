# Box2DNet

Box2DNet is a C# port of [Box2D](https://box2d.org) v3.2.0, Erin Catto's 2D physics engine, for .NET 10.

It is a function-by-function port of the C sources. The simulation is bit-exact with the C version: the
upstream cross-platform determinism test (FallingHinges, hash `0x465F3DF4`, sleep step 299) passes
with any number of worker threads.

The old v2 port (based on Box2DX) lives on the [`v2-legacy`](../../tree/v2-legacy) branch and in `legacy/`.

## Quick start

```csharp
using Box2DNet;
using static Box2DNet.B2;

WorldDef worldDef = DefaultWorldDef();
worldDef.gravity = new Vec2( 0.0f, -10.0f );
WorldId worldId = CreateWorld( worldDef );

BodyDef groundBodyDef = DefaultBodyDef();
groundBodyDef.position = new Vec2( 0.0f, -10.0f );
BodyId groundId = CreateBody( worldId, groundBodyDef );
CreatePolygonShape( groundId, DefaultShapeDef(), MakeBox( 50.0f, 10.0f ) );

BodyDef bodyDef = DefaultBodyDef();
bodyDef.type = BodyType.dynamicBody;
bodyDef.position = new Vec2( 0.0f, 4.0f );
BodyId bodyId = CreateBody( worldId, bodyDef );

ShapeDef shapeDef = DefaultShapeDef();
shapeDef.density = 1.0f;
shapeDef.material.friction = 0.3f;
CreatePolygonShape( bodyId, shapeDef, MakeBox( 1.0f, 1.0f ) );

for ( int i = 0; i < 90; ++i )
{
	World_Step( worldId, 1.0f / 60.0f, 4 );
	Vec2 position = Body_GetPosition( bodyId );
	Console.WriteLine( $"{position.x:F2} {position.y:F2} {Rot_GetAngle( Body_GetRotation( bodyId ) ):F2}" );
}

DestroyWorld( worldId );
```

## API mapping

The C API is kept one to one, so the [Box2D documentation](https://box2d.org/documentation/) and samples apply directly.

| C | C# |
| --- | --- |
| `b2CreateWorld( &def )` | `B2.CreateWorld( def )` (functions live in `static partial class B2`, add `using static Box2DNet.B2;`) |
| `b2Body_GetPosition( id )` | `B2.Body_GetPosition( id )` |
| `b2Vec2`, `b2BodyDef`, `b2WorldId` | `Vec2`, `BodyDef`, `WorldId` |
| `b2_dynamicBody` | `BodyType.dynamicBody` |
| struct fields `def.enableSleep` | same names (`def.enableSleep`); `base` is `@base` |
| `void* userData` / `void* context` | `object?` |
| callbacks (`b2CastResultFcn*` ...) | delegates |
| `b2Vec2* points, int count` | arrays or `ReadOnlySpan<Vec2>` |
| `b2DebugDraw` | `DebugDraw` with delegate fields |

Multithreading: set `WorldDef.workerCount` to use the built-in scheduler, or provide
`enqueueTask`/`finishTask` to plug in your own task system.

Not ported yet: recording/replay/snapshot, double precision (large worlds), and the 8-wide AVX2
contact solver (the 4-wide `Vector128` solver is used everywhere and gives identical results).

## Repository layout

```
src/Box2DNet/          the library (net10.0)
tests/Box2DNet.Tests/  upstream unit tests and the determinism test (xunit v3)
examples/Testbed/      interactive testbed: GLFW + OpenGL 3.3 + Dear ImGui (Hexa.NET.ImGui, latest cimgui)
doc/                   porting plan
tools/c2cs/            the C to C# conversion scripts used for the port
legacy/                the old v2 port
```

## Build and test

```bash
dotnet build Box2DNet.slnx
```

```bash
dotnet test --project tests/Box2DNet.Tests
```

```bash
dotnet run --project examples/Testbed
```

In the testbed: left drag moves bodies, right drag pans, the mouse wheel zooms, `P` pauses, `O` single steps
and `R` restarts the sample. `--smoke` runs every sample for a few frames and exits.

## License

MIT, same as Box2D.
