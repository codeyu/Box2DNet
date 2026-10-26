// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using Xunit;
using static Box2DNet.B2;

namespace Box2DNet.Tests;

// The program from docs/hello.md
public class HelloWorldTests
{
	[Fact]
	public void BoxFallsOnGround()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );
		WorldId worldId = CreateWorld( worldDef );
		Assert.True( World_IsValid( worldId ) );

		BodyDef groundBodyDef = DefaultBodyDef();
		groundBodyDef.position = new Vec2( 0.0f, -10.0f );
		BodyId groundId = CreateBody( worldId, groundBodyDef );
		Polygon groundBox = MakeBox( 50.0f, 10.0f );
		ShapeDef groundShapeDef = DefaultShapeDef();
		CreatePolygonShape( groundId, groundShapeDef, groundBox );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Vec2( 0.0f, 4.0f );
		BodyId bodyId = CreateBody( worldId, bodyDef );

		Polygon dynamicBox = MakeBox( 1.0f, 1.0f );
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.3f;
		CreatePolygonShape( bodyId, shapeDef, dynamicBox );

		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;

		Vec2 position = Body_GetPosition( bodyId );
		Rot rotation = Body_GetRotation( bodyId );
		for ( int i = 0; i < 90; ++i )
		{
			World_Step( worldId, timeStep, subStepCount );
			position = Body_GetPosition( bodyId );
			rotation = Body_GetRotation( bodyId );
		}

		DestroyWorld( worldId );

		// The box comes to rest on the ground, matching the output in docs/hello.md
		Assert.True( MathF.Abs( position.x ) < 0.01f );
		Assert.True( MathF.Abs( position.y - 1.0f ) < 0.01f );
		Assert.True( MathF.Abs( Rot_GetAngle( rotation ) ) < 0.01f );
	}
}
