// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using Xunit;
using static Box2DNet.B2;

namespace Box2DNet.Tests;

// Regression tests for bugs found while porting the samples
public class WorldQueryTests
{
	private sealed class Counter
	{
		public int count;
	}

	// World_OverlapShape used to copy the proxy into an unset ref field
	[Fact]
	public void OverlapShapeFindsShape()
	{
		WorldId worldId = CreateWorld( DefaultWorldDef() );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Vec2( 2.0f, 0.0f );
		BodyId bodyId = CreateBody( worldId, bodyDef );
		CreatePolygonShape( bodyId, DefaultShapeDef(), MakeBox( 1.0f, 1.0f ) );

		Vec2 center = Vec2_zero;
		ShapeProxy proxy = MakeProxy( in center, 1, 1.5f );

		Counter counter = new();
		World_OverlapShape( worldId, Vec2_zero, proxy, DefaultQueryFilter(), ( shapeId, context ) =>
		{
			( (Counter)context! ).count += 1;
			return true;
		}, counter );

		Assert.Equal( 1, counter.count );

		// A query away from the box finds nothing
		counter.count = 0;
		World_OverlapShape( worldId, new Vec2( -5.0f, 0.0f ), proxy, DefaultQueryFilter(), ( shapeId, context ) =>
		{
			( (Counter)context! ).count += 1;
			return true;
		}, counter );

		Assert.Equal( 0, counter.count );

		DestroyWorld( worldId );
	}
}
