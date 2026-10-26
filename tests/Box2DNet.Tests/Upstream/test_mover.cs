// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class MoverTests
{
	private static CollisionPlane MakePlane( Vec2 normal, float pushLimit )
	{
		return new CollisionPlane { plane = new() { normal = normal, offset = 0.0f }, pushLimit = pushLimit, push = 0.0f, clipVelocity = true };
	}

	private static int NoPlanesTest()
	{
		Vec2 targetDelta = new Vec2( 0.5f, -0.25f );
		PlaneSolverResult result = SolvePlanes( targetDelta, null, 0 );

		ENSURE( result.delta.x == targetDelta.x );
		ENSURE( result.delta.y == targetDelta.y );
		ENSURE( result.iterationCount == 0 );

		return 0;
	}

	private static int SolvePlaneTest()
	{
		CollisionPlane plane = MakePlane( new Vec2( 0.0f, 1.0f ), FLT_MAX );
		PlaneSolverResult result = SolvePlanes( new Vec2( 0.0f, -1.0f ), new Span<CollisionPlane>( ref plane ), 1 );

		// An overshoot would eject the mover out the far side of the floor
		ENSURE( result.delta.y <= 0.0f );

		// The mover comes to rest a slop under the surface, not through it
		ENSURE( result.delta.y > -10.0f * LinearSlop );
		ENSURE( result.delta.x == 0.0f );
		ENSURE( plane.push > 0.0f );

		return 0;
	}

	private static int PushLimitTest()
	{
		CollisionPlane plane = MakePlane( new Vec2( 0.0f, 1.0f ), 0.1f );
		PlaneSolverResult result = SolvePlanes( new Vec2( 0.0f, -1.0f ), new Span<CollisionPlane>( ref plane ), 1 );

		// Soft collision, so the mover sinks once the limit is spent
		ENSURE_SMALL( plane.push - 0.1f, FLT_EPSILON );
		ENSURE_SMALL( result.delta.y + 0.9f, FLT_EPSILON );

		return 0;
	}

	private static int CornerTest()
	{
		CollisionPlane[] planes = { MakePlane( new Vec2( 0.0f, 1.0f ), FLT_MAX ),
			MakePlane( new Vec2( 1.0f, 0.0f ), FLT_MAX ), };

		PlaneSolverResult result = SolvePlanes( new Vec2( -1.0f, -1.0f ), planes, planes.Length );

		// Both planes hold at once, rather than the second relaxing the first
		for ( int i = 0; i < planes.Length; ++i )
		{
			float separation = PlaneSeparation( planes[i].plane, result.delta );
			ENSURE_SMALL( separation + LinearSlop, LinearSlop );
			ENSURE( planes[i].push > 0.0f );
		}

		return 0;
	}

	private static int WedgeTest()
	{
		// An acute wedge takes several passes to settle, unlike a square corner
		CollisionPlane[] planes = { MakePlane( new Vec2( 0.0f, 1.0f ), FLT_MAX ),
			MakePlane( new Vec2( 0.8f, 0.6f ), FLT_MAX ), };

		PlaneSolverResult result = SolvePlanes( new Vec2( -1.0f, -1.0f ), planes, planes.Length );

		// Settling means resting on both faces. Stopping early leaves the mover shoved
		// clear of one of them, which reads as a large positive separation
		for ( int i = 0; i < planes.Length; ++i )
		{
			float separation = PlaneSeparation( planes[i].plane, result.delta );
			ENSURE_SMALL( separation + LinearSlop, LinearSlop );
		}

		return 0;
	}

	private static int ClipVectorTest()
	{
		CollisionPlane plane = MakePlane( new Vec2( 0.0f, 1.0f ), FLT_MAX );
		plane.push = 0.5f;

		Vec2 v = ClipVector( new Vec2( 1.0f, -1.0f ), new ReadOnlySpan<CollisionPlane>( in plane ), 1 );
		ENSURE_SMALL( v.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( v.y, FLT_EPSILON );

		// Separating velocity passes through untouched
		v = ClipVector( new Vec2( 1.0f, 1.0f ), new ReadOnlySpan<CollisionPlane>( in plane ), 1 );
		ENSURE_SMALL( v.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( v.y - 1.0f, FLT_EPSILON );

		plane.clipVelocity = false;
		v = ClipVector( new Vec2( 1.0f, -1.0f ), new ReadOnlySpan<CollisionPlane>( in plane ), 1 );
		ENSURE_SMALL( v.y + 1.0f, FLT_EPSILON );

		// Zero push means the solver never touched this plane
		plane.clipVelocity = true;
		plane.push = 0.0f;
		v = ClipVector( new Vec2( 1.0f, -1.0f ), new ReadOnlySpan<CollisionPlane>( in plane ), 1 );
		ENSURE_SMALL( v.y + 1.0f, FLT_EPSILON );

		return 0;
	}

	internal static int MoverTest()
	{
		RUN_SUBTEST( NoPlanesTest );
		RUN_SUBTEST( SolvePlaneTest );
		RUN_SUBTEST( PushLimitTest );
		RUN_SUBTEST( CornerTest );
		RUN_SUBTEST( WedgeTest );
		RUN_SUBTEST( ClipVectorTest );

		return 0;
	}

}
