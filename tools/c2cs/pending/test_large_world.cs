// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class LargeWorldTests
{
	// Read a body center of mass relative to a base position, in double precision mode. The public
	// getters demote to float (~1 m resolution at 1e7), so the precision check reaches into the body
	// sim. In float mode this is the same demoted vector.
	private static Vec2 BodyRelativeCenter( WorldId worldId, BodyId bodyId, Pos baseVec )
	{
		World world = GetWorldFromId( worldId );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim sim = ref GetBodySim( world, ref body );
		return SubPos( sim.center, baseVec );
	}

	private const int PYRAMID_BODY_COUNT = 9;
	internal struct PyramidResult
	{
		public int sleepStep;
		public /*TODO fixed PYRAMID_BODY_COUNT*/ Vec2[] rel;
	}

	// Build a stepped pyramid of unit boxes on a base position, settle it, and report the sleep frame
	// and the settled centers relative to the base. Integer offsets keep every float bodyDef.position
	// exact even at 1e7 (the public position API is still float), so the origin and large world runs
	// start from an identical relative configuration.
	private static PyramidResult RunPyramid( Pos baseVec )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = baseVec;
			BodyId groundId = CreateBody( worldId, ref bodyDef );

			// Ground top surface at baseY + 0.5
			Polygon boxInner = MakeBox( 10.0f, 0.5f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
		}

		// Each box is 1 m, centers on integer offsets. Row 0 rests on the ground, rows stack directly
		// above so the configuration is stable and sleeps quickly.
		Vec2[] offsets = { new( -2.0f, 1.0f ), new( -1.0f, 1.0f ), new( 0.0f, 1.0f ), new( 1.0f, 1.0f ), new( 2.0f, 1.0f ),
			new( -1.0f, 2.0f ), new( 0.0f, 2.0f ),  new( 1.0f, 2.0f ), new( 0.0f, 3.0f ), };

		Span<BodyId> bodyIds = stackalloc BodyId[PYRAMID_BODY_COUNT];
		Polygon box = MakeBox( 0.5f, 0.5f );
		ShapeDef shapeDef = DefaultShapeDef();
		for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = baseVec.x + offsets[i].x, y = baseVec.y + offsets[i].y };
			bodyIds[i] = CreateBody( worldId, ref bodyDef );
			CreatePolygonShape( bodyIds[i], ref shapeDef, ref box );
		}

		PyramidResult result = new PyramidResult { sleepStep = -1 };

		for ( int step = 0; step < 250; ++step )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );

			if ( result.sleepStep < 0 )
			{
				BodyEvents events = World_GetBodyEvents( worldId );
				if ( events.moveCount == 0 && World_GetAwakeBodyCount( worldId ) == 0 )
				{
					result.sleepStep = step;
				}
			}
		}

		for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
		{
			result.rel[i] = BodyRelativeCenter( worldId, bodyIds[i], baseVec );
		}

		DestroyWorld( worldId );
		return result;
	}

	// Far from the origin the contact boundary is differenced in double, so a settling pyramid must
	// follow the same relative trajectory as one at the origin and sleep on the same frame. A float
	// build cannot resolve the body positions at 1e7, which is the whole point of large world mode.
	private static int LargeWorldPyramidTest()
	{
		PyramidResult origin = RunPyramid( new Pos { x = 0.0f, y = 0.0f } );
		ENSURE( origin.sleepStep > 0 );

#if BOX2D_DOUBLE_PRECISION
		PyramidResult large = RunPyramid( new Pos { x = 1.0e7f, y = 0.0f } );

		ENSURE( large.sleepStep == origin.sleepStep );
		for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
		{
			ENSURE_SMALL( large.rel[i].x - origin.rel[i].x, 1e-3f );
			ENSURE_SMALL( large.rel[i].y - origin.rel[i].y, 1e-3f );
		}
#endif

		return 0;
	}

	// Fire a bullet at a thin wall and report where it ends up relative to the base. With continuous
	// collision the bullet stops at the near face, without it the bullet tunnels far past the wall.
	private static float RunBullet( Pos baseVec )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		// Thin tall wall centered on the base
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.staticBody;
		bodyDef.position = baseVec;
		BodyId wallId = CreateBody( worldId, ref bodyDef );
		Polygon wall = MakeBox( 0.05f, 5.0f );
		ShapeDef shapeDef = DefaultShapeDef();
		CreatePolygonShape( wallId, ref shapeDef, ref wall );

		// Bullet fired at the wall from the far side. At 200 m/s it crosses ~3.3 m per step, so without
		// continuous collision it passes the 0.1 m wall in the first step.
		bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = new Pos { x = baseVec.x + 10.0f, y = baseVec.y };
		bodyDef.linearVelocity = new Vec2( -200.0f, 0.0f );
		BodyId bulletId = CreateBody( worldId, ref bodyDef );
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.1f };
		shapeDef = DefaultShapeDef();
		CreateCircleShape( bulletId, ref shapeDef, ref circle );

		for ( int step = 0; step < 30; ++step )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		float relX = BodyRelativeCenter( worldId, bulletId, baseVec ).x;
		DestroyWorld( worldId );
		return relX;
	}

	// The swept query box is rounded back to world float, which at 1e7 has ~1 m resolution, the most
	// likely place to drop the hit. The re-centered TOI must still catch the wall with no tunneling.
	private static int LargeWorldBulletTest()
	{
		// At the origin the bullet must stop at the near face in both precision modes. Wall face at
		// 0.05 plus the 0.1 bullet radius puts the rest position near 0.15.
		float relX = RunBullet( new Pos { x = 0.0f, y = 0.0f } );
		ENSURE( relX > 0.0f && relX < 0.5f );

#if BOX2D_DOUBLE_PRECISION
		relX = RunBullet( new Pos { x = 1.0e7f, y = 0.0f } );
		ENSURE( relX > 0.0f && relX < 0.5f );
#endif

		return 0;
	}

	// Cast a ray at a unit box on the base and report the hit point relative to the base. The ray
	// origin is differenced against the body position in double, so the hit must land on the box face
	// regardless of how far the base is from the origin.
	private static Vec2 RunRayCast( Pos baseVec )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = baseVec;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );
		Polygon box = MakeBox( 0.5f, 0.5f );
		ShapeDef shapeDef = DefaultShapeDef();
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		// Ray from 5 m left of the box, traveling 10 m right. Hits the left face at base + {-0.5, 0}.
		Pos origin = new Pos { x = baseVec.x - 5.0f, y = baseVec.y };
		Vec2 translation = new Vec2( 10.0f, 0.0f );
		RayResult result = World_CastRayClosest( worldId, origin, translation, DefaultQueryFilter() );

		// A miss leaves the point at the origin, which the caller's position check rejects
		Vec2 rel = SubPos( result.point, baseVec );
		DestroyWorld( worldId );
		return rel;
	}

	// A float ray cast at 1e7 would resolve the hit only to the ~1 m coordinate ULP. The double origin
	// plus per-shape re-centering keeps the analytic hit point accurate far from the origin.
	private static int LargeWorldRayCastTest()
	{
		Vec2 rel = RunRayCast( new Pos { x = 0.0f, y = 0.0f } );
		ENSURE_SMALL( rel.x + 0.5f, 1e-4f );
		ENSURE_SMALL( rel.y, 1e-4f );

#if BOX2D_DOUBLE_PRECISION
		rel = RunRayCast( new Pos { x = 1.0e7f, y = 0.0f } );
		ENSURE_SMALL( rel.x + 0.5f, 1e-4f );
		ENSURE_SMALL( rel.y, 1e-4f );
#endif

		return 0;
	}

	internal struct OriginQueryData
	{
		public int overlapCount;
		public Pos castPoint;
		public float castFraction;
		public float moverFraction;
		public int planeCount;
		public bool insidePoint;
		public Pos shapeRayPoint;
		public bool shapeRayHit;
	}

	private static bool OriginOverlapFcn( ShapeId id, object? ctx )
	{
		id;
		OriginQueryData* data = ctx;
		data.overlapCount += 1;
		return true;
	}

	private static float OriginCastFcn( ShapeId id, Pos point, Vec2 normal, float fraction, object? ctx )
	{
		id;
		normal;
		OriginQueryData* data = ctx;
		data.castPoint = point;
		data.castFraction = fraction;
		return fraction;
	}

	private static bool OriginPlaneFcn( ShapeId id, in PlaneResult plane, object? ctx )
	{
		id;
		plane;
		OriginQueryData* data = ctx;
		data.planeCount += 1;
		return true;
	}

	// Issue every origin taking query against a unit box on the base, with all geometry relative to
	// the base. The results must match an origin zero run, which is what makes the origin plumbing
	// (tree lift, per shape re-centering, output compose) non vacuous far from the origin.
	private static OriginQueryData RunOriginQueries( Pos @base )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = @base;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );
		Polygon box = MakeBox( 0.5f, 0.5f );
		ShapeDef shapeDef = DefaultShapeDef();
		ShapeId shapeId = CreatePolygonShape( bodyId, ref shapeDef, ref box );

		QueryFilter filter = DefaultQueryFilter();
		OriginQueryData data = default;
		data.castFraction = 1.0f;

		// Overlap a small circle centered on the box
		Vec2 center = new Vec2( 0.0f, 0.0f );
		ShapeProxy overlapProxy = MakeProxy( ref center, 1, 0.1f );
		World_OverlapShape( worldId, @base, ref overlapProxy, filter, OriginOverlapFcn, data );

		// Cast a small circle at the left face. Center stops at -0.6, hit point on the face at -0.5.
		Vec2 start = new Vec2( -5.0f, 0.0f );
		ShapeProxy castProxy = MakeProxy( ref start, 1, 0.1f );
		World_CastShape( worldId, @base, ref castProxy, new Vec2( 10.0f, 0.0f ), filter, OriginCastFcn, data );

		// Mover cast at the box
		Capsule mover = new Capsule { center1 = new( -5.0f, -0.2f ), center2 = new( -5.0f, 0.2f ), radius = 0.3f };
		data.moverFraction = World_CastMover( worldId, @base, ref mover, new Vec2( 10.0f, 0.0f ), filter );

		// Mover overlapping the box gathers planes
		Capsule touching = new Capsule { center1 = new( -0.9f, -0.2f ), center2 = new( -0.9f, 0.2f ), radius = 0.5f };
		World_CollideMover( worldId, @base, ref touching, filter, OriginPlaneFcn, data );

		// Shape level queries at the base
		data.insidePoint = Shape_TestPoint( shapeId, @base );

		WorldCastOutput rayOutput =
			Shape_CastRay( shapeId, OffsetPos( @base, new Vec2( -5.0f, 0.0f ) ), new Vec2( 10.0f, 0.0f ) );
		data.shapeRayHit = rayOutput.hit;
		data.shapeRayPoint = rayOutput.point;

		DestroyWorld( worldId );
		return data;
	}

	private static int LargeWorldOriginQueryTest()
	{
		OriginQueryData origin = RunOriginQueries( Pos_zero );
		ENSURE( origin.overlapCount == 1 );
		ENSURE( origin.castFraction < 1.0f );
		ENSURE( origin.moverFraction < 1.0f );
		ENSURE( origin.planeCount >= 1 );
		ENSURE( origin.insidePoint == true );
		ENSURE( origin.shapeRayHit == true );

		Vec2 castRel = SubPos( origin.castPoint, Pos_zero );
		ENSURE_SMALL( castRel.x + 0.5f, 1e-3f );
		Vec2 rayRel = SubPos( origin.shapeRayPoint, Pos_zero );
		ENSURE_SMALL( rayRel.x + 0.5f, 1e-3f );

#if BOX2D_DOUBLE_PRECISION
		// The same relative queries far from the origin must reproduce the origin run. A float query
		// at 1e7 could not resolve the faces below the coordinate ULP.
		Pos @base = new Pos { x = 1.0e7f, y = 0.0f };
		OriginQueryData large = RunOriginQueries( @base );
		ENSURE( large.overlapCount == origin.overlapCount );
		ENSURE( large.planeCount == origin.planeCount );
		ENSURE( large.insidePoint == origin.insidePoint );
		ENSURE( large.shapeRayHit == origin.shapeRayHit );
		ENSURE_SMALL( large.castFraction - origin.castFraction, 1e-4f );
		ENSURE_SMALL( large.moverFraction - origin.moverFraction, 1e-4f );

		Vec2 castRelLarge = SubPos( large.castPoint, @base );
		ENSURE_SMALL( castRelLarge.x - castRel.x, 1e-3f );
		ENSURE_SMALL( castRelLarge.y - castRel.y, 1e-3f );

		Vec2 rayRelLarge = SubPos( large.shapeRayPoint, @base );
		ENSURE_SMALL( rayRelLarge.x - rayRel.x, 1e-3f );
		ENSURE_SMALL( rayRelLarge.y - rayRel.y, 1e-3f );
#endif

		return 0;
	}

	// A recording made far from the origin must replay with no divergence. The recorded world positions
	// ride the double precision wire format added for large world mode; demoting them to float would snap
	// a replayed body off the recorded path and the per step state hash would diverge. Float build: the
	// positions are float either way, so this is a plain recording round trip.
	private static int LargeWorldRecordingTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		// At 1e7 the double precision wire format is what keeps the replay on the recorded path. The
		// float build has no large world range (the broad phase asserts |coord| < Huge = 1e5), so it
		// records the same scene at the origin: a plain recording round trip.
#if BOX2D_DOUBLE_PRECISION
		Pos @base = new Pos { x = 1.0e7f, y = 0.0f };
#else
		Pos @base = new Pos { x = 0.0f, y = 0.0f };
#endif

		// Record from before any body exists so world positions ride the op stream (CreateBody,
		// SetTransform), exercising the recorded position wire format rather than the seed snapshot.
		Recording rec = CreateRecording( 0 );
		World_StartRecording( worldId, rec );

		BodyDef groundDef = DefaultBodyDef();
		groundDef.position = @base;
		BodyId groundId = CreateBody( worldId, ref groundDef );
		Polygon groundBox = MakeBox( 20.0f, 0.5f );
		ShapeDef groundShapeDef = DefaultShapeDef();
		CreatePolygonShape( groundId, ref groundShapeDef, ref groundBox );

		// A box settling on the ground exercises the contact solver far from the origin.
		BodyDef stackDef = DefaultBodyDef();
		stackDef.type = BodyType.dynamicBody;
		stackDef.position = new Pos { x = @base.x, y = @base.y + 1.0f };
		BodyId stackId = CreateBody( worldId, ref stackDef );
		Polygon box = MakeBox( 0.5f, 0.5f );
		ShapeDef boxDef = DefaultShapeDef();
		boxDef.density = 1.0f;
		CreatePolygonShape( stackId, ref boxDef, ref box );

		// A free body sliding along x so its center accrues sub-meter detail that float cannot hold at
		// 1e7. Its evolved double transform is fed back through SetTransform mid recording, so the op
		// stream carries a position no float could round trip.
		BodyDef sliderDef = DefaultBodyDef();
		sliderDef.type = BodyType.dynamicBody;
		sliderDef.gravityScale = 0.0f;
		sliderDef.position = new Pos { x = @base.x, y = @base.y + 5.0f };
		sliderDef.linearVelocity = new Vec2( 3.0f, 0.0f );
		BodyId sliderId = CreateBody( worldId, ref sliderDef );
		CreatePolygonShape( sliderId, ref boxDef, ref box );

		for ( int step = 0; step < 30; ++step )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		WorldTransform sliderXf = Body_GetTransform( sliderId );
		Body_SetTransform( sliderId, sliderXf.p, sliderXf.q );

		for ( int step = 0; step < 30; ++step )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		World_StopRecording( worldId );
		DestroyWorld( worldId );

		ref readonly byte data = ref Recording_GetData( rec );
		int size = Recording_GetSize( rec );
		ENSURE( size > 0 );

		// Replay at the recorded worker count and a different one. A demoted op stream position would
		// diverge the state hash; the double precision wire format reproduces the run exactly.
		ENSURE( ValidateRecording( data, size, 0 ) );
		ENSURE( ValidateRecording( data, size, 4 ) );

		DestroyRecording( rec );
		return 0;
	}

	internal static int LargeWorldTest()
	{
		RUN_SUBTEST( LargeWorldPyramidTest );
		RUN_SUBTEST( LargeWorldBulletTest );
		RUN_SUBTEST( LargeWorldRayCastTest );
		RUN_SUBTEST( LargeWorldOriginQueryTest );
		RUN_SUBTEST( LargeWorldRecordingTest );

		return 0;
	}

}
