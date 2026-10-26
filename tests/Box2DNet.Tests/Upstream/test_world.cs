// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Tests.TestMacros;


namespace Box2DNet.Tests;

public static partial class WorldTests
{
	private const int WORLD_COUNT = MaxWorlds / 2;

	// This is a simple example of building and running a simulation
	// using Box2D. Here we create a large ground box and a small dynamic
	// box.
	// There are no graphics for this example. Box2D is meant to be used
	// with your rendering engine in your game engine.
	internal static int HelloWorld()
	{
		// Construct a world object, which will hold and simulate the rigid bodies.
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );

		WorldId worldId = CreateWorld( ref worldDef );
		ENSURE( World_IsValid( worldId ) );

		// Define the ground body.
		BodyDef groundBodyDef = DefaultBodyDef();
		groundBodyDef.position = new Pos { x = 0.0f, y = -10.0f };

		// Call the body factory which allocates memory for the ground body
		// from a pool and creates the ground box shape (also from a pool).
		// The body is also added to the world.
		BodyId groundId = CreateBody( worldId, ref groundBodyDef );
		ENSURE( Body_IsValid( groundId ) );

		// Define the ground box shape. The extents are the half-widths of the box.
		Polygon groundBox = MakeBox( 50.0f, 10.0f );

		// Add the box shape to the ground body.
		ShapeDef groundShapeDef = DefaultShapeDef();
		CreatePolygonShape( groundId, ref groundShapeDef, ref groundBox );

		// Define the dynamic body. We set its position and call the body factory.
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 4.0f };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		// Define another box shape for our dynamic body.
		Polygon dynamicBox = MakeBox( 1.0f, 1.0f );

		// Define the dynamic body shape
		ShapeDef shapeDef = DefaultShapeDef();

		// Set the box density to be non-zero, so it will be dynamic.
		shapeDef.density = 1.0f;

		// Override the default friction.
		shapeDef.material.friction = 0.3f;

		// Add the shape to the body.
		CreatePolygonShape( bodyId, ref shapeDef, ref dynamicBox );

		// Prepare for simulation. Typically we use a time step of 1/60 of a
		// second (60Hz) and 4 sub-steps. This provides a high quality simulation
		// in most game scenarios.
		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;

		Pos position = Body_GetPosition( bodyId );
		Rot rotation = Body_GetRotation( bodyId );

		// This is our little game loop.
		for ( int i = 0; i < 90; ++i )
		{
			// Instruct the world to perform a single step of simulation.
			// It is generally best to keep the time step and iterations fixed.
			World_Step( worldId, timeStep, subStepCount );

			// Now print the position and angle of the body.
			position = Body_GetPosition( bodyId );
			rotation = Body_GetRotation( bodyId );

			// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, Rot_GetAngle(rotation));
		}

		// When the world destructor is called, all bodies and joints are freed. This can
		// create orphaned ids, so be careful about your world management.
		DestroyWorld( worldId );

		ENSURE( AbsFloat( position.x ) < 0.01f );
		ENSURE( AbsFloat( position.y - 1.00f ) < 0.01f );
		ENSURE( AbsFloat( Rot_GetAngle( rotation ) ) < 0.01f );

		return 0;
	}

	internal static int EmptyWorld()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );
		ENSURE( World_IsValid( worldId ) == true );

		float timeStep = 1.0f / 60.0f;
		int subStepCount = 1;

		for ( int i = 0; i < 60; ++i )
		{
			World_Step( worldId, timeStep, subStepCount );
		}

		DestroyWorld( worldId );

		ENSURE( World_IsValid( worldId ) == false );

		return 0;
	}

	private const int BODY_COUNT = 10;
	internal static int DestroyAllBodiesWorld()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );
		ENSURE( World_IsValid( worldId ) == true );

		int count = 0;
		bool creating = true;

		Span<BodyId> bodyIds = stackalloc BodyId[BODY_COUNT];
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		Polygon square = MakeSquare( 0.5f );

		for ( int i = 0; i < 2 * BODY_COUNT + 10; ++i )
		{
			if ( creating )
			{
				if ( count < BODY_COUNT )
				{
					bodyIds[count] = CreateBody( worldId, ref bodyDef );

					ShapeDef shapeDef = DefaultShapeDef();
					CreatePolygonShape( bodyIds[count], ref shapeDef, ref square );
					count += 1;
				}
				else
				{
					creating = false;
				}
			}
			else if ( count > 0 )
			{
				DestroyBody( bodyIds[count - 1] );
				bodyIds[count - 1] = nullBodyId;
				count -= 1;
			}

			World_Step( worldId, 1.0f / 60.0f, 3 );
		}

		Counters counters = World_GetCounters( worldId );
		ENSURE( counters.bodyCount == 0 );

		DestroyWorld( worldId );

		ENSURE( World_IsValid( worldId ) == false );

		return 0;
	}

	private static int TestIsValid()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );
		ENSURE( World_IsValid( worldId ) );

		BodyDef bodyDef = DefaultBodyDef();

		BodyId bodyId1 = CreateBody( worldId, ref bodyDef );
		ENSURE( Body_IsValid( bodyId1 ) == true );

		BodyId bodyId2 = CreateBody( worldId, ref bodyDef );
		ENSURE( Body_IsValid( bodyId2 ) == true );

		DestroyBody( bodyId1 );
		ENSURE( Body_IsValid( bodyId1 ) == false );

		DestroyBody( bodyId2 );
		ENSURE( Body_IsValid( bodyId2 ) == false );

		DestroyWorld( worldId );

		ENSURE( World_IsValid( worldId ) == false );
		ENSURE( Body_IsValid( bodyId2 ) == false );
		ENSURE( Body_IsValid( bodyId1 ) == false );

		return 0;
	}

	// #define WORLD_COUNT ( MaxWorlds / 2 )

	internal static int TestWorldRecycle()
	{
		int count = 100;

		Span<WorldId> worldIds = stackalloc WorldId[WORLD_COUNT];

		for ( int i = 0; i < count; ++i )
		{
			WorldDef worldDef = DefaultWorldDef();
			for ( int j = 0; j < WORLD_COUNT; ++j )
			{
				worldIds[j] = CreateWorld( ref worldDef );
				ENSURE( World_IsValid( worldIds[j] ) == true );

				BodyDef bodyDef = DefaultBodyDef();
				CreateBody( worldIds[j], ref bodyDef );
			}

			for ( int j = 0; j < WORLD_COUNT; ++j )
			{
				float timeStep = 1.0f / 60.0f;
				int subStepCount = 1;

				for ( int k = 0; k < 10; ++k )
				{
					World_Step( worldIds[j], timeStep, subStepCount );
				}
			}

			for ( int j = WORLD_COUNT - 1; j >= 0; --j )
			{
				DestroyWorld( worldIds[j] );
				ENSURE( World_IsValid( worldIds[j] ) == false );
				worldIds[j] = nullWorldId;
			}
		}

		return 0;
	}

	private static bool CustomFilter( ShapeId shapeIdA, ShapeId shapeIdB, object? context )
	{
		ENSURE( context == null );
		return true;
	}

	private static void PreSolveStatic( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object? context )
	{
	}

	// This test is here to ensure all API functions link correctly.
	internal static int TestWorldCoverage()
	{
		WorldDef worldDef = DefaultWorldDef();

		WorldId worldId = CreateWorld( ref worldDef );
		ENSURE( World_IsValid( worldId ) );

		World_EnableSleeping( worldId, true );
		World_EnableSleeping( worldId, false );
		bool flag = World_IsSleepingEnabled( worldId );
		ENSURE( flag == false );

		World_EnableContinuous( worldId, false );
		World_EnableContinuous( worldId, true );
		flag = World_IsContinuousEnabled( worldId );
		ENSURE( flag == true );

		World_SetRestitutionThreshold( worldId, 0.0f );
		World_SetRestitutionThreshold( worldId, 2.0f );
		float value = World_GetRestitutionThreshold( worldId );
		ENSURE( value == 2.0f );

		World_SetHitEventThreshold( worldId, 0.0f );
		World_SetHitEventThreshold( worldId, 100.0f );
		value = World_GetHitEventThreshold( worldId );
		ENSURE( value == 100.0f );

		World_SetCustomFilterCallback( worldId, CustomFilter, null );
		World_SetPreSolveCallback( worldId, PreSolveStatic, null, null );

		Vec2 g = new Vec2( 1.0f, 2.0f );
		World_SetGravity( worldId, g );
		Vec2 v = World_GetGravity( worldId );
		ENSURE( v.x == g.x );
		ENSURE( v.y == g.y );

		ExplosionDef explosionDef = DefaultExplosionDef();
		World_Explode( worldId, ref explosionDef );

		World_SetContactTuning( worldId, 10.0f, 2.0f, 4.0f );
		float hertz = 0.0f, dampingRatio = 0.0f, pushSpeed = 0.0f;
		World_GetContactTuning( worldId, ref hertz, ref dampingRatio, ref pushSpeed );
		ENSURE( hertz == 10.0f && dampingRatio == 2.0f && pushSpeed == 4.0f );

		World_SetMaxLinearSpeed( worldId, 10.0f );
		value = World_GetMaxLinearSpeed( worldId );
		ENSURE( value == 10.0f );

		World_EnableWarmStarting( worldId, true );
		flag = World_IsWarmStartingEnabled( worldId );
		ENSURE( flag == true );

		int count = World_GetAwakeBodyCount( worldId );
		ENSURE( count == 0 );

		object boxedValue = value;
		World_SetUserData( worldId, boxedValue );
		object? userData = World_GetUserData( worldId );
		ENSURE( ReferenceEquals( userData, boxedValue ) );

		World_Step( worldId, 1.0f, 1 );

		DestroyWorld( worldId );

		return 0;
	}

	private static int TestSensor()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		// Wall from x = 1 to x = 2
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.staticBody;
		bodyDef.position.x = 1.5f;
		bodyDef.position.y = 11.0f;
		BodyId wallId = CreateBody( worldId, ref bodyDef );
		Polygon box = MakeBox( 0.5f, 10.0f );
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = true;
		CreatePolygonShape( wallId, ref shapeDef, ref box );

		// Bullet fired towards the wall
		bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = new Pos { x = 7.39814f, y = 4.0f };
		bodyDef.linearVelocity = new Vec2( -20.0f, 0.0f );
		BodyId bulletId = CreateBody( worldId, ref bodyDef );
		shapeDef = DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.1f };
		CreateCircleShape( bulletId, ref shapeDef, ref circle );

		int beginCount = 0;
		int endCount = 0;

		while ( true )
		{
			float timeStep = 1.0f / 60.0f;
			int subStepCount = 4;
			World_Step( worldId, timeStep, subStepCount );

			Pos bulletPos = Body_GetPosition( bulletId );
			// Console.WriteLine( $"Bullet pos: {bulletPos.x:G6} {bulletPos.y:G6}" );

			SensorEvents events = World_GetSensorEvents( worldId );

			if ( events.beginCount > 0 )
			{
				beginCount += 1;
			}

			if ( events.endCount > 0 )
			{
				endCount += 1;
			}

			if ( bulletPos.x < -1.0f )
			{
				break;
			}
		}

		DestroyWorld( worldId );

		ENSURE( beginCount == 1 );
		ENSURE( endCount == 1 );

		return 0;
	}


	private static int TestSetWorkerCount()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( worldDef );
		ENSURE( World_IsValid( worldId ) );
		ENSURE( World_GetWorkerCount( worldId ) == 1 );

		CreateJunkyard( worldId );
		StepJunkyard( worldId, 1 );

		World_SetWorkerCount( worldId, 4 );
		ENSURE( World_GetWorkerCount( worldId ) == 4 );

		StepJunkyard( worldId, 2 );

		World_SetWorkerCount( worldId, 4 );
		ENSURE( World_GetWorkerCount( worldId ) == 4 );

		StepJunkyard( worldId, 3 );

		World_SetWorkerCount( worldId, 0 );
		ENSURE( World_GetWorkerCount( worldId ) == 1 );

		StepJunkyard( worldId, 4 );

		World_SetWorkerCount( worldId, -5 );
		ENSURE( World_GetWorkerCount( worldId ) == 1 );

		StepJunkyard( worldId, 5 );

		World_SetWorkerCount( worldId, MaxWorkers + 10 );
		ENSURE( World_GetWorkerCount( worldId ) == MaxWorkers );

		StepJunkyard( worldId, 2 );

		DestroyWorld( worldId );

		return 0;
	}

	private static int ChainSegmentShapeTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		ChainSegment cs = default;
		cs.ghost1 = new Vec2( 2.0f, 0.0f );
		cs.segment.point1 = new Vec2( 1.0f, 0.0f );
		cs.segment.point2 = new Vec2( -1.0f, 0.0f );
		cs.ghost2 = new Vec2( -2.0f, 0.0f );
		cs.chainId = 99;

		ShapeDef shapeDef = DefaultShapeDef();
		ShapeId orphanShape = CreateChainSegmentShape( groundId, ref shapeDef, ref cs );
		ENSURE( orphanShape.IsNonNull );

		ENSURE( Shape_GetType( orphanShape ) == ShapeType.chainSegmentShape );

		ChainId parentChain = Shape_GetParentChain( orphanShape );
		ENSURE( parentChain.IsNull );

		ChainSegment got = Shape_GetChainSegment( orphanShape );
		ENSURE_SMALL( got.ghost1.x - cs.ghost1.x, 1e-5f );
		ENSURE_SMALL( got.ghost1.y - cs.ghost1.y, 1e-5f );
		ENSURE_SMALL( got.segment.point1.x - cs.segment.point1.x, 1e-5f );
		ENSURE_SMALL( got.segment.point1.y - cs.segment.point1.y, 1e-5f );
		ENSURE_SMALL( got.segment.point2.x - cs.segment.point2.x, 1e-5f );
		ENSURE_SMALL( got.segment.point2.y - cs.segment.point2.y, 1e-5f );
		ENSURE_SMALL( got.ghost2.x - cs.ghost2.x, 1e-5f );
		ENSURE_SMALL( got.ghost2.y - cs.ghost2.y, 1e-5f );
		ENSURE( got.chainId == NullIndex );

		BodyDef dynamicDef = DefaultBodyDef();
		dynamicDef.type = BodyType.dynamicBody;
		dynamicDef.position = new Pos { x = 0.0f, y = 2.0f };
		BodyId circleBodyId = CreateBody( worldId, ref dynamicDef );
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		ShapeDef circleShapeDef = DefaultShapeDef();
		CreateCircleShape( circleBodyId, ref circleShapeDef, ref circle );

		for ( int i = 0; i < 120; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		Pos circlePos = Body_GetPosition( circleBodyId );
		ENSURE( circlePos.y > 0.0f );

		ChainSegment cs2 = default;
		cs2.ghost1 = new Vec2( 3.0f, 0.0f );
		cs2.segment.point1 = new Vec2( 2.0f, 0.0f );
		cs2.segment.point2 = new Vec2( -2.0f, 0.0f );
		cs2.ghost2 = new Vec2( -3.0f, 0.0f );
		cs2.chainId = NullIndex;

		Shape_SetChainSegment( orphanShape, ref cs2 );

		ChainSegment got2 = Shape_GetChainSegment( orphanShape );
		ENSURE_SMALL( got2.segment.point1.x - cs2.segment.point1.x, 1e-5f );
		ENSURE_SMALL( got2.segment.point1.y - cs2.segment.point1.y, 1e-5f );
		ENSURE_SMALL( got2.segment.point2.x - cs2.segment.point2.x, 1e-5f );
		ENSURE_SMALL( got2.segment.point2.y - cs2.segment.point2.y, 1e-5f );
		ENSURE_SMALL( got2.ghost1.x - cs2.ghost1.x, 1e-5f );
		ENSURE_SMALL( got2.ghost1.y - cs2.ghost1.y, 1e-5f );
		ENSURE_SMALL( got2.ghost2.x - cs2.ghost2.x, 1e-5f );
		ENSURE_SMALL( got2.ghost2.y - cs2.ghost2.y, 1e-5f );
		ENSURE( got2.chainId == NullIndex );

		ChainId parentChain2 = Shape_GetParentChain( orphanShape );
		ENSURE( parentChain2.IsNull );

		BodyId convBody = CreateBody( worldId, ref bodyDef );
		Circle convCircle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };
		ShapeId convShape = CreateCircleShape( convBody, ref shapeDef, ref convCircle );
		ENSURE( Shape_GetType( convShape ) == ShapeType.circleShape );

		Shape_SetChainSegment( convShape, ref cs2 );

		ENSURE( Shape_GetType( convShape ) == ShapeType.chainSegmentShape );
		ChainSegment got3 = Shape_GetChainSegment( convShape );
		ENSURE_SMALL( got3.ghost1.x - cs2.ghost1.x, 1e-5f );
		ENSURE_SMALL( got3.ghost1.y - cs2.ghost1.y, 1e-5f );
		ENSURE_SMALL( got3.ghost2.x - cs2.ghost2.x, 1e-5f );
		ENSURE_SMALL( got3.ghost2.y - cs2.ghost2.y, 1e-5f );
		ENSURE( got3.chainId == NullIndex );

		ChainId parentChain3 = Shape_GetParentChain( convShape );
		ENSURE( parentChain3.IsNull );

		DestroyShape( orphanShape, true );
		DestroyWorld( worldId );

		return 0;
	}

	private static int DeferredMassFlagSyncTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.updateBodyMass = false;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		CreateCircleShape( bodyId, ref shapeDef, ref circle );

		Body_UpdateMassFromShapes( bodyId );

		World_Step( worldId, 1.0f / 60.0f, 4 );

		DestroyWorld( worldId );
		return 0;
	}

	private static int EnableSleepFlagSyncTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.enableSleep = false;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ENSURE( Body_IsSleepingEnabled( bodyId ) == false );

		Body_EnableSleeping( bodyId, true );
		ENSURE( Body_IsSleepingEnabled( bodyId ) == true );

		World_Step( worldId, 1.0f / 60.0f, 4 );

		DestroyWorld( worldId );
		return 0;
	}

	private static int EnableContactRecyclingTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		// Default is enabled
		BodyId bodyA = CreateBody( worldId, ref bodyDef );
		ENSURE( Body_IsContactRecyclingEnabled( bodyA ) == true );

		Body_EnableContactRecycling( bodyA, false );
		ENSURE( Body_IsContactRecyclingEnabled( bodyA ) == false );

		Body_EnableContactRecycling( bodyA, true );
		ENSURE( Body_IsContactRecyclingEnabled( bodyA ) == true );

		// Per-def opt-out at creation
		bodyDef.enableContactRecycling = false;
		BodyId bodyB = CreateBody( worldId, ref bodyDef );
		ENSURE( Body_IsContactRecyclingEnabled( bodyB ) == false );

		// Stepping after toggling must not trip the flag-sync validator
		World_Step( worldId, 1.0f / 60.0f, 4 );

		DestroyWorld( worldId );
		return 0;
	}

	private static void PreSolveReject( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object? context )
	{
		ref bool reject = ref ( (StrongBox<bool>)context! ).Value;
		if ( reject )
		{
			manifold.pointCount = 0;
		}
	}

	// A rejected pre-solve empties the manifold while the shapes may already touch. The next step must
	// rebuild the contact and ask again instead of recycling the empty pair. Box3D discussion #175.
	private static int PreSolveRejectRecycleTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.enableSleep = false;
		WorldId worldId = CreateWorld( ref worldDef );

		StrongBox<bool> reject = new StrongBox<bool>( true );
		World_SetPreSolveCallback( worldId, PreSolveReject, null, reject );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -0.5f };
		BodyId groundId = CreateBody( worldId, ref bodyDef );
		Polygon groundBox = MakeBox( 10.0f, 0.5f );
		ShapeDef shapeDef = DefaultShapeDef();
		CreatePolygonShape( groundId, ref shapeDef, ref groundBox );

		float radius = 0.25f;
		bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = radius };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		shapeDef.enablePreSolveEvents = true;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = radius };
		CreateCircleShape( bodyId, ref shapeDef, ref circle );

		World_Step( worldId, 1.0f / 60.0f, 4 );

		reject.Value = false;

		float minSeparation = FLT_MAX;
		for ( int i = 0; i < 30; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );

			Pos p = Body_GetPosition( bodyId );
			minSeparation = MinFloat( minSeparation, (float)p.y - radius );
		}

		DestroyWorld( worldId );

		ENSURE( minSeparation > -LinearSlop );

		return 0;
	}

	private static int SetBulletDriftTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.isBullet = false;
			BodyId bodyId = CreateBody( worldId, ref bodyDef );

			ENSURE( Body_IsBullet( bodyId ) == false );

			Body_SetBullet( bodyId, true );
			ENSURE( Body_IsBullet( bodyId ) == true );

			MotionLocks locks = default;
			locks.linearX = true;
			Body_SetMotionLocks( bodyId, locks );

			ENSURE( Body_IsBullet( bodyId ) == true );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.isBullet = true;
			BodyId bodyId = CreateBody( worldId, ref bodyDef );

			ENSURE( Body_IsBullet( bodyId ) == true );

			Body_SetBullet( bodyId, false );
			ENSURE( Body_IsBullet( bodyId ) == false );

			MotionLocks locks = default;
			locks.linearX = true;
			Body_SetMotionLocks( bodyId, locks );

			ENSURE( Body_IsBullet( bodyId ) == false );
		}

		DestroyWorld( worldId );
		return 0;
	}

	// Counts marked nodes, the root included. Free nodes are never marked.
	private static int CountEnlargedNodes( DynamicTree tree )
	{
		int count = 0;
		for ( int i = 0; i < tree.nodeEnd; ++i )
		{
			count += IsNodeMoved( tree.nodes[i] ) ? 1 : 0;
		}

		return count;
	}

	// A proxy can be enlarged in one step and destroyed before the next. The remove recomputes the
	// ancestor marks from the surviving children, so the mover's marks go with it and the next step
	// starts clean.
	private static int EnlargedProxyDestroyed( int workerCount )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );

		World world = GetWorldFromId( worldId );
		DynamicTree tree = world.broadPhase.trees[BodyType.dynamicBody];
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();
		Circle circle = new Circle { center = Vec2_zero, radius = 0.25f };

		// Resting bodies keep the tree deep enough that the mover has an ancestor that outlives it
		for ( int i = 0; i < 8; ++i )
		{
			bodyDef.position = new Pos { x = 2.0f * i, y = 0.0f };
			BodyId restingId = CreateBody( worldId, ref bodyDef );
			CreateCircleShape( restingId, ref shapeDef, ref circle );
		}

		// This body outruns its AABB margin every step
		bodyDef.position = new Pos { x = 7.0f, y = 1.0f };
		bodyDef.linearVelocity = new Vec2( 6.0f, 0.0f );
		BodyId moverId = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( moverId, ref shapeDef, ref circle );

		float timeStep = 1.0f / 60.0f;
		World_Step( worldId, timeStep, 4 );

		ENSURE( CountEnlargedNodes( tree ) > 0 );

		DestroyBody( moverId );

		ENSURE( CountEnlargedNodes( tree ) == 0 );

		World_Step( worldId, timeStep, 4 );

		ENSURE( CountEnlargedNodes( tree ) == 0 );

		DestroyWorld( worldId );
		return 0;
	}

	private static int EnlargedProxyDestroyedTest()
	{
		ENSURE( EnlargedProxyDestroyed( 1 ) == 0 );
		ENSURE( EnlargedProxyDestroyed( 4 ) == 0 );

		return 0;
	}

	// Broad phase pair creation through every route that marks a proxy: creation, teleport, a static
	// shape that asks for contacts, refiltering, collide connected, enable, a type change, a static
	// rebuild that must keep its pending marks, and the filters that reject a pair.
	// Bodies never move on their own here, gravity is off and sleep is disabled, so a contact can only
	// come from the route under test. The count is every live contact, touching or not, which is what
	// the broad phase decides.

	internal struct PairBox
	{
		public BodyId bodyId;
		public ShapeId shapeId;
	}

	private static PairBox CreatePairBox( WorldId worldId, BodyType type, float x, float y, in ShapeDef shapeDef )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = type;
		bodyDef.position = new Pos { x = x, y = y };

		PairBox box;
		box.bodyId = CreateBody( worldId, ref bodyDef );

		Polygon square = MakeSquare( 0.5f );
		box.shapeId = CreatePolygonShape( box.bodyId, shapeDef, ref square );
		return box;
	}

	private static WorldId CreatePairWorld( int workerCount )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		worldDef.enableSleep = false;
		worldDef.workerCount = workerCount;
		return CreateWorld( ref worldDef );
	}

	private static int PairContactCount( WorldId worldId )
	{
		return World_GetCounters( worldId ).contactCount;
	}

	private static void StepPairWorld( WorldId worldId )
	{
		World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	// Every kind of proxy created over a resting body
	private static int PairCreation( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		// Dynamic created on static
		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 0.5f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		// Dynamic created on a resting dynamic
		CreatePairBox( worldId, BodyType.dynamicBody, 10.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );
		CreatePairBox( worldId, BodyType.dynamicBody, 10.5f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		// Kinematic created on a resting dynamic, the mark is on the kinematic side
		CreatePairBox( worldId, BodyType.dynamicBody, 20.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		CreatePairBox( worldId, BodyType.kinematicBody, 20.5f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 3 );

		// A static shape created on a resting dynamic pairs by default and not when told to stay quiet
		ShapeDef quietDef = DefaultShapeDef();
		quietDef.invokeContactCreation = false;
		CreatePairBox( worldId, BodyType.dynamicBody, 30.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		CreatePairBox( worldId, BodyType.staticBody, 30.5f, 0.0f, ref quietDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 3 );

		CreatePairBox( worldId, BodyType.staticBody, 29.5f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 4 );

		DestroyWorld( worldId );
		return 0;
	}

	// A teleport marks the moved proxy, kinematic and static ones included
	private static int PairTeleport( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );
		PairBox mover = CreatePairBox( worldId, BodyType.dynamicBody, 10.0f, 0.0f, ref shapeDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 20.0f, 0.0f, ref shapeDef );
		PairBox wall = CreatePairBox( worldId, BodyType.staticBody, 30.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Body_SetTransform( mover.bodyId, new Pos { x = 0.5f, y = 0.0f }, Rot_identity );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_SetTransform( wall.bodyId, new Pos { x = 20.5f, y = 0.0f }, Rot_identity );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		// Kinematic onto a resting dynamic, the mark is on the kinematic side
		CreatePairBox( worldId, BodyType.dynamicBody, 40.0f, 0.0f, ref shapeDef );
		PairBox pusher = CreatePairBox( worldId, BodyType.kinematicBody, 50.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		Body_SetTransform( pusher.bodyId, new Pos { x = 40.5f, y = 0.0f }, Rot_identity );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 3 );

		DestroyWorld( worldId );
		return 0;
	}

	// A filter change must find the pairs the old filter rejected and drop the ones it now rejects
	private static int PairRefilter( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );

		ShapeDef pickyDef = DefaultShapeDef();
		pickyDef.filter.categoryBits = 2;
		pickyDef.filter.maskBits = 1;

		ShapeDef closedDef = DefaultShapeDef();
		closedDef.filter.categoryBits = 4;

		Filter openFilter = DefaultFilter();
		openFilter.categoryBits = 1;

		PairBox ground = CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref closedDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 0.5f, 0.0f, ref pickyDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Shape_SetFilter( ground.shapeId, openFilter );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Shape_SetFilter( ground.shapeId, closedDef.filter );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		// Same on a dynamic shape
		PairBox other = CreatePairBox( worldId, BodyType.dynamicBody, 10.0f, 0.0f, ref closedDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 10.5f, 0.0f, ref pickyDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Shape_SetFilter( other.shapeId, openFilter );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		DestroyWorld( worldId );
		return 0;
	}

	// Collide connected is off by default and turning it on must find the pair, static partner too.
	// The weld frames coincide so the joint does not move anything.
	private static int PairCollideConnected( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		PairBox boxA = CreatePairBox( worldId, BodyType.dynamicBody, 0.0f, 0.0f, ref shapeDef );
		PairBox boxB = CreatePairBox( worldId, BodyType.dynamicBody, 0.5f, 0.0f, ref shapeDef );

		WeldJointDef jointDef = DefaultWeldJointDef();
		jointDef.@base.bodyIdA = boxA.bodyId;
		jointDef.@base.bodyIdB = boxB.bodyId;
		jointDef.@base.localFrameB.p = new Vec2( -0.5f, 0.0f );
		JointId jointId = CreateWeldJoint( worldId, ref jointDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Joint_SetCollideConnected( jointId, true );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Joint_SetCollideConnected( jointId, false );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		// Static partner, the mark goes on the dynamic body
		PairBox wall = CreatePairBox( worldId, BodyType.staticBody, 10.0f, 0.0f, ref shapeDef );
		PairBox boxC = CreatePairBox( worldId, BodyType.dynamicBody, 10.5f, 0.0f, ref shapeDef );
		jointDef.@base.bodyIdA = wall.bodyId;
		jointDef.@base.bodyIdB = boxC.bodyId;
		JointId wallJointId = CreateWeldJoint( worldId, ref jointDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Joint_SetCollideConnected( wallJointId, true );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		DestroyWorld( worldId );
		return 0;
	}

	private static bool RejectTaggedShape( ShapeId shapeIdA, ShapeId shapeIdB, object? context )
	{
		return Shape_GetUserData( shapeIdA ) == null && Shape_GetUserData( shapeIdB ) == null;
	}

	// The filters that reject a pair. The benchmarks never exercise these, so this is the only guard.
	private static int PairFilters( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		World_SetCustomFilterCallback( worldId, RejectTaggedShape, null );

		// Category and mask must pass in both directions
		ShapeDef defA = DefaultShapeDef();
		defA.filter.categoryBits = 1;
		defA.filter.maskBits = 2;
		ShapeDef defB = DefaultShapeDef();
		defB.filter.categoryBits = 2;
		defB.filter.maskBits = 1;
		ShapeDef defC = DefaultShapeDef();
		defC.filter.categoryBits = 2;
		defC.filter.maskBits = 4;
		CreatePairBox( worldId, BodyType.dynamicBody, 0.0f, 0.0f, ref defA );
		CreatePairBox( worldId, BodyType.dynamicBody, 0.5f, 0.0f, ref defB );
		CreatePairBox( worldId, BodyType.dynamicBody, -0.5f, 0.0f, ref defC );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		// Group index overrides the bits, negative rejects and positive accepts
		ShapeDef negativeDef = DefaultShapeDef();
		negativeDef.filter.groupIndex = -1;
		CreatePairBox( worldId, BodyType.dynamicBody, 10.0f, 0.0f, ref negativeDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 10.5f, 0.0f, ref negativeDef );
		ShapeDef positiveDef = defC;
		positiveDef.filter.groupIndex = 1;
		CreatePairBox( worldId, BodyType.dynamicBody, 20.0f, 0.0f, ref positiveDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 20.5f, 0.0f, ref positiveDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		// Custom filter rejects a tagged shape
		ShapeDef customDef = DefaultShapeDef();
		customDef.enableCustomFiltering = true;
		CreatePairBox( worldId, BodyType.dynamicBody, 30.0f, 0.0f, ref customDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 30.5f, 0.0f, ref customDef );
		customDef.userData = 1;
		CreatePairBox( worldId, BodyType.dynamicBody, 40.0f, 0.0f, ref customDef );
		customDef.userData = null;
		CreatePairBox( worldId, BodyType.dynamicBody, 40.5f, 0.0f, ref customDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 3 );

		// Sensors never make contacts
		ShapeDef sensorDef = DefaultShapeDef();
		sensorDef.isSensor = true;
		ShapeDef plainDef = DefaultShapeDef();
		CreatePairBox( worldId, BodyType.staticBody, 50.0f, 0.0f, ref sensorDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 50.5f, 0.0f, ref plainDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 3 );

		DestroyWorld( worldId );
		return 0;
	}

	// A body that outran its margin on the step it went to sleep still gets its contact. This is the
	// hazard behind section G: the marks live in the tree and do not care which set the body is in.
	private static int PairSleepEscape( int workerCount, bool sleepBeforePairs )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );

		ShapeDef shapeDef = DefaultShapeDef();
		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -2.0f, y = 0.0f };
		bodyDef.linearVelocity = new Vec2( 58.0f, 0.0f );
		BodyId moverId = CreateBody( worldId, ref bodyDef );
		Polygon square = MakeSquare( 0.5f );
		CreatePolygonShape( moverId, ref shapeDef, ref square );

		// One step carries the mover 0.97, far past its margin and into the fat box of the wall with a
		// few centimeters to spare, but not into the wall itself
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		if ( sleepBeforePairs )
		{
			Body_SetAwake( moverId, false );
			ENSURE( Body_IsAwake( moverId ) == false );
		}

		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		DestroyWorld( worldId );
		return 0;
	}

	// Enable recreates the proxies with marks, on either side of a resting pair. The fat boxes
	// overlap across the gap while the shapes do not, so nothing pushes and nothing moves.
	private static int PairEnable( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		PairBox ground = CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );
		PairBox box = CreatePairBox( worldId, BodyType.dynamicBody, 1.1f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_Disable( box.bodyId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Body_Enable( box.bodyId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		// The static side, the resting box has no mark of its own
		Body_Disable( ground.bodyId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		Body_Enable( ground.bodyId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		DestroyWorld( worldId );
		return 0;
	}

	// A type change recreates the proxies with marks and must find the resting neighbors again
	private static int PairSetType( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );
		PairBox middle = CreatePairBox( worldId, BodyType.dynamicBody, 1.1f, 0.0f, ref shapeDef );
		CreatePairBox( worldId, BodyType.dynamicBody, 2.2f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		// Kinematic drops the static pair and keeps the dynamic one
		Body_SetType( middle.bodyId, BodyType.kinematicBody );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_SetType( middle.bodyId, BodyType.dynamicBody );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		// Static drops the static pair too and the dynamic pair becomes static versus dynamic
		Body_SetType( middle.bodyId, BodyType.staticBody );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_SetType( middle.bodyId, BodyType.dynamicBody );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		DestroyWorld( worldId );
		return 0;
	}

	// Bullet is a flag only, the pair must be untouched
	private static int PairSetBullet( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.0f, ref shapeDef );
		PairBox box = CreatePairBox( worldId, BodyType.dynamicBody, 1.1f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_SetBullet( box.bodyId, true );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		Body_SetBullet( box.bodyId, false );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		DestroyWorld( worldId );
		return 0;
	}

	// A static rebuild must keep the pending marks. The resting box has none of its own, so a lost
	// mark means the pair is never found.
	private static int PairRebuildStatic( int workerCount )
	{
		WorldId worldId = CreatePairWorld( workerCount );
		ShapeDef shapeDef = DefaultShapeDef();

		PairBox box = CreatePairBox( worldId, BodyType.dynamicBody, 0.0f, 0.0f, ref shapeDef );
		CreatePairBox( worldId, BodyType.staticBody, 10.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		// No marks to keep
		World_RebuildStaticTree( worldId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 0 );

		CreatePairBox( worldId, BodyType.staticBody, 1.1f, 0.0f, ref shapeDef );
		World_RebuildStaticTree( worldId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		// A quiet static shape has no mark to keep
		ShapeDef quietDef = DefaultShapeDef();
		quietDef.invokeContactCreation = false;
		CreatePairBox( worldId, BodyType.staticBody, -1.1f, 0.0f, ref quietDef );
		World_RebuildStaticTree( worldId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		// Overlapping the resting box, the contact must touch on the first step after the rebuild
		CreatePairBox( worldId, BodyType.staticBody, 0.0f, 0.5f, ref shapeDef );
		World_RebuildStaticTree( worldId );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 2 );

		FixedArray4<ContactData> contactData = default;
		int touchingCount = Body_GetContactData( box.bodyId, contactData, 4 );
		ENSURE( touchingCount == 1 );
		ENSURE( contactData[0].manifold.pointCount > 0 );

		DestroyWorld( worldId );
		return 0;
	}

	// A static shape created over a sleeping body still gets its contact, parked with the sleeper
	// until it wakes. The world counter only sees awake contacts, so this counts on the body.
	private static int PairSleepingStatic( int workerCount )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );

		ShapeDef shapeDef = DefaultShapeDef();
		PairBox sleeper = CreatePairBox( worldId, BodyType.dynamicBody, 0.0f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		Body_SetAwake( sleeper.bodyId, false );
		ENSURE( Body_IsAwake( sleeper.bodyId ) == false );

		CreatePairBox( worldId, BodyType.staticBody, 0.5f, 0.0f, ref shapeDef );
		StepPairWorld( worldId );
		ENSURE( Body_GetContactCapacity( sleeper.bodyId ) == 1 );
		ENSURE( Body_IsAwake( sleeper.bodyId ) == false );

		// The quiet kind never pairs with a sleeper
		ShapeDef quietDef = DefaultShapeDef();
		quietDef.invokeContactCreation = false;
		CreatePairBox( worldId, BodyType.staticBody, -0.5f, 0.0f, ref quietDef );
		StepPairWorld( worldId );
		ENSURE( Body_GetContactCapacity( sleeper.bodyId ) == 1 );

		// Waking brings the parked contact into the step and it touches at once
		Body_SetAwake( sleeper.bodyId, true );
		StepPairWorld( worldId );
		ENSURE( PairContactCount( worldId ) == 1 );

		FixedArray4<ContactData> contactData = default;
		int touchingCount = Body_GetContactData( sleeper.bodyId, contactData, 4 );
		ENSURE( touchingCount == 1 );
		ENSURE( contactData[0].manifold.pointCount > 0 );

		DestroyWorld( worldId );
		return 0;
	}

	// The contact sim carries a persistent locator for each body sim instead of a copy written by the
	// collide task every step. These scenarios drive every site that changes a body's set or local index
	// while its contacts are live, then step under validation so ValidateSolverSets and the prepare
	// asserts are the real check. Worker count parity on the state hash guards the multithreaded paths.

	private static void StepLocatorWorld( WorldId worldId, int stepCount )
	{
		for ( int i = 0; i < stepCount; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}
	}

	private static bool StepUntilAsleep( WorldId worldId, BodyId bodyId, int maxSteps )
	{
		for ( int i = 0; i < maxSteps; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
			if ( Body_IsAwake( bodyId ) == false )
			{
				return true;
			}
		}

		return false;
	}

	private static WorldId CreateLocatorWorld( int workerCount, bool enableSleep )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );
		worldDef.enableSleep = enableSleep;
		worldDef.workerCount = workerCount;
		return CreateWorld( ref worldDef );
	}

	private static BodyId CreateLocatorGround( WorldId worldId, float x, float halfWidth )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = x, y = -0.5f };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( halfWidth, 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );
		return bodyId;
	}

	private static BodyId CreateLocatorBox( WorldId worldId, BodyType type, float x, float y )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = type;
		bodyDef.position = new Pos { x = x, y = y };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon square = MakeSquare( 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref square );
		return bodyId;
	}

	// A pile sleeps on static ground and a dropped body wakes it. Covers first touch, the graph add and
	// the wake of a whole sleeping set.
	private static int LocatorSleepWake( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, true );
		CreateLocatorGround( worldId, 0.0f, 10.0f );

		FixedArray3<BodyId> pile = default;
		for ( int i = 0; i < 3; ++i )
		{
			pile[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 0.5f + i );
		}

		ENSURE( StepUntilAsleep( worldId, pile[0], 600 ) );
		ENSURE( Body_IsAwake( pile[2] ) == false );

		BodyId dropper = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 6.0f );
		ENSURE( Body_IsAwake( pile[0] ) == false );

		StepLocatorWorld( worldId, 60 );
		ENSURE( Body_IsAwake( pile[0] ) );
		ENSURE( Body_IsAwake( dropper ) );

		ENSURE( StepUntilAsleep( worldId, dropper, 900 ) );
		ENSURE( Body_IsAwake( pile[0] ) == false );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// Destroying a body in the middle of the awake set swaps the last awake sim into the hole while that
	// body's contacts are live.
	private static int LocatorDestroyAwakeBody( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, false );
		CreateLocatorGround( worldId, 0.0f, 20.0f );

		FixedArray4<BodyId> stack = default;
		for ( int i = 0; i < 4; ++i )
		{
			stack[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 0.5f + i );
		}

		// A second stack far away owns the tail of the awake set
		FixedArray3<BodyId> neighbors = default;
		for ( int i = 0; i < 3; ++i )
		{
			neighbors[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, 10.0f, 0.5f + i );
		}

		StepLocatorWorld( worldId, 120 );
		float restY = (float)Body_GetPosition( neighbors[2] ).y;

		DestroyBody( stack[0] );
		StepLocatorWorld( worldId, 30 );

		ENSURE_SMALL( (float)Body_GetPosition( neighbors[2] ).y - restY, 0.01f );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// Destroying a static body swaps the last static sim into the hole. The moved body still has awake
	// contacts, so its side of them has to be re-encoded or the collide task reads past the static set.
	private static int LocatorDestroyStaticBody( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, false );

		FixedArray3<BodyId> platforms = default;
		FixedArray3<BodyId> boxes = default;
		for ( int i = 0; i < 3; ++i )
		{
			float x = 10.0f * i;
			platforms[i] = CreateLocatorGround( worldId, x, 2.0f );
			boxes[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, x, 0.5f );
		}

		StepLocatorWorld( worldId, 60 );
		float restY = (float)Body_GetPosition( boxes[2] ).y;

		DestroyBody( platforms[0] );
		StepLocatorWorld( worldId, 40 );

		ENSURE_SMALL( (float)Body_GetPosition( boxes[2] ).y - restY, 0.01f );
		ENSURE( (float)Body_GetPosition( boxes[0] ).y < -1.0f );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// A kinematic that never sleeps keeps a non touching contact to a body that falls asleep. That
	// contact stays in the awake set with its sleeping side on the cold path, then the kinematic closes
	// the gap and wakes it.
	private static int LocatorSleepingNeighbor( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, true );
		CreateLocatorGround( worldId, 0.0f, 10.0f );

		BodyId sleeper = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 0.5f );

		// The fat boxes overlap across a tenth of a meter, well past the speculative distance
		BodyId mover = CreateLocatorBox( worldId, BodyType.kinematicBody, 1.1f, 0.5f );
		Body_EnableSleeping( mover, false );

		ENSURE( StepUntilAsleep( worldId, sleeper, 600 ) );
		ENSURE( Body_IsAwake( mover ) );

		FixedArray4<ContactData> contactData = default;
		ENSURE( Body_GetContactCapacity( mover ) == 1 );
		ENSURE( Body_GetContactData( mover, contactData, 4 ) == 0 );

		StepLocatorWorld( worldId, 30 );
		ENSURE( Body_IsAwake( sleeper ) == false );

		Body_SetLinearVelocity( mover, new Vec2( -2.0f, 0.0f ) );
		StepLocatorWorld( worldId, 10 );
		ENSURE( Body_IsAwake( sleeper ) );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// Type change, disable and enable all route through the body transfer, which pulls a body sim out of
	// the middle of a set while other bodies hold contacts.
	private static int LocatorTypeChange( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, false );
		CreateLocatorGround( worldId, 0.0f, 20.0f );

		BodyId subject = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 0.5f );
		CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 1.5f );

		FixedArray3<BodyId> neighbors = default;
		for ( int i = 0; i < 3; ++i )
		{
			neighbors[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, 10.0f, 0.5f + i );
		}

		StepLocatorWorld( worldId, 120 );
		float restY = (float)Body_GetPosition( neighbors[2] ).y;

		Body_SetType( subject, BodyType.staticBody );
		StepLocatorWorld( worldId, 30 );
		ENSURE( Body_GetType( subject ) == BodyType.staticBody );

		Body_SetType( subject, BodyType.dynamicBody );
		StepLocatorWorld( worldId, 30 );
		ENSURE( Body_GetType( subject ) == BodyType.dynamicBody );

		Body_Disable( subject );
		StepLocatorWorld( worldId, 10 );
		ENSURE( Body_IsEnabled( subject ) == false );

		Body_Enable( subject );
		StepLocatorWorld( worldId, 30 );
		ENSURE( Body_IsEnabled( subject ) );

		ENSURE_SMALL( (float)Body_GetPosition( neighbors[2] ).y - restY, 0.01f );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// A joint between two sleeping bodies merges their sets instead of waking them, while an awake
	// kinematic holds a non touching contact to one of them across the merge.
	private static int LocatorSetMerge( int workerCount, ref ulong hash )
	{
		WorldId worldId = CreateLocatorWorld( workerCount, true );
		CreateLocatorGround( worldId, 0.0f, 20.0f );

		BodyId left = CreateLocatorBox( worldId, BodyType.dynamicBody, -5.0f, 0.5f );
		BodyId right = CreateLocatorBox( worldId, BodyType.dynamicBody, 5.0f, 0.5f );

		BodyId watcher = CreateLocatorBox( worldId, BodyType.kinematicBody, -3.9f, 0.5f );
		Body_EnableSleeping( watcher, false );

		ENSURE( StepUntilAsleep( worldId, left, 600 ) );
		ENSURE( Body_IsAwake( right ) == false );
		ENSURE( Body_IsAwake( watcher ) );
		ENSURE( Body_GetContactCapacity( watcher ) == 1 );

		DistanceJointDef jointDef = DefaultDistanceJointDef();
		jointDef.@base.bodyIdA = left;
		jointDef.@base.bodyIdB = right;
		jointDef.length = 10.0f;
		JointId jointId = CreateDistanceJoint( worldId, ref jointDef );

		ENSURE( Body_IsAwake( left ) == false );
		ENSURE( Body_IsAwake( right ) == false );

		StepLocatorWorld( worldId, 30 );
		ENSURE( Body_IsAwake( left ) == false );

		Body_SetAwake( left, true );
		StepLocatorWorld( worldId, 10 );
		ENSURE( Body_IsAwake( right ) );

		DestroyJoint( jointId );
		StepLocatorWorld( worldId, 10 );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// A body moving far enough in one step carries the fast flag, which the collide task now reads off
	// the sim. A public setter mid flight makes sure a flag sync does not drop it.
	private static int LocatorFastBody( int workerCount, ref ulong hash )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );

		CreateLocatorBox( worldId, BodyType.staticBody, 10.0f, 0.0f );
		BodyId bullet = CreateLocatorBox( worldId, BodyType.dynamicBody, 0.0f, 0.0f );
		Body_SetLinearVelocity( bullet, new Vec2( 40.0f, 0.0f ) );

		// A step carries it two thirds of a meter, well past half its minimum extent
		StepLocatorWorld( worldId, 4 );
		ENSURE( (float)Body_GetPosition( bullet ).x > 2.0f );

		Body_EnableSleeping( bullet, false );
		StepLocatorWorld( worldId, 4 );

		Body_EnableSleeping( bullet, true );
		StepLocatorWorld( worldId, 30 );

		// It stopped at the wall instead of tunneling through
		ENSURE( (float)Body_GetPosition( bullet ).x < 9.1f );
		ENSURE( (float)Body_GetPosition( bullet ).x > 8.5f );

		hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return 0;
	}

	// A row of boxes at rest in zero gravity is unaffected by sleeping, so the run that sleeps and wakes
	// on impact has to land where the run that never sleeps lands. Positions rather than the state hash
	// because the hash folds in set and local index, which sleeping is entitled to change.
	private static int LocatorSleepParity( int workerCount )
	{
		float[,] finalX = new float[2, 3];

		for ( int pass = 0; pass < 2; ++pass )
		{
			WorldDef worldDef = DefaultWorldDef();
			worldDef.gravity = Vec2_zero;
			worldDef.enableSleep = pass == 0;
			worldDef.workerCount = workerCount;
			WorldId worldId = CreateWorld( ref worldDef );

			FixedArray3<BodyId> boxes = default;
			for ( int i = 0; i < 3; ++i )
			{
				boxes[i] = CreateLocatorBox( worldId, BodyType.dynamicBody, (float)i, 0.0f );
			}

			StepLocatorWorld( worldId, 60 );
			ENSURE( Body_IsAwake( boxes[0] ) == ( pass == 1 ) );

			BodyId impactor = CreateLocatorBox( worldId, BodyType.dynamicBody, -5.0f, 0.0f );
			Body_SetLinearVelocity( impactor, new Vec2( 5.0f, 0.0f ) );

			StepLocatorWorld( worldId, 120 );
			ENSURE( Body_IsAwake( boxes[2] ) );

			for ( int i = 0; i < 3; ++i )
			{
				finalX[pass, i] = (float)Body_GetPosition( boxes[i] ).x;
			}

			DestroyWorld( worldId );
		}

		for ( int i = 0; i < 3; ++i )
		{
			ENSURE_SMALL( finalX[0, i] - finalX[1, i], 0.01f );
		}

		return 0;
	}

	private static int BodySimLocatorTest()
	{
		int[] workerCounts = { 1, 4 };
		FixedArray2<ulong> hashes = default;

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorSleepWake( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorDestroyAwakeBody( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorDestroyStaticBody( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorSleepingNeighbor( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorTypeChange( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorSetMerge( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorFastBody( workerCounts[i], ref hashes[i] ) == 0 );
		}
		ENSURE( hashes[0] == hashes[1] );

		for ( int i = 0; i < 2; ++i )
		{
			ENSURE( LocatorSleepParity( workerCounts[i] ) == 0 );
		}

		return 0;
	}

	private static int BroadPhasePairsTest()
	{
		int[] workerCounts = { 1, 4 };
		for ( int i = 0; i < 2; ++i )
		{
			int workerCount = workerCounts[i];
			ENSURE( PairCreation( workerCount ) == 0 );
			ENSURE( PairTeleport( workerCount ) == 0 );
			ENSURE( PairRefilter( workerCount ) == 0 );
			ENSURE( PairCollideConnected( workerCount ) == 0 );
			ENSURE( PairFilters( workerCount ) == 0 );
			ENSURE( PairEnable( workerCount ) == 0 );
			ENSURE( PairSetType( workerCount ) == 0 );
			ENSURE( PairSetBullet( workerCount ) == 0 );
			ENSURE( PairRebuildStatic( workerCount ) == 0 );
			ENSURE( PairSleepEscape( workerCount, false ) == 0 );
			ENSURE( PairSleepEscape( workerCount, true ) == 0 );
			ENSURE( PairSleepingStatic( workerCount ) == 0 );
		}

		return 0;
	}

	// A joint with no dynamic body lives in the static set even when one body is an awake kinematic.
	// Disabling and enabling that body has to respect this, whether the joint was created that way or
	// got there through body type changes. Box3D issue #164.
	private static int NonDynamicJointDisableTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = Vec2_zero;
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyIdA = CreateBody( worldId, ref bodyDef );
		bodyDef.position = new Pos { x = 2.0f, y = 0.0f };
		BodyId bodyIdB = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( bodyIdA, ref shapeDef, ref box );
		CreatePolygonShape( bodyIdB, ref shapeDef, ref box );

		WeldJointDef jointDef = DefaultWeldJointDef();
		jointDef.@base.bodyIdA = bodyIdA;
		jointDef.@base.bodyIdB = bodyIdB;
		JointId weldId = CreateWeldJoint( worldId, ref jointDef );

		Body_SetType( bodyIdA, BodyType.staticBody );
		Body_SetType( bodyIdB, BodyType.kinematicBody );
		World_Step( worldId, 1.0f / 60.0f, 4 );

		Body_Disable( bodyIdB );
		World_Step( worldId, 1.0f / 60.0f, 4 );
		Body_Enable( bodyIdB );
		World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( Joint_IsValid( weldId ) );

		World world = GetWorldFromId( worldId );
		ENSURE( GetJointFullId( world, weldId ).setIndex == b2_staticSet );

		Body_SetType( bodyIdB, BodyType.dynamicBody );
		World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( GetJointFullId( world, weldId ).setIndex == b2_awakeSet );

		// Created directly between a static and a kinematic body
		bodyDef.type = BodyType.kinematicBody;
		bodyDef.position = new Pos { x = 4.0f, y = 0.0f };
		BodyId bodyIdC = CreateBody( worldId, ref bodyDef );
		CreatePolygonShape( bodyIdC, ref shapeDef, ref box );

		RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();
		revoluteDef.@base.bodyIdA = bodyIdA;
		revoluteDef.@base.bodyIdB = bodyIdC;
		JointId revoluteId = CreateRevoluteJoint( worldId, ref revoluteDef );

		Body_Disable( bodyIdC );
		Body_Enable( bodyIdC );
		World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( GetJointFullId( world, revoluteId ).setIndex == b2_staticSet );

		DestroyWorld( worldId );
		return 0;
	}

	internal static int WorldTest()
	{
		RUN_SUBTEST( HelloWorld );
		RUN_SUBTEST( EmptyWorld );
		RUN_SUBTEST( DestroyAllBodiesWorld );
		RUN_SUBTEST( TestIsValid );
		RUN_SUBTEST( TestWorldRecycle );
		RUN_SUBTEST( TestWorldCoverage );
		RUN_SUBTEST( TestSensor );
		RUN_SUBTEST( TestSetWorkerCount );
		RUN_SUBTEST( ChainSegmentShapeTest );
		RUN_SUBTEST( SetBulletDriftTest );
		RUN_SUBTEST( DeferredMassFlagSyncTest );
		RUN_SUBTEST( EnableSleepFlagSyncTest );
		RUN_SUBTEST( EnableContactRecyclingTest );
		RUN_SUBTEST( PreSolveRejectRecycleTest );
		RUN_SUBTEST( EnlargedProxyDestroyedTest );
		RUN_SUBTEST( BroadPhasePairsTest );
		RUN_SUBTEST( BodySimLocatorTest );
		RUN_SUBTEST( NonDynamicJointDisableTest );

		return 0;
	}

}
