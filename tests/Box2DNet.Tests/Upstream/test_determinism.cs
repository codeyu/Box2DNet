// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;
using static Box2DNet.Tests.DeterminismShared;

namespace Box2DNet.Tests;

public static partial class DeterminismTests
{
#if BOX2D_PROFILE
#else
	// #define TracyCFrameMark
#endif

	private static int SingleMultithreadingTest( int workerCount )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = workerCount;

		WorldId worldId = CreateWorld( ref worldDef );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;
		int stepLimit = 500;
		for ( int i = 0; i < stepLimit; ++i )
		{
			int subStepCount = 4;
			World_Step( worldId, timeStep, subStepCount );
			bool done = UpdateFallingHinges( worldId, ref data );
			if ( done )
			{
				break;
			}
		}

		DestroyWorld( worldId );

		if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
		{
			Console.WriteLine( $"  workers={workerCount} sleepStep={data.sleepStep} hash=0x{data.hash:X}" );
		}

		ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
		ENSURE( data.hash == EXPECTED_HASH );

		DestroyFallingHinges( ref data );

		return 0;
	}

	// Test multithreaded determinism.
	private static int MultithreadingTest()
	{
		for ( int run = 0; run < 3; ++run )
		{
			for ( int workerCount = 1; workerCount < 16; workerCount += 2 )
			{
				int result = SingleMultithreadingTest( workerCount );
				ENSURE( result == 0 );
			}

			for ( int workerCount = 32; workerCount >= 0; workerCount -= 5 )
			{
				int result = SingleMultithreadingTest( workerCount );
				ENSURE( result == 0 );
			}
		}

		return 0;
	}

	// Test determinism using the built-in scheduler (no external task system).
	private static int BuiltInSchedulerTest()
	{
		for ( int workerCount = 2; workerCount <= 8; workerCount += 2 )
		{
			WorldDef worldDef = DefaultWorldDef();
			worldDef.workerCount = workerCount;

			WorldId worldId = CreateWorld( ref worldDef );

			FallingHingeData data = CreateFallingHinges( worldId );

			float timeStep = 1.0f / 60.0f;
			int stepLimit = 1000;
			for ( int i = 0; i < stepLimit; ++i )
			{
				int subStepCount = 4;
				World_Step( worldId, timeStep, subStepCount );

				bool done = UpdateFallingHinges( worldId, ref data );
				if ( done )
				{
					break;
				}
			}

			DestroyWorld( worldId );

			if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
			{
				Console.WriteLine( $"  built-in scheduler workers={workerCount} sleepStep={data.sleepStep} hash=0x{data.hash:X}" );
			}

			ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
			ENSURE( data.hash == EXPECTED_HASH );

			DestroyFallingHinges( ref data );
		}

		return 0;
	}

	// Test cross-platform determinism.
	private static int CrossPlatformTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;

		bool done = false;
		while ( done == false )
		{
			int subStepCount = 4;
			World_Step( worldId, timeStep, subStepCount );
			done = UpdateFallingHinges( worldId, ref data );
		}

		if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
		{
			Console.WriteLine( $"  cross-platform sleepStep={data.sleepStep} hash=0x{data.hash:X}" );
		}

		ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
		ENSURE( data.hash == EXPECTED_HASH );

		DestroyFallingHinges( ref data );

		DestroyWorld( worldId );

		return 0;
	}

	private static int SingleFallingHingeTest( int workerCount, bool sse2Fallback )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );
		World_EnableSSE2Fallback( worldId, sse2Fallback );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;
		bool done = false;
		while ( done == false )
		{
			World_Step( worldId, timeStep, 4 );
			done = UpdateFallingHinges( worldId, ref data );
		}

		DestroyWorld( worldId );

		if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
		{
			Console.WriteLine( $"  sse2Fallback={( sse2Fallback ? 1 : 0 )} workers={workerCount} sleepStep={data.sleepStep} hash=0x{data.hash:X}" );
		}

		ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
		ENSURE( data.hash == EXPECTED_HASH );

		DestroyFallingHinges( ref data );

		return 0;
	}

	private static WorldId CreateBouncePile( int workerCount, bool sse2Fallback )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );
		World_EnableSSE2Fallback( worldId, sse2Fallback );

		BodyDef groundDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref groundDef );
		ShapeDef groundShapeDef = DefaultShapeDef();
		groundShapeDef.material.tangentSpeed = 0.5f;
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref groundShapeDef, ref segment );

		Polygon box = MakeRoundedBox( 0.4f, 0.3f, 0.05f );
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.35f };

		for ( int i = 0; i < 300; ++i )
		{
			int column = i % 30;
			int row = i / 30;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -15.0f + 1.0f * (float)column + 0.1f * (float)( row % 2 ), y = 1.0f + 0.9f * (float)row };
			bodyDef.rotation = MakeRot( 0.3f * (float)i );
			BodyId bodyId = CreateBody( worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.restitution = ( i % 3 ) == 0 ? 0.0f : 0.25f * (float)( i % 5 );
			shapeDef.material.friction = 0.2f + 0.1f * (float)( i % 7 );
			shapeDef.material.rollingResistance = ( i % 4 ) == 0 ? 0.1f : 0.0f;
			shapeDef.enableHitEvents = ( i % 2 ) == 0;

			if ( i % 3 == 1 )
			{
				CreateCircleShape( bodyId, ref shapeDef, ref circle );
			}
			else
			{
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}

		return worldId;
	}

	// Width 4 and the native width must produce bitwise identical simulations.
	private static int SIMDWidthTest()
	{
		if ( IsAVX2Available() == false )
		{
			Console.WriteLine( $"  subtest skipped: SIMDWidthTest, native SIMD width is 4" );
			return 0;
		}

		for ( int workerCount = 0; workerCount <= 4; workerCount += 4 )
		{
			ENSURE( SingleFallingHingeTest( workerCount, true ) == 0 );

			WorldId narrowId = CreateBouncePile( workerCount, true );
			WorldId wideId = CreateBouncePile( workerCount, false );

			float timeStep = 1.0f / 60.0f;
			for ( int i = 0; i < 300; ++i )
			{
				World_Step( narrowId, timeStep, 4 );
				World_Step( wideId, timeStep, 4 );
				ENSURE( World_GetStateHash( narrowId ) == World_GetStateHash( wideId ) );
			}

			DestroyWorld( narrowId );
			DestroyWorld( wideId );
		}

		return 0;
	}

	internal static int DeterminismTest()
	{
		RUN_SUBTEST( MultithreadingTest );
		RUN_SUBTEST( BuiltInSchedulerTest );
		RUN_SUBTEST( CrossPlatformTest );
		RUN_SUBTEST( SIMDWidthTest );

		return 0;
	}

}
