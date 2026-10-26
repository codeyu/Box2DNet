// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;
using static Box2DNet.Tests.HumanShared;

namespace Box2DNet.Tests;

public static partial class Benchmarks
{
#if NDEBUG
	private const int BENCHMARK_DEBUG = 0;
#else
	private const int BENCHMARK_DEBUG = 1;
#endif

	internal static Capacity GetJointGridCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 7, dynamicShapeCount = BENCHMARK_DEBUG ? 440 : 11000, staticBodyCount = 7, dynamicBodyCount = BENCHMARK_DEBUG ? 440 : 11000, arenaByteCount = ( BENCHMARK_DEBUG ? 34 : 120 ) * 1024 };

		return c;
	}

	internal static void CreateJointGrid( WorldId worldId )
	{
		World_EnableSleeping( worldId, false );

		int N = BENCHMARK_DEBUG ? 20 : 100;

		// Allocate to avoid huge stack usage
		Span<BodyId> bodies = malloc( N * N * Unsafe.SizeOf<BodyId>() );
		int index = 0;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = ~2u;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.4f };

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.@base.drawScale = 0.4f;

		BodyDef bodyDef = DefaultBodyDef();

		for ( int k = 0; k < N; ++k )
		{
			for ( int i = 0; i < N; ++i )
			{
				float fk = (float)k;
				float fi = (float)i;

				if ( k >= N / 2 - 3 && k <= N / 2 + 3 && i == 0 )
				{
					bodyDef.type = BodyType.staticBody;
				}
				else
				{
					bodyDef.type = BodyType.dynamicBody;
				}

				bodyDef.position = new Pos { x = fk, y = -fi };

				BodyId body = CreateBody( worldId, ref bodyDef );

				CreateCircleShape( body, ref shapeDef, ref circle );

				if ( i > 0 )
				{
					jointDef.@base.bodyIdA = bodies[index - 1];
					jointDef.@base.bodyIdB = body;
					jointDef.@base.localFrameA.p = new Vec2( 0.0f, -0.5f );
					jointDef.@base.localFrameB.p = new Vec2( 0.0f, 0.5f );
					CreateRevoluteJoint( worldId, ref jointDef );
				}

				if ( k > 0 )
				{
					jointDef.@base.bodyIdA = bodies[index - N];
					jointDef.@base.bodyIdB = body;
					jointDef.@base.localFrameA.p = new Vec2( 0.5f, 0.0f );
					jointDef.@base.localFrameB.p = new Vec2( -0.5f, 0.0f );
					CreateRevoluteJoint( worldId, ref jointDef );
				}

				bodies[index++] = body;
			}
		}

		free( bodies );
	}

	internal static Capacity GetLargePyramidCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 1, dynamicShapeCount = BENCHMARK_DEBUG ? 240 : 5600, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 240 : 5600, contactCount = BENCHMARK_DEBUG ? 740 : 19000, arenaByteCount = ( BENCHMARK_DEBUG ? 130 : 2800 ) * 1024 };

		return c;
	}

	internal static void CreateLargePyramid( WorldId worldId )
	{
		World_EnableSleeping( worldId, false );

		int baseCount = BENCHMARK_DEBUG ? 20 : 100;

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( worldId, ref bodyDefInner );

			Polygon boxInner = MakeBox( 100.0f, 1.0f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		// bodyDef.enableSleep = false;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;

		float a = 0.5f;
		Polygon box = MakeSquare( a );

		float shift = 1.0f * a;

		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * shift;

			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - a * baseCount;

				bodyDef.position = new Pos { x = x, y = y };

				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	private static void CreateSmallPyramid( WorldId worldId, int baseCount, float extent, float centerX, float baseY )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();

		Polygon box = MakeSquare( extent );

		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * extent + baseY;

			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * extent + 2.0f * ( j - i ) * extent + centerX - 0.5f;
				bodyDef.position = new Pos { x = x, y = y };

				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	internal static Capacity GetManyPyramidsCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = BENCHMARK_DEBUG ? 5 : 22, dynamicShapeCount = BENCHMARK_DEBUG ? 1600 : 25000, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 1600 : 25000, contactCount = BENCHMARK_DEBUG ? 4600 : 73000, arenaByteCount = ( BENCHMARK_DEBUG ? 700 : 11000 ) * 1024 };

		return c;
	}

	internal static void CreateManyPyramids( WorldId worldId )
	{
		World_EnableSleeping( worldId, false );

		int baseCount = 10;
		float extent = 0.5f;
		int rowCount = BENCHMARK_DEBUG ? 5 : 20;
		int columnCount = BENCHMARK_DEBUG ? 5 : 20;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		float groundDeltaY = 2.0f * extent * ( baseCount + 1.0f );
		float groundWidth = 2.0f * extent * columnCount * ( baseCount + 1.0f );
		ShapeDef shapeDef = DefaultShapeDef();

		float groundY = 0.0f;

		for ( int i = 0; i < rowCount; ++i )
		{
			Segment segment = new Segment { point1 = new( -0.5f * groundWidth, groundY ), point2 = new( 0.5f * groundWidth, groundY ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
			groundY += groundDeltaY;
		}

		float baseWidth = 2.0f * extent * baseCount;
		float baseY = 0.0f;

		for ( int i = 0; i < rowCount; ++i )
		{
			for ( int j = 0; j < columnCount; ++j )
			{
				float centerX = -0.5f * groundWidth + j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
				CreateSmallPyramid( worldId, baseCount, extent, centerX, baseY );
			}

			baseY += groundDeltaY;
		}
	}

#if NDEBUG
	enum RainConstants
	{
		RAIN_ROW_COUNT = 5,
		RAIN_COLUMN_COUNT = 40,
		RAIN_GROUP_SIZE = 5,
	};
#else
	enum RainConstants
	{
		RAIN_ROW_COUNT = 3,
		RAIN_COLUMN_COUNT = 10,
		RAIN_GROUP_SIZE = 2,
	};
#endif

	internal struct Group
	{
		public /*TODO fixed RAIN_GROUP_SIZE*/ Human[] humans;
	}

	internal struct RainData
	{
		public /*TODO fixed RAIN_ROW_COUNT * RAIN_COLUMN_COUNT*/ Group[] groups;
		public float gridSize;
		public int gridCount;
		public int columnCount;
		public int columnIndex;
	}

	private static RainData g_rainData;

	internal static Capacity GetRainCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = BENCHMARK_DEBUG ? 670 : 2800, dynamicShapeCount = BENCHMARK_DEBUG ? 860 : 15000, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 730 : 13000, contactCount = BENCHMARK_DEBUG ? 1600 : 29000, arenaByteCount = ( BENCHMARK_DEBUG ? 150 : 1900 ) * 1024 };

		return c;
	}

	internal static void CreateRain( WorldId worldId )
	{
		g_rainData = default;

		g_rainData.gridSize = 0.5f;
		g_rainData.gridCount = BENCHMARK_DEBUG ? 200 : 500;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			float y = 0.0f;
			float width = g_rainData.gridSize;
			float height = g_rainData.gridSize;

			for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
			{
				float x = -0.5f * g_rainData.gridCount * g_rainData.gridSize;
				for ( int j = 0; j <= g_rainData.gridCount; ++j )
				{
					Polygon box = MakeOffsetBox( 0.5f * width, 0.5f * height, new Vec2( x, y ), Rot_identity );
					CreatePolygonShape( groundId, ref shapeDef, ref box );

					// Segment segment = new Segment { point1 = new( x - 0.5f * width, y ), point2 = new( x + 0.5f * width, y ) };
					// CreateSegmentShape( groundId, ref shapeDef, ref segment );

					x += g_rainData.gridSize;
				}

				y += 45.0f;
			}
		}

		g_rainData.columnCount = 0;
		g_rainData.columnIndex = 0;
	}

	internal static void CreateGroup( WorldId worldId, int rowIndex, int columnIndex )
	{
		assert( rowIndex < RAIN_ROW_COUNT && columnIndex < RAIN_COLUMN_COUNT );

		int groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

		float span = g_rainData.gridCount * g_rainData.gridSize;
		float groupDistance = 1.0f * span / RAIN_COLUMN_COUNT;

		Pos position = default;
		position.x = -0.5f * span + groupDistance * ( columnIndex + 0.5f );
		position.y = 40.0f + 45.0f * rowIndex;

		float scale = 1.0f;
		float jointFriction = 0.05f;
		float jointHertz = 5.0f;
		float jointDamping = 0.5f;

		for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
		{
			Human* human = g_rainData.groups[groupIndex].humans + i;
			CreateHuman( ref human, worldId, position, scale, jointFriction, jointHertz, jointDamping, i + 1, null, false );
			position.x += 0.5f;
		}
	}

	internal static void DestroyGroup( int rowIndex, int columnIndex )
	{
		assert( rowIndex < RAIN_ROW_COUNT && columnIndex < RAIN_COLUMN_COUNT );

		int groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

		for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
		{
			DestroyHuman( g_rainData.groups[groupIndex].humans + i );
		}
	}

	internal static float StepRain( WorldId worldId, int stepCount )
	{
		int delay = BENCHMARK_DEBUG ? 0x1F : 0x7;

		if ( ( stepCount & delay ) == 0 )
		{
			if ( g_rainData.columnCount < RAIN_COLUMN_COUNT )
			{
				for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
				{
					CreateGroup( worldId, i, g_rainData.columnCount );
				}

				g_rainData.columnCount += 1;
			}
			else
			{
				for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
				{
					DestroyGroup( i, g_rainData.columnIndex );
					CreateGroup( worldId, i, g_rainData.columnIndex );
				}

				g_rainData.columnIndex = ( g_rainData.columnIndex + 1 ) % RAIN_COLUMN_COUNT;
			}
		}

		return 0.0f;
	}

	private const int SPINNER_POINT_COUNT = 360;
	internal struct SpinnerData
	{
		public JointId spinnerId;
	}

	private static SpinnerData g_spinnerData;

	internal static Capacity GetSpinnerCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 400, dynamicShapeCount = BENCHMARK_DEBUG ? 550 : 6700, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 550 : 6700, contactCount = BENCHMARK_DEBUG ? 2700 : 41000, arenaByteCount = ( BENCHMARK_DEBUG ? 360 : 4600 ) * 1024 };

		return c;
	}

	internal static void CreateSpinner( WorldId worldId )
	{
		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( worldId, ref bodyDefInner );

			Span<Vec2> points = stackalloc Vec2[SPINNER_POINT_COUNT];

			Rot q = MakeRot( -2.0f * Pi / SPINNER_POINT_COUNT );
			Vec2 p = new Vec2( 40.0f, 0.0f );
			for ( int i = 0; i < SPINNER_POINT_COUNT; ++i )
			{
				points[i] = new Vec2( p.x, p.y + 32.0f );
				p = RotateVector( q, p );
			}

			SurfaceMaterial material = default;
			material.friction = 0.1f;

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = SPINNER_POINT_COUNT;
			chainDef.isLoop = true;
			chainDef.materials = &material;
			chainDef.materialCount = 1;

			CreateChain( groundId, ref chainDef );
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.type = BodyType.dynamicBody;
			bodyDefInner.position = new Pos { x = 0.0, y = 12.0f };
			bodyDefInner.enableSleep = false;

			BodyId spinnerId = CreateBody( worldId, ref bodyDefInner );

			Polygon box = MakeRoundedBox( 0.4f, 20.0f, 0.2f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.material.friction = 0.0f;
			CreatePolygonShape( spinnerId, ref shapeDefInner, ref box );

			float motorSpeed = 5.0f;
			// float maxMotorTorque = 100.0f * 40000.0f;
			float maxMotorTorque = FLT_MAX;
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = spinnerId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, bodyDefInner.position );
			jointDef.enableMotor = true;
			jointDef.motorSpeed = motorSpeed;
			jointDef.maxMotorTorque = maxMotorTorque;

			g_spinnerData.spinnerId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		Capsule capsule = new Capsule { center1 = new( -0.25f, 0.0f ), center2 = new( 0.25f, 0.0f ), radius = 0.25f };
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.35f };
		Polygon square = MakeSquare( 0.35f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.1f;
		shapeDef.material.restitution = 0.1f;
		shapeDef.density = 0.25f;

		int bodyCount = BENCHMARK_DEBUG ? 499 : 2 * 3038;

		float x = -23.0f, y = 2.0f;
		for ( int i = 0; i < bodyCount; ++i )
		{
			bodyDef.position = new Pos { x = x, y = y };
			BodyId bodyId = CreateBody( worldId, ref bodyDef );

			int remainder = i % 3;
			if ( remainder == 0 )
			{
				CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
			}
			else if ( remainder == 1 )
			{
				CreateCircleShape( bodyId, ref shapeDef, ref circle );
			}
			else if ( remainder == 2 )
			{
				CreatePolygonShape( bodyId, ref shapeDef, ref square );
			}

			x += 0.5f;

			if ( x >= 23.0f )
			{
				x = -23.0f;
				y += 0.5f;
			}
		}
	}

	internal static float StepSpinner( WorldId worldId, int stepCount )
	{
		worldId;
		stepCount;

		return RevoluteJoint_GetCurrentAngle( g_spinnerData.spinnerId );
	}

	internal static Capacity GetSmashCapacity()
	{
		Capacity c = new Capacity { dynamicShapeCount = BENCHMARK_DEBUG ? 230 : 11000, dynamicBodyCount = BENCHMARK_DEBUG ? 230 : 11000, contactCount = BENCHMARK_DEBUG ? 2300 : 55000, arenaByteCount = ( BENCHMARK_DEBUG ? 280 : 4900 ) * 1024 };

		return c;
	}

	internal static void CreateSmash( WorldId worldId )
	{
		World_SetGravity( worldId, Vec2_zero );

		{
			Polygon boxInner = MakeBox( 4.0f, 4.0f );

			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.type = BodyType.dynamicBody;
			bodyDefInner.position = new Pos { x = -20.0f, y = 0.0f };
			bodyDefInner.linearVelocity = new Vec2( 40.0f, 0.0f );
			BodyId bodyId = CreateBody( worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.density = 8.0f;
			CreatePolygonShape( bodyId, ref shapeDefInner, ref boxInner );
		}

		float d = 0.4f;
		Polygon box = MakeSquare( 0.5f * d );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.isAwake = false;

		ShapeDef shapeDef = DefaultShapeDef();

		int columns = BENCHMARK_DEBUG ? 20 : 120;
		int rows = BENCHMARK_DEBUG ? 10 : 80;

		for ( int i = 0; i < columns; ++i )
		{
			for ( int j = 0; j < rows; ++j )
			{
				bodyDef.position.x = i * d + 30.0f;
				bodyDef.position.y = ( j - rows / 2.0f ) * d;
				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	internal static Capacity GetTumblerCapacity()
	{
		Capacity c = new Capacity { dynamicShapeCount = BENCHMARK_DEBUG ? 450 : 2300, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 450 : 2300, contactCount = BENCHMARK_DEBUG ? 2400 : 13000, arenaByteCount = ( BENCHMARK_DEBUG ? 220 : 1100 ) * 1024 };

		return c;
	}

	internal static void CreateTumbler( WorldId worldId )
	{
		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( worldId, ref bodyDefInner );
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.type = BodyType.dynamicBody;
			bodyDefInner.position = new Pos { x = 0.0f, y = 10.0f };
			BodyId bodyId = CreateBody( worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.density = 50.0f;

			Polygon polygonInner = default;
			polygonInner = MakeOffsetBox( 0.5f, 10.0f, new Vec2( 10.0f, 0.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );
			polygonInner = MakeOffsetBox( 0.5f, 10.0f, new Vec2( -10.0f, 0.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );
			polygonInner = MakeOffsetBox( 10.0f, 0.5f, new Vec2( 0.0f, 10.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );
			polygonInner = MakeOffsetBox( 10.0f, 0.5f, new Vec2( 0.0f, -10.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );

			float motorSpeed = 25.0f;

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = new Vec2( 0.0f, 10.0f );
			jointDef.@base.localFrameB.p = new Vec2( 0.0f, 0.0f );
			jointDef.motorSpeed = ( Pi / 180.0f ) * motorSpeed;
			jointDef.maxMotorTorque = 1e8f;
			jointDef.enableMotor = true;

			CreateRevoluteJoint( worldId, ref jointDef );
		}

		int gridCount = BENCHMARK_DEBUG ? 20 : 45;

		Polygon polygon = MakeBox( 0.125f, 0.125f );
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();

		float y = -0.2f * gridCount + 10.0f;
		for ( int i = 0; i < gridCount; ++i )
		{
			float x = -0.2f * gridCount;

			for ( int j = 0; j < gridCount; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( worldId, ref bodyDef );

				CreatePolygonShape( bodyId, ref shapeDef, ref polygon );

				x += 0.4f;
			}

			y += 0.4f;
		}
	}

	internal static Capacity GetWasherCapacity()
	{
		Capacity c = new Capacity { dynamicShapeCount = BENCHMARK_DEBUG ? 440 : 9000, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 450 : 9000, contactCount = BENCHMARK_DEBUG ? 2300 : 52000, arenaByteCount = ( BENCHMARK_DEBUG ? 300 : 6000 ) * 1024 };

		return c;
	}

	internal static void CreateWasher( WorldId worldId )
	{
		bool kinematic = true;

		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( worldId, ref bodyDefInner );
		}

		{
			float motorSpeed = 25.0f;

			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.position = new Pos { x = 0.0f, y = 10.0f };

			if ( kinematic == true )
			{
				bodyDefInner.type = BodyType.kinematicBody;
				bodyDefInner.angularVelocity = ( Pi / 180.0f ) * motorSpeed;
				bodyDefInner.linearVelocity = new Vec2( 0.001f, -0.002f );
			}
			else
			{
				bodyDefInner.type = BodyType.dynamicBody;
			}

			BodyId bodyId = CreateBody( worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();

			float r0 = 14.0f;
			float r1 = 16.0f;
			float r2 = 18.0f;

			float angle = Pi / 18.0f;
			Rot q = new Rot( cosf( angle ), sinf( angle ) );
			Rot qo = new Rot( cosf( 0.1f * angle ), sinf( 0.1f * angle ) );
			Vec2 u1 = new Vec2( 1.0f, 0.0f );
			for ( int i = 0; i < 36; ++i )
			{
				Vec2 u2 = default;
				if ( i == 35 )
				{
					u2 = new Vec2( 1.0f, 0.0f );
				}
				else
				{
					u2 = RotateVector( q, u1 );
				}

				{
					Vec2 a1 = InvRotateVector( qo, u1 );
					Vec2 a2 = RotateVector( qo, u2 );

					Vec2 p1 = MulSV( r1, a1 );
					Vec2 p2 = MulSV( r2, a1 );
					Vec2 p3 = MulSV( r1, a2 );
					Vec2 p4 = MulSV( r2, a2 );

					Vec2[] points = { p1, p2, p3, p4 };
					Hull hull = ComputeHull( points, 4 );

					Polygon polygonInner = MakePolygon( ref hull, 0.0f );
					CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );
				}

				if ( i % 9 == 0 )
				{
					Vec2 p1 = MulSV( r0, u1 );
					Vec2 p2 = MulSV( r1, u1 );
					Vec2 p3 = MulSV( r0, u2 );
					Vec2 p4 = MulSV( r1, u2 );

					Vec2[] points = { p1, p2, p3, p4 };
					Hull hull = ComputeHull( points, 4 );

					Polygon polygonInner = MakePolygon( ref hull, 0.0f );
					CreatePolygonShape( bodyId, ref shapeDefInner, ref polygonInner );
				}

				u1 = u2;
			}

			if ( kinematic == false )
			{
				RevoluteJointDef jointDef = DefaultRevoluteJointDef();
				jointDef.@base.bodyIdA = groundId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = new Vec2( 0.0f, 10.0f );
				jointDef.@base.localFrameB.p = new Vec2( 0.0f, 0.0f );
				jointDef.motorSpeed = ( Pi / 180.0f ) * motorSpeed;
				jointDef.maxMotorTorque = 1e8f;
				jointDef.enableMotor = true;

				CreateRevoluteJoint( worldId, ref jointDef );
			}
		}

		int gridCount = BENCHMARK_DEBUG ? 20 : 90;
		float a = 0.1f;

		Polygon polygon = MakeSquare( a );
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableHitEvents = true;

		float y = -1.1f * a * gridCount + 10.0f;
		for ( int i = 0; i < gridCount; ++i )
		{
			float x = -1.1f * a * gridCount;

			for ( int j = 0; j < gridCount; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( worldId, ref bodyDef );

				CreatePolygonShape( bodyId, ref shapeDef, ref polygon );

				x += 2.1f * a;
			}

			y += 2.1f * a;
		}
	}

	internal struct JunkyardData
	{
		public BodyId pusherId;
	}

	static JunkyardData g_junkyardData;

	internal static Capacity GetJunkyardCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 290, dynamicShapeCount = BENCHMARK_DEBUG ? 440 : 8800, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 450 : 8900, contactCount = BENCHMARK_DEBUG ? 1500 : 38000, arenaByteCount = ( BENCHMARK_DEBUG ? 170 : 3700 ) * 1024 };

		return c;
	}

	internal static void CreateJunkyard( WorldId worldId )
	{
		{
			float gridSize = 1.0f;

			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();

			float y = 0.0f;
			float x = -80.0f * gridSize;
			for ( int i = 0; i < 161; ++i )
			{
				Polygon boxInner = MakeOffsetBox( 0.55f * gridSize, 0.5f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
				x += gridSize;
			}

			y = gridSize;
			x = -80.0f * gridSize;
			for ( int i = 0; i < 50; ++i )
			{
				Polygon boxInner = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
				y += gridSize;
			}

			y = gridSize;
			x = 80.0f * gridSize;
			for ( int i = 0; i < 50; ++i )
			{
				Polygon boxInner = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
				y += gridSize;
			}
		}

		int columnCount = 200;
		int rowCount = BENCHMARK_DEBUG ? 2 : 40;

		float radius = 0.25f;
		Polygon polygon = default;
		{
			// Fibonacci sphere algorithm
			float phi = Pi * ( MathF.Sqrt( 5.0f ) - 1.0f );
			Span<Vec2> points = stackalloc Vec2[5];

			for ( int i = 0; i < 5; ++i )
			{
				float theta = phi * i;
				CosSin cs = ComputeCosSin( theta );
				points[i].x = radius * cs.cosine;
				points[i].y = radius * cs.sine;
			}

			Hull hull = ComputeHull( points, 5 );
			polygon = MakePolygon( ref hull, 0.0f );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();

		float side = -0.1f;
		float yStart = 15.0f;

		for ( int i = 0; i < columnCount; ++i )
		{
			float x = 1.5f * ( 2.0f * i - columnCount ) * radius;

			for ( int j = 0; j < rowCount; ++j )
			{
				float y = 4.0f * j * radius + yStart;

				bodyDef.position = new Pos { x = x + side, y = y };
				side = -side;

				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
			}
		}

		bodyDef.type = BodyType.kinematicBody;
		bodyDef.position = Pos_zero;
		g_junkyardData.pusherId = CreateBody( worldId, ref bodyDef );
		Polygon box = MakeOffsetBox( 2.0f, 4.0f, new Vec2( 0.0f, 4.0f ), Rot_identity );
		CreatePolygonShape( g_junkyardData.pusherId, ref shapeDef, ref box );
	}

	internal static float StepJunkyard( WorldId worldId, int stepCount )
	{
		float timeStep = 1.0f / 60.0f;
		float time = timeStep * stepCount;
		CosSin cs = ComputeCosSin( 0.2f * time );
		WorldTransform target = new WorldTransform { p = new Pos { x = 60.0f * cs.sine, y = 0.0f }, q = Rot_identity };
		Body_SetTargetTransform( g_junkyardData.pusherId, target, timeStep, true );
		return 0.0f;
	}

	private const int SLEEP_PYRAMID_COUNT = 10;
	internal struct SleepData
	{
		public BodyId[] bodyIdA;
		public BodyId[] bodyIdB;
	}

	static SleepData g_sleepData;

	// Reports two bodies that are certain to share a touching contact, so the filter joint below always
	// lands inside one island.
	private static void CreateSleepPyramid( WorldId worldId, int baseCount, float extent, float centerX, ref BodyId bodyIdA, ref BodyId bodyIdB )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.sleepThreshold = 1.0f;

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeSquare( extent );

		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * extent;

			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * extent + 2.0f * ( j - i ) * extent + centerX;
				bodyDef.position = new Pos { x = x, y = y };

				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );

				if ( i == 0 && j == 0 )
				{
					bodyIdA = bodyId;
				}
				else if ( i == 1 && j == 1 )
				{
					// Rests on the base body above, so the pair always has a touching contact
					bodyIdB = bodyId;
				}
			}
		}
	}

	internal static Capacity GetSleepCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 1, dynamicShapeCount = BENCHMARK_DEBUG ? 400 : 21000, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 400 : 21000, contactCount = BENCHMARK_DEBUG ? 1200 : 68000, arenaByteCount = ( BENCHMARK_DEBUG ? 190 : 10000 ) * 1024 };

		return c;
	}

	// Stress tests waking and sleeping.
	internal static void CreateSleep( WorldId worldId )
	{
		g_sleepData = default(SleepData);

		int baseCount = BENCHMARK_DEBUG ? 8 : 60;
		float extent = 0.5f;

		float baseWidth = 2.0f * extent * baseCount;
		float pitch = baseWidth + 8.0f * extent;
		float span = pitch * ( SLEEP_PYRAMID_COUNT - 1 );

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( worldId, ref bodyDef );

			Polygon box = MakeBox( 0.5f * span + baseWidth, 1.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		for ( int i = 0; i < SLEEP_PYRAMID_COUNT; ++i )
		{
			float centerX = -0.5f * span + i * pitch - 0.5f * baseWidth;
			CreateSleepPyramid( worldId, baseCount, extent, centerX, g_sleepData.bodyIdA + i, g_sleepData.bodyIdB + i );
		}
	}

	internal static float StepSleep( WorldId worldId, int stepCount )
	{
		for ( int i = 0; i < SLEEP_PYRAMID_COUNT; ++i )
		{
			if ( Body_IsAwake( g_sleepData.bodyIdA[i] ) == false )
			{
				// Creating and destroying a joint engages the island splitter
				FilterJointDef jointDef = DefaultFilterJointDef();
				jointDef.@base.bodyIdA = g_sleepData.bodyIdA[i];
				jointDef.@base.bodyIdB = g_sleepData.bodyIdB[i];
				JointId jointId = CreateFilterJoint( worldId, ref jointDef );

				// This wakes the island
				DestroyJoint( jointId );

				// Only one per step
				break;
			}
		}

		return 0.0f;
	}

	internal static Capacity GetCompoundsCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = 310, dynamicShapeCount = BENCHMARK_DEBUG ? 880 : 6600, staticBodyCount = 1, dynamicBodyCount = BENCHMARK_DEBUG ? 440 : 3300, contactCount = BENCHMARK_DEBUG ? 4400 : 37000, arenaByteCount = ( BENCHMARK_DEBUG ? 370 : 2900 ) * 1024 };

		return c;
	}

	// Each dynamic body is a compound of two triangular polygon shapes.
	internal static void CreateCompounds( WorldId worldId )
	{
		{
			float gridSize = 1.0f;

			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				Polygon box = MakeOffsetBox( 0.55f * gridSize, 0.5f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref box );
				x += gridSize;
			}

			y = gridSize;
			x = -40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				Polygon box = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref box );
				y += gridSize;
			}

			y = gridSize;
			x = 40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				Polygon box = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new Vec2( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref box );
				y += gridSize;
			}

			Segment segment = new Segment { point1 = new( -800.0f, -80.0f ), point2 = new( 800.0f, -80.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		int columnCount = BENCHMARK_DEBUG ? 10 : 20;
		int rowCount = BENCHMARK_DEBUG ? 40 : 150;

		Vec2[] leftPoints = { new( -1.0f, 0.0f ), new( 0.5f, 1.0f ), new( 0.0f, 2.0f ) };
		Hull leftHull = ComputeHull( leftPoints, 3 );
		Polygon left = MakePolygon( ref leftHull, 0.0f );

		Vec2[] rightPoints = { new( 1.0f, 0.0f ), new( -0.5f, 1.0f ), new( 0.0f, 2.0f ) };
		Hull rightHull = ComputeHull( rightPoints, 3 );
		Polygon right = MakePolygon( ref rightHull, 0.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		// Match the sample exactly: centery is computed before shift is reset for the compound branch.
		float shift = 2.0f;
		float extray = 0.25f;
		float side = 0.25f;
		float centerx = shift * columnCount / 2.0f - 1.0f;
		float centery = 1.15f / 2.0f;
		float yStart = 5.0f;

		for ( int i = 0; i < columnCount; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < rowCount; ++j )
			{
				float y = j * ( shift + extray ) + centery + yStart;

				bodyDef.position = new Pos { x = x + side, y = y };
				side = -side;

				BodyId bodyId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref left );
				CreatePolygonShape( bodyId, ref shapeDef, ref right );
			}
		}
	}

	// Query benchmark. A sparse grid of static boxes like the cast sample, a pile of dynamic boxes
	// settling on a ground inside it with a kinematic paddle keeping some of them awake, and a few
	// static sensors in the pile. Every step runs a fixed set of closest rays, circle casts and box
	// overlaps, once with the default filter and once with a mask that hits one category in three.
	// The queries are precomputed so only the tree work lands in the step time.

	// #define QUERY_COUNT ( BENCHMARK_DEBUG ? 100 : 1000 )

	internal struct QueryBenchmark
	{
		public /*TODO fixed 1000*/ Pos[] origins;
		public /*TODO fixed 1000*/ Vec2[] translations;
		public BodyId paddleId;
		public TreeStats stats;
		public float extent;
		public float paddleMinX;
		public float paddleMaxX;
	}

	static QueryBenchmark g_queryBenchmark;
	static uint g_queryRandomState;

	// Repeatable is all this needs to be
	private static float QueryRandom( float lower, float upper )
	{
		g_queryRandomState = 1664525u * g_queryRandomState + 1013904223u;
		float unit = (float)( g_queryRandomState >> 8 ) * ( 1.0f / 16777216.0f );
		return lower + ( upper - lower ) * unit;
	}

	internal static Capacity GetQueriesCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = BENCHMARK_DEBUG ? 670 : 28000, dynamicShapeCount = BENCHMARK_DEBUG ? 550 : 2200, staticBodyCount = BENCHMARK_DEBUG ? 670 : 28000, dynamicBodyCount = BENCHMARK_DEBUG ? 560 : 2300, contactCount = BENCHMARK_DEBUG ? 2400 : 9800, arenaByteCount = ( BENCHMARK_DEBUG ? 270 : 930 ) * 1024 };

		return c;
	}

	internal static void CreateQueries( WorldId worldId )
	{
		g_queryRandomState = 1234;

		float extent = BENCHMARK_DEBUG ? 100.0f : 500.0f;
		int cellCount = BENCHMARK_DEBUG ? 100 : 500;
		float fill = 0.1f;

		// The pile keeps this region of the grid clear
		float pileX = 0.4f * extent;
		float pileY = 0.3f * extent;
		AABB clear = new AABB { lowerBound = new( pileX - 10.0f, pileY - 10.0f ), upperBound = new( pileX + 50.0f, pileY + 60.0f ) };

		BodyDef bodyDef = DefaultBodyDef();
		ShapeDef shapeDef = DefaultShapeDef();

		for ( int i = 0; i < cellCount; ++i )
		{
			float y = (float)i;
			for ( int j = 0; j < cellCount; ++j )
			{
				float x = (float)j;

				float fillTest = QueryRandom( 0.0f, 1.0f );
				float ratio = QueryRandom( 1.0f, 5.0f );
				float halfWidth = QueryRandom( 0.05f, 0.25f );
				float orientation = QueryRandom( 0.0f, 1.0f );
				int category = (int)QueryRandom( 0.0f, 2.999f );

				if ( fillTest > fill )
				{
					continue;
				}

				if ( clear.lowerBound.x <= x && x <= clear.upperBound.x && clear.lowerBound.y <= y && y <= clear.upperBound.y )
				{
					continue;
				}

				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( worldId, ref bodyDef );

				Polygon box =
					orientation > 0.5f ? MakeBox( ratio * halfWidth, halfWidth ) : MakeBox( halfWidth, ratio * halfWidth );
				shapeDef.filter.categoryBits = 1ul << category;
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}

		shapeDef.filter.categoryBits = 1;

		// Ground for the pile
		{
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			BodyId groundId = CreateBody( worldId, ref bodyDef );
			Segment segment = new Segment { point1 = new( pileX - 10.0f, pileY ), point2 = new( pileX + 50.0f, pileY ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Static sensors sitting in the pile
		{
			ShapeDef sensorDef = DefaultShapeDef();
			sensorDef.isSensor = true;
			Polygon sensorBox = MakeSquare( 2.0f );
			for ( int i = 0; i < 8; ++i )
			{
				bodyDef.position = new Pos { x = pileX + 4.0f * i, y = pileY + 2.0f };
				BodyId sensorId = CreateBody( worldId, ref bodyDef );
				CreatePolygonShape( sensorId, ref sensorDef, ref sensorBox );
			}
		}

		// The pile, dropped from just above the ground so it settles quickly
		{
			int columnCount = BENCHMARK_DEBUG ? 20 : 40;
			int rowCount = BENCHMARK_DEBUG ? 25 : 50;
			float spacing = 0.55f;
			Polygon box = MakeSquare( 0.25f );

			bodyDef.type = BodyType.dynamicBody;
			for ( int i = 0; i < rowCount; ++i )
			{
				for ( int j = 0; j < columnCount; ++j )
				{
					bodyDef.position = new Pos { x = pileX + spacing * j, y = pileY + 1.0f + spacing * i };
					BodyId bodyId = CreateBody( worldId, ref bodyDef );
					CreatePolygonShape( bodyId, ref shapeDef, ref box );
				}
			}
		}

		// Kinematic paddle sweeping through the bottom of the pile
		{
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = pileX - 5.0f, y = pileY + 1.2f };
			bodyDef.linearVelocity = new Vec2( 5.0f, 0.0f );
			g_queryBenchmark.paddleId = CreateBody( worldId, ref bodyDef );
			Polygon paddle = MakeBox( 1.0f, 1.0f );
			CreatePolygonShape( g_queryBenchmark.paddleId, ref shapeDef, ref paddle );
			g_queryBenchmark.paddleMinX = pileX - 5.0f;
			g_queryBenchmark.paddleMaxX = pileX + 30.0f;
		}

		// Rays and casts cross the whole world, overlaps are centered on the origins
		int queryCount = QUERY_COUNT;
		for ( int i = 0; i < queryCount; ++i )
		{
			float x1 = QueryRandom( 0.0f, extent );
			float y1 = QueryRandom( 0.0f, extent );
			float x2 = QueryRandom( 0.0f, extent );
			float y2 = QueryRandom( 0.0f, extent );
			g_queryBenchmark.origins[i] = new Pos { x = x1, y = y1 };
			g_queryBenchmark.translations[i] = new Vec2( x2 - x1, y2 - y1 );
		}

		g_queryBenchmark.stats = default(TreeStats);
		g_queryBenchmark.extent = extent;
	}

	private static float QueryCastClosest( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		shapeId;
		point;
		normal;
		*(float*)context = fraction;
		return fraction;
	}

	private static bool QueryOverlapCount( ShapeId shapeId, object? context )
	{
		shapeId;
		*(int*)context += 1;
		return true;
	}

	internal static float StepQueries( WorldId worldId, int stepCount )
	{
		stepCount;

		QueryBenchmark* data = &g_queryBenchmark;

		// Reverse the paddle at the ends of its sweep
		Pos paddlePosition = Body_GetPosition( data.paddleId );
		if ( paddlePosition.x > data.paddleMaxX )
		{
			Body_SetLinearVelocity( data.paddleId, new Vec2( -5.0f, 0.0f ) );
		}
		else if ( paddlePosition.x < data.paddleMinX )
		{
			Body_SetLinearVelocity( data.paddleId, new Vec2( 5.0f, 0.0f ) );
		}

		QueryFilter[] filters = { DefaultQueryFilter(), DefaultQueryFilter() };
		filters[1].maskBits = 1;

		Vec2 circleCenter = Vec2_zero;
		ShapeProxy circle = MakeProxy( ref circleCenter, 1, 0.1f );

		int queryCount = QUERY_COUNT;
		int hitCount = 0;
		TreeStats stats = default;

		for ( int f = 0; f < 2; ++f )
		{
			QueryFilter filter = filters[f];

			for ( int i = 0; i < queryCount; ++i )
			{
				RayResult result = World_CastRayClosest( worldId, data.origins[i], data.translations[i], filter );
				stats.nodeVisits += result.nodeVisits;
				stats.leafVisits += result.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			for ( int i = 0; i < queryCount; ++i )
			{
				float fraction = 1.0f;
				TreeStats castStats = World_CastShape( worldId, data.origins[i], ref circle, data.translations[i], filter,
														   QueryCastClosest, ref fraction );
				stats.nodeVisits += castStats.nodeVisits;
				stats.leafVisits += castStats.leafVisits;
				hitCount += fraction < 1.0f ? 1 : 0;
			}

			for ( int i = 0; i < queryCount; ++i )
			{
				int overlapCount = 0;
				AABB aabb = new AABB { lowerBound = new( -5.0f, -5.0f ), upperBound = new( 5.0f, 5.0f ) };
				TreeStats overlapStats =
					World_OverlapAABB( worldId, data.origins[i], aabb, filter, QueryOverlapCount, ref overlapCount );
				stats.nodeVisits += overlapStats.nodeVisits;
				stats.leafVisits += overlapStats.leafVisits;
				hitCount += overlapCount;
			}
		}

		data.stats.nodeVisits += stats.nodeVisits;
		data.stats.leafVisits += stats.leafVisits;

		return (float)hitCount;
	}

	internal static TreeStats GetQueryBenchmarkStats()
	{
		return g_queryBenchmark.stats;
	}

	internal static int GetQueryBenchmarkCount()
	{
		return QUERY_COUNT;
	}

	internal static float GetQueryBenchmarkExtent()
	{
		return g_queryBenchmark.extent;
	}

	internal static void GetQueryBenchmarkRay( int index, ref Pos origin, ref Vec2 translation )
	{
		origin = g_queryBenchmark.origins[index];
		translation = g_queryBenchmark.translations[index];
	}

	// Tree cast benchmark. The queries grid goes into a bare dynamic tree one proxy at a time and is
	// never rebuilt, which is the tree a game runs on. Every step casts the same rays through
	// the tree at full extent, at 25 units and at 5 units, with the default mask and with
	// a mask that hits one category in three, then the same directions as box casts. The callback
	// clips to the tight box, so hits and fraction sums match across builds and nothing but the tree
	// is in the step time. Full extent rays shrink the fraction as they go, 25 unit rays sit at the
	// cache edge of this tree, 5 unit rays are the game case.

	// #define TREE_CAST_RAY_COUNT ( BENCHMARK_DEBUG ? 100 : 1000 )
	private const int TREE_CAST_LENGTH_COUNT = 3;
	// Static proxies get the speculative distance twice, once in the shape box and once as the margin
	// #define TREE_CAST_MARGIN ( 2.0f * SpeculativeDistance )

	// The queries benchmark casts a circle of this radius, the tree sees its box
	private const float TREE_CAST_BOX_EXTENT = 0.1f;
	internal struct TreeCastBenchmark
	{
		public DynamicTree tree;
		public /*TODO pointer*/ AABB[] tightBoxes;
		public int proxyCount;
		public int proxyCapacity;
		public /*TODO fixed 1000*/ Vec2[] origins;
		public /*TODO fixed TREE_CAST_LENGTH_COUNT][1000*/ Vec2[] translations;
		public TreeStats stats;
		public bool created;
	}

	internal ref struct TreeCastContext : ITreeRayCastCallback
	{
		public float Invoke( in RayCastInput input, int proxyId, ulong userData ) => TreeCastRayCallback( input, proxyId, userData, ref this );

		public ref AABB tightBoxes;
		public float fraction;
	}

	static TreeCastBenchmark g_treeCast;

	// Entry fraction of a ray into a box, negative for a miss. An origin inside or on the box is a miss,
	// which matches the polygon cast the world runs, and zero would end the traversal.
	private static float TreeCastRayBox( AABB box, Vec2 p, Vec2 d, float maxFraction )
	{
		float tmin = 0.0f;
		float tmax = maxFraction;

		if ( d.x == 0.0f )
		{
			if ( p.x < box.lowerBound.x || box.upperBound.x < p.x )
			{
				return -1.0f;
			}
		}
		else
		{
			float inv = 1.0f / d.x;
			float t1 = ( box.lowerBound.x - p.x ) * inv;
			float t2 = ( box.upperBound.x - p.x ) * inv;
			if ( t1 > t2 )
			{
				float t = t1;
				t1 = t2;
				t2 = t;
			}

			tmin = MaxFloat( tmin, t1 );
			tmax = MinFloat( tmax, t2 );
			if ( tmin > tmax )
			{
				return -1.0f;
			}
		}

		if ( d.y == 0.0f )
		{
			if ( p.y < box.lowerBound.y || box.upperBound.y < p.y )
			{
				return -1.0f;
			}
		}
		else
		{
			float inv = 1.0f / d.y;
			float t1 = ( box.lowerBound.y - p.y ) * inv;
			float t2 = ( box.upperBound.y - p.y ) * inv;
			if ( t1 > t2 )
			{
				float t = t1;
				t1 = t2;
				t2 = t;
			}

			tmin = MaxFloat( tmin, t1 );
			tmax = MinFloat( tmax, t2 );
			if ( tmin > tmax )
			{
				return -1.0f;
			}
		}

		if ( tmin <= 0.0f )
		{
			return -1.0f;
		}

		return tmin;
	}

	private static float TreeCastRayCallback( in RayCastInput input, int proxyId, ulong userData, ref TreeCastContext cast )
	{
		proxyId;
		float fraction = TreeCastRayBox( cast.tightBoxes[userData], input.origin, input.translation, input.maxFraction );
		if ( fraction < 0.0f )
		{
			return -1.0f;
		}

		cast.fraction = fraction;
		return fraction;
	}

	// A box sweep against a box is a ray from the cast box center against the target grown by the extents
	private static float TreeCastBoxCallback( in BoxCastInput input, int proxyId, ulong userData, ref TreeCastContext cast )
	{
		proxyId;
		AABB tight = cast.tightBoxes[userData];
		Vec2 extents = AABB_Extents( input.box );
		AABB grown = new AABB { lowerBound = Sub( tight.lowerBound, extents ), upperBound = Add( tight.upperBound, extents ) };
		float fraction = TreeCastRayBox( grown, AABB_Center( input.box ), input.translation, input.maxFraction );
		if ( fraction < 0.0f )
		{
			return -1.0f;
		}

		cast.fraction = fraction;
		return fraction;
	}

	private static void TreeCastAddBox( Vec2 center, float hx, float hy, ulong categoryBits )
	{
		TreeCastBenchmark* data = &g_treeCast;
		assert( data.proxyCount < data.proxyCapacity );

		AABB tight = new AABB { lowerBound = new( center.x - hx, center.y - hy ), upperBound = new( center.x + hx, center.y + hy ) };
		AABB fat = new AABB { lowerBound = new( tight.lowerBound.x - TREE_CAST_MARGIN, tight.lowerBound.y - TREE_CAST_MARGIN ), upperBound = new( tight.upperBound.x + TREE_CAST_MARGIN, tight.upperBound.y + TREE_CAST_MARGIN ) };

		int index = data.proxyCount;
		data.tightBoxes[index] = tight;
		data.proxyCount += 1;
		CreateTreeProxy( data.tree, fat, categoryBits, (ulong)index );
	}

	internal static void DestroyTreeCast()
	{
		TreeCastBenchmark* data = &g_treeCast;
		if ( data.created )
		{
			DestroyDynamicTree( data.tree );
			free( data.tightBoxes );
		}

		// The harness destroys after each run and prints the stats after the last one
		TreeStats stats = data.stats;
		data = default;
		data.stats = stats;
	}

	internal static void CreateTreeCast( WorldId worldId )
	{
		worldId;
		DestroyTreeCast();

		TreeCastBenchmark* data = &g_treeCast;

		// Same generator and seed as the queries benchmark so the grid matches it
		g_queryRandomState = 1234;

		float extent = BENCHMARK_DEBUG ? 100.0f : 500.0f;
		int cellCount = BENCHMARK_DEBUG ? 100 : 500;
		float fill = 0.1f;

		// The pile keeps this region of the grid clear
		float pileX = 0.4f * extent;
		float pileY = 0.3f * extent;
		AABB clear = new AABB { lowerBound = new( pileX - 10.0f, pileY - 10.0f ), upperBound = new( pileX + 50.0f, pileY + 60.0f ) };

		// A tenth of the cells fill, the ground and the sensors come after
		data.proxyCapacity = cellCount * cellCount / 5;
		data.tightBoxes = malloc( data.proxyCapacity * Unsafe.SizeOf<AABB>() );
		data.tree = CreateDynamicTree( data.proxyCapacity );
		data.created = true;

		for ( int i = 0; i < cellCount; ++i )
		{
			float y = (float)i;
			for ( int j = 0; j < cellCount; ++j )
			{
				float x = (float)j;

				float fillTest = QueryRandom( 0.0f, 1.0f );
				float ratio = QueryRandom( 1.0f, 5.0f );
				float halfWidth = QueryRandom( 0.05f, 0.25f );
				float orientation = QueryRandom( 0.0f, 1.0f );
				int category = (int)QueryRandom( 0.0f, 2.999f );

				if ( fillTest > fill )
				{
					continue;
				}

				if ( clear.lowerBound.x <= x && x <= clear.upperBound.x && clear.lowerBound.y <= y && y <= clear.upperBound.y )
				{
					continue;
				}

				float hx = orientation > 0.5f ? ratio * halfWidth : halfWidth;
				float hy = orientation > 0.5f ? halfWidth : ratio * halfWidth;
				TreeCastAddBox( new Vec2( x, y ), hx, hy, 1ul << category );
			}
		}

		// Ground for the pile, a degenerate box
		TreeCastAddBox( new Vec2( pileX + 20.0f, pileY ), 30.0f, 0.0f, 1 );

		// Static sensors sitting in the pile
		for ( int i = 0; i < 8; ++i )
		{
			TreeCastAddBox( new Vec2( pileX + 4.0f * i, pileY + 2.0f ), 2.0f, 2.0f, 1 );
		}

		// Rays cross the whole world, then the same directions cut to the shorter lengths
		float[] lengths = { 0.0f, 25.0f, 5.0f };
		for ( int i = 0; i < TREE_CAST_RAY_COUNT; ++i )
		{
			float x1 = QueryRandom( 0.0f, extent );
			float y1 = QueryRandom( 0.0f, extent );
			float x2 = QueryRandom( 0.0f, extent );
			float y2 = QueryRandom( 0.0f, extent );
			Vec2 t = new Vec2( x2 - x1, y2 - y1 );
			data.origins[i] = new Vec2( x1, y1 );
			data.translations[0][i] = t;

			float length = Length( t );
			for ( int k = 1; k < TREE_CAST_LENGTH_COUNT; ++k )
			{
				float scale = length > 0.0f ? lengths[k] / length : 0.0f;
				data.translations[k][i] = MulSV( scale, t );
			}
		}

		data.stats = default(TreeStats);
	}

	internal static float StepTreeCast( WorldId worldId, int stepCount )
	{
		worldId;
		stepCount;

		TreeCastBenchmark* data = &g_treeCast;
		DynamicTree tree = data.tree;
		ulong[] masks = { DefaultMaskBits, 1 };
		int rayCount = TREE_CAST_RAY_COUNT;
		TreeStats stats = default;
		int hitCount = 0;

		for ( int k = 0; k < TREE_CAST_LENGTH_COUNT; ++k )
		{
			ReadOnlySpan<Vec2> translations = data.translations[k];
			for ( int m = 0; m < 2; ++m )
			{
				for ( int i = 0; i < rayCount; ++i )
				{
					RayCastInput input = new RayCastInput { origin = data.origins[i], translation = translations[i], maxFraction = 1.0f };
					TreeCastContext context = new TreeCastContext { tightBoxes = ref data.tightBoxes, fraction = 1.0f };
					TreeStats castStats = DynamicTree_CastRay( tree, ref input, masks[m], ref context );
					stats.nodeVisits += castStats.nodeVisits;
					stats.leafVisits += castStats.leafVisits;
					hitCount += context.fraction < 1.0f ? 1 : 0;
				}
			}

			for ( int i = 0; i < rayCount; ++i )
			{
				Vec2 o = data.origins[i];
				AABB box = new AABB { lowerBound = new( o.x - TREE_CAST_BOX_EXTENT, o.y - TREE_CAST_BOX_EXTENT ), upperBound = new( o.x + TREE_CAST_BOX_EXTENT, o.y + TREE_CAST_BOX_EXTENT ) };
				BoxCastInput input = new BoxCastInput { box = box, translation = translations[i], maxFraction = 1.0f };
				TreeCastContext context = new TreeCastContext { tightBoxes = ref data.tightBoxes, fraction = 1.0f };
				TreeStats castStats = DynamicTree_CastBox( tree, ref input, DefaultMaskBits, ref context );
				stats.nodeVisits += castStats.nodeVisits;
				stats.leafVisits += castStats.leafVisits;
				hitCount += context.fraction < 1.0f ? 1 : 0;
			}
		}

		data.stats.nodeVisits += stats.nodeVisits;
		data.stats.leafVisits += stats.leafVisits;

		return (float)hitCount;
	}

	internal static TreeStats GetTreeCastBenchmarkStats()
	{
		return g_treeCast.stats;
	}

	// Tile world benchmark. The TileWorld sample's terrain, a wall of rounded boxes whose height follows
	// a sine, on a static body every ten columns, with a box stack every twelfth cycle. Each step streams
	// the terrain, the leftmost ground body is destroyed and a new one continues the sine on the right,
	// then short rays and box overlaps are cast across the live span. A large static tree under churn,
	// the shape of a level that streams.

	private const float TILE_PERIOD = 40.0f;
	// #define TILE_CYCLE_COUNT ( BENCHMARK_DEBUG ? 10 : 600 )
	private const float TILE_GRID_SIZE = 1.0f;
	private const int TILE_COLUMNS_PER_BODY = 10;
	// #define TILE_QUERY_COUNT ( BENCHMARK_DEBUG ? 50 : 500 )
	private const float TILE_RAY_LENGTH = 25.0f;
	internal struct TileWorldBenchmark
	{
		public /*TODO pointer*/ BodyId[] groundIds;
		public int groundCount;
		public int groundHead;
		public float bodySpan;
		public float xLeft;
		public float xNext;
		public TreeStats stats;
	}

	static TileWorldBenchmark g_tileWorld;

	private static float TileRandom( Span<uint> state, float lower, float upper )
	{
		state = 1664525u * state[1013904223u];
		float unit = (float)( state >> 8 ) * ( 1.0f / 16777216.0f );
		return lower + ( upper - lower ) * unit;
	}

	// Ten columns on one body, the column height from the sine at the column's world x
	private static BodyId CreateTileGround( WorldId worldId, float xBody )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position.x = xBody;
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.invokeContactCreation = false;

		float omega = 2.0f * Pi / TILE_PERIOD;
		float xShape = 0.0f;
		for ( int i = 0; i < TILE_COLUMNS_PER_BODY; ++i )
		{
			int columnCount = (int)roundf( 4.0f * cosf( omega * ( xBody + xShape ) ) ) + 12;
			float y = 0.0f;
			for ( int j = 0; j < columnCount; ++j )
			{
				Polygon square =
					MakeOffsetBox( 0.4f * TILE_GRID_SIZE, 0.4f * TILE_GRID_SIZE, new Vec2( xShape, y ), Rot_identity );
				square.radius = 0.1f;
				CreatePolygonShape( groundId, ref shapeDef, ref square );
				y += TILE_GRID_SIZE;
			}

			xShape += TILE_GRID_SIZE;
		}

		return groundId;
	}

	internal static void DestroyTileWorld()
	{
		TileWorldBenchmark* data = &g_tileWorld;
		free( data.groundIds );

		// The harness destroys after each run and prints the stats after the last one
		TreeStats stats = data.stats;
		data = default;
		data.stats = stats;
	}

	internal static Capacity GetTileWorldCapacity()
	{
		Capacity c = new Capacity { staticShapeCount = BENCHMARK_DEBUG ? 5300 : 320000, dynamicShapeCount = BENCHMARK_DEBUG ? 55 : 2200, staticBodyCount = BENCHMARK_DEBUG ? 44 : 2700, dynamicBodyCount = BENCHMARK_DEBUG ? 55 : 2200, contactCount = BENCHMARK_DEBUG ? 200 : 8800, arenaByteCount = ( BENCHMARK_DEBUG ? 14 : 1300 ) * 1024 };

		return c;
	}

	internal static void CreateTileWorld( WorldId worldId )
	{
		DestroyTileWorld();

		TileWorldBenchmark* data = &g_tileWorld;

		int gridCount = (int)( TILE_CYCLE_COUNT * TILE_PERIOD / TILE_GRID_SIZE );
		data.groundCount = gridCount / TILE_COLUMNS_PER_BODY;
		data.groundIds = malloc( data.groundCount * Unsafe.SizeOf<BodyId>() );
		data.bodySpan = TILE_COLUMNS_PER_BODY * TILE_GRID_SIZE;
		data.xLeft = -0.5f * ( TILE_CYCLE_COUNT * TILE_PERIOD );
		data.xNext = data.xLeft;
		data.groundHead = 0;

		for ( int i = 0; i < data.groundCount; ++i )
		{
			data.groundIds[i] = CreateTileGround( worldId, data.xNext );
			data.xNext += data.bodySpan;
		}

		// Box stacks in the valleys of every twelfth cycle, past the fifth of the span that streams out
		// during the run
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.3f, 0.2f );

		for ( int cycleIndex = TILE_CYCLE_COUNT / 5; cycleIndex < TILE_CYCLE_COUNT; cycleIndex += 12 )
		{
			float xBase = ( 0.5f + cycleIndex ) * TILE_PERIOD + data.xLeft;
			bodyDef.position.x = xBase - 3.0f;
			for ( int i = 0; i < 10; ++i )
			{
				bodyDef.position.y = 10.0f;
				for ( int j = 0; j < 5; ++j )
				{
					BodyId bodyId = CreateBody( worldId, ref bodyDef );
					CreatePolygonShape( bodyId, ref shapeDef, ref box );
					bodyDef.position.y += 0.5f;
				}

				bodyDef.position.x += 0.6f;
			}
		}

		data.stats = default(TreeStats);
	}

	private static bool TileOverlapCount( ShapeId shapeId, object? context )
	{
		shapeId;
		*(int*)context += 1;
		return true;
	}

	internal static float StepTileWorld( WorldId worldId, int stepCount )
	{
		TileWorldBenchmark* data = &g_tileWorld;

		// Stream one ground body, the leftmost goes and the sine continues on the right
		DestroyBody( data.groundIds[data.groundHead] );
		data.groundIds[data.groundHead] = CreateTileGround( worldId, data.xNext );
		data.groundHead = ( data.groundHead + 1 ) % data.groundCount;
		data.xNext += data.bodySpan;
		data.xLeft += data.bodySpan;

		// A fresh set of short rays and box overlaps across the live span each step
		uint state = 1234u + 7919u * (uint)stepCount;
		float span = data.groundCount * data.bodySpan;
		QueryFilter filter = DefaultQueryFilter();
		AABB aabb = new AABB { lowerBound = new( -5.0f, -5.0f ), upperBound = new( 5.0f, 5.0f ) };
		TreeStats stats = default;
		int hitCount = 0;

		for ( int i = 0; i < TILE_QUERY_COUNT; ++i )
		{
			Pos origin = new Pos { x = data.xLeft + TileRandom( state, 0.0f, span ), y = TileRandom( state, -2.0f, 18.0f ) };
			float angle = TileRandom( state, 0.0f, 2.0f * Pi );
			Vec2 translation = new Vec2( TILE_RAY_LENGTH * cosf( angle ), TILE_RAY_LENGTH * sinf( angle ) );

			RayResult result = World_CastRayClosest( worldId, origin, translation, filter );
			stats.nodeVisits += result.nodeVisits;
			stats.leafVisits += result.leafVisits;
			hitCount += result.hit ? 1 : 0;

			int overlapCount = 0;
			TreeStats overlapStats = World_OverlapAABB( worldId, origin, aabb, filter, TileOverlapCount, overlapCount );
			stats.nodeVisits += overlapStats.nodeVisits;
			stats.leafVisits += overlapStats.leafVisits;
			hitCount += overlapCount;
		}

		data.stats.nodeVisits += stats.nodeVisits;
		data.stats.leafVisits += stats.leafVisits;

		return (float)hitCount;
	}

	internal static TreeStats GetTileWorldBenchmarkStats()
	{
		return g_tileWorld.stats;
	}

}
