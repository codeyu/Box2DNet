// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

// From shared/determinism.h
internal struct FallingHingeData
{
	public BodyId[] bodyIds;
	public int bodyCount;
	public int stepCount;
	public int sleepStep;
	public uint hash;
}

public static partial class DeterminismShared
{
	// These match shared/determinism.h for single precision
	internal const int EXPECTED_SLEEP_STEP = 299;
	internal const uint EXPECTED_HASH = 0x465F3DF4;

	private static void assert( bool condition ) => ENSURE( condition );

	internal static FallingHingeData CreateFallingHinges( WorldId worldId )
	{
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( worldId, ref bodyDef );

			Polygon box = MakeBox( 40.0f, 1.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		int columnCount = 4;
		int rowCount = 20;
		int bodyCount = rowCount * columnCount;

		BodyId[] bodyIds = new BodyId[bodyCount];
		float h = 0.25f;
		float r = 0.1f * h;
		Polygon hingeBox = MakeRoundedBox( h - r, h - r, r );
		hingeBox = MakeSquare( h );

		ShapeDef hingeShapeDef = DefaultShapeDef();
		// shapeDef.material.friction = 0.3f;

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.1f * Pi;
		jointDef.upperAngle = 0.2f * Pi;
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0f;
		jointDef.dampingRatio = 1.0f;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.25f;
		jointDef.@base.localFrameA.p = new Vec2( -h, h );
		jointDef.@base.localFrameB.p = new Vec2( -h, -h );
		jointDef.@base.constraintHertz = 60.0f;
		jointDef.@base.constraintDampingRatio = 0.0f;
		jointDef.@base.drawScale = 0.5f;

		int bodyIndex = 0;
		float offset = 0.4f * h;
		float dx = 10.0f * h;
		float xBase = -0.5f * dx * ( columnCount - 1.0f );

		for ( int j = 0; j < columnCount; ++j )
		{
			float x = xBase + j * dx;

			BodyId prevBodyId = nullBodyId;

			for ( int i = 0; i < rowCount; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;

				bodyDef.position.x = x + offset * i;
				bodyDef.position.y = h + 2.0f * h * i;

				// this tests the deterministic cosine and sine functions
				float angle = ( i & 1 ) == 0 ? -0.1f : 0.1f;
				bodyDef.rotation = MakeRot( angle );

				BodyId bodyId = CreateBody( worldId, ref bodyDef );

				if ( ( i & 1 ) == 0 )
				{
					prevBodyId = bodyId;
				}
				else
				{
					jointDef.@base.bodyIdA = prevBodyId;
					jointDef.@base.bodyIdB = bodyId;
					CreateRevoluteJoint( worldId, ref jointDef );
					prevBodyId = nullBodyId;
				}

				CreatePolygonShape( bodyId, ref hingeShapeDef, ref hingeBox );

				assert( bodyIndex < bodyCount );
				bodyIds[bodyIndex] = bodyId;

				bodyIndex += 1;
			}
		}

		assert( bodyIndex == bodyCount );

		FallingHingeData data = new FallingHingeData { bodyIds = bodyIds, bodyCount = bodyCount, stepCount = 0, sleepStep = -1, hash = 0 };
		return data;
	}

	internal static bool UpdateFallingHinges( WorldId worldId, ref FallingHingeData data )
	{
		if ( data.hash == 0 )
		{
			BodyEvents bodyEvents = World_GetBodyEvents( worldId );

			if ( bodyEvents.moveCount == 0 )
			{
				assert( World_GetAwakeBodyCount( worldId ) == 0 );

				data.hash = HashInit;
				for ( int i = 0; i < data.bodyCount; ++i )
				{
					WorldTransform transform = Body_GetTransform( data.bodyIds[i] );
					data.hash = Hash( data.hash, MemoryMarshal.AsBytes( new ReadOnlySpan<WorldTransform>( in transform ) ) );
				}

				data.sleepStep = data.stepCount;
			}
		}

		data.stepCount += 1;

		return data.hash != 0;
	}

	internal static void DestroyFallingHinges( ref FallingHingeData data )
	{
		data.bodyIds = null!;
		data.bodyIds = null;
	}

}
