// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class InvalidInputTests
{
	// The guard logs once per rejected call in every build config, while the assert only fires in
	// debug. So the log count is the portable signal that bad input was turned away.
	static int logCount;

	private static void CountingLogFcn( string message )
	{
		MAYBE_UNUSED( message );
		logCount += 1;
	}

	private static int SilentAssertFcn( string condition, string fileName, int lineNumber )
	{
		MAYBE_UNUSED( condition );
		MAYBE_UNUSED( fileName );
		MAYBE_UNUSED( lineNumber );

		// Keep going so the subtest can report rather than break to the debugger
		return 0;
	}

	private static void BeginRejectCount()
	{
		logCount = 0;
		SetLogCallback( CountingLogFcn );
		SetAssertCallback( SilentAssertFcn );
	}

	private static void EndRejectCount()
	{
		SetAssertCallback( TestAssertFcn );
		SetLogCallback( TestLogFcn );
	}

	// #define REJECTS( CALL )                                                                                                          \
		do                                                                                                                           \
		{                                                                                                                            \
			int countBefore = logCount;                                                                                              \
			CALL;                                                                                                                    \
			if ( logCount != countBefore + 1 )                                                                                       \
			{                                                                                                                        \
				printf( "not rejected: " #CALL "\n" );                                                                               \
				return 1;                                                                                                            \
			}                                                                                                                        \
		}                                                                                                                            \
		while ( false )

	// #define ACCEPTS( CALL )                                                                                                          \
		do                                                                                                                           \
		{                                                                                                                            \
			int countBefore = logCount;                                                                                              \
			CALL;                                                                                                                    \
			if ( logCount != countBefore )                                                                                           \
			{                                                                                                                        \
				printf( "wrongly rejected: " #CALL "\n" );                                                                           \
				return 1;                                                                                                            \
			}                                                                                                                        \
		}                                                                                                                            \
		while ( false )

	private static readonly float[] badValues = { NAN, INFINITY, -INFINITY };

	private static bool IsBodyFinite( BodyId bodyId )
	{
		return IsValidPos( Body_GetPosition( bodyId ) ) && IsValidRot( Body_GetRotation( bodyId ) ) &&
			   IsValidVec2( Body_GetLinearVelocity( bodyId ) ) && IsValidFloat( Body_GetAngularVelocity( bodyId ) );
	}

	private static int WorldInputTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		Vec2 gravity = World_GetGravity( worldId );
		float restitutionThreshold = World_GetRestitutionThreshold( worldId );
		float hitEventThreshold = World_GetHitEventThreshold( worldId );
		float recycleDistance = World_GetContactRecycleDistance( worldId );
		float maximumLinearSpeed = World_GetMaxLinearSpeed( worldId );
		int restitutionIterations = World_GetRestitutionIterations( worldId );

		BeginRejectCount();

		for ( int i = 0; i < badValues.Length; ++i )
		{
			float bad = badValues[i];

			REJECTS( World_SetGravity( worldId, new Vec2( bad, 0.0f ) ) );
			REJECTS( World_SetGravity( worldId, new Vec2( 0.0f, bad ) ) );
			REJECTS( World_SetRestitutionThreshold( worldId, bad ) );
			REJECTS( World_SetHitEventThreshold( worldId, bad ) );
			REJECTS( World_SetContactRecycleDistance( worldId, bad ) );
			REJECTS( World_SetMaxLinearSpeed( worldId, bad ) );
			REJECTS( World_SetContactTuning( worldId, bad, 10.0f, 3.0f ) );
			REJECTS( World_SetContactTuning( worldId, 30.0f, bad, 3.0f ) );
			REJECTS( World_SetContactTuning( worldId, 30.0f, 10.0f, bad ) );
			REJECTS( World_Step( worldId, bad, 4 ) );
		}

		REJECTS( World_SetRestitutionIterations( worldId, -1 ) );

		ACCEPTS( World_Step( worldId, 1.0f / 60.0f, 4 ) );

		EndRejectCount();

		ENSURE( World_GetGravity( worldId ).x == gravity.x && World_GetGravity( worldId ).y == gravity.y );
		ENSURE( World_GetRestitutionThreshold( worldId ) == restitutionThreshold );
		ENSURE( World_GetHitEventThreshold( worldId ) == hitEventThreshold );
		ENSURE( World_GetContactRecycleDistance( worldId ) == recycleDistance );
		ENSURE( World_GetMaxLinearSpeed( worldId ) == maximumLinearSpeed );
		ENSURE( World_GetRestitutionIterations( worldId ) == restitutionIterations );

		DestroyWorld( worldId );
		return 0;
	}

	private static int BodyInputTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		BeginRejectCount();

		for ( int i = 0; i < badValues.Length; ++i )
		{
			float bad = badValues[i];
			Vec2 badVec = new Vec2( bad, 0.0f );
			Pos badPos = new Pos { x = bad, y = 0.0f };
			Rot badRot = new Rot( bad, 0.0f );
			WorldTransform badTarget = new WorldTransform { p = badPos, q = Rot_identity };

			REJECTS( Body_SetLinearVelocity( bodyId, badVec ) );
			REJECTS( Body_SetAngularVelocity( bodyId, bad ) );
			REJECTS( Body_SetTransform( bodyId, badPos, Rot_identity ) );
			REJECTS( Body_SetTransform( bodyId, Pos_zero, badRot ) );
			REJECTS( Body_SetTargetTransform( bodyId, badTarget, 1.0f / 60.0f, true ) );
			REJECTS( Body_SetTargetTransform( bodyId, WorldTransform_identity, bad, true ) );
			REJECTS( Body_ApplyForce( bodyId, badVec, Pos_zero, true ) );
			REJECTS( Body_ApplyForce( bodyId, Vec2_zero, badPos, true ) );
			REJECTS( Body_ApplyForceToCenter( bodyId, badVec, true ) );
			REJECTS( Body_ApplyTorque( bodyId, bad, true ) );
			REJECTS( Body_ApplyLinearImpulse( bodyId, badVec, Pos_zero, true ) );
			REJECTS( Body_ApplyLinearImpulse( bodyId, Vec2_zero, badPos, true ) );
			REJECTS( Body_ApplyLinearImpulseToCenter( bodyId, badVec, true ) );
			REJECTS( Body_ApplyAngularImpulse( bodyId, bad, true ) );
			REJECTS( Body_SetMassData( bodyId, new MassData { mass = bad, center = Vec2_zero, rotationalInertia = 1.0f } ) );
			REJECTS( Body_SetMassData( bodyId, new MassData { mass = 1.0f, center = badVec, rotationalInertia = 1.0f } ) );
			REJECTS( Body_SetMassData( bodyId, new MassData { mass = 1.0f, center = Vec2_zero, rotationalInertia = bad } ) );
			REJECTS( Body_SetLinearDamping( bodyId, bad ) );
			REJECTS( Body_SetAngularDamping( bodyId, bad ) );
			REJECTS( Body_SetGravityScale( bodyId, bad ) );
			REJECTS( Body_SetSleepThreshold( bodyId, bad ) );
			REJECTS( Body_SetSafetyFactor( bodyId, bad ) );
		}

		// A negative value on a quantity the API documents as non-negative is turned away too,
		// since negative damping grows velocity without bound.
		REJECTS( Body_SetLinearDamping( bodyId, -1.0f ) );
		REJECTS( Body_SetAngularDamping( bodyId, -1.0f ) );

		ACCEPTS( Body_SetLinearVelocity( bodyId, new Vec2( 1.0f, 2.0f ) ) );
		ACCEPTS( Body_ApplyForceToCenter( bodyId, new Vec2( 0.0f, 10.0f ), true ) );

		for ( int i = 0; i < 30; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		EndRejectCount();

		ENSURE( IsBodyFinite( bodyId ) );

		DestroyWorld( worldId );
		return 0;
	}

	private static int ShapeInputTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		ShapeId shapeId = CreatePolygonShape( bodyId, ref shapeDef, ref box );

		float density = Shape_GetDensity( shapeId );
		float friction = Shape_GetFriction( shapeId );
		float restitution = Shape_GetRestitution( shapeId );

		BeginRejectCount();

		for ( int i = 0; i < badValues.Length; ++i )
		{
			float bad = badValues[i];
			Vec2 badVec = new Vec2( bad, 0.0f );

			Circle badCircle = new Circle { center = badVec, radius = 1.0f };
			Capsule badCapsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = bad };
			Segment badSegment = new Segment { point1 = badVec, point2 = new( 1.0f, 0.0f ) };
			ChainSegment badChainSegment = new ChainSegment { ghost1 = badVec, segment = new() { point1 = new( 0.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) }, ghost2 = new( 2.0f, 0.0f ), chainId = -1 };

			Polygon badPolygon = box;
			badPolygon.radius = bad;

			Polygon badVertexPolygon = box;
			badVertexPolygon.vertices[0] = badVec;

			SurfaceMaterial badMaterial = DefaultSurfaceMaterial();
			badMaterial.friction = bad;

			REJECTS( Shape_SetDensity( shapeId, bad, true ) );
			REJECTS( Shape_SetFriction( shapeId, bad ) );
			REJECTS( Shape_SetRestitution( shapeId, bad ) );
			REJECTS( Shape_SetSurfaceMaterial( shapeId, ref badMaterial ) );
			REJECTS( Shape_ApplyWind( shapeId, badVec, 1.0f, 1.0f, true ) );
			REJECTS( Shape_ApplyWind( shapeId, Vec2_zero, bad, 1.0f, true ) );
			REJECTS( Shape_ApplyWind( shapeId, Vec2_zero, 1.0f, bad, true ) );
			REJECTS( Shape_SetCircle( shapeId, ref badCircle ) );
			REJECTS( Shape_SetCapsule( shapeId, ref badCapsule ) );
			REJECTS( Shape_SetSegment( shapeId, ref badSegment ) );
			REJECTS( Shape_SetChainSegment( shapeId, ref badChainSegment ) );
			REJECTS( Shape_SetPolygon( shapeId, ref badPolygon ) );
			REJECTS( Shape_SetPolygon( shapeId, ref badVertexPolygon ) );
		}

		ACCEPTS( Shape_SetFriction( shapeId, 0.4f ) );

		for ( int i = 0; i < 10; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		EndRejectCount();

		// The rejected geometry never reached the shape, so the proxy is still the original box
		ENSURE( Shape_GetType( shapeId ) == ShapeType.polygonShape );
		ENSURE( IsValidAABB( Shape_GetBounds( shapeId ) ) );
		ENSURE( IsValidAABB( World_GetBounds( worldId ) ) );
		ENSURE( Shape_GetDensity( shapeId ) == density );
		ENSURE( Shape_GetFriction( shapeId ) == 0.4f && friction == friction );
		ENSURE( Shape_GetRestitution( shapeId ) == restitution );
		ENSURE( IsBodyFinite( bodyId ) );

		DestroyWorld( worldId );
		return 0;
	}

	private static int JointInputTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.@base.bodyIdA = groundId;
		jointDef.@base.bodyIdB = bodyId;
		jointDef.@base.localFrameB.p = new Vec2( 0.0f, 1.0f );
		JointId jointId = CreateRevoluteJoint( worldId, ref jointDef );

		BeginRejectCount();

		for ( int i = 0; i < badValues.Length; ++i )
		{
			float bad = badValues[i];
			Transform badFrame = new Transform { p = new( bad, 0.0f ), q = Rot_identity };

			REJECTS( Joint_SetLocalFrameA( jointId, badFrame ) );
			REJECTS( Joint_SetLocalFrameB( jointId, badFrame ) );
			REJECTS( Joint_SetConstraintTuning( jointId, bad, 1.0f ) );
			REJECTS( Joint_SetConstraintTuning( jointId, 60.0f, bad ) );
			REJECTS( Joint_SetForceThreshold( jointId, bad ) );
			REJECTS( Joint_SetTorqueThreshold( jointId, bad ) );
			REJECTS( RevoluteJoint_SetSpringHertz( jointId, bad ) );
			REJECTS( RevoluteJoint_SetSpringDampingRatio( jointId, bad ) );
			REJECTS( RevoluteJoint_SetTargetAngle( jointId, bad ) );
			REJECTS( RevoluteJoint_SetLimits( jointId, bad, 1.0f ) );
			REJECTS( RevoluteJoint_SetLimits( jointId, -1.0f, bad ) );
			REJECTS( RevoluteJoint_SetTargetMotorSpeed( jointId, bad ) );
			REJECTS( RevoluteJoint_SetMaxMotorTorque( jointId, bad ) );
		}

		ACCEPTS( RevoluteJoint_SetTargetMotorSpeed( jointId, 2.0f ) );

		for ( int i = 0; i < 30; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		EndRejectCount();

		ENSURE( IsBodyFinite( bodyId ) );
		ENSURE( IsValidVec2( Joint_GetConstraintForce( jointId ) ) );
		ENSURE( IsValidFloat( Joint_GetConstraintTorque( jointId ) ) );

		DestroyWorld( worldId );
		return 0;
	}

	private static int CreateInputTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		BeginRejectCount();

		for ( int i = 0; i < badValues.Length; ++i )
		{
			float bad = badValues[i];
			Vec2 badVec = new Vec2( bad, 0.0f );

			WorldDef badWorldDef = DefaultWorldDef();
			badWorldDef.gravity = badVec;
			WorldId badWorldId = CreateWorld( ref badWorldDef );
			ENSURE( World_IsValid( badWorldId ) == false );

			BodyDef badBodyDef = DefaultBodyDef();
			badBodyDef.position = new Pos { x = bad, y = 0.0f };
			ENSURE( Body_IsValid( CreateBody( worldId, ref badBodyDef ) ) == false );

			ShapeDef badShapeDef = DefaultShapeDef();
			badShapeDef.density = bad;
			ENSURE( Shape_IsValid( CreatePolygonShape( bodyId, ref badShapeDef, ref box ) ) == false );

			Circle badCircle = new Circle { center = badVec, radius = 1.0f };
			ENSURE( Shape_IsValid( CreateCircleShape( bodyId, ref shapeDef, ref badCircle ) ) == false );

			Capsule badCapsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = bad };
			ENSURE( Shape_IsValid( CreateCapsuleShape( bodyId, ref shapeDef, ref badCapsule ) ) == false );

			Segment badSegment = new Segment { point1 = badVec, point2 = new( 1.0f, 0.0f ) };
			ENSURE( Shape_IsValid( CreateSegmentShape( bodyId, ref shapeDef, ref badSegment ) ) == false );

			Polygon badPolygon = box;
			badPolygon.radius = bad;
			ENSURE( Shape_IsValid( CreatePolygonShape( bodyId, ref shapeDef, ref badPolygon ) ) == false );

			Vec2[] chainPoints = { new( -4.0f, 0.0f ), new( 0.0f, 0.0f ), new( 4.0f, 0.0f ), new( 8.0f, 0.0f ) };
			chainPoints[2] = badVec;
			SurfaceMaterial material = DefaultSurfaceMaterial();
			ChainDef badChainDef = DefaultChainDef();
			badChainDef.points = chainPoints;
			badChainDef.pointCount = 4;
			badChainDef.materials = &material;
			badChainDef.materialCount = 1;
			badChainDef.isLoop = true;
			ENSURE( Chain_IsValid( CreateChain( bodyId, ref badChainDef ) ) == false );

			RevoluteJointDef badJointDef = DefaultRevoluteJointDef();
			badJointDef.@base.bodyIdA = bodyId;
			badJointDef.@base.bodyIdB = CreateBody( worldId, ref bodyDef );
			badJointDef.hertz = bad;
			ENSURE( Joint_IsValid( CreateRevoluteJoint( worldId, ref badJointDef ) ) == false );

			ExplosionDef badExplosionDef = DefaultExplosionDef();
			badExplosionDef.radius = bad;
			REJECTS( World_Explode( worldId, ref badExplosionDef ) );
		}

		for ( int i = 0; i < 10; ++i )
		{
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		EndRejectCount();

		ENSURE( IsBodyFinite( bodyId ) );
		ENSURE( IsValidAABB( World_GetBounds( worldId ) ) );

		DestroyWorld( worldId );
		return 0;
	}

	private static int RecordReplayTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );
		worldDef.workerCount = 1;
		WorldId worldId = CreateWorld( ref worldDef );

		Recording recording = CreateRecording( 0 );
		World_StartRecording( worldId, recording );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Segment ground = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref ground );

		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 4.0f };
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		BeginRejectCount();

		// Interleave turned away calls with real ones so the op stream carries both. A mutator is
		// rejected before it is recorded, while a create is recorded with the null id it returned.
		for ( int i = 0; i < 30; ++i )
		{
			float bad = badValues[i % badValues.Length];

			REJECTS( Body_ApplyForceToCenter( bodyId, new Vec2( bad, 0.0f ), true ) );
			REJECTS( Body_SetAngularVelocity( bodyId, bad ) );

			ShapeDef badShapeDef = DefaultShapeDef();
			badShapeDef.density = bad;
			ENSURE( Shape_IsValid( CreatePolygonShape( bodyId, ref badShapeDef, ref box ) ) == false );

			Body_ApplyForceToCenter( bodyId, new Vec2( 0.0f, 1.0f ), true );
			World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		ENSURE( IsBodyFinite( bodyId ) );

		World_StopRecording( worldId );
		DestroyWorld( worldId );

		ref readonly byte data = ref Recording_GetData( recording );
		int size = Recording_GetSize( recording );
		ENSURE( size > 0 );
		ENSURE( ValidateRecording( data, size, 0 ) );
		ENSURE( ValidateRecording( data, size, 4 ) );

		EndRejectCount();

		DestroyRecording( recording );
		return 0;
	}

	internal static int InvalidInputTest()
	{
		RUN_SUBTEST( WorldInputTest );
		RUN_SUBTEST( BodyInputTest );
		RUN_SUBTEST( ShapeInputTest );
		RUN_SUBTEST( JointInputTest );
		RUN_SUBTEST( CreateInputTest );
		RUN_SUBTEST( RecordReplayTest );

		return 0;
	}

}
