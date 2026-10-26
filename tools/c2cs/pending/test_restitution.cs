// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class RestitutionTests
{
	// #define TIME_STEP ( 1.0f / 60.0f )
	private const int SUB_STEP_COUNT = 4;
	// A two point landing converges its Gauss-Seidel coupling in four stage iterations and the default
	// is two, so the tests that read the coefficient or the spin of a flat landing pin the count
	private const int RESTITUTION_ITERATIONS = 4;
	// Impact speed shared by every scenario that measures a coefficient. Well above the default
	// restitution threshold so the bounce is always armed.
	private const float IMPACT_SPEED = 5.0f;
	// Each subtest prints all of its measurements before asserting, and the battery runs every subtest
	// so one bad scenario does not hide the rest.
	// #define RUN_MEASUREMENT( T )                                                                                                     \
		do                                                                                                                           \
		{                                                                                                                            \
			if ( T() != 0 )                                                                                                          \
			{                                                                                                                        \
				Console.WriteLine( $"  subtest failed: " #T "" );                                                                              \
				failureCount += 1;                                                                                                   \
			}                                                                                                                        \
			else                                                                                                                     \
			{                                                                                                                        \
				Console.WriteLine( $"  subtest passed: " #T "" );                                                                              \
			}                                                                                                                        \
		}                                                                                                                            \
		while ( false )

	private static WorldId MakeWorld( float gravityY )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, gravityY );
		worldDef.enableSleep = false;
		return CreateWorld( ref worldDef );
	}

	private static WorldId MakeWorldIterations( float gravityY, int restitutionIterations )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, gravityY );
		worldDef.enableSleep = false;
		worldDef.restitutionIterations = restitutionIterations;
		return CreateWorld( ref worldDef );
	}

	// Ground with its top surface at y = 0
	private static void MakeGround( WorldId worldId, float restitution )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
		BodyId groundId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;
		Polygon box = MakeBox( 40.0f, 1.0f );
		CreatePolygonShape( groundId, ref shapeDef, ref box );
	}

	private static BodyId MakeBall( WorldId worldId, float x, float y, float velocityY, float restitution )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = x, y = y };
		bodyDef.linearVelocity = new Vec2( 0.0f, velocityY );
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		CreateCircleShape( bodyId, ref shapeDef, ref circle );
		return bodyId;
	}

	// Two free circles closing head on with no gravity. The coefficient is the ratio of relative normal
	// speeds at the contact, which is the mass independent definition.
	private static float MeasureHeadOn( float restitution, float densityB, ref float momentumError )
	{
		WorldId worldId = MakeWorld( 0.0f );

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -1.0f, y = 0.0f };
		bodyDef.linearVelocity = new Vec2( 0.5f * IMPACT_SPEED, 0.0f );
		BodyId idA = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( idA, ref shapeDef, ref circle );

		bodyDef.position = new Pos { x = 1.0f, y = 0.0f };
		bodyDef.linearVelocity = new Vec2( -0.5f * IMPACT_SPEED, 0.0f );
		BodyId idB = CreateBody( worldId, ref bodyDef );
		shapeDef.density = densityB;
		CreateCircleShape( idB, ref shapeDef, ref circle );

		float massA = Body_GetMass( idA );
		float massB = Body_GetMass( idB );
		float momentum0 = massA * Body_GetLinearVelocity( idA ).x + massB * Body_GetLinearVelocity( idB ).x;

		for ( int i = 0; i < 120; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float vA = Body_GetLinearVelocity( idA ).x;
		float vB = Body_GetLinearVelocity( idB ).x;
		float momentum1 = massA * vA + massB * vB;

		DestroyWorld( worldId );

		momentumError = ( momentum1 - momentum0 ) / ( massA + massB );
		return ( vB - vA ) / IMPACT_SPEED;
	}

	// Ball driven into static ground with no gravity so the measurement carries no gravity bias. The
	// gap shifts where inside the time step the impact lands.
	private static float MeasureGroundBounce( float restitution, float gap )
	{
		WorldId worldId = MakeWorld( 0.0f );
		MakeGround( worldId, 0.0f );

		BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + gap, -IMPACT_SPEED, restitution );

		for ( int i = 0; i < 60; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float speed = Body_GetLinearVelocity( ballId ).y;
		DestroyWorld( worldId );
		return speed / IMPACT_SPEED;
	}

	// Impactor driven onto a column of resting balls pinned against the ground. The support balls are
	// dead so only the top contact can bounce. A supported target has infinite effective mass along the
	// normal, so the coefficient must not depend on the column height.
	private static float MeasureSupportedBounce( float restitution, int supportCount, bool propagate )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, 0.0f );
		worldDef.enableSleep = false;
		worldDef.enableRestitutionPropagation = propagate;
		worldDef.restitutionIterations = RESTITUTION_ITERATIONS;
		WorldId worldId = CreateWorld( ref worldDef );
		MakeGround( worldId, 0.0f );

		for ( int i = 0; i < supportCount; ++i )
		{
			MakeBall( worldId, 0.0f, 0.5f + 1.0f * i, 0.0f, 0.0f );
		}

		float y = 0.5f + 1.0f * supportCount + 0.5f * IMPACT_SPEED * TIME_STEP;
		BodyId ballId = MakeBall( worldId, 0.0f, y, -IMPACT_SPEED, restitution );

		for ( int i = 0; i < 60; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float speed = Body_GetLinearVelocity( ballId ).y;
		DestroyWorld( worldId );
		return speed / IMPACT_SPEED;
	}

	// Flat box landing on both corners at once with no gravity
	private static float MeasureFlatBounce( float restitution, int subStepCount, ref float spin )
	{
		WorldId worldId = MakeWorldIterations( 0.0f, RESTITUTION_ITERATIONS );
		MakeGround( worldId, 0.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 0.25f + 0.5f * IMPACT_SPEED * TIME_STEP };
		bodyDef.linearVelocity = new Vec2( 0.0f, -IMPACT_SPEED );
		BodyId boxId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;
		Polygon box = MakeBox( 1.0f, 0.25f );
		CreatePolygonShape( boxId, ref shapeDef, ref box );

		for ( int i = 0; i < 60; ++i )
		{
			World_Step( worldId, TIME_STEP, subStepCount );
		}

		float speed = Body_GetLinearVelocity( boxId ).y;
		spin = Body_GetAngularVelocity( boxId );
		DestroyWorld( worldId );
		return speed / IMPACT_SPEED;
	}

	// Perfectly elastic ball dropped onto a segment, the shape the Restitution sample uses
	private static int MeasureDrop( float dropHeight, Span<float> apexes, int capacity )
	{
		WorldId worldId = MakeWorld( -10.0f );

		BodyDef groundDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref groundDef );
		ShapeDef groundShape = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -60.0f, 0.0f ), point2 = new( 60.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref groundShape, ref segment );

		BodyId ballId = MakeBall( worldId, 0.0f, dropHeight, 0.0f, 1.0f );

		int apexCount = 0;
		float previousSpeed = 0.0f;

		for ( int i = 0; i < 4000 && apexCount < capacity; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

			float speed = Body_GetLinearVelocity( ballId ).y;
			if ( previousSpeed > 0.0f && speed <= 0.0f )
			{
				apexes[apexCount] = (float)Body_GetPosition( ballId ).y;
				apexCount += 1;
			}
			previousSpeed = speed;
		}

		DestroyWorld( worldId );
		return apexCount;
	}

	private static int HeadOnTest()
	{
		float[] restitutions = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
		float[] densities = { 1.0f, 10.0f, 100.0f };
		const float tolerance = 0.02f;

		int failed = 0;

		for ( int i = 0; i < densities.Length; ++i )
		{
			float worstError = 0.0f;
			float worstMomentum = 0.0f;

			for ( int j = 0; j < restitutions.Length; ++j )
			{
				float momentumError = 0.0f;
				float measured = MeasureHeadOn( restitutions[j], densities[i], ref momentumError );

				float error = AbsFloat( measured - restitutions[j] );
				if ( error > worstError )
				{
					worstError = error;
				}

				if ( AbsFloat( momentumError ) > AbsFloat( worstMomentum ) )
				{
					worstMomentum = momentumError;
				}

				if ( error > tolerance )
				{
					Console.WriteLine( $"    head on density {densities[i]:F0} e {restitutions[j]:F2} . {measured:F4}" );
					failed = 1;
				}
			}

			Console.WriteLine( $"    head on density {densities[i]:F1} worst error {worstError:F4} momentum drift %.2e" );

			if ( AbsFloat( worstMomentum ) > 1.0e-4f )
			{
				failed = 1;
			}
		}

		return failed;
	}

	private static int PhaseTest()
	{
		float[] restitutions = { 0.25f, 0.5f, 0.9f };
		const int sampleCount = 16;
		const float tolerance = 0.05f;

		int failed = 0;

		for ( int j = 0; j < restitutions.Length; ++j )
		{
			float minimum = FLT_MAX;
			float maximum = -FLT_MAX;
			float sum = 0.0f;

			for ( int i = 0; i < sampleCount; ++i )
			{
				// Sweep the gap across one step of travel so the impact lands at every phase
				float gap = ( (float)i / (float)sampleCount ) * IMPACT_SPEED * TIME_STEP;
				float measured = MeasureGroundBounce( restitutions[j], gap );
				minimum = MinFloat( minimum, measured );
				maximum = MaxFloat( maximum, measured );
				sum += measured;
			}

			float mean = sum / sampleCount;
			float spread = maximum - minimum;
			Console.WriteLine( $"    phase e {restitutions[j]:F2} . mean {mean:F4} spread {spread:F4} [{minimum:F4}, {maximum:F4}]" );

			if ( spread > tolerance || AbsFloat( mean - restitutions[j] ) > tolerance )
			{
				failed = 1;
			}
		}

		return failed;
	}

	// By default the restitution stage sweeps only the contacts that carry restitution, so the ball
	// pushes the column down and the ground under it gets no say until the next step: e = 0.9 gives
	// about 0.52 on one support. With propagation enabled the stage sweeps every contact, the ground
	// contact supplies the reaction within the stage and e = 0.9 gives 0.85 on one support and 0.73
	// on three at the pinned four iterations (0.71 and 0.65 at the default two), decaying with depth
	// because the column needs an iteration per level. Propagation costs a normal pass over every
	// contact per iteration, hence the option. Both modes are gated: never above the free coefficient,
	// which would be energy from nowhere, and never collapsed.
	private static int SupportedTest()
	{
		float[] restitutions = { 0.5f, 0.9f };

		int failed = 0;

		for ( int propagate = 0; propagate <= 1; ++propagate )
		{
			float floor = propagate ? 0.75f : 0.5f;

			for ( int j = 0; j < restitutions.Length; ++j )
			{
				for ( int n = 0; n <= 3; ++n )
				{
					float measured = MeasureSupportedBounce( restitutions[j], n, propagate == 1 );
					Console.WriteLine( $"    supported e {restitutions[j]:F2} supports {n} propagation {propagate} . {measured:F4}" );

					if ( measured > restitutions[j] + 0.01f || measured < floor * restitutions[j] )
					{
						failed = 1;
					}
				}
			}
		}

		return failed;
	}

	// A symmetric two point landing. The stage solves the two points in sequence, so a single sweep
	// leaves the box spinning: the first impulse tilts it and the second point sees a larger closing
	// speed. Four stage iterations converge the coupling and leave a small residual, two leave the box
	// with 0.3 rad/s. The tolerance admits that residual and rejects the single sweep behavior.
	private static int TwoPointTest()
	{
		float[] restitutions = { 0.5f, 0.9f };
		int[] subStepCounts = { 4, 8 };
		const float speedTolerance = 0.01f;
		const float spinTolerance = 0.03f;

		int failed = 0;

		for ( int j = 0; j < restitutions.Length; ++j )
		{
			for ( int k = 0; k < subStepCounts.Length; ++k )
			{
				float spin = 0.0f;
				float measured = MeasureFlatBounce( restitutions[j], subStepCounts[k], ref spin );
				Console.WriteLine( $"    two point e {restitutions[j]:F2} substeps {subStepCounts[k]} . {measured:F4} spin {spin:F4}" );

				// Only the shipping sub step count is a gate. The wider count is reported so a
				// convergence problem can be told apart from a formulation problem.
				if ( subStepCounts[k] != SUB_STEP_COUNT )
				{
					continue;
				}

				if ( AbsFloat( measured - restitutions[j] ) > speedTolerance || AbsFloat( spin ) > spinTolerance )
				{
					failed = 1;
				}
			}
		}

		return failed;
	}

	// A flat box carrying spin. Both corners stay in contact and the constraints are linear in the body
	// velocity, so perfect restitution reverses linear and angular velocity. Four stage iterations
	// bring both within a few hundredths. One left a residual an order of magnitude larger.
	private static int SpinTest()
	{
		float[] spins = { 0.0f, 0.5f, 1.0f, 2.0f };

		int failed = 0;

		for ( int j = 0; j < spins.Length; ++j )
		{
			WorldId worldId = MakeWorldIterations( 0.0f, RESTITUTION_ITERATIONS );
			MakeGround( worldId, 0.0f );

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 0.25f + 0.5f * IMPACT_SPEED * TIME_STEP };
			bodyDef.linearVelocity = new Vec2( 0.0f, -IMPACT_SPEED );
			bodyDef.angularVelocity = spins[j];
			BodyId boxId = CreateBody( worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.material.restitution = 1.0f;
			Polygon box = MakeBox( 1.0f, 0.25f );
			CreatePolygonShape( boxId, ref shapeDef, ref box );

			for ( int i = 0; i < 60; ++i )
			{
				World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			}

			float speed = Body_GetLinearVelocity( boxId ).y;
			float spin = Body_GetAngularVelocity( boxId );
			DestroyWorld( worldId );

			Console.WriteLine( $"    spin in {spins[j]:F2} . vy {speed:F4} (want {IMPACT_SPEED:F4})  w {spin:F4} (want {-spins[j]:F4})" );

			if ( AbsFloat( speed - IMPACT_SPEED ) > 0.05f || AbsFloat( spin + spins[j] ) > 0.05f )
			{
				failed = 1;
			}
		}

		return failed;
	}

	// Perfectly elastic ball under gravity. Only heights where continuous collision engages are used.
	// Continuous collision lands the ball on the surface, so the bounce is armed from the true impact
	// speed and the apex holds. Slower drops resolve the impact inside the overlap and the penetration
	// recovery adds height, by design. The 10 m case is the long horizon read: the per manifold energy
	// budget crept up 0.05% a bounce there while 20 m and 40 m looked clean.
	private static int DropTest()
	{
		float[] heights = { 40.0f, 20.0f, 10.0f };

		int failed = 0;

		for ( int j = 0; j < heights.Length; ++j )
		{
			float[] apexes = { 0 };
			int apexCount = MeasureDrop( heights[j], apexes, apexes.Length );

			Console.WriteLine( $"    drop {heights[j]:F1} ." );
			for ( int i = 0; i < apexCount; ++i )
			{
				Console.WriteLine( $" {apexes[i]:F3}" );
			}
			Console.WriteLine( $"" );

			if ( apexCount < apexes.Length )
			{
				failed = 1;
				continue;
			}

			float highest = apexes[0];
			for ( int i = 1; i < apexCount; ++i )
			{
				highest = MaxFloat( highest, apexes[i] );
			}

			// A later apex above the first, or a first apex above the drop, is energy from nowhere
			if ( highest > 1.001f * apexes[0] || apexes[0] > 1.001f * heights[j] || apexes[apexCount - 1] < 0.8f * apexes[0] )
			{
				failed = 1;
			}
		}

		return failed;
	}

	// Mirrors the SingleBoxRestitution sample: square box, perfectly elastic, dropped flat onto a
	// segment with an aggressive continuous safety factor so it lands square on the surface. A flat
	// landing solves two coplanar points in sequence, and one sweep over-delivers: the first impulse
	// tilts the box, the second point sees a larger closing speed and the sum overshoots the rigid body
	// answer, leaving spin that tilts the box for the next landing. Four stage iterations converge the
	// coupling, at the default two the plank leaves with 0.13 rad/s and tumbles on its second bounce.
	// The bounds below reject the single sweep behavior and any apex above the drop.
	private static int SingleBoxTest()
	{
		WorldId worldId = MakeWorldIterations( -10.0f, RESTITUTION_ITERATIONS );

		BodyDef groundDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref groundDef );
		ShapeDef groundShape = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref groundShape, ref segment );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		Polygon box = MakeBox( 0.5f, 0.5f );

		const float dropHeight = 10.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = dropHeight };
		bodyDef.safetyFactor = 0.01f;
		BodyId boxId = CreateBody( worldId, ref bodyDef );
		CreatePolygonShape( boxId, ref shapeDef, ref box );

		float firstSpin = 0.0f;
		float[] apexes = { 0 };
		float[] spins = { 0 };
		int apexCount = 0;
		float previousSpeed = 0.0f;

		// A nearly elastic bounce from 10 m takes about 165 steps, so budget for four of them
		for ( int i = 0; i < 1200 && apexCount < apexes.Length; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

			float speed = Body_GetLinearVelocity( boxId ).y;

			if ( apexCount == 0 && previousSpeed <= 0.0f && speed > 0.0f )
			{
				firstSpin = AbsFloat( Body_GetAngularVelocity( boxId ) );
			}

			if ( previousSpeed > 0.0f && speed <= 0.0f )
			{
				apexes[apexCount] = (float)Body_GetPosition( boxId ).y;
				spins[apexCount] = AbsFloat( Body_GetAngularVelocity( boxId ) );
				apexCount += 1;
			}
			previousSpeed = speed;
		}

		DestroyWorld( worldId );

		Console.WriteLine( $"    single box first bounce spin {firstSpin:F4} apexes" );
		for ( int i = 0; i < apexCount; ++i )
		{
			Console.WriteLine( $" {apexes[i]:F3} (spin {spins[i]:F2})" );
		}
		Console.WriteLine( $"" );

		ENSURE( apexCount == apexes.Length );

		int failed = 0;

		if ( firstSpin > 0.01f )
		{
			failed = 1;
		}

		for ( int i = 0; i < apexCount; ++i )
		{
			if ( apexes[i] < 0.95f * dropHeight || apexes[i] > 1.001f * dropHeight )
			{
				failed = 1;
			}
		}

		return failed;
	}

	private static int ThresholdTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		float threshold = worldDef.restitutionThreshold;

		float[] scales = { 0.5f, 0.9f, 1.5f, 4.0f };

		int failed = 0;

		for ( int j = 0; j < scales.Length; ++j )
		{
			float speed = scales[j] * threshold;

			WorldId worldId = MakeWorld( 0.0f );
			MakeGround( worldId, 0.0f );
			BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + 0.5f * speed * TIME_STEP, -speed, 1.0f );

			for ( int i = 0; i < 60; ++i )
			{
				World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			}

			float ratio = Body_GetLinearVelocity( ballId ).y / speed;
			DestroyWorld( worldId );

			Console.WriteLine( $"    threshold {scales[j]:F2}x . {ratio:F4}" );

			if ( scales[j] < 1.0f && ratio > 0.05f )
			{
				failed = 1;
			}

			if ( scales[j] > 1.0f && ratio < 0.9f )
			{
				failed = 1;
			}
		}

		return failed;
	}

	// High restitution must not wake a settled stack back up
	private static int RestingTest()
	{
		WorldId worldId = MakeWorld( -10.0f );
		MakeGround( worldId, 0.9f );

		const int boxCount = 10;
		Span<BodyId> boxIds = stackalloc BodyId[10];

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.6f;
		shapeDef.material.restitution = 0.9f;
		Polygon box = MakeBox( 0.5f, 0.5f );

		for ( int i = 0; i < boxCount; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 0.5f + 1.0f * i };
			boxIds[i] = CreateBody( worldId, ref bodyDef );
			CreatePolygonShape( boxIds[i], ref shapeDef, ref box );
		}

		for ( int i = 0; i < 120; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float settled = (float)Body_GetPosition( boxIds[boxCount - 1] ).y;
		float drift = 0.0f;
		float peakSpeed = 0.0f;

		for ( int i = 0; i < 300; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

			float y = (float)Body_GetPosition( boxIds[boxCount - 1] ).y;
			drift = MaxFloat( drift, AbsFloat( y - settled ) );

			for ( int j = 0; j < boxCount; ++j )
			{
				peakSpeed = MaxFloat( peakSpeed, Length( Body_GetLinearVelocity( boxIds[j] ) ) );
			}
		}

		DestroyWorld( worldId );

		Console.WriteLine( $"    resting drift {drift:F5} peak speed {peakSpeed:F5}" );

		ENSURE( drift < 0.01f );
		ENSURE( peakSpeed < 0.05f );
		return 0;
	}

	// The manifold point normal velocity is published only for contacts that enabled hit events, and it
	// is not a stale value otherwise, it is zero, so a reader can tell "not measured" from "measured
	// zero" by whether the shape enables the events. Restitution no longer reads it at all, so this is
	// the only thing keeping the field alive.
	private static float MeasureFirstTouchNormalVelocity( bool enableHitEvents )
	{
		WorldId worldId = MakeWorld( 0.0f );
		MakeGround( worldId, 0.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 0.6f };
		BodyId ballId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;
		shapeDef.enableHitEvents = enableHitEvents;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		CreateCircleShape( ballId, ref shapeDef, ref circle );

		float normalVelocity = FLT_MAX;

		// Hold the closing speed so the sampled value does not depend on which step the pair is created
		for ( int i = 0; i < 20; ++i )
		{
			Body_SetLinearVelocity( ballId, new Vec2( 0.0f, -IMPACT_SPEED ) );
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

			ContactData contactData = default;
			if ( Body_GetContactData( ballId, ref contactData, 1 ) == 1 && contactData.manifold.pointCount > 0 )
			{
				normalVelocity = contactData.manifold.points[0].normalVelocity;
				break;
			}
		}

		DestroyWorld( worldId );
		return normalVelocity;
	}

	private static int NormalVelocityTest()
	{
		float quiet = MeasureFirstTouchNormalVelocity( false );
		float published = MeasureFirstTouchNormalVelocity( true );

		Console.WriteLine( $"    normal velocity hit events off {quiet:F4} on {published:F4} (closing at {-IMPACT_SPEED:F4})" );

		int failed = 0;

		if ( quiet != 0.0f )
		{
			failed = 1;
		}

		if ( published > -0.8f * IMPACT_SPEED )
		{
			failed = 1;
		}

		return failed;
	}

	// Mechanical energy of a body as symplectic Euler conserves it. The integrator updates velocity
	// before position, so the quantity that is exactly constant in free flight is not kinetic plus
	// potential but that plus half a sub-step of the gravity power, m * dot(v, g) * h / 2. Plain
	// kinetic plus potential sampled at step boundaries jumps by about g * v * h at every bounce and
	// drifts down by g^2 h^2 / 2 each sub-step, which would hide a real gain or fake one. With this
	// measure a contact that never does positive work leaves the energy flat to round-off.
	private static float MeasureEnergy( BodyId bodyId, WorldId worldId )
	{
		MassData massData = Body_GetMassData( bodyId );
		Vec2 v = Body_GetLinearVelocity( bodyId );
		float w = Body_GetAngularVelocity( bodyId );
		Vec2 gravity = MulSV( Body_GetGravityScale( bodyId ), World_GetGravity( worldId ) );
		float h = TIME_STEP / SUB_STEP_COUNT;

		float kinetic = 0.5f * massData.mass * Dot( v, v ) + 0.5f * massData.rotationalInertia * w * w;
		float potential = -massData.mass * Dot( gravity, ToVec2( Body_GetWorldCenter( bodyId ) ) );
		float phase = 0.5f * h * massData.mass * Dot( v, gravity );

		return kinetic + potential + phase;
	}

	internal struct OvershootResult
	{
		public float firstApex;
		public float firstEnergyRatio;
		public float peakEnergyRatio;
		public float speedRatio;
		public int bounceCount;
	}

	// A unit box dropped flat on a floor narrower than itself, perfectly elastic, so the two contact
	// points sit inboard of the box corners. A perfectly elastic bounce cannot come back higher than it
	// was dropped from and cannot gain energy.
	private static OvershootResult MeasureOvershoot( bool continuous )
	{
		WorldId worldId = MakeWorldIterations( -10.0f, RESTITUTION_ITERATIONS );

		BodyDef floorDef = DefaultBodyDef();
		floorDef.position = new Pos { x = 0.0f, y = -0.25f };
		BodyId floorId = CreateBody( worldId, ref floorDef );

		ShapeDef floorShape = DefaultShapeDef();
		Polygon floor = MakeBox( 0.375f, 0.25f );
		CreatePolygonShape( floorId, ref floorShape, ref floor );

		const float dropHeight = 10.0f;

		BodyDef boxDef = DefaultBodyDef();
		boxDef.type = BodyType.dynamicBody;
		boxDef.position = new Pos { x = 0.0f, y = dropHeight };
		if ( continuous )
		{
			boxDef.safetyFactor = 0.1f;
		}
		BodyId boxId = CreateBody( worldId, ref boxDef );

		ShapeDef boxShape = DefaultShapeDef();
		boxShape.material.restitution = 1.0f;
		Polygon box = MakeBox( 0.5f, 0.5f );
		CreatePolygonShape( boxId, ref boxShape, ref box );

		OvershootResult result = default;

		float startEnergy = MeasureEnergy( boxId, worldId );
		float impactSpeed = 0.0f;
		float reboundSpeed = 0.0f;
		float bounceHeight = 0.0f;
		float previousSpeed = 0.0f;

		for ( int i = 0; i < 600 && result.bounceCount < 2; ++i )
		{
			float before = Body_GetLinearVelocity( boxId ).y;
			float heightBefore = (float)Body_GetPosition( boxId ).y;
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			float speed = Body_GetLinearVelocity( boxId ).y;

			if ( reboundSpeed == 0.0f && before < 0.0f && speed > 0.0f )
			{
				impactSpeed = -before;
				reboundSpeed = speed;
				bounceHeight = heightBefore;
			}

			if ( reboundSpeed == 0.0f )
			{
				continue;
			}

			// Every step, not just at the apexes, because the impact step itself is where any gain appears
			float ratio = MeasureEnergy( boxId, worldId ) / startEnergy;
			if ( ratio > result.peakEnergyRatio )
			{
				result.peakEnergyRatio = ratio;
			}

			if ( previousSpeed > 0.0f && speed <= 0.0f )
			{
				if ( result.bounceCount == 0 )
				{
					result.firstApex = (float)Body_GetPosition( boxId ).y;
					result.firstEnergyRatio = ratio;
				}

				result.bounceCount += 1;
			}

			previousSpeed = speed;
		}

		DestroyWorld( worldId );

		result.speedRatio = reboundSpeed / impactSpeed;

		Console.WriteLine( $"    overshoot continuous {continuous ? 1 : 0} apex {result.firstApex:F4} of {dropHeight:F4} at y {bounceHeight:F4}, speed ratio {result.speedRatio:F4}, energy {result.firstEnergyRatio:F4} then {result.peakEnergyRatio:F4}" );

		return result;
	}

	// Two runs, because two unrelated effects were tangled together here.
	//
	// The impulse must never return more speed than it received. That is the restitution invariant and it
	// is checked in both runs. The unconverged multi point solve failed it.
	//
	// Everything else depends on whether continuous collision engages. Without it the box moves 0.23 m
	// in the step before contact, just under the 0.25 m trigger, so the first manifold appears with the
	// box already deep. It bounces from down there and climbs back through that depth for free, which is
	// worth a fraction of a metre of apex and a little energy, and none of it is restitution. With
	// continuous collision the box lands on the surface and both the apex and the energy come in under
	// where they started. The safety factor here is the supported answer for a body whose elastic
	// accuracy matters.
	//
	// The discrete run is only gated on its first bounce. Later bounces there are a genuine blow-up, not
	// a tolerance question: the free height tilts the box and the next landing is a deep corner impact.
	// Lowering the safety factor is the fix; the discrete run is kept to pin what happens when it is
	// left alone, and because its speed ratio is what caught the real bug.
	private static int OvershootTest()
	{
		const float dropHeight = 10.0f;

		OvershootResult discrete = MeasureOvershoot( false );
		OvershootResult continuous = MeasureOvershoot( true );

		// Perfectly elastic returns exactly the speed it received, so any excess is round-off. The
		// overflow and colored solvers round differently and either can land an ulp over one.
		float speedTolerance = 1.0e-5f;

		int failed = 0;

		if ( discrete.speedRatio > 1.0f + speedTolerance || continuous.speedRatio > 1.0f + speedTolerance )
		{
			failed = 1;
		}

		if ( continuous.firstApex > dropHeight || continuous.peakEnergyRatio > 1.001f )
		{
			failed = 1;
		}

		if ( discrete.firstApex > dropHeight + 0.15f || discrete.firstEnergyRatio > 1.02f )
		{
			failed = 1;
		}

		if ( discrete.bounceCount < 2 || continuous.bounceCount < 2 )
		{
			failed = 1;
		}

		return failed;
	}

	internal struct ImpulseResult
	{
		public float worstError;
		public float approachSpeed;
		public float firstSeparation;
		public float bounceImpulse;
		public int contactSteps;
		public int toiSteps;
		public int toiImpulseSteps;
	}

	// Ball dropped under gravity with the contact impulse read back every step. Nothing else touches the
	// ball, so once gravity is taken out the change in momentum over a step is the impulse the contact
	// applied, and that is what the total normal impulse must report. A time of impact step is left out
	// of the balance: no contact was solved on it, and the sweep hands back the gravity of the time it
	// cut short.
	private static ImpulseResult MeasureDropImpulse( float restitution, float dropHeight )
	{
		WorldId worldId = MakeWorld( -10.0f );
		MakeGround( worldId, 0.0f );

		BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + dropHeight, 0.0f, restitution );
		World world = GetWorldFromId( worldId );
		float mass = Body_GetMass( ballId );
		float gravityY = World_GetGravity( worldId ).y;

		ImpulseResult result = default;
		bool touched = false;
		bool bouncing = false;

		// Enough for the longest fall and the bounce that follows it
		for ( int i = 0; i < 240; ++i )
		{
			float speedBefore = Body_GetLinearVelocity( ballId ).y;
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			float speedAfter = Body_GetLinearVelocity( ballId ).y;

			float measured = 0.0f;
			float separation = 0.0f;
			FixedArray4<ContactData> contactData = default;
			int contactCount = Body_GetContactData( ballId, contactData, contactData.Length );
			for ( int c = 0; c < contactCount; ++c )
			{
				ref readonly Manifold manifold = ref contactData[c].manifold;
				for ( int p = 0; p < manifold.pointCount; ++p )
				{
					measured += manifold.points[p].totalNormalImpulse;
					separation = manifold.points[p].separation;
				}
			}

			// Transient body flags are cleared and rewritten every step
			ref Body ball = ref GetBodyFullId( world, ballId );
			ref BodySim ballSim = ref GetBodySim( world, ball );
			if ( ( ballSim.flags & b2_hadTimeOfImpact ) != 0 )
			{
				result.toiSteps += 1;
				if ( measured != 0.0f )
				{
					result.toiImpulseSteps += 1;
				}
				continue;
			}

			float expected = mass * ( speedAfter - speedBefore ) - mass * gravityY * TIME_STEP;
			result.worstError = MaxFloat( result.worstError, AbsFloat( measured - expected ) );

			if ( contactCount > 0 )
			{
				if ( touched == false )
				{
					touched = true;
					bouncing = true;
					result.approachSpeed = -speedBefore;
					result.firstSeparation = separation;
				}

				if ( bouncing )
				{
					result.bounceImpulse += measured;
					result.contactSteps += 1;
				}
			}
			else
			{
				bouncing = false;
			}
		}

		DestroyWorld( worldId );
		return result;
	}

	// Heights on both sides of the continuous collision threshold, so the impulse is checked for a ball
	// that lands inside the overlap and for one the sweep sets down on the surface. The threshold is
	// derived from the body so the split survives a change to the default safety factor.
	//
	// The step balance is the real gate. The bounce total is bracketed as well so the restitution sweep
	// means something: the contact has to reverse the approach at the coefficient and may carry the
	// weight for at most as long as it lasted. The stage runs after the sub-steps, so the reversal
	// lands a step late and the ball can come in up to a step of gravity under it.
	private static int ImpulseTest()
	{
		float[] restitutions = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
		float[] heights = { 1.0f, 5.0f, 20.0f, 45.0f };

		float mass;
		float fastSpeed;
		{
			WorldId worldId = MakeWorld( -10.0f );
			BodyId ballId = MakeBall( worldId, 0.0f, 0.5f, 0.0f, 0.0f );
			mass = Body_GetMass( ballId );
			fastSpeed = Body_GetSafetyFactor( ballId ) * Body_GetMinExtent( ballId ) / TIME_STEP;
			DestroyWorld( worldId );
		}

		float weightImpulse = mass * 10.0f * TIME_STEP;

		int failed = 0;

		for ( int j = 0; j < heights.Length; ++j )
		{
			float impactSpeed = MathF.Sqrt( 20.0f * heights[j] );
			bool expectToi = impactSpeed > fastSpeed;

			// Float noise in the solver velocities scales with the impact speed, and the ball is heavy
			float tolerance = 1e-5f * mass * ( 10.0f + impactSpeed );

			for ( int k = 0; k < restitutions.Length; ++k )
			{
				ImpulseResult result = MeasureDropImpulse( restitutions[k], heights[j] );

				float reversal = ( 1.0f + restitutions[k] ) * mass * result.approachSpeed;
				float lower = reversal - weightImpulse;
				float upper = reversal + weightImpulse * result.contactSteps;

				Console.WriteLine( $"    impulse drop {heights[j]:F1} e {restitutions[k]:F2} toi {result.toiSteps} at {result.firstSeparation:F4} . worst step error %.1e, bounce {result.worstError:F1} in [{result.bounceImpulse:F1}, {lower:F1}] over {upper} steps" );

				if ( result.worstError > tolerance )
				{
					failed = 1;
				}

				if ( result.contactSteps == 0 || result.toiImpulseSteps > 0 )
				{
					failed = 1;
				}

				float slack = result.contactSteps * tolerance;
				if ( result.bounceImpulse < lower - slack || result.bounceImpulse > upper + slack )
				{
					failed = 1;
				}

				if ( ( result.toiSteps > 0 ) != expectToi )
				{
					Console.WriteLine( $"    continuous collision {( expectToi ? "expected" : "unexpected" )} at {impactSpeed:F1} m/s (threshold {fastSpeed:F1} m/s)" );
					failed = 1;
				}
			}
		}

		return failed;
	}

	internal struct EnergyScene
	{
		public string? name;
		public float safetyFactor;
		public float friction;
		public float angle;
		public float spin;
		public float height;
		public float velocityY;
		public float halfWidth;
		public float halfHeight;
		public bool bullet;
		public bool stack;
		public int stepCount;
	}

	// Peak energy over the run relative to the start, for every dynamic body in the scene. Perfectly
	// elastic and the energy may still only go down: friction and the soft contact position correction
	// dissipate, and a bounce may return at most what the contact absorbed.
	private static float MeasurePeakEnergy( in EnergyScene scene )
	{
		WorldId worldId = MakeWorld( -10.0f );

		BodyDef groundDef = DefaultBodyDef();
		BodyId groundId = CreateBody( worldId, ref groundDef );
		ShapeDef groundShape = DefaultShapeDef();
		groundShape.material.friction = scene.friction;
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref groundShape, ref segment );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = scene.friction;
		shapeDef.material.restitution = 1.0f;
		Polygon box = MakeBox( scene.halfWidth, scene.halfHeight );

		FixedArray4<BodyId> bodyIds = default;
		int bodyCount = 0;

		if ( scene.stack )
		{
			// Resting box under the dropped one, like the TwoBoxRestitution sample
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = scene.halfHeight };
			bodyDef.safetyFactor = scene.safetyFactor;
			bodyIds[bodyCount] = CreateBody( worldId, ref bodyDef );
			CreatePolygonShape( bodyIds[bodyCount], ref shapeDef, ref box );
			bodyCount += 1;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = scene.height };
			bodyDef.rotation = MakeRot( scene.angle );
			bodyDef.angularVelocity = scene.spin;
			bodyDef.linearVelocity = new Vec2( 0.0f, scene.velocityY );
			bodyDef.safetyFactor = scene.safetyFactor;
			bodyDef.isBullet = scene.bullet;
			bodyIds[bodyCount] = CreateBody( worldId, ref bodyDef );
			CreatePolygonShape( bodyIds[bodyCount], ref shapeDef, ref box );
			bodyCount += 1;
		}

		float start = 0.0f;
		for ( int k = 0; k < bodyCount; ++k )
		{
			start += MeasureEnergy( bodyIds[k], worldId );
		}

		float peak = start;
		int peakStep = -1;

		for ( int i = 0; i < scene.stepCount; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

			float energy = 0.0f;
			for ( int k = 0; k < bodyCount; ++k )
			{
				energy += MeasureEnergy( bodyIds[k], worldId );
			}

			if ( energy > peak )
			{
				peak = energy;
				peakStep = i;
			}
		}

		DestroyWorld( worldId );

		float ratio = peak / start;
		Console.WriteLine( $"    energy {scene.name} peak {ratio:F5} at step {peakStep}" );
		return ratio;
	}

	// Perfectly elastic material with friction, dropped every way that used to gain energy: a spinning
	// corner landing, tumbling, a flat landing with spin, a box falling on a resting box, and a tall box
	// landing flat on its narrow end. Energy must never rise above the start. This is the load bearing
	// property of restitution. Accuracy of the coefficient and freedom from spin are secondary.
	//
	// The tall boxes guard the stage iteration count. Their contact points sit close together while the
	// inertia is long, so the off diagonal of the two point normal mass is large and one sweep
	// overshoots: the narrowest gains 1.7% frictionless and 1.0% with friction at a single sweep and is
	// flat from two on.
	private static int EnergyTest()
	{
		EnergyScene[] scenes = { new( "spinning corner", 0.01f, 0.6f, -1.43f, 7.9f, 0.7f, -1.9f, 0.5f, 0.5f, false, false, 300 ),
			new( "spinning corner fast", 0.01f, 0.6f, -0.8f, 12.0f, 3.0f, -8.0f, 0.5f, 0.5f, false, false, 300 ),
			new( "tumble", 0.01f, 0.6f, -0.3f, 3.0f, 8.0f, 0.0f, 0.5f, 0.5f, false, false, 600 ),
			new( "tumble discrete", 0.5f, 0.6f, -0.3f, 3.0f, 8.0f, 0.0f, 0.5f, 0.5f, false, false, 600 ),
			new( "flat with spin", 0.01f, 0.6f, 0.0f, 2.0f, 2.0f, -5.0f, 0.5f, 0.5f, false, false, 300 ),
			new( "flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 0.5f, false, false, 1200 ),
			new( "corner", 0.01f, 0.6f, -0.2f, 0.0f, 5.0f, 0.0f, 0.5f, 0.5f, false, false, 600 ),
			new( "two box", 0.02f, 0.6f, 0.0f, 0.0f, 4.0f, 0.0f, 0.5f, 0.5f, true, true, 1500 ),
			new( "two box discrete", 0.5f, 0.6f, 0.0f, 0.0f, 4.0f, 0.0f, 0.5f, 0.5f, false, true, 1500 ),
			new( "tall flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 1.0f, false, false, 1200 ),
			new( "tall flat friction", 0.01f, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 1.0f, false, false, 1200 ),
			new( "narrow flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.25f, 1.0f, false, false, 1200 ),
			new( "narrow flat friction", 0.01f, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f, 0.25f, 1.0f, false, false, 1200 ), };

		// The soft contact position correction lifts a body that lands inside the ground, which is
		// potential energy from nowhere at any restitution. Continuous collision keeps landings on the
		// surface, so the tolerance is round-off. The discrete scenes land deep and are allowed the lift.
		float tolerance = 1.0e-3f;
		float discreteTolerance = 2.0e-2f;

		int failed = 0;

		for ( int i = 0; i < scenes.Length; ++i )
		{
			float ratio = MeasurePeakEnergy( scenes + i );
			float limit = scenes[i].safetyFactor < 0.1f ? tolerance : discreteTolerance;
			if ( ratio > 1.0f + limit )
			{
				failed = 1;
			}
		}

		return failed;
	}

	private static ulong RunWorkerScene( int workerCount )
	{
		WorldDef worldDef = DefaultWorldDef();
		worldDef.gravity = new Vec2( 0.0f, -10.0f );
		worldDef.enableSleep = false;
		worldDef.workerCount = workerCount;
		WorldId worldId = CreateWorld( ref worldDef );

		MakeGround( worldId, 0.6f );

		for ( int i = 0; i < 20; ++i )
		{
			MakeBall( worldId, -10.0f + 1.05f * i, 3.0f + 0.13f * i, 0.0f, 0.6f );
		}

		for ( int i = 0; i < 200; ++i )
		{
			World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		ulong hash = World_GetStateHash( worldId );
		DestroyWorld( worldId );
		return hash;
	}

	// The sampled approach speed and the Poisson accumulator are per manifold point state, so they must
	// survive the split across workers
	private static int WorkerParityTest()
	{
		ulong hash0 = RunWorkerScene( 0 );
		ulong hash1 = RunWorkerScene( 1 );
		ulong hash4 = RunWorkerScene( 4 );

		Console.WriteLine( $"    worker hashes 0x{(unsigned long long)hash0:x} 0x{(unsigned long long)hash1:x} 0x{(unsigned long long)hash4:x}" );

		ENSURE( hash0 == hash1 );
		ENSURE( hash0 == hash4 );
		return 0;
	}

	internal static int RestitutionTest()
	{
		int failureCount = 0;

		RUN_MEASUREMENT( HeadOnTest );
		RUN_MEASUREMENT( PhaseTest );
		RUN_MEASUREMENT( SupportedTest );
		RUN_MEASUREMENT( TwoPointTest );
		RUN_MEASUREMENT( SpinTest );
		RUN_MEASUREMENT( DropTest );
		RUN_MEASUREMENT( SingleBoxTest );
		RUN_MEASUREMENT( ThresholdTest );
		RUN_MEASUREMENT( RestingTest );
		RUN_MEASUREMENT( NormalVelocityTest );
		RUN_MEASUREMENT( OvershootTest );
		RUN_MEASUREMENT( EnergyTest );
		RUN_MEASUREMENT( ImpulseTest );
		RUN_MEASUREMENT( WorkerParityTest );

		return failureCount > 0 ? 1 : 0;
	}

}
