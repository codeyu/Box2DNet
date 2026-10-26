// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using Hexa.NET.ImGui;
using Box2DNet.Shared;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Testbed.DrawFunctions;
using static Box2DNet.Testbed.Glfw;
using static Box2DNet.Testbed.Printf;
using static Box2DNet.Testbed.SampleApp;
using static Box2DNet.Testbed.Helpers_sample_benchmark;
using static Box2DNet.Testbed.Helpers_sample_bodies;
using static Box2DNet.Testbed.Helpers_sample_character;
using static Box2DNet.Testbed.Helpers_sample_collision;
using static Box2DNet.Testbed.Helpers_sample_continuous;
using static Box2DNet.Testbed.Helpers_sample_determinism;
using static Box2DNet.Testbed.Helpers_sample_events;
using static Box2DNet.Testbed.Helpers_sample_issues;
using static Box2DNet.Testbed.Helpers_sample_joints;
using static Box2DNet.Testbed.Helpers_sample_restitution;
using static Box2DNet.Testbed.Helpers_sample_robustness;
using static Box2DNet.Testbed.Helpers_sample_shapes;
using static Box2DNet.Testbed.Helpers_sample_stacking;
using static Box2DNet.Testbed.Helpers_sample_world;
using static Box2DNet.Testbed.Helpers_dynamic_mover;
using static Box2DNet.Testbed.Helpers_geometric_mover;

namespace Box2DNet.Testbed;

public static partial class Helpers_geometric_mover
{
	// Optional shape user data read by GeometryMover when it gathers collision planes.
	// Shapes without user data are treated as rigid and fully clipping.



	// Extra data kept alongside each collision plane so the mover can push rigid bodies.



	// Geometric character mover. The capsule is moved by casting
	// and by solving collision planes, so the game keeps full control of the motion.
	//
	// A pogo spring holds the capsule above the ground so it can move up steps.










	public static float GeometricMoverCastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastResult? result = (CastResult?)context;
		result.point = point;
		result.normal = normal;
		result.bodyId = Shape_GetBody( shapeId );
		result.fraction = fraction;
		result.hit = true;
		return fraction;
	}

	public static bool PlaneResultFcn( ShapeId shapeId, in PlaneResult planeResult, object? context )
	{
		Debug.Assert( planeResult.hit == true );

		GeometricMover? self = (GeometricMover?)( context );
		float maxPush = float.MaxValue;
		bool clipVelocity = true;
		MoverShapeUserData? userData = (MoverShapeUserData?)( Shape_GetUserData( shapeId ) );
		if ( userData != null )
		{
			maxPush = userData.maxPush;
			clipVelocity = userData.clipVelocity;
		}

		if ( self.m_planeCount < GeometricMover.m_planeCapacity )
		{
			Debug.Assert( IsValidPlane( planeResult.plane ) );
			self.m_planes[self.m_planeCount] = new CollisionPlane { plane = planeResult.plane, pushLimit = maxPush, push = 0.0f, clipVelocity = clipVelocity };

			// The plane point is relative to the origin passed to the collide call
			self.m_planeExtras[self.m_planeCount] = new() { point = OffsetPos( self.m_position, planeResult.point ), shapeId = shapeId };
			self.m_planeCount += 1;
		}

		return true;
	}







	// Solve a normal constraint between the mover and each touched rigid body. This is the
	// same velocity constraint the solver uses for a contact, except the mover is treated as
	// having infinite mass, so all of the impulse lands on the body.


	// Movement follows the Quake ground and air model:
	// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
}

public struct GeometricMoverDef
{
	public GeometricMoverDef()
	{
	}

	public Pos position = default;
	public Capsule capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };
	public QueryFilter filter = DefaultQueryFilter();
	public float jumpSpeed = 10.0f;
	public float maxSpeed = 6.0f;
	public float minSpeed = 0.1f;
	public float stopSpeed = 3.0f;
	public float accelerate = 20.0f;
	public float airSteer = 0.2f;
	public float friction = 8.0f;
	public float gravity = 30.0f;
	public float pogoRestLength = 0.9f;
	public float pogoHertz = 5.0f;
	public float pogoDampingRatio = 0.8f;
	public float pogoCompressionScale = 100.0f;
	public float pogoTensionScale = 100.0f;
}

public class MoverShapeUserData
{
	// Distance limit on the push this surface may apply to the mover. float.MaxValue is rigid.
	// Use a small value for surfaces that must not shove the mover, such as an elevator
	// that would otherwise push it through the floor.
	public float maxPush;

	// False keeps the velocity along this surface, which is what soft collision needs.
	public bool clipVelocity;
}

public class PlaneExtra
{
	public Pos point;
	public ShapeId shapeId;
}

public class GeometricMover
{

	// Place the mover. The capsule and tuning may be changed at any time.

	// Advance the mover. Throttle is the horizontal input on [-1, 1].

	// Jump if grounded. Returns false when airborne.

	public const int m_planeCapacity = 8;

	public float m_jumpSpeed;
	public float m_maxSpeed;
	public float m_minSpeed;
	public float m_stopSpeed;
	public float m_accelerate;

	// Fraction of the ground acceleration available in the air.
	public float m_airSteer;

	// Ground friction in units of 1/time.
	public float m_friction;

	// The mover is not simulated so it needs its own gravity.
	public float m_gravity;

	public float m_pogoRestLength;
	public float m_pogoHertz;
	public float m_pogoDampingRatio;

	// Downward force applied to whatever the pogo stick is resting on.
	public float m_pogoGroundForce;

	// Surfaces steeper than this are not ground.
	public float m_minGroundNormalY;

	// Push touched rigid bodies out of the way by solving a normal constraint against
	// each collision plane. Without this the mover slides along a crate instead of
	// shoving it.
	public bool m_pushBodies;

	// The capsule is relative to the mover position.
	public Capsule m_capsule;

	public QueryFilter m_filter;

	public WorldId m_worldId;

	// The center of the capsule.
	public Pos m_position;

	public Vec2 m_velocity;
	public float m_pogoVelocity;
	public bool m_onGround;

	// Planes from the last solve, kept for debug drawing.
	public CollisionPlane[] m_planes = new CollisionPlane[m_planeCapacity];
	public PlaneExtra[] m_planeExtras = new PlaneExtra[m_planeCapacity];
	public int m_planeCount;
	public int m_totalIterations;

	public Pos m_pogoOrigin;
	public Vec2 m_pogoTranslation;
	public float m_pogoFraction;
	public bool m_pogoHit;


	public GeometricMover(  )
	{
		m_jumpSpeed = 10.0f;
		m_maxSpeed = 6.0f;
		m_minSpeed = 0.1f;
		m_stopSpeed = 3.0f;
		m_accelerate = 20.0f;
		m_airSteer = 0.2f;
		m_friction = 8.0f;
		m_gravity = 30.0f;
		m_pogoRestLength = 0.9f;
		m_pogoHertz = 5.0f;
		m_pogoDampingRatio = 0.8f;
		m_pogoGroundForce = 50.0f;
		m_pushBodies = true;

		m_capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };

		m_filter = DefaultQueryFilter();

		m_worldId = nullWorldId;
		m_position = Pos_zero;
		m_velocity = Vec2_zero;
		m_pogoVelocity = 0.0f;
		m_minGroundNormalY = 0.7f;
		m_onGround = false;

		m_planeCount = 0;
		m_totalIterations = 0;

		m_pogoOrigin = Pos_zero;
		m_pogoTranslation = Vec2_zero;
		m_pogoFraction = 1.0f;
		m_pogoHit = false;
	}

	public void Create( WorldId worldId, in GeometricMoverDef def )
	{
		m_capsule = def.capsule;
		m_filter = def.filter;
		m_jumpSpeed = def.jumpSpeed;
		m_maxSpeed = def.maxSpeed;
		m_minSpeed = def.minSpeed;
		m_stopSpeed = def.stopSpeed;
		m_accelerate = def.accelerate;
		m_airSteer = def.airSteer;
		m_friction = def.friction;
		m_gravity = def.gravity;
		m_pogoRestLength = def.pogoRestLength;
		m_pogoHertz = def.pogoHertz;
		m_pogoDampingRatio = def.pogoDampingRatio;
		m_minGroundNormalY = 0.7f;

		m_worldId = worldId;
		m_position = def.position;
		m_velocity = Vec2_zero;
		m_pogoVelocity = 0.0f;
		m_onGround = false;
		m_planeCount = 0;
		m_totalIterations = 0;
		m_pogoFraction = 1.0f;
		m_pogoHit = false;
	}

	public bool Jump(  )
	{
		if ( m_onGround == false )
		{
			return false;
		}

		m_velocity.y = m_jumpSpeed;
		m_onGround = false;
		return true;
	}

	public void PushBodies(  )
	{
		for ( int i = 0; i < m_planeCount; ++i )
		{
			BodyId bodyId = Shape_GetBody( m_planeExtras[i].shapeId );
			if ( Body_GetType( bodyId ) != BodyType.dynamicBody )
			{
				continue;
			}

			// Plane normals point at the mover
			Pos point = m_planeExtras[i].point;
			Vec2 normal = -m_planes[i].plane.normal;

			float invMassA = 0.0f;

			float massB = Body_GetMass( bodyId );
			float invMassB = massB > 0.0f ? 1.0f / massB : 0.0f;

			float inertiaB = Body_GetRotationalInertia( bodyId );
			float invIB = inertiaB > 0.0f ? 1.0f / inertiaB : 0.0f;

			Pos centerB = Body_GetWorldCenter( bodyId );
			Vec2 rB = point - centerB;

			float rnB = Cross( rB, normal );
			float kNormal = invMassA + invMassB + invIB * rnB * rnB;
			float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			Vec2 vB = Body_GetLinearVelocity( bodyId );
			float omegaB = Body_GetAngularVelocity( bodyId );
			Vec2 vrB = vB + CrossSV( omegaB, rB );
			float vn = Dot( vrB - m_velocity, normal );

			// Push only, a separating body is left alone
			float impulse = MaxFloat( -normalMass * vn, 0.0f );

			Vec2 P = impulse * normal;
			m_velocity = MulSub( m_velocity, invMassA, P );

			Body_ApplyLinearImpulse( bodyId, P, point, true );
		}
	}

	public void Update( float timeStep, float throttle )
	{
		// Friction
		float speed = Length( m_velocity );
		if ( speed < m_minSpeed )
		{
			m_velocity.x = 0.0f;
			m_velocity.y = 0.0f;
		}
		else if ( m_onGround )
		{
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			float control = speed < m_stopSpeed ? m_stopSpeed : speed;

			// friction has units of 1/time
			float drop = control * m_friction * timeStep;
			float newSpeed = MaxFloat( 0.0f, speed - drop );
			m_velocity *= newSpeed / speed;
		}

		Vec2 desiredVelocity = new Vec2( m_maxSpeed * throttle, 0.0f );
		float desiredSpeed;
		Vec2 desiredDirection = GetLengthAndNormalize( out desiredSpeed, desiredVelocity );

		if ( desiredSpeed > m_maxSpeed )
		{
			desiredSpeed = m_maxSpeed;
		}

		// Accelerate
		float currentSpeed = Dot( m_velocity, desiredDirection );
		float addSpeed = desiredSpeed - currentSpeed;
		if ( addSpeed > 0.0f )
		{
			float steer = m_onGround ? 1.0f : m_airSteer;
			float accelSpeed = steer * m_accelerate * m_maxSpeed * timeStep;
			if ( accelSpeed > addSpeed )
			{
				accelSpeed = addSpeed;
			}

			m_velocity += accelSpeed * desiredDirection;
		}

		// The probe reaches past the rest length so the spring can be stretched
		float rayLength = m_onGround ? 2.0f * m_pogoRestLength : m_pogoRestLength;
		Vec2 translation = new Vec2( 0.0f, -rayLength );
		Pos origin = m_position + m_capsule.center1;

		CastResult castResult = new();
		World_CastRay( m_worldId, origin, translation, m_filter, GeometricMoverCastCallback, castResult );

		m_pogoOrigin = origin;
		m_pogoTranslation = translation;
		m_pogoFraction = castResult.hit ? castResult.fraction : 1.0f;
		m_pogoHit = castResult.hit;

		// Avoid snapping to ground if still going up
		bool haveGround = castResult.hit && castResult.normal.y > m_minGroundNormalY;

		if ( m_onGround == false )
		{
			m_onGround = haveGround && m_velocity.y <= 0.01f;
		}
		else
		{
			m_onGround = haveGround;
		}

		if ( m_onGround == false )
		{
			m_velocity.y -= m_gravity * timeStep;
			m_pogoVelocity = 0.0f;
		}
		else
		{
			m_velocity.y = 0.0f;

			float pogoCurrentLength = castResult.fraction * rayLength;

			float offset = pogoCurrentLength - m_pogoRestLength;
			m_pogoVelocity = SpringDamper( m_pogoHertz, m_pogoDampingRatio, offset, m_pogoVelocity, timeStep );

			Body_ApplyForce( castResult.bodyId, new( 0.0f, -m_pogoGroundForce ), castResult.point, true );
		}

		Pos target = m_position + timeStep * m_velocity + timeStep * m_pogoVelocity * new Vec2( 0.0f, 1.0f );

		m_totalIterations = 0;
		float tolerance = 0.01f;

		// Don't cast against other movers.
		QueryFilter castFilter = new QueryFilter { categoryBits = m_filter.categoryBits, maskBits = ( m_filter.maskBits & ~m_filter.categoryBits ) };

		for ( int iteration = 0; iteration < 5; ++iteration )
		{
			m_planeCount = 0;

			World_CollideMover( m_worldId, m_position, ref m_capsule, m_filter, PlaneResultFcn, this );
			PlaneSolverResult result = SolvePlanes( target - m_position, m_planes, m_planeCount );

			m_totalIterations += result.iterationCount;

			float fraction = World_CastMover( m_worldId, m_position, ref m_capsule, result.delta, castFilter );

			Vec2 delta = fraction * result.delta;
			m_position = m_position + delta;

			if ( LengthSquared( delta ) < tolerance * tolerance )
			{
				break;
			}
		}

		if ( m_pushBodies )
		{
			PushBodies();
		}

		// Stop pushing into surfaces so speed can't accumulate against a wall
		m_velocity = ClipVector( m_velocity, m_planes, m_planeCount );
	}
}

public class CastResult
{
	public Pos point;
	public Vec2 normal;
	public BodyId bodyId;
	public float fraction;
	public bool hit;
}
