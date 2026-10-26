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

public static partial class Helpers_dynamic_mover
{
	// Dynamic character mover. The capsule is a rigid body with a locked rotation, so it
	// collides and is pushed like everything else in the world. A mover joint drives it to
	// the target velocity and a pogo joint holds it above the ground so it can climb
	// steps without touching the capsule.
	//
	// The dynamic mover should be vendored into your project so you can alter it to meet
	// your game's specific needs.
	//
	// This system is inspired by Tom Waterson's presentation:
	// https://www.youtube.com/watch?v=_jRLlTDqoGI






	public static float DynamicMoverCastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		DynamicMoverCastResult result = (DynamicMoverCastResult)context!;
		result.point = point;
		result.normal = normal;
		result.bodyId = Shape_GetBody( shapeId );
		result.userMaterialId = Shape_GetUserMaterial( shapeId );
		result.fraction = fraction;
		result.hit = true;
		return fraction;
	}













	// Movement follows the Quake ground and air model:
	// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390

	// onGround - controls friction, mover max force, and pogo step-down extension
	// walkable - controls air steer
	// jumping - blocks the pogo from pulling down
}

public struct DynamicMoverDef
{
	public DynamicMoverDef()
	{
	}

	public Pos position = default;
	public Capsule capsule = default;
	public float density = 1.0f;
	public Filter filter = DefaultFilter();
	public float jumpSpeed = 7.0f;
	public float maxSpeed = 6.0f;
	public float minSpeed = 0.1f;
	public float stopSpeed = 3.0f;
	public float accelerate = 20.0f;
	public float airSteer = 0.5f;
	public float friction = 8.0f;
	public float gravityScale = 1.5f;
	public float maxGroundForce = 70.0f;
	public float maxAirForce = 20.0f;
	public float pogoRestLength = 0.9f;
	public float pogoHertz = 5.0f;
	public float pogoDampingRatio = 0.8f;
	public float pogoCompressionScale = 100.0f;
	public float pogoTensionScale = 100.0f;
	public ulong platformMaterialId = 0;
	public bool enableSidePogo = false;
	public bool enablePreSolveEvents = false;
}

public class DynamicMoverCastResult
{
	public Pos point;
	public Vec2 normal;
	public BodyId bodyId;
	public ulong userMaterialId;
	public float fraction;
	public bool hit;
}

public struct DynamicMoverProbe
{
	public Pos origin;
	public Vec2 translation;
	public float fraction;
	public bool hit;

	public DynamicMoverProbe( Pos origin, Vec2 translation, float fraction, bool hit )
	{
		this.origin = origin;
		this.translation = translation;
		this.fraction = fraction;
		this.hit = hit;
	}
}

public class DynamicMover
{


	// Advance the mover. Throttle is the horizontal input on [-1, 1]

	// Jump if grounded. Returns false when airborne.

	// Gravity scale is used to size the pogo forces, so change it here.


	public float m_jumpSpeed;
	public float m_maxSpeed;
	public float m_minSpeed;
	public float m_stopSpeed;
	public float m_accelerate;

	// Fraction of the ground acceleration available in the air.
	public float m_airSteer;

	// Ground friction in units of 1/time.
	public float m_friction;

	// A character usually wants to fall faster than the rest of the world.
	public float m_gravityScale;

	// Force budget the mover joint has for reaching the target velocity. The air value
	// is small so the character keeps its momentum while jumping.
	public float m_maxGroundForce;
	public float m_maxAirForce;

	public float m_pogoRestLength;
	public float m_pogoHertz;
	public float m_pogoDampingRatio;

	// Pogo force limits as a multiple of the mover weight.
	public float m_pogoCompressionScale;
	public float m_pogoTensionScale;

	// Enables extra ray casts when the central one misses.
	public bool m_enableSidePogo;

	// Used for the pogo to detect moving platforms.
	public ulong m_platformMaterialId;

	// Surfaces steeper than this are not ground.
	public float m_minGroundNormalY;

	public Capsule m_capsule;
	public Filter m_filter;

	public WorldId m_worldId;
	public BodyId m_groundId;
	public BodyId m_moverId;
	public JointId m_moverJointId;
	public JointId m_pogoJointId;

	public DynamicMoverCastResult m_castResult = new();

	public Vec2 m_velocity;
	public bool m_onGround;
	public bool m_walkable;
	public bool m_jumping;
	public int m_jumpTicks;

	// Pogo joint state, carried across the joint being recreated each solve.
	public float m_pogoImpulse;
	public float m_pogoVelocity;
	public float m_pogoLength;

	// Pogo ray cast results for drawing.
	public DynamicMoverProbe[] m_probes = new DynamicMoverProbe[3];
	public int m_probeCount;
	public int m_selectedProbe;

	public DynamicMover(  )
	{
		m_jumpSpeed = 5.0f;
		m_maxSpeed = 6.0f;
		m_minSpeed = 0.1f;
		m_stopSpeed = 3.0f;
		m_accelerate = 20.0f;
		m_airSteer = 0.2f;
		m_friction = 8.0f;
		m_gravityScale = 1.5f;
		m_maxGroundForce = 70.0f;
		m_maxAirForce = 20.0f;
		m_pogoRestLength = 0.9f;
		m_pogoHertz = 5.0f;
		m_pogoDampingRatio = 0.8f;
		m_pogoCompressionScale = 100.0f;
		m_pogoTensionScale = 100.0f;
		m_enableSidePogo = false;
		m_platformMaterialId = 0;
		m_minGroundNormalY = 0.7f;

		m_capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };

		m_filter = DefaultFilter();
		m_worldId = nullWorldId;
		m_groundId = nullBodyId;
		m_moverId = nullBodyId;
		m_moverJointId = nullJointId;
		m_pogoJointId = nullJointId;

		m_velocity = Vec2_zero;
		m_onGround = false;
		m_walkable = false;
		m_jumping = false;
		m_jumpTicks = 0;

		m_pogoImpulse = 0.0f;
		m_pogoVelocity = 0.0f;
		m_pogoLength = 0.0f;

		for ( int i = 0; i < m_probes.Length; ++i )
		{
			m_probes[i] = new( Pos_zero, Vec2_zero, 1.0f, false );
		}

		m_probeCount = 0;
		m_selectedProbe = 0;

		m_castResult = new DynamicMoverCastResult {  };
	}

	public void Create( WorldId worldId, in DynamicMoverDef def )
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
		m_gravityScale = def.gravityScale;
		m_maxGroundForce = def.maxGroundForce;
		m_maxAirForce = def.maxAirForce;
		m_pogoRestLength = def.pogoRestLength;
		m_pogoHertz = def.pogoHertz;
		m_pogoDampingRatio = def.pogoDampingRatio;
		m_pogoCompressionScale = def.pogoCompressionScale;
		m_pogoTensionScale = def.pogoTensionScale;
		m_enableSidePogo = def.enableSidePogo;
		m_platformMaterialId = def.platformMaterialId;
		m_minGroundNormalY = 0.7f;

		m_worldId = worldId;
		m_velocity = Vec2_zero;
		m_onGround = false;
		m_walkable = false;
		m_jumping = false;
		m_pogoImpulse = 0.0f;
		m_pogoVelocity = 0.0f;
		m_pogoLength = 0.0f;
		m_pogoJointId = nullJointId;

		BodyDef bodyDef = DefaultBodyDef();
		m_groundId = CreateBody( worldId, ref bodyDef );

		// Position is the center of the capsule.
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = def.position;
		bodyDef.gravityScale = m_gravityScale;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.enableSleep = false;
		bodyDef.name = "mover";

		m_moverId = CreateBody( worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = def.density;

		// The mover joint and the pogo joint handle all the movement, surface friction would fight them
		shapeDef.material.friction = 0.0f;
		shapeDef.filter = def.filter;
		shapeDef.enablePreSolveEvents = def.enablePreSolveEvents;

		CreateCapsuleShape( m_moverId, ref shapeDef, ref m_capsule );

		CreateMoverJoint( m_groundId, new( m_maxGroundForce, 0.0f ) );
	}

	public void CreateMoverJoint( BodyId baseId, Vec2 maxForce )
	{
		MoverJointDef moverDef = DefaultMoverJointDef();
		moverDef.linearVelocity = m_velocity;
		moverDef.maxVelocityForce = maxForce;
		moverDef.@base.bodyIdA = baseId;
		moverDef.@base.bodyIdB = m_moverId;
		moverDef.@base.collideConnected = true;

		m_moverJointId = B2.CreateMoverJoint( m_worldId, ref moverDef );
	}

	public void Destroy(  )
	{
		if ( m_moverId.IsNonNull )
		{
			DestroyBody( m_moverId );
		}

		if ( m_groundId.IsNonNull )
		{
			DestroyBody( m_groundId );
		}

		m_groundId = nullBodyId;
		m_moverId = nullBodyId;

		// Joints are implicitly destroyed.
		m_moverJointId = nullJointId;
		m_pogoJointId = nullJointId;
	}

	public void SetGravityScale( float gravityScale )
	{
		m_gravityScale = gravityScale;

		if ( m_moverId.IsNonNull )
		{
			Body_SetGravityScale( m_moverId, gravityScale );
		}
	}

	public bool Jump(  )
	{
		if ( m_onGround == false || m_walkable == false )
		{
			return false;
		}

		float surfaceVelocity = 0.0f;
		if ( Body_IsValid( m_castResult.bodyId ) )
		{
			Vec2 v = Body_GetWorldPointVelocity( m_castResult.bodyId, m_castResult.point );
			surfaceVelocity = v.y;
		}

		float mass = Body_GetMass( m_moverId );
		float vy = Body_GetLinearVelocity( m_moverId ).y;

		// Remove the pogo constraint velocity but add the surface velocity.
		float dv = MaxFloat( 0.0f, m_jumpSpeed - vy ) + surfaceVelocity;
		Body_ApplyLinearImpulseToCenter( m_moverId, new( 0.0f, mass * dv ), true );

		// This removes the pogo step down on the next update.
		m_onGround = false;
		m_walkable = false;
		m_jumping = true;
		m_jumpTicks = 0;
		return true;
	}

	public void Update( float timeStep, float throttle, float speedScale )
	{
		// Reach further while grounded so the spring can find the ground over a step
		float stepDownLength = m_pogoRestLength;
		float rayLength = m_onGround ? m_pogoRestLength + stepDownLength : m_pogoRestLength;
		Vec2 translation = new Vec2( 0.0f, -rayLength );

		Pos position = Body_GetPosition( m_moverId );
		Pos origin = position + m_capsule.center1;

		QueryFilter queryFilter = new QueryFilter { categoryBits = m_filter.categoryBits, maskBits = m_filter.maskBits };

		m_castResult = new DynamicMoverCastResult {  };
		World_CastRay( m_worldId, origin, translation, queryFilter, DynamicMoverCastCallback, m_castResult );

		m_probes[0] = new( origin, translation, m_castResult.hit ? m_castResult.fraction : 1.0f, m_castResult.hit );
		m_probeCount = 1;
		m_selectedProbe = 0;

		Vec2 pogoOffset = Vec2_zero;

		if ( m_castResult.hit == false && m_enableSidePogo )
		{
			float direction = Body_GetLinearVelocity( m_moverId ).x < 0.0f ? -1.0f : 1.0f;
			Vec2[] offsets = {
				new( direction * m_capsule.radius, 0.0f ), new( -direction * m_capsule.radius, 0.0f ),
			};

			float bestDisplacement = float.MaxValue;
			foreach ( Vec2 offset in offsets )
			{
				DynamicMoverCastResult result = new();
				Pos sideOrigin = origin + offset;
				World_CastRay( m_worldId, sideOrigin, translation, queryFilter, DynamicMoverCastCallback, result );

				m_probes[m_probeCount] = new( sideOrigin, translation, result.hit ? result.fraction : 1.0f, result.hit );

				// Use the result that causes the least movement.
				if ( result.hit )
				{
					float displacement = AbsFloat( result.fraction * rayLength - m_pogoRestLength );
					if ( displacement < bestDisplacement )
					{
						bestDisplacement = displacement;
						m_castResult = result;
						m_selectedProbe = m_probeCount;
						pogoOffset = offset;
					}
				}

				m_probeCount += 1;
			}
		}

		// Should jumping end?
		if ( m_jumping )
		{
			m_jumpTicks += 1;

			// Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
			// when the character approaches the current ground.
			if ( m_jumpTicks > 2 && m_castResult.hit && Dot( m_velocity, m_castResult.normal ) <= 0.0f )
			{
				m_jumping = false;
			}
		}

		if ( m_castResult.hit )
		{
			// The cast can still hit while jumping.
			m_onGround = m_jumping == false;

			// Is the ground too steep to walk?
			m_walkable = m_castResult.normal.y >= m_minGroundNormalY;
		}
		else
		{
			m_onGround = false;
			m_walkable = false;
		}

		// Connect the mover joint to platform. Acts like friction.
		BodyId baseId = m_groundId;
		if ( m_castResult.hit && m_platformMaterialId != 0 && m_castResult.userMaterialId == m_platformMaterialId &&
			 Body_GetType( m_castResult.bodyId ) == BodyType.kinematicBody )
		{
			baseId = m_castResult.bodyId;
		}

		if ( ( Joint_GetBodyA( m_moverJointId ) ).Equals( baseId ) == false )
		{
			Vec2 maxForce = MoverJoint_GetMaxVelocityForce( m_moverJointId );
			DestroyJoint( m_moverJointId );
			CreateMoverJoint( baseId, maxForce );
		}

		m_velocity = Body_GetLinearVelocity( m_moverId ) - Body_GetLinearVelocity( baseId );

		// Friction
		if ( m_onGround )
		{
			float speed = Length( m_velocity );
			if ( speed < m_minSpeed )
			{
				m_velocity.x = 0.0f;
			}
			else
			{
				// Linear damping above stopSpeed and fixed reduction below stopSpeed
				float control = speed < m_stopSpeed ? m_stopSpeed : speed;

				// friction has units of 1/time
				float drop = control * m_friction * timeStep;
				float newSpeed = MaxFloat( 0.0f, speed - drop );
				m_velocity.x *= newSpeed / speed;
			}
		}

		// Mover force
		if ( m_onGround )
		{
			float maxForce = m_walkable ? m_maxGroundForce : 0.0f;
			MoverJoint_SetMaxVelocityForce( m_moverJointId, new( maxForce, 0.0f ) );
		}
		else
		{
			MoverJoint_SetMaxVelocityForce( m_moverJointId, new( m_maxAirForce, 0.0f ) );
		}

		float maxSpeed = speedScale * m_maxSpeed;
		Vec2 desiredVelocity = new Vec2( maxSpeed * throttle, 0.0f );
		float desiredSpeed;
		Vec2 desiredDirection = GetLengthAndNormalize( out desiredSpeed, desiredVelocity );

		if ( desiredSpeed > maxSpeed )
		{
			desiredSpeed = maxSpeed;
		}

		// Accelerate
		float currentSpeed = Dot( m_velocity, desiredDirection );
		float addSpeed = desiredSpeed - currentSpeed;
		if ( addSpeed > 0.0f )
		{
			float steer = m_walkable ? 1.0f : m_airSteer;
			float accelSpeed = steer * m_accelerate * maxSpeed * timeStep;
			if ( accelSpeed > addSpeed )
			{
				accelSpeed = addSpeed;
			}

			m_velocity.x += accelSpeed * desiredDirection.x;
		}

		// The pogo joint is rebuilt every solve because it may land on a different body
		if ( Joint_IsValid( m_pogoJointId ) )
		{
			m_pogoLength = PogoJoint_GetCurrentLength( m_pogoJointId );
			m_pogoImpulse = PogoJoint_GetImpulse( m_pogoJointId );
			m_pogoVelocity = PogoJoint_GetCurrentSpeed( m_pogoJointId );

			DestroyJoint( m_pogoJointId );
			m_pogoJointId = nullJointId;
		}

		if ( m_castResult.hit == true )
		{
			float moverMass = Body_GetMass( m_moverId );
			float moverWeight = m_gravityScale * Length( World_GetGravity( m_worldId ) ) * moverMass;

			PogoJointDef pogoDef = DefaultPogoJointDef();
			pogoDef.@base.localFrameA.p = Body_GetLocalPoint( m_castResult.bodyId, m_castResult.point );
			pogoDef.@base.localFrameB.p = m_capsule.center1 + pogoOffset;
			pogoDef.normal = m_castResult.normal;
			pogoDef.@base.bodyIdA = m_castResult.bodyId;
			pogoDef.@base.bodyIdB = m_moverId;
			pogoDef.@base.collideConnected = true;
			pogoDef.restLength = m_pogoRestLength;
			pogoDef.hertz = m_pogoHertz;
			pogoDef.dampingRatio = m_pogoDampingRatio;
			pogoDef.maxCompressionForce = m_pogoCompressionScale * moverWeight;

			// Warm start from the joint that was just destroyed
			pogoDef.impulse = m_pogoImpulse;
			pogoDef.velocity = m_pogoVelocity;

			if ( m_jumping )
			{
				// Don't allow the pogo to pull down
				pogoDef.maxTensionForce = 0.0f;
			}
			else
			{
				// The pogo can pull down at a multiple of the gravity force.
				pogoDef.maxTensionForce = m_pogoTensionScale * moverWeight;
			}

			m_pogoJointId = CreatePogoJoint( m_worldId, ref pogoDef );
		}
		else
		{
			m_pogoImpulse = 0.0f;
			m_pogoVelocity = 0.0f;
		}

		MoverJoint_SetLinearVelocity( m_moverJointId, m_velocity );
	}
}
