// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// needed for dll export

	static readonly Vec2 pogoAxis = new Vec2( 0.0f, 1.0f );

	public static void PogoJoint_SetRestLength( JointId jointId, float length )
	{
		if ( CheckInput( IsValidFloat( length ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		joint.pogoJoint.restLength = length;
	}

	public static float PogoJoint_GetRestLength( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		return joint.pogoJoint.restLength;
	}

	public static float PogoJoint_GetSpringHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		return joint.pogoJoint.hertz;
	}

	public static void PogoJoint_SetSpringHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		 World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		joint.pogoJoint.hertz = hertz;
	}

	public static void PogoJoint_SetSpringDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		 World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		joint.pogoJoint.dampingRatio = dampingRatio;
	}

	public static float PogoJoint_GetSpringDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		return joint.pogoJoint.dampingRatio;
	}

	public static float PogoJoint_GetCurrentLength( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim jointSim = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		// Relative to body A so the difference stays in float precision far from the origin
		WorldTransform wxfA = GetBodyTransform( world, jointSim.bodyIdA );
		Transform transformA = ToRelativeTransform( wxfA, wxfA.p );
		Transform transformB = ToRelativeTransform( GetBodyTransform( world, jointSim.bodyIdB ), wxfA.p );

		Vec2 axis = RotateVector( jointSim.localFrameB.q, pogoAxis );
		axis = RotateVector( transformB.q, axis );
		Vec2 pA = TransformPoint( transformA, jointSim.localFrameA.p );
		Vec2 pB = TransformPoint( transformB, jointSim.localFrameB.p );
		Vec2 d = Sub( pB, pA );
		float length = Dot( d, axis );
		return length;
	}

	public static float PogoJoint_GetImpulse( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		return joint.pogoJoint.impulse;
	}

	public static float PogoJoint_GetCurrentSpeed( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.pogoJoint );
		return joint.pogoJoint.velocity;
	}

	internal static Vec2 GetPogoJointForce( World world, ref JointSim @base )
	{
		Vec2 force = MulSV( world.inv_h * @base.pogoJoint.impulse, @base.pogoJoint.normal );
		return force;
	}

	internal static void PreparePogoJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.pogoJoint );

		World world = context.world;
		// chase body id to the solver set where the body lives
		int idA = @base.bodyIdA;
		int idB = @base.bodyIdB;

		ref Body bodyA = ref world.bodies[idA];
		ref Body bodyB = ref world.bodies[idB];
		B2_ASSERT( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet );

		ref SolverSet setA = ref world.solverSets[bodyA.setIndex];
		ref SolverSet setB = ref world.solverSets[bodyB.setIndex];
		int localIndexA = bodyA.localIndex;
		int localIndexB = bodyB.localIndex;

		ref BodySim bodySimA = ref setA.bodySims[localIndexA];
		ref BodySim bodySimB = ref setB.bodySims[localIndexB];
		float mA = bodySimA.invMass;
		float iA = bodySimA.invInertia;
		float mB = bodySimB.invMass;
		float iB = bodySimB.invInertia;

		@base.invMassA = mA;
		@base.invMassB = mB;
		@base.invIA = iA;
		@base.invIB = iB;

		ref PogoJoint joint = ref @base.pogoJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		// Compute joint anchor frames with world space rotation, relative to center of mass
		joint.frameA.q = MulRot( bodySimA.transform.q, @base.localFrameA.q );
		joint.frameA.p = RotateVector( bodySimA.transform.q, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		joint.frameB.q = MulRot( bodySimB.transform.q, @base.localFrameB.q );
		joint.frameB.p = RotateVector( bodySimB.transform.q, Sub( @base.localFrameB.p, bodySimB.localCenter ) );

		// Compute the initial center delta. Incremental position updates are relative to this.
		joint.deltaCenter = SubPos( bodySimB.center, bodySimA.center );

		Vec2 rA = joint.frameA.p;
		Vec2 rB = joint.frameB.p;

		// compute effective mass
		float crA = Cross( rA, joint.normal );
		float crB = Cross( rB, joint.normal );
		float k = mA + mB + iA * crA * crA + iB * crB * crB;
		joint.linearMass = k > 0.0f ? 1.0f / k : 0.0f;

		if ( context.enableWarmStarting == false )
		{
			joint.impulse = 0.0f;
		}
	}

	internal static void WarmStartPogoJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.pogoJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		ref PogoJoint joint = ref @base.pogoJoint;
		if ( joint.hertz == 0.0f )
		{
			joint.impulse = 0.0f;
			return;
		}

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 linearImpulse = MulSV( joint.impulse, joint.normal );

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, linearImpulse );
			stateA.angularVelocity -= iA * Cross( rA, linearImpulse );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, linearImpulse );
			stateB.angularVelocity += iB * Cross( rB, linearImpulse );
		}
	}

	internal static void SolvePogoJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.pogoJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref PogoJoint joint = ref @base.pogoJoint;
		if ( joint.hertz == 0.0f )
		{
			joint.impulse = 0.0f;
			return;
		}

		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		float bias = 0.0f;
		if ( useBias )
		{
			// This is a custom pogo position correction designed to avoid feeding velocity into
			// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
			// works with continuous collision.

			Vec2 dcA = stateA.deltaPosition;
			Vec2 dcB = stateB.deltaPosition;
			Vec2 d = Add( Add( Sub( dcB, dcA ), Sub( rB, rA ) ), joint.deltaCenter );

			// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
			Vec2 worldPogoAxis = RotateVector( joint.frameB.q, pogoAxis );
			float c = Dot( worldPogoAxis, d ) - joint.restLength;

			// This could be divided by dot(joint.normal, pogoAxis) to account for the constraint
			// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
			joint.velocity = SpringDamper( joint.hertz, joint.dampingRatio, c, joint.velocity, context.h );
			bias = -joint.velocity;
		}

		Vec2 vr = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
		float cdot = Dot( joint.normal, vr );

		float maxTensionImpulse = context.h * joint.maxTensionForce;
		float maxCompressionImpulse = context.h * joint.maxCompressionForce;
		float oldImpulse = joint.impulse;
		float impulse = -joint.linearMass * (cdot + bias);
		joint.impulse = ClampFloat( joint.impulse + impulse, -maxTensionImpulse, maxCompressionImpulse );
		impulse = joint.impulse - oldImpulse;

		Vec2 P = MulSV( impulse, joint.normal );
		vA = MulSub( vA, mA, P );
		wA -= iA * Cross( rA, P );
		vB = MulAdd( vB, mB, P );
		wB += iB * Cross( rB, P );

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = vA;
			stateA.angularVelocity = wA;
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = vB;
			stateB.angularVelocity = wB;
		}
	}

}
