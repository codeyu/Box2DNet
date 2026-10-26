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

	public static void MoverJoint_SetLinearVelocity( JointId jointId, Vec2 velocity )
	{
		if ( CheckInput( IsValidVec2( velocity ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.moverJoint );
		joint.moverJoint.linearVelocity = velocity;
	}

	public static Vec2 MoverJoint_GetLinearVelocity( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.moverJoint );
		return joint.moverJoint.linearVelocity;
	}

	public static void MoverJoint_SetMaxVelocityForce( JointId jointId, Vec2 maxForce )
	{
		if ( CheckInput( IsValidVec2( maxForce ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.moverJoint );
		joint.moverJoint.maxVelocityForce = maxForce;
	}

	public static Vec2 MoverJoint_GetMaxVelocityForce( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.moverJoint );
		return joint.moverJoint.maxVelocityForce;
	}

	internal static Vec2 GetMoverJointForce( World world, ref JointSim @base )
	{
		Vec2 force = MulSV( world.inv_h, @base.moverJoint.linearVelocityImpulse );
		return force;
	}

	internal static void PrepareMoverJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.moverJoint );

		int idA = @base.bodyIdA;
		int idB = @base.bodyIdB;

		World world = context.world;
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
		float mB = bodySimB.invMass;

		@base.invMassA = mA;
		@base.invMassB = mB;
		@base.invIA = 0.0f;
		@base.invIB = 0.0f;

		ref MoverJoint joint = ref @base.moverJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		float k = mA + mB;
		joint.linearMass = k > 0.0f ? 1.0f / k : 0.0f;

		if ( context.enableWarmStarting == false )
		{
			joint.linearVelocityImpulse = Vec2_zero;
		}
	}

	internal static void WarmStartMoverJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.moverJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;

		ref MoverJoint joint = ref @base.moverJoint;
		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, joint.linearVelocityImpulse );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, joint.linearVelocityImpulse );
		}
	}

	internal static void SolveMoverJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.moverJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref MoverJoint joint = ref @base.moverJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		Vec2 vB = stateB.linearVelocity;

		// linear velocity
		if ( joint.maxVelocityForce.x > 0.0f || joint.maxVelocityForce.y > 0.0f )
		{
			Vec2 cdot = Sub( vB, vA );
			cdot = Sub( cdot, joint.linearVelocity );
			Vec2 b = MulSV( joint.linearMass, cdot );
			Vec2 impulse = new Vec2( -b.x, -b.y );

			Vec2 oldImpulse = joint.linearVelocityImpulse;
			joint.linearVelocityImpulse = Add( joint.linearVelocityImpulse, impulse );
			Vec2 maxImpulse = MulSV( context.h, joint.maxVelocityForce );

			// clamp x and y separately
			joint.linearVelocityImpulse = Clamp( joint.linearVelocityImpulse, Neg( maxImpulse ), maxImpulse );

			impulse = Sub( joint.linearVelocityImpulse, oldImpulse );
			vA = MulSub( vA, mA, impulse );
			vB = MulAdd( vB, mB, impulse );
		}
		else
		{
			joint.linearVelocityImpulse = Vec2_zero;
		}

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = vA;
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = vB;
		}
	}

}
