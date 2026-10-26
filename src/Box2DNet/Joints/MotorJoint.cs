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

	public static void MotorJoint_SetLinearVelocity( JointId jointId, Vec2 velocity )
	{
		if ( CheckInput( IsValidVec2( velocity ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.linearVelocity = velocity;
	}

	public static Vec2 MotorJoint_GetLinearVelocity( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.linearVelocity;
	}

	public static void MotorJoint_SetAngularVelocity( JointId jointId, float velocity )
	{
		if ( CheckInput( IsValidFloat( velocity ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.angularVelocity = velocity;
	}

	public static float MotorJoint_GetAngularVelocity( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.angularVelocity;
	}

	public static void MotorJoint_SetMaxVelocityTorque( JointId jointId, float maxTorque )
	{
		if ( CheckInput( IsValidFloat( maxTorque ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.maxVelocityTorque = maxTorque;
	}

	public static float MotorJoint_GetMaxVelocityTorque( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.maxVelocityTorque;
	}

	public static void MotorJoint_SetMaxVelocityForce( JointId jointId, float maxForce )
	{
		if ( CheckInput( IsValidFloat( maxForce ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.maxVelocityForce = maxForce;
	}

	public static float MotorJoint_GetMaxVelocityForce( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.maxVelocityForce;
	}

	public static void MotorJoint_SetLinearHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.linearHertz = hertz;
	}

	public static float MotorJoint_GetLinearHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.linearHertz;
	}

	public static void MotorJoint_SetLinearDampingRatio( JointId jointId, float damping )
	{
		if ( CheckInput( IsValidFloat( damping ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.linearDampingRatio = damping;
	}

	public static float MotorJoint_GetLinearDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.linearDampingRatio;
	}

	public static void MotorJoint_SetAngularHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.angularHertz = hertz;
	}

	public static float MotorJoint_GetAngularHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.angularHertz;
	}

	public static void MotorJoint_SetAngularDampingRatio( JointId jointId, float damping )
	{
		if ( CheckInput( IsValidFloat( damping ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.angularDampingRatio = damping;
	}

	public static float MotorJoint_GetAngularDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.angularDampingRatio;
	}

	public static void MotorJoint_SetMaxSpringForce( JointId jointId, float maxForce )
	{
		if ( CheckInput( IsValidFloat( maxForce ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.maxSpringForce = MaxFloat( 0.0f, maxForce );
	}

	public static float MotorJoint_GetMaxSpringForce( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.maxSpringForce;
	}

	public static void MotorJoint_SetMaxSpringTorque( JointId jointId, float maxTorque )
	{
		if ( CheckInput( IsValidFloat( maxTorque ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		joint.motorJoint.maxSpringTorque = MaxFloat( 0.0f, maxTorque );
	}

	public static float MotorJoint_GetMaxSpringTorque( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.motorJoint );
		return joint.motorJoint.maxSpringTorque;
	}

	internal static Vec2 GetMotorJointForce( World world, ref JointSim @base )
	{
		Vec2 force = MulSV( world.inv_h, Add( @base.motorJoint.linearVelocityImpulse, @base.motorJoint.linearSpringImpulse ) );
		return force;
	}

	internal static float GetMotorJointTorque( World world, ref JointSim @base )
	{
		return world.inv_h * ( @base.motorJoint.angularVelocityImpulse + @base.motorJoint.angularSpringImpulse );
	}

	// Point-to-point constraint
	// C = p2 - p1
	// Cdot = v2 - v1
	//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
	// J = [-I -r1_skew I r2_skew ]
	// Identity used:
	// w k % (rx i + ry j) = w * (-ry i + rx j)

	// Angle constraint
	// C = angle2 - angle1 - referenceAngle
	// Cdot = w2 - w1
	// J = [0 0 -1 0 0 1]
	// K = invI1 + invI2

	internal static void PrepareMotorJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.motorJoint );

		// chase body id to the solver set where the body lives
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
		float iA = bodySimA.invInertia;
		float mB = bodySimB.invMass;
		float iB = bodySimB.invInertia;

		@base.invMassA = mA;
		@base.invMassB = mB;
		@base.invIA = iA;
		@base.invIB = iB;

		ref MotorJoint joint = ref @base.motorJoint;
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

		joint.linearSpring = MakeSoft( joint.linearHertz, joint.linearDampingRatio, context.h );
		joint.angularSpring = MakeSoft( joint.angularHertz, joint.angularDampingRatio, context.h );

		Mat22 kl = default;
		kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		kl.cy.x = kl.cx.y;
		kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		joint.linearMass = GetInverse22( kl );

		float ka = iA + iB;
		joint.angularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

		if ( context.enableWarmStarting == false )
		{
			joint.linearVelocityImpulse = Vec2_zero;
			joint.angularVelocityImpulse = 0.0f;
			joint.linearSpringImpulse = Vec2_zero;
			joint.angularSpringImpulse = 0.0f;
		}
	}

	internal static void WarmStartMotorJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.motorJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		ref MotorJoint joint = ref @base.motorJoint;
		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 linearImpulse = Add( joint.linearVelocityImpulse, joint.linearSpringImpulse );
		float angularImpulse = joint.angularVelocityImpulse + joint.angularSpringImpulse;

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, linearImpulse );
			stateA.angularVelocity -= iA * ( Cross( rA, linearImpulse ) + angularImpulse );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, linearImpulse );
			stateB.angularVelocity += iB * ( Cross( rB, linearImpulse ) + angularImpulse );
		}
	}

	internal static void SolveMotorJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.motorJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref MotorJoint joint = ref @base.motorJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		// angular spring
		if ( joint.maxSpringTorque > 0.0f && joint.angularHertz > 0.0f )
		{
			Rot qA = MulRot( stateA.deltaRotation, joint.frameA.q );
			Rot qB = MulRot( stateB.deltaRotation, joint.frameB.q );
			Rot relQ = InvMulRot( qA, qB );

			float c = Rot_GetAngle( relQ );
			float bias = joint.angularSpring.biasRate * c;
			float massScale = joint.angularSpring.massScale;
			float impulseScale = joint.angularSpring.impulseScale;

			float cdot = wB - wA;

			float maxImpulse = context.h * joint.maxSpringTorque;
			float oldImpulse = joint.angularSpringImpulse;
			float impulse = -massScale * joint.angularMass * ( cdot + bias ) - impulseScale * oldImpulse;
			joint.angularSpringImpulse = ClampFloat( oldImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint.angularSpringImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// angular velocity
		if ( joint.maxVelocityTorque > 0.0f )
		{
			float cdot = wB - wA - joint.angularVelocity;
			float impulse = -joint.angularMass * cdot;

			float maxImpulse = context.h * joint.maxVelocityTorque;
			float oldImpulse = joint.angularVelocityImpulse;
			joint.angularVelocityImpulse = ClampFloat( oldImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint.angularVelocityImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		// linear spring
		if ( joint.maxSpringForce > 0.0f && joint.linearHertz > 0.0f )
		{
			Vec2 dcA = stateA.deltaPosition;
			Vec2 dcB = stateB.deltaPosition;
			Vec2 c = Add( Add( Sub( dcB, dcA ), Sub( rB, rA ) ), joint.deltaCenter );

			Vec2 bias = MulSV( joint.linearSpring.biasRate, c );
			float massScale = joint.linearSpring.massScale;
			float impulseScale = joint.linearSpring.impulseScale;

			Vec2 cdot = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
			cdot = Add( cdot, bias );

			// Updating the effective mass here may be overkill
			Mat22 kl = default;
			kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
			kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
			kl.cy.x = kl.cx.y;
			kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
			joint.linearMass = GetInverse22( kl );

			Vec2 b = MulMV( joint.linearMass, cdot );

			Vec2 oldImpulse = joint.linearSpringImpulse;
			Vec2 impulse = new Vec2( -massScale * b.x - impulseScale * oldImpulse.x, -massScale * b.y - impulseScale * oldImpulse.y );

			float maxImpulse = context.h * joint.maxSpringForce;
			joint.linearSpringImpulse = Add( joint.linearSpringImpulse, impulse );

			if ( LengthSquared( joint.linearSpringImpulse ) > maxImpulse * maxImpulse )
			{
				joint.linearSpringImpulse = Normalize( joint.linearSpringImpulse );
				joint.linearSpringImpulse.x *= maxImpulse;
				joint.linearSpringImpulse.y *= maxImpulse;
			}

			impulse = Sub( joint.linearSpringImpulse, oldImpulse );

			vA = MulSub( vA, mA, impulse );
			wA -= iA * Cross( rA, impulse );
			vB = MulAdd( vB, mB, impulse );
			wB += iB * Cross( rB, impulse );
		}

		// linear velocity
		if ( joint.maxVelocityForce > 0.0f )
		{
			Vec2 cdot = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
			cdot = Sub( cdot, joint.linearVelocity );
			Vec2 b = MulMV( joint.linearMass, cdot );
			Vec2 impulse = new Vec2( -b.x, -b.y );

			Vec2 oldImpulse = joint.linearVelocityImpulse;
			float maxImpulse = context.h * joint.maxVelocityForce;
			joint.linearVelocityImpulse = Add( joint.linearVelocityImpulse, impulse );

			if ( LengthSquared( joint.linearVelocityImpulse ) > maxImpulse * maxImpulse )
			{
				joint.linearVelocityImpulse = Normalize( joint.linearVelocityImpulse );
				joint.linearVelocityImpulse.x *= maxImpulse;
				joint.linearVelocityImpulse.y *= maxImpulse;
			}

			impulse = Sub( joint.linearVelocityImpulse, oldImpulse );

			vA = MulSub( vA, mA, impulse );
			wA -= iA * Cross( rA, impulse );
			vB = MulAdd( vB, mB, impulse );
			wB += iB * Cross( rB, impulse );
		}

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
