// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
#if false && true
	// #define _CRT_SECURE_NO_WARNINGS
#endif


	// needed for dll export


	// Point-to-point constraint
	// C = pB - pA
	// Cdot = vB - vA
	//      = vB + cross(wB, rB) - vA - cross(wA, rA)
	// J = [-E -skew(rA) E skew(rB) ]

	// Identity used:
	// w k % (rx i + ry j) = w * (-ry i + rx j)

	// Motor constraint
	// Cdot = wB - wA
	// J = [0 0 -1 0 0 1]
	// K = invIA + invIB

	public static void RevoluteJoint_EnableSpring( JointId jointId, bool enableSpring )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		if ( enableSpring != joint.revoluteJoint.enableSpring )
		{
			joint.revoluteJoint.enableSpring = enableSpring;
			joint.revoluteJoint.springImpulse = 0.0f;
		}
	}

	public static bool RevoluteJoint_IsSpringEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.enableSpring;
	}

	public static void RevoluteJoint_SetSpringHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		joint.revoluteJoint.hertz = hertz;
	}

	public static float RevoluteJoint_GetSpringHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.hertz;
	}

	public static void RevoluteJoint_SetSpringDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		joint.revoluteJoint.dampingRatio = dampingRatio;
	}

	public static float RevoluteJoint_GetSpringDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.dampingRatio;
	}

	public static void RevoluteJoint_SetTargetAngle( JointId jointId, float angle )
	{
		if ( CheckInput( IsValidFloat( angle ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		joint.revoluteJoint.targetAngle = angle;
	}

	public static float RevoluteJoint_GetTargetAngle( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.targetAngle;
	}

	public static void RevoluteJoint_EnableLimit( JointId jointId, bool enableLimit )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		if ( enableLimit != joint.revoluteJoint.enableLimit )
		{
			joint.revoluteJoint.enableLimit = enableLimit;
			joint.revoluteJoint.lowerImpulse = 0.0f;
			joint.revoluteJoint.upperImpulse = 0.0f;
		}
	}

	public static bool RevoluteJoint_IsLimitEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.enableLimit;
	}

	public static float RevoluteJoint_GetLowerLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.lowerAngle;
	}

	public static float RevoluteJoint_GetUpperLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.upperAngle;
	}

	public static void RevoluteJoint_SetLimits( JointId jointId, float lower, float upper )
	{
		if ( CheckInput( IsValidFloat( lower ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( upper ) ) == false )
			{
				return;
			};

		B2_ASSERT( lower <= upper );

		World world = GetWorld( jointId.world0 );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		float lowerAngle = MinFloat( lower, upper );
		float upperAngle = MaxFloat( lower, upper );
		joint.revoluteJoint.lowerAngle = ClampFloat( lowerAngle, -0.99f * Pi, 0.99f * Pi );
		joint.revoluteJoint.upperAngle = ClampFloat( upperAngle, -0.99f * Pi, 0.99f * Pi );
	}

	public static void RevoluteJoint_EnableMotor( JointId jointId, bool enableMotor )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		if ( enableMotor != joint.revoluteJoint.enableMotor )
		{
			joint.revoluteJoint.enableMotor = enableMotor;
			joint.revoluteJoint.motorImpulse = 0.0f;
		}
	}

	public static bool RevoluteJoint_IsMotorEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.enableMotor;
	}

	public static void RevoluteJoint_SetTargetMotorSpeed( JointId jointId, float motorSpeed )
	{
		if ( CheckInput( IsValidFloat( motorSpeed ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		joint.revoluteJoint.motorSpeed = motorSpeed;
	}

	public static float RevoluteJoint_GetTargetMotorSpeed( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.motorSpeed;
	}

	public static void RevoluteJoint_SetMaxMotorTorque( JointId jointId, float torque )
	{
		if ( CheckInput( IsValidFloat( torque ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		joint.revoluteJoint.maxMotorTorque = torque;
	}

	public static float RevoluteJoint_GetMaxMotorTorque( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return joint.revoluteJoint.maxMotorTorque;
	}

	public static float RevoluteJoint_GetCurrentAngle( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim jointSim = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		Rot qA = MulRot( GetBodyTransform( world, jointSim.bodyIdA ).q, jointSim.localFrameA.q );
		Rot qB = MulRot( GetBodyTransform( world, jointSim.bodyIdB ).q, jointSim.localFrameB.q );

		float angle = RelativeAngle( qA, qB );
		return angle;
	}

	public static float RevoluteJoint_GetCurrentSpeed( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );

		ref Body bodyA = ref world.bodies[@base.bodyIdA];
		ref Body bodyB = ref world.bodies[@base.bodyIdB];
		ref BodyState bodyStateA = ref GetBodyState( world, ref bodyA );
		ref BodyState bodyStateB = ref GetBodyState( world, ref bodyB );
		float wA = Unsafe.IsNullRef( ref bodyStateA ) == false ? bodyStateA.angularVelocity : 0.0f;
		float wB = Unsafe.IsNullRef( ref bodyStateB ) == false ? bodyStateB.angularVelocity : 0.0f;

		return wB - wA;
	}

	public static float RevoluteJoint_GetCurrentMotorTorque( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.revoluteJoint );
		return world.inv_h * joint.revoluteJoint.motorImpulse;
	}

	internal static Vec2 GetRevoluteJointForce( World world, ref JointSim @base )
	{
		Vec2 force = MulSV( world.inv_h, @base.revoluteJoint.linearImpulse );
		return force;
	}

	internal static float GetRevoluteJointTorque( World world, ref JointSim @base )
	{
		ref readonly RevoluteJoint revolute = ref @base.revoluteJoint;
		float torque = world.inv_h * ( revolute.motorImpulse + revolute.lowerImpulse - revolute.upperImpulse );
		return torque;
	}

	internal static void PrepareRevoluteJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.revoluteJoint );

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

		ref RevoluteJoint joint = ref @base.revoluteJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		// Compute joint anchor frames with world space rotation, relative to center of mass.
		// Avoid round-off here as much as possible.
		// Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
		// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
		// pf = rot(xf.q, f.p - lc)
		joint.frameA.q = MulRot( bodySimA.transform.q, @base.localFrameA.q );
		joint.frameA.p = RotateVector( bodySimA.transform.q, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		joint.frameB.q = MulRot( bodySimB.transform.q, @base.localFrameB.q );
		joint.frameB.p = RotateVector( bodySimB.transform.q, Sub( @base.localFrameB.p, bodySimB.localCenter ) );

		// Compute the initial center delta. Incremental position updates are relative to this.
		joint.deltaCenter = SubPos( bodySimB.center, bodySimA.center );

		float k = iA + iB;
		joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

		joint.springSoftness = MakeSoft( joint.hertz, joint.dampingRatio, context.h );

		if ( context.enableWarmStarting == false )
		{
			joint.linearImpulse = Vec2_zero;
			joint.springImpulse = 0.0f;
			joint.motorImpulse = 0.0f;
			joint.lowerImpulse = 0.0f;
			joint.upperImpulse = 0.0f;
		}
	}

	internal static void WarmStartRevoluteJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.revoluteJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref RevoluteJoint joint = ref @base.revoluteJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
			stateA.angularVelocity -= iA * ( Cross( rA, joint.linearImpulse ) + axialImpulse );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
			stateB.angularVelocity += iB * ( Cross( rB, joint.linearImpulse ) + axialImpulse );
		}
	}

	internal static void SolveRevoluteJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.revoluteJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref RevoluteJoint joint = ref @base.revoluteJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		Rot qA = MulRot( stateA.deltaRotation, joint.frameA.q );
		Rot qB = MulRot( stateB.deltaRotation, joint.frameB.q );
		Rot relQ = InvMulRot( qA, qB );

		// current anchors
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 linearCdot = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
		float angularCdot = wB - wA;
		float axialImpulse = 0.0f;

		float k = iA + iB;
		bool fixedRotation = ( k == 0.0f );

		// Solve spring.
		if ( joint.enableSpring && fixedRotation == false )
		{
			float jointAngle = Rot_GetAngle( relQ );
			float jointAngleDelta = UnwindAngle( jointAngle - joint.targetAngle );

			float C = jointAngleDelta;
			float bias = joint.springSoftness.biasRate * C;
			float massScale = joint.springSoftness.massScale;
			float impulseScale = joint.springSoftness.impulseScale;

			float Cdot = angularCdot;
			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
			joint.springImpulse += impulse;

			angularCdot += k * impulse;
			axialImpulse += impulse;
		}

		// Solve motor constraint.
		if ( joint.enableMotor && fixedRotation == false )
		{
			float Cdot = angularCdot - joint.motorSpeed;
			float oldImpulse = joint.motorImpulse;
			float maxImpulse = context.h * joint.maxMotorTorque;
			float impulse = ClampFloat( -joint.axialMass * Cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse );
			joint.motorImpulse = oldImpulse + impulse;

			angularCdot += k * impulse;
			axialImpulse += impulse;
		}

		if ( joint.enableLimit && fixedRotation == false )
		{
			float jointAngle = Rot_GetAngle( relQ );

			// Lower limit
			{
				float C = jointAngle - joint.lowerAngle;
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context.inv_h;
				}
				else if ( useBias )
				{
					bias = @base.constraintSoftness.biasRate * C;
					massScale = @base.constraintSoftness.massScale;
					impulseScale = @base.constraintSoftness.impulseScale;
				}

				float Cdot = angularCdot;
				float oldImpulse = joint.lowerImpulse;
				float impulse = MaxFloat( -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse, -oldImpulse );
				joint.lowerImpulse = oldImpulse + impulse;

				angularCdot += k * impulse;
				axialImpulse += impulse;
			}

			// Upper limit
			// Note: signs are flipped to keep C positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				float C = joint.upperAngle - jointAngle;
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context.inv_h;
				}
				else if ( useBias )
				{
					bias = @base.constraintSoftness.biasRate * C;
					massScale = @base.constraintSoftness.massScale;
					impulseScale = @base.constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				float Cdot = -angularCdot;
				float oldImpulse = joint.upperImpulse;
				float impulse = MaxFloat( -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse, -oldImpulse );
				joint.upperImpulse = oldImpulse + impulse;

				// sign flipped on applied impulse
				axialImpulse -= impulse;
			}
		}

		// Solve point-to-point constraint
		{
			// J = [-I -r1_skew I r2_skew]
			// r_skew = [-ry; rx]
			// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
			//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

			Vec2 Cdot = MulAdd( linearCdot, axialImpulse, Add( CrossSV( iA, rA ), CrossSV( iB, rB ) ) );

			Vec2 bias = Vec2_zero;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				Vec2 dcA = stateA.deltaPosition;
				Vec2 dcB = stateB.deltaPosition;

				Vec2 separation = Add( Add( Sub( dcB, dcA ), Sub( rB, rA ) ), joint.deltaCenter );
				bias = MulSV( @base.constraintSoftness.biasRate, separation );
				massScale = @base.constraintSoftness.massScale;
				impulseScale = @base.constraintSoftness.impulseScale;
			}

			Mat22 K = default;
			K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
			K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
			K.cx.y = K.cy.x;
			K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
			Vec2 b = Solve22( K, Add( Cdot, bias ) );

			Vec2 impulse = default;
			impulse.x = -massScale * b.x - impulseScale * joint.linearImpulse.x;
			impulse.y = -massScale * b.y - impulseScale * joint.linearImpulse.y;
			joint.linearImpulse.x += impulse.x;
			joint.linearImpulse.y += impulse.y;

			vA = MulSub( vA, mA, impulse );
			wA -= iA * ( Cross( rA, impulse ) + axialImpulse );
			vB = MulAdd( vB, mB, impulse );
			wB += iB * ( Cross( rB, impulse ) + axialImpulse );
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

	internal static void DrawRevoluteJoint( ref DebugDraw draw, ref JointSim @base, WorldTransform transformA, WorldTransform transformB, float drawScale )
	{
		B2_ASSERT( @base.type == JointType.revoluteJoint );

		ref RevoluteJoint joint = ref @base.revoluteJoint;
		WorldTransform frameA = MulWorldTransforms( transformA, @base.localFrameA );
		WorldTransform frameB = MulWorldTransforms( transformB, @base.localFrameB );

		float radius = 0.25f * drawScale;
		draw.DrawCircleFcn( frameB.p, radius, HexColor.colorGray, draw.context );

		Vec2 rx = new Vec2( radius, 0.0f );
		Vec2 r = RotateVector( frameA.q, rx );
		draw.DrawLineFcn( frameA.p, OffsetPos( frameA.p, r ), HexColor.colorGray, draw.context );

		r = RotateVector( frameB.q, rx );
		draw.DrawLineFcn( frameB.p, OffsetPos( frameB.p, r ), HexColor.colorBlue, draw.context );

		if ( draw.drawJointExtras )
		{
			float jointAngle = RelativeAngle( frameA.q, frameB.q );
			string buffer = $" {180.0f * jointAngle / Pi:F1} deg";
			draw.DrawStringFcn( OffsetPos( frameA.p, r ), buffer, HexColor.colorWhite, draw.context );
		}

		float lowerAngle = joint.lowerAngle;
		float upperAngle = joint.upperAngle;

		if ( joint.enableLimit )
		{
			Rot rotLo = MulRot( frameA.q, MakeRot( lowerAngle ) );
			Vec2 rlo = RotateVector( rotLo, rx );

			Rot rotHi = MulRot( frameA.q, MakeRot( upperAngle ) );
			Vec2 rhi = RotateVector( rotHi, rx );

			draw.DrawLineFcn( frameB.p, OffsetPos( frameB.p, rlo ), HexColor.colorGreen, draw.context );
			draw.DrawLineFcn( frameB.p, OffsetPos( frameB.p, rhi ), HexColor.colorRed, draw.context );
		}

		if ( joint.enableSpring )
		{
			Rot q = MulRot( frameA.q, MakeRot( joint.targetAngle ) );
			Vec2 v = RotateVector( q, rx );
			draw.DrawLineFcn( frameB.p, OffsetPos( frameB.p, v ), HexColor.colorViolet, draw.context );
		}

		HexColor color = HexColor.colorGold;
		draw.DrawLineFcn( frameA.p, frameB.p, color, draw.context );

		if ( draw.drawJointExtras )
		{
			draw.DrawLineFcn( transformA.p, frameA.p, color, draw.context );
			draw.DrawLineFcn( transformB.p, frameB.p, color, draw.context );
		}

		// byte buffer[32];
		// sprintf(buffer, "%.1f", Length(joint.impulse));
		// draw.DrawString(pA, buffer, draw.context);
	}

}
