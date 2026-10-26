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

	public static void WheelJoint_EnableSpring( JointId jointId, bool enableSpring )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		if ( enableSpring != joint.wheelJoint.enableSpring )
		{
			joint.wheelJoint.enableSpring = enableSpring;
			joint.wheelJoint.springImpulse = 0.0f;
		}
	}

	public static bool WheelJoint_IsSpringEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.enableSpring;
	}

	public static void WheelJoint_SetSpringHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		joint.wheelJoint.hertz = hertz;
	}

	public static float WheelJoint_GetSpringHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.hertz;
	}

	public static void WheelJoint_SetSpringDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		joint.wheelJoint.dampingRatio = dampingRatio;
	}

	public static float WheelJoint_GetSpringDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.dampingRatio;
	}

	public static void WheelJoint_EnableLimit( JointId jointId, bool enableLimit )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		if ( joint.wheelJoint.enableLimit != enableLimit )
		{
			joint.wheelJoint.lowerImpulse = 0.0f;
			joint.wheelJoint.upperImpulse = 0.0f;
			joint.wheelJoint.enableLimit = enableLimit;
		}
	}

	public static bool WheelJoint_IsLimitEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.enableLimit;
	}

	public static float WheelJoint_GetLowerLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.lowerTranslation;
	}

	public static float WheelJoint_GetUpperLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.upperTranslation;
	}

	public static void WheelJoint_SetLimits( JointId jointId, float lower, float upper )
	{
		if ( CheckInput( IsValidFloat( lower ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( upper ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( lower <= upper );

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		joint.wheelJoint.lowerTranslation = MinFloat( lower, upper );
		joint.wheelJoint.upperTranslation = MaxFloat( lower, upper );
	}

	public static void WheelJoint_EnableMotor( JointId jointId, bool enableMotor )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		if ( joint.wheelJoint.enableMotor != enableMotor )
		{
			joint.wheelJoint.motorImpulse = 0.0f;
			joint.wheelJoint.enableMotor = enableMotor;
		}
	}

	public static bool WheelJoint_IsMotorEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.enableMotor;
	}

	public static void WheelJoint_SetTargetMotorSpeed( JointId jointId, float motorSpeed )
	{
		if ( CheckInput( IsValidFloat( motorSpeed ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		joint.wheelJoint.motorSpeed = motorSpeed;
	}

	public static float WheelJoint_GetTargetMotorSpeed( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.motorSpeed;
	}

	public static float WheelJoint_GetCurrentMotorTorque( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return world.inv_h * joint.wheelJoint.motorImpulse;
	}

	public static void WheelJoint_SetMaxMotorTorque( JointId jointId, float torque )
	{
		if ( CheckInput( IsValidFloat( torque ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		joint.wheelJoint.maxMotorTorque = torque;
	}

	public static float WheelJoint_GetMaxMotorTorque( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.wheelJoint );
		return joint.wheelJoint.maxMotorTorque;
	}

	internal static Vec2 GetWheelJointForce( World world, ref JointSim @base )
	{
		Rot qA = GetBodyTransform( world, @base.bodyIdA ).q;

		Vec2 localAxisA = RotateVector( @base.localFrameA.q, new Vec2( 1.0f, 0.0f ) );
		Vec2 axisA = RotateVector( qA, localAxisA );
		Vec2 perpA = LeftPerp( axisA );

		ref WheelJoint joint = ref @base.wheelJoint;
		float perpForce = world.inv_h * joint.perpImpulse;
		float axialForce = world.inv_h * ( joint.springImpulse + joint.lowerImpulse - joint.upperImpulse );

		Vec2 force = Add( MulSV( perpForce, perpA ), MulSV( axialForce, axisA ) );
		return force;
	}

	internal static float GetWheelJointTorque( World world, ref JointSim @base )
	{
		return world.inv_h * @base.wheelJoint.motorImpulse;
	}

	// Linear constraint (point-to-line)
	// d = pB - pA = xB + rB - xA - rA
	// C = dot(ay, d)
	// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
	//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
	// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

	// Spring linear constraint
	// C = dot(ax, d)
	// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
	// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

	// Motor rotational constraint
	// Cdot = wB - wA
	// J = [0 0 -1 0 0 1]

	internal static void PrepareWheelJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.wheelJoint );

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

		ref WheelJoint joint = ref @base.wheelJoint;
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

		Vec2 d = Add( joint.deltaCenter, Sub( rB, rA ) );
		Vec2 axisA = RotateVector( joint.frameA.q, new Vec2( 1.0f, 0.0f ) );
		Vec2 perpA = LeftPerp( axisA );

		// perpendicular constraint (keep wheel on line)
		float s1 = Cross( Add( d, rA ), perpA );
		float s2 = Cross( rB, perpA );

		float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		joint.perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

		// spring constraint
		float a1 = Cross( Add( d, rA ), axisA );
		float a2 = Cross( rB, axisA );

		float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
		joint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

		joint.springSoftness = MakeSoft( joint.hertz, joint.dampingRatio, context.h );

		float km = iA + iB;
		joint.motorMass = km > 0.0f ? 1.0f / km : 0.0f;

		if ( context.enableWarmStarting == false )
		{
			joint.perpImpulse = 0.0f;
			joint.springImpulse = 0.0f;
			joint.motorImpulse = 0.0f;
			joint.lowerImpulse = 0.0f;
			joint.upperImpulse = 0.0f;
		}
	}

	internal static void WarmStartWheelJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.wheelJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref WheelJoint joint = ref @base.wheelJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 d = Add( Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), Sub( rB, rA ) );
		Vec2 axisA = RotateVector( joint.frameA.q, new Vec2( 1.0f, 0.0f ) );
		axisA = RotateVector( stateA.deltaRotation, axisA );
		Vec2 perpA = LeftPerp( axisA );

		float a1 = Cross( Add( d, rA ), axisA );
		float a2 = Cross( rB, axisA );
		float s1 = Cross( Add( d, rA ), perpA );
		float s2 = Cross( rB, perpA );

		float axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;

		Vec2 P = Add( MulSV( axialImpulse, axisA ), MulSV( joint.perpImpulse, perpA ) );
		float LA = axialImpulse * a1 + joint.perpImpulse * s1 + joint.motorImpulse;
		float LB = axialImpulse * a2 + joint.perpImpulse * s2 + joint.motorImpulse;

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, P );
			stateA.angularVelocity -= iA * LA;
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, P );
			stateB.angularVelocity += iB * LB;
		}
	}

	internal static void SolveWheelJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.wheelJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref WheelJoint joint = ref @base.wheelJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		bool fixedRotation = ( iA + iB == 0.0f );

		// current anchors
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 d = Add( Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), Sub( rB, rA ) );
		Vec2 axisA = RotateVector( joint.frameA.q, new Vec2( 1.0f, 0.0f ) );
		axisA = RotateVector( stateA.deltaRotation, axisA );
		float translation = Dot( axisA, d );

		float a1 = Cross( Add( d, rA ), axisA );
		float a2 = Cross( rB, axisA );

		// motor constraint
		if ( joint.enableMotor && fixedRotation == false )
		{
			float Cdot = wB - wA - joint.motorSpeed;
			float impulse = -joint.motorMass * Cdot;
			float oldImpulse = joint.motorImpulse;
			float maxImpulse = context.h * joint.maxMotorTorque;
			joint.motorImpulse = ClampFloat( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint.motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// spring constraint
		if ( joint.enableSpring )
		{
			// This is a real spring and should be applied even during relax
			float C = translation;
			float bias = joint.springSoftness.biasRate * C;
			float massScale = joint.springSoftness.massScale;
			float impulseScale = joint.springSoftness.impulseScale;

			float Cdot = Dot( axisA, Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
			joint.springImpulse += impulse;

			Vec2 P = MulSV( impulse, axisA );
			float LA = impulse * a1;
			float LB = impulse * a2;

			vA = MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = MulAdd( vB, mB, P );
			wB += iB * LB;
		}

		if ( joint.enableLimit )
		{
			// Lower limit
			{
				float C = translation - joint.lowerTranslation;
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

				float Cdot = Dot( axisA, Sub( vB, vA ) ) + a2 * wB - a1 * wA;
				float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.lowerImpulse;
				float oldImpulse = joint.lowerImpulse;
				joint.lowerImpulse = MaxFloat( oldImpulse + impulse, 0.0f );
				impulse = joint.lowerImpulse - oldImpulse;

				Vec2 P = MulSV( impulse, axisA );
				float LA = impulse * a1;
				float LB = impulse * a2;

				vA = MulSub( vA, mA, P );
				wA -= iA * LA;
				vB = MulAdd( vB, mB, P );
				wB += iB * LB;
			}

			// Upper limit
			// Note: signs are flipped to keep C positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				// sign flipped
				float C = joint.upperTranslation - translation;
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
				float Cdot = Dot( axisA, Sub( vA, vB ) ) + a1 * wA - a2 * wB;
				float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.upperImpulse;
				float oldImpulse = joint.upperImpulse;
				joint.upperImpulse = MaxFloat( oldImpulse + impulse, 0.0f );
				impulse = joint.upperImpulse - oldImpulse;

				Vec2 P = MulSV( impulse, axisA );
				float LA = impulse * a1;
				float LB = impulse * a2;

				// sign flipped on applied impulse
				vA = MulAdd( vA, mA, P );
				wA += iA * LA;
				vB = MulSub( vB, mB, P );
				wB -= iB * LB;
			}
		}

		// point to line constraint
		{
			Vec2 perpA = LeftPerp( axisA );

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				float C = Dot( perpA, d );
				bias = @base.constraintSoftness.biasRate * C;
				massScale = @base.constraintSoftness.massScale;
				impulseScale = @base.constraintSoftness.impulseScale;
			}

			float s1 = Cross( Add( d, rA ), perpA );
			float s2 = Cross( rB, perpA );
			float Cdot = Dot( perpA, Sub( vB, vA ) ) + s2 * wB - s1 * wA;

			float impulse = -massScale * joint.perpMass * ( Cdot + bias ) - impulseScale * joint.perpImpulse;
			joint.perpImpulse += impulse;

			Vec2 P = MulSV( impulse, perpA );
			float LA = impulse * s1;
			float LB = impulse * s2;

			vA = MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = MulAdd( vB, mB, P );
			wB += iB * LB;
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

#if false
	internal static void WheelJoint_Dump()
	{
		int32 indexA = joint.bodyA.joint.islandIndex;
		int32 indexB = joint.bodyB.joint.islandIndex;

		Dump("  WheelJointDef jd;\n");
		Dump("  jd.bodyA = sims[%d];\n", indexA);
		Dump("  jd.bodyB = sims[%d];\n", indexB);
		Dump("  jd.collideConnected = bool(%d);\n", joint.collideConnected);
		Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", joint.localAnchorA.x, joint.localAnchorA.y);
		Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", joint.localAnchorB.x, joint.localAnchorB.y);
		Dump("  jd.referenceAngle = %.9g;\n", joint.referenceAngle);
		Dump("  jd.enableLimit = bool(%d);\n", joint.enableLimit);
		Dump("  jd.lowerAngle = %.9g;\n", joint.lowerAngle);
		Dump("  jd.upperAngle = %.9g;\n", joint.upperAngle);
		Dump("  jd.enableMotor = bool(%d);\n", joint.enableMotor);
		Dump("  jd.motorSpeed = %.9g;\n", joint.motorSpeed);
		Dump("  jd.maxMotorTorque = %.9g;\n", joint.maxMotorTorque);
		Dump("  joints[%d] = joint.world.CreateJoint(ref jd);\n", joint.index);
	}
#endif

	internal static void DrawWheelJoint( ref DebugDraw draw, ref JointSim @base, WorldTransform transformA, WorldTransform transformB, float drawScale )
	{
		B2_ASSERT( @base.type == JointType.wheelJoint );

		ref WheelJoint joint = ref @base.wheelJoint;
		WorldTransform frameA = MulWorldTransforms( transformA, @base.localFrameA );
		WorldTransform frameB = MulWorldTransforms( transformB, @base.localFrameB );
		Vec2 axisA = RotateVector( frameA.q, new Vec2( 1.0f, 0.0f ) );

		HexColor c1 = HexColor.colorGray;
		HexColor c2 = HexColor.colorGreen;
		HexColor c3 = HexColor.colorRed;
		HexColor c4 = HexColor.colorDimGray;
		HexColor c5 = HexColor.colorBlue;

		draw.DrawLineFcn( frameA.p, frameB.p, c5, draw.context );

		if ( joint.enableLimit )
		{
			Pos lower = OffsetPos( frameA.p, MulSV( joint.lowerTranslation, axisA ) );
			Pos upper = OffsetPos( frameA.p, MulSV( joint.upperTranslation, axisA ) );
			Vec2 perp = LeftPerp( axisA );
			draw.DrawLineFcn( lower, upper, c1, draw.context );
			draw.DrawLineFcn( OffsetPos( lower, MulSV( -0.1f * drawScale, perp ) ),
							   OffsetPos( lower, MulSV( 0.1f * drawScale, perp ) ), c2, draw.context );
			draw.DrawLineFcn( OffsetPos( upper, MulSV( -0.1f * drawScale, perp ) ),
							   OffsetPos( upper, MulSV( 0.1f * drawScale, perp ) ), c3, draw.context );
		}
		else
		{
			draw.DrawLineFcn( OffsetPos( frameA.p, Neg( axisA ) ), OffsetPos( frameA.p, axisA ), c1, draw.context );
		}

		draw.DrawPointFcn( frameA.p, 5.0f, c1, draw.context );
		draw.DrawPointFcn( frameB.p, 5.0f, c4, draw.context );
	}

}
