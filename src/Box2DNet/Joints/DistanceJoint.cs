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

	public static void DistanceJoint_SetRestLength( JointId jointId, float length )
	{
		if ( CheckInput( IsValidFloat( length ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		joint.length = ClampFloat( length, LinearSlop, Huge );
		joint.impulse = 0.0f;
		joint.lowerImpulse = 0.0f;
		joint.upperImpulse = 0.0f;
	}

	public static float DistanceJoint_GetRestLength( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		return joint.length;
	}

	public static void DistanceJoint_EnableLimit( JointId jointId, bool enableLimit )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		joint.enableLimit = enableLimit;
	}

	public static bool DistanceJoint_IsLimitEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return joint.distanceJoint.enableLimit;
	}

	public static void DistanceJoint_SetLimits( JointId jointId, float minLength, float maxLength )
	{
		if ( CheckInput( IsValidFloat( minLength ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( maxLength ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		minLength = ClampFloat( minLength, LinearSlop, Huge );
		maxLength = ClampFloat( maxLength, LinearSlop, Huge );
		joint.minLength = MinFloat( minLength, maxLength );
		joint.maxLength = MaxFloat( minLength, maxLength );
	}

	public static float DistanceJoint_GetLowerLimit( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		return joint.minLength;
	}

	public static float DistanceJoint_GetUpperLimit( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		return joint.maxLength;
	}

	public static float DistanceJoint_GetCurrentLength( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return 0.0f;
		}

		// Relative to body A so the difference stays in float precision far from the origin
		WorldTransform wxfA = GetBodyTransform( world, @base.bodyIdA );
		Transform transformA = ToRelativeTransform( wxfA, wxfA.p );
		Transform transformB = ToRelativeTransform( GetBodyTransform( world, @base.bodyIdB ), wxfA.p );

		Vec2 pA = TransformPoint( transformA, @base.localFrameA.p );
		Vec2 pB = TransformPoint( transformB, @base.localFrameB.p );
		Vec2 d = Sub( pB, pA );
		float length = Length( d );
		return length;
	}

	public static void DistanceJoint_EnableSpring( JointId jointId, bool enableSpring )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		@base.distanceJoint.enableSpring = enableSpring;
	}

	public static bool DistanceJoint_IsSpringEnabled( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return @base.distanceJoint.enableSpring;
	}

	public static void DistanceJoint_SetSpringForceRange( JointId jointId, float lowerForce, float upperForce )
	{
		if ( CheckInput( IsValidFloat( lowerForce ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( upperForce ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( lowerForce <= upperForce );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		@base.distanceJoint.lowerSpringForce = MinFloat( lowerForce, upperForce );
		@base.distanceJoint.upperSpringForce = MaxFloat( lowerForce, upperForce );
	}

	public static void DistanceJoint_GetSpringForceRange( JointId jointId, ref float lowerForce, ref float upperForce )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		lowerForce = @base.distanceJoint.lowerSpringForce;
		upperForce = @base.distanceJoint.upperSpringForce;
	}

	public static void DistanceJoint_SetSpringHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		@base.distanceJoint.hertz = hertz;
	}

	public static void DistanceJoint_SetSpringDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		@base.distanceJoint.dampingRatio = dampingRatio;
	}

	public static float DistanceJoint_GetSpringHertz( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		return joint.hertz;
	}

	public static float DistanceJoint_GetSpringDampingRatio( JointId jointId )
	{
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		return joint.dampingRatio;
	}

	public static float DistanceJoint_GetSpringForce( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		ref DistanceJoint joint = ref @base.distanceJoint;
		if ( joint.enableSpring )
		{
			return joint.impulse * world.inv_h;
		}

		return 0.0f;
	}

	public static void DistanceJoint_EnableMotor( JointId jointId, bool enableMotor )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		if ( enableMotor != joint.distanceJoint.enableMotor )
		{
			joint.distanceJoint.enableMotor = enableMotor;
			joint.distanceJoint.motorImpulse = 0.0f;
		}
	}

	public static bool DistanceJoint_IsMotorEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return joint.distanceJoint.enableMotor;
	}

	public static void DistanceJoint_SetTargetMotorSpeed( JointId jointId, float motorSpeed )
	{
		if ( CheckInput( IsValidFloat( motorSpeed ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		joint.distanceJoint.motorSpeed = motorSpeed;
	}

	public static float DistanceJoint_GetTargetMotorSpeed( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return joint.distanceJoint.motorSpeed;
	}

	public static float DistanceJoint_GetCurrentMotorForce( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return world.inv_h * @base.distanceJoint.motorImpulse;
	}

	public static void DistanceJoint_SetMaxMotorForce( JointId jointId, float force )
	{
		if ( CheckInput( IsValidFloat( force ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		joint.distanceJoint.maxMotorForce = force;
	}

	public static float DistanceJoint_GetMaxMotorForce( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.distanceJoint );
		return joint.distanceJoint.maxMotorForce;
	}

	internal static Vec2 GetDistanceJointForce( World world, ref JointSim @base )
	{
		ref DistanceJoint joint = ref @base.distanceJoint;
		WorldTransform worldTransformA = GetBodyTransform( world, @base.bodyIdA );
		Transform transformA = ToRelativeTransform( worldTransformA, worldTransformA.p );
		WorldTransform worldTransformB = GetBodyTransform( world, @base.bodyIdB );
		Transform transformB = ToRelativeTransform( worldTransformB, worldTransformA.p );

		Vec2 pA = TransformPoint( transformA, @base.localFrameA.p );
		Vec2 pB = TransformPoint( transformB, @base.localFrameB.p );
		Vec2 d = Sub( pB, pA );
		Vec2 axis = Normalize( d );
		float force = ( joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse ) * world.inv_h;
		return MulSV( force, axis );
	}

	// 1-D constrained system
	// m (v2 - v1) = lambda
	// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
	// x2 = x1 + h * v2

	// 1-D mass-damper-spring system
	// m (v2 - v1) + h * d * v2 + h * k *

	// C = norm(p2 - p1) - L
	// u = (p2 - p1) / norm(p2 - p1)
	// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
	// J = [-u -cross(r1, u) u cross(r2, u)]
	// K = J * invM * JT
	//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

	internal static void PrepareDistanceJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.distanceJoint );

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

		ref DistanceJoint joint = ref @base.distanceJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		// initial anchors in world space
		joint.anchorA = RotateVector( bodySimA.transform.q, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		joint.anchorB = RotateVector( bodySimB.transform.q, Sub( @base.localFrameB.p, bodySimB.localCenter ) );
		joint.deltaCenter = SubPos( bodySimB.center, bodySimA.center );

		Vec2 rA = joint.anchorA;
		Vec2 rB = joint.anchorB;
		Vec2 separation = Add( Sub( rB, rA ), joint.deltaCenter );
		Vec2 axis = Normalize( separation );

		// compute effective mass
		float crA = Cross( rA, axis );
		float crB = Cross( rB, axis );
		float k = mA + mB + iA * crA * crA + iB * crB * crB;
		joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

		joint.distanceSoftness = MakeSoft( joint.hertz, joint.dampingRatio, context.h );

		if ( context.enableWarmStarting == false )
		{
			joint.impulse = 0.0f;
			joint.lowerImpulse = 0.0f;
			joint.upperImpulse = 0.0f;
			joint.motorImpulse = 0.0f;
		}
	}

	internal static void WarmStartDistanceJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.distanceJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref DistanceJoint joint = ref @base.distanceJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.anchorA );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.anchorB );

		Vec2 ds = Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), Sub( rB, rA ) );
		Vec2 separation = Add( joint.deltaCenter, ds );
		Vec2 axis = Normalize( separation );

		float axialImpulse = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse;
		Vec2 P = MulSV( axialImpulse, axis );

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, P );
			stateA.angularVelocity -= iA * Cross( rA, P );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, P );
			stateB.angularVelocity += iB * Cross( rB, P );
		}
	}

	internal static void SolveDistanceJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.distanceJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref DistanceJoint joint = ref @base.distanceJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		// current anchors
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.anchorA );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.anchorB );

		// current separation
		Vec2 ds = Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), Sub( rB, rA ) );
		Vec2 separation = Add( joint.deltaCenter, ds );

		float length = Length( separation );
		Vec2 axis = Normalize( separation );

		// joint is soft if
		// - spring is enabled
		// - and (joint limit is disabled or limits are not equal)
		if ( joint.enableSpring && ( joint.minLength < joint.maxLength || joint.enableLimit == false ) )
		{
			// spring
			if ( joint.hertz > 0.0f )
			{
				// Cdot = dot(u, v + cross(w, r))
				Vec2 vr = Add( Sub( vB, vA ), Sub( CrossSV( wB, rB ), CrossSV( wA, rA ) ) );
				float cdot = Dot( axis, vr );
				float c = length - joint.length;
				float bias = joint.distanceSoftness.biasRate * c;

				float m = joint.distanceSoftness.massScale * joint.axialMass;
				float oldImpulse = joint.impulse;
				float impulse = -m * ( cdot + bias ) - joint.distanceSoftness.impulseScale * oldImpulse;

				float h = context.h;
				joint.impulse = ClampFloat( joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h );
				impulse = joint.impulse - oldImpulse;

				Vec2 P = MulSV( impulse, axis );
				vA = MulSub( vA, mA, P );
				wA -= iA * Cross( rA, P );
				vB = MulAdd( vB, mB, P );
				wB += iB * Cross( rB, P );
			}

			if ( joint.enableMotor )
			{
				Vec2 vr = Add( Sub( vB, vA ), Sub( CrossSV( wB, rB ), CrossSV( wA, rA ) ) );
				float Cdot = Dot( axis, vr );
				float impulse = joint.axialMass * ( joint.motorSpeed - Cdot );
				float oldImpulse = joint.motorImpulse;
				float maxImpulse = context.h * joint.maxMotorForce;
				joint.motorImpulse = ClampFloat( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
				impulse = joint.motorImpulse - oldImpulse;

				Vec2 P = MulSV( impulse, axis );
				vA = MulSub( vA, mA, P );
				wA -= iA * Cross( rA, P );
				vB = MulAdd( vB, mB, P );
				wB += iB * Cross( rB, P );
			}

			if ( joint.enableLimit )
			{
				// lower limit
				{
					Vec2 vr = Add( Sub( vB, vA ), Sub( CrossSV( wB, rB ), CrossSV( wA, rA ) ) );
					float cdot = Dot( axis, vr );

					float c = length - joint.minLength;

					float bias = 0.0f;
					float massScale = 1.0f;
					float impulseScale = 0.0f;
					if ( c > 0.0f )
					{
						// speculative
						bias = c * context.inv_h;
					}
					else if ( useBias )
					{
						bias = @base.constraintSoftness.biasRate * c;
						massScale = @base.constraintSoftness.massScale;
						impulseScale = @base.constraintSoftness.impulseScale;
					}

					float impulse = -massScale * joint.axialMass * ( cdot + bias ) - impulseScale * joint.lowerImpulse;
					float newImpulse = MaxFloat( 0.0f, joint.lowerImpulse + impulse );
					impulse = newImpulse - joint.lowerImpulse;
					joint.lowerImpulse = newImpulse;

					Vec2 P = MulSV( impulse, axis );
					vA = MulSub( vA, mA, P );
					wA -= iA * Cross( rA, P );
					vB = MulAdd( vB, mB, P );
					wB += iB * Cross( rB, P );
				}

				// upper
				{
					Vec2 vr = Add( Sub( vA, vB ), Sub( CrossSV( wA, rA ), CrossSV( wB, rB ) ) );
					float Cdot = Dot( axis, vr );

					float C = joint.maxLength - length;

					float bias = 0.0f;
					float massScale = 1.0f;
					float impulseScale = 0.0f;
					if ( C > 0.0f )
					{
						// speculative
						bias = C * context.inv_h;
					}
					else if ( useBias )
					{
						bias = @base.constraintSoftness.biasRate * C;
						massScale = @base.constraintSoftness.massScale;
						impulseScale = @base.constraintSoftness.impulseScale;
					}

					float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.upperImpulse;
					float newImpulse = MaxFloat( 0.0f, joint.upperImpulse + impulse );
					impulse = newImpulse - joint.upperImpulse;
					joint.upperImpulse = newImpulse;

					Vec2 P = MulSV( -impulse, axis );
					vA = MulSub( vA, mA, P );
					wA -= iA * Cross( rA, P );
					vB = MulAdd( vB, mB, P );
					wB += iB * Cross( rB, P );
				}
			}
		}
		else
		{
			// rigid constraint
			Vec2 vr = Add( Sub( vB, vA ), Sub( CrossSV( wB, rB ), CrossSV( wA, rA ) ) );
			float Cdot = Dot( axis, vr );

			float C = length - joint.length;

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				bias = @base.constraintSoftness.biasRate * C;
				massScale = @base.constraintSoftness.massScale;
				impulseScale = @base.constraintSoftness.impulseScale;
			}

			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.impulse;
			joint.impulse += impulse;

			Vec2 P = MulSV( impulse, axis );
			vA = MulSub( vA, mA, P );
			wA -= iA * Cross( rA, P );
			vB = MulAdd( vB, mB, P );
			wB += iB * Cross( rB, P );
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

	internal static void DrawDistanceJoint( ref DebugDraw draw, ref JointSim @base, WorldTransform transformA, WorldTransform transformB )
	{
		B2_ASSERT( @base.type == JointType.distanceJoint );

		ref DistanceJoint joint = ref @base.distanceJoint;
		Pos pA = TransformWorldPoint( transformA, @base.localFrameA.p );
		Pos pB = TransformWorldPoint( transformB, @base.localFrameB.p );

		Vec2 axis = Normalize( SubPos( pB, pA ) );

		if ( joint.minLength < joint.maxLength && joint.enableLimit )
		{
			Pos pMin = OffsetPos( pA, MulSV( joint.minLength, axis ) );
			Pos pMax = OffsetPos( pA, MulSV( joint.maxLength, axis ) );
			Vec2 offset = MulSV( 0.05f * GetLengthUnitsPerMeter(), RightPerp( axis ) );

			if ( joint.minLength > LinearSlop )
			{
				draw.DrawLineFcn( OffsetPos( pMin, Neg( offset ) ), OffsetPos( pMin, offset ), HexColor.colorLightGreen,
								   draw.context );
			}

			if ( joint.maxLength < Huge )
			{
				draw.DrawLineFcn( OffsetPos( pMax, Neg( offset ) ), OffsetPos( pMax, offset ), HexColor.colorRed, draw.context );
			}

			if ( joint.minLength > LinearSlop && joint.maxLength < Huge )
			{
				draw.DrawLineFcn( pMin, pMax, HexColor.colorGray, draw.context );
			}
		}

		draw.DrawLineFcn( pA, pB, HexColor.colorWhite, draw.context );
		draw.DrawPointFcn( pA, 4.0f, HexColor.colorWhite, draw.context );
		draw.DrawPointFcn( pB, 4.0f, HexColor.colorWhite, draw.context );

		if ( joint.hertz > 0.0f && joint.enableSpring )
		{
			Pos pRest = OffsetPos( pA, MulSV( joint.length, axis ) );
			draw.DrawPointFcn( pRest, 4.0f, HexColor.colorBlue, draw.context );
		}
	}

}
