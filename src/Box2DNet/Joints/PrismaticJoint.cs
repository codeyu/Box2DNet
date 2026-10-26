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

	public static void PrismaticJoint_EnableSpring( JointId jointId, bool enableSpring )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		if ( enableSpring != joint.prismaticJoint.enableSpring )
		{
			joint.prismaticJoint.enableSpring = enableSpring;
			joint.prismaticJoint.springImpulse = 0.0f;
		}
	}

	public static bool PrismaticJoint_IsSpringEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.enableSpring;
	}

	public static void PrismaticJoint_SetSpringHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.hertz = hertz;
	}

	public static float PrismaticJoint_GetSpringHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.hertz;
	}

	public static void PrismaticJoint_SetSpringDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.dampingRatio = dampingRatio;
	}

	public static float PrismaticJoint_GetSpringDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.dampingRatio;
	}

	public static void PrismaticJoint_SetTargetTranslation( JointId jointId, float translation )
	{
		if ( CheckInput( IsValidFloat( translation ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.targetTranslation = translation;
	}

	public static float PrismaticJoint_GetTargetTranslation( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.targetTranslation;
	}

	public static void PrismaticJoint_EnableLimit( JointId jointId, bool enableLimit )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		if ( enableLimit != joint.prismaticJoint.enableLimit )
		{
			joint.prismaticJoint.enableLimit = enableLimit;
			joint.prismaticJoint.lowerImpulse = 0.0f;
			joint.prismaticJoint.upperImpulse = 0.0f;
		}
	}

	public static bool PrismaticJoint_IsLimitEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.enableLimit;
	}

	public static float PrismaticJoint_GetLowerLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.lowerTranslation;
	}

	public static float PrismaticJoint_GetUpperLimit( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.upperTranslation;
	}

	public static void PrismaticJoint_SetLimits( JointId jointId, float lower, float upper )
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

		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.lowerTranslation = MinFloat( lower, upper );
		joint.prismaticJoint.upperTranslation = MaxFloat( lower, upper );
	}

	public static void PrismaticJoint_EnableMotor( JointId jointId, bool enableMotor )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		if ( enableMotor != joint.prismaticJoint.enableMotor )
		{
			joint.prismaticJoint.enableMotor = enableMotor;
			joint.prismaticJoint.motorImpulse = 0.0f;
		}
	}

	public static bool PrismaticJoint_IsMotorEnabled( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.enableMotor;
	}

	public static void PrismaticJoint_SetTargetMotorSpeed( JointId jointId, float motorSpeed )
	{
		if ( CheckInput( IsValidFloat( motorSpeed ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.motorSpeed = motorSpeed;
	}

	public static float PrismaticJoint_GetTargetMotorSpeed( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.motorSpeed;
	}

	public static float PrismaticJoint_GetCurrentMotorForce( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim @base = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return world.inv_h * @base.prismaticJoint.motorImpulse;
	}

	public static void PrismaticJoint_SetMaxMotorForce( JointId jointId, float force )
	{
		if ( CheckInput( IsValidFloat( force ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		joint.prismaticJoint.maxMotorForce = force;
	}

	public static float PrismaticJoint_GetMaxMotorForce( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		return joint.prismaticJoint.maxMotorForce;
	}

	public static float PrismaticJoint_GetCurrentTranslation( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref JointSim jointSim = ref GetJointSimCheckType( jointId, JointType.prismaticJoint );
		// Relative to body A so the difference stays in float precision far from the origin
		WorldTransform wxfA = GetBodyTransform( world, jointSim.bodyIdA );
		Transform transformA = ToRelativeTransform( wxfA, wxfA.p );
		Transform transformB = ToRelativeTransform( GetBodyTransform( world, jointSim.bodyIdB ), wxfA.p );

		Vec2 localAxisA = RotateVector( jointSim.localFrameA.q, new Vec2( 1.0f, 0.0f ) );
		Vec2 axisA = RotateVector( transformA.q, localAxisA );
		Vec2 pA = TransformPoint( transformA, jointSim.localFrameA.p );
		Vec2 pB = TransformPoint( transformB, jointSim.localFrameB.p );
		Vec2 d = Sub( pB, pA );
		float translation = Dot( d, axisA );
		return translation;
	}

	public static float PrismaticJoint_GetCurrentSpeed( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		B2_ASSERT( joint.type == JointType.prismaticJoint );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		B2_ASSERT( @base.type == JointType.prismaticJoint );

		ref Body bodyA = ref world.bodies[@base.bodyIdA];
		ref Body bodyB = ref world.bodies[@base.bodyIdB];
		ref BodySim bodySimA = ref GetBodySim( world, ref bodyA );
		ref BodySim bodySimB = ref GetBodySim( world, ref bodyB );
		ref BodyState bodyStateA = ref GetBodyState( world, ref bodyA );
		ref BodyState bodyStateB = ref GetBodyState( world, ref bodyB );
		Rot qA = bodySimA.transform.q;
		Rot qB = bodySimB.transform.q;

		Vec2 localAxisA = RotateVector( @base.localFrameA.q, new Vec2( 1.0f, 0.0f ) );
		Vec2 axisA = RotateVector( qA, localAxisA );
		Vec2 rA = RotateVector( qA, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		Vec2 rB = RotateVector( qB, Sub( @base.localFrameB.p, bodySimB.localCenter ) );

		// Difference the centers in double so the speed stays exact far from the origin
		Vec2 dc = SubPos( bodySimB.center, bodySimA.center );
		Vec2 d = Add( dc, Sub( rB, rA ) );

		Vec2 vA = Unsafe.IsNullRef( ref bodyStateA ) == false ? bodyStateA.linearVelocity : Vec2_zero;
		Vec2 vB = Unsafe.IsNullRef( ref bodyStateB ) == false ? bodyStateB.linearVelocity : Vec2_zero;
		float wA = Unsafe.IsNullRef( ref bodyStateA ) == false ? bodyStateA.angularVelocity : 0.0f;
		float wB = Unsafe.IsNullRef( ref bodyStateB ) == false ? bodyStateB.angularVelocity : 0.0f;

		Vec2 vRel = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
		float speed = Dot( d, CrossSV( wA, axisA ) ) + Dot( axisA, vRel );
		return speed;
	}

	internal static Vec2 GetPrismaticJointForce( World world, ref JointSim @base )
	{
		Rot qA = GetBodyTransform( world, @base.bodyIdA ).q;

		ref PrismaticJoint joint = ref @base.prismaticJoint;
		Vec2 localAxisA = RotateVector( @base.localFrameA.q, new Vec2( 1.0f, 0.0f ) );
		Vec2 axisA = RotateVector( qA, localAxisA );
		Vec2 perpA = LeftPerp( axisA );

		float inv_h = world.inv_h;
		float perpForce = inv_h * joint.impulse.x;
		float axialForce = inv_h * ( joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse );

		Vec2 force = Add( MulSV( perpForce, perpA ), MulSV( axialForce, axisA ) );
		return force;
	}

	internal static float GetPrismaticJointTorque( World world, ref JointSim @base )
	{
		return world.inv_h * @base.prismaticJoint.impulse.y;
	}

	// Linear constraint (point-to-line)
	// d = pB - pA = xB + rB - xA - rA
	// C = dot(perp, d)
	// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
	//      = -dot(perp, vA) - dot(cross(rA + d, perp), wA) + dot(perp, vB) + dot(cross(rB, perp), vB)
	// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
	//
	// Angular constraint
	// C = aB - aA + a_initial
	// Cdot = wB - wA
	// J = [0 0 -1 0 0 1]
	//
	// K = J * invM * JT
	//
	// J = [-a -sA a sB]
	//     [0  -1  0  1]
	// a = perp
	// sA = cross(rA + d, a) = cross(pB - xA, a)
	// sB = cross(rB, a) = cross(pB - xB, a)

	// Motor/Limit linear constraint
	// C = dot(axA, d)
	// Cdot = -dot(axA, vA) - dot(cross(rA + d, axA), wA) + dot(axA, vB) + dot(cross(rB, axA), vB)
	// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]

	// Predictive limit is applied even when the limit is not active.
	// Prevents a constraint speed that can lead to a constraint error in one time step.
	// Want C2 = C1 + h * Cdot >= 0
	// Or:
	// Cdot + C1/h >= 0
	// I do not apply a negative constraint error because that is handled in position correction.
	// So:
	// Cdot + max(C1, 0)/h >= 0

	// Block Solver
	// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
	//
	// The Jacobian has 2 rows:
	// J = [-uT -s1 uT s2] // linear
	//     [0   -1   0  1] // angular
	//
	// u = perp
	// s1 = cross(d + r1, u), s2 = cross(r2, u)
	// a1 = cross(d + r1, v), a2 = cross(r2, v)

	internal static void PreparePrismaticJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.prismaticJoint );

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

		ref PrismaticJoint joint = ref @base.prismaticJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		// Compute joint anchor frames with world space rotation, relative to center of mass
		joint.frameA.q = MulRot( bodySimA.transform.q, @base.localFrameA.q );
		joint.frameA.p = RotateVector( bodySimA.transform.q, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		joint.frameB.q = MulRot( bodySimB.transform.q, @base.localFrameB.q );
		joint.frameB.p = RotateVector( bodySimB.transform.q, Sub( @base.localFrameB.p, bodySimB.localCenter ) );

		// Compute the initial center delta. Incremental position updates are relative to this.
		joint.deltaCenter = SubPos( bodySimB.center, bodySimA.center );

		joint.springSoftness = MakeSoft( joint.hertz, joint.dampingRatio, context.h );

		if ( context.enableWarmStarting == false )
		{
			joint.impulse = Vec2_zero;
			joint.springImpulse = 0.0f;
			joint.motorImpulse = 0.0f;
			joint.lowerImpulse = 0.0f;
			joint.upperImpulse = 0.0f;
		}
	}

	internal static void WarmStartPrismaticJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.prismaticJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref PrismaticJoint joint = ref @base.prismaticJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Vec2 d = Add( Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), Sub( rB, rA ) );

		Vec2 axisA = RotateVector( joint.frameA.q, new Vec2( 1.0f, 0.0f ) );
		axisA = RotateVector( stateA.deltaRotation, axisA );

		// impulse is applied at anchor point on body B
		float a1 = Cross( Add( rA, d ), axisA );
		float a2 = Cross( rB, axisA );
		float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

		// perpendicular constraint
		Vec2 perpA = LeftPerp( axisA );
		float s1 = Cross( Add( rA, d ), perpA );
		float s2 = Cross( rB, perpA );
		float perpImpulse = joint.impulse.x;
		float angleImpulse = joint.impulse.y;

		Vec2 P = Add( MulSV( axialImpulse, axisA ), MulSV( perpImpulse, perpA ) );
		float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
		float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

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

	internal static void SolvePrismaticJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.prismaticJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref PrismaticJoint joint = ref @base.prismaticJoint;
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

		Vec2 d = Add( Add( Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), Sub( rB, rA ) );

		Vec2 axisA = RotateVector( joint.frameA.q, new Vec2( 1.0f, 0.0f ) );
		axisA = RotateVector( stateA.deltaRotation, axisA );
		float translation = Dot( axisA, d );

		// These scalars are for torques generated by axial forces
		float a1 = Cross( Add( rA, d ), axisA );
		float a2 = Cross( rB, axisA );

		float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
		float axialMass = k > 0.0f ? 1.0f / k : 0.0f;

		Softness softness = @base.constraintSoftness;

		// spring constraint
		if ( joint.enableSpring )
		{
			// This is a real spring and should be applied even during relax
			float C = translation - joint.targetTranslation;
			float bias = joint.springSoftness.biasRate * C;
			float massScale = joint.springSoftness.massScale;
			float impulseScale = joint.springSoftness.impulseScale;

			float Cdot = Dot( axisA, Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			float deltaImpulse = -massScale * axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
			joint.springImpulse += deltaImpulse;

			Vec2 P = MulSV( deltaImpulse, axisA );
			float LA = deltaImpulse * a1;
			float LB = deltaImpulse * a2;

			vA = MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = MulAdd( vB, mB, P );
			wB += iB * LB;
		}

		// Solve motor constraint
		if ( joint.enableMotor )
		{
			float Cdot = Dot( axisA, Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			float impulse = axialMass * ( joint.motorSpeed - Cdot );
			float oldImpulse = joint.motorImpulse;
			float maxImpulse = context.h * joint.maxMotorForce;
			joint.motorImpulse = ClampFloat( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint.motorImpulse - oldImpulse;

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
			// Clamp the speculative distance to a reasonable value
			float speculativeDistance = 0.25f * ( joint.upperTranslation - joint.lowerTranslation );

			// Lower limit
			{
				float C = translation - joint.lowerTranslation;

				if ( C < speculativeDistance )
				{
					float bias = 0.0f;
					float massScale = 1.0f;
					float impulseScale = 0.0f;

					if ( C > 0.0f )
					{
						// speculation
						float safe = GetLengthUnitsPerMeter();
						bias = MinFloat( C, safe ) * context.inv_h;
					}
					else if ( useBias )
					{
						bias = softness.biasRate * C;
						massScale = softness.massScale;
						impulseScale = softness.impulseScale;
					}

					float oldImpulse = joint.lowerImpulse;
					float Cdot = Dot( axisA, Sub( vB, vA ) ) + a2 * wB - a1 * wA;
					float deltaImpulse = -axialMass * massScale * ( Cdot + bias ) - impulseScale * oldImpulse;
					joint.lowerImpulse = MaxFloat( oldImpulse + deltaImpulse, 0.0f );
					deltaImpulse = joint.lowerImpulse - oldImpulse;

					Vec2 P = MulSV( deltaImpulse, axisA );
					float LA = deltaImpulse * a1;
					float LB = deltaImpulse * a2;

					vA = MulSub( vA, mA, P );
					wA -= iA * LA;
					vB = MulAdd( vB, mB, P );
					wB += iB * LB;
				}
				else
				{
					joint.lowerImpulse = 0.0f;
				}
			}

			// Upper limit
			// Note: signs are flipped to keep C positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				// sign flipped
				float C = joint.upperTranslation - translation;

				if ( C < speculativeDistance )
				{
					float bias = 0.0f;
					float massScale = 1.0f;
					float impulseScale = 0.0f;

					if ( C > 0.0f )
					{
						// speculation
						float safe = GetLengthUnitsPerMeter();
						bias = MinFloat( C, safe ) * context.inv_h;
					}
					else if ( useBias )
					{
						bias = softness.biasRate * C;
						massScale = softness.massScale;
						impulseScale = softness.impulseScale;
					}

					float oldImpulse = joint.upperImpulse;

					// sign flipped
					float Cdot = Dot( axisA, Sub( vA, vB ) ) + a1 * wA - a2 * wB;
					float deltaImpulse = -axialMass * massScale * ( Cdot + bias ) - impulseScale * oldImpulse;
					joint.upperImpulse = MaxFloat( oldImpulse + deltaImpulse, 0.0f );
					deltaImpulse = joint.upperImpulse - oldImpulse;

					Vec2 P = MulSV( deltaImpulse, axisA );
					float LA = deltaImpulse * a1;
					float LB = deltaImpulse * a2;

					// sign flipped
					vA = MulAdd( vA, mA, P );
					wA += iA * LA;
					vB = MulSub( vB, mB, P );
					wB -= iB * LB;
				}
				else
				{
					joint.upperImpulse = 0.0f;
				}
			}
		}

		// Solve the prismatic constraint in block form
		{
			Vec2 perpA = LeftPerp( axisA );

			// These scalars are for torques generated by the perpendicular constraint force
			float s1 = Cross( Add( d, rA ), perpA );
			float s2 = Cross( rB, perpA );

			Vec2 Cdot = default;
			Cdot.x = Dot( perpA, Sub( vB, vA ) ) + s2 * wB - s1 * wA;
			Cdot.y = wB - wA;

			Vec2 bias = Vec2_zero;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				Vec2 C = default;
				C.x = Dot( perpA, d );
				C.y = Rot_GetAngle( relQ );

				bias = MulSV( softness.biasRate, C );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
			float k12 = iA * s1 + iB * s2;
			float k22 = iA + iB;
			if ( k22 == 0.0f )
			{
				// For bodies with fixed rotation.
				k22 = 1.0f;
			}

			Mat22 K = new Mat22 { cx = new( k11, k12 ), cy = new( k12, k22 ) };

			Vec2 b = Solve22( K, Add( Cdot, bias ) );
			Vec2 deltaImpulse = default;
			deltaImpulse.x = -massScale * b.x - impulseScale * joint.impulse.x;
			deltaImpulse.y = -massScale * b.y - impulseScale * joint.impulse.y;

			joint.impulse.x += deltaImpulse.x;
			joint.impulse.y += deltaImpulse.y;

			Vec2 P = MulSV( deltaImpulse.x, perpA );
			float LA = deltaImpulse.x * s1 + deltaImpulse.y;
			float LB = deltaImpulse.x * s2 + deltaImpulse.y;

			vA = MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = MulAdd( vB, mB, P );
			wB += iB * LB;
		}

		B2_ASSERT( IsValidVec2( vA ) );
		B2_ASSERT( IsValidFloat( wA ) );
		B2_ASSERT( IsValidVec2( vB ) );
		B2_ASSERT( IsValidFloat( wB ) );

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

	internal static void DrawPrismaticJoint( ref DebugDraw draw, ref JointSim @base, WorldTransform transformA, WorldTransform transformB, float drawScale )
	{
		B2_ASSERT( @base.type == JointType.prismaticJoint );

		ref PrismaticJoint joint = ref @base.prismaticJoint;
		WorldTransform frameA = MulWorldTransforms( transformA, @base.localFrameA );
		WorldTransform frameB = MulWorldTransforms( transformB, @base.localFrameB );
		Vec2 axisA = RotateVector( frameA.q, new Vec2( 1.0f, 0.0f ) );

		draw.DrawLineFcn( frameA.p, frameB.p, HexColor.colorDimGray, draw.context );

		if ( joint.enableLimit )
		{
			float b = 0.25f * drawScale;
			Pos lower = OffsetPos( frameA.p, MulSV( joint.lowerTranslation, axisA ) );
			Pos upper = OffsetPos( frameA.p, MulSV( joint.upperTranslation, axisA ) );
			Vec2 perp = LeftPerp( axisA );
			draw.DrawLineFcn( lower, upper, HexColor.colorGray, draw.context );
			draw.DrawLineFcn( OffsetPos( lower, MulSV( -b, perp ) ), OffsetPos( lower, MulSV( b, perp ) ), HexColor.colorGreen,
							   draw.context );
			draw.DrawLineFcn( OffsetPos( upper, MulSV( -b, perp ) ), OffsetPos( upper, MulSV( b, perp ) ), HexColor.colorRed,
							   draw.context );
		}
		else
		{
			draw.DrawLineFcn( OffsetPos( frameA.p, Neg( axisA ) ), OffsetPos( frameA.p, axisA ), HexColor.colorGray,
							   draw.context );
		}

		if ( joint.enableSpring )
		{
			Pos p = OffsetPos( frameA.p, MulSV( joint.targetTranslation, axisA ) );
			draw.DrawPointFcn( p, 8.0f, HexColor.colorViolet, draw.context );
		}

		draw.DrawPointFcn( frameA.p, 5.0f, HexColor.colorGray, draw.context );
		draw.DrawPointFcn( frameB.p, 5.0f, HexColor.colorBlue, draw.context );
	}

}
