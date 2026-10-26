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

	private const int B2_WELD_BLOCK_SOLVE = 0;
#if B2_WELD_BLOCK_SOLVE
	internal struct Vec3
	{
		public float x;
		public float y;
		public float z;
	}

	// A 3-by-3 matrix. Stored in column-major order.
	internal struct Mat33
	{
		public Vec3 cx;
		public Vec3 cy;
		public Vec3 cz;
	}

	internal static float Dot3( Vec3 a, Vec3 b )
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	internal static Vec3 Cross3( Vec3 a, Vec3 b )
	{
		return new Vec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
	}

	internal static Vec3 Solve33( in Mat33 m, Vec3 b )
	{
		float det = Dot3( m.cx, Cross3( m.cy, m.cz ) );
		if ( det != 0.0f )
		{
			det = 1.0f / det;
		}

		Vec3 x = default;
		x.x = det * Dot3( b, Cross3( m.cy, m.cz ) );
		x.y = det * Dot3( m.cx, Cross3( b, m.cz ) );
		x.z = det * Dot3( m.cx, Cross3( m.cy, b ) );
		return x;
	}
#endif

	public static void WeldJoint_SetLinearHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( IsValidFloat( hertz ) && hertz >= 0.0f );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		joint.weldJoint.linearHertz = hertz;
	}

	public static float WeldJoint_GetLinearHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		return joint.weldJoint.linearHertz;
	}

	public static void WeldJoint_SetLinearDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		joint.weldJoint.linearDampingRatio = dampingRatio;
	}

	public static float WeldJoint_GetLinearDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		return joint.weldJoint.linearDampingRatio;
	}

	public static void WeldJoint_SetAngularHertz( JointId jointId, float hertz )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( IsValidFloat( hertz ) && hertz >= 0.0f );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		joint.weldJoint.angularHertz = hertz;
	}

	public static float WeldJoint_GetAngularHertz( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		return joint.weldJoint.angularHertz;
	}

	public static void WeldJoint_SetAngularDampingRatio( JointId jointId, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		B2_ASSERT( IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		joint.weldJoint.angularDampingRatio = dampingRatio;
	}

	public static float WeldJoint_GetAngularDampingRatio( JointId jointId )
	{
		ref JointSim joint = ref GetJointSimCheckType( jointId, JointType.weldJoint );
		return joint.weldJoint.angularDampingRatio;
	}

	internal static Vec2 GetWeldJointForce( World world, ref JointSim @base )
	{
		Vec2 force = MulSV( world.inv_h, @base.weldJoint.linearImpulse );
		return force;
	}

	internal static float GetWeldJointTorque( World world, ref JointSim @base )
	{
		return world.inv_h * @base.weldJoint.angularImpulse;
	}

	// Point-to-point constraint
	// C = p2 - p1
	// Cdot = v2 - v1
	//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
	// J = [-E -r1_skew E r2_skew ]
	// Identity used:
	// w k % (rx i + ry j) = w * (-ry i + rx j)

	// Angle constraint
	// C = angle2 - angle1 - referenceAngle
	// Cdot = w2 - w1
	// J = [0 0 -1 0 0 1]
	// K = invI1 + invI2

	// 3x3 Block
	// K = [J1] * invM * [J1T J2T]
	//     [J2]
	//   = [J1] * [invM * J1T invM * J2T]
	//     [J2]
	//   = [J1 * invM * J1T J1 * invM * J2T]
	//     [J2 * invM * J1T J2 * invM * J2T]

	internal static void PrepareWeldJoint( ref JointSim @base, ref StepContext context )
	{
		B2_ASSERT( @base.type == JointType.weldJoint );

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

		ref WeldJoint joint = ref @base.weldJoint;
		joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : NullIndex;
		joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : NullIndex;

		// Compute joint anchor frames with world space rotation, relative to center of mass
		joint.frameA.q = MulRot( bodySimA.transform.q, @base.localFrameA.q );
		joint.frameA.p = RotateVector( bodySimA.transform.q, Sub( @base.localFrameA.p, bodySimA.localCenter ) );
		joint.frameB.q = MulRot( bodySimB.transform.q, @base.localFrameB.q );
		joint.frameB.p = RotateVector( bodySimB.transform.q, Sub( @base.localFrameB.p, bodySimB.localCenter ) );

		// Compute the initial center delta. Incremental position updates are relative to this.
		joint.deltaCenter = SubPos( bodySimB.center, bodySimA.center );

		float ka = iA + iB;
		joint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

		if ( joint.linearHertz == 0.0f )
		{
			joint.linearSpring = @base.constraintSoftness;
		}
		else
		{
			joint.linearSpring = MakeSoft( joint.linearHertz, joint.linearDampingRatio, context.h );
		}

		if ( joint.angularHertz == 0.0f )
		{
			joint.angularSpring = @base.constraintSoftness;
		}
		else
		{
			joint.angularSpring = MakeSoft( joint.angularHertz, joint.angularDampingRatio, context.h );
		}

		if ( context.enableWarmStarting == false )
		{
			joint.linearImpulse = Vec2_zero;
			joint.angularImpulse = 0.0f;
		}
	}

	internal static void WarmStartWeldJoint( ref JointSim @base, ref StepContext context )
	{
		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref WeldJoint joint = ref @base.weldJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		if ( ( stateA.flags & b2_dynamicFlag ) != 0 )
		{
			stateA.linearVelocity = MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
			stateA.angularVelocity -= iA * ( Cross( rA, joint.linearImpulse ) + joint.angularImpulse );
		}

		if ( ( stateB.flags & b2_dynamicFlag ) != 0 )
		{
			stateB.linearVelocity = MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
			stateB.angularVelocity += iB * ( Cross( rB, joint.linearImpulse ) + joint.angularImpulse );
		}
	}

	internal static void SolveWeldJoint( ref JointSim @base, ref StepContext context, bool useBias )
	{
		B2_ASSERT( @base.type == JointType.weldJoint );

		float mA = @base.invMassA;
		float mB = @base.invMassB;
		float iA = @base.invIA;
		float iB = @base.invIB;

		// dummy state for static bodies
		BodyState dummyState = identityBodyState;

		ref WeldJoint joint = ref @base.weldJoint;
		ref BodyState stateA = ref joint.indexA == NullIndex ? ref dummyState : ref context.states[joint.indexA];
		ref BodyState stateB = ref joint.indexB == NullIndex ? ref dummyState : ref context.states[joint.indexB];
		Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		// Block solve doesn't work correctly with mixed stiffness values
#if B2_WELD_BLOCK_SOLVE
		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
		Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
		Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

		Mat33 K = default;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cz.x = -rA.y * iA - rB.y * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		K.cz.y = rA.x * iA + rB.x * iB;
		K.cx.z = K.cz.x;
		K.cy.z = K.cz.y;
		K.cz.z = iA + iB;

		Vec3 bias = new( 0.0f, 0.0f, 0.0f );
		float linearMassScale = 1.0f;
		float linearImpulseScale = 0.0f;
		if ( useBias || joint.linearHertz > 0.0f )
		{
			// linear
			Vec2 dcA = stateA.deltaPosition;
			Vec2 dcB = stateB.deltaPosition;
			Vec2 jointTranslation = Add( Add( Sub( dcB, dcA ), Sub( rB, rA ) ), joint.deltaCenter );

			bias.x = joint.linearSpring.biasRate * jointTranslation.x;
			bias.y = joint.linearSpring.biasRate * jointTranslation.y;

			linearMassScale = joint.linearSpring.massScale;
			linearImpulseScale = joint.linearSpring.impulseScale;
		}

		float angularMassScale = 1.0f;
		float angularImpulseScale = 0.0f;
		if ( useBias || joint.angularHertz > 0.0f )
		{
			// angular
			Rot qA = MulRot( stateA.deltaRotation, joint.frameA.q );
			Rot qB = MulRot( stateB.deltaRotation, joint.frameB.q );
			Rot relQ = InvMulRot( qA, qB );
			float jointAngle = Rot_GetAngle( relQ );

			bias.z = joint.angularSpring.biasRate * jointAngle;

			angularMassScale = joint.angularSpring.massScale;
			angularImpulseScale = joint.angularSpring.impulseScale;
		}

		Vec2 Cdot1 = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
		float Cdot2 = wB - wA;

		Vec3 Cdot = new( Cdot1.x + bias.x, Cdot1.y + bias.y, Cdot2 + bias.z );

		Vec3 b = Solve33( ref K, Cdot );

		Vec2 linearImpulse = new Vec2( -linearMassScale * b.x - linearImpulseScale * joint.linearImpulse.x, -linearMassScale * b.y - linearImpulseScale * joint.linearImpulse.y );
		joint.linearImpulse = Add( joint.linearImpulse, linearImpulse );

		float angularImpulse = -angularMassScale * b.z - angularImpulseScale * joint.angularImpulse;
		joint.angularImpulse += angularImpulse;

		vA = MulSub( vA, mA, linearImpulse );
		wA -= iA * (Cross( rA, linearImpulse ) + angularImpulse);
		vB = MulAdd( vB, mB, linearImpulse );
		wB += iB * (Cross( rB, linearImpulse ) + angularImpulse);

		// todo debugging
		Cdot1 = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );
		Cdot2 = wB - wA;

		if ( useBias == false && Length(Cdot1) > 0.0001f )
		{
			Cdot1.x += 0.0f;
		}

		if ( useBias == false && AbsFloat( Cdot2 ) > 0.0001f )
		{
			Cdot2 += 0.0f;
		}

#else

		// angular constraint
		{
			Rot qA = MulRot( stateA.deltaRotation, joint.frameA.q );
			Rot qB = MulRot( stateB.deltaRotation, joint.frameB.q );
			Rot relQ = InvMulRot( qA, qB );
			float jointAngle = Rot_GetAngle( relQ );

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias || joint.angularHertz > 0.0f )
			{
				float C = jointAngle;
				bias = joint.angularSpring.biasRate * C;
				massScale = joint.angularSpring.massScale;
				impulseScale = joint.angularSpring.impulseScale;
			}

			float Cdot = wB - wA;
			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.angularImpulse;
			joint.angularImpulse += impulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// linear constraint
		{
			Vec2 rA = RotateVector( stateA.deltaRotation, joint.frameA.p );
			Vec2 rB = RotateVector( stateB.deltaRotation, joint.frameB.p );

			Vec2 bias = Vec2_zero;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias || joint.linearHertz > 0.0f )
			{
				Vec2 dcA = stateA.deltaPosition;
				Vec2 dcB = stateB.deltaPosition;
				Vec2 C = Add( Add( Sub( dcB, dcA ), Sub( rB, rA ) ), joint.deltaCenter );

				bias = MulSV( joint.linearSpring.biasRate, C );
				massScale = joint.linearSpring.massScale;
				impulseScale = joint.linearSpring.impulseScale;
			}

			Vec2 Cdot = Sub( Add( vB, CrossSV( wB, rB ) ), Add( vA, CrossSV( wA, rA ) ) );

			Mat22 K = default;
			K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
			K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
			K.cx.y = K.cy.x;
			K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
			Vec2 b = Solve22( K, Add( Cdot, bias ) );

			Vec2 impulse = new Vec2( -massScale * b.x - impulseScale * joint.linearImpulse.x, -massScale * b.y - impulseScale * joint.linearImpulse.y );

			joint.linearImpulse = Add( joint.linearImpulse, impulse );

			vA = MulSub( vA, mA, impulse );
			wA -= iA * Cross( rA, impulse );
			vB = MulAdd( vB, mB, impulse );
			wB += iB * Cross( rB, impulse );
		}
#endif

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

	internal static void DrawWeldJoint( ref DebugDraw draw, ref JointSim @base, WorldTransform transformA, WorldTransform transformB, float drawScale )
	{
		B2_ASSERT( @base.type == JointType.weldJoint );

		WorldTransform frameA = MulWorldTransforms( transformA, @base.localFrameA );
		WorldTransform frameB = MulWorldTransforms( transformB, @base.localFrameB );

		Polygon box = MakeBox( 0.25f * drawScale, 0.125f * drawScale );
		draw.DrawPolygonFcn( frameA, box.vertices, 4, HexColor.colorDarkOrange, draw.context );
		draw.DrawPolygonFcn( frameB, box.vertices, 4, HexColor.colorDarkCyan, draw.context );
	}

}
