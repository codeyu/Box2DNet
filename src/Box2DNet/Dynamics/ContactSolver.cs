// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// contact separation for sub-stepping
	// s = s0 + dot(cB + rB - cA - rA, normal)
	// normal is held constant
	// body positions c can translation and anchors r can rotate
	// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
	// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
	// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
	// s_base = s0 + dot(cB0 - cA0, normal)

	internal static void PrepareContacts_Overflow( ref StepContext context )
	{
		World world = context.world;
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		int contactCount = color.contactSims.count;
		Span<ContactSim> contacts = color.contactSims.data;
		Span<BodyState> states = context.states;
#if BOX2D_VALIDATE
		Span<Body> bodies = world.bodies.data;
#endif

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		Softness contactSoftness = context.contactSoftness;
		Softness staticSoftness = context.staticSoftness;

		float warmStartScale = world.enableWarmStarting ? 1.0f : 0.0f;
		bool anyRestitution = false;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref ContactSim contactSim = ref contacts[i];
			ref Manifold manifold = ref contactSim.manifold;
			int pointCount = manifold.pointCount;

			B2_ASSERT( 0 < pointCount && pointCount <= 2 );

			int indexA = DecodeAwakeIndex( contactSim.encodedBodySimA );
			int indexB = DecodeAwakeIndex( contactSim.encodedBodySimB );

#if BOX2D_VALIDATE
			ref Body bodyA = ref bodies[contactSim.bodyIdA];
			B2_ASSERT( contactSim.encodedBodySimA == EncodeBodySimIndex( bodyA ) );

			ref Body bodyB = ref bodies[contactSim.bodyIdB];
			B2_ASSERT( contactSim.encodedBodySimB == EncodeBodySimIndex( bodyB ) );
#endif

			ref ContactConstraint constraint = ref constraints[i];
			// 0 is null
			constraint.indexA = indexA + 1;
			constraint.indexB = indexB + 1;
			constraint.normal = manifold.normal;
			constraint.friction = contactSim.friction;
			constraint.restitution = contactSim.restitution;
			constraint.rollingResistance = contactSim.rollingResistance;
			constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
			constraint.tangentSpeed = contactSim.tangentSpeed;
			constraint.pointCount = pointCount;

			bool haveRestitution = contactSim.restitution > 0.0f;
			bool hitEvents = ( contactSim.simFlags & b2_simEnableHitEvent ) != 0;
			bool sampleVelocity = haveRestitution || hitEvents;
			anyRestitution = anyRestitution || haveRestitution;

			Vec2 vA = Vec2_zero;
			float wA = 0.0f;
			Vec2 vB = Vec2_zero;
			float wB = 0.0f;

			if ( sampleVelocity )
			{
				if ( indexA != NullIndex )
				{
					vA = states[indexA].linearVelocity;
					wA = states[indexA].angularVelocity;
				}

				if ( indexB != NullIndex )
				{
					vB = states[indexB].linearVelocity;
					wB = states[indexB].angularVelocity;
				}
			}

			float mA = contactSim.invMassA;
			float iA = contactSim.invIA;
			float mB = contactSim.invMassB;
			float iB = contactSim.invIB;

			if ( indexA == NullIndex || indexB == NullIndex )
			{
				constraint.softness = staticSoftness;
			}
			else
			{
				constraint.softness = contactSoftness;
			}

			// copy mass into constraint to avoid cache misses during sub-stepping
			constraint.invMassA = mA;
			constraint.invIA = iA;
			constraint.invMassB = mB;
			constraint.invIB = iB;

			{
				float k = iA + iB;
				constraint.rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
			}

			Vec2 normal = constraint.normal;
			Vec2 tangent = RightPerp( constraint.normal );

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ManifoldPoint mp = ref manifold.points[j];
				ref ContactConstraintPoint cp = ref constraint.points[j];
				cp.normalImpulse = warmStartScale * mp.normalImpulse;
				cp.tangentImpulse = warmStartScale * mp.tangentImpulse;
				cp.totalNormalImpulse = 0.0f;
				cp.restitutionImpulse = 0.0f;

				Vec2 rA = mp.anchorA;
				Vec2 rB = mp.anchorB;

				cp.anchorA = rA;
				cp.anchorB = rB;
				cp.baseSeparation = mp.separation - Dot( Sub( rB, rA ), normal );

				float rnA = Cross( rA, normal );
				float rnB = Cross( rB, normal );
				float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
				cp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

				float rtA = Cross( rA, tangent );
				float rtB = Cross( rB, tangent );
				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
				cp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

				// Only compute the normal velocity terms if needed.
				if ( sampleVelocity )
				{
					Vec2 vrA = Add( vA, CrossSV( wA, rA ) );
					Vec2 vrB = Add( vB, CrossSV( wB, rB ) );
					float vn = Dot( normal, Sub( vrB, vrA ) );

					cp.relativeVelocity = vn;
					mp.normalVelocity = vn;
				}
				else
				{
					cp.relativeVelocity = 0.0f;
					mp.normalVelocity = 0.0f;
				}
			}
		}

		if ( anyRestitution )
		{
			// Check if set first? Store per task context and OR?
			AtomicStoreInt( ref context.anyRestitution, 1 );
		}
	}

	internal static void WarmStartContacts_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		int contactCount = color.contactSims.count;
		World world = context.world;
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		Span<BodyState> states = awakeSet.bodyStates.data;
		// This is a dummy state to represent a static body because static bodies don't have a solver body.
		BodyState dummyState = identityBodyState;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref ContactConstraint constraint = ref constraints[i];
			int indexA = constraint.indexA - 1;
			int indexB = constraint.indexB - 1;

			ref BodyState stateA = ref indexA == NullIndex ? ref dummyState : ref states[indexA];
			ref BodyState stateB = ref indexB == NullIndex ? ref dummyState : ref states[indexB];
			Vec2 vA = stateA.linearVelocity;
			float wA = stateA.angularVelocity;
			Vec2 vB = stateB.linearVelocity;
			float wB = stateB.angularVelocity;

			float mA = constraint.invMassA;
			float iA = constraint.invIA;
			float mB = constraint.invMassB;
			float iB = constraint.invIB;

			// Stiffer for static contacts to avoid bodies getting pushed through the ground
			Vec2 normal = constraint.normal;
			Vec2 tangent = RightPerp( constraint.normal );
			int pointCount = constraint.pointCount;

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				// fixed anchors
				Vec2 rA = cp.anchorA;
				Vec2 rB = cp.anchorB;

				Vec2 P = Add( MulSV( cp.normalImpulse, normal ), MulSV( cp.tangentImpulse, tangent ) );

				cp.totalNormalImpulse += cp.normalImpulse;

				wA -= iA * Cross( rA, P );
				vA = MulAdd( vA, -mA, P );
				wB += iB * Cross( rB, P );
				vB = MulAdd( vB, mB, P );
			}

			wA -= iA * constraint.rollingImpulse;
			wB += iB * constraint.rollingImpulse;

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

	// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
	internal static void PushContacts_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		int contactCount = color.contactSims.count;
		World world = context.world;
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		Span<BodyState> states = awakeSet.bodyStates.data;
		float inv_h = context.inv_h;
		float contactSpeed = context.world.contactSpeed;

		// This is a dummy body to represent a static body since static bodies don't have a solver body.
		BodyState dummyState = identityBodyState;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref ContactConstraint constraint = ref constraints[i];
			float mA = constraint.invMassA;
			float iA = constraint.invIA;
			float mB = constraint.invMassB;
			float iB = constraint.invIB;

			int indexA = constraint.indexA - 1;
			int indexB = constraint.indexB - 1;

			ref BodyState stateA = ref indexA == NullIndex ? ref dummyState : ref states[indexA];
			Vec2 vA = stateA.linearVelocity;
			float wA = stateA.angularVelocity;
			Rot dqA = stateA.deltaRotation;

			ref BodyState stateB = ref indexB == NullIndex ? ref dummyState : ref states[indexB];
			Vec2 vB = stateB.linearVelocity;
			float wB = stateB.angularVelocity;
			Rot dqB = stateB.deltaRotation;

			Vec2 dp = Sub( stateB.deltaPosition, stateA.deltaPosition );

			Vec2 normal = constraint.normal;
			Softness softness = constraint.softness;
			int pointCount = constraint.pointCount;

			float vnc = Dot( Sub( vB, vA ), normal );

			FixedArray2<float> rnA = default;

			FixedArray2<float> rnB = default;
			FixedArray2<float> impulses = default;
			float normalImpulse = 0.0f;
			float angularA = 0.0f;
			float angularB = 0.0f;

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				// fixed anchor points
				Vec2 rA = cp.anchorA;
				Vec2 rB = cp.anchorB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				Vec2 ds = Add( dp, Sub( RotateVector( dqB, rB ), RotateVector( dqA, rA ) ) );
				float s = cp.baseSeparation + Dot( ds, normal );

				float velocityBias;
				float massScale;
				float impulseScale;
				if ( s > 0.0f )
				{
					// speculative bias is positive
					velocityBias = s * inv_h;
					massScale = 1.0f;
					impulseScale = 0.0f;
				}
				else
				{
					// overlap bias is negative
					velocityBias = MaxFloat( softness.massScale * softness.biasRate * s, -contactSpeed );
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				rnA[j] = Cross( rA, normal );
				rnB[j] = Cross( rB, normal );

				// Relative normal velocity at contact.
				float vn = vnc + wB * rnB[j] - wA * rnA[j];
				for ( int k = 0; k < j; ++k )
				{
					// Point 2 only depends on the impulse from point 1.
					vn += ( mA + mB + iA * rnA[k] * rnA[j] + iB * rnB[k] * rnB[j] ) * impulses[k];
				}

				float impulse = MaxFloat( -cp.normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp.normalImpulse,
											-cp.normalImpulse );
				cp.normalImpulse += impulse;
				cp.totalNormalImpulse += impulse;

				impulses[j] = impulse;

				// These accumulators are not used until the loop is done.
				normalImpulse += impulse;
				angularA += impulse * rnA[j];
				angularB += impulse * rnB[j];
			}

			Vec2 P = MulSV( normalImpulse, normal );
			vA = MulSub( vA, mA, P );
			wA -= iA * angularA;
			vB = MulAdd( vB, mB, P );
			wB += iB * angularB;

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

	// Solve contacts: normal, friction and rolling resistance.
	internal static void SolveContacts_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		int contactCount = color.contactSims.count;
		World world = context.world;
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		Span<BodyState> states = awakeSet.bodyStates.data;
		float inv_h = context.inv_h;

		// This is a dummy body to represent a static body since static bodies don't have a solver body.
		BodyState dummyState = identityBodyState;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref ContactConstraint constraint = ref constraints[i];
			float mA = constraint.invMassA;
			float iA = constraint.invIA;
			float mB = constraint.invMassB;
			float iB = constraint.invIB;

			int indexA = constraint.indexA - 1;
			int indexB = constraint.indexB - 1;

			ref BodyState stateA = ref indexA == NullIndex ? ref dummyState : ref states[indexA];
			Vec2 vA = stateA.linearVelocity;
			float wA = stateA.angularVelocity;
			Rot dqA = stateA.deltaRotation;

			ref BodyState stateB = ref indexB == NullIndex ? ref dummyState : ref states[indexB];
			Vec2 vB = stateB.linearVelocity;
			float wB = stateB.angularVelocity;
			Rot dqB = stateB.deltaRotation;

			Vec2 dp = Sub( stateB.deltaPosition, stateA.deltaPosition );

			Vec2 normal = constraint.normal;
			Vec2 tangent = RightPerp( normal );
			float friction = constraint.friction;

			int pointCount = constraint.pointCount;
			float totalNormalImpulse = 0.0f;

			Vec2 dv = Sub( vB, vA );
			float vnc = Dot( dv, normal );

			// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			//    = dot(vrB - vrA, tangent) - (sA + sB)
			float vtc = Dot( dv, tangent ) - constraint.tangentSpeed;

			FixedArray2<float> normalImpulses = default;
			FixedArray2<float> tangentImpulses = default;

			// Cache cross products
			FixedArray2<float> rnA = default;
			FixedArray2<float> rnB = default;
			FixedArray2<float> rtA = default;
			FixedArray2<float> rtB = default;
			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				rnA[j] = Cross( cp.anchorA, normal );
				rnB[j] = Cross( cp.anchorB, normal );
				rtA[j] = Cross( cp.anchorA, tangent );
				rtB[j] = Cross( cp.anchorB, tangent );
			}

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				// Fixed anchor points.
				Vec2 rA = cp.anchorA;
				Vec2 rB = cp.anchorB;

				// Compute current separation.
				// This is subject to round-off error if the anchor is far from the body center of mass.
				Vec2 ds = Add( dp, Sub( RotateVector( dqB, rB ), RotateVector( dqA, rA ) ) );
				float s = cp.baseSeparation + Dot( ds, normal );

				// Speculative bias, zero when overlapped.
				float velocityBias = s > 0.0f ? s * inv_h : 0.0f;

				// Relative normal velocity at contact.
				float vn = vnc + wB * rnB[j] - wA * rnA[j];
				for ( int k = 0; k < j; ++k )
				{
					// This allows getting point 2 normal velocity with only one dependency.
					vn += ( mA + mB + iA * rnA[k] * rnA[j] + iB * rnB[k] * rnB[j] ) * normalImpulses[k];
				}

				float impulse = MaxFloat( -cp.normalMass * ( vn + velocityBias ), -cp.normalImpulse );
				cp.normalImpulse += impulse;
				cp.totalNormalImpulse += impulse;
				normalImpulses[j] = impulse;

				// This is used for rolling resistance
				totalNormalImpulse += cp.normalImpulse;
			}

			// Rolling resistance
			float rollingImpulse;
			{
				float Cdot = wB - wA;
				for ( int j = 0; j < pointCount; ++j )
				{
					Cdot += ( iA * rnA[j] + iB * rnB[j] ) * normalImpulses[j];
				}

				float lambda = constraint.rollingImpulse;
				float maxLambda = constraint.rollingResistance * totalNormalImpulse;
				constraint.rollingImpulse = ClampFloat( lambda - constraint.rollingMass * Cdot, -maxLambda, maxLambda );
				rollingImpulse = constraint.rollingImpulse - lambda;
			}

			// Friction
			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				// relative tangent velocity at contact
				float vt = vtc + wB * rtB[j] - wA * rtA[j] + ( iA * rtA[j] + iB * rtB[j] ) * rollingImpulse;
				for ( int k = 0; k < pointCount; ++k )
				{
					vt += ( iA * rtA[j] * rnA[k] + iB * rtB[j] * rnB[k] ) * normalImpulses[k];
				}

				for ( int k = 0; k < j; ++k )
				{
					vt += ( mA + mB + iA * rtA[k] * rtA[j] + iB * rtB[k] * rtB[j] ) * tangentImpulses[k];
				}

				// clamp the accumulated force
				float maxFriction = friction * cp.normalImpulse;
				float impulse = ClampFloat( -cp.tangentMass * vt, -maxFriction - cp.tangentImpulse, maxFriction - cp.tangentImpulse );
				cp.tangentImpulse += impulse;
				tangentImpulses[j] = impulse;
			}

			float normalImpulse = 0.0f;
			float tangentImpulse = 0.0f;
			float angularA = rollingImpulse;
			float angularB = rollingImpulse;
			for ( int j = 0; j < pointCount; ++j )
			{
				normalImpulse += normalImpulses[j];
				tangentImpulse += tangentImpulses[j];
				angularA += normalImpulses[j] * rnA[j] + tangentImpulses[j] * rtA[j];
				angularB += normalImpulses[j] * rnB[j] + tangentImpulses[j] * rtB[j];
			}

			Vec2 P = Add( MulSV( normalImpulse, normal ), MulSV( tangentImpulse, tangent ) );
			vA = MulSub( vA, mA, P );
			wA -= iA * angularA;
			vB = MulAdd( vB, mB, P );
			wB += iB * angularB;

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

	internal static void ApplyRestitution_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		int contactCount = color.contactSims.count;
		World world = context.world;
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		Span<BodyState> states = awakeSet.bodyStates.data;
		float threshold = world.restitutionThreshold;
		float inv_h = context.inv_h;
		bool propagate = world.enableRestitutionPropagation;

		BodyState dummyState = identityBodyState;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref ContactConstraint constraint = ref constraints[i];
			if ( propagate == false && constraint.restitution == 0.0f )
			{
				continue;
			}

			float mA = constraint.invMassA;
			float iA = constraint.invIA;
			float mB = constraint.invMassB;
			float iB = constraint.invIB;

			int indexA = constraint.indexA - 1;
			int indexB = constraint.indexB - 1;

			ref BodyState stateA = ref indexA == NullIndex ? ref dummyState : ref states[indexA];
			Vec2 vA = stateA.linearVelocity;
			float wA = stateA.angularVelocity;
			Rot dqA = stateA.deltaRotation;

			ref BodyState stateB = ref indexB == NullIndex ? ref dummyState : ref states[indexB];
			Vec2 vB = stateB.linearVelocity;
			float wB = stateB.angularVelocity;
			Rot dqB = stateB.deltaRotation;

			Vec2 dp = Sub( stateB.deltaPosition, stateA.deltaPosition );

			Vec2 normal = constraint.normal;
			float restitution = constraint.restitution;
			int pointCount = constraint.pointCount;

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ContactConstraintPoint cp = ref constraint.points[j];
				Vec2 rA = cp.anchorA;
				Vec2 rB = cp.anchorB;

				// The total normal impulse is 0 for speculative points.
				float compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse;
				bool armed = restitution > 0.0f && cp.relativeVelocity < -threshold && compressionImpulse > 0.0f;

				float velocityBias;
				if ( armed )
				{
					velocityBias = restitution * cp.relativeVelocity;
				}
				else
				{
					Vec2 ds = Add( dp, Sub( RotateVector( dqB, rB ), RotateVector( dqA, rA ) ) );
					float s = cp.baseSeparation + Dot( ds, normal );

					velocityBias = s > 0.0f ? s * inv_h : 0.0f;
				}

				Vec2 vrA = Add( vA, CrossSV( wA, rA ) );
				Vec2 vrB = Add( vB, CrossSV( wB, rB ) );
				float vn = Dot( Sub( vrB, vrA ), normal );

				float impulse = -cp.normalMass * ( vn + velocityBias );

				float newImpulse = MaxFloat( cp.normalImpulse + impulse, 0.0f );
				impulse = newImpulse - cp.normalImpulse;

				float approachImpulse = MinFloat( MaxFloat( -cp.normalMass * vn, 0.0f ), MaxFloat( impulse, 0.0f ) );

				if ( armed )
				{
					// Poisson kinetic restitution guarantees no energy gain.
					float allowance = restitution * ( compressionImpulse + approachImpulse ) - cp.restitutionImpulse;
					impulse = MinFloat( impulse, approachImpulse + MaxFloat( allowance, 0.0f ) );
				}

				cp.normalImpulse += impulse;
				cp.restitutionImpulse += impulse - approachImpulse;
				cp.totalNormalImpulse += impulse;

				Vec2 P = MulSV( impulse, normal );
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
	}

	internal static void StoreImpulses_Overflow( ref StepContext context )
	{
		World world = context.world;
		ConstraintGraph graph = context.graph;
		ref GraphColor color = ref graph.colors[OverflowIndex];
		Span<ContactConstraint> constraints = color.overflowConstraints;
		Span<ContactSim> contactSims = color.contactSims.data;
		ref TaskContext taskContext = ref world.taskContexts.data[0];
		ref BitSet hitEventBitSet = ref taskContext.hitEventBitSet;
		int contactCount = color.contactSims.count;
		float negHitThreshold = -world.hitEventThreshold;
		bool hasHitEvents = taskContext.hasHitEvents;

		for ( int i = 0; i < contactCount; ++i )
		{
			ref readonly ContactConstraint constraint = ref constraints[i];
			ref ContactSim contactSim = ref contactSims[i];
			ref Manifold manifold = ref contactSim.manifold;
			int pointCount = manifold.pointCount;

			for ( int j = 0; j < pointCount; ++j )
			{
				ref ManifoldPoint mp = ref manifold.points[j];
				mp.normalImpulse = constraint.points[j].normalImpulse;
				mp.tangentImpulse = constraint.points[j].tangentImpulse;
				mp.totalNormalImpulse = constraint.points[j].totalNormalImpulse;
			}

			if ( ( contactSim.simFlags & b2_simEnableHitEvent ) != 0 )
			{
				for ( int j = 0; j < contactSim.manifold.pointCount; ++j )
				{
					ref ManifoldPoint mp = ref manifold.points[j];
					// Need to check total impulse because the point may be speculative and not colliding
					if ( mp.normalVelocity < negHitThreshold && mp.totalNormalImpulse > 0.0f )
					{
						SetBit( ref hitEventBitSet, contactSim.contactId );
						hasHitEvents = true;
						break;
					}
				}
			}

			manifold.rollingImpulse = constraint.rollingImpulse;
		}

		taskContext.hasHitEvents = hasHitEvents;
	}

	internal static void PrepareContacts_Wide( SolverBlock block, ref StepContext context )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			PrepareContacts_WideW8( block, context );
			return;
		}
#endif

		PrepareContacts_WideW4( block, ref context );
	}

	internal static void WarmStartContacts_Wide( SolverBlock block, ref StepContext context )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			WarmStartContacts_WideW8( block, context );
			return;
		}
#endif

		WarmStartContacts_WideW4( block, ref context );
	}

	internal static void PushContacts_Wide( SolverBlock block, ref StepContext context )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			PushContacts_WideW8( block, context );
			return;
		}
#endif

		PushContacts_WideW4( block, ref context );
	}

	internal static void SolveContacts_Wide( SolverBlock block, ref StepContext context )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			SolveContacts_WideW8( block, context );
			return;
		}
#endif

		SolveContacts_WideW4( block, ref context );
	}

	internal static void ApplyRestitution_Wide( SolverBlock block, ref StepContext context )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			ApplyRestitution_WideW8( block, context );
			return;
		}
#endif

		ApplyRestitution_WideW4( block, ref context );
	}

	internal static void StoreImpulses_Wide( SolverBlock block, ref StepContext context, int workerIndex )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( context.world.simdWidth == 8 )
		{
			StoreImpulses_WideW8( block, context, workerIndex );
			return;
		}
#endif

		StoreImpulses_WideW4( block, ref context, workerIndex );
	}

	internal static int GetWideContactConstraintByteCount( int simdWidth )
	{
#if B2_SIMD_HAS_WIDTH_8
		if ( simdWidth == 8 )
		{
			return GetWideContactConstraintByteCountW8();
		}
#else
#endif

		return GetWideContactConstraintByteCountW4();
	}

}
