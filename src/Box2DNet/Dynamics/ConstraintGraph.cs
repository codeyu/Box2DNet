// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// Solver using graph coloring. Islands are only used for sleep.
	// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
	// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

	// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
	// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
	// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
	// cause horrible cache stalls. To make this feasible I would need a way to block these writes.
	// todo should be possible to branch on the scatters to avoid writing to kinematic bodies

	// This is used for debugging by making all constraints be assigned to overflow.
	// Don't expect the same result for overflow and graph colored. They have a different solve order.
	private const int B2_FORCE_OVERFLOW = 0;
	private static int GetLinearShare( int total, int rampCount, int rampIndex )
	{
		long weight = rampCount - rampIndex;
		long weightSum = (long)rampCount * ( rampCount + 1 ) / 2;
		return (int)( total * weight / weightSum );
	}

	internal static void CreateGraph( ConstraintGraph graph, in Capacity capacity )
	{

		graph.colors = default;

		int bodyCapacity = MaxInt( capacity.staticBodyCount + capacity.dynamicBodyCount, 16 );

		int staticContactCount = capacity.contactCount / 8;
		int dynamicContactCount = capacity.contactCount - staticContactCount;
		int dynamicRampCount = DynamicColorCount / 2;
		int staticRampCount = GraphColorCount - DynamicColorCount;

		// Initialize graph color bit set.
		// No bitset for overflow color.
		for ( int i = 0; i < OverflowIndex; ++i )
		{
			ref GraphColor color = ref graph.colors[i];
			color.bodySet = CreateBitSet( (uint)bodyCapacity );
			SetBitCountAndClear( ref color.bodySet, bodyCapacity );

			int contactCapacity = 0;
			if ( i < dynamicRampCount )
			{
				contactCapacity += GetLinearShare( dynamicContactCount, dynamicRampCount, i );
			}

			int staticRampIndex = OverflowIndex - 1 - i;
			if ( staticRampIndex < staticRampCount )
			{
				contactCapacity += GetLinearShare( staticContactCount, staticRampCount, staticRampIndex );
			}

			color.contactSims.Reserve( MaxInt( 16, contactCapacity ) );
		}
	}

	internal static void DestroyGraph( ConstraintGraph graph )
	{
		for ( int i = 0; i < GraphColorCount; ++i )
		{
			ref GraphColor color = ref graph.colors[i];
			// The bit set should never be used on the overflow color
			B2_ASSERT( i != OverflowIndex || color.bodySet.bits == null );

			DestroyBitSet( ref color.bodySet );
			color.contactSims.Destroy();
			color.jointSims.Destroy();
		}
	}

	// Contacts are always created as non-touching. They get moved into the constraint
	// graph once they are found to be touching.
	internal static void AddContactToGraph( World world, ref ContactSim contactSim, ref Contact contact )
	{
		B2_ASSERT( contactSim.manifold.pointCount > 0 );
		B2_ASSERT( ( contactSim.simFlags & b2_simTouchingFlag ) != 0 );
		B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) != 0 );

		ConstraintGraph graph = world.constraintGraph;
		int colorIndex = OverflowIndex;

		int bodyIdA = contact.edges[0].bodyId;
		int bodyIdB = contact.edges[1].bodyId;
		ref Body bodyA = ref world.bodies[bodyIdA];
		ref Body bodyB = ref world.bodies[bodyIdB];
		BodyType typeA = bodyA.type;
		BodyType typeB = bodyB.type;
		B2_ASSERT( typeA == BodyType.dynamicBody || typeB == BodyType.dynamicBody );

#if true
		if ( typeA == BodyType.dynamicBody && typeB == BodyType.dynamicBody )
		{
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for ( int i = 0; i < DynamicColorCount; ++i )
			{
				ref GraphColor colorInner = ref graph.colors[i];
				if ( GetBit( ref colorInner.bodySet, bodyIdA ) || GetBit( ref colorInner.bodySet, bodyIdB ) )
				{
					continue;
				}

				SetBitGrow( ref colorInner.bodySet, bodyIdA );
				SetBitGrow( ref colorInner.bodySet, bodyIdB );
				colorIndex = i;
				break;
			}
		}
		else if ( typeA == BodyType.dynamicBody )
		{
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for ( int i = OverflowIndex - 1; i >= 1; --i )
			{
				ref GraphColor colorInner = ref graph.colors[i];
				if ( GetBit( ref colorInner.bodySet, bodyIdA ) )
				{
					continue;
				}

				SetBitGrow( ref colorInner.bodySet, bodyIdA );
				colorIndex = i;
				break;
			}
		}
		else if ( typeB == BodyType.dynamicBody )
		{
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for ( int i = OverflowIndex - 1; i >= 1; --i )
			{
				ref GraphColor colorInner = ref graph.colors[i];
				if ( GetBit( ref colorInner.bodySet, bodyIdB ) )
				{
					continue;
				}

				SetBitGrow( ref colorInner.bodySet, bodyIdB );
				colorIndex = i;
				break;
			}
		}
#endif

		ref GraphColor color = ref graph.colors[colorIndex];
		contact.colorIndex = colorIndex;
		contact.localIndex = color.contactSims.count;

		ref ContactSim newContact = ref color.contactSims.Emplace();
		newContact = contactSim;

		// todo perhaps skip this if the contact is already awake

		if ( typeA == BodyType.staticBody )
		{
			newContact.encodedBodySimA = EncodeBodySimIndex( bodyA );
			newContact.invMassA = 0.0f;
			newContact.invIA = 0.0f;
		}
		else
		{
			B2_ASSERT( bodyA.setIndex == b2_awakeSet );
			ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
			int localIndex = bodyA.localIndex;
			newContact.encodedBodySimA = localIndex;

			ref BodySim bodySimA = ref awakeSet.bodySims[localIndex];
			newContact.invMassA = bodySimA.invMass;
			newContact.invIA = bodySimA.invInertia;
		}

		if ( typeB == BodyType.staticBody )
		{
			newContact.encodedBodySimB = EncodeBodySimIndex( bodyB );
			newContact.invMassB = 0.0f;
			newContact.invIB = 0.0f;
		}
		else
		{
			B2_ASSERT( bodyB.setIndex == b2_awakeSet );
			ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
			int localIndex = bodyB.localIndex;
			newContact.encodedBodySimB = localIndex;

			ref BodySim bodySimB = ref awakeSet.bodySims[localIndex];
			newContact.invMassB = bodySimB.invMass;
			newContact.invIB = bodySimB.invInertia;
		}
	}

	internal static void RemoveContactFromGraph( World world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
	{
		ConstraintGraph graph = world.constraintGraph;
		B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );
		ref GraphColor color = ref graph.colors[colorIndex];
		if ( colorIndex != OverflowIndex )
		{
			// This might clear a bit for a kinematic or static body, but this has no effect
			ClearBit( ref color.bodySet, bodyIdA );
			ClearBit( ref color.bodySet, bodyIdB );
		}

		int movedIndex = color.contactSims.RemoveSwap( localIndex );
		if ( movedIndex != NullIndex )
		{
			// Fix index on swapped contact
			ref ContactSim movedContactSim = ref color.contactSims.data[localIndex];
			// Fix moved contact
			int movedId = movedContactSim.contactId;
			ref Contact movedContact = ref world.contacts[movedId];
			B2_ASSERT( movedContact.setIndex == b2_awakeSet );
			B2_ASSERT( movedContact.colorIndex == colorIndex );
			B2_ASSERT( movedContact.localIndex == movedIndex );
			movedContact.localIndex = localIndex;
		}
	}

	// Notice that a joint cannot share the same color as a contact between the same two bodies. This means I can solve contacts and
	// joints in parallel with each other within each color.
	private static int AssignJointColor( ConstraintGraph graph, int bodyIdA, int bodyIdB, BodyType typeA, BodyType typeB )
	{
		B2_ASSERT( typeA == BodyType.dynamicBody || typeB == BodyType.dynamicBody );

#if true
		if ( typeA == BodyType.dynamicBody && typeB == BodyType.dynamicBody )
		{
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for ( int i = 0; i < DynamicColorCount; ++i )
			{
				ref GraphColor color = ref graph.colors[i];
				if ( GetBit( ref color.bodySet, bodyIdA ) || GetBit( ref color.bodySet, bodyIdB ) )
				{
					continue;
				}

				SetBitGrow( ref color.bodySet, bodyIdA );
				SetBitGrow( ref color.bodySet, bodyIdB );
				return i;
			}
		}
		else if ( typeA == BodyType.dynamicBody )
		{
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for ( int i = OverflowIndex - 1; i >= 1; --i )
			{
				ref GraphColor color = ref graph.colors[i];
				if ( GetBit( ref color.bodySet, bodyIdA ) )
				{
					continue;
				}

				SetBitGrow( ref color.bodySet, bodyIdA );
				return i;
			}
		}
		else if ( typeB == BodyType.dynamicBody )
		{
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for ( int i = OverflowIndex - 1; i >= 1; --i )
			{
				ref GraphColor color = ref graph.colors[i];
				if ( GetBit( ref color.bodySet, bodyIdB ) )
				{
					continue;
				}

				SetBitGrow( ref color.bodySet, bodyIdB );
				return i;
			}
		}
#else
#endif

		return OverflowIndex;
	}

	internal static ref JointSim CreateJointInGraph( World world, ref Joint joint )
	{
		ConstraintGraph graph = world.constraintGraph;
		int bodyIdA = joint.edges[0].bodyId;
		int bodyIdB = joint.edges[1].bodyId;
		ref Body bodyA = ref world.bodies[bodyIdA];
		ref Body bodyB = ref world.bodies[bodyIdB];
		int colorIndex = AssignJointColor( graph, bodyIdA, bodyIdB, bodyA.type, bodyB.type );

		ref JointSim jointSim = ref graph.colors[colorIndex].jointSims.Emplace();
		jointSim = default;

		joint.colorIndex = colorIndex;
		joint.localIndex = graph.colors[colorIndex].jointSims.count - 1;
		return ref jointSim;
	}

	internal static void AddJointToGraph( World world, ref JointSim jointSim, ref Joint joint )
	{
		ref JointSim jointDst = ref CreateJointInGraph( world, ref joint );
		jointDst = jointSim;
	}

	internal static void RemoveJointFromGraph( World world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
	{
		ConstraintGraph graph = world.constraintGraph;
		B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );
		ref GraphColor color = ref graph.colors[colorIndex];
		if ( colorIndex != OverflowIndex )
		{
			// May clear static bodies, no effect
			ClearBit( ref color.bodySet, bodyIdA );
			ClearBit( ref color.bodySet, bodyIdB );
		}

		int movedIndex = color.jointSims.RemoveSwap( localIndex );
		if ( movedIndex != NullIndex )
		{
			// Fix moved joint
			ref JointSim movedJointSim = ref color.jointSims.data[localIndex];
			int movedId = movedJointSim.jointId;
			ref Joint movedJoint = ref world.joints[movedId];
			B2_ASSERT( movedJoint.setIndex == b2_awakeSet );
			B2_ASSERT( movedJoint.colorIndex == colorIndex );
			B2_ASSERT( movedJoint.localIndex == movedIndex );
			movedJoint.localIndex = localIndex;
		}
	}

	private static readonly HexColor[] graphColors = { HexColor.colorRed,	   HexColor.colorOrange,		HexColor.colorYellow,			HexColor.colorLimeGreen,		 HexColor.colorSpringGreen,
		HexColor.colorAqua,	   HexColor.colorDodgerBlue,	HexColor.colorBlueViolet,		HexColor.colorMagenta,		 HexColor.colorDeepPink,
		HexColor.colorCrimson,   HexColor.colorCoral,		HexColor.colorGold,			HexColor.colorGreenYellow,	 HexColor.colorMediumSeaGreen,
		HexColor.colorTurquoise, HexColor.colorDeepSkyBlue, HexColor.colorCornflowerBlue, HexColor.colorMediumSlateBlue, HexColor.colorMediumOrchid,
		HexColor.colorHotPink,   HexColor.colorTomato,		HexColor.colorKhaki,			HexColor.colorSilver, };

	public static HexColor GetGraphColor( int index )
	{
		B2_ASSERT( 0 <= index && index < GraphColorCount );
		return graphColors[index];
	}

}
