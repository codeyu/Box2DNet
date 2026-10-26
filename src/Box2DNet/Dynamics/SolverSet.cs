// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	internal static void DestroySolverSet( World world, int setIndex )
	{
		ref SolverSet set = ref world.solverSets[setIndex];
		set.bodySims.Destroy();
		set.bodyStates.Destroy();
		set.contactSims.Destroy();
		set.jointSims.Destroy();
		set.islandSims.Destroy();
		FreeId( ref world.solverSetIdPool, setIndex );
		set = default(SolverSet);
		set.setIndex = NullIndex;
	}

	// Wake a solver set. Does not merge islands.
	// Contacts can be in several places:
	// 1. non-touching contacts in the disabled set
	// 2. non-touching contacts already in the awake set
	// 3. touching contacts in the sleeping set
	// This handles contact types 1 and 3. Type 2 doesn't need any action.
	internal static void WakeSolverSet( World world, int setIndex )
	{
		B2_ASSERT( setIndex >= b2_firstSleepingSet );
		ref SolverSet set = ref world.solverSets[setIndex];
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		ref SolverSet disabledSet = ref world.solverSets[b2_disabledSet];
		Span<Body> bodies = world.bodies.data;
		int bodyCount = set.bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			ref BodySim simSrc = ref set.bodySims.data[i];
			ref Body body = ref bodies[simSrc.bodyId];
			B2_ASSERT( body.setIndex == setIndex );
			body.setIndex = b2_awakeSet;
			body.localIndex = awakeSet.bodySims.count;

			// Reset sleep timer
			body.sleepTime = 0.0f;

			ref BodySim simDst = ref awakeSet.bodySims.Emplace();
			simDst = simSrc;

			ref BodyState state = ref awakeSet.bodyStates.Emplace();
			state = identityBodyState;
			state.flags = body.flags;

			// Move non-touching contacts from disabled set to awake set.
			int encodedBodySimIndex = body.localIndex;
			int contactKey = body.headContactKey;
			while ( contactKey != NullIndex )
			{
				int edgeIndex = contactKey & 1;
				int contactId = contactKey >> 1;

				ref Contact contact = ref world.contacts[contactId];
				contactKey = contact.edges[edgeIndex].nextKey;

				if ( contact.setIndex != b2_disabledSet )
				{
					B2_ASSERT( contact.setIndex == b2_awakeSet || contact.setIndex == setIndex );

					// If this contact is awake but not touching I need to fixup the body sim index here.
					if ( contact.setIndex == b2_awakeSet )
					{
						B2_ASSERT( contact.colorIndex == NullIndex );
						ref ContactSim awakeContactSimInner = ref awakeSet.contactSims[contact.localIndex];
						if ( edgeIndex == 0 )
						{
							awakeContactSimInner.encodedBodySimA = encodedBodySimIndex;
						}
						else
						{
							awakeContactSimInner.encodedBodySimB = encodedBodySimIndex;
						}
					}

					continue;
				}

				int localIndex = contact.localIndex;
				ref ContactSim contactSim = ref disabledSet.contactSims[localIndex];
				B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) == 0 && contactSim.manifold.pointCount == 0 );

				contact.setIndex = b2_awakeSet;
				contact.localIndex = awakeSet.contactSims.count;
				ref ContactSim awakeContactSim = ref awakeSet.contactSims.Emplace();
				awakeContactSim = contactSim;

				if ( edgeIndex == 0 )
				{
					awakeContactSim.encodedBodySimA = encodedBodySimIndex;
				}
				else
				{
					awakeContactSim.encodedBodySimB = encodedBodySimIndex;
				}

				int movedLocalIndex = disabledSet.contactSims.RemoveSwap( localIndex );
				if ( movedLocalIndex != NullIndex )
				{
					// fix moved element
					ref ContactSim movedContactSim = ref disabledSet.contactSims.data[localIndex];
					ref Contact movedContact = ref world.contacts[movedContactSim.contactId];
					B2_ASSERT( movedContact.localIndex == movedLocalIndex );
					movedContact.localIndex = localIndex;
				}
			}
		}

		// transfer touching contacts from sleeping set to contact graph
		{
			int contactCount = set.contactSims.count;
			for ( int i = 0; i < contactCount; ++i )
			{
				ref ContactSim contactSim = ref set.contactSims.data[i];
				ref Contact contact = ref world.contacts[contactSim.contactId];
				B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) != 0 );
				B2_ASSERT( ( contactSim.simFlags & b2_simTouchingFlag ) != 0 );
				B2_ASSERT( contactSim.manifold.pointCount > 0 );
				B2_ASSERT( contact.setIndex == setIndex );
				AddContactToGraph( world, ref contactSim, ref contact );
				contact.setIndex = b2_awakeSet;
			}
		}

		// transfer joints from sleeping set to awake set
		{
			int jointCount = set.jointSims.count;
			for ( int i = 0; i < jointCount; ++i )
			{
				ref JointSim jointSim = ref set.jointSims.data[i];
				ref Joint joint = ref world.joints[jointSim.jointId];
				B2_ASSERT( joint.setIndex == setIndex );
				AddJointToGraph( world, ref jointSim, ref joint );
				joint.setIndex = b2_awakeSet;
			}
		}

		// transfer island from sleeping set to awake set
		// Usually a sleeping set has only one island, but it is possible
		// that joints are created between sleeping islands and they
		// are moved to the same sleeping set.
		{
			int islandCount = set.islandSims.count;
			for ( int i = 0; i < islandCount; ++i )
			{
				ref IslandSim islandSrc = ref set.islandSims.data[i];
				ref Island island = ref world.islands[islandSrc.islandId];
				island.setIndex = b2_awakeSet;
				island.localIndex = awakeSet.islandSims.count;
				ref IslandSim islandDst = ref awakeSet.islandSims.Emplace();
				islandDst = islandSrc;
			}
		}

		// destroy the sleeping set
		DestroySolverSet( world, setIndex );
	}

	// Islands need to have a deterministic order because data is moved to a sleeping set according
	// to island order.
	internal static void TrySleepIsland( World world, int islandId )
	{
		ref Island island = ref world.islands[islandId];
		B2_ASSERT( island.setIndex == b2_awakeSet );

		// Cannot put an island to sleep while it has a pending split and more than one body.
		if ( island.constraintRemoveCount > 0 && island.bodies.count > 1 )
		{
			return;
		}

		// island is sleeping
		// - create new sleeping solver set
		// - move island to sleeping solver set
		// - identify non-touching contacts that should move to sleeping solver set or disabled set
		// - remove old island
		// - fix island
		int sleepSetId = AllocId( ref world.solverSetIdPool );
		if ( sleepSetId == world.solverSets.count )
		{
			SolverSet set = default;
			set.setIndex = NullIndex;
			world.solverSets.Push( set );
		}

		ref SolverSet sleepSet = ref world.solverSets[sleepSetId];
		sleepSet = default(SolverSet);

		// grab awake set after creating the sleep set because the solver set array may have been resized
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		B2_ASSERT( 0 <= island.localIndex && island.localIndex < awakeSet.islandSims.count );

		sleepSet.setIndex = sleepSetId;
		sleepSet.bodySims.CreateN( island.bodies.count );
		sleepSet.contactSims.CreateN( island.contacts.count );
		sleepSet.jointSims.CreateN( island.joints.count );

		// move awake bodies to sleeping set
		// this shuffles around bodies in the awake set
		{
			ref SolverSet disabledSet = ref world.solverSets[b2_disabledSet];
			for ( int i = 0; i < island.bodies.count; ++i )
			{
				int bodyId = island.bodies.data[i];
				ref Body body = ref world.bodies[bodyId];
				B2_ASSERT( body.setIndex == b2_awakeSet );
				B2_ASSERT( body.islandId == islandId );
				B2_ASSERT( body.islandIndex == i );

				body.flags &= ~b2_bodyTransientFlags;

				// Update the body move event to indicate this body fell asleep
				// It could happen the body is forced asleep before it ever moves.
				if ( body.bodyMoveIndex != NullIndex )
				{
					ref BodyMoveEvent moveEvent = ref world.bodyMoveEvents[body.bodyMoveIndex];
					B2_ASSERT( moveEvent.bodyId.index1 - 1 == bodyId );
					B2_ASSERT( moveEvent.bodyId.generation == body.generation );
					moveEvent.fellAsleep = true;
					body.bodyMoveIndex = NullIndex;
				}

				int awakeBodyIndex = body.localIndex;
				ref BodySim awakeSim = ref awakeSet.bodySims[awakeBodyIndex];
				awakeSim.flags &= ~b2_bodyTransientFlags;

				// move body sim to sleep set
				int sleepBodyIndex = sleepSet.bodySims.count;
				ref BodySim sleepBodySim = ref sleepSet.bodySims.Emplace();
				sleepBodySim = awakeSim;

				ref Body movedBody = ref RemoveBodySim( world, ref awakeSet, awakeBodyIndex );
				if ( Unsafe.IsNullRef( ref movedBody ) == false && movedBody.islandId != islandId )
				{
					// Update the contact sims for the moved body.
					RefreshBodyContactIndices( world, ref movedBody );
				}

				// Destroy state, no need to clone.
				awakeSet.bodyStates.RemoveSwap( awakeBodyIndex );

				body.setIndex = sleepSetId;
				body.localIndex = sleepBodyIndex;

				// Move non-touching contacts to the disabled set.
				// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
				int contactKey = body.headContactKey;
				while ( contactKey != NullIndex )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;

					ref Contact contact = ref world.contacts[contactId];
					B2_ASSERT( contact.setIndex == b2_awakeSet );

					contactKey = contact.edges[edgeIndex].nextKey;

					if ( contact.colorIndex != NullIndex )
					{
						// contact is touching and will be moved separately
						B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) != 0 );
						continue;
					}

					int localIndex = contact.localIndex;
					ref ContactSim contactSim = ref awakeSet.contactSims[localIndex];
					if ( edgeIndex == 0 )
					{
						contactSim.encodedBodySimA = NullIndex;
					}
					else
					{
						contactSim.encodedBodySimB = NullIndex;
					}

					// the other body may still be awake, it still may go to sleep and then it will be responsible
					// for moving this contact to the disabled set.
					int otherEdgeIndex = edgeIndex ^ 1;
					int otherBodyId = contact.edges[otherEdgeIndex].bodyId;
					ref Body otherBody = ref world.bodies[otherBodyId];
					if ( otherBody.setIndex == b2_awakeSet )
					{
						continue;
					}

					B2_ASSERT( contactSim.manifold.pointCount == 0 );
					B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) == 0 );

					// move the non-touching contact to the disabled set
					contact.setIndex = b2_disabledSet;
					contact.localIndex = disabledSet.contactSims.count;
					ref ContactSim disabledContactSim = ref disabledSet.contactSims.Emplace();
					disabledContactSim = contactSim;

					int movedLocalIndex = awakeSet.contactSims.RemoveSwap( localIndex );
					if ( movedLocalIndex != NullIndex )
					{
						// fix moved element
						ref ContactSim movedContactSim = ref awakeSet.contactSims.data[localIndex];
						ref Contact movedContact = ref world.contacts[movedContactSim.contactId];
						B2_ASSERT( movedContact.localIndex == movedLocalIndex );
						movedContact.localIndex = localIndex;
					}
				}
			}
		}

		// move touching contacts
		// this shuffles contacts in the awake set
		{
			for ( int i = 0; i < island.contacts.count; ++i )
			{
				ref ContactLink link = ref island.contacts.data[i];
				ref Contact contact = ref world.contacts[link.contactId];
				B2_ASSERT( contact.setIndex == b2_awakeSet );
				B2_ASSERT( contact.islandId == islandId );
				int colorIndex = contact.colorIndex;
				B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );

				ref GraphColor color = ref world.constraintGraph.colors[colorIndex];
				// Remove bodies from graph coloring associated with this constraint
				if ( colorIndex != OverflowIndex )
				{
					// might clear a bit for a static body, but this has no effect
					ClearBit( ref color.bodySet, contact.edges[0].bodyId );
					ClearBit( ref color.bodySet, contact.edges[1].bodyId );
				}

				int localIndex = contact.localIndex;
				ref ContactSim awakeContactSim = ref color.contactSims[localIndex];
				int sleepContactIndex = sleepSet.contactSims.count;
				ref ContactSim sleepContactSim = ref sleepSet.contactSims.Emplace();
				sleepContactSim = awakeContactSim;

				sleepContactSim.encodedBodySimA = SleepBodySimIndex( sleepContactSim.encodedBodySimA );
				sleepContactSim.encodedBodySimB = SleepBodySimIndex( sleepContactSim.encodedBodySimB );

				int movedLocalIndex = color.contactSims.RemoveSwap( localIndex );
				if ( movedLocalIndex != NullIndex )
				{
					// fix moved element
					ref ContactSim movedContactSim = ref color.contactSims.data[localIndex];
					ref Contact movedContact = ref world.contacts[movedContactSim.contactId];
					B2_ASSERT( movedContact.localIndex == movedLocalIndex );
					movedContact.localIndex = localIndex;
				}

				contact.setIndex = sleepSetId;
				contact.colorIndex = NullIndex;
				contact.localIndex = sleepContactIndex;
			}
		}

		// move joints
		// this shuffles joints in the awake set
		{
			for ( int i = 0; i < island.joints.count; ++i )
			{
				ref JointLink link = ref island.joints.data[i];
				ref Joint joint = ref world.joints[link.jointId];
				B2_ASSERT( joint.setIndex == b2_awakeSet );
				B2_ASSERT( joint.islandId == islandId );
				int colorIndex = joint.colorIndex;
				int localIndex = joint.localIndex;

				B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );

				ref GraphColor color = ref world.constraintGraph.colors[colorIndex];
				ref JointSim awakeJointSim = ref color.jointSims[localIndex];
				if ( colorIndex != OverflowIndex )
				{
					// might clear a bit for a static body, but this has no effect
					ClearBit( ref color.bodySet, joint.edges[0].bodyId );
					ClearBit( ref color.bodySet, joint.edges[1].bodyId );
				}

				int sleepJointIndex = sleepSet.jointSims.count;
				ref JointSim sleepJointSim = ref sleepSet.jointSims.Emplace();
				sleepJointSim = awakeJointSim;

				int movedIndex = color.jointSims.RemoveSwap( localIndex );
				if ( movedIndex != NullIndex )
				{
					// fix moved element
					ref JointSim movedJointSim = ref color.jointSims.data[localIndex];
					int movedId = movedJointSim.jointId;
					ref Joint movedJoint = ref world.joints[movedId];
					B2_ASSERT( movedJoint.localIndex == movedIndex );
					movedJoint.localIndex = localIndex;
				}

				joint.setIndex = sleepSetId;
				joint.colorIndex = NullIndex;
				joint.localIndex = sleepJointIndex;
			}
		}

		// move island struct
		{
			B2_ASSERT( island.setIndex == b2_awakeSet );

			int islandIndex = island.localIndex;
			ref IslandSim sleepIsland = ref sleepSet.islandSims.Emplace();
			sleepIsland.islandId = islandId;

			int movedIslandIndex = awakeSet.islandSims.RemoveSwap( islandIndex );
			if ( movedIslandIndex != NullIndex )
			{
				// fix index on moved element
				ref IslandSim movedIslandSim = ref awakeSet.islandSims.data[islandIndex];
				int movedIslandId = movedIslandSim.islandId;
				ref Island movedIsland = ref world.islands[movedIslandId];
				B2_ASSERT( movedIsland.localIndex == movedIslandIndex );
				movedIsland.localIndex = islandIndex;
			}

			island.setIndex = sleepSetId;
			island.localIndex = 0;
		}

		if ( world.splitIslandId == islandId )
		{
			world.splitIslandId = NullIndex;
		}

		ValidateSolverSets( world );
	}

	// This is called when joints are created between sets. I want to allow the sets
	// to continue sleeping if both are asleep. Otherwise one set is waked.
	// Islands will get merge when the set is waked.
	internal static void MergeSolverSets( World world, int setId1, int setId2 )
	{
		B2_ASSERT( setId1 >= b2_firstSleepingSet );
		B2_ASSERT( setId2 >= b2_firstSleepingSet );
		ref SolverSet set1 = ref world.solverSets[setId1];
		ref SolverSet set2 = ref world.solverSets[setId2];
		// Move the fewest number of bodies
		if ( set1.bodySims.count < set2.bodySims.count )
		{
			ref SolverSet tempSet = ref set1;
			set1 = ref set2;
			set2 = ref tempSet;

			int tempId = setId1;
			setId1 = setId2;
			setId2 = tempId;
		}

		// transfer bodies
		{
			Span<Body> bodies = world.bodies.data;
			int bodyCount = set2.bodySims.count;
			for ( int i = 0; i < bodyCount; ++i )
			{
				ref BodySim simSrc = ref set2.bodySims.data[i];
				ref Body body = ref bodies[simSrc.bodyId];
				B2_ASSERT( body.setIndex == setId2 );
				body.setIndex = setId1;
				body.localIndex = set1.bodySims.count;

				ref BodySim simDst = ref set1.bodySims.Emplace();
				simDst = simSrc;
			}
		}

		// transfer contacts
		{
			int contactCount = set2.contactSims.count;
			for ( int i = 0; i < contactCount; ++i )
			{
				ref ContactSim contactSrc = ref set2.contactSims.data[i];
				ref Contact contact = ref world.contacts[contactSrc.contactId];
				B2_ASSERT( contact.setIndex == setId2 );
				contact.setIndex = setId1;
				contact.localIndex = set1.contactSims.count;

				ref ContactSim contactDst = ref set1.contactSims.Emplace();
				contactDst = contactSrc;
			}
		}

		// transfer joints
		{
			int jointCount = set2.jointSims.count;
			for ( int i = 0; i < jointCount; ++i )
			{
				ref JointSim jointSrc = ref set2.jointSims.data[i];
				ref Joint joint = ref world.joints[jointSrc.jointId];
				B2_ASSERT( joint.setIndex == setId2 );
				joint.setIndex = setId1;
				joint.localIndex = set1.jointSims.count;

				ref JointSim jointDst = ref set1.jointSims.Emplace();
				jointDst = jointSrc;
			}
		}

		// transfer islands
		{
			int islandCount = set2.islandSims.count;
			for ( int i = 0; i < islandCount; ++i )
			{
				ref IslandSim islandSrc = ref set2.islandSims.data[i];
				int islandId = islandSrc.islandId;

				ref Island island = ref world.islands[islandId];
				island.setIndex = setId1;
				island.localIndex = set1.islandSims.count;

				ref IslandSim islandDst = ref set1.islandSims.Emplace();
				islandDst = islandSrc;
			}
		}

		// destroy the merged set
		DestroySolverSet( world, setId2 );

		ValidateSolverSets( world );
	}

	// Remove a body from a solver set and return the body swapped into the slot.
	internal static ref Body RemoveBodySim( World world, ref SolverSet set, int localIndex )
	{
		ref B2Array<BodySim> bodySims = ref set.bodySims;
		B2_ASSERT( 0 <= localIndex && localIndex < bodySims.count );
		int lastIndex = bodySims.count - 1;
		bodySims.count -= 1;

		if ( localIndex == lastIndex )
		{
			return ref Unsafe.NullRef<Body>();
		}

		bodySims.data[localIndex] = bodySims.data[lastIndex];
		ref Body movedBody = ref world.bodies[bodySims.data[localIndex].bodyId];
		B2_ASSERT( movedBody.localIndex == lastIndex );
		movedBody.localIndex = localIndex;

		return ref movedBody;
	}

	internal static void TransferBody( World world, ref SolverSet targetSet, ref SolverSet sourceSet, ref Body body )
	{
		B2_ASSERT( body.headContactKey == NullIndex );

		if ( Unsafe.AreSame( ref targetSet, ref sourceSet ) )
		{
			return;
		}

		// The use cases for transfer body (set body type, disable, etc) destroy all the contacts
		// first. Double check here because this function wouldn't work correctly otherwise.
		B2_ASSERT( body.headContactKey == NullIndex );

		int sourceIndex = body.localIndex;
		ref BodySim sourceSim = ref sourceSet.bodySims[sourceIndex];
		int targetIndex = targetSet.bodySims.count;
		ref BodySim targetSim = ref targetSet.bodySims.Emplace();
		targetSim = sourceSim;

		// Clear transient body flags
		body.flags &= ~b2_bodyTransientFlags;
		targetSim.flags &= ~( b2_isFast | b2_bodyTransientFlags );

		// Remove body sim from solver set that owns it
		ref Body movedBody = ref RemoveBodySim( world, ref sourceSet, sourceIndex );
		if ( Unsafe.IsNullRef( ref movedBody ) == false )
		{
			// Fixup the moved body's contact sims.
			RefreshBodyContactIndices( world, ref movedBody );
		}

		if ( sourceSet.setIndex == b2_awakeSet )
		{
			sourceSet.bodyStates.RemoveSwap( sourceIndex );
		}
		else if ( targetSet.setIndex == b2_awakeSet )
		{
			ref BodyState state = ref targetSet.bodyStates.Emplace();
			state = identityBodyState;
			state.flags = body.flags;
		}

		body.setIndex = targetSet.setIndex;
		body.localIndex = targetIndex;
	}

	internal static void TransferJoint( World world, ref SolverSet targetSet, ref SolverSet sourceSet, ref Joint joint )
	{
		if ( Unsafe.AreSame( ref targetSet, ref sourceSet ) )
		{
			return;
		}

		int localIndex = joint.localIndex;
		int colorIndex = joint.colorIndex;

		// Retrieve source.
		scoped ref JointSim sourceSim = ref Unsafe.NullRef<JointSim>();
		if ( sourceSet.setIndex == b2_awakeSet )
		{
			B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );
			ref GraphColor color = ref world.constraintGraph.colors[colorIndex];
			sourceSim = ref color.jointSims[localIndex];
		}
		else
		{
			B2_ASSERT( colorIndex == NullIndex );
			sourceSim = ref sourceSet.jointSims[localIndex];
		}

		// Create target and copy. Fix joint.
		if ( targetSet.setIndex == b2_awakeSet )
		{
			AddJointToGraph( world, ref sourceSim, ref joint );
			joint.setIndex = b2_awakeSet;
		}
		else
		{
			joint.setIndex = targetSet.setIndex;
			joint.localIndex = targetSet.jointSims.count;
			joint.colorIndex = NullIndex;

			ref JointSim targetSim = ref targetSet.jointSims.Emplace();
			targetSim = sourceSim;
		}

		// Destroy source.
		if ( sourceSet.setIndex == b2_awakeSet )
		{
			RemoveJointFromGraph( world, joint.edges[0].bodyId, joint.edges[1].bodyId, colorIndex, localIndex );
		}
		else
		{
			int movedIndex = sourceSet.jointSims.RemoveSwap( localIndex );
			if ( movedIndex != NullIndex )
			{
				// fix swapped element
				ref JointSim movedJointSim = ref sourceSet.jointSims.data[localIndex];
				int movedId = movedJointSim.jointId;
				ref Joint movedJoint = ref world.joints[movedId];
				movedJoint.localIndex = localIndex;
			}
		}
	}

}
