// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	internal static ref Island CreateIsland( World world, int setIndex )
	{
		B2_ASSERT( setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet );

		int islandId = AllocId( ref world.islandIdPool );

		if ( islandId == world.islands.count )
		{
			Island emptyIsland = default;
			world.islands.Push( emptyIsland );
		}
		else
		{
			B2_ASSERT( world.islands.data[islandId].setIndex == NullIndex );
		}

		ref SolverSet set = ref world.solverSets[setIndex];
		ref Island island = ref world.islands[islandId];
		island.setIndex = setIndex;
		island.localIndex = set.islandSims.count;
		island.islandId = islandId;
		island.bodies = default;
		island.contacts = default;
		island.joints = default;
		island.constraintRemoveCount = 0;

		ref IslandSim islandSim = ref set.islandSims.Emplace();
		islandSim.islandId = islandId;

		return ref island;
	}

	internal static void DestroyIsland( World world, int islandId )
	{
		if ( world.splitIslandId == islandId )
		{
			world.splitIslandId = NullIndex;
		}

		// assume island is empty
		ref Island island = ref world.islands[islandId];
		ref SolverSet set = ref world.solverSets[island.setIndex];
		{
			int localIndex = island.localIndex;
			int lastIndex = set.islandSims.count - 1;
			B2_ASSERT( 0 <= localIndex && localIndex <= lastIndex );
			int moveIslandId = set.islandSims.data[lastIndex].islandId;
			set.islandSims.data[localIndex] = set.islandSims.data[lastIndex];
			world.islands.data[moveIslandId].localIndex = localIndex;
			set.islandSims.count -= 1;
		}

		// Free island and id (preserve island revision)
		island.bodies.Destroy();
		island.contacts.Destroy();
		island.joints.Destroy();
		island.constraintRemoveCount = 0;
		island.localIndex = NullIndex;
		island.islandId = NullIndex;
		island.setIndex = NullIndex;

		B2_VALIDATE( island.localIndex == NullIndex );

		FreeId( ref world.islandIdPool, islandId );
	}

	private static int MergeIslands( World world, int islandIdA, int islandIdB )
	{
		if ( islandIdA == islandIdB )
		{
			return islandIdA;
		}

		if ( islandIdA == NullIndex )
		{
			B2_ASSERT( islandIdB != NullIndex );
			return islandIdB;
		}

		if ( islandIdB == NullIndex )
		{
			B2_ASSERT( islandIdA != NullIndex );
			return islandIdA;
		}

		scoped ref Island smallIsland = ref Unsafe.NullRef<Island>();
		scoped ref Island bigIsland = ref Unsafe.NullRef<Island>();
		{
			ref Island islandA = ref world.islands[islandIdA];
			ref Island islandB = ref world.islands[islandIdB];
			// Keep the biggest island to reduce cache misses
			if ( islandA.bodies.count >= islandB.bodies.count )
			{
				bigIsland = ref islandA;
				smallIsland = ref islandB;
			}
			else
			{
				bigIsland = ref islandB;
				smallIsland = ref islandA;
			}
		}

		int bigIslandId = bigIsland.islandId;
		bigIsland.bodies.Grow( bigIsland.bodies.count + smallIsland.bodies.count );

		// Move bodies from smaller island to larger island
		for ( int i = 0; i < smallIsland.bodies.count; ++i )
		{
			int bodyId = smallIsland.bodies.data[i];
			ref Body body = ref world.bodies[bodyId];
			B2_VALIDATE( body.islandId == smallIsland.islandId );
			body.islandId = bigIslandId;
			body.islandIndex = bigIsland.bodies.count;
			bigIsland.bodies.Push( bodyId );
		}

		// Migrate contacts from smaller island to larger island
		if ( smallIsland.contacts.count > 0 )
		{
			bigIsland.contacts.Grow( bigIsland.contacts.count + smallIsland.contacts.count );

			for ( int i = 0; i < smallIsland.contacts.count; ++i )
			{
				ref ContactLink link = ref smallIsland.contacts.data[i];
				ref Contact contact = ref world.contacts[link.contactId];
				contact.islandId = bigIslandId;
				contact.islandIndex = bigIsland.contacts.count;
				bigIsland.contacts.Push( link );
			}
		}

		// Migrate joints from smaller island to larger island
		if ( smallIsland.joints.count > 0 )
		{
			bigIsland.joints.Grow( bigIsland.joints.count + smallIsland.joints.count );

			for ( int i = 0; i < smallIsland.joints.count; ++i )
			{
				ref JointLink link = ref smallIsland.joints.data[i];
				ref Joint joint = ref world.joints[link.jointId];
				joint.islandId = bigIslandId;
				joint.islandIndex = bigIsland.joints.count;
				bigIsland.joints.Push( link );
			}
		}

		// Track removed constraints
		bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount;

		DestroyIsland( world, smallIsland.islandId );

		ValidateIsland( world, bigIslandId );

		return bigIslandId;
	}

	private static void AddContactToIsland( World world, int islandId, ref Contact contact )
	{
		B2_ASSERT( contact.islandId == NullIndex );
		B2_ASSERT( contact.islandIndex == NullIndex );

		ref Island island = ref world.islands[islandId];
		contact.islandId = islandId;
		contact.islandIndex = island.contacts.count;

		ContactLink link = default;
		link.contactId = contact.contactId;
		link.bodyIdA = contact.edges[0].bodyId;
		link.bodyIdB = contact.edges[1].bodyId;
		island.contacts.Push( link );

		ValidateIsland( world, islandId );
	}

	// Link a contact into an island.
	internal static void LinkContact( World world, ref Contact contact )
	{
		B2_ASSERT( ( contact.flags & b2_contactTouchingFlag ) != 0 );

		int bodyIdA = contact.edges[0].bodyId;
		int bodyIdB = contact.edges[1].bodyId;

		ref Body bodyA = ref world.bodies[bodyIdA];
		ref Body bodyB = ref world.bodies[bodyIdB];
		B2_ASSERT( bodyA.setIndex != b2_disabledSet && bodyB.setIndex != b2_disabledSet );
		B2_ASSERT( bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet );

		// Wake bodyB if bodyA is awake and bodyB is sleeping
		if ( bodyA.setIndex == b2_awakeSet && bodyB.setIndex >= b2_firstSleepingSet )
		{
			WakeSolverSet( world, bodyB.setIndex );
		}

		// Wake bodyA if bodyB is awake and bodyA is sleeping
		if ( bodyB.setIndex == b2_awakeSet && bodyA.setIndex >= b2_firstSleepingSet )
		{
			WakeSolverSet( world, bodyA.setIndex );
		}

		int islandIdA = bodyA.islandId;
		int islandIdB = bodyB.islandId;

		// Static bodies have null island indices.
		B2_ASSERT( bodyA.setIndex != b2_staticSet || islandIdA == NullIndex );
		B2_ASSERT( bodyB.setIndex != b2_staticSet || islandIdB == NullIndex );
		B2_ASSERT( islandIdA != NullIndex || islandIdB != NullIndex );

		// Merge islands. This will destroy one of the islands.
		int finalIslandId = MergeIslands( world, islandIdA, islandIdB );

		// Add contact to the island that survived
		AddContactToIsland( world, finalIslandId, ref contact );
	}

	// This is called when a contact no longer has contact points or when a contact is destroyed.
	internal static void UnlinkContact( World world, ref Contact contact )
	{
		B2_ASSERT( contact.islandId != NullIndex );

		// remove from island
		int islandId = contact.islandId;
		ref Island island = ref world.islands[islandId];
		int removeIndex = contact.islandIndex;
		B2_ASSERT( 0 <= removeIndex && removeIndex < island.contacts.count );
		B2_ASSERT( island.contacts.data[removeIndex].contactId == contact.contactId );

		int movedIndex = island.contacts.RemoveSwap( removeIndex );
		if ( movedIndex != NullIndex )
		{
			// Fix islandIndex on the contact that was swapped into removeIndex
			ref ContactLink movedLink = ref island.contacts.data[removeIndex];
			ref Contact movedContact = ref world.contacts[movedLink.contactId];
			B2_ASSERT( movedContact.islandIndex == movedIndex );
			movedContact.islandIndex = removeIndex;
		}

		contact.islandId = NullIndex;
		contact.islandIndex = NullIndex;
		island.constraintRemoveCount += 1;

		ValidateIsland( world, islandId );
	}

	private static void AddJointToIsland( World world, int islandId, ref Joint joint )
	{
		B2_ASSERT( joint.islandId == NullIndex );
		B2_ASSERT( joint.islandIndex == NullIndex );

		ref Island island = ref world.islands[islandId];
		joint.islandId = islandId;
		joint.islandIndex = island.joints.count;

		JointLink link = default;
		link.jointId = joint.jointId;
		link.bodyIdA = joint.edges[0].bodyId;
		link.bodyIdB = joint.edges[1].bodyId;
		island.joints.Push( link );

		ValidateIsland( world, islandId );
	}

	internal static void LinkJoint( World world, ref Joint joint )
	{
		ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
		ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
		B2_ASSERT( bodyA.type == BodyType.dynamicBody || bodyB.type == BodyType.dynamicBody );

		if ( bodyA.setIndex == b2_awakeSet && bodyB.setIndex >= b2_firstSleepingSet )
		{
			WakeSolverSet( world, bodyB.setIndex );
		}
		else if ( bodyB.setIndex == b2_awakeSet && bodyA.setIndex >= b2_firstSleepingSet )
		{
			WakeSolverSet( world, bodyA.setIndex );
		}

		int islandIdA = bodyA.islandId;
		int islandIdB = bodyB.islandId;

		B2_ASSERT( islandIdA != NullIndex || islandIdB != NullIndex );

		// Merge islands. This will destroy one of the islands.
		int finalIslandId = MergeIslands( world, islandIdA, islandIdB );

		// Add joint the island that survived
		AddJointToIsland( world, finalIslandId, ref joint );
	}

	internal static void UnlinkJoint( World world, ref Joint joint )
	{
		if ( joint.islandId == NullIndex )
		{
			return;
		}

		// remove from island
		int islandId = joint.islandId;
		ref Island island = ref world.islands[islandId];
		int removeIndex = joint.islandIndex;
		B2_ASSERT( 0 <= removeIndex && removeIndex < island.joints.count );
		B2_ASSERT( island.joints.data[removeIndex].jointId == joint.jointId );

		int movedIndex = island.joints.RemoveSwap( removeIndex );
		if ( movedIndex != NullIndex )
		{
			// Fix islandIndex on the joint that was swapped into removeIndex
			ref JointLink movedLink = ref island.joints.data[removeIndex];
			ref Joint movedJoint = ref world.joints[movedLink.jointId];
			B2_ASSERT( movedJoint.islandIndex == movedIndex );
			movedJoint.islandIndex = removeIndex;
		}

		joint.islandId = NullIndex;
		joint.islandIndex = NullIndex;
		island.constraintRemoveCount += 1;

		ValidateIsland( world, islandId );
	}

	// Find parent of a node. Use path halving to speed up further queries.
	internal static int IslandFindParent( Span<int> parents, int node )
	{
		// Walk the chain of parents to find the node that is its own parent (the root)
		while ( parents[node] != node )
		{
			int grandParent = parents[parents[node]];
			parents[node] = grandParent;
			node = grandParent;
		}

		return node;
	}

	// Connect the components containing node1 and node2.
	// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
	internal static void IslandUnion( Span<int> parents, Span<int> ranks, int node1, int node2, Span<int> contactCounts, Span<int> jointCounts )
	{
		int root1 = IslandFindParent( parents, node1 );
		int root2 = IslandFindParent( parents, node2 );
		if ( root1 != root2 )
		{
			if ( ranks[root1] < ranks[root2] )
			{
				parents[root1] = root2;
				contactCounts[root2] += contactCounts[root1];
				jointCounts[root2] += jointCounts[root1];
			}
			else if ( ranks[root1] > ranks[root2] )
			{
				parents[root2] = root1;
				contactCounts[root1] += contactCounts[root2];
				jointCounts[root1] += jointCounts[root2];
			}
			else
			{
				parents[root2] = root1;
				ranks[root1] += 1;
				contactCounts[root1] += contactCounts[root2];
				jointCounts[root1] += jointCounts[root2];
			}
		}
	}

	// This uses union-find.
	// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
	internal static void SplitIsland( World world, int baseId )
	{
		ref Island baseIsland = ref world.islands[baseId];
		B2_ASSERT( baseIsland.constraintRemoveCount > 0 );
		B2_ASSERT( baseIsland.setIndex == b2_awakeSet );

		ValidateIsland( world, baseId );

		// Cache base island fields before CreateIsland, which may reallocate
		// world.islands and invalidate the baseIsland pointer.
		int baseBodyCount = baseIsland.bodies.count;
		int[] baseBodyIds = baseIsland.bodies.data;
		int baseBodyCapacity = baseIsland.bodies.capacity;

		int baseContactCount = baseIsland.contacts.count;
		ContactLink[] baseContacts = baseIsland.contacts.data;
		int baseContactCapacity = baseIsland.contacts.capacity;

		int baseJointCount = baseIsland.joints.count;
		JointLink[] baseJoints = baseIsland.joints.data;
		int baseJointCapacity = baseIsland.joints.capacity;

		ArenaAllocator alloc = world.stack;
		// No lock is needed because I ensure the allocator is not used while this task is active.
		// Allocate contactCounts and jointCounts before ranks so ranks can be freed first (LIFO arena).
		Span<int> parents = alloc.Alloc<int>( baseBodyCount, "parents" );
		Span<int> contactCounts = alloc.Alloc<int>( baseBodyCount, "contact counts" );
		Span<int> jointCounts = alloc.Alloc<int>( baseBodyCount, "joint counts" );
		Span<int> ranks = alloc.Alloc<int>( baseBodyCount, "ranks" );
		for ( int i = 0; i < baseBodyCount; ++i )
		{
			parents[i] = i;
			ranks[i] = 0;
			contactCounts[i] = 0;
			jointCounts[i] = 0;
		}

		Span<Body> bodies = world.bodies.data;
		// Union over contacts, tracking per-component contact counts
		for ( int i = 0; i < baseContactCount; ++i )
		{
			int bodyIdA = baseContacts[i].bodyIdA;
			int bodyIdB = baseContacts[i].bodyIdB;
			B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world.bodies.count );
			B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world.bodies.count );
			ref Body bodyA = ref bodies[bodyIdA];
			ref Body bodyB = ref bodies[bodyIdB];
			int islandIndexA = bodyA.islandIndex;
			int islandIndexB = bodyB.islandIndex;

			// Only connect non-static bodies
			if ( islandIndexA != NullIndex && islandIndexB != NullIndex )
			{
				B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
				B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
				IslandUnion( parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts );
				int root = IslandFindParent( parents, islandIndexA );
				contactCounts[root] += 1;
			}
			else
			{
				int islandIndex = islandIndexA != NullIndex ? islandIndexA : islandIndexB;
				int root = IslandFindParent( parents, islandIndex );
				contactCounts[root] += 1;
			}
		}

		// Union over joints, tracking per-component joint counts
		for ( int i = 0; i < baseJointCount; ++i )
		{
			int bodyIdA = baseJoints[i].bodyIdA;
			int bodyIdB = baseJoints[i].bodyIdB;
			B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world.bodies.count );
			B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world.bodies.count );
			ref Body bodyA = ref bodies[bodyIdA];
			ref Body bodyB = ref bodies[bodyIdB];
			int islandIndexA = bodyA.islandIndex;
			int islandIndexB = bodyB.islandIndex;

			// Only connect non-static bodies
			if ( islandIndexA != NullIndex && islandIndexB != NullIndex )
			{
				B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
				B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
				IslandUnion( parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts );
				int root = IslandFindParent( parents, islandIndexA );
				jointCounts[root] += 1;
			}
			else
			{
				int islandIndex = islandIndexA != NullIndex ? islandIndexA : islandIndexB;
				int root = IslandFindParent( parents, islandIndex );
				jointCounts[root] += 1;
			}
		}

		// Done with ranks
		alloc.Free( ranks );
		ranks = null;

		// Flatten all parent indices and count connected components.
		int componentCount = 0;
		for ( int i = 0; i < baseBodyCount; ++i )
		{
			parents[i] = IslandFindParent( parents, i );
			if ( parents[i] == i )
			{
				componentCount += 1;
			}
		}

		// Early return — island is still fully connected, no split needed.
		if ( componentCount == 1 )
		{
			baseIsland.constraintRemoveCount = 0;
			alloc.Free( jointCounts );
			alloc.Free( contactCounts );
			alloc.Free( parents );
			return;
		}

		// Detach body/contact/joint arrays from base island so DestroyIsland won't free them
		baseIsland.bodies.data = null!;
		baseIsland.bodies.count = 0;
		baseIsland.bodies.capacity = 0;

		baseIsland.contacts.data = null!;
		baseIsland.contacts.count = 0;
		baseIsland.contacts.capacity = 0;

		baseIsland.joints.data = null!;
		baseIsland.joints.count = 0;
		baseIsland.joints.capacity = 0;

		// Null so code below doesn't accidentally use this.
		baseIsland = ref Unsafe.NullRef<Island>();

		// Map from body index to new island index. Only set for root bodies.
		Span<int> rootMap = alloc.Alloc<int>( baseBodyCount, "root map" );
		for ( int i = 0; i < baseBodyCount; ++i )
		{
			rootMap[i] = NullIndex;
		}

		Span<int> componentBodyCounts = alloc.Alloc<int>( componentCount, "component body counts" );
		Span<int> componentContactCounts = alloc.Alloc<int>( componentCount, "component contact counts" );
		Span<int> componentJointCounts = alloc.Alloc<int>( componentCount, "component joint counts" );
		int islandCount = 0;

		// Find the root body for each body and create islands as needed.
		// Extract per-component counts from the root nodes' accumulated counts.
		for ( int i = 0; i < baseBodyCount; ++i )
		{
			int rootIndex = parents[i];
			if ( rootMap[rootIndex] == NullIndex )
			{
				rootMap[rootIndex] = islandCount;
				componentBodyCounts[islandCount] = 0;
				componentContactCounts[islandCount] = contactCounts[rootIndex];
				componentJointCounts[islandCount] = jointCounts[rootIndex];
				islandCount += 1;
			}

			componentBodyCounts[rootMap[rootIndex]] += 1;
		}

		B2_ASSERT( islandCount == componentCount );

		// Map from new island index to island id
		Span<int> islandIds = alloc.Alloc<int>( islandCount, "island ids" );
		// Create new islands and reserve body/contact/joint arrays
		for ( int i = 0; i < islandCount; ++i )
		{
			// WARNING: this invalidates baseIsland pointer
			ref Island newIsland = ref CreateIsland( world, b2_awakeSet );
			islandIds[i] = newIsland.islandId;

			// Reserve arrays to avoid wasteful growth and memcpy.
			newIsland.bodies.Reserve( componentBodyCounts[i] );
			newIsland.contacts.Reserve( componentContactCounts[i] );
			newIsland.joints.Reserve( componentJointCounts[i] );
		}

		// Assign bodies to new islands
		for ( int i = 0; i < baseBodyCount; ++i )
		{
			int bodyId = baseBodyIds[i];
			int root = IslandFindParent( parents, i );
			int newIslandId = islandIds[rootMap[root]];

			ref Body body = ref world.bodies[bodyId];
			ref Island newIsland = ref world.islands[newIslandId];
			body.islandId = newIslandId;
			body.islandIndex = newIsland.bodies.count;

			// Ensure the array has the correct capacity
			B2_VALIDATE( newIsland.bodies.count < newIsland.bodies.capacity );
			newIsland.bodies.Push( bodyId );
		}

		// Assign contacts to the island of their bodies
		for ( int i = 0; i < baseContactCount; ++i )
		{
			ref ContactLink link = ref baseContacts[i];
			ref Contact contact = ref world.contacts[link.contactId];
			// Static bodies don't have an island id.
			ref Body bodyA = ref world.bodies[link.bodyIdA];
			ref Body bodyB = ref world.bodies[link.bodyIdB];
			int targetIslandId = bodyA.islandId != NullIndex ? bodyA.islandId : bodyB.islandId;

			ref Island targetIsland = ref world.islands[targetIslandId];
			contact.islandId = targetIslandId;
			contact.islandIndex = targetIsland.contacts.count;

			// Ensure the array has the correct capacity
			B2_VALIDATE( targetIsland.contacts.count < targetIsland.contacts.capacity );
			targetIsland.contacts.Push( link );
		}

		// Assign joints to the island of their bodies
		for ( int i = 0; i < baseJointCount; ++i )
		{
			ref JointLink link = ref baseJoints[i];
			ref Joint joint = ref world.joints[link.jointId];
			// Static bodies don't have an island id.
			ref Body bodyA = ref world.bodies[link.bodyIdA];
			ref Body bodyB = ref world.bodies[link.bodyIdB];
			int targetIslandId = bodyA.islandId != NullIndex ? bodyA.islandId : bodyB.islandId;

			ref Island targetIsland = ref world.islands[targetIslandId];
			joint.islandId = targetIslandId;
			joint.islandIndex = targetIsland.joints.count;

			// Ensure the array has the correct capacity
			B2_VALIDATE( targetIsland.joints.count < targetIsland.joints.capacity );
			targetIsland.joints.Push( link );
		}

		// Destroy the base island
		DestroyIsland( world, baseId );

		// Free the detached arrays manually
		Free( baseBodyIds, baseBodyCapacity );
		Free( baseContacts, baseContactCapacity );
		Free( baseJoints, baseJointCapacity );

		// Free arena items in LIFO order
		alloc.Free( islandIds );
		alloc.Free( componentJointCounts );
		alloc.Free( componentContactCounts );
		alloc.Free( componentBodyCounts );
		alloc.Free( rootMap );
		alloc.Free( jointCounts );
		alloc.Free( contactCounts );
		alloc.Free( parents );
	}

	// Split an island because some contacts and/or joints have been removed.
	// This is called during the constraint solve while islands are not being touched. This uses union find and
	// touches a lot of memory, so it can be slow.
	// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
	// Note: static bodies are never in an island
	// Note: this task interacts with some allocators without locks under the assumption that no other tasks
	// are interacting with these data structures.
	internal static void SplitIslandTask( object? context )
	{
		ulong ticks = GetTicks();
		World world = (World)context!;
		B2_ASSERT( world.splitIslandId != NullIndex );

		SplitIsland( world, world.splitIslandId );

		world.splitIslandId = NullIndex;
		world.profile.splitIslands += GetMilliseconds( ticks );
	}

#if BOX2D_VALIDATE
	internal static void ValidateIsland( World world, int islandId )
	{
		if ( islandId == NullIndex )
		{
			return;
		}

		ref Island island = ref world.islands[islandId];
		B2_ASSERT( island.islandId == islandId );
		B2_ASSERT( island.setIndex != NullIndex );

		{
			B2_ASSERT( island.bodies.count > 0 );
			B2_ASSERT( island.bodies.count <= GetIdCount( ref world.bodyIdPool ) );

			for ( int i = 0; i < island.bodies.count; ++i )
			{
				ref Body body = ref world.bodies[island.bodies.data[i]];
				B2_ASSERT( body.islandId == islandId );
				B2_ASSERT( body.islandIndex == i );
				B2_ASSERT( body.setIndex == island.setIndex );
			}
		}

		if ( island.contacts.count > 0 )
		{
			B2_ASSERT( island.contacts.count <= GetIdCount( ref world.contactIdPool ) );

			for ( int i = 0; i < island.contacts.count; ++i )
			{
				ref ContactLink link = ref island.contacts.data[i];
				ref Contact contact = ref world.contacts[link.contactId];
				B2_ASSERT( contact.setIndex == island.setIndex );
				B2_ASSERT( contact.islandId == islandId );
				B2_ASSERT( contact.islandIndex == i );
			}
		}

		if ( island.joints.count > 0 )
		{
			B2_ASSERT( island.joints.count <= GetIdCount( ref world.jointIdPool ) );

			for ( int i = 0; i < island.joints.count; ++i )
			{
				ref JointLink link = ref island.joints.data[i];
				ref Joint joint = ref world.joints[link.jointId];
				B2_ASSERT( joint.setIndex == island.setIndex );
				B2_ASSERT( joint.islandId == islandId );
				B2_ASSERT( joint.islandIndex == i );
			}
		}
	}

#else

	internal static void ValidateIsland( World world, int islandId )
	{
	}
#endif

}
