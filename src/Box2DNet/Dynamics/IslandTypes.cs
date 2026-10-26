// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// Cached contact data stored in the island for fast contiguous iteration.
	// Avoids touching Contact during union-find in SplitIsland.
	internal struct ContactLink
	{
		public int contactId;
		public int bodyIdA;
		public int bodyIdB;
	}

	// Cached joint data stored in the island for fast contiguous iteration.
	internal struct JointLink
	{
		public int jointId;
		public int bodyIdA;
		public int bodyIdB;
	}

	// Deterministic solver
	//
	// Collide all awake contacts
	// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
	// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
	// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
	// - stop touching: increment constraintRemoveCount

	// Persistent island for awake bodies, joints, and contacts.
	// Contacts are touching.
	// Contacts and joints may connect to static bodies, but static bodies are not in the island.
	// https://en.wikipedia.org/wiki/Component_(graph_theory)
	// https://en.wikipedia.org/wiki/Dynamic_connectivity
	internal struct Island
	{
		// index of solver set stored in World
		// may be NullIndex
		public int setIndex;

		// island index within set
		// may be NullIndex
		public int localIndex;

		public int islandId;

		// Keeps track of how many contacts have been removed from this island.
		// This is used to determine if an island is a candidate for splitting.
		public int constraintRemoveCount;

		// I tried using a stack array for this but the data pointer goes out of
		// sync when the world island array grows.
		public B2Array<int> bodies;

		// Contacts and joints that belong to this island. May connect to static
		// bodies not in the island.
		// Each link has the two body ids so that SplitIsland's union-find pass
		// never needs to touch Contact/Joint.
		public B2Array<ContactLink> contacts;
		public B2Array<JointLink> joints;

	}

	// This is used to move islands across solver sets
	internal struct IslandSim
	{
		public int islandId;
	}


	// Link contacts into the island graph when it starts having contact points

	// Unlink contact from the island graph when it stops having contact points

	// Link a joint into the island graph when it is created

	// Unlink a joint from the island graph when it is destroyed




}
