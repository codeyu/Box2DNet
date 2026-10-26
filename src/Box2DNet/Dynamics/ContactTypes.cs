// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// enum ContactFlags
		// Set when the solid shapes are touching.
		internal const uint b2_contactTouchingFlag = 0x00000001;

		// Contact has a hit event
		internal const uint b2_contactHitEventFlag = 0x00000002;

		// This contact wants contact events
		internal const uint b2_contactEnableContactEvents = 0x00000004;

		internal const uint b2_contactRecycleFlag = 0x00000008;

		// Set when the shapes are touching
		internal const uint b2_simTouchingFlag = 0x00010000;

		// This contact no longer has overlapping AABBs
		internal const uint b2_simDisjoint = 0x00020000;

		// This contact started touching
		internal const uint b2_simStartedTouching = 0x00040000;

		// This contact stopped touching
		internal const uint b2_simStoppedTouching = 0x00080000;

		// This contact has a hit event
		internal const uint b2_simEnableHitEvent = 0x00100000;

		// This contact wants pre-solve events
		internal const uint b2_simEnablePreSolveEvents = 0x00200000;

		// This contact has a cached relative transform
		internal const uint b2_simRelativeTransformValid = 0x00400000;

	// A contact edge is used to connect bodies and contacts together
	// in a contact graph where each body is a node and each contact
	// is an edge. A contact edge belongs to a doubly linked list
	// maintained in each attached body. Each contact has two contact
	// edges, one for each attached body.
	internal struct ContactEdge
	{
		public int bodyId;
		public int prevKey;
		public int nextKey;
	}

	// Cold contact data. Used as a persistent handle and for persistent island
	// connectivity.
	internal struct Contact
	{
		public FixedArray2<ContactEdge> edges;

		// A contact only belongs to an island if touching, otherwise NullIndex.
		public int islandId;

		// Index into the island's contacts array for O(1) swap-removal.
		// NullIndex when not in an island.
		public int islandIndex;

		// index of simulation set stored in World
		// NullIndex when slot is free
		public int setIndex;

		// index into the constraint graph color array
		// NullIndex for non-touching or sleeping contacts
		// NullIndex when slot is free
		public int colorIndex;

		// contact index within set or graph color
		// NullIndex when slot is free
		public int localIndex;

		public int shapeIdA;
		public int shapeIdB;
		public int contactId;

		// ContactFlags
		public uint flags;

		// This is monotonically advanced when a contact is allocated in this slot
		// Used to check for invalid ContactId
		public uint generation;
	}

	/// The class manages contact between two shapes. A contact exists for each overlapping
	/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	/// that has no contact points.
	internal struct ContactSim
	{
		public int contactId;

		// Cache for contact recycling.
		public Rot cachedRotationA;
		public Rot cachedRotationB;
		public Transform cachedRelativePose;

#if BOX2D_VALIDATE
		public int bodyIdA;
		public int bodyIdB;
#endif

		public int encodedBodySimA;
		public int encodedBodySimB;

		// ContactFlags
		public uint simFlags;

		public int shapeIdA;
		public int shapeIdB;

		public float invMassA;
		public float invIA;

		public float invMassB;
		public float invIB;

		public Manifold manifold;

		// Mixed friction and restitution
		public float friction;
		public float restitution;
		public float rollingResistance;
		public float tangentSpeed;

		public SimplexCache cache;
	}





	internal static float MixFriction( float frictionA, float frictionB )
	{
		return MathF.Sqrt( frictionA * frictionB );
	}

	internal static float MixRestitution( float restitutionA, float restitutionB )
	{
		return MaxFloat( restitutionA, restitutionB );
	}

}
