// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// enum BodyFlags
		// This body has fixed translation along the x-axis
		internal const uint b2_lockLinearX = 0x00000001;

		// This body has fixed translation along the y-axis
		internal const uint b2_lockLinearY = 0x00000002;

		// This body has fixed rotation
		internal const uint b2_lockAngularZ = 0x00000004;

		// This flag is used for debug draw and contact recycling. Only on BodySim.
		internal const uint b2_isFast = 0x00000008;

		// This dynamic body does a final CCD pass against all body types, but not other bullets
		internal const uint b2_isBullet = 0x00000010;

		// This body was speed capped in the current time step
		internal const uint b2_isSpeedCapped = 0x00000020;

		// This body had a time of impact event in the current time step (body sim only).
		internal const uint b2_hadTimeOfImpact = 0x00000040;

		// This body has no limit on angular velocity
		internal const uint b2_allowFastRotation = 0x00000080;

		// This bullet body needs to have its AABB increased. Needed because bullets don't follow
		// the standard broad-phase update.
		internal const uint b2_enlargeBulletBounds = 0x00000100;

		// This body is dynamic so the solver should write to it.
		// This prevents writing to kinematic bodies that causes a multithreaded sharing
		// cache coherence problem even when the values are not changing.
		// Used for BodyState flags.
		internal const uint b2_dynamicFlag = 0x00000200;

		// Flag to indicate the user has used the updateBodyMass option to defer mass
		// computation but Body_UpdateMassFromShapes was not called before the world step.
		internal const uint b2_dirtyMass = 0x00000400;

		internal const uint b2_enableSleep = 0x00000800;

		internal const uint b2_bodyEnableContactRecycling = 0x00001000;

		// All lock flags
		internal const uint b2_allLocks = b2_lockAngularZ | b2_lockLinearX | b2_lockLinearY;

		// If this flag is set then the body has fixed rotation
		internal const uint b2_fixedRotation = b2_lockAngularZ;

		// These flags are transient per time step. These may be different across Body, BodySim, and BodyState.
		internal const uint b2_bodyTransientFlags = b2_isSpeedCapped | b2_hadTimeOfImpact;

	// Body organizational details that are not used in the solver.
	internal struct Body
	{
		public object? userData;

		// index of solver set stored in World
		// may be NullIndex
		public int setIndex;

		// body sim and state index within set
		// may be NullIndex
		public int localIndex;

		// [31 : contactId | 1 : edgeIndex]
		public int headContactKey;
		public int contactCount;

		// todo maybe move this to the body sim
		public int headShapeId;
		public int shapeCount;

		public int headChainId;

		// [31 : jointId | 1 : edgeIndex]
		public int headJointKey;
		public int jointCount;

		// All enabled dynamic and kinematic bodies are in an island.
		public int islandId;

		// Need this island index for faster union-find
		public int islandIndex;

		public float mass;

		// Rotational inertia about the center of mass.
		public float inertia;

		public float sleepThreshold;
		public float sleepTime;
		public float safetyFactor;

		// this is used to adjust the fellAsleep flag in the body move array
		public int bodyMoveIndex;

		public int id;

		// BodyFlags
		public uint flags;

		public BodyType type;

		// This is monotonically advanced when a body is allocated in this slot
		// Used to check for invalid BodyId
		public ushort generation;

		public string? name;
	}

	// Body State
	// The body state is designed for fast conversion to and from SIMD via scatter-gather.
	// Only awake dynamic and kinematic bodies have a body state.
	// This is used in the performance critical constraint solver
	//
	// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
	// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
	// workers, even if the values don't change.
	// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
	// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
	// should be minimized.
	//
	// Solution 1:
	// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
	// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
	// is probably best if bodies move fast.
	//
	// Solution 2:
	// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
	// space. Potentially confusing and bug prone.
	//
	// Note:
	// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
	// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
	// round shapes.

	// 32 bytes
	internal struct BodyState
	{
		public Vec2 linearVelocity; // 8
		public float angularVelocity; // 4

		// BodyFlags
		// Important flags: locking, dynamic
		public uint flags; // 4

		// Using delta position reduces round-off error far from the origin
		public Vec2 deltaPosition; // 8

		// Using delta rotation because I cannot access the full rotation on static bodies in
		// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
		public Rot deltaRotation; // 8
	}

	// Identity body state, notice the deltaRotation is {1, 0}
	static readonly BodyState identityBodyState = new BodyState { linearVelocity = new( 0.0f, 0.0f ), angularVelocity = 0.0f, flags = 0, deltaPosition = new( 0.0f, 0.0f ), deltaRotation = new( 1.0f, 0.0f ) };

	// Body simulation data used for integration of position and velocity
	// Transform data used for collision and solver preparation.
	internal struct BodySim
	{
		// transform for body origin, double translation in large world mode
		public WorldTransform transform;

		// center of mass position in world space
		public Pos center;

		// previous rotation and COM for TOI
		public Rot rotation0;
		public Pos center0;

		// location of center of mass relative to the body origin
		public Vec2 localCenter;

		public Vec2 force;
		public float torque;

		// inverse inertia
		public float invMass;
		public float invInertia;

		public float minExtent;
		public float maxExtent;
		public float linearDamping;
		public float angularDamping;
		public float gravityScale;

		// Index of Body
		public int bodyId;

		// BodyFlags
		public uint flags;
	}

	// Get a validated body from a world using an id.


	// Create a BodyId from a raw id.



	// Careful calling this because it can invalidate body, state, joint, and contact pointers.


	// Build a sweep relative to a base position so continuous collision keeps float precision far
	// from the origin. The base cancels out of the relative motion the TOI actually solves.
	internal static Sweep MakeRelativeSweep( in BodySim bodySim, Pos @base )
	{
		Sweep s = default;
		s.c1 = SubPos( bodySim.center0, @base );
		s.c2 = SubPos( bodySim.center, @base );
		s.q1 = bodySim.rotation0;
		s.q2 = bodySim.transform.q;
		s.localCenter = bodySim.localCenter;
		return s;
	}

	internal static ref BodySim GetBodySim( World world, ref Body body )
	{
		ref SolverSet set = ref world.solverSets[body.setIndex];
		ref BodySim bodySim = ref set.bodySims[body.localIndex];
		return ref bodySim;
	}

	internal static ref BodyState GetBodyState( World world, ref Body body )
	{
		if ( body.setIndex == b2_awakeSet )
		{
			ref SolverSet set = ref world.solverSets[b2_awakeSet];
			return ref set.bodyStates[body.localIndex];
		}

		return ref Unsafe.NullRef<BodyState>();
	}

	// This encodes the body sim index for storage in the contact sim. This helps
	// avoid a lookup and cache miss in the narrow phase.
	internal static int EncodeBodySimIndex( in Body body )
	{
		if ( body.setIndex == b2_awakeSet )
		{
			return body.localIndex;
		}

		if ( body.setIndex == b2_staticSet )
		{
			return -( body.localIndex + 2 );
		}

		return NullIndex;
	}

	internal static bool IsStaticSimIndex( int encodedBodySimIndex )
	{
		return encodedBodySimIndex < NullIndex;
	}

	// In the contact solver a static body sim is expected to have null index.
	internal static int DecodeAwakeIndex( int encodedBodySimIndex )
	{
		return encodedBodySimIndex >= 0 ? encodedBodySimIndex : NullIndex;
	}

	// When a body goes to sleep the contact sim has a null index for it. This
	// also handles the case where the body is static and has an encoded
	// sim index of -2 or less.
	internal static int SleepBodySimIndex( int encodedIndex )
	{
		return encodedIndex >= 0 ? NullIndex : encodedIndex;
	}

}
