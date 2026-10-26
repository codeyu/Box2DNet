// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

#pragma warning disable CS8618 // fields are set by CreateWorld

public static partial class B2
{
	// Per thread task storage
	internal struct TaskContext
	{
		// Collect per thread sensor continuous hit events.
		public B2Array<SensorHit> sensorHits;

		// Broad-phase pairs.
		public B2Array<ulong> pairKeys;

		// These bits align with the contact id capacity and signal a change in contact status
		public BitSet contactStateBitSet;

		// These bits align with the contact id capacity and signal a hit event.
		public BitSet hitEventBitSet;

		// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
		public bool hasHitEvents;

		// These bits align with the joint id capacity and signal a change in contact status
		public BitSet jointStateBitSet;

		// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
		// that is very large when there are many static shapes.
		public BitSet enlargedSimBitSet;

		// Used to put islands to sleep
		public BitSet awakeIslandBitSet;

		// Per worker split island candidate
		public float splitSleepTime;
		public int splitIslandId;

		// Number of contacts recycled this step (collide pass).
		public int recycledContactCount;

	}

	// The world manages all physics entities, dynamic simulation,  and asynchronous queries.
	// The world also contains efficient memory management facilities.
	internal sealed class World
	{
		public ArenaAllocator stack;
		public BroadPhase broadPhase = new BroadPhase();
		public ConstraintGraph constraintGraph = new ConstraintGraph();

		// The body id pool is used to allocate and recycle body ids. Body ids
		// provide a stable identifier for users, but incur caches misses when used
		// to access body data. Aligns with Body.
		public IdPool bodyIdPool;

		// This is a sparse array that maps body ids to the body data
		// stored in solver sets. As sims move within a set or across set.
		// Indices come from id pool.
		public B2Array<Body> bodies;

		// Provides free list for solver sets.
		public IdPool solverSetIdPool;

		// Solvers sets allow sims to be stored in contiguous arrays. The first
		// set is all static sims. The second set is active sims. The third set is disabled
		// sims. The remaining sets are sleeping islands.
		public B2Array<SolverSet> solverSets;

		// Used to create stable ids for joints
		public IdPool jointIdPool;

		// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
		// or in the solver sets.
		public B2Array<Joint> joints;

		// Used to create stable ids for contacts
		public IdPool contactIdPool;

		// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
		// or in the solver sets.
		public B2Array<Contact> contacts;

		// Used to create stable ids for islands
		public IdPool islandIdPool;

		// Persistent islands
		public B2Array<Island> islands;

		public IdPool shapeIdPool;
		public IdPool chainIdPool;

		// These are sparse arrays that point into the pools above
		public B2Array<Shape> shapes;

		// Shape enlarged bounds (hot data split).
		public B2Array<AABB> fatAABBs;

		public B2Array<ChainShape> chainShapes;

		// This is a dense array of sensor data.
		public B2Array<Sensor> sensors;

		// Per thread storage
		public B2Array<TaskContext> taskContexts;
		public B2Array<SensorTaskContext> sensorTaskContexts;

		public B2Array<BodyMoveEvent> bodyMoveEvents;
		public B2Array<SensorBeginTouchEvent> sensorBeginEvents;
		public B2Array<ContactBeginTouchEvent> contactBeginEvents;

		// End events are double buffered so that the user doesn't need to flush events
		public FixedArray2<B2Array<SensorEndTouchEvent>> sensorEndEvents;
		public FixedArray2<B2Array<ContactEndTouchEvent>> contactEndEvents;
		public int endEventArrayIndex;

		public B2Array<ContactHitEvent> contactHitEvents;
		public B2Array<JointEvent> jointEvents;

		// todo consider deferred waking and impulses to make it possible
		// to apply forces and impulses from multiple threads
		// impulses must be deferred because sleeping bodies have no velocity state
		// Problems:
		// - multiple forces applied to the same body from multiple threads

		// Used to track debug draw
		public BitSet debugBodySet;
		public BitSet debugJointSet;
		public BitSet debugContactSet;
		public BitSet debugIslandSet;

		// Id that is incremented every time step
		public ulong stepIndex;

		// Identify islands for splitting as follows:
		// - I want to split islands so smaller islands can sleep
		// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
		//   if it has removed constraints
		// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
		// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
		// - if no bodies want to sleep then there is no reason to perform island splitting
		public int splitIslandId;

		public Vec2 gravity;
		public float hitEventThreshold;
		public float restitutionThreshold;
		public int restitutionIterations;
		public bool enableRestitutionPropagation;
		public float maxLinearSpeed;
		public float contactSpeed;
		public float contactHertz;
		public float contactDampingRatio;
		public float contactRecycleDistance;

		public FrictionCallbackFcn? frictionCallback;
		public RestitutionCallbackFcn? restitutionCallback;

		public ushort generation;

		public Profile profile;

		public Capacity maxCapacity;

		public PreSolveCallbackFcn? preSolveFcn;
		public PreContinuousCallbackFcn? preContinuousFcn;
		public object? preSolveContext;

		public CustomFilterCallbackFcn? customFilterFcn;
		public object? customFilterContext;

		public int workerCount;
		public int simdWidth;
		public EnqueueTaskCallbackFcn? enqueueTaskFcn;
		public FinishTaskCallbackFcn? finishTaskFcn;
		public object? userTaskContext;
		public object? userTreeTask;

		public Scheduler scheduler;

		public object? userData;

		public object? recording; // null unless World_StartRecording is active, owned by the host

		// Remember type step used for reporting forces and torques
		// inverse sub-step
		public float inv_h;

		// inverse full-step
		public float inv_dt;

		public int activeTaskCount;
		public int taskCount;

		public ushort worldId;

		public bool enableSleep;
		public bool locked;
		public bool enableWarmStarting;
		public bool enableContinuous;
		public bool inUse;
	}


	// Union of the broad-phase root bounds across all body types. Returns false when no tree holds a
	// proxy, so callers don't fold an empty world's origin into a running bounds.


}
