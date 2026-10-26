// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// Solver work is partitioned into fixed-size blocks that worker threads claim
	// in parallel via atomic CAS on a per-block syncIndex. The descriptor (SolverBlock)
	// and the atomic counter sit in a wrapping SyncBlock so the CAS-winner can
	// pass the descriptor by value into stage tasks without aliasing the atomic
	// memory other threads are CAS-writing. Three properties of this design
	// matter for performance:
	//
	// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line
	//    stampede that a single shared fetch_add counter would cause. Once a worker
	//    settles into a block range, its CAS targets live in its own L1.
	//
	// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
	//    solve, relax) reuse the same block array every sub-step iteration.
	//    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
	//    main thread never touches any per-block state between iterations.
	//    Non-iterative stages simply use syncIndex 1.
	//
	// 3. L2 affinity across iterations. Each worker picks a start offset from
	//    its workerIndex, then scans forward and (after wrap) backward:
	//
	//      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
	//                 ^           ^           ^   ^
	//                 W0          W1          W2  W3   <- start offsets
	//
	//    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
	//    each worker re-hits the same block range every iteration, keeping that
	//    range's hot data resident in its L2. A failed CAS means a neighbour
	//    already claimed the block, so the stealing worker stops. Preserving
	//    locality under mild imbalance while still draining the queue.
	//
	// A graph color stage lays out joint blocks first, then contact blocks:
	//
	//      stage.blocks .
	//        +------+------+------+------+------+------+------+
	//        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
	//        +------+------+------+------+------+------+------+
	//        <-- graphJointBlocks -.<---- graphContactBlocks ---.
	//
	// Each block carries its type so the dispatcher routes J-blocks to the joint
	// solver and C-blocks to the SIMD contact solver. Both kinds run concurrently
	// within the stage. No barrier between them. The type tag lives on the
	// block (not the stage) so that mixed-type stages can keep the concurrency.
	//
	// The solver threading model is inspired by https://github.com/bepu/bepuphysics2





	// Solver stages. Prepare joints and prepare contacts are split up
	// because there is no need to store joint impulses.
	internal enum SolverStageType
	{
		stagePrepareJoints,
		stagePrepareContacts,
		stageIntegrateVelocities,
		stageWarmStart,
		stageSolve,
		stageIntegratePositions,
		stageRelax,
		stageRestitution,
		stageStoreImpulses
	}

	internal enum SolverBlockType
	{
		bodyBlock,
		jointBlock,
		contactBlock,
		graphJointBlock,
		graphContactBlock
	}

	// Solver block describes a multithreaded unit of work.
	internal struct SolverBlock
	{
		public int startIndex;
		public ushort count;
		// SolverBlockType
		public byte blockType;
		public byte colorIndex;
	}

	// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
	// monotonically allowing the solver block to be re-used across sub-steps.
	internal struct SyncBlock
	{
		public SolverBlock block;
		public AtomicInt syncIndex;
		// byte padding[64 - Unsafe.SizeOf<SolverBlock>() - Unsafe.SizeOf<AtomicInt>()] (cache line padding, not needed in C#)
	}

	// Each stage must be completed before going to the next stage.
	// Non-iterative stages use a stage instance once while iterative stages re-use the same
	// instance each iteration.
	internal struct SolverStage
	{
		// The blocks of this stage are blocks[blockStart, blockStart + blockCount)
		public SyncBlock[] blocks;
		public int blockStart;
		public SolverStageType type;
		public int blockCount;
		public byte colorIndex;
		// byte padding1[64] (cache line padding, not needed in C#)
		public AtomicInt completionCount;
		// byte padding2[64] (cache line padding, not needed in C#)
	}

	// Constraint softness
	internal struct Softness
	{
		public float biasRate;
		public float massScale;
		public float impulseScale;
	}

	// Prepare/store run as a flat parallel-for over the whole wide-constraint
	// range. Each span maps a slice of that range back to the color's
	// contact sims so workers can decode flat wide-slot indices without touching
	// graph state. The spans array has one entry per active color plus a sentinel
	// whose start == wideContactCount.
	internal struct ContactPrepareSpan
	{
		public int start;
		public int count;
		public ContactSim[] contacts;
	}

	// Similar for the narrow phase.
	internal struct ContactCollideSpan
	{
		public int start;
		public ContactSim[] contacts;
	}

	// Similar for joints
	internal struct JointPrepareSpan
	{
		public int start;
		public int count;
		public JointSim[] joints;
	}

	// Context for a time step. Recreated each time step.
	internal sealed class StepContext
	{
		// time step
		public float dt;

		// inverse time step (0 if dt == 0).
		public float inv_dt;

		// sub-step
		public float h;
		public float inv_h;

		public int subStepCount;

		public Softness contactSoftness;
		public Softness staticSoftness;

		public float restitutionThreshold;
		public float maxLinearVelocity;

		public World world;
		public ConstraintGraph graph;

		// shortcut to body states from awake set
		public BodyState[] states;

		// shortcut to body sims from awake set
		public BodySim[] sims;

		// array of all shape ids for shapes that have enlarged AABBs
		public int[] enlargedShapes;
		public int enlargedShapeCount;

		// Array of bullet bodies that need continuous collision handling
		public int[] bulletBodies;
		public AtomicInt bulletBodyCount;

		public AtomicInt anyRestitution;

		// Graph color spans for the narrow-phase, includes non-touching contacts as well.
		public ContactCollideSpan[] collideSpans;

		// Flat view of the wide contact constraint array used by prepare and store.
		// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
		// at wideContactCount. wideContactConstraints is the contiguous base
		// pointer; per-color slices live at colors[i].wideConstraints.
		public ContactConstraintW4[] wideContactConstraints;
		public ContactPrepareSpan[] contactPrepareSpans;
		public int wideContactCount;

		public JointPrepareSpan[] jointPrepareSpans;
		public int jointCount;

		public int activeColorCount;
		public int workerCount;

		public SolverStage[] stages;
		public int stageCount;
		public bool enableWarmStarting;

		// padding to prevent false sharing
		// byte padding1[64] (cache line padding, not needed in C#)

		// This atomic is central to multi-threaded solver task synchronization.
		// It prevents ABA problems by monotonically growing as the solver advances.
		// This means a delayed worker thread will catch up without repeating already completed
		// work (causing a race condition).
		// sync index (16-bits) | stage type (16-bits)
		public AtomicU32 atomicSyncBits;

		// padding to prevent false sharing
		// byte padding2[64] (cache line padding, not needed in C#)

		// Race flag claimed by whichever runner reaches SolverTask with workerIndex 0 first.
		// The calling thread of World_Step also races for this slot so the orchestrator can
		// always make progress, regardless of how the user's task system schedules tasks (out
		// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
		// loser of the race no-ops as workerIndex 0.
		public AtomicInt mainClaimed;

		// padding to prevent false sharing
		// byte padding3[64] (cache line padding, not needed in C#)

	}

	internal static Softness MakeSoft( float hertz, float zeta, float h )
	{
		if ( hertz == 0.0f )
		{
			return new Softness { biasRate = 0.0f, massScale = 0.0f, impulseScale = 0.0f };
		}

		float omega = 2.0f * Pi * hertz;
		float a1 = 2.0f * zeta + h * omega;
		float a2 = h * omega * a1;
		float a3 = 1.0f / ( 1.0f + a2 );

		// bias = w / (2 * z + hw)
		// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
		// impulseScale = 1 / (1 + hw * (2 * z + hw))

		// If z == 0
		// bias = 1/h
		// massScale = hw^2 / (1 + hw^2)
		// impulseScale = 1 / (1 + hw^2)

		// w . inf
		// bias = 1/h
		// massScale = 1
		// impulseScale = 0

		// if w = pi / 4  * inv_h
		// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
		// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

		// In all cases:
		// massScale + impulseScale == 1

		return new Softness { biasRate = omega / a1, massScale = a2 * a3, impulseScale = a3 };
	}


}
