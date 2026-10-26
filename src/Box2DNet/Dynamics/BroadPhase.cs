// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
#if false && true
	// #define _CRT_SECURE_NO_WARNINGS
#endif




	internal static void CreateBroadPhase( BroadPhase bp, in Capacity capacity )
	{

		bp.pairSet = CreateSet( MaxInt( 32, 2 * capacity.contactCount ) );

		int staticCapacity = MaxInt( 16, capacity.staticShapeCount );
		CreateDynamicTreeInternal( bp.trees[BodyType.staticBody], staticCapacity );

		int kinematicCapacity = 16;
		CreateDynamicTreeInternal( bp.trees[BodyType.kinematicBody], kinematicCapacity );

		int dynamicCapacity = MaxInt( 16, capacity.dynamicShapeCount );
		CreateDynamicTreeInternal( bp.trees[BodyType.dynamicBody], dynamicCapacity );

		bp.movedSiblings = default;
	}

	internal static void DestroyBroadPhase( BroadPhase bp )
	{
		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			DestroyDynamicTreeInternal( bp.trees[i] );
		}

		DestroySet( ref bp.pairSet );
	}

	internal static int BroadPhase_CreateProxy( BroadPhase bp, BodyType proxyType, AABB aabb, ulong categoryBits, int shapeIndex, bool forcePairCreation )
	{
		B2_ASSERT( 0 <= proxyType && proxyType < BodyType.bodyTypeCount );

		bool mark = ( proxyType != BodyType.staticBody || forcePairCreation );

		int proxyId = CreateTreeProxyInternal( bp.trees[proxyType], aabb, categoryBits, (ulong)shapeIndex, mark );
		int proxyKey = B2_PROXY_KEY( proxyId, proxyType );
		return proxyKey;
	}

	internal static void BroadPhase_DestroyProxy( BroadPhase bp, int proxyKey )
	{
		BodyType proxyType = B2_PROXY_TYPE( proxyKey );
		int proxyId = B2_PROXY_ID( proxyKey );

		B2_ASSERT( 0 <= proxyType && proxyType < BodyType.bodyTypeCount );
		DestroyTreeProxy( bp.trees[proxyType], proxyId );
	}

	internal static void BroadPhase_MoveProxy( BroadPhase bp, int proxyKey, AABB aabb )
	{
		BodyType proxyType = B2_PROXY_TYPE( proxyKey );
		int proxyId = B2_PROXY_ID( proxyKey );

		bool mark = true;
		DynamicTree_MoveProxyInternal( bp.trees[proxyType], proxyId, aabb, mark );
	}

	// Gather the sibling pairs with a moved node. This is done serially it is cache friendly.
	private static int GatherMovedSiblings( DynamicTree tree, Span<int> pairIndices )
	{
		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		int nodeEnd = tree.nodeEnd;

		int count = 0;

		// Skip the root.
		for ( int pair = 2; pair < nodeEnd; pair += 2 )
		{
			// Push when either sibling moved.
			if ( ( ( nodes[pair].flagIndex | nodes[pair + 1].flagIndex ) & B2_MOVED_NODE ) != 0 )
			{
				pairIndices[count++] = pair;
			}
		}

		return count;
	}

	private const int B2_CANDIDATE_BATCH = 32;
	internal struct CandidatePair
	{
		public int shapeIdA;
		public int shapeIdB;
	}

	internal ref struct PairContext
	{
		public World world;
		public ref B2Array<ulong> pairKeys;
		public FixedArray32<CandidatePair> batch;
		public int batchCount;
	}

	internal struct NodePair
	{
		public TreeNode a;
		public TreeNode b;
	}

	internal struct IndexPair
	{
		public int a;
		public int b;
	}

	internal static void PrefetchHash( ref HashSet64 set, ulong hash )
	{
		uint capacity = set.capacity;
		uint index = (uint)hash & ( capacity - 1 );
	}

	private static void FlushCandidatePairs( scoped ref PairContext context )
	{
		World world = context.world;
		BroadPhase bp = world.broadPhase;
		int count1 = context.batchCount;
		context.batchCount = 0;

		// Prefetch hash set entries. Less than 1% gain from this but it might
		// matter more for Box3D. Also, if I switch to verstable then this might
		// not be possible.
		Span<ulong> keys = stackalloc ulong[B2_CANDIDATE_BATCH];
		Span<ulong> hashes = stackalloc ulong[B2_CANDIDATE_BATCH];
		for ( int i = 0; i < count1; ++i )
		{
			ref CandidatePair candidate = ref context.batch[i];
			keys[i] = HashSet64.ShapePairKey( candidate.shapeIdA, candidate.shapeIdB );
			hashes[i] = KeyHash( keys[i] );
			PrefetchHash( ref bp.pairSet, hashes[i] );
		}

		// Cull existing pairs.
		Span<CandidatePair> candidates = stackalloc CandidatePair[B2_CANDIDATE_BATCH];
		int count2 = 0;
		for ( int i = 0; i < count1; ++i )
		{
			bool pairExists = ContainsHashedKey( ref bp.pairSet, keys[i], hashes[i] );
			if ( pairExists == false )
			{
				candidates[count2] = context.batch[i];
				count2 += 1;
			}
		}

		// Prefetch shapes. Again, less than a 1% gain from this. Might matter more for Box3D.
		ReadOnlySpan<Shape> shapes = world.shapes.data;
		for ( int i = 0; i < count2; ++i )
		{
		}

		// Filter candidates.
		ref B2Array<ulong> pairKeys = ref context.pairKeys;
		for ( int i = 0; i < count2; ++i )
		{
			int shapeIdA = candidates[i].shapeIdA;
			int shapeIdB = candidates[i].shapeIdB;

			ref Shape shapeA = ref world.shapes[shapeIdA];
			ref Shape shapeB = ref world.shapes[shapeIdB];
			int bodyIdA = shapeA.bodyId;
			int bodyIdB = shapeB.bodyId;

			// Are the shapes on the same body?
			if ( bodyIdA == bodyIdB )
			{
				continue;
			}

			// Sensors are handled elsewhere
			if ( shapeA.sensorIndex != NullIndex || shapeB.sensorIndex != NullIndex )
			{
				continue;
			}

			if ( ShouldShapesCollide( shapeA.filter, shapeB.filter ) == false )
			{
				continue;
			}

			if ( CanCollide( shapeA.type, shapeB.type ) == false )
			{
				// For example, no segment vs segment collision
				continue;
			}

			// Does a joint override collision?
			ref Body bodyA = ref world.bodies[bodyIdA];
			ref Body bodyB = ref world.bodies[bodyIdB];
			if ( ShouldBodiesCollide( world, ref bodyA, ref bodyB ) == false )
			{
				continue;
			}

			// Custom user filter
			if ( shapeA.enableCustomFiltering || shapeB.enableCustomFiltering )
			{
				CustomFilterCallbackFcn customFilterFcn = world.customFilterFcn;
				if ( customFilterFcn != null )
				{
					ShapeId idA = new ShapeId { index1 = shapeIdA + 1, world0 = world.worldId, generation = shapeA.generation };
					ShapeId idB = new ShapeId { index1 = shapeIdB + 1, world0 = world.worldId, generation = shapeB.generation };
					bool shouldCollide = customFilterFcn( idA, idB, world.customFilterContext );
					if ( shouldCollide == false )
					{
						continue;
					}
				}
			}

			// The pair passed the gauntlet. A new contact will be created.
			pairKeys.Push( HashSet64.ShapePairKey( shapeIdA, shapeIdB ) );
		}
	}

		[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void AddCandidatePair( int shapeIdA, int shapeIdB, scoped ref PairContext context )
	{
		// Follow shape index order.
		ref CandidatePair candidate = ref context.batch[context.batchCount];
		candidate.shapeIdA = MinInt( shapeIdA, shapeIdB );
		candidate.shapeIdB = MaxInt( shapeIdA, shapeIdB );
		context.batchCount += 1;
		if ( context.batchCount == B2_CANDIDATE_BATCH )
		{
			FlushCandidatePairs( ref context );
		}
	}

	// Did either move and if so do they overlap?
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool TestPair( in TreeNode a, in TreeNode b )
	{
		if ( ( ( a.flagIndex | b.flagIndex ) & B2_MOVED_NODE ) == 0 )
		{
			return false;
		}

		return AABB_Overlaps( a.aabb, b.aabb );
	}

	private static void CollideProxyAndSubtree( in TreeNode proxy, ReadOnlySpan<TreeNode> nodes, int pair, scoped ref PairContext context )
	{
		uint proxyMark = proxy.flagIndex & B2_MOVED_NODE;
		AABB boxv = LoadAABBV( proxy.aabb );
		int shapeId = proxy.shapeIndex;

		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = pair;

		while ( stackCount > 0 )
		{
			pair = stack[--stackCount];
			ReadOnlySpan<TreeNode> children = nodes.Slice( pair, 2 );
			int movedMask = 3;
			if ( proxyMark == 0 )
			{
				movedMask = ( IsNodeMoved( children[0] ) ? 1 : 0 ) | ( IsNodeMoved( children[1] ) ? 2 : 0 );
			}

			int mask = OverlapNodePairMask( boxv, children, 0 ) & movedMask;
			while ( mask != 0 )
			{
				int i = (int)CTZ32( (uint)mask );
				mask &= mask - 1;

				ref readonly TreeNode node = ref children[i];
				if ( IsLeaf( node ) )
				{
					AddCandidatePair( shapeId, node.shapeIndex, ref context );
				}
				else
				{
					if ( stackCount < B2_TREE_STACK_SIZE )
					{
						stack[stackCount++] = GetLeftChild( node );
					}
					else
					{
						B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
					}
				}
			}
		}
	}

		[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void ExpandPair( ReadOnlySpan<TreeNode> arrayA, ReadOnlySpan<TreeNode> arrayB, in TreeNode nodeA, in TreeNode nodeB, scoped Span<IndexPair> stack, ref int stackCount, scoped ref PairContext context )
	{
		bool leafA = IsLeaf( nodeA );
		bool leafB = IsLeaf( nodeB );
		if ( leafA && leafB )
		{
			AddCandidatePair( nodeA.shapeIndex, nodeB.shapeIndex, ref context );
		}
		else if ( leafA )
		{
			CollideProxyAndSubtree( nodeA, arrayB, GetLeftChild( nodeB ), ref context );
		}
		else if ( leafB )
		{
			CollideProxyAndSubtree( nodeB, arrayA, GetLeftChild( nodeA ), ref context );
		}
		else
		{
			if ( stackCount < B2_TREE_STACK_SIZE )
			{
				stack[stackCount] = new IndexPair { a = GetLeftChild( nodeA ), b = GetLeftChild( nodeB ) };
				stackCount += 1;
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
			}
		}
	}

	// This collides two sub-trees against each other. They can live in the same dynamic tree.
	// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
	// this does not generate duplicate pairs.
	// For example consider the full binary tree A (B (D  E) C (F G))
	// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
	// Then colliding children of B can give the pair (D,E) and for C (F,G).
	// So no duplicates even when used for self-collision.
	// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
	// self collision gathers all those moved internal nodes and collides their subtrees together.
	// See Real-time collision detection section 6.3.2. This is faster than querying every moved
	// proxy against the whole tree. Scaling is linear instead of linear * log.
	// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
	private static void CollideCrossPairs( ReadOnlySpan<TreeNode> arrayA, ReadOnlySpan<TreeNode> arrayB, in TreeNode subtreeA, in TreeNode subtreeB, scoped ref PairContext context )
	{
		Span<IndexPair> stack = stackalloc IndexPair[B2_TREE_STACK_SIZE];
		int stackCount = 0;

		// Seed the stack.
		if ( TestPair( subtreeA, subtreeB ) )
		{
			ExpandPair( arrayA, arrayB, subtreeA, subtreeB, stack, ref stackCount, ref context );
		}

		while ( stackCount > 0 )
		{
			IndexPair pair = stack[--stackCount];
			ReadOnlySpan<TreeNode> pairA = arrayA.Slice( pair.a, 2 );
			ReadOnlySpan<TreeNode> pairB = arrayB.Slice( pair.b, 2 );
			int movedA = ( IsNodeMoved( pairA[0] ) ? 0x3 : 0 ) | ( IsNodeMoved( pairA[1] ) ? 0xC : 0 );
			int movedB = ( IsNodeMoved( pairB[0] ) ? 0x5 : 0 ) | ( IsNodeMoved( pairB[1] ) ? 0xA : 0 );

			int mask = OverlapPairMask( pairA, 0, pairB, 0 ) & ( movedA | movedB );
			while ( mask != 0 )
			{
				int lane = (int)CTZ32( (uint)mask );
				mask &= mask - 1;
				ExpandPair( arrayA, arrayB, pairA[lane >> 1], pairB[lane & 1], stack, ref stackCount, ref context );
			}
		}
	}

	// This takes moved internal nodes and collides their sub-trees against each other.
	private static void SelfPairsTask( int startIndex, int endIndex, int workerIndex, object? context )
	{
		World world = (World)context!;
		BroadPhase bp = world.broadPhase;
		DynamicTree tree = bp.trees[BodyType.dynamicBody];
		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		ReadOnlySpan<int> siblingIndices = bp.movedSiblings;
		PairContext pairContext = new PairContext { world = world, pairKeys = ref world.taskContexts.data[workerIndex].pairKeys };

		for ( int i = startIndex; i < endIndex; ++i )
		{
			int nodeIndex = siblingIndices[i];
			CollideCrossPairs( nodes, nodes, nodes[nodeIndex], nodes[nodeIndex + 1], ref pairContext );
		}

		FlushCandidatePairs( ref pairContext );
	}

	private const int B2_CROSS_SEED_COUNT = 64;
	// This does a serial cross-tree breadth first search until the queue is full. Then it returns
	// the queue pairs as seeds for a parallel search.
	private static int GatherCrossSeeds( DynamicTree treeA, DynamicTree treeB, Span<NodePair> seeds )
	{
		ReadOnlySpan<TreeNode> nodesA = treeA.nodes;
		ReadOnlySpan<TreeNode> nodesB = treeB.nodes;
		// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
		// survives nothing.
		Span<NodePair> queue = stackalloc NodePair[2 * B2_CROSS_SEED_COUNT];
		int mask = 2 * B2_CROSS_SEED_COUNT - 1;
		int head = 0;
		int tail = 0;

		ref readonly TreeNode rootA = ref nodesA[B2_ROOT_NODE];
		ref readonly TreeNode rootB = ref nodesB[B2_ROOT_NODE];
		if ( TestPair( rootA, rootB ) )
		{
			queue[tail & mask] = new NodePair { a = rootA, b = rootB };
			tail += 1;
		}

		int seedCount = 0;
		while ( head < tail && seedCount + ( tail - head ) + 3 < B2_CROSS_SEED_COUNT )
		{
			NodePair pair = queue[head & mask];
			head += 1;

			if ( IsLeaf( pair.a ) || IsLeaf( pair.b ) )
			{
				seeds[seedCount++] = pair;
				continue;
			}

			ReadOnlySpan<TreeNode> a = nodesA.Slice( GetLeftChild( pair.a ), 2 );
			ReadOnlySpan<TreeNode> b = nodesB.Slice( GetLeftChild( pair.b ), 2 );
			// Nodes have two children each, so four combinations.
			for ( int i = 0; i < 2; ++i )
			{
				for ( int j = 0; j < 2; ++j )
				{
					if ( TestPair( a[i], b[j] ) )
					{
						queue[tail & mask] = new NodePair { a = a[i], b = b[j] };
						tail += 1;
					}
				}
			}
		}

		while ( head < tail )
		{
			seeds[seedCount] = queue[head & mask];
			seedCount += 1;
			head += 1;
		}

		return seedCount;
	}

	internal sealed class CrossContext
	{
		public World world;
		public NodePair[] seeds;
		public int staticSeedCount;
	}

	private static void CrossPairsTask( int startIndex, int endIndex, int workerIndex, object? context )
	{
		CrossContext crossContext = (CrossContext)context!;
		World world = crossContext.world;
		BroadPhase bp = world.broadPhase;
		TreeNode[] staticNodes = bp.trees[BodyType.staticBody].nodes;
		TreeNode[] kinematicNodes = bp.trees[BodyType.kinematicBody].nodes;
		TreeNode[] dynamicNodes = bp.trees[BodyType.dynamicBody].nodes;
		PairContext pairContext = new PairContext { world = world, pairKeys = ref world.taskContexts.data[workerIndex].pairKeys };

		for ( int i = startIndex; i < endIndex; ++i )
		{
			TreeNode[] nodesB = i < crossContext.staticSeedCount ? staticNodes : kinematicNodes;
			NodePair seed = crossContext.seeds[i];
			CollideCrossPairs( dynamicNodes, nodesB, seed.a, seed.b, ref pairContext );
		}

		FlushCandidatePairs( ref pairContext );
	}

	// LSD radix sort. Faster than qsort when count is large and fast enough otherwise.
	// https://en.wikipedia.org/wiki/Radix_sort
	internal static void RadixSortKeys( Span<ulong> keys, Span<ulong> tempKeys, int count )
	{
		Span<int> digitCounts = stackalloc int[8 * 256];
		digitCounts.Clear();
		for ( int i = 0; i < count; ++i )
		{
			ulong key = keys[i];
			for ( int digit = 0; digit < 8; ++digit )
			{
				digitCounts[digit * 256 + (int)( ( key >> ( 8 * digit ) ) & 0xFF )] += 1;
			}
		}

		Span<ulong> source = keys;
		Span<ulong> target = tempKeys;
		bool sourceIsKeys = true;

		for ( int digit = 0; digit < 8; ++digit )
		{
			int shift = 8 * digit;
			Span<int> offsets = digitCounts.Slice( digit * 256, 256 );

			if ( offsets[(int)( ( source[0] >> shift ) & 0xFF )] == count )
			{
				// All keys have the same value for this digit. Common for shape pair keys.
				continue;
			}

			int sum = 0;
			for ( int i = 0; i < 256; ++i )
			{
				int digitCount = offsets[i];
				offsets[i] = sum;
				sum += digitCount;
			}

			for ( int i = 0; i < count; ++i )
			{
				ulong key = source[i];
				target[offsets[(int)( ( key >> shift ) & 0xFF )]++] = key;
			}

			// Swap for the next pass
			Span<ulong> swap = source;
			source = target;
			target = swap;
			sourceIsKeys = !sourceIsKeys;
		}

		if ( sourceIsKeys == false )
		{
			source.Slice( 0, count ).CopyTo( keys );
		}
	}

	private static void UpdateTreesTask( object? context )
	{
		World world = (World)context!;
		DynamicTree_Rebuild( world.broadPhase.trees[BodyType.dynamicBody], false );
		DynamicTree_Rebuild( world.broadPhase.trees[BodyType.kinematicBody], false );
	}

	// Task that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	private static void EnqueueTreeUpdate( World world )
	{
		if ( world.taskCount < MaxTasks )
		{
			world.userTreeTask = world.enqueueTaskFcn( UpdateTreesTask, world, world.userTaskContext );
			world.taskCount += 1;
			world.activeTaskCount += world.userTreeTask == null ? 0 : 1;
		}
		else
		{
			world.userTreeTask = null;
			UpdateTreesTask( world );
		}
	}

	internal static void UpdateBroadPhasePairs( World world )
	{
		BroadPhase bp = world.broadPhase;
		bool needUpdate = HasTreeMoved( bp.trees[BodyType.staticBody] );
		needUpdate = needUpdate || NeedsRebuild( bp.trees[BodyType.kinematicBody] );
		needUpdate = needUpdate || NeedsRebuild( bp.trees[BodyType.dynamicBody] );

		if ( needUpdate == false )
		{
			return;
		}
		ArenaAllocator alloc = world.stack;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			world.taskContexts.data[i].pairKeys.Clear();
		}

		// Generate pairs by querying the dynamic body tree against itself and against
		// the kinematic and static trees.
		{
			// Get the sibling pairs of the dynamic body tree that have moved.
			DynamicTree dynamicTree = bp.trees[BodyType.dynamicBody];
			int pairCapacity = dynamicTree.nodeEnd / 2;
			bp.movedSiblings = alloc.Alloc<int>( pairCapacity, "moved pairs" );
			int dynamicMoveCount = GatherMovedSiblings( dynamicTree, bp.movedSiblings );

			// Get seeds for colliding against the static and kinematic trees.
			NodePair[] crossSeeds = new NodePair[2 * B2_CROSS_SEED_COUNT];
			int staticSeedCount = GatherCrossSeeds( dynamicTree, bp.trees[BodyType.staticBody], crossSeeds );
			B2_ASSERT( staticSeedCount <= B2_CROSS_SEED_COUNT );
			int kinematicSeedCount = GatherCrossSeeds( dynamicTree, bp.trees[BodyType.kinematicBody], crossSeeds.AsSpan( staticSeedCount ) );
			B2_ASSERT( kinematicSeedCount <= B2_CROSS_SEED_COUNT );
			int crossMoveCount = staticSeedCount + kinematicSeedCount;

			// Collide the dynamic body tree against the static and kinematic trees.
			CrossContext crossContext = new CrossContext { world = world, seeds = crossSeeds, staticSeedCount = staticSeedCount };
			ParallelFor( world, CrossPairsTask, crossMoveCount, 1, crossContext );

			// Collide the dynamic body tree against itself.
			ParallelFor( world, SelfPairsTask, dynamicMoveCount, 64, world );
		}

		DynamicTree_ClearMoved( bp.trees[BodyType.staticBody] );
		// Update stale trees.
		EnqueueTreeUpdate( world );

		// Pairs arrive in deterministic order but scrambled relative to body and shape order
		// sorting them here improves solver performance.
		int pairCount = 0;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			pairCount += world.taskContexts.data[i].pairKeys.count;
		}

		Span<ulong> pairKeys = alloc.Alloc<ulong>( MaxInt( pairCount, 1 ), "pair keys" );
		int keyCount = 0;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			ref readonly B2Array<ulong> workerKeys = ref world.taskContexts.data[i].pairKeys;
			if ( workerKeys.count > 0 )
			{
				workerKeys.AsSpan().CopyTo( pairKeys.Slice( keyCount ) );
				keyCount += workerKeys.count;
			}
		}

		B2_ASSERT( keyCount == pairCount );

		if ( pairCount > 1 )
		{
			Span<ulong> tempKeys = alloc.Alloc<ulong>( pairCount, "temp keys" );
			RadixSortKeys( pairKeys, tempKeys, pairCount );
			alloc.Free( tempKeys );
		}

		for ( int i = 0; i < keyCount; ++i )
		{
			int shapeIdA = (int)( pairKeys[i] >> 32 );
			int shapeIdB = (int)( pairKeys[i] & 0xFFFFFFFF );
			ref Shape shapeA = ref world.shapes[shapeIdA];
			ref Shape shapeB = ref world.shapes[shapeIdB];
			CreateContact( world, ref shapeA, ref shapeB );
		}

		alloc.Free( pairKeys );

		alloc.Free( bp.movedSiblings );
		bp.movedSiblings = default;

		ValidateSolverSets( world );
	}

	internal static void ValidateBroadphase( BroadPhase bp )
	{
		DynamicTree_Validate( bp.trees[BodyType.dynamicBody] );
		DynamicTree_Validate( bp.trees[BodyType.kinematicBody] );

		// TODO_ERIN validate every shape AABB is contained in tree AABB
	}

	internal static void ValidateNoEnlarged( BroadPhase bp )
	{
#if BOX2D_VALIDATE
		for ( int j = 0; j < (int)BodyType.bodyTypeCount; ++j )
		{
			DynamicTree tree = bp.trees[j];
			DynamicTree_ValidateNoMoved( tree );
		}
#else
#endif
	}

}
