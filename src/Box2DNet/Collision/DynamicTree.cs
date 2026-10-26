// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
	// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
	// allocations can just bump.
	private static int AllocateSiblingPair( DynamicTree tree )
	{
		// Allocate from free list.
		if ( tree.pairFreeList != NullIndex )
		{
			int freePair = tree.pairFreeList;
			tree.pairFreeList = tree.parents[freePair];
			return freePair;
		}

		// Extend the bump allocator if needed.
		if ( tree.nodeEnd + 2 > tree.nodeCapacity )
		{
			int oldCapacity = tree.nodeCapacity;
			int newCapacity = oldCapacity + ( oldCapacity >> 1 );
			newCapacity += newCapacity & 1;
			tree.nodes = GrowAlloc( tree.nodes, oldCapacity, newCapacity );
			tree.parents = GrowAlloc( tree.parents, oldCapacity, newCapacity );
			tree.nodeCapacity = newCapacity;

			// The spare has to match, the rebuild allocates it again.
			Free( tree.swapNodes, oldCapacity );
			tree.swapNodes = null;
		}

		int pair = tree.nodeEnd;
		tree.nodeEnd += 2;
		return pair;
	}

	private static void FreePair( DynamicTree tree, int pair )
	{
		B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree.nodeEnd );
		tree.nodes[pair] = MakeEmptyNode();
		tree.nodes[pair + 1] = MakeEmptyNode();
		tree.parents[pair] = tree.pairFreeList;
		tree.parents[pair + 1] = NullIndex;
		tree.pairFreeList = pair;
	}

	internal static void CreateDynamicTreeInternal( DynamicTree tree, int proxyCapacity )
	{
		int capacity = MaxInt( proxyCapacity, 16 );

		// memset needed for deterministic serialization.
		tree.Clear();

		// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
		tree.nodeCapacity = 2 * capacity;
		tree.nodes = Alloc<TreeNode>( tree.nodeCapacity );
		tree.parents = Alloc<int>( tree.nodeCapacity );
		tree.pairFreeList = NullIndex;

		// The root and the empty node always exist
		tree.nodes[B2_ROOT_NODE] = MakeEmptyNode();
		tree.nodes[B2_ROOT_NODE + 1] = MakeEmptyNode();
		tree.parents[B2_ROOT_NODE] = NullIndex;
		tree.parents[B2_ROOT_NODE + 1] = NullIndex;

		// This is the bump index.
		tree.nodeEnd = 2;
		tree.dfsOrdered = true;

		tree.proxyCapacity = capacity;
		tree.proxyCount = 0;
		tree.proxies = Alloc<TreeProxy>( tree.proxyCapacity );

		// Build a linked list for the free list.
		for ( int i = 0; i < tree.proxyCapacity - 1; ++i )
		{
			tree.proxies[i].node = NullIndex;
			tree.proxies[i].next = i + 1;
		}

		tree.proxies[tree.proxyCapacity - 1].node = NullIndex;
		tree.proxies[tree.proxyCapacity - 1].next = NullIndex;
		tree.proxyFreeList = 0;

		tree.leafIndices = null;
		tree.leafNodes = null;
		tree.leafBoxes = null;
		tree.leafCenters = null;
		tree.binIndices = null;
		tree.rebuildCapacity = 0;
	}

	internal static void DestroyDynamicTreeInternal( DynamicTree tree )
	{
		Free( tree.nodes, tree.nodeCapacity );
		Free( tree.parents, tree.nodeCapacity );
		Free( tree.proxies, tree.proxyCapacity );
		Free( tree.swapNodes, tree.nodeCapacity );
		Free( tree.leafIndices, tree.rebuildCapacity );
		Free( tree.leafNodes, tree.rebuildCapacity );
		Free( tree.leafBoxes, tree.rebuildCapacity );
		Free( tree.leafCenters, tree.rebuildCapacity );
		Free( tree.binIndices, tree.rebuildCapacity );

		tree.Clear();
	}

	public static DynamicTree CreateDynamicTree( int proxyCapacity )
	{
		DynamicTree tree = new DynamicTree();
		CreateDynamicTreeInternal( tree, proxyCapacity );
		return tree;
	}

	public static void DestroyDynamicTree( DynamicTree tree )
	{
		if ( tree == null )
		{
			return;
		}

		DestroyDynamicTreeInternal( tree );
	}

	// Allocate a proxy from the pool. Grow the pool if necessary.
	private static int AllocateProxy( DynamicTree tree )
	{
		// Expand the pool as needed.
		if ( tree.proxyFreeList == NullIndex )
		{
			B2_ASSERT( tree.proxyCount == tree.proxyCapacity );

			// The free list is empty. Rebuild a bigger pool.
			// A restored pool can have capacity 1, so guarantee strict growth.
			int oldCapacity = tree.proxyCapacity;
			tree.proxyCapacity += MaxInt( oldCapacity >> 1, 1 );
			tree.proxies = GrowAlloc( tree.proxies, oldCapacity, tree.proxyCapacity );

			// Build a linked list for the free list.
			for ( int i = oldCapacity; i < tree.proxyCapacity - 1; ++i )
			{
				tree.proxies[i].node = NullIndex;
				tree.proxies[i].next = i + 1;
			}

			tree.proxies[tree.proxyCapacity - 1].node = NullIndex;
			tree.proxies[tree.proxyCapacity - 1].next = NullIndex;
			tree.proxyFreeList = oldCapacity;
		}

		// Peel a proxy off the free list.
		int proxyIndex = tree.proxyFreeList;
		tree.proxyFreeList = tree.proxies[proxyIndex].next;
		tree.proxies[proxyIndex] = default;
		tree.proxies[proxyIndex].node = NullIndex;
		tree.proxies[proxyIndex].next = NullIndex;
		++tree.proxyCount;
		return proxyIndex;
	}

	// Return a proxy to the pool.
	private static void FreeProxy( DynamicTree tree, int proxyId )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
		B2_ASSERT( 0 < tree.proxyCount );
		tree.proxies[proxyId].node = NullIndex;
		tree.proxies[proxyId].next = tree.proxyFreeList;
		tree.proxyFreeList = proxyId;
		--tree.proxyCount;
	}

	internal static int GetNodeHeight( in TreeNode node )
	{
		return IsLeaf( node ) ? 0 : node.height;
	}

	// The internal node above a children pair
	internal static TreeNode MakeInternalNode( ReadOnlySpan<TreeNode> nodes, int pair )
	{
		ref readonly TreeNode c1 = ref nodes[pair];
		ref readonly TreeNode c2 = ref nodes[pair + 1];
		TreeNode node = default;
		node.aabb = AABB_Union( c1.aabb, c2.aabb );
		node.flagIndex = (uint)pair | ( ( c1.flagIndex | c2.flagIndex ) & B2_MOVED_NODE );
		node.height = 1 + MaxInt( GetNodeHeight( c1 ), GetNodeHeight( c2 ) );
		return node;
	}

	internal static TreeNode MakeLeafNode( AABB aabb, int proxyId, ulong userData, bool moved )
	{
		TreeNode node = default;
		node.aabb = aabb;
		node.flagIndex = (uint)proxyId | B2_LEAF_NODE | ( moved ? B2_MOVED_NODE : 0 );
		node.shapeIndex = (int)userData;
		return node;
	}

	// A node landed at a new index, so tell what hangs below it
	internal static void LinkChildren( DynamicTree tree, int nodeIndex )
	{
		ref readonly TreeNode node = ref tree.nodes[nodeIndex];
		if ( IsLeaf( node ) )
		{
			tree.proxies[GetProxyId( node )].node = nodeIndex;
		}
		else
		{
			int pair = GetLeftChild( node );
			tree.parents[pair] = nodeIndex;
			tree.parents[pair + 1] = nodeIndex;
		}
	}

	// The sweep refit visits indices from high to low, so it needs every child above its parent
	internal static bool IsNodeOrdered( ReadOnlySpan<TreeNode> nodes, int nodeIndex )
	{
		ref readonly TreeNode node = ref nodes[nodeIndex];
		return IsLeaf( node ) || nodeIndex < GetLeftChild( node );
	}

	// Greedy algorithm for sibling selection using the SAH
	// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
	// 1: make a new parent for A and D : E-(A-(B,C), D)
	// 2: associate D with B
	//   a: B is a leaf : A-(E-(B,D), C)
	//   b: B is an internal node: A-(B{D},C)
	// 3: associate D with C
	//   a: C is a leaf : A-(B, E-(C,D))
	//   b: C is an internal node: A-(B, C{D})
	// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

	// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
	// cost of sibling H = area(union(H, D)) + increased area of ancestors

	// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
	// case1: D becomes a sibling of B
	// case2: D becomes a descendant of B along with a new internal node of area(D).
	private static int FindBestSibling( DynamicTree tree, AABB boxD )
	{
		Vec2 centerD = AABB_Center( boxD );
		float areaD = Perimeter( boxD );

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		int nodeIndex = B2_ROOT_NODE;

		if ( IsLeaf( nodes[nodeIndex] ) )
		{
			return nodeIndex;
		}

		AABB rootBox = nodes[nodeIndex].aabb;

		// Area of current node
		float areaBase = Perimeter( rootBox );

		// Area of inflated node
		float directCost = Perimeter( AABB_Union( rootBox, boxD ) );
		float inheritedCost = 0.0f;

		int bestSibling = nodeIndex;
		float bestCost = directCost;

		// Descend the tree, following a single greedy path.
		for ( ;; )
		{
			int child1 = GetLeftChild( nodes[nodeIndex] );
			int child2 = child1 + 1;

			// Cost of creating a new parent for this node and the new leaf
			float cost = directCost + inheritedCost;

			// Sometimes there are multiple identical costs within tolerance.
			// This breaks the ties using the centroid distance.
			if ( cost < bestCost )
			{
				bestSibling = nodeIndex;
				bestCost = cost;
			}

			// Inheritance cost seen by children
			inheritedCost += directCost - areaBase;

			bool leaf1 = IsLeaf( nodes[child1] );
			bool leaf2 = IsLeaf( nodes[child2] );

			// Cost of descending into child 1
			float lowerCost1 = FLT_MAX;
			AABB box1 = nodes[child1].aabb;
			float directCost1 = Perimeter( AABB_Union( box1, boxD ) );
			float area1 = 0.0f;
			if ( leaf1 )
			{
				// Child 1 is a leaf
				// Cost of creating new node and increasing area of node P
				float cost1 = directCost1 + inheritedCost;
				if ( cost1 < bestCost )
				{
					bestSibling = child1;
					bestCost = cost1;
				}
			}
			else
			{
				// Child 1 is an internal node
				area1 = Perimeter( box1 );

				// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
				// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
				// 2. A descendant of child1 could be the sibling with the lower bound cost of
				//       cost1 = inheritedCost + (directCost1 - area1) + areaD
				// This minimum here leads to the minimum of these two costs.
				lowerCost1 = inheritedCost + directCost1 + MinFloat( areaD - area1, 0.0f );
			}

			// Cost of descending into child 2
			float lowerCost2 = FLT_MAX;
			AABB box2 = nodes[child2].aabb;
			float directCost2 = Perimeter( AABB_Union( box2, boxD ) );
			float area2 = 0.0f;
			if ( leaf2 )
			{
				float cost2 = directCost2 + inheritedCost;
				if ( cost2 < bestCost )
				{
					bestSibling = child2;
					bestCost = cost2;
				}
			}
			else
			{
				area2 = Perimeter( box2 );
				lowerCost2 = inheritedCost + directCost2 + MinFloat( areaD - area2, 0.0f );
			}

			if ( leaf1 && leaf2 )
			{
				break;
			}

			// Can the cost possibly be decreased?
			if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
			{
				break;
			}

			if ( lowerCost1 == lowerCost2 && leaf1 == false )
			{
				B2_ASSERT( lowerCost1 < FLT_MAX );
				B2_ASSERT( lowerCost2 < FLT_MAX );

				// No clear choice based on lower bound surface area. This can happen when both
				// children fully contain D. Fall back to node distance.
				Vec2 d1 = Sub( AABB_Center( box1 ), centerD );
				Vec2 d2 = Sub( AABB_Center( box2 ), centerD );
				lowerCost1 = LengthSquared( d1 );
				lowerCost2 = LengthSquared( d2 );
			}

			// Descend
			if ( lowerCost1 < lowerCost2 && leaf1 == false )
			{
				nodeIndex = child1;
				areaBase = area1;
				directCost = directCost1;
			}
			else
			{
				nodeIndex = child2;
				areaBase = area2;
				directCost = directCost2;
			}
		}

		return bestSibling;
	}

	// Swap a child of A with a grandchild under the other child.
	// Example swapping B with G:
	// (A (B (D E) C (F G)) . (A (G) C (F B (D E))
	private static void SwapNodes( DynamicTree tree, int iDown, int iUp )
	{
		Span<TreeNode> nodes = tree.nodes;
		Swap( ref nodes[iDown], ref nodes[iUp] );
		LinkChildren( tree, iDown );
		LinkChildren( tree, iUp );

		if ( IsNodeOrdered( nodes, iDown ) == false || IsNodeOrdered( nodes, iUp ) == false )
		{
			tree.dfsOrdered = false;
		}

		// The sibling of the node that went down holds a different subtree now
		int iC = iDown ^ 1;
		nodes[iC] = MakeInternalNode( nodes, GetLeftChild( nodes[iC] ) );
	}

	// Perform a left or right rotation if node A is imbalanced.
	// Tree: (A (B (D E) C (F G))
	private static void RotateNodes( DynamicTree tree, int iA )
	{
		Span<TreeNode> nodes = tree.nodes;
		ref readonly TreeNode A = ref nodes[iA];
		B2_ASSERT( IsLeaf( A ) == false );

		int iB = GetLeftChild( A );
		int iC = iB + 1;
		ref readonly TreeNode B = ref nodes[iB];
		ref readonly TreeNode C = ref nodes[iC];
		bool leafB = IsLeaf( B );
		bool leafC = IsLeaf( C );
		if ( leafB && leafC )
		{
			return;
		}

		int bestDown = NullIndex;
		int bestUp = NullIndex;
		float bestDelta = 0.0f;

		if ( leafC == false )
		{
			// Swap B with F or G
			int iF = GetLeftChild( C );
			int iG = iF + 1;
			float areaC = Perimeter( C.aabb );

			// B <. F then C (B G)
			float deltaBF = Perimeter( UnionV( B.aabb, nodes[iG].aabb ) ) - areaC;
			if ( deltaBF < bestDelta )
			{
				bestDown = iB;
				bestUp = iF;
				bestDelta = deltaBF;
			}

			// B <. G then C (F B)
			float deltaBG = Perimeter( UnionV( B.aabb, nodes[iF].aabb ) ) - areaC;
			if ( deltaBG < bestDelta )
			{
				bestDown = iB;
				bestUp = iG;
				bestDelta = deltaBG;
			}
		}

		if ( leafB == false )
		{
			// Swap C with D or E
			int iD = GetLeftChild( B );
			int iE = iD + 1;
			float areaB = Perimeter( B.aabb );

			// C <. D then B (C E)
			float deltaCD = Perimeter( UnionV( C.aabb, nodes[iE].aabb ) ) - areaB;
			if ( deltaCD < bestDelta )
			{
				bestDown = iC;
				bestUp = iD;
				bestDelta = deltaCD;
			}

			// C <. E then B (D C)
			float deltaCE = Perimeter( UnionV( C.aabb, nodes[iD].aabb ) ) - areaB;
			if ( deltaCE < bestDelta )
			{
				bestDown = iC;
				bestUp = iE;
				bestDelta = deltaCE;
			}
		}

		if ( bestDown != NullIndex )
		{
			SwapNodes( tree, bestDown, bestUp );
		}
	}

	private static void InsertLeaf( DynamicTree tree, AABB aabb, int proxyId, bool moved, bool shouldRotate )
	{
		ref TreeProxy proxy = ref tree.proxies[proxyId];
		TreeNode leaf = MakeLeafNode( aabb, proxyId, proxy.userData, moved );

		if ( IsEmptyNode( tree.nodes[B2_ROOT_NODE] ) )
		{
			tree.nodes[B2_ROOT_NODE] = leaf;
			proxy.node = B2_ROOT_NODE;
			return;
		}

		int sibling = FindBestSibling( tree, aabb );

		// The sibling's position becomes the new parent. The sibling moves down into a new pair
		// beside the leaf, which puts it below its own children when it is internal.
		int pair = AllocateSiblingPair( tree );
		Span<TreeNode> nodes = tree.nodes;
		Span<int> parents = tree.parents;
		nodes[pair] = nodes[sibling];
		nodes[pair + 1] = leaf;
		parents[pair] = sibling;
		parents[pair + 1] = sibling;
		LinkChildren( tree, pair );
		proxy.node = pair + 1;

		nodes[sibling] = MakeInternalNode( nodes, pair );

		if ( IsNodeOrdered( nodes, sibling ) == false || IsNodeOrdered( nodes, pair ) == false )
		{
			tree.dfsOrdered = false;
		}

		// Walk back up the tree refitting ancestors, the root included.
		int index = sibling;
		while ( index != NullIndex )
		{
			if ( shouldRotate )
			{
				RotateNodes( tree, index );
			}

			nodes[index] = MakeInternalNode( nodes, GetLeftChild( nodes[index] ) );
			index = parents[index];
		}
	}

	private static void RemoveLeaf( DynamicTree tree, int proxyId )
	{
		Span<TreeNode> nodes = tree.nodes;
		Span<int> parents = tree.parents;
		int leaf = tree.proxies[proxyId].node;
		B2_ASSERT( 0 <= leaf && leaf < tree.nodeEnd );
		B2_ASSERT( IsLeaf( nodes[leaf] ) && GetProxyId( nodes[leaf] ) == proxyId );

		if ( leaf == B2_ROOT_NODE )
		{
			nodes[B2_ROOT_NODE] = MakeEmptyNode();
			return;
		}

		// The sibling takes the parent's position and the pair is freed. This keeps the order,
		// the sibling only moves down.
		int parent = parents[leaf];
		nodes[parent] = nodes[leaf ^ 1];
		LinkChildren( tree, parent );
		FreePair( tree, leaf & ~1 );

		// Update ancestors.
		int index = parents[parent];
		while ( index != NullIndex )
		{
			nodes[index] = MakeInternalNode( nodes, GetLeftChild( nodes[index] ) );
			index = parents[index];
		}
	}

	// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
	// the node pool.
	internal static int CreateTreeProxyInternal( DynamicTree tree, AABB aabb, ulong categoryBits, ulong userData, bool markMoved )
	{
		B2_VALIDATE( IsValidAABB( aabb ) );

		int proxyId = AllocateProxy( tree );

		ref TreeProxy proxy = ref tree.proxies[proxyId];
		proxy.categoryBits = categoryBits;
		proxy.userData = userData;

		bool shouldRotate = true;
		InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );

		return proxyId;
	}

	public static int CreateTreeProxy( DynamicTree tree, AABB aabb, ulong categoryBits, ulong userData )
	{
		return CreateTreeProxyInternal( tree, aabb, categoryBits, userData, false );
	}

	public static void DestroyTreeProxy( DynamicTree tree, int proxyId )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );

		RemoveLeaf( tree, proxyId );
		FreeProxy( tree, proxyId );
	}

	public static int DynamicTree_GetProxyCount( DynamicTree tree )
	{
		return tree.proxyCount;
	}

	internal static void DynamicTree_MoveProxyInternal( DynamicTree tree, int proxyId, AABB aabb, bool markMoved )
	{
		B2_VALIDATE( IsValidAABB( aabb ) );
		B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < Huge );
		B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < Huge );
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );

		RemoveLeaf( tree, proxyId );

		bool shouldRotate = false;
		InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );
	}

	public static void DynamicTree_MoveProxy( DynamicTree tree, int proxyId, AABB aabb )
	{
		DynamicTree_MoveProxyInternal( tree, proxyId, aabb, false );
	}

	internal static void DynamicTree_EnlargeProxy( DynamicTree tree, int proxyId, AABB aabb )
	{
		B2_VALIDATE( IsValidAABB( aabb ) );
		B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < Huge );
		B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < Huge );
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );

		Span<TreeNode> nodes = tree.nodes;
		ReadOnlySpan<int> parents = tree.parents;
		int index = tree.proxies[proxyId].node;
		ref TreeNode node = ref nodes[index];
		B2_ASSERT( IsLeaf( node ) );

		// Caller must ensure this
		B2_VALIDATE( AABB_Contains( node.aabb, aabb ) == false );

		node.aabb = aabb;
		node.flagIndex |= B2_MOVED_NODE;

		index = parents[index];
		while ( index != NullIndex )
		{
			node = ref nodes[index];
			bool changed = EnlargeAABB( ref node.aabb, aabb );

			// This is marked to ensure the root is marked in this loop or the one below.
			node.flagIndex |= B2_MOVED_NODE;

			index = parents[index];

			if ( changed == false )
			{
				break;
			}
		}

		// Mark all the way up to the root.
		while ( index != NullIndex )
		{
			node = ref nodes[index];
			if ( ( node.flagIndex & B2_MOVED_NODE ) != 0 )
			{
				// Early out because this ancestor was previously ascended and marked as moved.
				break;
			}

			node.flagIndex |= B2_MOVED_NODE;
			index = parents[index];
		}
	}

	public static void DynamicTree_SetCategoryBits( DynamicTree tree, int proxyId, ulong categoryBits )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
		tree.proxies[proxyId].categoryBits = categoryBits;
	}

	public static ulong DynamicTree_GetCategoryBits( DynamicTree tree, int proxyId )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
		return tree.proxies[proxyId].categoryBits;
	}

	public static int DynamicTree_GetHeight( DynamicTree tree )
	{
		return GetNodeHeight( tree.nodes[B2_ROOT_NODE] );
	}

	// The area ratio is the thing that SAH seeks to minimize. SAH
	// cannot do anything about leaf boxes or the root box. It seeks
	// to minimize the area of all non-root internal nodes. Divide this
	// by the root area to make the metric non-dimensional.
	// So this becomes a meaningful measure of tree quality.
	public static float DynamicTree_GetAreaRatio( DynamicTree tree )
	{
		if ( tree.proxyCount == 0 )
		{
			return 0.0f;
		}

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		float rootArea = Perimeter( nodes[B2_ROOT_NODE].aabb );
		if ( rootArea <= 0.0f )
		{
			return 0.0f;
		}

		// Free nodes and the empty node are leaf tagged.
		float internalArea = 0.0f;
		int nodeEnd = tree.nodeEnd;
		for ( int i = 2; i < nodeEnd; ++i )
		{
			if ( IsLeaf( nodes[i] ) == false )
			{
				internalArea += Perimeter( nodes[i].aabb );
			}
		}

		return internalArea / rootArea;
	}

	public static AABB DynamicTree_GetRootBounds( DynamicTree tree )
	{
		if ( tree.proxyCount == 0 )
		{
			return new AABB { lowerBound = Vec2_zero, upperBound = Vec2_zero };
		}

		return tree.nodes[B2_ROOT_NODE].aabb;
	}

#if BOX2D_VALIDATE

	private static int ValidateSubtree( DynamicTree tree, int nodeIndex, ref int leafCount )
	{
		B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree.nodeEnd );
		ref readonly TreeNode node = ref tree.nodes[nodeIndex];
		B2_ASSERT( IsEmptyNode( node ) == false );

		if ( IsLeaf( node ) )
		{
			int proxyId = GetProxyId( node );
			B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
			B2_ASSERT( tree.proxies[proxyId].node == nodeIndex );
			B2_ASSERT( (int)tree.proxies[proxyId].userData == node.shapeIndex );
			leafCount += 1;
			return 0;
		}

		int pair = GetLeftChild( node );
		B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree.nodeEnd );
		B2_ASSERT( tree.parents[pair] == nodeIndex );
		B2_ASSERT( tree.parents[pair + 1] == nodeIndex );
		B2_ASSERT( tree.dfsOrdered == false || nodeIndex < pair );

		ref readonly TreeNode c1 = ref tree.nodes[pair];
		ref readonly TreeNode c2 = ref tree.nodes[pair + 1];
		B2_ASSERT( AABB_Contains( node.aabb, c1.aabb ) );
		B2_ASSERT( AABB_Contains( node.aabb, c2.aabb ) );
		B2_ASSERT( IsNodeMoved( node ) == ( IsNodeMoved( c1 ) || IsNodeMoved( c2 ) ) );

		// A bad tree can stack overflow, but that is validation on its own.
		int height1 = ValidateSubtree( tree, pair, ref leafCount );
		int height2 = ValidateSubtree( tree, pair + 1, ref leafCount );
		int height = 1 + MaxInt( height1, height2 );
		B2_ASSERT( node.height == height );
		return height;
	}

#endif

	public static void DynamicTree_Validate( DynamicTree tree )
	{
#if BOX2D_VALIDATE
		B2_ASSERT( 2 <= tree.nodeEnd && tree.nodeEnd <= tree.nodeCapacity );
		B2_ASSERT( ( tree.nodeEnd & 1 ) == 0 );
		B2_ASSERT( tree.parents[B2_ROOT_NODE] == NullIndex );
		B2_ASSERT( IsEmptyNode( tree.nodes[B2_ROOT_NODE + 1] ) );

		int freePairCount = 0;
		int pair = tree.pairFreeList;
		while ( pair != NullIndex )
		{
			B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree.nodeEnd );
			B2_ASSERT( IsEmptyNode( tree.nodes[pair] ) );
			B2_ASSERT( IsEmptyNode( tree.nodes[pair + 1] ) );
			pair = tree.parents[pair];
			++freePairCount;
			B2_ASSERT( 2 * freePairCount < tree.nodeEnd );
		}

		// Validate proxy free list.
		int freeProxyCount = 0;
		int freeIndex = tree.proxyFreeList;
		while ( freeIndex != NullIndex )
		{
			B2_ASSERT( 0 <= freeIndex && freeIndex < tree.proxyCapacity );
			B2_ASSERT( tree.proxies[freeIndex].node == NullIndex );
			freeIndex = tree.proxies[freeIndex].next;
			++freeProxyCount;
		}
		B2_ASSERT( tree.proxyCount + freeProxyCount == tree.proxyCapacity );

		B2_ASSERT( tree.nodeEnd == 2 * MaxInt( tree.proxyCount, 1 ) + 2 * freePairCount );

		if ( tree.proxyCount == 0 )
		{
			B2_ASSERT( IsEmptyNode( tree.nodes[B2_ROOT_NODE] ) );
			return;
		}

		int leafCount = 0;
		ValidateSubtree( tree, B2_ROOT_NODE, ref leafCount );
		B2_ASSERT( leafCount == tree.proxyCount );

#else
#endif
	}

	internal static void DynamicTree_ValidateNoMoved( DynamicTree tree )
	{
#if BOX2D_VALIDATE
		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		int nodeEnd = tree.nodeEnd;
		for ( int i = 0; i < nodeEnd; ++i )
		{
			B2_ASSERT( IsNodeMoved( nodes[i] ) == false );
		}
#else
#endif
	}

	public static int DynamicTree_GetByteCount( DynamicTree tree )
	{
		int size = Unsafe.SizeOf<DynamicTree>();
		size += tree.nodeCapacity * Unsafe.SizeOf<TreeNode>();
		size += tree.nodeCapacity * Unsafe.SizeOf<int>();
		size += tree.proxyCapacity * Unsafe.SizeOf<TreeProxy>();
		size += tree.swapNodes == null ? 0 : tree.nodeCapacity * Unsafe.SizeOf<TreeNode>();

		// leafIndices
		size += tree.rebuildCapacity * Unsafe.SizeOf<int>();
		// leafNodes
		size += tree.rebuildCapacity * Unsafe.SizeOf<TreeNode>();
		// leafBoxes
		size += tree.rebuildCapacity * Unsafe.SizeOf<AABB>();
		// leafCenters
		size += tree.rebuildCapacity * Unsafe.SizeOf<Vec2>();
		// binIndices
		size += tree.rebuildCapacity * Unsafe.SizeOf<int>();

		return size;
	}

	public static ulong DynamicTree_GetUserData( DynamicTree tree, int proxyId )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
		return tree.proxies[proxyId].userData;
	}

	public static AABB DynamicTree_GetProxyBounds( DynamicTree tree, int proxyId )
	{
		B2_ASSERT( 0 <= proxyId && proxyId < tree.proxyCapacity );
		int nodeIndex = tree.proxies[proxyId].node;
		B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree.nodeEnd );
		return tree.nodes[nodeIndex].aabb;
	}

	public static TreeStats DynamicTree_OverlapAABB<T>( DynamicTree tree, AABB aabb, ulong maskBits, ref T callback ) where T : ITreeQueryCallback, allows ref struct
	{
		TreeStats result = default;

		if ( tree.proxyCount == 0 )
		{
			return result;
		}

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetRootPair( nodes );

		AABB boxv = LoadAABBV( ref aabb );

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			result.nodeVisits += 1;

			for ( int i = 0; i < 2; ++i )
			{
				ref readonly TreeNode node = ref nodes[pair + i];
				if ( OverlapNode( boxv, node ) )
				{
					if ( IsLeaf( node ) )
					{
						// callback to user code with proxy id
						int proxyId = GetProxyId( node );
						ref readonly TreeProxy proxy = ref tree.proxies[proxyId];
						if ( ( proxy.categoryBits & maskBits ) != 0 )
						{
							bool proceed = callback.Invoke( proxyId, proxy.userData );
							result.leafVisits += 1;

							if ( proceed == false )
							{
								return result;
							}
						}
					}
					else
					{
						if ( stackCount < B2_TREE_STACK_SIZE - 1 )
						{
							stack[stackCount++] = GetLeftChild( node );
						}
						else
						{
							B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
						}
					}
				}
			}
		}

		return result;
	}

	// This ignores the category bits. Useful for debug draw.
	public static TreeStats DynamicTree_OverlapAABBAll<T>( DynamicTree tree, AABB aabb, ref T callback ) where T : ITreeQueryCallback, allows ref struct
	{
		TreeStats result = default;

		if ( tree.proxyCount == 0 )
		{
			return result;
		}

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetRootPair( nodes );

		AABB boxv = LoadAABBV( ref aabb );

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			result.nodeVisits += 1;

			for ( int i = 0; i < 2; ++i )
			{
				ref readonly TreeNode node = ref nodes[pair + i];
				if ( OverlapNode( boxv, node ) )
				{
					if ( IsLeaf( node ) )
					{
						// callback to user code with proxy id
						int proxyId = GetProxyId( node );
						ref readonly TreeProxy proxy = ref tree.proxies[proxyId];
						bool proceed = callback.Invoke( proxyId, proxy.userData );
						result.leafVisits += 1;

						if ( proceed == false )
						{
							return result;
						}
					}
					else
					{
						if ( stackCount < B2_TREE_STACK_SIZE - 1 )
						{
							stack[stackCount++] = GetLeftChild( node );
						}
						else
						{
							B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
						}
					}
				}
			}
		}

		return result;
	}

	// A lot of optimization work went into this. It beats the slab test by a significant margin.
	// It is faster than having category bits in the nodes because of the cache line friendly node
	// size (64 bytes).
	public static TreeStats DynamicTree_CastRay<T>( DynamicTree tree, in RayCastInput input, ulong maskBits, ref T callback ) where T : ITreeRayCastCallback, allows ref struct
	{
		TreeStats result = default;

		if ( tree.proxyCount == 0 )
		{
			return result;
		}

		Vec2 p1 = input.origin;
		Vec2 d = input.translation;

		Vec2 r = Normalize( d );

		// v is perpendicular to the segment.
		Vec2 v = CrossSV( 1.0f, r );
		Vec2 abs_v = Abs( v );

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		float maxFraction = input.maxFraction;

		Vec2 p2 = MulAdd( p1, maxFraction, d );

		// Build a bounding box for the segment.
		AABB segmentAABB = new AABB { lowerBound = Min( p1, p2 ), upperBound = Max( p1, p2 ) };

		AABB boxv = LoadAABBV( ref segmentAABB );

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetRootPair( nodes );

		RayCastInput subInput = input;

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			result.nodeVisits += 1;

			FixedArray2<int> hit = default;
			FixedArray2<bool> isLeaf = default;
			int hitCount = 0;
			for ( int i = 0; i < 2; ++i )
			{
				ref readonly TreeNode node = ref nodes[pair + i];
				if ( OverlapNode( boxv, node ) == false )
				{
					continue;
				}

				// Separating axis for segment (Gino, p80).
				// |dot(v, p1 - c)| > dot(|v|, h)
				AABB nodeAABB = node.aabb;
				Vec2 c = AABB_Center( nodeAABB );
				Vec2 h = AABB_Extents( nodeAABB );
				float term1 = AbsFloat( Dot( v, Sub( p1, c ) ) );
				float term2 = Dot( abs_v, h );
				if ( term2 < term1 )
				{
					continue;
				}

				isLeaf[hitCount] = IsLeaf( node );
				hit[hitCount] = pair + i;
				hitCount += 1;
			}

			if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
			{
				Vec2 center1 = AABB_Center( nodes[hit[0]].aabb );
				Vec2 center2 = AABB_Center( nodes[hit[1]].aabb );
				float d1 = DistanceSquared( center1, p1 );
				float d2 = DistanceSquared( center2, p1 );

				// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
				if ( d1 < d2 )
				{
					Swap( ref hit[0], ref hit[1] );
				}
			}

			for ( int i = 0; i < hitCount; ++i )
			{
				if ( isLeaf[i] )
				{
					int proxyId = GetProxyId( nodes[hit[i]] );
					ref readonly TreeProxy proxy = ref tree.proxies[proxyId];
					if ( ( proxy.categoryBits & maskBits ) == 0 )
					{
						continue;
					}

					subInput.maxFraction = maxFraction;

					float value = callback.Invoke( subInput, proxyId, proxy.userData );
					result.leafVisits += 1;

					// The user may return -1 to indicate this shape should be skipped

					if ( value == 0.0f )
					{
						// The client has terminated the ray cast.
						return result;
					}

					if ( 0.0f < value && value <= maxFraction )
					{
						// Update segment bounding box.
						maxFraction = value;
						p2 = MulAdd( p1, maxFraction, d );
						segmentAABB.lowerBound = Min( p1, p2 );
						segmentAABB.upperBound = Max( p1, p2 );

						boxv = LoadAABBV( ref segmentAABB );
					}
				}
				else
				{
					if ( stackCount < B2_TREE_STACK_SIZE - 1 )
					{
						stack[stackCount++] = GetLeftChild( nodes[hit[i]] );
					}
					else
					{
						B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
					}
				}
			}
		}

		return result;
	}

	// Follows structure of ray cast with small tweaks to handle a swept box.
	public static TreeStats DynamicTree_CastBox<T>( DynamicTree tree, in BoxCastInput input, ulong maskBits, ref T callback ) where T : ITreeBoxCastCallback, allows ref struct
	{
		TreeStats result = default;

		if ( tree.proxyCount == 0 )
		{
			return result;
		}

		AABB originAABB = input.box;

		Vec2 p1 = AABB_Center( originAABB );
		Vec2 extension = AABB_Extents( originAABB );

		// v is perpendicular to the segment.
		Vec2 r = input.translation;
		Vec2 v = CrossSV( 1.0f, r );
		Vec2 abs_v = Abs( v );

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)

		float maxFraction = input.maxFraction;

		// Build total box for the cast
		Vec2 t = MulSV( maxFraction, input.translation );
		AABB totalAABB = new AABB { lowerBound = Min( originAABB.lowerBound, Add( originAABB.lowerBound, t ) ), upperBound = Max( originAABB.upperBound, Add( originAABB.upperBound, t ) ) };

		AABB boxv = LoadAABBV( ref totalAABB );

		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetRootPair( nodes );

		BoxCastInput subInput = input;

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			result.nodeVisits += 1;

			FixedArray2<int> hit = default;
			FixedArray2<bool> isLeaf = default;
			int hitCount = 0;
			for ( int i = 0; i < 2; ++i )
			{
				ref readonly TreeNode node = ref nodes[pair + i];
				if ( OverlapNode( boxv, node ) == false )
				{
					continue;
				}

				// Separating axis for segment (Gino, p80).
				// |dot(v, p1 - c)| > dot(|v|, h)
				// radius extension is added to the node in this case
				AABB nodeAABB = node.aabb;
				Vec2 c = AABB_Center( nodeAABB );
				Vec2 h = Add( AABB_Extents( nodeAABB ), extension );
				float term1 = AbsFloat( Dot( v, Sub( p1, c ) ) );
				float term2 = Dot( abs_v, h );
				if ( term2 < term1 )
				{
					continue;
				}

				isLeaf[hitCount] = IsLeaf( node );
				hit[hitCount] = pair + i;
				hitCount += 1;
			}

			if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
			{
				Vec2 center1 = AABB_Center( nodes[hit[0]].aabb );
				Vec2 center2 = AABB_Center( nodes[hit[1]].aabb );
				float d1 = DistanceSquared( center1, p1 );
				float d2 = DistanceSquared( center2, p1 );

				// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
				if ( d1 < d2 )
				{
					Swap( ref hit[0], ref hit[1] );
				}
			}

			for ( int i = 0; i < hitCount; ++i )
			{
				if ( isLeaf[i] )
				{
					int proxyId = GetProxyId( nodes[hit[i]] );
					ref readonly TreeProxy proxy = ref tree.proxies[proxyId];
					if ( ( proxy.categoryBits & maskBits ) == 0 )
					{
						continue;
					}

					subInput.maxFraction = maxFraction;

					float value = callback.Invoke( subInput, proxyId, proxy.userData );
					result.leafVisits += 1;

					// The user may return -1 to indicate this shape should be skipped

					if ( value == 0.0f )
					{
						// The client has terminated the ray cast.
						return result;
					}

					if ( 0.0f < value && value < maxFraction )
					{
						// Update cast bounding box.
						maxFraction = value;
						t = MulSV( maxFraction, input.translation );
						totalAABB.lowerBound = Min( originAABB.lowerBound, Add( originAABB.lowerBound, t ) );
						totalAABB.upperBound = Max( originAABB.upperBound, Add( originAABB.upperBound, t ) );
						boxv = LoadAABBV( ref totalAABB );
					}
				}
				else
				{
					if ( stackCount < B2_TREE_STACK_SIZE - 1 )
					{
						stack[stackCount++] = GetLeftChild( nodes[hit[i]] );
					}
					else
					{
						B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
					}
				}
			}
		}

		return result;
	}

	// Median split == 0, Surface area heuristic == 1
	private const int B2_TREE_HEURISTIC = 0;
#if true
	// Median split heuristic
	private static int PartitionMid( Span<int> indices, Span<Vec2> centers, int count )
	{
		// Handle trivial case
		if ( count <= 2 )
		{
			return count / 2;
		}

		Vec2 lowerBound = centers[0];
		Vec2 upperBound = centers[0];

		for ( int i = 1; i < count; ++i )
		{
			lowerBound = Min( lowerBound, centers[i] );
			upperBound = Max( upperBound, centers[i] );
		}

		Vec2 d = Sub( upperBound, lowerBound );
		Vec2 c = new Vec2( 0.5f * ( lowerBound.x + upperBound.x ), 0.5f * ( lowerBound.y + upperBound.y ) );

		// Partition longest axis using the Hoare partition scheme
		// https://en.wikipedia.org/wiki/Quicksort
		// https://nicholasvadivelu.com/2021/01/11/array-partition/
		int i1 = 0, i2 = count;
		if ( d.x > d.y )
		{
			float pivot = c.x;

			while ( i1 < i2 )
			{
				while ( i1 < i2 && centers[i1].x < pivot )
				{
					i1 += 1;
				};

				while ( i1 < i2 && centers[i2 - 1].x >= pivot )
				{
					i2 -= 1;
				};

				if ( i1 < i2 )
				{
					// Swap indices
					{
						int temp = indices[i1];
						indices[i1] = indices[i2 - 1];
						indices[i2 - 1] = temp;
					}

					// Swap centers
					{
						Vec2 temp = centers[i1];
						centers[i1] = centers[i2 - 1];
						centers[i2 - 1] = temp;
					}

					i1 += 1;
					i2 -= 1;
				}
			}
		}
		else
		{
			float pivot = c.y;

			while ( i1 < i2 )
			{
				while ( i1 < i2 && centers[i1].y < pivot )
				{
					i1 += 1;
				};

				while ( i1 < i2 && centers[i2 - 1].y >= pivot )
				{
					i2 -= 1;
				};

				if ( i1 < i2 )
				{
					// Swap indices
					{
						int temp = indices[i1];
						indices[i1] = indices[i2 - 1];
						indices[i2 - 1] = temp;
					}

					// Swap centers
					{
						Vec2 temp = centers[i1];
						centers[i1] = centers[i2 - 1];
						centers[i2 - 1] = temp;
					}

					i1 += 1;
					i2 -= 1;
				}
			}
		}
		B2_ASSERT( i1 == i2 );

		if ( i1 > 0 && i1 < count )
		{
			return i1;
		}

		return count / 2;
	}

#else

	private const int B2_BIN_COUNT = 8;
	internal struct TreeBin
	{
		public AABB aabb;
		public int count;
	}

	internal struct TreePlane
	{
		public AABB leftAABB;
		public AABB rightAABB;
		public int leftCount;
		public int rightCount;
	}

	// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
	// Returns the left child count
	private static int PartitionSAH( Span<int> indices, Span<int> binIndices, Span<AABB> boxes, int count )
	{
		B2_ASSERT( count > 0 );

		Span<TreeBin> bins = stackalloc TreeBin[B2_BIN_COUNT];
		Span<TreePlane> planes = stackalloc TreePlane[B2_BIN_COUNT - 1];

		Vec2 center = AABB_Center( boxes[0] );
		AABB centroidAABB = default;
		centroidAABB.lowerBound = center;
		centroidAABB.upperBound = center;

		for ( int i = 1; i < count; ++i )
		{
			center = AABB_Center( boxes[i] );
			centroidAABB.lowerBound = Min( centroidAABB.lowerBound, center );
			centroidAABB.upperBound = Max( centroidAABB.upperBound, center );
		}

		Vec2 d = Sub( centroidAABB.upperBound, centroidAABB.lowerBound );

		// Find longest axis
		int axisIndex;
		float invD;
		if ( d.x > d.y )
		{
			axisIndex = 0;
			invD = d.x;
		}
		else
		{
			axisIndex = 1;
			invD = d.y;
		}

		invD = invD > 0.0f ? 1.0f / invD : 0.0f;

		// Initialize bin bounds and count
		for ( int i = 0; i < B2_BIN_COUNT; ++i )
		{
			bins[i].aabb.lowerBound = new Vec2( FLT_MAX, FLT_MAX );
			bins[i].aabb.upperBound = new Vec2( -FLT_MAX, -FLT_MAX );
			bins[i].count = 0;
		}

		// Assign boxes to bins and compute bin boxes
		// TODO_ERIN optimize
		float binCount = B2_BIN_COUNT;
		FixedArray2<float> lowerBoundArray = default;
		lowerBoundArray[0] = centroidAABB.lowerBound.x;
		lowerBoundArray[1] = centroidAABB.lowerBound.y;
		float minC = lowerBoundArray[axisIndex];
		for ( int i = 0; i < count; ++i )
		{
			Vec2 c = AABB_Center( boxes[i] );
			FixedArray2<float> cArray = default;
			cArray[0] = c.x;
			cArray[1] = c.y;
			int binIndex = (int)( binCount * ( cArray[axisIndex] - minC ) * invD );
			binIndex = ClampInt( binIndex, 0, B2_BIN_COUNT - 1 );
			binIndices[i] = binIndex;
			bins[binIndex].count += 1;
			bins[binIndex].aabb = AABB_Union( bins[binIndex].aabb, boxes[i] );
		}

		int planeCount = B2_BIN_COUNT - 1;

		// Prepare all the left planes, candidates for left child
		planes[0].leftCount = bins[0].count;
		planes[0].leftAABB = bins[0].aabb;
		for ( int i = 1; i < planeCount; ++i )
		{
			planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
			planes[i].leftAABB = AABB_Union( planes[i - 1].leftAABB, bins[i].aabb );
		}

		// Prepare all the right planes, candidates for right child
		planes[planeCount - 1].rightCount = bins[planeCount].count;
		planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
		for ( int i = planeCount - 2; i >= 0; --i )
		{
			planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
			planes[i].rightAABB = AABB_Union( planes[i + 1].rightAABB, bins[i + 1].aabb );
		}

		// Find best split to minimize SAH
		float minCost = FLT_MAX;
		int bestPlane = 0;
		for ( int i = 0; i < planeCount; ++i )
		{
			float leftArea = Perimeter( planes[i].leftAABB );
			float rightArea = Perimeter( planes[i].rightAABB );
			int leftCount = planes[i].leftCount;
			int rightCount = planes[i].rightCount;

			float cost = leftCount * leftArea + rightCount * rightArea;
			if ( cost < minCost )
			{
				bestPlane = i;
				minCost = cost;
			}
		}

		// Partition node indices and boxes using the Hoare partition scheme
		// https://en.wikipedia.org/wiki/Quicksort
		// https://nicholasvadivelu.com/2021/01/11/array-partition/
		int i1 = 0, i2 = count;
		while ( i1 < i2 )
		{
			while ( i1 < i2 && binIndices[i1] < bestPlane )
			{
				i1 += 1;
			};

			while ( i1 < i2 && binIndices[i2 - 1] >= bestPlane )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap boxes
				{
					AABB temp = boxes[i1];
					boxes[i1] = boxes[i2 - 1];
					boxes[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
		B2_ASSERT( i1 == i2 );

		if ( i1 > 0 && i1 < count )
		{
			return i1;
		}
		else
		{
			return count / 2;
		}
	}

#endif

	// Temporary data used to track the rebuild of a tree node.
	internal struct RebuildItem
	{
		// Where this node is written and the pair its children go in.
		public int nodeIndex;
		public int pair;
		public int childCount;

		// Leaf indices
		public int startIndex;
		public int splitIndex;
		public int endIndex;
	}

	internal struct CopyItem
	{
		public int oldPair;
		public int newIndex;
	}

	// Bump allocate a pair of sibling nodes. Returns index to the first one.
	internal static int BumpPair( DynamicTree tree, int parent )
	{
		int pair = tree.nodeEnd;
		B2_ASSERT( pair + 2 <= tree.nodeCapacity );
		tree.nodeEnd += 2;
		tree.parents[pair] = parent;
		tree.parents[pair + 1] = parent;
		return pair;
	}

	internal static void SetLeftChild( ref TreeNode node, int pair )
	{
		node.flagIndex = ( node.flagIndex & ~B2_NODE_INDEX_MASK ) | (uint)pair;
	}

	// Copy a retained subtree from the old array into the rebuilt DFS array using
	// a bump allocator.
	private static void CopySubtree( DynamicTree tree, TreeNode node, int newIndex )
	{
		ReadOnlySpan<TreeNode> oldNodes = tree.nodes;
		Span<TreeNode> newNodes = tree.swapNodes;
		Span<TreeProxy> proxies = tree.proxies;
		if ( IsLeaf( ref node ) )
		{
			newNodes[newIndex] = node;
			proxies[GetProxyId( ref node )].node = newIndex;
			return;
		}

		Span<CopyItem> stack = stackalloc CopyItem[B2_TREE_STACK_SIZE];
		int stackCount = 0;

		int oldPair = GetLeftChild( ref node );
		int newPair = BumpPair( tree, newIndex );
		SetLeftChild( ref node, newPair );

		// Copy the subtree root. This can be a left or right sibling (even or odd).
		newNodes[newIndex] = node;

		for ( ;; )
		{
			Span<TreeNode> pair = newNodes.Slice( newPair );
			// Copy the siblings.
			pair[0] = oldNodes[oldPair];
			pair[1] = oldNodes[oldPair + 1];

			if ( IsLeaf( pair[1] ) )
			{
				// Hook up proxy on right sibling.
				int proxyId = GetProxyId( pair[1] );
				proxies[proxyId].node = newPair + 1;
			}
			else
			{
				// Push the left child of the right sibling.
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				int leftChild = GetLeftChild( pair[1] );
				stack[stackCount++] = new CopyItem { oldPair = leftChild, newIndex = newPair + 1 };
			}

			if ( IsLeaf( pair[0] ) == false )
			{
				// Descend into left subtree.
				int leftIndex = newPair;
				oldPair = GetLeftChild( pair[0] );
				newPair = BumpPair( tree, leftIndex );
				SetLeftChild( ref newNodes[leftIndex], newPair );
				continue;
			}

			// Hook up proxy on left sibling.
			proxies[GetProxyId( pair[0] )].node = newPair;

			if ( stackCount == 0 )
			{
				break;
			}

			// Descend a right subtree that was previously pushed.
			CopyItem item = stack[--stackCount];
			oldPair = item.oldPair;
			newPair = BumpPair( tree, item.newIndex );
			SetLeftChild( ref newNodes[item.newIndex], newPair );
		}
	}

	private static void PlaceLeaf( DynamicTree tree, TreeNode leaf, int newIndex )
	{
		if ( IsLeaf( ref leaf ) )
		{
			tree.swapNodes![newIndex] = leaf;
			tree.proxies[GetProxyId( ref leaf )].node = newIndex;
		}
		else
		{
			CopySubtree( tree, leaf, newIndex );
		}
	}

	private static void BuildTree( DynamicTree tree, int leafCount )
	{
		Span<TreeNode> nodes = tree.swapNodes;
		ReadOnlySpan<TreeNode> leaves = tree.leafNodes;
		Span<int> leafIndices = tree.leafIndices;
#if true
		Span<Vec2> leafCenters = tree.leafCenters;
#else
		Span<AABB> leafBoxes = tree.leafBoxes;
		Span<int> binIndices = tree.binIndices;
#endif

		// Bump allocation into the swap nodes.
		tree.nodeEnd = 2;
		nodes[B2_ROOT_NODE + 1] = MakeEmptyNode();
		tree.parents[B2_ROOT_NODE] = NullIndex;
		tree.parents[B2_ROOT_NODE + 1] = NullIndex;

		if ( leafCount == 1 )
		{
			PlaceLeaf( tree, leaves[leafIndices[0]], B2_ROOT_NODE );
			return;
		}

		Span<RebuildItem> stack = stackalloc RebuildItem[B2_TREE_STACK_SIZE];
		int top = 0;

		stack[0].nodeIndex = B2_ROOT_NODE;
		stack[0].pair = BumpPair( tree, B2_ROOT_NODE );
		stack[0].childCount = -1;
		stack[0].startIndex = 0;
		stack[0].endIndex = leafCount;
#if true
		stack[0].splitIndex = PartitionMid( leafIndices, leafCenters, leafCount );
#else
		stack[0].splitIndex = PartitionSAH( leafIndices, binIndices, leafBoxes, leafCount );
#endif

		while ( true )
		{
			ref RebuildItem item = ref stack[top];
			item.childCount += 1;

			if ( item.childCount == 2 )
			{
				// Both children written, so the parent node can be finalized.
				nodes[item.nodeIndex] = MakeInternalNode( nodes, item.pair );
				if ( top == 0 )
				{
					break;
				}

				top -= 1;
				continue;
			}

			int slot = item.childCount;
			int startIndex = slot == 0 ? item.startIndex : item.splitIndex;
			int endIndex = slot == 0 ? item.splitIndex : item.endIndex;
			int count = endIndex - startIndex;
			B2_ASSERT( count > 0 );

			int nodeIndex = item.pair + slot;
			if ( count == 1 )
			{
				PlaceLeaf( tree, leaves[leafIndices[startIndex]], nodeIndex );
				continue;
			}

			// todo fail gracefully if the stack runs out in release
			B2_ASSERT( top < B2_TREE_STACK_SIZE - 1 );

			top += 1;
			ref RebuildItem newItem = ref stack[top];
			newItem.nodeIndex = nodeIndex;
			newItem.pair = BumpPair( tree, nodeIndex );
			newItem.childCount = -1;
			newItem.startIndex = startIndex;
			newItem.endIndex = endIndex;
			newItem.splitIndex = startIndex;
#if true
			newItem.splitIndex += PartitionMid( leafIndices.Slice( startIndex ), leafCenters.Slice( startIndex ), count );
#else
			newItem.splitIndex += PartitionSAH( leafIndices.Slice( startIndex ), binIndices.Slice( startIndex ), leafBoxes.Slice( startIndex ), count );
#endif
		}
	}

	// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
	// refitting much faster. This is done async with threading, so the cost is hidden.
	// Not safe to access tree during this operation.
	internal static int DynamicTree_Rebuild( DynamicTree tree, bool fullBuild )
	{
		int proxyCount = tree.proxyCount;
		if ( proxyCount == 0 )
		{
			return 0;
		}

		// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		ref readonly TreeNode root = ref nodes[B2_ROOT_NODE];
		if ( fullBuild == false && IsNodeMoved( root ) == false && tree.dfsOrdered )
		{
			return 0;
		}

		if ( tree.swapNodes == null )
		{
			tree.swapNodes = Alloc<TreeNode>( tree.nodeCapacity );
		}

		if ( proxyCount > tree.rebuildCapacity )
		{
			int oldCapacity = tree.rebuildCapacity;
			int newCapacity = proxyCount + proxyCount / 2;

			Free( tree.leafIndices, oldCapacity );
			tree.leafIndices = Alloc<int>( newCapacity );

			Free( tree.leafNodes, oldCapacity );
			tree.leafNodes = Alloc<TreeNode>( newCapacity );

#if true
			Free( tree.leafCenters, oldCapacity );
			tree.leafCenters = Alloc<Vec2>( newCapacity );
#else
			Free( tree.leafBoxes, oldCapacity );
			tree.leafBoxes = Alloc<AABB>( newCapacity );
			Free( tree.binIndices, oldCapacity );
			tree.binIndices = Alloc<int>( newCapacity );
#endif
			tree.rebuildCapacity = newCapacity;
		}
		Span<int> leafIndices = tree.leafIndices;
		Span<TreeNode> leaves = tree.leafNodes;
#if true
		Span<Vec2> leafCenters = tree.leafCenters;
#else
		Span<AABB> leafBoxes = tree.leafBoxes;
#endif

		// Gather build leaves. A marked internal node is descended and abandoned in the old array.
		// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
		// on an unordered array, is the one build leaf.
		int leafCount = 0;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		if ( IsLeaf( root ) == false && ( fullBuild || IsNodeMoved( root ) ) )
		{
			stack[stackCount++] = GetLeftChild( root );
		}
		else
		{
			TreeNode node = root;
			node.flagIndex &= ~B2_MOVED_NODE;
			leafIndices[0] = 0;
			leaves[0] = node;
#if true
			leafCenters[0] = AABB_Center( node.aabb );
#else
			leafBoxes[0] = node.aabb;
#endif
			leafCount = 1;
		}

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			for ( int i = 0; i < 2; ++i )
			{
				TreeNode node = nodes[pair + i];
				if ( IsLeaf( ref node ) == false && ( fullBuild || IsNodeMoved( ref node ) ) )
				{
					// todo fail gracefully if the stack runs out in release
					B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
					stack[stackCount++] = GetLeftChild( ref node );
					continue;
				}

				node.flagIndex &= ~B2_MOVED_NODE;
				leafIndices[leafCount] = leafCount;
				leaves[leafCount] = node;
#if true
				leafCenters[leafCount] = AABB_Center( node.aabb );
#else
				leafBoxes[leafCount] = node.aabb;
#endif
				leafCount += 1;
			}
		}

		B2_ASSERT( 0 < leafCount && leafCount <= proxyCount );

		BuildTree( tree, leafCount );

		// The spare is now the tree and the old array is the spare. The build is dense, so the
		// stale tail of the old array sits above the end and is never read.
		Swap( ref tree.nodes, ref tree.swapNodes );
		tree.pairFreeList = NullIndex;
		tree.dfsOrdered = true;

		DynamicTree_Validate( tree );
		DynamicTree_ValidateNoMoved( tree );

		return leafCount;
	}

	// Set the moved flag on the ancestors of the proxy. Serial use case.
	internal static void DynamicTree_MarkProxyMovedSerial( DynamicTree tree, int proxyId )
	{
		B2_VALIDATE( 0 <= proxyId && proxyId < tree.proxyCapacity );

		Span<TreeNode> nodes = tree.nodes;
		ReadOnlySpan<int> parents = tree.parents;
		int index = tree.proxies[proxyId].node;
		B2_VALIDATE( 0 <= index && index < tree.nodeEnd );
		B2_VALIDATE( IsLeaf( nodes[index] ) );

		while ( index != NullIndex )
		{
			nodes[index].flagIndex |= B2_MOVED_NODE;
			index = parents[index];
		}
	}

	// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.
	internal static void DynamicTree_MarkProxyMoved( DynamicTree tree, int proxyId, AABB aabb )
	{
		B2_VALIDATE( 0 <= proxyId && proxyId < tree.proxyCapacity );

		Span<TreeNode> nodes = tree.nodes;
		ReadOnlySpan<int> parents = tree.parents;
		int index = tree.proxies[proxyId].node;
		B2_VALIDATE( 0 <= index && index < tree.nodeEnd );

		ref TreeNode node = ref nodes[index];
		B2_VALIDATE( IsLeaf( node ) );
		B2_VALIDATE( AABB_Contains( node.aabb, aabb ) == false );

		// This is not raced since it is the leaf.
		node.aabb = aabb;
		node.flagIndex |= B2_MOVED_NODE;

		index = parents[index];
		while ( index != NullIndex )
		{
			node = ref nodes[index];

			// Read first to avoid the FetchOr if possible.
			if ( ( AtomicLoadU32Raw( ref node.flagIndex ) & B2_MOVED_NODE ) != 0 )
			{
				break;
			}

			uint previousFlags = AtomicFetchOrU32( ref node.flagIndex, B2_MOVED_NODE );
			if ( ( previousFlags & B2_MOVED_NODE ) != 0 )
			{
				// Ancestor already visited.
				break;
			}

			index = parents[index];
		}
	}

	// Clear the moved flags from the entire tree.
	internal static void DynamicTree_ClearMoved( DynamicTree tree )
	{
		Span<TreeNode> nodes = tree.nodes;
		ref TreeNode root = ref nodes[B2_ROOT_NODE];
		if ( IsNodeMoved( root ) == false )
		{
			return;
		}

		root.flagIndex &= ~B2_MOVED_NODE;
		if ( IsLeaf( root ) )
		{
			return;
		}

		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetLeftChild( root );

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			for ( int i = 0; i < 2; ++i )
			{
				ref TreeNode node = ref nodes[pair + i];
				if ( ( node.flagIndex & B2_MOVED_NODE ) != 0 )
				{
					node.flagIndex &= ~B2_MOVED_NODE;
					if ( IsLeaf( node ) == false )
					{
						if ( stackCount < B2_TREE_STACK_SIZE )
						{
							stack[stackCount++] = GetLeftChild( node );
						}
						else
						{
							// Bad stuff will happen if the moved flags don't get cleared.
							B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
						}
					}
				}
			}
		}
	}

	internal static int DynamicTree_GatherMovedProxies( DynamicTree tree, Span<int> proxyIds )
	{
		ReadOnlySpan<TreeNode> nodes = tree.nodes;
		ref readonly TreeNode root = ref nodes[B2_ROOT_NODE];
		if ( IsNodeMoved( root ) == false )
		{
			return 0;
		}

		if ( IsLeaf( root ) )
		{
			proxyIds[0] = GetProxyId( root );
			return 1;
		}

		int count = 0;
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		stack[stackCount++] = GetLeftChild( root );

		while ( stackCount > 0 )
		{
			int pair = stack[--stackCount];
			for ( int i = 0; i < 2; ++i )
			{
				ref readonly TreeNode node = ref nodes[pair + i];
				if ( IsNodeMoved( node ) == false )
				{
					continue;
				}

				if ( IsLeaf( node ) )
				{
					proxyIds[count++] = GetProxyId( node );
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

		return count;
	}

	// Slow refit for unit tests.
	private static void RefitSubtree( Span<TreeNode> nodes, int rootIndex )
	{
		ref TreeNode root = ref nodes[rootIndex];
		if ( IsLeaf( root ) || IsNodeMoved( root ) == false )
		{
			return;
		}

		// Negative entries indicate deferred unions. A parent is pushed before its children so the
		// LIFO order guarantees both children are finished when the union runs.
		Span<int> stack = stackalloc int[B2_TREE_STACK_SIZE];
		int stackCount = 0;
		// ~0 == -1
		stack[stackCount++] = ~rootIndex;
		stack[stackCount++] = GetLeftChild( root );
		stack[stackCount++] = GetLeftChild( root ) + 1;

		while ( stackCount > 0 )
		{
			int item = stack[--stackCount];
			if ( item < 0 )
			{
				ref TreeNode parentNode = ref nodes[~item];
				int pair = GetLeftChild( parentNode );
				parentNode.aabb = UnionV( nodes[pair].aabb, nodes[pair + 1].aabb );
				continue;
			}

			ref TreeNode node = ref nodes[item];
			if ( IsLeaf( node ) || IsNodeMoved( node ) == false )
			{
				continue;
			}

			if ( stackCount + 3 <= B2_TREE_STACK_SIZE )
			{
				int pair = GetLeftChild( node );
				stack[stackCount++] = ~item;
				stack[stackCount++] = pair;
				stack[stackCount++] = pair + 1;
			}
			else
			{
				B2_ASSERT( stackCount + 3 <= B2_TREE_STACK_SIZE );
			}
		}
	}

	internal static void DynamicTree_Refit( DynamicTree tree )
	{
		if ( HasTreeMoved( tree ) == false )
		{
			return;
		}

		Span<TreeNode> nodes = tree.nodes;
		if ( tree.dfsOrdered == false )
		{
			// This only happens in unit tests.
			RefitSubtree( nodes, B2_ROOT_NODE );
			return;
		}

		for ( int pair = tree.nodeEnd - 2; pair >= 0; pair -= 2 )
		{
			Span<TreeNode> node = nodes.Slice( pair );
			uint flags1 = node[0].flagIndex;
			uint flags2 = node[1].flagIndex;

			// Did either move?
			if ( ( ( flags1 | flags2 ) & B2_MOVED_NODE ) == 0 )
			{
				continue;
			}

			// Is the node internal and moved?
			bool refit1 = ( flags1 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE;
			bool refit2 = ( flags2 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE;

			// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
			// The store function won't actually modify the node at the dummy index. This
			// avoids an unpredictable branch. SIMD FTW.
			int children1 = refit1 ? (int)( flags1 & B2_NODE_INDEX_MASK ) : pair;
			int children2 = refit2 ? (int)( flags2 & B2_NODE_INDEX_MASK ) : pair;

			// Conditionally store the union.
			StoreAABBV( ref node[0].aabb, UnionPairV( nodes, children1 ), refit1 );
			StoreAABBV( ref node[1].aabb, UnionPairV( nodes, children2 ), refit2 );
		}
	}


	private struct TreeQueryDelegate : ITreeQueryCallback
	{
		public TreeQueryCallbackFcn fcn;
		public object? context;

		public bool Invoke( int proxyId, ulong userData ) => fcn( proxyId, userData, context );
	}

	private struct TreeRayCastDelegate : ITreeRayCastCallback
	{
		public TreeRayCastCallbackFcn fcn;
		public object? context;

		public float Invoke( in RayCastInput input, int proxyId, ulong userData ) => fcn( input, proxyId, userData, context );
	}

	private struct TreeBoxCastDelegate : ITreeBoxCastCallback
	{
		public TreeBoxCastCallbackFcn fcn;
		public object? context;

		public float Invoke( in BoxCastInput input, int proxyId, ulong userData ) => fcn( input, proxyId, userData, context );
	}

	/// Query an AABB for overlapping proxies. The callback is called for each proxy that overlaps the
	/// supplied AABB.
	/// @return performance data
	public static TreeStats DynamicTree_OverlapAABB( DynamicTree tree, AABB aabb, ulong maskBits, TreeQueryCallbackFcn callback, object? context )
	{
		TreeQueryDelegate d = new() { fcn = callback, context = context };
		return DynamicTree_OverlapAABB( tree, aabb, maskBits, ref d );
	}

	/// Query an AABB for overlapping proxies. The callback is called for each proxy that overlaps the
	/// supplied AABB. No filtering is performed.
	/// @return performance data
	public static TreeStats DynamicTree_OverlapAABBAll( DynamicTree tree, AABB aabb, TreeQueryCallbackFcn callback, object? context )
	{
		TreeQueryDelegate d = new() { fcn = callback, context = context };
		return DynamicTree_OverlapAABBAll( tree, aabb, ref d );
	}

	/// Ray cast against the proxies in the tree. This relies on the callback to perform an exact ray
	/// cast in the case where the proxy contains a shape. The callback also performs any collision
	/// filtering.
	/// @return performance data
	public static TreeStats DynamicTree_CastRay( DynamicTree tree, in RayCastInput input, ulong maskBits, TreeRayCastCallbackFcn callback, object? context )
	{
		TreeRayCastDelegate d = new() { fcn = callback, context = context };
		return DynamicTree_CastRay( tree, input, maskBits, ref d );
	}

	/// Cast a swept AABB through the tree.
	/// @return performance data
	public static TreeStats DynamicTree_CastBox( DynamicTree tree, in BoxCastInput input, ulong maskBits, TreeBoxCastCallbackFcn callback, object? context )
	{
		TreeBoxCastDelegate d = new() { fcn = callback, context = context };
		return DynamicTree_CastBox( tree, input, maskBits, ref d );
	}
}
