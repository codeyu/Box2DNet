// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;

namespace Box2DNet;

internal struct TreeNode
{
	/// The node bounding box
	public AABB aabb;

	/// In 3D this space is used by the AABB z components.
	public ulong padding;

	/// - bit 31 : 1 for leaf node
	/// - bit 30 : 1 for moved flag
	/// - bits 0-29 : index of the sibling pair node or the proxy id for a leaf
	public uint flagIndex;

	/// The height of an internal node. A leaf has zero height.
	/// This is a union with shapeIndex in C.
	public int height;

	/// The shape index for a leaf. Truncated from proxy user data.
	public int shapeIndex
	{
		readonly get => height;
		set => height = value;
	}
}

internal struct TreeProxy
{
	/// User data is an index instead of void* because it is used internally as a shape index.
	public ulong userData;

	/// Category bits for collision filtering.
	public ulong categoryBits;

	/// The leaf node. B2_NULL_INDEX for a free proxy.
	public int node;

	/// Next free proxy.
	public int next;
}

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
public sealed class DynamicTree
{
	/// Array of nodes. The root is at index zero and index 1 is empty. Otherwise siblings are
	/// paired at even indices. Has holes for free node pairs.
	internal TreeNode[] nodes = null!;

	/// Parent index per node. The free list is interwoven.
	internal int[] parents = null!;

	/// Proxy data split from node array as cold data.
	internal TreeProxy[] proxies = null!;

	/// Every allocated node has a lower index than this.
	internal int nodeEnd;

	/// The allocated node space
	internal int nodeCapacity;

	/// Free pairs below nodeEnd
	internal int pairFreeList;

	/// Number of proxies created
	internal int proxyCount;

	/// The allocated proxy space
	internal int proxyCapacity;

	/// Proxy free list
	internal int proxyFreeList;

	/// Array of nodes for rebuild.
	internal TreeNode[]? swapNodes;

	/// Leaf indices for rebuild
	internal int[]? leafIndices;

	/// Leaves for the rebuild. May represent a proxy or a retained subtree.
	internal TreeNode[]? leafNodes;

	/// Leaf bounding boxes for rebuild
	internal AABB[]? leafBoxes;

	/// Leaf bounding box centers for rebuild
	internal Vec2[]? leafCenters;

	/// Bins for sorting during rebuild
	internal int[]? binIndices;

	/// Allocated space for rebuilding
	internal int rebuildCapacity;

	/// Rebuild orders the nodes so the children follow parents. Cache friendly for queries and
	/// refitting. The order can be disrupted by proxy creation.
	internal bool dfsOrdered;

	internal void Clear()
	{
		nodes = null!;
		parents = null!;
		proxies = null!;
		nodeEnd = 0;
		nodeCapacity = 0;
		pairFreeList = 0;
		proxyCount = 0;
		proxyCapacity = 0;
		proxyFreeList = 0;
		swapNodes = null;
		leafIndices = null;
		leafNodes = null;
		leafBoxes = null;
		leafCenters = null;
		binIndices = null;
		rebuildCapacity = 0;
		dfsOrdered = false;
	}
}

public static partial class B2
{
	internal const int B2_TREE_STACK_SIZE = 512;
	internal const uint B2_MOVED_NODE = 1u << 30;
	internal const uint B2_LEAF_NODE = 1u << 31;
	internal const uint B2_NODE_INDEX_MASK = 0xFFFFFFFFu & ~( B2_MOVED_NODE | B2_LEAF_NODE );
	internal const uint B2_EMPTY_NODE = B2_NODE_INDEX_MASK | B2_LEAF_NODE;
	internal const int B2_ROOT_NODE = 0;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool IsLeaf( in TreeNode node ) => ( node.flagIndex & B2_LEAF_NODE ) == B2_LEAF_NODE;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool IsNodeMoved( in TreeNode node ) => ( node.flagIndex & B2_MOVED_NODE ) == B2_MOVED_NODE;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool IsEmptyNode( in TreeNode node ) => node.flagIndex == B2_EMPTY_NODE;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int GetLeftChild( in TreeNode node ) => (int)( node.flagIndex & B2_NODE_INDEX_MASK );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int GetProxyId( in TreeNode node ) => (int)( node.flagIndex & B2_NODE_INDEX_MASK );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int GetRootPair( ReadOnlySpan<TreeNode> nodes )
	{
		ref readonly TreeNode root = ref nodes[B2_ROOT_NODE];
		return IsLeaf( root ) ? B2_ROOT_NODE : GetLeftChild( root );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static TreeNode MakeEmptyNode()
	{
		TreeNode node = default;
		node.aabb.lowerBound = new Vec2( float.PositiveInfinity, float.PositiveInfinity );
		node.aabb.upperBound = new Vec2( float.NegativeInfinity, float.NegativeInfinity );
		node.flagIndex = B2_EMPTY_NODE;
		node.height = 0;
		return node;
	}

	internal static bool HasTreeMoved( DynamicTree tree ) => IsNodeMoved( tree.nodes[B2_ROOT_NODE] );

	internal static bool NeedsRebuild( DynamicTree tree ) => IsNodeMoved( tree.nodes[B2_ROOT_NODE] ) || tree.dfsOrdered == false;
}
