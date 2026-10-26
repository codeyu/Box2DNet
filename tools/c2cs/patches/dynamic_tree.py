# Manual fixups applied after converting dynamic_tree.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# pointer pairs in the cast functions become node indices
s = s.replace('\t\t\tconst TreeNode* hit[2];', '\t\t\tFixedArray2<int> hit = default;')
s = s.replace('\t\t\t\thit[hitCount] = node;', '\t\t\t\thit[hitCount] = pair + i;')
s = s.replace('AABB_Center( hit[0].aabb )', 'AABB_Center( nodes[hit[0]].aabb )')
s = s.replace('AABB_Center( hit[1].aabb )', 'AABB_Center( nodes[hit[1]].aabb )')
s = s.replace('GetProxyId( hit[i] )', 'GetProxyId( nodes[hit[i]] )')
s = s.replace('GetLeftChild( hit[i] )', 'GetLeftChild( nodes[hit[i]] )')

# trailing comma in an initializer
s = re.sub(r',\n(\t+)\s*\);', r'\n\1);', s)

# callbacks: generic struct callbacks avoid delegate allocations in the engine
s = s.replace(
    'public static TreeStats DynamicTree_OverlapAABB( DynamicTree tree, AABB aabb, ulong maskBits, ref TreeQueryCallbackFcn callback, object? context )',
    'public static TreeStats DynamicTree_OverlapAABB<T>( DynamicTree tree, AABB aabb, ulong maskBits, ref T callback ) where T : ITreeQueryCallback, allows ref struct')
s = s.replace(
    'public static TreeStats DynamicTree_OverlapAABBAll( DynamicTree tree, AABB aabb, ref TreeQueryCallbackFcn callback, object? context )',
    'public static TreeStats DynamicTree_OverlapAABBAll<T>( DynamicTree tree, AABB aabb, ref T callback ) where T : ITreeQueryCallback, allows ref struct')
s = s.replace(
    'public static TreeStats DynamicTree_CastRay( DynamicTree tree, in RayCastInput input, ulong maskBits, ref TreeRayCastCallbackFcn callback, object? context )',
    'public static TreeStats DynamicTree_CastRay<T>( DynamicTree tree, in RayCastInput input, ulong maskBits, ref T callback ) where T : ITreeRayCastCallback, allows ref struct')
s = s.replace(
    'public static TreeStats DynamicTree_CastBox( DynamicTree tree, in BoxCastInput input, ulong maskBits, ref TreeBoxCastCallbackFcn callback, object? context )',
    'public static TreeStats DynamicTree_CastBox<T>( DynamicTree tree, in BoxCastInput input, ulong maskBits, ref T callback ) where T : ITreeBoxCastCallback, allows ref struct')
s = s.replace('callback( proxyId, proxy.userData, context )', 'callback.Invoke( proxyId, proxy.userData )')
s = s.replace('callback( ref subInput, proxyId, proxy.userData, context )', 'callback.Invoke( subInput, proxyId, proxy.userData )')

s = s.rstrip()
assert s.endswith('}')
s = s[:-1] + '''
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
'''
open(p, 'w', encoding='utf-8', newline='\n').write(s)

s = open(p, encoding='utf-8').read()


def rep(old, new, count=1):
    global s
    assert old in s, old
    s = s.replace(old, new) if count == 0 else s.replace(old, new, count)


rep('\t\t\tint pair = tree.pairFreeList;\n\t\t\ttree.pairFreeList = tree.parents[pair];\n\t\t\treturn pair;',
    '\t\t\tint freePair = tree.pairFreeList;\n\t\t\ttree.pairFreeList = tree.parents[freePair];\n\t\t\treturn freePair;')
rep('\t\t// memset needed for deterministic serialization.\n\t\ttree = default;', '\t\t// memset needed for deterministic serialization.\n\t\ttree.Clear();')
rep('\t\tFree( tree.binIndices, tree.rebuildCapacity );\n\n\t\ttree = default;', '\t\tFree( tree.binIndices, tree.rebuildCapacity );\n\n\t\ttree.Clear();')
rep('DynamicTree tree = Alloc<DynamicTree>( 1 );', 'DynamicTree tree = new DynamicTree();')
rep('\t\tDestroyDynamicTreeInternal( tree );\n\t\tFree( tree, 1 );', '\t\tDestroyDynamicTreeInternal( tree );')
rep('tree.proxies[proxyIndex = default];', 'tree.proxies[proxyIndex] = default;')
rep('node.shapeIndex = (uint)userData;', 'node.shapeIndex = (int)userData;')
s = s.replace('AABBV ', 'AABB ')
s = s.replace('newNodes.AsSpan( newPair )', 'newNodes.Slice( newPair )')
s = s.replace('nodes.AsSpan( pair )', 'nodes.Slice( pair )')
s = s.replace('new CopyItem( leftChild, newPair + 1 )', 'new CopyItem { oldPair = leftChild, newIndex = newPair + 1 }')
rep('if ( IsLeaf( pair ) == false )', 'if ( IsLeaf( pair[0] ) == false )')
rep('oldPair = GetLeftChild( pair );', 'oldPair = GetLeftChild( pair[0] );')
rep('proxies[GetProxyId( pair )].node = newPair;', 'proxies[GetProxyId( pair[0] )].node = newPair;')
s = s.replace('SetLeftChild( newNodes[leftIndex], newPair );', 'SetLeftChild( ref newNodes[leftIndex], newPair );')
s = s.replace('SetLeftChild( newNodes[item.newIndex], newPair );', 'SetLeftChild( ref newNodes[item.newIndex], newPair );')
s = s.replace('PartitionMid( leafIndices[startIndex], leafCenters[startIndex], count )',
              'PartitionMid( leafIndices.Slice( startIndex ), leafCenters.Slice( startIndex ), count )')
s = s.replace('PartitionSAH( leafIndices[startIndex], binIndices[startIndex], leafBoxes[startIndex], count )',
              'PartitionSAH( leafIndices.Slice( startIndex ), binIndices.Slice( startIndex ), leafBoxes.Slice( startIndex ), count )')
rep('if ( AtomicLoadU32Raw( ref node.flagIndex ) & B2_MOVED_NODE )', 'if ( ( AtomicLoadU32Raw( ref node.flagIndex ) & B2_MOVED_NODE ) != 0 )')
rep('\t\t\t\tref TreeNode node = ref nodes[~item];\n\t\t\t\tint pair = GetLeftChild( node );\n\t\t\t\tnode.aabb',
    '\t\t\t\tref TreeNode parentNode = ref nodes[~item];\n\t\t\t\tint pair = GetLeftChild( parentNode );\n\t\t\t\tparentNode.aabb')
s = s.replace('UnionPairV( nodes[children1] )', 'UnionPairV( nodes, children1 )')
s = s.replace('UnionPairV( nodes[children2] )', 'UnionPairV( nodes, children2 )')
s = s.replace('ValidateSubtree( tree, pair, leafCount )', 'ValidateSubtree( tree, pair, ref leafCount )')
s = s.replace('ValidateSubtree( tree, pair + 1, leafCount )', 'ValidateSubtree( tree, pair + 1, ref leafCount )')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
s = open(p, encoding='utf-8').read()
s = s.replace('\t\t\ttree.swapNodes[newIndex] = leaf;', '\t\t\ttree.swapNodes![newIndex] = leaf;')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
