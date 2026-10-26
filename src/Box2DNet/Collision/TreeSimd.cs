// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;

namespace Box2DNet;

// Scalar versions of the SIMD bounding box helpers in simd.h. The comparisons and min/max
// operations are exact, so these produce the same results as the SSE2/NEON versions.
public static partial class B2
{
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static AABB LoadAABBV( in AABB aabb ) => aabb;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool OverlapNode( in AABB av, in TreeNode node )
	{
		ref readonly AABB bv = ref node.aabb;
		return av.lowerBound.x <= bv.upperBound.x && av.lowerBound.y <= bv.upperBound.y && bv.lowerBound.x <= av.upperBound.x &&
			   bv.lowerBound.y <= av.upperBound.y;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool OverlapV( in AABB a, in AABB b )
	{
		return a.lowerBound.x <= b.upperBound.x && a.lowerBound.y <= b.upperBound.y && b.lowerBound.x <= a.upperBound.x &&
			   b.lowerBound.y <= a.upperBound.y;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static AABB UnionV( AABB a, AABB b ) => AABB_Union( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static AABB UnionPairV( ReadOnlySpan<TreeNode> nodes, int pair ) => AABB_Union( nodes[pair].aabb, nodes[pair + 1].aabb );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void StoreAABBV( ref AABB aabb, AABB value, bool condition )
	{
		if ( condition )
		{
			aabb = value;
		}
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int OverlapPairMask( ReadOnlySpan<TreeNode> nodesA, int pairA, ReadOnlySpan<TreeNode> nodesB, int pairB )
	{
		int mask = 0;
		mask |= OverlapV( nodesA[pairA].aabb, nodesB[pairB].aabb ) ? 1 : 0;
		mask |= OverlapV( nodesA[pairA].aabb, nodesB[pairB + 1].aabb ) ? 2 : 0;
		mask |= OverlapV( nodesA[pairA + 1].aabb, nodesB[pairB].aabb ) ? 4 : 0;
		mask |= OverlapV( nodesA[pairA + 1].aabb, nodesB[pairB + 1].aabb ) ? 8 : 0;
		return mask;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int OverlapNodePairMask( in AABB av, ReadOnlySpan<TreeNode> nodes, int pair )
	{
		int mask = 0;
		mask |= OverlapNode( av, nodes[pair] ) ? 1 : 0;
		mask |= OverlapNode( av, nodes[pair + 1] ) ? 2 : 0;
		return mask;
	}

	private static int s_simdWidth;

	// Only the 4 wide contact solver is ported. Both widths produce identical results in Box2D.
	private static int DetectSIMDWidth()
	{
		return 4;
	}

	internal static int GetSIMDWidth()
	{
		int width = System.Threading.Volatile.Read( ref s_simdWidth );
		if ( width == 0 )
		{
			width = DetectSIMDWidth();
			System.Threading.Volatile.Write( ref s_simdWidth, width );
		}

		return width;
	}

	internal static void SetSIMDWidth( int width )
	{
		B2_ASSERT( width == 0 || width == 4 || width == 8 );
		if ( width == 8 && DetectSIMDWidth() != 8 )
		{
			width = 4;
		}

		System.Threading.Volatile.Write( ref s_simdWidth, width );
	}
}
