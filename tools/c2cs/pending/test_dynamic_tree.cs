// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class DynamicTreeTests
{
	private static int TreeCreateDestroy()
	{
		AABB a = new( .lowerBound = new( -1.0f, -1.0f ),
			.upperBound = new( 2.0f, 2.0f ), );

		DynamicTree tree = default;
		CreateDynamicTreeInternal( ref tree, 16 );
		CreateTreeProxyInternal( ref tree, a, 1, 0, 0 );

		ENSURE( tree.nodeEnd == 2 );
		ENSURE( tree.proxyCount == 1 );

		DestroyDynamicTreeInternal( ref tree );

		ENSURE( tree.nodeEnd == 0 );
		ENSURE( tree.proxyCount == 0 );

		return 0;
	}

	internal static float RayCastCallbackFcn( in RayCastInput input, int proxyId, ulong userData, object? context )
	{
		(void)input;
		(void)userData;

		ref int proxyHit = ref context;
		proxyHit = proxyId;
		return 0.0f;
	}

	private static int TreeRayCastTest()
	{
		// Test AABB centered at origin with bounds [-1, -1] to [1, 1]
		AABB a = new( .lowerBound = new( -1.0f, -1.0f ), .upperBound = new( 1.0f, 1.0f ), );
		DynamicTree tree = CreateDynamicTree( 16 );
		int proxyId = CreateTreeProxyInternal( tree, a, 1, 0, 0 );

		RayCastInput input = new(  );
		input.maxFraction = 1.0f;

		// Test 1: Ray hits AABB from left side
		{
			Vec2 p1 = new(  -3.0f, 0.0f  );
			Vec2 p2 = new(  3.0f, 0.0f  );

			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 2: Ray hits AABB from right side
		{
			Vec2 p1 = new(  3.0f, 0.0f  );
			Vec2 p2 = new(  -3.0f, 0.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 3: Ray hits AABB from bottom
		{
			Vec2 p1 = new(  0.0f, -3.0f  );
			Vec2 p2 = new(  0.0f, 3.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 4: Ray hits AABB from top
		{
			Vec2 p1 = new(  0.0f, 3.0f  );
			Vec2 p2 = new(  0.0f, -3.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 5: Ray misses AABB completely (parallel to x-axis)
		{
			Vec2 p1 = new(  -3.0f, 2.0f  );
			Vec2 p2 = new(  3.0f, 2.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		// Test 6: Ray misses AABB completely (parallel to y-axis)
		{
			Vec2 p1 = new(  2.0f, -3.0f  );
			Vec2 p2 = new(  2.0f, 3.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		// Test 7: Ray starts inside AABB
		{
			Vec2 p1 = new(  0.0f, 0.0f  );
			Vec2 p2 = new(  2.0f, 0.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 8: Ray hits corner of AABB (diagonal ray)
		{
			Vec2 p1 = new(  -2.0f, -2.0f  );
			Vec2 p2 = new(  2.0f, 2.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 9: Ray parallel to AABB edge but outside
		{
			Vec2 p1 = new(  -2.0f, 1.5f  );
			Vec2 p2 = new(  2.0f, 1.5f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		// Test 10: Ray parallel to AABB edge and exactly on boundary
		{
			Vec2 p1 = new(  -2.0f, 1.0f  );
			Vec2 p2 = new(  2.0f, 1.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 11: Very short ray that doesn't reach AABB
		{
			Vec2 p1 = new(  -3.0f, 0.0f  );
			Vec2 p2 = new(  -2.5f, 0.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		// Test 12: Zero-length ray (degenerate case)
		{
			Vec2 p1 = new(  0.0f, 0.0f  );
			Vec2 p2 = new(  0.0f, 0.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// Test 13: Ray hits AABB at exact boundary condition (t = 1.0)
		{
			Vec2 p1 = new(  -2.0f, 0.0f  );
			Vec2 p2 = new(  -1.0f, 0.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyId );
		}

		// An off center box catches an axis aligned ray tested against the wrong axis, which the origin
		// centered box above cannot. Two leaves also give the root a child pair.
		AABB b = new( .lowerBound = new( 10.0f, 4.0f ), .upperBound = new( 11.0f, 6.0f ) );
		int proxyIdB = CreateTreeProxyInternal( tree, b, 1, 1, 0 );

		// Test 14: Horizontal ray through the off center box
		{
			Vec2 p1 = new(  0.0f, 5.0f  );
			Vec2 p2 = new(  21.0f, 5.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyIdB );
		}

		// Test 15: Vertical ray through the off center box
		{
			Vec2 p1 = new(  10.5f, 0.0f  );
			Vec2 p2 = new(  10.5f, 21.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == proxyIdB );
		}

		// Test 16: Horizontal ray passing above the off center box
		{
			Vec2 p1 = new(  0.0f, 7.0f  );
			Vec2 p2 = new(  21.0f, 7.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		// Test 17: Vertical ray passing beside the off center box
		{
			Vec2 p1 = new(  12.0f, 0.0f  );
			Vec2 p2 = new(  12.0f, 21.0f  );
			input.origin = p1;
			input.translation = Sub( p2, p1 );

			int proxyHit = -1;
			DynamicTree_CastRay( tree, ref input, 1, RayCastCallbackFcn, ref proxyHit );

			ENSURE( proxyHit == -1 );
		}

		DestroyDynamicTree( tree );

		return 0;
	}

	private static bool QueryCollectCallback( int proxyId, ulong userData, object? context )
	{
		(void)userData;
		int* @out = context;
		@out[proxyId] = 1;
		return true; // continue the query
	}

	private static bool QueryCollectListCallback( int proxyId, ulong userData, object? context )
	{
		(void)userData;
		Span<int> list = context;
		int count = list[0];
		list[count + 1] = proxyId;
		list[0] = count + 1;
		return true;
	}

	private static int TreeMultipleProxiesTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		AABB a1 = new( .lowerBound = new( -5.0f, -1.0f ), .upperBound = new( -3.0f, 1.0f ) );
		AABB a2 = new( .lowerBound = new( -1.0f, -1.0f ), .upperBound = new( 1.0f, 1.0f ) );
		AABB a3 = new( .lowerBound = new( 3.0f, -1.0f ), .upperBound = new( 5.0f, 1.0f ) );

		int id1 = CreateTreeProxyInternal( tree, a1, 0x1ull, 42, 0 );
		int id2 = CreateTreeProxyInternal( tree, a2, 0x2ull, 43, 0 );
		int id3 = CreateTreeProxyInternal( tree, a3, 0x4ull, 44, 0 );

		ENSURE( DynamicTree_GetProxyCount( tree ) == 3 );

		ENSURE( DynamicTree_GetUserData( tree, id1 ) == 42 );
		ENSURE( DynamicTree_GetUserData( tree, id2 ) == 43 );
		ENSURE( DynamicTree_GetUserData( tree, id3 ) == 44 );

		ENSURE( DynamicTree_GetCategoryBits( tree, id1 ) == 0x1ull );
		ENSURE( DynamicTree_GetCategoryBits( tree, id2 ) == 0x2ull );
		ENSURE( DynamicTree_GetCategoryBits( tree, id3 ) == 0x4ull );

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeQueryTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		AABB a1 = new( .lowerBound = new( -5.0f, -1.0f ), .upperBound = new( -3.0f, 1.0f ) );
		AABB a2 = new( .lowerBound = new( -1.0f, -1.0f ), .upperBound = new( 1.0f, 1.0f ) );
		AABB a3 = new( .lowerBound = new( 3.0f, -1.0f ), .upperBound = new( 5.0f, 1.0f ) );

		int id1 = CreateTreeProxyInternal( tree, a1, 0xFFull, 0, 0 );
		int id2 = CreateTreeProxyInternal( tree, a2, 0xFFull, 0, 0 );
		int id3 = CreateTreeProxyInternal( tree, a3, 0xFFull, 0, 0 );

		AABB queryA = new( .lowerBound = new( -2.0f, -2.0f ), .upperBound = new( 2.0f, 2.0f ) );

		int[] foundFlags = { 0 };
		TreeStats stats = DynamicTree_OverlapAABB( tree, queryA, 0xFFFFFFFFull, QueryCollectCallback, foundFlags );

		// We expect at least the middle proxy to be visited.
		ENSURE( foundFlags[id2] == 1 );
		ENSURE( stats.leafVisits >= 1 );

		// Test QueryAll using list collector
		int[] list = { 0 }; // list[0] holds count, following entries are ids
		TreeStats allStats = DynamicTree_OverlapAABBAll( tree, queryA, QueryCollectListCallback, list );
		ENSURE( list[0] >= 1 ); // at least one proxy should be collected
		ENSURE( allStats.leafVisits >= 1 );

		DestroyDynamicTree( tree );
		(void)id1; (void)id3;
		return 0;
	}

	private static int TreeMoveAndEnlargeTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		AABB a = new( .lowerBound = new( 0.0f, 0.0f ), .upperBound = new( 1.0f, 1.0f ) );
		int id = CreateTreeProxyInternal( tree, a, 0x1ull, 100, 0 );

		// Move proxy to a new place
		AABB moved = new( .lowerBound = new( 10.0f, 10.0f ), .upperBound = new( 11.0f, 11.0f ) );
		DynamicTree_MoveProxyInternal( tree, id, moved, 0 );

		AABB got = DynamicTree_GetProxyBounds( tree, id );
		ENSURE( got.lowerBound.x == moved.lowerBound.x );
		ENSURE( got.lowerBound.y == moved.lowerBound.y );
		ENSURE( got.upperBound.x == moved.upperBound.x );
		ENSURE( got.upperBound.y == moved.upperBound.y );

		// Now enlarge the proxy
		AABB enlarge = new( .lowerBound = new( 9.5f, 9.5f ), .upperBound = new( 11.5f, 11.5f ) );
		DynamicTree_EnlargeProxy( tree, id, enlarge );

		AABB got2 = DynamicTree_GetProxyBounds( tree, id );
		ENSURE( got2.lowerBound.x <= enlarge.lowerBound.x + 1e-6f );
		ENSURE( got2.upperBound.x >= enlarge.upperBound.x - 1e-6f );

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeRebuildAndValidateTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		// Create a number of proxies to make rebuild meaningful
		for ( int i = 0; i < 12; ++i )
		{
			float x = (float)i * 2.0f;
			AABB a = new( .lowerBound = new( x - 0.5f, -0.5f ), .upperBound = new( x + 0.5f, 0.5f ) );
			CreateTreeProxyInternal( tree, a, 0xFFull, (ulong)i, 0 );
		}

		int sorted = DynamicTree_Rebuild( tree, true );

		ENSURE( sorted >= 0 );
		ENSURE( DynamicTree_GetByteCount( tree ) > 0 );
		ENSURE( DynamicTree_GetHeight( tree ) > 0 );

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeRowHeightTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		int columnCount = 200;
		for (int i = 0; i < columnCount; ++i)
		{
			float x = 1.0f * i;
			AABB a = new( .lowerBound = new( x, 0.0f ), .upperBound = new( x + 1.0f, 1.0f ) );
			CreateTreeProxyInternal( tree, a, 1, (ulong)i, 0 );
		}

		float minHeight = log2f((float)columnCount);

		DynamicTree_Validate( tree );
		ENSURE( DynamicTree_GetHeight( tree ) < 2.0f * minHeight );

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeGridHeightTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		int columnCount = 20;
		int rowCount = 20;
		for (int i = 0; i < columnCount; ++i)
		{
			float x = 1.0f * i;
			for (int j = 0; j < rowCount; ++j)
			{
				float y = 1.0f * j;
				AABB a = new( .lowerBound = new( x, y ), .upperBound = new( x + 1.0f, y + 1.0f ) );
				CreateTreeProxyInternal( tree, a, 1, (ulong)i, 0 );
			}
		}

		float minHeight = log2f( (float)(rowCount * columnCount) );

		DynamicTree_Validate( tree );
		ENSURE( DynamicTree_GetHeight( tree ) < 2.0f * minHeight );

		DestroyDynamicTree( tree );
		return 0;
	}

	private const int GRID_COUNT = 20;
	private static int TreeGridMovementTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		Span<int> proxyIds = stackalloc int[GRID_COUNT * GRID_COUNT];
		int index = 0;
		for (int i = 0; i < GRID_COUNT; ++i)
		{
			float x = 1.0f * i;
			for (int j = 0; j < GRID_COUNT; ++j)
			{
				float y = 1.0f * j;
				AABB a = new( .lowerBound = new( x, y ), .upperBound = new( x + 1.0f, y + 1.0f ) );
				proxyIds[index] = CreateTreeProxyInternal( tree, a, 1, (ulong)i, 0 );
				index += 1;
			}
		}

		ENSURE( index == GRID_COUNT * GRID_COUNT );

		float minHeight = log2f( (float)( GRID_COUNT * GRID_COUNT ) );

		DynamicTree_Validate( tree );
		int height1 = DynamicTree_GetHeight( tree );
		ENSURE( height1 < 2.0f * minHeight );

		Vec2 offset = new( 10.0f, 20.0f );
		index = 0;
		for (int i = 0; i < GRID_COUNT; ++i)
		{
			for (int j = 0; j < GRID_COUNT; ++j)
			{
				AABB a = DynamicTree_GetProxyBounds( tree, proxyIds[index] );
				a.lowerBound = Add( a.lowerBound, offset );
				a.upperBound = Add( a.upperBound, offset );
				DynamicTree_MoveProxyInternal( tree, proxyIds[index], a, 0 );
				index += 1;
			}
		}

		DynamicTree_Validate( tree );
		int height2 = DynamicTree_GetHeight( tree );
		ENSURE( height2 < 3.0f * minHeight );

		DynamicTree_Rebuild( tree, true );

		DynamicTree_Validate( tree );
		int height3 = DynamicTree_GetHeight( tree );
		ENSURE( height3 < 2.0f * minHeight );

		DestroyDynamicTree( tree );
		return 0;
	}

	private const int TREE_TEST_PROXY_COUNT = 200;
	// Small generator so the random tests repeat run to run
	static uint treeRandomState = 12345u;

	private static float RandomTreeFloat( float lower, float upper )
	{
		treeRandomState = 1664525u * treeRandomState + 1013904223u;
		float unit = (float)( treeRandomState >> 8 ) * ( 1.0f / 16777216.0f );
		return lower + ( upper - lower ) * unit;
	}

	private static AABB RandomTreeBox( float maxHalfExtent )
	{
		float x = RandomTreeFloat( -50.0f, 50.0f );
		float y = RandomTreeFloat( -50.0f, 50.0f );
		float hx = RandomTreeFloat( 0.25f, maxHalfExtent );
		float hy = RandomTreeFloat( 0.25f, maxHalfExtent );
		return (AABB){ { x - hx, y - hy }, { x + hx, y + hy } };
	}

	private static bool BoxEquals( AABB a, AABB b )
	{
		return a.lowerBound.x == b.lowerBound.x && a.lowerBound.y == b.lowerBound.y && a.upperBound.x == b.upperBound.x &&
			   a.upperBound.y == b.upperBound.y;
	}

	private static AABB EnlargeTreeBox( AABB a, float margin )
	{
		return (AABB){ { a.lowerBound.x - margin, a.lowerBound.y - margin }, { a.upperBound.x + margin, a.upperBound.y + margin } };
	}

	typedef struct TreeHitSet
	{
		Span<bool> hit = stackalloc bool[TREE_TEST_PROXY_COUNT];
		int count;
		bool bad;
	} TreeHitSet;

	private static bool CollectHitCallback( int proxyId, ulong userData, object? context )
	{
		(void)userData;
		TreeHitSet* hits = context;
		if ( proxyId < 0 || TREE_TEST_PROXY_COUNT <= proxyId || hits.hit[proxyId] )
		{
			// Out of range or reported twice
			hits.bad = true;
			return true;
		}

		hits.hit[proxyId] = true;
		hits.count += 1;
		return true;
	}

	// Query the tree and compare against every live proxy box
	private static int CompareQuery( DynamicTree tree, ReadOnlySpan<int> proxyIds, int proxyCount, AABB box )
	{
		TreeHitSet hits = { 0 };
		DynamicTree_OverlapAABB( tree, box, DefaultMaskBits, CollectHitCallback, ref hits );
		ENSURE( hits.bad == false );

		int expectedCount = 0;
		for ( int i = 0; i < proxyCount; ++i )
		{
			int proxyId = proxyIds[i];
			bool overlaps = AABB_Overlaps( DynamicTree_GetProxyBounds( tree, proxyId ), box );
			ENSURE( hits.hit[proxyId] == overlaps );
			expectedCount += overlaps ? 1 : 0;
		}

		ENSURE( hits.count == expectedCount );
		return 0;
	}

	typedef struct TreeCastHit
	{
		DynamicTree tree = None;
		int proxyId;
		float fraction;
	} TreeCastHit;

	// Entry fraction along the full translation, negative for a miss
	private static float RayBoxFraction( AABB box, Vec2 origin, Vec2 translation )
	{
		CastOutput output = AABB_RayCast( box, origin, Add( origin, translation ) );
		return output.hit ? output.fraction : -1.0f;
	}

	// A box sweep against a box is a ray against the box grown by the swept extents
	private static float BoxCastFraction( AABB box, in BoxCastInput input )
	{
		Vec2 extents = AABB_Extents( input.box );
		AABB grown = new(  Sub( box.lowerBound, extents ), Add( box.upperBound, extents )  );
		return RayBoxFraction( grown, AABB_Center( input.box ), input.translation );
	}

	// The fraction is computed against the full translation rather than the clipped input so it
	// matches the brute force bit for bit
	private static float ClosestRayCallback( in RayCastInput input, int proxyId, ulong userData, object? context )
	{
		(void)userData;
		TreeCastHit* hit = context;
		float fraction = RayBoxFraction( DynamicTree_GetProxyBounds( hit.tree, proxyId ), input.origin, input.translation );
		if ( fraction < 0.0f )
		{
			return -1.0f;
		}

		if ( fraction < hit.fraction )
		{
			hit.proxyId = proxyId;
			hit.fraction = fraction;
		}

		return fraction;
	}

	private static float ClosestBoxCallback( in BoxCastInput input, int proxyId, ulong userData, object? context )
	{
		(void)userData;
		TreeCastHit* hit = context;
		float fraction = BoxCastFraction( DynamicTree_GetProxyBounds( hit.tree, proxyId ), input );
		if ( fraction < 0.0f )
		{
			return -1.0f;
		}

		if ( fraction < hit.fraction )
		{
			hit.proxyId = proxyId;
			hit.fraction = fraction;
		}

		return fraction;
	}

	// Closest ray hit from the tree against the closest over every live proxy
	private static int CompareRayCast( DynamicTree tree, ReadOnlySpan<int> proxyIds, int proxyCount, Vec2 origin, Vec2 translation )
	{
		TreeCastHit hit = { tree, NullIndex, 1.0f };
		RayCastInput input = new(  .origin = origin, .translation = translation, .maxFraction = 1.0f  );
		DynamicTree_CastRay( tree, ref input, DefaultMaskBits, ClosestRayCallback, ref hit );

		int bestId = NullIndex;
		float bestFraction = 1.0f;
		bool unique = true;
		for ( int i = 0; i < proxyCount; ++i )
		{
			int proxyId = proxyIds[i];
			float fraction = RayBoxFraction( DynamicTree_GetProxyBounds( tree, proxyId ), origin, translation );
			if ( fraction < 0.0f )
			{
				continue;
			}

			if ( fraction < bestFraction )
			{
				bestId = proxyId;
				bestFraction = fraction;
				unique = true;
			}
			else if ( fraction == bestFraction )
			{
				unique = false;
			}
		}

		ENSURE( hit.fraction == bestFraction );
		if ( unique )
		{
			ENSURE( hit.proxyId == bestId );
		}

		return 0;
	}

	private static int CompareBoxCast( DynamicTree tree, ReadOnlySpan<int> proxyIds, int proxyCount, AABB box, Vec2 translation )
	{
		TreeCastHit hit = { tree, NullIndex, 1.0f };
		BoxCastInput input = new(  .box = box, .translation = translation, .maxFraction = 1.0f  );
		DynamicTree_CastBox( tree, ref input, DefaultMaskBits, ClosestBoxCallback, ref hit );

		int bestId = NullIndex;
		float bestFraction = 1.0f;
		bool unique = true;
		for ( int i = 0; i < proxyCount; ++i )
		{
			int proxyId = proxyIds[i];
			float fraction = BoxCastFraction( DynamicTree_GetProxyBounds( tree, proxyId ), ref input );
			if ( fraction < 0.0f )
			{
				continue;
			}

			if ( fraction < bestFraction )
			{
				bestId = proxyId;
				bestFraction = fraction;
				unique = true;
			}
			else if ( fraction == bestFraction )
			{
				unique = false;
			}
		}

		ENSURE( hit.fraction == bestFraction );
		if ( unique )
		{
			ENSURE( hit.proxyId == bestId );
		}

		return 0;
	}

	// A node is marked exactly when a leaf below it is marked. Leaves are checked against the
	// set the test marked itself.
	private static int CheckMarks( DynamicTree tree, int nodeIndex, ReadOnlySpan<bool> markedProxies, ref bool markedBelow )
	{
		ref readonly TreeNode node = ref tree.nodes[nodeIndex];
		bool marked = IsNodeMoved( node );
		if ( IsLeaf( node ) )
		{
			ENSURE( marked == markedProxies[GetProxyId( node )] );
		}
		else
		{
			int pair = GetLeftChild( node );
			bool below1 = false;
			bool below2 = false;
			if ( CheckMarks( tree, pair, markedProxies, ref below1 ) != 0 )
			{
				return 1;
			}

			if ( CheckMarks( tree, pair + 1, markedProxies, ref below2 ) != 0 )
			{
				return 1;
			}

			ENSURE( marked == ( below1 || below2 ) );
		}

		markedBelow = marked;
		return 0;
	}

	// Every internal node holds exactly the union of its children
	private static int CheckBounds( DynamicTree tree, int nodeIndex, ref AABB bounds )
	{
		ref readonly TreeNode node = ref tree.nodes[nodeIndex];
		if ( IsLeaf( node ) == false )
		{
			int pair = GetLeftChild( node );
			AABB below1, below2;
			if ( CheckBounds( tree, pair, ref below1 ) != 0 )
			{
				return 1;
			}

			if ( CheckBounds( tree, pair + 1, ref below2 ) != 0 )
			{
				return 1;
			}

			ENSURE( BoxEquals( AABB_Union( below1, below2 ), node.aabb ) );
		}

		bounds = node.aabb;
		return 0;
	}

	private static bool RootMarked( DynamicTree tree )
	{
		return HasTreeMoved( tree );
	}

	private static int TreeOneProxyTest()
	{
		DynamicTree tree = CreateDynamicTree( 1 );
		AABB box = new( new( 0.0f, 0.0f ), new( 1.0f, 1.0f ) );
		int proxyId = CreateTreeProxyInternal( tree, box, 1, 7, false );
		int[] proxyIds = { proxyId };
		DynamicTree_Validate( tree );

		// One proxy is a leaf at the root beside the empty node
		ENSURE( tree.nodeEnd == 2 );
		ENSURE( tree.proxyCount == 1 );
		ENSURE( IsLeaf( tree.nodes[B2_ROOT_NODE] ) );
		ENSURE( DynamicTree_GetHeight( tree ) == 0 );
		ENSURE( DynamicTree_GetUserData( tree, proxyId ) == 7 );
		ENSURE( DynamicTree_GetCategoryBits( tree, proxyId ) == 1 );
		ENSURE( BoxEquals( DynamicTree_GetProxyBounds( tree, proxyId ), box ) );
		ENSURE( BoxEquals( DynamicTree_GetRootBounds( tree ), box ) );

		AABB inside = new( new( 0.4f, 0.4f ), new( 0.6f, 0.6f ) );
		AABB outside = new( new( 5.0f, 5.0f ), new( 6.0f, 6.0f ) );
		ENSURE( CompareQuery( tree, proxyIds, 1, inside ) == 0 );
		ENSURE( CompareQuery( tree, proxyIds, 1, outside ) == 0 );

		Vec2 origin = new(  -1.0f, 0.5f  );
		Vec2 translation = new(  3.0f, 0.0f  );
		TreeCastHit hit = { tree, NullIndex, 1.0f };
		RayCastInput input = new(  .origin = origin, .translation = translation, .maxFraction = 1.0f  );
		DynamicTree_CastRay( tree, ref input, DefaultMaskBits, ClosestRayCallback, ref hit );
		ENSURE( hit.proxyId == proxyId );
		ENSURE( CompareRayCast( tree, proxyIds, 1, origin, translation ) == 0 );

		Vec2 missOrigin = new(  -1.0f, 5.0f  );
		ENSURE( CompareRayCast( tree, proxyIds, 1, missOrigin, translation ) == 0 );

		AABB castBox = new( new( -3.0f, 0.25f ), new( -2.0f, 0.75f ) );
		Vec2 castTranslation = new(  10.0f, 0.0f  );
		ENSURE( CompareBoxCast( tree, proxyIds, 1, castBox, castTranslation ) == 0 );

		// A full build keeps the leaf at the root
		ENSURE( DynamicTree_Rebuild( tree, true ) == 1 );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 2 );
		ENSURE( tree.dfsOrdered );
		ENSURE( CompareQuery( tree, proxyIds, 1, inside ) == 0 );

		// The leaf is the root, so a mark and refit show up in the root bounds directly
		AABB enlarged = EnlargeTreeBox( box, 0.5f );
		DynamicTree_MarkProxyMoved( tree, proxyId, enlarged );
		ENSURE( RootMarked( tree ) );
		DynamicTree_Refit( tree );
		DynamicTree_Validate( tree );
		ENSURE( BoxEquals( DynamicTree_GetProxyBounds( tree, proxyId ), enlarged ) );
		ENSURE( BoxEquals( DynamicTree_GetRootBounds( tree ), enlarged ) );
		ENSURE( DynamicTree_Rebuild( tree, false ) == 1 );
		ENSURE( RootMarked( tree ) == false );
		DynamicTree_ValidateNoMoved( tree );

		DestroyTreeProxy( tree, proxyId );
		DynamicTree_Validate( tree );
		ENSURE( tree.proxyCount == 0 );
		ENSURE( tree.nodeEnd == 2 );
		ENSURE( IsEmptyNode( tree.nodes[B2_ROOT_NODE] ) );
		ENSURE( DynamicTree_GetHeight( tree ) == 0 );
		ENSURE( CompareQuery( tree, proxyIds, 0, inside ) == 0 );

		DestroyDynamicTree( tree );
		return 0;
	}

	// Removing down to one proxy puts the survivor at the root as a leaf, the next insert makes a
	// pair for the two of them from the hole, and a full build compacts the holes.
	private static int TreeLeafRootTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		AABB boxA = new( new( 0.0f, 0.0f ), new( 1.0f, 1.0f ) );
		AABB boxB = new( new( 10.0f, 0.0f ), new( 11.0f, 1.0f ) );
		AABB boxC = new( new( 20.0f, 0.0f ), new( 21.0f, 1.0f ) );
		AABB boxD = new( new( 30.0f, 0.0f ), new( 31.0f, 1.0f ) );
		AABB all = new( new( -1.0f, -1.0f ), new( 40.0f, 2.0f ) );

		int idA = CreateTreeProxyInternal( tree, boxA, 1, 0, false );
		int idB = CreateTreeProxyInternal( tree, boxB, 1, 1, false );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 4 );
		ENSURE( tree.proxyCount == 2 );
		ENSURE( IsLeaf( tree.nodes[B2_ROOT_NODE] ) == false );
		ENSURE( DynamicTree_GetHeight( tree ) == 1 );

		DestroyTreeProxy( tree, idA );
		DynamicTree_Validate( tree );
		ENSURE( IsLeaf( tree.nodes[B2_ROOT_NODE] ) );
		ENSURE( tree.nodeEnd == 4 );
		ENSURE( tree.pairFreeList == 2 );
		ENSURE( tree.proxyCount == 1 );
		int[] idsB = { idB };
		ENSURE( CompareQuery( tree, idsB, 1, all ) == 0 );
		ENSURE( CompareQuery( tree, idsB, 1, boxA ) == 0 );

		// The freed pair is reused
		int idC = CreateTreeProxyInternal( tree, boxC, 1, 2, false );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 4 );
		ENSURE( tree.pairFreeList == NullIndex );
		ENSURE( tree.proxyCount == 2 );
		int[] idsBC = { idB, idC };
		ENSURE( CompareQuery( tree, idsBC, 2, all ) == 0 );

		// A third proxy extends the array
		int idD = CreateTreeProxyInternal( tree, boxD, 1, 3, false );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 6 );
		ENSURE( tree.proxyCount == 3 );
		int[] idsBCD = { idB, idC, idD };
		ENSURE( CompareQuery( tree, idsBCD, 3, all ) == 0 );
		ENSURE( CompareQuery( tree, idsBCD, 3, boxC ) == 0 );

		// A remove leaves a hole that the full build compacts
		DestroyTreeProxy( tree, idB );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 6 );
		ENSURE( tree.pairFreeList != NullIndex );
		int[] idsCD = { idC, idD };
		ENSURE( CompareQuery( tree, idsCD, 2, all ) == 0 );
		ENSURE( DynamicTree_Rebuild( tree, true ) == 2 );
		DynamicTree_Validate( tree );
		ENSURE( tree.nodeEnd == 4 );
		ENSURE( tree.pairFreeList == NullIndex );
		ENSURE( CompareQuery( tree, idsCD, 2, all ) == 0 );

		DestroyTreeProxy( tree, idC );
		DynamicTree_Validate( tree );
		ENSURE( IsLeaf( tree.nodes[B2_ROOT_NODE] ) );

		DestroyTreeProxy( tree, idD );
		DynamicTree_Validate( tree );
		ENSURE( tree.proxyCount == 0 );
		ENSURE( IsEmptyNode( tree.nodes[B2_ROOT_NODE] ) );
		ENSURE( DynamicTree_GetHeight( tree ) == 0 );
		ENSURE( CompareQuery( tree, idsBCD, 0, all ) == 0 );

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeQueryBruteForceTest()
	{
		DynamicTree tree = CreateDynamicTree( TREE_TEST_PROXY_COUNT );
		Span<int> proxyIds = stackalloc int[TREE_TEST_PROXY_COUNT];
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
		{
			proxyIds[i] = CreateTreeProxyInternal( tree, RandomTreeBox( 2.0f ), 1, (ulong)i, false );
		}

		DynamicTree_Validate( tree );
		for ( int i = 0; i < 32; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
		}

		ENSURE( DynamicTree_Rebuild( tree, true ) == TREE_TEST_PROXY_COUNT );
		DynamicTree_Validate( tree );
		for ( int i = 0; i < 32; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
		}

		// Moves leave the built order behind
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; i += 2 )
		{
			DynamicTree_MoveProxyInternal( tree, proxyIds[i], RandomTreeBox( 2.0f ), false );
		}

		DynamicTree_Validate( tree );
		for ( int i = 0; i < 32; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
		}

		DestroyDynamicTree( tree );
		return 0;
	}

	private static int TreeCastBruteForceTest()
	{
		DynamicTree tree = CreateDynamicTree( TREE_TEST_PROXY_COUNT );
		Span<int> proxyIds = stackalloc int[TREE_TEST_PROXY_COUNT];
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
		{
			proxyIds[i] = CreateTreeProxyInternal( tree, RandomTreeBox( 2.0f ), 1, (ulong)i, false );
		}

		ENSURE( DynamicTree_Rebuild( tree, true ) == TREE_TEST_PROXY_COUNT );
		DynamicTree_Validate( tree );

		for ( int i = 0; i < 64; ++i )
		{
			Vec2 origin = new(  RandomTreeFloat( -60.0f, 60.0f ), RandomTreeFloat( -60.0f, 60.0f )  );
			float angle = RandomTreeFloat( -Pi, Pi );
			float length = RandomTreeFloat( 40.0f, 120.0f );
			Vec2 translation = new(  length * cosf( angle ), length * sinf( angle )  );
			ENSURE( CompareRayCast( tree, proxyIds, TREE_TEST_PROXY_COUNT, origin, translation ) == 0 );

			Vec2 halfExtent = new(  RandomTreeFloat( 0.25f, 2.0f ), RandomTreeFloat( 0.25f, 2.0f )  );
			AABB box = new(  Sub( origin, halfExtent ), Add( origin, halfExtent )  );
			ENSURE( CompareBoxCast( tree, proxyIds, TREE_TEST_PROXY_COUNT, box, translation ) == 0 );
		}

		DestroyDynamicTree( tree );
		return 0;
	}

	// Marks reach the root from any leaf, the sweep refit writes exact unions, and the rebuild clears
	private static int TreeMarkAndRefitTest()
	{
		DynamicTree tree = CreateDynamicTree( TREE_TEST_PROXY_COUNT );
		Span<int> proxyIds = stackalloc int[TREE_TEST_PROXY_COUNT];
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
		{
			proxyIds[i] = CreateTreeProxyInternal( tree, RandomTreeBox( 2.0f ), 1, (ulong)i, false );
		}

		ENSURE( DynamicTree_Rebuild( tree, true ) == TREE_TEST_PROXY_COUNT );
		ENSURE( tree.dfsOrdered );
		ENSURE( tree.nodeEnd == 2 * TREE_TEST_PROXY_COUNT );
		ENSURE( RootMarked( tree ) == false );

		bool[] marked = { 0 };
		AABB[] enlarged = { 0 };
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; i += 3 )
		{
			int proxyId = proxyIds[i];
			enlarged[proxyId] = EnlargeTreeBox( DynamicTree_GetProxyBounds( tree, proxyId ), 0.5f );
			DynamicTree_MarkProxyMoved( tree, proxyId, enlarged[proxyId] );
			marked[proxyId] = true;
		}

		bool markedBelow = false;
		ENSURE( CheckMarks( tree, B2_ROOT_NODE, marked, ref markedBelow ) == 0 );
		ENSURE( markedBelow );
		ENSURE( RootMarked( tree ) );

		// The leaf records took the fat boxes before the refit
		for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
		{
			if ( marked[i] )
			{
				ENSURE( BoxEquals( DynamicTree_GetProxyBounds( tree, i ), enlarged[i] ) );
			}
		}

		DynamicTree_Refit( tree );
		DynamicTree_Validate( tree );
		AABB bounds = default;
		ENSURE( CheckBounds( tree, B2_ROOT_NODE, ref bounds ) == 0 );
		ENSURE( BoxEquals( bounds, DynamicTree_GetRootBounds( tree ) ) );

		// Marks survive the refit
		ENSURE( CheckMarks( tree, B2_ROOT_NODE, marked, ref markedBelow ) == 0 );
		ENSURE( markedBelow );
		for ( int i = 0; i < 16; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
		}

		// The rebuild clears every mark and restores the order
		ENSURE( DynamicTree_Rebuild( tree, false ) > 0 );
		ENSURE( RootMarked( tree ) == false );
		marked = default;
		ENSURE( CheckMarks( tree, B2_ROOT_NODE, marked, ref markedBelow ) == 0 );
		ENSURE( markedBelow == false );
		ENSURE( tree.dfsOrdered );
		ENSURE( CheckBounds( tree, B2_ROOT_NODE, ref bounds ) == 0 );
		for ( int i = 0; i < 16; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
		}

		// A single deep leaf still marks the whole path to the root
		int loneId = proxyIds[TREE_TEST_PROXY_COUNT / 2];
		enlarged[loneId] = EnlargeTreeBox( DynamicTree_GetProxyBounds( tree, loneId ), 0.5f );
		DynamicTree_MarkProxyMoved( tree, loneId, enlarged[loneId] );
		marked[loneId] = true;
		ENSURE( CheckMarks( tree, B2_ROOT_NODE, marked, ref markedBelow ) == 0 );
		ENSURE( markedBelow );
		DynamicTree_Refit( tree );
		ENSURE( CheckBounds( tree, B2_ROOT_NODE, ref bounds ) == 0 );
		ENSURE( DynamicTree_Rebuild( tree, false ) > 0 );
		DynamicTree_ValidateNoMoved( tree );

		DestroyDynamicTree( tree );
		return 0;
	}

	// The rebuild orders the array, a remove keeps the order, an insert that moves an internal node
	// below its children loses it, and the rebuild runs on an unordered array even when nothing moved
	private static int TreeDfsOrderTest()
	{
		int proxyCount = 50;
		DynamicTree tree = CreateDynamicTree( TREE_TEST_PROXY_COUNT );
		Span<int> proxyIds = stackalloc int[TREE_TEST_PROXY_COUNT];
		for ( int i = 0; i < proxyCount; ++i )
		{
			proxyIds[i] = CreateTreeProxyInternal( tree, RandomTreeBox( 2.0f ), 1, (ulong)i, false );
		}

		ENSURE( DynamicTree_Rebuild( tree, true ) == proxyCount );
		ENSURE( tree.dfsOrdered );
		ENSURE( tree.nodeEnd == 2 * proxyCount );
		ENSURE( tree.pairFreeList == NullIndex );

		// A fresh pair lands above everything, so the order survives when the sibling is a leaf.
		// Validate checks the flag against the array either way.
		proxyIds[proxyCount] = CreateTreeProxyInternal( tree, RandomTreeBox( 2.0f ), 1, (ulong)proxyCount, false );
		proxyCount += 1;
		DynamicTree_Validate( tree );

		ENSURE( DynamicTree_Rebuild( tree, true ) == proxyCount );
		ENSURE( tree.dfsOrdered );

		DynamicTree_MoveProxyInternal( tree, proxyIds[7], RandomTreeBox( 2.0f ), false );
		DynamicTree_Validate( tree );

		ENSURE( DynamicTree_Rebuild( tree, true ) == proxyCount );
		ENSURE( tree.dfsOrdered );
		int orderedEnd = tree.nodeEnd;

		// Removes leave holes but every survivor stays above its parent
		DestroyTreeProxy( tree, proxyIds[3] );
		DestroyTreeProxy( tree, proxyIds[20] );
		proxyIds[3] = proxyIds[proxyCount - 1];
		proxyIds[20] = proxyIds[proxyCount - 2];
		proxyCount -= 2;
		ENSURE( tree.dfsOrdered );
		ENSURE( tree.nodeEnd == orderedEnd );
		ENSURE( tree.pairFreeList != NullIndex );
		DynamicTree_Validate( tree );

		AABB enlarged = EnlargeTreeBox( DynamicTree_GetProxyBounds( tree, proxyIds[10] ), 0.5f );
		DynamicTree_MarkProxyMoved( tree, proxyIds[10], enlarged );
		ENSURE( RootMarked( tree ) );
		DynamicTree_Refit( tree );
		DynamicTree_Validate( tree );
		AABB bounds = default;
		ENSURE( CheckBounds( tree, B2_ROOT_NODE, ref bounds ) == 0 );
		ENSURE( BoxEquals( bounds, DynamicTree_GetRootBounds( tree ) ) );
		for ( int i = 0; i < 16; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, proxyCount, RandomTreeBox( 8.0f ) ) == 0 );
		}

		ENSURE( DynamicTree_Rebuild( tree, false ) > 0 );
		ENSURE( tree.dfsOrdered );
		ENSURE( tree.nodeEnd == 2 * proxyCount );
		ENSURE( tree.pairFreeList == NullIndex );
		DynamicTree_Validate( tree );

		// A box around everything makes the root the best sibling, so the root's subtree moves into
		// the new pair below its own children and the order is lost with nothing marked
		AABB huge = new( new( -100.0f, -100.0f ), new( 100.0f, 100.0f ) );
		proxyIds[proxyCount] = CreateTreeProxyInternal( tree, huge, 1, (ulong)proxyCount, false );
		proxyCount += 1;
		ENSURE( tree.dfsOrdered == false );
		ENSURE( RootMarked( tree ) == false );
		DynamicTree_Validate( tree );

		// The refit falls back to a descent on an unordered array
		enlarged = EnlargeTreeBox( DynamicTree_GetProxyBounds( tree, proxyIds[10] ), 0.5f );
		DynamicTree_MarkProxyMoved( tree, proxyIds[10], enlarged );
		DynamicTree_Refit( tree );
		DynamicTree_Validate( tree );
		ENSURE( CheckBounds( tree, B2_ROOT_NODE, ref bounds ) == 0 );
		DynamicTree_ClearMoved( tree );
		ENSURE( RootMarked( tree ) == false );

		// The rebuild runs for the order alone
		ENSURE( DynamicTree_Rebuild( tree, false ) > 0 );
		ENSURE( tree.dfsOrdered );
		ENSURE( tree.nodeEnd == 2 * proxyCount );
		DynamicTree_Validate( tree );
		for ( int i = 0; i < 16; ++i )
		{
			ENSURE( CompareQuery( tree, proxyIds, proxyCount, RandomTreeBox( 8.0f ) ) == 0 );
		}

		DestroyDynamicTree( tree );
		return 0;
	}

	// A create and destroy between steps leaves no marks. The rebuild has to run anyway when the
	// array lost its order, or the next enlarge and refit leave the ancestors stale.
	private static int TreeStaleAfterInsertRemoveTest()
	{
		DynamicTree tree = CreateDynamicTree( 16 );
		Span<int> ids = stackalloc int[16];
		for ( int i = 0; i < 16; ++i )
		{
			AABB box = new( new( 2.0f * i, 0.0f ), new( 2.0f * i + 1.0f, 1.0f ) );
			ids[i] = CreateTreeProxyInternal( tree, box, 1, (ulong)i, true );
		}

		ENSURE( DynamicTree_Rebuild( tree, false ) == 16 );

		// Control: enlarge and refit on the ordered tree
		AABB big = new( new( 0.0f, 0.0f ), new( 40.0f, 40.0f ) );
		DynamicTree_MarkProxyMoved( tree, ids[0], big );
		DynamicTree_Refit( tree );
		ENSURE( AABB_Contains( DynamicTree_GetRootBounds( tree ), big ) );
		ENSURE( DynamicTree_Rebuild( tree, false ) > 0 );

		// Insert and remove between steps leaves no marks
		AABB extra = new( new( 100.0f, 100.0f ), new( 101.0f, 101.0f ) );
		int extraId = CreateTreeProxyInternal( tree, extra, 1, 99, true );
		DestroyTreeProxy( tree, extraId );
		ENSURE( HasTreeMoved( tree ) == false );

		// The world's rebuild call, then a proxy enlarges during the step
		DynamicTree_Rebuild( tree, false );
		AABB bigger = new( new( -10.0f, -10.0f ), new( 60.0f, 60.0f ) );
		DynamicTree_MarkProxyMoved( tree, ids[0], bigger );
		DynamicTree_Refit( tree );
		ENSURE( AABB_Contains( DynamicTree_GetRootBounds( tree ), bigger ) );
		DynamicTree_Validate( tree );

		DestroyDynamicTree( tree );
		return 0;
	}

	internal static int DynamicTreeTest()
	{
		RUN_SUBTEST( TreeCreateDestroy );
		RUN_SUBTEST( TreeRayCastTest );
		RUN_SUBTEST( TreeMultipleProxiesTest );
		RUN_SUBTEST( TreeQueryTest );
		RUN_SUBTEST( TreeMoveAndEnlargeTest );
		RUN_SUBTEST( TreeRebuildAndValidateTest );
		RUN_SUBTEST( TreeRowHeightTest );
		RUN_SUBTEST( TreeGridHeightTest );
		RUN_SUBTEST( TreeGridMovementTest );
		RUN_SUBTEST( TreeOneProxyTest );
		RUN_SUBTEST( TreeLeafRootTest );
		RUN_SUBTEST( TreeQueryBruteForceTest );
		RUN_SUBTEST( TreeCastBruteForceTest );
		RUN_SUBTEST( TreeMarkAndRefitTest );
		RUN_SUBTEST( TreeDfsOrderTest );
		RUN_SUBTEST( TreeStaleAfterInsertRemoveTest );

		// todo test queries versus brute force

		return 0;
	}

}
