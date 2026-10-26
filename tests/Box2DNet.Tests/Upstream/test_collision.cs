// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class CollisionTests
{
	private static int AABBTest()
	{
		AABB a = default;
		a.lowerBound = new Vec2( -1.0f, -1.0f );
		a.upperBound = new Vec2( -2.0f, -2.0f );

		ENSURE( IsValidAABB( a ) == false );

		a.upperBound = new Vec2( 1.0f, 1.0f );
		ENSURE( IsValidAABB( a ) == true );

		AABB b = new AABB { lowerBound = new Vec2 { x = 2.0f, y = 2.0f }, upperBound = new Vec2 { x = 4.0f, y = 4.0f } };
		ENSURE( AABB_Overlaps( a, b ) == false );
		ENSURE( AABB_Contains( a, b ) == false );

		return 0;
	}

	private static int AABBRayCastTest()
	{
		// Test AABB centered at origin with bounds [-1, -1] to [1, 1]
		AABB aabb = new AABB { lowerBound = new Vec2 { x = -1.0f, y = -1.0f }, upperBound = new Vec2 { x = 1.0f, y = 1.0f } };

		// Test 1: Ray hits AABB from left side
		{
			Vec2 p1 = new(  -3.0f, 0.0f  );
			Vec2 p2 = new(  3.0f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.point.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.point.y, FLT_EPSILON );
		}

		// Test 2: Ray hits AABB from right side
		{
			Vec2 p1 = new(  3.0f, 0.0f  );
			Vec2 p2 = new(  -3.0f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x - 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.point.x - 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.point.y, FLT_EPSILON );
		}

		// Test 3: Ray hits AABB from bottom
		{
			Vec2 p1 = new(  0.0f, -3.0f  );
			Vec2 p2 = new(  0.0f, 3.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.point.x, FLT_EPSILON );
			ENSURE_SMALL( output.point.y + 1.0f, FLT_EPSILON );
		}

		// Test 4: Ray hits AABB from top
		{
			Vec2 p1 = new(  0.0f, 3.0f  );
			Vec2 p2 = new(  0.0f, -3.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y - 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.point.x, FLT_EPSILON );
			ENSURE_SMALL( output.point.y - 1.0f, FLT_EPSILON );
		}

		// Test 5: Ray misses AABB completely (parallel to x-axis)
		{
			Vec2 p1 = new(  -3.0f, 2.0f  );
			Vec2 p2 = new(  3.0f, 2.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 6: Ray misses AABB completely (parallel to y-axis)
		{
			Vec2 p1 = new(  2.0f, -3.0f  );
			Vec2 p2 = new(  2.0f, 3.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 7: Ray starts inside AABB
		{
			Vec2 p1 = new(  0.0f, 0.0f  );
			Vec2 p2 = new(  2.0f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 8: Ray hits corner of AABB (diagonal ray)
		{
			Vec2 p1 = new(  -2.0f, -2.0f  );
			Vec2 p2 = new(  2.0f, 2.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
			// Normal should be either (-1, 0) or (0, -1) depending on which edge is hit first
			ENSURE( ( output.normal.x == -1.0f && output.normal.y == 0.0f ) ||
					( output.normal.x == 0.0f && output.normal.y == -1.0f ) );
		}

		// Test 9: Ray parallel to AABB edge but outside
		{
			Vec2 p1 = new(  -2.0f, 1.5f  );
			Vec2 p2 = new(  2.0f, 1.5f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 10: Ray parallel to AABB edge and exactly on boundary
		{
			Vec2 p1 = new(  -2.0f, 1.0f  );
			Vec2 p2 = new(  2.0f, 1.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		}

		// Test 11: Very short ray that doesn't reach AABB
		{
			Vec2 p1 = new(  -3.0f, 0.0f  );
			Vec2 p2 = new(  -2.5f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 12: Zero-length ray (degenerate case)
		{
			Vec2 p1 = new(  0.0f, 0.0f  );
			Vec2 p2 = new(  0.0f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == false );
		}

		// Test 13: Ray hits AABB at exact boundary condition (t = 1.0)
		{
			Vec2 p1 = new(  -2.0f, 0.0f  );
			Vec2 p2 = new(  -1.0f, 0.0f  );
			CastOutput output = AABB_RayCast( aabb, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		}

		// Test 14: Different AABB position (not centered at origin)
		{
			AABB offsetAABB = new AABB { lowerBound = new Vec2 { x = 2.0f, y = 3.0f }, upperBound = new Vec2 { x = 4.0f, y = 5.0f } };
			Vec2 p1 = new(  0.0f, 4.0f  );
			Vec2 p2 = new(  6.0f, 4.0f  );
			CastOutput output = AABB_RayCast( offsetAABB, p1, p2 );

			ENSURE( output.hit == true );
			ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.point.x - 2.0f, FLT_EPSILON );
			ENSURE_SMALL( output.point.y - 4.0f, FLT_EPSILON );
		}

		return 0;
	}

	// The narrow phase differences the two world positions in double then works in frame A, so a
	// manifold far from the origin must match the same manifold at the origin. Float loses this past
	// ~1e7 m where the ULP grows larger than the overlap, which is the whole point of large world mode.
	private static int LargeWorldManifoldTest()
	{
		Polygon boxA = MakeBox( 0.5f, 0.5f );
		Polygon boxB = MakeBox( 0.5f, 0.5f );

		// Centers 0.9 apart so the boxes overlap by 0.1 along x
		Vec2 sep = new(  0.9f, 0.0f  );

		WorldTransform xfAo = WorldTransform_identity;
		WorldTransform xfBo = new WorldTransform { p = OffsetPos( Pos_zero, sep ), q = Rot_identity };
		LocalManifold mOrigin = CollidePolygons( ref boxA, ref boxB, InvMulWorldTransforms( xfAo, xfBo ) );

		ENSURE( mOrigin.pointCount == 2 );
		ENSURE_SMALL( mOrigin.points[0].separation + 0.1f, 0.01f );
		ENSURE_SMALL( mOrigin.points[1].separation + 0.1f, 0.01f );

#if BOX2D_DOUBLE_PRECISION
		// Same relative configuration shifted far from the origin. The relative pose differences the
		// world positions in double, so in double the frame A manifold is preserved to float precision.
		// In float it would collapse since the offset is below the ULP.
		Pos @base = OffsetPos( Pos_zero, new Vec2( 1.0e7f, 1.0e7f ) );
		WorldTransform xfAl = new WorldTransform { p = @base, q = Rot_identity };
		WorldTransform xfBl = new WorldTransform { p = OffsetPos( @base, sep ), q = Rot_identity };
		LocalManifold mLarge = CollidePolygons( ref boxA, ref boxB, InvMulWorldTransforms( xfAl, xfBl ) );

		ENSURE( mLarge.pointCount == mOrigin.pointCount );
		ENSURE_SMALL( mLarge.normal.x - mOrigin.normal.x, 1e-4f );
		ENSURE_SMALL( mLarge.normal.y - mOrigin.normal.y, 1e-4f );
		for ( int i = 0; i < mLarge.pointCount; ++i )
		{
			ENSURE_SMALL( mLarge.points[i].separation - mOrigin.points[i].separation, 1e-4f );
			ENSURE_SMALL( mLarge.points[i].point.x - mOrigin.points[i].point.x, 1e-4f );
			ENSURE_SMALL( mLarge.points[i].point.y - mOrigin.points[i].point.y, 1e-4f );
		}
#endif

		return 0;
	}

	// Broad-phase AABBs are built in double and narrowed to float with directed outward rounding, so a
	// shape and its speculative margin stay inside their box far from the origin. A float build would
	// round the extent away into the ULP (~1 m at 1e7) and clip the shape out of its own box.
	private static int LargeWorldAABBTest()
	{
		// Rounded box: 0.5 half extents plus 0.1 radius, so the tight extent is 0.6 each way
		Polygon box = MakeRoundedBox( 0.5f, 0.5f, 0.1f );

		AABB aabbOrigin = ComputePolygonBounds( ref box, WorldTransform_identity );
		ENSURE_SMALL( aabbOrigin.lowerBound.x + 0.6f, FLT_EPSILON );
		ENSURE_SMALL( aabbOrigin.lowerBound.y + 0.6f, FLT_EPSILON );
		ENSURE_SMALL( aabbOrigin.upperBound.x - 0.6f, FLT_EPSILON );
		ENSURE_SMALL( aabbOrigin.upperBound.y - 0.6f, FLT_EPSILON );

#if BOX2D_DOUBLE_PRECISION
		double d = 1.0e7;
		WorldTransform xfLarge = new WorldTransform { p = new Pos { x = d, y = d }, q = Rot_identity };

		// Tight world AABB still contains the 0.6 m extent
		AABB tight = ComputePolygonBounds( ref box, xfLarge );
		ENSURE( (double)tight.lowerBound.x <= d - 0.6 );
		ENSURE( (double)tight.lowerBound.y <= d - 0.6 );
		ENSURE( (double)tight.upperBound.x >= d + 0.6 );
		ENSURE( (double)tight.upperBound.y >= d + 0.6 );

		// The fat helper folds the extra into the double step before the single outward rounding, so a
		// margin smaller than a float ULP at this range survives instead of becoming a no-op subtract.
		float extra = 0.05f;
		Shape shape = default;
		shape.type = ShapeType.polygonShape;
		shape.polygon = box;
		AABB fat = ComputeFatShapeAABB( ref shape, xfLarge, extra );
		ENSURE( (double)fat.lowerBound.x <= d - 0.6 - (double)extra );
		ENSURE( (double)fat.lowerBound.y <= d - 0.6 - (double)extra );
		ENSURE( (double)fat.upperBound.x >= d + 0.6 + (double)extra );
		ENSURE( (double)fat.upperBound.y >= d + 0.6 + (double)extra );
#endif

		return 0;
	}

	internal static int CollisionTest()
	{
		RUN_SUBTEST( AABBTest );
		RUN_SUBTEST( AABBRayCastTest );
		RUN_SUBTEST( LargeWorldManifoldTest );
		RUN_SUBTEST( LargeWorldAABBTest );

		return 0;
	}

}
