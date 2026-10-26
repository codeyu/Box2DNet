// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;

namespace Box2DNet;

public static partial class B2
{
	/// Evaluate the transform sweep at a specific time.
	public static Transform GetSweepTransform( in Sweep sweep, float time )
	{
		// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
		Transform xf;
		xf.p = Add( MulSV( 1.0f - time, sweep.c1 ), MulSV( time, sweep.c2 ) );

		Rot q = new( ( 1.0f - time ) * sweep.q1.c + time * sweep.q2.c, ( 1.0f - time ) * sweep.q1.s + time * sweep.q2.s );

		xf.q = NormalizeRot( q );

		// Shift to origin
		xf.p = Sub( xf.p, RotateVector( xf.q, sweep.localCenter ) );
		return xf;
	}

	/// Compute the distance between two line segments, clamping at the end points if needed.
	/// Follows Ericson 5.1.9 Closest Points of Two Line Segments
	public static SegmentDistanceResult ComputeSegmentDistance( Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2 )
	{
		SegmentDistanceResult result = default;

		Vec2 d1 = Sub( q1, p1 );
		Vec2 d2 = Sub( q2, p2 );
		Vec2 r = Sub( p1, p2 );
		float dd1 = Dot( d1, d1 );
		float dd2 = Dot( d2, d2 );
		float rd1 = Dot( r, d1 );
		float rd2 = Dot( r, d2 );

		const float epsSqr = FLT_EPSILON * FLT_EPSILON;

		if ( dd1 < epsSqr || dd2 < epsSqr )
		{
			// Handle all degeneracies
			if ( dd1 >= epsSqr )
			{
				// Segment 2 is degenerate
				result.fraction1 = ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
				result.fraction2 = 0.0f;
			}
			else if ( dd2 >= epsSqr )
			{
				// Segment 1 is degenerate
				result.fraction1 = 0.0f;
				result.fraction2 = ClampFloat( rd2 / dd2, 0.0f, 1.0f );
			}
			else
			{
				result.fraction1 = 0.0f;
				result.fraction2 = 0.0f;
			}
		}
		else
		{
			// Non-degenerate segments
			float d12 = Dot( d1, d2 );

			float denominator = dd1 * dd2 - d12 * d12;

			// Fraction on segment 1
			float f1 = 0.0f;
			if ( denominator != 0.0f )
			{
				// not parallel
				f1 = ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denominator, 0.0f, 1.0f );
			}

			// Compute point on segment 2 closest to p1 + f1 * d1
			float f2 = ( d12 * f1 + rd2 ) / dd2;

			// Clamping of segment 2 requires a do over on segment 1
			if ( f2 < 0.0f )
			{
				f2 = 0.0f;
				f1 = ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
			}
			else if ( f2 > 1.0f )
			{
				f2 = 1.0f;
				f1 = ClampFloat( ( d12 - rd1 ) / dd1, 0.0f, 1.0f );
			}

			result.fraction1 = f1;
			result.fraction2 = f2;
		}

		result.closest1 = MulAdd( p1, result.fraction1, d1 );
		result.closest2 = MulAdd( p2, result.fraction2, d2 );
		result.distanceSquared = DistanceSquared( result.closest1, result.closest2 );
		return result;
	}

	/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the
	/// points.
	public static ShapeProxy MakeProxy( ReadOnlySpan<Vec2> points, int count, float radius )
	{
		count = MinInt( count, MaxPolygonVertices );
		ShapeProxy proxy = default;
		for ( int i = 0; i < count; ++i )
		{
			proxy.points[i] = points[i];
		}

		proxy.count = count;
		proxy.radius = radius;
		return proxy;
	}

	/// Make a proxy for use in overlap, shape cast, and related functions.
	public static ShapeProxy MakeProxy( ReadOnlySpan<Vec2> points, float radius ) => MakeProxy( points, points.Length, radius );

	/// Make a proxy from consecutive points in memory, such as a single point or the two centers of a
	/// capsule. This mirrors passing a pointer in C.
	public static ShapeProxy MakeProxy( ref readonly Vec2 points, int count, float radius )
	{
		return MakeProxy( System.Runtime.InteropServices.MemoryMarshal.CreateReadOnlySpan( in points, count ), count, radius );
	}

	// Make a proxy from two points, such as a capsule or segment.
	internal static ShapeProxy MakeProxy2( Vec2 p1, Vec2 p2, float radius )
	{
		ShapeProxy proxy = default;
		proxy.points[0] = p1;
		proxy.points[1] = p2;
		proxy.count = 2;
		proxy.radius = radius;
		return proxy;
	}

	/// Make a proxy with a transform. This is a deep copy of the points.
	public static ShapeProxy MakeOffsetProxy( ReadOnlySpan<Vec2> points, int count, float radius, Vec2 position, Rot rotation )
	{
		count = MinInt( count, MaxPolygonVertices );
		Transform transform = new( position, rotation );
		ShapeProxy proxy = default;
		for ( int i = 0; i < count; ++i )
		{
			proxy.points[i] = TransformPoint( transform, points[i] );
		}

		proxy.count = count;
		proxy.radius = radius;
		return proxy;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static Vec2 Weight2( float a1, Vec2 w1, float a2, Vec2 w2 )
	{
		return new Vec2( a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static Vec2 Weight3( float a1, Vec2 w1, float a2, Vec2 w2, float a3, Vec2 w3 )
	{
		return new Vec2( a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static int FindSupport( in ShapeProxy proxy, Vec2 direction )
	{
		ReadOnlySpan<Vec2> points = proxy.points;
		int count = proxy.count;

		int bestIndex = 0;
		float bestValue = Dot( points[0], direction );
		for ( int i = 1; i < count; ++i )
		{
			float value = Dot( points[i], direction );
			if ( value > bestValue )
			{
				bestIndex = i;
				bestValue = value;
			}
		}

		return bestIndex;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static ref SimplexVertex GetSimplexVertex( ref Simplex s, int index )
	{
		switch ( index )
		{
			case 0:
				return ref s.v1;
			case 1:
				return ref s.v2;
			default:
				return ref s.v3;
		}
	}

	private static Simplex MakeSimplexFromCache( in SimplexCache cache, in ShapeProxy proxyA, in ShapeProxy proxyB )
	{
		B2_ASSERT( cache.count <= 3 );
		Simplex s = default;

		// Copy data from cache.
		s.count = cache.count;

		for ( int i = 0; i < s.count; ++i )
		{
			ref SimplexVertex v = ref GetSimplexVertex( ref s, i );
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			v.wA = proxyA.points[v.indexA];
			v.wB = proxyB.points[v.indexB];
			v.w = Sub( v.wA, v.wB );

			// invalid
			v.a = -1.0f;
		}

		// If the cache is empty or invalid ...
		if ( s.count == 0 )
		{
			ref SimplexVertex v = ref s.v1;
			v.indexA = 0;
			v.indexB = 0;
			v.wA = proxyA.points[0];
			v.wB = proxyB.points[0];
			v.w = Sub( v.wA, v.wB );
			v.a = 1.0f;
			s.count = 1;
		}

		return s;
	}

	private static SimplexCache MakeSimplexCache( ref Simplex simplex )
	{
		SimplexCache cache = default;
		cache.count = (ushort)simplex.count;
		for ( int i = 0; i < simplex.count; ++i )
		{
			ref SimplexVertex v = ref GetSimplexVertex( ref simplex, i );
			cache.indexA[i] = (byte)v.indexA;
			cache.indexB[i] = (byte)v.indexB;
		}

		return cache;
	}

	private static void ComputeWitnessPoints( in Simplex s, out Vec2 a, out Vec2 b )
	{
		switch ( s.count )
		{
			case 1:
				a = s.v1.wA;
				b = s.v1.wB;
				break;

			case 2:
				a = Weight2( s.v1.a, s.v1.wA, s.v2.a, s.v2.wA );
				b = Weight2( s.v1.a, s.v1.wB, s.v2.a, s.v2.wB );
				break;

			case 3:
				a = Weight3( s.v1.a, s.v1.wA, s.v2.a, s.v2.wA, s.v3.a, s.v3.wA );
				// todo why are these not equal?
				//*b = b2Weight3(s->v1.a, s->v1.wB, s->v2.a, s->v2.wB, s->v3.a, s->v3.wB);
				b = a;
				break;

			default:
				a = Vec2_zero;
				b = Vec2_zero;
				B2_ASSERT( false );
				break;
		}
	}

	// Solve a line segment using barycentric coordinates.
	//
	// p = a1 * w1 + a2 * w2
	// a1 + a2 = 1
	//
	// The vector from the origin to the closest point on the line is
	// perpendicular to the line.
	// e12 = w2 - w1
	// dot(p, e) = 0
	// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
	//
	// 2-by-2 linear system
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	//
	// Define
	// d12_1 =  dot(w2, e12)
	// d12_2 = -dot(w1, e12)
	// d12 = d12_1 + d12_2
	//
	// Solution
	// a1 = d12_1 / d12
	// a2 = d12_2 / d12
	//
	// returns a vector that points towards the origin
	private static Vec2 SolveSimplex2( ref Simplex s )
	{
		Vec2 w1 = s.v1.w;
		Vec2 w2 = s.v2.w;
		Vec2 e12 = Sub( w2, w1 );

		// w1 region
		float d12_2 = -Dot( w1, e12 );
		if ( d12_2 <= 0.0f )
		{
			// a2 <= 0, so we clamp it to 0
			s.v1.a = 1.0f;
			s.count = 1;
			return Neg( w1 );
		}

		// w2 region
		float d12_1 = Dot( w2, e12 );
		if ( d12_1 <= 0.0f )
		{
			// a1 <= 0, so we clamp it to 0
			s.v2.a = 1.0f;
			s.count = 1;
			s.v1 = s.v2;
			return Neg( w2 );
		}

		// Must be in e12 region.
		float inv_d12 = 1.0f / ( d12_1 + d12_2 );
		s.v1.a = d12_1 * inv_d12;
		s.v2.a = d12_2 * inv_d12;
		s.count = 2;
		return CrossSV( Cross( Add( w1, w2 ), e12 ), e12 );
	}

	private static Vec2 SolveSimplex3( ref Simplex s )
	{
		Vec2 w1 = s.v1.w;
		Vec2 w2 = s.v2.w;
		Vec2 w3 = s.v3.w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		Vec2 e12 = Sub( w2, w1 );
		float w1e12 = Dot( w1, e12 );
		float w2e12 = Dot( w2, e12 );
		float d12_1 = w2e12;
		float d12_2 = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		Vec2 e13 = Sub( w3, w1 );
		float w1e13 = Dot( w1, e13 );
		float w3e13 = Dot( w3, e13 );
		float d13_1 = w3e13;
		float d13_2 = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		Vec2 e23 = Sub( w3, w2 );
		float w2e23 = Dot( w2, e23 );
		float w3e23 = Dot( w3, e23 );
		float d23_1 = w3e23;
		float d23_2 = -w2e23;

		// Triangle123
		float n123 = Cross( e12, e13 );

		float d123_1 = n123 * Cross( w2, w3 );
		float d123_2 = n123 * Cross( w3, w1 );
		float d123_3 = n123 * Cross( w1, w2 );

		// w1 region
		if ( d12_2 <= 0.0f && d13_2 <= 0.0f )
		{
			s.v1.a = 1.0f;
			s.count = 1;
			return Neg( w1 );
		}

		// e12
		if ( d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f )
		{
			float inv_d12 = 1.0f / ( d12_1 + d12_2 );
			s.v1.a = d12_1 * inv_d12;
			s.v2.a = d12_2 * inv_d12;
			s.count = 2;
			return CrossSV( Cross( Add( w1, w2 ), e12 ), e12 );
		}

		// e13
		if ( d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f )
		{
			float inv_d13 = 1.0f / ( d13_1 + d13_2 );
			s.v1.a = d13_1 * inv_d13;
			s.v3.a = d13_2 * inv_d13;
			s.count = 2;
			s.v2 = s.v3;
			return CrossSV( Cross( Add( w1, w3 ), e13 ), e13 );
		}

		// w2 region
		if ( d12_1 <= 0.0f && d23_2 <= 0.0f )
		{
			s.v2.a = 1.0f;
			s.count = 1;
			s.v1 = s.v2;
			return Neg( w2 );
		}

		// w3 region
		if ( d13_1 <= 0.0f && d23_1 <= 0.0f )
		{
			s.v3.a = 1.0f;
			s.count = 1;
			s.v1 = s.v3;
			return Neg( w3 );
		}

		// e23
		if ( d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f )
		{
			float inv_d23 = 1.0f / ( d23_1 + d23_2 );
			s.v2.a = d23_1 * inv_d23;
			s.v3.a = d23_2 * inv_d23;
			s.count = 2;
			s.v1 = s.v3;
			return CrossSV( Cross( Add( w2, w3 ), e23 ), e23 );
		}

		// Must be in triangle123
		float inv_d123 = 1.0f / ( d123_1 + d123_2 + d123_3 );
		s.v1.a = d123_1 * inv_d123;
		s.v2.a = d123_2 * inv_d123;
		s.v3.a = d123_3 * inv_d123;
		s.count = 3;

		// No search direction
		return Vec2_zero;
	}

	/// Compute the closest points between two shapes represented as point clouds. b2SimplexCache cache
	/// is input/output. On the first call set b2SimplexCache.count to zero. The underlying GJK
	/// algorithm may be debugged by passing in debug simplexes. You may pass an empty span.
	// Uses GJK for computing the distance between convex shapes.
	// https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
	public static DistanceOutput ComputeShapeDistance( in DistanceInput input, ref SimplexCache cache, Span<Simplex> simplexes )
	{
		B2_ASSERT( input.proxyA.count > 0 && input.proxyB.count > 0 );
		B2_ASSERT( input.proxyA.radius >= 0.0f );
		B2_ASSERT( input.proxyB.radius >= 0.0f );

		DistanceOutput output = default;

		ref readonly ShapeProxy proxyA = ref input.proxyA;

		// Get proxyB in frame A to avoid further transforms in the main loop.
		// This is still a performance gain at 8 points.
		ShapeProxy localProxyB = default;
		{
			localProxyB.count = input.proxyB.count;
			localProxyB.radius = input.proxyB.radius;
			for ( int i = 0; i < localProxyB.count; ++i )
			{
				localProxyB.points[i] = TransformPoint( input.transform, input.proxyB.points[i] );
			}
		}

		// Initialize the simplex.
		Simplex simplex = MakeSimplexFromCache( cache, proxyA, localProxyB );

		int simplexIndex = 0;
		int simplexCapacity = simplexes.Length;
		if ( simplexIndex < simplexCapacity )
		{
			simplexes[simplexIndex] = simplex;
			simplexIndex += 1;
		}

		Vec2 nonUnitNormal = Vec2_zero;

		// These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
		FixedArray3<int> saveA = default, saveB = default;

		// Main iteration loop. All computations are done in frame A.
		const int maxIterations = 20;
		int iteration = 0;
		while ( iteration < maxIterations )
		{
			// Copy simplex so we can identify duplicates.
			int saveCount = simplex.count;
			for ( int i = 0; i < saveCount; ++i )
			{
				ref SimplexVertex sv = ref GetSimplexVertex( ref simplex, i );
				saveA[i] = sv.indexA;
				saveB[i] = sv.indexB;
			}

			Vec2 d = default;
			switch ( simplex.count )
			{
				case 1:
					d = Neg( simplex.v1.w );
					break;

				case 2:
					d = SolveSimplex2( ref simplex );
					break;

				case 3:
					d = SolveSimplex3( ref simplex );
					break;

				default:
					B2_ASSERT( false );
					break;
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if ( simplex.count == 3 )
			{
				// Overlap
				cache = MakeSimplexCache( ref simplex );
				ComputeWitnessPoints( simplex, out Vec2 localPointA, out Vec2 localPointB );
				output.distance = 0.0f;
				output.pointA = localPointA;
				output.pointB = localPointB;
				return output;
			}

#if DEBUG
			if ( simplexIndex < simplexCapacity )
			{
				simplexes[simplexIndex] = simplex;
				simplexIndex += 1;
			}
#endif

			// Ensure the search direction is numerically fit.
			if ( Dot( d, d ) < 1000.0f * FLT_MIN )
			{
				// This is unlikely but could lead to bad cycling.
				// The branch predictor seems to make this check have low cost.

				// The origin is probably contained by a line segment
				// or triangle. Thus the shapes are overlapped.

				cache = MakeSimplexCache( ref simplex );

				// Must return overlap due to invalid normal.
				ComputeWitnessPoints( simplex, out Vec2 localPointA, out Vec2 localPointB );
				output.distance = 0.0f;
				output.pointA = localPointA;
				output.pointB = localPointB;
				return output;
			}

			// Save the normal
			nonUnitNormal = d;

			// Compute a tentative new simplex vertex using support points.
			// support = support(a, d) - support(b, -d)
			ref SimplexVertex vertex = ref GetSimplexVertex( ref simplex, simplex.count );
			vertex.indexA = FindSupport( proxyA, d );
			vertex.wA = proxyA.points[vertex.indexA];
			vertex.indexB = FindSupport( localProxyB, Neg( d ) );
			vertex.wB = localProxyB.points[vertex.indexB];
			vertex.w = Sub( vertex.wA, vertex.wB );

			// Iteration count is equated to the number of support point calls.
			++iteration;

			// Check for duplicate support points. This is the main termination criteria.
			bool duplicate = false;
			for ( int i = 0; i < saveCount; ++i )
			{
				if ( vertex.indexA == saveA[i] && vertex.indexB == saveB[i] )
				{
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exit to avoid cycling.
			if ( duplicate )
			{
				break;
			}

			// New vertex is valid and needed.
			simplex.count += 1;
		}

#if DEBUG
		if ( simplexIndex < simplexCapacity )
		{
			simplexes[simplexIndex] = simplex;
			simplexIndex += 1;
		}
#endif

		// Prepare output in frame A
		Vec2 normal = Normalize( nonUnitNormal );
		B2_ASSERT( IsNormalized( normal ) );

		{
			ComputeWitnessPoints( simplex, out Vec2 localPointA, out Vec2 localPointB );
			output.normal = normal;
			output.distance = Distance( localPointA, localPointB );
			output.pointA = localPointA;
			output.pointB = localPointB;
			output.iterations = iteration;
			output.simplexCount = simplexIndex;
		}

		// Cache the simplex
		cache = MakeSimplexCache( ref simplex );

		// Apply radii if requested
		if ( input.useRadii )
		{
			float radiusA = input.proxyA.radius;
			float radiusB = input.proxyB.radius;
			output.distance = MaxFloat( 0.0f, output.distance - radiusA - radiusB );

			// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
			output.pointA = MulAdd( output.pointA, radiusA, normal );
			output.pointB = MulSub( output.pointB, radiusB, normal );
		}

		return output;
	}

	/// Compute the closest points between two shapes represented as point clouds.
	public static DistanceOutput ComputeShapeDistance( in DistanceInput input, ref SimplexCache cache )
	{
		return ComputeShapeDistance( input, ref cache, Span<Simplex>.Empty );
	}

	/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point,
	/// normal, and translation fraction. The query runs in frame A, so the hit point and normal are
	/// returned in frame A. Initially touching shapes return a zero fraction in the output.
	// Shape cast using conservative advancement
	public static CastOutput CastShape( in ShapeCastPairInput input )
	{
		// Compute tolerance
		float linearSlop = LinearSlop;
		float totalRadius = input.proxyA.radius + input.proxyB.radius;
		float target = MaxFloat( linearSlop, totalRadius - linearSlop );
		float tolerance = 0.25f * linearSlop;

		B2_ASSERT( target > tolerance );

		// Prepare input for distance query
		SimplexCache cache = default;

		float fraction = 0.0f;

		DistanceInput distanceInput = default;
		distanceInput.proxyA = input.proxyA;
		distanceInput.proxyB = input.proxyB;
		distanceInput.useRadii = false;

		// The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
		// which keeps the math near the local origin and avoids re-relativizing world poses.
		distanceInput.transform = input.transform;

		Vec2 delta2 = input.translationB;
		CastOutput output = default;

		int iteration = 0;
		const int maxIterations = 20;

		for ( ; iteration < maxIterations; ++iteration )
		{
			output.iterations += 1;

			DistanceOutput distanceOutput = ComputeShapeDistance( distanceInput, ref cache, Span<Simplex>.Empty );

			if ( distanceOutput.distance < target + tolerance )
			{
				if ( iteration == 0 )
				{
					if ( input.canEncroach && distanceOutput.distance > 2.0f * linearSlop )
					{
						target = distanceOutput.distance - linearSlop;
					}
					else
					{
						// Initial overlap
						output.hit = true;

						// Compute a common point
						Vec2 c1 = MulAdd( distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal );
						Vec2 c2 = MulAdd( distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal );
						output.point = Lerp( c1, c2, 0.5f );
						return output;
					}
				}
				else
				{
					// Regular hit
					B2_ASSERT( distanceOutput.distance > 0.0f && IsNormalized( distanceOutput.normal ) );
					output.fraction = fraction;
					output.point = MulAdd( distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal );
					output.normal = distanceOutput.normal;
					output.hit = true;
					return output;
				}
			}

			B2_ASSERT( distanceOutput.distance > 0.0f );
			B2_ASSERT( IsNormalized( distanceOutput.normal ) );

			// Check if shapes are approaching each other
			float denominator = Dot( delta2, distanceOutput.normal );
			if ( denominator >= 0.0f )
			{
				// Miss
				return output;
			}

			// Advance sweep
			fraction += ( target - distanceOutput.distance ) / denominator;
			if ( fraction >= input.maxFraction )
			{
				// Miss
				return output;
			}

			distanceInput.transform.p = MulAdd( input.transform.p, fraction, delta2 );
		}

		// Failure!
		return output;
	}

	private enum SeparationType
	{
		Points,
		FaceA,
		FaceB
	}

	private struct SeparationFunction
	{
		public ShapeProxy proxyA;
		public ShapeProxy proxyB;
		public Sweep sweepA, sweepB;
		public Vec2 localPoint;
		public Vec2 axis;
		public SeparationType type;
	}

	private static SeparationFunction MakeSeparationFunction( in SimplexCache cache, in ShapeProxy proxyA, in Sweep sweepA,
															  in ShapeProxy proxyB, in Sweep sweepB, float t1 )
	{
		SeparationFunction f = default;

		f.proxyA = proxyA;
		f.proxyB = proxyB;
		int count = cache.count;
		B2_ASSERT( 0 < count && count < 3 );

		f.sweepA = sweepA;
		f.sweepB = sweepB;

		Transform xfA = GetSweepTransform( sweepA, t1 );
		Transform xfB = GetSweepTransform( sweepB, t1 );

		if ( count == 1 )
		{
			f.type = SeparationType.Points;
			Vec2 localPointA = proxyA.points[cache.indexA[0]];
			Vec2 localPointB = proxyB.points[cache.indexB[0]];
			Vec2 pointA = TransformPoint( xfA, localPointA );
			Vec2 pointB = TransformPoint( xfB, localPointB );
			f.axis = Normalize( Sub( pointB, pointA ) );
			f.localPoint = Vec2_zero;
			return f;
		}

		if ( cache.indexA[0] == cache.indexA[1] )
		{
			// Two points on B and one on A.
			f.type = SeparationType.FaceB;
			Vec2 localPointB1 = proxyB.points[cache.indexB[0]];
			Vec2 localPointB2 = proxyB.points[cache.indexB[1]];

			f.axis = CrossVS( Sub( localPointB2, localPointB1 ), 1.0f );
			f.axis = Normalize( f.axis );
			Vec2 normal = RotateVector( xfB.q, f.axis );

			f.localPoint = new Vec2( 0.5f * ( localPointB1.x + localPointB2.x ), 0.5f * ( localPointB1.y + localPointB2.y ) );
			Vec2 pointB = TransformPoint( xfB, f.localPoint );

			Vec2 localPointA = proxyA.points[cache.indexA[0]];
			Vec2 pointA = TransformPoint( xfA, localPointA );

			float s = Dot( Sub( pointA, pointB ), normal );
			if ( s < 0.0f )
			{
				f.axis = Neg( f.axis );
			}

			return f;
		}

		{
			// Two points on A and one or two points on B.
			f.type = SeparationType.FaceA;
			Vec2 localPointA1 = proxyA.points[cache.indexA[0]];
			Vec2 localPointA2 = proxyA.points[cache.indexA[1]];

			f.axis = CrossVS( Sub( localPointA2, localPointA1 ), 1.0f );
			f.axis = Normalize( f.axis );
			Vec2 normal = RotateVector( xfA.q, f.axis );

			f.localPoint = new Vec2( 0.5f * ( localPointA1.x + localPointA2.x ), 0.5f * ( localPointA1.y + localPointA2.y ) );
			Vec2 pointA = TransformPoint( xfA, f.localPoint );

			Vec2 localPointB = proxyB.points[cache.indexB[0]];
			Vec2 pointB = TransformPoint( xfB, localPointB );

			float s = Dot( Sub( pointB, pointA ), normal );
			if ( s < 0.0f )
			{
				f.axis = Neg( f.axis );
			}

			return f;
		}
	}

	private static float FindMinSeparation( in SeparationFunction f, out int indexA, out int indexB, float t )
	{
		Transform xfA = GetSweepTransform( f.sweepA, t );
		Transform xfB = GetSweepTransform( f.sweepB, t );

		switch ( f.type )
		{
			case SeparationType.Points:
			{
				Vec2 axisA = InvRotateVector( xfA.q, f.axis );
				Vec2 axisB = InvRotateVector( xfB.q, Neg( f.axis ) );

				indexA = FindSupport( f.proxyA, axisA );
				indexB = FindSupport( f.proxyB, axisB );

				Vec2 localPointA = f.proxyA.points[indexA];
				Vec2 localPointB = f.proxyB.points[indexB];

				Vec2 pointA = TransformPoint( xfA, localPointA );
				Vec2 pointB = TransformPoint( xfB, localPointB );

				float separation = Dot( Sub( pointB, pointA ), f.axis );
				return separation;
			}

			case SeparationType.FaceA:
			{
				Vec2 normal = RotateVector( xfA.q, f.axis );
				Vec2 pointA = TransformPoint( xfA, f.localPoint );

				Vec2 axisB = InvRotateVector( xfB.q, Neg( normal ) );

				indexA = -1;
				indexB = FindSupport( f.proxyB, axisB );

				Vec2 localPointB = f.proxyB.points[indexB];
				Vec2 pointB = TransformPoint( xfB, localPointB );

				float separation = Dot( Sub( pointB, pointA ), normal );
				return separation;
			}

			case SeparationType.FaceB:
			{
				Vec2 normal = RotateVector( xfB.q, f.axis );
				Vec2 pointB = TransformPoint( xfB, f.localPoint );

				Vec2 axisA = InvRotateVector( xfA.q, Neg( normal ) );

				indexB = -1;
				indexA = FindSupport( f.proxyA, axisA );

				Vec2 localPointA = f.proxyA.points[indexA];
				Vec2 pointA = TransformPoint( xfA, localPointA );

				float separation = Dot( Sub( pointA, pointB ), normal );
				return separation;
			}

			default:
				B2_ASSERT( false );
				indexA = -1;
				indexB = -1;
				return 0.0f;
		}
	}

	private static float EvaluateSeparation( in SeparationFunction f, int indexA, int indexB, float t )
	{
		Transform xfA = GetSweepTransform( f.sweepA, t );
		Transform xfB = GetSweepTransform( f.sweepB, t );

		switch ( f.type )
		{
			case SeparationType.Points:
			{
				Vec2 localPointA = f.proxyA.points[indexA];
				Vec2 localPointB = f.proxyB.points[indexB];

				Vec2 pointA = TransformPoint( xfA, localPointA );
				Vec2 pointB = TransformPoint( xfB, localPointB );

				float separation = Dot( Sub( pointB, pointA ), f.axis );
				return separation;
			}

			case SeparationType.FaceA:
			{
				Vec2 normal = RotateVector( xfA.q, f.axis );
				Vec2 pointA = TransformPoint( xfA, f.localPoint );

				Vec2 localPointB = f.proxyB.points[indexB];
				Vec2 pointB = TransformPoint( xfB, localPointB );

				float separation = Dot( Sub( pointB, pointA ), normal );
				return separation;
			}

			case SeparationType.FaceB:
			{
				Vec2 normal = RotateVector( xfB.q, f.axis );
				Vec2 pointB = TransformPoint( xfB, f.localPoint );

				Vec2 localPointA = f.proxyA.points[indexA];
				Vec2 pointA = TransformPoint( xfA, localPointA );

				float separation = Dot( Sub( pointA, pointB ), normal );
				return separation;
			}

			default:
				B2_ASSERT( false );
				return 0.0f;
		}
	}

	/// Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
	/// between [0,maxFraction]. This uses a swept separating axis and may miss some intermediate,
	/// non-tunneling collisions. If you change the time interval, you should call this function again.
	// CCD via the local separating axis method. This seeks progression
	// by computing the largest time at which separation is maintained.
	public static TOIOutput ComputeTimeOfImpact( in TOIInput input )
	{
		TOIOutput output = default;
		output.state = TOIState.toiStateUnknown;
		output.fraction = input.maxFraction;

		Sweep sweepA = input.sweepA;
		Sweep sweepB = input.sweepB;
		B2_ASSERT( IsNormalizedRot( sweepA.q1 ) && IsNormalizedRot( sweepA.q2 ) );
		B2_ASSERT( IsNormalizedRot( sweepB.q1 ) && IsNormalizedRot( sweepB.q2 ) );

		ref readonly ShapeProxy proxyA = ref input.proxyA;
		ref readonly ShapeProxy proxyB = ref input.proxyB;

		float tMax = input.maxFraction;

		// Setup target distance and tolerance
		float totalRadius = proxyA.radius + proxyB.radius;
		float target = MaxFloat( LinearSlop, totalRadius - LinearSlop );
		float tolerance = 0.25f * LinearSlop;
		B2_ASSERT( target > tolerance );

		float t1 = 0.0f;
		const int k_maxIterations = 20;
		int distanceIterations = 0;

		// Prepare input for distance query.
		SimplexCache cache = default;
		DistanceInput distanceInput = default;
		distanceInput.proxyA = input.proxyA;
		distanceInput.proxyB = input.proxyB;
		distanceInput.useRadii = false;

		// The outer loop progressively attempts to compute new separating axes.
		// This loop terminates when an axis is repeated (no progress is made).
		for ( ;; )
		{
			// Get the distance between shapes. We can also use the results to get a separating axis.
			Transform xfA = GetSweepTransform( sweepA, t1 );
			Transform xfB = GetSweepTransform( sweepB, t1 );
			distanceInput.transform = InvMulTransforms( xfA, xfB );
			DistanceOutput distanceOutput = ComputeShapeDistance( distanceInput, ref cache, Span<Simplex>.Empty );

			// The distance query runs in frame A, project the witness data back to world
			Vec2 worldNormal = RotateVector( xfA.q, distanceOutput.normal );
			Vec2 worldPointA = TransformPoint( xfA, distanceOutput.pointA );
			Vec2 worldPointB = TransformPoint( xfA, distanceOutput.pointB );

			distanceIterations += 1;

			// If the shapes are overlapped, we give up on continuous collision.
			if ( distanceOutput.distance <= 0.0f )
			{
				// Failure!
				output.state = TOIState.toiStateOverlapped;
				output.fraction = 0.0f;
				break;
			}

			if ( distanceOutput.distance <= target + tolerance )
			{
				// Success!
				output.state = TOIState.toiStateHit;

				// Averaged hit point
				Vec2 pA = MulAdd( worldPointA, proxyA.radius, worldNormal );
				Vec2 pB = MulAdd( worldPointB, -proxyB.radius, worldNormal );
				output.point = Lerp( pA, pB, 0.5f );
				output.normal = worldNormal;
				output.fraction = t1;
				break;
			}

			// Initialize the separating axis.
			SeparationFunction fcn = MakeSeparationFunction( cache, proxyA, sweepA, proxyB, sweepB, t1 );

			// Compute the TOI on the separating axis. We do this by successively
			// resolving the deepest point. This loop is bounded by the number of vertices.
			bool done = false;
			float t2 = tMax;
			int pushBackIterations = 0;
			for ( ;; )
			{
				// Find the deepest point at t2. Store the witness point indices.
				float s2 = FindMinSeparation( fcn, out int indexA, out int indexB, t2 );

				// Is the final configuration separated?
				if ( s2 > target + tolerance )
				{
					// Victory!
					output.state = TOIState.toiStateSeparated;
					output.fraction = tMax;
					done = true;
					break;
				}

				// Has the separation reached tolerance?
				if ( s2 > target - tolerance )
				{
					// Advance the sweeps
					t1 = t2;
					break;
				}

				// Compute the initial separation of the witness points.
				float s1 = EvaluateSeparation( fcn, indexA, indexB, t1 );

				// Check for initial overlap. This might happen if the root finder runs out of iterations.
				if ( s1 < target - tolerance )
				{
					output.state = TOIState.toiStateFailed;
					output.fraction = t1;
					done = true;
					break;
				}

				// Check for touching
				if ( s1 <= target + tolerance )
				{
					// Success! t1 should hold the TOI (could be 0.0).
					output.state = TOIState.toiStateHit;

					// Averaged hit point
					Vec2 pA = MulAdd( worldPointA, proxyA.radius, worldNormal );
					Vec2 pB = MulAdd( worldPointB, -proxyB.radius, worldNormal );
					output.point = Lerp( pA, pB, 0.5f );
					output.normal = worldNormal;
					output.fraction = t1;
					done = true;
					break;
				}

				// Compute 1D root of: f(x) - target = 0
				int rootIterationCount = 0;
				float a1 = t1, a2 = t2;
				for ( ;; )
				{
					// Use a mix of false position and bisection.
					float t;
					if ( ( rootIterationCount & 1 ) != 0 )
					{
						// False position to improve convergence.
						t = a1 + ( target - s1 ) * ( a2 - a1 ) / ( s2 - s1 );
					}
					else
					{
						// Bisection to guarantee progress.
						t = 0.5f * ( a1 + a2 );
					}

					rootIterationCount += 1;

					float s = EvaluateSeparation( fcn, indexA, indexB, t );

					// Has the separation reached tolerance?
					if ( AbsFloat( s - target ) < tolerance )
					{
						// t2 holds a tentative value for t1
						t2 = t;
						break;
					}

					// Ensure we continue to bracket the root.
					if ( s > target )
					{
						a1 = t;
						s1 = s;
					}
					else
					{
						a2 = t;
						s2 = s;
					}

					if ( rootIterationCount == 50 )
					{
						break;
					}
				}

				pushBackIterations += 1;

				if ( pushBackIterations == MaxPolygonVertices )
				{
					break;
				}
			}

			if ( done )
			{
				break;
			}

			if ( distanceIterations == k_maxIterations )
			{
				// Root finder got stuck. Semi-victory.
				output.state = TOIState.toiStateFailed;

				// Averaged hit point
				Vec2 pA = MulAdd( worldPointA, proxyA.radius, worldNormal );
				Vec2 pB = MulAdd( worldPointB, -proxyB.radius, worldNormal );
				output.point = Lerp( pA, pB, 0.5f );
				output.normal = worldNormal;
				output.fraction = t1;
				break;
			}
		}

		return output;
	}
}
