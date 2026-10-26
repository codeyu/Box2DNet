// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static ushort B2_MAKE_ID( int a, int b ) => (ushort)( (byte)a << 8 | (byte)b );

	private static Polygon MakeCapsule( Vec2 p1, Vec2 p2, float radius )
	{
		Polygon shape = default;
		shape.vertices[0] = p1;
		shape.vertices[1] = p2;
		shape.centroid = Lerp( p1, p2, 0.5f );

		Vec2 d = Sub( p2, p1 );
		B2_ASSERT( LengthSquared( d ) > FLT_EPSILON );
		Vec2 axis = Normalize( d );
		Vec2 normal = RightPerp( axis );

		shape.normals[0] = normal;
		shape.normals[1] = Neg( normal );
		shape.count = 2;
		shape.radius = radius;

		return shape;
	}

	public static LocalManifold CollideCircles( in Circle circleA, in Circle circleB, Transform xf )
	{
		LocalManifold manifold = default;

		Vec2 pointA = circleA.center;
		Vec2 pointB = TransformPoint( xf, circleB.center );

		float distance;
		Vec2 normal = GetLengthAndNormalize( out distance, Sub( pointB, pointA ) );

		float radiusA = circleA.radius;
		float radiusB = circleB.radius;

		float separation = distance - radiusA - radiusB;
		if ( separation > SpeculativeDistance )
		{
			return manifold;
		}

		Vec2 cA = MulAdd( pointA, radiusA, normal );
		Vec2 cB = MulAdd( pointB, -radiusB, normal );

		manifold.normal = normal;
		ref LocalManifoldPoint mp = ref manifold.points[0];
		mp.point = Lerp( cA, cB, 0.5f );
		mp.separation = separation;
		mp.id = 0;
		manifold.pointCount = 1;
		return manifold;
	}

	/// Compute the collision manifold between a capsule and circle
	public static LocalManifold CollideCapsuleAndCircle( in Capsule capsuleA, in Circle circleB, Transform xf )
	{
		LocalManifold manifold = default;

		// Compute circle position in the frame of the capsule.
		Vec2 pB = TransformPoint( xf, circleB.center );

		// Compute closest point
		Vec2 p1 = capsuleA.center1;
		Vec2 p2 = capsuleA.center2;

		Vec2 e = Sub( p2, p1 );

		// dot(p - pA, e) = 0
		// dot(p - (p1 + s1 * e), e) = 0
		// s1 = dot(p - p1, e)
		Vec2 pA = default;
		float s1 = Dot( Sub( pB, p1 ), e );
		float s2 = Dot( Sub( p2, pB ), e );
		if ( s1 < 0.0f )
		{
			// p1 region
			pA = p1;
		}
		else if ( s2 < 0.0f )
		{
			// p2 region
			pA = p2;
		}
		else
		{
			// circle colliding with segment interior
			float s = s1 / Dot( e, e );
			pA = MulAdd( p1, s, e );
		}

		float distance;
		Vec2 normal = GetLengthAndNormalize( out distance, Sub( pB, pA ) );

		float radiusA = capsuleA.radius;
		float radiusB = circleB.radius;
		float separation = distance - radiusA - radiusB;
		if ( separation > SpeculativeDistance )
		{
			return manifold;
		}

		Vec2 cA = MulAdd( pA, radiusA, normal );
		Vec2 cB = MulAdd( pB, -radiusB, normal );

		manifold.normal = normal;
		ref LocalManifoldPoint mp = ref manifold.points[0];
		mp.point = Lerp( cA, cB, 0.5f );
		mp.separation = separation;
		mp.id = 0;
		manifold.pointCount = 1;
		return manifold;
	}

	public static LocalManifold CollidePolygonAndCircle( in Polygon polygonA, in Circle circleB, Transform xf )
	{
		LocalManifold manifold = default;
		float speculativeDistance = SpeculativeDistance;

		// Compute circle position in the frame of the polygon.
		Vec2 center = TransformPoint( xf, circleB.center );
		float radiusA = polygonA.radius;
		float radiusB = circleB.radius;
		float radius = radiusA + radiusB;

		// Find the min separating edge.
		int normalIndex = 0;
		float separation = -FLT_MAX;
		int vertexCount = polygonA.count;
		ReadOnlySpan<Vec2> vertices = polygonA.vertices;
		ReadOnlySpan<Vec2> normals = polygonA.normals;
		for ( int i = 0; i < vertexCount; ++i )
		{
			float s = Dot( normals[i], Sub( center, vertices[i] ) );
			if ( s > separation )
			{
				separation = s;
				normalIndex = i;
			}
		}

		if ( separation > radius + speculativeDistance )
		{
			return manifold;
		}

		// Vertices of the reference edge.
		int vertIndex1 = normalIndex;
		int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		Vec2 v1 = vertices[vertIndex1];
		Vec2 v2 = vertices[vertIndex2];

		// Compute barycentric coordinates
		float u1 = Dot( Sub( center, v1 ), Sub( v2, v1 ) );
		float u2 = Dot( Sub( center, v2 ), Sub( v1, v2 ) );

		if ( u1 < 0.0f && separation > FLT_EPSILON )
		{
			// Circle center is closest to v1 and safely outside the polygon
			Vec2 normal = Normalize( Sub( center, v1 ) );
			separation = Dot( Sub( center, v1 ), normal );
			if ( separation > radius + speculativeDistance )
			{
				return manifold;
			}

			Vec2 cA = MulAdd( v1, radiusA, normal );
			Vec2 cB = MulSub( center, radiusB, normal );
			Vec2 contactPointA = Lerp( cA, cB, 0.5f );

			manifold.normal = normal;
			ref LocalManifoldPoint mp = ref manifold.points[0];
			mp.point = contactPointA;
			mp.separation = Dot( Sub( cB, cA ), normal );
			mp.id = 0;
			manifold.pointCount = 1;
		}
		else if ( u2 < 0.0f && separation > FLT_EPSILON )
		{
			// Circle center is closest to v2 and safely outside the polygon
			Vec2 normal = Normalize( Sub( center, v2 ) );
			separation = Dot( Sub( center, v2 ), normal );
			if ( separation > radius + speculativeDistance )
			{
				return manifold;
			}

			Vec2 cA = MulAdd( v2, radiusA, normal );
			Vec2 cB = MulSub( center, radiusB, normal );
			Vec2 contactPointA = Lerp( cA, cB, 0.5f );

			manifold.normal = normal;
			ref LocalManifoldPoint mp = ref manifold.points[0];
			mp.point = contactPointA;
			mp.separation = Dot( Sub( cB, cA ), normal );
			mp.id = 0;
			manifold.pointCount = 1;
		}
		else
		{
			// Circle center is between v1 and v2. Center may be inside polygon
			Vec2 normal = normals[normalIndex];
			manifold.normal = normal;

			// cA is the projection of the circle center onto to the reference edge
			Vec2 cA = MulAdd( center, radiusA - Dot( Sub( center, v1 ), normal ), normal );

			// cB is the deepest point on the circle with respect to the reference edge
			Vec2 cB = MulSub( center, radiusB, normal );

			// The contact point is the midpoint
			ref LocalManifoldPoint mp = ref manifold.points[0];
			mp.point = Lerp( cA, cB, 0.5f );
			mp.separation = separation - radius;
			mp.id = 0;
			manifold.pointCount = 1;
		}

		return manifold;
	}

	// Follows Ericson 5.1.9 Closest Points of Two Line Segments
	// Adds some logic to support clipping to get two contact points
	public static LocalManifold CollideCapsules( in Capsule capsuleA, in Capsule capsuleB, Transform xf )
	{
		Vec2 origin = capsuleA.center1;

		// Shift to the origin in frame A for round-off, a pure translation in A's frame
		Transform xfs = new Transform { p = Sub( xf.p, origin ), q = xf.q };

		Vec2 p1 = Vec2_zero;
		Vec2 q1 = Sub( capsuleA.center2, origin );

		Vec2 p2 = TransformPoint( xfs, capsuleB.center1 );
		Vec2 q2 = TransformPoint( xfs, capsuleB.center2 );

		Vec2 d1 = Sub( q1, p1 );
		Vec2 d2 = Sub( q2, p2 );

		float dd1 = Dot( d1, d1 );
		float dd2 = Dot( d2, d2 );

		float epsSqr = FLT_EPSILON * FLT_EPSILON;
		B2_ASSERT( dd1 > epsSqr && dd2 > epsSqr );

		Vec2 r = Sub( p1, p2 );
		float rd1 = Dot( r, d1 );
		float rd2 = Dot( r, d2 );

		float d12 = Dot( d1, d2 );

		float denom = dd1 * dd2 - d12 * d12;

		// Fraction on segment 1
		float f1 = 0.0f;
		if ( denom != 0.0f )
		{
			// not parallel
			f1 = ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denom, 0.0f, 1.0f );
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

		Vec2 closest1 = MulAdd( p1, f1, d1 );
		Vec2 closest2 = MulAdd( p2, f2, d2 );
		float distanceSquared = DistanceSquared( closest1, closest2 );

		LocalManifold manifold = default;
		float radiusA = capsuleA.radius;
		float radiusB = capsuleB.radius;
		float radius = radiusA + radiusB;
		float maxDistance = radius + SpeculativeDistance;

		if ( distanceSquared > maxDistance * maxDistance )
		{
			return manifold;
		}

		float distance = MathF.Sqrt( distanceSquared );

		float length1, length2;
		Vec2 u1 = GetLengthAndNormalize( out length1, d1 );
		Vec2 u2 = GetLengthAndNormalize( out length2, d2 );

		// Does segment B project outside segment A?
		float fp2 = Dot( Sub( p2, p1 ), u1 );
		float fq2 = Dot( Sub( q2, p1 ), u1 );
		bool outsideA = ( fp2 <= 0.0f && fq2 <= 0.0f ) || ( fp2 >= length1 && fq2 >= length1 );

		// Does segment A project outside segment B?
		float fp1 = Dot( Sub( p1, p2 ), u2 );
		float fq1 = Dot( Sub( q1, p2 ), u2 );
		bool outsideB = ( fp1 <= 0.0f && fq1 <= 0.0f ) || ( fp1 >= length2 && fq1 >= length2 );

		if ( outsideA == false && outsideB == false )
		{
			// attempt to clip
			// this may yield contact points with excessive separation
			// in that case the algorithm falls back to single point collision

			// find reference edge using SAT
			Vec2 normalA = default;
			float separationA;

			{
				normalA = LeftPerp( u1 );
				float ss1 = Dot( Sub( p2, p1 ), normalA );
				float ss2 = Dot( Sub( q2, p1 ), normalA );
				float s1p = ss1 < ss2 ? ss1 : ss2;
				float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

				if ( s1p > s1n )
				{
					separationA = s1p;
				}
				else
				{
					separationA = s1n;
					normalA = Neg( normalA );
				}
			}

			Vec2 normalB = default;
			float separationB;
			{
				normalB = LeftPerp( u2 );
				float ss1 = Dot( Sub( p1, p2 ), normalB );
				float ss2 = Dot( Sub( q1, p2 ), normalB );
				float s1p = ss1 < ss2 ? ss1 : ss2;
				float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

				if ( s1p > s1n )
				{
					separationB = s1p;
				}
				else
				{
					separationB = s1n;
					normalB = Neg( normalB );
				}
			}

			// biased to avoid feature flip-flop
			// todo more testing?
			if ( separationA + 0.1f * LinearSlop >= separationB )
			{
				manifold.normal = normalA;

				Vec2 cp = p2;
				Vec2 cq = q2;

				// clip to p1
				if ( fp2 < 0.0f && fq2 > 0.0f )
				{
					cp = Lerp( p2, q2, ( 0.0f - fp2 ) / ( fq2 - fp2 ) );
				}
				else if ( fq2 < 0.0f && fp2 > 0.0f )
				{
					cq = Lerp( q2, p2, ( 0.0f - fq2 ) / ( fp2 - fq2 ) );
				}

				// clip to q1
				if ( fp2 > length1 && fq2 < length1 )
				{
					cp = Lerp( p2, q2, ( fp2 - length1 ) / ( fp2 - fq2 ) );
				}
				else if ( fq2 > length1 && fp2 < length1 )
				{
					cq = Lerp( q2, p2, ( fq2 - length1 ) / ( fq2 - fp2 ) );
				}

				float sp = Dot( Sub( cp, p1 ), normalA );
				float sq = Dot( Sub( cq, p1 ), normalA );

				if ( sp <= distance + LinearSlop || sq <= distance + LinearSlop )
				{
					scoped ref LocalManifoldPoint mp = ref Unsafe.NullRef<LocalManifoldPoint>();
					mp = ref manifold.points[0];
					mp.point = MulAdd( cp, 0.5f * ( radiusA - radiusB - sp ), normalA );
					mp.separation = sp - radius;
					mp.id = B2_MAKE_ID( 0, 0 );

					mp = ref manifold.points[1];
					mp.point = MulAdd( cq, 0.5f * ( radiusA - radiusB - sq ), normalA );
					mp.separation = sq - radius;
					mp.id = B2_MAKE_ID( 0, 1 );
					manifold.pointCount = 2;
				}
			}
			else
			{
				// normal always points from A to B
				manifold.normal = Neg( normalB );

				Vec2 cp = p1;
				Vec2 cq = q1;

				// clip to p2
				if ( fp1 < 0.0f && fq1 > 0.0f )
				{
					cp = Lerp( p1, q1, ( 0.0f - fp1 ) / ( fq1 - fp1 ) );
				}
				else if ( fq1 < 0.0f && fp1 > 0.0f )
				{
					cq = Lerp( q1, p1, ( 0.0f - fq1 ) / ( fp1 - fq1 ) );
				}

				// clip to q2
				if ( fp1 > length2 && fq1 < length2 )
				{
					cp = Lerp( p1, q1, ( fp1 - length2 ) / ( fp1 - fq1 ) );
				}
				else if ( fq1 > length2 && fp1 < length2 )
				{
					cq = Lerp( q1, p1, ( fq1 - length2 ) / ( fq1 - fp1 ) );
				}

				float sp = Dot( Sub( cp, p2 ), normalB );
				float sq = Dot( Sub( cq, p2 ), normalB );

				if ( sp <= distance + LinearSlop || sq <= distance + LinearSlop )
				{
					scoped ref LocalManifoldPoint mp = ref Unsafe.NullRef<LocalManifoldPoint>();
					mp = ref manifold.points[0];
					mp.point = MulAdd( cp, 0.5f * ( radiusB - radiusA - sp ), normalB );
					mp.separation = sp - radius;
					mp.id = B2_MAKE_ID( 0, 0 );
					mp = ref manifold.points[1];
					mp.point = MulAdd( cq, 0.5f * ( radiusB - radiusA - sq ), normalB );
					mp.separation = sq - radius;
					mp.id = B2_MAKE_ID( 1, 0 );
					manifold.pointCount = 2;
				}
			}
		}

		if ( manifold.pointCount == 0 )
		{
			// single point collision
			Vec2 normal = Sub( closest2, closest1 );
			if ( Dot( normal, normal ) > epsSqr )
			{
				normal = Normalize( normal );
			}
			else
			{
				normal = LeftPerp( u1 );
			}

			Vec2 c1 = MulAdd( closest1, radiusA, normal );
			Vec2 c2 = MulAdd( closest2, -radiusB, normal );

			int i1 = f1 == 0.0f ? 0 : 1;
			int i2 = f2 == 0.0f ? 0 : 1;

			manifold.normal = normal;
			manifold.points[0].point = Lerp( c1, c2, 0.5f );
			manifold.points[0].separation = MathF.Sqrt( distanceSquared ) - radius;
			manifold.points[0].id = B2_MAKE_ID( i1, i2 );
			manifold.pointCount = 1;
		}

		// Undo the origin shift so points are in frame A
		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			manifold.points[i].point = Add( manifold.points[i].point, origin );
		}

		return manifold;
	}

	public static LocalManifold CollideSegmentAndCapsule( in Segment segmentA, in Capsule capsuleB, Transform xf )
	{
		Capsule capsuleA = new Capsule { center1 = segmentA.point1, center2 = segmentA.point2, radius = 0.0f };
		return CollideCapsules( ref capsuleA, capsuleB, xf );
	}

	public static LocalManifold CollidePolygonAndCapsule( in Polygon polygonA, in Capsule capsuleB, Transform xf )
	{
		Polygon polyB = MakeCapsule( capsuleB.center1, capsuleB.center2, capsuleB.radius );
		return CollidePolygons( polygonA, ref polyB, xf );
	}

	// Polygon clipper used to compute contact points when there are potentially two contact points.
	private static LocalManifold ClipPolygons( in Polygon polyA, in Polygon polyB, int edgeA, int edgeB, bool flip )
	{
		LocalManifold manifold = default;

		// reference polygon
		scoped ref readonly Polygon poly1 = ref Unsafe.NullRef<Polygon>();
		int i11, i12;

		// incident polygon
		scoped ref readonly Polygon poly2 = ref Unsafe.NullRef<Polygon>();
		int i21, i22;

		if ( flip )
		{
			poly1 = ref polyB;
			poly2 = ref polyA;
			i11 = edgeB;
			i12 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;
			i21 = edgeA;
			i22 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;
		}
		else
		{
			poly1 = ref polyA;
			poly2 = ref polyB;
			i11 = edgeA;
			i12 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;
			i21 = edgeB;
			i22 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;
		}

		Vec2 normal = poly1.normals[i11];

		// Reference edge vertices
		Vec2 v11 = poly1.vertices[i11];
		Vec2 v12 = poly1.vertices[i12];

		// Incident edge vertices
		Vec2 v21 = poly2.vertices[i21];
		Vec2 v22 = poly2.vertices[i22];

		Vec2 tangent = CrossSV( 1.0f, normal );

		float lower1 = 0.0f;
		float upper1 = Dot( Sub( v12, v11 ), tangent );

		// Incident edge points opposite of tangent due to CCW winding
		float upper2 = Dot( Sub( v21, v11 ), tangent );
		float lower2 = Dot( Sub( v22, v11 ), tangent );

		// Are the segments disjoint?
		if ( upper2 < lower1 || upper1 < lower2 )
		{
			return manifold;
		}

		Vec2 vLower = default;
		if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
		{
			vLower = Lerp( v22, v21, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
		}
		else
		{
			vLower = v22;
		}

		Vec2 vUpper = default;
		if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
		{
			vUpper = Lerp( v22, v21, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
		}
		else
		{
			vUpper = v21;
		}

		// todo vLower can be very close to vUpper, reduce to one point?

		float separationLower = Dot( Sub( vLower, v11 ), normal );
		float separationUpper = Dot( Sub( vUpper, v11 ), normal );

		float r1 = poly1.radius;
		float r2 = poly2.radius;

		// Put contact points at midpoint, accounting for radii
		vLower = MulAdd( vLower, 0.5f * ( r1 - r2 - separationLower ), normal );
		vUpper = MulAdd( vUpper, 0.5f * ( r1 - r2 - separationUpper ), normal );

		float radius = r1 + r2;

		if ( flip == false )
		{
			manifold.normal = normal;
			scoped ref LocalManifoldPoint cp = ref manifold.points[0];
			{
				cp.point = vLower;
				cp.separation = separationLower - radius;
				cp.id = B2_MAKE_ID( i11, i22 );
				manifold.pointCount += 1;
				cp = ref manifold.points[1];
			}

			{
				cp.point = vUpper;
				cp.separation = separationUpper - radius;
				cp.id = B2_MAKE_ID( i12, i21 );
				manifold.pointCount += 1;
			}
		}
		else
		{
			manifold.normal = Neg( normal );
			scoped ref LocalManifoldPoint cp = ref manifold.points[0];
			{
				cp.point = vUpper;
				cp.separation = separationUpper - radius;
				cp.id = B2_MAKE_ID( i21, i12 );
				manifold.pointCount += 1;
				cp = ref manifold.points[1];
			}

			{
				cp.point = vLower;
				cp.separation = separationLower - radius;
				cp.id = B2_MAKE_ID( i22, i11 );
				manifold.pointCount += 1;
			}
		}

		return manifold;
	}

	// Find the max separation between poly1 and poly2 using edge normals from poly1.
	private static float FindMaxSeparation( ref int edgeIndex, in Polygon poly1, in Polygon poly2 )
	{
		int count1 = poly1.count;
		int count2 = poly2.count;
		ReadOnlySpan<Vec2> n1s = poly1.normals;
		ReadOnlySpan<Vec2> v1s = poly1.vertices;
		ReadOnlySpan<Vec2> v2s = poly2.vertices;

		int bestIndex = 0;
		float maxSeparation = -FLT_MAX;
		for ( int i = 0; i < count1; ++i )
		{
			// Get poly1 normal in frame2.
			Vec2 n = n1s[i];
			Vec2 v1 = v1s[i];

			// Find the deepest point for normal i.
			float si = FLT_MAX;
			for ( int j = 0; j < count2; ++j )
			{
				float sij = Dot( n, Sub( v2s[j], v1 ) );
				if ( sij < si )
				{
					si = sij;
				}
			}

			if ( si > maxSeparation )
			{
				maxSeparation = si;
				bestIndex = i;
			}
		}

		edgeIndex = bestIndex;
		return maxSeparation;
	}

	// Due to speculation, every polygon is rounded
	// Algorithm:
	//
	// compute edge separation using the separating axis test (SAT)
	// if (separation > speculation_distance)
	//   return
	// find reference and incident edge
	// if separation >= 0.1f * LinearSlop
	//   compute closest points between reference and incident edge
	//   if vertices are closest
	//      single vertex-vertex contact
	//   else
	//      clip edges
	//   end
	// else
	//   clip edges
	// end

	public static LocalManifold CollidePolygons( in Polygon polygonA, in Polygon polygonB, Transform xf )
	{
		Vec2 origin = polygonA.vertices[0];
		float linearSlop = LinearSlop;
		float speculativeDistance = SpeculativeDistance;

		// Shift to the origin in frame A for round-off, a pure translation in A's frame
		Transform xfs = new Transform { p = Sub( xf.p, origin ), q = xf.q };

		Polygon localPolyA = default;
		localPolyA.count = polygonA.count;
		localPolyA.radius = polygonA.radius;
		localPolyA.vertices[0] = Vec2_zero;
		localPolyA.normals[0] = polygonA.normals[0];
		for ( int i = 1; i < localPolyA.count; ++i )
		{
			localPolyA.vertices[i] = Sub( polygonA.vertices[i], origin );
			localPolyA.normals[i] = polygonA.normals[i];
		}

		// Put polyB in polyA's frame to reduce round-off error
		Polygon localPolyB = default;
		localPolyB.count = polygonB.count;
		localPolyB.radius = polygonB.radius;
		for ( int i = 0; i < localPolyB.count; ++i )
		{
			localPolyB.vertices[i] = TransformPoint( xfs, polygonB.vertices[i] );
			localPolyB.normals[i] = RotateVector( xfs.q, polygonB.normals[i] );
		}

		float radius = localPolyA.radius + localPolyB.radius;

		int edgeA = 0;
		float separationA = FindMaxSeparation( ref edgeA, ref localPolyA, ref localPolyB );
		if ( separationA > speculativeDistance + radius )
		{
			return default(LocalManifold);
		}

		int edgeB = 0;
		float separationB = FindMaxSeparation( ref edgeB, ref localPolyB, ref localPolyA );
		if ( separationB > speculativeDistance + radius )
		{
			return default(LocalManifold);
		}

		// Find incident edge
		bool flip;
		if ( separationA >= separationB )
		{
			flip = false;

			Vec2 searchDirection = localPolyA.normals[edgeA];

			// Find the incident edge on polyB
			int count = localPolyB.count;
			ReadOnlySpan<Vec2> normals = localPolyB.normals;
			edgeB = 0;
			float minDot = FLT_MAX;
			for ( int i = 0; i < count; ++i )
			{
				float dot = Dot( searchDirection, normals[i] );
				if ( dot < minDot )
				{
					minDot = dot;
					edgeB = i;
				}
			}
		}
		else
		{
			flip = true;

			Vec2 searchDirection = localPolyB.normals[edgeB];

			// Find the incident edge on polyA
			int count = localPolyA.count;
			ReadOnlySpan<Vec2> normals = localPolyA.normals;
			edgeA = 0;
			float minDot = FLT_MAX;
			for ( int i = 0; i < count; ++i )
			{
				float dot = Dot( searchDirection, normals[i] );
				if ( dot < minDot )
				{
					minDot = dot;
					edgeA = i;
				}
			}
		}

		LocalManifold manifold = default;

		// Using slop here to ensure vertex-vertex normal vectors can be safely normalized
		// todo this means edge clipping needs to handle slightly non-overlapping edges.
		if ( separationA > 0.1f * linearSlop || separationB > 0.1f * linearSlop )
		{
#if true
			// Edges are disjoint. Find closest points between reference edge and incident edge
			// Reference edge on polygon A
			int i11 = edgeA;
			int i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
			int i21 = edgeB;
			int i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;

			Vec2 v11 = localPolyA.vertices[i11];
			Vec2 v12 = localPolyA.vertices[i12];
			Vec2 v21 = localPolyB.vertices[i21];
			Vec2 v22 = localPolyB.vertices[i22];

			SegmentDistanceResult result = ComputeSegmentDistance( v11, v12, v21, v22 );
			B2_ASSERT( result.distanceSquared > 0.0f );
			float distance = MathF.Sqrt( result.distanceSquared );
			float separation = distance - radius;

			if ( distance - radius > speculativeDistance )
			{
				// This can happen in the vertex-vertex case
				return manifold;
			}

			// Attempt to clip edges
			manifold = ClipPolygons( ref localPolyA, ref localPolyB, edgeA, edgeB, flip );

			float minSeparation = FLT_MAX;
			for ( int i = 0; i < manifold.pointCount; ++i )
			{
				minSeparation = MinFloat( minSeparation, manifold.points[i].separation );
			}

			// Does vertex-vertex have substantially larger separation?
			if ( separation + 0.1f * linearSlop < minSeparation )
			{
				if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
				{
					// v11 - v21
					Vec2 normal = Sub( v21, v11 );
					float invDistance = 1.0f / distance;
					normal.x *= invDistance;
					normal.y *= invDistance;

					Vec2 c1 = MulAdd( v11, localPolyA.radius, normal );
					Vec2 c2 = MulAdd( v21, -localPolyB.radius, normal );

					manifold.normal = normal;
					manifold.points[0].point = Lerp( c1, c2, 0.5f );
					manifold.points[0].separation = distance - radius;
					manifold.points[0].id = B2_MAKE_ID( i11, i21 );
					manifold.pointCount = 1;
				}
				else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
				{
					// v11 - v22
					Vec2 normal = Sub( v22, v11 );
					float invDistance = 1.0f / distance;
					normal.x *= invDistance;
					normal.y *= invDistance;

					Vec2 c1 = MulAdd( v11, localPolyA.radius, normal );
					Vec2 c2 = MulAdd( v22, -localPolyB.radius, normal );

					manifold.normal = normal;
					manifold.points[0].point = Lerp( c1, c2, 0.5f );
					manifold.points[0].separation = distance - radius;
					manifold.points[0].id = B2_MAKE_ID( i11, i22 );
					manifold.pointCount = 1;
				}
				else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
				{
					// v12 - v21
					Vec2 normal = Sub( v21, v12 );
					float invDistance = 1.0f / distance;
					normal.x *= invDistance;
					normal.y *= invDistance;

					Vec2 c1 = MulAdd( v12, localPolyA.radius, normal );
					Vec2 c2 = MulAdd( v21, -localPolyB.radius, normal );

					manifold.normal = normal;
					manifold.points[0].point = Lerp( c1, c2, 0.5f );
					manifold.points[0].separation = distance - radius;
					manifold.points[0].id = B2_MAKE_ID( i12, i21 );
					manifold.pointCount = 1;
				}
				else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
				{
					// v12 - v22
					Vec2 normal = Sub( v22, v12 );
					float invDistance = 1.0f / distance;
					normal.x *= invDistance;
					normal.y *= invDistance;

					Vec2 c1 = MulAdd( v12, localPolyA.radius, normal );
					Vec2 c2 = MulAdd( v22, -localPolyB.radius, normal );

					manifold.normal = normal;
					manifold.points[0].point = Lerp( c1, c2, 0.5f );
					manifold.points[0].separation = distance - radius;
					manifold.points[0].id = B2_MAKE_ID( i12, i22 );
					manifold.pointCount = 1;
				}
			}
#else
			// Polygons are disjoint. Find closest points between reference edge and incident edge
			// Reference edge on polygon A
			int i11 = edgeA;
			int i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
			int i21 = edgeB;
			int i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;

			Vec2 v11 = localPolyA.vertices[i11];
			Vec2 v12 = localPolyA.vertices[i12];
			Vec2 v21 = localPolyB.vertices[i21];
			Vec2 v22 = localPolyB.vertices[i22];

			SegmentDistanceResult result = ComputeSegmentDistance( v11, v12, v21, v22 );

			if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
			{
				// v11 - v21
				Vec2 normal = Sub( v21, v11 );
				B2_ASSERT( result.distanceSquared > 0.0f );
				float distance = MathF.Sqrt( result.distanceSquared );
				if ( distance > SpeculativeDistance + radius )
				{
					return manifold;
				}
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				Vec2 c1 = MulAdd( v11, localPolyA.radius, normal );
				Vec2 c2 = MulAdd( v21, -localPolyB.radius, normal );

				manifold.normal = normal;
				manifold.points[0].point = Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i11, i21 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
			{
				// v11 - v22
				Vec2 normal = Sub( v22, v11 );
				B2_ASSERT( result.distanceSquared > 0.0f );
				float distance = MathF.Sqrt( result.distanceSquared );
				if ( distance > SpeculativeDistance + radius )
				{
					return manifold;
				}
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				Vec2 c1 = MulAdd( v11, localPolyA.radius, normal );
				Vec2 c2 = MulAdd( v22, -localPolyB.radius, normal );

				manifold.normal = normal;
				manifold.points[0].point = Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i11, i22 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
			{
				// v12 - v21
				Vec2 normal = Sub( v21, v12 );
				B2_ASSERT( result.distanceSquared > 0.0f );
				float distance = MathF.Sqrt( result.distanceSquared );
				if ( distance > SpeculativeDistance + radius )
				{
					return manifold;
				}
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				Vec2 c1 = MulAdd( v12, localPolyA.radius, normal );
				Vec2 c2 = MulAdd( v21, -localPolyB.radius, normal );

				manifold.normal = normal;
				manifold.points[0].point = Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i12, i21 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
			{
				// v12 - v22
				Vec2 normal = Sub( v22, v12 );
				B2_ASSERT( result.distanceSquared > 0.0f );
				float distance = MathF.Sqrt( result.distanceSquared );
				if ( distance > SpeculativeDistance + radius )
				{
					return manifold;
				}
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				Vec2 c1 = MulAdd( v12, localPolyA.radius, normal );
				Vec2 c2 = MulAdd( v22, -localPolyB.radius, normal );

				manifold.normal = normal;
				manifold.points[0].point = Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i12, i22 );
				manifold.pointCount = 1;
			}
			else
			{
				// Edge region
				manifold = ClipPolygons( ref localPolyA, ref localPolyB, edgeA, edgeB, flip );
			}
#endif
		}
		else
		{
			// Polygons overlap
			manifold = ClipPolygons( ref localPolyA, ref localPolyB, edgeA, edgeB, flip );
		}

		// Undo the origin shift so points are in frame A
		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			manifold.points[i].point = Add( manifold.points[i].point, origin );
		}

		return manifold;
	}

	public static LocalManifold CollideSegmentAndCircle( in Segment segmentA, in Circle circleB, Transform xf )
	{
		Capsule capsuleA = new Capsule { center1 = segmentA.point1, center2 = segmentA.point2, radius = 0.0f };
		return CollideCapsuleAndCircle( ref capsuleA, circleB, xf );
	}

	public static LocalManifold CollideSegmentAndPolygon( in Segment segmentA, in Polygon polygonB, Transform xf )
	{
		Polygon polygonA = MakeCapsule( segmentA.point1, segmentA.point2, 0.0f );
		return CollidePolygons( ref polygonA, polygonB, xf );
	}

	public static LocalManifold CollideChainSegmentAndCircle( in ChainSegment segmentA, in Circle circleB, Transform xf )
	{
		LocalManifold manifold = default;

		// Compute circle in frame of segment
		Vec2 pB = TransformPoint( xf, circleB.center );

		Vec2 p1 = segmentA.segment.point1;
		Vec2 p2 = segmentA.segment.point2;
		Vec2 e = Sub( p2, p1 );

		// Normal points to the right
		float offset = Dot( RightPerp( e ), Sub( pB, p1 ) );
		if ( offset < 0.0f )
		{
			// collision is one-sided
			return manifold;
		}

		// Barycentric coordinates
		float u = Dot( e, Sub( p2, pB ) );
		float v = Dot( e, Sub( pB, p1 ) );

		Vec2 pA = default;
		if ( v <= 0.0f )
		{
			// Behind point1?
			// Is pB in the Voronoi region of the previous edge?
			Vec2 prevEdge = Sub( p1, segmentA.ghost1 );
			float uPrev = Dot( prevEdge, Sub( pB, p1 ) );
			if ( uPrev <= 0.0f )
			{
				return manifold;
			}

			pA = p1;
		}
		else if ( u <= 0.0f )
		{
			// Ahead of point2?
			Vec2 nextEdge = Sub( segmentA.ghost2, p2 );
			float vNext = Dot( nextEdge, Sub( pB, p2 ) );

			// Is pB in the Voronoi region of the next edge?
			if ( vNext > 0.0f )
			{
				return manifold;
			}

			pA = p2;
		}
		else
		{
			float ee = Dot( e, e );
			pA = new Vec2( u * p1.x + v * p2.x, u * p1.y + v * p2.y );
			pA = ee > 0.0f ? MulSV( 1.0f / ee, pA ) : p1;
		}

		float distance;
		Vec2 normal = GetLengthAndNormalize( out distance, Sub( pB, pA ) );

		float radius = circleB.radius;
		float separation = distance - radius;
		if ( separation > SpeculativeDistance )
		{
			return manifold;
		}

		Vec2 cA = pA;
		Vec2 cB = MulAdd( pB, -radius, normal );

		manifold.normal = normal;

		ref LocalManifoldPoint mp = ref manifold.points[0];
		mp.point = Lerp( cA, cB, 0.5f );
		mp.separation = separation;
		mp.id = 0;
		manifold.pointCount = 1;
		return manifold;
	}

	public static LocalManifold CollideChainSegmentAndCapsule( in ChainSegment segmentA, in Capsule capsuleB, Transform xf, ref SimplexCache cache )
	{
		Polygon polyB = MakeCapsule( capsuleB.center1, capsuleB.center2, capsuleB.radius );
		return CollideChainSegmentAndPolygon( segmentA, polyB, xf, ref cache );
	}

	private static LocalManifold ClipSegments( Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2, Vec2 normal, float ra, float rb, ushort id1, ushort id2 )
	{
		LocalManifold manifold = default;

		Vec2 tangent = LeftPerp( normal );

		// Barycentric coordinates of each point relative to a1 along tangent
		float lower1 = 0.0f;
		float upper1 = Dot( Sub( a2, a1 ), tangent );

		// Incident edge points opposite of tangent due to CCW winding
		float upper2 = Dot( Sub( b1, a1 ), tangent );
		float lower2 = Dot( Sub( b2, a1 ), tangent );

		// Do segments overlap?
		if ( upper2 < lower1 || upper1 < lower2 )
		{
			return manifold;
		}

		Vec2 vLower = default;
		if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
		{
			vLower = Lerp( b2, b1, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
		}
		else
		{
			vLower = b2;
		}

		Vec2 vUpper = default;
		if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
		{
			vUpper = Lerp( b2, b1, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
		}
		else
		{
			vUpper = b1;
		}

		// todo vLower can be very close to vUpper, reduce to one point?

		float separationLower = Dot( Sub( vLower, a1 ), normal );
		float separationUpper = Dot( Sub( vUpper, a1 ), normal );

		// Put contact points at midpoint, accounting for radii
		vLower = MulAdd( vLower, 0.5f * ( ra - rb - separationLower ), normal );
		vUpper = MulAdd( vUpper, 0.5f * ( ra - rb - separationUpper ), normal );

		float radius = ra + rb;

		manifold.normal = normal;
		{
			scoped ref LocalManifoldPoint cp = ref manifold.points[0];
			cp.point = vLower;
			cp.separation = separationLower - radius;
			cp.id = id1;
		}

		{
			ref LocalManifoldPoint cp = ref manifold.points[1];
			cp.point = vUpper;
			cp.separation = separationUpper - radius;
			cp.id = id2;
		}

		manifold.pointCount = 2;

		return manifold;
	}

	private enum NormalType
	{
		// This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped
		b2_normalSkip,

		// This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision
		b2_normalAdmit,

		// This means the normal is in a region of a concave vertex and should be snapped to the segment normal
		b2_normalSnap
	}

	private struct ChainSegmentParams
	{
		public Vec2 edge1;
		public Vec2 normal0;
		public Vec2 normal2;
		public bool convex1;
		public bool convex2;
	}

	// Evaluate Gauss map
	// See https://box2d.org/posts/2020/06/ghost-collisions/
	static NormalType ClassifyNormal( ChainSegmentParams @params, Vec2 normal )
	{
		const float sinTol = 0.01f;

		if ( Dot( normal, @params.edge1 ) <= 0.0f )
		{
			// Normal points towards the segment tail
			if ( @params.convex1 )
			{
				if ( Cross( normal, @params.normal0 ) > sinTol )
				{
					return NormalType.b2_normalSkip;
				}

				return NormalType.b2_normalAdmit;
			}
			else
			{
				return NormalType.b2_normalSnap;
			}
		}
		else
		{
			// Normal points towards segment head
			if ( @params.convex2 )
			{
				if ( Cross( @params.normal2, normal ) > sinTol )
				{
					return NormalType.b2_normalSkip;
				}

				return NormalType.b2_normalAdmit;
			}
			else
			{
				return NormalType.b2_normalSnap;
			}
		}
	}

	public static LocalManifold CollideChainSegmentAndPolygon( in ChainSegment segmentA, in Polygon polygonB, Transform xf, ref SimplexCache cache )
	{
		LocalManifold manifold = default;

		Vec2 centroidB = TransformPoint( xf, polygonB.centroid );
		float radiusB = polygonB.radius;

		Vec2 p1 = segmentA.segment.point1;
		Vec2 p2 = segmentA.segment.point2;

		Vec2 edge1 = Normalize( Sub( p2, p1 ) );

		ChainSegmentParams smoothParams = default;
		smoothParams.edge1 = edge1;

		const float convexTol = 0.01f;
		Vec2 edge0 = Normalize( Sub( p1, segmentA.ghost1 ) );
		smoothParams.normal0 = RightPerp( edge0 );
		smoothParams.convex1 = Cross( edge0, edge1 ) >= convexTol;

		Vec2 edge2 = Normalize( Sub( segmentA.ghost2, p2 ) );
		smoothParams.normal2 = RightPerp( edge2 );
		smoothParams.convex2 = Cross( edge1, edge2 ) >= convexTol;

		// Normal points to the right
		Vec2 normal1 = RightPerp( edge1 );
		bool behind1 = Dot( normal1, Sub( centroidB, p1 ) ) < 0.0f;
		bool behind0 = true;
		bool behind2 = true;
		if ( smoothParams.convex1 )
		{
			behind0 = Dot( smoothParams.normal0, Sub( centroidB, p1 ) ) < 0.0f;
		}

		if ( smoothParams.convex2 )
		{
			behind2 = Dot( smoothParams.normal2, Sub( centroidB, p2 ) ) < 0.0f;
		}

		if ( behind1 && behind0 && behind2 )
		{
			// one-sided collision
			return manifold;
		}

		// Get polygonB in frameA
		int count = polygonB.count;
		FixedArray8<Vec2> vertices = default;
		FixedArray8<Vec2> normals = default;
		for ( int i = 0; i < count; ++i )
		{
			vertices[i] = TransformPoint( xf, polygonB.vertices[i] );
			normals[i] = RotateVector( xf.q, polygonB.normals[i] );
		}

		// Distance doesn't work correctly with partial polygons
		DistanceInput input = default;
		input.proxyA = MakeProxy2( segmentA.segment.point1, segmentA.segment.point2, 0.0f );
		input.proxyB = MakeProxy( vertices, count, 0.0f );
		input.transform = Transform_identity;
		input.useRadii = false;

		DistanceOutput output = ComputeShapeDistance( input, ref cache );

		if ( output.distance > radiusB + SpeculativeDistance )
		{
			return manifold;
		}

		// Snap concave normals for partial polygon
		Vec2 n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
		Vec2 n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;

		// Index of incident vertex on polygon
		int incidentIndex = -1;
		int incidentNormal = -1;

		if ( behind1 == false && output.distance > 0.1f * LinearSlop )
		{
			// The closest features may be two vertices or an edge and a vertex even when there should
			// be face contact

			if ( cache.count == 1 )
			{
				// vertex-vertex collision
				Vec2 pA = output.pointA;
				Vec2 pB = output.pointB;

				Vec2 normal = Normalize( Sub( pB, pA ) );

				NormalType type = ClassifyNormal( smoothParams, normal );
				if ( type == NormalType.b2_normalSkip )
				{
					return manifold;
				}

				if ( type == NormalType.b2_normalAdmit )
				{
					manifold.normal = normal;
					scoped ref LocalManifoldPoint cp = ref manifold.points[0];
					cp.point = pA;
					cp.separation = output.distance - radiusB;
					cp.id = B2_MAKE_ID( cache.indexA[0], cache.indexB[0] );
					manifold.pointCount = 1;
					return manifold;
				}

				// fall through b2_normalSnap
				incidentIndex = cache.indexB[0];
			}
			else
			{
				// vertex-edge collision
				B2_ASSERT( cache.count == 2 );

				int ia1 = cache.indexA[0];
				int ia2 = cache.indexA[1];
				int jb1 = cache.indexB[0];
				int jb2 = cache.indexB[1];

				if ( ia1 == ia2 )
				{
					// 1 point on A, expect 2 points on B
					B2_ASSERT( jb1 != jb2 );

					// Find polygon normal most aligned with vector between closest points.
					// This effectively sorts jb1 and jb2
					Vec2 normalB = Sub( output.pointA, output.pointB );
					float dot1 = Dot( normalB, normals[jb1] );
					float dot2 = Dot( normalB, normals[jb2] );
					int ib = dot1 > dot2 ? jb1 : jb2;

					// Use accurate normal
					normalB = normals[ib];

					NormalType type = ClassifyNormal( smoothParams, Neg( normalB ) );
					if ( type == NormalType.b2_normalSkip )
					{
						return manifold;
					}

					if ( type == NormalType.b2_normalAdmit )
					{
						// Get polygon edge associated with normal
						jb1 = ib;
						jb2 = ib < count - 1 ? ib + 1 : 0;

						Vec2 vb1 = vertices[jb1];
						Vec2 vb2 = vertices[jb2];

						// Find incident segment vertex
						dot1 = Dot( normalB, Sub( p1, vb1 ) );
						dot2 = Dot( normalB, Sub( p2, vb1 ) );

						if ( dot1 < dot2 )
						{
							if ( Dot( n0, normalB ) < Dot( normal1, normalB ) )
							{
								// Neighbor is incident
								return manifold;
							}
						}
						else
						{
							if ( Dot( n2, normalB ) < Dot( normal1, normalB ) )
							{
								// Neighbor is incident
								return manifold;
							}
						}

						manifold =
							ClipSegments( vb1, vb2, p1, p2, normalB, radiusB, 0.0f, B2_MAKE_ID( jb1, 1 ), B2_MAKE_ID( jb2, 0 ) );

						B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );
						if ( manifold.pointCount == 2 )
						{
							manifold.normal = Neg( normalB );
						}
						return manifold;
					}

					// fall through b2_normalSnap
					incidentNormal = ib;
				}
				else
				{
					// Get index of incident polygonB vertex
					float dot1 = Dot( normal1, Sub( vertices[jb1], p1 ) );
					float dot2 = Dot( normal1, Sub( vertices[jb2], p2 ) );
					incidentIndex = dot1 < dot2 ? jb1 : jb2;
				}
			}
		}
		else
		{
			// SAT edge normal
			float edgeSeparation = FLT_MAX;

			for ( int i = 0; i < count; ++i )
			{
				float s = Dot( normal1, Sub( vertices[i], p1 ) );
				if ( s < edgeSeparation )
				{
					edgeSeparation = s;
					incidentIndex = i;
				}
			}

			// Check convex neighbor for edge separation
			if ( smoothParams.convex1 )
			{
				float s0 = FLT_MAX;

				for ( int i = 0; i < count; ++i )
				{
					float s = Dot( smoothParams.normal0, Sub( vertices[i], p1 ) );
					if ( s < s0 )
					{
						s0 = s;
					}
				}

				if ( s0 > edgeSeparation )
				{
					edgeSeparation = s0;

					// Indicate neighbor owns edge separation
					incidentIndex = -1;
				}
			}

			// Check convex neighbor for edge separation
			if ( smoothParams.convex2 )
			{
				float s2 = FLT_MAX;

				for ( int i = 0; i < count; ++i )
				{
					float s = Dot( smoothParams.normal2, Sub( vertices[i], p2 ) );
					if ( s < s2 )
					{
						s2 = s;
					}
				}

				if ( s2 > edgeSeparation )
				{
					edgeSeparation = s2;

					// Indicate neighbor owns edge separation
					incidentIndex = -1;
				}
			}

			// SAT polygon normals
			float polygonSeparation = -FLT_MAX;
			int referenceIndex = -1;

			for ( int i = 0; i < count; ++i )
			{
				Vec2 n = normals[i];

				NormalType type = ClassifyNormal( smoothParams, Neg( n ) );
				if ( type != NormalType.b2_normalAdmit )
				{
					continue;
				}

				// Check the infinite sides of the partial polygon
				// if ((smoothParams.convex1 && Cross(n0, n) > 0.0f) || (smoothParams.convex2 && Cross(n, n2) > 0.0f))
				//{
				//	continue;
				//}

				Vec2 p = vertices[i];
				float s = MinFloat( Dot( n, Sub( p2, p ) ), Dot( n, Sub( p1, p ) ) );

				if ( s > polygonSeparation )
				{
					polygonSeparation = s;
					referenceIndex = i;
				}
			}

			if ( polygonSeparation > edgeSeparation )
			{
				int ia1 = referenceIndex;
				int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
				Vec2 a1 = vertices[ia1];
				Vec2 a2 = vertices[ia2];

				Vec2 n = normals[ia1];

				float dot1 = Dot( n, Sub( p1, a1 ) );
				float dot2 = Dot( n, Sub( p2, a1 ) );

				if ( dot1 < dot2 )
				{
					if ( Dot( n0, n ) < Dot( normal1, n ) )
					{
						// Neighbor is incident
						return manifold;
					}
				}
				else
				{
					if ( Dot( n2, n ) < Dot( normal1, n ) )
					{
						// Neighbor is incident
						return manifold;
					}
				}

				manifold = ClipSegments( a1, a2, p1, p2, normals[ia1], radiusB, 0.0f, B2_MAKE_ID( ia1, 1 ), B2_MAKE_ID( ia2, 0 ) );

				B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );
				if ( manifold.pointCount == 2 )
				{
					manifold.normal = Neg( normals[ia1] );
				}

				return manifold;
			}

			if ( incidentIndex == -1 )
			{
				// neighboring segment is the separating axis
				return manifold;
			}

			// fall through segment normal axis
		}

		B2_ASSERT( incidentNormal != -1 || incidentIndex != -1 );

		// Segment normal

		// Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
		Vec2 b1, b2;
		int ib1, ib2;

		if ( incidentNormal != -1 )
		{
			ib1 = incidentNormal;
			ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
		else
		{
			int i2 = incidentIndex;
			int i1 = i2 > 0 ? i2 - 1 : count - 1;
			float d1 = Dot( normal1, normals[i1] );
			float d2 = Dot( normal1, normals[i2] );
			if ( d1 < d2 )
			{
				ib1 = i1;
				ib2 = i2;
				b1 = vertices[ib1];
				b2 = vertices[ib2];
			}
			else
			{
				ib1 = i2;
				ib2 = i2 < count - 1 ? i2 + 1 : 0;
				b1 = vertices[ib1];
				b2 = vertices[ib2];
			}
		}

		manifold = ClipSegments( p1, p2, b1, b2, normal1, 0.0f, radiusB, B2_MAKE_ID( 0, ib2 ), B2_MAKE_ID( 1, ib1 ) );

		B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );

		return manifold;
	}

}
