// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;

namespace Box2DNet;

public static partial class B2
{
	/// Validate ray cast input data (NaN, etc)
	public static bool IsValidRay( in RayCastInput input )
	{
		bool isValid = IsValidVec2( input.origin ) && IsValidVec2( input.translation ) && IsValidFloat( input.maxFraction ) &&
					   0.0f <= input.maxFraction && input.maxFraction < Huge;
		return isValid;
	}

	private static Vec2 ComputePolygonCentroid( ReadOnlySpan<Vec2> vertices, int count )
	{
		Vec2 center = new( 0.0f, 0.0f );
		float area = 0.0f;

		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		Vec2 origin = vertices[0];

		const float inv3 = 1.0f / 3.0f;

		for ( int i = 1; i < count - 1; ++i )
		{
			// Triangle edges
			Vec2 e1 = Sub( vertices[i], origin );
			Vec2 e2 = Sub( vertices[i + 1], origin );
			float a = 0.5f * Cross( e1, e2 );

			// Area weighted centroid
			center = MulAdd( center, a * inv3, Add( e1, e2 ) );
			area += a;
		}

		B2_ASSERT( area > FLT_EPSILON );
		float invArea = 1.0f / area;
		center.x *= invArea;
		center.y *= invArea;

		// Restore offset
		center = Add( origin, center );

		return center;
	}

	/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	public static Polygon MakePolygon( in Hull hull, float radius )
	{
		B2_ASSERT( ValidateHull( hull ) );

		if ( hull.count < 3 )
		{
			// Handle a bad hull when assertions are disabled
			return MakeSquare( 0.5f );
		}

		Polygon shape = default;
		shape.count = hull.count;
		shape.radius = radius;

		// Copy vertices
		for ( int i = 0; i < shape.count; ++i )
		{
			shape.vertices[i] = hull.points[i];
		}

		// Compute normals. Ensure the edges have non-zero length.
		for ( int i = 0; i < shape.count; ++i )
		{
			int i1 = i;
			int i2 = i + 1 < shape.count ? i + 1 : 0;
			Vec2 edge = Sub( shape.vertices[i2], shape.vertices[i1] );
			B2_ASSERT( Dot( edge, edge ) > FLT_EPSILON * FLT_EPSILON );
			shape.normals[i] = Normalize( CrossVS( edge, 1.0f ) );
		}

		shape.centroid = ComputePolygonCentroid( shape.vertices, shape.count );

		return shape;
	}

	/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	public static Polygon MakeOffsetPolygon( in Hull hull, Vec2 position, Rot rotation )
	{
		return MakeOffsetRoundedPolygon( hull, position, rotation, 0.0f );
	}

	/// Make an offset convex polygon from a convex hull with a radius. This will assert if the hull is
	/// not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	public static Polygon MakeOffsetRoundedPolygon( in Hull hull, Vec2 position, Rot rotation, float radius )
	{
		B2_ASSERT( ValidateHull( hull ) );

		if ( hull.count < 3 )
		{
			// Handle a bad hull when assertions are disabled
			return MakeSquare( 0.5f );
		}

		Transform transform = new( position, rotation );

		Polygon shape = default;
		shape.count = hull.count;
		shape.radius = radius;

		// Copy vertices
		for ( int i = 0; i < shape.count; ++i )
		{
			shape.vertices[i] = TransformPoint( transform, hull.points[i] );
		}

		// Compute normals. Ensure the edges have non-zero length.
		for ( int i = 0; i < shape.count; ++i )
		{
			int i1 = i;
			int i2 = i + 1 < shape.count ? i + 1 : 0;
			Vec2 edge = Sub( shape.vertices[i2], shape.vertices[i1] );
			B2_ASSERT( Dot( edge, edge ) > FLT_EPSILON * FLT_EPSILON );
			shape.normals[i] = Normalize( CrossVS( edge, 1.0f ) );
		}

		shape.centroid = ComputePolygonCentroid( shape.vertices, shape.count );

		return shape;
	}

	/// Make a square polygon, bypassing the need for a convex hull.
	/// @param halfWidth the half-width
	public static Polygon MakeSquare( float halfWidth )
	{
		return MakeBox( halfWidth, halfWidth );
	}

	/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	public static Polygon MakeBox( float halfWidth, float halfHeight )
	{
		B2_ASSERT( IsValidFloat( halfWidth ) && halfWidth > 0.0f );
		B2_ASSERT( IsValidFloat( halfHeight ) && halfHeight > 0.0f );

		Polygon shape = default;
		shape.count = 4;
		shape.vertices[0] = new Vec2( -halfWidth, -halfHeight );
		shape.vertices[1] = new Vec2( halfWidth, -halfHeight );
		shape.vertices[2] = new Vec2( halfWidth, halfHeight );
		shape.vertices[3] = new Vec2( -halfWidth, halfHeight );
		shape.normals[0] = new Vec2( 0.0f, -1.0f );
		shape.normals[1] = new Vec2( 1.0f, 0.0f );
		shape.normals[2] = new Vec2( 0.0f, 1.0f );
		shape.normals[3] = new Vec2( -1.0f, 0.0f );
		shape.radius = 0.0f;
		shape.centroid = Vec2_zero;
		return shape;
	}

	/// Make a rounded box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param radius the radius of the rounded extension
	public static Polygon MakeRoundedBox( float halfWidth, float halfHeight, float radius )
	{
		B2_ASSERT( IsValidFloat( radius ) && radius >= 0.0f );
		Polygon shape = MakeBox( halfWidth, halfHeight );
		shape.radius = radius;
		return shape;
	}

	/// Make an offset box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param center the local center of the box
	/// @param rotation the local rotation of the box
	public static Polygon MakeOffsetBox( float halfWidth, float halfHeight, Vec2 center, Rot rotation )
	{
		Transform xf = new( center, rotation );

		Polygon shape = default;
		shape.count = 4;
		shape.vertices[0] = TransformPoint( xf, new Vec2( -halfWidth, -halfHeight ) );
		shape.vertices[1] = TransformPoint( xf, new Vec2( halfWidth, -halfHeight ) );
		shape.vertices[2] = TransformPoint( xf, new Vec2( halfWidth, halfHeight ) );
		shape.vertices[3] = TransformPoint( xf, new Vec2( -halfWidth, halfHeight ) );
		shape.normals[0] = RotateVector( xf.q, new Vec2( 0.0f, -1.0f ) );
		shape.normals[1] = RotateVector( xf.q, new Vec2( 1.0f, 0.0f ) );
		shape.normals[2] = RotateVector( xf.q, new Vec2( 0.0f, 1.0f ) );
		shape.normals[3] = RotateVector( xf.q, new Vec2( -1.0f, 0.0f ) );
		shape.radius = 0.0f;
		shape.centroid = xf.p;
		return shape;
	}

	/// Make an offset rounded box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param center the local center of the box
	/// @param rotation the local rotation of the box
	/// @param radius the radius of the rounded extension
	public static Polygon MakeOffsetRoundedBox( float halfWidth, float halfHeight, Vec2 center, Rot rotation, float radius )
	{
		B2_ASSERT( IsValidFloat( radius ) && radius >= 0.0f );
		Polygon shape = MakeOffsetBox( halfWidth, halfHeight, center, rotation );
		shape.radius = radius;
		return shape;
	}

	/// Transform a polygon. This is useful for transferring a shape from one body to another.
	public static Polygon TransformPolygon( Transform transform, in Polygon polygon )
	{
		Polygon p = polygon;

		for ( int i = 0; i < p.count; ++i )
		{
			p.vertices[i] = TransformPoint( transform, p.vertices[i] );
			p.normals[i] = RotateVector( transform.q, p.normals[i] );
		}

		p.centroid = TransformPoint( transform, p.centroid );

		return p;
	}

	/// Compute mass properties of a circle
	public static MassData ComputeCircleMass( in Circle shape, float density )
	{
		float rr = shape.radius * shape.radius;

		MassData massData;
		massData.mass = density * Pi * rr;
		massData.center = shape.center;

		// inertia about the center of mass
		massData.rotationalInertia = massData.mass * 0.5f * rr;

		return massData;
	}

	/// Compute mass properties of a capsule
	public static MassData ComputeCapsuleMass( in Capsule shape, float density )
	{
		float radius = shape.radius;
		float rr = radius * radius;
		Vec2 p1 = shape.center1;
		Vec2 p2 = shape.center2;
		float length = Length( Sub( p2, p1 ) );
		float ll = length * length;

		float circleMass = density * ( Pi * radius * radius );
		float boxMass = density * ( 2.0f * radius * length );

		MassData massData;
		massData.mass = circleMass + boxMass;
		massData.center.x = 0.5f * ( p1.x + p2.x );
		massData.center.y = 0.5f * ( p1.y + p2.y );

		// two offset half circles, both halves add up to full circle and each half is offset by half length
		// semi-circle centroid = 4 r / 3 pi
		// Need to apply parallel-axis theorem twice:
		// 1. shift semi-circle centroid to origin
		// 2. shift semi-circle to box end
		// m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		// I verified this formula by computing the convex hull of a 128 vertex capsule

		// half circle centroid
		float lc = 4.0f * radius / ( 3.0f * Pi );

		// half length of rectangular portion of capsule
		float h = 0.5f * length;

		float circleInertia = circleMass * ( 0.5f * rr + h * h + 2.0f * h * lc );
		float boxInertia = boxMass * ( 4.0f * rr + ll ) / 12.0f;
		massData.rotationalInertia = circleInertia + boxInertia;

		return massData;
	}

	/// Compute mass properties of a polygon
	public static MassData ComputePolygonMass( in Polygon shape, float density )
	{
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		B2_ASSERT( shape.count > 0 );

		if ( shape.count == 1 )
		{
			Circle circle;
			circle.center = shape.vertices[0];
			circle.radius = shape.radius;
			return ComputeCircleMass( circle, density );
		}

		if ( shape.count == 2 )
		{
			Capsule capsule;
			capsule.center1 = shape.vertices[0];
			capsule.center2 = shape.vertices[1];
			capsule.radius = shape.radius;
			return ComputeCapsuleMass( capsule, density );
		}

		FixedArray8<Vec2> vertices = default;
		int count = shape.count;
		float radius = shape.radius;

		if ( radius > 0.0f )
		{
			// Approximate mass of rounded polygons by pushing out the vertices.
			float sqrt2 = 1.412f;
			for ( int i = 0; i < count; ++i )
			{
				int j = i == 0 ? count - 1 : i - 1;
				Vec2 n1 = shape.normals[j];
				Vec2 n2 = shape.normals[i];

				Vec2 mid = Normalize( Add( n1, n2 ) );
				vertices[i] = MulAdd( shape.vertices[i], sqrt2 * radius, mid );
			}
		}
		else
		{
			for ( int i = 0; i < count; ++i )
			{
				vertices[i] = shape.vertices[i];
			}
		}

		Vec2 center = new( 0.0f, 0.0f );
		float area = 0.0f;
		float rotationalInertia = 0.0f;

		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		Vec2 r = vertices[0];

		const float inv3 = 1.0f / 3.0f;

		for ( int i = 1; i < count - 1; ++i )
		{
			// Triangle edges
			Vec2 e1 = Sub( vertices[i], r );
			Vec2 e2 = Sub( vertices[i + 1], r );

			float D = Cross( e1, e2 );

			float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid, r at origin
			center = MulAdd( center, triangleArea * inv3, Add( e1, e2 ) );

			float ex1 = e1.x, ey1 = e1.y;
			float ex2 = e2.x, ey2 = e2.y;

			float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
			float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

			rotationalInertia += ( 0.25f * inv3 * D ) * ( intx2 + inty2 );
		}

		MassData massData;

		// Total mass
		massData.mass = density * area;

		// Center of mass, shift back from origin at r
		B2_ASSERT( area > FLT_EPSILON );
		float invArea = 1.0f / area;
		center.x *= invArea;
		center.y *= invArea;
		massData.center = Add( r, center );

		// Inertia tensor relative to the local origin (point s).
		massData.rotationalInertia = density * rotationalInertia;

		// Shift inertia to center of mass
		massData.rotationalInertia -= massData.mass * Dot( center, center );

		// If this goes negative we are hosed
		B2_ASSERT( massData.rotationalInertia >= 0.0f );

		return massData;
	}

	// AABBs are built in double and narrowed to float once. In float mode the rounding helpers are
	// plain casts. The double arithmetic is kept to match the C rounding exactly.

	internal static AABB ComputeCircleFatAABB( in Circle shape, Transform xf, float extra )
	{
		Vec2 c = TransformWorldPoint( xf, shape.center );
		double r = (double)shape.radius + (double)extra;
		AABB aabb = new( new Vec2( (float)( c.x - r ), (float)( c.y - r ) ), new Vec2( (float)( c.x + r ), (float)( c.y + r ) ) );
		return aabb;
	}

	internal static AABB ComputeCapsuleFatAABB( in Capsule shape, Transform xf, float extra )
	{
		Vec2 v1 = TransformWorldPoint( xf, shape.center1 );
		Vec2 v2 = TransformWorldPoint( xf, shape.center2 );
		double r = (double)shape.radius + (double)extra;
		AABB aabb = new( new Vec2( (float)( ( v1.x < v2.x ? v1.x : v2.x ) - r ), (float)( ( v1.y < v2.y ? v1.y : v2.y ) - r ) ),
						 new Vec2( (float)( ( v1.x > v2.x ? v1.x : v2.x ) + r ), (float)( ( v1.y > v2.y ? v1.y : v2.y ) + r ) ) );
		return aabb;
	}

	internal static AABB ComputePolygonFatAABB( in Polygon shape, Transform xf, float extra )
	{
		B2_ASSERT( shape.count > 0 );
		Vec2 v = TransformWorldPoint( xf, shape.vertices[0] );
		double lx = v.x, ly = v.y, ux = v.x, uy = v.y;

		for ( int i = 1; i < shape.count; ++i )
		{
			v = TransformWorldPoint( xf, shape.vertices[i] );
			lx = v.x < lx ? v.x : lx;
			ly = v.y < ly ? v.y : ly;
			ux = v.x > ux ? v.x : ux;
			uy = v.y > uy ? v.y : uy;
		}

		double r = (double)shape.radius + (double)extra;
		AABB aabb = new( new Vec2( (float)( lx - r ), (float)( ly - r ) ), new Vec2( (float)( ux + r ), (float)( uy + r ) ) );
		return aabb;
	}

	internal static AABB ComputeSegmentFatAABB( in Segment shape, Transform xf, float extra )
	{
		Vec2 v1 = TransformWorldPoint( xf, shape.point1 );
		Vec2 v2 = TransformWorldPoint( xf, shape.point2 );
		double e = (double)extra;
		AABB aabb = new( new Vec2( (float)( ( v1.x < v2.x ? v1.x : v2.x ) - e ), (float)( ( v1.y < v2.y ? v1.y : v2.y ) - e ) ),
						 new Vec2( (float)( ( v1.x > v2.x ? v1.x : v2.x ) + e ), (float)( ( v1.y > v2.y ? v1.y : v2.y ) + e ) ) );
		return aabb;
	}

	/// Compute the bounding box of a transformed circle
	public static AABB ComputeCircleBounds( in Circle shape, Transform xf ) => ComputeCircleFatAABB( shape, xf, 0.0f );

	/// Compute the bounding box of a transformed capsule
	public static AABB ComputeCapsuleBounds( in Capsule shape, Transform xf ) => ComputeCapsuleFatAABB( shape, xf, 0.0f );

	/// Compute the bounding box of a transformed polygon
	public static AABB ComputePolygonBounds( in Polygon shape, Transform xf ) => ComputePolygonFatAABB( shape, xf, 0.0f );

	/// Compute the bounding box of a transformed line segment
	public static AABB ComputeSegmentBounds( in Segment shape, Transform xf ) => ComputeSegmentFatAABB( shape, xf, 0.0f );

	/// Test a point for overlap with a circle in local space
	public static bool PointInCircle( in Circle shape, Vec2 point )
	{
		Vec2 center = shape.center;
		return DistanceSquared( point, center ) <= shape.radius * shape.radius;
	}

	/// Test a point for overlap with a capsule in local space
	public static bool PointInCapsule( in Capsule shape, Vec2 point )
	{
		float rr = shape.radius * shape.radius;
		Vec2 p1 = shape.center1;
		Vec2 p2 = shape.center2;

		Vec2 d = Sub( p2, p1 );
		float dd = Dot( d, d );
		if ( dd == 0.0f )
		{
			// Capsule is really a circle
			return DistanceSquared( point, p1 ) <= rr;
		}

		// Get closest point on capsule segment
		// c = p1 + t * d
		// dot(point - c, d) = 0
		// dot(point - p1 - t * d, d) = 0
		// t = dot(point - p1, d) / dot(d, d)
		float t = Dot( Sub( point, p1 ), d ) / dd;
		t = ClampFloat( t, 0.0f, 1.0f );
		Vec2 c = MulAdd( p1, t, d );

		// Is query point within radius around closest point?
		return DistanceSquared( point, c ) <= rr;
	}

	/// Test a point for overlap with a convex polygon in local space
	public static bool PointInPolygon( in Polygon shape, Vec2 point )
	{
		DistanceInput input = default;
		input.proxyA = MakeProxy( shape.vertices, shape.count, 0.0f );
		input.proxyB = MakeProxy( new ReadOnlySpan<Vec2>( in point ), 1, 0.0f );
		input.transform = Transform_identity;
		input.useRadii = false;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( input, ref cache, Span<Simplex>.Empty );

		return output.distance <= shape.radius;
	}

	/// Ray cast versus circle shape in local space.
	// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
	// http://www.codercorner.com/blog/?p=321
	public static CastOutput CastRayCircle( in Circle shape, in RayCastInput input )
	{
		B2_ASSERT( IsValidRay( input ) );

		Vec2 p = shape.center;

		CastOutput output = default;

		// Shift ray so circle center is the origin
		Vec2 s = Sub( input.origin, p );

		float r = shape.radius;
		float rr = r * r;

		Vec2 d = GetLengthAndNormalize( out float length, input.translation );
		if ( length == 0.0f )
		{
			// zero length ray

			if ( LengthSquared( s ) < rr )
			{
				// initial overlap
				output.point = input.origin;
				output.hit = true;
			}

			return output;
		}

		// Find closest point on ray to origin

		// solve: dot(s + t * d, d) = 0
		float t = -Dot( s, d );

		// c is the closest point on the line to the origin
		Vec2 c = MulAdd( s, t, d );

		float cc = Dot( c, c );

		if ( cc > rr )
		{
			// closest point is outside the circle
			return output;
		}

		// Pythagoras
		float h = MathF.Sqrt( rr - cc );

		float fraction = t - h;

		if ( fraction < 0.0f || input.maxFraction * length < fraction )
		{
			// intersection is point outside the range of the ray segment

			if ( LengthSquared( s ) < rr )
			{
				// initial overlap
				output.point = input.origin;
				output.hit = true;
			}

			return output;
		}

		// hit point relative to center
		Vec2 hitPoint = MulAdd( s, fraction, d );

		output.fraction = fraction / length;
		output.normal = Normalize( hitPoint );
		output.point = MulAdd( p, shape.radius, output.normal );
		output.hit = true;

		return output;
	}

	/// Ray cast versus capsule shape in local space.
	public static CastOutput CastRayCapsule( in Capsule shape, in RayCastInput input )
	{
		B2_ASSERT( IsValidRay( input ) );

		CastOutput output = default;

		Vec2 v1 = shape.center1;
		Vec2 v2 = shape.center2;

		Vec2 e = Sub( v2, v1 );

		Vec2 a = GetLengthAndNormalize( out float capsuleLength, e );

		if ( capsuleLength < FLT_EPSILON )
		{
			// Capsule is really a circle
			Circle circle = new( v1, shape.radius );
			return CastRayCircle( circle, input );
		}

		Vec2 p1 = input.origin;
		Vec2 d = input.translation;

		// Ray from capsule start to ray start
		Vec2 q = Sub( p1, v1 );
		float qa = Dot( q, a );

		// Vector to ray start that is perpendicular to capsule axis
		Vec2 qp = MulAdd( q, -qa, a );

		float radius = shape.radius;

		// Does the ray start within the infinite length capsule?
		if ( Dot( qp, qp ) < radius * radius )
		{
			if ( qa < 0.0f )
			{
				// start point behind capsule segment
				Circle circle = new( v1, shape.radius );
				return CastRayCircle( circle, input );
			}

			if ( qa > capsuleLength )
			{
				// start point ahead of capsule segment
				Circle circle = new( v2, shape.radius );
				return CastRayCircle( circle, input );
			}

			// ray starts inside capsule -> no hit
			output.point = input.origin;
			output.hit = true;
			return output;
		}

		// Perpendicular to capsule axis, pointing right
		Vec2 n = new( a.y, -a.x );

		Vec2 u = GetLengthAndNormalize( out float rayLength, d );

		// Intersect ray with infinite length capsule
		// v1 + radius * n + s1 * a = p1 + s2 * u
		// v1 - radius * n + s1 * a = p1 + s2 * u

		// s1 * a - s2 * u = b
		// b = q - radius * ap
		// or
		// b = q + radius * ap

		// Cramer's rule [a -u]
		float den = -a.x * u.y + u.x * a.y;
		if ( -FLT_EPSILON < den && den < FLT_EPSILON )
		{
			// Ray is parallel to capsule and outside infinite length capsule
			return output;
		}

		Vec2 b1 = MulSub( q, radius, n );
		Vec2 b2 = MulAdd( q, radius, n );

		float invDen = 1.0f / den;

		// Cramer's rule [a b1]
		float s21 = ( a.x * b1.y - b1.x * a.y ) * invDen;

		// Cramer's rule [a b2]
		float s22 = ( a.x * b2.y - b2.x * a.y ) * invDen;

		float s2;
		Vec2 b;
		if ( s21 < s22 )
		{
			s2 = s21;
			b = b1;
		}
		else
		{
			s2 = s22;
			b = b2;
			n = Neg( n );
		}

		if ( s2 < 0.0f || input.maxFraction * rayLength < s2 )
		{
			return output;
		}

		// Cramer's rule [b -u]
		float s1 = ( -b.x * u.y + u.x * b.y ) * invDen;

		if ( s1 < 0.0f )
		{
			// ray passes behind capsule segment
			Circle circle = new( v1, shape.radius );
			return CastRayCircle( circle, input );
		}
		else if ( capsuleLength < s1 )
		{
			// ray passes ahead of capsule segment
			Circle circle = new( v2, shape.radius );
			return CastRayCircle( circle, input );
		}
		else
		{
			// ray hits capsule side
			output.fraction = s2 / rayLength;
			output.point = Add( Lerp( v1, v2, s1 / capsuleLength ), MulSV( shape.radius, n ) );
			output.normal = n;
			output.hit = true;
			return output;
		}
	}

	/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with
	/// hits from the left side being treated as a miss.
	public static CastOutput CastRaySegment( in Segment shape, in RayCastInput input, bool oneSided )
	{
		if ( oneSided )
		{
			// Skip left-side collision
			float offset = Cross( Sub( input.origin, shape.point1 ), Sub( shape.point2, shape.point1 ) );
			if ( offset < 0.0f )
			{
				return default;
			}
		}

		// Put the ray into the edge's frame of reference.
		Vec2 p1 = input.origin;
		Vec2 d = input.translation;

		Vec2 v1 = shape.point1;
		Vec2 v2 = shape.point2;
		Vec2 e = Sub( v2, v1 );

		CastOutput output = default;

		Vec2 eUnit = GetLengthAndNormalize( out float length, e );
		if ( length == 0.0f )
		{
			return output;
		}

		// Normal points to the right, looking from v1 towards v2
		Vec2 normal = RightPerp( eUnit );

		// Intersect ray with infinite segment using normal
		// Similar to intersecting a ray with an infinite plane
		// p = p1 + t * d
		// dot(normal, p - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		float numerator = Dot( normal, Sub( v1, p1 ) );
		float denominator = Dot( normal, d );

		if ( denominator == 0.0f )
		{
			// parallel
			return output;
		}

		float t = numerator / denominator;
		if ( t < 0.0f || input.maxFraction < t )
		{
			// out of ray range
			return output;
		}

		// Intersection point on infinite segment
		Vec2 p = MulAdd( p1, t, d );

		// Compute position of p along segment
		// p = v1 + s * e
		// s = dot(p - v1, e) / dot(e, e)

		float s = Dot( Sub( p, v1 ), eUnit );
		if ( s < 0.0f || length < s )
		{
			// out of segment range
			return output;
		}

		if ( numerator > 0.0f )
		{
			normal = Neg( normal );
		}

		output.fraction = t;
		output.point = p;
		output.normal = normal;
		output.hit = true;

		return output;
	}

	/// Ray cast versus polygon shape in local space.
	public static CastOutput CastRayPolygon( in Polygon shape, in RayCastInput input )
	{
		B2_ASSERT( IsValidRay( input ) );

		if ( shape.radius == 0.0f )
		{
			// Shift all math to first vertex since the polygon may be far
			// from the origin.
			Vec2 @base = shape.vertices[0];

			Vec2 p1 = Sub( input.origin, @base );
			Vec2 d = input.translation;

			float lower = 0.0f, upper = input.maxFraction;

			int index = -1;

			CastOutput output = default;

			for ( int edgeIndex = 0; edgeIndex < shape.count; ++edgeIndex )
			{
				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0
				Vec2 vertex = Sub( shape.vertices[edgeIndex], @base );
				float numerator = Dot( shape.normals[edgeIndex], Sub( vertex, p1 ) );
				float denominator = Dot( shape.normals[edgeIndex], d );

				if ( denominator == 0.0f )
				{
					// Parallel and runs outside edge
					if ( numerator < 0.0f )
					{
						return output;
					}
				}
				else
				{
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower > numerator.
					if ( denominator < 0.0f && numerator < lower * denominator )
					{
						// Increase lower.
						// The segment enters this half-space.
						lower = numerator / denominator;
						index = edgeIndex;
					}
					else if ( denominator > 0.0f && numerator < upper * denominator )
					{
						// Decrease upper.
						// The segment exits this half-space.
						upper = numerator / denominator;
					}
				}

				if ( upper < lower )
				{
					// Ray misses
					return output;
				}
			}

			B2_ASSERT( 0.0f <= lower && lower <= input.maxFraction );

			if ( index >= 0 )
			{
				output.fraction = lower;
				output.normal = shape.normals[index];
				output.point = MulAdd( input.origin, lower, d );
				output.hit = true;
			}
			else
			{
				// initial overlap
				output.point = input.origin;
				output.hit = true;
			}

			return output;
		}

		ShapeCastPairInput castInput;
		castInput.proxyA = MakeProxy( shape.vertices, shape.count, shape.radius );
		castInput.proxyB = MakeProxy( new ReadOnlySpan<Vec2>( in input.origin ), 1, 0.0f );
		castInput.transform = Transform_identity;
		castInput.translationB = input.translation;
		castInput.maxFraction = input.maxFraction;
		castInput.canEncroach = false;
		return CastShape( castInput );
	}

	/// Shape cast versus a circle.
	public static CastOutput CastShapeCircle( in Circle shape, in ShapeCastInput input )
	{
		ShapeCastPairInput pairInput;
		pairInput.proxyA = MakeProxy( new ReadOnlySpan<Vec2>( in shape.center ), 1, shape.radius );
		pairInput.proxyB = input.proxy;
		pairInput.transform = Transform_identity;
		pairInput.translationB = input.translation;
		pairInput.maxFraction = input.maxFraction;
		pairInput.canEncroach = input.canEncroach;

		return CastShape( pairInput );
	}

	/// Shape cast versus a capsule.
	public static CastOutput CastShapeCapsule( in Capsule shape, in ShapeCastInput input )
	{
		ShapeCastPairInput pairInput;
		pairInput.proxyA = MakeProxy2( shape.center1, shape.center2, shape.radius );
		pairInput.proxyB = input.proxy;
		pairInput.transform = Transform_identity;
		pairInput.translationB = input.translation;
		pairInput.maxFraction = input.maxFraction;
		pairInput.canEncroach = input.canEncroach;

		return CastShape( pairInput );
	}

	/// Shape cast versus a line segment.
	public static CastOutput CastShapeSegment( in Segment shape, in ShapeCastInput input )
	{
		ShapeCastPairInput pairInput;
		pairInput.proxyA = MakeProxy2( shape.point1, shape.point2, 0.0f );
		pairInput.proxyB = input.proxy;
		pairInput.transform = Transform_identity;
		pairInput.translationB = input.translation;
		pairInput.maxFraction = input.maxFraction;
		pairInput.canEncroach = input.canEncroach;

		return CastShape( pairInput );
	}

	/// Shape cast versus a convex polygon.
	public static CastOutput CastShapePolygon( in Polygon shape, in ShapeCastInput input )
	{
		ShapeCastPairInput pairInput;
		pairInput.proxyA = MakeProxy( shape.vertices, shape.count, shape.radius );
		pairInput.proxyB = input.proxy;
		pairInput.transform = Transform_identity;
		pairInput.translationB = input.translation;
		pairInput.maxFraction = input.maxFraction;
		pairInput.canEncroach = input.canEncroach;

		return CastShape( pairInput );
	}

	private static PlaneResult CollideMoverAndProxy( in Capsule mover, in ShapeProxy proxyA, float totalRadius )
	{
		DistanceInput distanceInput;
		distanceInput.proxyA = proxyA;
		distanceInput.proxyB = MakeProxy2( mover.center1, mover.center2, mover.radius );
		distanceInput.transform = Transform_identity;
		distanceInput.useRadii = false;

		SimplexCache cache = default;
		DistanceOutput distanceOutput = ComputeShapeDistance( distanceInput, ref cache, Span<Simplex>.Empty );

		if ( distanceOutput.distance <= totalRadius )
		{
			Plane plane = new( distanceOutput.normal, totalRadius - distanceOutput.distance );
			return new PlaneResult { plane = plane, point = distanceOutput.pointA, hit = true };
		}

		return default;
	}

	internal static PlaneResult CollideMoverAndCircle( in Capsule mover, in Circle shape )
	{
		ShapeProxy proxy = MakeProxy( new ReadOnlySpan<Vec2>( in shape.center ), 1, 0.0f );
		return CollideMoverAndProxy( mover, proxy, mover.radius + shape.radius );
	}

	internal static PlaneResult CollideMoverAndCapsule( in Capsule mover, in Capsule shape )
	{
		ShapeProxy proxy = MakeProxy2( shape.center1, shape.center2, 0.0f );
		return CollideMoverAndProxy( mover, proxy, mover.radius + shape.radius );
	}

	internal static PlaneResult CollideMoverAndPolygon( in Capsule mover, in Polygon shape )
	{
		ShapeProxy proxy = MakeProxy( shape.vertices, shape.count, shape.radius );
		return CollideMoverAndProxy( mover, proxy, mover.radius + shape.radius );
	}

	internal static PlaneResult CollideMoverAndSegment( in Capsule mover, in Segment shape )
	{
		ShapeProxy proxy = MakeProxy2( shape.point1, shape.point2, 0.0f );
		return CollideMoverAndProxy( mover, proxy, mover.radius );
	}
}

public static partial class B2
{
	// From geometry.c, needs the internal shape type
	internal static AABB ComputeFatShapeAABB( in Shape shape, Transform xf, float extra )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return ComputeCapsuleFatAABB( shape.capsule, xf, extra );
			case ShapeType.circleShape:
				return ComputeCircleFatAABB( shape.circle, xf, extra );
			case ShapeType.polygonShape:
				return ComputePolygonFatAABB( shape.polygon, xf, extra );
			case ShapeType.segmentShape:
				return ComputeSegmentFatAABB( shape.segment, xf, extra );
			case ShapeType.chainSegmentShape:
				return ComputeSegmentFatAABB( shape.chainSegment.segment, xf, extra );
			default:
			{
				B2_ASSERT( false );
				AABB empty = new( new Vec2( 0.0f, 0.0f ), new Vec2( 0.0f, 0.0f ) );
				return empty;
			}
		}
	}
}
