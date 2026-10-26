// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.CompilerServices;

namespace Box2DNet;

public static partial class B2
{
	/// Is this a valid bounding box? Not NaN or infinity. Upper bound greater than or equal to lower
	/// bound.
	public static bool IsValidAABB( AABB a )
	{
		Vec2 d = Sub( a.upperBound, a.lowerBound );
		bool valid = d.x >= 0.0f && d.y >= 0.0f;
		valid = valid && IsValidVec2( a.lowerBound ) && IsValidVec2( a.upperBound );
		return valid;
	}

	// Ray cast an AABB
	// From Real-time Collision Detection, p179.
	internal static CastOutput AABB_RayCast( AABB a, Vec2 p1, Vec2 p2 )
	{
		// Radius not handled
		CastOutput output = default;

		float tMin = -FLT_MAX;
		float tMax = FLT_MAX;

		Vec2 p = p1;
		Vec2 d = Sub( p2, p1 );
		Vec2 absD = Abs( d );

		Vec2 normal = Vec2_zero;

		// x-coordinate
		if ( absD.x < FLT_EPSILON )
		{
			// parallel
			if ( p.x < a.lowerBound.x || a.upperBound.x < p.x )
			{
				return output;
			}
		}
		else
		{
			float inv_d = 1.0f / d.x;
			float t1 = ( a.lowerBound.x - p.x ) * inv_d;
			float t2 = ( a.upperBound.x - p.x ) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if ( t1 > t2 )
			{
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
				s = 1.0f;
			}

			// Push the min up
			if ( t1 > tMin )
			{
				normal.y = 0.0f;
				normal.x = s;
				tMin = t1;
			}

			// Pull the max down
			tMax = MinFloat( tMax, t2 );

			if ( tMin > tMax )
			{
				return output;
			}
		}

		// y-coordinate
		if ( absD.y < FLT_EPSILON )
		{
			// parallel
			if ( p.y < a.lowerBound.y || a.upperBound.y < p.y )
			{
				return output;
			}
		}
		else
		{
			float inv_d = 1.0f / d.y;
			float t1 = ( a.lowerBound.y - p.y ) * inv_d;
			float t2 = ( a.upperBound.y - p.y ) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if ( t1 > t2 )
			{
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
				s = 1.0f;
			}

			// Push the min up
			if ( t1 > tMin )
			{
				normal.x = 0.0f;
				normal.y = s;
				tMin = t1;
			}

			// Pull the max down
			tMax = MinFloat( tMax, t2 );

			if ( tMin > tMax )
			{
				return output;
			}
		}

		// Does the ray start inside the box?
		if ( tMin < 0.0f )
		{
			return output;
		}

		// Does the ray intersect beyond the segment length?
		if ( 1.0f < tMin )
		{
			return output;
		}

		// Intersection.
		output.fraction = tMin;
		output.normal = normal;
		output.point = Lerp( p1, p2, tMin );
		output.hit = true;
		return output;
	}

	// Get surface area of an AABB (the perimeter length)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static float Perimeter( AABB a )
	{
		float wx = a.upperBound.x - a.lowerBound.x;
		float wy = a.upperBound.y - a.lowerBound.y;
		return 2.0f * ( wx + wy );
	}

	/// Enlarge a to contain b
	/// @return true if the AABB grew
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool EnlargeAABB( ref AABB a, AABB b )
	{
		bool changed = false;
		if ( b.lowerBound.x < a.lowerBound.x )
		{
			a.lowerBound.x = b.lowerBound.x;
			changed = true;
		}

		if ( b.lowerBound.y < a.lowerBound.y )
		{
			a.lowerBound.y = b.lowerBound.y;
			changed = true;
		}

		if ( a.upperBound.x < b.upperBound.x )
		{
			a.upperBound.x = b.upperBound.x;
			changed = true;
		}

		if ( a.upperBound.y < b.upperBound.y )
		{
			a.upperBound.y = b.upperBound.y;
			changed = true;
		}

		return changed;
	}

	// Translate a relative AABB into world space. In single precision this is a plain offset.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static AABB OffsetAABB( AABB box, Vec2 origin )
	{
		AABB result;
		result.lowerBound.x = origin.x + box.lowerBound.x;
		result.lowerBound.y = origin.y + box.lowerBound.y;
		result.upperBound.x = origin.x + box.upperBound.x;
		result.upperBound.y = origin.y + box.upperBound.y;
		return result;
	}
}
