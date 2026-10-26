// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using static Box2DNet.B2;

namespace Box2DNet.Shared;

// From shared/utils.h
public static partial class SharedLib
{
	public const int RAND_LIMIT = 32767;
	public const uint RAND_SEED = 12345;

	// Global seed for simple random number generator.
	[ThreadStatic]
	private static uint s_randomSeed;
	[ThreadStatic]
	private static bool s_seeded;

	public static uint g_randomSeed
	{
		get => s_seeded ? s_randomSeed : RAND_SEED;
		set
		{
			s_randomSeed = value;
			s_seeded = true;
		}
	}

	internal static void assert( bool condition )
	{
		if ( condition == false )
		{
			throw new InvalidOperationException( "assertion failed" );
		}
	}

	// Simple random number generator. Using this instead of rand() for cross-platform determinism.
	public static int RandomInt()
	{
		// XorShift32 algorithm
		uint x = g_randomSeed;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		g_randomSeed = x;

		// Map the 32-bit value to the range 0 to RAND_LIMIT
		return (int)( x % ( RAND_LIMIT + 1 ) );
	}

	// Random integer in range [lo, hi]
	public static int RandomIntRange( int lo, int hi )
	{
		return lo + RandomInt() % ( hi - lo + 1 );
	}

	// Random number in range [-1,1]
	public static float RandomFloat()
	{
		float r = (float)( RandomInt() & RAND_LIMIT );
		r /= RAND_LIMIT;
		r = 2.0f * r - 1.0f;
		return r;
	}

	// Random floating point number in range [lo, hi]
	public static float RandomFloatRange( float lo, float hi )
	{
		float r = (float)( RandomInt() & RAND_LIMIT );
		r /= RAND_LIMIT;
		r = ( hi - lo ) * r + lo;
		return r;
	}

	// Random vector with coordinates in range [lo, hi]
	public static Vec2 RandomVec2( float lo, float hi )
	{
		Vec2 v;
		v.x = RandomFloatRange( lo, hi );
		v.y = RandomFloatRange( lo, hi );
		return v;
	}

	// Random position with coordinates in range [lo, hi]
	public static Pos RandomPos( float lo, float hi )
	{
		Pos v;
		v.x = RandomFloatRange( lo, hi );
		v.y = RandomFloatRange( lo, hi );
		return v;
	}

	// Random rotation with angle in range [-pi, pi]
	public static Rot RandomRot()
	{
		float angle = RandomFloatRange( -Pi, Pi );
		return MakeRot( angle );
	}

	public static Polygon RandomPolygon( float extent )
	{
		Span<Vec2> points = stackalloc Vec2[MaxPolygonVertices];
		int count = 3 + RandomInt() % 6;
		for ( int i = 0; i < count; ++i )
		{
			points[i] = RandomVec2( -extent, extent );
		}

		Hull hull = ComputeHull( points, count );
		if ( hull.count > 0 )
		{
			return MakePolygon( hull, 0.0f );
		}

		return MakeSquare( extent );
	}

	public static int GetNumberOfCores()
	{
		return Environment.ProcessorCount;
	}
}
