// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.InteropServices;

namespace Box2DNet;

/// 2D vector This can be used to represent a point or free vector
[StructLayout( LayoutKind.Sequential )]
public struct Vec2
{
	/// coordinates
	public float x, y;

	public Vec2( float x, float y )
	{
		this.x = x;
		this.y = y;
	}

	public static readonly Vec2 Zero = new( 0.0f, 0.0f );

	public static Vec2 operator +( Vec2 a, Vec2 b ) => new( a.x + b.x, a.y + b.y );
	public static Vec2 operator -( Vec2 a, Vec2 b ) => new( a.x - b.x, a.y - b.y );
	public static Vec2 operator -( Vec2 a ) => new( -a.x, -a.y );
	public static Vec2 operator *( float a, Vec2 b ) => new( a * b.x, a * b.y );
	public static Vec2 operator *( Vec2 a, float b ) => new( a.x * b, a.y * b );
	public static bool operator ==( Vec2 a, Vec2 b ) => a.x == b.x && a.y == b.y;
	public static bool operator !=( Vec2 a, Vec2 b ) => a.x != b.x || a.y != b.y;

	public readonly override bool Equals( object? obj ) => obj is Vec2 v && this == v;
	public readonly override int GetHashCode() => System.HashCode.Combine( x, y );
	public readonly override string ToString() => $"({x}, {y})";
}

/// Cosine and sine pair This uses a custom implementation designed for cross-platform determinism
[StructLayout( LayoutKind.Sequential )]
public struct CosSin
{
	/// cosine and sine
	public float cosine;
	public float sine;
}

/// 2D rotation This is similar to using a complex number for rotation
[StructLayout( LayoutKind.Sequential )]
public struct Rot
{
	/// cosine and sine
	public float c, s;

	public Rot( float c, float s )
	{
		this.c = c;
		this.s = s;
	}

	public static readonly Rot Identity = new( 1.0f, 0.0f );

	public readonly override string ToString() => $"(c={c}, s={s})";
}

/// A 2D rigid transform
[StructLayout( LayoutKind.Sequential )]
public struct Transform
{
	public Vec2 p;
	public Rot q;

	public Transform( Vec2 p, Rot q )
	{
		this.p = p;
		this.q = q;
	}

	public static readonly Transform Identity = new( new Vec2( 0.0f, 0.0f ), new Rot( 1.0f, 0.0f ) );

	public readonly override string ToString() => $"p={p}, q={q}";
}

/// A 2-by-2 Matrix
[StructLayout( LayoutKind.Sequential )]
public struct Mat22
{
	/// columns
	public Vec2 cx, cy;

	public Mat22( Vec2 cx, Vec2 cy )
	{
		this.cx = cx;
		this.cy = cy;
	}

	public static readonly Mat22 Zero = default;
}

/// Axis-aligned bounding box
[StructLayout( LayoutKind.Sequential )]
public struct AABB
{
	public Vec2 lowerBound;
	public Vec2 upperBound;

	public AABB( Vec2 lowerBound, Vec2 upperBound )
	{
		this.lowerBound = lowerBound;
		this.upperBound = upperBound;
	}

	public readonly override string ToString() => $"[{lowerBound}, {upperBound}]";
}

/// separation = dot(normal, point) - offset
[StructLayout( LayoutKind.Sequential )]
public struct Plane
{
	public Vec2 normal;
	public float offset;

	public Plane( Vec2 normal, float offset )
	{
		this.normal = normal;
		this.offset = offset;
	}
}
