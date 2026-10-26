// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Numerics;
using Hexa.NET.ImGui;
using static Box2DNet.B2;

namespace Box2DNet.Testbed;

// From draw.h. The world y-axis points up, the screen y-axis points down.
public sealed class Camera
{
	// World point the view is centered on.
	public Pos center;

	// Half the view height in meters.
	public float zoom;
	public float width;
	public float height;
}

// Collects world space draws and renders them with the ImGui background draw list. Like draw.c, solid
// shapes go first and lines and points go on top.
public sealed class Draw
{
	internal readonly struct Solid
	{
		public readonly Vector2[] points;
		public readonly int count;
		public readonly uint color;

		public Solid( Vector2[] points, int count, uint color )
		{
			this.points = points;
			this.count = count;
			this.color = color;
		}
	}

	internal readonly List<(Pos center, float radius, Rot q, HexColor color)> solidCircles = new();
	internal readonly List<(Pos p1, Pos p2, float radius, HexColor color)> capsules = new();
	internal readonly List<(Transform xf, Vec2[] vertices, int count, float radius, HexColor color)> polygons = new();
	internal readonly List<(Pos center, float radius, HexColor color)> hollowCircles = new();
	internal readonly List<(Pos p1, Pos p2, HexColor color)> lines = new();
	internal readonly List<(Pos p, float size, HexColor color)> points = new();
	internal readonly List<(Vector2 p, string text, HexColor color)> strings = new();
	internal Pos origin;
}

public static class DrawFunctions
{
	public static Camera GetDefaultCamera()
	{
		return new Camera { center = new Pos( 0.0f, 20.0f ), zoom = 1.0f, width = 1920.0f, height = 1080.0f };
	}

	public static void ResetView( Camera camera )
	{
		camera.center = new Pos( 0.0f, 20.0f );
		camera.zoom = 1.0f;
	}

	public static Pos ConvertScreenToWorld( Camera camera, Vec2 screenPoint )
	{
		float w = camera.width;
		float h = camera.height;
		float u = screenPoint.x / w;
		float v = ( h - screenPoint.y ) / h;

		float ratio = w / h;
		Vec2 extents = new( camera.zoom * ratio, camera.zoom );

		Vec2 offset = new( extents.x * ( 2.0f * u - 1.0f ), extents.y * ( 2.0f * v - 1.0f ) );
		return OffsetPos( camera.center, offset );
	}

	public static Vec2 ConvertViewToScreen( Camera camera, Vec2 viewPoint )
	{
		float w = camera.width;
		float h = camera.height;
		float ratio = w / h;

		Vec2 extents = new( camera.zoom * ratio, camera.zoom );

		float u = ( viewPoint.x + extents.x ) / ( 2.0f * extents.x );
		float v = ( viewPoint.y + extents.y ) / ( 2.0f * extents.y );

		return new Vec2( u * w, ( 1.0f - v ) * h );
	}

	public static Vec2 ConvertWorldToScreen( Camera camera, Pos worldPoint )
	{
		return ConvertViewToScreen( camera, SubPos( worldPoint, camera.center ) );
	}

	public static AABB GetViewBounds( Camera camera )
	{
		if ( camera.height == 0.0f || camera.width == 0.0f )
		{
			return new AABB( Vec2_zero, Vec2_zero );
		}

		Pos lower = ConvertScreenToWorld( camera, new Vec2( 0.0f, camera.height ) );
		Pos upper = ConvertScreenToWorld( camera, new Vec2( camera.width, 0.0f ) );
		return new AABB( lower, upper );
	}

	public static void FocusOnBounds( Camera camera, AABB bounds )
	{
		if ( camera.width == 0 )
		{
			return;
		}

		Vec2 extents = AABB_Extents( bounds );

		if ( extents.x < LinearSlop || extents.y < LinearSlop )
		{
			return;
		}

		float invRatio = camera.height / camera.width;
		camera.zoom = MaxFloat( extents.x * invRatio, extents.y );

		// Need to guard against zero because zoom can get stuck there
		camera.zoom = MaxFloat( camera.zoom, 0.01f );

		camera.center = AABB_Center( bounds );
	}

	public static Draw CreateDraw() => new();

	public static void SetDrawOrigin( Draw draw, Pos origin )
	{
		draw.origin = origin;
	}

	// HexColor is 0xRRGGBB, ImGui wants 0xAABBGGRR
	public static uint ToImColor( HexColor color, float alpha = 1.0f )
	{
		uint c = (uint)color;
		uint r = ( c >> 16 ) & 0xFF;
		uint g = ( c >> 8 ) & 0xFF;
		uint b = c & 0xFF;
		uint a = (uint)( 255.0f * alpha );
		return r | ( g << 8 ) | ( b << 16 ) | ( a << 24 );
	}

	public static Vector4 MakeColor( HexColor hexColor )
	{
		uint c = (uint)hexColor;
		return new Vector4( ( ( c >> 16 ) & 0xFF ) / 255.0f, ( ( c >> 8 ) & 0xFF ) / 255.0f, ( c & 0xFF ) / 255.0f, 1.0f );
	}

	public static void DrawScreenString( Draw draw, float x, float y, HexColor color, string format, params object?[] args )
	{
		draw.strings.Add( ( new Vector2( x, y ), Printf.Fmt( format, args ), color ) );
	}

	public static void DrawString( Draw draw, Camera camera, Pos p, HexColor color, string format, params object?[] args )
	{
		Vec2 ps = ConvertWorldToScreen( camera, p );
		draw.strings.Add( ( new Vector2( ps.x, ps.y ), Printf.Fmt( format, args ), color ) );
	}

	public static void DrawPoint( Draw draw, Pos p, float size, HexColor color )
	{
		draw.points.Add( ( p, size, color ) );
	}

	public static void DrawLine( Draw draw, Pos p1, Pos p2, HexColor color )
	{
		draw.lines.Add( ( p1, p2, color ) );
	}

	public static void DrawCircle( Draw draw, Pos center, float radius, HexColor color )
	{
		draw.hollowCircles.Add( ( center, radius, color ) );
	}

	public static void DrawCapsule( Draw draw, Pos p1, Pos p2, float radius, HexColor color )
	{
		draw.capsules.Add( ( p1, p2, radius, color ) );
	}

	public static void DrawPolygon( Draw draw, Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, HexColor color )
	{
		Vec2 p1 = TransformPoint( transform, vertices[vertexCount - 1] );
		for ( int i = 0; i < vertexCount; ++i )
		{
			Vec2 p2 = TransformPoint( transform, vertices[i] );
			draw.lines.Add( ( p1, p2, color ) );
			p1 = p2;
		}
	}

	public static void DrawSolidCircle( Draw draw, Transform transform, Vec2 center, float radius, HexColor color )
	{
		draw.solidCircles.Add( ( TransformPoint( transform, center ), radius, transform.q, color ) );
	}

	public static void DrawSolidPolygon( Draw draw, Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius,
										 HexColor color )
	{
		draw.polygons.Add( ( transform, vertices[..vertexCount].ToArray(), vertexCount, radius, color ) );
	}

	public static void DrawTransform( Draw draw, Transform transform, float scale )
	{
		Vec2 p1 = transform.p;

		Vec2 p2 = MulAdd( p1, scale, Rot_GetXAxis( transform.q ) );
		draw.lines.Add( ( p1, p2, HexColor.colorRed ) );

		p2 = MulAdd( p1, scale, Rot_GetYAxis( transform.q ) );
		draw.lines.Add( ( p1, p2, HexColor.colorGreen ) );
	}

	public static void DrawBounds( Draw draw, AABB aabb, HexColor color )
	{
		Vec2 p1 = aabb.lowerBound;
		Vec2 p2 = new( aabb.upperBound.x, aabb.lowerBound.y );
		Vec2 p3 = aabb.upperBound;
		Vec2 p4 = new( aabb.lowerBound.x, aabb.upperBound.y );

		draw.lines.Add( ( p1, p2, color ) );
		draw.lines.Add( ( p2, p3, color ) );
		draw.lines.Add( ( p3, p4, color ) );
		draw.lines.Add( ( p4, p1, color ) );
	}

	private static Vector2 S( Camera camera, Pos p )
	{
		Vec2 v = ConvertWorldToScreen( camera, p );
		return new Vector2( v.x, v.y );
	}

	private static float Pixels( Camera camera, float meters ) => meters * camera.height / ( 2.0f * camera.zoom );

	private static void FillCapsule( ImDrawListPtr dl, Camera camera, Pos p1, Pos p2, float radius, uint fill )
	{
		Vector2 s1 = S( camera, p1 );
		Vector2 s2 = S( camera, p2 );
		float r = Pixels( camera, radius );
		dl.AddCircleFilled( s1, r, fill );
		dl.AddCircleFilled( s2, r, fill );

		Vec2 d = Sub( p2, p1 );
		if ( LengthSquared( d ) > 0.0f )
		{
			Vec2 n = MulSV( radius, LeftPerp( Normalize( d ) ) );
			Span<Vector2> quad = stackalloc Vector2[4];
			quad[0] = S( camera, Add( p1, n ) );
			quad[1] = S( camera, Sub( p1, n ) );
			quad[2] = S( camera, Sub( p2, n ) );
			quad[3] = S( camera, Add( p2, n ) );
			dl.AddConvexPolyFilled( ref quad[0], 4, fill );
		}
	}

	private static void OutlineCapsule( ImDrawListPtr dl, Camera camera, Pos p1, Pos p2, float radius, uint line )
	{
		Vec2 d = Sub( p2, p1 );
		float r = Pixels( camera, radius );
		if ( LengthSquared( d ) == 0.0f )
		{
			dl.AddCircle( S( camera, p1 ), r, line );
			return;
		}

		Vec2 axis = Normalize( d );
		Vec2 n = MulSV( radius, LeftPerp( axis ) );
		dl.AddLine( S( camera, Add( p1, n ) ), S( camera, Add( p2, n ) ), line );
		dl.AddLine( S( camera, Sub( p1, n ) ), S( camera, Sub( p2, n ) ), line );

		// half circle arcs, screen space angles are flipped because the y-axis points down
		float angle = MathF.Atan2( -axis.y, axis.x );
		Vector2 s1 = S( camera, p1 );
		Vector2 s2 = S( camera, p2 );
		dl.PathArcTo( s2, r, angle - 0.5f * MathF.PI, angle + 0.5f * MathF.PI );
		dl.PathStroke( line );
		dl.PathArcTo( s1, r, angle + 0.5f * MathF.PI, angle + 1.5f * MathF.PI );
		dl.PathStroke( line );
	}

	public static void FlushDraw( Draw draw, Camera camera )
	{
		ImDrawListPtr dl = ImGui.GetBackgroundDrawList();
		Span<Vector2> poly = stackalloc Vector2[MaxPolygonVertices + 1];

		foreach ( var c in draw.solidCircles )
		{
			Vector2 sc = S( camera, c.center );
			float r = Pixels( camera, c.radius );
			dl.AddCircleFilled( sc, r, ToImColor( c.color, 0.5f ) );
			dl.AddCircle( sc, r, ToImColor( c.color ) );

			// A radius line shows the rotation
			dl.AddLine( sc, S( camera, MulAdd( c.center, c.radius, Rot_GetXAxis( c.q ) ) ), ToImColor( c.color ) );
		}

		foreach ( var c in draw.capsules )
		{
			FillCapsule( dl, camera, c.p1, c.p2, c.radius, ToImColor( c.color, 0.5f ) );
			OutlineCapsule( dl, camera, c.p1, c.p2, c.radius, ToImColor( c.color ) );
		}

		foreach ( var p in draw.polygons )
		{
			uint fill = ToImColor( p.color, 0.5f );
			uint line = ToImColor( p.color );
			for ( int i = 0; i < p.count; ++i )
			{
				poly[i] = S( camera, TransformPoint( p.xf, p.vertices[i] ) );
			}

			if ( p.radius > 0.0f )
			{
				// Rounded polygon: the core plus a capsule along each edge
				for ( int i = 0; i < p.count; ++i )
				{
					Vec2 v1 = TransformPoint( p.xf, p.vertices[i] );
					Vec2 v2 = TransformPoint( p.xf, p.vertices[( i + 1 ) % p.count] );
					FillCapsule( dl, camera, v1, v2, p.radius, fill );
				}
			}

			if ( p.count >= 3 )
			{
				dl.AddConvexPolyFilled( ref poly[0], p.count, fill );
			}

			if ( p.radius > 0.0f )
			{
				for ( int i = 0; i < p.count; ++i )
				{
					Vec2 v1 = TransformPoint( p.xf, p.vertices[i] );
					Vec2 v2 = TransformPoint( p.xf, p.vertices[( i + 1 ) % p.count] );
					Vec2 n = MulSV( p.radius, RightPerp( Normalize( Sub( v2, v1 ) ) ) );
					dl.AddLine( S( camera, Add( v1, n ) ), S( camera, Add( v2, n ) ), line );
				}
			}
			else
			{
				dl.AddPolyline( ref poly[0], p.count, line, ImDrawFlags.Closed, 1.0f );
			}
		}

		foreach ( var c in draw.hollowCircles )
		{
			dl.AddCircle( S( camera, c.center ), Pixels( camera, c.radius ), ToImColor( c.color ) );
		}

		foreach ( var l in draw.lines )
		{
			dl.AddLine( S( camera, l.p1 ), S( camera, l.p2 ), ToImColor( l.color ) );
		}

		foreach ( var p in draw.points )
		{
			dl.AddCircleFilled( S( camera, p.p ), 0.5f * p.size, ToImColor( p.color ) );
		}

		foreach ( var s in draw.strings )
		{
			dl.AddText( s.p, ToImColor( s.color ), s.text );
		}

		draw.solidCircles.Clear();
		draw.capsules.Clear();
		draw.polygons.Clear();
		draw.hollowCircles.Clear();
		draw.lines.Clear();
		draw.points.Clear();
		draw.strings.Clear();
	}
}
