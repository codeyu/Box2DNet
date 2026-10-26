// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Numerics;
using Hexa.NET.ImGui;
using static Box2DNet.B2;

namespace Box2DNet.Testbed;

// Implements the Box2D debug draw callbacks with the ImGui background draw list.
public sealed class ImGuiDraw
{
	private readonly Camera m_camera;
	public DebugDraw debugDraw;

	public ImGuiDraw( Camera camera )
	{
		m_camera = camera;
		debugDraw = DefaultDebugDraw();
		debugDraw.DrawPolygonFcn = DrawPolygon;
		debugDraw.DrawSolidPolygonFcn = DrawSolidPolygon;
		debugDraw.DrawCircleFcn = DrawCircle;
		debugDraw.DrawSolidCircleFcn = DrawSolidCircle;
		debugDraw.DrawSolidCapsuleFcn = DrawSolidCapsule;
		debugDraw.DrawLineFcn = DrawLine;
		debugDraw.DrawTransformFcn = DrawTransform;
		debugDraw.DrawPointFcn = DrawPoint;
		debugDraw.DrawStringFcn = DrawString;
		debugDraw.DrawBoundsFcn = DrawBounds;
		debugDraw.drawShapes = true;
		debugDraw.drawJoints = true;
	}

	private static ImDrawListPtr DrawList => ImGui.GetBackgroundDrawList();

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

	private float Pixels( float meters ) => meters * m_camera.PixelsPerMeter;

	private void DrawPolygon( Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, HexColor color, object? context )
	{
		Span<Vector2> points = stackalloc Vector2[MaxPolygonVertices];
		for ( int i = 0; i < vertexCount; ++i )
		{
			points[i] = m_camera.ToScreen( TransformPoint( transform, vertices[i] ) );
		}

		DrawList.AddPolyline( ref points[0], vertexCount, ToImColor( color ), ImDrawFlags.Closed, 1.0f );
	}

	private void DrawSolidPolygon( Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, HexColor color,
								   object? context )
	{
		if ( radius > 0.0f )
		{
			// Rounded polygon: fill the core and the rounded edges with capsules
			for ( int i = 0; i < vertexCount; ++i )
			{
				Vec2 p1 = TransformPoint( transform, vertices[i] );
				Vec2 p2 = TransformPoint( transform, vertices[( i + 1 ) % vertexCount] );
				DrawSolidCapsule( p1, p2, radius, color, context );
			}
		}

		Span<Vector2> points = stackalloc Vector2[MaxPolygonVertices];
		for ( int i = 0; i < vertexCount; ++i )
		{
			points[i] = m_camera.ToScreen( TransformPoint( transform, vertices[i] ) );
		}

		DrawList.AddConvexPolyFilled( ref points[0], vertexCount, ToImColor( color, 0.5f ) );
		DrawList.AddPolyline( ref points[0], vertexCount, ToImColor( color ), ImDrawFlags.Closed, 1.0f );
	}

	private void DrawCircle( Vec2 center, float radius, HexColor color, object? context )
	{
		DrawList.AddCircle( m_camera.ToScreen( center ), Pixels( radius ), ToImColor( color ) );
	}

	private void DrawSolidCircle( Transform transform, Vec2 center, float radius, HexColor color, object? context )
	{
		Vec2 c = TransformPoint( transform, center );
		Vector2 sc = m_camera.ToScreen( c );
		DrawList.AddCircleFilled( sc, Pixels( radius ), ToImColor( color, 0.5f ) );
		DrawList.AddCircle( sc, Pixels( radius ), ToImColor( color ) );

		// A radius line shows the rotation
		Vec2 axis = RotateVector( transform.q, new Vec2( radius, 0.0f ) );
		DrawList.AddLine( sc, m_camera.ToScreen( Add( c, axis ) ), ToImColor( color ) );
	}

	private void DrawSolidCapsule( Vec2 p1, Vec2 p2, float radius, HexColor color, object? context )
	{
		uint fill = ToImColor( color, 0.5f );
		uint line = ToImColor( color );
		Vector2 s1 = m_camera.ToScreen( p1 );
		Vector2 s2 = m_camera.ToScreen( p2 );
		float r = Pixels( radius );

		Vec2 axis = Normalize( Sub( p2, p1 ) );
		Vec2 n = MulSV( radius, LeftPerp( axis ) );
		Span<Vector2> quad = stackalloc Vector2[4];
		quad[0] = m_camera.ToScreen( Add( p1, n ) );
		quad[1] = m_camera.ToScreen( Sub( p1, n ) );
		quad[2] = m_camera.ToScreen( Sub( p2, n ) );
		quad[3] = m_camera.ToScreen( Add( p2, n ) );

		DrawList.AddCircleFilled( s1, r, fill );
		DrawList.AddCircleFilled( s2, r, fill );
		DrawList.AddConvexPolyFilled( ref quad[0], 4, fill );
		DrawList.AddLine( quad[0], quad[3], line );
		DrawList.AddLine( quad[1], quad[2], line );
		DrawList.AddLine( s1, s2, line );
	}

	private void DrawLine( Vec2 p1, Vec2 p2, HexColor color, object? context )
	{
		DrawList.AddLine( m_camera.ToScreen( p1 ), m_camera.ToScreen( p2 ), ToImColor( color ) );
	}

	private void DrawTransform( Transform transform, object? context )
	{
		const float axisScale = 0.2f;
		Vec2 p = transform.p;
		DrawLine( p, MulAdd( p, axisScale, Rot_GetXAxis( transform.q ) ), HexColor.colorRed, context );
		DrawLine( p, MulAdd( p, axisScale, Rot_GetYAxis( transform.q ) ), HexColor.colorGreen, context );
	}

	private void DrawPoint( Vec2 p, float size, HexColor color, object? context )
	{
		DrawList.AddCircleFilled( m_camera.ToScreen( p ), 0.5f * size, ToImColor( color ) );
	}

	private void DrawString( Vec2 p, string s, HexColor color, object? context )
	{
		DrawList.AddText( m_camera.ToScreen( p ), ToImColor( color ), s );
	}

	private void DrawBounds( AABB aabb, HexColor color, object? context )
	{
		Vector2 a = m_camera.ToScreen( aabb.lowerBound );
		Vector2 b = m_camera.ToScreen( aabb.upperBound );
		DrawList.AddRect( new Vector2( a.X, b.Y ), new Vector2( b.X, a.Y ), ToImColor( color ) );
	}
}
