// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using Hexa.NET.ImGui;
using Box2DNet.Shared;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Testbed.DrawFunctions;
using static Box2DNet.Testbed.Glfw;
using static Box2DNet.Testbed.Printf;
using static Box2DNet.Testbed.SampleApp;
using static Box2DNet.Testbed.Helpers_sample_benchmark;
using static Box2DNet.Testbed.Helpers_sample_bodies;
using static Box2DNet.Testbed.Helpers_sample_character;
using static Box2DNet.Testbed.Helpers_sample_collision;
using static Box2DNet.Testbed.Helpers_sample_continuous;
using static Box2DNet.Testbed.Helpers_sample_determinism;
using static Box2DNet.Testbed.Helpers_sample_events;
using static Box2DNet.Testbed.Helpers_sample_issues;
using static Box2DNet.Testbed.Helpers_sample_joints;
using static Box2DNet.Testbed.Helpers_sample_restitution;
using static Box2DNet.Testbed.Helpers_sample_robustness;
using static Box2DNet.Testbed.Helpers_sample_shapes;
using static Box2DNet.Testbed.Helpers_sample_stacking;
using static Box2DNet.Testbed.Helpers_sample_world;
using static Box2DNet.Testbed.Helpers_dynamic_mover;
using static Box2DNet.Testbed.Helpers_geometric_mover;

namespace Box2DNet.Testbed;

public static partial class Helpers_sample_collision
{
	// Tests the Box2D bounding volume hierarchy (BVH). The dynamic tree
	// can be used independently as a spatial data structure.



	public static bool QueryCallback( int proxyId, ulong userData, object? context )
	{
		DynamicTreeSample? sample = (DynamicTreeSample?)( context );
		ref Proxy proxy = ref sample.m_proxies[userData];
		Debug.Assert( proxy.proxyId == proxyId );
		proxy.queryStamp = sample.m_timeStamp;
		return true;
	}

	public static float RayCallback( in RayCastInput input, int proxyId, ulong userData, object? context )
	{
		DynamicTreeSample? sample = (DynamicTreeSample?)( context );
		ref Proxy proxy = ref sample.m_proxies[userData];
		Debug.Assert( proxy.proxyId == proxyId );
		proxy.rayStamp = sample.m_timeStamp;
		return input.maxFraction;
	}




	// This shows how to filter a specific shape using using data.



	// Context for ray cast callbacks. Do what you want with this.



	// This callback finds the closest hit. This is the most common callback used in games.
	public static float RayCastClosestCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastContext? rayContext = (CastContext?)context;

		CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );

		// Ignore a specific shape. Also ignore initial overlap.
		if ( ( userData != null && userData.ignore ) || fraction == 0.0f )
		{
			// By returning -1, we instruct the calling code to ignore this shape and
			// continue the ray-cast to the next shape.
			return -1.0f;
		}

		rayContext.points[0] = point;
		rayContext.normals[0] = normal;
		rayContext.fractions[0] = fraction;
		rayContext.count = 1;

		// By returning the current fraction, we instruct the calling code to clip the ray and
		// continue the ray-cast to the next shape. WARNING: do not assume that shapes
		// are reported in order. However, by clipping, we can always get the closest shape.
		return fraction;
	}

	// This callback finds any hit. For this type of query we are usually just checking for obstruction,
	// so the hit data is not relevant.
	// NOTE: shape hits are not ordered, so this may not return the closest hit
	public static float RayCastAnyCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastContext? rayContext = (CastContext?)context;

		CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );

		// Ignore a specific shape. Also ignore initial overlap.
		if ( ( userData != null && userData.ignore ) || fraction == 0.0f )
		{
			// By returning -1, we instruct the calling code to ignore this shape and
			// continue the ray-cast to the next shape.
			return -1.0f;
		}

		rayContext.points[0] = point;
		rayContext.normals[0] = normal;
		rayContext.fractions[0] = fraction;
		rayContext.count = 1;

		// At this point we have a hit, so we know the ray is obstructed.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0f;
	}

	// This ray cast collects multiple hits along the ray.
	// The shapes are not necessary reported in order, so we might not capture
	// the closest shape.
	// NOTE: shape hits are not ordered, so this may return hits in any order. This means that
	// if you limit the number of results, you may discard the closest hit. You can see this
	// behavior in the sample.
	public static float RayCastMultipleCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastContext? rayContext = (CastContext?)context;

		CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );

		// Ignore a specific shape. Also ignore initial overlap.
		if ( ( userData != null && userData.ignore ) || fraction == 0.0f )
		{
			// By returning -1, we instruct the calling code to ignore this shape and
			// continue the ray-cast to the next shape.
			return -1.0f;
		}

		int count = rayContext.count;
		Debug.Assert( count < 3 );

		rayContext.points[count] = point;
		rayContext.normals[count] = normal;
		rayContext.fractions[count] = fraction;
		rayContext.count = count + 1;

		if ( rayContext.count == 3 )
		{
			// At this point the buffer is full.
			// By returning 0, we instruct the calling code to terminate the ray-cast.
			return 0.0f;
		}

		// By returning 1, we instruct the caller to continue without clipping the ray.
		return 1.0f;
	}

	// This ray cast collects multiple hits along the ray and sorts them.
	public static float RayCastSortedCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastContext? rayContext = (CastContext?)context;

		CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );

		// Ignore a specific shape. Also ignore initial overlap.
		if ( ( userData != null && userData.ignore ) || fraction == 0.0f )
		{
			// By returning -1, we instruct the calling code to ignore this shape and
			// continue the ray-cast to the next shape.
			return -1.0f;
		}

		int count = rayContext.count;
		Debug.Assert( count <= 3 );

		int index = 3;
		while ( fraction < rayContext.fractions[index - 1] )
		{
			index -= 1;

			if ( index == 0 )
			{
				break;
			}
		}

		if ( index == 3 )
		{
			// not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
			// this only happens once the buffer is full
			Debug.Assert( rayContext.count == 3 );
			Debug.Assert( rayContext.fractions[2] <= 1.0f );
			return rayContext.fractions[2];
		}

		for ( int j = 2; j > index; --j )
		{
			rayContext.points[j] = rayContext.points[j - 1];
			rayContext.normals[j] = rayContext.normals[j - 1];
			rayContext.fractions[j] = rayContext.fractions[j - 1];
		}

		rayContext.points[index] = point;
		rayContext.normals[index] = normal;
		rayContext.fractions[index] = fraction;
		rayContext.count = count < 3 ? count + 1 : 3;

		if ( rayContext.count == 3 )
		{
			return rayContext.fractions[2];
		}

		// By returning 1, we instruct the caller to continue without clipping the ray.
		return 1.0f;
	}

	// This sample shows how to use the ray and shape cast functions on a World. This
	// sample is configured to ignore initial overlap.






	// Tests manifolds and contact points
}

public class ShapeDistance : Sample
{
	public const int e_point = 0;
	public const int e_segment = 1;
	public const int e_triangle = 2;
	public const int e_box = 3;

	public ShapeDistance( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 3.0f;
		}

		m_point = Vec2_zero;
		m_segment = new Segment { point1 = new( -0.5f, 0.0f ), point2 = new( 0.5f, 0.0f ) };

		{
			Vec2[] points = { new( -0.5f, 0.0f ), new( 0.5f, 0.0f ), new( 0.0f, 1.0f ) };
			Hull hull = ComputeHull( points, 3 );
			m_triangle = MakePolygon( ref hull, 0.0f );

			// m_triangle = MakeSquare( 0.4f );
		}

		m_box = MakeSquare( 0.5f );

		// m_transform = { new( 1.5f, -1.5f ), Rot_identity };
		m_transform = new Transform { p = new( 0.0f, 0.0f ), q = Rot_identity };
		m_angle = 0.0f;

		m_cache = emptySimplexCache;
		m_simplexCount = 0;
		m_simplexIndex = 0;
		m_startPoint = new Pos { x = 0.0f, y = 0.0f };
		m_basePosition = new Vec2( 0.0f, 0.0f );
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showIndices = false;
		m_useCache = false;
		m_drawSimplex = false;

		m_typeA = e_box;
		m_typeB = e_box;
		m_radiusA = 0.0f;
		m_radiusB = 0.0f;

		m_proxyA = MakeProxy( m_typeA, m_radiusA );
		m_proxyB = MakeProxy( m_typeB, m_radiusB );
	}

	public ShapeProxy MakeProxy( int type, float radius )
	{
		ShapeProxy proxy = default;
		proxy.radius = radius;

		switch ( type )
		{
			case e_point:
				proxy.points[0] = Vec2_zero;
				proxy.count = 1;
				break;

			case e_segment:
				proxy.points[0] = m_segment.point1;
				proxy.points[1] = m_segment.point2;
				proxy.count = 2;
				break;

			case e_triangle:
				for ( int i = 0; i < m_triangle.count; ++i )
				{
					proxy.points[i] = m_triangle.vertices[i];
				}
				proxy.count = m_triangle.count;
				break;

			case e_box:
				proxy.points[0] = m_box.vertices[0];
				proxy.points[1] = m_box.vertices[1];
				proxy.points[2] = m_box.vertices[2];
				proxy.points[3] = m_box.vertices[3];
				proxy.count = 4;
				break;

			default:
				Debug.Assert( false );
				break;
		}

		return proxy;
	}

	public void DrawShape( int type, WorldTransform transform, float radius, HexColor color )
	{
		switch ( type )
		{
			case e_point:
			{
				Pos p = TransformWorldPoint( transform, m_point );
				if ( radius > 0.0f )
				{
					DrawSolidCircle( m_draw, new( p, transform.q ), Vec2_zero, radius, color );
				}
				else
				{
					DrawPoint( m_draw, p, 5.0f, color );
				}
			}
			break;

			case e_segment:
			{
				Pos p1 = TransformWorldPoint( transform, m_segment.point1 );
				Pos p2 = TransformWorldPoint( transform, m_segment.point2 );

				if ( radius > 0.0f )
				{
					DrawCapsule( m_draw, p1, p2, radius, color );
				}
				else
				{
					DrawLine( m_draw, p1, p2, color );
				}
			}
			break;

			case e_triangle:
				DrawSolidPolygon( m_draw, transform, m_triangle.vertices, m_triangle.count, radius, color );
				break;

			case e_box:
				DrawSolidPolygon( m_draw, transform, m_box.vertices, m_box.count, radius, color );
				break;

			default:
				Debug.Assert( false );
				break;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		string[] shapeTypes = { "point", "segment", "triangle", "box" };
		int shapeType = (int)( m_typeA );
		if ( ImGui.Combo( "shape A", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_typeA = (int)( shapeType );
			m_proxyA = MakeProxy( m_typeA, m_radiusA );
		}

		if ( ImGui.SliderFloat( "radius A", ref m_radiusA, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyA.radius = m_radiusA;
		}

		shapeType = (int)( m_typeB );
		if ( ImGui.Combo( "shape B", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_typeB = (int)( shapeType );
			m_proxyB = MakeProxy( m_typeB, m_radiusB );
		}

		if ( ImGui.SliderFloat( "radius B", ref m_radiusB, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyB.radius = m_radiusB;
		}

		ImGui.Separator();

		ImGui.SliderFloat( "x offset", ref m_transform.p.x, -2.0f, 2.0f, "%.2f" );
		ImGui.SliderFloat( "y offset", ref m_transform.p.y, -2.0f, 2.0f, "%.2f" );

		if ( ImGui.SliderFloat( "angle", ref m_angle, -Pi, Pi, "%.2f" ) )
		{
			m_transform.q = MakeRot( m_angle );
		}

		ImGui.PopItemWidth();

		ImGui.Separator();

		ImGui.Checkbox( "show indices", ref m_showIndices );
		ImGui.Checkbox( "use cache", ref m_useCache );

		ImGui.Separator();

		if ( ImGui.Checkbox( "draw simplex", ref m_drawSimplex ) )
		{
			m_simplexIndex = 0;
		}

		if ( m_drawSimplex && m_simplexCount > 0 )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			ImGui.SliderInt( "index", ref m_simplexIndex, 0, m_simplexCount - 1 );
			m_simplexIndex = ClampInt( m_simplexIndex, 0, m_simplexCount - 1 );
			ImGui.PopItemWidth();
		}

		return true;
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_dragging = true;
				m_startPoint = position;
				m_basePosition = m_transform.p;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_startPoint = position;
				m_baseAngle = m_angle;
			}
		}
	}

	public override void MouseUp( Pos unused9, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		Vec2 d = position - m_startPoint;

		if ( m_dragging )
		{
			m_transform.p = m_basePosition + 0.5f * d;
		}
		else if ( m_rotating )
		{
			m_angle = ClampFloat( m_baseAngle + 1.0f * d.x, -Pi, Pi );
			m_transform.q = MakeRot( m_angle );
		}
	}

	public static Vec2 Weight2( float a1, Vec2 w1, float a2, Vec2 w2 )
	{
		return new( a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y );
	}

	public static Vec2 Weight3( float a1, Vec2 w1, float a2, Vec2 w2, float a3, Vec2 w3 )
	{
		return new( a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y );
	}

	public void ComputeSimplexWitnessPoints( ref Vec2 a, ref Vec2 b, in Simplex s )
	{
		switch ( s.count )
		{
			case 0:
				Debug.Assert( false );
				break;

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
				b = a;
				break;

			default:
				Debug.Assert( false );
				break;
		}
	}

	public override void Step(  )
	{
		DistanceInput input = default;
		input.proxyA = m_proxyA;
		input.proxyB = m_proxyB;
		input.transform = m_transform;
		input.useRadii = m_radiusA > 0.0f || m_radiusB > 0.0f;

		if ( m_useCache == false )
		{
			m_cache.count = 0;
		}

		DistanceOutput output = ComputeShapeDistance( input, ref m_cache, m_simplexes );

		m_simplexCount = output.simplexCount;

		// Shape A is at the world origin, so its frame-A distance output doubles as world points
		DrawShape( m_typeA, WorldTransform_identity, m_radiusA, HexColor.colorCyan );
		DrawShape( m_typeB, MakeWorldTransform( m_transform ), m_radiusB, HexColor.colorBisque );

		if ( m_drawSimplex )
		{
			ref Simplex simplex = ref m_simplexes[m_simplexIndex];
			SimplexVertex[] vertices = { simplex.v1, simplex.v2, simplex.v3 };

			if ( m_simplexIndex > 0 )
			{
				// The first recorded simplex does not have valid barycentric coordinates
				Vec2 pointA = default, pointB = default;
				ComputeSimplexWitnessPoints( ref pointA, ref pointB, simplex );

				DrawLine( m_draw, ToPos( pointA ), ToPos( pointB ), HexColor.colorWhite );
				DrawPoint( m_draw, ToPos( pointA ), 10.0f, HexColor.colorWhite );
				DrawPoint( m_draw, ToPos( pointB ), 10.0f, HexColor.colorWhite );
			}

			HexColor[] colors = { HexColor.colorRed, HexColor.colorGreen, HexColor.colorBlue };

			for ( int i = 0; i < simplex.count; ++i )
			{
				ref SimplexVertex vertex = ref vertices[i];
				DrawPoint( m_draw, ToPos( vertex.wA ), 10.0f, colors[i] );
				DrawPoint( m_draw, ToPos( vertex.wB ), 10.0f, colors[i] );
			}
		}
		else
		{
			DrawLine( m_draw, ToPos( output.pointA ), ToPos( output.pointB ), HexColor.colorDimGray );
			DrawPoint( m_draw, ToPos( output.pointA ), 10.0f, HexColor.colorWhite );
			DrawPoint( m_draw, ToPos( output.pointB ), 10.0f, HexColor.colorWhite );

			DrawLine( m_draw, ToPos( output.pointA ), ToPos( output.pointA + 0.5f * output.normal ), HexColor.colorYellow );
		}

		if ( m_showIndices )
		{
			// Shape A sits at the world origin, shape B at the offset transform
			for ( int i = 0; i < m_proxyA.count; ++i )
			{
				DrawString( m_draw, m_camera, ToPos( m_proxyA.points[i] ), HexColor.colorWhite, " %d", i );
			}

			WorldTransform transformB = MakeWorldTransform( m_transform );
			for ( int i = 0; i < m_proxyB.count; ++i )
			{
				Pos p = TransformWorldPoint( transformB, m_proxyB.points[i] );
				DrawString( m_draw, m_camera, p, HexColor.colorWhite, " %d", i );
			}
		}

		DrawScreenTextLine( "mouse button 1: drag" );
		DrawScreenTextLine( "mouse button 1 + shift: rotate" );
		DrawScreenTextLine( "distance = %.2f, iterations = %d", output.distance, output.iterations );

		if ( m_cache.count == 1 )
		{
			DrawScreenTextLine( "cache = new( %d ), {%d}", m_cache.indexA[0], m_cache.indexB[0] );
		}
		else if ( m_cache.count == 2 )
		{
			DrawScreenTextLine( "cache = new( %d, %d ), {%d, %d}", m_cache.indexA[0], m_cache.indexA[1], m_cache.indexB[0],
								m_cache.indexB[1] );
		}
		else if ( m_cache.count == 3 )
		{
			DrawScreenTextLine( "cache = new( %d, %d, %d ), {%d, %d, %d}", m_cache.indexA[0], m_cache.indexA[1], m_cache.indexA[2],
								m_cache.indexB[0], m_cache.indexB[1], m_cache.indexB[2] );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new ShapeDistance( context );
	}

	public const int m_simplexCapacity = 20;

	public Polygon m_box;
	public Polygon m_triangle;
	public Vec2 m_point;
	public Segment m_segment;

	public int m_typeA;
	public int m_typeB;
	public float m_radiusA;
	public float m_radiusB;
	public ShapeProxy m_proxyA;
	public ShapeProxy m_proxyB;

	public SimplexCache m_cache;
	public Simplex[] m_simplexes = new Simplex[m_simplexCapacity];
	public int m_simplexCount;
	public int m_simplexIndex;

	public Transform m_transform;
	public float m_angle;

	public Vec2 m_basePosition;
	public Pos m_startPoint;
	public float m_baseAngle;

	public bool m_dragging;
	public bool m_rotating;
	public bool m_showIndices;
	public bool m_useCache;
	public bool m_drawSimplex;
}

public struct Proxy
{
	public AABB box;
	public AABB fatBox;
	public Vec2 position;
	public Vec2 width;
	public int proxyId;
	public int rayStamp;
	public int queryStamp;
	public bool moved;
}

public class DynamicTreeSample : Sample
{
	public DynamicTreeSample( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 500.0f, 500.0f );
			m_context.camera.zoom = 25.0f * 21.0f;
		}

		m_fill = 0.25f;
		m_moveFraction = 0.05f;
		m_moveDelta = 0.1f;
		m_proxies = null;
		m_proxyCount = 0;
		m_proxyCapacity = 0;
		m_ratio = 5.0f;
		m_grid = 1.0f;

		m_moveBuffer = null;
		m_moveCount = 0;

		m_rowCount = m_isDebug ? 100 : 1000;
		m_columnCount = m_isDebug ? 100 : 1000;
		m_tree = null;
		BuildTree();
		m_timeStamp = 0;

		m_startPoint = new Pos { x = 0.0f, y = 0.0f };
		m_endPoint = new Pos { x = 0.0f, y = 0.0f };
		m_queryDrag = false;
		m_rayDrag = false;
		m_validate = true;
	}

	public override void Dispose()
	{
		DestroyDynamicTree( m_tree );
		base.Dispose();
	}

	public void BuildTree(  )
	{
		DestroyDynamicTree( m_tree );
		m_proxyCapacity = m_rowCount * m_columnCount;
		m_proxies = new Proxy[m_proxyCapacity];
		m_proxyCount = 0;

		m_moveBuffer = new int[m_proxyCapacity];
		m_moveCount = 0;

		float y = -4.0f;

		m_tree = CreateDynamicTree( 16 );

		Vec2 aabbMargin = new Vec2( 0.1f, 0.1f );

		for ( int i = 0; i < m_rowCount; ++i )
		{
			float x = -40.0f;

			for ( int j = 0; j < m_columnCount; ++j )
			{
				float fillTest = RandomFloatRange( 0.0f, 1.0f );
				if ( fillTest <= m_fill )
				{
					Debug.Assert( m_proxyCount <= m_proxyCapacity );
					ref Proxy p = ref m_proxies[m_proxyCount];
					p.position = new( x, y );

					float ratio = RandomFloatRange( 1.0f, m_ratio );
					float width = RandomFloatRange( 0.1f, 0.5f );
					if ( RandomFloat() > 0.0f )
					{
						p.width.x = ratio * width;
						p.width.y = width;
					}
					else
					{
						p.width.x = width;
						p.width.y = ratio * width;
					}

					p.box.lowerBound = new( x, y );
					p.box.upperBound = new( x + p.width.x, y + p.width.y );
					p.fatBox.lowerBound = Sub( p.box.lowerBound, aabbMargin );
					p.fatBox.upperBound = Add( p.box.upperBound, aabbMargin );

					p.proxyId = CreateTreeProxy( m_tree, p.fatBox, DefaultCategoryBits, (ulong)m_proxyCount );
					p.rayStamp = -1;
					p.queryStamp = -1;
					p.moved = false;
					++m_proxyCount;
				}

				x += m_grid;
			}

			y += m_grid;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		if ( ImGui.SliderInt( "rows", ref m_rowCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderInt( "columns", ref m_columnCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "fill", ref m_fill, 0.0f, 1.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "grid", ref m_grid, 0.5f, 2.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "ratio", ref m_ratio, 1.0f, 10.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "move", ref m_moveFraction, 0.0f, 1.0f, "%.2f" ) )
		{
		}

		if ( ImGui.SliderFloat( "delta", ref m_moveDelta, 0.0f, 1.0f, "%.2f" ) )
		{
		}

		ImGui.PopItemWidth();

		ImGui.Separator();

		ImGui.Text( "mouse button 1: ray cast" );
		ImGui.Text( "mouse button 1 + shift: query" );

		if ( changed )
		{
			BuildTree();
		}

		return true;
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_queryDrag == false )
			{
				m_rayDrag = true;
				m_startPoint = position;
				m_endPoint = position;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_rayDrag == false )
			{
				m_queryDrag = true;
				m_startPoint = position;
				m_endPoint = position;
			}
		}
	}

	public override void MouseUp( Pos unused10, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_queryDrag = false;
			m_rayDrag = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		m_endPoint = position;
	}

	public override void Step(  )
	{
		if ( m_queryDrag )
		{
			// The dynamic tree is a float spatial structure, so narrow the world points to enter it
			Vec2 p1 = ToVec2( m_startPoint );
			Vec2 p2 = ToVec2( m_endPoint );
			AABB box = new AABB { lowerBound = Min( p1, p2 ), upperBound = Max( p1, p2 ) };
			DynamicTree_OverlapAABB( m_tree, box, DefaultMaskBits, QueryCallback, this );

			DrawBounds( m_draw, box, HexColor.colorWhite );
		}

		if ( m_rayDrag )
		{
			RayCastInput input = new RayCastInput { origin = ToVec2( m_startPoint ), translation = m_endPoint - m_startPoint, maxFraction = 1.0f };
			TreeStats result = DynamicTree_CastRay( m_tree, ref input, DefaultMaskBits, RayCallback, this );

			DrawLine( m_draw, m_startPoint, m_endPoint, HexColor.colorWhite );
			DrawPoint( m_draw, m_startPoint, 5.0f, HexColor.colorGreen );
			DrawPoint( m_draw, m_endPoint, 5.0f, HexColor.colorRed );

			DrawScreenTextLine( "node visits = %d, leaf visits = %d", result.nodeVisits, result.leafVisits );
		}

		HexColor c = HexColor.colorBlue;
		HexColor qc = HexColor.colorGreen;

		Vec2 aabbMargin = new Vec2( 0.1f, 0.1f );

		for ( int i = 0; i < m_proxyCount; ++i )
		{
			ref Proxy p = ref m_proxies[i];
			if ( p.queryStamp == m_timeStamp || p.rayStamp == m_timeStamp )
			{
				DrawBounds( m_draw, p.box, qc );
			}
			else
			{
				DrawBounds( m_draw, p.box, c );
			}

			float moveTest = RandomFloatRange( 0.0f, 1.0f );
			if ( m_moveFraction > moveTest )
			{
				float dx = m_moveDelta * RandomFloat();
				float dy = m_moveDelta * RandomFloat();

				p.position.x += dx;
				p.position.y += dy;

				p.box.lowerBound.x = p.position.x + dx;
				p.box.lowerBound.y = p.position.y + dy;
				p.box.upperBound.x = p.position.x + dx + p.width.x;
				p.box.upperBound.y = p.position.y + dy + p.width.y;

				if ( AABB_Contains( p.fatBox, p.box ) == false )
				{
					p.fatBox.lowerBound = Sub( p.box.lowerBound, aabbMargin );
					p.fatBox.upperBound = Add( p.box.upperBound, aabbMargin );
					p.moved = true;
				}
				else
				{
					p.moved = false;
				}
			}
			else
			{
				p.moved = false;
			}
		}

		{
			ulong ticks = GetTicks();
			for ( int i = 0; i < m_proxyCount; ++i )
			{
				ref Proxy p = ref m_proxies[i];
				if ( p.moved )
				{
					DynamicTree_MoveProxy( m_tree, p.proxyId, p.fatBox );
				}
			}
			float ms = GetMilliseconds( ticks );
			DrawScreenTextLine( "incremental : %.3f ms", ms );
		}

		int height = DynamicTree_GetHeight( m_tree );
		float areaRatio = DynamicTree_GetAreaRatio( m_tree );

		int hmin = (int)( MathF.Ceiling( MathF.Log( (float)m_proxyCount ) / MathF.Log( 2.0f ) - 1.0f ) );
		DrawScreenTextLine( "proxies = %d, height = %d, hmin = %d, area ratio = %.1f", m_proxyCount, height, hmin, areaRatio );

		DynamicTree_Validate( m_tree );

		m_timeStamp += 1;
	}

	public static Sample Create( SampleContext context )
	{
		return new DynamicTreeSample( context );
	}

	public DynamicTree m_tree;
	public int m_rowCount, m_columnCount;
	public Proxy[] m_proxies;
	public int[] m_moveBuffer;
	public int m_moveCount;
	public int m_proxyCapacity;
	public int m_proxyCount;
	public int m_timeStamp;
	public float m_fill;
	public float m_moveFraction;
	public float m_moveDelta;
	public float m_ratio;
	public float m_grid;

	public Pos m_startPoint;
	public Pos m_endPoint;

	public bool m_rayDrag;
	public bool m_queryDrag;
	public bool m_validate;
}

public class RayCast : Sample
{
	public RayCast( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 20.0f );
			m_context.camera.zoom = 17.5f;
		}

		m_circle = new Circle { center = new( 0.0f, 0.0f ), radius = 2.0f };
		m_capsule = new Capsule { center1 = new( -1.0f, 1.0f ), center2 = new( 1.0f, -1.0f ), radius = 1.5f };
		m_box = MakeBox( 2.0f, 2.0f );

		Vec2[] vertices = { new( -2.0f, 0.0f ), new( 2.0f, 0.0f ), new( 2.0f, 3.0f ) };
		Hull hull = ComputeHull( vertices, 3 );
		m_triangle = MakePolygon( ref hull, 0.0f );

		m_segment = new Segment { point1 = new( -3.0f, 0.0f ), point2 = new( 3.0f, 0.0f ) };

		m_transform = Transform_identity;
		m_angle = 0.0f;

		m_basePosition = new Vec2( 0.0f, 0.0f );
		m_baseAngle = 0.0f;
		m_startPosition = new Pos { x = 0.0f, y = 0.0f };

		m_rayStart = new Pos { x = 0.0f, y = 30.0f };
		m_rayEnd = new Pos { x = 0.0f, y = 0.0f };

		m_rayDrag = false;
		m_translating = false;
		m_rotating = false;

		m_showFraction = false;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		ImGui.SliderFloat( "x offset", ref m_transform.p.x, -2.0f, 2.0f, "%.2f" );
		ImGui.SliderFloat( "y offset", ref m_transform.p.y, -2.0f, 2.0f, "%.2f" );

		if ( ImGui.SliderFloat( "angle", ref m_angle, -Pi, Pi, "%.2f" ) )
		{
			m_transform.q = MakeRot( m_angle );
		}

		// if (ImGui.SliderFloat("ray radius", ref m_rayRadius, 0.0f, 1.0f, "%.1f"))
		//{
		// }

		ImGui.PopItemWidth();

		ImGui.Checkbox( "show fraction", ref m_showFraction );

		if ( ImGui.Button( "Reset" ) )
		{
			m_transform = Transform_identity;
			m_angle = 0.0f;
		}

		ImGui.Separator();

		ImGui.Text( "mouse btn 1: ray cast" );
		ImGui.Text( "mouse btn 1 + shft: translate" );
		ImGui.Text( "mouse btn 1 + ctrl: rotate" );

		return true;
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_startPosition = position;

			if ( mods == 0 )
			{
				m_rayStart = position;
				m_rayDrag = true;
			}
			else if ( mods == GLFW_MOD_SHIFT )
			{
				m_translating = true;
				m_basePosition = m_transform.p;
			}
			else if ( mods == GLFW_MOD_CONTROL )
			{
				m_rotating = true;
				m_baseAngle = m_angle;
			}
		}
	}

	public override void MouseUp( Pos unused11, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_rayDrag = false;
			m_rotating = false;
			m_translating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		Vec2 d = position - m_startPosition;

		if ( m_rayDrag )
		{
			m_rayEnd = position;
		}
		else if ( m_translating )
		{
			m_transform.p.x = m_basePosition.x + 0.5f * d.x;
			m_transform.p.y = m_basePosition.y + 0.5f * d.y;
		}
		else if ( m_rotating )
		{
			m_angle = ClampFloat( m_baseAngle + 0.5f * d.x, -Pi, Pi );
			m_transform.q = MakeRot( m_angle );
		}
	}

	public void DrawRay( in CastOutput output, Pos hitPoint )
	{
		Pos p1 = m_rayStart;
		Pos p2 = m_rayEnd;
		Vec2 d = p2 - p1;

		if ( output.hit )
		{
			Pos p = default;
			if ( output.fraction == 0.0f )
			{
				Debug.Assert( output.normal.x == 0.0f && output.normal.y == 0.0f );
				p = hitPoint;
				DrawPoint( m_draw, hitPoint, 5.0f, HexColor.colorPeru );
			}
			else
			{
				p = p1 + output.fraction * d;
				DrawLine( m_draw, p1, p, HexColor.colorWhite );
				DrawPoint( m_draw, p1, 5.0f, HexColor.colorGreen );
				DrawPoint( m_draw, hitPoint, 5.0f, HexColor.colorWhite );

				DrawLine( m_draw, p, p + output.normal, HexColor.colorViolet );
			}

			if ( m_showFraction )
			{
				DrawString( m_draw, m_camera, OffsetPos( p, new( 0.05f, -0.02f ) ), HexColor.colorWhite, "%.2f", output.fraction );
			}
		}
		else
		{
			DrawLine( m_draw, p1, p2, HexColor.colorWhite );
			DrawPoint( m_draw, p1, 5.0f, HexColor.colorGreen );
			DrawPoint( m_draw, p2, 5.0f, HexColor.colorRed );
		}
	}

	public override void Step(  )
	{
		Vec2 offset = new Vec2( -20.0f, 20.0f );
		Vec2 increment = new Vec2( 10.0f, 0.0f );

		HexColor color1 = HexColor.colorYellow;

		CastOutput output = default;
		Pos hitPoint = Pos_zero;
		float maxFraction = 1.0f;

		// circle
		{
			WorldTransform transform = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			DrawSolidCircle( m_draw, transform, m_circle.center, m_circle.radius, color1 );

			Vec2 start = InvTransformWorldPoint( transform, m_rayStart );
			Vec2 translation = InvRotateVector( transform.q, m_rayEnd - m_rayStart );
			RayCastInput input = new RayCastInput { origin = start, translation = translation, maxFraction = maxFraction };

			CastOutput localOutput = CastRayCircle( ref m_circle, ref input );
			if ( localOutput.hit )
			{
				output = localOutput;
				hitPoint = TransformWorldPoint( transform, localOutput.point );
				output.normal = RotateVector( transform.q, localOutput.normal );
				maxFraction = localOutput.fraction;
			}

			offset = Add( offset, increment );
		}

		// capsule
		{
			WorldTransform transform = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			Pos v1 = TransformWorldPoint( transform, m_capsule.center1 );
			Pos v2 = TransformWorldPoint( transform, m_capsule.center2 );
			DrawCapsule( m_draw, v1, v2, m_capsule.radius, color1 );

			Vec2 start = InvTransformWorldPoint( transform, m_rayStart );
			Vec2 translation = InvRotateVector( transform.q, m_rayEnd - m_rayStart );
			RayCastInput input = new RayCastInput { origin = start, translation = translation, maxFraction = maxFraction };

			CastOutput localOutput = CastRayCapsule( ref m_capsule, ref input );
			if ( localOutput.hit )
			{
				output = localOutput;
				hitPoint = TransformWorldPoint( transform, localOutput.point );
				output.normal = RotateVector( transform.q, localOutput.normal );
				maxFraction = localOutput.fraction;
			}

			offset = Add( offset, increment );
		}

		// box
		{
			WorldTransform transform = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			DrawSolidPolygon( m_draw, transform, m_box.vertices, m_box.count, 0.0f, color1 );

			Vec2 start = InvTransformWorldPoint( transform, m_rayStart );
			Vec2 translation = InvRotateVector( transform.q, m_rayEnd - m_rayStart );
			RayCastInput input = new RayCastInput { origin = start, translation = translation, maxFraction = maxFraction };

			CastOutput localOutput = CastRayPolygon( ref m_box, ref input );
			if ( localOutput.hit )
			{
				output = localOutput;
				hitPoint = TransformWorldPoint( transform, localOutput.point );
				output.normal = RotateVector( transform.q, localOutput.normal );
				maxFraction = localOutput.fraction;
			}

			offset = Add( offset, increment );
		}

		// triangle
		{
			WorldTransform transform = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			DrawSolidPolygon( m_draw, transform, m_triangle.vertices, m_triangle.count, 0.0f, color1 );

			Vec2 start = InvTransformWorldPoint( transform, m_rayStart );
			Vec2 translation = InvRotateVector( transform.q, m_rayEnd - m_rayStart );
			RayCastInput input = new RayCastInput { origin = start, translation = translation, maxFraction = maxFraction };

			CastOutput localOutput = CastRayPolygon( ref m_triangle, ref input );
			if ( localOutput.hit )
			{
				output = localOutput;
				hitPoint = TransformWorldPoint( transform, localOutput.point );
				output.normal = RotateVector( transform.q, localOutput.normal );
				maxFraction = localOutput.fraction;
			}

			offset = Add( offset, increment );
		}

		// segment
		{
			WorldTransform transform = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			Pos p1 = TransformWorldPoint( transform, m_segment.point1 );
			Pos p2 = TransformWorldPoint( transform, m_segment.point2 );
			DrawLine( m_draw, p1, p2, color1 );

			Vec2 start = InvTransformWorldPoint( transform, m_rayStart );
			Vec2 translation = InvRotateVector( transform.q, m_rayEnd - m_rayStart );
			RayCastInput input = new RayCastInput { origin = start, translation = translation, maxFraction = maxFraction };

			CastOutput localOutput = CastRaySegment( ref m_segment, ref input, false );
			if ( localOutput.hit )
			{
				output = localOutput;
				hitPoint = TransformWorldPoint( transform, localOutput.point );
				output.normal = RotateVector( transform.q, localOutput.normal );
				maxFraction = localOutput.fraction;
			}

			offset = Add( offset, increment );
		}

		DrawRay( ref output, hitPoint );
	}

	public static Sample Create( SampleContext context )
	{
		return new RayCast( context );
	}

	public Polygon m_box;
	public Polygon m_triangle;
	public Circle m_circle;
	public Capsule m_capsule;
	public Segment m_segment;

	public Transform m_transform;
	public float m_angle;

	public Pos m_rayStart;
	public Pos m_rayEnd;

	public Vec2 m_basePosition;
	public float m_baseAngle;

	public Pos m_startPosition;

	public bool m_rayDrag;
	public bool m_translating;
	public bool m_rotating;
	public bool m_showFraction;
}

public class CollisionShapeUserData
{
	public int index;
	public bool ignore;
}

public class CastContext
{
	public Pos[] points = new Pos[3];
	public Vec2[] normals = new Vec2[3];
	public float[] fractions = new float[3];
	public int count;
}

public class CastWorld : Sample
{
	public const int e_any = 0;
	public const int e_closest = 1;
	public const int e_multiple = 2;
	public const int e_sorted = 3;

	public const int e_rayCast = 0;
	public const int e_circleCast = 1;
	public const int e_capsuleCast = 2;
	public const int e_polygonCast = 3;

	public const int e_maxCount = 64;

	public CastWorld( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.0f, 14.0f );
			m_context.camera.zoom = 25.0f * 0.75f;
		}

		// Ground body
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Vec2[] vertices = { new( -0.1f, 0.0f ), new( 0.1f, 0.0f ), new( 0.0f, 1.5f ) };
			Hull hull = ComputeHull( vertices, 3 );
			m_polygons[0] = MakePolygon( ref hull, 0.0f );
			m_polygons[0].radius = 0.5f;
		}

		{
			float w = 1.0f;
			float b = w / ( 2.0f + MathF.Sqrt( 2.0f ) );
			float s = MathF.Sqrt( 2.0f ) * b;

			Vec2[] vertices = { new( 0.5f * s, 0.0f ), new( 0.5f * w, b ), new( 0.5f * w, b + s ), new( 0.5f * s, w ), new( -0.5f * s, w ), new( -0.5f * w, b + s ), new( -0.5f * w, b ),	  new( -0.5f * s, 0.0f ) };

			Hull hull = ComputeHull( vertices, 8 );
			m_polygons[1] = MakePolygon( ref hull, 0.0f );
		}

		m_box = MakeBox( 0.5f, 0.5f );
		m_capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
		m_circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		m_segment = new Segment { point1 = new( -1.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) };

		m_bodyIndex = 0;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodyIds[i] = nullBodyId;
		}

		m_mode = e_closest;
		m_ignoreIndex = 7;

		m_castType = e_rayCast;
		m_castRadius = 0.5f;

		m_rayStart = new Pos { x = -20.0f, y = 10.0f };
		m_rayEnd = new Pos { x = 20.0f, y = 10.0f };
		m_dragging = false;

		m_angle = 0.0f;
		m_baseAngle = 0.0f;
		m_angleAnchor = new Pos { x = 0.0f, y = 0.0f };
		m_rotating = false;

		m_simple = false;
	}

	public void Create( int index )
	{
		if ( m_bodyIds[m_bodyIndex].IsNonNull )
		{
			B2.DestroyBody( m_bodyIds[m_bodyIndex] );
			m_bodyIds[m_bodyIndex] = nullBodyId;
		}

		float x = RandomFloatRange( -20.0f, 20.0f );
		float y = RandomFloatRange( 0.0f, 20.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = x, y = y };
		bodyDef.rotation = MakeRot( RandomFloatRange( -Pi, Pi ) );

		int mod = m_bodyIndex % 3;
		if ( mod == 0 )
		{
			bodyDef.type = BodyType.staticBody;
		}
		else if ( mod == 1 )
		{
			bodyDef.type = BodyType.kinematicBody;
		}
		else if ( mod == 2 )
		{
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.gravityScale = 0.0f;
		}

		m_bodyIds[m_bodyIndex] = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.userData = m_userData[m_bodyIndex];
		m_userData[m_bodyIndex].ignore = false;
		if ( m_bodyIndex == m_ignoreIndex )
		{
			m_userData[m_bodyIndex].ignore = true;
		}

		if ( index == 0 )
		{
			int polygonIndex = ( m_bodyIndex & 1 );
			CreatePolygonShape( m_bodyIds[m_bodyIndex], ref shapeDef, m_polygons[polygonIndex] );
		}
		else if ( index == 1 )
		{
			CreatePolygonShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_box );
		}
		else if ( index == 2 )
		{
			CreateCircleShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_circle );
		}
		else if ( index == 3 )
		{
			CreateCapsuleShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_capsule );
		}
		else if ( index == 4 )
		{
			CreateSegmentShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_segment );
		}
		else
		{
			Vec2[] points = { new( 1.0f, 0.0f ), new( -1.0f, 0.0f ), new( -1.0f, -1.0f ), new( 1.0f, -1.0f ) };
			ChainDef chainDef = DefaultChainDef();
			chainDef.userData = shapeDef.userData;
			chainDef.points = points;
			chainDef.pointCount = 4;
			chainDef.isLoop = true;
			CreateChain( m_bodyIds[m_bodyIndex], ref chainDef );
		}

		m_bodyIndex = ( m_bodyIndex + 1 ) % e_maxCount;
	}

	public void CreateN( int index, int count )
	{
		for ( int i = 0; i < count; ++i )
		{
			Create( index );
		}
	}

	public void DestroyBody(  )
	{
		for ( int i = 0; i < e_maxCount; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				B2.DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
				return;
			}
		}
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_rayStart = position;
				m_rayEnd = position;
				m_dragging = true;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_angleAnchor = position;
				m_baseAngle = m_angle;
			}
		}
	}

	public override void MouseUp( Pos unused12, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos p )
	{
		if ( m_dragging )
		{
			m_rayEnd = p;
		}
		else if ( m_rotating )
		{
			float dx = ( p - m_angleAnchor ).x;
			m_angle = m_baseAngle + 1.0f * dx;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.Checkbox( "Simple", ref m_simple );

		if ( m_simple == false )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			string[] castTypes = { "Ray", "Circle", "Capsule", "Polygon" };
			int castType = (int)( m_castType );
			if ( ImGui.Combo( "Type", ref castType, castTypes, castTypes.Length ) )
			{
				m_castType = castType;
			}

			if ( m_castType != e_rayCast )
			{
				ImGui.SliderFloat( "Radius", ref m_castRadius, 0.0f, 2.0f, "%.1f" );
			}

			string[] modes = { "Any", "Closest", "Multiple", "Sorted" };
			int mode = (int)( m_mode );
			if ( ImGui.Combo( "Mode", ref mode, modes, modes.Length ) )
			{
				m_mode = mode;
			}
			ImGui.PopItemWidth();
		}

		if ( ImGui.Button( "Polygon" ) )
			Create( 0 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Poly" ) )
			CreateN( 0, 10 );

		if ( ImGui.Button( "Box" ) )
			Create( 1 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Box" ) )
			CreateN( 1, 10 );

		if ( ImGui.Button( "Circle" ) )
			Create( 2 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Circle" ) )
			CreateN( 2, 10 );

		if ( ImGui.Button( "Capsule" ) )
			Create( 3 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Capsule" ) )
			CreateN( 3, 10 );

		if ( ImGui.Button( "Segment" ) )
			Create( 4 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Segment" ) )
			CreateN( 4, 10 );

		if ( ImGui.Button( "Chain" ) )
			Create( 5 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Chain" ) )
			CreateN( 5, 10 );

		if ( ImGui.Button( "Destroy Shape" ) )
		{
			DestroyBody();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		DrawScreenTextLine( "Click left mouse button and drag to modify ray cast" );
		DrawScreenTextLine( "Shape 7 is intentionally ignored by the ray" );

		HexColor color1 = HexColor.colorGreen;
		HexColor color2 = HexColor.colorLightGray;
		HexColor color3 = HexColor.colorMagenta;

		Vec2 rayTranslation = SubPos( m_rayEnd, m_rayStart );

		if ( m_simple )
		{
			DrawScreenTextLine( "Simple closest point ray cast" );

			// This version doesn't have a callback, but it doesn't skip the ignored shape
			RayResult result = World_CastRayClosest( m_worldId, m_rayStart, rayTranslation, DefaultQueryFilter() );

			if ( result.hit == true && result.fraction > 0.0f )
			{
				Pos c = m_rayStart + result.fraction * rayTranslation;
				DrawPoint( m_draw, result.point, 5.0f, color1 );
				DrawLine( m_draw, m_rayStart, c, color2 );
				Pos head = result.point + 0.5f * result.normal;
				DrawLine( m_draw, result.point, head, color3 );
			}
			else
			{
				DrawLine( m_draw, m_rayStart, m_rayEnd, color2 );
			}
		}
		else
		{
			switch ( m_mode )
			{
				case e_any:
					DrawScreenTextLine( "Cast mode: any - check for obstruction - unsorted" );
					break;

				case e_closest:
					DrawScreenTextLine( "Cast mode: closest - find closest shape along the cast" );
					break;

				case e_multiple:
					DrawScreenTextLine( "Cast mode: multiple - gather up to 3 shapes - unsorted" );
					break;

				case e_sorted:
					DrawScreenTextLine( "Cast mode: sorted - gather up to 3 shapes sorted by closeness" );
					break;

				default:
					Debug.Assert( false );
					break;
			}

			CastResultCallbackFcn[] functions = {
				RayCastAnyCallback,
				RayCastClosestCallback,
				RayCastMultipleCallback,
				RayCastSortedCallback,
			};
			ref CastResultCallbackFcn modeFcn = ref functions[m_mode];
			CastContext context = new();

			// Must initialize fractions for sorting
			context.fractions[0] = float.MaxValue;
			context.fractions[1] = float.MaxValue;
			context.fractions[2] = float.MaxValue;

			Rot rotation = MakeRot( m_angle );
			Circle circle = new Circle { center = Vec2_zero, radius = m_castRadius };
			Capsule capsule = new Capsule { center1 = RotateVector( rotation, new( -0.25f, 0.0f ) ), center2 = RotateVector( rotation, new( 0.25f, 0.0f ) ), radius = m_castRadius };
			Polygon box = MakeOffsetRoundedBox( 0.125f, 0.25f, Vec2_zero, rotation, m_castRadius );
			ShapeProxy proxy = default;

			if ( m_castType == e_rayCast )
			{
				World_CastRay( m_worldId, m_rayStart, rayTranslation, DefaultQueryFilter(), modeFcn, context );
			}
			else
			{
				if ( m_castType == e_circleCast )
				{
					proxy = B2.MakeProxy( ref circle.center, 1, circle.radius );
				}
				else if ( m_castType == e_capsuleCast )
				{
					proxy = B2.MakeProxy( ref capsule.center1, 2, capsule.radius );
				}
				else
				{
					proxy = B2.MakeProxy( box.vertices, box.count, box.radius );
				}

				World_CastShape( m_worldId, m_rayStart, ref proxy, rayTranslation, DefaultQueryFilter(), modeFcn, context );
			}

			if ( context.count > 0 )
			{
				Debug.Assert( context.count <= 3 );
				HexColor[] colors = { HexColor.colorRed, HexColor.colorGreen, HexColor.colorBlue };
				for ( int i = 0; i < context.count; ++i )
				{
					Pos c = m_rayStart + context.fractions[i] * rayTranslation;
					Pos p = context.points[i];
					Vec2 n = context.normals[i];
					DrawPoint( m_draw, p, 5.0f, colors[i] );
					DrawLine( m_draw, m_rayStart, c, color2 );
					Pos head = p + 1.0f * n;
					DrawLine( m_draw, p, head, color3 );

					// The swept shape rests at the contact position c
					if ( m_castType == e_circleCast )
					{
						DrawSolidCircle( m_draw, new( c, Rot_identity ), Vec2_zero, m_castRadius, HexColor.colorYellow );
					}
					else if ( m_castType == e_capsuleCast )
					{
						DrawCapsule( m_draw, OffsetPos( c, capsule.center1 ), OffsetPos( c, capsule.center2 ), m_castRadius,
									 HexColor.colorYellow );
					}
					else if ( m_castType == e_polygonCast )
					{
						DrawSolidPolygon( m_draw, new( c, Rot_identity ), box.vertices, box.count, box.radius, HexColor.colorYellow );
					}
				}
			}
			else
			{
				DrawLine( m_draw, m_rayStart, m_rayEnd, color2 );

				// No hit, so the swept shape rests at the end of the ray
				if ( m_castType == e_circleCast )
				{
					DrawSolidCircle( m_draw, new( m_rayEnd, Rot_identity ), Vec2_zero, m_castRadius, HexColor.colorGray );
				}
				else if ( m_castType == e_capsuleCast )
				{
					DrawCapsule( m_draw, OffsetPos( m_rayEnd, capsule.center1 ), OffsetPos( m_rayEnd, capsule.center2 ),
								 m_castRadius, HexColor.colorYellow );
				}
				else if ( m_castType == e_polygonCast )
				{
					DrawSolidPolygon( m_draw, new( m_rayEnd, Rot_identity ), box.vertices, box.count, box.radius, HexColor.colorYellow );
				}
			}
		}

		DrawPoint( m_draw, m_rayStart, 5.0f, HexColor.colorGreen );

		if ( m_bodyIds[m_ignoreIndex].IsNonNull )
		{
			Pos p = Body_GetPosition( m_bodyIds[m_ignoreIndex] );
			p.x -= 0.2f;
			DrawString( m_draw, m_camera, p, HexColor.colorWhite, "ign" );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CastWorld( context );
	}

	public int m_bodyIndex;
	public BodyId[] m_bodyIds = new BodyId[e_maxCount];
	public CollisionShapeUserData[] m_userData = new CollisionShapeUserData[e_maxCount];
	public Polygon[] m_polygons = new Polygon[2];
	public Polygon m_box;
	public Capsule m_capsule;
	public Circle m_circle;
	public Segment m_segment;

	public bool m_simple;

	public int m_mode;
	public int m_ignoreIndex;

	public int m_castType;
	public float m_castRadius;

	public Pos m_angleAnchor;
	public float m_baseAngle;
	public float m_angle;
	public bool m_rotating;

	public Pos m_rayStart;
	public Pos m_rayEnd;
	public bool m_dragging;
}

public class OverlapWorld : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 1;
	public const int e_boxShape = 2;

	public const int e_maxCount = 64;
	public const int e_maxDoomed = 16;

	public static bool OverlapResultFcn( ShapeId shapeId, object? context )
	{
		CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );
		if ( userData != null && userData.ignore )
		{
			// continue the query
			return true;
		}

		OverlapWorld? sample = (OverlapWorld?)context;

		if ( sample.m_doomCount < e_maxDoomed )
		{
			int index = sample.m_doomCount;
			sample.m_doomIds[index] = shapeId;
			sample.m_doomCount += 1;
		}

		// continue the query
		return true;
	}

	public OverlapWorld( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 25.0f * 0.7f;
		}

		{
			Vec2[] vertices = { new( -0.5f, 0.0f ), new( 0.5f, 0.0f ), new( 0.0f, 1.5f ) };
			Hull hull = ComputeHull( vertices, 3 );
			m_polygons[0] = MakePolygon( ref hull, 0.0f );
		}

		{
			Vec2[] vertices = { new( -0.1f, 0.0f ), new( 0.1f, 0.0f ), new( 0.0f, 1.5f ) };
			Hull hull = ComputeHull( vertices, 3 );
			m_polygons[1] = MakePolygon( ref hull, 0.0f );
		}

		{
			float w = 1.0f;
			float b = w / ( 2.0f + MathF.Sqrt( 2.0f ) );
			float s = MathF.Sqrt( 2.0f ) * b;

			Vec2[] vertices = { new( 0.5f * s, 0.0f ), new( 0.5f * w, b ), new( 0.5f * w, b + s ), new( 0.5f * s, w ), new( -0.5f * s, w ), new( -0.5f * w, b + s ), new( -0.5f * w, b ),	  new( -0.5f * s, 0.0f ) };

			Hull hull = ComputeHull( vertices, 8 );
			m_polygons[2] = MakePolygon( ref hull, 0.0f );
		}

		m_polygons[3] = MakeBox( 0.5f, 0.5f );
		m_capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
		m_circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		m_segment = new Segment { point1 = new( -1.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) };

		m_bodyIndex = 0;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodyIds[i] = nullBodyId;
		}

		m_ignoreIndex = 7;

		m_shapeType = e_circleShape;

		m_position = new Pos { x = 0.0f, y = 10.0f };
		m_angle = 0.0f;
		m_dragging = false;
		m_rotating = false;

		m_doomCount = 0;

		CreateN( 0, 10 );
	}

	public void Create( int index )
	{
		if ( m_bodyIds[m_bodyIndex].IsNonNull )
		{
			B2.DestroyBody( m_bodyIds[m_bodyIndex] );
			m_bodyIds[m_bodyIndex] = nullBodyId;
		}

		float x = RandomFloatRange( -20.0f, 20.0f );
		float y = RandomFloatRange( 0.0f, 20.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = x, y = y };
		bodyDef.rotation = MakeRot( RandomFloatRange( -Pi, Pi ) );

		m_bodyIds[m_bodyIndex] = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.userData = m_userData[m_bodyIndex];
		m_userData[m_bodyIndex].index = m_bodyIndex;
		m_userData[m_bodyIndex].ignore = false;
		if ( m_bodyIndex == m_ignoreIndex )
		{
			m_userData[m_bodyIndex].ignore = true;
		}

		if ( index < 4 )
		{
			CreatePolygonShape( m_bodyIds[m_bodyIndex], ref shapeDef, m_polygons[index] );
		}
		else if ( index == 4 )
		{
			CreateCircleShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_circle );
		}
		else if ( index == 5 )
		{
			CreateCapsuleShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_capsule );
		}
		else
		{
			CreateSegmentShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref m_segment );
		}

		m_bodyIndex = ( m_bodyIndex + 1 ) % e_maxCount;
	}

	public void CreateN( int index, int count )
	{
		for ( int i = 0; i < count; ++i )
		{
			Create( index );
		}
	}

	public void DestroyBody(  )
	{
		for ( int i = 0; i < e_maxCount; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				B2.DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
				return;
			}
		}
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_dragging = true;
				m_position = position;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_startPosition = position;
				m_baseAngle = m_angle;
			}
		}
	}

	public override void MouseUp( Pos unused13, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		if ( m_dragging )
		{
			m_position = position;
		}
		else if ( m_rotating )
		{
			float dx = ( position - m_startPosition ).x;
			m_angle = m_baseAngle + 1.0f * dx;
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Polygon 1" ) )
			Create( 0 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Poly1" ) )
			CreateN( 0, 10 );

		if ( ImGui.Button( "Polygon 2" ) )
			Create( 1 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Poly2" ) )
			CreateN( 1, 10 );

		if ( ImGui.Button( "Polygon 3" ) )
			Create( 2 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Poly3" ) )
			CreateN( 2, 10 );

		if ( ImGui.Button( "Box" ) )
			Create( 3 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Box" ) )
			CreateN( 3, 10 );

		if ( ImGui.Button( "Circle" ) )
			Create( 4 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Circle" ) )
			CreateN( 4, 10 );

		if ( ImGui.Button( "Capsule" ) )
			Create( 5 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Capsule" ) )
			CreateN( 5, 10 );

		if ( ImGui.Button( "Segment" ) )
			Create( 6 );
		ImGui.SameLine();
		if ( ImGui.Button( "10x##Segment" ) )
			CreateN( 6, 10 );

		if ( ImGui.Button( "Destroy Shape" ) )
		{
			DestroyBody();
		}

		ImGui.Separator();
		ImGui.Text( "Overlap Shape" );
		ImGui.RadioButton( "Circle##Overlap", ref m_shapeType, e_circleShape );
		ImGui.RadioButton( "Capsule##Overlap", ref m_shapeType, e_capsuleShape );
		ImGui.RadioButton( "Box##Overlap", ref m_shapeType, e_boxShape );

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		DrawScreenTextLine( "left mouse button: drag query shape" );
		DrawScreenTextLine( "left mouse button + shift: rotate query shape" );

		m_doomCount = 0;

		// Build the query shape near the origin and place it with the world-space query origin so it
		// stays precise far from the origin in large world mode.
		Rot rotation = MakeRot( m_angle );
		ShapeProxy proxy = default;

		if ( m_shapeType == e_circleShape )
		{
			Circle circle = new Circle { center = Vec2_zero, radius = 1.0f };
			proxy = B2.MakeProxy( ref circle.center, 1, circle.radius );
			DrawSolidCircle( m_draw, new( m_position, Rot_identity ), circle.center, circle.radius, HexColor.colorWhite );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			Capsule capsule = new Capsule { center1 = RotateVector( rotation, new( -1.0f, 0.0f ) ), center2 = RotateVector( rotation, new( 1.0f, 0.0f ) ), radius = 0.5f };
			proxy = B2.MakeProxy( ref capsule.center1, 2, capsule.radius );
			DrawCapsule( m_draw, OffsetPos( m_position, capsule.center1 ), OffsetPos( m_position, capsule.center2 ),
						 capsule.radius, HexColor.colorWhite );
		}
		else if ( m_shapeType == e_boxShape )
		{
			Polygon box = MakeOffsetBox( 2.0f, 0.5f, Vec2_zero, rotation );
			proxy = B2.MakeProxy( box.vertices, box.count, box.radius );
			DrawPolygon( m_draw, new( m_position, Rot_identity ), box.vertices, box.count, HexColor.colorWhite );
		}

		World_OverlapShape( m_worldId, m_position, ref proxy, DefaultQueryFilter(), OverlapResultFcn, this );

		if ( m_bodyIds[m_ignoreIndex].IsNonNull )
		{
			Pos p = Body_GetPosition( m_bodyIds[m_ignoreIndex] );
			p.x -= 0.2f;
			DrawString( m_draw, m_camera, p, HexColor.colorWhite, "skip" );
		}

		for ( int i = 0; i < m_doomCount; ++i )
		{
			ShapeId shapeId = m_doomIds[i];
			CollisionShapeUserData? userData = (CollisionShapeUserData?)Shape_GetUserData( shapeId );
			if ( userData == null )
			{
				continue;
			}

			int index = userData.index;
			Debug.Assert( 0 <= index && index < e_maxCount );
			Debug.Assert( m_bodyIds[index].IsNonNull );

			B2.DestroyBody( m_bodyIds[index] );
			m_bodyIds[index] = nullBodyId;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new OverlapWorld( context );
	}

	public int m_bodyIndex;
	public BodyId[] m_bodyIds = new BodyId[e_maxCount];
	public CollisionShapeUserData[] m_userData = NewArray<CollisionShapeUserData>( e_maxCount );
	public Polygon[] m_polygons = new Polygon[4];
	public Capsule m_capsule;
	public Circle m_circle;
	public Segment m_segment;
	public int m_ignoreIndex;

	public ShapeId[] m_doomIds = new ShapeId[e_maxDoomed];
	public int m_doomCount;

	public int m_shapeType;
	public Transform m_transform;

	public Pos m_startPosition;

	public Pos m_position;
	public float m_angle;
	public float m_baseAngle;

	public bool m_dragging;
	public bool m_rotating;
}

public class ManifoldSample : Sample
{
	public ManifoldSample( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			// m_context.camera.center = new( 1.8f, 15.0f );
			m_context.camera.center = new( 1.8f, 0.0f );
			m_context.camera.zoom = 25.0f * 0.45f;
		}

		m_smgroxCache1 = emptySimplexCache;
		m_smgroxCache2 = emptySimplexCache;
		m_smgcapCache1 = emptySimplexCache;
		m_smgcapCache2 = emptySimplexCache;

		m_transform = Transform_identity;
		m_transform.p.x = 0.17f;
		m_transform.p.y = 1.12f;
		// m_transform.q = MakeRot( 0.5f * Pi );
		m_angle = 0.0f;
		m_round = 0.1f;

		m_startPoint = new Pos { x = 0.0f, y = 0.0f };
		m_basePosition = new Vec2( 0.0f, 0.0f );
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showCount = false;
		m_showIds = false;
		m_showSeparation = false;
		m_showAnchors = false;
		m_enableCaching = true;

		Vec2[] points = { new( -0.1f, -0.5f ), new( 0.1f, -0.5f ), new( 0.0f, 0.5f ) };
		m_wedge = ComputeHull( points, 3 );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		ImGui.SliderFloat( "x offset", ref m_transform.p.x, -2.0f, 2.0f, "%.2f" );
		ImGui.SliderFloat( "y offset", ref m_transform.p.y, -2.0f, 2.0f, "%.2f" );

		if ( ImGui.SliderFloat( "angle", ref m_angle, -Pi, Pi, "%.2f" ) )
		{
			m_transform.q = MakeRot( m_angle );
		}

		ImGui.SliderFloat( "round", ref m_round, 0.0f, 0.4f, "%.1f" );

		ImGui.PopItemWidth();

		ImGui.Separator();

		ImGui.Checkbox( "show count", ref m_showCount );
		ImGui.Checkbox( "show ids", ref m_showIds );
		ImGui.Checkbox( "show separation", ref m_showSeparation );
		ImGui.Checkbox( "show anchors", ref m_showAnchors );
		ImGui.Checkbox( "enable caching", ref m_enableCaching );

		if ( ImGui.Button( "Reset" ) )
		{
			m_transform = Transform_identity;
			m_angle = 0.0f;
		}

		return true;
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		bool handled = false;

		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_dragging = true;
				m_startPoint = position;
				m_basePosition = m_transform.p;
				handled = true;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_startPoint = position;
				m_baseAngle = m_angle;
				handled = true;
			}
		}

		if ( handled == false )
		{
			base.MouseDown( position, button, mods );
		}
	}

	public override void MouseUp( Pos unused14, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		Vec2 d = position - m_startPoint;

		if ( m_dragging )
		{
			m_transform.p.x = m_basePosition.x + 0.5f * d.x;
			m_transform.p.y = m_basePosition.y + 0.5f * d.y;
		}
		else if ( m_rotating )
		{
			m_angle = ClampFloat( m_baseAngle + 1.0f * d.x, -Pi, Pi );
			m_transform.q = MakeRot( m_angle );
		}
		else
		{
			base.MouseMove( position );
		}
	}

	public void DrawManifold( in LocalManifold manifold, WorldTransform transformA, WorldTransform transformB )
	{
		Vec2 normal = RotateVector( transformA.q, manifold.normal );

		if ( m_showCount )
		{
			Pos p = LerpPos( transformA.p, transformB.p, 0.5f );
			DrawString( m_draw, m_camera, p, HexColor.colorWhite, "%d", manifold.pointCount );
		}

		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			ref readonly LocalManifoldPoint mp = ref manifold.points[i];
			Pos p1 = OffsetPos( transformA.p, RotateVector( transformA.q, mp.point ) );
			Pos p2 = p1 + 0.5f * normal;
			DrawLine( m_draw, p1, p2, HexColor.colorViolet );

			if ( m_showAnchors )
			{
				DrawPoint( m_draw, p1, 5.0f, HexColor.colorRed );
			}
			else
			{
				DrawPoint( m_draw, p1, 10.0f, HexColor.colorBlue );
			}

			if ( m_showIds )
			{
				// uint indexA = mp.id >> 8;
				// uint indexB = 0xFF & mp.id;
				DrawString( m_draw, m_camera, OffsetPos( p1, new( 0.05f, -0.02f ) ), HexColor.colorWhite, "0x%04x", mp.id );
			}

			if ( m_showSeparation )
			{
				DrawString( m_draw, m_camera, OffsetPos( p1, new( 0.05f, 0.03f ) ), HexColor.colorWhite, "%.3f", mp.separation );
			}
		}
	}

	public override void Step(  )
	{
		Vec2 offset = new Vec2( -10.0f, -5.0f );
		Vec2 increment = new Vec2( 4.0f, 0.0f );

		HexColor color1 = HexColor.colorAquamarine;
		HexColor color2 = HexColor.colorPaleGoldenRod;

		if ( m_enableCaching == false )
		{
			m_smgroxCache1 = emptySimplexCache;
			m_smgroxCache2 = emptySimplexCache;
			m_smgcapCache1 = emptySimplexCache;
			m_smgcapCache2 = emptySimplexCache;
		}

#if true
		// circle-circle
		{
			Circle circle1 = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			Circle circle2 = new Circle { center = new( 0.0f, 0.0f ), radius = 1.0f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollideCircles( ref circle1, ref circle2, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidCircle( m_draw, transform1, circle1.center, circle1.radius, color1 );
			DrawSolidCircle( m_draw, transform2, circle2.center, circle2.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// capsule-circle
		{
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollideCapsuleAndCircle( ref capsule, ref circle, InvMulWorldTransforms( transform1, transform2 ) );

			Pos v1 = TransformWorldPoint( transform1, capsule.center1 );
			Pos v2 = TransformWorldPoint( transform1, capsule.center2 );
			DrawCapsule( m_draw, v1, v2, capsule.radius, color1 );
			DrawSolidCircle( m_draw, transform2, circle.center, circle.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// segment-circle
		{
			Segment segment = new Segment { point1 = new( -1.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) };
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollideSegmentAndCircle( ref segment, ref circle, InvMulWorldTransforms( transform1, transform2 ) );

			Pos p1 = TransformWorldPoint( transform1, segment.point1 );
			Pos p2 = TransformWorldPoint( transform1, segment.point2 );
			DrawLine( m_draw, p1, p2, color1 );
			DrawSolidCircle( m_draw, transform2, circle.center, circle.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// box-circle
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			Polygon box = MakeSquare( 0.5f );
			box.radius = m_round;

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollidePolygonAndCircle( ref box, ref circle, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box.vertices, box.count, m_round, color1 );
			DrawSolidCircle( m_draw, transform2, circle.center, circle.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// capsule-capsule
		{
			Capsule capsule1 = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			Capsule capsule2 = new Capsule { center1 = new( 0.25f, 0.0f ), center2 = new( 1.0f, 0.0f ), radius = 0.1f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollideCapsules( ref capsule1, ref capsule2, InvMulWorldTransforms( transform1, transform2 ) );

			Pos v1 = TransformWorldPoint( transform1, capsule1.center1 );
			Pos v2 = TransformWorldPoint( transform1, capsule1.center2 );
			DrawCapsule( m_draw, v1, v2, capsule1.radius, color1 );

			v1 = TransformWorldPoint( transform2, capsule2.center1 );
			v2 = TransformWorldPoint( transform2, capsule2.center2 );
			DrawCapsule( m_draw, v1, v2, capsule2.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// box-capsule
		{
			Capsule capsule = new Capsule { center1 = new( -0.4f, 0.0f ), center2 = new( -0.1f, 0.0f ), radius = 0.1f };
			Polygon box = MakeOffsetBox( 0.25f, 1.0f, new( 1.0f, -1.0f ), MakeRot( 0.25f * Pi ) );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollidePolygonAndCapsule( ref box, ref capsule, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box.vertices, box.count, box.radius, color1 );

			Pos v1 = TransformWorldPoint( transform2, capsule.center1 );
			Pos v2 = TransformWorldPoint( transform2, capsule.center2 );
			DrawCapsule( m_draw, v1, v2, capsule.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// segment-capsule
		{
			Segment segment = new Segment { point1 = new( -1.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) };
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m =
				CollideSegmentAndCapsule( ref segment, ref capsule, InvMulWorldTransforms( transform1, transform2 ) );

			Pos p1 = TransformWorldPoint( transform1, segment.point1 );
			Pos p2 = TransformWorldPoint( transform1, segment.point2 );
			DrawLine( m_draw, p1, p2, color1 );

			p1 = TransformWorldPoint( transform2, capsule.center1 );
			p2 = TransformWorldPoint( transform2, capsule.center2 );
			DrawCapsule( m_draw, p1, p2, capsule.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		offset = new Vec2( -10.0f, 0.0f );
#endif

#if true
		// square-square
		{
			Polygon box1 = MakeSquare( 0.5f );
			Polygon box = MakeSquare( 0.5f );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m = CollidePolygons( ref box1, ref box, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box1.vertices, box1.count, box1.radius, color1 );
			DrawSolidPolygon( m_draw, transform2, box.vertices, box.count, box.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// box-box
		{
			Polygon box1 = MakeBox( 2.0f, 0.1f );
			Polygon box = MakeSquare( 0.25f );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({0.0f, -0.1f}, offset), q = new( 0.0f, 1.0f ) };

			LocalManifold m = CollidePolygons( ref box1, ref box, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box1.vertices, box1.count, box1.radius, color1 );
			DrawSolidPolygon( m_draw, transform2, box.vertices, box.count, box.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// box-rox
		{
			Polygon box = MakeSquare( 0.5f );
			float h = 0.5f - m_round;
			Polygon rox = MakeRoundedBox( h, h, m_round );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({0.0f, -0.1f}, offset), q = new( 0.0f, 1.0f ) };

			LocalManifold m = CollidePolygons( ref box, ref rox, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box.vertices, box.count, box.radius, color1 );
			DrawSolidPolygon( m_draw, transform2, rox.vertices, rox.count, rox.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// rox-rox
		{
			float h = 0.5f - m_round;
			Polygon rox = MakeRoundedBox( h, h, m_round );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform1 = new Transform { p = new( 6.48024225f, 2.07872653f ), q = new( -0.938356698f, 0.345668465f ) };
			// Transform transform2 = new Transform { p = new( 5.52862263f, 2.51146317f ), q = new( -0.859374702f, -0.511346340f ) };

			LocalManifold m = CollidePolygons( ref rox, ref rox, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, rox.vertices, rox.count, rox.radius, color1 );
			DrawSolidPolygon( m_draw, transform2, rox.vertices, rox.count, rox.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// segment-rox
		{
			Segment segment = new Segment { point1 = new( -1.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) };
			float h = 0.5f - m_round;
			Polygon rox = MakeRoundedBox( h, h, m_round );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({-1.44583416f, 0.397352695f}, offset), q = m_transform.q };

			LocalManifold m = CollideSegmentAndPolygon( ref segment, ref rox, InvMulWorldTransforms( transform1, transform2 ) );

			Pos p1 = TransformWorldPoint( transform1, segment.point1 );
			Pos p2 = TransformWorldPoint( transform1, segment.point2 );
			DrawLine( m_draw, p1, p2, color1 );
			DrawSolidPolygon( m_draw, transform2, rox.vertices, rox.count, rox.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}
#endif

		// wox-wox
		{
			Polygon wox = MakePolygon( ref m_wedge, m_round );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({0.0f, -0.1f}, offset), q = new( 0.0f, 1.0f ) };

			LocalManifold m = CollidePolygons( ref wox, ref wox, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, wox.vertices, wox.count, wox.radius, color1 );
			DrawSolidPolygon( m_draw, transform1, wox.vertices, wox.count, 0.0f, color1 );
			DrawSolidPolygon( m_draw, transform2, wox.vertices, wox.count, wox.radius, color2 );
			DrawSolidPolygon( m_draw, transform2, wox.vertices, wox.count, 0.0f, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

#if true
		// wox-wox
		{
			Vec2[] p1s = { new( 0.175740838f, 0.224936664f ), new( -0.301293969f, 0.194021404f ), new( -0.105151534f, -0.432157338f ) };
			Vec2[] p2s = { new( -0.427884758f, -0.225028217f ), new( 0.0566576123f, -0.128772855f ), new( 0.176625848f, 0.338923335f ) };

			Hull h1 = ComputeHull( p1s, 3 );
			Hull h2 = ComputeHull( p2s, 3 );
			Polygon w1 = MakePolygon( ref h1, 0.158798501f );
			Polygon w2 = MakePolygon( ref h2, 0.205900759f );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({0.0f, -0.1f}, offset), q = new( 0.0f, 1.0f ) };

			LocalManifold m = CollidePolygons( ref w1, ref w2, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, w1.vertices, w1.count, w1.radius, color1 );
			DrawSolidPolygon( m_draw, transform1, w1.vertices, w1.count, 0.0f, color1 );
			DrawSolidPolygon( m_draw, transform2, w2.vertices, w2.count, w2.radius, color2 );
			DrawSolidPolygon( m_draw, transform2, w2.vertices, w2.count, 0.0f, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		offset = new Vec2( -10.0f, 5.0f );

		// box-triangle
		{
			Polygon box = MakeBox( 1.0f, 1.0f );
			Vec2[] points = { new( -0.05f, 0.0f ), new( 0.05f, 0.0f ), new( 0.0f, 0.1f ) };
			Hull hull = ComputeHull( points, 3 );
			Polygon tri = MakePolygon( ref hull, 0.0f );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );
			// Transform transform2 = new Transform { p = Add({0.0f, -0.1f}, offset), q = new( 0.0f, 1.0f ) };

			LocalManifold m = CollidePolygons( ref box, ref tri, InvMulWorldTransforms( transform1, transform2 ) );

			DrawSolidPolygon( m_draw, transform1, box.vertices, box.count, 0.0f, color1 );
			DrawSolidPolygon( m_draw, transform2, tri.vertices, tri.count, 0.0f, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset = Add( offset, increment );
		}

		// chain-segment vs circle
		{
			ChainSegment segment = new ChainSegment { ghost1 = new( 2.0f, 1.0f ), segment = new() { point1 = new( 1.0f, 1.0f ), point2 = new( -1.0f, 0.0f ) }, ghost2 = new( -2.0f, 0.0f ), chainId = -1 };
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			LocalManifold m =
				CollideChainSegmentAndCircle( ref segment, ref circle, InvMulWorldTransforms( transform1, transform2 ) );

			Pos g1 = TransformWorldPoint( transform1, segment.ghost1 );
			Pos g2 = TransformWorldPoint( transform1, segment.ghost2 );
			Pos p1 = TransformWorldPoint( transform1, segment.segment.point1 );
			Pos p2 = TransformWorldPoint( transform1, segment.segment.point2 );
			DrawLine( m_draw, g1, p1, HexColor.colorLightGray );
			DrawLine( m_draw, p1, p2, color1 );
			DrawLine( m_draw, p2, g2, HexColor.colorLightGray );
			DrawSolidCircle( m_draw, transform2, circle.center, circle.radius, color2 );

			DrawManifold( ref m, transform1, transform2 );

			offset.x += 2.0f * increment.x;
		}

		// chain-segment vs rounded polygon
		{
			ChainSegment segment1 = new ChainSegment { ghost1 = new( 2.0f, 1.0f ), segment = new() { point1 = new( 1.0f, 1.0f ), point2 = new( -1.0f, 0.0f ) }, ghost2 = new( -2.0f, 0.0f ), chainId = -1 };
			ChainSegment segment2 = new ChainSegment { ghost1 = new( 3.0f, 1.0f ), segment = new() { point1 = new( 2.0f, 1.0f ), point2 = new( 1.0f, 1.0f ) }, ghost2 = new( -1.0f, 0.0f ), chainId = -1 };
			// ChainSegment segment1 = new ChainSegment { ghost1 = new( 2.0f, 0.0f ), segment = new() { point1 = new( 1.0f, 0.0f ), point2 = new( -1.0f, 0.0f ) }, ghost2 = new( -2.0f, 0.0f ), chainId = -1 };
			// ChainSegment segment2 = new ChainSegment { ghost1 = new( 3.0f, 0.0f ), segment = new() { point1 = new( 2.0f, 0.0f ), point2 = new( 1.0f, 0.0f ) }, ghost2 = new( -1.0f, 0.0f ), chainId = -1 };
			// ChainSegment segment1 = new ChainSegment { ghost1 = new( 0.5f, 1.0f ), segment = new() { point1 = new( 0.0f, 2.0f ), point2 = new( -0.5f, 1.0f ) }, ghost2 = new( -1.0f, 0.0f ), chainId = -1 };
			// ChainSegment segment2 = new ChainSegment { ghost1 = new( 1.0f, 0.0f ), segment = new() { point1 = new( 0.5f, 1.0f ), point2 = new( 0.0f, 2.0f ) }, ghost2 = new( -0.5f, 1.0f ), chainId = -1 };
			float h = 0.5f - m_round;
			Polygon rox = MakeRoundedBox( h, h, m_round );

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			Transform xf = InvMulWorldTransforms( transform1, transform2 );
			LocalManifold m1 = CollideChainSegmentAndPolygon( ref segment1, ref rox, xf, ref m_smgroxCache1 );
			LocalManifold m2 = CollideChainSegmentAndPolygon( ref segment2, ref rox, xf, ref m_smgroxCache2 );

			{
				Pos g2 = TransformWorldPoint( transform1, segment1.ghost2 );
				Pos p1 = TransformWorldPoint( transform1, segment1.segment.point1 );
				Pos p2 = TransformWorldPoint( transform1, segment1.segment.point2 );
				DrawLine( m_draw, p1, p2, color1 );
				DrawPoint( m_draw, p1, 4.0f, color1 );
				DrawPoint( m_draw, p2, 4.0f, color1 );
				DrawLine( m_draw, p2, g2, HexColor.colorLightGray );
			}

			{
				Pos g1 = TransformWorldPoint( transform1, segment2.ghost1 );
				Pos p1 = TransformWorldPoint( transform1, segment2.segment.point1 );
				Pos p2 = TransformWorldPoint( transform1, segment2.segment.point2 );
				DrawLine( m_draw, g1, p1, HexColor.colorLightGray );
				DrawLine( m_draw, p1, p2, color1 );
				DrawPoint( m_draw, p1, 4.0f, color1 );
				DrawPoint( m_draw, p2, 4.0f, color1 );
			}

			DrawSolidPolygon( m_draw, transform2, rox.vertices, rox.count, rox.radius, color2 );
			DrawPoint( m_draw, TransformWorldPoint( transform2, rox.centroid ), 5.0f, HexColor.colorGainsboro );

			DrawManifold( ref m1, transform1, transform2 );
			DrawManifold( ref m2, transform1, transform2 );

			offset.x += 2.0f * increment.x;
		}

		// chain-segment vs capsule
		{
			ChainSegment segment1 = new ChainSegment { ghost1 = new( 2.0f, 1.0f ), segment = new() { point1 = new( 1.0f, 1.0f ), point2 = new( -1.0f, 0.0f ) }, ghost2 = new( -2.0f, 0.0f ), chainId = -1 };
			ChainSegment segment2 = new ChainSegment { ghost1 = new( 3.0f, 1.0f ), segment = new() { point1 = new( 2.0f, 1.0f ), point2 = new( 1.0f, 1.0f ) }, ghost2 = new( -1.0f, 0.0f ), chainId = -1 };
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };

			WorldTransform transform1 = MakeWorldTransform( new( offset, Rot_identity ) );
			WorldTransform transform2 = MakeWorldTransform( new( Add( m_transform.p, offset ), m_transform.q ) );

			Transform xf = InvMulWorldTransforms( transform1, transform2 );
			LocalManifold m1 = CollideChainSegmentAndCapsule( ref segment1, ref capsule, xf, ref m_smgcapCache1 );
			LocalManifold m2 = CollideChainSegmentAndCapsule( ref segment2, ref capsule, xf, ref m_smgcapCache2 );

			{
				Pos g2 = TransformWorldPoint( transform1, segment1.ghost2 );
				Pos p1Inner = TransformWorldPoint( transform1, segment1.segment.point1 );
				Pos p2Inner = TransformWorldPoint( transform1, segment1.segment.point2 );
				// DrawSegment(g1, p1Inner, HexColor.colorLightGray);
				DrawLine( m_draw, p1Inner, p2Inner, color1 );
				DrawPoint( m_draw, p1Inner, 4.0f, color1 );
				DrawPoint( m_draw, p2Inner, 4.0f, color1 );
				DrawLine( m_draw, p2Inner, g2, HexColor.colorLightGray );
			}

			{
				Pos g1 = TransformWorldPoint( transform1, segment2.ghost1 );
				Pos p1Inner = TransformWorldPoint( transform1, segment2.segment.point1 );
				Pos p2Inner = TransformWorldPoint( transform1, segment2.segment.point2 );
				DrawLine( m_draw, g1, p1Inner, HexColor.colorLightGray );
				DrawLine( m_draw, p1Inner, p2Inner, color1 );
				DrawPoint( m_draw, p1Inner, 4.0f, color1 );
				DrawPoint( m_draw, p2Inner, 4.0f, color1 );
				// DrawSegment(p2Inner, g2, HexColor.colorLightGray);
			}

			Pos p1 = TransformWorldPoint( transform2, capsule.center1 );
			Pos p2 = TransformWorldPoint( transform2, capsule.center2 );
			DrawCapsule( m_draw, p1, p2, capsule.radius, color2 );

			DrawPoint( m_draw, LerpPos( p1, p2, 0.5f ), 5.0f, HexColor.colorGainsboro );

			DrawManifold( ref m1, transform1, transform2 );
			DrawManifold( ref m2, transform1, transform2 );

			offset.x += 2.0f * increment.x;
		}
#endif

		DrawScreenTextLine( "mouse button 1: drag" );
		DrawScreenTextLine( "mouse button 1 + shift: rotate" );
	}

	public static Sample Create( SampleContext context )
	{
		return new ManifoldSample( context );
	}

	public SimplexCache m_smgroxCache1;
	public SimplexCache m_smgroxCache2;
	public SimplexCache m_smgcapCache1;
	public SimplexCache m_smgcapCache2;

	public Hull m_wedge;

	public Transform m_transform;
	public float m_angle;
	public float m_round;

	public Vec2 m_basePosition;
	public Pos m_startPoint;
	public float m_baseAngle;

	public bool m_dragging;
	public bool m_rotating;
	public bool m_showCount;
	public bool m_showIds;
	public bool m_showAnchors;
	public bool m_showSeparation;
	public bool m_enableCaching;
}

public class SmoothManifold : Sample
{
	public const int e_circleShape = 0;
	public const int e_boxShape = 0 + 1;

	public SmoothManifold( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.0f, 20.0f );
			m_context.camera.zoom = 21.0f;
		}

		m_shapeType = e_boxShape;
		m_transform = new Transform { p = new( 0.0f, 20.0f ), q = Rot_identity };
		m_angle = 0.0f;
		m_round = 0.0f;

		m_startPoint = new Pos { x = 0.0f, y = 00.0f };
		m_basePosition = new Vec2( 0.0f, 0.0f );
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showIds = false;
		m_showAnchors = false;
		m_showSeparation = false;

		// https://betravis.github.io/shape-tools/path-to-polygon/
		m_count = 36;

		Vec2[] points = new Vec2[36];
		points[0] = new( -20.58325f, 14.54175f );
		points[1] = new( -21.90625f, 15.8645f );
		points[2] = new( -24.552f, 17.1875f );
		points[3] = new( -27.198f, 11.89575f );
		points[4] = new( -29.84375f, 15.8645f );
		points[5] = new( -29.84375f, 21.15625f );
		points[6] = new( -25.875f, 23.802f );
		points[7] = new( -20.58325f, 25.125f );
		points[8] = new( -25.875f, 29.09375f );
		points[9] = new( -20.58325f, 31.7395f );
		points[10] = new( -11.0089998f, 23.2290001f );
		points[11] = new( -8.67700005f, 21.15625f );
		points[12] = new( -6.03125f, 21.15625f );
		points[13] = new( -7.35424995f, 29.09375f );
		points[14] = new( -3.38549995f, 29.09375f );
		points[15] = new( 1.90625f, 30.41675f );
		points[16] = new( 5.875f, 17.1875f );
		points[17] = new( 11.16675f, 25.125f );
		points[18] = new( 9.84375f, 29.09375f );
		points[19] = new( 13.8125f, 31.7395f );
		points[20] = new( 21.75f, 30.41675f );
		points[21] = new( 28.3644981f, 26.448f );
		points[22] = new( 25.71875f, 18.5105f );
		points[23] = new( 24.3957481f, 13.21875f );
		points[24] = new( 17.78125f, 11.89575f );
		points[25] = new( 15.1355f, 7.92700005f );
		points[26] = new( 5.875f, 9.25f );
		points[27] = new( 1.90625f, 11.89575f );
		points[28] = new( -3.25f, 11.89575f );
		points[29] = new( -3.25f, 9.9375f );
		points[30] = new( -4.70825005f, 9.25f );
		points[31] = new( -8.67700005f, 9.25f );
		points[32] = new( -11.323f, 11.89575f );
		points[33] = new( -13.96875f, 11.89575f );
		points[34] = new( -15.29175f, 14.54175f );
		points[35] = new( -19.2605f, 14.54175f );

		m_segments = new ChainSegment[m_count];

		for ( int i = 0; i < m_count; ++i )
		{
			int i0 = i > 0 ? i - 1 : m_count - 1;
			int i1 = i;
			int i2 = i1 < m_count - 1 ? i1 + 1 : 0;
			int i3 = i2 < m_count - 1 ? i2 + 1 : 0;

			Vec2 g1 = points[i0];
			Vec2 p1 = points[i1];
			Vec2 p2 = points[i2];
			Vec2 g2 = points[i3];

			m_segments[i] = new ChainSegment { ghost1 = g1, segment = new Segment { point1 = p1, point2 = p2 }, ghost2 = g2, chainId = -1 };
		}
	}

	public override void Dispose()
	{
		base.Dispose();
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		{
			string[] shapeTypes = { "Circle", "Box" };
			int shapeType = (int)( m_shapeType );
			ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length );
			m_shapeType = (int)( shapeType );
		}

		ImGui.SliderFloat( "x Offset", ref m_transform.p.x, -2.0f, 2.0f, "%.2f" );
		ImGui.SliderFloat( "y Offset", ref m_transform.p.y, -2.0f, 2.0f, "%.2f" );

		if ( ImGui.SliderFloat( "Angle", ref m_angle, -Pi, Pi, "%.2f" ) )
		{
			m_transform.q = MakeRot( m_angle );
		}

		ImGui.SliderFloat( "Round", ref m_round, 0.0f, 0.4f, "%.1f" );

		ImGui.PopItemWidth();

		ImGui.Checkbox( "Show Ids", ref m_showIds );
		ImGui.Checkbox( "Show Separation", ref m_showSeparation );
		ImGui.Checkbox( "Show Anchors", ref m_showAnchors );

		if ( ImGui.Button( "Reset" ) )
		{
			m_transform = Transform_identity;
			m_angle = 0.0f;
		}

		ImGui.Separator();

		ImGui.Text( "mouse button 1: drag" );
		ImGui.Text( "mouse button 1 + shift: rotate" );

		return true;
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_dragging = true;
				m_startPoint = position;
				m_basePosition = m_transform.p;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_startPoint = position;
				m_baseAngle = m_angle;
			}
		}
	}

	public override void MouseUp( Pos unused15, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		Vec2 d = position - m_startPoint;

		if ( m_dragging )
		{
			m_transform.p.x = m_basePosition.x + d.x;
			m_transform.p.y = m_basePosition.y + d.y;
		}
		else if ( m_rotating )
		{
			m_angle = ClampFloat( m_baseAngle + 1.0f * d.x, -Pi, Pi );
			m_transform.q = MakeRot( m_angle );
		}
	}

	public void DrawManifold( in LocalManifold manifold, WorldTransform transformA )
	{
		Vec2 normal = RotateVector( transformA.q, manifold.normal );

		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			ref readonly LocalManifoldPoint mp = ref manifold.points[i];
			Pos p1 = OffsetPos( transformA.p, RotateVector( transformA.q, mp.point ) );
			Pos p2 = p1 + 0.5f * normal;
			DrawLine( m_draw, p1, p2, HexColor.colorWhite );

			if ( m_showAnchors )
			{
				DrawPoint( m_draw, p1, 5.0f, HexColor.colorGreen );
			}
			else
			{
				DrawPoint( m_draw, p1, 5.0f, HexColor.colorGreen );
			}

			if ( m_showIds )
			{
				// uint indexA = mp.id >> 8;
				// uint indexB = 0xFF & mp.id;
				DrawString( m_draw, m_camera, OffsetPos( p1, new( 0.05f, -0.02f ) ), HexColor.colorWhite, "0x%04x", mp.id );
			}

			if ( m_showSeparation )
			{
				DrawString( m_draw, m_camera, OffsetPos( p1, new( 0.05f, 0.03f ) ), HexColor.colorWhite, "%.3f", mp.separation );
			}
		}
	}

	public override void Step(  )
	{
		HexColor color1 = HexColor.colorYellow;
		HexColor color2 = HexColor.colorMagenta;

		WorldTransform transform1 = WorldTransform_identity;
		WorldTransform transform2 = MakeWorldTransform( m_transform );

		for ( int i = 0; i < m_count; ++i )
		{
			ref readonly ChainSegment segment = ref m_segments[i];
			Pos p1 = TransformWorldPoint( transform1, segment.segment.point1 );
			Pos p2 = TransformWorldPoint( transform1, segment.segment.point2 );
			DrawLine( m_draw, p1, p2, color1 );
			DrawPoint( m_draw, p1, 4.0f, color1 );
		}

		// chain-segment vs circle
		if ( m_shapeType == e_circleShape )
		{
			float radius = 0.5f;
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = radius };
			DrawSolidCircle( m_draw, transform2, circle.center, circle.radius, color2 );

			for ( int i = 0; i < m_count; ++i )
			{
				ref readonly ChainSegment segment = ref m_segments[i];
				LocalManifold m =
					CollideChainSegmentAndCircle( segment, ref circle, InvMulWorldTransforms( transform1, transform2 ) );
				DrawManifold( ref m, transform1 );
			}
		}
		else if ( m_shapeType == e_boxShape )
		{
			float h = 0.5f - m_round;
			Polygon rox = MakeRoundedBox( h, h, m_round );
			DrawSolidPolygon( m_draw, transform2, rox.vertices, rox.count, rox.radius, color2 );

			for ( int i = 0; i < m_count; ++i )
			{
				ref readonly ChainSegment segment = ref m_segments[i];
				SimplexCache cache = default;
				LocalManifold m =
					CollideChainSegmentAndPolygon( segment, ref rox, InvMulWorldTransforms( transform1, transform2 ), ref cache );
				DrawManifold( ref m, transform1 );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SmoothManifold( context );
	}

	public int m_shapeType;

	public ChainSegment[] m_segments;
	public int m_count;

	public Transform m_transform;
	public float m_angle;
	public float m_round;

	public Vec2 m_basePosition;
	public Pos m_startPoint;
	public float m_baseAngle;

	public bool m_dragging;
	public bool m_rotating;
	public bool m_showIds;
	public bool m_showAnchors;
	public bool m_showSeparation;
}

public class ShapeCast : Sample
{
	public const int e_point = 0;
	public const int e_segment = 1;
	public const int e_triangle = 2;
	public const int e_box = 3;

	public ShapeCast( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.25f );
			m_context.camera.zoom = 3.0f;
		}

		m_point = Vec2_zero;
		m_segment = new Segment { point1 = new( 0.0f, 0.0f ), point2 = new( 0.5f, 0.0f ) };

		{
			Vec2[] points = { new( -0.5f, 0.0f ), new( 0.5f, 0.0f ), new( 0.0f, 1.0f ) };
			Hull hull = ComputeHull( points, 3 );
			m_triangle = MakePolygon( ref hull, 0.0f );
		}

#if false
		{
			Vec2[] points = new Vec2[4];
			points[0].x = -0.599999964f;
			points[0].y = -0.700000048f;
			points[1].x = 0.449999988f;
			points[1].y = -0.700000048f;
			points[2].x = 0.449999988f;
			points[2].y = 0.350000024f;
			points[3].x = -0.599999964f;
			points[3].y = 0.350000024f;

			points[0] = new( 3.0f, -0.5f );
			points[1] = new( 3.0f, 0.5f );
			points[2] = new( -3.0f, 0.5f );
			points[3] = new( -3.0f, -0.5f );
			Hull hull = ComputeHull( points, 4 );
			bool isValid = ValidateHull( ref hull );
			Debug.Assert( isValid );

			m_triangle = MakePolygon( ref hull, 0.0f );
		}
#endif

		// m_box = MakeOffsetBox( 0.5f, 0.5f, new( 0.0f, 0.0f ), Rot_identity );
		m_box = MakeBox( 8.984375f, 0.5f );

#if false
		{
			Vec2[] points = new Vec2[4];
			points[0].x = 0.449999988f;
			points[0].y = -0.100000001f;
			points[1].x = 0.550000012f;
			points[1].y = -0.100000001f;
			points[2].x = 0.550000012f;
			points[2].y = 0.100000001f;
			points[3].x = 0.449999988f;
			points[3].y = 0.100000001f;
			Hull hull = ComputeHull( points, 4 );
			m_box = MakePolygon( ref hull, 0.0f );
		}
#endif

		m_transform = new Transform { p = new( -0.6f, 0.0f ), q = Rot_identity };
		m_translation = new Vec2( 2.0f, 0.0f );
		m_angle = 0.0f;
		m_startPoint = new Pos { x = 0.0f, y = 0.0f };
		m_basePosition = new Vec2( 0.0f, 0.0f );
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_sweeping = false;
		m_rotating = false;
		m_showIndices = false;
		m_drawSimplex = false;
		m_encroach = false;

		m_typeA = e_box;
		m_typeB = e_point;
		m_radiusA = 0.0f;
		m_radiusB = 0.2f;

		m_proxyA = MakeProxy( m_typeA, m_radiusA );
		m_proxyB = MakeProxy( m_typeB, m_radiusB );
	}

	public ShapeProxy MakeProxy( int type, float radius )
	{
		ShapeProxy proxy = default;
		proxy.radius = radius;

		switch ( type )
		{
			case e_point:
				proxy.points[0] = Vec2_zero;
				proxy.count = 1;
				break;

			case e_segment:
				proxy.points[0] = m_segment.point1;
				proxy.points[1] = m_segment.point2;
				proxy.count = 2;
				break;

			case e_triangle:
				for ( int i = 0; i < m_triangle.count; ++i )
				{
					proxy.points[i] = m_triangle.vertices[i];
				}
				proxy.count = m_triangle.count;
				break;

			case e_box:
				proxy.points[0] = m_box.vertices[0];
				proxy.points[1] = m_box.vertices[1];
				proxy.points[2] = m_box.vertices[2];
				proxy.points[3] = m_box.vertices[3];
				proxy.count = 4;
				break;

			default:
				Debug.Assert( false );
				break;
		}

		return proxy;
	}

	public void DrawShape( int type, WorldTransform transform, float radius, HexColor color )
	{
		switch ( type )
		{
			case e_point:
			{
				Pos p = TransformWorldPoint( transform, m_point );
				if ( radius > 0.0f )
				{
					DrawSolidCircle( m_draw, new( p, transform.q ), Vec2_zero, radius, color );
				}
				else
				{
					DrawPoint( m_draw, p, 5.0f, color );
				}
			}
			break;

			case e_segment:
			{
				Pos p1 = TransformWorldPoint( transform, m_segment.point1 );
				Pos p2 = TransformWorldPoint( transform, m_segment.point2 );

				if ( radius > 0.0f )
				{
					DrawCapsule( m_draw, p1, p2, radius, color );
				}
				else
				{
					DrawLine( m_draw, p1, p2, color );
				}
			}
			break;

			case e_triangle:
				DrawSolidPolygon( m_draw, transform, m_triangle.vertices, m_triangle.count, radius, color );
				break;

			case e_box:
				DrawSolidPolygon( m_draw, transform, m_box.vertices, m_box.count, radius, color );
				break;

			default:
				Debug.Assert( false );
				break;
		}
	}

	public override void MouseDown( Pos position, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 )
			{
				m_dragging = true;
				m_sweeping = false;
				m_rotating = false;
				m_startPoint = position;
				m_basePosition = m_transform.p;
			}
			else if ( mods == GLFW_MOD_SHIFT )
			{
				m_dragging = false;
				m_sweeping = false;
				m_rotating = true;
				m_startPoint = position;
				m_baseAngle = m_angle;
			}
			else if ( mods == GLFW_MOD_CONTROL )
			{
				m_dragging = false;
				m_sweeping = true;
				m_rotating = false;
				m_startPoint = position;
				m_basePosition = Vec2_zero;
			}
		}
	}

	public override void MouseUp( Pos unused16, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_sweeping = false;
			m_rotating = false;
		}
	}

	public override void MouseMove( Pos position )
	{
		Vec2 d = position - m_startPoint;

		if ( m_dragging )
		{
			m_transform.p = m_basePosition + 0.5f * d;
		}
		else if ( m_rotating )
		{
			m_angle = ClampFloat( m_baseAngle + 1.0f * d.x, -Pi, Pi );
			m_transform.q = MakeRot( m_angle );
		}
		else if ( m_sweeping )
		{
			m_translation = d;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		string[] shapeTypes = { "point", "segment", "triangle", "box" };
		int shapeType = (int)( m_typeA );
		if ( ImGui.Combo( "shape A", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_typeA = (int)( shapeType );
			m_proxyA = MakeProxy( m_typeA, m_radiusA );
		}

		if ( ImGui.SliderFloat( "radius A", ref m_radiusA, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyA.radius = m_radiusA;
		}

		shapeType = (int)( m_typeB );
		if ( ImGui.Combo( "shape B", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_typeB = (int)( shapeType );
			m_proxyB = MakeProxy( m_typeB, m_radiusB );
		}

		if ( ImGui.SliderFloat( "radius B", ref m_radiusB, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyB.radius = m_radiusB;
		}

		ImGui.Separator();

		ImGui.SliderFloat( "x offset", ref m_transform.p.x, -2.0f, 2.0f, "%.2f" );
		ImGui.SliderFloat( "y offset", ref m_transform.p.y, -2.0f, 2.0f, "%.2f" );

		if ( ImGui.SliderFloat( "angle", ref m_angle, -Pi, Pi, "%.2f" ) )
		{
			m_transform.q = MakeRot( m_angle );
		}

		ImGui.PopItemWidth();

		ImGui.Separator();

		ImGui.Checkbox( "show indices", ref m_showIndices );
		ImGui.Checkbox( "encroach", ref m_encroach );

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		ShapeCastPairInput input = default;
		input.proxyA = m_proxyA;
		input.proxyB = m_proxyB;
		input.transform = m_transform;
		input.translationB = m_translation;
		input.maxFraction = 1.0f;
		input.canEncroach = m_encroach;

		CastOutput output = CastShape( ref input );

		float t = output.hit ? output.fraction : 1.0f;
		Transform transform = default;
		transform.q = m_transform.q;
		transform.p = MulAdd( m_transform.p, t, input.translationB );

		DistanceInput distanceInput = default;
		distanceInput.proxyA = m_proxyA;
		distanceInput.proxyB = m_proxyB;
		distanceInput.transform = transform;
		distanceInput.useRadii = false;
		SimplexCache distanceCache = default;
		distanceCache.count = 0;
		DistanceOutput distanceOutput = ComputeShapeDistance( ref distanceInput, ref distanceCache );

		DrawScreenTextLine( "hit = %s, iterations = %d, fraction = %g, distance = %g", output.hit ? "true" : "false",
							output.iterations, output.fraction, distanceOutput.distance );

		DrawShape( m_typeA, WorldTransform_identity, m_radiusA, HexColor.colorCyan );
		DrawShape( m_typeB, MakeWorldTransform( m_transform ), m_radiusB, HexColor.colorLightGreen );
		Transform transform2 = new Transform { p = m_transform.p + m_translation, q = m_transform.q };
		DrawShape( m_typeB, MakeWorldTransform( transform2 ), m_radiusB, HexColor.colorIndianRed );

		if ( output.hit )
		{
			DrawShape( m_typeB, MakeWorldTransform( transform ), m_radiusB, HexColor.colorPlum );

			if ( output.fraction > 0.0f )
			{
				DrawPoint( m_draw, ToPos( output.point ), 5.0f, HexColor.colorWhite );
				DrawLine( m_draw, ToPos( output.point ), ToPos( output.point + 0.5f * output.normal ), HexColor.colorYellow );
			}
			else
			{
				DrawPoint( m_draw, ToPos( output.point ), 5.0f, HexColor.colorPeru );
			}
		}

		if ( m_showIndices )
		{
			// Shape A sits at the world origin, shape B at the offset transform
			for ( int i = 0; i < m_proxyA.count; ++i )
			{
				DrawString( m_draw, m_camera, ToPos( m_proxyA.points[i] ), HexColor.colorWhite, " %d", i );
			}

			WorldTransform transformB = MakeWorldTransform( m_transform );
			for ( int i = 0; i < m_proxyB.count; ++i )
			{
				Pos p = TransformWorldPoint( transformB, m_proxyB.points[i] );
				DrawString( m_draw, m_camera, p, HexColor.colorWhite, " %d", i );
			}
		}

		DrawScreenTextLine( "mouse button 1: drag" );
		DrawScreenTextLine( "mouse button 1 + shift: rotate" );
		DrawScreenTextLine( "mouse button 1 + control: sweep" );
		DrawScreenTextLine( "distance = %.2f, iterations = %d", distanceOutput.distance, output.iterations );
	}

	public static Sample Create( SampleContext context )
	{
		return new ShapeCast( context );
	}

	public Polygon m_box;
	public Polygon m_triangle;
	public Vec2 m_point;
	public Segment m_segment;

	public int m_typeA;
	public int m_typeB;
	public float m_radiusA;
	public float m_radiusB;
	public ShapeProxy m_proxyA;
	public ShapeProxy m_proxyB;

	public Transform m_transform;
	public float m_angle;
	public Vec2 m_translation;

	public Vec2 m_basePosition;
	public Pos m_startPoint;
	public float m_baseAngle;

	public bool m_dragging;
	public bool m_sweeping;
	public bool m_rotating;
	public bool m_showIndices;
	public bool m_drawSimplex;
	public bool m_encroach;
}

public class TimeOfImpact : Sample
{
	public TimeOfImpact( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.6f, 2.0f );
			m_context.camera.center = new( -16, 45 );
			m_context.camera.zoom = 5.0f;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new TimeOfImpact( context );
	}

	public override void Step(  )
	{
		base.Step();

		Sweep sweepA = new Sweep { localCenter = Vec2_zero, c1 = new( 0.0f, 0.0f ), c2 = new( 0.0f, 0.0f ), q1 = Rot_identity, q2 = Rot_identity };
		Sweep sweepB = new Sweep { localCenter = Vec2_zero, c1 = new( -15.8332710f, 45.3520279f ), c2 = new( -15.8324337f, 45.3413048f ), q1 = new( -0.540891349f, 0.841092527f ), q2 = new( -0.457797021f, 0.889056742f ) };

		TOIInput input = default;
		input.proxyA = B2.MakeProxy( m_verticesA, m_countA, m_radiusA );
		input.proxyB = B2.MakeProxy( m_verticesB, m_countB, m_radiusB );
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.maxFraction = 1.0f;

		TOIOutput output = ComputeTimeOfImpact( ref input );

		DrawScreenTextLine( "toi = %g", output.fraction );

		// Draw A
		WorldTransform transformA = MakeWorldTransform( GetSweepTransform( ref sweepA, 0.0f ) );
		DrawPolygon( m_draw, transformA, m_verticesA, m_countA, HexColor.colorGray );

		// Draw B at t = 0
		WorldTransform transformB = MakeWorldTransform( GetSweepTransform( ref sweepB, 0.0f ) );
		DrawCapsule( m_draw, TransformWorldPoint( transformB, m_verticesB[0] ),
					 TransformWorldPoint( transformB, m_verticesB[1] ), m_radiusB, HexColor.colorGreen );

		// Draw B at t = hit_time
		transformB = MakeWorldTransform( GetSweepTransform( ref sweepB, output.fraction ) );
		DrawPolygon( m_draw, transformB, m_verticesB, m_countB, HexColor.colorOrange );

		// Draw B at t = 1
		transformB = MakeWorldTransform( GetSweepTransform( ref sweepB, 1.0f ) );
		DrawCapsule( m_draw, TransformWorldPoint( transformB, m_verticesB[0] ),
					 TransformWorldPoint( transformB, m_verticesB[1] ), m_radiusB, HexColor.colorRed );

		if ( output.state == TOIState.toiStateHit )
		{
			DistanceInput distanceInput = default;
			distanceInput.proxyA = input.proxyA;
			distanceInput.proxyB = input.proxyB;
			distanceInput.transform = InvMulTransforms( GetSweepTransform( ref sweepA, output.fraction ),
														  GetSweepTransform( ref sweepB, output.fraction ) );
			distanceInput.useRadii = false;
			SimplexCache cache = default;
			DistanceOutput distanceOutput = ComputeShapeDistance( ref distanceInput, ref cache );
			DrawScreenTextLine( "distance = %g", distanceOutput.distance );
		}
	}

	public Vec2[] m_verticesA = { new( -16.25f, 44.75f ), new( -15.75f, 44.75f ), new( -15.75f, 45.25f ), new( -16.25f, 45.25f ) };
	public Vec2[] m_verticesB = { new( 0.0f, -0.125000000f ), new( 0.0f, 0.125000000f ) };

	public int m_countA = 4;
	public int m_countB = 2;

	public float m_radiusA = 0.0f;
	public float m_radiusB = 0.0299999993f;
}
