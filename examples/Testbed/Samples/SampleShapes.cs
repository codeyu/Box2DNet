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

public static partial class Helpers_sample_shapes
{
	// This sample shows how careful creation of compound shapes leads to better simulation and avoids
	// objects getting stuck.
	// This also shows how to get the combined AABB for the body.






	// This shows how to use custom filtering















	// This sample shows how to modify the geometry on an existing shape. This is only supported on
	// dynamic and kinematic shapes because static shapes don't look for new collisions.



	// Shows how to link to chain shapes together. This is a useful technique for building large game levels with smooth collision.












	// This shows how to use explosions and demonstrates the projected perimeter



	// This sample tests a static shape being recreated every step.
}

public class ChainShape : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_boxShape = 0 + 2;

	public ChainShape( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 1.75f;
		}

		m_groundId = nullBodyId;
		m_bodyId = nullBodyId;
		m_chainId = nullChainId;
		m_shapeId = nullShapeId;
		m_shapeType = e_circleShape;

		m_material = DefaultSurfaceMaterial();
		m_material.friction = 0.2f;
		m_material.customColor = (uint)HexColor.colorSteelBlue;
		m_material.userMaterialId = 42;

		CreateScene();
		Launch();
	}

	public void CreateScene(  )
	{
		if ( m_groundId.IsNonNull )
		{
			DestroyBody( m_groundId );
		}

		// https://betravis.github.io/shape-tools/path-to-polygon/
		// Vec2 points[] = {{-20.58325, 14.54175}, new( -21.90625, 15.8645 ), new( -24.552, 17.1875 ),
		//				   {-27.198, 11.89575}, new( -29.84375, 15.8645 ), new( -29.84375, 21.15625 ),
		//				   {-25.875, 23.802}, new( -20.58325, 25.125 ), new( -25.875, 29.09375 ),
		//				   {-20.58325, 31.7395}, new( -11.0089998, 23.2290001 ), new( -8.67700005, 21.15625 ),
		//				   {-6.03125, 21.15625}, new( -7.35424995, 29.09375 ), new( -3.38549995, 29.09375 ),
		//				   {1.90625, 30.41675}, new( 5.875, 17.1875 ), new( 11.16675, 25.125 ),
		//				   {9.84375, 29.09375}, new( 13.8125, 31.7395 ), new( 21.75, 30.41675 ),
		//				   {28.3644981, 26.448}, new( 25.71875, 18.5105 ), new( 24.3957481, 13.21875 ),
		//				   {17.78125, 11.89575}, new( 15.1355, 7.92700005 ), new( 5.875, 9.25 ),
		//				   {1.90625, 11.89575}, new( -3.25, 11.89575 ), new( -3.25, 9.9375 ),
		//				   {-4.70825005, 9.25}, new( -8.67700005, 9.25 ), new( -11.323, 11.89575 ),
		//				   {-13.96875, 11.89575}, new( -15.29175, 14.54175 ),	 {-19.2605, 14.54175}};

		Vec2[] points = {
			new( -56.885498f, 12.8985004f ), new( -56.885498f, 16.2057495f ),	new( 56.885498f, 16.2057495f ), new( 56.885498f, -16.2057514f ), new( 51.5935059f, -16.2057514f ), new( 43.6559982f, -10.9139996f ),	new( 35.7184982f, -10.9139996f ), new( 27.7809982f, -10.9139996f ), new( 21.1664963f, -14.2212505f ), new( 11.9059982f, -16.2057514f ),	new( 0, -16.2057514f ), new( -10.5835037f, -14.8827496f ), new( -17.1980019f, -13.5597477f ), new( -21.1665001f, -12.2370014f ), new( -25.1355019f, -9.5909977f ), new( -31.75f, -3.63799858f ), new( -38.3644981f, 6.2840004f ), new( -42.3334999f, 9.59125137f ),	new( -47.625f, 11.5755005f ),
		};

		int count = points.Length;

		// float scale = 0.25f;
		// Vec2 lower = new Vec2( float.MaxValue, float.MaxValue );
		// Vec2 upper = new Vec2( -float.MaxValue, -float.MaxValue );
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i].x = 2.0f * scale * points[i].x;
		//	points[i].y = -scale * points[i].y;

		//	lower = Min(lower, points[i]);
		//	upper = Max(upper, points[i]);
		//}

		// Vec2 center = MulSV(0.5f, Add(lower, upper));
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i] = Sub(points[i], center);
		// }

		// for (int i = 0; i < count / 2; ++i)
		//{
		//	Vec2 temp = points[i];
		//	points[i] = points[count - 1 - i];
		//	points[count - 1 - i] = temp;
		// }

		// Console.Write( Fmt( "{" ) );
		// for (int i = 0; i < count; ++i)
		//{
		//	Console.Write( Fmt( "{%.9g, %.9g}, ", points[i].x, points[i].y ) );
		// }
		// Console.Write( Fmt( "};\n" ) );

		ChainDef chainDef = DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = count;
		chainDef.materials = [m_material];
		chainDef.materialCount = 1;
		chainDef.isLoop = true;

		BodyDef bodyDef = DefaultBodyDef();
		m_groundId = CreateBody( m_worldId, ref bodyDef );

		m_chainId = CreateChain( m_groundId, ref chainDef );
	}

	public void Launch(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -55.0f, y = 13.5f };
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material = m_material;

		if ( m_shapeType == e_circleShape )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			m_shapeId = CreateCircleShape( m_bodyId, ref shapeDef, ref circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			m_shapeId = CreateCapsuleShape( m_bodyId, ref shapeDef, ref capsule );
		}
		else
		{
			float h = 0.5f;
			Polygon box = MakeBox( h, h );
			m_shapeId = CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}

		// toiCalls = 0;
		// toiHitCount = 0;

		m_stepCount = 0;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		string[] shapeTypes = { "Circle", "Capsule", "Box" };
		int shapeType = (int)( m_shapeType );
		if ( ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_shapeType = (int)( shapeType );
			Launch();
		}

		if ( ImGui.SliderFloat( "Friction", ref m_material.friction, 0.0f, 1.0f, "%.2f" ) )
		{
			Shape_SetSurfaceMaterial( m_shapeId, ref m_material );
			Chain_SetAllSurfaceMaterials( m_chainId, ref m_material );
		}

		if ( ImGui.SliderFloat( "Restitution", ref m_material.restitution, 0.0f, 2.0f, "%.1f" ) )
		{
			Shape_SetSurfaceMaterial( m_shapeId, ref m_material );
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Launch" ) )
		{
			Launch();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		DrawLine( m_draw, Pos_zero, ToPos( new( 0.5f, 0.0f ) ), HexColor.colorRed );
		DrawLine( m_draw, Pos_zero, ToPos( new( 0.0f, 0.5f ) ), HexColor.colorGreen );

		// DrawTextLine( "toi calls, hits = %d, %d", toiCalls, toiHitCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new ChainShape( context );
	}

	public BodyId m_groundId;
	public BodyId m_bodyId;
	public ChainId m_chainId;
	public int m_shapeType;
	public ShapeId m_shapeId;
	public SurfaceMaterial m_material;
}

public class ChainSegmentShape : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_boxShape = 0 + 2;

	public ChainSegmentShape( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 1.0f;
		}

		m_bodyId = nullBodyId;
		m_shapeType = e_circleShape;
		m_mutateIndex = 0;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			// Walk right-to-left so the right-perpendicular normal of (point2 - point1) points up.
			for ( int i = 0; i < m_pointCount; ++i )
			{
				float x = 25.0f - 50.0f * i / ( m_pointCount - 1 );
				float y = 1.5f * MathF.Sin( 0.18f * x );
				m_points[i] = new( x, y );
			}

			ShapeDef shapeDef = DefaultShapeDef();
			for ( int i = 0; i < m_segmentCount; ++i )
			{
				ChainSegment chainSegment = default;
				chainSegment.ghost1 = m_points[i];
				chainSegment.segment.point1 = m_points[i + 1];
				chainSegment.segment.point2 = m_points[i + 2];
				chainSegment.ghost2 = m_points[i + 3];
				chainSegment.chainId = -1;
				m_segmentShapes[i] = CreateChainSegmentShape( groundId, ref shapeDef, ref chainSegment );
			}
		}

		Launch();
	}

	public void Launch(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -18.0f, y = 5.0f };
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		if ( m_shapeType == e_circleShape )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };
			CreateCircleShape( m_bodyId, ref shapeDef, ref circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			CreateCapsuleShape( m_bodyId, ref shapeDef, ref capsule );
		}
		else
		{
			Polygon box = MakeSquare( 0.5f );
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}
	}

	public void Mutate(  )
	{
		// Get an index in [1,pointCount - 2]
		// index 0 and pointCount-1 are ghost vertices and are not mutated
		int index = m_mutateIndex + 1;
		Debug.Assert( 1 <= index && index <= m_pointCount - 2 );

		m_mutateIndex += 1;
		if ( m_mutateIndex == m_segmentCount )
		{
			m_mutateIndex = 0;
		}

		m_points[index].y += 0.25f;

		ChainSegment cs = default;
		cs.ghost1 = m_points[index - 1];
		cs.segment.point1 = m_points[index];
		cs.segment.point2 = m_points[index + 1];
		cs.ghost2 = m_points[index + 2];
		cs.chainId = -1;

		Debug.Assert( 0 <= index - 1 && index - 1 < m_segmentCount );
		Shape_SetChainSegment( m_segmentShapes[index - 1], ref cs );

		if ( index - 1 > 0 )
		{
			Debug.Assert( 0 <= index - 2 );
			ChainSegment cs2 = default;
			cs2.ghost1 = m_points[index - 2];
			cs2.segment.point1 = m_points[index - 1];
			cs2.segment.point2 = m_points[index];
			cs2.ghost2 = m_points[index + 1];
			cs2.chainId = -1;
			Debug.Assert( 0 <= index - 2 && index - 2 < m_segmentCount );
			Shape_SetChainSegment( m_segmentShapes[index - 2], ref cs2 );
		}

		if ( index + 1 < m_pointCount - 2 )
		{
			Debug.Assert( index + 3 < m_pointCount );
			ChainSegment cs3 = default;
			cs3.ghost1 = m_points[index];
			cs3.segment.point1 = m_points[index + 1];
			cs3.segment.point2 = m_points[index + 2];
			cs3.ghost2 = m_points[index + 3];
			cs3.chainId = -1;
			Debug.Assert( 0 <= index && index < m_segmentCount );
			Shape_SetChainSegment( m_segmentShapes[index], ref cs3 );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		string[] shapeTypes = { "Circle", "Capsule", "Box" };
		int shapeType = (int)( m_shapeType );
		if ( ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_shapeType = (int)( shapeType );
			Launch();
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Launch" ) )
		{
			Launch();
		}

		if ( ImGui.Button( "Mutate" ) )
		{
			Mutate();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new ChainSegmentShape( context );
	}

	public const int m_segmentCount = 32;
	public const int m_pointCount = m_segmentCount + 3;
	public BodyId m_bodyId;
	public int m_shapeType;
	public ShapeId[] m_segmentShapes = new ShapeId[m_segmentCount];
	public Vec2[] m_points = new Vec2[m_pointCount];
	public int m_mutateIndex;
}

public class CompoundShapes : Sample
{
	public CompoundShapes( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( 50.0f, 0.0f ), point2 = new( -50.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Table 1
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -15.0f, y = 1.0f };
			m_table1Id = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon top = MakeOffsetBox( 3.0f, 0.5f, new( 0.0f, 3.5f ), Rot_identity );
			Polygon leftLeg = MakeOffsetBox( 0.5f, 1.5f, new( -2.5f, 1.5f ), Rot_identity );
			Polygon rightLeg = MakeOffsetBox( 0.5f, 1.5f, new( 2.5f, 1.5f ), Rot_identity );

			CreatePolygonShape( m_table1Id, ref shapeDef, ref top );
			CreatePolygonShape( m_table1Id, ref shapeDef, ref leftLeg );
			CreatePolygonShape( m_table1Id, ref shapeDef, ref rightLeg );
		}

		// Table 2
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -5.0f, y = 1.0f };
			m_table2Id = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon top = MakeOffsetBox( 3.0f, 0.5f, new( 0.0f, 3.5f ), Rot_identity );
			Polygon leftLeg = MakeOffsetBox( 0.5f, 2.0f, new( -2.5f, 2.0f ), Rot_identity );
			Polygon rightLeg = MakeOffsetBox( 0.5f, 2.0f, new( 2.5f, 2.0f ), Rot_identity );

			CreatePolygonShape( m_table2Id, ref shapeDef, ref top );
			CreatePolygonShape( m_table2Id, ref shapeDef, ref leftLeg );
			CreatePolygonShape( m_table2Id, ref shapeDef, ref rightLeg );
		}

		// Spaceship 1
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 5.0f, y = 1.0f };
			m_ship1Id = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Vec2[] vertices = new Vec2[3];

			vertices[0] = new( -2.0f, 0.0f );
			vertices[1] = new( 0.0f, 4.0f / 3.0f );
			vertices[2] = new( 0.0f, 4.0f );
			Hull hull = ComputeHull( vertices, 3 );
			Polygon left = MakePolygon( ref hull, 0.0f );

			vertices[0] = new( 2.0f, 0.0f );
			vertices[1] = new( 0.0f, 4.0f / 3.0f );
			vertices[2] = new( 0.0f, 4.0f );
			hull = ComputeHull( vertices, 3 );
			Polygon right = MakePolygon( ref hull, 0.0f );

			CreatePolygonShape( m_ship1Id, ref shapeDef, ref left );
			CreatePolygonShape( m_ship1Id, ref shapeDef, ref right );
		}

		// Spaceship 2
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 15.0f, y = 1.0f };
			m_ship2Id = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Vec2[] vertices = new Vec2[3];

			vertices[0] = new( -2.0f, 0.0f );
			vertices[1] = new( 1.0f, 2.0f );
			vertices[2] = new( 0.0f, 4.0f );
			Hull hull = ComputeHull( vertices, 3 );
			Polygon left = MakePolygon( ref hull, 0.0f );

			vertices[0] = new( 2.0f, 0.0f );
			vertices[1] = new( -1.0f, 2.0f );
			vertices[2] = new( 0.0f, 4.0f );
			hull = ComputeHull( vertices, 3 );
			Polygon right = MakePolygon( ref hull, 0.0f );

			CreatePolygonShape( m_ship2Id, ref shapeDef, ref left );
			CreatePolygonShape( m_ship2Id, ref shapeDef, ref right );
		}

		m_drawBodyAABBs = false;
	}

	public void Spawn(  )
	{
		// Table 1 obstruction
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = Body_GetPosition( m_table1Id );
			bodyDef.rotation = Body_GetRotation( m_table1Id );
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 4.0f, 0.1f, new( 0.0f, 3.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// Table 2 obstruction
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = Body_GetPosition( m_table2Id );
			bodyDef.rotation = Body_GetRotation( m_table2Id );
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 4.0f, 0.1f, new( 0.0f, 3.0f ), Rot_identity );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// Ship 1 obstruction
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = Body_GetPosition( m_ship1Id );
			bodyDef.rotation = Body_GetRotation( m_ship1Id );
			// bodyDef.gravityScale = 0.0f;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Circle circle = new Circle { center = new( 0.0f, 2.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		// Ship 2 obstruction
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = Body_GetPosition( m_ship2Id );
			bodyDef.rotation = Body_GetRotation( m_ship2Id );
			// bodyDef.gravityScale = 0.0f;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Circle circle = new Circle { center = new( 0.0f, 2.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Intrude" ) )
		{
			Spawn();
		}

		ImGui.Checkbox( "Body AABBs", ref m_drawBodyAABBs );

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_drawBodyAABBs )
		{
			AABB aabb = Body_ComputeBounds( m_table1Id );
			DrawBounds( m_draw, aabb, HexColor.colorYellow );

			aabb = Body_ComputeBounds( m_table2Id );
			DrawBounds( m_draw, aabb, HexColor.colorYellow );

			aabb = Body_ComputeBounds( m_ship1Id );
			DrawBounds( m_draw, aabb, HexColor.colorYellow );

			aabb = Body_ComputeBounds( m_ship2Id );
			DrawBounds( m_draw, aabb, HexColor.colorYellow );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CompoundShapes( context );
	}

	public BodyId m_table1Id;
	public BodyId m_table2Id;
	public BodyId m_ship1Id;
	public BodyId m_ship2Id;
	public bool m_drawBodyAABBs;
}

public class ShapeFilter : Sample
{
	public const ulong GROUND = 0x00000001;
	public const ulong TEAM1 = 0x00000002;
	public const ulong TEAM2 = 0x00000004;
	public const ulong TEAM3 = 0x00000008;
	public const ulong ALL_BITS = ( ~0u );

	public ShapeFilter( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.5f;
			m_context.camera.center = new( 0.0f, 5.0f );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = GROUND;
			shapeDef.filter.maskBits = ALL_BITS;

			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			bodyDef.position = new Pos { x = 0.0f, y = 2.0f };
			m_player1Id = CreateBody( m_worldId, ref bodyDef );

			bodyDef.position = new Pos { x = 0.0f, y = 5.0f };
			m_player2Id = CreateBody( m_worldId, ref bodyDef );

			bodyDef.position = new Pos { x = 0.0f, y = 8.0f };
			m_player3Id = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 2.0f, 1.0f );

			ShapeDef shapeDef = DefaultShapeDef();

			shapeDef.filter.categoryBits = TEAM1;
			shapeDef.filter.maskBits = GROUND | TEAM2 | TEAM3;
			m_shape1Id = CreatePolygonShape( m_player1Id, ref shapeDef, ref box );

			shapeDef.filter.categoryBits = TEAM2;
			shapeDef.filter.maskBits = GROUND | TEAM1 | TEAM3;
			m_shape2Id = CreatePolygonShape( m_player2Id, ref shapeDef, ref box );

			shapeDef.filter.categoryBits = TEAM3;
			shapeDef.filter.maskBits = GROUND | TEAM1 | TEAM2;
			m_shape3Id = CreatePolygonShape( m_player3Id, ref shapeDef, ref box );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.Text( "Player 1 Collides With" );
		{
			Filter filter1 = Shape_GetFilter( m_shape1Id );
			bool team2 = ( filter1.maskBits & TEAM2 ) == TEAM2;
			if ( ImGui.Checkbox( "Team 2##1", ref team2 ) )
			{
				if ( team2 )
				{
					filter1.maskBits |= TEAM2;
				}
				else
				{
					filter1.maskBits &= ~TEAM2;
				}

				Shape_SetFilter( m_shape1Id, filter1 );
			}

			bool team3 = ( filter1.maskBits & TEAM3 ) == TEAM3;
			if ( ImGui.Checkbox( "Team 3##1", ref team3 ) )
			{
				if ( team3 )
				{
					filter1.maskBits |= TEAM3;
				}
				else
				{
					filter1.maskBits &= ~TEAM3;
				}

				Shape_SetFilter( m_shape1Id, filter1 );
			}

			return true;
		}

		ImGui.Separator();

		ImGui.Text( "Player 2 Collides With" );
		{
			Filter filter2 = Shape_GetFilter( m_shape2Id );
			bool team1 = ( filter2.maskBits & TEAM1 ) == TEAM1;
			if ( ImGui.Checkbox( "Team 1##2", ref team1 ) )
			{
				if ( team1 )
				{
					filter2.maskBits |= TEAM1;
				}
				else
				{
					filter2.maskBits &= ~TEAM1;
				}

				Shape_SetFilter( m_shape2Id, filter2 );
			}

			bool team3 = ( filter2.maskBits & TEAM3 ) == TEAM3;
			if ( ImGui.Checkbox( "Team 3##2", ref team3 ) )
			{
				if ( team3 )
				{
					filter2.maskBits |= TEAM3;
				}
				else
				{
					filter2.maskBits &= ~TEAM3;
				}

				Shape_SetFilter( m_shape2Id, filter2 );
			}
		}

		ImGui.Separator();

		ImGui.Text( "Player 3 Collides With" );
		{
			Filter filter3 = Shape_GetFilter( m_shape3Id );
			bool team1 = ( filter3.maskBits & TEAM1 ) == TEAM1;
			if ( ImGui.Checkbox( "Team 1##3", ref team1 ) )
			{
				if ( team1 )
				{
					filter3.maskBits |= TEAM1;
				}
				else
				{
					filter3.maskBits &= ~TEAM1;
				}

				Shape_SetFilter( m_shape3Id, filter3 );
			}

			bool team2 = ( filter3.maskBits & TEAM2 ) == TEAM2;
			if ( ImGui.Checkbox( "Team 2##3", ref team2 ) )
			{
				if ( team2 )
				{
					filter3.maskBits |= TEAM2;
				}
				else
				{
					filter3.maskBits &= ~TEAM2;
				}

				Shape_SetFilter( m_shape3Id, filter3 );
			}
		}
	}

	public override void Step(  )
	{
		base.Step();

		Pos p1 = Body_GetPosition( m_player1Id );
		DrawString( m_draw, m_camera, new( p1.x - 0.5f, p1.y ), HexColor.colorWhite, "player 1" );

		Pos p2 = Body_GetPosition( m_player2Id );
		DrawString( m_draw, m_camera, new( p2.x - 0.5f, p2.y ), HexColor.colorWhite, "player 2" );

		Pos p3 = Body_GetPosition( m_player3Id );
		DrawString( m_draw, m_camera, new( p3.x - 0.5f, p3.y ), HexColor.colorWhite, "player 3" );
	}

	public static Sample Create( SampleContext context )
	{
		return new ShapeFilter( context );
	}

	public BodyId m_player1Id;
	public BodyId m_player2Id;
	public BodyId m_player3Id;

	public ShapeId m_shape1Id;
	public ShapeId m_shape2Id;
	public ShapeId m_shape3Id;
}

public class CustomFilter : Sample
{
	public const int e_count = 10;

	public CustomFilter( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 10.0f;
		}

		// Register custom filter
		World_SetCustomFilterCallback( m_worldId, CustomFilterStatic, this );

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );
			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };

			ShapeDef shapeDefInner = DefaultShapeDef();

			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableCustomFiltering = true;
		Polygon box = MakeSquare( 1.0f );
		float x = -e_count;

		for ( int i = 0; i < e_count; ++i )
		{
			bodyDef.position = new Pos { x = x, y = 5.0f };
			m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );

			shapeDef.userData = ( ( i + 1 ) );
			m_shapeIds[i] = CreatePolygonShape( m_bodyIds[i], ref shapeDef, ref box );
			x += 2.0f;
		}
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "Custom filter disables collision between odd and even shapes" );

		base.Step();

		for ( int i = 0; i < e_count; ++i )
		{
			Pos p = Body_GetPosition( m_bodyIds[i] );
			DrawString( m_draw, m_camera, new( p.x, p.y ), HexColor.colorWhite, "%d", i );
		}
	}

	public bool ShouldCollide( ShapeId shapeIdA, ShapeId shapeIdB )
	{
		object? userDataA = Shape_GetUserData( shapeIdA );
		object? userDataB = Shape_GetUserData( shapeIdB );
		if ( userDataA == null || userDataB == null )
		{
			return true;
		}

		int indexA = (int)( ( userDataA ) );
		int indexB = (int)( ( userDataB ) );

		return ( ( indexA & 1 ) + ( indexB & 1 ) ) != 1;
	}

	public static bool CustomFilterStatic( ShapeId shapeIdA, ShapeId shapeIdB, object? context )
	{
		CustomFilter? customFilter = (CustomFilter?)( context );
		return customFilter.ShouldCollide( shapeIdA, shapeIdB );
	}

	public static Sample Create( SampleContext context )
	{
		return new CustomFilter( context );
	}

	public BodyId[] m_bodyIds = new BodyId[e_count];
	public ShapeId[] m_shapeIds = new ShapeId[e_count];
}

public class Friction : Sample
{
	public Friction( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 14.0f );
			m_context.camera.zoom = 25.0f * 0.6f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.2f;

			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			Polygon box = MakeOffsetBox( 13.0f, 0.25f, new( -4.0f, 22.0f ), MakeRot( -0.25f ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 0.25f, 1.0f, new( 10.5f, 19.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 13.0f, 0.25f, new( 4.0f, 14.0f ), MakeRot( 0.25f ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 0.25f, 1.0f, new( -10.5f, 11.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 13.0f, 0.25f, new( -4.0f, 6.0f ), MakeRot( -0.25f ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			Polygon box = MakeBox( 0.5f, 0.5f );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 25.0f;

			float[] friction = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

			for ( int i = 0; i < 5; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = -15.0f + 4.0f * i, y = 28.0f };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				shapeDef.material.friction = friction[i];
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Friction( context );
	}
}

public class RollingResistance : Sample
{
	public RollingResistance( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 5.0f, 20.0f );
			m_context.camera.zoom = 27.5f;
		}

		m_lift = 0.0f;
		m_resistScale = 0.02f;
		CreateScene();
	}

	public void CreateScene(  )
	{
		Circle circle = new Circle { center = Vec2_zero, radius = 0.5f };

		ShapeDef shapeDef = DefaultShapeDef();

		for ( int i = 0; i < 20; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -40.0f, 2.0f * i ), point2 = new( 40.0f, 2.0f * i + m_lift ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -39.5f, y = 2.0f * i + 0.75f };
			bodyDef.angularVelocity = -10.0f;
			bodyDef.linearVelocity = new Vec2( 5.0f, 0.0f );

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			shapeDef.material.rollingResistance = m_resistScale * i;
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override void Keyboard( int key, int action, int mods )
	{
		switch ( key )
		{
			case GLFW_KEY_1:
				m_lift = 0.0f;
				CreateWorld();
				CreateScene();
				break;

			case GLFW_KEY_2:
				m_lift = 5.0f;
				CreateWorld();
				CreateScene();
				break;

			case GLFW_KEY_3:
				m_lift = -5.0f;
				CreateWorld();
				CreateScene();
				break;

			default:
				base.Keyboard( key, action, mods );
				break;
		}
	}

	public override void Step(  )
	{
		base.Step();

		for ( int i = 0; i < 20; ++i )
		{
			DrawString( m_draw, m_camera, new( -41.5f, 2.0f * i + 1.0f ), HexColor.colorWhite, "%.2f", m_resistScale * i );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new RollingResistance( context );
	}

	public float m_resistScale;
	public float m_lift;
}

public class ConveyorBelt : Sample
{
	public ConveyorBelt( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.0f, 7.5f );
			m_context.camera.zoom = 12.0f;
		}

		// Ground
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		// Platform
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = -5.0f, y = 5.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeRoundedBox( 10.0f, 0.25f, 0.25f );

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.material.friction = 0.8f;
			shapeDefInner.material.tangentSpeed = 2.0f;

			CreatePolygonShape( bodyId, ref shapeDefInner, ref box );
		}

		// Boxes
		ShapeDef shapeDef = DefaultShapeDef();
		Polygon cube = MakeSquare( 0.5f );
		for ( int i = 0; i < 5; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -10.0f + 2.0f * i, y = 7.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			CreatePolygonShape( bodyId, ref shapeDef, ref cube );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new ConveyorBelt( context );
	}
}

public class TangentSpeed : Sample
{
	public TangentSpeed( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 60.0f, -15.0f );
			m_context.camera.zoom = 38.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			// string path = "M 613.8334,185.20833 H 500.06255 L 470.95838,182.5625 444.50004,174.625 418.04171,161.39583 "
			//				   "394.2292,140.22917 h "
			//				   "-13.22916 v 44.97916 H 68.791712 V 0 h -21.16671 v 206.375 l 566.208398,-1e-5 z";

			string path = "m 613.8334,185.20833 -42.33338,0 h -37.04166 l -34.39581,0 -29.10417,-2.64583 -26.45834,-7.9375 " +
							   "-26.45833,-13.22917 -23.81251,-21.16666 h -13.22916 v 44.97916 H 68.791712 V 0 h -21.16671 v " +
							   "206.375 l 566.208398,-1e-5 z";

			Vec2 offset = new Vec2( -47.375002f, 0.25f );

			float scale = 0.2f;
			Vec2[] points = new Vec2[20];
			int count = ParsePath( path, offset, points, 20, scale, true );

			SurfaceMaterial[] materials = new SurfaceMaterial[20];
			for ( int i = 0; i < 20; ++i )
			{
				materials[i].friction = 0.6f;
			}

			materials[0].tangentSpeed = -10.0f;
			materials[0].customColor = (uint)HexColor.colorDarkBlue;
			materials[1].tangentSpeed = -20.0f;
			materials[1].customColor = (uint)HexColor.colorDarkCyan;
			materials[2].tangentSpeed = -30.0f;
			materials[2].customColor = (uint)HexColor.colorDarkGoldenRod;
			materials[3].tangentSpeed = -40.0f;
			materials[3].customColor = (uint)HexColor.colorDarkGray;
			materials[4].tangentSpeed = -50.0f;
			materials[4].customColor = (uint)HexColor.colorDarkGreen;
			materials[5].tangentSpeed = -60.0f;
			materials[5].customColor = (uint)HexColor.colorDarkKhaki;
			materials[6].tangentSpeed = -70.0f;
			materials[6].customColor = (uint)HexColor.colorDarkMagenta;

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;
			chainDef.materials = materials;
			chainDef.materialCount = count;

			CreateChain( groundId, ref chainDef );

			m_friction = 0.6f;
			m_rollingResistance = 0.3f;
		}
	}

	public BodyId DropBall(  )
	{
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 110.0f, y = -30.0f };
		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = m_friction;
		shapeDef.material.rollingResistance = m_rollingResistance;
		CreateCircleShape( bodyId, ref shapeDef, ref circle );
		return bodyId;
	}

	public void Reset(  )
	{
		int count = (int)( m_bodyIds.Count );
		for ( int i = 0; i < count; ++i )
		{
			DestroyBody( m_bodyIds[i] );
		}

		m_bodyIds.Clear();
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Friction", ref m_friction, 0.0f, 2.0f, "%.2f" ) )
		{
			Reset();
		}

		if ( ImGui.SliderFloat( "Rolling Resistance", ref m_rollingResistance, 0.0f, 1.0f, "%.2f" ) )
		{
			Reset();
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( m_stepCount % 25 == 0 && m_bodyIds.Count < m_totalCount && m_context.pause == false )
		{
			BodyId id = DropBall();
			m_bodyIds.Add( id );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new TangentSpeed( context );
	}

	public const int m_totalCount = 200;
	public List<BodyId> m_bodyIds = new();
	public float m_friction;
	public float m_rollingResistance;
}

public class ModifyGeometry : Sample
{
	public ModifyGeometry( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.25f;
			m_context.camera.center = new( 0.0f, 5.0f );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 10.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 4.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeBox( 1.0f, 1.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			m_shapeType = ShapeType.circleShape;
			m_scale = 1.0f;
			m_circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			m_shapeId = CreateCircleShape( bodyId, ref shapeDef, ref m_circle );
		}
	}

	public void UpdateShape(  )
	{
		switch ( m_shapeType )
		{
			case ShapeType.circleShape:
				m_circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f * m_scale };
				Shape_SetCircle( m_shapeId, ref m_circle );
				break;

			case ShapeType.capsuleShape:
				m_capsule = new Capsule { center1 = new( -0.5f * m_scale, 0.0f ), center2 = new( 0.0f, 0.5f * m_scale ), radius = 0.5f * m_scale };
				Shape_SetCapsule( m_shapeId, ref m_capsule );
				break;

			case ShapeType.segmentShape:
				m_segment = new Segment { point1 = new( -0.5f * m_scale, 0.0f ), point2 = new( 0.75f * m_scale, 0.0f ) };
				Shape_SetSegment( m_shapeId, ref m_segment );
				break;

			case ShapeType.polygonShape:
				m_polygon = MakeBox( 0.5f * m_scale, 0.75f * m_scale );
				Shape_SetPolygon( m_shapeId, ref m_polygon );
				break;

			default:
				Debug.Assert( false );
				break;
		}

		BodyId bodyId = Shape_GetBody( m_shapeId );
		Body_UpdateMassFromShapes( bodyId );
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.RadioButton( "Circle", m_shapeType == ShapeType.circleShape ) )
		{
			m_shapeType = ShapeType.circleShape;
			UpdateShape();
		}

		if ( ImGui.RadioButton( "Capsule", m_shapeType == ShapeType.capsuleShape ) )
		{
			m_shapeType = ShapeType.capsuleShape;
			UpdateShape();
		}

		if ( ImGui.RadioButton( "Segment", m_shapeType == ShapeType.segmentShape ) )
		{
			m_shapeType = ShapeType.segmentShape;
			UpdateShape();
		}

		if ( ImGui.RadioButton( "Polygon", m_shapeType == ShapeType.polygonShape ) )
		{
			m_shapeType = ShapeType.polygonShape;
			UpdateShape();
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		if ( ImGui.SliderFloat( "Scale", ref m_scale, 0.1f, 10.0f, "%.2f" ) )
		{
			UpdateShape();
		}
		ImGui.PopItemWidth();

		BodyId bodyId = Shape_GetBody( m_shapeId );
		BodyType bodyType = Body_GetType( bodyId );

		if ( ImGui.RadioButton( "Static", bodyType == BodyType.staticBody ) )
		{
			Body_SetType( bodyId, BodyType.staticBody );
		}

		if ( ImGui.RadioButton( "Kinematic", bodyType == BodyType.kinematicBody ) )
		{
			Body_SetType( bodyId, BodyType.kinematicBody );
		}

		if ( ImGui.RadioButton( "Dynamic", bodyType == BodyType.dynamicBody ) )
		{
			Body_SetType( bodyId, BodyType.dynamicBody );
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new ModifyGeometry( context );
	}

	public ShapeId m_shapeId;
	public ShapeType m_shapeType;
	public float m_scale;

	public Circle m_circle;
	public Capsule m_capsule;
	public Segment m_segment;
	public Polygon m_polygon;
}

public class ChainLink : Sample
{
	public ChainLink( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		Vec2[] points1 = { new( 0.0f, 0.0f ), new( -40.0f, 0.0f ), new( -40.0f, -1.0f ), new( 0.0f, -1.0f ) };
		Vec2[] points2 = { new( 0.0f, -1.0f ), new( 40.0f, -1.0f ), new( 40.0f, 0.0f ), new( 0.0f, 0.0f ) };

		int count1 = points1.Length;
		int count2 = points2.Length;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		{
			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points1;
			chainDef.pointCount = count1;

			// The other chain carries on past both ends of this one
			chainDef.ghost1 = new Vec2( 40.0f, 1.0f );
			chainDef.ghost2 = new Vec2( 40.0f, -1.0f );
			chainDef.isLoop = false;
			CreateChain( groundId, ref chainDef );
		}

		{
			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points2;
			chainDef.pointCount = count2;
			chainDef.ghost1 = new Vec2( -40.0f, -1.0f );
			chainDef.ghost2 = new Vec2( -40.0f, 0.0f );
			chainDef.isLoop = false;
			CreateChain( groundId, ref chainDef );
		}

		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();

		{
			bodyDef.position = new Pos { x = -5.0f, y = 2.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		{
			bodyDef.position = new Pos { x = 0.0f, y = 2.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
		}

		{
			bodyDef.position = new Pos { x = 5.0f, y = 2.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			float h = 0.5f;
			Polygon box = MakeBox( h, h );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public override void Step(  )
	{
		base.Step();

		DrawScreenTextLine( "This shows how to link together two chain shapes" );
	}

	public static Sample Create( SampleContext context )
	{
		return new ChainLink( context );
	}
}

public class RoundedShapes : Sample
{
	public RoundedShapes( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.55f;
			m_context.camera.center = new( 2.0f, 8.0f );
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 20.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			box = MakeOffsetBox( 1.0f, 5.0f, new( 19.0f, 5.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			box = MakeOffsetBox( 1.0f, 5.0f, new( -19.0f, 5.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );
		}

		// Capsule capsule = new Capsule { center1 = new( -0.25f, 0.0f ), center2 = new( 0.25f, 0.0f ), radius = 0.25f };
		// Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.35f };
		// Polygon square = MakeSquare(0.35f);

		// Vec2 points[3] = {{-0.1f, -0.5f}, new( 0.1f, -0.5f ), new( 0.0f, 0.5f )};
		// Hull wedgeHull = ComputeHull(points, 3);
		// Polygon wedge = MakePolygon(ref wedgeHull, 0.0f);

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.rollingResistance = 0.3f;

		float y = 2.0f;
		int xcount = 10, ycount = 10;

		for ( int i = 0; i < ycount; ++i )
		{
			float x = -5.0f;
			for ( int j = 0; j < xcount; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				Polygon poly = RandomPolygon( 0.5f );
				poly.radius = RandomFloatRange( 0.05f, 0.25f );
				CreatePolygonShape( bodyId, ref shapeDef, ref poly );

				x += 1.0f;
			}

			y += 1.0f;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new RoundedShapes( context );
	}
}

public class EllipseShape : Sample
{
	public EllipseShape( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.55f;
			m_context.camera.center = new( 2.0f, 8.0f );
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 20.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			box = MakeOffsetBox( 1.0f, 5.0f, new( 19.0f, 5.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			box = MakeOffsetBox( 1.0f, 5.0f, new( -19.0f, 5.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );
		}

		Vec2[] points = {
			new( 0.0f, -0.25f ), new( 0.0f, 0.25f ), new( 0.05f, 0.075f ), new( -0.05f, 0.075f ), new( 0.05f, -0.075f ), new( -0.05f, -0.075f ),
		};
		Hull diamondHull = ComputeHull( points, 6 );
		Polygon poly = MakePolygon( ref diamondHull, 0.2f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.rollingResistance = 0.2f;

		float y = 2.0f;
		int xCount = 10, yCount = 10;

		for ( int i = 0; i < yCount; ++i )
		{
			float x = -5.0f;
			for ( int j = 0; j < xCount; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref poly );

				x += 1.0f;
			}

			y += 1.0f;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new EllipseShape( context );
	}
}

public class OffsetShapes : Sample
{
	public OffsetShapes( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.55f;
			m_context.camera.center = new( 2.0f, 8.0f );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = -1.0f, y = 1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 1.0f, 1.0f, new( 10.0f, -2.0f ), MakeRot( 0.5f * Pi ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			Capsule capsule = new Capsule { center1 = new( -5.0f, 1.0f ), center2 = new( -4.0f, 1.0f ), radius = 0.25f };
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 13.5f, y = -0.75f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
		}

		{
			Polygon box = MakeOffsetBox( 0.75f, 0.5f, new( 9.0f, 2.0f ), MakeRot( 0.5f * Pi ) );
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public override void Step(  )
	{
		base.Step();

		DrawTransform( m_draw, WorldTransform_identity, 1.0f );
	}

	public static Sample Create( SampleContext context )
	{
		return new OffsetShapes( context );
	}
}

public class Explosion : Sample
{
	public Explosion( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 14.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 0.0f;
		ShapeDef shapeDef = DefaultShapeDef();

		m_referenceAngle = 0.0f;

		WeldJointDef weldDef = DefaultWeldJointDef();
		weldDef.@base.bodyIdA = groundId;
		weldDef.@base.localFrameA.q = MakeRot( m_referenceAngle );
		weldDef.@base.localFrameB.p = Vec2_zero;
		weldDef.angularHertz = 0.5f;
		weldDef.angularDampingRatio = 0.7f;
		weldDef.linearHertz = 0.5f;
		weldDef.linearDampingRatio = 0.7f;

		float r = 8.0f;
		for ( float angle = 0.0f; angle < 360.0f; angle += 30.0f )
		{
			CosSin cosSin = ComputeCosSin( angle * Pi / 180.0f );
			bodyDef.position = new Pos { x = r * cosSin.cosine, y = r * cosSin.sine };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 1.0f, 0.1f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			weldDef.@base.localFrameA.p = ToVec2( bodyDef.position );
			weldDef.@base.bodyIdB = bodyId;

			JointId jointId = CreateWeldJoint( m_worldId, ref weldDef );
			m_jointIds.Add( jointId );
		}

		m_radius = 7.0f;
		m_falloff = 3.0f;
		m_impulse = 10.0f;
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Explode" ) )
		{
			ExplosionDef def = DefaultExplosionDef();
			def.position = Pos_zero;
			def.radius = m_radius;
			def.falloff = m_falloff;
			def.impulsePerLength = m_impulse;
			World_Explode( m_worldId, ref def );
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "radius", ref m_radius, 0.0f, 20.0f, "%.1f" );
		ImGui.SliderFloat( "falloff", ref m_falloff, 0.0f, 20.0f, "%.1f" );
		ImGui.SliderFloat( "impulse", ref m_impulse, -20.0f, 20.0f, "%.1f" );
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			m_referenceAngle += m_context.hertz > 0.0f ? 60.0f * Pi / 180.0f / m_context.hertz : 0.0f;
			m_referenceAngle = UnwindAngle( m_referenceAngle );

			int count = (int)m_jointIds.Count;
			for ( int i = 0; i < count; ++i )
			{
				Transform localFrameA = Joint_GetLocalFrameA( m_jointIds[i] );
				localFrameA.q = MakeRot( m_referenceAngle );
				Joint_SetLocalFrameA( m_jointIds[i], localFrameA );
			}
		}

		base.Step();

		DrawScreenTextLine( "reference angle = %g", m_referenceAngle );

		DrawCircle( m_draw, Pos_zero, m_radius + m_falloff, HexColor.colorBox2DBlue );
		DrawCircle( m_draw, Pos_zero, m_radius, HexColor.colorBox2DYellow );
	}

	public static Sample Create( SampleContext context )
	{
		return new Explosion( context );
	}

	public List<JointId> m_jointIds = new();
	public float m_radius;
	public float m_falloff;
	public float m_impulse;
	public float m_referenceAngle;
}

public class RecreateStatic : Sample
{
	public RecreateStatic( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 3.5f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		ShapeDef shapeDef = DefaultShapeDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		Polygon box = MakeBox( 1.0f, 1.0f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		m_groundId = new BodyId {  };
	}

	public override void Step(  )
	{
		if ( m_groundId.IsNonNull )
		{
			DestroyBody( m_groundId );
			m_groundId = new BodyId {  };
		}

		BodyDef bodyDef = DefaultBodyDef();
		m_groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();

		// Invoke contact creation so that contact points are created immediately
		// on a static body.
		shapeDef.invokeContactCreation = true;

		Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
		CreateSegmentShape( m_groundId, ref shapeDef, ref segment );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new RecreateStatic( context );
	}

	public BodyId m_groundId;
}

public class Wind : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_boxShape = 0 + 2;

	public Wind( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.0f );
			m_context.camera.zoom = 2.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			m_groundId = CreateBody( m_worldId, ref bodyDef );
		}

		m_shapeType = e_capsuleShape;
		m_wind = new Vec2( 6.0f, 0.0f );
		m_drag = 1.0f;
		m_lift = 0.75f;
		m_count = 10;
		m_noise = new Vec2( 0.0f, 0.0f );

		CreateScene();
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_maxCount; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
			}
		}

		float radius = 0.1f;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = radius };
		Capsule capsule = new Capsule { center1 = new( 0.0f, -radius ), center2 = new( 0.0f, radius ), radius = 0.25f * radius };
		Polygon box = MakeBox( 0.25f * radius, 1.25f * radius );

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.@base.bodyIdA = m_groundId;
		jointDef.@base.localFrameA.p = new( 0.0f, 2.0f + radius );
		jointDef.@base.drawScale = 0.1f;
		jointDef.hertz = 0.1f;
		jointDef.dampingRatio = 0.0f;
		jointDef.enableSpring = true;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 20.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 0.5f;
		bodyDef.enableSleep = false;

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = new Pos { x = 0.0f, y = 2.0f - 2.0f * radius * i };
			m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );

			if ( m_shapeType == e_circleShape )
			{
				CreateCircleShape( m_bodyIds[i], ref shapeDef, ref circle );
			}
			else if ( m_shapeType == e_capsuleShape )
			{
				CreateCapsuleShape( m_bodyIds[i], ref shapeDef, ref capsule );
			}
			else
			{
				CreatePolygonShape( m_bodyIds[i], ref shapeDef, ref box );
			}

			jointDef.@base.bodyIdB = m_bodyIds[i];
			jointDef.@base.localFrameB.p = new( 0.0f, radius );
			CreateRevoluteJoint( m_worldId, ref jointDef );

			jointDef.@base.bodyIdA = m_bodyIds[i];
			jointDef.@base.localFrameA.p = new( 0.0f, -radius );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		string[] shapeTypes = { "Circle", "Capsule", "Box" };
		int shapeType = (int)( m_shapeType );
		if ( ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length ) )
		{
			m_shapeType = (int)( shapeType );
			CreateScene();
		}

		ImGui.SliderFloat2( "Wind", ref m_wind.x, -20.0f, 20.0f, "%.1f" );
		ImGui.SliderFloat( "Drag", ref m_drag, 0.0f, 1.0f, "%.2f" );
		ImGui.SliderFloat( "Lift", ref m_lift, 0.0f, 4.0f, "%.2f" );
		if ( ImGui.SliderInt( "Count", ref m_count, 1, m_maxCount, "%d" ) )
		{
			CreateScene();
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			float speed;
			Vec2 direction = GetLengthAndNormalize( out speed, m_wind );
			Vec2 wind = MulSV( speed, Add( direction, m_noise ) );

			for ( int i = 0; i < m_count; ++i )
			{
				ShapeId[] shapeIds = new ShapeId[1];
				int count = Body_GetShapes( m_bodyIds[i], shapeIds, 1 );
				for ( int j = 0; j < count; ++j )
				{
					Shape_ApplyWind( shapeIds[j], wind, m_drag, m_lift, true );
				}
			}

			Vec2 rand = RandomVec2( -0.3f, 0.3f );
			m_noise = Lerp( m_noise, rand, 0.05f );

			DrawLine( m_draw, Pos_zero, ToPos( MulSV( 0.2f, wind ) ), HexColor.colorFuchsia );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new Wind( context );
	}

	public const int m_maxCount = 60;

	public int m_shapeType;
	public Vec2 m_wind;
	public float m_drag;
	public float m_lift;
	public Vec2 m_noise;
	public BodyId m_groundId;
	public BodyId[] m_bodyIds = new BodyId[m_maxCount];
	public int m_count;
}
