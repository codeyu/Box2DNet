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

public static partial class Helpers_sample_stacking
{
	// This sample shows some aspects of Box2D continuous collision:
	// - bullet dynamic bodies which support continuous collision with non-bullet dynamic bodies
	// - prevention of chain reaction tunneling
	// Try disabling continuous collision and firing a bullet. You might see a bullet push a
	// a through the static wall.



	// A simple circle stack that also shows how to collect hit events


















	// From PEEL
}

public class SingleBox : Sample
{
	public SingleBox( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 3.5f;
		}

		float extent = 1.0f;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		float groundWidth = 66.0f * extent;
		ShapeDef shapeDef = DefaultShapeDef();
		// shapeDef.friction = 0.5f;

		Segment segment = new Segment { point1 = new( -0.5f * 2.0f * groundWidth, 0.0f ), point2 = new( 0.5f * 2.0f * groundWidth, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );
		bodyDef.type = BodyType.dynamicBody;

		Polygon box = MakeBox( extent, extent );
		bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
		bodyDef.linearVelocity = new Vec2( 5.0f, 0.0f );
		m_bodyId = CreateBody( m_worldId, ref bodyDef );
		CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
	}

	public override void Step(  )
	{
		base.Step();

		// DrawCircle({0.0f, 2.0f}, 1.0f, HexColor.colorWhite);

		Pos position = Body_GetPosition( m_bodyId );
		DrawScreenTextLine( "(x, y) = (%.2g, %.2g)", position.x, position.y );
	}

	public static Sample Create( SampleContext context )
	{
		return new SingleBox( context );
	}

	public BodyId m_bodyId;
}

public class TiltedStack : Sample
{
	public TiltedStack( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 7.5f, 7.5f );
			m_context.camera.zoom = 20.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Polygon boxInner = MakeBox( 1000.0f, 1.0f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
		}

		for ( int i = 0; i < m_rows * m_columns; ++i )
		{
			m_bodies[i] = nullBodyId;
		}

		Polygon box = MakeRoundedBox( 0.45f, 0.45f, 0.05f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.3f;

		float offset = 0.2f;
		float dx = 5.0f;
		float xroot = -0.5f * dx * ( m_columns - 1.0f );

		for ( int j = 0; j < m_columns; ++j )
		{
			float x = xroot + j * dx;

			for ( int i = 0; i < m_rows; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;

				int n = j * m_rows + i;

				bodyDef.position = new Pos { x = x + offset * i, y = 0.5f + 1.0f * i };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				m_bodies[n] = bodyId;

				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new TiltedStack( context );
	}

	public const int m_columns = 10;
	public const int m_rows = 10;

	public BodyId[] m_bodies = new BodyId[m_rows * m_columns];
}

public class VerticalStack : Sample
{
	public const int e_maxColumns = 10;
	public const int e_maxRows = 160;
	public const int e_maxBullets = 8;

	public const int e_circleShape = 0;
	public const int e_boxShape = 0 + 1;

	public VerticalStack( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( -7.0f, 9.0f );
			m_context.camera.zoom = 14.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			Segment segment = new Segment { point1 = new( 10.0f, 0.0f ), point2 = new( 10.0f, 80.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			segment = new Segment { point1 = new( -100.0f, 0.0f ), point2 = new( 100.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = nullBodyId;
		}

		for ( int i = 0; i < e_maxBullets; ++i )
		{
			m_bullets[i] = nullBodyId;
		}

		m_shapeType = e_boxShape;
		m_rowCount = 12;
		m_columnCount = 1;
		m_bulletCount = 1;
		m_bulletType = e_circleShape;

		CreateStacks();
	}

	public void CreateStacks(  )
	{
		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( m_bodies[i].IsNonNull )
			{
				B2.DestroyBody( m_bodies[i] );
				m_bodies[i] = nullBodyId;
			}
		}

		Circle circle = default;
		circle.radius = 0.5f;

		Polygon box = MakeRoundedBox( 0.45f, 0.45f, 0.05f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.3f;

		float offset;

		if ( m_shapeType == e_circleShape )
		{
			offset = 0.0f;
		}
		else
		{
			offset = 0.01f;
		}

		float dx = -3.0f;
		float xroot = 8.0f;

		for ( int j = 0; j < m_columnCount; ++j )
		{
			float x = xroot + j * dx;

			for ( int i = 0; i < m_rowCount; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;

				int n = j * m_rowCount + i;

				float shift = ( i % 2 == 0 ? -offset : offset );
				bodyDef.position = new Pos { x = x + shift, y = 0.5f + 1.0f * i };
				// bodyDef.position = new( x + shift, 1.0f + 1.51f * i );
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				m_bodies[n] = bodyId;

				if ( m_shapeType == e_circleShape )
				{
					CreateCircleShape( bodyId, ref shapeDef, ref circle );
				}
				else
				{
					CreatePolygonShape( bodyId, ref shapeDef, ref box );
				}
			}
		}
	}

	public void DestroyBody(  )
	{
		for ( int j = 0; j < m_columnCount; ++j )
		{
			for ( int i = 0; i < m_rowCount; ++i )
			{
				int n = j * m_rowCount + i;

				if ( m_bodies[n].IsNonNull )
				{
					B2.DestroyBody( m_bodies[n] );
					m_bodies[n] = nullBodyId;
					break;
				}
			}
		}
	}

	public void DestroyBullets(  )
	{
		for ( int i = 0; i < e_maxBullets; ++i )
		{
			BodyId bullet = m_bullets[i];

			if ( bullet.IsNonNull )
			{
				B2.DestroyBody( bullet );
				m_bullets[i] = nullBodyId;
			}
		}
	}

	public void FireBullets(  )
	{
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };
		Polygon box = MakeBox( 0.25f, 0.25f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 4.0f;

		for ( int i = 0; i < m_bulletCount; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -26.7f - i, y = 6.0f };
			float speed = RandomFloatRange( 200.0f, 300.0f );
			bodyDef.linearVelocity = new Vec2( speed, 0.0f );
			bodyDef.isBullet = true;

			BodyId bullet = CreateBody( m_worldId, ref bodyDef );

			if ( m_bulletType == e_boxShape )
			{
				CreatePolygonShape( bullet, ref shapeDef, ref box );
			}
			else
			{
				CreateCircleShape( bullet, ref shapeDef, ref circle );
			}
			Debug.Assert( m_bullets[i].IsNull );
			m_bullets[i] = bullet;
		}
	}

	public override void Keyboard( int key, int action, int mods )
	{
		bool consumed = false;

		switch ( key )
		{
			case 'B':
				FireBullets();
				consumed = true;
				break;

			default:
				break;
		}

		if ( consumed == false )
		{
			base.Keyboard( key, action, mods );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		string[] shapeTypes = { "Circle", "Box" };

		int shapeType = (int)( m_shapeType );
		changed = changed || ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length );
		m_shapeType = (int)( shapeType );

		changed = changed || ImGui.SliderInt( "Rows", ref m_rowCount, 1, e_maxRows );
		changed = changed || ImGui.SliderInt( "Columns", ref m_columnCount, 1, e_maxColumns );

		ImGui.SliderInt( "Bullets", ref m_bulletCount, 1, e_maxBullets );

		int bulletType = (int)( m_bulletType );
		ImGui.Combo( "Bullet Shape", ref bulletType, shapeTypes, shapeTypes.Length );
		m_bulletType = (int)( bulletType );

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Fire Bullets" ) )
		{
			DestroyBullets();
			FireBullets();
		}

		if ( ImGui.Button( "Destroy Body" ) )
		{
			DestroyBody();
		}

		changed = changed || ImGui.Button( "Reset Stack" );

		if ( changed )
		{
			DestroyBullets();
			CreateStacks();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new VerticalStack( context );
	}

	public BodyId[] m_bullets = new BodyId[e_maxBullets];
	public BodyId[] m_bodies = new BodyId[e_maxRows * e_maxColumns];
	public int m_columnCount;
	public int m_rowCount;
	public int m_bulletCount;
	public int m_shapeType;
	public int m_bulletType;
}

public class CircleStack : Sample
{
	public struct Event
	{
		public int indexA, indexB;

		public Event( int indexA, int indexB )
		{
			this.indexA = indexA;
			this.indexB = indexB;
		}
	}

	public CircleStack( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 6.0f;
		}

		int shapeIndex = 0;

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.userData = ( ( shapeIndex ) );
			shapeIndex += 1;

			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		World_SetGravity( m_worldId, new( 0.0f, -20.0f ) );
		World_SetContactTuning( m_worldId, 0.25f * 360.0f, 10.0f, 3.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		Circle circle = default;
		circle.radius = 0.5f;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		// shapeDef.rollingResistance = 0.2f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.8f;

		float y = 0.75f;

		for ( int i = 0; i < 4; ++i )
		{
			bodyDef.position.y = y;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			shapeDef.userData = ( ( shapeIndex ) );
			shapeDef.density = 1.0f + 4.0f * i;
			shapeIndex += 1;
			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			y += 1.25f;
		}
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			ref ContactHitEvent @event = ref events.hitEvents[i];
			object? userDataA = Shape_GetUserData( @event.shapeIdA );
			object? userDataB = Shape_GetUserData( @event.shapeIdB );
			int indexA = (int)( ( userDataA ) );
			int indexB = (int)( ( userDataB ) );

			DrawPoint( m_draw, @event.point, 10.0f, HexColor.colorWhite );

			m_events.Add( new( indexA, indexB ) );
		}

		int eventCount = (int)m_events.Count;
		for ( int i = 0; i < eventCount; ++i )
		{
			DrawScreenTextLine( "%d, %d", m_events[i].indexA, m_events[i].indexB );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CircleStack( context );
	}

	public List<Event> m_events = new();
}

public class CapsuleStack : Sample
{
	public struct Event
	{
		public int indexA, indexB;
	}

	public CapsuleStack( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 6.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Polygon polygon = MakeBox( 10.0f, 1.0f );
			CreatePolygonShape( groundId, ref shapeDefInner, ref polygon );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		float a = 0.25f;
		Capsule capsule = new Capsule { center1 = new( -4.0f * a, 0.0f ), center2 = new( 4.0f * a, 0.0f ), radius = a };

		ShapeDef shapeDef = DefaultShapeDef();

		// rolling resistance increases stacking stability
		// shapeDef.rollingResistance = 0.2f;

		float y = 2.0f * a;

		for ( int i = 0; i < 20; ++i )
		{
			bodyDef.position.y = y;
			// bodyDef.position.x += ( i & 1 ) == 1 ? -0.5f * a : 0.5f * a;
			// bodyDef.linearVelocity = new( 0.0f, -10.0f );
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

			y += 3.0f * a;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CapsuleStack( context );
	}
}

public class Cliff : Sample
{
	public Cliff( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 0.5f;
			m_context.camera.center = new( 0.0f, 5.0f );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 100.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			Segment segment = new Segment { point1 = new( -14.0f, 4.0f ), point2 = new( -8.0f, 4.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			box = MakeOffsetBox( 3.0f, 0.5f, new( 0.0f, 4.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			Capsule capsule = new Capsule { center1 = new( 8.5f, 4.0f ), center2 = new( 13.5f, 4.0f ), radius = 0.5f };
			CreateCapsuleShape( groundId, ref shapeDef, ref capsule );
		}

		m_flip = false;

		for ( int i = 0; i < 9; ++i )
		{
			m_bodyIds[i] = nullBodyId;
		}

		CreateBodies();
	}

	public void CreateBodies(  )
	{
		for ( int i = 0; i < 9; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				B2.DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
			}
		}

		float sign = m_flip ? -1.0f : 1.0f;

		Capsule capsule = new Capsule { center1 = new( -0.25f, 0.0f ), center2 = new( 0.25f, 0.0f ), radius = 0.25f };
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		Polygon square = MakeSquare( 0.5f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		{
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.01f;
			bodyDef.linearVelocity = new Vec2( 2.0f * sign, 0.0f );

			float offset = m_flip ? -4.0f : 0.0f;

			bodyDef.position = new Pos { x = -9.0f + offset, y = 4.25f };
			m_bodyIds[0] = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( m_bodyIds[0], ref shapeDef, ref capsule );

			bodyDef.position = new Pos { x = 2.0f + offset, y = 4.75f };
			m_bodyIds[1] = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( m_bodyIds[1], ref shapeDef, ref capsule );

			bodyDef.position = new Pos { x = 13.0f + offset, y = 4.75f };
			m_bodyIds[2] = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( m_bodyIds[2], ref shapeDef, ref capsule );
		}

		{
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.01f;
			bodyDef.linearVelocity = new Vec2( 2.5f * sign, 0.0f );

			bodyDef.position = new Pos { x = -11.0f, y = 4.5f };
			m_bodyIds[3] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[3], ref shapeDef, ref square );

			bodyDef.position = new Pos { x = 0.0f, y = 5.0f };
			m_bodyIds[4] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[4], ref shapeDef, ref square );

			bodyDef.position = new Pos { x = 11.0f, y = 5.0f };
			m_bodyIds[5] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[5], ref shapeDef, ref square );
		}

		{
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.2f;
			bodyDef.linearVelocity = new Vec2( 1.5f * sign, 0.0f );

			float offset = m_flip ? 4.0f : 0.0f;

			bodyDef.position = new Pos { x = -13.0f + offset, y = 4.5f };
			m_bodyIds[6] = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( m_bodyIds[6], ref shapeDef, ref circle );

			bodyDef.position = new Pos { x = -2.0f + offset, y = 5.0f };
			m_bodyIds[7] = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( m_bodyIds[7], ref shapeDef, ref circle );

			bodyDef.position = new Pos { x = 9.0f + offset, y = 5.0f };
			m_bodyIds[8] = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( m_bodyIds[8], ref shapeDef, ref circle );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Flip" ) )
		{
			m_flip = !m_flip;
			CreateBodies();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new Cliff( context );
	}

	public BodyId[] m_bodyIds = new BodyId[9];
	public bool m_flip;
}

public class Arch : Sample
{
	public Arch( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 25.0f * 0.35f;
		}

		Vec2[] ps1 = { new( 16.0f, 0.0f ), new( 14.93803712795643f, 5.133601056842984f ), new( 13.79871746027416f, 10.24928069555078f ), new( 12.56252963284711f, 15.34107019122473f ), new( 11.20040987372525f, 20.39856541571217f ), new( 9.66521217819836f, 25.40369899225096f ), new( 7.87179930638133f, 30.3179337000085f ), new( 5.635199558196225f, 35.03820717801641f ),
						  new( 2.405937953536585f, 39.09554102558315f ) };

		Vec2[] ps2 = { new( 24.0f, 0.0f ), new( 22.33619528222415f, 6.02299846205841f ), new( 20.54936888969905f, 12.00964361211476f ), new( 18.60854610798073f, 17.9470321677465f ), new( 16.46769273811807f, 23.81367936585418f ), new( 14.05325025774858f, 29.57079353071012f ), new( 11.23551045834022f, 35.13775818285372f ), new( 7.752568160730571f, 40.30450679009583f ),
						  new( 3.016931552701656f, 44.28891593799322f ) };

		float scale = 0.25f;
		for ( int i = 0; i < 9; ++i )
		{
			ps1[i] = MulSV( scale, ps1[i] );
			ps2[i] = MulSV( scale, ps2[i] );
		}

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.6f;

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );
			Segment segment = new Segment { point1 = new( -100.0f, 0.0f ), point2 = new( 100.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		for ( int i = 0; i < 8; ++i )
		{
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			Vec2[] ps = { ps1[i], ps2[i], ps2[i + 1], ps1[i + 1] };
			Hull hull = ComputeHull( ps, 4 );
			Polygon polygon = MakePolygon( ref hull, 0.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		}

		for ( int i = 0; i < 8; ++i )
		{
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			Vec2[] ps = { new( -ps2[i].x, ps2[i].y ), new( -ps1[i].x, ps1[i].y ), new( -ps1[i + 1].x, ps1[i + 1].y ),
							 new( -ps2[i + 1].x, ps2[i + 1].y ) };
			Hull hull = ComputeHull( ps, 4 );
			Polygon polygon = MakePolygon( ref hull, 0.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		}

		{
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			Vec2[] ps = { ps1[8], ps2[8], new( -ps2[8].x, ps2[8].y ), new( -ps1[8].x, ps1[8].y ) };
			Hull hull = ComputeHull( ps, 4 );
			Polygon polygon = MakePolygon( ref hull, 0.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		}

		for ( int i = 0; i < 4; ++i )
		{
			Polygon box = MakeBox( 2.0f, 0.5f );
			bodyDef.position = new Pos { x = 0.0f, y = 0.5f + ps2[8].y + 1.0f * i };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Arch( context );
	}
}

public class DoubleDomino : Sample
{
	public DoubleDomino( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 4.0f );
			m_context.camera.zoom = 25.0f * 0.25f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			Polygon boxInner = MakeBox( 100.0f, 1.0f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDefInner, ref boxInner );
		}

		Polygon box = MakeBox( 0.125f, 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.6f;
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		int count = 15;
		float x = -0.5f * count;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = new Pos { x = x, y = 0.5f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
			if ( i == 0 )
			{
				Body_ApplyLinearImpulse( bodyId, new Vec2( 0.2f, 0.0f ), new Pos( x, 1.0f ), true );
			}

			x += 1.0f;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new DoubleDomino( context );
	}
}

public class Confined : Sample
{
	public Confined( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Capsule capsule = default;
			capsule = new Capsule { center1 = new( -10.5f, 0.0f ), center2 = new( 10.5f, 0.0f ), radius = 0.5f };
			CreateCapsuleShape( groundId, ref shapeDefInner, ref capsule );
			capsule = new Capsule { center1 = new( -10.5f, 0.0f ), center2 = new( -10.5f, 20.5f ), radius = 0.5f };
			CreateCapsuleShape( groundId, ref shapeDefInner, ref capsule );
			capsule = new Capsule { center1 = new( 10.5f, 0.0f ), center2 = new( 10.5f, 20.5f ), radius = 0.5f };
			CreateCapsuleShape( groundId, ref shapeDefInner, ref capsule );
			capsule = new Capsule { center1 = new( -10.5f, 20.5f ), center2 = new( 10.5f, 20.5f ), radius = 0.5f };
			CreateCapsuleShape( groundId, ref shapeDefInner, ref capsule );
		}

		m_row = 0;
		m_column = 0;
		m_count = 0;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 0.0f;

		ShapeDef shapeDef = DefaultShapeDef();
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

		while ( m_count < m_maxCount )
		{
			m_row = 0;
			for ( int i = 0; i < m_gridCount; ++i )
			{
				float x = -8.75f + m_column * 18.0f / m_gridCount;
				float y = 1.5f + m_row * 18.0f / m_gridCount;

				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreateCircleShape( bodyId, ref shapeDef, ref circle );

				m_count += 1;
				m_row += 1;
			}
			m_column += 1;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Confined( context );
	}

	public const int m_gridCount = 25;
	public const int m_maxCount = m_gridCount * m_gridCount;
	public int m_row;
	public int m_column;
	public int m_count;
}

public class CardHouse : Sample
{
	public CardHouse( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.75f, 0.9f );
			m_context.camera.zoom = 25.0f * 0.05f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -2.0f };
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.7f;

		Polygon groundBox = MakeBox( 40.0f, 2.0f );
		CreatePolygonShape( groundId, ref shapeDef, ref groundBox );

		float cardHeight = 0.2f;
		float cardThickness = 0.001f;

		float angle0 = 25.0f * Pi / 180.0f;
		float angle1 = -25.0f * Pi / 180.0f;
		float angle2 = 0.5f * Pi;

		Polygon cardBox = MakeBox( cardThickness, cardHeight );
		bodyDef.type = BodyType.dynamicBody;

		int Nb = 5;
		float z0 = 0.0f;
		float y = cardHeight - 0.02f;
		while ( Nb != 0 )
		{
			float z = z0;
			for ( int i = 0; i < Nb; i++ )
			{
				if ( i != Nb - 1 )
				{
					bodyDef.position = new Pos { x = z + 0.25f, y = y + cardHeight - 0.015f };
					bodyDef.rotation = MakeRot( angle2 );
					BodyId bodyIdInner = CreateBody( m_worldId, ref bodyDef );
					CreatePolygonShape( bodyIdInner, ref shapeDef, ref cardBox );
				}

				bodyDef.position = new Pos { x = z, y = y };
				bodyDef.rotation = MakeRot( angle1 );
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref cardBox );

				z += 0.175f;

				bodyDef.position = new Pos { x = z, y = y };
				bodyDef.rotation = MakeRot( angle0 );
				bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref cardBox );

				z += 0.175f;
			}
			y += cardHeight * 2.0f - 0.03f;
			z0 += 0.175f;
			Nb--;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CardHouse( context );
	}
}
