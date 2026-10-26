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

public static partial class Helpers_sample_continuous
{
	// extern "C" int toiHitCount;



















	// This sample shows ghost bumps



	// Speculative collision failure case suggested by Dirk Gregorius. This uses
	// a simple fallback scheme to prevent tunneling.






	// This shows that while Box2D uses speculative collision, it does not lead to speculative ghost collisions at small distances



	// This shows that Box2D does not have pixel perfect collision.









	// This shows a fast moving body that uses continuous collision versus static and dynamic bodies.
	// This is achieved by setting the ball body as a bullet*.



	// This shows the importance of secondary collisions in continuous physics.
	// This also shows a difficult setup for the solver with an acute angle.



	// This shows how adjusting the CCD safety factor can engage continuous collision at
	// lower speeds to avoid the slight overlap that happens with discrete collision detection.
	// The best way to run this sample is:
	// 1. press pause (P)
	// 2. restart (R) or press the Drop button
	// 3. then single step (.)
	// Then look at the metrics and overlap. The color of the shape gets darker when it is using
	// continuous collision detection.
}

public class BounceHouse : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_boxShape = 0 + 2;

	public struct HitEvent
	{
		public Pos point;
		public float speed;
		public int stepIndex;
	}

	public BounceHouse( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 0.45f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		{
			Segment segment = new Segment { point1 = new( -10.0f, -10.0f ), point2 = new( 10.0f, -10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( 10.0f, -10.0f ), point2 = new( 10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( 10.0f, 10.0f ), point2 = new( -10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( -10.0f, 10.0f ), point2 = new( -10.0f, -10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_shapeType = e_circleShape;
		m_bodyId = nullBodyId;
		m_enableHitEvents = true;

		Array.Clear( m_hitEvents );

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
		bodyDef.linearVelocity = new Vec2( 10.0f, 20.0f );
		bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
		bodyDef.gravityScale = 0.0f;
		bodyDef.isBullet = true;

		// Circle shapes centered on the body can spin fast without risk of tunnelling.
		bodyDef.allowFastRotation = m_shapeType == e_circleShape;

		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = m_enableHitEvents;

		if ( m_shapeType == e_circleShape )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( m_bodyId, ref shapeDef, ref circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
			CreateCapsuleShape( m_bodyId, ref shapeDef, ref capsule );
		}
		else
		{
			float h = 0.1f;
			Polygon box = MakeBox( 20.0f * h, h );
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
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

		if ( ImGui.Checkbox( "hit events", ref m_enableHitEvents ) )
		{
			Body_EnableHitEvents( m_bodyId, m_enableHitEvents );
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			ref ContactHitEvent @event = ref events.hitEvents[i];
			ref HitEvent e = ref m_hitEvents[0];
			for ( int j = 1; j < 4; ++j )
			{
				if ( m_hitEvents[j].stepIndex < e.stepIndex )
				{
					e = m_hitEvents[j];
				}
			}

			e.point = @event.point;
			e.speed = @event.approachSpeed;
			e.stepIndex = m_stepCount;
		}

		for ( int i = 0; i < 4; ++i )
		{
			ref HitEvent e = ref m_hitEvents[i];
			if ( e.stepIndex > 0 && m_stepCount <= e.stepIndex + 30 )
			{
				DrawCircle( m_draw, e.point, 0.1f, HexColor.colorOrangeRed );
				DrawString( m_draw, m_camera, e.point, HexColor.colorWhite, "%.1f", e.speed );
			}
		}

		if ( m_stepCount == 1000 )
		{
			m_stepCount += 0;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BounceHouse( context );
	}

	public HitEvent[] m_hitEvents = new HitEvent[4];
	public BodyId m_bodyId;
	public int m_shapeType;
	public bool m_enableHitEvents;
}

public class BounceHumans : Sample
{
	public BounceHumans( SampleContext context ) : base( context )
	{
		m_context.camera.center = new( 0.0f, 0.0f );
		m_context.camera.zoom = 12.0f;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.restitution = 1.3f;
		shapeDef.material.friction = 0.1f;

		{
			Segment segment = new Segment { point1 = new( -10.0f, -10.0f ), point2 = new( 10.0f, -10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( 10.0f, -10.0f ), point2 = new( 10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( 10.0f, 10.0f ), point2 = new( -10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			Segment segment = new Segment { point1 = new( -10.0f, 10.0f ), point2 = new( -10.0f, -10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 2.0f };
		shapeDef.material.restitution = 2.0f;
		CreateCircleShape( groundId, ref shapeDef, ref circle );
	}

	public override void Step(  )
	{
		if ( m_humanCount < 5 && m_countDown <= 0.0f )
		{
			float jointFrictionTorque = 0.0f;
			float jointHertz = 1.0f;
			float jointDampingRatio = 0.1f;

			CreateHuman( ref m_humans[m_humanCount], m_worldId, new( 0.0f, 5.0f ), 1.0f, jointFrictionTorque, jointHertz,
						 jointDampingRatio, 1, null, true );
			// Human_SetVelocity( m_humans[m_humanCount], new( 10.0f - 5.0f * m_humanCount, -20.0f + 5.0f * m_humanCount ) );

			m_countDown = 2.0f;
			m_humanCount += 1;
		}

		float timeStep = 1.0f / 60.0f;
		CosSin cs1 = ComputeCosSin( 0.5f * m_time );
		CosSin cs2 = ComputeCosSin( m_time );
		float gravity = 10.0f;
		Vec2 gravityVec = new Vec2( gravity * cs1.sine, gravity * cs2.cosine );
		DrawLine( m_draw, Pos_zero, ToPos( new Vec2( 3.0f * cs1.sine, 3.0f * cs2.cosine ) ), HexColor.colorWhite );
		m_time += timeStep;
		m_countDown -= timeStep;
		World_SetGravity( m_worldId, gravityVec );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BounceHumans( context );
	}

	public Human[] m_humans = new Human[5];
	public int m_humanCount = 0;
	public float m_countDown = 0.0f;
	public float m_time = 0.0f;
}

public class ChainDrop : Sample
{
	public ChainDrop( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 0.35f;
		}

		//
		// World_SetContactTuning( m_worldId, 30.0f, 1.0f, 100.0f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -6.0f };
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		Vec2[] points = { new( -10.0f, -2.0f ), new( 10.0f, -2.0f ), new( 10.0f, 1.0f ), new( -10.0f, 1.0f ) };

		ChainDef chainDef = DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 4;
		chainDef.isLoop = true;

		CreateChain( groundId, ref chainDef );

		m_bodyId = nullBodyId;
		m_yOffset = -0.1f;
		m_speed = -42.0f;

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
		bodyDef.linearVelocity = new Vec2( 0.0f, m_speed );
		bodyDef.position = new Pos { x = 0.0f, y = 10.0f + m_yOffset };
		bodyDef.rotation = MakeRot( 0.5f * Pi );
		bodyDef.motionLocks.angularZ = true;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		m_shapeId = CreateCircleShape( m_bodyId, ref shapeDef, ref circle );

		// Capsule capsule = new Capsule { center1 = new( -0.5f, 0.0f ), center2 = new( 0.5f, 0.0f ), radius = 0.25f };
		// m_shapeId = CreateCapsuleShape( m_bodyId, ref shapeDef, ref capsule );

		// float h = 0.5f;
		// Polygon box = MakeBox( h, h );
		// m_shapeId = CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "Speed", ref m_speed, -100.0f, 0.0f, "%.0f" );
		ImGui.SliderFloat( "Y Offset", ref m_yOffset, -1.0f, 1.0f, "%.1f" );
		ImGui.PopItemWidth();

		if ( ImGui.Button( "Launch" ) )
		{
			Launch();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new ChainDrop( context );
	}

	public BodyId m_bodyId;
	public ShapeId m_shapeId;
	public float m_yOffset;
	public float m_speed;
}

public class ChainSlide : Sample
{
	public ChainSlide( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 15.0f;
		}

		// toiHitCount = 0;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			const int count = 80;
			Vec2[] points = new Vec2[count];

			float w = 2.0f;
			float h = 1.0f;
			float x = 20.0f, y = 0.0f;
			for ( int i = 0; i < 20; ++i )
			{
				points[i] = new( x, y );
				x -= w;
			}

			for ( int i = 20; i < 40; ++i )
			{
				points[i] = new( x, y );
				y += h;
			}

			for ( int i = 40; i < 60; ++i )
			{
				points[i] = new( x, y );
				x += w;
			}

			for ( int i = 60; i < 80; ++i )
			{
				points[i] = new( x, y );
				y -= h;
			}

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			CreateChain( groundId, ref chainDef );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.linearVelocity = new Vec2( 100.0f, 0.0f );
			bodyDef.position = new Pos { x = -19.5f, y = 0.0f + 0.5f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.0f;
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override void Step(  )
	{
		base.Step();

		// DrawTextLine( "toi hits = %d", toiHitCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new ChainSlide( context );
	}
}

public class SegmentSlide : Sample
{
	public SegmentSlide( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 15.0f;
		}

		// toiHitCount = 0;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			segment = new Segment { point1 = new( 40.0f, 0.0f ), point2 = new( 40.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.linearVelocity = new Vec2( 100.0f, 0.0f );
			bodyDef.position = new Pos { x = -20.0f, y = 0.7f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			// shapeDef.friction = 0.0f;
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override void Step(  )
	{
		base.Step();

		// DrawTextLine("toi hits = %d", toiHitCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new SegmentSlide( context );
	}
}

public class SkinnyBox : Sample
{
	public SkinnyBox( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.0f, 5.0f );
			m_context.camera.zoom = 25.0f * 0.25f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.9f;
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			Polygon box = MakeOffsetBox( 0.1f, 1.0f, new( 0.0f, 1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_autoTest = false;
		m_bullet = false;
		m_capsule = false;

		m_bodyId = nullBodyId;
		m_bulletId = nullBodyId;

		Launch();
	}

	public void Launch(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
		}

		if ( m_bulletId.IsNonNull )
		{
			DestroyBody( m_bulletId );
		}

		m_angularVelocity = RandomFloatRange( -50.0f, 50.0f );
		// m_angularVelocity = -30.6695766f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 8.0f };
		bodyDef.angularVelocity = m_angularVelocity;
		bodyDef.linearVelocity = new Vec2( 0.0f, -100.0f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.9f;

		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		if ( m_capsule )
		{
			Capsule capsule = new Capsule { center1 = new( 0.0f, -1.0f ), center2 = new( 0.0f, 1.0f ), radius = 0.1f };
			CreateCapsuleShape( m_bodyId, ref shapeDef, ref capsule );
		}
		else
		{
			Polygon polygon = MakeBox( 2.0f, 0.05f );
			CreatePolygonShape( m_bodyId, ref shapeDef, ref polygon );
		}

		if ( m_bullet )
		{
			Polygon polygon = MakeBox( 0.25f, 0.25f );
			m_x = RandomFloatRange( -1.0f, 1.0f );
			bodyDef.position = new Pos { x = m_x, y = 10.0f };
			bodyDef.linearVelocity = new Vec2( 0.0f, -50.0f );
			m_bulletId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bulletId, ref shapeDef, ref polygon );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.Checkbox( "Capsule", ref m_capsule );

		if ( ImGui.Button( "Launch" ) )
		{
			Launch();
		}

		ImGui.Checkbox( "Auto Test", ref m_autoTest );

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_autoTest && m_stepCount % 60 == 0 )
		{
			Launch();
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SkinnyBox( context );
	}

	public BodyId m_bodyId, m_bulletId;
	public float m_angularVelocity;
	public float m_x;
	public bool m_capsule;
	public bool m_autoTest;
	public bool m_bullet;
}

public class GhostBumps : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_boxShape = 0 + 2;

	public GhostBumps( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.5f, 16.0f );
			m_context.camera.zoom = 25.0f * 0.8f;
		}

		m_groundId = nullBodyId;
		m_bodyId = nullBodyId;
		m_shapeId = nullShapeId;
		m_shapeType = e_circleShape;
		m_round = 0.0f;
		m_friction = 0.2f;
		m_bevel = 0.0f;
		m_useChain = true;

		CreateScene();
		Launch();
	}

	public void CreateScene(  )
	{
		if ( m_groundId.IsNonNull )
		{
			DestroyBody( m_groundId );
		}

		m_shapeId = nullShapeId;

		BodyDef bodyDef = DefaultBodyDef();
		m_groundId = CreateBody( m_worldId, ref bodyDef );

		float m = 1.0f / MathF.Sqrt( 2.0f );
		float mm = 2.0f * ( MathF.Sqrt( 2.0f ) - 1.0f );
		float hx = 4.0f, hy = 0.25f;

		if ( m_useChain )
		{
			Vec2[] points = new Vec2[20];
			points[0] = new( -3.0f * hx, hy );
			points[1] = Add( points[0], new( -2.0f * hx * m, 2.0f * hx * m ) );
			points[2] = Add( points[1], new( -2.0f * hx * m, 2.0f * hx * m ) );
			points[3] = Add( points[2], new( -2.0f * hx * m, 2.0f * hx * m ) );
			points[4] = Add( points[3], new( -2.0f * hy * m, -2.0f * hy * m ) );
			points[5] = Add( points[4], new( 2.0f * hx * m, -2.0f * hx * m ) );
			points[6] = Add( points[5], new( 2.0f * hx * m, -2.0f * hx * m ) );
			points[7] =
				Add( points[6], new( 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ), -2.0f * hx * m - 2.0f * hy * ( 1.0f - m ) ) );
			points[8] = Add( points[7], new( 2.0f * hx + hy * mm, 0.0f ) );
			points[9] = Add( points[8], new( 2.0f * hx, 0.0f ) );
			points[10] = Add( points[9], new( 2.0f * hx + hy * mm, 0.0f ) );
			points[11] =
				Add( points[10], new( 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ), 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ) ) );
			points[12] = Add( points[11], new( 2.0f * hx * m, 2.0f * hx * m ) );
			points[13] = Add( points[12], new( 2.0f * hx * m, 2.0f * hx * m ) );
			points[14] = Add( points[13], new( -2.0f * hy * m, 2.0f * hy * m ) );
			points[15] = Add( points[14], new( -2.0f * hx * m, -2.0f * hx * m ) );
			points[16] = Add( points[15], new( -2.0f * hx * m, -2.0f * hx * m ) );
			points[17] = Add( points[16], new( -2.0f * hx * m, -2.0f * hx * m ) );
			points[18] = Add( points[17], new( -2.0f * hx, 0.0f ) );
			points[19] = Add( points[18], new( -2.0f * hx, 0.0f ) );

			SurfaceMaterial material = default;
			material.friction = m_friction;

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = 20;
			chainDef.isLoop = true;
			chainDef.materials = [material];
			chainDef.materialCount = 1;

			CreateChain( m_groundId, ref chainDef );
		}
		else
		{
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = m_friction;

			Hull hull = default;

			if ( m_bevel > 0.0f )
			{
				float hb = m_bevel;
				Vec2[] vs = { new( hx + hb, hy - 0.05f ),	new( hx, hy ), new( -hx, hy ), new( -hx - hb, hy - 0.05f ), new( -hx - hb, -hy + 0.05f ), new( -hx, -hy ), new( hx, -hy ), new( hx + hb, -hy + 0.05f ) };
				hull = ComputeHull( vs, 8 );
			}
			else
			{
				Vec2[] vs = { new( hx, hy ), new( -hx, hy ), new( -hx, -hy ), new( hx, -hy ) };
				hull = ComputeHull( vs, 4 );
			}

			Transform transform = default;
			float x, y;

			// Left slope
			x = -3.0f * hx - m * hx - m * hy;
			y = hy + m * hx - m * hy;
			transform.q = MakeRot( -0.25f * Pi );

			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}

			x = -2.0f * hx;
			y = 0.0f;
			transform.q = MakeRot( 0.0f );

			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * hx;
			}

			x = 3.0f * hx + m * hx + m * hy;
			y = hy + m * hx - m * hy;
			transform.q = MakeRot( 0.25f * Pi );

			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = new Vec2( x, y );
				Polygon polygon = MakeOffsetPolygon( ref hull, transform.p, transform.q );
				CreatePolygonShape( m_groundId, ref shapeDef, ref polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
		}
	}

	public void Launch(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
			m_shapeId = nullShapeId;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -28.0f, y = 18.0f };
		bodyDef.linearVelocity = new Vec2( 0.0f, 0.0f );
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = m_friction;

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
			float h = 0.5f - m_round;
			Polygon box = MakeRoundedBox( h, 2.0f * h, m_round );
			m_shapeId = CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.Checkbox( "Chain", ref m_useChain ) )
		{
			CreateScene();
		}

		if ( m_useChain == false )
		{
			if ( ImGui.SliderFloat( "Bevel", ref m_bevel, 0.0f, 1.0f, "%.2f" ) )
			{
				CreateScene();
			}
		}

		{
			string[] shapeTypes = { "Circle", "Capsule", "Box" };
			int shapeType = (int)( m_shapeType );
			ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length );
			m_shapeType = (int)( shapeType );
		}

		if ( m_shapeType == e_boxShape )
		{
			ImGui.SliderFloat( "Round", ref m_round, 0.0f, 0.4f, "%.1f" );
		}

		if ( ImGui.SliderFloat( "Friction", ref m_friction, 0.0f, 1.0f, "%.1f" ) )
		{
			if ( m_shapeId.IsNonNull )
			{
				Shape_SetFriction( m_shapeId, m_friction );
			}

			CreateScene();
		}

		if ( ImGui.Button( "Launch" ) )
		{
			Launch();
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new GhostBumps( context );
	}

	public BodyId m_groundId;
	public BodyId m_bodyId;
	public ShapeId m_shapeId;
	public int m_shapeType;
	public float m_round;
	public float m_friction;
	public float m_bevel;
	public bool m_useChain;
}

public class SpeculativeFallback : Sample
{
	public SpeculativeFallback( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.0f, 5.0f );
			m_context.camera.zoom = 25.0f * 0.25f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			Vec2[] points = { new( -2.0f, 4.0f ), new( 2.0f, 4.0f ), new( 2.0f, 4.1f ), new( -0.5f, 4.2f ), new( -2.0f, 4.2f ) };
			Hull hull = ComputeHull( points, 5 );
			Polygon poly = MakePolygon( ref hull, 0.0f );
			CreatePolygonShape( groundId, ref shapeDef, ref poly );
		}

		// Fast moving skinny box. Also testing a large shape offset.
		{
			float offset = 8.0f;
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = offset, y = 12.0f };
			bodyDef.linearVelocity = new Vec2( 0.0f, -100.0f );
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 2.0f, 0.05f, new( -offset, 0.0f ), MakeRot( Pi ) );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SpeculativeFallback( context );
	}
}

public class SpeculativeSliver : Sample
{
	public SpeculativeSliver( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.75f );
			m_context.camera.zoom = 2.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 12.0f };
			bodyDef.linearVelocity = new Vec2( 0.0f, -100.0f );
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Vec2[] points = { new( -2.0f, 0.0f ), new( -1.0f, 0.0f ), new( 2.0f, 0.5f ) };
			Hull hull = ComputeHull( points, 3 );
			Polygon poly = MakePolygon( ref hull, 0.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref poly );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SpeculativeSliver( context );
	}
}

public class SpeculativeGhost : Sample
{
	public SpeculativeGhost( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.75f );
			m_context.camera.zoom = 2.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			Polygon box = MakeOffsetBox( 1.0f, 0.1f, new( 0.0f, 0.9f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			// The speculative distance is 0.02 meters, so this avoid it
			bodyDef.position = new Pos { x = 0.015f, y = 2.515f };
			bodyDef.linearVelocity = new Vec2( 0.1f * 1.25f * m_context.hertz, -0.1f * 1.25f * m_context.hertz );
			bodyDef.gravityScale = 0.0f;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeSquare( 0.25f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SpeculativeGhost( context );
	}
}

public class PixelImperfect : Sample
{
	public PixelImperfect( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 7.0f, 5.0f );
			m_context.camera.zoom = 6.0f;
		}

		float pixelsPerMeter = 30.0f;

		{
			BodyDef block4BodyDef = DefaultBodyDef();
			block4BodyDef.type = BodyType.staticBody;
			block4BodyDef.position = new Pos { x = 175.0f / pixelsPerMeter, y = 150.0f / pixelsPerMeter };
			BodyId block4BodyId = CreateBody( m_worldId, ref block4BodyDef );
			Polygon block4Shape = MakeBox( 20.0f / pixelsPerMeter, 10.0f / pixelsPerMeter );
			ShapeDef block4ShapeDef = DefaultShapeDef();
			block4ShapeDef.material.friction = 0.0f;
			CreatePolygonShape( block4BodyId, ref block4ShapeDef, ref block4Shape );
		}

		{
			BodyDef ballBodyDef = DefaultBodyDef();
			ballBodyDef.type = BodyType.dynamicBody;
			ballBodyDef.position = new Pos { x = 200.0f / pixelsPerMeter, y = 275.0f / pixelsPerMeter };
			ballBodyDef.gravityScale = 0.0f;

			m_ballId = CreateBody( m_worldId, ref ballBodyDef );
			// Ball shape
			// Polygon ballShape = MakeBox( 5.0f / pixelsPerMeter, 5.0f / pixelsPerMeter );
			Polygon ballShape = MakeRoundedBox( 4.0f / pixelsPerMeter, 4.0f / pixelsPerMeter, 0.9f / pixelsPerMeter );
			ShapeDef ballShapeDef = DefaultShapeDef();
			ballShapeDef.material.friction = 0.0f;
			// ballShapeDef.restitution = 1.0f;
			CreatePolygonShape( m_ballId, ref ballShapeDef, ref ballShape );
			Body_SetLinearVelocity( m_ballId, new( 0.0f, -5.0f ) );
			Body_SetMotionLocks( m_ballId, new MotionLocks { linearX = false, linearY = false, angularZ = true } );
		}
	}

	public override void Step(  )
	{
		ContactData data = default;
		Body_GetContactData( m_ballId, new Span<ContactData>( ref data ), 1 );

		Pos p = Body_GetPosition( m_ballId );
		Vec2 v = Body_GetLinearVelocity( m_ballId );
		DrawScreenTextLine( "p.x = %.9f, v.y = %.9f", p.x, v.y );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new PixelImperfect( context );
	}

	public BodyId m_ballId;
}

public class RestitutionThreshold : Sample
{
	public RestitutionThreshold( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 7.0f, 5.0f );
			m_context.camera.zoom = 6.0f;
		}

		float pixelsPerMeter = 30.0f;

		// With the default threshold the ball will not bounce.
		World_SetRestitutionThreshold( m_worldId, 0.1f );

		{
			BodyDef block0BodyDef = DefaultBodyDef();
			block0BodyDef.type = BodyType.staticBody;
			block0BodyDef.position = new Pos { x = 205.0f / pixelsPerMeter, y = 120.0f / pixelsPerMeter };
			block0BodyDef.rotation = MakeRot( 70.0f * 3.14f / 180.0f );
			BodyId block0BodyId = CreateBody( m_worldId, ref block0BodyDef );
			Polygon block0Shape = MakeBox( 50.0f / pixelsPerMeter, 5.0f / pixelsPerMeter );
			ShapeDef block0ShapeDef = DefaultShapeDef();
			block0ShapeDef.material.friction = 0.0f;
			CreatePolygonShape( block0BodyId, ref block0ShapeDef, ref block0Shape );
		}

		{
			// Make a ball
			BodyDef ballBodyDef = DefaultBodyDef();
			ballBodyDef.type = BodyType.dynamicBody;
			ballBodyDef.position = new Pos { x = 200.0f / pixelsPerMeter, y = 250.0f / pixelsPerMeter };
			m_ballId = CreateBody( m_worldId, ref ballBodyDef );

			Circle ballShape = default;
			ballShape.radius = 5.0f / pixelsPerMeter;
			ShapeDef ballShapeDef = DefaultShapeDef();
			ballShapeDef.material.friction = 0.0f;
			ballShapeDef.material.restitution = 1.0f;
			CreateCircleShape( m_ballId, ref ballShapeDef, ref ballShape );

			Body_SetLinearVelocity( m_ballId, new( 0.0f, -2.9f ) );	   // Initial velocity
			Body_SetMotionLocks( m_ballId, new MotionLocks { linearX = false, linearY = false, angularZ = true } ); // Do not rotate a ball
		}
	}

	public override void Step(  )
	{
		ContactData data = default;
		Body_GetContactData( m_ballId, new Span<ContactData>( ref data ), 1 );

		Pos p = Body_GetPosition( m_ballId );
		Vec2 v = Body_GetLinearVelocity( m_ballId );
		DrawScreenTextLine( "p.x = %.9f, v.y = %.9f", p.x, v.y );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new RestitutionThreshold( context );
	}

	public BodyId m_ballId;
}

public class Drop : Sample
{
	public Drop( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.5f );
			m_context.camera.zoom = 3.0f;
			m_context.enableSleep = false;
			m_context.debugDraw.drawJoints = false;
		}

#if false
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			float w = 0.25f;
			int count = 40;

			float x = -0.5f * count * w;
			float h = 0.05f;
			for ( int j = 0; j <= count; ++j )
			{
				Polygon box = MakeOffsetBox( w, h, new( x, -h ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDef, ref box );
				x += w;
			}
		}
#endif

		m_human = new Human {  };
		m_frameSkip = 0;
		m_frameCount = 0;
		m_continuous = true;
		m_speculative = true;

		Scene1();
	}

	public void Clear(  )
	{
		for ( int i = 0; i < m_bodyIds.Count; ++i )
		{
			DestroyBody( m_bodyIds[i] );
		}

		m_bodyIds.Clear();

		if ( m_human.isSpawned )
		{
			DestroyHuman( ref m_human );
		}
	}

	public void CreateGround1(  )
	{
		for ( int i = 0; i < m_groundIds.Count; ++i )
		{
			DestroyBody( m_groundIds[i] );
		}
		m_groundIds.Clear();

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();

		float w = 0.25f;
		int count = 40;
		Segment segment = new Segment { point1 = new( -0.5f * count * w, 0.0f ), point2 = new( 0.5f * count * w, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		m_groundIds.Add( groundId );
	}

	public void CreateGround2(  )
	{
		for ( int i = 0; i < m_groundIds.Count; ++i )
		{
			DestroyBody( m_groundIds[i] );
		}
		m_groundIds.Clear();

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();

		float w = 0.25f;
		int count = 40;

		float x = -0.5f * count * w;
		float h = 0.05f;
		for ( int j = 0; j <= count; ++j )
		{
			Polygon box = MakeOffsetBox( 0.5f * w, h, new( x, 0.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
			x += w;
		}

		m_groundIds.Add( groundId );
	}

	public void CreateGround3(  )
	{
		for ( int i = 0; i < m_groundIds.Count; ++i )
		{
			DestroyBody( m_groundIds[i] );
		}
		m_groundIds.Clear();

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();

		float w = 0.25f;
		int count = 40;
		Segment segment = new Segment { point1 = new( -0.5f * count * w, 0.0f ), point2 = new( 0.5f * count * w, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );
		segment = new Segment { point1 = new( 3.0f, 0.0f ), point2 = new( 3.0f, 8.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		m_groundIds.Add( groundId );
	}

	// ball
	public void Scene1(  )
	{
		Clear();
		CreateGround2();

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 4.0f };
		bodyDef.linearVelocity = new Vec2( 0.0f, -100.0f );

		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.125f };
		CreateCircleShape( bodyId, ref shapeDef, ref circle );

		m_bodyIds.Add( bodyId );
		m_frameCount = 1;
	}

	// ruler
	public void Scene2(  )
	{
		Clear();
		CreateGround1();

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 4.0f };
		bodyDef.rotation = MakeRot( 0.5f * Pi );
		bodyDef.linearVelocity = new Vec2( 0.0f, 0.0f );
		bodyDef.angularVelocity = -0.5f;

		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.75f, 0.01f );
		CreatePolygonShape( bodyId, ref shapeDef, ref box );

		m_bodyIds.Add( bodyId );
		m_frameCount = 1;
	}

	// ragdoll
	public void Scene3(  )
	{
		Clear();
		CreateGround2();

		float jointFrictionTorque = 0.03f;
		float jointHertz = 1.0f;
		float jointDampingRatio = 0.5f;

		CreateHuman( ref m_human, m_worldId, new( 0.0f, 40.0f ), 1.0f, jointFrictionTorque, jointHertz, jointDampingRatio, 1, null,
					 true );

		m_frameCount = 1;
	}

	public void Scene4(  )
	{
		Clear();
		CreateGround3();

		float a = 0.25f;
		Polygon box = MakeSquare( a );

		ShapeDef shapeDef = DefaultShapeDef();

		float offset = 0.01f;

		for ( int i = 0; i < 5; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			float shift = ( i % 2 == 0 ? -offset : offset );
			bodyDef.position = new Pos { x = 2.5f + shift, y = a + 2.0f * a * i };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			m_bodyIds.Add( bodyId );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.125f };
		shapeDef.density = 4.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -7.7f, y = 1.9f };
			bodyDef.linearVelocity = new Vec2( 200.0f, 0.0f );
			bodyDef.isBullet = true;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
			m_bodyIds.Add( bodyId );
		}

		m_frameCount = 1;
	}

	public override void Keyboard( int key, int action, int mods )
	{
		switch ( key )
		{
			case GLFW_KEY_1:
				Scene1();
				break;

			case GLFW_KEY_2:
				Scene2();
				break;

			case GLFW_KEY_3:
				Scene3();
				break;

			case GLFW_KEY_4:
				Scene4();
				break;

			case GLFW_KEY_C:
				Clear();
				m_continuous = !m_continuous;
				break;

			case GLFW_KEY_S:
				m_frameSkip = m_frameSkip > 0 ? 0 : 60;
				break;

			default:
				base.Keyboard( key, action, mods );
				break;
		}
	}

	public override void Step(  )
	{
#if false
		ImGui.SetNextWindowPos( new Vector2( 0.0f, 0.0f ) );
		ImGui.SetNextWindowSize( new Vector2( (float)( m_context.camera.width ), (float)( m_camera.height ) ) );
		ImGui.SetNextWindowBgAlpha( 0.0f );
		ImGui.Begin( "DropBackground", null,
					  ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize |
						  ImGuiWindowFlags.NoScrollbar );

		ref ImDrawList drawList = ref ImGui.GetWindowDrawList();
		string ContinuousText = m_continuous && m_speculative ? "Continuous ON" : "Continuous OFF";
		drawList.AddText( m_largeFont, m_largeFont.FontSize, new( 40.0f, 40.0f ), IM_COL32_WHITE, ContinuousText );

		if ( m_frameSkip > 0 )
		{
			drawList.AddText( m_mediumFont, m_mediumFont.FontSize, new( 40.0f, 40.0f + 64.0f + 20.0f ),
							   IM_COL32( 200, 200, 200, 255 ), "Slow Time" );
		}

		ImGui.End();
#endif

		// if (m_frameCount == 165)
		//{
		//	settings.pause = true;
		//	m_frameSkip = 30;
		// }

		m_context.enableContinuous = m_continuous;

		if ( ( m_frameSkip == 0 || m_frameCount % m_frameSkip == 0 ) && m_context.pause == false )
		{
			base.Step();
		}
		else
		{
			bool pause = m_context.pause;
			m_context.pause = true;
			base.Step();
			m_context.pause = pause;
		}

		m_frameCount += 1;
	}

	public static Sample Create( SampleContext context )
	{
		return new Drop( context );
	}

	public List<BodyId> m_groundIds = new();
	public List<BodyId> m_bodyIds = new();
	public Human m_human;
	public int m_frameSkip;
	public int m_frameCount;
	public bool m_continuous;
	public bool m_speculative;
}

public class Pinball : Sample
{
	public Pinball( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 9.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		m_context.debugDraw.drawJoints = false;

		// Ground body
		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Vec2[] vs = { new( -8.0f, 6.0f ), new( -8.0f, 20.0f ), new( 8.0f, 20.0f ), new( 8.0f, 6.0f ), new( 0.0f, -2.0f ) };

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = vs;
			chainDef.pointCount = 5;
			chainDef.isLoop = true;
			CreateChain( groundId, ref chainDef );
		}

		// Flippers
		{
			Pos p1 = new( -2.0f, 0.0f ), p2 = new( 2.0f, 0.0f );

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.enableSleep = false;

			bodyDef.position = p1;
			BodyId leftFlipperId = CreateBody( m_worldId, ref bodyDef );

			bodyDef.position = p2;
			BodyId rightFlipperId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 1.75f, 0.2f );

			ShapeDef shapeDef = DefaultShapeDef();

			CreatePolygonShape( leftFlipperId, ref shapeDef, ref box );
			CreatePolygonShape( rightFlipperId, ref shapeDef, ref box );

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.localFrameB.p = Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 1000.0f;
			jointDef.enableLimit = true;

			jointDef.motorSpeed = 0.0f;
			jointDef.@base.localFrameA.p = ToVec2( p1 );
			jointDef.@base.bodyIdB = leftFlipperId;
			jointDef.lowerAngle = -30.0f * Pi / 180.0f;
			jointDef.upperAngle = 5.0f * Pi / 180.0f;
			m_leftJointId = CreateRevoluteJoint( m_worldId, ref jointDef );

			jointDef.motorSpeed = 0.0f;
			jointDef.@base.localFrameA.p = ToVec2( p2 );
			jointDef.@base.bodyIdB = rightFlipperId;
			jointDef.lowerAngle = -5.0f * Pi / 180.0f;
			jointDef.upperAngle = 30.0f * Pi / 180.0f;
			m_rightJointId = CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		// Spinners
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -4.0f, y = 17.0f };

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box1 = MakeBox( 1.5f, 0.125f );
			Polygon box2 = MakeBox( 0.125f, 1.5f );

			CreatePolygonShape( bodyId, ref shapeDef, ref box1 );
			CreatePolygonShape( bodyId, ref shapeDef, ref box2 );

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = ToVec2( bodyDef.position );
			jointDef.@base.localFrameB.p = Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 0.1f;
			CreateRevoluteJoint( m_worldId, ref jointDef );

			bodyDef.position = new Pos { x = 4.0f, y = 8.0f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box1 );
			CreatePolygonShape( bodyId, ref shapeDef, ref box2 );
			jointDef.@base.localFrameA.p = ToVec2( bodyDef.position );
			jointDef.@base.bodyIdB = bodyId;
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		// Bumpers
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = -4.0f, y = 8.0f };

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.restitution = 1.5f;

			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 1.0f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			bodyDef.position = new Pos { x = 4.0f, y = 17.0f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		// Ball
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 1.0f, y = 15.0f };
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.isBullet = true;

			m_ballId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.2f };
			CreateCircleShape( m_ballId, ref shapeDef, ref circle );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.Text( "Flipper: press A" );
		return true;
	}

	public override void Step(  )
	{
		base.Step();

		if ( glfwGetKey( m_context.window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			RevoluteJoint_SetTargetMotorSpeed( m_leftJointId, 20.0f );
			RevoluteJoint_SetTargetMotorSpeed( m_rightJointId, -20.0f );
		}
		else
		{
			RevoluteJoint_SetTargetMotorSpeed( m_leftJointId, -10.0f );
			RevoluteJoint_SetTargetMotorSpeed( m_rightJointId, 10.0f );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Pinball( context );
	}

	public JointId m_leftJointId;
	public JointId m_rightJointId;
	public BodyId m_ballId;
}

public class Wedge : Sample
{
	public Wedge( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.5f );
			m_context.camera.zoom = 6.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -4.0f, 8.0f ), point2 = new( 0.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
			segment = new Segment { point1 = new( 0.0f, 0.0f ), point2 = new( 0.0f, 8.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -0.45f, y = 10.75f };
			bodyDef.linearVelocity = new Vec2( 0.0f, -200.0f );

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Circle circle = default;
			circle.radius = 0.3f;
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.2f;
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Wedge( context );
	}
}

public class SafetyFactor : Sample
{
	public SafetyFactor( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 3.0f );
			m_context.camera.zoom = 6.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeBox( 20.0f, 1.0f );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_bodyId = nullBodyId;
		m_extent = 0.5f;
		m_height = 0.3f;
		m_safetyFactor = 0.1f;
		m_overlap = 0.0f;

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
		bodyDef.position = new Pos { x = 0.0f, y = m_height + m_extent };
		bodyDef.safetyFactor = m_safetyFactor;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeSquare( m_extent );
		CreatePolygonShape( m_bodyId, ref shapeDef, ref box );

		m_overlap = 0.0f;
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_didStep )
		{
			// Contacts are computed at the beginning of the step, so the manifold holds the initial separation.
			ContactData data = default;
			int count = Body_GetContactData( m_bodyId, new Span<ContactData>( ref data ), 1 );
			if ( count == 1 )
			{
				for ( int j = 0; j < data.manifold.pointCount; ++j )
				{
					m_overlap = MaxFloat( m_overlap, -data.manifold.points[j].separation );
				}
			}
		}

		DrawLine( m_draw, ToPos( new Vec2( -3.0f, -m_overlap ) ), ToPos( new Vec2( 3.0f, -m_overlap ) ), HexColor.colorRed );

		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
		float motion = timeStep * Length( Body_GetLinearVelocity( m_bodyId ) );

		DrawScreenTextLine( "actual movement = %.3f m", motion );
		DrawScreenTextLine( "fast movement = %.3f m", m_safetyFactor * m_extent );
		DrawScreenTextLine( "overlap = %.4f m", m_overlap );
	}

	public override bool DrawControls(  )
	{
		bool changed = false;

		ImGui.PushItemWidth( 10.0f * ImGui.GetFontSize() );
		changed |= ImGui.SliderFloat( "Height", ref m_height, 0.0f, 2.0f, "%.2f" );
		changed |= ImGui.SliderFloat( "Safety", ref m_safetyFactor, 0.0f, 1.0f, "%.2f" );
		ImGui.PopItemWidth();

		if ( ImGui.Button( "Drop" ) || changed )
		{
			Launch();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new SafetyFactor( context );
	}

	public BodyId m_bodyId;
	public float m_extent;
	public float m_height;
	public float m_safetyFactor;
	public float m_overlap;
}
