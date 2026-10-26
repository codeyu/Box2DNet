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

public static partial class Helpers_sample_restitution
{
	public static void ComputeEnergy( WorldId worldId, ReadOnlySpan<BodyId> bodyIds, int count, ref float linear, ref float angular, ref float potential )
	{
		Vec2 gravity = World_GetGravity( worldId );

		float linearSum = 0.0f;
		float angularSum = 0.0f;
		float potentialSum = 0.0f;

		for ( int i = 0; i < count; ++i )
		{
			MassData massData = Body_GetMassData( bodyIds[i] );
			Vec2 v = Body_GetLinearVelocity( bodyIds[i] );
			float w = Body_GetAngularVelocity( bodyIds[i] );

			linearSum += 0.5f * massData.mass * Dot( v, v );
			angularSum += 0.5f * massData.rotationalInertia * w * w;
			potentialSum -= massData.mass * Dot( gravity, ToVec2( Body_GetWorldCenter( bodyIds[i] ) ) );
		}

		linear = linearSum;
		angular = angularSum;
		potential = potentialSum;
	}

	// Restitution is approximate since Box2D uses speculative collision



	// Tests how a single bouncing box behaves.






	// Similar to MeasureSupportedBounce unit test.



	// A bouncy ball landing on a resting body. Three balls fall together from the same height onto bare
	// ground, one crate and two crates. Without restitution propagation the restitution stage only
	// visits the ball's own contact, so the rebound is shared with the crate, which is driven into the
	// ground and pushed back the next step. With propagation the ground contact under the crate can react
	// in the restitution sover.



	// Similar to MeasureFlatBounce and SpinTest unit tests.






	// Based on https://github.com/erincatto/box2d/discussions/957
}

public class VaryingRestitution : Sample
{
	public const int e_circleShape = 0;
	public const int e_boxShape = 0 + 1;

	public VaryingRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 4.0f, 17.0f );
			m_context.camera.zoom = 27.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			float h = 1.0f * m_count;
			Segment segment = new Segment { point1 = new( -h, 0.0f ), point2 = new( h, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_shapeType = e_circleShape;

		CreateBodies();
	}

	public void CreateBodies(  )
	{
		for ( int i = 0; i < m_count; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
			}
		}

		Circle circle = default;
		circle.radius = 0.5f;

		Polygon box = MakeBox( 0.5f, 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 0.0f;
		shapeDef.material.friction = 0.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		float dr = 1.0f / ( m_count > 1 ? m_count - 1 : 1 );
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			string buffer = "";
			buffer = Fmt( "%.2f", shapeDef.material.restitution );
			bodyDef.name = buffer;
			bodyDef.position = new Pos { x = x, y = 40.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			m_bodyIds[i] = bodyId;

			if ( m_shapeType == e_circleShape )
			{
				CreateCircleShape( bodyId, ref shapeDef, ref circle );
			}
			else
			{
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}

			shapeDef.material.restitution += dr;
			x += dx;
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

		ImGui.PopItemWidth();

		changed = changed || ImGui.Button( "Reset" );

		if ( changed )
		{
			CreateBodies();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float h = 1.0f * m_count;
		DrawLine( m_draw, new( -h, 40.5f ), new( h, 40.5f ), HexColor.colorRed );
	}

	public static Sample Create( SampleContext context )
	{
		return new VaryingRestitution( context );
	}

	public const int m_count = 40;

	public BodyId[] m_bodyIds = new BodyId[m_count];
	public int m_shapeType;
}

public class SingleBoxRestitution : Sample
{
	public SingleBoxRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 8.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			Segment segment = new Segment { point1 = new( -4.0f, 0.0f ), point2 = new( 4.0f, 0.0f ) };
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		m_height = 5.0f;
		m_maxY = 5.0f;

		Polygon box = MakeBox( 0.5f, 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = true;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = m_height };
		bodyDef.safetyFactor = 0.01f;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );
		CreatePolygonShape( m_bodyId, ref shapeDef, ref box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyId ), 1, ref linear, ref angular, ref potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	public override void Step(  )
	{
		base.Step();

		DrawLine( m_draw, new( -4.0f, m_height + 0.5f ), new( 4.0f, m_height + 0.5f ), HexColor.colorRed );

		ContactEvents events = World_GetContactEvents( m_worldId );

		Pos p = Body_GetPosition( m_bodyId );
		if ( events.hitCount == 1 )
		{
			m_maxY = (float)p.y;
		}
		else
		{
			m_maxY = MaxFloat( m_maxY, (float)p.y );
		}

		DrawScreenTextLine( "maxY = %.2f", m_maxY );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyId ), 1, ref linear, ref angular, ref potential );
		float total = linear + angular + potential;
		m_peakEnergy = MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	public static Sample Create( SampleContext context )
	{
		return new SingleBoxRestitution( context );
	}

	public BodyId m_bodyId;
	public float m_maxY;
	public float m_startEnergy = 0.0f;
	public float m_peakEnergy = 0.0f;
	public float m_height;
}

public class SingleCircleRestitution : Sample
{
	public SingleCircleRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 8.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			Segment segment = new Segment { point1 = new( -4.0f, 0.0f ), point2 = new( 4.0f, 0.0f ) };
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		Circle circle = new Circle { center = Vec2_zero, radius = 0.5f };

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = true;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 10.0f };
		bodyDef.safetyFactor = 0.01f;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );
		CreateCircleShape( m_bodyId, ref shapeDef, ref circle );

		m_maxY = bodyDef.position.y;

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyId ), 1, ref linear, ref angular, ref potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );

		Pos p = Body_GetPosition( m_bodyId );
		if ( events.hitCount == 1 )
		{
			m_maxY = (float)p.y;
		}
		else
		{
			m_maxY = MaxFloat( m_maxY, (float)p.y );
		}

		DrawLine( m_draw, new( -4.0f, 10.5f ), new( 4.0f, 10.5f ), HexColor.colorRed );

		DrawScreenTextLine( "maxY = %.2f", m_maxY );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyId ), 1, ref linear, ref angular, ref potential );
		float total = linear + angular + potential;
		m_peakEnergy = MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	public static Sample Create( SampleContext context )
	{
		return new SingleCircleRestitution( context );
	}

	public BodyId m_bodyId;
	public float m_maxY;
	public float m_startEnergy = 0.0f;
	public float m_peakEnergy = 0.0f;
}

public class CircleStackRestitution : Sample
{
	public const float m_impactSpeed = 5.0f;
	public const int m_maxStackCount = 3;

	public CircleStackRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 6.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;
		Polygon box = MakeBox( 40.0f, 1.0f );
		CreatePolygonShape( groundId, ref shapeDef, ref box );

		CreateScene();
	}

	public BodyId CreateBall( float y, float velocityY, float restitution, bool hitEvents )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = y };
		bodyDef.linearVelocity = new Vec2( 0.0f, velocityY );
		bodyDef.enableSleep = false;
		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;
		shapeDef.enableHitEvents = hitEvents;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		CreateCircleShape( bodyId, ref shapeDef, ref circle );

		return bodyId;
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			DestroyBody( m_bodyIds[i] );
		}
		m_bodyCount = 0;

		if ( m_gravity )
		{
			World_SetGravity( m_worldId, new( 0.0f, -10.0f ) );
		}
		else
		{
			World_SetGravity( m_worldId, Vec2_zero );
		}

		for ( int i = 0; i < m_stackCount; ++i )
		{
			m_bodyIds[m_bodyCount] = CreateBall( 0.5f + 1.0f * i, 0.0f, 0.0f, false );
			m_bodyCount += 1;
		}

		// Start half a step of travel above contact so the impact lands mid step, matching the test
		m_startHeight = 0.5f + 1.0f * m_stackCount + 0.5f * m_impactSpeed * ( 1.0f / 60.0f );

		m_impactorId = CreateBall( m_startHeight, -m_impactSpeed, m_restitution, true );
		m_bodyIds[m_bodyCount] = m_impactorId;
		m_bodyCount += 1;

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, m_bodyCount, ref linear, ref angular, ref potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;

		m_coefficient = 0.0f;
		m_latched = false;
		m_hit = false;
	}

	public override bool DrawControls(  )
	{
		bool rebuild = false;

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Restitution", ref m_restitution, 0.0f, 1.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		if ( ImGui.SliderInt( "Stack", ref m_stackCount, 0, m_maxStackCount ) )
		{
			rebuild = true;
		}

		ImGui.PopItemWidth();

		if ( ImGui.Checkbox( "Gravity", ref m_gravity ) )
		{
			rebuild = true;
		}

		if ( ImGui.Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		if ( events.hitCount > 0 )
		{
			m_hit = true;
		}

		float vy = Body_GetLinearVelocity( m_impactorId ).y;

		// The rebound only means something once the impact has happened and the impactor is leaving
		if ( m_latched == false && m_hit && vy > 0.0f )
		{
			m_coefficient = vy / m_impactSpeed;
			m_latched = true;
		}

		float supportSpeed = 0.0f;
		for ( int i = 0; i < m_stackCount; ++i )
		{
			supportSpeed = MaxFloat( supportSpeed, Length( Body_GetLinearVelocity( m_bodyIds[i] ) ) );
		}

		DrawLine( m_draw, new( -2.0f, m_startHeight ), new( 2.0f, m_startHeight ), HexColor.colorRed );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, m_bodyCount, ref linear, ref angular, ref potential );
		float total = linear + angular + potential;
		m_peakEnergy = MaxFloat( m_peakEnergy, total );

		DrawScreenTextLine( "impactor vy = %.3f m/s", vy );

		DrawScreenTextLine( "coefficient = %.4f live (target %.2f)", vy / m_impactSpeed, m_restitution );

		if ( m_latched )
		{
			DrawScreenTextLine( "coefficient = %.4f at first rebound", m_coefficient );
		}

		DrawScreenTextLine( "support speed = %.4f m/s", supportSpeed );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	public static Sample Create( SampleContext context )
	{
		return new CircleStackRestitution( context );
	}

	public BodyId[] m_bodyIds = new BodyId[m_maxStackCount + 1];
	public BodyId m_impactorId = nullBodyId;
	public int m_bodyCount = 0;
	public int m_stackCount = 2;
	public float m_restitution = 1.0f;
	public float m_startHeight = 0.0f;
	public float m_coefficient = 0.0f;
	public float m_startEnergy = 0.0f;
	public float m_peakEnergy = 0.0f;
	public bool m_gravity = false;
	public bool m_latched = false;
	public bool m_hit = false;
}

public class RestitutionPropagation : Sample
{
	public const int m_columnCount = 3;
	public const float m_dropHeight = 5.0f;

	public RestitutionPropagation( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 4.0f );
			m_context.camera.zoom = 10.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.6f;
		Polygon box = MakeBox( 40.0f, 1.0f );
		CreatePolygonShape( groundId, ref shapeDef, ref box );

		CreateScene();
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			DestroyBody( m_bodyIds[i] );
		}
		m_bodyCount = 0;
		m_crateCount = 0;
		m_kick = 0.0f;

		Polygon box = MakeBox( 0.5f, 0.5f );
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

		for ( int column = 0; column < m_columnCount; ++column )
		{
			float x = 4.0f * ( column - 1 );

			for ( int k = 0; k < column; ++k )
			{
				BodyDef bodyDefInner = DefaultBodyDef();
				bodyDefInner.type = BodyType.dynamicBody;
				bodyDefInner.position = new Pos { x = x, y = 0.5f + 1.0f * k };
				bodyDefInner.enableSleep = false;
				BodyId crateId = CreateBody( m_worldId, ref bodyDefInner );

				ShapeDef shapeDefInner = DefaultShapeDef();
				CreatePolygonShape( crateId, ref shapeDefInner, ref box );

				m_bodyIds[m_bodyCount++] = crateId;
				m_crateIds[m_crateCount++] = crateId;
			}

			m_restHeight[column] = 0.5f + 1.0f * column;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = x, y = m_restHeight[column] + m_dropHeight };
			bodyDef.enableSleep = false;
			BodyId ballId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.restitution = m_restitution;
			CreateCircleShape( ballId, ref shapeDef, ref circle );

			m_bodyIds[m_bodyCount++] = ballId;
			m_ballIds[column] = ballId;
			m_apex[column] = 0.0f;
			m_rebounding[column] = false;
			m_done[column] = false;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		bool rebuild = ImGui.SliderFloat( "Restitution", ref m_restitution, 0.0f, 1.0f, "%.2f" );
		ImGui.PopItemWidth();

		if ( ImGui.Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		string[] labels = { "ground", "one crate", "two crates" };

		for ( int column = 0; column < m_columnCount; ++column )
		{
			Vec2 v = Body_GetLinearVelocity( m_ballIds[column] );
			float height = (float)( Body_GetPosition( m_ballIds[column] ).y ) - m_restHeight[column];

			// The first rebound apex: the ball starts at rest and falls, so upward velocity means it bounced
			if ( m_rebounding[column] == false && v.y > 0.0f )
			{
				m_rebounding[column] = true;
			}

			if ( m_rebounding[column] && m_done[column] == false )
			{
				m_apex[column] = MaxFloat( m_apex[column], height );
				if ( v.y < 0.0f )
				{
					m_done[column] = true;
				}
			}

			float x = 4.0f * ( column - 1 );
			DrawLine( m_draw, new( x - 1.0f, m_restHeight[column] + m_dropHeight ), new( x + 1.0f, m_restHeight[column] + m_dropHeight ), HexColor.colorRed );
			if ( m_apex[column] > 0.0f )
			{
				DrawLine( m_draw, new( x - 1.0f, m_restHeight[column] + m_apex[column] ), new( x + 1.0f, m_restHeight[column] + m_apex[column] ), HexColor.colorGreen );
			}

			DrawScreenTextLine( "%s: apex %.2f m (%.0f%% of drop)", labels[column], m_apex[column],
								100.0f * m_apex[column] / m_dropHeight );
		}

		for ( int i = 0; i < m_crateCount; ++i )
		{
			m_kick = MaxFloat( m_kick, Length( Body_GetLinearVelocity( m_crateIds[i] ) ) );
		}

		DrawScreenTextLine( "peak crate speed %.2f m/s", m_kick );
	}

	public static Sample Create( SampleContext context )
	{
		return new RestitutionPropagation( context );
	}

	public BodyId[] m_bodyIds = new BodyId[2 * m_columnCount];
	public BodyId[] m_ballIds = new BodyId[m_columnCount];
	public BodyId[] m_crateIds = new BodyId[m_columnCount];
	public float[] m_restHeight = new float[m_columnCount];
	public float[] m_apex = new float[m_columnCount];
	public bool[] m_rebounding = new bool[m_columnCount];
	public bool[] m_done = new bool[m_columnCount];
	public int m_bodyCount = 0;
	public int m_crateCount = 0;
	public float m_restitution = 0.9f;
	public float m_kick = 0.0f;
}

public class BoxRestitution : Sample
{
	public const float m_impactSpeed = 5.0f;

	public BoxRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 10.0f;
		}

		// Gravity would bias the measured coefficient
		World_SetGravity( m_worldId, Vec2_zero );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;
		Polygon box = MakeBox( 40.0f, 1.0f );
		CreatePolygonShape( groundId, ref shapeDef, ref box );

		CreateScene();
	}

	public void CreateScene(  )
	{
		if ( m_boxId.IsNonNull )
		{
			DestroyBody( m_boxId );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = 0.25f + 0.5f * m_impactSpeed * ( 1.0f / 60.0f ) };
		bodyDef.linearVelocity = new Vec2( 0.0f, -m_impactSpeed );
		bodyDef.angularVelocity = m_spin;
		bodyDef.enableSleep = false;
		m_boxId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = m_restitution;
		shapeDef.enableHitEvents = true;
		Polygon box = MakeBox( 1.0f, 0.25f );
		CreatePolygonShape( m_boxId, ref shapeDef, ref box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_boxId ), 1, ref linear, ref angular, ref potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;

		m_coefficient = 0.0f;
		m_latched = false;
		m_hit = false;
	}

	public override bool DrawControls(  )
	{
		bool rebuild = false;

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Restitution", ref m_restitution, 0.0f, 1.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		if ( ImGui.SliderFloat( "Spin", ref m_spin, 0.0f, 2.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		if ( events.hitCount > 0 )
		{
			m_hit = true;
		}

		float vy = Body_GetLinearVelocity( m_boxId ).y;
		float w = Body_GetAngularVelocity( m_boxId );

		if ( m_latched == false && m_hit && vy > 0.0f )
		{
			m_coefficient = vy / m_impactSpeed;
			m_latched = true;
		}

		DrawPoint( m_draw, Body_GetWorldPoint( m_boxId, new( -1.0f, -0.25f ) ), 8.0f, HexColor.colorYellow );
		DrawPoint( m_draw, Body_GetWorldPoint( m_boxId, new( 1.0f, -0.25f ) ), 8.0f, HexColor.colorYellow );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_boxId ), 1, ref linear, ref angular, ref potential );
		float total = linear + angular + potential;
		m_peakEnergy = MaxFloat( m_peakEnergy, total );

		DrawScreenTextLine( "vy = %.3f m/s", vy );

		DrawScreenTextLine( "coefficient = %.4f live (target %.2f)", vy / m_impactSpeed, m_restitution );

		if ( m_latched )
		{
			DrawScreenTextLine( "coefficient = %.4f at first rebound", m_coefficient );
		}

		DrawScreenTextLine( "spin in = %.2f, spin out = %.4f (ideal %.2f at e = 1)", m_spin, w, -m_spin );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	public static Sample Create( SampleContext context )
	{
		return new BoxRestitution( context );
	}

	public BodyId m_boxId = nullBodyId;
	public float m_restitution = 0.9f;
	public float m_spin = 0.0f;
	public float m_coefficient = 0.0f;
	public float m_startEnergy = 0.0f;
	public float m_peakEnergy = 0.0f;
	public bool m_latched = false;
	public bool m_hit = false;
}

public class RotatedBoxRestitution : Sample
{
	public const int m_count = 12;
	public const float m_dropHeight = 10.0f;

	public RotatedBoxRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 4.0f );
			m_context.camera.zoom = 14.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		float h = 2.0f * m_count + 1.0f;
		Segment segment = new Segment { point1 = new( -h, 0.0f ), point2 = new( h, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		CreateScene();
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_count; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = nullBodyId;
			}
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.safetyFactor = m_safetyFactor;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = m_friction;
		shapeDef.material.restitution = m_restitution;

		// Boxes in a negative group never collide with each other.
		shapeDef.filter.groupIndex = -1;

		Polygon box = MakeBox( 0.5f, 0.5f );

		// A square repeats every quarter turn
		float dq = 0.5f * Pi / m_count;
		float x = -1.0f * ( m_count - 1 );

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = new Pos { x = x, y = m_dropHeight };
			bodyDef.rotation = MakeRot( i * dq );
			m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[i], ref shapeDef, ref box );

			float linear = 0.0f;
			float angular = 0.0f;
			float potential = 0.0f;
			ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyIds[i] ), 1, ref linear, ref angular, ref potential );
			m_startEnergy[i] = linear + angular + potential;

			x += 2.0f;
		}
	}

	public override bool DrawControls(  )
	{
		bool rebuild = false;

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Restitution", ref m_restitution, 0.0f, 1.0f, "%.1f" ) )
		{
			rebuild = true;
		}

		if ( ImGui.SliderFloat( "Friction", ref m_friction, 0.0f, 1.0f, "%.1f" ) )
		{
			rebuild = true;
		}

		if ( ImGui.SliderFloat( "Safety Factor", ref m_safetyFactor, 0.01f, 0.5f, "%.2f" ) )
		{
			rebuild = true;
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float h = 1.0f * m_count;
		DrawLine( m_draw, new( -h, m_dropHeight + 0.5f ), new( h, m_dropHeight + 0.5f ), HexColor.colorRed );

		float minPercent = 0.0f;
		float maxPercent = 0.0f;
		float x = -1.0f * ( m_count - 1 );

		for ( int i = 0; i < m_count; ++i )
		{
			float linear = 0.0f;
			float angular = 0.0f;
			float potential = 0.0f;
			ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in m_bodyIds[i] ), 1, ref linear, ref angular, ref potential );
			float total = linear + angular + potential;

			float percent = m_startEnergy[i] != 0.0f ? 100.0f * total / m_startEnergy[i] : 0.0f;
			minPercent = i == 0 ? percent : MinFloat( minPercent, percent );
			maxPercent = i == 0 ? percent : MaxFloat( maxPercent, percent );

			DrawString( m_draw, m_camera, ToPos( new( x - 0.5f, m_dropHeight + 1.5f ) ), HexColor.colorWhite, "%.1f%%", percent );
			x += 2.0f;
		}

		DrawScreenTextLine( "energy: min %.2f%%, max %.2f%% of start", minPercent, maxPercent );
	}

	public static Sample Create( SampleContext context )
	{
		return new RotatedBoxRestitution( context );
	}

	public BodyId[] m_bodyIds = new BodyId[m_count];
	public float[] m_startEnergy = new float[m_count];
	public float m_restitution = 0.9f;
	public float m_friction = 0.0f;
	public float m_safetyFactor = 0.5f;
}

public class TwoBoxRestitution : Sample
{
	public TwoBoxRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 3.0f );
			m_context.camera.zoom = 5.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		Polygon box = MakeSquare( 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.restitution = 0.9f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		bodyDef.position.y = 0.5f;
		m_bodyIds[0] = CreateBody( m_worldId, ref bodyDef );
		CreatePolygonShape( m_bodyIds[0], ref shapeDef, ref box );

		bodyDef.position.x += 0.1f;
		bodyDef.position.y = 6.0f;
		m_bodyIds[1] = CreateBody( m_worldId, ref bodyDef );
		CreatePolygonShape( m_bodyIds[1], ref shapeDef, ref box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, 2, ref linear, ref angular, ref potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	public override void Step(  )
	{
		base.Step();

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, 2, ref linear, ref angular, ref potential );
		float total = linear + angular + potential;
		m_peakEnergy = MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	public static Sample Create( SampleContext context )
	{
		return new TwoBoxRestitution( context );
	}

	public BodyId[] m_bodyIds = new BodyId[2];
	public float m_startEnergy = 0.0f;
	public float m_peakEnergy = 0.0f;
}

public class CircleRestitution : Sample
{
	public CircleRestitution( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 10.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			float h = 2.0f * m_count;
			Segment segment = new Segment { point1 = new( -h, 0.0f ), point2 = new( h, 0.0f ) };
			ShapeDef shapeDefInner = DefaultShapeDef();

			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		Circle circle = new Circle { center = Vec2_zero, radius = 0.5f };

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 0.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		float dr = 1.0f / ( m_count > 1 ? m_count - 1 : 1 );
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			string buffer = "";
			buffer = Fmt( "%.2f", shapeDef.material.restitution );

			bodyDef.position = new Pos { x = x, y = 1.0f };
			bodyDef.name = buffer;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			bodyDef.position = new Pos { x = x, y = 4.0f };
			bodyDef.name = buffer;
			bodyId = CreateBody( m_worldId, ref bodyDef );

			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			shapeDef.material.restitution += dr;
			x += dx;
		}
	}
	public static Sample Create( SampleContext context )
	{
		return new CircleRestitution( context );
	}

	public const int m_count = 10;
}
