// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Threading;
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

public static partial class Helpers_sample_character
{
	public const ulong StaticBit = 0x0001;
	public const ulong MoverBit = 0x0002;
	public const ulong DynamicBit = 0x0004;
	public const ulong AllBits = ~0u;


	// The ground probe is a point, a circle, or a segment, all carried by the proxy.
	public static void DrawPogo( Draw draw, Pos origin, Vec2 translation, float fraction, bool hit )
	{
		HexColor color = hit ? HexColor.colorPlum : HexColor.colorGray;
		Vec2 delta = fraction * translation;

		DrawLine( draw, origin, origin + delta, HexColor.colorGray );
		DrawPoint( draw, origin + delta, 10.0f, color );
	}







	// This shows how to mitigate ghost collisions using the presolve callback.
	// Based on: https://briansemrau.github.io/dealing-with-ghost-collisions/
}

public class GeometryMoverSample : Sample
{
	public GeometryMoverSample( SampleContext context ) : base( context )
	{
		if ( context.restart == false )
		{
			m_camera.center = new( 20.0f, 9.0f );
			m_camera.zoom = 10.0f;
		}

		context.debugDraw.drawJoints = false;

		GeometricMoverDef moverDef = new();
		moverDef.position = new Pos { x = 0.0f, y = 8.0f };
		moverDef.capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };
		moverDef.filter = new QueryFilter { categoryBits = MoverBit, maskBits = StaticBit | DynamicBit | MoverBit };
		m_mover.Create( m_worldId, ref moverDef );

		BodyId groundId1 = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "svg";
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			groundId1 = CreateBody( m_worldId, ref bodyDef );

			string path =
				"M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
				"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 -15.875,2e-5 v -3.96875 l -17.19792,2e-5 v 2.64582 l " +
				"-17.19791,0 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 " +
				"h -3.96875 v -1.32292 h -3.96875 v -1.32291 l -3.968754,0 v -1.32292 h -3.96875 v -1.32292 h -3.968751 v " +
				"-1.32291 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 l -3.96875,0 v " +
				"-1.32292 h -3.968751 v -1.32291 h -3.96875 l -2e-6,-1.32294 H 52.916669 L 39.6875,177.27083 h -5.291667 l " +
				"-7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25";

			Vec2[] points = new Vec2[64];

			Vec2 offset = new Vec2( -50.0f, -200.0f );
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			CreateChain( groundId1, ref chainDef );
		}

		BodyId groundId2 = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 98.0f, y = 0.0f };
			groundId2 = CreateBody( m_worldId, ref bodyDef );

			string path =
				"M 2.6458333,201.08333 H 399.52085 v -23.8125 h -23.8125 l 21.16667,18.52084 -232.83335,-1e-5 -0.0575,2.64584 h " +
				"-5.29166 v -2.64583 l -7.86855,-1e-5 -0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l " +
				"-2.64584,2.64583 -2.58891,-2.64584 -10.64031,2e-5 v -2.64583 l -7.9375,0 v -2.64584 l -7.9375,0 v -2.64583 l " +
				"-7.9375,0 v -2.64583 l -7.9375,0 v -2.64583 l -7.937501,-1e-5 v -2.64584 l -7.9375,1e-5 v -2.64583 l " +
				"-7.937501,-2e-5 V 174.625 l -7.937501,1e-5 v -2.64584 l -5.291669,0 -7.9375,-2.64584 -7.9375,-2.64583 " +
				"-5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H 2.6458334 l -10e-8,47.625";

			Vec2[] points = new Vec2[64];

			Vec2 offset = new Vec2( 0.0f, -200.0f );
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			CreateChain( groundId2, ref chainDef );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 32.0f, y = 4.5f };

			ShapeDef shapeDef = DefaultShapeDef();
			m_friendlyShape.maxPush = 0.025f;
			m_friendlyShape.clipVelocity = false;

			shapeDef.filter = new Filter { categoryBits = MoverBit, maskBits = AllBits, groupIndex = 0 };
			shapeDef.userData = m_friendlyShape;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( bodyId, ref shapeDef, ref m_mover.m_capsule );
		}

		{
			Polygon box = MakeBox( 0.55f, 0.125f );

			ShapeDef shapeDef = DefaultShapeDef();

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.maxMotorTorque = 10.0f;
			jointDef.enableMotor = true;
			jointDef.hertz = 3.0f;
			jointDef.dampingRatio = 0.8f;
			jointDef.enableSpring = true;

			float xBase = 48.7f;
			float yBase = 9.2f;
			int count = 50;
			BodyId prevBodyId = groundId1;
			for ( int i = 0; i < count; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = xBase + 0.5f + 1.0f * i, y = yBase };
				bodyDef.angularDamping = 0.2f;
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );

				Pos pivotInner = new Pos { x = xBase + 1.0f * i, y = yBase };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotInner );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotInner );
				CreateRevoluteJoint( m_worldId, ref jointDef );

				prevBodyId = bodyId;
			}

			Pos pivot = new Pos { x = xBase + 1.0f * count, y = yBase };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = groundId2;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		for ( int i = 0; i < 5; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 7.0f, y = 7.0f + 0.5f * i };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			Circle circle = new Circle { center = Vec2_zero, radius = 0.25f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = m_elevatorBase.x, y = m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = CreateBody( m_worldId, ref bodyDef );

			m_elevatorShape = new() { maxPush = 0.1f, clipVelocity = true };
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			shapeDef.userData = m_elevatorShape;

			Polygon box = MakeBox( 2.0f, 0.1f );
			CreatePolygonShape( m_elevatorId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position.x = 140.0f;
			float a = 0.25f;
			Polygon square = MakeSquare( a );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			for ( int i = 0; i < 10; ++i )
			{
				bodyDef.position.y = ( 2.0f * i + 1.0f ) * a + 1.0f;
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref square );
			}
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 160.0f, y = 5.0f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId body = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.1f, 4.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreatePolygonShape( body, ref shapeDef, ref box );

			Pos pivot = bodyDef.position + new Vec2( 0.0f, 4.0f );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId2;
			jointDef.@base.bodyIdB = body;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 1.0f;
			jointDef.maxMotorTorque = 500.0f;

			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		m_jumpReleased = true;
		m_lockCamera = true;
		m_time = 0.0f;
	}

	public override bool DrawControls(  )
	{
		ImGui.SliderFloat( "Jump Speed", ref m_mover.m_jumpSpeed, 0.0f, 40.0f, "%.0f" );
		ImGui.SliderFloat( "Min Speed", ref m_mover.m_minSpeed, 0.0f, 1.0f, "%.2f" );
		ImGui.SliderFloat( "Max Speed", ref m_mover.m_maxSpeed, 0.0f, 20.0f, "%.0f" );
		ImGui.SliderFloat( "Stop Speed", ref m_mover.m_stopSpeed, 0.0f, 10.0f, "%.1f" );
		ImGui.SliderFloat( "Accelerate", ref m_mover.m_accelerate, 0.0f, 100.0f, "%.0f" );
		ImGui.SliderFloat( "Friction", ref m_mover.m_friction, 0.0f, 10.0f, "%.1f" );
		ImGui.SliderFloat( "Gravity", ref m_mover.m_gravity, 0.0f, 100.0f, "%.1f" );
		ImGui.SliderFloat( "Air Steer", ref m_mover.m_airSteer, 0.0f, 1.0f, "%.2f" );
		ImGui.SliderFloat( "Pogo Hertz", ref m_mover.m_pogoHertz, 0.0f, 30.0f, "%.0f" );
		ImGui.SliderFloat( "Pogo Damping", ref m_mover.m_pogoDampingRatio, 0.0f, 4.0f, "%.1f" );

		ImGui.Separator();

		ImGui.Checkbox( "Push Bodies", ref m_mover.m_pushBodies );
		ImGui.Checkbox( "Lock Camera", ref m_lockCamera );

		return true;
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "left/right/jump = A/D/Space" );

		bool pause = false;
		if ( m_context.pause )
		{
			pause = m_context.singleStep == 0;
		}

		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
		if ( pause )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f )
		{
			Pos point = new Pos { x = m_elevatorBase.x, y = m_elevatorAmplitude * MathF.Cos( 1.0f * m_time + Pi ) + m_elevatorBase.y };

			bool wake = true;
			Body_SetTargetTransform( m_elevatorId, new( point, Rot_identity ), timeStep, wake );
		}

		m_time += timeStep;

		base.Step();

		if ( pause == false )
		{
			float throttle = 0.0f;

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_A ) != 0 ) )
			{
				throttle -= 1.0f;
			}

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_D ) != 0 ) )
			{
				throttle += 1.0f;
			}

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_SPACE ) != 0 ) )
			{
				if ( m_jumpReleased && m_mover.Jump() )
				{
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			m_mover.Update( timeStep, throttle );
		}

		DrawPogo( m_draw, m_mover.m_pogoOrigin, m_mover.m_pogoTranslation, m_mover.m_pogoFraction, m_mover.m_pogoHit );

		DrawTransform( m_draw, new( m_mover.m_position, Rot_identity ), 0.25f );

		Capsule capsule = m_mover.m_capsule;

		int count = m_mover.m_planeCount;
		for ( int i = 0; i < count; ++i )
		{
			Plane plane = m_mover.m_planes[i].plane;
			Pos p1Inner = m_mover.m_position + ( plane.offset - capsule.radius ) * plane.normal;
			Pos p2Inner = p1Inner + 0.1f * plane.normal;
			DrawPoint( m_draw, p1Inner, 5.0f, HexColor.colorYellow );
			DrawLine( m_draw, p1Inner, p2Inner, HexColor.colorYellow );
		}

		Pos p1 = m_mover.m_position + capsule.center1;
		Pos p2 = m_mover.m_position + capsule.center2;

		HexColor color = m_mover.m_onGround ? HexColor.colorOrange : HexColor.colorAquamarine;
		DrawCapsule( m_draw, p1, p2, capsule.radius, color );
		DrawLine( m_draw, m_mover.m_position, m_mover.m_position + m_mover.m_velocity, HexColor.colorPurple );

		Pos p = m_mover.m_position;
		DrawScreenTextLine( "position %.2f %.2f", p.x, p.y );
		DrawScreenTextLine( "velocity %.2f %.2f", m_mover.m_velocity.x, m_mover.m_velocity.y );
		DrawScreenTextLine( "iterations %d", m_mover.m_totalIterations );

		if ( m_lockCamera )
		{
			m_camera.center.x = m_mover.m_position.x;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new GeometryMoverSample( context );
	}

	public static readonly Vec2 m_elevatorBase = new Vec2( 112.0f, 10.0f );
	public const float m_elevatorAmplitude = 4.0f;

	public GeometricMover m_mover = new();
	public BodyId m_elevatorId;
	public MoverShapeUserData m_friendlyShape = new();
	public MoverShapeUserData m_elevatorShape = new();
	public float m_time;
	public bool m_jumpReleased;
	public bool m_lockCamera;
}

public class DynamicMoverSample : Sample
{
	public DynamicMoverSample( SampleContext context ) : base( context )
	{
		if ( context.restart == false )
		{
			m_camera.center = new( 20.0f, 9.0f );
			m_camera.zoom = 10.0f;
		}

		context.debugDraw.drawJoints = false;

		// This is the mover the player controls
		{
			DynamicMoverDef def = new();
			def.position = new Pos { x = 0.0f, y = 8.0f };
			def.capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };
			def.filter = new Filter { categoryBits = MoverBit, maskBits = StaticBit | DynamicBit };
			def.enablePreSolveEvents = true;
			def.platformMaterialId = m_platformMaterial;
			m_mover.Create( m_worldId, ref def );
		}

		m_elevatorId = nullBodyId;
		for ( int i = 0; i < m_platformIds.Length; ++i )
		{
			m_platformIds[i] = nullBodyId;
		}

		BodyId groundId1 = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "svg";
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			groundId1 = CreateBody( m_worldId, ref bodyDef );

#if false
			string path = "M -34.4,201.08 H 293.69 V 34.4 L -21.17,198.44 H -31.75 l -0,-92.6 h -2.65 l 0,95.25";
#else
			string path =
				"M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
				"-31.73269,7.9375 -21.16667,2.64583 -23.8125,13.22919 -15.875,2e-5 0,-6.61461 -17.19792,2e-5 v 2.64582 h " +
				"-17.19791 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 h " +
				"-3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.968754 v -1.32292 h -3.96875 v -1.32292 h -3.968751 v -1.32291 h " +
				"-3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h " +
				"-3.968751 v -1.32291 h -3.96875 l -2e-6,-1.32294 H 52.916669 L 39.6875,177.27083 h -5.291667 l " +
				"-7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25";
#endif

			Vec2[] points = new Vec2[64];

			Vec2 offset = new Vec2( -50.0f, -200.0f );
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			CreateChain( groundId1, ref chainDef );
		}

		BodyId groundId2 = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 98.0f, y = 0.0f };
			groundId2 = CreateBody( m_worldId, ref bodyDef );

			string path =
				"M 2.6458333,201.08333 H 399.52085 v -23.8125 h -23.8125 l 21.16667,18.52084 -232.83335,-1e-5 -0.0575,2.64584 h " +
				"-5.29166 v -2.64583 l -7.86855,-1e-5 -0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l " +
				"-2.64584,2.64583 -2.58891,-2.64584 -10.64031,2e-5 v -2.64583 l -7.9375,0 v -2.64584 l -7.9375,0 v -2.64583 l " +
				"-7.9375,0 v -2.64583 l -7.9375,0 v -2.64583 l -7.937501,-1e-5 v -2.64584 l -7.9375,1e-5 v -2.64583 l " +
				"-7.937501,-2e-5 V 174.625 l -7.937501,1e-5 v -2.64584 l -5.291669,0 -7.9375,-2.64584 -7.9375,-2.64583 " +
				"-5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H 2.6458334 l -10e-8,47.625";

			Vec2[] points = new Vec2[64];

			Vec2 offset = new Vec2( 0.0f, -200.0f );
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			CreateChain( groundId2, ref chainDef );
		}

		{
			Polygon box = MakeBox( 0.55f, 0.125f );
			ShapeDef shapeDef = DefaultShapeDef();

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.maxMotorTorque = 10.0f;
			jointDef.enableMotor = true;
			jointDef.hertz = 3.0f;
			jointDef.dampingRatio = 0.8f;
			jointDef.enableSpring = true;

			float xBase = 48.7f;
			float yBase = 9.2f;
			int count = 50;
			BodyId prevBodyId = groundId1;
			for ( int i = 0; i < count; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = xBase + 0.5f + 1.0f * i, y = yBase };
				bodyDef.angularDamping = 0.2f;
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );

				Pos pivotInner = new Pos { x = xBase + 1.0f * i, y = yBase };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotInner );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotInner );
				CreateRevoluteJoint( m_worldId, ref jointDef );

				prevBodyId = bodyId;
			}

			Pos pivot = new Pos { x = xBase + 1.0f * count, y = yBase };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = groundId2;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		for ( int i = 0; i < 5; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 7.0f, y = 7.0f + 0.5f * i };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			Circle circle = new Circle { center = Vec2_zero, radius = 0.25f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = m_elevatorBase.x, y = m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };

			Polygon box = MakeBox( 2.0f, 0.1f );
			CreatePolygonShape( m_elevatorId, ref shapeDef, ref box );
		}

		// Platform ledges
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId ledgeId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			Polygon box = MakeOffsetBox( 2.95f, 0.25f, new( 117.05f, 13.85f ), Rot_identity );
			CreatePolygonShape( ledgeId, ref shapeDef, ref box );

			box = MakeOffsetBox( 2.0f, 0.25f, new( 136.0f, 13.85f ), Rot_identity );
			CreatePolygonShape( ledgeId, ref shapeDef, ref box );

			box = MakeOffsetBox( 1.75f, 0.25f, new( 151.75f, 13.85f ), Rot_identity );
			CreatePolygonShape( ledgeId, ref shapeDef, ref box );
		}

		for ( int i = 0; i < m_platformCount; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = m_platforms[i].@base;
			m_platformIds[i] = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			shapeDef.material.userMaterialId = m_platformMaterial;

			Polygon box = MakeBox( 1.5f, 0.1f );
			CreatePolygonShape( m_platformIds[i], ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position.x = 140.0f;
			float a = 0.25f;
			Polygon square = MakeSquare( a );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter = new Filter { categoryBits = DynamicBit, maskBits = AllBits, groupIndex = 0 };
			for ( int i = 0; i < 10; ++i )
			{
				bodyDef.position.y = ( 2.0f * i + 1.0f ) * a + 1.0f;
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref square );
			}
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 160.0f, y = 4.0f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId body = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.1f, 4.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreatePolygonShape( body, ref shapeDef, ref box );

			Pos pivot = bodyDef.position + new Vec2( 0.0f, 4.0f );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId2;
			jointDef.@base.bodyIdB = body;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 1.0f;
			jointDef.maxMotorTorque = 500.0f;

			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		m_preSolve = true;

		// This is used for one-way collision on the kinematic elevator. But pre-solve doesn't work with recording.
		if ( m_preSolve )
		{
			World_SetPreSolveCallback( m_worldId, PreSolveStatic, PreContinuousStatic, this );
		}

		m_jumpReleased = true;
		m_lockCamera = true;
		m_time = 0.0f;
	}

	public void PreSolve( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold )
	{
		BodyId idA = Shape_GetBody( shapeIdA );
		BodyId idB = Shape_GetBody( shapeIdB );

		if ( ( idA ).Equals( m_elevatorId ) && ( idB ).Equals( m_mover.m_moverId ) )
		{
			if ( manifold.normal.y < 0.0f )
			{
				manifold.pointCount = 0;
			}
		}
		else if ( ( idA ).Equals( m_mover.m_moverId ) && ( idB ).Equals( m_elevatorId ) )
		{
			if ( manifold.normal.y > 0.0f )
			{
				manifold.pointCount = 0;
			}
		}
	}

	public static void PreSolveStatic( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object? context )
	{
		DynamicMoverSample? sample = (DynamicMoverSample?)context;
		sample.PreSolve( shapeIdA, shapeIdB, ref manifold );
	}

	public bool PreContinuous( ShapeId shapeIdA, ShapeId shapeIdB, Pos point, Vec2 normal )
	{

		BodyId idA = Shape_GetBody( shapeIdA );
		BodyId idB = Shape_GetBody( shapeIdB );

		if ( ( idA ).Equals( m_elevatorId ) && ( idB ).Equals( m_mover.m_moverId ) )
		{
			if ( normal.y < 0.0f )
			{
				return false;
			}

			return true;
		}

		if ( ( idA ).Equals( m_mover.m_moverId ) && ( idB ).Equals( m_elevatorId ) )
		{
			if ( normal.y > 0.0f )
			{
				return false;
			}

			return true;
		}

		return true;
	}

	public static bool PreContinuousStatic( ShapeId shapeIdA, ShapeId shapeIdB, Pos point, Vec2 normal, object? context )
	{
		DynamicMoverSample? sample = (DynamicMoverSample?)context;
		return sample.PreContinuous( shapeIdA, shapeIdB, point, normal );
	}

	public override bool DrawControls(  )
	{
		ImGui.SliderFloat( "Jump Speed", ref m_mover.m_jumpSpeed, 0.0f, 40.0f, "%.0f" );
		ImGui.SliderFloat( "Min Speed", ref m_mover.m_minSpeed, 0.0f, 1.0f, "%.2f" );
		ImGui.SliderFloat( "Max Speed", ref m_mover.m_maxSpeed, 0.0f, 20.0f, "%.0f" );
		ImGui.SliderFloat( "Stop Speed", ref m_mover.m_stopSpeed, 0.0f, 10.0f, "%.1f" );
		ImGui.SliderFloat( "Accelerate", ref m_mover.m_accelerate, 0.0f, 100.0f, "%.0f" );
		ImGui.SliderFloat( "Friction", ref m_mover.m_friction, 0.0f, 10.0f, "%.1f" );

		float gravityScale = m_mover.m_gravityScale;
		if ( ImGui.SliderFloat( "Gravity Scale", ref gravityScale, 0.0f, 4.0f, "%.1f" ) )
		{
			m_mover.SetGravityScale( gravityScale );
		}

		ImGui.SliderFloat( "Air Steer", ref m_mover.m_airSteer, 0.0f, 1.0f, "%.2f" );
		ImGui.SliderFloat( "Pogo Hertz", ref m_mover.m_pogoHertz, 0.0f, 30.0f, "%.0f" );
		ImGui.SliderFloat( "Pogo Damping", ref m_mover.m_pogoDampingRatio, 0.0f, 4.0f, "%.1f" );
		ImGui.Checkbox( "Side Pogo", ref m_mover.m_enableSidePogo );

		ImGui.Separator();

		ImGui.Checkbox( "Lock Camera", ref m_lockCamera );

		return true;
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "left/right/jump/sprint/walk = A/D/Space/Shift/Ctrl" );

		bool pause = false;
		if ( m_context.pause )
		{
			pause = m_context.singleStep == 0;
		}

		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
		if ( pause )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f && m_elevatorId.IsNonNull )
		{
			Pos point = new Pos { x = m_elevatorBase.x, y = m_elevatorAmplitude * MathF.Cos( 1.0f * m_time + Pi ) + m_elevatorBase.y };

			bool wake = true;
			Body_SetTargetTransform( m_elevatorId, new( point, Rot_identity ), timeStep, wake );
		}

		for ( int i = 0; i < m_platformCount; ++i )
		{
			if ( timeStep > 0.0f && m_platformIds[i].IsNonNull )
			{
				MovingPlatform platform = m_platforms[i];
				Pos point = new Pos { x = platform.amplitude * MathF.Sin( platform.rate * m_time ) + platform.@base.x, y = platform.@base.y };

				bool wake = true;
				Body_SetTargetTransform( m_platformIds[i], new( point, Rot_identity ), timeStep, wake );
			}
		}

		m_time += timeStep;

		if ( pause == false )
		{
			float throttle = 0.0f;

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_A ) != 0 ) )
			{
				throttle -= 1.0f;
			}

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_D ) != 0 ) )
			{
				throttle += 1.0f;
			}

			float speedScale = 1.0f;
			if ( glfwGetKey( m_context.window, GLFW_KEY_LEFT_CONTROL ) || ( glfwGetKey( m_context.window, GLFW_KEY_RIGHT_CONTROL ) != 0 ) )
			{
				speedScale = 0.25f;
			}
			else if ( glfwGetKey( m_context.window, GLFW_KEY_LEFT_SHIFT ) || ( glfwGetKey( m_context.window, GLFW_KEY_RIGHT_SHIFT ) != 0 ) )
			{
				speedScale = 1.5f;
			}

			if ( ( glfwGetKey( m_context.window, GLFW_KEY_SPACE ) != 0 ) )
			{
				if ( m_jumpReleased && m_mover.Jump() )
				{
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			m_mover.Update( timeStep, throttle, speedScale );
		}

		base.Step();

		for ( int i = 0; i < m_mover.m_probeCount; ++i )
		{
			DynamicMoverProbe probe = m_mover.m_probes[i];
			bool selected = probe.hit && i == m_mover.m_selectedProbe;
			DrawPogo( m_draw, probe.origin, probe.translation, probe.fraction, selected );
		}

		Pos position = Body_GetPosition( m_mover.m_moverId );

		HexColor color = m_mover.m_onGround ? HexColor.colorOrange : HexColor.colorAquamarine;
		DrawLine( m_draw, position, position + m_mover.m_velocity, color );

		DrawScreenTextLine( "position %.2f %.2f", position.x, position.y );
		DrawScreenTextLine( "velocity %.2f %.2f", m_mover.m_velocity.x, m_mover.m_velocity.y );
		DrawScreenTextLine( "pogo: impulse %.3f, velocity %.3f", m_mover.m_pogoImpulse, m_mover.m_pogoVelocity );
		DrawScreenTextLine( "pogo: length = %.2f/%.2f", m_mover.m_pogoLength, m_mover.m_pogoRestLength );
		DrawScreenTextLine( "on ground: %d", m_mover.m_onGround ? 1 : 0 );
		float normalY = m_mover.m_castResult.hit ? m_mover.m_castResult.normal.y : 0.0f;
		DrawScreenTextLine( "walkable: %d, normal y: %.3f/%.3f", m_mover.m_walkable ? 1 : 0, normalY, m_mover.m_minGroundNormalY );
		DrawScreenTextLine( "jumping/ticks: %d/%d", m_mover.m_jumping ? 1 : 0, m_mover.m_jumpTicks );

		if ( m_lockCamera )
		{
			m_camera.center.x = position.x;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new DynamicMoverSample( context );
	}

	public static readonly Vec2 m_elevatorBase = new Vec2( 112.0f, 10.0f );
	public const float m_elevatorAmplitude = 4.0f;
	public struct MovingPlatform
	{
		public Pos @base;
		public float amplitude;
		public float rate;

		public MovingPlatform( Pos @base, float amplitude, float rate )
		{
			this.@base = @base;
			this.amplitude = amplitude;
			this.rate = rate;
		}
	}

	public const int m_platformCount = 2;
	MovingPlatform[] m_platforms = {
		new( new( 127.0f, 14.0f ), 5.5f, 0.5f ),
		new( new( 144.0f, 14.0f ), 4.5f, 0.8f ),
	};

	public const ulong m_platformMaterial = 1;

	public DynamicMover m_mover = new();
	public BodyId m_elevatorId;
	public BodyId[] m_platformIds = new BodyId[m_platformCount];
	public float m_time;
	public bool m_preSolve;
	public bool m_jumpReleased;
	public bool m_lockCamera;
}

public class GhostCulling : Sample
{
	public GhostCulling( SampleContext context ) : base( context )
	{
		if ( context.restart == false )
		{
			m_camera.center = new( 0.0f, 1.5f );
			m_camera.zoom = 12.0f;
		}

		BodyDef groundBodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref groundBodyDef );

		float[] surfaceHeights = { 3.0f, 0.0f };
		foreach ( float surfaceHeight in surfaceHeights )
		{
			for ( int i = 0; i < 10; ++i )
			{
				float x = -9.0f + 2.0f * i;
				Polygon tile = MakeOffsetBox( 1.0f, 0.25f, new( x, surfaceHeight - 0.25f ), Rot_identity );

				ShapeDef shapeDefInner = DefaultShapeDef();
				shapeDefInner.material.friction = 0.0f;

				// Flag the horizontal tiles with a special material.
				shapeDefInner.material.userMaterialId = m_floorMaterial;

				shapeDefInner.material.customColor = (uint)( i % 2 == 0 ? HexColor.colorGray : HexColor.colorDarkGray );

				CreatePolygonShape( groundId, ref shapeDefInner, ref tile );
			}

			Polygon wall = MakeOffsetBox( 0.25f, 1.25f, new( 10.25f, surfaceHeight + 1.25f ), Rot_identity );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.0f;
			shapeDef.material.customColor = (uint)HexColor.colorDarkGray;
			CreatePolygonShape( groundId, ref shapeDef, ref wall );
		}

		m_speed = 20.0f;
		m_playerIds[0] = nullBodyId;
		m_playerIds[1] = nullBodyId;
		Volatile.Write( ref m_rejectedCount, 0 );

		World_SetPreSolveCallback( m_worldId, PreSolveStatic, PreContinuousStatic, this );
		Launch();
	}

	public void Launch(  )
	{
		Volatile.Write( ref m_rejectedCount, 0 );

		float[] surfaceHeights = { 3.0f, 0.0f };
		for ( int i = 0; i < 2; ++i )
		{
			if ( m_playerIds[i].IsNonNull )
			{
				DestroyBody( m_playerIds[i] );
			}

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -9.0f, y = surfaceHeights[i] + 0.5f };
			bodyDef.linearVelocity = new Vec2( m_speed, 0.0f );
			bodyDef.motionLocks.angularZ = true;

			// Pre-solve must see a fresh manifold at every tile boundary.
			bodyDef.enableContactRecycling = false;

			m_playerIds[i] = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			shapeDef.material.friction = 0.0f;
			shapeDef.enablePreSolveEvents = i == 1;

			Polygon player = MakeBox( 0.5f, 0.5f );
			CreatePolygonShape( m_playerIds[i], ref shapeDef, ref player );
		}
	}

	public void PreSolve( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold )
	{
		if ( manifold.pointCount != 2 )
		{
			return;
		}

		Vec2 normal = manifold.normal;
		BodyId bodyIdA = Shape_GetBody( shapeIdA );
		if ( ( bodyIdA ).Equals( m_playerIds[1] ) == false )
		{
			BodyId bodyIdB = Shape_GetBody( shapeIdB );
			if ( ( bodyIdB ).Equals( m_playerIds[1] ) == false )
			{
				return;
			}

			normal = Neg( normal );
		}

		Vec2 velocity = Body_GetLinearVelocity( m_playerIds[1] );
		if ( Dot( normal, velocity ) < 0.0f )
		{
			return;
		}

		// A short face contact is overlap slop at a tile seam, not a wall.
		float contactArea = Distance( manifold.points[0].anchorA, manifold.points[1].anchorA );
		if ( contactArea < 4.0f * LinearSlop )
		{
			manifold.pointCount = 0;

			// PreSolve is called from a thread and should not have data race.
			Interlocked.Increment( ref m_rejectedCount );
		}
	}

	public static void PreSolveStatic( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object? context )
	{
		GhostCulling? sample = (GhostCulling?)( context );
		sample.PreSolve( shapeIdA, shapeIdB, ref manifold );
	}

	// I want to disable horizontal continuous collision between the player and the floor tiles.
	// Continuous collision in Box2D doesn't affect velocity, but it does cause time loss, which
	// may be perceived as stuttering or non-smooth movement.
	// 
	// CCD doesn't get the full manifold so the contactArea cannot be used. Instead I disable
	// the CCD event if the normal is horizontal and the other shape is a horizontal tile,
	// identified by material id.
	public bool PreContinuous( ShapeId shapeIdA, ShapeId shapeIdB, Vec2 normal )
	{
		// Only filter horizontal hits.
		if ( AbsFloat( normal.y ) > 0.1f )
		{
			return true;
		}

		BodyId idA = Shape_GetBody( shapeIdA );

		// Determine if this is player[1] and determine the other shape.
		ShapeId floorShapeId = default;
		if ( ( idA ).Equals( m_playerIds[1] ) )
		{
			floorShapeId = shapeIdB;
		}
		else
		{
			BodyId idB = Shape_GetBody( shapeIdB );
			if ( ( idB ).Equals( m_playerIds[1] ) )
			{
				floorShapeId = shapeIdA;
			}
			else
			{
				return true;
			}
		}

		ulong material = Shape_GetUserMaterial( floorShapeId );
		if ( material == m_floorMaterial )
		{
			// This is a floor tile.
			return false;
		}

		// Not a floor time, allow CCD.
		return true;
	}

	public static bool PreContinuousStatic( ShapeId shapeIdA, ShapeId shapeIdB, Pos point, Vec2 normal, object? context )
	{
		GhostCulling? sample = (GhostCulling?)context;
		return sample.PreContinuous( shapeIdA, shapeIdB, normal );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "Speed", ref m_speed, 1.0f, 100.0f, "%.1f" );
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

		DrawString( m_draw, m_camera, new( -9.8f, 4.2f ), HexColor.colorWhite, "unfiltered" );
		DrawString( m_draw, m_camera, new( -9.8f, 1.2f ), HexColor.colorWhite, "shallow contacts rejected" );
		DrawScreenTextLine( "Rejected contacts: %d", Volatile.Read( ref m_rejectedCount ) );

		for ( int i = 0; i < 2; ++i )
		{
			Vec2 v = Body_GetLinearVelocity( m_playerIds[i] );
			DrawScreenTextLine( "player %d velocity.x = %.3f", i, v.x );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new GhostCulling( context );
	}

	public const ulong m_floorMaterial = 42;

	public BodyId[] m_playerIds = new BodyId[2];
	public int m_rejectedCount;
	public float m_speed;
}
