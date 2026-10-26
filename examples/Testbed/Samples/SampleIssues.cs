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

public static partial class Helpers_sample_issues
{
	// This simulations stresses the solver by putting a light mass between two bodies on a prismatic joint with a stiff spring.
	// This can be made stable by increasing the size of the middle circle and/or increasing the number of sub-steps.
	// todo try applying the impulse at the mid point to avoid breaking Newton's 3rd law



	// This configuration has stability problems.
}

public class UnstablePrismaticJoints : Sample
{
	public UnstablePrismaticJoints( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.75f );
			m_context.camera.zoom = 32.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -100.0f, 0.0f ), point2 = new( 100.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		BodyId centerId = default;
		{
			BodyDef bd = DefaultBodyDef();
			bd.type = BodyType.dynamicBody;
			bd.position = new Pos { x = 0, y = 3 };
			centerId = CreateBody( m_worldId, ref bd );

			ShapeDef sd = DefaultShapeDef();

			Circle circle = default;
			circle.center = new Vec2( 0, 0 );

			// Note: this will crash due to divergence (inf/nan) with a radius of 0.1
			// circle.radius = 0.1f;
			circle.radius = 0.5f;

			CreateCircleShape( centerId, ref sd, ref circle );
		}

		PrismaticJointDef jd = DefaultPrismaticJointDef();
		jd.enableSpring = true;
		jd.hertz = 10.0f;
		jd.dampingRatio = 2.0f;

		{
			BodyDef bd = DefaultBodyDef();
			bd.type = BodyType.dynamicBody;
			bd.position = new Pos { x = -3.5f, y = 3 };

			BodyId leftId = CreateBody( m_worldId, ref bd );

			ShapeDef sd = DefaultShapeDef();

			Circle circle = default;
			circle.center = new Vec2( 0, 0 );
			circle.radius = 2.0f;
			CreateCircleShape( leftId, ref sd, ref circle );

			jd.@base.bodyIdA = centerId;
			jd.@base.bodyIdB = leftId;
			jd.targetTranslation = -3.0f;
			CreatePrismaticJoint( m_worldId, ref jd );
		}

		{
			BodyDef bd = DefaultBodyDef();
			bd.type = BodyType.dynamicBody;
			bd.position = new Pos { x = 3.5f, y = 3 };
			BodyId rightId = CreateBody( m_worldId, ref bd );

			ShapeDef sd = DefaultShapeDef();

			Circle circle = default;
			circle.center = new Vec2( 0, 0 );
			circle.radius = 2.0f;

			CreateCircleShape( rightId, ref sd, ref circle );

			jd.@base.bodyIdA = centerId;
			jd.@base.bodyIdB = rightId;
			jd.targetTranslation = 3.0f;
			CreatePrismaticJoint( m_worldId, ref jd );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new UnstablePrismaticJoints( context );
	}
}

public class UnstableWindmill : Sample
{
	public UnstableWindmill( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.75f );
			m_context.camera.zoom = 32.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -100.0f, -10.0f ), point2 = new( 100.0f, -10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		BodyDef bdef = DefaultBodyDef();
		bdef.gravityScale = 0.0f;
		bdef.type = BodyType.dynamicBody;
		ShapeDef sdef = DefaultShapeDef();
		sdef.material = DefaultSurfaceMaterial();
		sdef.material.friction = 0.1f;

		// center
		bdef.position = new Pos { x = 10, y = 10 };
		BodyId center = CreateBody( m_worldId, ref bdef );
		Circle circle = new Circle { center = new( 0, 0 ), radius = 5 };
		CreateCircleShape( center, ref sdef, ref circle );

		// rotors
		WeldJointDef wjdef = DefaultWeldJointDef();

		// This simulation can be stabilized by using a lower constraint stiffness
		//wjdef.base.constraintHertz = 30.0f;
		wjdef.@base.bodyIdA = center;

		Polygon polygon = default;
		bdef.position = new Pos { x = 10, y = 0 };
		BodyId body = CreateBody( m_worldId, ref bdef );
		CreatePolygonShape( body, ref sdef, MakeBox( 4, 5 ) );
		wjdef.@base.localFrameA = new() { p = new( 0, -5 ), q = Rot_identity };
		wjdef.@base.bodyIdB = body;
		wjdef.@base.localFrameB = new() { p = new( 0, 5 ), q = Rot_identity };
		CreateWeldJoint( m_worldId, ref wjdef );

		bdef.position = new Pos { x = 20, y = 10 };
		body = CreateBody( m_worldId, ref bdef );
		CreatePolygonShape( body, ref sdef, MakeBox( 5, 4 ) );
		wjdef.@base.localFrameA = new() { p = new( 5, 0 ), q = Rot_identity };
		wjdef.@base.bodyIdB = body;
		wjdef.@base.localFrameB = new() { p = new( -5, 0 ), q = Rot_identity };
		CreateWeldJoint( m_worldId, ref wjdef );

		bdef.position = new Pos { x = 10, y = 20 };
		body = CreateBody( m_worldId, ref bdef );
		CreatePolygonShape( body, ref sdef, MakeBox( 4, 5 ) );
		wjdef.@base.localFrameA = new() { p = new( 0, 5 ), q = Rot_identity };
		wjdef.@base.bodyIdB = body;
		wjdef.@base.localFrameB = new() { p = new( 0, -5 ), q = Rot_identity };
		CreateWeldJoint( m_worldId, ref wjdef );

		bdef.position = new Pos { x = 0, y = 10 };
		body = CreateBody( m_worldId, ref bdef );
		CreatePolygonShape( body, ref sdef, MakeBox( 5, 4 ) );
		wjdef.@base.localFrameA = new() { p = new( -5, 0 ), q = Rot_identity };
		wjdef.@base.bodyIdB = body;
		wjdef.@base.localFrameB = new() { p = new( 5, 0 ), q = Rot_identity };
		CreateWeldJoint( m_worldId, ref wjdef );
	}

	public static Sample Create( SampleContext context )
	{
		return new UnstableWindmill( context );
	}
}
