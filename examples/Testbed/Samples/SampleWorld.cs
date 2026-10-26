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

public static partial class Helpers_sample_world
{
	// A pyramid built far from the origin to exercise double precision world positions. The contact
	// solver runs in delta space, so the stack settles the same as one at the origin while the body
	// transforms hold their full double coordinate. Record it (R) and open it in the Replay Viewer:
	// the recorded doubles survive the snapshot and the motion reproduces with no divergence.






	// The Gear Lift mechanism from the Joints samples, rebuilt 1000 km from the origin. This sample is
	// not guarded, so it compiles and renders in both precision modes. The debug draw subtracts the
	// camera center before narrowing to float, so the view is correct either way. Build double and the
	// gears mesh cleanly out here. Build single and the float grid (printed on screen) is coarser than
	// the gear teeth, so the same mechanism grinds and jitters. One scene, two builds, a side by side.
}

public class TileWorld : Sample
{
	public TileWorld( SampleContext context ) : base( context )
	{
		m_period = 40.0f;
		float omega = 2.0f * Pi / m_period;
		m_cycleCount = m_isDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)( m_cycleCount * m_period / m_gridSize );

		float xStart = -0.5f * ( m_cycleCount * m_period );

		m_viewPosition = new Pos { x = xStart, y = 15.0f };

		if ( m_context.restart == false )
		{
			m_context.camera.center = m_viewPosition;
			m_context.camera.zoom = 25.0f * 1.0f;
			m_context.debugDraw.drawJoints = false;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			ShapeDef shapeDef = DefaultShapeDef();

			// Setting this to false significantly reduces the cost of creating
			// static bodies and shapes.
			shapeDef.invokeContactCreation = false;

			float height = 4.0f;
			float xBody = xStart;
			float xShape = xStart;

			BodyId groundId = default;
			for ( int i = 0; i < m_gridCount; ++i )
			{
				// Create a new body regularly so that shapes are not too far from the body origin.
				// Most algorithms in Box2D work in local coordinates, but contact points are computed
				// relative to the body origin.
				// This makes a noticeable improvement in stability far from the origin.
				if ( i % 10 == 0 )
				{
					bodyDef.position.x = xBody;
					groundId = CreateBody( m_worldId, ref bodyDef );
					xShape = 0.0f;
				}

				float y = 0.0f;

				int ycount = (int)MathF.Round( height * MathF.Cos( omega * xBody ) ) + 12;

				for ( int j = 0; j < ycount; ++j )
				{
					Polygon square = MakeOffsetBox( 0.4f * m_gridSize, 0.4f * m_gridSize, new( xShape, y ), Rot_identity );
					square.radius = 0.1f;
					CreatePolygonShape( groundId, ref shapeDef, ref square );

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
		for ( int cycleIndex = 0; cycleIndex < m_cycleCount; ++cycleIndex )
		{
			float xbase = ( 0.5f + cycleIndex ) * m_period + xStart;

			int remainder = cycleIndex % 3;
			if ( remainder == 0 )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = xbase - 3.0f, y = 10.0f };

				ShapeDef shapeDef = DefaultShapeDef();
				Polygon box = MakeBox( 0.3f, 0.2f );

				for ( int i = 0; i < 10; ++i )
				{
					bodyDef.position.y = 10.0f;
					for ( int j = 0; j < 5; ++j )
					{
						BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
						CreatePolygonShape( bodyId, ref shapeDef, ref box );
						bodyDef.position.y += 0.5f;
					}
					bodyDef.position.x += 0.6f;
				}
			}
			else if ( remainder == 1 )
			{
				Pos position = new Pos { x = xbase - 2.0f, y = 10.0f };
				for ( int i = 0; i < 5; ++i )
				{
					Human human = default;
					CreateHuman( ref human, m_worldId, position, 1.5f, 0.05f, 0.0f, 0.0f, humanIndex + 1, null, false );
					humanIndex += 1;
					position.x += 1.0f;
				}
			}
			else
			{
				Pos position = new Pos { x = xbase - 4.0f, y = 12.0f };

				for ( int i = 0; i < 5; ++i )
				{
					Donut donut = new();
					donut.Create( m_worldId, position, 0.75f, 0, false, null );
					position.x += 2.0f;
				}
			}
		}

		m_car.Spawn( m_worldId, new( xStart + 20.0f, 40.0f ), 10.0f, 2.0f, 0.7f, 2000.0f, null );

		m_cycleIndex = 0;
		m_speed = 0.0f;
		m_explosionPosition = new Pos { x = ( 0.5f + m_cycleIndex ) * m_period + xStart, y = 7.0f };
		m_explode = true;
		m_followCar = false;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "speed", ref m_speed, -400.0f, 400.0f, "%.0f" );
		ImGui.PopItemWidth();
		if ( ImGui.Button( "stop" ) )
		{
			m_speed = 0.0f;
		}

		ImGui.Checkbox( "explode", ref m_explode );
		ImGui.Checkbox( "follow car", ref m_followCar );

		ImGui.Text( Fmt( "world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f ) );

		return true;
	}

	public override void Step(  )
	{
		float span = 0.5f * ( m_period * m_cycleCount );
		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;

		if ( m_context.pause )
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = ClampFloat( m_viewPosition.x, -span, span );

		if ( m_speed != 0.0f )
		{
			m_context.camera.center = m_viewPosition;
		}

		if ( m_followCar )
		{
			m_context.camera.center.x = Body_GetPosition( m_car.m_chassisId ).x;
		}

		float radius = 2.0f;
		if ( ( m_stepCount & 0x1 ) == 0x1 && m_explode )
		{
			m_explosionPosition.x = ( 0.5f + m_cycleIndex ) * m_period - span;

			ExplosionDef def = DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = radius;
			def.falloff = 0.1f;
			def.impulsePerLength = 1.0f;
			World_Explode( m_worldId, ref def );

			m_cycleIndex = ( m_cycleIndex + 1 ) % m_cycleCount;
		}

		if ( m_explode )
		{
			DrawCircle( m_draw, m_explosionPosition, radius, HexColor.colorAzure );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 20.0f );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_car.SetSpeed( -5.0f );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new TileWorld( context );
	}

	public Car m_car = new();
	public Pos m_viewPosition;
	public float m_period;
	public int m_cycleCount;
	public int m_cycleIndex;
	public float m_gridCount;
	public float m_gridSize;
	public float m_speed;

	public Pos m_explosionPosition;
	public bool m_explode;
	public bool m_followCar;
}

public class FarPyramid : Sample
{
	public FarPyramid( SampleContext context ) : base( context )
	{
		// 1e7 is exactly representable in float, so integer box offsets stay exact and a run here
		// can be compared against one at the origin.
		Pos origin = new Pos( 10.0e6f, 0.0f );

		if ( m_context.restart == false )
		{
			m_context.camera.center = OffsetPos( origin, new( 0.0f, 12.0f ) );
			m_context.camera.zoom = 17.0f;
		}

		float h = 0.25f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = origin;
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		Polygon box = MakeBox( h, h );
		bodyDef.type = BodyType.dynamicBody;

		int baseCount = 50;
		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * h;
			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * h + 2.0f * ( j - i ) * h - h * baseCount;
				bodyDef.position = OffsetPos( origin, new( x, y ) );
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref box );
			}
		}
	}

	public override void Step(  )
	{
		base.Step();

		if ( IsDoublePrecision() == true )
		{
			DrawScreenTextLine( "Double precision" );
		}
		else
		{
			DrawScreenTextLine( "Single precision" );
		}

		Pos c = m_context.camera.center;
		DrawScreenTextLine( "view center %.0f km from world origin", 0.001f * c.x );
	}

	public static Sample Create( SampleContext context )
	{
		return new FarPyramid( context );
	}
}

public class FarRagdolls : Sample
{
	public FarRagdolls( SampleContext context ) : base( context )
	{
		Pos origin = new Pos( 10.0e6f, 0.0f );

		if ( m_context.restart == false )
		{
			m_context.camera.center = OffsetPos( origin, new( 0.0f, 6.0f ) );
			m_context.camera.zoom = 10.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = origin;
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		float w = 6.0f;
		float h = 12.0f;
		Segment floor = new Segment { point1 = new( -w, 0.0f ), point2 = new( w, 0.0f ) };
		Segment leftWall = new Segment { point1 = new( -w, 0.0f ), point2 = new( -w, h ) };
		Segment rightWall = new Segment { point1 = new( w, 0.0f ), point2 = new( w, h ) };
		CreateSegmentShape( groundId, ref shapeDef, ref floor );
		CreateSegmentShape( groundId, ref shapeDef, ref leftWall );
		CreateSegmentShape( groundId, ref shapeDef, ref rightWall );

		float scale = 1.0f;
		int index = 0;
		for ( int i = 0; i < e_rowCount; ++i )
		{
			for ( int j = 0; j < e_columnCount; ++j )
			{
				float x = 2.4f * scale * ( j - 0.5f * ( e_columnCount - 1 ) ) + RandomFloatRange( -0.3f, 0.3f );
				float y = 2.0f + 2.2f * scale * i;
				Pos p = OffsetPos( origin, new( x, y ) );
				CreateHuman( ref m_humans[index], m_worldId, p, scale, 0.05f, 1.0f, 0.5f, index + 1, null, false );
				index += 1;
			}
		}

		m_humanCount = index;
	}

	public override void Step(  )
	{
		base.Step();

		if ( IsDoublePrecision() == true )
		{
			DrawScreenTextLine( "Double precision" );
		}
		else
		{
			DrawScreenTextLine( "Single precision" );
		}

		Pos c = m_context.camera.center;
		DrawScreenTextLine( "%d ragdolls piled %.0f km from the world origin", m_humanCount, 0.001f * c.x );
	}

	public static Sample Create( SampleContext context )
	{
		return new FarRagdolls( context );
	}

	public const int e_columnCount = 5;
	public const int e_rowCount = 5;
	public Human[] m_humans = new Human[e_columnCount * e_rowCount];
	public int m_humanCount = 0;
}

public class FarGate : Sample
{
	public FarGate( SampleContext context ) : base( context )
	{
		// 1e6 is exactly representable in float so the placement adds no error of its own. Any
		// difference between the two builds comes from how positions are stored as the sim runs.
		Pos origin = new Pos { x = 1.0e6f, y = 0.0f };
		m_origin = origin;

		if ( m_context.restart == false )
		{
			m_context.camera.center = origin + new Vec2( 0.0f, 6.0f );
			m_context.camera.zoom = 7.0f;
			m_context.debugDraw.drawJoints = false;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = origin;
			groundId = CreateBody( m_worldId, ref bodyDef );

			string path =
				"m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
				"-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
				"-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
				"-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166";

			Vec2[] points = new Vec2[128];

			Vec2 offset = new Vec2( -120.0f, -200.0f );
			float scale = 0.2f;
			int count = ParsePath( path, offset, points, 64, scale, false );

			SurfaceMaterial material = DefaultSurfaceMaterial();
			material.customColor = (uint)HexColor.colorDarkSeaGreen;

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;
			chainDef.materials = [material];
			chainDef.materialCount = 1;

			CreateChain( groundId, ref chainDef );
		}

		float gearRadius = 1.0f;
		float toothHalfWidth = 0.09f;
		float toothHalfHeight = 0.06f;
		float toothRadius = 0.03f;
		float linkHalfLength = 0.07f;
		float linkRadius = 0.05f;
		float linkCount = 40;
		float doorHalfHeight = 1.5f;

		Pos gearPosition1 = origin + new Vec2( -4.25f, 9.75f );
		Pos gearPosition2 = gearPosition1 + new Vec2( 2.0f, 1.0f );
		Pos linkAttachPosition = gearPosition2 + new Vec2( gearRadius + 2.0f * toothHalfWidth + toothRadius, 0.0f );
		Pos doorPosition = linkAttachPosition - new Vec2( 0.0f, 2.0f * linkCount * linkHalfLength + doorHalfHeight );

		{
			Pos position = gearPosition1;
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = position;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = (uint)HexColor.colorSaddleBrown;
			Circle circle = new Circle { center = Vec2_zero, radius = gearRadius };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			int count = 16;
			float deltaAngle = 2.0f * Pi / 16;
			Rot dq = MakeRot( deltaAngle );
			Vec2 center = new Vec2( gearRadius + toothHalfHeight, 0.0f );
			Rot rotation = Rot_identity;

			for ( int i = 0; i < count; ++i )
			{
				Polygon tooth = MakeOffsetRoundedBox( toothHalfWidth, toothHalfHeight, center, rotation, toothRadius );
				shapeDef.material.customColor = (uint)HexColor.colorGray;
				CreatePolygonShape( bodyId, ref shapeDef, ref tooth );

				rotation = MulRot( dq, rotation );
				center = RotateVector( rotation, new( gearRadius + toothHalfHeight, 0.0f ) );
			}

			RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();

			m_motorTorque = 80.0f;
			m_motorSpeed = -0.3f;
			m_enableMotor = true;

			revoluteDef.@base.bodyIdA = groundId;
			revoluteDef.@base.bodyIdB = bodyId;
			revoluteDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, position );
			revoluteDef.@base.localFrameB.p = Vec2_zero;
			revoluteDef.enableMotor = m_enableMotor;
			revoluteDef.maxMotorTorque = m_motorTorque;
			revoluteDef.motorSpeed = m_motorSpeed;
			m_driverId = CreateRevoluteJoint( m_worldId, ref revoluteDef );
		}

		BodyId followerId = default;
		{
			Pos position = gearPosition2;
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = position;

			followerId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = (uint)HexColor.colorSaddleBrown;
			Circle circle = new Circle { center = Vec2_zero, radius = gearRadius };
			CreateCircleShape( followerId, ref shapeDef, ref circle );

			int count = 16;
			float deltaAngle = 2.0f * Pi / 16;
			Rot dq = MakeRot( deltaAngle );
			Vec2 center = new Vec2( gearRadius + toothHalfWidth, 0.0f );
			Rot rotation = Rot_identity;

			for ( int i = 0; i < count; ++i )
			{
				Polygon tooth = MakeOffsetRoundedBox( toothHalfWidth, toothHalfHeight, center, rotation, toothRadius );
				shapeDef.material.customColor = (uint)HexColor.colorGray;
				CreatePolygonShape( followerId, ref shapeDef, ref tooth );

				rotation = MulRot( dq, rotation );
				center = RotateVector( rotation, new( gearRadius + toothHalfWidth, 0.0f ) );
			}

			RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();

			revoluteDef.@base.bodyIdA = groundId;
			revoluteDef.@base.bodyIdB = followerId;
			revoluteDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, position );
			revoluteDef.@base.localFrameA.q = MakeRot( 0.25f * Pi );
			revoluteDef.@base.localFrameB.p = Vec2_zero;
			revoluteDef.enableMotor = true;
			revoluteDef.maxMotorTorque = 0.5f;
			revoluteDef.lowerAngle = -0.3f * Pi;
			revoluteDef.upperAngle = 0.8f * Pi;
			revoluteDef.enableLimit = true;
			CreateRevoluteJoint( m_worldId, ref revoluteDef );
		}

		BodyId lastLinkId = default;
		{
			Capsule capsule = new Capsule { center1 = new( 0.0f, -linkHalfLength ), center2 = new( 0.0f, linkHalfLength ), radius = linkRadius };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;
			shapeDef.material.customColor = (uint)HexColor.colorLightSteelBlue;

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.maxMotorTorque = 0.05f;
			jointDef.enableMotor = true;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			Pos position = linkAttachPosition + new Vec2( 0.0f, -linkHalfLength );

			int count = 40;
			BodyId prevBodyId = followerId;
			for ( int i = 0; i < count; ++i )
			{
				bodyDef.position = position;

				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

				Pos pivot = new Pos { x = position.x, y = position.y + linkHalfLength };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
				jointDef.@base.drawScale = 0.2f;
				CreateRevoluteJoint( m_worldId, ref jointDef );

				position.y -= 2.0f * linkHalfLength;
				prevBodyId = bodyId;
			}

			lastLinkId = prevBodyId;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = doorPosition;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.15f, doorHalfHeight );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = (uint)HexColor.colorDarkCyan;
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			{
				Pos pivot = doorPosition + new Vec2( 0.0f, doorHalfHeight );
				RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();
				revoluteDef.@base.bodyIdA = lastLinkId;
				revoluteDef.@base.bodyIdB = bodyId;
				revoluteDef.@base.localFrameA.p = Body_GetLocalPoint( lastLinkId, pivot );
				revoluteDef.@base.localFrameB.p = new( 0.0f, doorHalfHeight );
				revoluteDef.enableMotor = true;
				revoluteDef.maxMotorTorque = 0.05f;
				CreateRevoluteJoint( m_worldId, ref revoluteDef );
			}

			{
				Vec2 localAxis = new Vec2( 0.0f, 1.0f );
				PrismaticJointDef jointDef = DefaultPrismaticJointDef();
				jointDef.@base.bodyIdA = groundId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, doorPosition );
				jointDef.@base.localFrameA.q = MakeRotFromUnitVector( localAxis );
				jointDef.@base.localFrameB.p = Vec2_zero;
				jointDef.@base.localFrameB.q = MakeRotFromUnitVector( localAxis );
				jointDef.maxMotorForce = 0.2f;
				jointDef.enableMotor = true;
				jointDef.@base.collideConnected = true;
				CreatePrismaticJoint( m_worldId, ref jointDef );
			}
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.rollingResistance = 0.3f;

			HexColor[] colors = {
				HexColor.colorGray, HexColor.colorGainsboro, HexColor.colorLightGray, HexColor.colorLightSlateGray, HexColor.colorDarkGray,
			};

			float y = 4.25f;
			int xCount = 10, yCount = 20;
			for ( int i = 0; i < yCount; ++i )
			{
				float x = -3.15f;
				for ( int j = 0; j < xCount; ++j )
				{
					bodyDef.position = origin + new Vec2( x, y );
					BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
					Polygon poly = RandomPolygon( 0.1f );
					poly.radius = RandomFloatRange( 0.01f, 0.02f );

					int colorIndex = RandomIntRange( 0, 4 );
					shapeDef.material.customColor = (uint)colors[colorIndex];

					CreatePolygonShape( bodyId, ref shapeDef, ref poly );
					x += 0.2f;
				}

				y += 0.2f;
			}
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			RevoluteJoint_EnableMotor( m_driverId, m_enableMotor );
			Joint_WakeBodies( m_driverId );
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Max Torque", ref m_motorTorque, 0.0f, 100.0f, "%.0f" ) )
		{
			RevoluteJoint_SetMaxMotorTorque( m_driverId, m_motorTorque );
			Joint_WakeBodies( m_driverId );
		}

		if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -0.3f, 0.3f, "%.2f" ) )
		{
			RevoluteJoint_SetTargetMotorSpeed( m_driverId, m_motorSpeed );
			Joint_WakeBodies( m_driverId );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( ( glfwGetKey( m_context.window, GLFW_KEY_A ) != 0 ) )
		{
			m_motorSpeed = MaxFloat( -0.3f, m_motorSpeed - 0.01f );
			RevoluteJoint_SetTargetMotorSpeed( m_driverId, m_motorSpeed );
			Joint_WakeBodies( m_driverId );
		}

		if ( ( glfwGetKey( m_context.window, GLFW_KEY_D ) != 0 ) )
		{
			m_motorSpeed = MinFloat( 0.3f, m_motorSpeed + 0.01f );
			RevoluteJoint_SetTargetMotorSpeed( m_driverId, m_motorSpeed );
			Joint_WakeBodies( m_driverId );
		}

		base.Step();

		float fx = (float)m_origin.x;
		float gridStep = MathF.BitIncrement( fx ) - fx;

		if ( IsDoublePrecision() == true )
		{
			DrawScreenTextLine( "Double precision" );
		}
		else
		{
			DrawScreenTextLine( "Single precision" );
		}

		DrawScreenTextLine( "%.0f km from the origin", m_origin.x / 1000.0f );
		DrawScreenTextLine( "float grid step here = %g m, gear teeth are about 0.12 m", gridStep );
	}

	public static Sample Create( SampleContext context )
	{
		return new FarGate( context );
	}

	public Pos m_origin;
	public JointId m_driverId;
	public float m_motorTorque;
	public float m_motorSpeed;
	public bool m_enableMotor;
}
