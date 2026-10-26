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

public static partial class Helpers_sample_bodies
{
	public static float FrictionCallback( float unused1, ulong unused2, float unused3, ulong unused4 )
	{
		return 0.1f;
	}

	public static float RestitutionCallback( float unused5, ulong unused6, float unused7, ulong unused8 )
	{
		return 1.0f;
	}










	// This shows how to set the initial angular velocity to get a specific movement.



	// This shows how to drive a kinematic body to reach a target



	// Motion locking can be a bit squishy
}

public class BodyTypeSample : Sample
{
	public BodyTypeSample( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.8f, 6.4f );
			m_context.camera.zoom = 25.0f * 0.4f;
		}

		m_type = BodyType.staticBody;
		m_isEnabled = true;

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "ground";
			groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Define attachment
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -2.0f, y = 3.0f };
			bodyDef.name = "attach1";
			m_attachmentId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.5f, 2.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreatePolygonShape( m_attachmentId, ref shapeDef, ref box );
		}

		// Define second attachment
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = 3.0f, y = 3.0f };
			bodyDef.name = "attach2";
			m_secondAttachmentId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.5f, 2.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreatePolygonShape( m_secondAttachmentId, ref shapeDef, ref box );
		}

		// Define platform
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = -4.0f, y = 5.0f };
			bodyDef.name = "platform";
			m_platformId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeOffsetBox( 0.5f, 4.0f, new( 4.0f, 0.0f ), MakeRot( 0.5f * Pi ) );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;
			CreatePolygonShape( m_platformId, ref shapeDef, ref box );

			RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();
			Pos pivot = new Pos { x = -2.0f, y = 5.0f };
			revoluteDef.@base.bodyIdA = m_attachmentId;
			revoluteDef.@base.bodyIdB = m_platformId;
			revoluteDef.@base.localFrameA.p = Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.@base.localFrameB.p = Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			CreateRevoluteJoint( m_worldId, ref revoluteDef );

			pivot = new Pos { x = 3.0f, y = 5.0f };
			revoluteDef.@base.bodyIdA = m_secondAttachmentId;
			revoluteDef.@base.bodyIdB = m_platformId;
			revoluteDef.@base.localFrameA.p = Body_GetLocalPoint( m_secondAttachmentId, pivot );
			revoluteDef.@base.localFrameB.p = Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			CreateRevoluteJoint( m_worldId, ref revoluteDef );

			PrismaticJointDef prismaticDef = DefaultPrismaticJointDef();
			Pos anchor = new Pos { x = 0.0f, y = 5.0f };
			prismaticDef.@base.bodyIdA = groundId;
			prismaticDef.@base.bodyIdB = m_platformId;
			prismaticDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, anchor );
			prismaticDef.@base.localFrameB.p = Body_GetLocalPoint( m_platformId, anchor );
			prismaticDef.maxMotorForce = 1000.0f;
			prismaticDef.motorSpeed = 0.0f;
			prismaticDef.enableMotor = true;
			prismaticDef.lowerTranslation = -10.0f;
			prismaticDef.upperTranslation = 10.0f;
			prismaticDef.enableLimit = true;

			CreatePrismaticJoint( m_worldId, ref prismaticDef );

			m_speed = 3.0f;
		}

		// Create a payload
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -3.0f, y = 8.0f };
			bodyDef.name = "crate1";
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.75f, 0.75f );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;

			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// Create a second payload
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = 2.0f, y = 8.0f };
			bodyDef.name = "crate2";
			m_secondPayloadId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.75f, 0.75f );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;

			CreatePolygonShape( m_secondPayloadId, ref shapeDef, ref box );
		}

		// Create a separate body on the ground
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = 8.0f, y = 0.2f };
			bodyDef.name = "debris";
			m_touchingBodyId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = new( 1.0f, 0.0f ), radius = 0.25f };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;

			CreateCapsuleShape( m_touchingBodyId, ref shapeDef, ref capsule );
		}

		// Create a separate body on the ground
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.staticBody;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = 8.5f, y = 0.2f };
			bodyDef.name = "debris";
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = new( 1.0f, 0.0f ), radius = 0.5f };

			ShapeDef shapeDef = DefaultShapeDef();
			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
		}

		// Create a separate floating body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = new Pos { x = -8.0f, y = 12.0f };
			bodyDef.gravityScale = 0.0f;
			bodyDef.name = "floater";
			m_floatingBodyId = CreateBody( m_worldId, ref bodyDef );

			Circle circle = new Circle { center = new( 0.0f, 0.5f ), radius = 0.25f };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 2.0f;

			CreateCircleShape( m_floatingBodyId, ref shapeDef, ref circle );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.RadioButton( "Static", m_type == BodyType.staticBody ) )
		{
			m_type = BodyType.staticBody;
			Body_SetType( m_platformId, BodyType.staticBody );
			Body_SetType( m_secondAttachmentId, BodyType.staticBody );
			Body_SetType( m_secondPayloadId, BodyType.staticBody );
			Body_SetType( m_touchingBodyId, BodyType.staticBody );
			Body_SetType( m_floatingBodyId, BodyType.staticBody );
		}

		if ( ImGui.RadioButton( "Kinematic", m_type == BodyType.kinematicBody ) )
		{
			m_type = BodyType.kinematicBody;
			Body_SetType( m_platformId, BodyType.kinematicBody );
			Body_SetLinearVelocity( m_platformId, new( -m_speed, 0.0f ) );
			Body_SetAngularVelocity( m_platformId, 0.0f );

			Body_SetType( m_secondAttachmentId, BodyType.kinematicBody );
			Body_SetLinearVelocity( m_secondAttachmentId, Vec2_zero );
			Body_SetAngularVelocity( m_secondAttachmentId, 0.0f );

			Body_SetType( m_secondPayloadId, BodyType.kinematicBody );
			Body_SetType( m_touchingBodyId, BodyType.kinematicBody );
			Body_SetType( m_floatingBodyId, BodyType.kinematicBody );
		}

		if ( ImGui.RadioButton( "Dynamic", m_type == BodyType.dynamicBody ) )
		{
			m_type = BodyType.dynamicBody;
			Body_SetType( m_platformId, BodyType.dynamicBody );
			Body_SetType( m_secondAttachmentId, BodyType.dynamicBody );
			Body_SetType( m_secondPayloadId, BodyType.dynamicBody );
			Body_SetType( m_touchingBodyId, BodyType.dynamicBody );
			Body_SetType( m_floatingBodyId, BodyType.dynamicBody );
		}

		if ( ImGui.Checkbox( "Enable", ref m_isEnabled ) )
		{
			if ( m_isEnabled )
			{
				Body_Enable( m_attachmentId );
				Body_Enable( m_secondPayloadId );
				Body_Enable( m_floatingBodyId );
			}
			else
			{
				Body_Disable( m_attachmentId );
				Body_Disable( m_secondPayloadId );
				Body_Disable( m_floatingBodyId );
			}
		}

		return true;
	}

	public override void Step(  )
	{
		// Drive the kinematic body.
		if ( m_type == BodyType.kinematicBody )
		{
			Pos p = Body_GetPosition( m_platformId );
			Vec2 v = Body_GetLinearVelocity( m_platformId );

			if ( ( p.x < -14.0f && v.x < 0.0f ) || ( p.x > 6.0f && v.x > 0.0f ) )
			{
				v.x = -v.x;
				Body_SetLinearVelocity( m_platformId, v );
			}
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BodyTypeSample( context );
	}

	public BodyId m_attachmentId;
	public BodyId m_secondAttachmentId;
	public BodyId m_platformId;
	public BodyId m_secondPayloadId;
	public BodyId m_touchingBodyId;
	public BodyId m_floatingBodyId;
	public BodyType m_type;
	public float m_speed;
	public bool m_isEnabled;
}

public class Weeble : Sample
{
	public Weeble( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.3f, 10.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		// Test friction and restitution callbacks
		World_SetFrictionCallback( m_worldId, FrictionCallback );
		World_SetRestitutionCallback( m_worldId, RestitutionCallback );

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Build weeble
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 3.0f };
			bodyDef.rotation = MakeRot( 0.25f * Pi );
			m_weebleId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, -1.0f ), center2 = new( 0.0f, 1.0f ), radius = 1.0f };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateCapsuleShape( m_weebleId, ref shapeDef, ref capsule );

			float mass = Body_GetMass( m_weebleId );
			float inertiaTensor = Body_GetRotationalInertia( m_weebleId );

			float offset = 1.5f;

			// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
			inertiaTensor += mass * offset * offset;

			MassData massData = new MassData { mass = mass, center = new( 0.0f, -offset ), rotationalInertia = inertiaTensor };
			Body_SetMassData( m_weebleId, massData );
		}

		m_explosionPosition = new Pos { x = 0.0f, y = 0.0f };
		m_explosionRadius = 2.0f;
		m_explosionMagnitude = 8.0f;
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Teleport" ) )
		{
			Body_SetTransform( m_weebleId, new( 0.0f, 5.0f ), MakeRot( 0.95f * Pi ) );
		}

		if ( ImGui.Button( "Explode" ) )
		{
			ExplosionDef def = DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = m_explosionRadius;
			def.falloff = 0.1f;
			def.impulsePerLength = m_explosionMagnitude;
			World_Explode( m_worldId, ref def );
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "Magnitude", ref m_explosionMagnitude, -100.0f, 100.0f, "%.1f" );
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		DrawCircle( m_context.draw, m_explosionPosition, m_explosionRadius, HexColor.colorAzure );

		// This shows how to get the velocity of a point on a body
		Vec2 localPoint = new Vec2( 0.0f, 2.0f );
		Pos worldPoint = Body_GetWorldPoint( m_weebleId, localPoint );

		Vec2 v1 = Body_GetLocalPointVelocity( m_weebleId, localPoint );
		Vec2 v2 = Body_GetWorldPointVelocity( m_weebleId, worldPoint );

		Vec2 offset = new Vec2( 0.05f, 0.0f );
		DrawLine( m_context.draw, worldPoint, worldPoint + v1, HexColor.colorRed );
		DrawLine( m_context.draw, worldPoint + offset, worldPoint + v2 + offset, HexColor.colorGreen );
	}

	public static Sample Create( SampleContext context )
	{
		return new Weeble( context );
	}

	public BodyId m_weebleId;
	public Pos m_explosionPosition;
	public float m_explosionRadius;
	public float m_explosionMagnitude;
}

public class Sleep : Sample
{
	public Sleep( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 3.0f, 50.0f );
			m_context.camera.zoom = 25.0f * 2.2f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			m_groundShapeId = CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Sleeping body with sensors
		for ( int i = 0; i < 2; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -4.0f, y = 3.0f + 2.0f * i };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 1.0f ), center2 = new( 1.0f, 1.0f ), radius = 0.75f };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			capsule.radius = 1.0f;
			m_sensorIds[i] = CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
			m_sensorTouching[i] = false;
		}

		// Sleeping body but sleep is disabled
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 3.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = false;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Circle circle = new Circle { center = new( 1.0f, 1.0f ), radius = 1.0f };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		// Awake body and sleep is disabled
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 5.0f, y = 3.0f };
			bodyDef.isAwake = true;
			bodyDef.enableSleep = false;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeOffsetBox( 1.0f, 1.0f, new( 0.0f, 1.0f ), MakeRot( 0.25f * Pi ) );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// A sleeping body to test waking on collision
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 5.0f, y = 1.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeSquare( 1.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// A long pendulum
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 100.0f };
			bodyDef.angularDamping = 0.5f;
			bodyDef.sleepThreshold = 0.05f;
			m_pendulumId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = new( 90.0f, 0.0f ), radius = 0.25f };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateCapsuleShape( m_pendulumId, ref shapeDef, ref capsule );

			Pos pivot = bodyDef.position;
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_pendulumId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		// A sleeping body to test waking on contact destroyed
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -10.0f, y = 1.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeSquare( 1.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		m_staticBodyId = nullBodyId;
	}

	public void ToggleInvoker(  )
	{
		if ( m_staticBodyId.IsNull )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = -10.5f, y = 3.0f };
			m_staticBodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeOffsetBox( 2.0f, 0.1f, new( 0.0f, 0.0f ), MakeRot( 0.25f * Pi ) );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.invokeContactCreation = true;
			CreatePolygonShape( m_staticBodyId, ref shapeDef, ref box );
		}
		else
		{
			DestroyBody( m_staticBodyId );
			m_staticBodyId = nullBodyId;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		ImGui.Text( "Pendulum Tuning" );

		float sleepVelocity = Body_GetSleepThreshold( m_pendulumId );
		if ( ImGui.SliderFloat( "sleep velocity", ref sleepVelocity, 0.0f, 1.0f, "%.2f" ) )
		{
			Body_SetSleepThreshold( m_pendulumId, sleepVelocity );
			Body_SetAwake( m_pendulumId, true );
		}

		float angularDamping = Body_GetAngularDamping( m_pendulumId );
		if ( ImGui.SliderFloat( "angular damping", ref angularDamping, 0.0f, 2.0f, "%.2f" ) )
		{
			Body_SetAngularDamping( m_pendulumId, angularDamping );
		}

		ImGui.PopItemWidth();

		ImGui.Separator();

		if ( m_staticBodyId.IsNull )
		{
			if ( ImGui.Button( "Create" ) )
			{
				ToggleInvoker();
			}
		}
		else
		{
			if ( ImGui.Button( "Destroy" ) )
			{
				ToggleInvoker();
			}
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		// Detect sensors touching the ground
		SensorEvents sensorEvents = World_GetSensorEvents( m_worldId );

		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			ref SensorBeginTouchEvent @event = ref sensorEvents.beginEvents[i];
			if ( ( @event.visitorShapeId ).Equals( m_groundShapeId ) )
			{
				if ( ( @event.sensorShapeId ).Equals( m_sensorIds[0] ) )
				{
					m_sensorTouching[0] = true;
				}
				else if ( ( @event.sensorShapeId ).Equals( m_sensorIds[1] ) )
				{
					m_sensorTouching[1] = true;
				}
			}
		}

		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			ref SensorEndTouchEvent @event = ref sensorEvents.endEvents[i];
			if ( ( @event.visitorShapeId ).Equals( m_groundShapeId ) )
			{
				if ( ( @event.sensorShapeId ).Equals( m_sensorIds[0] ) )
				{
					m_sensorTouching[0] = false;
				}
				else if ( ( @event.sensorShapeId ).Equals( m_sensorIds[1] ) )
				{
					m_sensorTouching[1] = false;
				}
			}
		}

		for ( int i = 0; i < 2; ++i )
		{
			DrawScreenTextLine( "sensor touch %d = %s", i, m_sensorTouching[i] ? "true" : "false" );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new Sleep( context );
	}

	public BodyId m_pendulumId;
	public BodyId m_staticBodyId;
	public ShapeId m_groundShapeId;
	public ShapeId[] m_sensorIds = new ShapeId[2];
	public bool[] m_sensorTouching = new bool[2];
}

public class BadBody : Sample
{
	public BadBody( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.3f, 10.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Build a bad body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 3.0f };
			bodyDef.angularVelocity = 0.5f;
			bodyDef.rotation = MakeRot( 0.25f * Pi );

			m_badBodyId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, -1.0f ), center2 = new( 0.0f, 1.0f ), radius = 1.0f };
			ShapeDef shapeDef = DefaultShapeDef();

			// density set to zero intentionally to create a bad body
			shapeDef.density = 0.0f;
			CreateCapsuleShape( m_badBodyId, ref shapeDef, ref capsule );
		}

		// Build a normal body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 2.0f, y = 3.0f };
			bodyDef.rotation = MakeRot( 0.25f * Pi );

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, -1.0f ), center2 = new( 0.0f, 1.0f ), radius = 1.0f };
			ShapeDef shapeDef = DefaultShapeDef();

			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
		}
	}

	public override void Step(  )
	{
		base.Step();

		DrawScreenTextLine( "A bad body is a dynamic body with no mass and behaves like a kinematic body." );
		DrawScreenTextLine( "Bad bodies are considered invalid and a user bug. Behavior is not guaranteed." );

		// For science
		Body_ApplyForceToCenter( m_badBodyId, new( 0.0f, 10.0f ), true );
	}

	public static Sample Create( SampleContext context )
	{
		return new BadBody( context );
	}

	public BodyId m_badBodyId;
}

public class Pivot : Sample
{
	public Pivot( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.8f, 6.4f );
			m_context.camera.zoom = 25.0f * 0.4f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			ShapeDef shapeDef = DefaultShapeDef();
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Create a separate body on the ground
		{
			Vec2 v = new Vec2( 5.0f, 0.0f );

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 3.0f };
			bodyDef.gravityScale = 1.0f;
			bodyDef.linearVelocity = v;

			m_bodyId = CreateBody( m_worldId, ref bodyDef );

			m_lever = 3.0f;
			Vec2 r = new Vec2( 0.0f, -m_lever );

			float omega = Cross( v, r ) / Dot( r, r );
			Body_SetAngularVelocity( m_bodyId, omega );

			Polygon box = MakeBox( 0.1f, m_lever );

			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}
	}

	public override void Step(  )
	{
		base.Step();

		Vec2 v = Body_GetLinearVelocity( m_bodyId );
		float omega = Body_GetAngularVelocity( m_bodyId );
		Vec2 r = Body_GetWorldVector( m_bodyId, new( 0.0f, -m_lever ) );

		Vec2 vp = v + CrossSV( omega, r );
		DrawScreenTextLine( "pivot velocity = (%g, %g)", vp.x, vp.y );
	}

	public static Sample Create( SampleContext context )
	{
		return new Pivot( context );
	}

	public BodyId m_bodyId;
	public float m_lever;
}

public class Kinematic : Sample
{
	public Kinematic( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 4.0f;
		}

		m_amplitude = 2.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position.x = 2.0f * m_amplitude;

			m_bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 0.1f, 1.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}

		m_time = 0.0f;
	}

	public override void Step(  )
	{
		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
		if ( m_context.pause && m_context.singleStep == 0 )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f )
		{
			Pos point = new Pos { x = 2.0f * m_amplitude * MathF.Cos( m_time ), y = m_amplitude * MathF.Sin( 2.0f * m_time ) };
			Rot rotation = MakeRot( 2.0f * m_time );

			Vec2 axis = RotateVector( rotation, new( 0.0f, 1.0f ) );
			DrawLine( m_context.draw, point - 0.5f * axis, point + 0.5f * axis, HexColor.colorPlum );
			DrawPoint( m_context.draw, point, 10.0f, HexColor.colorPlum );

			bool wake = true;
			Body_SetTargetTransform( m_bodyId, new( point , rotation ), timeStep, wake );
		}

		base.Step();

		m_time += timeStep;
	}

	public static Sample Create( SampleContext context )
	{
		return new Kinematic( context );
	}

	public BodyId m_bodyId;
	public float m_amplitude;
	public float m_time;
}

public class MixedLocks : Sample
{
	public MixedLocks( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 3.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		Polygon box = MakeSquare( 0.5f );
		ShapeDef shapeDef = DefaultShapeDef();

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 2.0f, y = 1.0f };
			bodyDef.name = "static";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 1.0f, y = 1.0f };
			bodyDef.name = "free";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 1.0f, y = 3.0f };
			bodyDef.name = "free";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -1.0f, y = 1.0f };
			bodyDef.motionLocks.angularZ = true;
			bodyDef.name = "angular z";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -2.0f, y = 2.0f };
			bodyDef.motionLocks.linearX = true;
			bodyDef.name = "linear x";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -1.0f, y = 2.5f };
			bodyDef.motionLocks.linearY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.name = "lin y ang z";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
			bodyDef.motionLocks.linearX = true;
			bodyDef.motionLocks.linearY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.name = "full";

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new MixedLocks( context );
	}
}

public class SetVelocity : Sample
{
	public SetVelocity( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 3.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -0.25f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeBox( 20.0f, 0.25f );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeSquare( 0.5f );
			bodyDef.position = new Pos { x = 0.0f, y = 0.5f };
			m_bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );
		}
	}

	public override void Step(  )
	{
		base.Step();

		Body_SetLinearVelocity( m_bodyId, new( 0.0f, -20.0f ) );

		Pos position = Body_GetPosition( m_bodyId );
		DrawScreenTextLine( "(x, y) = (%.2g, %.2g)", position.x, position.y );
	}

	public static Sample Create( SampleContext context )
	{
		return new SetVelocity( context );
	}

	public BodyId m_bodyId;
}

public class WakeTouching : Sample
{
	public WakeTouching( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 4.0f );
			m_context.camera.zoom = 8.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			m_groundId = CreateBody( m_worldId, ref bodyDefInner );

			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreateSegmentShape( m_groundId, ref shapeDefInner, ref segment );
		}

		Polygon box = MakeBox( 0.5f, 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		float x = -1.0f * ( m_count - 1 );

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = new Pos { x = x, y = 4.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
			x += 2.0f;
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Wake Touching" ) )
		{
			Body_WakeTouching( m_groundId );
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new WakeTouching( context );
	}

	public const int m_count = 10;

	public BodyId m_groundId;
}
