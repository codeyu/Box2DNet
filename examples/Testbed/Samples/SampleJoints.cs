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

public static partial class Helpers_sample_joints
{
	// Test the distance joint and all options



	/// This test shows how to use a motor joint. A motor joint
	/// can be used to animate a dynamic body. With finite motor forces
	/// the body can be blocked by collision with other bodies.






	// This sample shows how to use a filter joint to prevent collision between two bodies.
	// This is more specific than shape filters. It also shows that sleeping is coupled by the filter joint.












	// A suspension bridge






	// This sample shows the limitations of an iterative solver. The cantilever sags even though the weld
	// joint is stiff as possible.



	// This test ensures joints work correctly with bodies that have motion locks



	// This sample shows how to break joints when the internal reaction force becomes large.



	// This sample shows how to measure joint separation. This is the unresolved constraint error.



	// This shows how you can implement a constraint outside of Box2D



	// This is a fun demo that shows off the wheel joint


















	// A top down door






	// Theo Jansen's walking machine, inspired by a contribution from roman_m
	// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)



	// The Anglepoise Original 1227 desk lamp.
	// https://www.anglepoise.com/usa/product-category/original-1227/
}

public class DistanceJoint : Sample
{
	public const int e_maxCount = 10;

	public DistanceJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 12.0f );
			m_context.camera.zoom = 25.0f * 0.35f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			m_groundId = CreateBody( m_worldId, ref bodyDef );
		}

		m_count = 0;
		m_hertz = 5.0f;
		m_dampingRatio = 0.5f;
		m_length = 1.0f;
		m_minLength = m_length;
		m_maxLength = m_length;
		m_tensionForce = 2000.0f;
		m_compressionForce = 100.0f;
		m_enableSpring = false;
		m_enableLimit = false;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodyIds[i] = nullBodyId;
			m_jointIds[i] = nullJointId;
		}

		CreateScene( 1 );
	}

	public void CreateScene( int newCount )
	{
		for ( int i = 0; i < m_count; ++i )
		{
			DestroyJoint( m_jointIds[i] );
			m_jointIds[i] = nullJointId;
		}

		for ( int i = 0; i < m_count; ++i )
		{
			DestroyBody( m_bodyIds[i] );
			m_bodyIds[i] = nullBodyId;
		}

		m_count = newCount;

		float radius = 0.25f;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = radius };

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 20.0f;

		float yOffset = 20.0f;

		DistanceJointDef jointDef = DefaultDistanceJointDef();
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.length = m_length;
		jointDef.lowerSpringForce = -m_tensionForce;
		jointDef.upperSpringForce = m_compressionForce;
		jointDef.minLength = m_minLength;
		jointDef.maxLength = m_maxLength;
		jointDef.enableSpring = m_enableSpring;
		jointDef.enableLimit = m_enableLimit;

		BodyId prevBodyId = m_groundId;
		for ( int i = 0; i < m_count; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.angularDamping = 1.0f;
			bodyDef.position = new Pos { x = m_length * ( i + 1.0f ), y = yOffset };
			m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( m_bodyIds[i], ref shapeDef, ref circle );

			Pos pivotA = new Pos { x = m_length * i, y = yOffset };
			Pos pivotB = new Pos { x = m_length * ( i + 1.0f ), y = yOffset };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = m_bodyIds[i];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotA );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotB );
			m_jointIds[i] = CreateDistanceJoint( m_worldId, ref jointDef );

			prevBodyId = m_bodyIds[i];
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Length", ref m_length, 0.1f, 4.0f, "%3.1f" ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DistanceJoint_SetRestLength( m_jointIds[i], m_length );
				Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( ImGui.Checkbox( "Spring", ref m_enableSpring ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DistanceJoint_EnableSpring( m_jointIds[i], m_enableSpring );
				Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableSpring )
		{
			if ( ImGui.SliderFloat( "Tension", ref m_tensionForce, 0.0f, 4000.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui.SliderFloat( "Compression", ref m_compressionForce, 0.0f, 200.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 15.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetSpringHertz( m_jointIds[i], m_hertz );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui.SliderFloat( "Damping", ref m_dampingRatio, 0.0f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetSpringDampingRatio( m_jointIds[i], m_dampingRatio );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		if ( ImGui.Checkbox( "Limit", ref m_enableLimit ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DistanceJoint_EnableLimit( m_jointIds[i], m_enableLimit );
				Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableLimit )
		{
			if ( ImGui.SliderFloat( "Min Length", ref m_minLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetLimits( m_jointIds[i], m_minLength, m_maxLength );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui.SliderFloat( "Max Length", ref m_maxLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					DistanceJoint_SetLimits( m_jointIds[i], m_minLength, m_maxLength );
					Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		int count = m_count;
		if ( ImGui.SliderInt( "Count", ref count, 1, e_maxCount ) )
		{
			CreateScene( count );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new DistanceJoint( context );
	}

	public BodyId m_groundId;
	public BodyId[] m_bodyIds = new BodyId[e_maxCount];
	public JointId[] m_jointIds = new JointId[e_maxCount];
	public int m_count;
	public float m_hertz;
	public float m_dampingRatio;
	public float m_length;
	public float m_tensionForce;
	public float m_compressionForce;
	public float m_minLength;
	public float m_maxLength;
	public bool m_enableSpring;
	public bool m_enableLimit;
}

public class MotorJoint : Sample
{
	public MotorJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 7.0f );
			m_context.camera.zoom = 25.0f * 0.4f;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_transform = new() { p = new( 0.0f, 8.0f ), q = Rot_identity };

		// Define a target body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = m_transform.p;
			m_targetId = CreateBody( m_worldId, ref bodyDef );
		}

		// Define motorized body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = m_transform.p;
			m_bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 2.0f, 0.5f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyId, ref shapeDef, ref box );

			m_maxForce = 5000.0f;
			m_maxTorque = 500.0f;

			MotorJointDef jointDef = DefaultMotorJointDef();
			jointDef.@base.bodyIdA = m_targetId;
			jointDef.@base.bodyIdB = m_bodyId;
			jointDef.linearHertz = 4.0f;
			jointDef.linearDampingRatio = 0.7f;
			jointDef.angularHertz = 4.0f;
			jointDef.angularDampingRatio = 0.7f;
			jointDef.maxSpringForce = m_maxForce;
			jointDef.maxSpringTorque = m_maxTorque;

			m_jointId = CreateMotorJoint( m_worldId, ref jointDef );
		}

		// Define spring body
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -2.0f, y = 2.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeSquare( 0.5f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			MotorJointDef jointDef = DefaultMotorJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Add( ToVec2( bodyDef.position ), new( 0.25f, 0.25f ) );
			jointDef.@base.localFrameB.p = new( 0.25f, 0.25f );
			jointDef.linearHertz = 7.5f;
			jointDef.linearDampingRatio = 0.7f;
			jointDef.angularHertz = 7.5f;
			jointDef.angularDampingRatio = 0.7f;
			jointDef.maxSpringForce = 500.0f;
			jointDef.maxSpringTorque = 10.0f;

			CreateMotorJoint( m_worldId, ref jointDef );
		}

		m_speed = 1.0f;
		m_time = 0.0f;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Speed", ref m_speed, -5.0f, 5.0f, "%.0f" ) )
		{
		}

		if ( ImGui.SliderFloat( "Max Force", ref m_maxForce, 0.0f, 10000.0f, "%.0f" ) )
		{
			MotorJoint_SetMaxSpringForce( m_jointId, m_maxForce );
		}

		if ( ImGui.SliderFloat( "Max Torque", ref m_maxTorque, 0.0f, 10000.0f, "%.0f" ) )
		{
			MotorJoint_SetMaxSpringTorque( m_jointId, m_maxTorque );
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Apply Impulse" ) )
		{
			Body_ApplyLinearImpulseToCenter( m_bodyId, new( 100.0f, 0.0f ), true );
		}

		return true;
	}

	public override void Step(  )
	{
		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;

		if ( m_context.pause )
		{
			if ( m_context.singleStep == 0 )
			{
				timeStep = 0.0f;
			}
		}

		if ( timeStep > 0.0f )
		{
			m_time += m_speed * timeStep;

			Pos linearOffset = default;
			linearOffset.x = 6.0f * MathF.Sin( 2.0f * m_time );
			linearOffset.y = 8.0f + 4.0f * MathF.Sin( 1.0f * m_time );

			float angularOffset = 2.0f * m_time;
			m_transform = new WorldTransform { p = linearOffset, q = MakeRot( angularOffset ) };

			bool wake = true;
			Body_SetTargetTransform( m_targetId, m_transform, timeStep, wake );
		}

		DrawTransform( m_draw, m_transform, 1.0f );

		base.Step();

		Vec2 force = Joint_GetConstraintForce( m_jointId );
		float torque = Joint_GetConstraintTorque( m_jointId );

		DrawScreenTextLine( "force = new( %3.0f, %3.0f ), torque = %3.0f", force.x, force.y, torque );
	}

	public static Sample Create( SampleContext context )
	{
		return new MotorJoint( context );
	}

	public BodyId m_targetId;
	public BodyId m_bodyId;
	public JointId m_jointId;
	public WorldTransform m_transform;
	public float m_time;
	public float m_speed;
	public float m_maxForce;
	public float m_maxTorque;
}

public class TopDownFriction : Sample
{
	public TopDownFriction( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 12.0f;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDefInner );
			ShapeDef shapeDefInner = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );

			segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( -10.0f, 20.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );

			segment = new Segment { point1 = new( 10.0f, 0.0f ), point2 = new( 10.0f, 20.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );

			segment = new Segment { point1 = new( -10.0f, 20.0f ), point2 = new( 10.0f, 20.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		MotorJointDef jointDef = DefaultMotorJointDef();
		jointDef.@base.bodyIdA = groundId;
		jointDef.@base.collideConnected = true;
		jointDef.maxVelocityForce = 10.0f;
		jointDef.maxVelocityTorque = 10.0f;

		Capsule capsule = new Capsule { center1 = new( -0.25f, 0.0f ), center2 = new( 0.25f, 0.0f ), radius = 0.25f };
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.35f };
		Polygon square = MakeSquare( 0.35f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 0.0f;
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.restitution = 0.8f;

		int n = 10;
		float x = -5.0f, y = 15.0f;
		for ( int i = 0; i < n; ++i )
		{
			for ( int j = 0; j < n; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				int remainder = ( n * i + j ) % 4;
				if ( remainder == 0 )
				{
					CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );
				}
				else if ( remainder == 1 )
				{
					CreateCircleShape( bodyId, ref shapeDef, ref circle );
				}
				else if ( remainder == 2 )
				{
					CreatePolygonShape( bodyId, ref shapeDef, ref square );
				}
				else
				{
					Polygon poly = RandomPolygon( 0.75f );
					poly.radius = 0.1f;
					CreatePolygonShape( bodyId, ref shapeDef, ref poly );
				}

				jointDef.@base.bodyIdB = bodyId;
				CreateMotorJoint( m_worldId, ref jointDef );

				x += 1.0f;
			}

			x = -5.0f;
			y -= 1.0f;
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "Explode" ) )
		{
			ExplosionDef def = DefaultExplosionDef();
			def.position = new Pos { x = 0.0f, y = 10.0f };
			def.radius = 10.0f;
			def.falloff = 5.0f;
			def.impulsePerLength = 10.0f;
			World_Explode( m_worldId, ref def );

			DrawCircle( m_draw, def.position, 10.0f, HexColor.colorWhite );
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new TopDownFriction( context );
	}
}

public class FilterJoint : Sample
{
	public FilterJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 7.0f );
			m_context.camera.zoom = 25.0f * 0.4f;
		}

		{
			BodyId groundId = default;
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -4.0f, y = 2.0f };
			BodyId bodyId1 = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeSquare( 2.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( bodyId1, ref shapeDef, ref box );

			bodyDef.position = new Pos { x = 4.0f, y = 2.0f };
			BodyId bodyId2 = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId2, ref shapeDef, ref box );

			FilterJointDef jointDef = DefaultFilterJointDef();
			jointDef.@base.bodyIdA = bodyId1;
			jointDef.@base.bodyIdB = bodyId2;

			CreateFilterJoint( m_worldId, ref jointDef );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new FilterJoint( context );
	}
}

public class RevoluteJoint : Sample
{
	public RevoluteJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 15.5f );
			m_context.camera.zoom = 25.0f * 0.7f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			groundId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 40.0f, 1.0f );

			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_enableSpring = false;
		m_enableLimit = false;
		m_enableMotor = false;
		m_hertz = 2.0f;
		m_dampingRatio = 0.5f;
		m_targetDegrees = 45.0f;
		m_motorSpeed = 1.0f;
		m_motorTorque = 1000.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -10.0f, y = 20.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;

			Capsule capsule = new Capsule { center1 = new( 0.0f, -1.0f ), center2 = new( 0.0f, 6.0f ), radius = 0.5f };
			CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

			Pos pivot = new Pos { x = -10.0f, y = 20.5f };
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.q = MakeRot( 0.5f * Pi );
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.targetAngle = Pi * m_targetDegrees / 180.0f;
			jointDef.enableSpring = m_enableSpring;
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = m_motorTorque;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerAngle = -0.5f * Pi;
			jointDef.upperAngle = 0.05f * Pi;
			jointDef.enableLimit = m_enableLimit;

			m_jointId1 = CreateRevoluteJoint( m_worldId, ref jointDef );

			Joint_SetConstraintTuning( m_jointId1, 60.0f, 20.0f );
		}

		{
			Circle circle = default;
			circle.radius = 2.0f;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 5.0f, y = 30.0f };
			m_ball = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;

			CreateCircleShape( m_ball, ref shapeDef, ref circle );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 20.0f, y = 10.0f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId body = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeOffsetBox( 10.0f, 0.5f, new( -10.0f, 0.0f ), Rot_identity );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreatePolygonShape( body, ref shapeDef, ref box );

			Pos pivot = new Pos { x = 19.0f, y = 10.0f };
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = body;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.lowerAngle = -0.25f * Pi;
			jointDef.upperAngle = 0.0f * Pi;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = m_motorTorque;

			m_jointId2 = CreateRevoluteJoint( m_worldId, ref jointDef );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Limit", ref m_enableLimit ) )
		{
			RevoluteJoint_EnableLimit( m_jointId1, m_enableLimit );
			Joint_WakeBodies( m_jointId1 );
		}

		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			RevoluteJoint_EnableMotor( m_jointId1, m_enableMotor );
			Joint_WakeBodies( m_jointId1 );
		}

		if ( m_enableMotor )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Max Torque", ref m_motorTorque, 0.0f, 5000.0f, "%.0f" ) )
			{
				RevoluteJoint_SetMaxMotorTorque( m_jointId1, m_motorTorque );
				Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -20.0f, 20.0f, "%.0f" ) )
			{
				RevoluteJoint_SetTargetMotorSpeed( m_jointId1, m_motorSpeed );
				Joint_WakeBodies( m_jointId1 );
			}
			ImGui.PopItemWidth();
		}

		if ( ImGui.Checkbox( "Spring", ref m_enableSpring ) )
		{
			RevoluteJoint_EnableSpring( m_jointId1, m_enableSpring );
			Joint_WakeBodies( m_jointId1 );
		}

		if ( m_enableSpring )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 30.0f, "%.1f" ) )
			{
				RevoluteJoint_SetSpringHertz( m_jointId1, m_hertz );
				Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui.SliderFloat( "Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				RevoluteJoint_SetSpringDampingRatio( m_jointId1, m_dampingRatio );
				Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui.SliderFloat( "Degrees", ref m_targetDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				RevoluteJoint_SetTargetAngle( m_jointId1, Pi * m_targetDegrees / 180.0f );
				Joint_WakeBodies( m_jointId1 );
			}
			ImGui.PopItemWidth();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float angle1 = RevoluteJoint_GetCurrentAngle( m_jointId1 );
		DrawScreenTextLine( "Angle (Deg) 1 = %2.1f", angle1 );

		float torque1 = RevoluteJoint_GetCurrentMotorTorque( m_jointId1 );
		DrawScreenTextLine( "Motor Torque 1 = %4.1f", torque1 );

		float torque2 = RevoluteJoint_GetCurrentMotorTorque( m_jointId2 );
		DrawScreenTextLine( "Motor Torque 2 = %4.1f", torque2 );
	}

	public static Sample Create( SampleContext context )
	{
		return new RevoluteJoint( context );
	}

	public BodyId m_ball;
	public JointId m_jointId1;
	public JointId m_jointId2;
	public float m_motorSpeed;
	public float m_motorTorque;
	public float m_hertz;
	public float m_dampingRatio;
	public float m_targetDegrees;
	public bool m_enableSpring;
	public bool m_enableMotor;
	public bool m_enableLimit;
}

public class PrismaticJoint : Sample
{
	public PrismaticJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
		}

		m_enableSpring = false;
		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 2.0f;
		m_motorForce = 25.0f;
		m_hertz = 1.0f;
		m_dampingRatio = 0.5f;
		m_translation = 0.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 10.0f };
			bodyDef.type = BodyType.dynamicBody;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeBox( 0.5f, 2.0f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = new Pos { x = 0.0f, y = 9.0f };
			// Vec2 axis = Normalize({1.0f, 0.0f});
			Vec2 axis = Normalize( new( 1.0f, 1.0f ) );
			PrismaticJointDef jointDef = DefaultPrismaticJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameA.q = MakeRotFromUnitVector( axis );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.localFrameB.q = MakeRotFromUnitVector( axis );
			jointDef.@base.drawScale = 2.0f;
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorForce = m_motorForce;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerTranslation = -10.0f;
			jointDef.upperTranslation = 10.0f;
			jointDef.enableLimit = m_enableLimit;
			jointDef.enableSpring = m_enableSpring;
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;

			m_jointId = CreatePrismaticJoint( m_worldId, ref jointDef );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Limit", ref m_enableLimit ) )
		{
			PrismaticJoint_EnableLimit( m_jointId, m_enableLimit );
			Joint_WakeBodies( m_jointId );
		}

		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			PrismaticJoint_EnableMotor( m_jointId, m_enableMotor );
			Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Max Force", ref m_motorForce, 0.0f, 200.0f, "%.0f" ) )
			{
				PrismaticJoint_SetMaxMotorForce( m_jointId, m_motorForce );
				Joint_WakeBodies( m_jointId );
			}

			if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -40.0f, 40.0f, "%.0f" ) )
			{
				PrismaticJoint_SetTargetMotorSpeed( m_jointId, m_motorSpeed );
				Joint_WakeBodies( m_jointId );
			}
			ImGui.PopItemWidth();
		}

		if ( ImGui.Checkbox( "Spring", ref m_enableSpring ) )
		{
			PrismaticJoint_EnableSpring( m_jointId, m_enableSpring );
			Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpring )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				PrismaticJoint_SetSpringHertz( m_jointId, m_hertz );
				Joint_WakeBodies( m_jointId );
			}

			if ( ImGui.SliderFloat( "Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				PrismaticJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
				Joint_WakeBodies( m_jointId );
			}

			if ( ImGui.SliderFloat( "Translation", ref m_translation, -15.0f, 15.0f, "%.1f" ) )
			{
				PrismaticJoint_SetTargetTranslation( m_jointId, m_translation );
				Joint_WakeBodies( m_jointId );
			}
			ImGui.PopItemWidth();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float force = PrismaticJoint_GetCurrentMotorForce( m_jointId );
		DrawScreenTextLine( "Motor Force = %4.1f", force );

		float translation = PrismaticJoint_GetCurrentTranslation( m_jointId );
		DrawScreenTextLine( "Translation = %4.1f", translation );

		float speed = PrismaticJoint_GetCurrentSpeed( m_jointId );
		DrawScreenTextLine( "Speed = %4.8f", speed );
	}

	public static Sample Create( SampleContext context )
	{
		return new PrismaticJoint( context );
	}

	public JointId m_jointId;
	public float m_motorSpeed;
	public float m_motorForce;
	public float m_hertz;
	public float m_dampingRatio;
	public float m_translation;
	public bool m_enableSpring;
	public bool m_enableMotor;
	public bool m_enableLimit;
}

public class WheelJoint : Sample
{
	public WheelJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 10.0f );
			m_context.camera.zoom = 25.0f * 0.15f;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDefInner );
		}

		m_enableSpring = true;
		m_enableLimit = true;
		m_enableMotor = true;
		m_motorSpeed = 2.0f;
		m_motorTorque = 5.0f;
		m_hertz = 1.0f;
		m_dampingRatio = 0.7f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = 0.0f, y = 10.25f };
		bodyDef.type = BodyType.dynamicBody;
		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Capsule capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.5f };
		CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

		Pos pivot = new Pos { x = 0.0f, y = 10.0f };
		Vec2 axis = Normalize( new( 1.0f, 1.0f ) );
		WheelJointDef jointDef = DefaultWheelJointDef();
		jointDef.@base.bodyIdA = groundId;
		jointDef.@base.bodyIdB = bodyId;
		jointDef.@base.localFrameA.q = MakeRotFromUnitVector( axis );
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
		jointDef.motorSpeed = m_motorSpeed;
		jointDef.maxMotorTorque = m_motorTorque;
		jointDef.enableMotor = m_enableMotor;
		jointDef.lowerTranslation = -3.0f;
		jointDef.upperTranslation = 3.0f;
		jointDef.enableLimit = m_enableLimit;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;

		m_jointId = CreateWheelJoint( m_worldId, ref jointDef );
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Limit", ref m_enableLimit ) )
		{
			WheelJoint_EnableLimit( m_jointId, m_enableLimit );
		}

		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			WheelJoint_EnableMotor( m_jointId, m_enableMotor );
		}

		if ( m_enableMotor )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Torque", ref m_motorTorque, 0.0f, 20.0f, "%.0f" ) )
			{
				WheelJoint_SetMaxMotorTorque( m_jointId, m_motorTorque );
			}

			if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -20.0f, 20.0f, "%.0f" ) )
			{
				WheelJoint_SetTargetMotorSpeed( m_jointId, m_motorSpeed );
			}
			ImGui.PopItemWidth();
		}

		if ( ImGui.Checkbox( "Spring", ref m_enableSpring ) )
		{
			WheelJoint_EnableSpring( m_jointId, m_enableSpring );
		}

		if ( m_enableSpring )
		{
			ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
			if ( ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				WheelJoint_SetSpringHertz( m_jointId, m_hertz );
			}

			if ( ImGui.SliderFloat( "Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				WheelJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
			}
			ImGui.PopItemWidth();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float torque = WheelJoint_GetCurrentMotorTorque( m_jointId );
		DrawScreenTextLine( "Motor Torque = %4.1f", torque );
	}

	public static Sample Create( SampleContext context )
	{
		return new WheelJoint( context );
	}

	public JointId m_jointId;
	public float m_hertz;
	public float m_dampingRatio;
	public float m_motorSpeed;
	public float m_motorTorque;
	public bool m_enableSpring;
	public bool m_enableMotor;
	public bool m_enableLimit;
}

public class Bridge : Sample
{
	public Bridge( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.zoom = 25.0f * 2.5f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
		}

		{
			m_constraintHertz = 60.0f;
			m_constraintDampingRatio = 0.0f;
			m_springHertz = 2.0f;
			m_springDampingRatio = 0.7f;
			m_frictionTorque = 200.0f;

			Polygon box = MakeBox( 0.5f, 0.125f );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 20.0f;

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			jointDef.enableSpring = true;
			jointDef.hertz = m_springHertz;
			jointDef.dampingRatio = m_springDampingRatio;

			int jointIndex = 0;

			float xbase = -80.0f;

			BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = xbase + 0.5f + 1.0f * i, y = 20.0f };
				bodyDef.linearDamping = 0.1f;
				bodyDef.angularDamping = 0.1f;

				m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );

				CreatePolygonShape( m_bodyIds[i], ref shapeDef, ref box );

				Pos pivotInner = new Pos { x = xbase + 1.0f * i, y = 20.0f };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = m_bodyIds[i];
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotInner );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotInner );
				m_jointIds[jointIndex++] = CreateRevoluteJoint( m_worldId, ref jointDef );

				prevBodyId = m_bodyIds[i];
			}

			Pos pivot = new Pos { x = xbase + 1.0f * m_count, y = 20.0f };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = groundId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			m_jointIds[jointIndex++] = CreateRevoluteJoint( m_worldId, ref jointDef );

			Debug.Assert( jointIndex == m_count + 1 );
		}

		for ( int i = 0; i < 2; ++i )
		{
			Vec2[] vertices = { new( -0.5f, 0.0f ), new( 0.5f, 0.0f ), new( 0.0f, 1.5f ) };

			Hull hull = ComputeHull( vertices, 3 );
			Polygon triangle = MakePolygon( ref hull, 0.0f );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 20.0f;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -8.0f + 8.0f * i, y = 22.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref triangle );
		}

		for ( int i = 0; i < 3; ++i )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 20.0f;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -6.0f + 6.0f * i, y = 25.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool updateFriction = ImGui.SliderFloat( "Joint Friction", ref m_frictionTorque, 0.0f, 10000.0f, "%2.0f" );
		if ( updateFriction )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				RevoluteJoint_SetMaxMotorTorque( m_jointIds[i], m_frictionTorque );
			}
		}

		if ( ImGui.SliderFloat( "Spring hertz", ref m_springHertz, 0.0f, 30.0f, "%.0f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				RevoluteJoint_SetSpringHertz( m_jointIds[i], m_springHertz );
			}
		}

		if ( ImGui.SliderFloat( "Spring damping", ref m_springDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				RevoluteJoint_SetSpringDampingRatio( m_jointIds[i], m_springDampingRatio );
			}
		}

		if ( ImGui.SliderFloat( "Constraint hertz", ref m_constraintHertz, 15.0f, 240.0f, "%.0f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				Joint_SetConstraintTuning( m_jointIds[i], m_constraintHertz, m_constraintDampingRatio );
			}
		}

		if ( ImGui.SliderFloat( "Constraint damping", ref m_constraintDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				Joint_SetConstraintTuning( m_jointIds[i], m_constraintHertz, m_constraintDampingRatio );
			}
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new Bridge( context );
	}

	public const int m_count = 160;
	public BodyId[] m_bodyIds = new BodyId[m_count];
	public JointId[] m_jointIds = new JointId[m_count + 1];
	public float m_frictionTorque;
	public float m_constraintHertz;
	public float m_constraintDampingRatio;
	public float m_springHertz;
	public float m_springDampingRatio;
}

public class BallAndChain : Sample
{
	public BallAndChain( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, -8.0f );
			m_context.camera.zoom = 27.5f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
		}

		m_frictionTorque = 100.0f;

		{
			float hx = 0.5f;
			Capsule capsule = new Capsule { center1 = new( -hx, 0.0f ), center2 = new( hx, 0.0f ), radius = 0.125f };

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.filter.categoryBits = 0x1;
			shapeDef.filter.maskBits = 0x2;
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();

			int jointIndex = 0;

			BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				BodyDef bodyDefInner = DefaultBodyDef();
				bodyDefInner.type = BodyType.dynamicBody;
				bodyDefInner.position = new Pos { x = ( 1.0f + 2.0f * i ) * hx, y = m_count * hx };
				BodyId bodyIdInner = CreateBody( m_worldId, ref bodyDefInner );
				CreateCapsuleShape( bodyIdInner, ref shapeDef, ref capsule );

				Pos pivotInner = new Pos { x = ( 2.0f * i ) * hx, y = m_count * hx };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = bodyIdInner;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotInner );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotInner );
				jointDef.enableMotor = true;
				jointDef.maxMotorTorque = m_frictionTorque;
				jointDef.enableSpring = i > 0;
				jointDef.hertz = 4.0f;
				m_jointIds[jointIndex++] = CreateRevoluteJoint( m_worldId, ref jointDef );

				prevBodyId = bodyIdInner;
			}

			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 4.0f };

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = ( 1.0f + 2.0f * m_count ) * hx + circle.radius - hx, y = m_count * hx };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			shapeDef.filter.categoryBits = 0x2;
			shapeDef.filter.maskBits = 0x1;
			CreateCircleShape( bodyId, ref shapeDef, ref circle );

			Pos pivot = new Pos { x = ( 2.0f * m_count ) * hx, y = m_count * hx };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			jointDef.enableSpring = true;
			jointDef.hertz = 4.0f;
			m_jointIds[jointIndex++] = CreateRevoluteJoint( m_worldId, ref jointDef );
			Debug.Assert( jointIndex == m_count + 1 );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		bool updateFriction = ImGui.SliderFloat( "Joint Friction", ref m_frictionTorque, 0.0f, 1000.0f, "%2.0f" );
		ImGui.PopItemWidth();
		if ( updateFriction )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				RevoluteJoint_SetMaxMotorTorque( m_jointIds[i], m_frictionTorque );
			}
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new BallAndChain( context );
	}

	public const int m_count = 30;
	public JointId[] m_jointIds = new JointId[m_count + 1];
	public float m_frictionTorque;
}

public class Cantilever : Sample
{
	public const int e_count = 8;

	public Cantilever( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 0.35f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );
		}

		{
			m_linearHertz = 15.0f;
			m_linearDampingRatio = 0.5f;
			m_angularHertz = 5.0f;
			m_angularDampingRatio = 0.5f;
			m_gravityScale = 1.0f;
			m_collideConnected = false;

			float hx = 0.5f;
			Capsule capsule = new Capsule { center1 = new( -hx, 0.0f ), center2 = new( hx, 0.0f ), radius = 0.125f };
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 20.0f;

			WeldJointDef jointDef = DefaultWeldJointDef();

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.isAwake = false;

			BodyId prevBodyId = groundId;
			for ( int i = 0; i < e_count; ++i )
			{
				bodyDef.position = new Pos { x = ( 1.0f + 2.0f * i ) * hx, y = 0.0f };
				m_bodyIds[i] = CreateBody( m_worldId, ref bodyDef );
				CreateCapsuleShape( m_bodyIds[i], ref shapeDef, ref capsule );

				Pos pivot = new Pos { x = ( 2.0f * i ) * hx, y = 0.0f };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = m_bodyIds[i];
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
				jointDef.linearHertz = m_linearHertz;
				jointDef.linearDampingRatio = m_linearDampingRatio;
				jointDef.angularHertz = m_angularHertz;
				jointDef.angularDampingRatio = m_angularDampingRatio;
				jointDef.@base.collideConnected = m_collideConnected;
				m_jointIds[i] = CreateWeldJoint( m_worldId, ref jointDef );

				// Experimental tuning
				Joint_SetConstraintTuning( m_jointIds[i], 120.0f, 10.0f );

				prevBodyId = m_bodyIds[i];
			}

			m_tipId = prevBodyId;
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Linear Hertz", ref m_linearHertz, 0.0f, 20.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				WeldJoint_SetLinearHertz( m_jointIds[i], m_linearHertz );
			}
		}

		if ( ImGui.SliderFloat( "Linear Damping Ratio", ref m_linearDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				WeldJoint_SetLinearDampingRatio( m_jointIds[i], m_linearDampingRatio );
			}
		}

		if ( ImGui.SliderFloat( "Angular Hertz", ref m_angularHertz, 0.0f, 20.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				WeldJoint_SetAngularHertz( m_jointIds[i], m_angularHertz );
			}
		}

		if ( ImGui.SliderFloat( "Angular Damping Ratio", ref m_angularDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				WeldJoint_SetAngularDampingRatio( m_jointIds[i], m_angularDampingRatio );
			}
		}

		if ( ImGui.SliderFloat( "Gravity Scale", ref m_gravityScale, -1.0f, 1.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Body_SetGravityScale( m_bodyIds[i], m_gravityScale );
			}
		}

		ImGui.PopItemWidth();

		if ( ImGui.Checkbox( "Collide Connected", ref m_collideConnected ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Joint_SetCollideConnected( m_jointIds[i], m_collideConnected );
			}
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		Pos tipPosition = Body_GetPosition( m_tipId );
		DrawScreenTextLine( "tip-y = %.2f", tipPosition.y );
	}

	public static Sample Create( SampleContext context )
	{
		return new Cantilever( context );
	}

	public float m_linearHertz;
	public float m_linearDampingRatio;
	public float m_angularHertz;
	public float m_angularDampingRatio;
	public float m_gravityScale;
	public BodyId m_tipId;
	public BodyId[] m_bodyIds = new BodyId[e_count];
	public JointId[] m_jointIds = new JointId[e_count];
	public bool m_collideConnected;
}

public class MotionLocksSample : Sample
{
	public const int e_count = 6;

	public MotionLocksSample( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 25.0f * 0.7f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		m_motionLocks = new MotionLocks { linearX = false, linearY = false, angularZ = true };

		for ( int i = 0; i < e_count; ++i )
		{
			m_bodyIds[i] = nullBodyId;
		}

		Pos position = new Pos { x = -12.5f, y = 10.0f };
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.motionLocks = m_motionLocks;

		Polygon box = MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			float length = 2.0f;
			Pos pivot1 = new Pos { x = position.x, y = position.y + 1.0f + length };
			Pos pivot2 = new Pos { x = position.x, y = position.y + 1.0f };
			DistanceJointDef jointDef = DefaultDistanceJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot1 );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot2 );
			jointDef.length = length;
			CreateDistanceJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// motor joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			MotorJointDef jointDef = DefaultMotorJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = ToVec2( position );
			jointDef.maxVelocityForce = 200.0f;
			jointDef.maxVelocityTorque = 200.0f;
			CreateMotorJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// prismatic joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			PrismaticJointDef jointDef = DefaultPrismaticJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			CreatePrismaticJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// revolute joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// weld joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WeldJointDef jointDef = DefaultWeldJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.angularHertz = 1.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.linearHertz = 1.0f;
			jointDef.linearDampingRatio = 0.5f;
			CreateWeldJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// wheel joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WheelJointDef jointDef = DefaultWheelJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			CreateWheelJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Lock Linear X", ref m_motionLocks.linearX ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui.Checkbox( "Lock Linear Y", ref m_motionLocks.linearY ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui.Checkbox( "Lock Angular Z", ref m_motionLocks.angularZ ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_L ) == GLFW_PRESS )
		{
			Body_ApplyLinearImpulseToCenter( m_bodyIds[0], new( 100.0f, 0.0f ), true );
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new MotionLocksSample( context );
	}

	public BodyId[] m_bodyIds = new BodyId[e_count];
	public MotionLocks m_motionLocks;
}

public class BreakableJoint : Sample
{
	public const int e_count = 6;

	public BreakableJoint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 25.0f * 0.7f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		for ( int i = 0; i < e_count; ++i )
		{
			m_jointIds[i] = nullJointId;
		}

		Pos position = new Pos { x = -12.5f, y = 10.0f };
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.enableSleep = false;

		Polygon box = MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			float length = 2.0f;
			Pos pivot1 = new Pos { x = position.x, y = position.y + 1.0f + length };
			Pos pivot2 = new Pos { x = position.x, y = position.y + 1.0f };
			DistanceJointDef jointDef = DefaultDistanceJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot1 );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateDistanceJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// motor joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			MotorJointDef jointDef = DefaultMotorJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = ToVec2( position );
			jointDef.maxVelocityForce = 1000.0f;
			jointDef.maxVelocityTorque = 20.0f;
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateMotorJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// prismatic joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			PrismaticJointDef jointDef = DefaultPrismaticJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreatePrismaticJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// revolute joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// weld joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WeldJointDef jointDef = DefaultWeldJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateWeldJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		// wheel joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WheelJointDef jointDef = DefaultWheelJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateWheelJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;

		m_breakForce = 1000.0f;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		ImGui.SliderFloat( "break force", ref m_breakForce, 0.0f, 10000.0f, "%.1f" );

		Vec2 gravity = World_GetGravity( m_worldId );
		if ( ImGui.SliderFloat( "gravity", ref gravity.y, -50.0f, 50.0f, "%.1f" ) )
		{
			World_SetGravity( m_worldId, gravity );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_jointIds[i].IsNull )
			{
				continue;
			}

			Vec2 force = Joint_GetConstraintForce( m_jointIds[i] );
			if ( LengthSquared( force ) > m_breakForce * m_breakForce )
			{
				DestroyJoint( m_jointIds[i] );
				m_jointIds[i] = nullJointId;
			}
			else
			{
				Transform localFrame = Joint_GetLocalFrameA( m_jointIds[i] );
				DrawString( m_draw, m_camera, ToPos( localFrame.p ), HexColor.colorWhite, "(%.1f, %.1f)", force.x, force.y );
			}
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BreakableJoint( context );
	}

	public JointId[] m_jointIds = new JointId[e_count];
	public float m_breakForce;
}

public class JointSeparation : Sample
{
	public const int e_count = 5;

	public JointSeparation( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 8.0f );
			m_context.camera.zoom = 25.0f;
		}

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		Pos position = new Pos { x = -20.0f, y = 10.0f };
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.enableSleep = false;

		Polygon box = MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			float length = 2.0f;
			Pos pivot1 = new Pos { x = position.x, y = position.y + 1.0f + length };
			Pos pivot2 = new Pos { x = position.x, y = position.y + 1.0f };
			DistanceJointDef jointDef = DefaultDistanceJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot1 );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateDistanceJoint( m_worldId, ref jointDef );
		}

		position.x += 10.0f;
		++index;

		// prismatic joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			PrismaticJointDef jointDef = DefaultPrismaticJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreatePrismaticJoint( m_worldId, ref jointDef );
		}

		position.x += 10.0f;
		++index;

		// revolute joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		position.x += 10.0f;
		++index;

		// weld joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WeldJointDef jointDef = DefaultWeldJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateWeldJoint( m_worldId, ref jointDef );
		}

		position.x += 10.0f;
		++index;

		// wheel joint
		{
			Debug.Assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( m_bodyIds[index], ref shapeDef, ref box );

			Pos pivot = new Pos { x = position.x - 1.0f, y = position.y };
			WheelJointDef jointDef = DefaultWheelJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_bodyIds[index];
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			jointDef.@base.collideConnected = true;
			m_jointIds[index] = CreateWheelJoint( m_worldId, ref jointDef );
		}

		m_impulse = 500.0f;
		m_jointHertz = 60.0f;
		m_jointDampingRatio = 2.0f;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		Vec2 gravity = World_GetGravity( m_worldId );
		if ( ImGui.SliderFloat( "gravity", ref gravity.y, -500.0f, 500.0f, "%.0f" ) )
		{
			World_SetGravity( m_worldId, gravity );
		}

		ImGui.SliderFloat( "magnitude", ref m_impulse, 0.0f, 1000.0f, "%.0f" );

		if ( ImGui.SliderFloat( "hertz", ref m_jointHertz, 15.0f, 120.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Joint_SetConstraintTuning( m_jointIds[i], m_jointHertz, m_jointDampingRatio );
			}
		}

		if ( ImGui.SliderFloat( "damping", ref m_jointDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Joint_SetConstraintTuning( m_jointIds[i], m_jointHertz, m_jointDampingRatio );
			}
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "impulse" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				Pos p = Body_GetWorldPoint( m_bodyIds[i], new( 1.0f, 1.0f ) );
				Body_ApplyLinearImpulse( m_bodyIds[i], new( m_impulse, -m_impulse ), p, true );
			}
		}

		return true;
	}

	public override void Step(  )
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_jointIds[i].IsNull )
			{
				continue;
			}

			float linear = Joint_GetLinearSeparation( m_jointIds[i] );
			float angular = Joint_GetAngularSeparation( m_jointIds[i] );
			Transform localFrame = Joint_GetLocalFrameA( m_jointIds[i] );
			DrawString( m_draw, m_camera, ToPos( localFrame.p ), HexColor.colorWhite, "%.2f m, %.1f deg", linear,
						180.0f * angular / Pi );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new JointSeparation( context );
	}

	public BodyId[] m_bodyIds = new BodyId[e_count];
	public JointId[] m_jointIds = new JointId[e_count];
	public float m_impulse;
	public float m_jointHertz;
	public float m_jointDampingRatio;
}

public class UserConstraint : Sample
{
	public UserConstraint( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 3.0f, -1.0f );
			m_context.camera.zoom = 25.0f * 0.15f;
		}

		Polygon box = MakeBox( 1.0f, 0.5f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 20.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 1.0f;
		bodyDef.angularDamping = 0.5f;
		bodyDef.linearDamping = 0.2f;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );
		CreatePolygonShape( m_bodyId, ref shapeDef, ref box );

		m_impulses[0] = 0.0f;
		m_impulses[1] = 0.0f;
	}

	public override void Step(  )
	{
		base.Step();

		DrawTransform( m_draw, WorldTransform_identity, 1.0f );

		if ( m_context.pause )
		{
			return;
		}

		float timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
		if ( timeStep == 0.0f )
		{
			return;
		}

		float invTimeStep = m_context.hertz;

		float hertz = 3.0f;
		float zeta = 0.7f;
		float maxForce = 1000.0f;
		float omega = 2.0f * Pi * hertz;
		float sigma = 2.0f * zeta + timeStep * omega;
		float s = timeStep * omega * sigma;
		float impulseCoefficient = 1.0f / ( 1.0f + s );
		float massCoefficient = s * impulseCoefficient;
		float biasCoefficient = omega / sigma;

		Vec2[] localAnchors = { new( 1.0f, -0.5f ), new( 1.0f, 0.5f ) };
		float mass = Body_GetMass( m_bodyId );
		float invMass = mass < 0.0001f ? 0.0f : 1.0f / mass;
		float inertiaTensor = Body_GetRotationalInertia( m_bodyId );
		float invI = inertiaTensor < 0.0001f ? 0.0f : 1.0f / inertiaTensor;

		Vec2 vB = Body_GetLinearVelocity( m_bodyId );
		float omegaB = Body_GetAngularVelocity( m_bodyId );
		Pos pB = Body_GetWorldCenter( m_bodyId );

		for ( int i = 0; i < 2; ++i )
		{
			Pos anchorA = new Pos { x = 3.0f, y = 0.0f };
			Pos anchorB = Body_GetWorldPoint( m_bodyId, localAnchors[i] );

			Vec2 deltaAnchor = SubPos( anchorB, anchorA );

			float slackLength = 1.0f;
			float length = Length( deltaAnchor );
			float C = length - slackLength;
			if ( C < 0.0f || length < 0.001f )
			{
				DrawLine( m_draw, anchorA, anchorB, HexColor.colorLightCyan );
				m_impulses[i] = 0.0f;
				continue;
			}

			DrawLine( m_draw, anchorA, anchorB, HexColor.colorViolet );
			Vec2 axis = Normalize( deltaAnchor );

			Vec2 rB = SubPos( anchorB, pB );
			float Jb = Cross( rB, axis );
			float K = invMass + Jb * invI * Jb;
			float invK = K < 0.0001f ? 0.0f : 1.0f / K;

			float Cdot = Dot( vB, axis ) + Jb * omegaB;
			float impulse = -massCoefficient * invK * ( Cdot + biasCoefficient * C );
			float appliedImpulse = ClampFloat( impulse, -maxForce * timeStep, 0.0f );

			vB = MulAdd( vB, invMass * appliedImpulse, axis );
			omegaB += appliedImpulse * invI * Jb;

			m_impulses[i] = appliedImpulse;
		}

		Body_SetLinearVelocity( m_bodyId, vB );
		Body_SetAngularVelocity( m_bodyId, omegaB );

		DrawScreenTextLine( "forces = %g, %g", m_impulses[0] * invTimeStep, m_impulses[1] * invTimeStep );
	}

	public static Sample Create( SampleContext context )
	{
		return new UserConstraint( context );
	}

	public BodyId m_bodyId;
	public float[] m_impulses = new float[2];
}

public class Driving : Sample
{
	public Driving( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center.y = 5.0f;
			m_context.camera.zoom = 25.0f * 0.4f;
			m_context.debugDraw.drawJoints = false;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDef );

			Vec2[] points = new Vec2[25];
			int count = 24;

			// fill in reverse to match line list convention
			points[count--] = new( -20.0f, -20.0f );
			points[count--] = new( -20.0f, 0.0f );
			points[count--] = new( 20.0f, 0.0f );

			float[] hs = { 0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f };
			float x = 20.0f, dx = 5.0f;

			for ( int j = 0; j < 2; ++j )
			{
				for ( int i = 0; i < 10; ++i )
				{
					float y2 = hs[i];
					points[count--] = new( x + dx, y2 );
					x += dx;
				}
			}

			// flat before bridge
			points[count--] = new( x + 40.0f, 0.0f );
			points[count--] = new( x + 40.0f, -20.0f );

			Debug.Assert( count == -1 );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = 25;
			chainDef.isLoop = true;
			CreateChain( groundId, ref chainDef );

			// flat after bridge
			x += 80.0f;
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( x, 0.0f ), point2 = new( x + 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			// jump ramp
			x += 40.0f;
			segment = new Segment { point1 = new( x, 0.0f ), point2 = new( x + 10.0f, 5.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			// final corner
			x += 20.0f;
			segment = new Segment { point1 = new( x, 0.0f ), point2 = new( x + 40.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			x += 40.0f;
			segment = new Segment { point1 = new( x, 0.0f ), point2 = new( x, 20.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Teeter
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 140.0f, y = 1.0f };
			bodyDef.angularVelocity = 1.0f;
			bodyDef.type = BodyType.dynamicBody;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeBox( 10.0f, 0.25f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			Pos pivot = bodyDef.position;
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.lowerAngle = -8.0f * Pi / 180.0f;
			jointDef.upperAngle = 8.0f * Pi / 180.0f;
			jointDef.enableLimit = true;
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		// Bridge
		{
			int N = 20;
			ShapeDef shapeDef = DefaultShapeDef();
			Capsule capsule = new Capsule { center1 = new( -1.0f, 0.0f ), center2 = new( 1.0f, 0.0f ), radius = 0.125f };

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();

			BodyId prevBodyId = groundId;
			for ( int i = 0; i < N; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Pos { x = 161.0f + 2.0f * i, y = -0.125f };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreateCapsuleShape( bodyId, ref shapeDef, ref capsule );

				Pos pivotInner = new Pos { x = 160.0f + 2.0f * i, y = -0.125f };
				jointDef.@base.bodyIdA = prevBodyId;
				jointDef.@base.bodyIdB = bodyId;
				jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivotInner );
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivotInner );
				CreateRevoluteJoint( m_worldId, ref jointDef );

				prevBodyId = bodyId;
			}

			Pos pivot = new Pos { x = 160.0f + 2.0f * N, y = -0.125f };
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = groundId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 50.0f;
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		// Boxes
		{
			Polygon box = MakeBox( 0.5f, 0.5f );

			BodyId bodyId = default;
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.25f;
			shapeDef.material.restitution = 0.25f;
			shapeDef.density = 0.25f;

			bodyDef.position = new Pos { x = 230.0f, y = 0.5f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			bodyDef.position = new Pos { x = 230.0f, y = 1.5f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			bodyDef.position = new Pos { x = 230.0f, y = 2.5f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			bodyDef.position = new Pos { x = 230.0f, y = 3.5f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			bodyDef.position = new Pos { x = 230.0f, y = 4.5f };
			bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		// Car

		m_throttle = 0.0f;
		m_speed = 35.0f;
		m_torque = 5.0f;
		m_hertz = 5.0f;
		m_dampingRatio = 0.7f;

		m_car.Spawn( m_worldId, new( 0.0f, 0.0f ), 1.0f, m_hertz, m_dampingRatio, m_torque, null );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		if ( ImGui.SliderFloat( "Spring Hertz", ref m_hertz, 0.0f, 20.0f, "%.0f" ) )
		{
			m_car.SetHertz( m_hertz );
		}

		if ( ImGui.SliderFloat( "Damping Ratio", ref m_dampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			m_car.SetDampingRadio( m_dampingRatio );
		}

		if ( ImGui.SliderFloat( "Speed", ref m_speed, 0.0f, 50.0f, "%.0f" ) )
		{
			m_car.SetSpeed( m_throttle * m_speed );
		}

		if ( ImGui.SliderFloat( "Torque", ref m_torque, 0.0f, 10.0f, "%.1f" ) )
		{
			m_car.SetTorque( m_torque );
		}
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( glfwGetKey( m_context.window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_throttle = 1.0f;
			m_car.SetSpeed( m_speed );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_throttle = 0.0f;
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_throttle = -1.0f;
			m_car.SetSpeed( -m_speed );
		}

		DrawScreenTextLine( "Keys: left = a, brake = s, right = d" );

		Vec2 linearVelocity = Body_GetLinearVelocity( m_car.m_chassisId );
		float kph = linearVelocity.x * 3.6f;
		DrawScreenTextLine( "speed in kph: %.2g", kph );

		Pos carPosition = Body_GetPosition( m_car.m_chassisId );
		m_context.camera.center.x = carPosition.x;

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new Driving( context );
	}

	public Car m_car = new();

	public float m_throttle;
	public float m_hertz;
	public float m_dampingRatio;
	public float m_torque;
	public float m_speed;
}

public class Ragdoll : Sample
{
	public Ragdoll( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 12.0f );
			m_context.camera.zoom = 16.0f;

			// m_context.camera.center = new( 0.0f, 26.0f );
			// m_context.camera.zoom = 1.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_jointFrictionTorque = 0.03f;
		m_jointHertz = 5.0f;
		m_jointDampingRatio = 0.5f;

		m_human = new Human {  };

		Spawn();

		World_SetContactTuning( m_worldId, 240.0f, 0.0f, 2.0f );
	}

	public void Spawn(  )
	{
		CreateHuman( ref m_human, m_worldId, new( 0.0f, 25.0f ), 1.0f, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 null, false );
		// Human_ApplyRandomAngularImpulse( ref m_human, 10.0f );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Friction", ref m_jointFrictionTorque, 0.0f, 1.0f, "%3.2f" ) )
		{
			Human_SetJointFrictionTorque( ref m_human, m_jointFrictionTorque );
		}

		if ( ImGui.SliderFloat( "Hertz", ref m_jointHertz, 0.0f, 10.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( ref m_human, m_jointHertz );
		}

		if ( ImGui.SliderFloat( "Damping", ref m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( ref m_human, m_jointDampingRatio );
		}

		ImGui.PopItemWidth();

		if ( ImGui.Button( "Respawn" ) )
		{
			DestroyHuman( ref m_human );
			Spawn();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new Ragdoll( context );
	}

	public Human m_human;
	public float m_jointFrictionTorque;
	public float m_jointHertz;
	public float m_jointDampingRatio;
}

public class SoftBody : Sample
{
	public SoftBody( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 25.0f * 0.25f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_donut.Create( m_worldId, new( 0.0f, 10.0f ), 2.0f, 0, false, null );
	}

	public static Sample Create( SampleContext context )
	{
		return new SoftBody( context );
	}

	public Donut m_donut = new();
}

public class DoohickeyFarm : Sample
{
	public DoohickeyFarm( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 25.0f * 0.35f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			Polygon box = MakeOffsetBox( 1.0f, 1.0f, new( 0.0f, 1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		float y = 4.0f;
		for ( int i = 0; i < 4; ++i )
		{
			Doohickey doohickey = new();
			doohickey.Spawn( m_worldId, new( 0.0f, y ), 0.5f );
			y += 2.0f;
		}
	}

	public override void Step(  )
	{
		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new DoohickeyFarm( context );
	}
}

public class ScissorLift : Sample
{
	public ScissorLift( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 9.0f );
			m_context.camera.zoom = 25.0f * 0.4f;
		}

		// Need 8 sub-steps for smoother operation
		m_context.subStepCount = 8;

		BodyId groundId = default;
		{
			BodyDef bodyDefInner = DefaultBodyDef();
			groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.sleepThreshold = 0.01f;

		ShapeDef shapeDef = DefaultShapeDef();
		Capsule capsule = new Capsule { center1 = new( -2.5f, 0.0f ), center2 = new( 2.5f, 0.0f ), radius = 0.15f };

		BodyId baseId1 = groundId;
		BodyId baseId2 = groundId;
		Vec2 baseAnchor1 = new Vec2( -2.5f, 0.2f );
		Vec2 baseAnchor2 = new Vec2( 2.5f, 0.2f );
		float y = 0.5f;

		BodyId linkId1 = default;
		int N = 3;

		float constraintDampingRatio = 20.0f;
		float constraintHertz = 240.0f;

		for ( int i = 0; i < N; ++i )
		{
			bodyDef.position = new Pos { x = 0.0f, y = y };
			bodyDef.rotation = MakeRot( 0.15f );
			BodyId bodyId1 = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( bodyId1, ref shapeDef, ref capsule );

			bodyDef.position = new Pos { x = 0.0f, y = y };
			bodyDef.rotation = MakeRot( -0.15f );

			BodyId bodyId2 = CreateBody( m_worldId, ref bodyDef );
			CreateCapsuleShape( bodyId2, ref shapeDef, ref capsule );

			if ( i == 1 )
			{
				linkId1 = bodyId2;
			}

			RevoluteJointDef revoluteDefInner = DefaultRevoluteJointDef();

			// left pin
			revoluteDefInner.@base.bodyIdA = baseId1;
			revoluteDefInner.@base.bodyIdB = bodyId1;
			revoluteDefInner.@base.localFrameA.p = baseAnchor1;
			revoluteDefInner.@base.localFrameB.p = new( -2.5f, 0.0f );
			revoluteDefInner.@base.collideConnected = ( i == 0 ) ? true : false;
			revoluteDefInner.@base.constraintDampingRatio = constraintDampingRatio;
			revoluteDefInner.@base.constraintHertz = constraintHertz;

			CreateRevoluteJoint( m_worldId, ref revoluteDefInner );

			// right pin
			if ( i == 0 )
			{
				WheelJointDef wheelDefInner = DefaultWheelJointDef();
				wheelDefInner.@base.bodyIdA = baseId2;
				wheelDefInner.@base.bodyIdB = bodyId2;
				wheelDefInner.@base.localFrameA.p = baseAnchor2;
				wheelDefInner.@base.localFrameB.p = new( 2.5f, 0.0f );
				wheelDefInner.enableSpring = false;
				wheelDefInner.@base.collideConnected = true;
				wheelDefInner.@base.constraintDampingRatio = constraintDampingRatio;
				wheelDefInner.@base.constraintHertz = constraintHertz;

				CreateWheelJoint( m_worldId, ref wheelDefInner );
			}
			else
			{
				revoluteDefInner.@base.bodyIdA = baseId2;
				revoluteDefInner.@base.bodyIdB = bodyId2;
				revoluteDefInner.@base.localFrameA.p = baseAnchor2;
				revoluteDefInner.@base.localFrameB.p = new( 2.5f, 0.0f );
				revoluteDefInner.@base.collideConnected = false;
				revoluteDefInner.@base.constraintDampingRatio = constraintDampingRatio;
				revoluteDefInner.@base.constraintHertz = constraintHertz;

				CreateRevoluteJoint( m_worldId, ref revoluteDefInner );
			}

			// middle pin
			revoluteDefInner.@base.bodyIdA = bodyId1;
			revoluteDefInner.@base.bodyIdB = bodyId2;
			revoluteDefInner.@base.localFrameA.p = new( 0.0f, 0.0f );
			revoluteDefInner.@base.localFrameB.p = new( 0.0f, 0.0f );
			revoluteDefInner.@base.collideConnected = false;
			revoluteDefInner.@base.constraintDampingRatio = constraintDampingRatio;
			revoluteDefInner.@base.constraintHertz = constraintHertz;

			CreateRevoluteJoint( m_worldId, ref revoluteDefInner );

			baseId1 = bodyId2;
			baseId2 = bodyId1;
			baseAnchor1 = new Vec2( -2.5f, 0.0f );
			baseAnchor2 = new Vec2( 2.5f, 0.0f );
			y += 1.0f;
		}

		bodyDef.position = new Pos { x = 0.0f, y = y };
		bodyDef.rotation = Rot_identity;
		BodyId platformId = CreateBody( m_worldId, ref bodyDef );

		Polygon box = MakeBox( 3.0f, 0.2f );
		CreatePolygonShape( platformId, ref shapeDef, ref box );

		// left pin
		RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();
		revoluteDef.@base.bodyIdA = platformId;
		revoluteDef.@base.bodyIdB = baseId1;
		revoluteDef.@base.localFrameA.p = new( -2.5f, -0.4f );
		revoluteDef.@base.localFrameB.p = baseAnchor1;
		revoluteDef.@base.collideConnected = true;
		revoluteDef.@base.constraintDampingRatio = constraintDampingRatio;
		revoluteDef.@base.constraintHertz = constraintHertz;
		CreateRevoluteJoint( m_worldId, ref revoluteDef );

		// right pin
		WheelJointDef wheelDef = DefaultWheelJointDef();
		wheelDef.@base.bodyIdA = platformId;
		wheelDef.@base.bodyIdB = baseId2;
		wheelDef.@base.localFrameA.p = new( 2.5f, -0.4f );
		wheelDef.@base.localFrameB.p = baseAnchor2;
		wheelDef.enableSpring = false;
		wheelDef.@base.collideConnected = true;
		wheelDef.@base.constraintDampingRatio = constraintDampingRatio;
		wheelDef.@base.constraintHertz = constraintHertz;
		CreateWheelJoint( m_worldId, ref wheelDef );

		m_enableMotor = false;
		m_motorSpeed = 0.25f;
		m_motorForce = 2000.0f;

		DistanceJointDef distanceDef = DefaultDistanceJointDef();
		distanceDef.@base.bodyIdA = groundId;
		distanceDef.@base.bodyIdB = linkId1;
		distanceDef.@base.localFrameA.p = new( -2.5f, 0.2f );
		distanceDef.@base.localFrameB.p = new( 0.5f, 0.0f );
		distanceDef.enableSpring = true;
		distanceDef.minLength = 0.2f;
		distanceDef.maxLength = 5.5f;
		distanceDef.enableLimit = true;
		distanceDef.enableMotor = m_enableMotor;
		distanceDef.motorSpeed = m_motorSpeed;
		distanceDef.maxMotorForce = m_motorForce;
		m_liftJointId = CreateDistanceJoint( m_worldId, ref distanceDef );

		Car car = new();
		car.Spawn( m_worldId, new( 0.0f, y + 2.0f ), 1.0f, 3.0f, 0.7f, 0.0f, null );
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			DistanceJoint_EnableMotor( m_liftJointId, m_enableMotor );
			Joint_WakeBodies( m_liftJointId );
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Max Force", ref m_motorForce, 0.0f, 3000.0f, "%.0f" ) )
		{
			DistanceJoint_SetMaxMotorForce( m_liftJointId, m_motorForce );
			Joint_WakeBodies( m_liftJointId );
		}

		if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -0.3f, 0.3f, "%.2f" ) )
		{
			DistanceJoint_SetTargetMotorSpeed( m_liftJointId, m_motorSpeed );
			Joint_WakeBodies( m_liftJointId );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new ScissorLift( context );
	}

	public JointId m_liftJointId;
	public float m_motorForce;
	public float m_motorSpeed;
	public bool m_enableMotor;
}

public class GearLift : Sample
{
	public GearLift( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 7.0f;
			m_context.debugDraw.drawJoints = false;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
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

		Pos gearPosition1 = new Pos { x = -4.25f, y = 9.75f };
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
			m_motorSpeed = 0.0f;
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
					bodyDef.position = new Pos { x = x, y = y };
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
	}

	public static Sample Create( SampleContext context )
	{
		return new GearLift( context );
	}

	public JointId m_driverId;
	public float m_motorTorque;
	public float m_motorSpeed;
	public bool m_enableMotor;
}

public class Door : Sample
{
	public Door( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 4.0f;
		}

		BodyId groundId = nullBodyId;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			groundId = CreateBody( m_worldId, ref bodyDef );
		}

		m_enableLimit = true;
		m_impulse = 50000.0f;
		m_translationError = 0.0f;
		m_jointHertz = 240.0f;
		m_jointDampingRatio = 1.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 1.5f };
			bodyDef.gravityScale = 0.0f;

			m_doorId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1000.0f;

			Polygon box = MakeBox( 0.1f, 1.5f );
			CreatePolygonShape( m_doorId, ref shapeDef, ref box );

			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_doorId;
			jointDef.@base.localFrameA.p = new( 0.0f, 0.0f );
			jointDef.@base.localFrameB.p = new( 0.0f, -1.5f );
			jointDef.@base.constraintHertz = m_jointHertz;
			jointDef.@base.constraintDampingRatio = m_jointDampingRatio;
			jointDef.targetAngle = 0.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = 0.0f;
			jointDef.enableMotor = false;
			jointDef.lowerAngle = -0.5f * Pi;
			jointDef.upperAngle = 0.5f * Pi;
			jointDef.enableLimit = m_enableLimit;

			m_jointId = CreateRevoluteJoint( m_worldId, ref jointDef );
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Button( "impulse" ) )
		{
			Pos p = Body_GetWorldPoint( m_doorId, new( 0.0f, 1.5f ) );
			Body_ApplyLinearImpulse( m_doorId, new( m_impulse, 0.0f ), p, true );
			m_translationError = 0.0f;
		}

		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		ImGui.SliderFloat( "magnitude", ref m_impulse, 1000.0f, 100000.0f, "%.0f" );

		if ( ImGui.SliderFloat( "hertz", ref m_jointHertz, 15.0f, 480.0f, "%.0f" ) )
		{
			Joint_SetConstraintTuning( m_jointId, m_jointHertz, m_jointDampingRatio );
		}

		if ( ImGui.SliderFloat( "damping", ref m_jointDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			Joint_SetConstraintTuning( m_jointId, m_jointHertz, m_jointDampingRatio );
		}

		ImGui.PopItemWidth();

		if ( ImGui.Checkbox( "limit", ref m_enableLimit ) )
		{
			RevoluteJoint_EnableLimit( m_jointId, m_enableLimit );
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		Pos p = Body_GetWorldPoint( m_doorId, new( 0.0f, 1.5f ) );
		DrawPoint( m_draw, p, 5.0f, HexColor.colorDarkKhaki );

		float translationError = Joint_GetLinearSeparation( m_jointId );
		m_translationError = MaxFloat( m_translationError, translationError );

		DrawScreenTextLine( "translation error = %g", m_translationError );
	}

	public static Sample Create( SampleContext context )
	{
		return new Door( context );
	}

	public BodyId m_doorId;
	public JointId m_jointId;
	public float m_impulse;
	public float m_translationError;
	public float m_jointHertz;
	public float m_jointDampingRatio;
	public bool m_enableLimit;
}

public class ScaleRagdoll : Sample
{
	public ScaleRagdoll( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 4.5f );
			m_context.camera.zoom = 6.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			Polygon box = MakeOffsetBox( 20.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_scale = 1.0f;

		m_human = new Human {  };

		Spawn();
	}

	public void Spawn(  )
	{
		float jointFrictionTorque = 0.03f;
		float jointHertz = 1.0f;
		float jointDampingRatio = 0.5f;
		CreateHuman( ref m_human, m_worldId, new( 0.0f, 5.0f ), m_scale, jointFrictionTorque, jointHertz, jointDampingRatio, 1, null,
					 false );
		Human_ApplyRandomAngularImpulse( ref m_human, 0.1f );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Scale", ref m_scale, 0.1f, 10.0f, "%3.2f", ImGuiSliderFlags.ClampOnInput ) )
		{
			Human_SetScale( ref m_human, m_scale );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new ScaleRagdoll( context );
	}

	public Human m_human;
	public float m_scale;
}

public class TheoJansen : Sample
{
	public TheoJansen( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 25.0f * 0.5f;
		}

		m_offset = new Vec2( 0.0f, 8.0f );
		m_motorSpeed = 2.0f;
		m_enableMotor = true;
		Vec2 pivot = new Vec2( 0.0f, 0.8f );

		// Ground
		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			Segment segment = new Segment { point1 = new( -50.0f, 0.0f ), point2 = new( 50.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			segment = new Segment { point1 = new( -50.0f, 0.0f ), point2 = new( -50.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );

			segment = new Segment { point1 = new( 50.0f, 0.0f ), point2 = new( 50.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		// Balls
		for ( int i = 0; i < 40; ++i )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -40.0f + 2.0f * i, y = 0.5f };

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		// Chassis
		{
			Polygon box = MakeBox( 2.5f, 1.0f );

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = ToPos( pivot ) + m_offset;
			m_chassisId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			shapeDef.filter.groupIndex = -1;
			CreatePolygonShape( m_chassisId, ref shapeDef, ref box );
		}

		// Wheel
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 1.6f };

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = ToPos( pivot ) + m_offset;
			m_wheelId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.density = 1.0f;
			shapeDef.filter.groupIndex = -1;
			CreateCircleShape( m_wheelId, ref shapeDef, ref circle );
		}

		{
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = m_wheelId;
			jointDef.@base.bodyIdB = m_chassisId;

			Pos motorPivot = ToPos( pivot ) + m_offset;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, motorPivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, motorPivot );

			jointDef.enableMotor = m_enableMotor;
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = 400.0f;
			m_motorJointId = CreateRevoluteJoint( m_worldId, ref jointDef );
		}

		Vec2 wheelAnchor = pivot + new Vec2( 0.0f, -0.8f );

		CreateLeg( -1.0f, wheelAnchor );
		CreateLeg( 1.0f, wheelAnchor );

		// Stagger three leg pairs around the wheel so some feet always touch the ground
		Body_SetTransform( m_wheelId, Body_GetPosition( m_wheelId ), MakeRot( 120.0f * Pi / 180.0f ) );
		CreateLeg( -1.0f, wheelAnchor );
		CreateLeg( 1.0f, wheelAnchor );

		Body_SetTransform( m_wheelId, Body_GetPosition( m_wheelId ), MakeRot( -120.0f * Pi / 180.0f ) );
		CreateLeg( -1.0f, wheelAnchor );
		CreateLeg( 1.0f, wheelAnchor );
	}

	public void CreateLeg( float s, Vec2 wheelAnchor )
	{
		Vec2 p1 = new Vec2( 5.4f * s, -6.1f );
		Vec2 p2 = new Vec2( 7.2f * s, -1.2f );
		Vec2 p3 = new Vec2( 4.3f * s, -1.9f );
		Vec2 p4 = new Vec2( 3.1f * s, 0.8f );
		Vec2 p5 = new Vec2( 6.0f * s, 1.5f );
		Vec2 p6 = new Vec2( 2.5f * s, 3.7f );

		// The hull computation handles the mirrored winding for s < 0
		Vec2[] vertices1 = { p1, p2, p3 };
		Vec2[] vertices2 = { Vec2_zero, p5 - p4, p6 - p4 };

		Hull hull1 = ComputeHull( vertices1, 3 );
		Hull hull2 = ComputeHull( vertices2, 3 );
		Polygon poly1 = MakePolygon( ref hull1, 0.0f );
		Polygon poly2 = MakePolygon( ref hull2, 0.0f );

		BodyDef bodyDef1 = DefaultBodyDef();
		bodyDef1.type = BodyType.dynamicBody;
		bodyDef1.position = ToPos( m_offset );
		bodyDef1.angularDamping = 10.0f;
		BodyId body1 = CreateBody( m_worldId, ref bodyDef1 );

		BodyDef bodyDef2 = DefaultBodyDef();
		bodyDef2.type = BodyType.dynamicBody;
		bodyDef2.position = ToPos( p4 ) + m_offset;
		bodyDef2.angularDamping = 10.0f;
		BodyId body2 = CreateBody( m_worldId, ref bodyDef2 );

		// Negative group index keeps the legs from colliding with the chassis and wheel
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.filter.groupIndex = -1;
		CreatePolygonShape( body1, ref shapeDef, ref poly1 );
		CreatePolygonShape( body2, ref shapeDef, ref poly2 );

		{
			// Soft distance constraints reduce jitter and act like a suspension system
			DistanceJointDef jointDef = DefaultDistanceJointDef();
			jointDef.enableSpring = true;
			jointDef.hertz = 10.0f;
			jointDef.dampingRatio = 0.5f;

			jointDef.@base.bodyIdA = body1;
			jointDef.@base.bodyIdB = body2;

			Pos anchorA = ToPos( p2 ) + m_offset;
			Pos anchorB = ToPos( p5 ) + m_offset;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( body1, anchorA );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( body2, anchorB );
			jointDef.length = Length( anchorA - anchorB );
			CreateDistanceJoint( m_worldId, ref jointDef );

			anchorA = ToPos( p3 ) + m_offset;
			anchorB = ToPos( p4 ) + m_offset;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( body1, anchorA );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( body2, anchorB );
			jointDef.length = Length( anchorA - anchorB );
			CreateDistanceJoint( m_worldId, ref jointDef );

			jointDef.@base.bodyIdB = m_wheelId;
			anchorB = ToPos( wheelAnchor ) + m_offset;
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( m_wheelId, anchorB );
			jointDef.length = Length( anchorA - anchorB );
			CreateDistanceJoint( m_worldId, ref jointDef );

			jointDef.@base.bodyIdA = body2;
			anchorA = ToPos( p6 ) + m_offset;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( body2, anchorA );
			jointDef.length = Length( anchorA - anchorB );
			CreateDistanceJoint( m_worldId, ref jointDef );
		}

		{
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = body2;
			jointDef.@base.bodyIdB = m_chassisId;

			Pos legPivot = ToPos( p4 ) + m_offset;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( body2, legPivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( m_chassisId, legPivot );
			CreateRevoluteJoint( m_worldId, ref jointDef );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.Checkbox( "Motor", ref m_enableMotor ) )
		{
			RevoluteJoint_EnableMotor( m_motorJointId, m_enableMotor );
		}

		if ( ImGui.SliderFloat( "Speed", ref m_motorSpeed, -5.0f, 5.0f, "%.1f" ) )
		{
			RevoluteJoint_SetTargetMotorSpeed( m_motorJointId, m_motorSpeed );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "Keys: left = a, brake = s, right = d, toggle motor = f" );

		base.Step();
	}

	public override void Keyboard( int key, int unused26, int unused27 )
	{
		switch ( key )
		{
			case GLFW_KEY_A:
				RevoluteJoint_SetTargetMotorSpeed( m_motorJointId, -m_motorSpeed );
				break;

			case GLFW_KEY_S:
				RevoluteJoint_SetTargetMotorSpeed( m_motorJointId, 0.0f );
				break;

			case GLFW_KEY_D:
				RevoluteJoint_SetTargetMotorSpeed( m_motorJointId, m_motorSpeed );
				break;

			case GLFW_KEY_F:
				m_enableMotor = !RevoluteJoint_IsMotorEnabled( m_motorJointId );
				RevoluteJoint_EnableMotor( m_motorJointId, m_enableMotor );
				break;

			default:
				break;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new TheoJansen( context );
	}

	public Vec2 m_offset;
	public BodyId m_chassisId;
	public BodyId m_wheelId;
	public JointId m_motorJointId;
	public float m_motorSpeed;
	public bool m_enableMotor;
}

public class DeskLamp : Sample
{
	public const int e_springCount = 2;

	public DeskLamp( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.2f, 1.42f );
			m_context.camera.zoom = 1.6f;
		}

		// The parallelogram is narrow, so it wants the extra resolution
		m_context.subStepCount = 8;

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			Segment segment = new Segment { point1 = new( -20.0f, 0.0f ), point2 = new( 20.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		// Dimensions taken off an Original 1227 in meters, then scaled up. At life size the silver
		// link is two centimeters, which is far too close to the linear slop.
		float scale = 5.0f;

		float baseHalfWidth = 0.070f * scale;
		float baseHalfHeight = 0.018f * scale;
		float postHalfWidth = 0.008f * scale;
		float pivotHeight = 0.185f * scale;
		float springAnchorHeight = 0.095f * scale;
		float topLinkLength = 0.296f * scale;
		float silverFoot = 0.024f * scale;
		float topLeverLength = 0.019f * scale;
		float lowerLeverLength = 0.028f * scale;
		float upperArmLength = 0.324f * scale;
		float tubeRadius = 0.0036f * scale;
		float shadeLength = 0.161f * scale;
		float shadeNeckRadius = 0.020f * scale;
		float shadeMouthRadius = 0.061f * scale;

		// The silver link measured in the top link frame. The same span separates the two pins on the
		// upper arm, which is what closes the parallelogram.
		Vec2 silverSpan = new Vec2( 0.0212f * scale, 0.0109f * scale );
		float silverLength = Length( silverSpan );

		float topLinkAngle = -0.878f;
		float upperArmAngle = 1.192f;
		float shadeTilt = -0.41f;

		// The arm spring balances the elevation, the head spring balances the upper arm through the
		// parallelogram. That is what the real lamp uses two springs for.
		m_springHertz[0] = 5.0f;
		m_springHertz[1] = 5.0f;
		m_springDamping = 4.0f;
		m_lowerFriction = 100.0f;
		m_elbowFriction = 100.0f;
		m_headAngle = 0.0f;
		Rot topRotation = MakeRot( topLinkAngle );
		Rot upperRotation = MakeRot( upperArmAngle );

		Vec2 topAxis = Rot_GetYAxis( topRotation );
		Vec2 silverWorld = RotateVector( topRotation, silverSpan );

		Pos postPivot = new Pos { x = 0.0f, y = pivotHeight };
		Pos upperPin = OffsetPos( postPivot, MulSV( topLinkLength, topAxis ) );
		Pos silverBase = OffsetPos( postPivot, MulSV( silverFoot, topAxis ) );
		Pos lowerFoot = OffsetPos( silverBase, silverWorld );
		Pos lowerPin = OffsetPos( upperPin, silverWorld );

		// The lower link is parallel to the top link and shorter by the silver link foot
		float lowerLinkLength = topLinkLength - silverFoot;
		Rot silverRotation = MakeRotFromUnitVector( RightPerp( Normalize( silverWorld ) ) );

		Pos headPivot = OffsetPos( upperPin, MulSV( upperArmLength, Rot_GetYAxis( upperRotation ) ) );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();

		BodyId baseId = default;
		{
			// The base is static so the linkage is all that moves
			bodyDef.type = BodyType.staticBody;
			bodyDef.position = Pos_zero;
			bodyDef.rotation = Rot_identity;
			baseId = CreateBody( m_worldId, ref bodyDef );

			Polygon plinth = MakeOffsetBox( baseHalfWidth, baseHalfHeight, new( 0.0f, baseHalfHeight ), Rot_identity );
			float postHalfHeight = 0.5f * ( pivotHeight - 2.0f * baseHalfHeight );
			Polygon post =
				MakeOffsetBox( postHalfWidth, postHalfHeight, new( 0.0f, pivotHeight - postHalfHeight ), Rot_identity );

			CreatePolygonShape( baseId, ref shapeDef, ref plinth );
			CreatePolygonShape( baseId, ref shapeDef, ref post );
		}

		bodyDef.type = BodyType.dynamicBody;
		shapeDef.density = 20.0f;

		// Both long links run past their feet into the levers the springs pull on
		{
			bodyDef.position = postPivot;
			bodyDef.rotation = topRotation;
			m_topLinkId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, -topLeverLength ), center2 = new( 0.0f, topLinkLength ), radius = tubeRadius };
			CreateCapsuleShape( m_topLinkId, ref shapeDef, ref capsule );
		}

		BodyId lowerLinkId = default;
		{
			bodyDef.position = lowerFoot;
			bodyDef.rotation = topRotation;
			lowerLinkId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, -lowerLeverLength ), center2 = new( 0.0f, lowerLinkLength ), radius = tubeRadius };
			CreateCapsuleShape( lowerLinkId, ref shapeDef, ref capsule );
		}

		BodyId silverId = default;
		{
			bodyDef.position = silverBase;
			bodyDef.rotation = silverRotation;
			silverId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = new( 0.0f, silverLength ), radius = 1.4f * tubeRadius };
			shapeDef.material.customColor = (uint)HexColor.colorSilver;
			CreateCapsuleShape( silverId, ref shapeDef, ref capsule );
			shapeDef.material.customColor = 0;
		}

		BodyId upperArmId = default;
		{
			bodyDef.position = upperPin;
			bodyDef.rotation = upperRotation;
			upperArmId = CreateBody( m_worldId, ref bodyDef );

			Capsule capsule = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = new( 0.0f, upperArmLength ), radius = 0.8f * tubeRadius };
			CreateCapsuleShape( upperArmId, ref shapeDef, ref capsule );

			// The coupler span reaches back to the lower link pin
			Capsule bracket = new Capsule { center1 = new( 0.0f, 0.0f ), center2 = Body_GetLocalPoint( upperArmId, lowerPin ), radius = tubeRadius };
			CreateCapsuleShape( upperArmId, ref shapeDef, ref bracket );
		}

		RevoluteJointDef pinDef = DefaultRevoluteJointDef();
		pinDef.@base.drawScale = 0.03f * scale;
		pinDef.@base.constraintHertz = 240.0f;
		pinDef.@base.constraintDampingRatio = 2.0f;

		// Only the top link reaches the post. The springs carry it, so it needs little friction.
		pinDef.@base.bodyIdA = baseId;
		pinDef.@base.bodyIdB = m_topLinkId;
		pinDef.@base.localFrameA.p = Body_GetLocalPoint( baseId, postPivot );
		pinDef.@base.localFrameB.p = Vec2_zero;
		pinDef.enableMotor = true;
		pinDef.maxMotorTorque = m_lowerFriction;
		m_postJointId = CreateRevoluteJoint( m_worldId, ref pinDef );

		pinDef.enableMotor = false;

		// The silver link hangs off the top link and carries the foot of the lower link
		pinDef.@base.bodyIdA = m_topLinkId;
		pinDef.@base.bodyIdB = silverId;
		pinDef.@base.localFrameA.p = new( 0.0f, silverFoot );
		pinDef.@base.localFrameB.p = Vec2_zero;
		CreateRevoluteJoint( m_worldId, ref pinDef );

		pinDef.@base.bodyIdA = silverId;
		pinDef.@base.bodyIdB = lowerLinkId;
		pinDef.@base.localFrameA.p = new( 0.0f, silverLength );
		pinDef.@base.localFrameB.p = Vec2_zero;
		CreateRevoluteJoint( m_worldId, ref pinDef );

		// Both long links reach the upper arm, at points a silver link apart.
		pinDef.@base.bodyIdA = lowerLinkId;
		pinDef.@base.bodyIdB = upperArmId;
		pinDef.@base.localFrameA.p = new( 0.0f, lowerLinkLength );
		pinDef.@base.localFrameB.p = Body_GetLocalPoint( upperArmId, lowerPin );
		CreateRevoluteJoint( m_worldId, ref pinDef );

		// No spring reaches the elbow, so it is a friction pivot like the real one.
		pinDef.@base.bodyIdA = m_topLinkId;
		pinDef.@base.bodyIdB = upperArmId;
		pinDef.@base.localFrameA.p = new( 0.0f, topLinkLength );
		pinDef.@base.localFrameB.p = Vec2_zero;
		pinDef.enableMotor = true;
		pinDef.maxMotorTorque = m_elbowFriction;
		m_elbowJointId = CreateRevoluteJoint( m_worldId, ref pinDef );

		DistanceJointDef springDef = DefaultDistanceJointDef();
		springDef.@base.drawScale = 0.03f * scale;
		springDef.enableSpring = true;
		springDef.dampingRatio = m_springDamping;

		// A coil spring only pulls
		//springDef.upperSpringForce = 0.0f;

		// Carwardine's trick. With almost no free length the spring torque about the pivot is
		// k times cross(lever, anchor), which tracks the gravity torque as the arm swings, so the
		// arm balances over its whole range instead of at one angle.
		springDef.length = 0.01f;

		Pos springBase = new Pos { x = 0.0f, y = springAnchorHeight };
		BodyId[] leverIds = { m_topLinkId, lowerLinkId };
		float[] leverLengths = { topLeverLength, lowerLeverLength };

		for ( int k = 0; k < e_springCount; ++k )
		{
			springDef.@base.bodyIdA = baseId;
			springDef.@base.bodyIdB = leverIds[k];
			springDef.@base.localFrameA.p = Body_GetLocalPoint( baseId, springBase );
			springDef.@base.localFrameB.p = new( 0.0f, -leverLengths[k] );
			springDef.hertz = m_springHertz[k];
			m_springJointIds[k] = CreateDistanceJoint( m_worldId, ref springDef );
		}

		{
			bodyDef.position = headPivot;
			bodyDef.rotation = MakeRot( shadeTilt );
			BodyId headId = CreateBody( m_worldId, ref bodyDef );

			// The pivot is on the shade center line, where the arm capsule ends. The shade opens along
			// the local negative y.
			float neck = 0.45f * shadeLength;
			float mouth = neck - shadeLength;
			Vec2[] points = { new( -shadeNeckRadius, neck ), new( shadeNeckRadius, neck ), new( shadeMouthRadius, mouth ),
								 new( -shadeMouthRadius, mouth ) };
			Hull hull = ComputeHull( points, 4 );
			Polygon shade = MakePolygon( ref hull, 0.01f * scale );

			// The shade is a thin shell.
			shapeDef.density = 1.0f;
			CreatePolygonShape( headId, ref shapeDef, ref shade );

			// Offset the reference frame so the head slider reads zero at the pose above.
			pinDef.@base.bodyIdA = upperArmId;
			pinDef.@base.bodyIdB = headId;
			pinDef.@base.localFrameA.p = new( 0.0f, upperArmLength );
			pinDef.@base.localFrameA.q = InvMulRot( upperRotation, MakeRot( shadeTilt ) );
			pinDef.@base.localFrameB.p = Vec2_zero;
			pinDef.enableMotor = false;
			pinDef.maxMotorTorque = 10.0f;
			pinDef.enableSpring = true;
			pinDef.hertz = 8.0f;
			pinDef.dampingRatio = 2.0f;
			pinDef.targetAngle = m_headAngle;
			pinDef.enableLimit = true;
			pinDef.lowerAngle = -1.2f;
			pinDef.upperAngle = 1.2f;
			m_headJointId = CreateRevoluteJoint( m_worldId, ref pinDef );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		if ( ImGui.SliderFloat( "Arm Spring", ref m_springHertz[0], 0.0f, 20.0f, "%.2f" ) )
		{
			DistanceJoint_SetSpringHertz( m_springJointIds[0], m_springHertz[0] );
			Joint_WakeBodies( m_springJointIds[0] );
		}

		if ( ImGui.SliderFloat( "Head Spring", ref m_springHertz[1], 0.0f, 20.0f, "%.2f" ) )
		{
			DistanceJoint_SetSpringHertz( m_springJointIds[1], m_springHertz[1] );
			Joint_WakeBodies( m_springJointIds[1] );
		}

		if ( ImGui.SliderFloat( "Spring Damping", ref m_springDamping, 0.0f, 5.0f, "%.1f" ) )
		{
			for ( int k = 0; k < e_springCount; ++k )
			{
				DistanceJoint_SetSpringDampingRatio( m_springJointIds[k], m_springDamping );
				Joint_WakeBodies( m_springJointIds[k] );
			}
		}

		if ( ImGui.SliderFloat( "Post Friction", ref m_lowerFriction, 0.0f, 20.0f, "%.1f" ) )
		{
			RevoluteJoint_SetMaxMotorTorque( m_postJointId, m_lowerFriction );
			Joint_WakeBodies( m_postJointId );
		}

		if ( ImGui.SliderFloat( "Elbow Friction", ref m_elbowFriction, 0.0f, 60.0f, "%.0f" ) )
		{
			RevoluteJoint_SetMaxMotorTorque( m_elbowJointId, m_elbowFriction );
			Joint_WakeBodies( m_elbowJointId );
		}

		if ( ImGui.SliderFloat( "Head", ref m_headAngle, -1.2f, 1.2f, "%.2f" ) )
		{
			RevoluteJoint_SetTargetAngle( m_headJointId, m_headAngle );
			Joint_WakeBodies( m_headJointId );
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		float postTorque = RevoluteJoint_GetCurrentMotorTorque( m_postJointId );
		float elbowTorque = RevoluteJoint_GetCurrentMotorTorque( m_elbowJointId );
		DrawScreenTextLine( "friction carrying: post %.2f, elbow %.2f N m", postTorque, elbowTorque );

		float spring1 = DistanceJoint_GetSpringForce( m_springJointIds[0] );
		float spring2 = DistanceJoint_GetSpringForce( m_springJointIds[1] );
		DrawScreenTextLine( "spring forces: %.2f, %.2f N", spring1, spring2 );
	}

	public static Sample Create( SampleContext context )
	{
		return new DeskLamp( context );
	}

	public JointId[] m_springJointIds = new JointId[e_springCount];
	public JointId m_postJointId;
	public JointId m_elbowJointId;
	public JointId m_headJointId;
	public BodyId m_topLinkId;
	public float[] m_springHertz = new float[e_springCount];
	public float m_springDamping;
	public float m_lowerFriction;
	public float m_elbowFriction;
	public float m_headAngle;
}
