// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using static Box2DNet.B2;

namespace Box2DNet.Testbed;

public sealed class Settings
{
	public float hertz = 60.0f;
	public int subStepCount = 4;
	public int workerCount = 1;
	public bool enableSleep = true;
	public bool enableContinuous = true;
	public bool pause;
	public bool singleStep;
	public bool drawShapes = true;
	public bool drawJoints = true;
	public bool drawJointExtras;
	public bool drawBounds;
	public bool drawMass;
	public bool drawContacts;
	public bool drawContactNormals;
	public bool drawContactForces;
	public bool drawFrictionForces;
	public bool drawIslands;
	public bool drawProfile = true;
}

// Base class for testbed samples, modeled on shared/sample.cpp. Each sample owns a world and
// the mouse joint used to drag bodies around.
public abstract class Sample : IDisposable
{
	private const float k_mouseForceScale = 100.0f;

	protected readonly Camera m_camera;
	protected WorldId m_worldId;
	private BodyId m_mouseBodyId;
	private JointId m_mouseJointId;
	private Vec2 m_mousePoint;
	public int stepCount;

	protected Sample( Settings settings, Camera camera )
	{
		m_camera = camera;
		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = settings.workerCount;
		worldDef.enableSleep = settings.enableSleep;
		m_worldId = CreateWorld( worldDef );
	}

	public WorldId WorldId => m_worldId;

	public void Dispose()
	{
		DestroyWorld( m_worldId );
		m_worldId = default;
	}

	public virtual void UpdateGui()
	{
	}

	public virtual void Step( Settings settings, ref DebugDraw draw )
	{
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		if ( settings.pause )
		{
			if ( settings.singleStep )
			{
				settings.singleStep = false;
			}
			else
			{
				timeStep = 0.0f;
			}
		}

		if ( Body_IsValid( m_mouseBodyId ) )
		{
			Body_SetTargetTransform( m_mouseBodyId, new Transform( m_mousePoint, Rot_identity ), timeStep, true );
		}

		World_EnableSleeping( m_worldId, settings.enableSleep );
		World_EnableContinuous( m_worldId, settings.enableContinuous );

		World_Step( m_worldId, timeStep, settings.subStepCount );
		if ( timeStep > 0.0f )
		{
			++stepCount;
		}

		draw.drawShapes = settings.drawShapes;
		draw.drawJoints = settings.drawJoints;
		draw.drawJointExtras = settings.drawJointExtras;
		draw.drawBounds = settings.drawBounds;
		draw.drawMass = settings.drawMass;
		draw.drawContacts = settings.drawContacts;
		draw.drawContactNormals = settings.drawContactNormals;
		draw.drawContactForces = settings.drawContactForces;
		draw.drawFrictionForces = settings.drawFrictionForces;
		draw.drawIslands = settings.drawIslands;
		draw.drawingBounds = m_camera.GetViewBounds();
		World_Draw( m_worldId, ref draw );
	}

	private sealed class QueryContext
	{
		public Vec2 point;
		public BodyId bodyId;
	}

	private static bool QueryCallback( ShapeId shapeId, object? context )
	{
		QueryContext queryContext = (QueryContext)context!;

		BodyId bodyId = Shape_GetBody( shapeId );
		if ( Body_GetType( bodyId ) != BodyType.dynamicBody )
		{
			// continue query
			return true;
		}

		if ( Shape_TestPoint( shapeId, queryContext.point ) )
		{
			// found shape
			queryContext.bodyId = bodyId;
			return false;
		}

		return true;
	}

	public void MouseDown( Vec2 p )
	{
		if ( Joint_IsValid( m_mouseJointId ) )
		{
			return;
		}

		// Make a small box.
		Vec2 d = new( 0.001f, 0.001f );
		AABB box = new( Sub( p, d ), Add( p, d ) );

		// Query the world for overlapping shapes.
		QueryContext queryContext = new() { point = p, bodyId = nullBodyId };
		World_OverlapAABB( m_worldId, Vec2_zero, box, DefaultQueryFilter(), QueryCallback, queryContext );

		if ( queryContext.bodyId.IsNull )
		{
			return;
		}

		m_mousePoint = p;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.kinematicBody;
		bodyDef.position = p;
		bodyDef.enableSleep = false;
		m_mouseBodyId = CreateBody( m_worldId, bodyDef );

		MotorJointDef jointDef = DefaultMotorJointDef();
		jointDef.@base.bodyIdA = m_mouseBodyId;
		jointDef.@base.bodyIdB = queryContext.bodyId;
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( queryContext.bodyId, p );
		jointDef.linearHertz = 7.5f;
		jointDef.linearDampingRatio = 1.0f;

		MassData massData = Body_GetMassData( queryContext.bodyId );
		float g = Length( World_GetGravity( m_worldId ) );
		float mg = massData.mass * g;
		jointDef.maxSpringForce = k_mouseForceScale * mg;

		if ( massData.mass > 0.0f )
		{
			// This acts like angular friction
			float lever = MathF.Sqrt( massData.rotationalInertia / massData.mass );
			jointDef.maxVelocityTorque = 0.25f * lever * mg;
		}

		m_mouseJointId = CreateMotorJoint( m_worldId, jointDef );
	}

	public void MouseUp()
	{
		if ( Joint_IsValid( m_mouseJointId ) )
		{
			DestroyJoint( m_mouseJointId );
			m_mouseJointId = default;
		}

		if ( Body_IsValid( m_mouseBodyId ) )
		{
			DestroyBody( m_mouseBodyId );
			m_mouseBodyId = default;
		}
	}

	public void MouseMove( Vec2 p )
	{
		m_mousePoint = p;
	}

	protected BodyId CreateGround( float halfWidth = 40.0f )
	{
		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, bodyDef );
		Segment segment = new() { point1 = new Vec2( -halfWidth, 0.0f ), point2 = new Vec2( halfWidth, 0.0f ) };
		ShapeDef shapeDef = DefaultShapeDef();
		CreateSegmentShape( groundId, shapeDef, segment );
		return groundId;
	}
}
