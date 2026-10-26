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

public class Car
{



	public BodyId m_chassisId;
	public BodyId m_rearWheelId;
	public BodyId m_frontWheelId;
	public JointId m_rearAxleId;
	public JointId m_frontAxleId;
	public bool m_isSpawned;

	public Car(  )
	{
		m_chassisId = new BodyId {  };
		m_rearWheelId = new BodyId {  };
		m_frontWheelId = new BodyId {  };
		m_rearAxleId = new JointId {  };
		m_frontAxleId = new JointId {  };
		m_isSpawned = false;
	}

	public void Spawn( WorldId worldId, Pos position, float scale, float hertz, float dampingRatio, float torque, object? userData )
	{
		Debug.Assert( m_isSpawned == false );

		Debug.Assert( m_chassisId.IsNull );
		Debug.Assert( m_frontWheelId.IsNull );
		Debug.Assert( m_rearWheelId.IsNull );

		Vec2[] vertices = {
			new( -1.5f, -0.5f ), new( 1.5f, -0.5f ), new( 1.5f, 0.0f ), new( 0.0f, 0.9f ), new( -1.15f, 0.9f ), new( -1.5f, 0.2f ),
		};

		for ( int i = 0; i < 6; ++i )
		{
			vertices[i].x *= 0.85f * scale;
			vertices[i].y *= 0.85f * scale;
		}

		Hull hull = ComputeHull( vertices, 6 );
		Polygon chassis = MakePolygon( ref hull, 0.15f * scale );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f / scale;
		shapeDef.material.friction = 0.2f;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.4f * scale };

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = OffsetPos( position, new( 0.0f, 1.0f * scale ) );
		m_chassisId = CreateBody( worldId, ref bodyDef );
		CreatePolygonShape( m_chassisId, ref shapeDef, ref chassis );

		shapeDef.density = 2.0f / scale;
		shapeDef.material.friction = 1.5f;
		shapeDef.material.rollingResistance = 0.1f;

		bodyDef.position = OffsetPos( position, new( -1.0f * scale, 0.35f * scale ) );
		bodyDef.allowFastRotation = true;
		m_rearWheelId = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_rearWheelId, ref shapeDef, ref circle );

		bodyDef.position = OffsetPos( position, new( 1.0f * scale, 0.4f * scale ) );
		bodyDef.allowFastRotation = true;
		m_frontWheelId = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_frontWheelId, ref shapeDef, ref circle );

		Pos pivot = Body_GetPosition( m_rearWheelId );

		// float throttle = 0.0f;
		// float speed = 35.0f;
		// float torque = 2.5f * scale;
		// float hertz = 5.0f;
		// float dampingRatio = 0.7f;

		WheelJointDef jointDef = DefaultWheelJointDef();

		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_rearWheelId;
		jointDef.@base.localFrameA.q = MakeRot( 0.5f * Pi );
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
		jointDef.motorSpeed = 0.0f;
		jointDef.maxMotorTorque = torque;
		jointDef.enableMotor = true;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.lowerTranslation = -0.25f * scale;
		jointDef.upperTranslation = 0.25f * scale;
		jointDef.enableLimit = true;
		m_rearAxleId = CreateWheelJoint( worldId, ref jointDef );

		pivot = Body_GetPosition( m_frontWheelId );
		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_frontWheelId;
		jointDef.@base.localFrameA.q = MakeRot( 0.5f * Pi );
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
		jointDef.motorSpeed = 0.0f;
		jointDef.maxMotorTorque = torque;
		jointDef.enableMotor = true;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.lowerTranslation = -0.25f * scale;
		jointDef.upperTranslation = 0.25f * scale;
		jointDef.enableLimit = true;
		m_frontAxleId = CreateWheelJoint( worldId, ref jointDef );
	}

	public void Despawn(  )
	{
		Debug.Assert( m_isSpawned == true );

		DestroyJoint( m_rearAxleId );
		DestroyJoint( m_frontAxleId );
		DestroyBody( m_rearWheelId );
		DestroyBody( m_frontWheelId );
		DestroyBody( m_chassisId );

		m_isSpawned = false;
	}

	public void SetSpeed( float speed )
	{
		WheelJoint_SetTargetMotorSpeed( m_rearAxleId, speed );
		WheelJoint_SetTargetMotorSpeed( m_frontAxleId, speed );
		Joint_WakeBodies( m_rearAxleId );
	}

	public void SetTorque( float torque )
	{
		WheelJoint_SetMaxMotorTorque( m_rearAxleId, torque );
		WheelJoint_SetMaxMotorTorque( m_frontAxleId, torque );
	}

	public void SetHertz( float hertz )
	{
		WheelJoint_SetSpringHertz( m_rearAxleId, hertz );
		WheelJoint_SetSpringHertz( m_frontAxleId, hertz );
	}

	public void SetDampingRadio( float dampingRatio )
	{
		WheelJoint_SetSpringDampingRatio( m_rearAxleId, dampingRatio );
		WheelJoint_SetSpringDampingRatio( m_frontAxleId, dampingRatio );
	}
}

public class Truck
{



	public BodyId m_chassisId;
	public BodyId m_rearWheelId;
	public BodyId m_frontWheelId;
	public JointId m_rearAxleId;
	public JointId m_frontAxleId;
	public bool m_isSpawned;

	public Truck(  )
	{
		m_chassisId = new BodyId {  };
		m_rearWheelId = new BodyId {  };
		m_frontWheelId = new BodyId {  };
		m_rearAxleId = new JointId {  };
		m_frontAxleId = new JointId {  };
		m_isSpawned = false;
	}

	public void Spawn( WorldId worldId, Pos position, float scale, float hertz, float dampingRatio, float torque, float density, object? userData )
	{
		Debug.Assert( m_isSpawned == false );

		Debug.Assert( m_chassisId.IsNull );
		Debug.Assert( m_frontWheelId.IsNull );
		Debug.Assert( m_rearWheelId.IsNull );

		// Vec2 vertices[6] = {
		//	{ -1.5f, -0.5f }, new( 1.5f, -0.5f ), new( 1.5f, 0.0f ), new( 0.0f, 0.9f ), new( -1.15f, 0.9f ), new( -1.5f, 0.2f ),
		// };

		Vec2[] vertices = {
			new( -0.65f, -0.4f ), new( 1.5f, -0.4f ), new( 1.5f, 0.0f ), new( 0.0f, 0.9f ), new( -0.65f, 0.9f ),
		};

		for ( int i = 0; i < 5; ++i )
		{
			vertices[i].x *= 0.85f * scale;
			vertices[i].y *= 0.85f * scale;
		}

		Hull hull = ComputeHull( vertices, 5 );
		Polygon chassis = MakePolygon( ref hull, 0.15f * scale );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = density;
		shapeDef.material.friction = 0.2f;
		shapeDef.material.customColor = (uint)HexColor.colorHotPink;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = OffsetPos( position, new( 0.0f, 1.0f * scale ) );
		m_chassisId = CreateBody( worldId, ref bodyDef );
		CreatePolygonShape( m_chassisId, ref shapeDef, ref chassis );

		Polygon box = MakeOffsetBox( 1.25f * scale, 0.1f * scale, new( -2.05f * scale, -0.275f * scale ), Rot_identity );
		box.radius = 0.1f * scale;
		CreatePolygonShape( m_chassisId, ref shapeDef, ref box );

		box = MakeOffsetBox( 0.05f * scale, 0.35f * scale, new( -3.25f * scale, 0.375f * scale ), Rot_identity  );
		box.radius = 0.1f * scale;
		CreatePolygonShape( m_chassisId, ref shapeDef, ref box );

		shapeDef.density = 2.0f * density;
		shapeDef.material.friction = 2.5f;
		shapeDef.material.customColor = (uint)HexColor.colorSilver;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.4f * scale };
		bodyDef.position = OffsetPos( position, new( -2.75f * scale, 0.3f * scale ) );
		m_rearWheelId = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_rearWheelId, ref shapeDef, ref circle );

		bodyDef.position = OffsetPos( position, new( 0.8f * scale, 0.3f * scale ) );
		m_frontWheelId = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_frontWheelId, ref shapeDef, ref circle );

		Pos pivot = Body_GetPosition( m_rearWheelId );

		// float throttle = 0.0f;
		// float speed = 35.0f;
		// float torque = 2.5f * scale;
		// float hertz = 5.0f;
		// float dampingRatio = 0.7f;

		WheelJointDef jointDef = DefaultWheelJointDef();

		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_rearWheelId;
		jointDef.@base.localFrameA.q = MakeRot( 0.5f * Pi );
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
		jointDef.motorSpeed = 0.0f;
		jointDef.maxMotorTorque = torque;
		jointDef.enableMotor = true;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.lowerTranslation = -0.25f * scale;
		jointDef.upperTranslation = 0.25f * scale;
		jointDef.enableLimit = true;
		m_rearAxleId = CreateWheelJoint( worldId, ref jointDef );

		pivot = Body_GetPosition( m_frontWheelId );
		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_frontWheelId;
		jointDef.@base.localFrameA.q = MakeRot( 0.5f * Pi );
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
		jointDef.motorSpeed = 0.0f;
		jointDef.maxMotorTorque = torque;
		jointDef.enableMotor = true;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.lowerTranslation = -0.25f * scale;
		jointDef.upperTranslation = 0.25f * scale;
		jointDef.enableLimit = true;
		m_frontAxleId = CreateWheelJoint( worldId, ref jointDef );
	}

	public void Despawn(  )
	{
		Debug.Assert( m_isSpawned == true );

		DestroyJoint( m_rearAxleId );
		DestroyJoint( m_frontAxleId );
		DestroyBody( m_rearWheelId );
		DestroyBody( m_frontWheelId );
		DestroyBody( m_chassisId );

		m_isSpawned = false;
	}

	public void SetSpeed( float speed )
	{
		WheelJoint_SetTargetMotorSpeed( m_rearAxleId, speed );
		WheelJoint_SetTargetMotorSpeed( m_frontAxleId, speed );
		Joint_WakeBodies( m_rearAxleId );
	}

	public void SetTorque( float torque )
	{
		WheelJoint_SetMaxMotorTorque( m_rearAxleId, torque );
		WheelJoint_SetMaxMotorTorque( m_frontAxleId, torque );
	}

	public void SetHertz( float hertz )
	{
		WheelJoint_SetSpringHertz( m_rearAxleId, hertz );
		WheelJoint_SetSpringHertz( m_frontAxleId, hertz );
	}

	public void SetDampingRadio( float dampingRatio )
	{
		WheelJoint_SetSpringDampingRatio( m_rearAxleId, dampingRatio );
		WheelJoint_SetSpringDampingRatio( m_frontAxleId, dampingRatio );
	}
}
