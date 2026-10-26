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

public class Doohickey
{


	public BodyId m_wheelId1;
	public BodyId m_wheelId2;
	public BodyId m_barId1;
	public BodyId m_barId2;

	public JointId m_axleId1;
	public JointId m_axleId2;
	public JointId m_sliderId;

	public bool m_isSpawned;

	public Doohickey(  )
	{
		m_wheelId1 = new BodyId {  };
		m_wheelId2 = new BodyId {  };
		m_barId1 = new BodyId {  };
		m_barId2 = new BodyId {  };

		m_axleId1 = new JointId {  };
		m_axleId2 = new JointId {  };
		m_sliderId = new JointId {  };

		m_isSpawned = false;
	}

	public void Spawn( WorldId worldId, Pos position, float scale )
	{
		Debug.Assert( m_isSpawned == false );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.rollingResistance = 0.1f;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 1.0f * scale };
		Capsule capsule = new Capsule { center1 = new( -3.5f * scale, 0.0f ), center2 = new( 3.5f * scale, 0.0f ), radius = 0.15f * scale };

		bodyDef.position = OffsetPos( position, MulSV( scale, new( -5.0f, 3.0f ) ) );
		m_wheelId1 = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_wheelId1, ref shapeDef, ref circle );

		bodyDef.position = OffsetPos( position, MulSV( scale, new( 5.0f, 3.0f ) ) );
		m_wheelId2 = CreateBody( worldId, ref bodyDef );
		CreateCircleShape( m_wheelId2, ref shapeDef, ref circle );

		bodyDef.position = OffsetPos( position, MulSV( scale, new( -1.5f, 3.0f ) ) );
		m_barId1 = CreateBody( worldId, ref bodyDef );
		CreateCapsuleShape( m_barId1, ref shapeDef, ref capsule );

		bodyDef.position = OffsetPos( position, MulSV( scale, new( 1.5f, 3.0f ) ) );
		m_barId2 = CreateBody( worldId, ref bodyDef );
		CreateCapsuleShape( m_barId2, ref shapeDef, ref capsule );

		RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();

		revoluteDef.@base.bodyIdA = m_wheelId1;
		revoluteDef.@base.bodyIdB = m_barId1;
		revoluteDef.@base.localFrameA.p = new( 0.0f, 0.0f );
		revoluteDef.@base.localFrameB.p = new( -3.5f * scale, 0.0f );
		revoluteDef.enableMotor = true;
		revoluteDef.maxMotorTorque = 2.0f * scale;
		CreateRevoluteJoint( worldId, ref revoluteDef );

		revoluteDef.@base.bodyIdA = m_wheelId2;
		revoluteDef.@base.bodyIdB = m_barId2;
		revoluteDef.@base.localFrameA.p = new( 0.0f, 0.0f );
		revoluteDef.@base.localFrameB.p = new( 3.5f * scale, 0.0f );
		revoluteDef.enableMotor = true;
		revoluteDef.maxMotorTorque = 2.0f * scale;
		CreateRevoluteJoint( worldId, ref revoluteDef );

		PrismaticJointDef prismaticDef = DefaultPrismaticJointDef();
		prismaticDef.@base.bodyIdA = m_barId1;
		prismaticDef.@base.bodyIdB = m_barId2;
		prismaticDef.@base.localFrameA.p = new( 2.0f * scale, 0.0f );
		prismaticDef.@base.localFrameB.p = new( -2.0f * scale, 0.0f );
		prismaticDef.lowerTranslation = -2.0f * scale;
		prismaticDef.upperTranslation = 2.0f * scale;
		prismaticDef.enableLimit = true;
		prismaticDef.enableMotor = true;
		prismaticDef.maxMotorForce = 2.0f * scale;
		prismaticDef.enableSpring = true;
		prismaticDef.hertz = 1.0f;
		prismaticDef.dampingRatio = 0.5f;
		CreatePrismaticJoint( worldId, ref prismaticDef );
	}

	public void Despawn(  )
	{
		Debug.Assert( m_isSpawned == true );

		DestroyJoint( m_axleId1 );
		DestroyJoint( m_axleId2 );
		DestroyJoint( m_sliderId );

		DestroyBody( m_wheelId1 );
		DestroyBody( m_wheelId2 );
		DestroyBody( m_barId1 );
		DestroyBody( m_barId2 );

		m_isSpawned = false;
	}
}
