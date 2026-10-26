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

public class Donut
{


	public const int m_sides = 7;
	public BodyId[] m_bodyIds = new BodyId[m_sides];
	public JointId[] m_jointIds = new JointId[m_sides];
	public bool m_isSpawned;

	public Donut(  )
	{
		for ( int i = 0; i < m_sides; ++i )
		{
			m_bodyIds[i] = nullBodyId;
			m_jointIds[i] = nullJointId;
		}

		m_isSpawned = false;
	}

	public void Create( WorldId worldId, Pos position, float scale, int groupIndex, bool enableSensorEvents, object? userData )
	{
		Debug.Assert( m_isSpawned == false );

		for ( int i = 0; i < m_sides; ++i )
		{
			Debug.Assert( m_bodyIds[i].IsNull );
			Debug.Assert( m_jointIds[i].IsNull );
		}

		float radius = 1.0f * scale;
		float deltaAngle = 2.0f * Pi / m_sides;
		float length = 2.0f * Pi * radius / m_sides;

		Capsule capsule = new Capsule { center1 = new( 0.0f, -0.5f * length ), center2 = new( 0.0f, 0.5f * length ), radius = 0.25f * scale };

		Pos center = position;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.userData = userData;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = enableSensorEvents;
		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.material.friction = 0.3f;

		// Create bodies
		float angle = 0.0f;
		for ( int i = 0; i < m_sides; ++i )
		{
			bodyDef.position = new Pos { x = radius * MathF.Cos( angle ) + center.x, y = radius * MathF.Sin( angle ) + center.y };
			bodyDef.rotation = MakeRot( angle );

			m_bodyIds[i] = CreateBody( worldId, ref bodyDef );
			CreateCapsuleShape( m_bodyIds[i], ref shapeDef, ref capsule );

			angle += deltaAngle;
		}

		// Create joints
		WeldJointDef weldDef = DefaultWeldJointDef();
		weldDef.angularHertz = 5.0f;
		weldDef.angularDampingRatio = 0.0f;
		weldDef.@base.localFrameA.p = new( 0.0f, 0.5f * length );
		weldDef.@base.localFrameB.p = new( 0.0f, -0.5f * length );
		weldDef.@base.drawScale = 0.5f * scale;

		BodyId prevBodyId = m_bodyIds[m_sides - 1];
		for ( int i = 0; i < m_sides; ++i )
		{
			weldDef.@base.bodyIdA = prevBodyId;
			weldDef.@base.bodyIdB = m_bodyIds[i];
			Rot qA = Body_GetRotation( prevBodyId );
			Rot qB = Body_GetRotation( m_bodyIds[i] );
			weldDef.@base.localFrameA.q = InvMulRot( qA, qB );
			m_jointIds[i] = CreateWeldJoint( worldId, ref weldDef );
			prevBodyId = weldDef.@base.bodyIdB;
		}

		m_isSpawned = true;
	}

	public void Destroy(  )
	{
		Debug.Assert( m_isSpawned == true );

		for ( int i = 0; i < m_sides; ++i )
		{
			DestroyBody( m_bodyIds[i] );
			m_bodyIds[i] = nullBodyId;
			m_jointIds[i] = nullJointId;
		}

		m_isSpawned = false;
	}
}
