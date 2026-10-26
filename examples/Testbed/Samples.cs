// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using Hexa.NET.ImGui;
using static Box2DNet.B2;

namespace Box2DNet.Testbed;

public sealed record SampleEntry( string category, string name, Func<Settings, Camera, Sample> create );

public static class SampleRegistry
{
	public static readonly SampleEntry[] entries =
	[
		new( "Stacking", "Pyramid", ( s, c ) => new Pyramid( s, c ) ),
		new( "Stacking", "Card House", ( s, c ) => new CardHouse( s, c ) ),
		new( "Joints", "Bridge", ( s, c ) => new Bridge( s, c ) ),
		new( "Joints", "Car", ( s, c ) => new Car( s, c ) ),
		new( "Determinism", "Falling Hinges", ( s, c ) => new FallingHinges( s, c ) ),
		new( "Shapes", "Shape Rain", ( s, c ) => new ShapeRain( s, c ) ),
	];
}

// Pyramid of boxes, from sample_stacking.cpp
public sealed class Pyramid : Sample
{
	private int m_baseCount = 20;

	public Pyramid( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.0f, 13.5f );
		camera.zoom = 0.65f;
		CreateGround();
		Build();
	}

	private void Build()
	{
		float h = 0.5f;
		Polygon box = MakeRoundedBox( h - 0.05f, h - 0.05f, 0.05f );
		ShapeDef shapeDef = DefaultShapeDef();
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		float shift = 1.0f * h;
		for ( int i = 0; i < m_baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * shift;
			for ( int j = i; j < m_baseCount; ++j )
			{
				float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * m_baseCount;
				bodyDef.position = new Vec2( x, y );
				BodyId bodyId = CreateBody( m_worldId, bodyDef );
				CreatePolygonShape( bodyId, shapeDef, box );
			}
		}
	}

	public override void UpdateGui()
	{
		ImGui.SliderInt( "Base Count", ref m_baseCount, 1, 100 );
		ImGui.TextDisabled( "Restart the sample to apply" );
	}
}

// Card house, from sample_stacking.cpp
public sealed class CardHouse : Sample
{
	public CardHouse( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.75f, 0.9f );
		camera.zoom = 0.05f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Vec2( 0.0f, -2.0f );
		BodyId groundId = CreateBody( m_worldId, bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.7f;
		Polygon groundBox = MakeBox( 40.0f, 2.0f );
		CreatePolygonShape( groundId, shapeDef, groundBox );

		float cardHeight = 0.2f;
		float cardThickness = 0.001f;
		float angle0 = 25.0f * MathF.PI / 180.0f;
		float angle1 = -25.0f * MathF.PI / 180.0f;
		float angle2 = 0.5f * MathF.PI;

		Polygon cardBox = MakeBox( cardThickness, cardHeight );
		bodyDef.type = BodyType.dynamicBody;

		int Nb = 5;
		float z0 = 0.0f;
		float y = cardHeight - 0.02f;
		while ( Nb > 0 )
		{
			float z = z0;
			for ( int i = 0; i < Nb; i++ )
			{
				if ( i != Nb - 1 )
				{
					bodyDef.position = new Vec2( z + 0.25f, y + cardHeight - 0.015f );
					bodyDef.rotation = MakeRot( angle2 );
					BodyId bid = CreateBody( m_worldId, bodyDef );
					CreatePolygonShape( bid, shapeDef, cardBox );
				}

				bodyDef.position = new Vec2( z, y );
				bodyDef.rotation = MakeRot( angle1 );
				BodyId bodyId = CreateBody( m_worldId, bodyDef );
				CreatePolygonShape( bodyId, shapeDef, cardBox );

				z += 0.175f;

				bodyDef.position = new Vec2( z, y );
				bodyDef.rotation = MakeRot( angle0 );
				bodyId = CreateBody( m_worldId, bodyDef );
				CreatePolygonShape( bodyId, shapeDef, cardBox );

				z += 0.175f;
			}

			y += cardHeight * 2.0f - 0.03f;
			z0 += 0.175f;
			Nb--;
		}
	}
}

// A bridge of planks linked with revolute joints, from sample_joints.cpp
public sealed class Bridge : Sample
{
	private const int k_count = 160;

	public Bridge( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.0f, 3.0f );
		camera.zoom = 0.9f;

		BodyId groundId = CreateBody( m_worldId, DefaultBodyDef() );

		Polygon box = MakeBox( 0.5f, 0.125f );
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 20.0f;

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 200.0f;

		float xbase = -80.0f;
		BodyId prevBodyId = groundId;
		for ( int i = 0; i < k_count; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Vec2( xbase + 0.5f + 1.0f * i, 20.0f );
			bodyDef.linearDamping = 0.1f;
			bodyDef.angularDamping = 0.1f;
			BodyId bodyId = CreateBody( m_worldId, bodyDef );
			CreatePolygonShape( bodyId, shapeDef, box );

			Vec2 pivot = new( xbase + 1.0f * i, 20.0f );
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( prevBodyId, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( bodyId, pivot );
			CreateRevoluteJoint( m_worldId, jointDef );

			prevBodyId = bodyId;
		}

		{
			Vec2 pivot = new( xbase + 1.0f * k_count, 20.0f );
			jointDef.@base.bodyIdA = prevBodyId;
			jointDef.@base.bodyIdB = groundId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( prevBodyId, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( groundId, pivot );
			CreateRevoluteJoint( m_worldId, jointDef );
		}

		// some things to drop on the bridge
		ShapeDef dropDef = DefaultShapeDef();
		for ( int i = 0; i < 4; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Vec2( -8.0f + 8.0f * i, 22.0f );
			BodyId bodyId = CreateBody( m_worldId, bodyDef );
			Circle circle = new() { center = Vec2_zero, radius = 0.5f + 0.25f * i };
			CreateCircleShape( bodyId, dropDef, circle );
		}
	}
}

// A simple car with wheel joints on rolling hills, after shared/car.c
public sealed class Car : Sample
{
	private readonly BodyId m_chassisId;
	private readonly JointId m_rearAxleId;
	private readonly JointId m_frontAxleId;
	private float m_speed = 35.0f;
	private float m_torque = 5.0f;
	private float m_hertz = 5.0f;
	private float m_dampingRatio = 0.7f;
	private int m_throttle;

	public Car( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.0f, 0.0f );
		camera.zoom = 0.4f;

		// rolling ground made from a chain
		{
			BodyId groundId = CreateBody( m_worldId, DefaultBodyDef() );

			const int count = 120;
			Vec2[] points = new Vec2[count];
			for ( int i = 0; i < count; ++i )
			{
				float x = 2.0f * ( count - 1 - i ) - 120.0f;
				float y = 1.5f * MathF.Sin( 0.15f * x ) + ( i < 10 || i > count - 10 ? 8.0f : 0.0f );
				points[i] = new Vec2( x, y );
			}

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = false;

			// an open chain needs ghost points beyond each end
			chainDef.ghost1 = Add( points[0], Sub( points[0], points[1] ) );
			chainDef.ghost2 = Add( points[count - 1], Sub( points[count - 1], points[count - 2] ) );
			CreateChain( groundId, chainDef );
		}

		// chassis
		float scale = 1.0f;
		Span<Vec2> vertices =
		[
			new( -1.5f, -0.5f ), new( 1.5f, -0.5f ), new( 1.5f, 0.0f ), new( 0.0f, 0.9f ), new( -1.15f, 0.9f ), new( -1.5f, 0.2f ),
		];

		Hull hull = ComputeHull( vertices );
		Polygon chassis = MakePolygon( hull, 0.15f * scale );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f / scale;
		shapeDef.material.friction = 0.2f;

		Circle circle = new() { center = Vec2_zero, radius = 0.4f * scale };

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Vec2( 0.0f, 3.0f );
		m_chassisId = CreateBody( m_worldId, bodyDef );
		CreatePolygonShape( m_chassisId, shapeDef, chassis );

		shapeDef.density = 2.0f / scale;
		shapeDef.material.friction = 1.5f;
		shapeDef.material.rollingResistance = 0.1f;

		bodyDef.position = new Vec2( -1.0f, 2.65f );
		bodyDef.allowFastRotation = true;
		BodyId rearWheelId = CreateBody( m_worldId, bodyDef );
		CreateCircleShape( rearWheelId, shapeDef, circle );

		bodyDef.position = new Vec2( 1.0f, 2.65f );
		BodyId frontWheelId = CreateBody( m_worldId, bodyDef );
		CreateCircleShape( frontWheelId, shapeDef, circle );

		Vec2 axis = new( 0.0f, 1.0f );
		Rot axisRotation = MakeRotFromUnitVector( axis );

		WheelJointDef jointDef = DefaultWheelJointDef();

		Vec2 pivot = Body_GetPosition( rearWheelId );
		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = rearWheelId;
		jointDef.@base.localFrameA.q = axisRotation;
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( m_chassisId, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( rearWheelId, pivot );
		jointDef.motorSpeed = 0.0f;
		jointDef.maxMotorTorque = m_torque;
		jointDef.enableMotor = true;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.lowerTranslation = -0.25f * scale;
		jointDef.upperTranslation = 0.25f * scale;
		jointDef.enableLimit = true;
		m_rearAxleId = CreateWheelJoint( m_worldId, jointDef );

		pivot = Body_GetPosition( frontWheelId );
		jointDef.@base.bodyIdB = frontWheelId;
		jointDef.@base.localFrameA.p = Body_GetLocalPoint( m_chassisId, pivot );
		jointDef.@base.localFrameB.p = Body_GetLocalPoint( frontWheelId, pivot );
		m_frontAxleId = CreateWheelJoint( m_worldId, jointDef );
	}

	public override void UpdateGui()
	{
		ImGui.TextWrapped( "Hold Left/Right (or the buttons) to drive" );
		if ( ImGui.Button( "Left" ) ) m_throttle = 1;
		ImGui.SameLine();
		if ( ImGui.Button( "Brake" ) ) m_throttle = 0;
		ImGui.SameLine();
		if ( ImGui.Button( "Right" ) ) m_throttle = -1;

		if ( ImGui.IsKeyDown( ImGuiKey.LeftArrow ) ) m_throttle = 1;
		else if ( ImGui.IsKeyDown( ImGuiKey.RightArrow ) ) m_throttle = -1;
		else if ( ImGui.IsKeyReleased( ImGuiKey.LeftArrow ) || ImGui.IsKeyReleased( ImGuiKey.RightArrow ) ) m_throttle = 0;

		ImGui.SliderFloat( "Speed", ref m_speed, 0.0f, 50.0f );
		if ( ImGui.SliderFloat( "Torque", ref m_torque, 0.0f, 10.0f ) )
		{
			WheelJoint_SetMaxMotorTorque( m_rearAxleId, m_torque );
			WheelJoint_SetMaxMotorTorque( m_frontAxleId, m_torque );
		}

		if ( ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 20.0f ) )
		{
			WheelJoint_SetSpringHertz( m_rearAxleId, m_hertz );
			WheelJoint_SetSpringHertz( m_frontAxleId, m_hertz );
		}

		if ( ImGui.SliderFloat( "Damping", ref m_dampingRatio, 0.0f, 2.0f ) )
		{
			WheelJoint_SetSpringDampingRatio( m_rearAxleId, m_dampingRatio );
			WheelJoint_SetSpringDampingRatio( m_frontAxleId, m_dampingRatio );
		}
	}

	public override void Step( Settings settings, ref DebugDraw draw )
	{
		WheelJoint_SetTargetMotorSpeed( m_rearAxleId, m_throttle * m_speed );
		WheelJoint_SetTargetMotorSpeed( m_frontAxleId, m_throttle * m_speed );
		Joint_WakeBodies( m_rearAxleId );

		base.Step( settings, ref draw );

		// follow the car
		Vec2 p = Body_GetPosition( m_chassisId );
		m_camera.center = new Vec2( p.x, m_camera.center.y );
	}
}

// The falling hinges scene used by the cross platform determinism test
public sealed class FallingHinges : Sample
{
	private const int k_columns = 4;
	private const int k_rows = 30;
	private readonly BodyId[] m_bodies = new BodyId[k_rows * k_columns];
	private ulong m_hash;
	private int m_sleepStep = -1;

	public FallingHinges( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.0f, 7.5f );
		camera.zoom = 0.5f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Vec2( 0.0f, -1.0f );
			BodyId groundId = CreateBody( m_worldId, bodyDef );
			Polygon box = MakeBox( 20.0f, 1.0f );
			CreatePolygonShape( groundId, DefaultShapeDef(), box );
		}

		float h = 0.25f;
		float r = 0.1f * h;
		Polygon boxShape = MakeRoundedBox( h - r, h - r, r );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.3f;

		float offset = 0.4f * h;
		float dx = 10.0f * h;
		float xroot = -0.5f * dx * ( k_columns - 1.0f );

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.1f * MathF.PI;
		jointDef.upperAngle = 0.2f * MathF.PI;
		jointDef.enableSpring = true;
		jointDef.hertz = 0.5f;
		jointDef.dampingRatio = 0.5f;
		jointDef.@base.localFrameA.p = new Vec2( h, h );
		jointDef.@base.localFrameB.p = new Vec2( offset, -h );
		jointDef.@base.drawScale = 2.0f;

		int bodyIndex = 0;
		for ( int j = 0; j < k_columns; ++j )
		{
			float x = xroot + j * dx;
			BodyId prevBodyId = nullBodyId;

			for ( int i = 0; i < k_rows; ++i )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = new Vec2( x + offset * i, h + 2.0f * h * i );

				// this tests the deterministic cosine and sine functions
				bodyDef.rotation = MakeRot( 0.1f * i - 1.0f );

				BodyId bodyId = CreateBody( m_worldId, bodyDef );

				if ( ( i & 1 ) == 0 )
				{
					prevBodyId = bodyId;
				}
				else
				{
					jointDef.@base.bodyIdA = prevBodyId;
					jointDef.@base.bodyIdB = bodyId;
					CreateRevoluteJoint( m_worldId, jointDef );
					prevBodyId = nullBodyId;
				}

				CreatePolygonShape( bodyId, shapeDef, boxShape );
				m_bodies[bodyIndex++] = bodyId;
			}
		}
	}

	public override void Step( Settings settings, ref DebugDraw draw )
	{
		base.Step( settings, ref draw );

		if ( m_sleepStep < 0 )
		{
			bool done = true;
			foreach ( BodyId bodyId in m_bodies )
			{
				if ( Body_IsAwake( bodyId ) )
				{
					done = false;
					break;
				}
			}

			if ( done )
			{
				m_sleepStep = stepCount - 1;
				uint hash = HashInit;
				foreach ( BodyId bodyId in m_bodies )
				{
					Transform xf = Body_GetTransform( bodyId );
					hash = Hash( hash, System.Runtime.InteropServices.MemoryMarshal.AsBytes( new ReadOnlySpan<Transform>( in xf ) ) );
				}

				m_hash = hash;
			}
		}
	}

	public override void UpdateGui()
	{
		ImGui.TextWrapped( "With 60 Hz and 4 sub-steps this matches the upstream determinism test: hash 0x465F3DF4, sleep step 299" );
		if ( m_sleepStep >= 0 )
		{
			ImGui.Text( $"sleep step = {m_sleepStep}, hash = 0x{m_hash:X8}" );
		}
		else
		{
			ImGui.Text( $"step = {stepCount}" );
		}
	}
}

// Falling mix of circles, capsules and polygons
public sealed class ShapeRain : Sample
{
	private readonly Random m_random = new( 1234 );
	private int m_frame;

	public ShapeRain( Settings settings, Camera camera ) : base( settings, camera )
	{
		camera.center = new Vec2( 0.0f, 15.0f );
		camera.zoom = 0.75f;

		BodyId groundId = CreateBody( m_worldId, DefaultBodyDef() );
		ShapeDef shapeDef = DefaultShapeDef();
		CreateSegmentShape( groundId, shapeDef, new Segment { point1 = new Vec2( -20.0f, 0.0f ), point2 = new Vec2( 20.0f, 0.0f ) } );
		CreateSegmentShape( groundId, shapeDef, new Segment { point1 = new Vec2( -20.0f, 0.0f ), point2 = new Vec2( -25.0f, 30.0f ) } );
		CreateSegmentShape( groundId, shapeDef, new Segment { point1 = new Vec2( 20.0f, 0.0f ), point2 = new Vec2( 25.0f, 30.0f ) } );
	}

	public override void Step( Settings settings, ref DebugDraw draw )
	{
		if ( settings.pause == false && m_frame++ % 4 == 0 && stepCount < 2000 )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Vec2( -15.0f + 30.0f * m_random.NextSingle(), 35.0f );
			bodyDef.rotation = MakeRot( MathF.PI * m_random.NextSingle() );
			BodyId bodyId = CreateBody( m_worldId, bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			switch ( m_random.Next( 3 ) )
			{
				case 0:
					CreateCircleShape( bodyId, shapeDef, new Circle { center = Vec2_zero, radius = 0.3f + 0.3f * m_random.NextSingle() } );
					break;
				case 1:
					CreateCapsuleShape( bodyId, shapeDef,
										new Capsule { center1 = new Vec2( -0.4f, 0.0f ), center2 = new Vec2( 0.4f, 0.0f ), radius = 0.25f } );
					break;
				default:
					CreatePolygonShape( bodyId, shapeDef, MakeBox( 0.25f + 0.4f * m_random.NextSingle(), 0.25f + 0.4f * m_random.NextSingle() ) );
					break;
			}
		}

		base.Step( settings, ref draw );
	}
}
