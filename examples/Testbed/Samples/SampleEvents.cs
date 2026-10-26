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

public static partial class Helpers_sample_events
{
	// This shows how to process body events.






	// This sample shows how to break joints when the internal reaction force becomes large. Instead of polling, this uses events.



	// This sample shows how to track a contact across time steps. This can be used
	// for things like rolling or sliding sounds.






	// This shows how to create a projectile that explodes on impact
}

public class SensorFunnel : Sample
{
	public const int e_donut = 1;
	public const int e_human = 2;
	public const int e_count = 32;

	public SensorFunnel( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 1.333f;
		}

		m_context.debugDraw.drawJoints = false;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			// Vec2 points[] = {
			//{42.333, 44.979},	new( 177.271, 44.979 ),	new( 177.271, 100.542 ), new( 142.875, 121.708 ), new( 177.271, 121.708 ),
			//{177.271, 171.979}, new( 142.875, 193.146 ), new( 177.271, 193.146 ), new( 177.271, 222.250 ), new( 124.354, 261.938 ),
			//{124.354, 293.688}, new( 95.250, 293.688 ),	new( 95.250, 261.938 ),	new( 42.333, 222.250 ),	new( 42.333, 193.146 ),
			//{76.729, 193.146},	new( 42.333, 171.979 ),	new( 42.333, 121.708 ),	new( 76.729, 121.708 ),	new( 42.333, 100.542 ),
			//};

			Vec2[] points = {
				new( -16.8672504f, 31.088623f ), new( 16.8672485f, 31.088623f ), new( 16.8672485f, 17.1978741f ), new( 8.26824951f, 11.906374f ), new( 16.8672485f, 11.906374f ), new( 16.8672485f, -0.661376953f ), new( 8.26824951f, -5.953125f ), new( 16.8672485f, -5.953125f ), new( 16.8672485f, -13.229126f ), new( 3.63799858f, -23.151123f ), new( 3.63799858f, -31.088623f ),	new( -3.63800049f, -31.088623f ), new( -3.63800049f, -23.151123f ), new( -16.8672504f, -13.229126f ),	new( -16.8672504f, -5.953125f ), new( -8.26825142f, -5.953125f ), new( -16.8672504f, -0.661376953f ), new( -16.8672504f, 11.906374f ), new( -8.26825142f, 11.906374f ), new( -16.8672504f, 17.1978741f ),
			};

			int count = points.Length;

			// float scale = 0.25f;
			// Vec2 lower = new Vec2( float.MaxValue, float.MaxValue );
			// Vec2 upper = new Vec2( -float.MaxValue, -float.MaxValue );
			// for (int i = 0; i < count; ++i)
			//{
			//	points[i].x = scale * points[i].x;
			//	points[i].y = -scale * points[i].y;

			//	lower = Min(lower, points[i]);
			//	upper = Max(upper, points[i]);
			//}

			// Vec2 center = MulSV(0.5f, Add(lower, upper));
			// for (int i = 0; i < count; ++i)
			//{
			//	points[i] = Sub(points[i], center);
			// }

			// for (int i = 0; i < count / 2; ++i)
			//{
			//	Vec2 temp = points[i];
			//	points[i] = points[count - 1 - i];
			//	points[count - 1 - i] = temp;
			// }

			// Console.Write( Fmt( "{" ) );
			// for (int i = 0; i < count; ++i)
			//{
			//	Console.Write( Fmt( "{%.9g, %.9g}, ", points[i].x, points[i].y ) );
			// }
			// Console.Write( Fmt( "};\n" ) );

			SurfaceMaterial material = default;
			material.friction = 0.2f;

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;
			chainDef.materials = [material];
			chainDef.materialCount = 1;
			CreateChain( groundId, ref chainDef );

			float sign = 1.0f;
			float y = 14.0f;
			for ( int i = 0; i < 3; ++i )
			{
				bodyDef.position = new Pos { x = 0.0f, y = y };
				bodyDef.type = BodyType.dynamicBody;

				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				Polygon box = MakeBox( 6.0f, 0.5f );
				ShapeDef shapeDef = DefaultShapeDef();
				shapeDef.material.friction = 0.1f;
				shapeDef.material.restitution = 1.0f;
				shapeDef.density = 1.0f;

				CreatePolygonShape( bodyId, ref shapeDef, ref box );

				RevoluteJointDef revoluteDef = DefaultRevoluteJointDef();
				revoluteDef.@base.bodyIdA = groundId;
				revoluteDef.@base.bodyIdB = bodyId;
				revoluteDef.@base.localFrameA.p = ToVec2( bodyDef.position );
				revoluteDef.@base.localFrameB.p = Vec2_zero;
				revoluteDef.maxMotorTorque = 200.0f;
				revoluteDef.motorSpeed = 2.0f * sign;
				revoluteDef.enableMotor = true;

				CreateRevoluteJoint( m_worldId, ref revoluteDef );

				y -= 14.0f;
				sign = -sign;
			}

			{
				Polygon box = MakeOffsetBox( 4.0f, 1.0f, new( 0.0f, -30.5f ), Rot_identity );
				ShapeDef shapeDef = DefaultShapeDef();
				shapeDef.isSensor = true;
				shapeDef.enableSensorEvents = true;

				CreatePolygonShape( groundId, ref shapeDef, ref box );
			}
		}

		m_wait = 0.5f;
		m_side = -15.0f;
		m_type = e_human;

		for ( int i = 0; i < e_count; ++i )
		{
			m_isSpawned[i] = false;
		}

		Array.Clear( m_humans );

		CreateElement();
	}

	public void CreateElement(  )
	{
		int index = -1;
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_isSpawned[i] == false )
			{
				index = i;
				break;
			}
		}

		if ( index == -1 )
		{
			return;
		}

		Pos center = new Pos { x = m_side, y = 29.5f };

		if ( m_type == e_donut )
		{
			Donut? donut = m_donuts[index];
			donut.Create( m_worldId, center, 1.0f, 0, true, donut );
		}
		else
		{
			ref Human human = ref m_humans[index];
			float scale = 2.0f;
			float jointFriction = 0.05f;
			float jointHertz = 6.0f;
			float jointDamping = 0.5f;
			bool colorize = true;
			CreateHuman( ref human, m_worldId, center, scale, jointFriction, jointHertz, jointDamping, index + 1, index, colorize );
			Human_EnableSensorEvents( ref human, true );
		}

		m_isSpawned[index] = true;
		m_side = -m_side;
	}

	public void DestroyElement( int index )
	{
		if ( m_type == e_donut )
		{
			Donut? donut = m_donuts[index];
			donut.Destroy();
		}
		else
		{
			ref Human human = ref m_humans[index];
			DestroyHuman( ref human );
		}

		m_isSpawned[index] = false;
	}

	public void Clear(  )
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_isSpawned[i] == true )
			{
				if ( m_type == e_donut )
				{
					m_donuts[i].Destroy();
				}
				else
				{
					DestroyHuman( ref m_humans[i] );
				}

				m_isSpawned[i] = false;
			}
		}
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.RadioButton( "donut", m_type == e_donut ) )
		{
			Clear();
			m_type = e_donut;
		}

		if ( ImGui.RadioButton( "human", m_type == e_human ) )
		{
			Clear();
			m_type = e_human;
		}

		return true;
	}

	public override void Step(  )
	{
		if ( m_stepCount == 832 )
		{
			m_stepCount += 0;
		}

		base.Step();

		// Discover rings that touch the bottom sensor
		bool[] deferredDestruction = new bool[e_count];
		SensorEvents sensorEvents = World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			SensorBeginTouchEvent @event = sensorEvents.beginEvents[i];
			ShapeId visitorId = @event.visitorShapeId;
			BodyId bodyId = Shape_GetBody( visitorId );

			if ( m_type == e_donut )
			{
				Donut? donut = (Donut?)Body_GetUserData( bodyId );
				if ( donut != null )
				{
					int index = Array.IndexOf( m_donuts, donut );
					Debug.Assert( 0 <= index && index < e_count );

					// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
					deferredDestruction[index] = true;
				}
			}
			else
			{
				object? humanIndex = Body_GetUserData( bodyId );
				if ( humanIndex != null )
				{
					int index = (int)humanIndex;
					Debug.Assert( 0 <= index && index < e_count );

					// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
					deferredDestruction[index] = true;
				}
			}
		}

		// todo destroy mouse joint if necessary

		// Safely destroy rings that hit the bottom sensor
		for ( int i = 0; i < e_count; ++i )
		{
			if ( deferredDestruction[i] )
			{
				DestroyElement( i );
			}
		}

		if ( m_context.hertz > 0.0f && m_context.pause == false )
		{
			m_wait -= 1.0f / m_context.hertz;
			if ( m_wait < 0.0f )
			{
				CreateElement();
				m_wait += 0.5f;
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SensorFunnel( context );
	}

	public Human[] m_humans = new Human[e_count];
	public Donut[] m_donuts = NewArray<Donut>( e_count );
	public bool[] m_isSpawned = new bool[e_count];
	public int m_type;
	public float m_wait;
	public float m_side;
}

public class SensorBookend : Sample
{
	public SensorBookend( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 7.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			Segment groundSegment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			groundSegment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( -10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			groundSegment = new Segment { point1 = new( 10.0f, 0.0f ), point2 = new( 10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			m_isVisiting1 = false;
			m_isVisiting2 = false;
			m_sensorsOverlapCount = 0;
		}

		CreateSensor1();
		CreateSensor2();
		CreateVisitor();
	}

	public void CreateSensor1(  )
	{
		BodyDef bodyDef = DefaultBodyDef();

		bodyDef.position = new Pos { x = -2.0f, y = 1.0f };
		m_sensorBodyId1 = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;

		Polygon box = MakeSquare( 1.0f );
		m_sensorShapeId1 = CreatePolygonShape( m_sensorBodyId1, ref shapeDef, ref box );
	}

	public void CreateSensor2(  )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 2.0f, y = 1.0f };
		m_sensorBodyId2 = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;

		Polygon box = MakeRoundedBox( 0.5f, 0.5f, 0.5f );
		m_sensorShapeId2 = CreatePolygonShape( m_sensorBodyId2, ref shapeDef, ref box );

		// Solid middle
		shapeDef.isSensor = false;
		shapeDef.enableSensorEvents = false;
		box = MakeSquare( 0.5f );
		CreatePolygonShape( m_sensorBodyId2, ref shapeDef, ref box );
	}

	public void CreateVisitor(  )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.position = new Pos { x = -4.0f, y = 1.0f };
		bodyDef.type = BodyType.dynamicBody;

		m_visitorBodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		m_visitorShapeId = CreateCircleShape( m_visitorBodyId, ref shapeDef, ref circle );
	}

	public override bool DrawControls(  )
	{
		if ( m_visitorBodyId.IsNull )
		{
			if ( ImGui.Button( "create visitor" ) )
			{
				CreateVisitor();
			}
		}
		else
		{
			if ( ImGui.Button( "destroy visitor" ) )
			{
				DestroyBody( m_visitorBodyId );
				m_visitorBodyId = nullBodyId;
				// Retain m_visitorShapeId for end events.
			}
			else
			{
				bool enabledEvents = Shape_AreSensorEventsEnabled( m_visitorShapeId );
				if ( ImGui.Checkbox( "visitor events", ref enabledEvents ) )
				{
					Shape_EnableSensorEvents( m_visitorShapeId, enabledEvents );
				}

				bool enabledBody = Body_IsEnabled( m_visitorBodyId );
				if ( ImGui.Checkbox( "enable visitor body", ref enabledBody ) )
				{
					if ( enabledBody )
					{
						Body_Enable( m_visitorBodyId );
					}
					else
					{
						Body_Disable( m_visitorBodyId );
					}
				}
			}
		}

		ImGui.Separator();

		if ( m_sensorBodyId1.IsNull )
		{
			if ( ImGui.Button( "create sensor1" ) )
			{
				CreateSensor1();
			}
		}
		else
		{
			if ( ImGui.Button( "destroy sensor1" ) )
			{
				DestroyBody( m_sensorBodyId1 );
				m_sensorBodyId1 = nullBodyId;
				// Retain m_sensorShapeId1 for end events.
			}
			else
			{
				bool enabledEvents = Shape_AreSensorEventsEnabled( m_sensorShapeId1 );
				if ( ImGui.Checkbox( "sensor 1 events", ref enabledEvents ) )
				{
					Shape_EnableSensorEvents( m_sensorShapeId1, enabledEvents );
				}

				bool enabledBody = Body_IsEnabled( m_sensorBodyId1 );
				if ( ImGui.Checkbox( "enable sensor1 body", ref enabledBody ) )
				{
					if ( enabledBody )
					{
						Body_Enable( m_sensorBodyId1 );
					}
					else
					{
						Body_Disable( m_sensorBodyId1 );
					}
				}
			}
		}

		ImGui.Separator();

		if ( m_sensorBodyId2.IsNull )
		{
			if ( ImGui.Button( "create sensor2" ) )
			{
				CreateSensor2();
			}
		}
		else
		{
			if ( ImGui.Button( "destroy sensor2" ) )
			{
				DestroyBody( m_sensorBodyId2 );
				m_sensorBodyId2 = nullBodyId;
				// Retain m_sensorShapeId2 for end events.
			}
			else
			{
				bool enabledEvents = Shape_AreSensorEventsEnabled( m_sensorShapeId2 );
				if ( ImGui.Checkbox( "sensor2 events", ref enabledEvents ) )
				{
					Shape_EnableSensorEvents( m_sensorShapeId2, enabledEvents );
				}

				bool enabledBody = Body_IsEnabled( m_sensorBodyId2 );
				if ( ImGui.Checkbox( "enable sensor2 body", ref enabledBody ) )
				{
					if ( enabledBody )
					{
						Body_Enable( m_sensorBodyId2 );
					}
					else
					{
						Body_Disable( m_sensorBodyId2 );
					}
				}
			}
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		SensorEvents sensorEvents = World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			SensorBeginTouchEvent @event = sensorEvents.beginEvents[i];

			if ( ( @event.sensorShapeId ).Equals( m_sensorShapeId1 ) )
			{
				if ( ( @event.visitorShapeId ).Equals( m_visitorShapeId ) )
				{
					Debug.Assert( m_isVisiting1 == false );
					m_isVisiting1 = true;
				}
				else
				{
					Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorShapeId2 ) );
					m_sensorsOverlapCount += 1;
				}
			}
			else
			{
				Debug.Assert( ( @event.sensorShapeId ).Equals( m_sensorShapeId2 ) );

				if ( ( @event.visitorShapeId ).Equals( m_visitorShapeId ) )
				{
					Debug.Assert( m_isVisiting2 == false );
					m_isVisiting2 = true;
				}
				else
				{
					Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorShapeId1 ) );
					m_sensorsOverlapCount += 1;
				}
			}
		}

		Debug.Assert( m_sensorsOverlapCount == 0 || m_sensorsOverlapCount == 2 );

		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			SensorEndTouchEvent @event = sensorEvents.endEvents[i];

			if ( ( @event.sensorShapeId ).Equals( m_sensorShapeId1 ) )
			{
				if ( ( @event.visitorShapeId ).Equals( m_visitorShapeId ) )
				{
					Debug.Assert( m_isVisiting1 == true );
					m_isVisiting1 = false;
				}
				else
				{
					Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorShapeId2 ) );
					m_sensorsOverlapCount -= 1;
				}
			}
			else
			{
				Debug.Assert( ( @event.sensorShapeId ).Equals( m_sensorShapeId2 ) );

				if ( ( @event.visitorShapeId ).Equals( m_visitorShapeId ) )
				{
					Debug.Assert( m_isVisiting2 == true );
					m_isVisiting2 = false;
				}
				else
				{
					Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorShapeId1 ) );
					m_sensorsOverlapCount -= 1;
				}
			}
		}

		Debug.Assert( m_sensorsOverlapCount == 0 || m_sensorsOverlapCount == 2 );

		// Nullify invalid shape ids after end events are processed.
		if ( Shape_IsValid( m_visitorShapeId ) == false )
		{
			m_visitorShapeId = nullShapeId;
		}

		if ( Shape_IsValid( m_sensorShapeId1 ) == false )
		{
			m_sensorShapeId1 = nullShapeId;
		}

		if ( Shape_IsValid( m_sensorShapeId2 ) == false )
		{
			m_sensorShapeId2 = nullShapeId;
		}

		DrawScreenTextLine( "visiting 1 == %s", m_isVisiting1 ? "true" : "false" );
		DrawScreenTextLine( "visiting 2 == %s", m_isVisiting2 ? "true" : "false" );
		DrawScreenTextLine( "sensors overlap count == %d", m_sensorsOverlapCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new SensorBookend( context );
	}

	public BodyId m_sensorBodyId1;
	public ShapeId m_sensorShapeId1;

	public BodyId m_sensorBodyId2;
	public ShapeId m_sensorShapeId2;

	public BodyId m_visitorBodyId;
	public ShapeId m_visitorShapeId;

	public bool m_isVisiting1;
	public bool m_isVisiting2;
	public int m_sensorsOverlapCount;
}

public class FootSensor : Sample
{
	public const ulong GROUND = 0x00000001;
	public const ulong PLAYER = 0x00000002;
	public const ulong FOOT = 0x00000004;
	public const ulong ALL_BITS = ( ~0u );

	public FootSensor( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 7.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Vec2[] points = new Vec2[18];
			float x = 9.0f;
			for ( int i = 0; i < 18; ++i )
			{
				points[i] = new( x, 0.0f );
				x -= 1.0f;
			}

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = 18;
			chainDef.ghost1 = new Vec2( 10.0f, 0.0f );
			chainDef.ghost2 = new Vec2( -9.0f, 0.0f );
			chainDef.filter.categoryBits = GROUND;
			chainDef.filter.maskBits = FOOT | PLAYER;
			chainDef.isLoop = false;
			chainDef.enableSensorEvents = true;

			CreateChain( groundId, ref chainDef );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
			m_playerId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = PLAYER;
			shapeDef.filter.maskBits = GROUND;
			shapeDef.material.friction = 0.3f;
			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.5f };
			CreateCapsuleShape( m_playerId, ref shapeDef, ref capsule );

			Polygon box = MakeOffsetBox( 0.5f, 0.25f, new( 0.0f, -1.0f ), Rot_identity );
			shapeDef.filter.categoryBits = FOOT;
			shapeDef.filter.maskBits = GROUND;
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			m_sensorId = CreatePolygonShape( m_playerId, ref shapeDef, ref box );
		}

		m_overlapCount = 0;
	}

	public override void Step(  )
	{
		if ( glfwGetKey( m_context.window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			Body_ApplyForceToCenter( m_playerId, new( -50.0f, 0.0f ), true );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			Body_ApplyForceToCenter( m_playerId, new( 50.0f, 0.0f ), true );
		}

		base.Step();

		SensorEvents sensorEvents = World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			SensorBeginTouchEvent @event = sensorEvents.beginEvents[i];

			Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorId ) == false );

			if ( ( @event.sensorShapeId ).Equals( m_sensorId ) )
			{
				m_overlapCount += 1;
			}
		}

		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			SensorEndTouchEvent @event = sensorEvents.endEvents[i];

			Debug.Assert( ( @event.visitorShapeId ).Equals( m_sensorId ) == false );

			if ( ( @event.sensorShapeId ).Equals( m_sensorId ) )
			{
				m_overlapCount -= 1;
			}
		}

		DrawScreenTextLine( "count == %d", m_overlapCount );

		int capacity = Shape_GetSensorCapacity( m_sensorId );
		m_visitorIds.Clear();
		m_visitorIds.resize( capacity );
		int count = Shape_GetSensorData( m_sensorId, m_visitorIds.AsSpan(), capacity );
		for ( int i = 0; i < count; ++i )
		{
			ShapeId shapeId = m_visitorIds[i];
			AABB aabb = Shape_GetBounds( shapeId );
			Vec2 point = AABB_Center( aabb );
			DrawPoint( m_draw, ToPos( point ), 10.0f, HexColor.colorWhite );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new FootSensor( context );
	}

	public BodyId m_playerId;
	public ShapeId m_sensorId;
	public List<ShapeId> m_visitorIds = new();
	public int m_overlapCount;
}

public class BodyUserData
{
	public int index;
}

public class ContactEvent : Sample
{
	public const int e_count = 20;

	public ContactEvent( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 25.0f * 1.75f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Vec2[] points = { new( 40.0f, -40.0f ), new( -40.0f, -40.0f ), new( -40.0f, 40.0f ), new( 40.0f, 40.0f ) };

			ChainDef chainDef = DefaultChainDef();
			chainDef.pointCount = 4;
			chainDef.points = points;
			chainDef.isLoop = true;

			CreateChain( groundId, ref chainDef );
		}

		// Player
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.gravityScale = 0.0f;
			bodyDef.linearDamping = 0.5f;
			bodyDef.angularDamping = 0.5f;
			bodyDef.isBullet = true;
			m_playerId = CreateBody( m_worldId, ref bodyDef );

			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 1.0f };
			ShapeDef shapeDef = DefaultShapeDef();

			// Enable contact events for the player shape
			shapeDef.enableContactEvents = true;

			m_coreShapeId = CreateCircleShape( m_playerId, ref shapeDef, ref circle );
		}

		for ( int i = 0; i < e_count; ++i )
		{
			m_debrisIds[i] = nullBodyId;
			m_bodyUserData[i].index = i;
		}

		m_wait = 0.5f;
		m_force = 200.0f;
	}

	public void SpawnDebris(  )
	{
		int index = -1;
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_debrisIds[i].IsNull )
			{
				index = i;
				break;
			}
		}

		if ( index == -1 )
		{
			return;
		}

		// Debris
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = RandomFloatRange( -38.0f, 38.0f ), y = RandomFloatRange( -38.0f, 38.0f ) };
		bodyDef.rotation = MakeRot( RandomFloatRange( -Pi, Pi ) );
		bodyDef.linearVelocity = new Vec2( RandomFloatRange( -5.0f, 5.0f ), RandomFloatRange( -5.0f, 5.0f ) );
		bodyDef.angularVelocity = RandomFloatRange( -1.0f, 1.0f );
		bodyDef.gravityScale = 0.0f;
		bodyDef.userData = m_bodyUserData[index];
		m_debrisIds[index] = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.restitution = 0.8f;

		// No events when debris hits debris
		shapeDef.enableContactEvents = false;

		if ( ( index + 1 ) % 3 == 0 )
		{
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( m_debrisIds[index], ref shapeDef, ref circle );
		}
		else if ( ( index + 1 ) % 2 == 0 )
		{
			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.25f ), center2 = new( 0.0f, 0.25f ), radius = 0.25f };
			CreateCapsuleShape( m_debrisIds[index], ref shapeDef, ref capsule );
		}
		else
		{
			Polygon box = MakeBox( 0.4f, 0.6f );
			CreatePolygonShape( m_debrisIds[index], ref shapeDef, ref box );
		}
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderFloat( "force", ref m_force, 100.0f, 500.0f, "%.1f" );
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "move using WASD" );

		Pos position = Body_GetPosition( m_playerId );

		if ( glfwGetKey( m_context.window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			Body_ApplyForce( m_playerId, new( -m_force, 0.0f ), position, true );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			Body_ApplyForce( m_playerId, new( m_force, 0.0f ), position, true );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_W ) == GLFW_PRESS )
		{
			Body_ApplyForce( m_playerId, new( 0.0f, m_force ), position, true );
		}

		if ( glfwGetKey( m_context.window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			Body_ApplyForce( m_playerId, new( 0.0f, -m_force ), position, true );
		}

		base.Step();

		// Discover rings that touch the bottom sensor
		int[] debrisToAttach = new int[e_count];
		ShapeId[] shapesToDestroy = { nullShapeId };
		int attachCount = 0;
		int destroyCount = 0;

		List<ContactData> contactData = default;

		// Process contact begin touch events.
		ContactEvents contactEvents = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < contactEvents.beginCount; ++i )
		{
			ContactBeginTouchEvent @event = contactEvents.beginEvents[i];
			BodyId bodyIdA = Shape_GetBody( @event.shapeIdA );
			BodyId bodyIdB = Shape_GetBody( @event.shapeIdB );

			// The begin touch events have the contact manifolds, but the impulses are zero. This is because the manifolds
			// are gathered before the contact solver is run.

			// We can get the final contact data from the shapes. The manifold is shared by the two shapes, so we just need the
			// contact data from one of the shapes. Choose the one with the smallest number of contacts.

			int capacityA = Shape_GetContactCapacity( @event.shapeIdA );
			int capacityB = Shape_GetContactCapacity( @event.shapeIdB );

			if ( capacityA < capacityB )
			{
				contactData.resize( capacityA );

				// The count may be less than the capacity
				int countA = Shape_GetContactData( @event.shapeIdA, contactData.AsSpan(), capacityA );
				Debug.Assert( countA >= 1 );

				for ( int j = 0; j < countA; ++j )
				{
					ShapeId idA = contactData[j].shapeIdA;
					ShapeId idB = contactData[j].shapeIdB;
					if ( ( idA ).Equals( @event.shapeIdB ) || ( idB ).Equals( @event.shapeIdB ) )
					{
						Debug.Assert( ( idA ).Equals( @event.shapeIdA ) || ( idB ).Equals( @event.shapeIdA ) );

						Manifold manifold = contactData[j].manifold;
						Vec2 normal = manifold.normal;
						Debug.Assert( AbsFloat( Length( normal ) - 1.0f ) < 4.0f * float.Epsilon );
						BodyId bidA = Shape_GetBody( idA );

						for ( int k = 0; k < manifold.pointCount; ++k )
						{
							ManifoldPoint point = manifold.points[k];
							Pos posA = Body_GetWorldCenter( bidA );
							Pos p1 = posA + point.anchorA;
							Pos p2 = p1 + point.totalNormalImpulse * normal;

							DrawLine( m_draw, p1, p2, HexColor.colorBlueViolet );
							DrawPoint( m_draw, p1, 10.0f, HexColor.colorWhite );
						}
					}
				}
			}
			else
			{
				contactData.resize( capacityB );

				// The count may be less than the capacity
				int countB = Shape_GetContactData( @event.shapeIdB, contactData.AsSpan(), capacityB );
				Debug.Assert( countB >= 1 );

				for ( int j = 0; j < countB; ++j )
				{
					ShapeId idA = contactData[j].shapeIdA;
					ShapeId idB = contactData[j].shapeIdB;

					if ( ( idA ).Equals( @event.shapeIdA ) || ( idB ).Equals( @event.shapeIdA ) )
					{
						Debug.Assert( ( idA ).Equals( @event.shapeIdB ) || ( idB ).Equals( @event.shapeIdB ) );

						Manifold manifold = contactData[j].manifold;
						Vec2 normal = manifold.normal;
						Debug.Assert( AbsFloat( Length( normal ) - 1.0f ) < 4.0f * float.Epsilon );
						BodyId bidA = Shape_GetBody( idA );

						for ( int k = 0; k < manifold.pointCount; ++k )
						{
							ManifoldPoint point = manifold.points[k];
							Pos posA = Body_GetWorldCenter( bidA );
							Pos p1 = posA + point.anchorA;
							Pos p2 = p1 + point.totalNormalImpulse * normal;

							DrawLine( m_draw, p1, p2, HexColor.colorYellowGreen );
							DrawPoint( m_draw, p1, 10.0f, HexColor.colorWhite );
						}
					}
				}
			}

			if ( ( bodyIdA ).Equals( m_playerId ) )
			{
				BodyUserData? userDataB = (BodyUserData?)( Body_GetUserData( bodyIdB ) );
				if ( userDataB == null )
				{
					if ( ( @event.shapeIdA ).Equals( m_coreShapeId ) == false && destroyCount < e_count )
					{
						// player non-core shape hit the wall

						bool found = false;
						for ( int j = 0; j < destroyCount; ++j )
						{
							if ( ( @event.shapeIdA ).Equals( shapesToDestroy[j] ) )
							{
								found = true;
								break;
							}
						}

						// avoid double deletion
						if ( found == false )
						{
							shapesToDestroy[destroyCount] = @event.shapeIdA;
							destroyCount += 1;
						}
					}
				}
				else if ( attachCount < e_count )
				{
					debrisToAttach[attachCount] = userDataB.index;
					attachCount += 1;
				}
			}
			else
			{
				// Only expect events for the player
				Debug.Assert( ( bodyIdB ).Equals( m_playerId ) );
				BodyUserData? userDataA = (BodyUserData?)( Body_GetUserData( bodyIdA ) );
				if ( userDataA == null )
				{
					if ( ( @event.shapeIdB ).Equals( m_coreShapeId ) == false && destroyCount < e_count )
					{
						// player non-core shape hit the wall

						bool found = false;
						for ( int j = 0; j < destroyCount; ++j )
						{
							if ( ( @event.shapeIdB ).Equals( shapesToDestroy[j] ) )
							{
								found = true;
								break;
							}
						}

						// avoid double deletion
						if ( found == false )
						{
							shapesToDestroy[destroyCount] = @event.shapeIdB;
							destroyCount += 1;
						}
					}
				}
				else if ( attachCount < e_count )
				{
					debrisToAttach[attachCount] = userDataA.index;
					attachCount += 1;
				}
			}
		}

		// Attach debris to player body
		for ( int i = 0; i < attachCount; ++i )
		{
			int index = debrisToAttach[i];
			BodyId debrisId = m_debrisIds[index];
			if ( Body_IsValid( debrisId ) == false )
			{
				continue;
			}

			WorldTransform playerTransform = Body_GetTransform( m_playerId );
			WorldTransform debrisTransform = Body_GetTransform( debrisId );
			Transform relativeTransform = InvMulWorldTransforms( playerTransform, debrisTransform );

			int shapeCount = Body_GetShapeCount( debrisId );
			if ( shapeCount == 0 )
			{
				continue;
			}

			ShapeId shapeId = default;
			Body_GetShapes( debrisId, new Span<ShapeId>( ref shapeId ), 1 );

			ShapeType type = Shape_GetType( shapeId );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.enableContactEvents = true;

			switch ( type )
			{
				case ShapeType.circleShape:
				{
					Circle circle = Shape_GetCircle( shapeId );
					circle.center = TransformPoint( relativeTransform, circle.center );

					CreateCircleShape( m_playerId, ref shapeDef, ref circle );
				}
				break;

				case ShapeType.capsuleShape:
				{
					Capsule capsule = Shape_GetCapsule( shapeId );
					capsule.center1 = TransformPoint( relativeTransform, capsule.center1 );
					capsule.center2 = TransformPoint( relativeTransform, capsule.center2 );

					CreateCapsuleShape( m_playerId, ref shapeDef, ref capsule );
				}
				break;

				case ShapeType.polygonShape:
				{
					Polygon originalPolygon = Shape_GetPolygon( shapeId );
					Polygon polygon = TransformPolygon( relativeTransform, ref originalPolygon );

					CreatePolygonShape( m_playerId, ref shapeDef, ref polygon );
				}
				break;

				default:
					Debug.Assert( false );
					break;
			}

			DestroyBody( debrisId );
			m_debrisIds[index] = nullBodyId;
		}

		for ( int i = 0; i < destroyCount; ++i )
		{
			bool updateMass = false;
			DestroyShape( shapesToDestroy[i], updateMass );
		}

		if ( destroyCount > 0 )
		{
			// Update mass just once
			Body_UpdateMassFromShapes( m_playerId );
		}

		if ( m_context.hertz > 0.0f && m_context.pause == false )
		{
			m_wait -= 1.0f / m_context.hertz;
			if ( m_wait < 0.0f )
			{
				SpawnDebris();
				m_wait += 0.5f;
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new ContactEvent( context );
	}

	public BodyId m_playerId;
	public ShapeId m_coreShapeId;
	public BodyId[] m_debrisIds = new BodyId[e_count];
	public BodyUserData[] m_bodyUserData = NewArray<BodyUserData>( e_count );
	public float m_force;
	public float m_wait;
}

public class BodyMove : Sample
{
	public const int e_count = 50;

	public BodyMove( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 2.0f, 8.0f );
			m_context.camera.zoom = 25.0f * 0.55f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.material.friction = 0.1f;

			Polygon box = MakeOffsetBox( 12.0f, 0.1f, new( -10.0f, -0.1f ), MakeRot( -0.15f * Pi ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 12.0f, 0.1f, new( 10.0f, -0.1f ), MakeRot( 0.15f * Pi ) );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			shapeDef.material.restitution = 0.8f;

			box = MakeOffsetBox( 0.1f, 10.0f, new( 19.9f, 10.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 0.1f, 10.0f, new( -19.9f, 10.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );

			box = MakeOffsetBox( 20.0f, 0.1f, new( 0.0f, 20.1f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_sleepCount = 0;
		m_count = 0;

		m_explosionPosition = new Pos { x = 0.0f, y = -5.0f };
		m_explosionRadius = 10.0f;
		m_explosionMagnitude = 10.0f;
	}

	public void CreateBodies(  )
	{
		Capsule capsule = new Capsule { center1 = new( -0.25f, 0.0f ), center2 = new( 0.25f, 0.0f ), radius = 0.25f };
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.35f };
		Polygon square = MakeSquare( 0.35f );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		ShapeDef shapeDef = DefaultShapeDef();

		float x = -5.0f, y = 10.0f;
		for ( int i = 0; i < 10 && m_count < e_count; ++i )
		{
			bodyDef.position = new Pos { x = x, y = y };
			bodyDef.isBullet = ( m_count % 12 == 0 );
			bodyDef.userData = m_count;
			m_bodyIds[m_count] = CreateBody( m_worldId, ref bodyDef );
			m_sleeping[m_count] = false;

			int remainder = m_count % 4;
			if ( remainder == 0 )
			{
				CreateCapsuleShape( m_bodyIds[m_count], ref shapeDef, ref capsule );
			}
			else if ( remainder == 1 )
			{
				CreateCircleShape( m_bodyIds[m_count], ref shapeDef, ref circle );
			}
			else if ( remainder == 2 )
			{
				CreatePolygonShape( m_bodyIds[m_count], ref shapeDef, ref square );
			}
			else
			{
				Polygon poly = RandomPolygon( 0.75f );
				poly.radius = 0.1f;
				CreatePolygonShape( m_bodyIds[m_count], ref shapeDef, ref poly );
			}

			m_count += 1;
			x += 1.0f;
		}
	}

	public override bool DrawControls(  )
	{
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
		ImGui.SliderFloat( "Magnitude", ref m_explosionMagnitude, -20.0f, 20.0f, "%.1f" );
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( m_context.pause == false && ( m_stepCount & 15 ) == 15 && m_count < e_count )
		{
			CreateBodies();
		}

		base.Step();

		// Process body events
		BodyEvents events = World_GetBodyEvents( m_worldId );
		for ( int i = 0; i < events.moveCount; ++i )
		{
			ref readonly BodyMoveEvent @event = ref events.moveEvents[i];
			if ( @event.userData == null )
			{
				// The mouse joint body has no user data
				continue;
			}

			// draw the transform of every body that moved (not sleeping)
			DrawTransform( m_draw, @event.transform, 1.0f );

			WorldTransform transform = Body_GetTransform( @event.bodyId );
			Debug.Assert( transform.p.x == @event.transform.p.x );
			Debug.Assert( transform.p.y == @event.transform.p.y );
			Debug.Assert( transform.q.c == @event.transform.q.c );
			Debug.Assert( transform.q.s == @event.transform.q.s );

			// this shows a somewhat contrived way to track body sleeping
			int diff = (int)@event.userData!;
			ref bool sleeping = ref m_sleeping[diff];
			if ( @event.fellAsleep )
			{
				sleeping = true;
				m_sleepCount += 1;
			}
			else
			{
				if ( sleeping )
				{
					sleeping = false;
					m_sleepCount -= 1;
				}
			}
		}

		DrawCircle( m_draw, m_explosionPosition, m_explosionRadius, HexColor.colorAzure );

		DrawScreenTextLine( "sleep count: %d", m_sleepCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new BodyMove( context );
	}

	public BodyId[] m_bodyIds = new BodyId[e_count];
	public bool[] m_sleeping = new bool[e_count];
	public int m_count;
	public int m_sleepCount;
	public Pos m_explosionPosition;
	public float m_explosionRadius;
	public float m_explosionMagnitude;
}

public class SensorTypes : Sample
{
	public const ulong GROUND = 0x00000001;
	public const ulong SENSOR = 0x00000002;
	public const ulong DEFAULT = 0x00000004;
	public const ulong ALL_BITS = ( ~0u );

	public SensorTypes( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 3.0f );
			m_context.camera.zoom = 4.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "ground";

			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			// Enable sensor events, but filter them out as a test
			shapeDef.filter.categoryBits = GROUND;
			shapeDef.filter.maskBits = DEFAULT;
			shapeDef.enableSensorEvents = true;

			Segment groundSegment = new Segment { point1 = new( -6.0f, 0.0f ), point2 = new( 6.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			groundSegment = new Segment { point1 = new( -6.0f, 0.0f ), point2 = new( -6.0f, 4.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			groundSegment = new Segment { point1 = new( 6.0f, 0.0f ), point2 = new( 6.0f, 4.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "static sensor";
			bodyDef.type = BodyType.staticBody;
			bodyDef.position = new Pos { x = -3.0f, y = 0.8f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = SENSOR;
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			Polygon box = MakeSquare( 1.0f );
			m_staticSensorId = CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "kinematic sensor";
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 0.0f };
			bodyDef.linearVelocity = new Vec2( 0.0f, 1.0f );
			m_kinematicBodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = SENSOR;
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			Polygon box = MakeSquare( 1.0f );
			m_kinematicSensorId = CreatePolygonShape( m_kinematicBodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "dynamic sensor";
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 3.0f, y = 1.0f };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = SENSOR;
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			Polygon box = MakeSquare( 1.0f );
			m_dynamicSensorId = CreatePolygonShape( bodyId, ref shapeDef, ref box );

			// Add some real collision so the dynamic body is valid
			shapeDef.filter.categoryBits = DEFAULT;
			shapeDef.isSensor = false;
			shapeDef.enableSensorEvents = false;
			box = MakeSquare( 0.8f );
			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "ball_01";
			bodyDef.position = new Pos { x = -5.0f, y = 1.0f };
			bodyDef.type = BodyType.dynamicBody;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.filter.categoryBits = DEFAULT;
			shapeDef.filter.maskBits = GROUND | DEFAULT | SENSOR;
			shapeDef.enableSensorEvents = true;

			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public void PrintOverlaps( ShapeId sensorShapeId, string prefix )
	{
		string buffer = "";

		// Determine the necessary capacity
		int capacity = Shape_GetSensorCapacity( sensorShapeId );
		m_visitorIds.resize( capacity );

		// Get all overlaps and record the actual count
		int count = Shape_GetSensorData( sensorShapeId, m_visitorIds.AsSpan(), capacity );
		m_visitorIds.resize( count );

		buffer = Fmt( "%s: ", prefix );
		for ( int i = 0; i < count && buffer.Length < 256; ++i )
		{
			ShapeId visitorId = m_visitorIds[i];
			if ( Shape_IsValid( visitorId ) == false )
			{
				continue;
			}

			BodyId bodyId = Shape_GetBody( visitorId );
			string name = Body_GetName( bodyId );
			if ( name == null )
			{
				continue;
			}

			// todo fix this
			buffer += Fmt( "%s, ", name );
		}

		DrawScreenTextLine( buffer );
	}

	public override void Step(  )
	{
		Pos position = Body_GetPosition( m_kinematicBodyId );
		if ( position.y < 0.0f )
		{
			Body_SetLinearVelocity( m_kinematicBodyId, new( 0.0f, 1.0f ) );
		}
		else if ( position.y > 3.0f )
		{
			Body_SetLinearVelocity( m_kinematicBodyId, new( 0.0f, -1.0f ) );
		}

		base.Step();

		PrintOverlaps( m_staticSensorId, "static" );
		PrintOverlaps( m_kinematicSensorId, "kinematic" );
		PrintOverlaps( m_dynamicSensorId, "dynamic" );

		Pos origin = new Pos { x = 5.0f, y = 1.0f };
		Vec2 translation = new Vec2( -10.0f, 0.0f );
		RayResult result = World_CastRayClosest( m_worldId, origin, translation, DefaultQueryFilter() );
		DrawLine( m_draw, origin, origin + translation, HexColor.colorDimGray );

		if ( result.hit )
		{
			DrawPoint( m_draw, result.point, 10.0f, HexColor.colorCyan );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new SensorTypes( context );
	}

	public ShapeId m_staticSensorId;
	public ShapeId m_kinematicSensorId;
	public ShapeId m_dynamicSensorId;

	public BodyId m_kinematicBodyId;

	public List<ShapeId> m_visitorIds = new();
}

public class JointEventSample : Sample
{
	public const int e_count = 6;

	public JointEventSample( SampleContext context ) : base( context )
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

		float forceThreshold = 20000.0f;
		float torqueThreshold = 10000.0f;

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
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
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
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
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
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
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
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
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
			jointDef.angularHertz = 2.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
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
			jointDef.@base.forceThreshold = forceThreshold;
			jointDef.@base.torqueThreshold = torqueThreshold;
			jointDef.@base.collideConnected = true;
			jointDef.@base.userData = index;
			m_jointIds[index] = CreateWheelJoint( m_worldId, ref jointDef );
		}

		position.x += 5.0f;
		++index;
	}

	public override void Step(  )
	{
		base.Step();

		// Process joint events
		JointEvents events = World_GetJointEvents( m_worldId );
		for ( int i = 0; i < events.count; ++i )
		{
			// Destroy the joint if it is still valid
			ref readonly JointEvent @event = ref events.jointEvents[i];
			if ( Joint_IsValid( @event.jointId ) )
			{
				int index = (int)@event.userData;
				Debug.Assert( 0 <= index && index < e_count );
				DestroyJoint( @event.jointId );
				m_jointIds[index] = nullJointId;
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new JointEventSample( context );
	}

	public JointId[] m_jointIds = new JointId[e_count];
}

public class PersistentContact : Sample
{
	public PersistentContact( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 6.0f );
			m_context.camera.zoom = 7.5f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Vec2[] points = new Vec2[22];
			float x = 10.0f;
			for ( int i = 0; i < 20; ++i )
			{
				points[i] = new( x, 0.0f );
				x -= 1.0f;
			}

			points[20] = new( -9.0f, 10.0f );
			points[21] = new( 10.0f, 10.0f );

			ChainDef chainDef = DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = 22;
			chainDef.isLoop = true;

			CreateChain( groundId, ref chainDef );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = -8.0f, y = 1.0f };
			bodyDef.linearVelocity = new Vec2( 2.0f, 0.0f );

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.enableContactEvents = true;
			Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}

		m_contactId = nullContactId;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.beginCount && i < 1; ++i )
		{
			m_contactId = events.beginEvents[i].contactId;
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			if ( ( m_contactId ).Equals( events.endEvents[i].contactId ) )
			{
				m_contactId = nullContactId;
				break;
			}
		}

		// Contacts are created and destroyed by Box2D, so it is important to make
		// sure a contact id is valid before accessing contact data.
		if ( m_contactId.IsNonNull && Contact_IsValid( m_contactId ) )
		{
			ContactData data = Contact_GetData( m_contactId );

			for ( int i = 0; i < data.manifold.pointCount; ++i )
			{
				ref readonly ManifoldPoint manifoldPoint = ref data.manifold.points[i];
				BodyId bodyIdA = Shape_GetBody( data.shapeIdA );
				Pos p1 = Body_GetWorldCenter( bodyIdA ) + manifoldPoint.anchorA;
				Pos p2 = p1 + manifoldPoint.totalNormalImpulse * data.manifold.normal;
				DrawLine( m_draw, p1, p2, HexColor.colorCrimson );
				DrawPoint( m_draw, p1, 6.0f, HexColor.colorCrimson );
				DrawString( m_draw, m_camera, p1, HexColor.colorWhite, "%.2f", manifoldPoint.totalNormalImpulse );
			}
		}
		else
		{
			m_contactId = nullContactId;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new PersistentContact( context );
	}

	public ContactId m_contactId;
}

public class SensorHits : Sample
{
	public SensorHits( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 5.0f );
			m_context.camera.zoom = 7.5f;
		}

		BodyId groundId = default;
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "ground";

			groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			Segment groundSegment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );

			groundSegment = new Segment { point1 = new( 10.0f, 0.0f ), point2 = new( 10.0f, 10.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref groundSegment );
		}

		// Static sensor
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "static sensor";
			bodyDef.position = new Pos { x = -4.0f, y = 1.0f };

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			Segment segment = new Segment { point1 = new( 0.0f, 0.0f ), point2 = new( 0.0f, 10.0f ) };
			m_staticSensorId = CreateSegmentShape( bodyId, ref shapeDef, ref segment );
		}

		// Kinematic sensor
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "kinematic sensor";
			bodyDef.type = BodyType.kinematicBody;
			bodyDef.position = new Pos { x = 0.0f, y = 1.0f };
			bodyDef.linearVelocity = new Vec2( 0.5f, 0.0f );

			m_kinematicBodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			Segment segment = new Segment { point1 = new( 0.0f, 0.0f ), point2 = new( 0.0f, 10.0f ) };
			m_kinematicSensorId = CreateSegmentShape( m_kinematicBodyId, ref shapeDef, ref segment );
		}

		// Dynamic sensor
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.name = "dynamic sensor";
			bodyDef.type = BodyType.dynamicBody;
			bodyDef.position = new Pos { x = 4.0f, y = 1.0f };

			m_dynamicBodyId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			Capsule capsule = new Capsule { center1 = new( 0.0f, 1.0f ), center2 = new( 0.0f, 9.0f ), radius = 0.1f };
			m_dynamicSensorId = CreateCapsuleShape( m_dynamicBodyId, ref shapeDef, ref capsule );

			Pos pivot = bodyDef.position + new Vec2( 0.0f, 6.0f );
			Vec2 axis = new Vec2( 1.0f, 0.0f );
			PrismaticJointDef jointDef = DefaultPrismaticJointDef();
			jointDef.@base.bodyIdA = groundId;
			jointDef.@base.bodyIdB = m_dynamicBodyId;
			jointDef.@base.localFrameA.q = MakeRotFromUnitVector( axis );
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( groundId, pivot );
			jointDef.@base.localFrameB.q = MakeRotFromUnitVector( axis );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( m_dynamicBodyId, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorForce = 1000.0f;
			jointDef.motorSpeed = 0.5f;

			m_jointId = CreatePrismaticJoint( m_worldId, ref jointDef );
		}

		m_beginCount = 0;
		m_endCount = 0;
		m_bodyId = new BodyId {  };
		m_shapeId = new ShapeId {  };
		m_transformCount = 0;
		m_isBullet = true;

		Launch();
	}

	public void Launch(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
		}

		m_transformCount = 0;
		m_beginCount = 0;
		m_endCount = 0;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = -26.7f, y = 6.0f };
		float speed = RandomFloatRange( 200.0f, 300.0f );
		bodyDef.linearVelocity = new Vec2( speed, 0.0f );
		bodyDef.isBullet = m_isBullet;
		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = true;
		shapeDef.material.friction = 0.8f;
		shapeDef.material.rollingResistance = 0.01f;
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };
		m_shapeId = CreateCircleShape( m_bodyId, ref shapeDef, ref circle );
	}

	public override bool DrawControls(  )
	{
		ImGui.Checkbox( "Bullet", ref m_isBullet );

		if ( ImGui.Button( "Launch" ) || glfwGetKey( m_context.window, GLFW_KEY_B ) == GLFW_PRESS )
		{
			Launch();
		}

		return true;
	}

	public void CollectTransforms( ShapeId sensorShapeId )
	{
		const int capacity = 5;
		ShapeId[] visitorIds = new ShapeId[capacity];
		int count = Shape_GetSensorData( sensorShapeId, visitorIds, capacity );

		for ( int i = 0; i < count && m_transformCount < m_transformCapacity; ++i )
		{
			BodyId sensorBodyId = Shape_GetBody( sensorShapeId );
			m_transforms[m_transformCount] = Body_GetTransform( sensorBodyId );
			m_transformCount += 1;
		}
	}

	public override void Step(  )
	{
		Pos p = Body_GetPosition( m_kinematicBodyId );
		if ( p.x > 1.0f )
		{
			Body_SetLinearVelocity( m_kinematicBodyId, new( -0.5f, 0.0f ) );
		}
		else if ( p.x < -1.0f )
		{
			Body_SetLinearVelocity( m_kinematicBodyId, new( 0.5f, 0.0f ) );
		}

		float x = PrismaticJoint_GetCurrentTranslation( m_jointId );
		if ( x > 1.0f )
		{
			PrismaticJoint_SetTargetMotorSpeed( m_jointId, -0.5f );
		}
		else if ( x < -1.0f )
		{
			PrismaticJoint_SetTargetMotorSpeed( m_jointId, 0.5f );
		}

		base.Step();

		for ( int i = 0; i < m_transformCount; ++i )
		{
			DrawTransform( m_draw, m_transforms[i], 1.0f );
		}

		SensorEvents sensorEvents = World_GetSensorEvents( m_worldId );
		m_beginCount += sensorEvents.beginCount;
		m_endCount += sensorEvents.endCount;

		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			ref readonly SensorBeginTouchEvent @event = ref sensorEvents.beginEvents[i];
			if ( Shape_IsValid( @event.sensorShapeId ) == true )
			{
				CollectTransforms( @event.sensorShapeId );
			}
		}

		DrawScreenTextLine( "begin touch count = %d", m_beginCount );
		DrawScreenTextLine( "end touch count = %d", m_endCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new SensorHits( context );
	}

	public ShapeId m_staticSensorId;
	public ShapeId m_kinematicSensorId;
	public ShapeId m_dynamicSensorId;

	public BodyId m_kinematicBodyId;
	public BodyId m_dynamicBodyId;
	public JointId m_jointId;

	public BodyId m_bodyId;
	public ShapeId m_shapeId;

	public const int m_transformCapacity = 20;
	public int m_transformCount;
	public WorldTransform[] m_transforms = new WorldTransform[m_transformCapacity];

	public bool m_isBullet;
	public int m_beginCount;
	public int m_endCount;
}

public class ProjectileEvent : Sample
{
	public ProjectileEvent( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( -7.0f, 9.0f );
			m_context.camera.zoom = 14.0f;
		}

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			bodyDefInner.position = new Pos { x = 0.0f, y = 0.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.enableSensorEvents = true;

			Segment segment = new Segment { point1 = new( 10.0f, 0.0f ), point2 = new( 10.0f, 20.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );

			segment = new Segment { point1 = new( -30.0f, 0.0f ), point2 = new( 30.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDefInner, ref segment );
		}

		m_projectileId = new BodyId {  };
		m_projectileShapeId = new ShapeId {  };
		m_dragging = false;
		m_point1 = Pos_zero;
		m_point2 = Pos_zero;

		Polygon box = MakeRoundedBox( 0.45f, 0.45f, 0.05f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		float offset = 0.01f;

		float x = 8.0f;
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		for ( int i = 0; i < 8; ++i )
		{
			float shift = ( i % 2 == 0 ? -offset : offset );
			bodyDef.position = new Pos { x = x + shift, y = 0.5f + 1.0f * i };

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			CreatePolygonShape( bodyId, ref shapeDef, ref box );
		}
	}

	public void FireProjectile(  )
	{
		if ( m_projectileId.IsNonNull )
		{
			DestroyBody( m_projectileId );
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = m_point1;
		bodyDef.linearVelocity = 4.0f * ( m_point2 - m_point1 );
		bodyDef.isBullet = true;

		m_projectileId = CreateBody( m_worldId, ref bodyDef );

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.25f };
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		m_projectileShapeId = CreateCircleShape( m_projectileId, ref shapeDef, ref circle );
	}

	public override void MouseDown( Pos p, int button, int mods )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == GLFW_MOD_CONTROL )
			{
				m_dragging = true;
				m_point1 = p;
			}
		}
	}

	public override void MouseUp( Pos unused23, int button )
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( m_dragging )
			{
				m_dragging = false;
				FireProjectile();
			}
		}
	}

	public override void MouseMove( Pos p )
	{
		if ( m_dragging )
		{
			m_point2 = p;
		}
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "Use Ctrl + Left Mouse to drag and shoot a projectile" );

		base.Step();

		if ( m_dragging )
		{
			DrawLine( m_draw, m_point1, m_point2, HexColor.colorWhite );
			DrawPoint( m_draw, m_point1, 5.0f, HexColor.colorGreen );
			DrawPoint( m_draw, m_point2, 5.0f, HexColor.colorRed );
		}

		ContactEvents contactEvents = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < contactEvents.beginCount; ++i )
		{
			ref readonly ContactBeginTouchEvent @event = ref contactEvents.beginEvents[i];
			if ( ( @event.shapeIdA ).Equals( m_projectileShapeId ) || ( @event.shapeIdB ).Equals( m_projectileShapeId ) )
			{
				if ( Contact_IsValid( @event.contactId ) )
				{
					ContactData data = Contact_GetData( @event.contactId );

					if ( data.manifold.pointCount > 0 )
					{
						ExplosionDef explosionDef = DefaultExplosionDef();
						explosionDef.position =
							Body_GetWorldCenter( Shape_GetBody( data.shapeIdA ) ) + data.manifold.points[0].anchorA;
						explosionDef.radius = 1.0f;
						explosionDef.impulsePerLength = 20.0f;
						World_Explode( m_worldId, ref explosionDef );

						DestroyBody( m_projectileId );
						m_projectileId = nullBodyId;
					}
				}

				break;
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new ProjectileEvent( context );
	}

	public BodyId m_projectileId;
	public ShapeId m_projectileShapeId;
	public Pos m_point1;
	public Pos m_point2;
	public bool m_dragging;
}

public class CircleImpulse : Sample
{
	public struct Event
	{
		public float impulse;
		public float totalImpulse;
		public float speed;
	}

	public CircleImpulse( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.7f );
			m_context.camera.zoom = 3.4f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			Segment segment = new Segment { point1 = new( -10.0f, 0.0f ), point2 = new( 10.0f, 0.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		m_gravity = 10.0f;
		m_restitution = 0.25f;
		m_useGravity = false;
		m_useRestitution = false;
		m_mass = 1.0f;
		m_bodyId = nullBodyId;

		Spawn();
	}

	public void Spawn(  )
	{
		if ( m_bodyId.IsNonNull )
		{
			DestroyBody( m_bodyId );
			m_bodyId = nullBodyId;
		}

		m_events.Clear();

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = m_useGravity ? 1.0f : 0.0f;
		bodyDef.linearVelocity.y = -25.0f;
		bodyDef.position.y = 5.5f;

		Circle circle = default;
		circle.radius = 0.25f;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = m_useRestitution ? m_restitution : 0.0f;

		m_bodyId = CreateBody( m_worldId, ref bodyDef );

		CreateCircleShape( m_bodyId, ref shapeDef, ref circle );

		// Override mass
		MassData massData = Body_GetMassData( m_bodyId );
		float ratio = m_mass / massData.mass;
		massData.mass = m_mass;
		massData.rotationalInertia *= ratio;
		Body_SetMassData( m_bodyId, massData );
	}

	public override bool DrawControls(  )
	{
		if ( ImGui.Checkbox( "gravity", ref m_useGravity ) )
		{
			Spawn();
		}

		if ( ImGui.Checkbox( "restitution", ref m_useRestitution ) )
		{
			Spawn();
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			ref ContactHitEvent @event = ref events.hitEvents[i];
			DrawPoint( m_draw, @event.point, 10.0f, HexColor.colorWhite );

			ContactData data = Contact_GetData( @event.contactId );

			Event e = default;
			e.speed = @event.approachSpeed;
			if ( data.manifold.pointCount > 0 )
			{
				e.impulse = data.manifold.points[0].normalImpulse;
				e.totalImpulse = data.manifold.points[0].totalNormalImpulse;
			}
			m_events.Add( e );
		}

		DrawScreenTextLine( "mass = %g, gravity = %g, restitution = %g", m_mass, m_useGravity ? 10.0f : 0.0f,
							m_useRestitution ? m_restitution : 0.0f );

		int eventCount = (int)m_events.Count;
		for ( int i = 0; i < eventCount; ++i )
		{
			Event e = m_events[i];
			DrawScreenTextLine( "hit speed = %g, hit momentum = %g, final impulse = %g, total impulse = %g", e.speed,
								m_mass * e.speed, e.impulse, e.totalImpulse );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new CircleImpulse( context );
	}

	public float m_mass;
	public List<Event> m_events = new();
	public BodyId m_bodyId;
	public float m_gravity;
	public float m_restitution;
	public bool m_useGravity;
	public bool m_useRestitution;
}
