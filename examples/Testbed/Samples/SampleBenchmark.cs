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

public static partial class Helpers_sample_benchmark
{

	// these are not accessible in some build types
	// extern "C" int toiCalls;
	// extern "C" int toiHitCount;

	// Note: resetting the scene is non-deterministic because the world uses freelists



	// This is used to compare performance with Box2D v2.4









	// This stresses most aspects of simulation:
	// - broad-phase because pairs constantly change
	// - narrow-phase because contact recycling doesn't always apply
	// - constraint solver because there are many contacts
	// - hit events are enabled so there is extra velocity computation and reporting



	// todo try removing kinematics from graph coloring




























	public const int e_rayCast = 0;
	public const int e_circleCast = 1;
	public const int e_overlap = 2;




















	// This benchmark pushes Box2D to the limit for a large pile. It terminates once simulation is deemed to be slow.
	// The higher the body count achieved, the better.
	// Note: this benchmark stresses the sleep system more than any other benchmark. Better results are achieved if sleeping
	// is disabled.






	// The queries benchmark scene with one of its queries drawn and the cost of the whole set
}

public class BenchmarkBarrel : Sample
{
	public const int e_circleShape = 0;
	public const int e_capsuleShape = 0 + 1;
	public const int e_mixShape = 0 + 2;
	public const int e_compoundShape = 0 + 3;
	public const int e_humanShape = 0 + 4;

	public const int e_maxColumns = 26;
	public const int e_maxRows = 150;

	public BenchmarkBarrel( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 8.0f, 53.0f );
			m_context.camera.zoom = 25.0f * 2.35f;
		}

		m_context.debugDraw.drawJoints = false;

		{
			float gridSize = 1.0f;

			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				Polygon box = MakeOffsetBox( 0.55f * gridSize, 0.5f * gridSize, new( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDef, ref box );
				x += gridSize;
			}

			y = gridSize;
			x = -40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				Polygon box = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDef, ref box );
				y += gridSize;
			}

			y = gridSize;
			x = 40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				Polygon box = MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, new( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDef, ref box );
				y += gridSize;
			}

			Segment segment = new Segment { point1 = new( -800.0f, -80.0f ), point2 = new( 800.0f, -80.0f ) };
			CreateSegmentShape( groundId, ref shapeDef, ref segment );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = nullBodyId;
		}

		Array.Clear( m_humans );

		m_shapeType = e_compoundShape;

		CreateScene();
	}

	public void CreateScene(  )
	{
		g_randomSeed = 42;

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( m_bodies[i].IsNonNull )
			{
				DestroyBody( m_bodies[i] );
				m_bodies[i] = nullBodyId;
			}

			if ( m_humans[i].isSpawned )
			{
				DestroyHuman( ref m_humans[i] );
			}
		}

		m_columnCount = m_isDebug ? 10 : e_maxColumns;
		m_rowCount = m_isDebug ? 40 : e_maxRows;

		if ( m_shapeType == e_compoundShape )
		{
			if ( m_isDebug == false )
			{
				m_columnCount = 20;
			}
		}
		else if ( m_shapeType == e_humanShape )
		{
			if ( m_isDebug )
			{
				m_rowCount = 5;
				m_columnCount = 10;
			}
			else
			{
				m_rowCount = 30;
			}
		}

		float rad = 0.5f;

		float shift = 1.15f;
		float centerx = shift * m_columnCount / 2.0f;
		float centery = shift / 2.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		// todo eliminate this once rolling resistance is added
		if ( m_shapeType == e_mixShape )
		{
			bodyDef.angularDamping = 0.3f;
		}

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		Capsule capsule = new Capsule { center1 = new( 0.0f, -0.25f ), center2 = new( 0.0f, 0.25f ), radius = rad };
		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = rad };

		Vec2[] points = { new( -0.1f, -0.5f ), new( 0.1f, -0.5f ), new( 0.0f, 0.5f ) };
		Hull wedgeHull = ComputeHull( points, 3 );
		Polygon wedge = MakePolygon( ref wedgeHull, 0.0f );

		Vec2[] vertices = new Vec2[3];
		vertices[0] = new( -1.0f, 0.0f );
		vertices[1] = new( 0.5f, 1.0f );
		vertices[2] = new( 0.0f, 2.0f );
		Hull hull = ComputeHull( vertices, 3 );
		Polygon left = MakePolygon( ref hull, 0.0f );

		vertices[0] = new( 1.0f, 0.0f );
		vertices[1] = new( -0.5f, 1.0f );
		vertices[2] = new( 0.0f, 2.0f );
		hull = ComputeHull( vertices, 3 );
		Polygon right = MakePolygon( ref hull, 0.0f );

		// Polygon top = MakeOffsetBox(0.8f, 0.2f, new( 0.0f, 0.8f ), 0.0f);
		// Polygon leftLeg = MakeOffsetBox(0.2f, 0.5f, new( -0.6f, 0.5f ), 0.0f);
		// Polygon rightLeg = MakeOffsetBox(0.2f, 0.5f, new( 0.6f, 0.5f ), 0.0f);

		float side = -0.1f;
		float extray = 0.5f;

		if ( m_shapeType == e_compoundShape )
		{
			extray = 0.25f;
			side = 0.25f;
			shift = 2.0f;
			centerx = shift * m_columnCount / 2.0f - 1.0f;
		}
		else if ( m_shapeType == e_humanShape )
		{
			extray = 0.5f;
			side = 0.55f;
			shift = 2.5f;
			centerx = shift * m_columnCount / 2.0f;
		}

		int index = 0;
		float yStart = m_shapeType == e_humanShape ? 2.0f : 100.0f;

		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < m_rowCount; ++j )
			{
				float y = j * ( shift + extray ) + centery + yStart;

				bodyDef.position = new Pos { x = x + side, y = y };
				side = -side;

				if ( m_shapeType == e_circleShape )
				{
					m_bodies[index] = CreateBody( m_worldId, ref bodyDef );
					circle.radius = RandomFloatRange( 0.25f, 0.75f );
					shapeDef.material.rollingResistance = 0.2f;
					CreateCircleShape( m_bodies[index], ref shapeDef, ref circle );
				}
				else if ( m_shapeType == e_capsuleShape )
				{
					m_bodies[index] = CreateBody( m_worldId, ref bodyDef );
					capsule.radius = RandomFloatRange( 0.25f, 0.5f );
					float length = RandomFloatRange( 0.25f, 1.0f );
					capsule.center1 = new Vec2( 0.0f, -0.5f * length );
					capsule.center2 = new Vec2( 0.0f, 0.5f * length );
					shapeDef.material.rollingResistance = 0.2f;
					CreateCapsuleShape( m_bodies[index], ref shapeDef, ref capsule );
				}
				else if ( m_shapeType == e_mixShape )
				{
					m_bodies[index] = CreateBody( m_worldId, ref bodyDef );

					int mod = index % 3;
					if ( mod == 0 )
					{
						circle.radius = RandomFloatRange( 0.25f, 0.75f );
						CreateCircleShape( m_bodies[index], ref shapeDef, ref circle );
					}
					else if ( mod == 1 )
					{
						capsule.radius = RandomFloatRange( 0.25f, 0.5f );
						float length = RandomFloatRange( 0.25f, 1.0f );
						capsule.center1 = new Vec2( 0.0f, -0.5f * length );
						capsule.center2 = new Vec2( 0.0f, 0.5f * length );
						CreateCapsuleShape( m_bodies[index], ref shapeDef, ref capsule );
					}
					else if ( mod == 2 )
					{
						float width = RandomFloatRange( 0.1f, 0.5f );
						float height = RandomFloatRange( 0.5f, 0.75f );
						Polygon box = MakeBox( width, height );

						// Don't put a function call into a macro.
						float value = RandomFloatRange( -1.0f, 1.0f );
						box.radius = 0.25f * MaxFloat( 0.0f, value );
						CreatePolygonShape( m_bodies[index], ref shapeDef, ref box );
					}
					else
					{
						wedge.radius = RandomFloatRange( 0.1f, 0.25f );
						CreatePolygonShape( m_bodies[index], ref shapeDef, ref wedge );
					}
				}
				else if ( m_shapeType == e_compoundShape )
				{
					m_bodies[index] = CreateBody( m_worldId, ref bodyDef );

					CreatePolygonShape( m_bodies[index], ref shapeDef, ref left );
					CreatePolygonShape( m_bodies[index], ref shapeDef, ref right );
					// CreatePolygonShape(m_bodies[index], ref shapeDef, ref top);
					// CreatePolygonShape(m_bodies[index], ref shapeDef, ref leftLeg);
					// CreatePolygonShape(m_bodies[index], ref shapeDef, ref rightLeg);
				}
				else if ( m_shapeType == e_humanShape )
				{
					float scale = 3.5f;
					float jointFriction = 0.05f;
					float jointHertz = 5.0f;
					float jointDamping = 0.5f;
					CreateHuman( ref m_humans[index], m_worldId, bodyDef.position, scale, jointFriction, jointHertz, jointDamping,
								 index + 1, null, false );
				}

				index += 1;
			}
		}
	}

	public override bool DrawControls(  )
	{
		bool changed = false;
		string[] shapeTypes = { "Circle", "Capsule", "Mix", "Compound", "Human" };

		int shapeType = (int)( m_shapeType );
		changed = changed || ImGui.Combo( "Shape", ref shapeType, shapeTypes, shapeTypes.Length );
		m_shapeType = (int)( shapeType );

		changed = changed || ImGui.Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkBarrel( context );
	}

	public BodyId[] m_bodies = new BodyId[e_maxRows * e_maxColumns];
	public Human[] m_humans = new Human[e_maxRows * e_maxColumns];
	public int m_columnCount;
	public int m_rowCount;

	public int m_shapeType;
}

public class BenchmarkBarrel24 : Sample
{
	public BenchmarkBarrel24( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 8.0f, 53.0f );
			m_context.camera.zoom = 25.0f * 2.35f;
		}

		float groundSize = 25.0f;

		{
			BodyDef bodyDefInner = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDefInner );

			Polygon box = MakeBox( groundSize, 1.2f );
			ShapeDef shapeDefInner = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			bodyDefInner.rotation = MakeRot( 0.5f * Pi );
			bodyDefInner.position = new Pos { x = groundSize, y = 2.0f * groundSize };
			groundId = CreateBody( m_worldId, ref bodyDefInner );

			box = MakeBox( 2.0f * groundSize, 1.2f );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );

			bodyDefInner.position = new Pos { x = -groundSize, y = 2.0f * groundSize };
			groundId = CreateBody( m_worldId, ref bodyDefInner );
			CreatePolygonShape( groundId, ref shapeDefInner, ref box );
		}

		int num = 26;
		float rad = 0.5f;

		float shift = rad * 2.0f;
		float centerx = shift * num / 2.0f;
		float centery = shift / 2.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		Polygon cuboid = MakeSquare( 0.5f );

		// Polygon top = MakeOffsetBox(0.8f, 0.2f, new( 0.0f, 0.8f ), 0.0f);
		// Polygon leftLeg = MakeOffsetBox(0.2f, 0.5f, new( -0.6f, 0.5f ), 0.0f);
		// Polygon rightLeg = MakeOffsetBox(0.2f, 0.5f, new( 0.6f, 0.5f ), 0.0f);

#if DEBUG
		int numj = 5;
#else
		int numj = 5 * num;
#endif
		for ( int i = 0; i < num; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < numj; ++j )
			{
				float y = j * shift + centery + 2.0f;

				bodyDef.position = new Pos { x = x, y = y };

				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref cuboid );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkBarrel24( context );
	}
}

public class BenchmarkCompounds : Sample
{
	public BenchmarkCompounds( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 50.0f );
			m_context.camera.zoom = 25.0f * 2.2f;
			m_context.enableSleep = false;
		}

		CreateCompounds( m_worldId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkCompounds( context );
	}
}

public class BenchmarkTumbler : Sample
{
	public BenchmarkTumbler( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.5f, 10.0f );
			m_context.camera.zoom = 15.0f;
		}

		CreateTumbler( m_worldId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkTumbler( context );
	}
}

public class BenchmarkWasher : Sample
{
	public BenchmarkWasher( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.5f, 10.0f );
			m_context.camera.zoom = 20.0f;
		}

		CreateWasher( m_worldId );
	}

	public override void Step(  )
	{
		base.Step();

		ContactEvents events = World_GetContactEvents( m_worldId );
		DrawScreenTextLine( "hits = %d", events.hitCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkWasher( context );
	}
}

public class BenchmarkManyTumblers : Sample
{
	public BenchmarkManyTumblers( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 1.0f, -5.5f );
			m_context.camera.zoom = 25.0f * 3.4f;
			m_context.debugDraw.drawJoints = false;
		}

		BodyDef bodyDef = DefaultBodyDef();
		m_groundId = CreateBody( m_worldId, ref bodyDef );

		m_rowCount = m_isDebug ? 2 : 19;
		m_columnCount = m_isDebug ? 2 : 19;

		m_tumblerIds = null;
		m_positions = null;
		m_tumblerCount = 0;

		m_bodyIds = null;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		m_angularSpeed = 25.0f;

		CreateScene();
	}

	public override void Dispose()
	{
		base.Dispose();
	}

	public void CreateTumbler( Pos position, int index )
	{
		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.kinematicBody;
		bodyDef.position = position;
		bodyDef.angularVelocity = ( Pi / 180.0f ) * m_angularSpeed;
		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
		m_tumblerIds[index] = bodyId;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 50.0f;

		Polygon polygon = default;
		polygon = MakeOffsetBox( 0.25f, 2.0f, new( 2.0f, 0.0f ), Rot_identity );
		CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		polygon = MakeOffsetBox( 0.25f, 2.0f, new( -2.0f, 0.0f ), Rot_identity );
		CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		polygon = MakeOffsetBox( 2.0f, 0.25f, new( 0.0f, 2.0f ), Rot_identity );
		CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
		polygon = MakeOffsetBox( 2.0f, 0.25f, new( 0.0f, -2.0f ), Rot_identity );
		CreatePolygonShape( bodyId, ref shapeDef, ref polygon );
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			if ( m_bodyIds[i].IsNonNull )
			{
				DestroyBody( m_bodyIds[i] );
			}
		}

		for ( int i = 0; i < m_tumblerCount; ++i )
		{
			DestroyBody( m_tumblerIds[i] );
		}
		m_tumblerCount = m_rowCount * m_columnCount;
		m_tumblerIds = new BodyId[m_tumblerCount];
		m_positions = new Pos[m_tumblerCount];

		int index = 0;
		float x = -4.0f * m_rowCount;
		for ( int i = 0; i < m_rowCount; ++i )
		{
			float y = -4.0f * m_columnCount;
			for ( int j = 0; j < m_columnCount; ++j )
			{
				m_positions[index] = new( x, y );
				CreateTumbler( m_positions[index], index );
				++index;
				y += 8.0f;
			}

			x += 8.0f;
		}
		int bodiesPerTumbler = m_isDebug ? 8 : 50;
		m_bodyCount = bodiesPerTumbler * m_tumblerCount;

		m_bodyIds = new BodyId[m_bodyCount];

		m_bodyIndex = 0;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		changed = changed || ImGui.SliderInt( "Row Count", ref m_rowCount, 1, 32 );
		changed = changed || ImGui.SliderInt( "Column Count", ref m_columnCount, 1, 32 );

		if ( changed )
		{
			CreateScene();
		}

		if ( ImGui.SliderFloat( "Speed", ref m_angularSpeed, 0.0f, 100.0f, "%.f" ) )
		{
			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				Body_SetAngularVelocity( m_tumblerIds[i], ( Pi / 180.0f ) * m_angularSpeed );
				Body_SetAwake( m_tumblerIds[i], true );
			}
		}

		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_bodyIndex < m_bodyCount && ( m_stepCount & 0x7 ) == 0 )
		{
			ShapeDef shapeDef = DefaultShapeDef();

			Capsule capsule = new Capsule { center1 = new( -0.1f, 0.0f ), center2 = new( 0.1f, 0.0f ), radius = 0.075f };

			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				Debug.Assert( m_bodyIndex < m_bodyCount );

				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.dynamicBody;
				bodyDef.position = m_positions[i];
				m_bodyIds[m_bodyIndex] = CreateBody( m_worldId, ref bodyDef );
				CreateCapsuleShape( m_bodyIds[m_bodyIndex], ref shapeDef, ref capsule );

				m_bodyIndex += 1;
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkManyTumblers( context );
	}

	public BodyId m_groundId;

	public int m_rowCount;
	public int m_columnCount;

	public BodyId[] m_tumblerIds;
	public Pos[] m_positions;
	public int m_tumblerCount;

	public BodyId[] m_bodyIds;
	public int m_bodyCount;
	public int m_bodyIndex;

	public float m_angularSpeed;
}

public class BenchmarkLargePyramid : Sample
{
	public BenchmarkLargePyramid( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 50.0f );
			m_context.camera.zoom = 25.0f * 2.2f;
			m_context.enableSleep = false;
		}

		CreateLargePyramid( m_worldId );

		m_stepWhilePaused = false;
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkLargePyramid( context );
	}
}

public class BenchmarkManyPyramids : Sample
{
	public BenchmarkManyPyramids( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 23.0f, 72.5f );
			m_context.camera.zoom = 165.0f;
			m_context.enableSleep = false;
		}

		CreateManyPyramids( m_worldId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkManyPyramids( context );
	}
}

public class BenchmarkCreateDestroy : Sample
{
	public const int e_maxBaseCount = 100;
	public const int e_maxBodyCount = e_maxBaseCount * ( e_maxBaseCount + 1 ) / 2;

	public BenchmarkCreateDestroy( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 50.0f );
			m_context.camera.zoom = 25.0f * 2.2f;
		}

		float groundSize = 100.0f;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		Polygon box = MakeBox( groundSize, 1.0f );
		ShapeDef shapeDef = DefaultShapeDef();
		CreatePolygonShape( groundId, ref shapeDef, ref box );

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			m_bodies[i] = nullBodyId;
		}

		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		m_baseCount = m_isDebug ? 40 : 100;
		m_iterations = m_isDebug ? 1 : 10;
		m_bodyCount = 0;
	}

	public void CreateScene(  )
	{
		ulong ticks = GetTicks();

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			if ( m_bodies[i].IsNonNull )
			{
				DestroyBody( m_bodies[i] );
				m_bodies[i] = nullBodyId;
			}
		}

		m_destroyTime += GetMillisecondsAndReset( ref ticks );

		int count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		float h = 0.5f;
		Polygon box = MakeRoundedBox( h, h, 0.0f );

		int index = 0;

		for ( int i = 0; i < count; ++i )
		{
			float y = i * shift + centery;

			for ( int j = i; j < count; ++j )
			{
				float x = 0.5f * i * shift + ( j - i ) * shift - centerx;
				bodyDef.position = new Pos { x = x, y = y };

				Debug.Assert( index < e_maxBodyCount );
				m_bodies[index] = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( m_bodies[index], ref shapeDef, ref box );

				index += 1;
			}
		}

		m_createTime += GetMilliseconds( ticks );

		m_bodyCount = index;

		World_Step( m_worldId, 1.0f / 60.0f, 4 );
	}

	public override void Step(  )
	{
		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		for ( int i = 0; i < m_iterations; ++i )
		{
			CreateScene();
		}

		DrawScreenTextLine( "total: create = %g ms, destroy = %g ms", m_createTime, m_destroyTime );

		float createPerBody = 1000.0f * m_createTime / m_iterations / m_bodyCount;
		float destroyPerBody = 1000.0f * m_destroyTime / m_iterations / m_bodyCount;
		DrawScreenTextLine( "body: create = %g us, destroy = %g us", createPerBody, destroyPerBody );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkCreateDestroy( context );
	}

	public float m_createTime;
	public float m_destroyTime;
	public BodyId[] m_bodies = new BodyId[e_maxBodyCount];
	public int m_bodyCount;
	public int m_baseCount;
	public int m_iterations;
}

public class BenchmarkSleep : Sample
{
	public BenchmarkSleep( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 20.0f );
			m_context.camera.zoom = 25.0f * 6.0f;
		}

		CreateSleep( m_worldId );
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			StepSleep( m_worldId, m_stepCount );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkSleep( context );
	}
}

public class BenchmarkJointGrid : Sample
{
	public BenchmarkJointGrid( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 60.0f, -57.0f );
			m_context.camera.zoom = 25.0f * 2.5f;
			m_context.enableSleep = false;
		}

		CreateJointGrid( m_worldId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkJointGrid( context );
	}
}

public class BenchmarkSmash : Sample
{
	public BenchmarkSmash( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 60.0f, 6.0f );
			m_context.camera.zoom = 25.0f * 1.6f;
		}

		CreateSmash( m_worldId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkSmash( context );
	}
}

public class BenchmarkLargeCompounds : Sample
{
	public BenchmarkLargeCompounds( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 18.0f, 115.0f );
			m_context.camera.zoom = 25.0f * 5.5f;
		}

		float grid = 1.0f;
#if !DEBUG
		int height = 200;
		int width = 200;
#else
		int height = 100;
		int width = 100;
#endif
		{

			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = grid * j;
					Polygon square = MakeOffsetBox( 0.5f * grid, 0.5f * grid, new( x, y ), Rot_identity );
					CreatePolygonShape( groundId, ref shapeDef, ref square );
				}
			}

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = -grid * j;
					Polygon square = MakeOffsetBox( 0.5f * grid, 0.5f * grid, new( x, y ), Rot_identity );
					CreatePolygonShape( groundId, ref shapeDef, ref square );
				}
			}
		}

		{
#if !DEBUG
			int span = 20;
			int count = 5;
#else
			int span = 5;
			int count = 5;
#endif

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			// defer mass properties to avoid n-squared mass computations
			ShapeDef shapeDef = DefaultShapeDef();
			shapeDef.updateBodyMass = false;

			for ( int m = 0; m < count; ++m )
			{
				float ybody = ( 100.0f + m * span ) * grid;

				for ( int n = 0; n < count; ++n )
				{
					float xbody = -0.5f * grid * count * span + n * span * grid;
					bodyDef.position = new Pos { x = xbody, y = ybody };
					BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

					for ( int i = 0; i < span; ++i )
					{
						float y = i * grid;
						for ( int j = 0; j < span; ++j )
						{
							float x = j * grid;
							Polygon square = MakeOffsetBox( 0.5f * grid, 0.5f * grid, new( x, y ), Rot_identity );
							CreatePolygonShape( bodyId, ref shapeDef, ref square );
						}
					}

					// All shapes have been added so I can efficiently compute the mass properties.
					Body_UpdateMassFromShapes( bodyId );
				}
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkLargeCompounds( context );
	}
}

public class BenchmarkKinematic : Sample
{
	public BenchmarkKinematic( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 150.0f;
		}

		float grid = 1.0f;

#if !DEBUG
		int span = 100;
#else
		int span = 20;
#endif

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.kinematicBody;
		bodyDef.angularVelocity = 1.0f;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		shapeDef.filter.maskBits = 2;

		// defer mass properties to avoid n-squared mass computations
		shapeDef.updateBodyMass = false;

		BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

		for ( int i = -span; i < span; ++i )
		{
			float y = i * grid;
			for ( int j = -span; j < span; ++j )
			{
				float x = j * grid;
				Polygon square = MakeOffsetBox( 0.5f * grid, 0.5f * grid, new( x, y ), Rot_identity );
				CreatePolygonShape( bodyId, ref shapeDef, ref square );
			}
		}

		// All shapes have been added so I can efficiently compute the mass properties.
		Body_UpdateMassFromShapes( bodyId );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkKinematic( context );
	}
}

public class BenchmarkCast : Sample
{
	public BenchmarkCast( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 500.0f, 500.0f );
			m_context.camera.zoom = 25.0f * 21.0f;
			// settings.drawShapes = m_isDebug;
		}

		m_queryType = e_circleCast;
		m_ratio = 5.0f;
		m_grid = 1.0f;
		m_fill = 0.1f;
		m_rowCount = m_isDebug ? 100 : 1000;
		m_columnCount = m_isDebug ? 100 : 1000;
		m_minTime = 1e6f;
		m_drawIndex = 0;
		m_topDown = false;
		m_buildTime = 0.0f;
		m_radius = 0.1f;

		g_randomSeed = 1234;
		int sampleCount = m_isDebug ? 100 : 10000;
		m_origins.resize( sampleCount );
		m_translations.resize( sampleCount );
		float extent = m_rowCount * m_grid;

		// Pre-compute rays to avoid randomizer overhead
		for ( int i = 0; i < sampleCount; ++i )
		{
			Pos rayStart = RandomPos( 0.0f, extent );
			Pos rayEnd = RandomPos( 0.0f, extent );

			m_origins[i] = rayStart;
			m_translations[i] = rayEnd - rayStart;
		}

		BuildScene();
	}

	public void BuildScene(  )
	{
		g_randomSeed = 1234;
		DestroyWorld( m_worldId );
		WorldDef worldDef = DefaultWorldDef();
		m_worldId = B2.CreateWorld( ref worldDef );

		ulong ticks = GetTicks();

		BodyDef bodyDef = DefaultBodyDef();
		ShapeDef shapeDef = DefaultShapeDef();

		float y = 0.0f;

		for ( int i = 0; i < m_rowCount; ++i )
		{
			float x = 0.0f;

			for ( int j = 0; j < m_columnCount; ++j )
			{
				float fillTest = RandomFloatRange( 0.0f, 1.0f );
				if ( fillTest <= m_fill )
				{
					bodyDef.position = new Pos { x = x, y = y };
					BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

					float ratio = RandomFloatRange( 1.0f, m_ratio );
					float halfWidth = RandomFloatRange( 0.05f, 0.25f );

					Polygon box = default;
					if ( RandomFloat() > 0.0f )
					{
						box = MakeBox( ratio * halfWidth, halfWidth );
					}
					else
					{
						box = MakeBox( halfWidth, ratio * halfWidth );
					}

					int category = RandomIntRange( 0, 2 );
					shapeDef.filter.categoryBits = 1ul << category;
					if ( category == 0 )
					{
						shapeDef.material.customColor = (uint)HexColor.colorBox2DBlue;
					}
					else if ( category == 1 )
					{
						shapeDef.material.customColor = (uint)HexColor.colorBox2DYellow;
					}
					else
					{
						shapeDef.material.customColor = (uint)HexColor.colorBox2DGreen;
					}

					CreatePolygonShape( bodyId, ref shapeDef, ref box );
				}

				x += m_grid;
			}

			y += m_grid;
		}

		if ( m_topDown )
		{
			World_RebuildStaticTree( m_worldId );
		}

		m_buildTime = GetMilliseconds( ticks );
		m_minTime = 1e6f;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;

		string[] queryTypes = { "Ray", "Circle", "Overlap" };
		int queryType = (int)( m_queryType );
		if ( ImGui.Combo( "Query", ref queryType, queryTypes, queryTypes.Length ) )
		{
			m_queryType = queryType;
			if ( m_queryType == e_overlap )
			{
				m_radius = 5.0f;
			}
			else
			{
				m_radius = 0.1f;
			}

			changed = true;
		}

		if ( ImGui.SliderInt( "rows", ref m_rowCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderInt( "columns", ref m_columnCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "fill", ref m_fill, 0.0f, 1.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "grid", ref m_grid, 0.5f, 2.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui.SliderFloat( "ratio", ref m_ratio, 1.0f, 10.0f, "%.2f" ) )
		{
			changed = true;
		}

		ImGui.PopItemWidth();

		if ( ImGui.Checkbox( "top down", ref m_topDown ) )
		{
			changed = true;
		}

		if ( ImGui.Button( "Draw Next" ) )
		{
			m_drawIndex = ( m_drawIndex + 1 ) % m_origins.Count;
		}

		if ( changed )
		{
			BuildScene();
		}

		return true;
	}

	public class CastResult
	{
		public Pos point;
		public float fraction;
		public bool hit;
	}

	public static float CastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		CastResult? result = (CastResult?)context;
		result.point = point;
		result.fraction = fraction;
		result.hit = true;
		return fraction;
	}

	public class OverlapResult
	{
		public Vec2[] points = new Vec2[32];
		public int count;
	}

	public static bool OverlapCallback( ShapeId shapeId, object? context )
	{
		OverlapResult? result = (OverlapResult?)context;
		if ( result.count < 32 )
		{
			AABB aabb = Shape_GetBounds( shapeId );
			result.points[result.count] = AABB_Center( aabb );
			result.count += 1;
		}

		return true;
	}

	public override void Step(  )
	{
		base.Step();

		QueryFilter filter = DefaultQueryFilter();
		filter.maskBits = 1;
		int hitCount = 0;
		int nodeVisits = 0;
		int leafVisits = 0;
		float ms = 0.0f;
		int sampleCount = (int)m_origins.Count;

		if ( m_queryType == e_rayCast )
		{
			ulong ticks = GetTicks();

			RayResult drawResult = default;

			for ( int i = 0; i < sampleCount; ++i )
			{
				Pos origin = m_origins[i];
				Vec2 translation = m_translations[i];

				RayResult result = World_CastRayClosest( m_worldId, origin, translation, filter );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += result.nodeVisits;
				leafVisits += result.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = GetMilliseconds( ticks );

			m_minTime = MinFloat( m_minTime, ms );

			Pos p1 = m_origins[m_drawIndex];
			Pos p2 = m_origins[m_drawIndex] + m_translations[m_drawIndex];
			DrawLine( m_context.draw, p1, p2, HexColor.colorWhite );
			DrawPoint( m_context.draw, p1, 5.0f, HexColor.colorGreen );
			DrawPoint( m_context.draw, p2, 5.0f, HexColor.colorRed );
			if ( drawResult.hit )
			{
				DrawPoint( m_context.draw, drawResult.point, 5.0f, HexColor.colorWhite );
			}
		}
		else if ( m_queryType == e_circleCast )
		{
			ulong ticks = GetTicks();

			CastResult drawResult = new();

			for ( int i = 0; i < sampleCount; ++i )
			{
				ShapeProxy proxy = MakeProxy( in Vec2_zero, 1, m_radius );
				Vec2 translation = m_translations[i];

				CastResult result = new();
				TreeStats traversalResult =
					World_CastShape( m_worldId, m_origins[i], ref proxy, translation, filter, CastCallback, result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = GetMilliseconds( ticks );

			m_minTime = MinFloat( m_minTime, ms );

			Pos p1 = m_origins[m_drawIndex];
			Pos p2 = m_origins[m_drawIndex] + m_translations[m_drawIndex];
			DrawLine( m_context.draw, p1, p2, HexColor.colorWhite );
			DrawPoint( m_context.draw, p1, 5.0f, HexColor.colorGreen );
			DrawPoint( m_context.draw, p2, 5.0f, HexColor.colorRed );
			if ( drawResult.hit )
			{
				Pos t = m_origins[m_drawIndex] + MulSV( drawResult.fraction, m_translations[m_drawIndex] );
				DrawCircle( m_context.draw, t, m_radius, HexColor.colorWhite );
				DrawPoint( m_context.draw, drawResult.point, 5.0f, HexColor.colorWhite );
			}
		}
		else if ( m_queryType == e_overlap )
		{
			ulong ticks = GetTicks();

			OverlapResult drawResult = new();
			Vec2 extent = new Vec2( m_radius, m_radius );
			OverlapResult result = new();

			for ( int i = 0; i < sampleCount; ++i )
			{
				AABB aabbInner = new AABB { lowerBound = -extent, upperBound = extent };

				result.count = 0;
				TreeStats traversalResult =
					World_OverlapAABB( m_worldId, m_origins[i], aabbInner, filter, OverlapCallback, result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.count;
			}

			ms = GetMilliseconds( ticks );

			m_minTime = MinFloat( m_minTime, ms );

			Pos origin = m_origins[m_drawIndex];
			AABB aabb = new AABB { lowerBound = new( RoundDownFloat( origin.x - extent.x ), RoundDownFloat( origin.y - extent.y ) ), upperBound = new( RoundUpFloat( origin.x + extent.x ), RoundUpFloat( origin.y + extent.y ) ) };

			DrawBounds( m_context.draw, aabb, HexColor.colorWhite );

			for ( int i = 0; i < drawResult.count; ++i )
			{
				DrawPoint( m_context.draw, ToPos( drawResult.points[i] ), 5.0f, HexColor.colorHotPink );
			}
		}

		DrawScreenTextLine( "build time ms = %g", m_buildTime );
		DrawScreenTextLine( "hit count = %d, node visits = %d, leaf visits = %d", hitCount, nodeVisits, leafVisits );
		DrawScreenTextLine( "total ms = %.3f", ms );
		DrawScreenTextLine( "min total ms = %.3f", m_minTime );

		float aveRayCost = 1000.0f * m_minTime / (float)( sampleCount );
		DrawScreenTextLine( "average us = %.2f", aveRayCost );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkCast( context );
	}

	public int m_queryType;

	public List<Pos> m_origins = new();
	public List<Vec2> m_translations = new();
	public float m_minTime;
	public float m_buildTime;

	public int m_rowCount, m_columnCount;
	public int m_updateType;
	public int m_drawIndex;
	public float m_radius;
	public float m_fill;
	public float m_ratio;
	public float m_grid;
	public bool m_topDown;
}

public class BenchmarkSpinner : Sample
{
	public BenchmarkSpinner( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 32.0f );
			m_context.camera.zoom = 42.0f;
		}

		// toiCalls = 0;
		// toiHitCount = 0;

		CreateSpinner( m_worldId );
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_stepCount == 1000 && false )
		{
			// 0.1 : 46544, 25752
			// 0.25 : 5745, 1947
			// 0.5 : 2197, 660
			m_context.pause = true;
		}

		// DrawScreenTextLine( "toi calls, hits = %d, %d", toiCalls, toiHitCount );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkSpinner( context );
	}
}

public class BenchmarkRain : Sample
{
	public BenchmarkRain( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 110.0f );
			m_context.camera.zoom = 125.0f;
			m_context.enableSleep = true;
		}

		m_context.debugDraw.drawJoints = false;

		CreateRain( m_worldId );
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			StepRain( m_worldId, m_stepCount );
		}

		base.Step();

		if ( m_stepCount % 1000 == 0 )
		{
			m_stepCount += 0;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkRain( context );
	}
}

public class BenchmarkShapeDistance : Sample
{
	public BenchmarkShapeDistance( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.0f );
			m_context.camera.zoom = 3.0f;
		}

		{
			Vec2[] points = new Vec2[8];
			Rot q = MakeRot( 2.0f * Pi / 8.0f );
			Vec2 p = new Vec2( 0.5f, 0.0f );
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = RotateVector( q, points[i - 1] );
			}

			Hull hull = ComputeHull( points, 8 );
			m_polygonA = MakePolygon( ref hull, 0.0f );
		}

		{
			Vec2[] points = new Vec2[8];
			Rot q = MakeRot( 2.0f * Pi / 8.0f );
			Vec2 p = new Vec2( 0.5f, 0.0f );
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = RotateVector( q, points[i - 1] );
			}

			Hull hull = ComputeHull( points, 8 );
			m_polygonB = MakePolygon( ref hull, 0.1f );
		}

		m_transformAs = new WorldTransform[m_count];
		m_transformBs = new WorldTransform[m_count];
		m_outputs = new DistanceOutput[m_count];

		g_randomSeed = 42;
		for ( int i = 0; i < m_count; ++i )
		{
			m_transformAs[i] = new( RandomPos( -0.1f, 0.1f ), RandomRot() );
			m_transformBs[i] = new( RandomPos( 0.25f, 2.0f ), RandomRot() );
		}

		m_drawIndex = 0;
		m_minMilliseconds = float.MaxValue;
	}

	public override void Dispose()
	{
		base.Dispose();
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderInt( "draw index", ref m_drawIndex, 0, m_count - 1 );
		ImGui.PopItemWidth();

		return true;
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			DistanceInput input = default;
			input.proxyA = MakeProxy( m_polygonA.vertices, m_polygonA.count, m_polygonA.radius );
			input.proxyB = MakeProxy( m_polygonB.vertices, m_polygonB.count, m_polygonB.radius );
			input.useRadii = true;
			int totalIterations = 0;

			ulong start = GetTicks();
			for ( int i = 0; i < m_count; ++i )
			{
				SimplexCache cache = default;
				input.transform = InvMulWorldTransforms( m_transformAs[i], m_transformBs[i] );
				m_outputs[i] = ComputeShapeDistance( ref input, ref cache );
				totalIterations += m_outputs[i].iterations;
			}

			float ms = GetMilliseconds( start );
			m_minMilliseconds = MinFloat( m_minMilliseconds, ms );

			DrawScreenTextLine( "count = %d", m_count );
			DrawScreenTextLine( "min ms = %g, ave us = %g", m_minMilliseconds, 1000.0f * m_minMilliseconds / (float)( m_count ) );
			DrawScreenTextLine( "average iterations = %g", totalIterations / (float)( m_count ) );
		}

		WorldTransform xfA = m_transformAs[m_drawIndex];
		WorldTransform xfB = m_transformBs[m_drawIndex];
		DistanceOutput output = m_outputs[m_drawIndex];

		// The query returns frame A results, project to world for drawing
		Pos worldPointA = TransformWorldPoint( xfA, output.pointA );
		Pos worldPointB = TransformWorldPoint( xfA, output.pointB );
		Vec2 worldNormal = RotateVector( xfA.q, output.normal );

		DrawSolidPolygon( m_context.draw, xfA, m_polygonA.vertices, m_polygonA.count, m_polygonA.radius, HexColor.colorBox2DGreen );
		DrawSolidPolygon( m_context.draw, xfB, m_polygonB.vertices, m_polygonB.count, m_polygonB.radius, HexColor.colorBox2DBlue );
		DrawLine( m_context.draw, worldPointA, worldPointB, HexColor.colorDimGray );
		DrawPoint( m_context.draw, worldPointA, 10.0f, HexColor.colorWhite );
		DrawPoint( m_context.draw, worldPointB, 10.0f, HexColor.colorWhite );
		DrawLine( m_context.draw, worldPointA, worldPointA + 0.5f * worldNormal, HexColor.colorYellow );
		DrawScreenTextLine( "distance = %g", output.distance );

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkShapeDistance( context );
	}

	public const int m_count = m_isDebug ? 100 : 10000;
	public WorldTransform[] m_transformAs;
	public WorldTransform[] m_transformBs;
	public DistanceOutput[] m_outputs;
	public Polygon m_polygonA;
	public Polygon m_polygonB;
	public float m_minMilliseconds;
	public int m_drawIndex;
}

public class ShapeUserData
{
	public int row;
	public bool active;
}

public class BenchmarkSensor : Sample
{
	public BenchmarkSensor( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 105.0f );
			m_context.camera.zoom = 125.0f;
		}

		World_SetCustomFilterCallback( m_worldId, FilterFcn, this );

		m_activeSensor.row = 0;
		m_activeSensor.active = true;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );

		{
			float gridSize = 3.0f;

			ShapeDef shapeDefInner = DefaultShapeDef();
			shapeDefInner.isSensor = true;
			shapeDefInner.enableSensorEvents = true;
			shapeDefInner.userData = m_activeSensor;

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				Polygon box = MakeOffsetBox( 0.45f * gridSize, 0.45f * gridSize, new( x, y ), Rot_identity );
				CreatePolygonShape( groundId, ref shapeDefInner, ref box );
				x += gridSize;
			}
		}

		g_randomSeed = 42;

		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;

		float yStart = 10.0f;
		m_filterRow = m_rowCount >> 1;

		for ( int j = 0; j < m_rowCount; ++j )
		{
			m_passiveSensors[j].row = j;
			m_passiveSensors[j].active = false;
			shapeDef.userData = m_passiveSensors[j];

			if ( j == m_filterRow )
			{
				shapeDef.enableCustomFiltering = true;
				shapeDef.material.customColor = (uint)HexColor.colorFuchsia;
			}
			else
			{
				shapeDef.enableCustomFiltering = false;
				shapeDef.material.customColor = 0;
			}

			float y = j * shift + yStart;
			for ( int i = 0; i < m_columnCount; ++i )
			{
				float x = i * shift - xCenter;
				Polygon box = MakeOffsetRoundedBox( 0.5f, 0.5f, new( x, y ), Rot_identity, 0.1f );
				CreatePolygonShape( groundId, ref shapeDef, ref box );
			}
		}

		m_maxBeginCount = 0;
		m_maxEndCount = 0;
		m_lastStepCount = 0;
	}

	public void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = new Vec2( 0.0f, -5.0f );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		Circle circle = new Circle { center = new( 0.0f, 0.0f ), radius = 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			// stagger bodies to avoid bunching up events into a single update
			float yOffset = RandomFloatRange( -1.0f, 1.0f );
			bodyDef.position = new Pos { x = shift * i - xCenter, y = y + yOffset };
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreateCircleShape( bodyId, ref shapeDef, ref circle );
		}
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		SortedSet<BodyId> zombies = new( Comparer<BodyId>.Create( CppCompat.Compare ) );

		SensorEvents events = World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			ref SensorBeginTouchEvent @event = ref events.beginEvents[i];
			// shapes on begin touch are always valid

			ShapeUserData? userData = (ShapeUserData?)( Shape_GetUserData( @event.sensorShapeId ) );

			if ( userData.active )
			{
				zombies.Add( Shape_GetBody( @event.visitorShapeId ) );
			}
			else
			{
				// Check custom filter correctness
				Debug.Assert( userData.row != m_filterRow );

				// Modify color while overlapped with a sensor
				SurfaceMaterial surfaceMaterial = Shape_GetSurfaceMaterial( @event.visitorShapeId );
				surfaceMaterial.customColor = (uint)HexColor.colorLime;
				Shape_SetSurfaceMaterial( @event.visitorShapeId, ref surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			ref SensorEndTouchEvent @event = ref events.endEvents[i];
			if ( Shape_IsValid( @event.visitorShapeId ) == false )
			{
				continue;
			}

			// Restore color to default
			SurfaceMaterial surfaceMaterial = Shape_GetSurfaceMaterial( @event.visitorShapeId );
			surfaceMaterial.customColor = 0;
			Shape_SetSurfaceMaterial( @event.visitorShapeId, ref surfaceMaterial );
		}

		foreach ( BodyId bodyId in zombies )
		{
			DestroyBody( bodyId );
		}

		int delay = 0x1F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 10.0f + m_rowCount * 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = MaxInt( events.endCount, m_maxEndCount );
		DrawScreenTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawScreenTextLine( "max end touch events = %d", m_maxEndCount );
	}

	public bool Filter( ShapeId idA, ShapeId idB )
	{
		ShapeUserData? userData = null;
		if ( Shape_IsSensor( idA ) )
		{
			userData = (ShapeUserData?)Shape_GetUserData( idA );
		}
		else if ( Shape_IsSensor( idB ) )
		{
			userData = (ShapeUserData?)Shape_GetUserData( idB );
		}

		if ( userData != null )
		{
			return userData.active == true || userData.row != m_filterRow;
		}

		return true;
	}

	public static bool FilterFcn( ShapeId idA, ShapeId idB, object? context )
	{
		BenchmarkSensor? self = (BenchmarkSensor?)context;
		return self.Filter( idA, idB );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkSensor( context );
	}

	public const int m_columnCount = 40;
	public const int m_rowCount = 40;
	public int m_maxBeginCount;
	public int m_maxEndCount;
	public ShapeUserData[] m_passiveSensors = NewArray<ShapeUserData>( m_rowCount );
	public ShapeUserData m_activeSensor = new();
	public int m_lastStepCount;
	public int m_filterRow;
}

public class BenchmarkCapacity : Sample
{
	public BenchmarkCapacity( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 150.0f );
			m_context.camera.zoom = 200.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position.y = -5.0f;
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			Polygon box = MakeBox( 800.0f, 5.0f );
			ShapeDef shapeDef = DefaultShapeDef();
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		m_square = MakeSquare( 0.5f );
		m_done = false;
		m_reachCount = 0;
	}

	public override void Step(  )
	{
		base.Step();

		float millisecondLimit = 20.0f;

		Profile profile = World_GetProfile( m_worldId );
		if ( profile.step > millisecondLimit )
		{
			m_reachCount += 1;
			if ( m_reachCount > 60 )
			{
				// Hit the millisecond limit 60 times in a row
				m_done = true;
			}
		}
		else
		{
			m_reachCount = 0;
		}

		if ( m_done == true )
		{
			return;
		}

		if ( ( m_stepCount & 0x1F ) != 0x1F )
		{
			return;
		}

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position.y = 200.0f;

		ShapeDef shapeDef = DefaultShapeDef();

		int count = 200;
		float x = -1.0f * count;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position.x = x;
			bodyDef.position.y += 0.5f;

			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
			CreatePolygonShape( bodyId, ref shapeDef, ref m_square );

			x += 2.0f;
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkCapacity( context );
	}

	public Polygon m_square;
	public int m_reachCount;
	public bool m_done;
}

public class BenchmarkJunkyard : Sample
{
	public BenchmarkJunkyard( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 8.0f, 25.0f );
			m_context.camera.zoom = 60.0f;
		}

		CreateJunkyard( m_worldId );
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			StepJunkyard( m_worldId, m_stepCount );
		}

		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkJunkyard( context );
	}
}

public class BenchmarkQueries : Sample
{
	public BenchmarkQueries( SampleContext context ) : base( context )
	{
		CreateQueries( m_worldId );

		if ( m_context.restart == false )
		{
			float extent = GetQueryBenchmarkExtent();
			m_context.camera.center = new( 0.5f * extent, 0.5f * extent );
			m_context.camera.zoom = 0.55f * extent;
		}

		m_drawIndex = 0;
		m_minTime = 1e6f;
		m_lastTime = 0.0f;
		m_hitCount = 0;
		m_nodeVisits = 0;
		m_leafVisits = 0;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );
		ImGui.SliderInt( "draw", ref m_drawIndex, 0, GetQueryBenchmarkCount() - 1, "%d" );
		ImGui.PopItemWidth();
		return false;
	}

	public override void Step(  )
	{
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			TreeStats before = GetQueryBenchmarkStats();
			ulong ticks = GetTicks();
			m_hitCount = (int)StepQueries( m_worldId, m_stepCount );
			m_lastTime = GetMilliseconds( ticks );
			m_minTime = MinFloat( m_minTime, m_lastTime );
			TreeStats after = GetQueryBenchmarkStats();
			m_nodeVisits = after.nodeVisits - before.nodeVisits;
			m_leafVisits = after.leafVisits - before.leafVisits;
		}

		base.Step();

		Pos origin = default;
		Vec2 translation = default;
		GetQueryBenchmarkRay( m_drawIndex, ref origin, ref translation );
		Pos end = origin + translation;
		DrawLine( m_context.draw, origin, end, HexColor.colorWhite );
		DrawPoint( m_context.draw, origin, 5.0f, HexColor.colorGreen );
		DrawPoint( m_context.draw, end, 5.0f, HexColor.colorRed );

		// The overlap box the same query runs
		AABB box = new AABB { lowerBound = new( RoundDownFloat( origin.x - 5.0f ), RoundDownFloat( origin.y - 5.0f ) ), upperBound = new( RoundUpFloat( origin.x + 5.0f ), RoundUpFloat( origin.y + 5.0f ) ) };
		DrawBounds( m_context.draw, box, HexColor.colorWhite );

		DrawScreenTextLine( "%d queries of each kind and filter, %d hits", GetQueryBenchmarkCount(), m_hitCount );
		DrawScreenTextLine( "node visits = %d, leaf visits = %d", m_nodeVisits, m_leafVisits );
		DrawScreenTextLine( "queries ms = %.3f, min = %.3f", m_lastTime, m_minTime );
	}

	public static Sample Create( SampleContext context )
	{
		return new BenchmarkQueries( context );
	}

	public int m_drawIndex;
	public float m_minTime;
	public float m_lastTime;
	public int m_hitCount;
	public int m_nodeVisits;
	public int m_leafVisits;
}
