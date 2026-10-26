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

public static partial class Helpers_sample_robustness
{
	// Pyramid with heavy box on top



	// Big box on small boxes



	// Big box on small triangles






	// A pyramid of 5cm squares. Stacking tiny objects is challenging for physics engines due to rotational effects.
	// This is also challenging for Box2D because of the AABB margin and linear slop are close to the shape size. This
	// leads to many collision pairs and some shape overlap.



	// High gravity and high mass ratio. This shows how to tune contact and joint stiffness values to
	// achieve an improved result at a high sub-step count.
	// There is still a fair bit of bounce with some settings.



	// Ensure prismatic joint stability when highly distorted
}

public class HighMassRatio1 : Sample
{
	public HighMassRatio1( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 3.0f, 14.0f );
			m_context.camera.zoom = 25.0f;
		}

		float extent = 1.0f;

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 50.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			Polygon box = MakeBox( extent, extent );
			ShapeDef shapeDef = DefaultShapeDef();

			for ( int j = 0; j < 3; ++j )
			{
				int count = 10;
				float offset = -20.0f * extent + 2.0f * ( count + 1.0f ) * extent * j;
				float y = extent;
				while ( count > 0 )
				{
					for ( int i = 0; i < count; ++i )
					{
						float coeff = i - 0.5f * count;

						float yy = count == 1 ? y + 2.0f : y;
						bodyDef.position = new Pos { x = 2.0f * coeff * extent + offset, y = yy };
						BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

						shapeDef.density = count == 1 ? ( j + 1.0f ) * 100.0f : 1.0f;
						CreatePolygonShape( bodyId, ref shapeDef, ref box );
					}

					--count;
					y += 2.0f * extent;
				}
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new HighMassRatio1( context );
	}
}

public class HighMassRatio2 : Sample
{
	public HighMassRatio2( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 16.5f );
			m_context.camera.zoom = 25.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 50.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			ShapeDef shapeDef = DefaultShapeDef();

			float extent = 1.0f;
			Polygon smallBox = MakeBox( 0.5f * extent, 0.5f * extent );
			Polygon bigBox = MakeBox( 10.0f * extent, 10.0f * extent );

			{
				bodyDef.position = new Pos { x = -9.0f * extent, y = 0.5f * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref smallBox );
			}

			{
				bodyDef.position = new Pos { x = 9.0f * extent, y = 0.5f * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref smallBox );
			}

			{
				bodyDef.position = new Pos { x = 0.0f, y = ( 10.0f + 16.0f ) * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref bigBox );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new HighMassRatio2( context );
	}
}

public class HighMassRatio3 : Sample
{
	public HighMassRatio3( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 16.5f );
			m_context.camera.zoom = 25.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 50.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;
			ShapeDef shapeDef = DefaultShapeDef();

			float extent = 1.0f;
			Vec2[] points = { new( -0.5f * extent, 0.0f ), new( 0.5f * extent, 0.0f ), new( 0.0f, 1.0f * extent ) };
			Hull hull = ComputeHull( points, 3 );
			Polygon smallTriangle = MakePolygon( ref hull, 0.0f );
			Polygon bigBox = MakeBox( 10.0f * extent, 10.0f * extent );

			{
				bodyDef.position = new Pos { x = -9.0f * extent, y = 0.5f * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref smallTriangle );
			}

			{
				bodyDef.position = new Pos { x = 9.0f * extent, y = 0.5f * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref smallTriangle );
			}

			{
				bodyDef.position = new Pos { x = 0.0f, y = ( 10.0f + 4.0f ) * extent };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
				CreatePolygonShape( bodyId, ref shapeDef, ref bigBox );
			}
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new HighMassRatio3( context );
	}
}

public class OverlapRecovery : Sample
{
	public OverlapRecovery( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 2.5f );
			m_context.camera.zoom = 3.75f;
		}

		m_bodyIds = null;
		m_bodyCount = 0;
		m_baseCount = 4;
		m_overlap = 0.25f;
		m_extent = 0.5f;
		m_speed = 3.0f;
		m_hertz = 30.0f;
		m_dampingRatio = 10.0f;

		BodyDef bodyDef = DefaultBodyDef();
		BodyId groundId = CreateBody( m_worldId, ref bodyDef );
		ShapeDef shapeDef = DefaultShapeDef();
		Segment segment = new Segment { point1 = new( -40.0f, 0.0f ), point2 = new( 40.0f, 0.0f ) };
		CreateSegmentShape( groundId, ref shapeDef, ref segment );

		CreateScene();
	}

	public override void Dispose()
	{
		base.Dispose();
	}

	public void CreateScene(  )
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			DestroyBody( m_bodyIds[i] );
		}

		World_SetContactTuning( m_worldId, m_hertz, m_dampingRatio, m_speed );

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;

		Polygon box = MakeBox( m_extent, m_extent );
		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1.0f;

		m_bodyCount = m_baseCount * ( m_baseCount + 1 ) / 2;
		Array.Resize( ref m_bodyIds, m_bodyCount );

		int bodyIndex = 0;
		float fraction = 1.0f - m_overlap;
		float y = m_extent;
		for ( int i = 0; i < m_baseCount; ++i )
		{
			float x = fraction * m_extent * ( i - m_baseCount );
			for ( int j = i; j < m_baseCount; ++j )
			{
				bodyDef.position = new Pos { x = x, y = y };
				BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

				CreatePolygonShape( bodyId, ref shapeDef, ref box );

				m_bodyIds[bodyIndex++] = bodyId;

				x += 2.0f * fraction * m_extent;
			}

			y += 2.0f * fraction * m_extent;
		}

		Debug.Assert( bodyIndex == m_bodyCount );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		changed = changed || ImGui.SliderFloat( "Extent", ref m_extent, 0.1f, 1.0f, "%.1f" );
		changed = changed || ImGui.SliderInt( "Base Count", ref m_baseCount, 1, 10 );
		changed = changed || ImGui.SliderFloat( "Overlap", ref m_overlap, 0.0f, 1.0f, "%.2f" );
		changed = changed || ImGui.SliderFloat( "Speed", ref m_speed, 0.0f, 10.0f, "%.1f" );
		changed = changed || ImGui.SliderFloat( "Hertz", ref m_hertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui.SliderFloat( "Damping Ratio", ref m_dampingRatio, 0.0f, 20.0f, "%.1f" );
		changed = changed || ImGui.Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new OverlapRecovery( context );
	}

	public BodyId[] m_bodyIds;
	public int m_bodyCount;
	public int m_baseCount;
	public float m_overlap;
	public float m_extent;
	public float m_speed;
	public float m_hertz;
	public float m_dampingRatio;
}

public class TinyPyramid : Sample
{
	public TinyPyramid( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 0.8f );
			m_context.camera.zoom = 1.0f;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );
			ShapeDef shapeDef = DefaultShapeDef();
			Polygon box = MakeOffsetBox( 5.0f, 1.0f, new( 0.0f, -1.0f ), Rot_identity );
			CreatePolygonShape( groundId, ref shapeDef, ref box );
		}

		{
			m_extent = 0.025f;
			int baseCount = 30;

			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.type = BodyType.dynamicBody;

			ShapeDef shapeDef = DefaultShapeDef();

			Polygon box = MakeSquare( m_extent );

			for ( int i = 0; i < baseCount; ++i )
			{
				float y = ( 2.0f * i + 1.0f ) * m_extent;

				for ( int j = i; j < baseCount; ++j )
				{
					float x = ( i + 1.0f ) * m_extent + 2.0f * ( j - i ) * m_extent - baseCount * m_extent;
					bodyDef.position = new Pos { x = x, y = y };

					BodyId bodyId = CreateBody( m_worldId, ref bodyDef );
					CreatePolygonShape( bodyId, ref shapeDef, ref box );
				}
			}
		}
	}

	public override void Step(  )
	{
		DrawScreenTextLine( "%.1fcm squares", 200.0f * m_extent );
		base.Step();
	}

	public static Sample Create( SampleContext context )
	{
		return new TinyPyramid( context );
	}

	public float m_extent;
}

public class Cart : Sample
{
	public Cart( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 1.0f );
			m_context.camera.zoom = 1.5f;
			m_context.subStepCount = 12;
		}

		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = -1.0f };
			BodyId groundId = CreateBody( m_worldId, ref bodyDef );

			ShapeDef shapeDef = DefaultShapeDef();
			Polygon groundBox = MakeBox( 20.0f, 1.0f );
			CreatePolygonShape( groundId, ref shapeDef, ref groundBox );
		}

		World_SetGravity( m_worldId, new( 0, -22.0f ) );

		m_contactHertz = 240.0f;
		m_contactDampingRatio = 10.0f;
		m_contactSpeed = 0.5f;
		World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );

		m_constraintHertz = 240.0f;
		m_constraintDampingRatio = 0.0f;

		m_chassisId = new BodyId {  };
		m_wheelId1 = new BodyId {  };
		m_wheelId2 = new BodyId {  };
		m_jointId1 = new JointId {  };
		m_jointId2 = new JointId {  };

		CreateScene();
	}

	public void CreateScene(  )
	{
		if ( m_chassisId.IsNonNull )
		{
			DestroyBody( m_chassisId );
		}

		if ( m_wheelId1.IsNonNull )
		{
			DestroyBody( m_wheelId1 );
		}

		if ( m_wheelId2.IsNonNull )
		{
			DestroyBody( m_wheelId2 );
		}

		float yBase = 2.0f;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.position = new Pos { x = 0.0f, y = yBase };
		m_chassisId = CreateBody( m_worldId, ref bodyDef );

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.density = 1000.0f;

		Polygon box = MakeOffsetBox( 1.0f, 0.25f, new( 0.0f, 0.25f ), Rot_identity );
		CreatePolygonShape( m_chassisId, ref shapeDef, ref box );

		shapeDef = DefaultShapeDef();
		shapeDef.material.rollingResistance = 0.02f;
		shapeDef.density = 50.0f;

		Circle circle = new Circle { center = Vec2_zero, radius = 0.1f };
		bodyDef.position = new Pos { x = -0.9f, y = yBase - 0.15f };
		m_wheelId1 = CreateBody( m_worldId, ref bodyDef );
		CreateCircleShape( m_wheelId1, ref shapeDef, ref circle );

		bodyDef.position = new Pos { x = 0.9f, y = yBase - 0.15f };
		m_wheelId2 = CreateBody( m_worldId, ref bodyDef );
		CreateCircleShape( m_wheelId2, ref shapeDef, ref circle );

		RevoluteJointDef jointDef = DefaultRevoluteJointDef();
		jointDef.@base.constraintHertz = 120.0f;
		jointDef.@base.constraintDampingRatio = 0.0f;

		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_wheelId1;
		jointDef.@base.localFrameA.p = new( -0.9f, -0.15f );
		jointDef.@base.localFrameB.p = new( 0.0f, 0.0f );

		m_jointId1 = CreateRevoluteJoint( m_worldId, ref jointDef );
		Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );

		jointDef.@base.bodyIdA = m_chassisId;
		jointDef.@base.bodyIdB = m_wheelId2;
		jointDef.@base.localFrameA.p = new( 0.9f, -0.15f );
		jointDef.@base.localFrameB.p = new( 0.0f, 0.0f );

		m_jointId2 = CreateRevoluteJoint( m_worldId, ref jointDef );
		Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		ImGui.Text( "Contact" );
		changed = changed || ImGui.SliderFloat( "Hertz##contact", ref m_contactHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui.SliderFloat( "Damping Ratio##contact", ref m_contactDampingRatio, 0.0f, 100.0f, "%.f" );
		changed = changed || ImGui.SliderFloat( "Speed", ref m_contactSpeed, 0.0f, 5.0f, "%.1f" );

		if ( changed )
		{
			World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );
			CreateScene();
		}

		ImGui.Separator();

		changed = false;
		ImGui.Text( "Joint" );
		changed = changed || ImGui.SliderFloat( "Hertz##joint", ref m_constraintHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui.SliderFloat( "Damping Ratio##joint", ref m_constraintDampingRatio, 0.0f, 20.0f, "%.f" );

		ImGui.Separator();

		changed = changed || ImGui.Button( "Reset Scene" );

		if ( changed )
		{
			Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );
			Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			CreateScene();
		}

		ImGui.PopItemWidth();

		return true;
	}

	public static Sample Create( SampleContext context )
	{
		return new Cart( context );
	}

	public BodyId m_chassisId;
	public BodyId m_wheelId1;
	public BodyId m_wheelId2;
	public JointId m_jointId1;
	public JointId m_jointId2;

	public float m_contactHertz;
	public float m_contactDampingRatio;
	public float m_contactSpeed;
	public float m_constraintHertz;
	public float m_constraintDampingRatio;
}

public class MultiplePrismatic : Sample
{
	public MultiplePrismatic( SampleContext context ) : base( context )
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

		ShapeDef shapeDef = DefaultShapeDef();
		Polygon box = MakeBox( 0.5f, 0.5f );
		PrismaticJointDef jointDef = DefaultPrismaticJointDef();

		m_hertz = jointDef.@base.constraintHertz;
		m_dampingRatio = jointDef.@base.constraintDampingRatio;

		jointDef.@base.bodyIdA = groundId;
		jointDef.@base.localFrameA.p = new( 0.0f, 0.0f );
		jointDef.@base.localFrameB.p = new( 0.0f, -0.6f );
		jointDef.@base.drawScale = 1.0f;
		jointDef.lowerTranslation = -6.0f;
		jointDef.upperTranslation = 6.0f;
		jointDef.enableLimit = true;

		for ( int i = 0; i < m_count; ++i )
		{
			BodyDef bodyDef = DefaultBodyDef();
			bodyDef.position = new Pos { x = 0.0f, y = 0.6f + 1.2f * i };
			bodyDef.type = BodyType.dynamicBody;
			BodyId bodyId = CreateBody( m_worldId, ref bodyDef );

			CreatePolygonShape( bodyId, ref shapeDef, ref box );

			jointDef.@base.bodyIdB = bodyId;
			m_jointIds[i] = CreatePrismaticJoint( m_worldId, ref jointDef );

			jointDef.@base.bodyIdA = bodyId;
			jointDef.@base.localFrameA.p = new( 0.0f, 0.6f );
		}

		// Increase the mouse force
		m_mouseForceScale = 100000.0f;
	}

	public override bool DrawControls(  )
	{
		ImGui.PushItemWidth( 6.0f * ImGui.GetFontSize() );

		bool changed = false;
		ImGui.Text( "Multiple Prismatic" );
		changed |= ImGui.SliderFloat( "Hertz##contact", ref m_hertz, 0.0f, 480.0f, "%.f" );
		changed |= ImGui.SliderFloat( "Damping Ratio##contact", ref m_dampingRatio, 0.0f, 100.0f, "%.f" );

		if ( changed )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				Joint_SetConstraintTuning( m_jointIds[i], m_hertz, m_dampingRatio );
			}
		}

		ImGui.PopItemWidth();

		return true;
	}
	public static Sample Create( SampleContext context )
	{
		return new MultiplePrismatic( context );
	}

	public const int m_count = 6;

	public JointId[] m_jointIds = new JointId[m_count];
	public float m_hertz;
	public float m_dampingRatio;
}
