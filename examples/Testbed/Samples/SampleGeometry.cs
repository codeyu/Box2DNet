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

public class ConvexHull : Sample
{
	public const int e_count = MaxPolygonVertices;

	public ConvexHull( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.5f, 0.0f );
			m_context.camera.zoom = 25.0f * 0.3f;
		}

		m_generation = 0;
		m_auto = false;
		m_bulk = false;
		Generate();
	}

	public void Generate(  )
	{
#if false
		m_points[0] = new( 5.65314484f, 0.204832315f );
		m_points[1] = new( -5.65314484f, -0.204832315f );
		m_points[2] = new( 2.34463644f, 1.15731204f );
		m_points[3] = new( 0.0508846045f, 3.23230696f );
		m_points[4] = new( -5.65314484f, -0.204832315f );
		m_points[5] = new( -5.65314484f, -0.204832315f );
		m_points[6] = new( 3.73758054f, -1.11098099f );
		m_points[7] = new( 1.33504069f, -4.43795443f );

		m_count = e_count;
#elif false
		m_points[0] = new( -0.328125f, 0.179688f );
		m_points[1] = new( -0.203125f, 0.304688f );
		m_points[2] = new( 0.171875f, 0.304688f );
		m_points[3] = new( 0.359375f, 0.117188f );
		m_points[4] = new( 0.359375f, -0.195313f );
		m_points[5] = new( 0.234375f, -0.320313f );
		m_points[6] = new( -0.265625f, -0.257813f );
		m_points[7] = new( -0.328125f, -0.132813f );

		Hull hull = ComputeHull( m_points, 8 );
		bool valid = ValidateHull( ref hull );
		if ( valid == false )
		{
			Debug.Assert( valid );
		}

		m_count = e_count;
#else

		float angle = Pi * RandomFloat();
		Rot r = MakeRot( angle );

		Vec2 lowerBound = new Vec2( -4.0f, -4.0f );
		Vec2 upperBound = new Vec2( 4.0f, 4.0f );

		for ( int i = 0; i < e_count; ++i )
		{
			float x = 10.0f * RandomFloat();
			float y = 10.0f * RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			Vec2 v = Clamp( new( x, y ), lowerBound, upperBound );
			m_points[i] = RotateVector( r, v );
		}

		m_count = e_count;
#endif

		m_generation += 1;
	}

	public override void Keyboard( int key, int unused24, int unused25 )
	{
		switch ( key )
		{
			case GLFW_KEY_A:
				m_auto = !m_auto;
				break;

			case GLFW_KEY_B:
				m_bulk = !m_bulk;
				break;

			case GLFW_KEY_G:
				Generate();
				break;

			default:
				break;
		}
	}

	public override void Step(  )
	{
		base.Step();

		DrawScreenTextLine( "Options: generate(g), var(a), bulk(b)" );

		Hull hull = default;
		bool valid = false;
		float milliseconds = 0.0f;

		if ( m_bulk )
		{
#if true
			// defect hunting
			for ( int i = 0; i < 10000; ++i )
			{
				Generate();
				hull = ComputeHull( m_points, m_count );
				if ( hull.count == 0 )
				{
					// m_bulk = false;
					// break;
					continue;
				}

				valid = ValidateHull( ref hull );
				if ( valid == false || m_bulk == false )
				{
					m_bulk = false;
					break;
				}
			}
#else
			// performance
			Generate();
			Timer timer = default;
			for ( int i = 0; i < 1000000; ++i )
			{
				hull = ComputeHull( m_points, m_count );
			}
			valid = hull.count > 0;
			milliseconds = timer.GetMilliseconds();
#endif
		}
		else
		{
			if ( m_auto )
			{
				Generate();
			}

			hull = ComputeHull( m_points, m_count );
			if ( hull.count > 0 )
			{
				valid = ValidateHull( ref hull );
				if ( valid == false )
				{
					m_auto = false;
				}
			}
		}

		if ( valid == false )
		{
			DrawScreenTextLine( "generation = %d, FAILED", m_generation );
		}
		else
		{
			DrawScreenTextLine( "generation = %d, count = %d", m_generation, hull.count );
		}

		if ( milliseconds > 0.0f )
		{
			DrawScreenTextLine( "milliseconds = %g", milliseconds );
		}

		DrawPolygon( m_draw, WorldTransform_identity, hull.points, hull.count, HexColor.colorGray );

		for ( int i = 0; i < m_count; ++i )
		{
			DrawPoint( m_draw, ToPos( m_points[i] ), 5.0f, HexColor.colorBlue );
			DrawString( m_draw, m_camera, ToPos( Add( m_points[i], new( 0.1f, 0.1f ) ) ), HexColor.colorWhite, "%d", i );
		}

		for ( int i = 0; i < hull.count; ++i )
		{
			DrawPoint( m_draw, ToPos( hull.points[i] ), 6.0f, HexColor.colorGreen );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new ConvexHull( context );
	}

	public Vec2[] m_points = new Vec2[MaxPolygonVertices];
	public int m_count;
	public int m_generation;
	public bool m_auto;
	public bool m_bulk;
}
