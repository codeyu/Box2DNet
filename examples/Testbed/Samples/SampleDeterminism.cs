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

public static partial class Helpers_sample_determinism
{
	// This sample provides a visual representation of the cross platform determinism unit test.
	// The scenario is designed to produce a chaotic result engaging:
	// - continuous collision
	// - joint limits (approximate atan2)
	// - MakeRot (approximate sin/cos)
	// Once all the bodies go to sleep the step counter and transform hash is emitted which
	// can then be transferred to the unit test and tested in GitHub build actions.
	// See CrossPlatformTest in the unit tests.



	// Records a snapshot every few steps of a full run, then rolls the world back to any of them.
	// Resimulating from a rollback point has to reach the same sleep step and transform hash as the
	// original run. This demonstrates rollback determinism.
}

public class FallingHinges : Sample
{
	public FallingHinges( SampleContext context ) : base( context )
	{
		if ( m_context.restart == false )
		{
			m_context.camera.center = new( 0.0f, 7.5f );
			m_context.camera.zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
	}

	public override void Dispose()
	{
		DestroyFallingHinges( ref m_data );
		base.Dispose();
	}

	public static bool OverlapFcn( ShapeId unused17, object? unused18 )
	{
		return true;
	}

	public static float CastFcn( ShapeId unused19, Pos unused20, Vec2 unused21, float fraction, object? unused22 )
	{
		return 1.0f;
	}

	public override void Step(  )
	{
		base.Step();

		if ( m_context.pause == false && m_done == false )
		{
			m_done = UpdateFallingHinges( m_worldId, ref m_data );

			QueryFilter filter = DefaultQueryFilter();
			AABB scanBox = new AABB { lowerBound = new( 5.0f, 1.0f ), upperBound = new( 7.0f, 2.5f ) };
			World_OverlapAABB( m_worldId, Pos_zero, scanBox, filter, OverlapFcn, null );

			Pos origin = new Pos { x = 0.0f, y = 12.0f };
			Vec2 translation = new Vec2( 0.0f, -14.0f );

			if ( m_stepCount == 30 )
			{
				World_CastRayClosest( m_worldId, origin, translation, filter );
			}

			if ( m_stepCount < 5 || 100 < m_stepCount )
			{
				origin = new Pos { x = -10.0f, y = 2.0f };
				translation = new Vec2( 20.0f, 0.0f );
				World_CastRay( m_worldId, origin, translation, filter, CastFcn, null );
			}

			if ( m_done )
			{
				Console.Write( Fmt( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash ) );

			}
		}
		else
		{
			DrawScreenTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
		}
	}

	public static Sample Create( SampleContext context )
	{
		return new FallingHinges( context );
	}

	public FallingHingeData m_data;
	public bool m_done;
}
