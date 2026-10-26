// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	public static PlaneSolverResult SolvePlanes( Vec2 targetDelta, Span<CollisionPlane> planes, int count )
	{
		for ( int i = 0; i < count; ++i )
		{
			planes[i].push = 0.0f;
		}

		Vec2 delta = targetDelta;
		float tolerance = LinearSlop;

		int iteration;
		for ( iteration = 0; iteration < 20; ++iteration )
		{
			float totalPush = 0.0f;
			for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
			{
				ref CollisionPlane plane = ref planes[planeIndex];
				// Add slop to prevent jitter
				float separation = PlaneSeparation( plane.plane, delta ) + LinearSlop;
				// if (separation > 0.0f)
				//{
				//	continue;
				// }

				float push = -separation;

				// Clamp accumulated push
				float accumulatedPush = plane.push;
				plane.push = ClampFloat( plane.push + push, 0.0f, plane.pushLimit );
				push = plane.push - accumulatedPush;
				delta = MulAdd( delta, push, plane.plane.normal );

				// Track maximum push for convergence
				totalPush += AbsFloat( push );
			}

			if ( totalPush < tolerance )
			{
				break;
			}
		}

		return new PlaneSolverResult { delta = delta, iterationCount = iteration };
	}

	public static Vec2 ClipVector( Vec2 vector, ReadOnlySpan<CollisionPlane> planes, int count )
	{
		Vec2 v = vector;

		for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
		{
			ref readonly CollisionPlane plane = ref planes[planeIndex];
			if ( plane.push == 0.0f || plane.clipVelocity == false )
			{
				continue;
			}

			v = MulSub( v, MinFloat( 0.0f, Dot( v, plane.plane.normal ) ), plane.plane.normal );
		}

		return v;
	}

}
