// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class DistanceTests
{
	private static int SegmentDistanceTest()
	{
		Vec2 p1 = new(  -1.0f, -1.0f  );
		Vec2 q1 = new(  -1.0f, 1.0f  );
		Vec2 p2 = new(  2.0f, 0.0f  );
		Vec2 q2 = new(  1.0f, 0.0f  );

		SegmentDistanceResult result = ComputeSegmentDistance( p1, q1, p2, q2 );

		ENSURE_SMALL( result.fraction1 - 0.5f, FLT_EPSILON );
		ENSURE_SMALL( result.fraction2 - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( result.closest1.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( result.closest1.y, FLT_EPSILON );
		ENSURE_SMALL( result.closest2.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( result.closest2.y, FLT_EPSILON );
		ENSURE_SMALL( result.distanceSquared - 4.0f, FLT_EPSILON );

		return 0;
	}

	private static int ShapeDistanceTest()
	{
		Vec2[] vas = { new Vec2( -1.0f, -1.0f ), new Vec2( 1.0f, -1.0f ), new Vec2( 1.0f, 1.0f ), new Vec2( -1.0f, 1.0f ) };

		Vec2[] vbs = { new Vec2( 2.0f, -1.0f ),
			new Vec2( 2.0f, 1.0f ), };

		DistanceInput input = default;
		input.proxyA = MakeProxy( vas, vas.Length, 0.0f );
		input.proxyB = MakeProxy( vbs, vbs.Length, 0.0f );
		input.transform = Transform_identity;
		input.useRadii = false;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance(ref input,  ref cache );

		ENSURE_SMALL( output.distance - 1.0f, FLT_EPSILON );

		return 0;
	}

	private static int ShapeCastTest()
	{
		Vec2[] vas = { new Vec2( -1.0f, -1.0f ), new Vec2( 1.0f, -1.0f ), new Vec2( 1.0f, 1.0f ), new Vec2( -1.0f, 1.0f ) };

		Vec2[] vbs = { new Vec2( 2.0f, -1.0f ),
			new Vec2( 2.0f, 1.0f ), };

		ShapeCastPairInput input = default;
		input.proxyA = MakeProxy( vas, vas.Length, 0.0f );
		input.proxyB = MakeProxy( vbs, vbs.Length, 0.0f );
		input.transform = Transform_identity;
		input.translationB = new Vec2( -2.0f, 0.0f );
		input.maxFraction = 1.0f;
		input.canEncroach = false;

		CastOutput output = CastShape( ref input );

		ENSURE( output.hit );
		ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

		return 0;
	}

	// A thin plank and a point a linear slop off the middle of its short end face. The closest feature
	// is that end face, so GJK finishes on a two point simplex spanning it.
	//
	// SolveSimplex2 hands back a search direction of magnitude 2 * distance * edge^2, which the
	// caller weighs against an absolute epsilon. That test is cubic in length, so it gives out on
	// short edges: below an edge of about 3.5 mm at default length units it declares the shapes
	// overlapped and returns a zero distance with no normal, though they are plainly apart.
	//
	// Measured cutoff matches 2 * distance * edge^2 < FLT_EPSILON to three digits, and it tracks the
	// contact edge alone. Widening the plank from 1 m to 100 m changes nothing.
	private static int ShapeDistanceShortEdgeTest()
	{
		float halfThickness = 0.0016f;

		Vec2[] corners = { new Vec2( -0.5f, -halfThickness ),
			new Vec2( 0.5f, -halfThickness ),
			new Vec2( 0.5f, halfThickness ),
			new Vec2( -0.5f, halfThickness ), };

		Vec2 point = new(  0.5f + LinearSlop, 0.0f  );

		DistanceInput input = default;
		input.proxyA = MakeProxy( corners, corners.Length, 0.0f );
		input.proxyB = MakeProxy( ref point, 1, 0.0f );
		input.transform = Transform_identity;
		input.useRadii = false;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( ref input, ref cache );

		ENSURE( output.distance > 0.0f );
		ENSURE( IsNormalized( output.normal ) );
		ENSURE_SMALL( output.distance - LinearSlop, 1e-6f );

		return 0;
	}

	// The same false overlap seen through CastShape, which is how it was reported.
	//
	// Conservative advancement stops the cores a target apart and target is at least a linear slop, so
	// the query can never legitimately answer overlap after the first iteration. Here iteration 1 does,
	// and CastShape has no fallback: it trips
	// B2_ASSERT( distanceOutput.distance > 0.0f && IsNormalized( distanceOutput.normal ) ).
	//
	// Enable once ComputeShapeDistance stops reporting the false overlap. The assert aborts the run, so
	// leaving it on would take the rest of the suite with it.
	private const int B2_SHORT_EDGE_CAST_REPRO = 0;
#if B2_SHORT_EDGE_CAST_REPRO
	private static int ShapeCastShortEdgeTest()
	{
		float halfThickness = 0.0002f;

		Vec2[] corners = { new Vec2( -0.5f, -halfThickness ),
			new Vec2( 0.5f, -halfThickness ),
			new Vec2( 0.5f, halfThickness ),
			new Vec2( -0.5f, halfThickness ), };

		// falls past the end of the plank and drifts into it, so the second query lands on the end face
		Vec2 start = new(  0.506f, 0.0035f  );

		ShapeCastPairInput input = default;
		input.proxyA = MakeProxy( corners, corners.Length, 0.0f );
		input.proxyB = MakeProxy( ref start, 1, 0.0f );
		input.transform = Transform_identity;
		input.translationB = new Vec2( -0.01f, -0.5f );
		input.maxFraction = 1.0f;
		input.canEncroach = false;

		CastOutput output = CastShape( ref input );

		ENSURE( output.hit == false || IsNormalized( output.normal ) );

		return 0;
	}
#endif

	private static int TimeOfImpactTest()
	{
		Vec2[] vas = { new( -1.0f, -1.0f ), new( 1.0f, -1.0f ), new( 1.0f, 1.0f ), new( -1.0f, 1.0f ) };

		Vec2[] vbs = { new( 2.0f, -1.0f ),
			new( 2.0f, 1.0f ), };

		TOIInput input = default;
		input.proxyA = MakeProxy( vas, vas.Length, 0.0f );
		input.proxyB = MakeProxy( vbs, vbs.Length, 0.0f );
		input.sweepA = new Sweep { localCenter = Vec2_zero, c1 = Vec2_zero, c2 = Vec2_zero, q1 = Rot_identity, q2 = Rot_identity };
		input.sweepB = new Sweep { localCenter = Vec2_zero, c1 = Vec2_zero, c2 = new Vec2( -2.0f, 0.0f ), q1 = Rot_identity, q2 = Rot_identity };
		input.maxFraction = 1.0f;

		TOIOutput output = ComputeTimeOfImpact( ref input );

		ENSURE( output.state == TOIState.toiStateHit );
		ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

		return 0;
	}

	internal static int DistanceTest()
	{
		RUN_SUBTEST( SegmentDistanceTest );
		RUN_SUBTEST( ShapeDistanceTest );
		RUN_SUBTEST( ShapeCastTest );
		RUN_SUBTEST( ShapeDistanceShortEdgeTest );
#if B2_SHORT_EDGE_CAST_REPRO
		RUN_SUBTEST( ShapeCastShortEdgeTest );
#endif
		RUN_SUBTEST( TimeOfImpactTest );

		return 0;
	}

}
