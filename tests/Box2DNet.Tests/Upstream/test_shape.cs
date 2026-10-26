// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class ShapeTests
{
	static Capsule capsule = new Capsule { center1 = new Vec2 { x = -1.0f, y = 0.0f }, center2 = new Vec2 { x = 1.0f, y = 0.0f }, radius = 1.0f };
	static Circle circle = new Circle { center = new Vec2 { x = 1.0f, y = 0.0f }, radius = 1.0f };
	static Polygon box;
	static Segment segment = new Segment { point1 = new Vec2 { x = 0.0f, y = 1.0f }, point2 = new Vec2 { x = 0.0f, y = -1.0f } };

	private const int N = 4;
	private static int ShapeMassTest()
	{
		{
			MassData md = ComputeCircleMass( ref circle, 1.0f );
			ENSURE_SMALL( md.mass - Pi, FLT_EPSILON );
			ENSURE( md.center.x == 1.0f && md.center.y == 0.0f );
			ENSURE_SMALL( md.rotationalInertia - 0.5f * Pi, FLT_EPSILON );
		}

		{
			float radius = capsule.radius;
			float length = Distance( capsule.center1, capsule.center2 );

			MassData md = ComputeCapsuleMass( ref capsule, 1.0f );

			// Box that full contains capsule
			Polygon r = MakeBox( radius + 0.5f * length, radius );
			MassData mdUpper = ComputePolygonMass( ref r, 1.0f );

			// Approximate capsule using convex hull
			Span<Vec2> points = stackalloc Vec2[2 * N];
			float d = Pi / ( N - 1.0f );
			float angle = -0.5f * Pi;
			for ( int i = 0; i < N; ++i )
			{
				points[i].x = 1.0f + radius * cosf( angle );
				points[i].y = radius * sinf( angle );
				angle += d;
			}

			angle = 0.5f * Pi;
			for ( int i = N; i < 2 * N; ++i )
			{
				points[i].x = -1.0f + radius * cosf( angle );
				points[i].y = radius * sinf( angle );
				angle += d;
			}

			Hull hull = ComputeHull( points, 2 * N );
			Polygon ac = MakePolygon( ref hull, 0.0f );
			MassData mdLower = ComputePolygonMass( ref ac, 1.0f );

			ENSURE( mdLower.mass < md.mass && md.mass < mdUpper.mass );
			ENSURE( mdLower.rotationalInertia < md.rotationalInertia && md.rotationalInertia < mdUpper.rotationalInertia );
		}

		{
			MassData md = ComputePolygonMass( ref box, 1.0f );
			ENSURE_SMALL( md.mass - 4.0f, FLT_EPSILON );
			ENSURE_SMALL( md.center.x, FLT_EPSILON );
			ENSURE_SMALL( md.center.y, FLT_EPSILON );
			ENSURE_SMALL( md.rotationalInertia - 8.0f / 3.0f, 2.0f * FLT_EPSILON );
		}

		{
			Vec2 offset = new( 0.4f, -0.7f );
			Polygon b1 = MakeBox( 0.25f, 0.5f );
			Polygon b2 = MakeOffsetBox( 0.25f, 0.5f, offset, Rot_identity );

			MassData m1 = ComputePolygonMass( ref b1, 1.0f );
			MassData m2 = ComputePolygonMass( ref b2, 1.0f );

			ENSURE_SMALL( m1.mass - m2.mass, FLT_EPSILON );
			ENSURE_SMALL( m1.rotationalInertia - m2.rotationalInertia, FLT_EPSILON );
			ENSURE_SMALL( m2.center.x - offset.x, FLT_EPSILON );
			ENSURE_SMALL( m2.center.y - offset.y, FLT_EPSILON );
		}

		return 0;
	}

	private static int ShapeAABBTest()
	{
		{
			AABB b = ComputeCircleBounds( ref circle, WorldTransform_identity );
			ENSURE_SMALL( b.lowerBound.x, FLT_EPSILON );
			ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.x - 2.0f, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		}

		{
			AABB b = ComputePolygonBounds( ref box, WorldTransform_identity );
			ENSURE_SMALL( b.lowerBound.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.x - 1.0f, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		}

		{
			AABB b = ComputeSegmentBounds( ref segment, WorldTransform_identity );
			ENSURE_SMALL( b.lowerBound.x, FLT_EPSILON );
			ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.x, FLT_EPSILON );
			ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
		}

		return 0;
	}

	private static int PointInShapeTest()
	{
		Vec2 p1 = new(  0.5f, 0.5f  );
		Vec2 p2 = new(  4.0f, -4.0f  );

		{
			bool hit;
			hit = PointInCircle(ref circle,  p1 );
			ENSURE( hit == true );
			hit = PointInCircle( ref circle, p2 );
			ENSURE( hit == false );
		}

		{
			bool hit;
			hit = PointInPolygon( ref box,  p1);
			ENSURE( hit == true );
			hit = PointInPolygon(ref box,  p2);
			ENSURE( hit == false );
		}

		return 0;
	}

	private static int RayCastShapeTest()
	{
		RayCastInput input = new RayCastInput { origin = new Vec2 { x = -4.0f, y = 0.0f }, translation = new Vec2 { x = 8.0f, y = 0.0f }, maxFraction = 1.0f };

		{
			CastOutput output = CastRayCircle( ref circle, ref input );
			ENSURE( output.hit );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.fraction - 0.5f, FLT_EPSILON );
		}

		{
			CastOutput output = CastRayPolygon( ref box, ref input);
			ENSURE( output.hit );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.fraction - 3.0f / 8.0f, FLT_EPSILON );
		}

		{
			CastOutput output = CastRaySegment( ref segment, ref input, true );
			ENSURE( output.hit );
			ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
			ENSURE_SMALL( output.normal.y, FLT_EPSILON );
			ENSURE_SMALL( output.fraction - 0.5f, FLT_EPSILON );
		}

		return 0;
	}

	internal static int ShapeTest()
	{
		box = MakeBox( 1.0f, 1.0f );

		RUN_SUBTEST( ShapeMassTest );
		RUN_SUBTEST( ShapeAABBTest );
		RUN_SUBTEST( PointInShapeTest );
		RUN_SUBTEST( RayCastShapeTest );

		return 0;
	}

}
