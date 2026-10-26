// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class MathTests
{
	// 0.0023 degrees
	private const float ATAN_TOL = 0.00004f;
	internal static int MathTest()
	{
		for ( float t = -10.0f; t < 10.0f; t += 0.01f )
		{
			float angle = Pi * t;
			Rot r = MakeRot( angle );
			float c = cosf( angle );
			float s = sinf( angle );

			// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
			// printf( "%g %g\n", r.c - c, r.s - s );
			ENSURE_SMALL( r.c - c, 0.002f );
			ENSURE_SMALL( r.s - s, 0.002f );

			float xn = UnwindAngle( angle );
			ENSURE( -Pi <= xn && xn <= Pi );

			float a = Atan2( s, c );
			ENSURE( IsValidFloat( a ) );

			float diff = AbsFloat( a - xn );

			// The two results can be off by 360 degrees (-pi and pi)
			if ( diff > Pi )
			{
				diff -= 2.0f * Pi;
			}

			// The approximate atan2 is quite accurate
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		for ( float y = -1.0f; y <= 1.0f; y += 0.01f )
		{
			for ( float x = -1.0f; x <= 1.0f; x += 0.01f )
			{
				float a1 = Atan2( y, x );
				float a2 = atan2f( y, x );
				float diff = AbsFloat( a1 - a2 );
				ENSURE( IsValidFloat( a1 ) );
				ENSURE_SMALL( diff, ATAN_TOL );
			}
		}

		{
			float a1 = Atan2( 1.0f, 0.0f );
			float a2 = atan2f( 1.0f, 0.0f );
			float diff = AbsFloat( a1 - a2 );
			ENSURE( IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		{
			float a1 = Atan2( -1.0f, 0.0f );
			float a2 = atan2f( -1.0f, 0.0f );
			float diff = AbsFloat( a1 - a2 );
			ENSURE( IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		{
			float a1 = Atan2( 0.0f, 1.0f );
			float a2 = atan2f( 0.0f, 1.0f );
			float diff = AbsFloat( a1 - a2 );
			ENSURE( IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		{
			float a1 = Atan2( 0.0f, -1.0f );
			float a2 = atan2f( 0.0f, -1.0f );
			float diff = AbsFloat( a1 - a2 );
			ENSURE( IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		{
			float a1 = Atan2( 0.0f, 0.0f );
			float a2 = atan2f( 0.0f, 0.0f );
			float diff = AbsFloat( a1 - a2 );
			ENSURE( IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}

		Vec2 zero = Vec2_zero;
		Vec2 one = new(  1.0f, 1.0f  );
		Vec2 two = new(  2.0f, 2.0f  );

		Vec2 v = Add( one, two );
		ENSURE( v.x == 3.0f && v.y == 3.0f );

		v = Sub( zero, two );
		ENSURE( v.x == -2.0f && v.y == -2.0f );

		v = Add( two, two );
		ENSURE( v.x != 5.0f && v.y != 5.0f );

		Transform transform1 = new Transform { p = new Vec2 { x = -2.0f, y = 3.0f }, q = MakeRot( 1.0f ) };
		Transform transform2 = new Transform { p = new Vec2 { x = 1.0f, y = 0.0f }, q = MakeRot( -2.0f ) };

		Transform transform = MulTransforms( transform2, transform1 );

		v = TransformPoint( transform2, TransformPoint( transform1, two ) );

		Vec2 u = TransformPoint( transform, two );

		ENSURE_SMALL( u.x - v.x, 10.0f * FLT_EPSILON );
		ENSURE_SMALL( u.y - v.y, 10.0f * FLT_EPSILON );

		v = TransformPoint( transform1, two );
		v = InvTransformPoint( transform1, v );

		ENSURE_SMALL( v.x - two.x, 8.0f * FLT_EPSILON );
		ENSURE_SMALL( v.y - two.y, 8.0f * FLT_EPSILON );

		v = Normalize(new Vec2( 0.2f, -0.5f ));
		for ( float y = -1.0f; y <= 1.0f; y += 0.01f )
		{
			for ( float x = -1.0f; x <= 1.0f; x += 0.01f )
			{
				if (x == 0.0f && y == 0.0f)
				{
					continue;
				}

				u = Normalize( new Vec2( x, y ) );

				Rot r = ComputeRotationBetweenUnitVectors( v, u );

				Vec2 w = RotateVector( r, v );
				ENSURE_SMALL( w.x - u.x, 4.0f * FLT_EPSILON );
				ENSURE_SMALL( w.y - u.y, 4.0f * FLT_EPSILON );
			}
		}

		// NLerp of Rot has an error of over 4 degrees.
		// 2D quaternions should have an error under 1 degree.
		Rot q1 = Rot_identity;
		Rot q2 = MakeRot(0.5f * Pi);
		int n = 100;
		for (int i = 0; i <= n; ++i)
		{
			float alpha = (float)i / (float)n;
			Rot q = NLerp(q1, q2, alpha);
			float angle = Rot_GetAngle(q);
			ENSURE_SMALL( alpha * 0.5f * Pi - angle, 5.0f * Pi / 180.0f );
			//printf("angle = [%g %g %g]\n", alpha, alpha * 0.5f * Pi, angle);
		}

		// Test relative angle
		float baseAngle = 0.75f * Pi;
		q1 = MakeRot(baseAngle);
		for ( float t = -10.0f; t < 10.0f; t += 0.01f )
		{
			float angle = Pi * t;
			q2 = MakeRot(angle);

			float relativeAngle = RelativeAngle( q1, q2 );
			float unwoundAngle = UnwindAngle( angle - baseAngle );
			float tolerance = 0.1f * Pi / 180.0f;
			ENSURE_SMALL( relativeAngle - unwoundAngle, tolerance );
		}

		// World position boundary helpers. With large world mode off these collapse to the float
		// ops, so the round trips hold in both builds.
		{
			Vec2 d = new(  0.25f, -0.5f  );
			Pos @base = new Pos { x = 10.0f, y = -20.0f };
			Pos p = OffsetPos( @base, d );
			Vec2 back = SubPos( p, @base );
			ENSURE_SMALL( back.x - d.x, 8.0f * FLT_EPSILON );
			ENSURE_SMALL( back.y - d.y, 8.0f * FLT_EPSILON );

			Vec2 r = ToVec2( @base );
			ENSURE( r.x == 10.0f && r.y == -20.0f );

			ENSURE( IsValidPos( p ) );
			ENSURE( IsValidPos( Pos_zero ) );
			ENSURE( IsValidWorldTransform( WorldTransform_identity ) );

			WorldTransform wt = new WorldTransform { p = new Pos { x = 3.0f, y = -4.0f }, q = MakeRot( 0.7f ) };
			ENSURE( IsValidWorldTransform( wt ) );

			// Local to world to local round trip
			Vec2 local = new(  1.5f, 2.5f  );
			Pos world = TransformWorldPoint( wt, local );
			Vec2 backLocal = InvTransformWorldPoint( wt, world );
			ENSURE_SMALL( backLocal.x - local.x, 8.0f * FLT_EPSILON );
			ENSURE_SMALL( backLocal.y - local.y, 8.0f * FLT_EPSILON );

			// Relative transform of B in A matches a float reference at modest coordinates
			WorldTransform A = new WorldTransform { p = new Pos { x = -2.0f, y = 3.0f }, q = MakeRot( 1.0f ) };
			WorldTransform B = new WorldTransform { p = new Pos { x = 1.0f, y = 0.0f }, q = MakeRot( -2.0f ) };
			Transform rel = InvMulWorldTransforms( A, B );
			Transform refA = new Transform { p = ToVec2( A.p ), q = A.q };
			Transform refB = new Transform { p = ToVec2( B.p ), q = B.q };
			Transform @ref = InvMulTransforms( refA, refB );
			ENSURE_SMALL( rel.p.x - @ref.p.x, 8.0f * FLT_EPSILON );
			ENSURE_SMALL( rel.p.y - @ref.p.y, 8.0f * FLT_EPSILON );
		}

#if BOX2D_DOUBLE_PRECISION
		// Far from the origin a float vector cannot resolve sub meter motion, but a double world
		// position can. This is the whole point of large world mode.
		{
			Vec2 d = new(  0.25f, -0.5f  );
			Pos @base = new Pos { x = 1.0e7f, y = 0.0f };
			Pos p = OffsetPos( @base, d );
			Vec2 back = SubPos( p, @base );
			ENSURE_SMALL( back.x - d.x, 1.0e-4f );
			ENSURE_SMALL( back.y - d.y, 1.0e-4f );
		}
#endif

		return 0;
	}

}
