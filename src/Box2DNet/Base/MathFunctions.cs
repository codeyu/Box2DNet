// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;

namespace Box2DNet;

public static partial class B2
{
	/// https://en.wikipedia.org/wiki/Pi
	public const float Pi = 3.14159265359f;

	internal const float FLT_EPSILON = 1.1920929e-7f;
	internal const float FLT_MIN = 1.17549435e-38f;
	internal const float FLT_MAX = float.MaxValue;

	public static readonly Vec2 Vec2_zero = new( 0.0f, 0.0f );
	public static readonly Rot Rot_identity = new( 1.0f, 0.0f );
	public static readonly Transform Transform_identity = new( new Vec2( 0.0f, 0.0f ), new Rot( 1.0f, 0.0f ) );
	public static readonly Mat22 Mat22_zero = default;
	public static readonly Vec2 Pos_zero = new( 0.0f, 0.0f );
	public static readonly Transform WorldTransform_identity = new( new Vec2( 0.0f, 0.0f ), new Rot( 1.0f, 0.0f ) );

	/// Is this a valid number? Not NaN or infinity.
	public static bool IsValidFloat( float a ) => float.IsFinite( a );

	/// Is this a valid vector? Not NaN or infinity.
	public static bool IsValidVec2( Vec2 v ) => float.IsFinite( v.x ) && float.IsFinite( v.y );

	/// Is this a valid rotation? Not NaN or infinity. Is normalized.
	public static bool IsValidRot( Rot q )
	{
		if ( float.IsFinite( q.s ) == false || float.IsFinite( q.c ) == false )
		{
			return false;
		}

		return IsNormalizedRot( q );
	}

	/// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
	public static bool IsValidTransform( Transform t )
	{
		if ( IsValidVec2( t.p ) == false )
		{
			return false;
		}

		return IsValidRot( t.q );
	}

	/// Is this a valid plane? Normal is a unit vector. Not NaN or infinity.
	public static bool IsValidPlane( Plane a )
	{
		return IsValidVec2( a.normal ) && IsNormalized( a.normal ) && IsValidFloat( a.offset );
	}

	/// Is this a valid world position? Not NaN or infinity.
	public static bool IsValidPos( Vec2 p ) => IsValidVec2( p );

	/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
	public static bool IsValidWorldTransform( Transform t ) => IsValidTransform( t );

	/// @return the minimum of two integers
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static int MinInt( int a, int b ) => a < b ? a : b;

	/// @return the maximum of two integers
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static int MaxInt( int a, int b ) => a > b ? a : b;

	/// @return the absolute value of an integer
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static int AbsInt( int a ) => a < 0 ? -a : a;

	/// @return an integer clamped between a lower and upper bound
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static int ClampInt( int a, int lower, int upper ) => a < lower ? lower : ( a > upper ? upper : a );

	/// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
	public static int CeilingInt( int numerator, int denominator )
	{
		B2_VALIDATE( denominator > 0 && numerator >= 0 );
		return ( numerator + denominator - 1 ) / denominator;
	}

	/// @return the minimum of two floats
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float MinFloat( float a, float b ) => a < b ? a : b;

	/// @return the maximum of two floats
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float MaxFloat( float a, float b ) => a > b ? a : b;

	/// @return the absolute value of a float
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float AbsFloat( float a ) => a < 0 ? -a : a;

	/// @return a float clamped between a lower and upper bound
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float ClampFloat( float a, float lower, float upper ) => a < lower ? lower : ( a > upper ? upper : a );

	/// Compute an approximate arctangent in the range [-pi, pi] This is hand coded for cross-platform
	/// determinism. The atan2f function in the standard library is not cross-platform deterministic.
	/// Accurate to around 0.0023 degrees
	// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
	public static float Atan2( float y, float x )
	{
		// Added check for (0,0) to match atan2f and avoid NaN
		if ( x == 0.0f && y == 0.0f )
		{
			return 0.0f;
		}

		float ax = AbsFloat( x );
		float ay = AbsFloat( y );
		float mx = MaxFloat( ay, ax );
		float mn = MinFloat( ay, ax );
		float a = mn / mx;

		// Minimax polynomial approximation to atan(a) on [0,1]
		float s = a * a;
		float c = s * a;
		float q = s * s;
		float r = 0.024840285f * q + 0.18681418f;
		float t = -0.094097948f * q - 0.33213072f;
		r = r * s + t;
		r = r * c + a;

		// Map to full circle
		if ( ay > ax )
		{
			r = 1.57079637f - r;
		}

		if ( x < 0 )
		{
			r = 3.14159274f - r;
		}

		if ( y < 0 )
		{
			r = -r;
		}

		return r;
	}

	/// Compute the cosine and sine of an angle in radians. Implemented for cross-platform determinism.
	// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
	// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
	// this result.
	// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
	public static CosSin ComputeCosSin( float radians )
	{
		float x = UnwindAngle( radians );
		float pi2 = Pi * Pi;

		// cosine needs angle in [-pi/2, pi/2]
		float c;
		if ( x < -0.5f * Pi )
		{
			float y = x + Pi;
			float y2 = y * y;
			c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}
		else if ( x > 0.5f * Pi )
		{
			float y = x - Pi;
			float y2 = y * y;
			c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}
		else
		{
			float y2 = x * x;
			c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}

		// sine needs angle in [0, pi]
		float s;
		if ( x < 0.0f )
		{
			float y = x + Pi;
			s = -16.0f * y * ( Pi - y ) / ( 5.0f * pi2 - 4.0f * y * ( Pi - y ) );
		}
		else
		{
			s = 16.0f * x * ( Pi - x ) / ( 5.0f * pi2 - 4.0f * x * ( Pi - x ) );
		}

		float mag = MathF.Sqrt( s * s + c * c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		CosSin cs = new CosSin { cosine = c * invMag, sine = s * invMag };
		return cs;
	}

	/// Vector dot product
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float Dot( Vec2 a, Vec2 b ) => a.x * b.x + a.y * b.y;

	/// Vector cross product. In 2D this yields a scalar.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float Cross( Vec2 a, Vec2 b ) => a.x * b.y - a.y * b.x;

	/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 CrossVS( Vec2 v, float s ) => new( s * v.y, -s * v.x );

	/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 CrossSV( float s, Vec2 v ) => new( -s * v.y, s * v.x );

	/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 LeftPerp( Vec2 v ) => new( -v.y, v.x );

	/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 RightPerp( Vec2 v ) => new( v.y, -v.x );

	/// Vector addition
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Add( Vec2 a, Vec2 b ) => new( a.x + b.x, a.y + b.y );

	/// Vector subtraction
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Sub( Vec2 a, Vec2 b ) => new( a.x - b.x, a.y - b.y );

	/// Vector negation
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Neg( Vec2 a ) => new( -a.x, -a.y );

	/// Vector linear interpolation
	/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Lerp( Vec2 a, Vec2 b, float t ) => new( ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y );

	/// Component-wise multiplication
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Mul( Vec2 a, Vec2 b ) => new( a.x * b.x, a.y * b.y );

	/// Multiply a scalar and vector
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 MulSV( float s, Vec2 v ) => new( s * v.x, s * v.y );

	/// a + s * b
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 MulAdd( Vec2 a, float s, Vec2 b ) => new( a.x + s * b.x, a.y + s * b.y );

	/// a - s * b
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 MulSub( Vec2 a, float s, Vec2 b ) => new( a.x - s * b.x, a.y - s * b.y );

	/// Component-wise absolute vector
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Abs( Vec2 a ) => new( AbsFloat( a.x ), AbsFloat( a.y ) );

	/// Component-wise minimum vector
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Min( Vec2 a, Vec2 b ) => new( MinFloat( a.x, b.x ), MinFloat( a.y, b.y ) );

	/// Component-wise maximum vector
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Max( Vec2 a, Vec2 b ) => new( MaxFloat( a.x, b.x ), MaxFloat( a.y, b.y ) );

	/// Component-wise clamp vector v into the range [a, b]
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Clamp( Vec2 v, Vec2 a, Vec2 b ) => new( ClampFloat( v.x, a.x, b.x ), ClampFloat( v.y, a.y, b.y ) );

	/// Get the length of this vector (the norm)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float Length( Vec2 v ) => MathF.Sqrt( v.x * v.x + v.y * v.y );

	/// Get the distance between two points
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float Distance( Vec2 a, Vec2 b )
	{
		float dx = b.x - a.x;
		float dy = b.y - a.y;
		return MathF.Sqrt( dx * dx + dy * dy );
	}

	/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Normalize( Vec2 a )
	{
		float lengthSquared = a.x * a.x + a.y * a.y;
		if ( lengthSquared > 1000.0f * FLT_MIN )
		{
			float s = 1.0f / MathF.Sqrt( lengthSquared );
			return new Vec2( s * a.x, s * a.y );
		}

		return new Vec2( 0.0f, 0.0f );
	}

	/// Determines if the provided vector is normalized (norm(a) == 1).
	public static bool IsNormalized( Vec2 a )
	{
		float aa = Dot( a, a );
		return AbsFloat( 1.0f - aa ) < 100.0f * FLT_EPSILON;
	}

	/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also outputs
	/// the length.
	public static Vec2 GetLengthAndNormalize( out float length, Vec2 a )
	{
		float lengthSquared = a.x * a.x + a.y * a.y;
		if ( lengthSquared > 1000.0f * FLT_MIN )
		{
			length = MathF.Sqrt( lengthSquared );
			float s = 1.0f / length;
			return new Vec2( s * a.x, s * a.y );
		}

		length = 0.0f;
		return new Vec2( 0.0f, 0.0f );
	}

	/// Normalize rotation
	public static Rot NormalizeRot( Rot q )
	{
		float mag = MathF.Sqrt( q.s * q.s + q.c * q.c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		return new Rot( q.c * invMag, q.s * invMag );
	}

	/// Integrate rotation from angular velocity
	/// @param q1 initial rotation
	/// @param deltaAngle the angular displacement in radians
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Rot IntegrateRotation( Rot q1, float deltaAngle )
	{
		// dc/dt = -omega * sin(t)
		// ds/dt = omega * cos(t)
		// c2 = c1 - omega * h * s1
		// s2 = s1 + omega * h * c1
		Rot q2 = new Rot( q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c );
		float mag = MathF.Sqrt( q2.s * q2.s + q2.c * q2.c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		return new Rot( q2.c * invMag, q2.s * invMag );
	}

	/// Get the length squared of this vector
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float LengthSquared( Vec2 v ) => v.x * v.x + v.y * v.y;

	/// Get the distance squared between points
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float DistanceSquared( Vec2 a, Vec2 b )
	{
		Vec2 c = new( b.x - a.x, b.y - a.y );
		return c.x * c.x + c.y * c.y;
	}

	/// Make a rotation using an angle in radians
	public static Rot MakeRot( float radians )
	{
		CosSin cs = ComputeCosSin( radians );
		return new Rot( cs.cosine, cs.sine );
	}

	/// Make a rotation using a unit vector
	public static Rot MakeRotFromUnitVector( Vec2 unitVector )
	{
		B2_ASSERT( IsNormalized( unitVector ) );
		return new Rot( unitVector.x, unitVector.y );
	}

	/// Compute the rotation between two unit vectors
	public static Rot ComputeRotationBetweenUnitVectors( Vec2 v1, Vec2 v2 )
	{
		B2_ASSERT( AbsFloat( 1.0f - Length( v1 ) ) < 100.0f * FLT_EPSILON );
		B2_ASSERT( AbsFloat( 1.0f - Length( v2 ) ) < 100.0f * FLT_EPSILON );

		Rot rot;
		rot.c = Dot( v1, v2 );
		rot.s = Cross( v1, v2 );
		return NormalizeRot( rot );
	}

	/// Is this rotation normalized?
	public static bool IsNormalizedRot( Rot q )
	{
		// larger tolerance due to failure on mingw 32-bit
		float qq = q.s * q.s + q.c * q.c;
		return 1.0f - 0.0006f < qq && qq < 1.0f + 0.0006f;
	}

	/// Get the inverse of a rotation
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Rot InvertRot( Rot a ) => new( a.c, -a.s );

	/// Normalized linear interpolation
	/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	/// https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
	public static Rot NLerp( Rot q1, Rot q2, float t )
	{
		float omt = 1.0f - t;
		Rot q = new Rot( omt * q1.c + t * q2.c, omt * q1.s + t * q2.s );

		float mag = MathF.Sqrt( q.s * q.s + q.c * q.c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		return new Rot( q.c * invMag, q.s * invMag );
	}

	/// Compute the angular velocity necessary to rotate between two rotations over a given time
	/// @param q1 initial rotation
	/// @param q2 final rotation
	/// @param inv_h inverse time step
	public static float ComputeAngularVelocity( Rot q1, Rot q2, float inv_h )
	{
		// ds/dt = omega * cos(t)
		// dc/dt = -omega * sin(t)
		// s2 = s1 + omega * h * c1
		// c2 = c1 - omega * h * s1

		// omega * h * s1 = c1 - c2
		// omega * h * c1 = s2 - s1
		// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
		// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
		// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
		float omega = inv_h * ( q2.s * q1.c - q2.c * q1.s );
		return omega;
	}

	/// Get the angle in radians in the range [-pi, pi]
	public static float Rot_GetAngle( Rot q ) => Atan2( q.s, q.c );

	/// Get the x-axis
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Rot_GetXAxis( Rot q ) => new( q.c, q.s );

	/// Get the y-axis
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 Rot_GetYAxis( Rot q ) => new( -q.s, q.c );

	/// Multiply two rotations: q * r
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Rot MulRot( Rot q, Rot r )
	{
		// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
		// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
		// s(q + r) = qs * rc + qc * rs
		// c(q + r) = qc * rc - qs * rs
		Rot qr;
		qr.s = q.s * r.c + q.c * r.s;
		qr.c = q.c * r.c - q.s * r.s;
		return qr;
	}

	/// Transpose multiply two rotations: inv(a) * b This rotates a vector local in frame b into a
	/// vector local in frame a
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Rot InvMulRot( Rot a, Rot b )
	{
		// [ ac as] * [bc -bs] = [ac*bc+as*bs -ac*bs+as*bc]
		// [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
		// s(b - a) = ac * bs - as * bc
		// c(b - a) = ac * bc + as * bs
		Rot r;
		r.s = a.c * b.s - a.s * b.c;
		r.c = a.c * b.c + a.s * b.s;
		return r;
	}

	/// The angle of rotation b relative to rotation a. Radians.
	public static float RelativeAngle( Rot a, Rot b )
	{
		// sin(b - a) = bs * ac - bc * as
		// cos(b - a) = bc * ac + bs * as
		float s = a.c * b.s - a.s * b.c;
		float c = a.c * b.c + a.s * b.s;
		return Atan2( s, c );
	}

	/// Convert any angle into the range [-pi, pi].
	public static float UnwindAngle( float radians )
	{
		// This function can be implemented using remainderf.
		// However that is a libc dependency not present in WASM.
		// This function doesn't work well with huge angles. So unwind your angles regularly.
		B2_VALIDATE( AbsFloat( radians ) < 1.0e4f );

		float x = ClampFloat( radians, -1.0e6f, 1.0e6f );

		// Work in doubles. Adding and removing this constant rounds to the nearest integer.
		// https://stackoverflow.com/questions/17035464/a-fast-method-to-round-a-double-to-a-32-bit-int-explained
		double twoPi = 2.0f * Pi;
		double roundToNearest = 6755399441055744.0;
		double a = x;
		double k = ( a / twoPi + roundToNearest ) - roundToNearest;
		return (float)( a - k * twoPi );
	}

	/// Rotate a vector.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 RotateVector( Rot q, Vec2 v ) => new( q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y );

	/// Inverse rotate a vector.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 InvRotateVector( Rot q, Vec2 v ) => new( q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y );

	/// Transform a point (e.g. local space to world space).
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 TransformPoint( Transform t, Vec2 p )
	{
		float x = ( t.q.c * p.x - t.q.s * p.y ) + t.p.x;
		float y = ( t.q.s * p.x + t.q.c * p.y ) + t.p.y;
		return new Vec2( x, y );
	}

	/// Inverse transform a point (e.g. world space to local space).
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 InvTransformPoint( Transform t, Vec2 p )
	{
		float vx = p.x - t.p.x;
		float vy = p.y - t.p.y;
		return new Vec2( t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy );
	}

	/// Multiply two transforms. If the result is applied to a point p local to frame B, the transform
	/// would first convert p to a point local to frame A, then into a point in the world frame.
	/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
	///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
	public static Transform MulTransforms( Transform A, Transform B )
	{
		Transform C;
		C.q = MulRot( A.q, B.q );
		C.p = Add( RotateVector( A.q, B.p ), A.p );
		return C;
	}

	/// Creates a transform that converts a local point in frame B to a local point in frame A.
	/// v2 = A.q' * (B.q * v1 + B.p - A.p)
	///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
	public static Transform InvMulTransforms( Transform A, Transform B )
	{
		Transform C;
		C.q = InvMulRot( A.q, B.q );
		C.p = InvRotateVector( A.q, Sub( B.p, A.p ) );
		return C;
	}

	/// Convert a vector to a world position. No-op in single precision.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 ToPos( Vec2 v ) => v;

	/// Lossy conversion of a world position to a float vector. No-op in single precision.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 ToVec2( Vec2 p ) => p;

	/// Narrow a world coordinate to float, rounding toward negative infinity. With large world mode off
	/// this is a plain conversion.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float RoundDownFloat( float x ) => x;

	/// Narrow a world coordinate to float, rounding toward positive infinity.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float RoundUpFloat( float x ) => x;

	/// a - b, demoted to float. The primary precision boundary operation.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 SubPos( Vec2 a, Vec2 b ) => new( a.x - b.x, a.y - b.y );

	/// p + d
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 OffsetPos( Vec2 p, Vec2 d ) => new( p.x + d.x, p.y + d.y );

	/// World position interpolation for sweeps and sampling.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 LerpPos( Vec2 a, Vec2 b, float t ) => new( ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y );

	/// Transform a local point to a world position. Rotation in float, translation in double.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 TransformWorldPoint( Transform t, Vec2 p )
	{
		float rx = t.q.c * p.x - t.q.s * p.y;
		float ry = t.q.s * p.x + t.q.c * p.y;
		return new Vec2( t.p.x + rx, t.p.y + ry );
	}

	/// Transform a world position to a local point. One double subtraction, then float.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Vec2 InvTransformWorldPoint( Transform t, Vec2 p )
	{
		float vx = p.x - t.p.x;
		float vy = p.y - t.p.y;
		return new Vec2( t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy );
	}

	/// Relative transform of frame B in frame A.
	public static Transform InvMulWorldTransforms( Transform A, Transform B )
	{
		Transform C;
		C.q = InvMulRot( A.q, B.q );
		Vec2 d = new( B.p.x - A.p.x, B.p.y - A.p.y );
		C.p = InvRotateVector( A.q, d );
		return C;
	}

	/// Convert a local transform B into world space using world transform A.
	public static Transform MulWorldTransforms( Transform A, Transform B )
	{
		Transform C;
		C.q = MulRot( A.q, B.q );
		C.p = OffsetPos( A.p, RotateVector( A.q, B.p ) );
		return C;
	}

	/// Shift a world transform into the frame of a base position.
	public static Transform ToRelativeTransform( Transform t, Vec2 @base )
	{
		Transform r;
		r.q = t.q;
		r.p = new Vec2( t.p.x - @base.x, t.p.y - @base.y );
		return r;
	}

	/// Promote a float transform to a world transform. Lossless.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static Transform MakeWorldTransform( Transform t ) => t;

	/// Multiply a 2-by-2 matrix times a 2D vector
	public static Vec2 MulMV( Mat22 A, Vec2 v )
	{
		return new Vec2( A.cx.x * v.x + A.cy.x * v.y, A.cx.y * v.x + A.cy.y * v.y );
	}

	/// Get the inverse of a 2-by-2 matrix
	public static Mat22 GetInverse22( Mat22 A )
	{
		float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
		float det = a * d - b * c;
		if ( det != 0.0f )
		{
			det = 1.0f / det;
		}

		return new Mat22( new Vec2( det * d, -det * c ), new Vec2( -det * b, det * a ) );
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
	/// in one-shot cases.
	public static Vec2 Solve22( Mat22 A, Vec2 b )
	{
		float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
		float det = a11 * a22 - a12 * a21;
		if ( det != 0.0f )
		{
			det = 1.0f / det;
		}

		return new Vec2( det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) );
	}

	/// Does a fully contain b
	public static bool AABB_Contains( AABB a, AABB b )
	{
		bool s = true;
		s = s && a.lowerBound.x <= b.lowerBound.x;
		s = s && a.lowerBound.y <= b.lowerBound.y;
		s = s && b.upperBound.x <= a.upperBound.x;
		s = s && b.upperBound.y <= a.upperBound.y;
		return s;
	}

	/// Get the center of the AABB.
	public static Vec2 AABB_Center( AABB a )
	{
		return new Vec2( 0.5f * ( a.lowerBound.x + a.upperBound.x ), 0.5f * ( a.lowerBound.y + a.upperBound.y ) );
	}

	/// Get the extents of the AABB (half-widths).
	public static Vec2 AABB_Extents( AABB a )
	{
		return new Vec2( 0.5f * ( a.upperBound.x - a.lowerBound.x ), 0.5f * ( a.upperBound.y - a.lowerBound.y ) );
	}

	/// Union of two AABBs
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static AABB AABB_Union( AABB a, AABB b )
	{
		AABB c;
		c.lowerBound.x = MinFloat( a.lowerBound.x, b.lowerBound.x );
		c.lowerBound.y = MinFloat( a.lowerBound.y, b.lowerBound.y );
		c.upperBound.x = MaxFloat( a.upperBound.x, b.upperBound.x );
		c.upperBound.y = MaxFloat( a.upperBound.y, b.upperBound.y );
		return c;
	}

	/// Do a and b overlap
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static bool AABB_Overlaps( AABB a, AABB b )
	{
		return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
				  a.lowerBound.y > b.upperBound.y );
	}

	/// Compute the bounding box of an array of circles
	public static AABB MakeAABB( ReadOnlySpan<Vec2> points, int count, float radius )
	{
		B2_ASSERT( count > 0 );
		AABB a = new( points[0], points[0] );
		for ( int i = 1; i < count; ++i )
		{
			a.lowerBound = Min( a.lowerBound, points[i] );
			a.upperBound = Max( a.upperBound, points[i] );
		}

		Vec2 r = new( radius, radius );
		a.lowerBound = Sub( a.lowerBound, r );
		a.upperBound = Add( a.upperBound, r );

		return a;
	}

	/// Signed separation of a point from a plane
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static float PlaneSeparation( Plane plane, Vec2 point ) => Dot( plane.normal, point ) - plane.offset;

	/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and
	/// time step. You can then compute the new position using:
	/// position += timeStep * newVelocity
	/// This drives towards a zero position. By using implicit integration we get a stable solution that
	/// doesn't require transcendental functions.
	public static float SpringDamper( float hertz, float dampingRatio, float position, float velocity, float timeStep )
	{
		float omega = 2.0f * Pi * hertz;
		float omegaH = omega * timeStep;
		return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
	}

	/// This value is exported for efficiency. Don't modify directly.
	internal static float lengthUnitsPerMeter = 1.0f;

	/// Box2D bases all length units on meters, but you may need different units for your game. You can
	/// set this value to use different units. This should be done at application startup and only
	/// modified once. Default value is 1. A good rule of thumb is to pass the height of your player
	/// character to this function.
	/// @warning This must be modified before any calls to Box2D
	public static void SetLengthUnitsPerMeter( float lengthUnits )
	{
		if ( CheckInput( IsValidFloat( lengthUnits ) && lengthUnits > 0.0f ) == false )
		{
			return;
		}

		lengthUnitsPerMeter = lengthUnits;
	}

	/// Get the current length units per meter.
	public static float GetLengthUnitsPerMeter() => lengthUnitsPerMeter;
}
