// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace Box2DNet;

// The 4 wide float math used by the wide contact solver (contact_solver_wide.inl). This maps to
// SSE2 on x64 and NEON on arm64. Every operation mirrors the SSE2 version in Box2D, including the
// handling of signed zero in min, max and negation, so results match bit for bit.
public static partial class B2
{
	// wide float holds 4 numbers
	internal struct Vec2W
	{
		public Vector128<float> X, Y;
	}

	internal struct RotW
	{
		public Vector128<float> C, S;
	}

	private static readonly Vector128<float> s_signMask = Vector128.Create( -0.0f );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> ZeroW() => Vector128<float>.Zero;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> SplatW( float scalar ) => Vector128.Create( scalar );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> SetW( float a, float b, float c, float d ) => Vector128.Create( a, b, c, d );

	// XOR the sign bit, like the SSE2 version, so -(+0) = -0
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> NegW( Vector128<float> a ) => Vector128.Xor( a, s_signMask );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> AddW( Vector128<float> a, Vector128<float> b ) => Vector128.Add( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> SubW( Vector128<float> a, Vector128<float> b ) => Vector128.Subtract( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> MulW( Vector128<float> a, Vector128<float> b ) => Vector128.Multiply( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> DivW( Vector128<float> a, Vector128<float> b ) => Vector128.Divide( a, b );

	// a + b * c, no fused multiply-add for determinism
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> MulAddW( Vector128<float> a, Vector128<float> b, Vector128<float> c ) =>
		Vector128.Add( a, Vector128.Multiply( b, c ) );

	// a - b * c
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> MulSubW( Vector128<float> a, Vector128<float> b, Vector128<float> c ) =>
		Vector128.Subtract( a, Vector128.Multiply( b, c ) );

	// minps semantics: a < b ? a : b
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> MinW( Vector128<float> a, Vector128<float> b ) =>
		Vector128.ConditionalSelect( Vector128.LessThan( a, b ), a, b );

	// maxps semantics: a > b ? a : b
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> MaxW( Vector128<float> a, Vector128<float> b ) =>
		Vector128.ConditionalSelect( Vector128.GreaterThan( a, b ), a, b );

	// a = clamp(a, -b, b)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> SymClampW( Vector128<float> a, Vector128<float> b )
	{
		Vector128<float> nb = Vector128.Xor( b, s_signMask );
		return MaxW( nb, MinW( a, b ) );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> AndW( Vector128<float> a, Vector128<float> b ) => Vector128.BitwiseAnd( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> GreaterThanW( Vector128<float> a, Vector128<float> b ) => Vector128.GreaterThan( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> LessThanW( Vector128<float> a, Vector128<float> b ) => Vector128.LessThan( a, b );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool AnyTrueW( Vector128<float> mask ) => Vector128.ExtractMostSignificantBits( mask ) != 0;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool AllZeroW( Vector128<float> a ) => Vector128.EqualsAll( a, Vector128<float>.Zero );

	// component-wise returns mask ? b : a
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> BlendW( Vector128<float> a, Vector128<float> b, Vector128<float> mask ) =>
		Vector128.ConditionalSelect( mask, b, a );

	// unpacklo: (a0, b0, a1, b1)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> UnpackLoW( Vector128<float> a, Vector128<float> b ) =>
		Vector128.Create( a.GetElement( 0 ), b.GetElement( 0 ), a.GetElement( 1 ), b.GetElement( 1 ) );

	// unpackhi: (a2, b2, a3, b3)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> UnpackHiW( Vector128<float> a, Vector128<float> b ) =>
		Vector128.Create( a.GetElement( 2 ), b.GetElement( 2 ), a.GetElement( 3 ), b.GetElement( 3 ) );

	// mask set where either body index is zero (static/kinematic)
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> SoftMaskW( in FixedArray4<int> indexA, in FixedArray4<int> indexB )
	{
		Vector128<int> a = Vector128.Equals( Vector128.Create( indexA[0], indexA[1], indexA[2], indexA[3] ), Vector128<int>.Zero );
		Vector128<int> b = Vector128.Equals( Vector128.Create( indexB[0], indexB[1], indexB[2], indexB[3] ), Vector128<int>.Zero );
		return Vector128.BitwiseOr( a, b ).AsSingle();
	}

	// Load 4 floats from consecutive memory. Used for the manifold point transpose.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> LoadManifoldPoint( ref ManifoldPoint point, int floatOffset )
	{
		ref byte bytes = ref Unsafe.As<ManifoldPoint, byte>( ref point );
		return Unsafe.ReadUnaligned<Vector128<float>>( ref Unsafe.Add( ref bytes, 4 * floatOffset ) );
	}

	// a - b
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vec2W SubVW( Vec2W a, Vec2W b ) => new Vec2W { X = SubW( a.X, b.X ), Y = SubW( a.Y, b.Y ) };

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vec2W RightPerpW( Vec2W a ) => new Vec2W { X = a.Y, Y = NegW( a.X ) };

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> DotW( Vec2W a, Vec2W b ) => AddW( MulW( a.X, b.X ), MulW( a.Y, b.Y ) );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vector128<float> CrossW( Vec2W a, Vec2W b ) => SubW( MulW( a.X, b.Y ), MulW( a.Y, b.X ) );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static Vec2W RotateVectorW( RotW q, Vec2W v ) =>
		new Vec2W { X = SubW( MulW( q.C, v.X ), MulW( q.S, v.Y ) ), Y = AddW( MulW( q.S, v.X ), MulW( q.C, v.Y ) ) };

	// Wide version of BodyState.
	internal struct BodyStateW
	{
		public Vec2W v;
		public Vector128<float> w;
		public Vector128<float> flags;
		public Vec2W dp;
		public RotW dq;
	}

	// This is a load and transpose
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static BodyStateW GatherBodies( ReadOnlySpanStates states, in FixedArray4<int> indices )
	{
		B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

		BodyState identity = identityBodyState;

		// zero means null
		int i1 = indices[0] - 1;
		int i2 = indices[1] - 1;
		int i3 = indices[2] - 1;
		int i4 = indices[3] - 1;

		BodyState s1 = i1 == NullIndex ? identity : states.data[i1];
		BodyState s2 = i2 == NullIndex ? identity : states.data[i2];
		BodyState s3 = i3 == NullIndex ? identity : states.data[i3];
		BodyState s4 = i4 == NullIndex ? identity : states.data[i4];

		BodyStateW simdBody;
		simdBody.v.X = Vector128.Create( s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x );
		simdBody.v.Y = Vector128.Create( s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y );
		simdBody.w = Vector128.Create( s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity );

		// The SSE2 version transposes the raw flag bits into the float lanes
		simdBody.flags = Vector128.Create( s1.flags, s2.flags, s3.flags, s4.flags ).AsSingle();
		simdBody.dp.X = Vector128.Create( s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x );
		simdBody.dp.Y = Vector128.Create( s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y );
		simdBody.dq.C = Vector128.Create( s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c );
		simdBody.dq.S = Vector128.Create( s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s );

		return simdBody;
	}

	// This writes only the velocities back to the solver bodies
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void ScatterBodies( ReadOnlySpanStates states, in FixedArray4<int> indices, in BodyStateW simdBody )
	{
		B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

		for ( int lane = 0; lane < 4; ++lane )
		{
			// zero means null
			int i = indices[lane] - 1;
			if ( i != NullIndex && ( states.data[i].flags & b2_dynamicFlag ) != 0 )
			{
				ref BodyState state = ref states.data[i];
				state.linearVelocity.x = simdBody.v.X.GetElement( lane );
				state.linearVelocity.y = simdBody.v.Y.GetElement( lane );
				state.angularVelocity = simdBody.w.GetElement( lane );
			}
		}
	}

	// Wrapper so gather and scatter can take the state array with the same signature as C
	internal readonly struct ReadOnlySpanStates
	{
		public readonly BodyState[] data;

		public ReadOnlySpanStates( BodyState[] data )
		{
			this.data = data;
		}
	}
}
