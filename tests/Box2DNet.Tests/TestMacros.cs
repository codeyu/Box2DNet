// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;
using Xunit;

namespace Box2DNet.Tests;

// Mirrors test_macros.h. A failed check throws so xUnit reports the expression and location.
public static class TestMacros
{
	public static void ENSURE( bool condition, [CallerArgumentExpression( nameof( condition ) )] string expression = "",
							   [CallerFilePath] string file = "", [CallerLineNumber] int line = 0 )
	{
		if ( condition == false )
		{
			Assert.Fail( $"condition false: {expression} ({System.IO.Path.GetFileName( file )}:{line})" );
		}
	}

	public static void ENSURE_SMALL( float value, float tol, [CallerArgumentExpression( nameof( value ) )] string expression = "",
									 [CallerFilePath] string file = "", [CallerLineNumber] int line = 0 )
	{
		if ( value < -tol || tol < value )
		{
			Assert.Fail( $"condition false: abs({expression}) < {tol}, value = {value} ({System.IO.Path.GetFileName( file )}:{line})" );
		}
	}

	public static void RUN_SUBTEST( Func<int> test, [CallerArgumentExpression( nameof( test ) )] string name = "" )
	{
		int result = test();
		if ( result == 1 )
		{
			Assert.Fail( $"subtest failed: {name}" );
		}
	}

	public static void RUN_TEST( Func<int> test, [CallerArgumentExpression( nameof( test ) )] string name = "" )
	{
		RUN_SUBTEST( test, name );
	}

	// World_GetStateHash lives in snapshot.c which is not ported yet. This hashes the state of all
	// bodies, which is enough for the tests that compare two simulations.
	internal static ulong World_GetStateHash( WorldId worldId )
	{
		B2.World world = B2.GetWorldFromId( worldId );
		uint hash = B2.HashInit;
		for ( int i = 0; i < world.bodies.count; ++i )
		{
			ref B2.Body body = ref world.bodies.data[i];
			if ( body.id == B2.NullIndex )
			{
				continue;
			}

			Transform transform = B2.GetBodyTransform( world, i );
			hash = B2.Hash( hash, System.Runtime.InteropServices.MemoryMarshal.AsBytes( new ReadOnlySpan<Transform>( in transform ) ) );
		}

		return hash;
	}

	public static float sqrtf( float x ) => MathF.Sqrt( x );
	public static float cosf( float x ) => MathF.Cos( x );
	public static float sinf( float x ) => MathF.Sin( x );
	public static float atan2f( float y, float x ) => MathF.Atan2( y, x );
	public static float fabsf( float x ) => MathF.Abs( x );
}
