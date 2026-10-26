// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class SortTests
{
	private const int MAX_KEYS = 4096;
	static ulong s_keys[MAX_KEYS];
	static ulong s_expected[MAX_KEYS];
	static ulong s_temp[MAX_KEYS];

	static ulong s_state = 0x9E3779B97F4A7C15ull;

	private static ulong NextRandom()
	{
		// xorshift64*
		s_state ^= s_state >> 12;
		s_state ^= s_state << 25;
		s_state ^= s_state >> 27;
		return s_state * 0x2545F4914F6CDD1Dull;
	}

	private static int CompareKeys( object? a, object? b )
	{
		ulong ka = *(const uint64_t*)a;
		ulong kb = *(const uint64_t*)b;
		return ka < kb ? -1 : ( ka > kb ? 1 : 0 );
	}

	private static int SortAndCheck( int count )
	{
		memcpy( s_expected, s_keys, count * Unsafe.SizeOf<ulong>() );
		qsort( s_expected, count, Unsafe.SizeOf<ulong>(), CompareKeys );

		RadixSortKeys( s_keys, s_temp, count );

		for ( int i = 0; i < count; ++i )
		{
			ENSURE( s_keys[i] == s_expected[i] );
		}

		return 0;
	}

	private static int TestRandomFullWidth()
	{
		int[] counts = { 2, 3, 17, 255, 256, 257, 1000, MAX_KEYS };
		for ( int c = 0; c < (int)( Unsafe.SizeOf<counts>() / Unsafe.SizeOf<counts[0]>() ); ++c )
		{
			for ( int i = 0; i < counts[c]; ++i )
			{
				s_keys[i] = NextRandom();
			}

			ENSURE( SortAndCheck( counts[c] ) == 0 );
		}

		return 0;
	}

	private static int TestShapePairKeys()
	{
		int count = 2000;
		for ( int i = 0; i < count; ++i )
		{
			ulong shapeA = NextRandom() % 300;
			ulong shapeB = NextRandom() % 300;
			s_keys[i] = ( shapeA << 32 ) | shapeB;
		}

		return SortAndCheck( count );
	}

	private static int TestSingleDigit()
	{
		// One active digit leaves the result in the temp buffer
		int count = 500;
		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = 0xABCD000000000000ull | ( NextRandom() & 0xFF );
		}

		ENSURE( SortAndCheck( count ) == 0 );

		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = 0x1234ull | ( ( NextRandom() & 0xFF ) << 56 );
		}

		return SortAndCheck( count );
	}

	private static int TestTwoDigits()
	{
		int count = 500;
		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = ( NextRandom() & 0xFF ) | ( ( NextRandom() & 0xFF ) << 40 );
		}

		return SortAndCheck( count );
	}

	private static int TestAllEqual()
	{
		int count = 100;
		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = 0x0102030405060708ull;
		}

		return SortAndCheck( count );
	}

	private static int TestOrdered()
	{
		int count = 1000;
		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = (ulong)i * 0x0101010101ull;
		}

		ENSURE( SortAndCheck( count ) == 0 );

		for ( int i = 0; i < count; ++i )
		{
			s_keys[i] = (ulong)( count - i ) * 0x0101010101ull;
		}

		return SortAndCheck( count );
	}

	private static int TestExtremes()
	{
		s_keys[0] = ulong.MaxValue;
		s_keys[1] = 0;
		s_keys[2] = 1ull << 63;
		s_keys[3] = ( 1ull << 63 ) - 1;
		s_keys[4] = 0;
		s_keys[5] = ulong.MaxValue;

		return SortAndCheck( 6 );
	}

	internal static int SortTest()
	{
		RUN_SUBTEST( TestRandomFullWidth );
		RUN_SUBTEST( TestShapePairKeys );
		RUN_SUBTEST( TestSingleDigit );
		RUN_SUBTEST( TestTwoDigits );
		RUN_SUBTEST( TestAllEqual );
		RUN_SUBTEST( TestOrdered );
		RUN_SUBTEST( TestExtremes );

		return 0;
	}

}
