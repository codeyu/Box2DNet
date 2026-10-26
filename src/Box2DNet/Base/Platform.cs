// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.CompilerServices;
using System.Threading;

namespace Box2DNet;

// Atomic helpers mirroring platform.h
public static partial class B2
{
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void AtomicStoreInt( ref int a, int value ) => Volatile.Write( ref a, value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int AtomicLoadInt( ref int a ) => Volatile.Read( ref a );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int AtomicFetchAddInt( ref int a, int increment ) => Interlocked.Add( ref a, increment ) - increment;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool AtomicCompareExchangeInt( ref int a, int expected, int desired ) =>
		Interlocked.CompareExchange( ref a, desired, expected ) == expected;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void AtomicStoreU32( ref uint a, uint value ) => Volatile.Write( ref a, value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint AtomicLoadU32( ref uint a ) => Volatile.Read( ref a );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint AtomicLoadU32Raw( ref uint a ) => Volatile.Read( ref a );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint AtomicFetchOrU32( ref uint a, uint mask ) => Interlocked.Or( ref a, mask );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static long AtomicFetchAddI64( ref long a, long increment ) => Interlocked.Add( ref a, increment ) - increment;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static long AtomicLoadI64( ref long a ) => Volatile.Read( ref a );
}

internal struct AtomicInt
{
	public int value;
}

internal struct AtomicU32
{
	public uint value;
}

internal struct AtomicI64
{
	public long value;
}

public static partial class B2
{
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void AtomicStoreInt( ref AtomicInt a, int value ) => Volatile.Write( ref a.value, value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int AtomicLoadInt( ref AtomicInt a ) => Volatile.Read( ref a.value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int AtomicFetchAddInt( ref AtomicInt a, int increment ) => Interlocked.Add( ref a.value, increment ) - increment;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool AtomicCompareExchangeInt( ref AtomicInt a, int expected, int desired ) =>
		Interlocked.CompareExchange( ref a.value, desired, expected ) == expected;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void AtomicStoreU32( ref AtomicU32 a, uint value ) => Volatile.Write( ref a.value, value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint AtomicLoadU32( ref AtomicU32 a ) => Volatile.Read( ref a.value );
}
