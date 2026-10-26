// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class ThreadTests
{
	// Semaphore tests

	private static int SemaphoreCreateDestroyTest()
	{
		ref Semaphore s = ref CreateSemaphore( 0 );
		ENSURE( Unsafe.IsNullRef( ref s ) == false );
		DestroySemaphore( s );
		return 0;
	}

	internal struct SemData
	{
		public /*TODO pointer*/ Semaphore[] sem;
		public int value;
	}

	private static void SemWorker( object? context )
	{
		SemData* data = (SemData*)context;
		data.value = 99;
		SignalSemaphore( data.sem );
	}

	private static int SemaphoreSignalWaitTest()
	{
		SemData data = default;
		data.sem = CreateSemaphore( 0 );
		data.value = 0;

		ref Thread thread = ref CreateThread( SemWorker, ref data, "sem test" );
		WaitSemaphore( data.sem );

		ENSURE( data.value == 99 );

		JoinThread( thread );
		DestroySemaphore( data.sem );
		return 0;
	}

	private static int SemaphoreInitialCountTest()
	{
		// Semaphore initialized to 3 should allow 3 waits without blocking
		ref Semaphore s = ref CreateSemaphore( 3 );
		// These should not block since count starts at 3
		WaitSemaphore( s );
		WaitSemaphore( s );
		WaitSemaphore( s );

		// Signal once so the next wait doesn't block
		SignalSemaphore( s );
		WaitSemaphore( s );

		DestroySemaphore( s );
		return 0;
	}

	// Thread tests

	private static int ThreadCreateJoinTest()
	{
		SemData data = default;
		data.sem = CreateSemaphore( 0 );
		data.value = 0;

		ref Thread thread = ref CreateThread( SemWorker, ref data, "join test" );
		JoinThread( thread );

		ENSURE( data.value == 99 );

		DestroySemaphore( data.sem );
		return 0;
	}

	internal struct SumData
	{
		public /*TODO pointer*/ Mutex[] mutex;
		public int sum;
	}

	private static void SumWorker( object? context )
	{
		SumData* data = (SumData*)context;
		for ( int i = 0; i < 1000; ++i )
		{
			LockMutex( data.mutex );
			data.sum += 1;
			UnlockMutex( data.mutex );
		}
	}

	private static int ThreadMultipleTest()
	{
		SumData data = default;
		data.mutex = CreateMutex();
		data.sum = 0;

		enum { THREAD_COUNT = 4 };
		Thread* threads[THREAD_COUNT];
		for ( int i = 0; i < THREAD_COUNT; ++i )
		{
			Span<byte> name = stackalloc byte[16];
			snprintf( name, Unsafe.SizeOf<name>(), "sum test %d", i );
			threads[i] = CreateThread( SumWorker, ref data, name );
		}

		for ( int i = 0; i < THREAD_COUNT; ++i )
		{
			JoinThread( threads[i] );
		}

		ENSURE( data.sum == THREAD_COUNT * 1000 );

		DestroyMutex( data.mutex );
		return 0;
	}

	internal static int ThreadTest()
	{
		RUN_SUBTEST( SemaphoreCreateDestroyTest );
		RUN_SUBTEST( SemaphoreSignalWaitTest );
		RUN_SUBTEST( SemaphoreInitialCountTest );
		RUN_SUBTEST( ThreadCreateJoinTest );
		RUN_SUBTEST( ThreadMultipleTest );
		return 0;
	}

}
