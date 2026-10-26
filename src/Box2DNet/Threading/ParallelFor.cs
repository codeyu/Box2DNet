// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Threading;

namespace Box2DNet;

// Callback invoked by ParallelFor to process a range of items. May be called multiple times per
// worker: work is divided into blocks that workers claim atomically, so a worker that finishes
// early picks up the next unclaimed block instead of sitting idle. workerIndex is the worker
// identity and is stable across all invocations from the same worker, so it is safe to use as an
// index into per-worker state.
internal delegate void ParallelForCallback( int startIndex, int endIndex, int workerIndex, object? context );

public static partial class B2
{
	// Shared state for one ParallelFor invocation. Workers race on nextBlock to claim work, so a slow
	// chunk can't strand the other threads.
	private sealed class ParallelForShared
	{
		public int nextBlock;
		public int blockCount;
		public int blockSize;
		public int itemCount;
		public ParallelForCallback callback = null!;
		public object? context;
	}

	private sealed class ParallelForTask
	{
		public ParallelForShared shared = null!;
		public int workerIndex;
	}

	private static TaskCallbackFcn s_parallelForTrampoline = ParallelForTrampoline;

	private static void ParallelForTrampoline( object? taskContext )
	{
		ParallelForTask task = (ParallelForTask)taskContext!;
		ParallelForShared shared = task.shared;
		int workerIndex = task.workerIndex;
		object? context = shared.context;
		ParallelForCallback callback = shared.callback;
		int blockCount = shared.blockCount;
		int blockSize = shared.blockSize;
		int itemCount = shared.itemCount;

		for ( ;; )
		{
			int blockIndex = Interlocked.Increment( ref shared.nextBlock ) - 1;
			if ( blockIndex >= blockCount )
			{
				break;
			}

			int start = blockIndex * blockSize;
			int end = start + blockSize;
			if ( end > itemCount )
			{
				end = itemCount;
			}

			callback( start, end, workerIndex, context );
		}
	}

	// Divide [0, itemCount) into blocks and process them with cooperative claiming: up to
	// world.workerCount tasks are enqueued, and each task loops, atomically claiming the next
	// unclaimed block until the range is drained. Blocks the caller until all work is complete.
	// minRange is the minimum block size.
	internal static void ParallelFor( World world, ParallelForCallback callback, int itemCount, int minRange, object? context )
	{
		if ( itemCount <= 0 )
		{
			return;
		}

		B2_ASSERT( minRange > 0 );

		int workerCount = world.workerCount;
		B2_ASSERT( 0 < workerCount && workerCount <= MaxWorkers );

		// Target multiple blocks per worker to reduce thread stalls.
		// block size grows once items exceed maxBlockCount * minRange
		// so the block count stays bounded and per-block sync overhead stays low.
		int blocksPerWorker = 8;
		int maxBlockCount = blocksPerWorker * workerCount;
		int blockSize;
		int blockCount;
		if ( itemCount <= minRange * maxBlockCount )
		{
			blockSize = minRange;
			blockCount = ( itemCount + blockSize - 1 ) / blockSize;
		}
		else
		{
			blockSize = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
			blockCount = ( itemCount + blockSize - 1 ) / blockSize;
		}

		B2_ASSERT( blockCount >= 1 );
		B2_ASSERT( blockSize * blockCount >= itemCount );

		// No point enqueueing more tasks than blocks.
		int taskCount = workerCount < blockCount ? workerCount : blockCount;

		ParallelForShared shared = new()
		{
			blockCount = blockCount,
			blockSize = blockSize,
			itemCount = itemCount,
			callback = callback,
			context = context,
			nextBlock = 0,
		};

		Span<object?> handles = new object?[MaxWorkers];
		for ( int i = 0; i < taskCount; ++i )
		{
			ParallelForTask task = new() { shared = shared, workerIndex = i };

			if ( world.taskCount < MaxTasks )
			{
				handles[i] = world.enqueueTaskFcn!( s_parallelForTrampoline, task, world.userTaskContext );
				world.taskCount += 1;
			}
			else
			{
				handles[i] = null;
				ParallelForTrampoline( task );
			}
		}

		for ( int i = 0; i < taskCount; ++i )
		{
			if ( handles[i] != null )
			{
				world.finishTaskFcn!( handles[i], world.userTaskContext );
			}
		}
	}
}
