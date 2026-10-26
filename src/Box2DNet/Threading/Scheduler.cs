// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System.Threading;

namespace Box2DNet;

// The built-in task system. Box2D runs its own threads when no task callbacks are hooked up.
internal sealed class Scheduler
{
	private const float B2_SCHEDULER_SPIN_MILLISECONDS = 0.05f;

	private const int b2_schedulerFree = 0;
	private const int b2_schedulerPending = 1;
	private const int b2_schedulerClaimed = 2;
	private const int b2_schedulerComplete = 3;

	// Task slots are reused every step. The slot object is handed to the user as the task handle.
	internal sealed class SchedulerTask
	{
		public TaskCallbackFcn? callback;
		public object? taskContext;
		public int status;
	}

	private readonly Thread?[] threads = new Thread?[B2.MaxWorkers];

	// total workers including main thread
	private readonly int workerCount;

	// threads created = workerCount - 1
	private readonly int threadCount;

	private readonly SchedulerTask[] tasks = new SchedulerTask[B2.MaxTasks];
	private int nextSlot;

	private readonly SemaphoreSlim taskSemaphore;
	private int sleeperCount;
	private int shutdown;

	public Scheduler( int workerCount )
	{
		B2.B2_ASSERT( 0 < workerCount && workerCount <= B2.MaxWorkers );

		for ( int i = 0; i < tasks.Length; ++i )
		{
			tasks[i] = new SchedulerTask();
		}

		this.workerCount = workerCount;
		threadCount = workerCount - 1;
		taskSemaphore = new SemaphoreSlim( 0 );
		Volatile.Write( ref sleeperCount, 0 );
		Volatile.Write( ref shutdown, 0 );
		Volatile.Write( ref nextSlot, 0 );

		// Background threads use indices 1..workerCount-1.
		// Main thread uses index 0.
		for ( int i = 0; i < threadCount; ++i )
		{
			Thread thread = new( WorkerMain ) { IsBackground = true, Name = $"box2d_worker_{i + 1:D2}" };
			threads[i] = thread;
			thread.Start();
		}
	}

	public int WorkerCount => workerCount;

	private bool HasPending()
	{
		int taskCount = Volatile.Read( ref nextSlot );
		for ( int t = 0; t < taskCount; ++t )
		{
			if ( Volatile.Read( ref tasks[t].status ) == b2_schedulerPending )
			{
				return true;
			}
		}

		return false;
	}

	private bool Spin()
	{
		ulong ticks = B2.GetTicks();
		while ( true )
		{
			if ( HasPending() || Volatile.Read( ref shutdown ) != 0 )
			{
				return true;
			}

			if ( B2.GetMilliseconds( ticks ) >= B2_SCHEDULER_SPIN_MILLISECONDS )
			{
				return false;
			}

			Thread.SpinWait( 8 );
		}
	}

	private bool TakeSleeper()
	{
		int count = Volatile.Read( ref sleeperCount );
		while ( count > 0 )
		{
			if ( Interlocked.CompareExchange( ref sleeperCount, count - 1, count ) == count )
			{
				return true;
			}

			count = Volatile.Read( ref sleeperCount );
		}

		return false;
	}

	// Try to claim and execute one pending task.
	// Returns true if work was performed, false otherwise.
	private bool ExecuteOne()
	{
		int taskCount = Volatile.Read( ref nextSlot );
		for ( int t = 0; t < taskCount; ++t )
		{
			SchedulerTask task = tasks[t];
			if ( Volatile.Read( ref task.status ) != b2_schedulerPending )
			{
				continue;
			}

			if ( Interlocked.CompareExchange( ref task.status, b2_schedulerClaimed, b2_schedulerPending ) != b2_schedulerPending )
			{
				continue;
			}

			task.callback!( task.taskContext );

			Volatile.Write( ref task.status, b2_schedulerComplete );
			return true;
		}

		return false;
	}

	// Background worker thread entry point.
	private void WorkerMain()
	{
		while ( true )
		{
			// Claim and execute all available work
			while ( ExecuteOne() )
			{
			}

			if ( Volatile.Read( ref shutdown ) != 0 )
			{
				break;
			}

			if ( Spin() )
			{
				continue;
			}

			Interlocked.Increment( ref sleeperCount );

			if ( HasPending() || Volatile.Read( ref shutdown ) != 0 )
			{
				TakeSleeper();
				continue;
			}

			taskSemaphore.Wait();
		}
	}

	public void Destroy()
	{
		Volatile.Write( ref shutdown, 1 );

		// Wake all background threads so they see the shutdown flag
		for ( int i = 0; i < threadCount; ++i )
		{
			taskSemaphore.Release();
		}

		for ( int i = 0; i < threadCount; ++i )
		{
			threads[i]!.Join();
			threads[i] = null;
		}

		taskSemaphore.Dispose();
	}

	public void Reset()
	{
		Volatile.Write( ref nextSlot, 0 );
	}

	// See EnqueueTaskCallbackFcn
	public static object? EnqueueTask( TaskCallbackFcn task, object? taskContext, object? userContext )
	{
		Scheduler scheduler = (Scheduler)userContext!;

		int slot = Interlocked.Increment( ref scheduler.nextSlot ) - 1;
		B2.B2_ASSERT( slot < B2.MaxTasks );

		SchedulerTask schedulerTask = scheduler.tasks[slot];
		schedulerTask.callback = task;
		schedulerTask.taskContext = taskContext;

		// Memory fence: status must be published after callback and context are written
		Volatile.Write( ref schedulerTask.status, b2_schedulerPending );

		// One wake per enqueue is enough: at most one worker picks up each task.
		if ( scheduler.TakeSleeper() )
		{
			scheduler.taskSemaphore.Release();
		}

		return schedulerTask;
	}

	// See FinishTaskCallbackFcn
	public static void FinishTask( object? userTask, object? userContext )
	{
		if ( userTask == null )
		{
			return;
		}

		Scheduler scheduler = (Scheduler)userContext!;
		SchedulerTask waitTask = (SchedulerTask)userTask;

		// Main thread helps execute any available work while waiting for the
		// target task to complete. This keeps the main thread from idling when
		// background threads are busy on other tasks from the same phase.
		while ( Volatile.Read( ref waitTask.status ) != b2_schedulerComplete )
		{
			if ( scheduler.ExecuteOne() == false )
			{
				Thread.Yield();
			}
		}
	}
}

public static partial class B2
{
	internal static Scheduler CreateScheduler( int workerCount ) => new Scheduler( workerCount );

	internal static void DestroyScheduler( Scheduler scheduler ) => scheduler.Destroy();

	internal static void ResetScheduler( Scheduler scheduler ) => scheduler.Reset();

	internal static object? SchedulerEnqueueTask( TaskCallbackFcn task, object? taskContext, object? userContext ) =>
		Scheduler.EnqueueTask( task, taskContext, userContext );

	internal static void SchedulerFinishTask( object? userTask, object? userContext ) => Scheduler.FinishTask( userTask, userContext );

	// .NET does not flush denormals, so determinism is preserved.
	internal static bool IsDenormalFlushEnabled() => false;
}
