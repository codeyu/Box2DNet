// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Threading;

namespace Box2DNet;

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
public delegate int AssertCallbackFcn( string condition, string fileName, int lineNumber );

/// Prototype for user log callback. Used to log warnings.
public delegate void LogCallbackFcn( string message );

/// Version numbering scheme.
/// See https://semver.org/
public struct Version
{
	/// Significant changes
	public int major;

	/// Incremental changes
	public int minor;

	/// Bug fixes
	public int revision;
}

public static partial class B2
{
	/// Used to indicate an unset or invalid index value.
	public const int NullIndex = -1;

	/// Significant API and behavior changes.
	public const int VersionMajor = 3;

	/// API changes and features.
	public const int VersionMinor = 2;

	/// Bug fixes. No API changes.
	public const int VersionRevision = 0;

	/// Starting hash value.
	public const uint HashInit = 5381;

	// Use to validate definitions. Do not take my cookie.
	internal const int SecretCookie = 1152023;

	private static int DefaultAssertFcn( string condition, string fileName, int lineNumber )
	{
		Console.Error.WriteLine( $"BOX2D ASSERTION: {condition}, {fileName}, line {lineNumber}" );
		Console.Error.Flush();

		// return non-zero to break to debugger
		return 1;
	}

	private static AssertCallbackFcn s_assertHandler = DefaultAssertFcn;

	/// Override the default assert function
	/// @param assertFcn a non-null assert callback
	public static void SetAssertCallback( AssertCallbackFcn assertFcn )
	{
		B2_ASSERT( assertFcn != null );
		s_assertHandler = assertFcn;
	}

	[Conditional( "DEBUG" )]
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void B2_ASSERT( [DoesNotReturnIf( false )] bool condition,
									[CallerArgumentExpression( nameof( condition ) )] string expression = "",
									[CallerFilePath] string file = "", [CallerLineNumber] int line = 0 )
	{
		if ( condition == false )
		{
			InternalAssert( expression, file, line );
		}
	}

	private static void InternalAssert( string condition, string fileName, int lineNumber )
	{
		int result = s_assertHandler( condition, fileName, lineNumber );
		if ( result != 0 )
		{
			if ( Debugger.IsAttached )
			{
				Debugger.Break();
			}
			else
			{
				throw new AssertionException( $"{condition} ({fileName}:{lineNumber})" );
			}
		}
	}

	// Validation is used in debug builds
	[Conditional( "BOX2D_VALIDATE" )]
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void B2_VALIDATE( bool condition, [CallerArgumentExpression( nameof( condition ) )] string expression = "",
									  [CallerFilePath] string file = "", [CallerLineNumber] int line = 0 )
	{
		if ( condition == false )
		{
			InternalAssert( expression, file, line );
		}
	}

	/// Used to validate API inputs. Logs, asserts in debug builds, and returns the condition so the
	/// caller can return early. This mirrors B2_CHECK_INPUT.
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool CheckInput( bool condition, [CallerArgumentExpression( nameof( condition ) )] string expression = "",
									 [CallerMemberName] string member = "", [CallerFilePath] string file = "",
									 [CallerLineNumber] int line = 0 )
	{
		if ( condition == false )
		{
			InvalidInput( expression, member, file, line );
			return false;
		}

		return true;
	}

	private static void InvalidInput( string expression, string member, string file, int line )
	{
		Log( $"invalid input: {expression} in {member}" );
#if DEBUG
		InternalAssert( expression, file, line );
#endif
	}

	private static void DefaultLogFcn( string message )
	{
		Console.WriteLine( $"Box2D: {message}" );
	}

	private static LogCallbackFcn s_logHandler = DefaultLogFcn;

	/// Override the default log function
	/// @param logFcn a non-null log callback
	public static void SetLogCallback( LogCallbackFcn logFcn )
	{
		B2_ASSERT( logFcn != null );
		s_logHandler = logFcn;
	}

	internal static void Log( string message )
	{
		s_logHandler( message );
	}

	/// Get the current version of Box2D
	public static Version GetVersion()
	{
		return new Version { major = VersionMajor, minor = VersionMinor, revision = VersionRevision };
	}

	/// @return true if the library was built with BOX2D_DOUBLE_PRECISION (large world mode)
	public static bool IsDoublePrecision() => false;

	/// @return true if the current CPU has the AVX2 instruction set.
	public static bool IsAVX2Available() => System.Runtime.Intrinsics.X86.Avx2.IsSupported;

	private static long s_byteCount;

	/// @return the total bytes allocated by Box2D
	public static long GetByteCount() => Interlocked.Read( ref s_byteCount );

	// Allocation tracking. Managed arrays are used instead of raw memory, but the byte count
	// is still tracked to match the C API.
	internal static T[] Alloc<T>( int count )
	{
		if ( count == 0 )
		{
			return Array.Empty<T>();
		}

		Interlocked.Add( ref s_byteCount, (long)count * Unsafe.SizeOf<T>() );
		return new T[count];
	}

	internal static void Free<T>( T[]? mem, int count )
	{
		if ( mem == null || count == 0 )
		{
			return;
		}

		Interlocked.Add( ref s_byteCount, -(long)count * Unsafe.SizeOf<T>() );
	}

	// Grow an array, keeping the old content. New elements are zero.
	internal static T[] GrowAlloc<T>( T[]? oldMem, int oldCount, int newCount )
	{
		B2_ASSERT( newCount > oldCount );
		T[] newMem = Alloc<T>( newCount );
		if ( oldCount > 0 )
		{
			Array.Copy( oldMem!, newMem, oldCount );
			Free( oldMem, oldCount );
		}

		return newMem;
	}

	/// Get the absolute number of system ticks. The value is platform specific.
	public static ulong GetTicks() => (ulong)Stopwatch.GetTimestamp();

	/// Get the milliseconds passed from an initial tick value.
	public static float GetMilliseconds( ulong ticks )
	{
		ulong ticksNow = GetTicks();
		return (float)( 1000.0 / Stopwatch.Frequency * ( ticksNow - ticks ) );
	}

	/// Get the milliseconds passed from an initial tick value. Resets the passed in value to the
	/// current tick value.
	public static float GetMillisecondsAndReset( ref ulong ticks )
	{
		ulong ticksNow = GetTicks();
		float ms = (float)( 1000.0 / Stopwatch.Frequency * ( ticksNow - ticks ) );
		ticks = ticksNow;
		return ms;
	}

	/// Yield to be used in a busy loop.
	public static void Yield() => Thread.Yield();

	/// Simple djb2 hash function for determinism testing
	public static uint Hash( uint hash, ReadOnlySpan<byte> data )
	{
		uint result = hash;
		for ( int i = 0; i < data.Length; i++ )
		{
			result = ( result << 5 ) + result + data[i];
		}

		return result;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void Swap<T>( ref T x, ref T y )
	{
		T temp = x;
		x = y;
		y = temp;
	}
}

/// Thrown when a Box2D assertion fails and no debugger is attached.
public sealed class AssertionException : Exception
{
	public AssertionException( string message ) : base( message )
	{
	}
}
