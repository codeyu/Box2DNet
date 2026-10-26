// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.CompilerServices;

namespace Box2DNet;

/// Dynamic array mirroring b2Array(T). This is a struct with the same value semantics as the C
/// version: a default instance is a valid empty array, and copies share storage. Always access
/// an array through a ref or a field path so that count changes are not lost.
internal struct B2Array<T>
{
	public T[] data;
	public int count;
	public int capacity;

	public static B2Array<T> Create( int n )
	{
		B2Array<T> a;
		a.data = n > 0 ? B2.Alloc<T>( n ) : Array.Empty<T>();
		a.count = 0;
		a.capacity = n;
		return a;
	}

	// Alternative to zero initialization with an initial capacity
	public void CreateN( int n )
	{
		this = Create( n );
	}

	public void Destroy()
	{
		B2.Free( data, capacity );
		data = Array.Empty<T>();
		count = 0;
		capacity = 0;
	}

	/// Get a reference to an element
	public readonly ref T this[int index]
	{
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
		get
		{
			B2.B2_ASSERT( 0 <= index && index < count );
			return ref data[index];
		}
	}

	/// Get a reference to an element
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public readonly ref T Get( int index )
	{
		B2.B2_ASSERT( 0 <= index && index < count );
		return ref data[index];
	}

	public readonly Span<T> AsSpan() => data.AsSpan( 0, count );

	public void Reserve( int n )
	{
		if ( capacity < n )
		{
			data = B2.GrowAlloc( data, capacity, n );
			capacity = n;
		}
	}

	public void Grow( int n )
	{
		if ( capacity < n )
		{
			int newCapacity = 2 * capacity > n ? 2 * capacity : n;
			Reserve( newCapacity );
		}
	}

	public void Resize( int n )
	{
		Reserve( n );
		count = n;
	}

	public void ResizeAndSetZero( int n )
	{
		Reserve( n );
		Array.Clear( data, 0, n );
		count = n;
	}

	/// Push a new element by value
	public void Push( T value )
	{
		if ( count >= capacity )
		{
			int newCapacity = capacity == 0 ? 8 : 2 * capacity;
			data = B2.GrowAlloc( data, capacity, newCapacity );
			capacity = newCapacity;
		}

		data[count] = value;
		count += 1;
	}

	/// Create a new element and return a reference to it. Unlike C the element is reset to default
	/// because managed arrays can hold stale data from removed elements.
	public ref T Emplace()
	{
		if ( count >= capacity )
		{
			int newCapacity = capacity == 0 ? 8 : 2 * capacity;
			data = B2.GrowAlloc( data, capacity, newCapacity );
			capacity = newCapacity;
		}

		ref T element = ref data[count];
		element = default!;
		count += 1;
		return ref element;
	}

	/// Remove the last element and return it by value.
	public T Pop()
	{
		B2.B2_ASSERT( 0 < count );
		count -= 1;
		T value = data[count];
		if ( RuntimeHelpers.IsReferenceOrContainsReferences<T>() )
		{
			data[count] = default!;
		}

		return value;
	}

	/// Remove an element by swapping with the last element. If the index is the last element it returns
	/// B2_NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
	public int RemoveSwap( int index )
	{
		B2.B2_ASSERT( 0 <= index && index < count );
		count -= 1;
		if ( index != count )
		{
			data[index] = data[count];
			if ( RuntimeHelpers.IsReferenceOrContainsReferences<T>() )
			{
				data[count] = default!;
			}

			return count;
		}

		if ( RuntimeHelpers.IsReferenceOrContainsReferences<T>() )
		{
			data[count] = default!;
		}

		return B2.NullIndex;
	}

	public void Clear()
	{
		count = 0;
	}

	public readonly int ByteCount() => capacity * Unsafe.SizeOf<T>();
}

/// Fixed size inline arrays used in place of C fixed arrays.
[InlineArray( 2 )]
public struct FixedArray2<T>
{
	private T _element0;
}

[InlineArray( 3 )]
public struct FixedArray3<T>
{
	private T _element0;

	/// Index by body type, mirroring C arrays sized by b2_bodyTypeCount.
	[System.Diagnostics.CodeAnalysis.UnscopedRef]
	public ref T this[BodyType index] => ref this[(int)index];
}

[InlineArray( 4 )]
public struct FixedArray4<T>
{
	private T _element0;
}

[InlineArray( 8 )]
public struct FixedArray8<T>
{
	private T _element0;
}

[InlineArray( 24 )]
public struct FixedArray24<T>
{
	private T _element0;
}

[InlineArray( 32 )]
public struct FixedArray32<T>
{
	private T _element0;
}
