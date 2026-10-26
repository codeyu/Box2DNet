// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Buffers;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace Box2DNet;

// Bit set provides fast operations on large arrays of bits.
internal struct BitSet
{
	public ulong[] bits;
	public uint blockCapacity;
	public uint blockCount;

	public static BitSet Create( uint bitCapacity )
	{
		BitSet bitSet = default;
		bitSet.blockCapacity = ( bitCapacity + sizeof( ulong ) * 8 - 1 ) / ( sizeof( ulong ) * 8 );
		bitSet.blockCount = 0;
		bitSet.bits = B2.Alloc<ulong>( (int)bitSet.blockCapacity );
		return bitSet;
	}

	public void Destroy()
	{
		B2.Free( bits, (int)blockCapacity );
		blockCapacity = 0;
		blockCount = 0;
		bits = null!;
	}

	public void SetBitCountAndClear( uint bitCount )
	{
		uint newBlockCount = ( bitCount + sizeof( ulong ) * 8 - 1 ) / ( sizeof( ulong ) * 8 );
		if ( blockCapacity < newBlockCount )
		{
			Destroy();
			uint newBitCapacity = bitCount + ( bitCount >> 1 );
			this = Create( newBitCapacity );
		}

		blockCount = newBlockCount;
		Array.Clear( bits, 0, (int)blockCount );
	}

	public void Grow( uint newBlockCount )
	{
		B2.B2_ASSERT( newBlockCount > blockCount );
		if ( newBlockCount > blockCapacity )
		{
			uint oldCapacity = blockCapacity;
			blockCapacity = newBlockCount + newBlockCount / 2;
			ulong[] newBits = B2.Alloc<ulong>( (int)blockCapacity );
			B2.B2_ASSERT( bits != null );
			Array.Copy( bits, newBits, (int)oldCapacity );
			B2.Free( bits, (int)oldCapacity );
			bits = newBits;
		}

		blockCount = newBlockCount;
	}

	public void InPlaceUnion( in BitSet setB )
	{
		B2.B2_ASSERT( blockCount == setB.blockCount );
		uint count = blockCount;
		for ( uint i = 0; i < count; ++i )
		{
			bits[i] |= setB.bits[i];
		}
	}

	public readonly int CountSetBits()
	{
		int popCount = 0;
		uint count = blockCount;
		for ( uint i = 0; i < count; ++i )
		{
			popCount += BitOperations.PopCount( bits[i] );
		}

		return popCount;
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public readonly void SetBit( uint bitIndex )
	{
		uint blockIndex = bitIndex / 64;
		B2.B2_ASSERT( blockIndex < blockCount );
		bits[blockIndex] |= 1ul << (int)( bitIndex % 64 );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public void SetBitGrow( uint bitIndex )
	{
		uint blockIndex = bitIndex / 64;
		if ( blockIndex >= blockCount )
		{
			Grow( blockIndex + 1 );
		}

		bits[blockIndex] |= 1ul << (int)( bitIndex % 64 );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public readonly void ClearBit( uint bitIndex )
	{
		uint blockIndex = bitIndex / 64;
		if ( blockIndex >= blockCount )
		{
			return;
		}

		bits[blockIndex] &= ~( 1ul << (int)( bitIndex % 64 ) );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public readonly bool GetBit( uint bitIndex )
	{
		uint blockIndex = bitIndex / 64;
		if ( blockIndex >= blockCount )
		{
			return false;
		}

		return ( bits[blockIndex] & ( 1ul << (int)( bitIndex % 64 ) ) ) != 0;
	}

	public readonly int GetBytes() => (int)blockCapacity * sizeof( ulong );
}

internal struct IdPool
{
	public B2Array<int> freeArray;
	public int nextIndex;

	public static IdPool Create()
	{
		IdPool pool = default;
		pool.freeArray = B2Array<int>.Create( 32 );
		return pool;
	}

	public void Destroy()
	{
		freeArray.Destroy();
		this = default;
	}

	public int AllocId()
	{
		int count = freeArray.count;
		if ( count > 0 )
		{
			int id = freeArray.Pop();
			return id;
		}

		int newId = nextIndex;
		nextIndex += 1;
		return newId;
	}

	public void FreeId( int id )
	{
		B2.B2_ASSERT( nextIndex > 0 );
		B2.B2_ASSERT( 0 <= id && id < nextIndex );
		freeArray.Push( id );
	}

	[System.Diagnostics.Conditional( "BOX2D_VALIDATE" )]
	public readonly void ValidateFreeId( int id )
	{
		int freeCount = freeArray.count;
		for ( int i = 0; i < freeCount; ++i )
		{
			if ( freeArray.data[i] == id )
			{
				return;
			}
		}

		B2.B2_ASSERT( false );
	}

	[System.Diagnostics.Conditional( "BOX2D_VALIDATE" )]
	public readonly void ValidateUsedId( int id )
	{
		int freeCount = freeArray.count;
		for ( int i = 0; i < freeCount; ++i )
		{
			if ( freeArray.data[i] == id )
			{
				B2.B2_ASSERT( false );
			}
		}
	}

	public readonly int GetIdCount() => nextIndex - freeArray.count;

	public readonly int GetIdCapacity() => nextIndex;

	public readonly int GetIdBytes() => freeArray.ByteCount();
}

// Open addressing hash set of non-zero 64-bit keys.
internal struct HashSet64
{
	public ulong[] items;
	public uint capacity;
	public uint count;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static ulong ShapePairKey( int k1, int k2 )
	{
		return k1 < k2 ? (ulong)(uint)k1 << 32 | (uint)k2 : (ulong)(uint)k2 << 32 | (uint)k1;
	}

	// I need a good hash because the keys are built from pairs of increasing integers.
	// A simple hash like hash = (integer1 XOR integer2) has many collisions.
	// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
	// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	public static ulong KeyHash( ulong key )
	{
		// Murmur hash
		ulong h = key;
		h ^= h >> 33;
		h *= 0xff51afd7ed558ccdUL;
		h ^= h >> 33;
		h *= 0xc4ceb9fe1a85ec53UL;
		h ^= h >> 33;

		return h;
	}

	public static HashSet64 Create( int capacity )
	{
		HashSet64 set = default;

		// Capacity must be a power of 2
		if ( capacity > 16 )
		{
			set.capacity = (uint)B2.RoundUpPowerOf2( capacity );
		}
		else
		{
			set.capacity = 16;
		}

		set.count = 0;
		set.items = B2.Alloc<ulong>( (int)set.capacity );
		return set;
	}

	public void Destroy()
	{
		B2.Free( items, (int)capacity );
		items = null!;
		count = 0;
		capacity = 0;
	}

	public void Clear()
	{
		count = 0;
		Array.Clear( items, 0, (int)capacity );
	}

	private readonly int FindSlot( ulong key, ulong hash )
	{
		uint cap = capacity;
		uint index = (uint)hash & ( cap - 1 );
		ulong[] keys = items;
		while ( keys[index] != 0 && keys[index] != key )
		{
			index = ( index + 1 ) & ( cap - 1 );
		}

		return (int)index;
	}

	private void AddKeyHaveCapacity( ulong key, ulong hash )
	{
		int index = FindSlot( key, hash );
		B2.B2_ASSERT( items[index] == 0 );
		items[index] = key;
		count += 1;
	}

	private void GrowTable()
	{
		uint oldCount = count;
		uint oldCapacity = capacity;
		ulong[] oldItems = items;

		count = 0;
		// Capacity must be a power of 2
		capacity = 2 * oldCapacity;
		items = B2.Alloc<ulong>( (int)capacity );

		// Transfer items into new array
		for ( uint i = 0; i < oldCapacity; ++i )
		{
			ulong key = oldItems[i];
			if ( key == 0 )
			{
				// this item was empty
				continue;
			}

			ulong hash = KeyHash( key );
			AddKeyHaveCapacity( key, hash );
		}

		B2.B2_ASSERT( count == oldCount );
		B2.Free( oldItems, (int)oldCapacity );
	}

	public readonly bool ContainsKey( ulong key )
	{
		// key of zero is a sentinel
		B2.B2_VALIDATE( key != 0 );
		ulong hash = KeyHash( key );
		int index = FindSlot( key, hash );
		return items[index] == key;
	}

	public readonly bool ContainsHashedKey( ulong key, ulong hash )
	{
		// key of zero is a sentinel
		B2.B2_VALIDATE( key != 0 );
		int index = FindSlot( key, hash );
		return items[index] == key;
	}

	public readonly int GetBytes() => (int)capacity * sizeof( ulong );

	// Returns true if key was already in set
	public bool AddKey( ulong key )
	{
		// key of zero is a sentinel
		B2.B2_ASSERT( key != 0 );

		ulong hash = KeyHash( key );
		B2.B2_ASSERT( hash != 0 );

		int index = FindSlot( key, hash );
		if ( items[index] != 0 )
		{
			// Already in set
			B2.B2_ASSERT( items[index] == key );
			return true;
		}

		if ( 2 * count >= capacity )
		{
			GrowTable();
		}

		AddKeyHaveCapacity( key, hash );
		return false;
	}

	// Returns true if the key was found
	// See https://en.wikipedia.org/wiki/Open_addressing
	public bool RemoveKey( ulong key )
	{
		ulong hash = KeyHash( key );
		int i = FindSlot( key, hash );
		ulong[] keys = items;
		if ( keys[i] == 0 )
		{
			// Not in set
			return false;
		}

		// Mark item i as unoccupied
		keys[i] = 0;

		B2.B2_ASSERT( count > 0 );
		count -= 1;

		// Attempt to fill item i
		int j = i;
		uint cap = capacity;
		for ( ;; )
		{
			j = (int)( ( (uint)j + 1 ) & ( cap - 1 ) );
			if ( keys[j] == 0 )
			{
				break;
			}

			// k is the first item for the hash of j
			ulong hash_j = KeyHash( keys[j] );
			int k = (int)( hash_j & ( cap - 1 ) );

			// determine if k lies cyclically in (i,j]
			// i <= j: | i..k..j |
			// i > j: |.k..j  i....| or |....j     i..k.|
			if ( i <= j )
			{
				if ( i < k && k <= j )
				{
					continue;
				}
			}
			else
			{
				if ( i < k || k <= j )
				{
					continue;
				}
			}

			// Move j into i
			keys[i] = keys[j];

			// Mark item j as unoccupied
			keys[j] = 0;

			i = j;
		}

		return true;
	}

	public readonly int GetCount() => (int)count;

	public readonly int GetCapacity() => (int)capacity;
}

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// The managed version rents typed arrays from a pool.
internal sealed class ArenaAllocator
{
	private struct StackEntry
	{
		public object data;
		public string name;
		public int size;
	}

	private B2Array<StackEntry> m_entries;
	private int m_allocation;
	private int m_maxAllocation;
	private int m_capacity;

	public ArenaAllocator( int capacity )
	{
		B2.B2_ASSERT( capacity >= 0 );
		m_capacity = capacity;
		m_entries = B2Array<StackEntry>.Create( 32 );
	}

	public void Destroy()
	{
		m_entries.Destroy();
	}

	// Allocate an array with at least count elements. The content is not cleared.
	public T[] Alloc<T>( int count, string name )
	{
		int size = count * Unsafe.SizeOf<T>();
		int alignedSize = size == 0 ? 0 : ( ( size - 1 ) | ( 64 - 1 ) ) + 1;

		T[] data = count > 0 ? ArrayPool<T>.Shared.Rent( count ) : Array.Empty<T>();

		StackEntry entry;
		entry.data = data;
		entry.name = name;
		entry.size = alignedSize;

		m_allocation += alignedSize;
		if ( m_allocation > m_maxAllocation )
		{
			m_maxAllocation = m_allocation;
		}

		m_entries.Push( entry );
		return data;
	}

	// Allocate and clear
	public T[] AllocZero<T>( int count, string name )
	{
		T[] data = Alloc<T>( count, name );
		Array.Clear( data, 0, count );
		return data;
	}

	public void Free<T>( T[] mem )
	{
		int entryCount = m_entries.count;
		B2.B2_ASSERT( entryCount > 0 );
		ref StackEntry entry = ref m_entries[entryCount - 1];
		B2.B2_ASSERT( ReferenceEquals( mem, entry.data ) );

		if ( mem.Length > 0 )
		{
			ArrayPool<T>.Shared.Return( mem, RuntimeHelpers.IsReferenceOrContainsReferences<T>() );
		}

		m_allocation -= entry.size;
		m_entries.Pop();
	}

	// Free an allocation that was handed out as a span. The span must start at the array.
	public void Free<T>( Span<T> mem )
	{
		int entryCount = m_entries.count;
		B2.B2_ASSERT( entryCount > 0 );
		ref StackEntry entry = ref m_entries[entryCount - 1];
		T[] array = (T[])entry.data;
		B2.B2_ASSERT( mem.IsEmpty || Unsafe.AreSame( ref MemoryMarshal.GetReference( mem ), ref MemoryMarshal.GetArrayDataReference( array ) ) );
		Free( array );
	}

	// Grow the stack based on usage
	public void Grow()
	{
		// Stack must not be in use
		B2.B2_ASSERT( m_allocation == 0 );

		if ( m_maxAllocation > m_capacity )
		{
			m_capacity = m_maxAllocation + m_maxAllocation / 2;
		}
	}

	public int GetCapacity() => m_capacity;

	public int GetAllocation() => m_allocation;

	public int GetMaxAllocation() => m_maxAllocation;
}

public static partial class B2
{
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint CTZ32( uint block ) => (uint)BitOperations.TrailingZeroCount( block );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint CLZ32( uint value ) => (uint)BitOperations.LeadingZeroCount( value );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static uint CTZ64( ulong block ) => (uint)BitOperations.TrailingZeroCount( block );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int PopCount64( ulong block ) => BitOperations.PopCount( block );

	internal static bool IsPowerOf2( int x ) => ( x & ( x - 1 ) ) == 0;

	internal static int BoundingPowerOf2( int x )
	{
		if ( x <= 1 )
		{
			return 1;
		}

		return 32 - (int)CLZ32( (uint)x - 1 );
	}

	internal static int RoundUpPowerOf2( int x )
	{
		if ( x <= 1 )
		{
			return 1;
		}

		return 1 << ( 32 - (int)CLZ32( (uint)x - 1 ) );
	}

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void Pause() => Thread.SpinWait( 1 );
}
