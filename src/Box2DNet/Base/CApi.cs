// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.CompilerServices;

namespace Box2DNet;

// Free function versions of the container APIs, matching the C names used by the ported code.
public static partial class B2
{
	// bitset.h
	internal static BitSet CreateBitSet( uint bitCapacity ) => BitSet.Create( bitCapacity );
	internal static void DestroyBitSet( ref BitSet bitSet ) => bitSet.Destroy();
	internal static void SetBitCountAndClear( ref BitSet bitSet, uint bitCount ) => bitSet.SetBitCountAndClear( bitCount );
	internal static void SetBitCountAndClear( ref BitSet bitSet, int bitCount ) => bitSet.SetBitCountAndClear( (uint)bitCount );
	internal static void InPlaceUnion( ref BitSet setA, in BitSet setB ) => setA.InPlaceUnion( setB );
	internal static void GrowBitSet( ref BitSet bitSet, uint blockCount ) => bitSet.Grow( blockCount );
	internal static int CountSetBits( ref BitSet bitSet ) => bitSet.CountSetBits();

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void SetBit( ref BitSet bitSet, uint bitIndex ) => bitSet.SetBit( bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void SetBit( ref BitSet bitSet, int bitIndex ) => bitSet.SetBit( (uint)bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void SetBitGrow( ref BitSet bitSet, uint bitIndex ) => bitSet.SetBitGrow( bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void SetBitGrow( ref BitSet bitSet, int bitIndex ) => bitSet.SetBitGrow( (uint)bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void ClearBit( ref BitSet bitSet, uint bitIndex ) => bitSet.ClearBit( bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static void ClearBit( ref BitSet bitSet, int bitIndex ) => bitSet.ClearBit( (uint)bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool GetBit( in BitSet bitSet, uint bitIndex ) => bitSet.GetBit( bitIndex );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static bool GetBit( in BitSet bitSet, int bitIndex ) => bitSet.GetBit( (uint)bitIndex );

	internal static int GetBitSetBytes( ref BitSet bitSet ) => bitSet.GetBytes();

	// id_pool.h
	internal static IdPool CreateIdPool() => IdPool.Create();
	internal static void DestroyIdPool( ref IdPool pool ) => pool.Destroy();
	internal static int AllocId( ref IdPool pool ) => pool.AllocId();
	internal static void FreeId( ref IdPool pool, int id ) => pool.FreeId( id );
	internal static void ValidateFreeId( ref IdPool pool, int id ) => pool.ValidateFreeId( id );
	internal static void ValidateUsedId( ref IdPool pool, int id ) => pool.ValidateUsedId( id );
	internal static int GetIdCount( ref IdPool pool ) => pool.GetIdCount();
	internal static int GetIdCapacity( ref IdPool pool ) => pool.GetIdCapacity();
	internal static int GetIdBytes( ref IdPool pool ) => pool.GetIdBytes();

	// table.h
	internal static HashSet64 CreateSet( int capacity ) => HashSet64.Create( capacity );
	internal static void DestroySet( ref HashSet64 set ) => set.Destroy();
	internal static void ClearSet( ref HashSet64 set ) => set.Clear();
	internal static bool AddKey( ref HashSet64 set, ulong key ) => set.AddKey( key );
	internal static bool RemoveKey( ref HashSet64 set, ulong key ) => set.RemoveKey( key );
	internal static bool ContainsKey( in HashSet64 set, ulong key ) => set.ContainsKey( key );
	internal static bool ContainsHashedKey( in HashSet64 set, ulong key, ulong hash ) => set.ContainsHashedKey( key, hash );
	internal static int GetHashSetBytes( ref HashSet64 set ) => set.GetBytes();
	internal static int GetSetCount( ref HashSet64 set ) => set.GetCount();
	internal static int GetSetCapacity( ref HashSet64 set ) => set.GetCapacity();

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static ulong KeyHash( ulong key ) => HashSet64.KeyHash( key );

	// arena_allocator.h
	internal static ArenaAllocator CreateStack( int capacity ) => new ArenaAllocator( capacity );
	internal static void DestroyStack( ArenaAllocator alloc ) => alloc.Destroy();
	internal static void GrowStack( ArenaAllocator alloc ) => alloc.Grow();
	internal static int GetStackCapacity( ArenaAllocator alloc ) => alloc.GetCapacity();
	internal static int GetStackAllocation( ArenaAllocator alloc ) => alloc.GetAllocation();
	internal static int GetMaxStackAllocation( ArenaAllocator alloc ) => alloc.GetMaxAllocation();
}
