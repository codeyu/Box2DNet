// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Runtime.InteropServices;

namespace Box2DNet;

// These ids serve as handles to internal Box2D objects. These should be considered opaque data and
// passed by value. All ids are considered null if initialized to zero (default).

/// World id references a world instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct WorldId : IEquatable<WorldId>
{
	public ushort index1;
	public ushort generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( WorldId other ) => index1 == other.index1 && generation == other.generation;
	public readonly override bool Equals( object? obj ) => obj is WorldId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, generation );
	public static bool operator ==( WorldId a, WorldId b ) => a.Equals( b );
	public static bool operator !=( WorldId a, WorldId b ) => !a.Equals( b );
	public readonly override string ToString() => $"World({index1}, g{generation})";
}

/// Body id references a body instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct BodyId : IEquatable<BodyId>
{
	public int index1;
	public ushort world0;
	public ushort generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( BodyId other ) =>
		index1 == other.index1 && world0 == other.world0 && generation == other.generation;

	public readonly override bool Equals( object? obj ) => obj is BodyId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, world0, generation );
	public static bool operator ==( BodyId a, BodyId b ) => a.Equals( b );
	public static bool operator !=( BodyId a, BodyId b ) => !a.Equals( b );
	public readonly override string ToString() => $"Body({index1}, w{world0}, g{generation})";
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct ShapeId : IEquatable<ShapeId>
{
	public int index1;
	public ushort world0;
	public ushort generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( ShapeId other ) =>
		index1 == other.index1 && world0 == other.world0 && generation == other.generation;

	public readonly override bool Equals( object? obj ) => obj is ShapeId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, world0, generation );
	public static bool operator ==( ShapeId a, ShapeId b ) => a.Equals( b );
	public static bool operator !=( ShapeId a, ShapeId b ) => !a.Equals( b );
	public readonly override string ToString() => $"Shape({index1}, w{world0}, g{generation})";
}

/// Chain id references a chain instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct ChainId : IEquatable<ChainId>
{
	public int index1;
	public ushort world0;
	public ushort generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( ChainId other ) =>
		index1 == other.index1 && world0 == other.world0 && generation == other.generation;

	public readonly override bool Equals( object? obj ) => obj is ChainId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, world0, generation );
	public static bool operator ==( ChainId a, ChainId b ) => a.Equals( b );
	public static bool operator !=( ChainId a, ChainId b ) => !a.Equals( b );
	public readonly override string ToString() => $"Chain({index1}, w{world0}, g{generation})";
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct JointId : IEquatable<JointId>
{
	public int index1;
	public ushort world0;
	public ushort generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( JointId other ) =>
		index1 == other.index1 && world0 == other.world0 && generation == other.generation;

	public readonly override bool Equals( object? obj ) => obj is JointId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, world0, generation );
	public static bool operator ==( JointId a, JointId b ) => a.Equals( b );
	public static bool operator !=( JointId a, JointId b ) => !a.Equals( b );
	public readonly override string ToString() => $"Joint({index1}, w{world0}, g{generation})";
}

/// Contact id references a contact instance. This should be treated as an opaque handle.
[StructLayout( LayoutKind.Sequential )]
public struct ContactId : IEquatable<ContactId>
{
	public int index1;
	public ushort world0;
	public short padding;
	public uint generation;

	public readonly bool IsNull => index1 == 0;
	public readonly bool IsNonNull => index1 != 0;

	public readonly bool Equals( ContactId other ) =>
		index1 == other.index1 && world0 == other.world0 && generation == other.generation;

	public readonly override bool Equals( object? obj ) => obj is ContactId other && Equals( other );
	public readonly override int GetHashCode() => HashCode.Combine( index1, world0, generation );
	public static bool operator ==( ContactId a, ContactId b ) => a.Equals( b );
	public static bool operator !=( ContactId a, ContactId b ) => !a.Equals( b );
	public readonly override string ToString() => $"Contact({index1}, w{world0}, g{generation})";
}

public static partial class B2
{
	/// Use these to make your identifiers null. You may also use zero initialization to get null.
	public static readonly WorldId nullWorldId = default;
	public static readonly BodyId nullBodyId = default;
	public static readonly ShapeId nullShapeId = default;
	public static readonly ChainId nullChainId = default;
	public static readonly JointId nullJointId = default;
	public static readonly ContactId nullContactId = default;

	/// Store a world id into a uint32_t.
	public static uint StoreWorldId( WorldId id ) => ( (uint)id.index1 << 16 ) | id.generation;

	/// Load a uint32_t into a world id.
	public static WorldId LoadWorldId( uint x ) => new() { index1 = (ushort)( x >> 16 ), generation = (ushort)x };

	/// Store a body id into a uint64_t.
	public static ulong StoreBodyId( BodyId id ) => ( (ulong)(uint)id.index1 << 32 ) | ( (ulong)id.world0 ) << 16 | id.generation;

	/// Load a uint64_t into a body id.
	public static BodyId LoadBodyId( ulong x ) =>
		new() { index1 = (int)( x >> 32 ), world0 = (ushort)( x >> 16 ), generation = (ushort)x };

	/// Store a shape id into a uint64_t.
	public static ulong StoreShapeId( ShapeId id ) => ( (ulong)(uint)id.index1 << 32 ) | ( (ulong)id.world0 ) << 16 | id.generation;

	/// Load a uint64_t into a shape id.
	public static ShapeId LoadShapeId( ulong x ) =>
		new() { index1 = (int)( x >> 32 ), world0 = (ushort)( x >> 16 ), generation = (ushort)x };

	/// Store a chain id into a uint64_t.
	public static ulong StoreChainId( ChainId id ) => ( (ulong)(uint)id.index1 << 32 ) | ( (ulong)id.world0 ) << 16 | id.generation;

	/// Load a uint64_t into a chain id.
	public static ChainId LoadChainId( ulong x ) =>
		new() { index1 = (int)( x >> 32 ), world0 = (ushort)( x >> 16 ), generation = (ushort)x };

	/// Store a joint id into a uint64_t.
	public static ulong StoreJointId( JointId id ) => ( (ulong)(uint)id.index1 << 32 ) | ( (ulong)id.world0 ) << 16 | id.generation;

	/// Load a uint64_t into a joint id.
	public static JointId LoadJointId( ulong x ) =>
		new() { index1 = (int)( x >> 32 ), world0 = (ushort)( x >> 16 ), generation = (ushort)x };

	/// Store a contact id.
	public static void StoreContactId( ContactId id, Span<uint> values )
	{
		values[0] = (uint)id.index1;
		values[1] = id.world0;
		values[2] = id.generation;
	}

	/// Load a contact id.
	public static ContactId LoadContactId( ReadOnlySpan<uint> values )
	{
		ContactId id;
		id.index1 = (int)values[0];
		id.world0 = (ushort)values[1];
		id.padding = 0;
		id.generation = values[2];
		return id;
	}
}
