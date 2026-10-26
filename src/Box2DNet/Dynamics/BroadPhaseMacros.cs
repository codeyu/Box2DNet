// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Runtime.CompilerServices;

namespace Box2DNet;

// Macros from broad_phase.h. Store the proxy type in the lower 2 bits of the proxy key.
public static partial class B2
{
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static BodyType B2_PROXY_TYPE( int key ) => (BodyType)( key & 3 );

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int B2_PROXY_ID( int key ) => key >> 2;

	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	internal static int B2_PROXY_KEY( int id, BodyType type ) => ( id << 2 ) | (int)type;
}
