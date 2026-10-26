// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Box2DNet.Testbed;

// The few std::vector operations the samples use that List does not have.
public static class CppCompat
{
	public static void resize<T>( this List<T> list, int count )
	{
		if ( count < list.Count )
		{
			list.RemoveRange( count, list.Count - count );
		}
		else
		{
			CollectionsMarshal.SetCount( list, count );
		}
	}

	public static Span<T> AsSpan<T>( this List<T> list ) => CollectionsMarshal.AsSpan( list );

	public static int Compare( BodyId a, BodyId b ) => B2.StoreBodyId( a ).CompareTo( B2.StoreBodyId( b ) );
}
