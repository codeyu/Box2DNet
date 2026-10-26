// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class BitsetTests
{
	private const int COUNT = 169;
	internal static int BitSetTest()
	{
		BitSet bitSet = CreateBitSet( COUNT );

		SetBitCountAndClear( ref bitSet, COUNT );
		bool[] values = new bool[COUNT];

		int i1 = 0, i2 = 1;
		SetBit( ref bitSet, i1 );
		values[i1] = true;

		while ( i2 < COUNT )
		{
			SetBit( ref bitSet, i2 );
			values[i2] = true;
			int next = i1 + i2;
			i1 = i2;
			i2 = next;
		}

		for ( int i = 0; i < COUNT; ++i )
		{
			bool value = GetBit( ref bitSet, i );
			ENSURE( value == values[i] );
		}

		DestroyBitSet( ref bitSet );

		return 0;
	}

}
