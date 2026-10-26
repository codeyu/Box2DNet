// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class IdTests
{
	internal static int IdTest()
	{
		uint a = 0x01234567;

		{
			WorldId id = LoadWorldId( a );
			uint b = StoreWorldId( id );
			ENSURE( b == a );
		}

		ulong x = 0x0123456789ABCDEFull;

		{
			BodyId id = LoadBodyId( x );
			ulong y = StoreBodyId( id );
			ENSURE( x == y );
		}

		{
			ShapeId id = LoadShapeId( x );
			ulong y = StoreShapeId( id );
			ENSURE( x == y );
		}

		{
			ChainId id = LoadChainId( x );
			ulong y = StoreChainId( id );
			ENSURE( x == y );
		}

		{
			JointId id = LoadJointId( x );
			ulong y = StoreJointId( id );
			ENSURE( x == y );
		}

		return 0;
	}

}
