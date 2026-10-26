// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class ChainTests
{
	// The point array is the collidable surface and the ghosts sit outside it, so segment i spans
	// points i and i + 1 and there is one index space for everything: segmentCount is pointCount
	// for a loop and pointCount - 1 otherwise, and material slot i is segment i.

	private static readonly Vec2[] loopTriangle = { new( -2.0f, 0.0f ),
		new( 2.0f, 0.0f ),
		new( 0.0f, 3.0f ), };

	private static readonly Vec2[] loopSquare = { new( -2.0f, 0.0f ),
		new( 2.0f, 0.0f ),
		new( 2.0f, 4.0f ),
		new( -2.0f, 4.0f ), };

	private static readonly Vec2[] openLine = { new( 0.0f, 0.0f ), new( 2.0f, 1.0f ), new( 4.0f, 1.0f ), new( 6.0f, 0.0f ), new( 8.0f, 0.0f ), new( 10.0f, 1.0f ), };

	private static readonly Vec2[] openPair = { new( 0.0f, 0.0f ),
		new( 4.0f, 0.0f ), };

	static readonly Vec2 ghostBegin = new Vec2( -2.0f, -1.0f );
	static readonly Vec2 ghostEnd = new Vec2( 12.0f, 2.0f );

	static int assertCount;

	private static int CountingAssertFcn( string condition, string fileName, int lineNumber )
	{
		MAYBE_UNUSED( condition );
		MAYBE_UNUSED( fileName );
		MAYBE_UNUSED( lineNumber );
		assertCount += 1;

		// Keep going so the subtest can report rather than break to the debugger
		return 0;
	}

	private static void BeginCountingAsserts()
	{
		assertCount = 0;
		SetAssertCallback( CountingAssertFcn );
	}

	// Always zero in a build without asserts, so a check on this is a debug signal only
	private static int EndCountingAsserts()
	{
		SetAssertCallback( TestAssertFcn );
		return assertCount;
	}

	// Tag each material so a segment can be traced back to the slot it came from
	private static ChainId CreateTaggedChain( BodyId bodyId, in Vec2 points, int pointCount, int materialCount, bool isLoop )
	{
		FixedArray8<SurfaceMaterial> materials = default;
		for ( int i = 0; i < materialCount; ++i )
		{
			materials[i] = DefaultSurfaceMaterial();
			materials[i].userMaterialId = (ulong)( i + 1 );
		}

		ChainDef chainDef = DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = pointCount;
		chainDef.materials = materials;
		chainDef.materialCount = materialCount;
		chainDef.isLoop = isLoop;
		chainDef.ghost1 = ghostBegin;
		chainDef.ghost2 = ghostEnd;

		return CreateChain( bodyId, ref chainDef );
	}

	private static void ReadSegmentTags( ChainId chainId, Span<ulong> tags, int count )
	{
		FixedArray8<ShapeId> segments = default;
		Chain_GetSegments( chainId, segments, count );
		for ( int i = 0; i < count; ++i )
		{
			tags[i] = Shape_GetSurfaceMaterial( segments[i] ).userMaterialId;
		}
	}

	// Points are cloned into body local space with no transform applied, so an exact compare is
	// right here and a tolerance would only hide a wiring mistake
	private static bool SamePoint( Vec2 a, Vec2 b )
	{
		return a.x == b.x && a.y == b.y;
	}

	// A loop closes, so every point leads a segment. An open chain leaves the last point trailing.
	private static int SegmentCountTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		int triangle = Chain_GetSegmentCount( CreateTaggedChain( bodyId, loopTriangle, 3, 1, true ) );
		int square = Chain_GetSegmentCount( CreateTaggedChain( bodyId, loopSquare, 4, 1, true ) );
		int pair = Chain_GetSegmentCount( CreateTaggedChain( bodyId, openPair, 2, 1, false ) );
		int line = Chain_GetSegmentCount( CreateTaggedChain( bodyId, openLine, 6, 1, false ) );

		DestroyWorld( worldId );

		ENSURE( triangle == 3 );
		ENSURE( square == 4 );
		ENSURE( pair == 1 );
		ENSURE( line == 5 );

		return 0;
	}

	// Segment i spans points i and i + 1 and takes its ghosts from the neighbors. A loop wraps for
	// both ends. An open chain takes the def ghosts at the two ends and neighbors everywhere else.
	private static int SegmentGeometryTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId bodyId = CreateBody( worldId, ref bodyDef );

		ChainId loopId = CreateTaggedChain( bodyId, loopSquare, 4, 1, true );
		ChainId openId = CreateTaggedChain( bodyId, openLine, 6, 1, false );

		FixedArray4<ShapeId> loopSegments = default;
		Span<ShapeId> openSegments = stackalloc ShapeId[5];
		int loopCount = Chain_GetSegments( loopId, loopSegments, 4 );
		int openCount = Chain_GetSegments( openId, openSegments, 5 );

		FixedArray4<ChainSegment> loop = default;
		Span<ChainSegment> open = stackalloc ChainSegment[5];
		for ( int i = 0; i < 4; ++i )
		{
			loop[i] = Shape_GetChainSegment( loopSegments[i] );
		}
		for ( int i = 0; i < 5; ++i )
		{
			open[i] = Shape_GetChainSegment( openSegments[i] );
		}

		DestroyWorld( worldId );

		ENSURE( loopCount == 4 );
		ENSURE( openCount == 5 );

		// ENSURE stringizes into a format string, so keep a percent out of the condition
		for ( int i = 0; i < 4; ++i )
		{
			int next = ( i + 1 ) % 4;
			int after = ( i + 2 ) % 4;
			int prev = ( i + 3 ) % 4;
			ENSURE( SamePoint( loop[i].segment.point1, loopSquare[i] ) );
			ENSURE( SamePoint( loop[i].segment.point2, loopSquare[next] ) );
			ENSURE( SamePoint( loop[i].ghost1, loopSquare[prev] ) );
			ENSURE( SamePoint( loop[i].ghost2, loopSquare[after] ) );
		}

		for ( int i = 0; i < 5; ++i )
		{
			ENSURE( SamePoint( open[i].segment.point1, openLine[i] ) );
			ENSURE( SamePoint( open[i].segment.point2, openLine[i + 1] ) );
		}

		// The def ghosts extend the chain past its ends without consuming a point or a segment
		ENSURE( SamePoint( open[0].ghost1, ghostBegin ) );
		ENSURE( SamePoint( open[4].ghost2, ghostEnd ) );

		for ( int i = 1; i < 5; ++i )
		{
			ENSURE( SamePoint( open[i].ghost1, openLine[i - 1] ) );
		}
		for ( int i = 0; i < 4; ++i )
		{
			ENSURE( SamePoint( open[i].ghost2, openLine[i + 2] ) );
		}

		return 0;
	}

	private static int LoopMaterialTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId bodyId = CreateBody( worldId, ref bodyDef );
		ChainId chainId = CreateTaggedChain( bodyId, loopSquare, 4, 4, true );

		FixedArray4<ulong> created = default;
		ReadSegmentTags( chainId, created, 4 );

		SurfaceMaterial material = DefaultSurfaceMaterial();
		material.userMaterialId = 99;

		BeginCountingAsserts();
		Chain_SetSurfaceMaterial( chainId, ref material, 1 );
		int assertsTripped = EndCountingAsserts();

		FixedArray4<ulong> updated = default;
		FixedArray4<ulong> reported = default;
		ReadSegmentTags( chainId, updated, 4 );
		for ( int i = 0; i < 4; ++i )
		{
			reported[i] = Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
		}

		DestroyWorld( worldId );

		ENSURE( assertsTripped == 0 );

		for ( int i = 0; i < 4; ++i )
		{
			ulong expected = i == 1 ? 99 : (ulong)( i + 1 );
			ENSURE( created[i] == (ulong)( i + 1 ) );
			ENSURE( updated[i] == expected );

			// The getter and the segment shape cannot disagree, there is one copy
			ENSURE( reported[i] == expected );
		}

		return 0;
	}

	// No ghost offset anywhere. Slot 0 and the last slot are ordinary segments, which is the part
	// that used to be inert on an open chain.
	private static int OpenMaterialTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId bodyId = CreateBody( worldId, ref bodyDef );
		ChainId chainId = CreateTaggedChain( bodyId, openLine, 6, 5, false );

		Span<ulong> created = stackalloc ulong[5];
		ReadSegmentTags( chainId, created, 5 );

		SurfaceMaterial material = DefaultSurfaceMaterial();
		material.userMaterialId = 99;

		BeginCountingAsserts();
		Chain_SetSurfaceMaterial( chainId, ref material, 0 );
		material.userMaterialId = 98;
		Chain_SetSurfaceMaterial( chainId, ref material, 4 );
		int assertsTripped = EndCountingAsserts();

		Span<ulong> updated = stackalloc ulong[5];
		Span<ulong> reported = stackalloc ulong[5];
		ReadSegmentTags( chainId, updated, 5 );
		for ( int i = 0; i < 5; ++i )
		{
			reported[i] = Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
		}

		DestroyWorld( worldId );

		ENSURE( assertsTripped == 0 );

		for ( int i = 0; i < 5; ++i )
		{
			ulong expected = (ulong)( i + 1 );
			if ( i == 0 )
			{
				expected = 99;
			}
			else if ( i == 4 )
			{
				expected = 98;
			}

			ENSURE( created[i] == (ulong)( i + 1 ) );
			ENSURE( updated[i] == expected );
			ENSURE( reported[i] == expected );
		}

		return 0;
	}

	// One material at creation covers the chain and does not lock it that way. A broadcast and a
	// per segment set both work afterwards.
	private static int SingleMaterialTest()
	{
		WorldDef worldDef = DefaultWorldDef();
		WorldId worldId = CreateWorld( ref worldDef );

		BodyDef bodyDef = DefaultBodyDef();
		BodyId bodyId = CreateBody( worldId, ref bodyDef );
		ChainId chainId = CreateTaggedChain( bodyId, openLine, 6, 1, false );

		Span<ulong> created = stackalloc ulong[5];
		ReadSegmentTags( chainId, created, 5 );

		SurfaceMaterial material = DefaultSurfaceMaterial();
		material.userMaterialId = 99;

		BeginCountingAsserts();
		Chain_SetAllSurfaceMaterials( chainId, ref material );
		int assertsTripped = EndCountingAsserts();

		Span<ulong> broadcast = stackalloc ulong[5];
		ReadSegmentTags( chainId, broadcast, 5 );

		// One material at creation must not narrow the segment index, which is what the old count
		// bound would have rejected here
		material.userMaterialId = 98;
		BeginCountingAsserts();
		Chain_SetSurfaceMaterial( chainId, ref material, 2 );
		assertsTripped += EndCountingAsserts();

		Span<ulong> single = stackalloc ulong[5];
		Span<ulong> reported = stackalloc ulong[5];
		ReadSegmentTags( chainId, single, 5 );
		for ( int i = 0; i < 5; ++i )
		{
			reported[i] = Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
		}

		DestroyWorld( worldId );

		ENSURE( assertsTripped == 0 );

		for ( int i = 0; i < 5; ++i )
		{
			ulong expected = i == 2 ? 98 : 99;
			ENSURE( created[i] == 1 );
			ENSURE( broadcast[i] == 99 );
			ENSURE( single[i] == expected );
			ENSURE( reported[i] == expected );
		}

		return 0;
	}

	// Each subtest probes a different part of the index space, so report all of them instead of
	// stopping at the first failure the way RUN_SUBTEST does
	private static int RunChainSubtest( string name, Func<int> subtest )
	{
		int result = subtest();
		Console.WriteLine( $"  subtest {( result == 1 ? "failed" : "passed" )}: {name}" );
		return result;
	}

	internal static int ChainTest()
	{
		int result = 0;

		result |= RunChainSubtest( "SegmentCountTest", SegmentCountTest );
		result |= RunChainSubtest( "SegmentGeometryTest", SegmentGeometryTest );
		result |= RunChainSubtest( "LoopMaterialTest", LoopMaterialTest );
		result |= RunChainSubtest( "OpenMaterialTest", OpenMaterialTest );
		result |= RunChainSubtest( "SingleMaterialTest", SingleMaterialTest );

		return result;
	}

}
