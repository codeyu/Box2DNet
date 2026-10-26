// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using Xunit;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

// Entry points for the tests ported from box2d/test. Each upstream test function returns 0 on
// success, and a failed check inside throws with the failing expression.
public class UpstreamTests
{
	[Fact] public void CollisionTest() => RUN_TEST( CollisionTests.CollisionTest );
	[Fact] public void DistanceTest() => RUN_TEST( DistanceTests.DistanceTest );
	[Fact] public void MathTest() => RUN_TEST( MathTests.MathTest );
	[Fact] public void ShapeTest() => RUN_TEST( ShapeTests.ShapeTest );
	[Fact] public void BitSetTest() => RUN_TEST( BitsetTests.BitSetTest );
	[Fact] public void MoverTest() => RUN_TEST( MoverTests.MoverTest );
	[Fact] public void WorldTest() => RUN_TEST( WorldTests.WorldTest );
	[Fact] public void DeterminismTest() => RUN_TEST( DeterminismTests.DeterminismTest );
}
