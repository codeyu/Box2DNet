// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
	// is touching many other bodies.
	// #define OverflowIndex ( GraphColorCount - 1 )

	// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
	// involving a dynamic and static bodies. This reduces tunneling due to push through.
	// #define DynamicColorCount ( GraphColorCount - 4 )

	internal struct GraphColor
	{
		// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
		// however I never traverse these bits or use the bit count for anything
		// This bitset is unused on the overflow color.
		//
		// Dirk suggested having a uint64_t per body that tracks the graph color membership
		// but I think this would make debugging harder and be less flexible. With the bitset
		// I can trivially increase the number of graph colors beyond 64. See usage of CountSetBits
		// for validation.
		public BitSet bodySet;

		// cache friendly arrays
		public B2Array<ContactSim> contactSims;
		public B2Array<JointSim> jointSims;

		// transient
		// In C this is a union of the wide constraint pointer and the overflow constraint pointer.
		// The wide constraints are an offset into StepContext.wideContactConstraints.
		public int wideConstraintStart;
		public ContactConstraint[] overflowConstraints;

		public int wideConstraintCount;

	}


	internal sealed class ConstraintGraph
	{
		// including overflow at the end
		public FixedArray24<GraphColor> colors;
	}




}
