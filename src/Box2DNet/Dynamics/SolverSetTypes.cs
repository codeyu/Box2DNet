// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// The solver set type by index
	// enum SolverSetType
		// Static set for static bodies and joints between static bodies
		internal const int b2_staticSet = 0;

		// Disabled set for disabled bodies and their joints
		internal const int b2_disabledSet = 1;

		// Awake set for awake bodies and awake non-touching contacts. Awake touching contacts
		// and awake joints live in the constraint graph
		internal const int b2_awakeSet = 2;

		// The index of the first sleeping set. Each island that goes to sleep is put into
		// a sleeping set. This holds all bodies, contacts, and joints from the sleeping island.
		// A separate set for each sleeping island makes it very efficient to wake a single island.
		internal const int b2_firstSleepingSet = 3;

	// This holds solver set data. The following sets are used:
	// - static set for all static bodies and joints between static bodies
	// - active set for all active bodies with body states (no contacts or joints)
	// - disabled set for disabled bodies and their joints
	// - all further sets are sleeping island sets along with their contacts and joints
	// The purpose of solver sets is to achieve high memory locality.
	// https://www.youtube.com/watch?v=nZNd5FjSquk
	internal struct SolverSet
	{
		// Body array. Empty for unused set.
		public B2Array<BodySim> bodySims;

		// Body state only exists for active set
		public B2Array<BodyState> bodyStates;

		// This holds sleeping/disabled joints. Empty for static/active set.
		public B2Array<JointSim> jointSims;

		// This holds all contacts for sleeping sets.
		// This holds non-touching contacts for the awake set.
		public B2Array<ContactSim> contactSims;

		// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
		// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
		// islands will be naturally merged with the set is woken.
		// The static and disabled sets have no islands.
		// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
		public B2Array<IslandSim> islandSims;

		// Aligns with World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
		public int setIndex;
	}



	// Merge set 2 into set 1 then destroy set 2.
	// Warning: any pointers into these sets will be orphaned.




}
