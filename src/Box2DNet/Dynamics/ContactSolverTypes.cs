// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	internal struct ContactConstraintPoint
	{
		public Vec2 anchorA;
		public Vec2 anchorB;
		public float baseSeparation;
		public float relativeVelocity;
		public float normalImpulse;
		public float tangentImpulse;
		public float totalNormalImpulse;
		public float restitutionImpulse;
		public float normalMass;
		public float tangentMass;
	}

	internal struct ContactConstraint
	{
		// base-1, 0 for null
		public int indexA;
		public int indexB;
		public FixedArray2<ContactConstraintPoint> points;
		public Vec2 normal;
		public float invMassA;
		public float invMassB;
		public float invIA;
		public float invIB;
		public float friction;
		public float restitution;
		public float tangentSpeed;
		public float rollingResistance;
		public float rollingMass;
		public float rollingImpulse;
		public Softness softness;
		public int pointCount;
	}

	// This function allows hiding SIMD intrinsics in the source file to improve compilation performance.

	// Overflow contacts don't fit into the constraint graph coloring

	// Contacts that live within the constraint graph coloring



}
