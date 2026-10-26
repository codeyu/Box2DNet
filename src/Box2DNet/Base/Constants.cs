// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2DNet;

// Constants. Some can be configured. I try to include most internal constants here for convenience
// and clarity. This is why they are not all configurable.
public static partial class B2
{
	/// Used to detect bad values. Positions greater than about 16km will have precision problems in
	/// single precision, so 100km as a limit should be fine in all cases.
	public static float Huge => ( 100.0f * 1000.0f ) * lengthUnitsPerMeter;

	/// Maximum parallel workers. Used for some fixed size arrays.
	public const int MaxWorkers = 32;

	/// Maximum number of tasks queued per world step.
	public const int MaxTasks = 256;

	/// Maximum number of colors in the constraint graph. Constraints that cannot find a color are added
	/// to the overflow set, which is solved single-threaded.
	public const int GraphColorCount = 24;

	/// A small length used as a collision and constraint tolerance. Usually it is chosen to be
	/// numerically significant, but visually insignificant. In meters. Normally this is 0.5cm.
	/// @warning modifying this can have a significant impact on stability
	public static float LinearSlop => 0.005f * lengthUnitsPerMeter;

	/// Maximum number of simultaneous worlds that can be allocated
	public const int MaxWorlds = 128;

	/// Maximum length of the body name. Can be 0 if you don't need names.
	public const int NameLength = 10;

	/// The maximum rotation of a body per time step. This limit is very large and is used to prevent
	/// numerical problems. You shouldn't need to adjust this.
	/// @warning increasing this to 0.5f * B2_PI or greater will break continuous collision.
	public const float MaxRotation = 0.25f * Pi;

	/// Box2D uses limited speculative collision. This reduces jitter. Normally this is 2cm.
	/// @warning modifying this can have a significant impact on performance and stability
	public static float SpeculativeDistance => 4.0f * LinearSlop;

	/// The default contact recycling distance.
	public static float ContactRecycleDistance => 10.0f * LinearSlop;

	/// The default contact recycling world angle threshold. 0.98 ~= 11.5 degrees
	public const float ContactRecycleCosAngle = 0.98f;

	/// This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
	/// without triggering a tree adjustment. This is in meters. Normally this is 5cm.
	/// @warning modifying this can have a significant impact on performance
	public static float MaxAABBMargin => 0.05f * lengthUnitsPerMeter;

	/// For small objects the margin is limited to this fraction times the maximum extent
	public const float AABBMarginFraction = 0.125f;

	/// The time that a body must be still before it will go to sleep. In seconds.
	public const float TimeToSleep = 0.5f;

	/// The maximum number of restitution iterations. Needed to avoid a solver overflow.
	public const int MaxRestitutionIterations = 63;
}

// Constants from constraint_graph.h
public static partial class B2
{
	// This holds constraints that cannot fit the graph color limit
	internal const int OverflowIndex = GraphColorCount - 1;

	// This keeps constraints involving static or kinematic bodies from occupying the
	// colors used by dynamic constraints. See constraint_graph.c.
	internal const int DynamicColorCount = GraphColorCount - 4;
}
