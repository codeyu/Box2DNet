// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	internal struct Shape
	{
		public int id;
		public int bodyId;
		public int prevShapeId;
		public int nextShapeId;
		public int sensorIndex;
		public ShapeType type;
		public SurfaceMaterial material;
		public float density;
		public float aabbMargin;
		public AABB aabb;
		public Vec2 localCentroid;
		public int proxyKey;

		public Filter filter;
		public object? userData;
		public ShapeUnion u;
		[UnscopedRef] public ref Capsule capsule => ref u.capsule;
		[UnscopedRef] public ref Circle circle => ref u.circle;
		[UnscopedRef] public ref Polygon polygon => ref u.polygon;
		[UnscopedRef] public ref Segment segment => ref u.segment;
		[UnscopedRef] public ref ChainSegment chainSegment => ref u.chainSegment;

		public ushort generation;
		public bool enableSensorEvents;
		public bool enableContactEvents;
		public bool enableCustomFiltering;
		public bool enableHitEvents;
		public bool enablePreSolveEvents;
	}

	[StructLayout( LayoutKind.Explicit )]
	internal struct ShapeUnion
	{
		[FieldOffset( 0 )] public Capsule capsule;
		[FieldOffset( 0 )] public Circle circle;
		[FieldOffset( 0 )] public Polygon polygon;
		[FieldOffset( 0 )] public Segment segment;
		[FieldOffset( 0 )] public ChainSegment chainSegment;
	}

	internal struct ChainShape
	{
		public int id;
		public int bodyId;
		public int nextChainId;
		public int segmentCount;
		public ushort generation;
		public /*TODO pointer*/ int[] shapeIndices;
	}

	internal struct ShapeExtent
	{
		public float minExtent;
		public float maxExtent;
	}

	// Sensors are shapes that live in the broad-phase but never have contacts.
	// At the end of the time step all sensors are queried for overlap with any other shapes.
	// Sensors ignore body type and sleeping.
	// Sensors generate events when there is a new overlap or and overlap disappears.
	// The sensor overlaps don't get cleared until the next time step regardless of the overlapped
	// shapes being destroyed.
	// When a sensor is destroyed.
	internal struct SensorOverlaps
	{
		public B2Array<int> overlaps;
	}




	// Conservative world AABB for a shape, inflated by extra margin. In large world mode this is
	// computed in double and rounded outward so the inflation is not lost far from the origin.




	internal static float GetShapeRadius( in Shape shape )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return shape.capsule.radius;
			case ShapeType.circleShape:
				return shape.circle.radius;
			case ShapeType.polygonShape:
				return shape.polygon.radius;
			default:
				return 0.0f;
		}
	}

	internal static bool ShouldShapesCollide( Filter filterA, Filter filterB )
	{
		if ( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
		{
			return filterA.groupIndex > 0;
		}

		return ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
	}

	internal static bool ShouldQueryCollide( Filter shapeFilter, QueryFilter queryFilter )
	{
		return ( shapeFilter.categoryBits & queryFilter.maskBits ) != 0 && ( shapeFilter.maskBits & queryFilter.categoryBits ) != 0;
	}


}
