// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// needed for dll export



	private static ref Shape GetShape( World world, ShapeId shapeId )
	{
		int id = shapeId.index1 - 1;
		ref Shape shape = ref world.shapes[id];
		B2_ASSERT( shape.id == id && shape.generation == shapeId.generation );
		return ref shape;
	}

	private static ref ChainShape GetChainShape( World world, ChainId chainId )
	{
		int id = chainId.index1 - 1;
		ref ChainShape chain = ref world.chainShapes[id];
		B2_ASSERT( chain.id == id && chain.generation == chainId.generation );
		return ref chain;
	}

	private static float ComputeShapeMargin( ref Shape shape )
	{
		float margin = 0.0f;

		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
			{
				margin = 0.5f * Distance( shape.capsule.center2, shape.capsule.center1 ) + shape.capsule.radius;
			}
			break;

			case ShapeType.circleShape:
			{
				margin = shape.circle.radius;
			}
			break;

			case ShapeType.polygonShape:
			{
				ref readonly Polygon poly = ref shape.polygon;
				float maxExtentSqr = 0.0f;
				int count = poly.count;
				for ( int i = 0; i < count; ++i )
				{
					float distanceSqr = DistanceSquared( poly.vertices[i], poly.centroid );
					maxExtentSqr = MaxFloat( maxExtentSqr, distanceSqr );
				}

				margin = MathF.Sqrt( maxExtentSqr );
			}
			break;

			case ShapeType.segmentShape:
			{
				margin = 0.5f * Distance( shape.segment.point1, shape.segment.point2 );
			}
			break;

			case ShapeType.chainSegmentShape:
			{
				margin = 0.5f * Distance( shape.chainSegment.segment.point1, shape.chainSegment.segment.point2 );
			}
			break;

			default:
				B2_VALIDATE( false );
				return MaxAABBMargin;
		}

		return MinFloat( MaxAABBMargin, AABBMarginFraction * margin );
	}

	private static void UpdateShapeAABBs( ref Shape shape, ref AABB fatAABB, WorldTransform transform, BodyType proxyType )
	{
		// Compute a bounding box with a speculative margin
		float speculativeDistance = SpeculativeDistance;
		float aabbMargin = shape.aabbMargin;

		AABB aabb = ComputeFatShapeAABB( shape, transform, speculativeDistance );
		shape.aabb = aabb;

		// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
		float margin = proxyType == BodyType.staticBody ? speculativeDistance : aabbMargin;
		fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
		fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
		fatAABB.upperBound.x = aabb.upperBound.x + margin;
		fatAABB.upperBound.y = aabb.upperBound.y + margin;
	}

	private static ref Shape CreateShapeInternal( World world, ref Body body, WorldTransform transform, in ShapeDef def, in ShapeUnion geometry, ShapeType shapeType )
	{
		int shapeId = AllocId( ref world.shapeIdPool );

		if ( shapeId == world.shapes.count )
		{
			world.shapes.Push( default(Shape) );
			world.fatAABBs.Push( default(AABB) );
		}
		else
		{
			B2_ASSERT( world.shapes.data[shapeId].id == NullIndex );
		}

		B2_ASSERT( world.fatAABBs.count == world.shapes.count );

		ref Shape shape = ref world.shapes[shapeId];
		switch ( shapeType )
		{
			case ShapeType.capsuleShape:
				shape.capsule = geometry.capsule;
				break;

			case ShapeType.circleShape:
				shape.circle = geometry.circle;
				break;

			case ShapeType.polygonShape:
				shape.polygon = geometry.polygon;
				break;

			case ShapeType.segmentShape:
				shape.segment = geometry.segment;
				break;

			case ShapeType.chainSegmentShape:
				shape.chainSegment = geometry.chainSegment;
				break;

			default:
				B2_ASSERT( false );
				break;
		}

		shape.id = shapeId;
		shape.bodyId = body.id;
		shape.type = shapeType;
		shape.density = def.density;
		shape.material = def.material;
		shape.filter = def.filter;
		shape.userData = def.userData;
		shape.enableSensorEvents = def.enableSensorEvents;
		shape.enableContactEvents = def.enableContactEvents;
		shape.enableCustomFiltering = def.enableCustomFiltering;
		shape.enableHitEvents = def.enableHitEvents;
		shape.enablePreSolveEvents = def.enablePreSolveEvents;
		shape.proxyKey = NullIndex;
		shape.localCentroid = GetShapeCentroid( shape );
		shape.aabbMargin = ComputeShapeMargin( ref shape );
		shape.aabb = new AABB { lowerBound = Vec2_zero, upperBound = Vec2_zero };
		world.fatAABBs.data[shapeId] = new AABB { lowerBound = Vec2_zero, upperBound = Vec2_zero };
		shape.generation += 1;

		if ( body.setIndex != b2_disabledSet )
		{
			BodyType proxyType = body.type;
			CreateShapeProxy( world, ref shape, proxyType, transform, def.invokeContactCreation || def.isSensor );
		}

		// Add to shape doubly linked list
		if ( body.headShapeId != NullIndex )
		{
			ref Shape headShape = ref world.shapes[body.headShapeId];
			headShape.prevShapeId = shapeId;
		}

		shape.prevShapeId = NullIndex;
		shape.nextShapeId = body.headShapeId;
		body.headShapeId = shapeId;
		body.shapeCount += 1;

		if ( def.isSensor )
		{
			shape.sensorIndex = world.sensors.count;
			Sensor sensor = default;
			sensor.shapeId = shapeId;
			sensor.hits.CreateN( 4 );
			sensor.overlaps1.CreateN( 16 );
			sensor.overlaps2.CreateN( 16 );
			world.sensors.Push( sensor );
		}
		else
		{
			shape.sensorIndex = NullIndex;
		}

		ValidateSolverSets( world );

		return ref shape;
	}

	private static bool IsValidMaterial( in SurfaceMaterial material )
	{
		return IsValidFloat( material.friction ) && material.friction >= 0.0f && IsValidFloat( material.restitution ) &&
			   material.restitution >= 0.0f && IsValidFloat( material.rollingResistance ) &&
			   material.rollingResistance >= 0.0f && IsValidFloat( material.tangentSpeed );
	}

	private static bool IsValidCircle( in Circle circle )
	{
		return IsValidVec2( circle.center ) && IsValidFloat( circle.radius ) && circle.radius >= 0.0f;
	}

	private static bool IsValidCapsule( in Capsule capsule )
	{
		return IsValidVec2( capsule.center1 ) && IsValidVec2( capsule.center2 ) && IsValidFloat( capsule.radius ) &&
			   capsule.radius >= 0.0f;
	}

	private static bool IsValidSegment( in Segment segment )
	{
		return IsValidVec2( segment.point1 ) && IsValidVec2( segment.point2 );
	}

	private static bool IsValidChainSegment( in ChainSegment chainSegment )
	{
		return IsValidVec2( chainSegment.ghost1 ) && IsValidSegment( chainSegment.segment ) &&
			   IsValidVec2( chainSegment.ghost2 );
	}

	private static bool IsValidPolygon( in Polygon polygon )
	{
		if ( polygon.count < 1 || polygon.count > MaxPolygonVertices )
		{
			return false;
		}

		if ( IsValidFloat( polygon.radius ) == false || polygon.radius < 0.0f )
		{
			return false;
		}

		if ( IsValidVec2( polygon.centroid ) == false )
		{
			return false;
		}

		for ( int i = 0; i < polygon.count; ++i )
		{
			if ( IsValidVec2( polygon.vertices[i] ) == false || IsValidVec2( polygon.normals[i] ) == false )
			{
				return false;
			}
		}

		return true;
	}

	private static ShapeId CreateShape( BodyId bodyId, in ShapeDef def, in ShapeUnion geometry, ShapeType shapeType )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidFloat( def.density ) && def.density >= 0.0f ) == false )
			{
				return default(ShapeId);
			};
		if ( CheckInput( IsValidMaterial( def.material ) ) == false )
			{
				return default(ShapeId);
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return default(ShapeId);
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );

		ref Shape shape = ref CreateShapeInternal( world, ref body, transform, def, geometry, shapeType );
		if ( def.updateBodyMass == true )
		{
			UpdateBodyMassData( world, ref body );
		}
		else if ( ( body.flags & b2_dirtyMass ) == 0 )
		{
			body.flags |= b2_dirtyMass;
			SyncBodyFlags( world, ref body );
		}

		ValidateSolverSets( world );

		ShapeId id = new ShapeId { index1 = shape.id + 1, world0 = bodyId.world0, generation = shape.generation };
		return id;
	}

	public static ShapeId CreateCircleShape( BodyId bodyId, in ShapeDef def, in Circle circle )
	{
		if ( CheckInput( IsValidCircle( circle ) ) == false )
			{
				return nullShapeId;
			};

		ShapeId id = CreateShape( bodyId, def, new ShapeUnion { circle = circle }, ShapeType.circleShape );

		World world = GetWorld( bodyId.world0 );

		return id;
	}

	public static ShapeId CreateCapsuleShape( BodyId bodyId, in ShapeDef def, in Capsule capsule )
	{
		if ( CheckInput( IsValidCapsule( capsule ) ) == false )
			{
				return nullShapeId;
			};

		float lengthSqr = DistanceSquared( capsule.center1, capsule.center2 );
		if ( lengthSqr <= LinearSlop * LinearSlop )
		{
			return nullShapeId;
		}

		ShapeId id = CreateShape( bodyId, def, new ShapeUnion { capsule = capsule }, ShapeType.capsuleShape );

		World world = GetWorld( bodyId.world0 );

		return id;
	}

	public static ShapeId CreatePolygonShape( BodyId bodyId, in ShapeDef def, in Polygon polygon )
	{
		if ( CheckInput( IsValidPolygon( polygon ) ) == false )
			{
				return nullShapeId;
			};

		ShapeId id = CreateShape( bodyId, def, new ShapeUnion { polygon = polygon }, ShapeType.polygonShape );

		World world = GetWorld( bodyId.world0 );

		return id;
	}

	public static ShapeId CreateSegmentShape( BodyId bodyId, in ShapeDef def, in Segment segment )
	{
		if ( CheckInput( IsValidSegment( segment ) ) == false )
			{
				return nullShapeId;
			};

		float lengthSqr = DistanceSquared( segment.point1, segment.point2 );
		if ( lengthSqr <= LinearSlop * LinearSlop )
		{
			B2_ASSERT( false );
			return nullShapeId;
		}

		ShapeId id = CreateShape( bodyId, def, new ShapeUnion { segment = segment }, ShapeType.segmentShape );

		World world = GetWorld( bodyId.world0 );

		return id;
	}

	public static ShapeId CreateChainSegmentShape( BodyId bodyId, in ShapeDef def, in ChainSegment chainSegment )
	{
		if ( CheckInput( IsValidChainSegment( chainSegment ) ) == false )
			{
				return nullShapeId;
			};

		float lengthSqr = DistanceSquared( chainSegment.segment.point1, chainSegment.segment.point2 );
		if ( lengthSqr <= LinearSlop * LinearSlop )
		{
			B2_ASSERT( false );
			return nullShapeId;
		}

		// No parent chain shape
		ChainSegment local = chainSegment;
		local.chainId = NullIndex;

		ShapeId id = CreateShape( bodyId, def, new ShapeUnion { chainSegment = local }, ShapeType.chainSegmentShape );

		World world = GetWorld( bodyId.world0 );

		return id;
	}

	// Destroy a shape on a body. This doesn't need to be called when destroying a body.
	private static void DestroyShapeInternal( World world, ref Shape shape, ref Body body )
	{
		int shapeId = shape.id;

		// Remove the shape from the body's doubly linked list.
		if ( shape.prevShapeId != NullIndex )
		{
			ref Shape prevShape = ref world.shapes[shape.prevShapeId];
			prevShape.nextShapeId = shape.nextShapeId;
		}

		if ( shape.nextShapeId != NullIndex )
		{
			ref Shape nextShape = ref world.shapes[shape.nextShapeId];
			nextShape.prevShapeId = shape.prevShapeId;
		}

		if ( shapeId == body.headShapeId )
		{
			body.headShapeId = shape.nextShapeId;
		}

		body.shapeCount -= 1;

		// Remove from broad-phase.
		DestroyShapeProxy( ref shape, world.broadPhase );

		// Destroy any contacts associated with the shape.
		int contactKey = body.headContactKey;
		while ( contactKey != NullIndex )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			contactKey = contact.edges[edgeIndex].nextKey;

			if ( contact.shapeIdA == shapeId || contact.shapeIdB == shapeId )
			{
				DestroyContact( world, ref contact );
			}
		}

		if ( shape.sensorIndex != NullIndex )
		{
			ref Sensor sensor = ref world.sensors[shape.sensorIndex];
			for ( int i = 0; i < sensor.overlaps2.count; ++i )
			{
				ref Visitor @ref = ref sensor.overlaps2.data[i];
				SensorEndTouchEvent @event = new SensorEndTouchEvent { sensorShapeId = new() { index1 = shapeId + 1, world0 = world.worldId, generation = shape.generation }, visitorShapeId = new() { index1 = @ref.shapeId + 1, world0 = world.worldId, generation = @ref.generation } };

				world.sensorEndEvents[world.endEventArrayIndex].Push( @event );
			}

			// Destroy sensor
			sensor.hits.Destroy();
			sensor.overlaps1.Destroy();
			sensor.overlaps2.Destroy();

			int movedIndex = world.sensors.RemoveSwap( shape.sensorIndex );
			if ( movedIndex != NullIndex )
			{
				// Fixup moved sensor
				ref Sensor movedSensor = ref world.sensors[shape.sensorIndex];
				ref Shape otherSensorShape = ref world.shapes[movedSensor.shapeId];
				otherSensorShape.sensorIndex = shape.sensorIndex;
			}
		}

		// Return shape to free list.
		FreeId( ref world.shapeIdPool, shapeId );
		shape.id = NullIndex;

		ValidateSolverSets( world );
	}

	public static void DestroyShape( ShapeId shapeId, bool updateBodyMass )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		// Cannot destroy a chain segment that has a parent chain shape
		if ( shape.type == ShapeType.chainSegmentShape && shape.chainSegment.chainId != NullIndex )
		{
			B2_ASSERT( false );
			return;
		}


		ref Body body = ref world.bodies[shape.bodyId];
		DestroyShapeInternal( world, ref shape, ref body );

		if ( updateBodyMass == true )
		{
			UpdateBodyMassData( world, ref body );
		}
	}

	public static ChainId CreateChain( BodyId bodyId, in ChainDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( def.isLoop ? def.pointCount >= 3 : def.pointCount >= 2 ) == false )
			{
				return default(ChainId);
			};

		int segmentCount = def.isLoop ? def.pointCount : def.pointCount - 1;

		if ( CheckInput( def.materialCount == 1 || def.materialCount == segmentCount ) == false )
			{
				return default(ChainId);
			};

		for ( int i = 0; i < def.pointCount; ++i )
		{
			if ( CheckInput( IsValidVec2( def.points[i] ) ) == false )
			{
				return default(ChainId);
			};
		}

		for ( int i = 0; i < def.materialCount; ++i )
		{
			if ( CheckInput( IsValidMaterial( def.materials[i] ) ) == false )
			{
				return default(ChainId);
			};
		}

		if ( def.isLoop == false )
		{
			// These are set to INFINITY in the default def as a sentinel to ensure they are set.
			if ( CheckInput( IsValidVec2( def.ghost1 ) ) == false )
			{
				return default(ChainId);
			};
			if ( CheckInput( IsValidVec2( def.ghost2 ) ) == false )
			{
				return default(ChainId);
			};
		}

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return default(ChainId);
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );

		int chainId = AllocId( ref world.chainIdPool );

		if ( chainId == world.chainShapes.count )
		{
			world.chainShapes.Push( default(ChainShape) );
		}
		else
		{
			B2_ASSERT( world.chainShapes.data[chainId].id == NullIndex );
		}

		ref ChainShape chainShape = ref world.chainShapes[chainId];
		chainShape.id = chainId;
		chainShape.bodyId = body.id;
		chainShape.nextChainId = body.headChainId;
		chainShape.segmentCount = segmentCount;
		chainShape.generation += 1;

		int materialCount = def.materialCount;

		body.headChainId = chainId;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.userData = def.userData;
		shapeDef.filter = def.filter;
		shapeDef.enableSensorEvents = def.enableSensorEvents;
		shapeDef.enableContactEvents = false;
		shapeDef.enableHitEvents = false;

		ReadOnlySpan<Vec2> points = def.points;
		int n = segmentCount;
		chainShape.shapeIndices = Alloc<int>( n );

		float tolSqr = LinearSlop * LinearSlop;
		if ( def.isLoop )
		{
			ChainSegment chainSegment = default;
			int prevIndex = n - 1;
			for ( int i = 0; i < n; ++i )
			{
				chainSegment.ghost1 = points[prevIndex];
				chainSegment.segment.point1 = points[i];
				chainSegment.segment.point2 = points[( i + 1 ) % n];
				chainSegment.ghost2 = points[( i + 2 ) % n];

				B2_ASSERT( DistanceSquared( chainSegment.ghost1, chainSegment.segment.point1 ) > tolSqr );
				B2_ASSERT( DistanceSquared( chainSegment.segment.point1, chainSegment.segment.point2 ) > tolSqr );

				chainSegment.chainId = chainId;
				prevIndex = i;

				int materialIndex = materialCount == 1 ? 0 : i;
				shapeDef.material = def.materials[materialIndex];

				ref Shape shape = ref CreateShapeInternal( world, ref body, transform, shapeDef, new ShapeUnion { chainSegment = chainSegment }, ShapeType.chainSegmentShape );
				chainShape.shapeIndices[i] = shape.id;
			}
		}
		else
		{
			ChainSegment chainSegment = default;
			for ( int i = 0; i < n; ++i )
			{
				B2_ASSERT( i + 1 < def.pointCount );
				chainSegment.ghost1 = i == 0 ? def.ghost1 : points[i - 1];
				chainSegment.segment.point1 = points[i + 0];
				chainSegment.segment.point2 = points[i + 1];
				chainSegment.ghost2 = i == n - 1 ? def.ghost2 : points[i + 2];

				B2_ASSERT( DistanceSquared( chainSegment.ghost1, chainSegment.segment.point1 ) > tolSqr );
				B2_ASSERT( DistanceSquared( chainSegment.segment.point1, chainSegment.segment.point2 ) > tolSqr );
				B2_ASSERT( DistanceSquared( chainSegment.segment.point2, chainSegment.ghost2 ) > tolSqr );

				chainSegment.chainId = chainId;

				int materialIndex = materialCount == 1 ? 0 : i;
				shapeDef.material = def.materials[materialIndex];

				ref Shape shape = ref CreateShapeInternal( world, ref body, transform, shapeDef, new ShapeUnion { chainSegment = chainSegment }, ShapeType.chainSegmentShape );
				chainShape.shapeIndices[i] = shape.id;
			}
		}

		ChainId id = new ChainId { index1 = chainId + 1, world0 = world.worldId, generation = chainShape.generation };


		return id;
	}

	internal static void FreeChainData( ref ChainShape chain )
	{
		Free( chain.shapeIndices, chain.segmentCount );
		chain.shapeIndices = default;
	}

	public static void DestroyChain( ChainId chainId )
	{
		World world = GetWorldLocked( chainId.world0 );
		if ( world == null )
		{
			return;
		}


		ref ChainShape chain = ref GetChainShape( world, chainId );
		ref Body body = ref world.bodies[chain.bodyId];
		// Remove the chain from the body's singly linked list.
		ref int chainIdPtr = ref body.headChainId;
		bool found = false;
		while ( chainIdPtr != NullIndex )
		{
			if ( chainIdPtr == chain.id )
			{
				chainIdPtr = chain.nextChainId;
				found = true;
				break;
			}

			chainIdPtr = ref world.chainShapes.data[chainIdPtr].nextChainId;
		}

		B2_ASSERT( found == true );
		if ( found == false )
		{
			return;
		}

		int count = chain.segmentCount;
		for ( int i = 0; i < count; ++i )
		{
			int shapeId = chain.shapeIndices[i];
			ref Shape shape = ref world.shapes[shapeId];
			DestroyShapeInternal( world, ref shape, ref body );
		}

		FreeChainData( ref chain );

		// Return chain to free list.
		FreeId( ref world.chainIdPool, chain.id );
		chain.id = NullIndex;

		ValidateSolverSets( world );
	}

	public static WorldId Chain_GetWorld( ChainId chainId )
	{
		World world = GetWorld( chainId.world0 );
		return new WorldId { index1 = (ushort)( chainId.world0 + 1 ), generation = world.generation };
	}

	public static int Chain_GetSegmentCount( ChainId chainId )
	{
		World world = GetWorldLocked( chainId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref ChainShape chain = ref GetChainShape( world, chainId );
		return chain.segmentCount;
	}

	public static int Chain_GetSegments( ChainId chainId, Span<ShapeId> segmentArray, int capacity )
	{
		World world = GetWorldLocked( chainId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref ChainShape chain = ref GetChainShape( world, chainId );
		int count = MinInt( chain.segmentCount, capacity );
		for ( int i = 0; i < count; ++i )
		{
			int shapeId = chain.shapeIndices[i];
			ref Shape shape = ref world.shapes[shapeId];
			segmentArray[i] = new ShapeId { index1 = shapeId + 1, world0 = chainId.world0, generation = shape.generation };
		}

		return count;
	}

	public static void Chain_SetSurfaceMaterial( ChainId chainId, in SurfaceMaterial material, int segmentIndex )
	{
		if ( CheckInput( IsValidMaterial( material ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( chainId.world0 );
		if ( world == null )
		{
			return;
		}


		ref ChainShape chainShape = ref GetChainShape( world, chainId );
		B2_ASSERT( 0 <= segmentIndex && segmentIndex < chainShape.segmentCount );
		int shapeId = chainShape.shapeIndices[segmentIndex];
		ref Shape shape = ref world.shapes[shapeId];
		shape.material = material;
	}

	public static void Chain_SetAllSurfaceMaterials( ChainId chainId, in SurfaceMaterial material )
	{
		if ( CheckInput( IsValidMaterial( material ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( chainId.world0 );
		if ( world == null )
		{
			return;
		}


		ref ChainShape chainShape = ref GetChainShape( world, chainId );
		for ( int i = 0; i < chainShape.segmentCount; ++i )
		{
			int shapeId = chainShape.shapeIndices[i];
			ref Shape shape = ref world.shapes[shapeId];
			shape.material = material;
		}
	}

	public static SurfaceMaterial Chain_GetSurfaceMaterial( ChainId chainId, int segmentIndex )
	{
		World world = GetWorld( chainId.world0 );
		ref ChainShape chainShape = ref GetChainShape( world, chainId );
		B2_ASSERT( 0 <= segmentIndex && segmentIndex < chainShape.segmentCount );
		int shapeId = chainShape.shapeIndices[segmentIndex];
		ref Shape shape = ref world.shapes[shapeId];
		return shape.material;
	}

	internal static AABB ComputeShapeAABB( in Shape shape, WorldTransform xf )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return ComputeCapsuleBounds( ref shape.capsule, xf );
			case ShapeType.circleShape:
				return ComputeCircleBounds( ref shape.circle, xf );
			case ShapeType.polygonShape:
				return ComputePolygonBounds( ref shape.polygon, xf );
			case ShapeType.segmentShape:
				return ComputeSegmentBounds( ref shape.segment, xf );
			case ShapeType.chainSegmentShape:
				return ComputeSegmentBounds( ref shape.chainSegment.segment, xf );
			default:
			{
				B2_ASSERT( false );
				AABB empty = new AABB { lowerBound = new( 0.0f, 0.0f ), upperBound = new( 0.0f, 0.0f ) };
				return empty;
			}
		}
	}

	internal static Vec2 GetShapeCentroid( in Shape shape )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return Lerp( shape.capsule.center1, shape.capsule.center2, 0.5f );
			case ShapeType.circleShape:
				return shape.circle.center;
			case ShapeType.polygonShape:
				return shape.polygon.centroid;
			case ShapeType.segmentShape:
				return Lerp( shape.segment.point1, shape.segment.point2, 0.5f );
			case ShapeType.chainSegmentShape:
				return Lerp( shape.chainSegment.segment.point1, shape.chainSegment.segment.point2, 0.5f );
			default:
				return Vec2_zero;
		}
	}

	// todo_erin maybe compute this on shape creation
	internal static float GetShapePerimeter( in Shape shape )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return 2.0f * Length( Sub( shape.capsule.center1, shape.capsule.center2 ) ) +
					   2.0f * Pi * shape.capsule.radius;
			case ShapeType.circleShape:
				return 2.0f * Pi * shape.circle.radius;
			case ShapeType.polygonShape:
			{
				ReadOnlySpan<Vec2> points = shape.polygon.vertices;
				int count = shape.polygon.count;
				float perimeter = 2.0f * Pi * shape.polygon.radius;
				B2_ASSERT( count > 0 );
				Vec2 prev = points[count - 1];
				for ( int i = 0; i < count; ++i )
				{
					Vec2 next = points[i];
					perimeter += Length( Sub( next, prev ) );
					prev = next;
				}

				return perimeter;
			}
			case ShapeType.segmentShape:
				return 2.0f * Length( Sub( shape.segment.point1, shape.segment.point2 ) );
			case ShapeType.chainSegmentShape:
				return 2.0f * Length( Sub( shape.chainSegment.segment.point1, shape.chainSegment.segment.point2 ) );
			default:
				return 0.0f;
		}
	}

	// This projects the shape perimeter onto an infinite line
	internal static float GetShapeProjectedPerimeter( in Shape shape, Vec2 line )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
			{
				Vec2 axis = Sub( shape.capsule.center2, shape.capsule.center1 );
				float projectedLength = AbsFloat( Dot( axis, line ) );
				return projectedLength + 2.0f * shape.capsule.radius;
			}

			case ShapeType.circleShape:
				return 2.0f * shape.circle.radius;

			case ShapeType.polygonShape:
			{
				ReadOnlySpan<Vec2> points = shape.polygon.vertices;
				int count = shape.polygon.count;
				B2_ASSERT( count > 0 );
				float value = Dot( points[0], line );
				float lower = value;
				float upper = value;
				for ( int i = 1; i < count; ++i )
				{
					value = Dot( points[i], line );
					lower = MinFloat( lower, value );
					upper = MaxFloat( upper, value );
				}

				return ( upper - lower ) + 2.0f * shape.polygon.radius;
			}

			case ShapeType.segmentShape:
			{
				float value1 = Dot( shape.segment.point1, line );
				float value2 = Dot( shape.segment.point2, line );
				return AbsFloat( value2 - value1 );
			}

			case ShapeType.chainSegmentShape:
			{
				float value1 = Dot( shape.chainSegment.segment.point1, line );
				float value2 = Dot( shape.chainSegment.segment.point2, line );
				return AbsFloat( value2 - value1 );
			}

			default:
				return 0.0f;
		}
	}

	internal static MassData ComputeShapeMass( in Shape shape )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return ComputeCapsuleMass( ref shape.capsule, shape.density );
			case ShapeType.circleShape:
				return ComputeCircleMass( ref shape.circle, shape.density );
			case ShapeType.polygonShape:
				return ComputePolygonMass( ref shape.polygon, shape.density );
			default:
				return default(MassData);
		}
	}

	internal static ShapeExtent ComputeShapeExtent( in Shape shape, Vec2 localCenter )
	{
		ShapeExtent extent = default;

		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
			{
				float radius = shape.capsule.radius;
				extent.minExtent = radius;
				Vec2 c1 = Sub( shape.capsule.center1, localCenter );
				Vec2 c2 = Sub( shape.capsule.center2, localCenter );
				extent.maxExtent = MathF.Sqrt( MaxFloat( LengthSquared( c1 ), LengthSquared( c2 ) ) ) + radius;
			}
			break;

			case ShapeType.circleShape:
			{
				float radius = shape.circle.radius;
				extent.minExtent = radius;
				extent.maxExtent = Length( Sub( shape.circle.center, localCenter ) ) + radius;
			}
			break;

			case ShapeType.polygonShape:
			{
				ref readonly Polygon poly = ref shape.polygon;
				float minExtent = Huge;
				float maxExtentSqr = 0.0f;
				int count = poly.count;
				for ( int i = 0; i < count; ++i )
				{
					Vec2 v = poly.vertices[i];
					float planeOffset = Dot( poly.normals[i], Sub( v, poly.centroid ) );
					minExtent = MinFloat( minExtent, planeOffset );

					float distanceSqr = LengthSquared( Sub( v, localCenter ) );
					maxExtentSqr = MaxFloat( maxExtentSqr, distanceSqr );
				}

				extent.minExtent = minExtent + poly.radius;
				extent.maxExtent = MathF.Sqrt( maxExtentSqr ) + poly.radius;
			}
			break;

			case ShapeType.segmentShape:
			{
				extent.minExtent = 0.0f;
				Vec2 c1 = Sub( shape.segment.point1, localCenter );
				Vec2 c2 = Sub( shape.segment.point2, localCenter );
				extent.maxExtent = MathF.Sqrt( MaxFloat( LengthSquared( c1 ), LengthSquared( c2 ) ) );
			}
			break;

			case ShapeType.chainSegmentShape:
			{
				extent.minExtent = 0.0f;
				Vec2 c1 = Sub( shape.chainSegment.segment.point1, localCenter );
				Vec2 c2 = Sub( shape.chainSegment.segment.point2, localCenter );
				extent.maxExtent = MathF.Sqrt( MaxFloat( LengthSquared( c1 ), LengthSquared( c2 ) ) );
			}
			break;

			default:
				break;
		}

		return extent;
	}

	internal static CastOutput RayCastShape( in RayCastInput input, in Shape shape, Transform transform )
	{
		RayCastInput localInput = input;
		localInput.origin = InvTransformPoint( transform, input.origin );
		localInput.translation = InvRotateVector( transform.q, input.translation );

		CastOutput output = default;
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				output = CastRayCapsule( ref shape.capsule, ref localInput );
				break;
			case ShapeType.circleShape:
				output = CastRayCircle( ref shape.circle, ref localInput );
				break;
			case ShapeType.polygonShape:
				output = CastRayPolygon( ref shape.polygon, ref localInput );
				break;
			case ShapeType.segmentShape:
				output = CastRaySegment( ref shape.segment, ref localInput, false );
				break;
			case ShapeType.chainSegmentShape:
				output = CastRaySegment( ref shape.chainSegment.segment, ref localInput, true );
				break;
			default:
				return output;
		}

		// The output point stays in the frame of the input transform, a caller chosen frame that is
		// typically re-centered near the origin.
		output.point = TransformPoint( transform, output.point );
		output.normal = RotateVector( transform.q, output.normal );
		return output;
	}

	internal static CastOutput ShapeCastShape( in ShapeCastInput input, in Shape shape, Transform transform )
	{
		CastOutput output = default;

		if ( input.proxy.count == 0 )
		{
			return output;
		}

		ShapeCastInput localInput = input;

		for ( int i = 0; i < localInput.proxy.count; ++i )
		{
			localInput.proxy.points[i] = InvTransformPoint( transform, input.proxy.points[i] );
		}

		localInput.translation = InvRotateVector( transform.q, input.translation );

		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				output = CastShapeCapsule( ref shape.capsule, ref localInput );
				break;
			case ShapeType.circleShape:
				output = CastShapeCircle( ref shape.circle, ref localInput );
				break;
			case ShapeType.polygonShape:
				output = CastShapePolygon( ref shape.polygon, ref localInput );
				break;
			case ShapeType.segmentShape:
				output = CastShapeSegment( ref shape.segment, ref localInput );
				break;
			case ShapeType.chainSegmentShape:
			{
				// Check for back side collision
				Vec2 approximateCentroid = localInput.proxy.points[0];
				for ( int i = 1; i < localInput.proxy.count; ++i )
				{
					approximateCentroid = Add( approximateCentroid, localInput.proxy.points[i] );
				}

				approximateCentroid = MulSV( 1.0f / localInput.proxy.count, approximateCentroid );

				Vec2 edge = Sub( shape.chainSegment.segment.point2, shape.chainSegment.segment.point1 );
				Vec2 r = Sub( approximateCentroid, shape.chainSegment.segment.point1 );

				if ( Cross( r, edge ) < 0.0f )
				{
					// Shape cast starts behind
					return output;
				}

				output = CastShapeSegment( ref shape.chainSegment.segment, ref localInput );
			}
			break;
			default:
				return output;
		}

		// Same frame contract as RayCastShape, the point stays in the input transform frame
		output.point = TransformPoint( transform, output.point );
		output.normal = RotateVector( transform.q, output.normal );
		return output;
	}

	internal static PlaneResult CollideMover( in Capsule mover, in Shape shape, Transform transform )
	{
		Capsule localMover = default;
		localMover.center1 = InvTransformPoint( transform, mover.center1 );
		localMover.center2 = InvTransformPoint( transform, mover.center2 );
		localMover.radius = mover.radius;

		PlaneResult result = default;
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				result = CollideMoverAndCapsule( ref localMover, ref shape.capsule );
				break;
			case ShapeType.circleShape:
				result = CollideMoverAndCircle( ref localMover, ref shape.circle );
				break;
			case ShapeType.polygonShape:
				result = CollideMoverAndPolygon( ref localMover, ref shape.polygon );
				break;
			case ShapeType.segmentShape:
				result = CollideMoverAndSegment( ref localMover, ref shape.segment );
				break;
			case ShapeType.chainSegmentShape:
				result = CollideMoverAndSegment( ref localMover, ref shape.chainSegment.segment );
				break;
			default:
				return result;
		}

		if ( result.hit == false )
		{
			return result;
		}

		result.plane.normal = RotateVector( transform.q, result.plane.normal );
		return result;
	}

	internal static void CreateShapeProxy( World world, ref Shape shape, BodyType type, WorldTransform transform, bool forcePairCreation )
	{
		B2_ASSERT( shape.proxyKey == NullIndex );

		ref AABB fatAABB = ref world.fatAABBs.data[shape.id];
		UpdateShapeAABBs( ref shape, ref fatAABB, transform, type );

		// Create proxies in the broad-phase.
		shape.proxyKey = BroadPhase_CreateProxy( world.broadPhase, type, fatAABB, shape.filter.categoryBits, shape.id,
													forcePairCreation );
		B2_ASSERT( B2_PROXY_TYPE( shape.proxyKey ) < BodyType.bodyTypeCount );
	}

	internal static void DestroyShapeProxy( ref Shape shape, BroadPhase bp )
	{
		if ( shape.proxyKey != NullIndex )
		{
			BroadPhase_DestroyProxy( bp, shape.proxyKey );
			shape.proxyKey = NullIndex;
		}
	}

	internal static ShapeProxy MakeShapeDistanceProxy( in Shape shape )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				return MakeProxy( ref shape.capsule.center1, 2, shape.capsule.radius );
			case ShapeType.circleShape:
				return MakeProxy( ref shape.circle.center, 1, shape.circle.radius );
			case ShapeType.polygonShape:
				return MakeProxy( shape.polygon.vertices, shape.polygon.count, shape.polygon.radius );
			case ShapeType.segmentShape:
				return MakeProxy( ref shape.segment.point1, 2, 0.0f );
			case ShapeType.chainSegmentShape:
				return MakeProxy( ref shape.chainSegment.segment.point1, 2, 0.0f );
			default:
			{
				B2_ASSERT( false );
				ShapeProxy empty = default;
				return empty;
			}
		}
	}

	public static BodyId Shape_GetBody( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return MakeBodyId( world, shape.bodyId );
	}

	public static WorldId Shape_GetWorld( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		return new WorldId { index1 = (ushort)( shapeId.world0 + 1 ), generation = world.generation };
	}

	public static void Shape_SetUserData( ShapeId shapeId, object? userData )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		shape.userData = userData;
	}

	public static object? Shape_GetUserData( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.userData;
	}

	public static bool Shape_IsSensor( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.sensorIndex != NullIndex;
	}

	public static bool Shape_TestPoint( ShapeId shapeId, Pos point )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		WorldTransform transform = GetBodyTransform( world, shape.bodyId );
		Vec2 localPoint = InvTransformWorldPoint( transform, point );

		bool result;
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
				result = PointInCapsule( ref shape.capsule, localPoint );
				break;

			case ShapeType.circleShape:
				result = PointInCircle( ref shape.circle, localPoint );
				break;

			case ShapeType.polygonShape:
				result = PointInPolygon( ref shape.polygon, localPoint );
				break;

			default:
				result = false;
				break;
		}

		return result;
	}

	public static WorldCastOutput Shape_CastRay( ShapeId shapeId, Pos origin, Vec2 translation )
	{
		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidVec2( translation ) );

		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		// Re-center on the origin so the cast runs in float precision
		Transform transform = ToRelativeTransform( GetBodyTransform( world, shape.bodyId ), origin );

		// The ray starts at the origin, so its origin in the re-centered frame is zero
		RayCastInput input = new RayCastInput { origin = Vec2_zero, translation = translation, maxFraction = 1.0f };

		// Lift the re-centered float result back to a world position
		CastOutput local = RayCastShape( ref input, shape, transform );
		WorldCastOutput output = default;
		output.normal = local.normal;
		output.point = OffsetPos( origin, local.point );
		output.fraction = local.fraction;
		output.iterations = local.iterations;
		output.hit = local.hit;

		return output;
	}

	public static void Shape_SetDensity( ShapeId shapeId, float density, bool updateBodyMass )
	{
		if ( CheckInput( IsValidFloat( density ) && density >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		if ( density == shape.density )
		{
			// early return to avoid expensive function
			return;
		}

		shape.density = density;

		if ( updateBodyMass == true )
		{
			ref Body body = ref world.bodies[shape.bodyId];
			UpdateBodyMassData( world, ref body );
		}
	}

	public static float Shape_GetDensity( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.density;
	}

	public static void Shape_SetFriction( ShapeId shapeId, float friction )
	{
		if ( CheckInput( IsValidFloat( friction ) && friction >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorld( shapeId.world0 );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.material.friction = friction;
	}

	public static float Shape_GetFriction( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.material.friction;
	}

	public static void Shape_SetRestitution( ShapeId shapeId, float restitution )
	{
		if ( CheckInput( IsValidFloat( restitution ) && restitution >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorld( shapeId.world0 );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.material.restitution = restitution;
	}

	public static float Shape_GetRestitution( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.material.restitution;
	}

	public static void Shape_SetUserMaterial( ShapeId shapeId, ulong material )
	{
		World world = GetWorld( shapeId.world0 );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.material.userMaterialId = material;
	}

	public static ulong Shape_GetUserMaterial( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.material.userMaterialId;
	}

	public static SurfaceMaterial Shape_GetSurfaceMaterial( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.material;
	}

	public static void Shape_SetSurfaceMaterial( ShapeId shapeId, in SurfaceMaterial surfaceMaterial )
	{
		if ( CheckInput( IsValidMaterial( surfaceMaterial ) ) == false )
			{
				return;
			};

		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		shape.material = surfaceMaterial;
	}

	public static Filter Shape_GetFilter( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.filter;
	}

	private static void ResetProxy( World world, ref Shape shape, bool destroyProxy )
	{
		ref Body body = ref world.bodies[shape.bodyId];
		int shapeId = shape.id;

		// destroy all contacts associated with this shape
		int contactKey = body.headContactKey;
		while ( contactKey != NullIndex )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			contactKey = contact.edges[edgeIndex].nextKey;

			if ( contact.shapeIdA == shapeId || contact.shapeIdB == shapeId )
			{
				DestroyContact( world, ref contact );
			}
		}

		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		if ( shape.proxyKey != NullIndex )
		{
			BodyType proxyType = B2_PROXY_TYPE( shape.proxyKey );
			ref AABB fatAABB = ref world.fatAABBs.data[shapeId];
			UpdateShapeAABBs( ref shape, ref fatAABB, transform, proxyType );

			if ( destroyProxy )
			{
				BroadPhase_DestroyProxy( world.broadPhase, shape.proxyKey );

				bool forcePairCreation = true;
				shape.proxyKey = BroadPhase_CreateProxy( world.broadPhase, proxyType, fatAABB, shape.filter.categoryBits,
															shapeId, forcePairCreation );
			}
			else
			{
				BroadPhase_MoveProxy( world.broadPhase, shape.proxyKey, fatAABB );
			}
		}
		else
		{
			BodyType proxyType = body.type;
			UpdateShapeAABBs( ref shape, ref world.fatAABBs.data[shapeId], transform, proxyType );
		}

		ValidateSolverSets( world );
	}

	public static void Shape_SetFilter( ShapeId shapeId, Filter filter )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		if ( filter.maskBits == shape.filter.maskBits && filter.categoryBits == shape.filter.categoryBits &&
			 filter.groupIndex == shape.filter.groupIndex )
		{
			return;
		}

		// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
		bool destroyProxy = filter.categoryBits != shape.filter.categoryBits;

		shape.filter = filter;

		ResetProxy( world, ref shape, destroyProxy );

		// note: this does not immediately update sensor overlaps. Instead sensor
		// overlaps are updated the next time step
	}

	public static void Shape_EnableSensorEvents( ShapeId shapeId, bool flag )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.enableSensorEvents = flag;
	}

	public static bool Shape_AreSensorEventsEnabled( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.enableSensorEvents;
	}

	public static void Shape_EnableContactEvents( ShapeId shapeId, bool flag )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.enableContactEvents = flag;
	}

	public static bool Shape_AreContactEventsEnabled( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.enableContactEvents;
	}

	public static void Shape_EnablePreSolveEvents( ShapeId shapeId, bool flag )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.enablePreSolveEvents = flag;
	}

	public static bool Shape_ArePreSolveEventsEnabled( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.enablePreSolveEvents;
	}

	public static void Shape_EnableHitEvents( ShapeId shapeId, bool flag )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.enableHitEvents = flag;
	}

	public static bool Shape_AreHitEventsEnabled( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.enableHitEvents;
	}

	public static ShapeType Shape_GetType( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		return shape.type;
	}

	public static Circle Shape_GetCircle( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		B2_ASSERT( shape.type == ShapeType.circleShape );
		return shape.circle;
	}

	public static Segment Shape_GetSegment( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		B2_ASSERT( shape.type == ShapeType.segmentShape );
		return shape.segment;
	}

	public static ChainSegment Shape_GetChainSegment( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		B2_ASSERT( shape.type == ShapeType.chainSegmentShape );
		return shape.chainSegment;
	}

	public static Capsule Shape_GetCapsule( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		B2_ASSERT( shape.type == ShapeType.capsuleShape );
		return shape.capsule;
	}

	public static Polygon Shape_GetPolygon( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		B2_ASSERT( shape.type == ShapeType.polygonShape );
		return shape.polygon;
	}

	public static void Shape_SetCircle( ShapeId shapeId, in Circle circle )
	{
		if ( CheckInput( IsValidCircle( circle ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.circle = circle;
		shape.type = ShapeType.circleShape;
		shape.aabbMargin = ComputeShapeMargin( ref shape );

		bool destroyProxy = true;
		ResetProxy( world, ref shape, destroyProxy );
	}

	public static void Shape_SetCapsule( ShapeId shapeId, in Capsule capsule )
	{
		if ( CheckInput( IsValidCapsule( capsule ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}

		float lengthSqr = DistanceSquared( capsule.center1, capsule.center2 );
		if ( lengthSqr <= LinearSlop * LinearSlop )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.capsule = capsule;
		shape.type = ShapeType.capsuleShape;
		shape.aabbMargin = ComputeShapeMargin( ref shape );

		bool destroyProxy = true;
		ResetProxy( world, ref shape, destroyProxy );
	}

	public static void Shape_SetSegment( ShapeId shapeId, in Segment segment )
	{
		if ( CheckInput( IsValidSegment( segment ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.segment = segment;
		shape.type = ShapeType.segmentShape;
		shape.aabbMargin = ComputeShapeMargin( ref shape );

		bool destroyProxy = true;
		ResetProxy( world, ref shape, destroyProxy );
	}

	public static void Shape_SetPolygon( ShapeId shapeId, in Polygon polygon )
	{
		if ( CheckInput( IsValidPolygon( polygon ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		shape.polygon = polygon;
		shape.type = ShapeType.polygonShape;
		shape.aabbMargin = ComputeShapeMargin( ref shape );

		bool destroyProxy = true;
		ResetProxy( world, ref shape, destroyProxy );
	}

	public static void Shape_SetChainSegment( ShapeId shapeId, in ChainSegment chainSegment )
	{
		if ( CheckInput( IsValidChainSegment( chainSegment ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		// Cannot modify a chain segment that has a parent chain shape
		if ( shape.type == ShapeType.chainSegmentShape && shape.chainSegment.chainId != NullIndex )
		{
			B2_ASSERT( false );
			return;
		}

		float lengthSqr = DistanceSquared( chainSegment.segment.point1, chainSegment.segment.point2 );
		if ( lengthSqr <= LinearSlop * LinearSlop )
		{
			return;
		}


		shape.chainSegment = chainSegment;
		shape.chainSegment.chainId = NullIndex;
		shape.type = ShapeType.chainSegmentShape;
		shape.aabbMargin = ComputeShapeMargin( ref shape );

		bool destroyProxy = true;
		ResetProxy( world, ref shape, destroyProxy );
	}

	public static ChainId Shape_GetParentChain( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		ref Shape shape = ref GetShape( world, shapeId );
		if ( shape.type == ShapeType.chainSegmentShape )
		{
			int chainId = shape.chainSegment.chainId;
			if ( chainId != NullIndex )
			{
				ref ChainShape chain = ref world.chainShapes[chainId];
				ChainId id = new ChainId { index1 = chainId + 1, world0 = shapeId.world0, generation = chain.generation };
				return id;
			}
		}

		return default(ChainId);
	}

	public static int Shape_GetContactCapacity( ShapeId shapeId )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		if ( shape.sensorIndex != NullIndex )
		{
			return 0;
		}

		ref Body body = ref world.bodies[shape.bodyId];
		// Conservative and fast
		return body.contactCount;
	}

	public static int Shape_GetContactData( ShapeId shapeId, Span<ContactData> contactData, int capacity )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		if ( shape.sensorIndex != NullIndex )
		{
			return 0;
		}

		ref Body body = ref world.bodies[shape.bodyId];
		int contactKey = body.headContactKey;
		int index = 0;
		while ( contactKey != NullIndex && index < capacity )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			// Does contact involve this shape and is it touching?
			if ( ( contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1 ) &&
				 ( contact.flags & b2_contactTouchingFlag ) != 0 )
			{
				ref Shape shapeA = ref world.shapes.data[contact.shapeIdA];
				ref Shape shapeB = ref world.shapes.data[contact.shapeIdB];
				contactData[index].contactId = new ContactId { index1 = contact.contactId + 1, world0 = shapeId.world0, padding = 0, generation = contact.generation };
				contactData[index].shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = shapeId.world0, generation = shapeA.generation };
				contactData[index].shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = shapeId.world0, generation = shapeB.generation };

				ref ContactSim contactSim = ref GetContactSim( world, ref contact );
				contactData[index].manifold = contactSim.manifold;
				index += 1;
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}

		B2_ASSERT( index <= capacity );

		return index;
	}

	public static int Shape_GetSensorCapacity( ShapeId shapeId )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		if ( shape.sensorIndex == NullIndex )
		{
			return 0;
		}

		ref Sensor sensor = ref world.sensors[shape.sensorIndex];
		return sensor.overlaps2.count;
	}

	public static int Shape_GetSensorData( ShapeId shapeId, Span<ShapeId> visitorIds, int capacity )
	{
		World world = GetWorldLocked( shapeId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		if ( shape.sensorIndex == NullIndex )
		{
			return 0;
		}

		ref Sensor sensor = ref world.sensors[shape.sensorIndex];
		int count = MinInt( sensor.overlaps2.count, capacity );
		Span<Visitor> refs = sensor.overlaps2.data;
		for ( int i = 0; i < count; ++i )
		{
			ShapeId visitorId = new ShapeId { index1 = refs[i].shapeId + 1, world0 = shapeId.world0, generation = refs[i].generation };

			visitorIds[i] = visitorId;
		}

		return count;
	}

	public static AABB Shape_GetBounds( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		if ( world == null )
		{
			return default(AABB);
		}

		ref Shape shape = ref GetShape( world, shapeId );
		return shape.aabb;
	}

	public static MassData Shape_ComputeMassData( ShapeId shapeId )
	{
		World world = GetWorld( shapeId.world0 );
		if ( world == null )
		{
			return default(MassData);
		}

		ref Shape shape = ref GetShape( world, shapeId );
		return ComputeShapeMass( shape );
	}

	public static Pos Shape_GetClosestPoint( ShapeId shapeId, Pos target )
	{
		World world = GetWorld( shapeId.world0 );
		if ( world == null )
		{
			return Pos_zero;
		}

		ref Shape shape = ref GetShape( world, shapeId );
		ref Body body = ref world.bodies[shape.bodyId];
		WorldTransform transform = GetBodyTransformQuick( world, ref body );

		// The target rides in as the frame of proxy B, so the relative pose is differenced in double
		// and the result stays exact far from the origin
		Vec2 zero = Vec2_zero;
		WorldTransform targetTransform = new WorldTransform { p = target, q = Rot_identity };

		DistanceInput input = default;
		input.proxyA = MakeShapeDistanceProxy( shape );
		input.proxyB = MakeProxy( ref zero, 1, 0.0f );
		input.transform = InvMulWorldTransforms( transform, targetTransform );
		input.useRadii = true;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( ref input, ref cache );

		return TransformWorldPoint( transform, output.pointA );
	}

	// https://en.wikipedia.org/wiki/Density_of_air
	// https://www.engineeringtoolbox.com/wind-load-d_1775.html
	// force = 0.5 * air_density * velocity^2 * area
	// https://en.wikipedia.org/wiki/Lift_(force)
	public static void Shape_ApplyWind( ShapeId shapeId, Vec2 wind, float drag, float lift, bool wake )
	{
		if ( CheckInput( IsValidVec2( wind ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( drag ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( lift ) ) == false )
			{
				return;
			};

		World world = GetWorld( shapeId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Shape shape = ref GetShape( world, shapeId );
		ShapeType shapeType = shape.type;
		if ( shapeType != ShapeType.circleShape && shapeType != ShapeType.capsuleShape && shapeType != ShapeType.polygonShape )
		{
			return;
		}

		ref Body body = ref world.bodies[shape.bodyId];
		if ( body.type != BodyType.dynamicBody )
		{
			return;
		}

		if ( body.setIndex >= b2_firstSleepingSet && wake == false )
		{
			return;
		}

		ref BodySim sim = ref GetBodySim( world, ref body );
		if ( body.setIndex != b2_awakeSet )
		{
			// Must wake for state to exist
			WakeBody( world, ref body );
		}

		B2_ASSERT( body.setIndex == b2_awakeSet );

		ref BodyState state = ref GetBodyState( world, ref body );
		WorldTransform transform = sim.transform;

		float lengthUnits = GetLengthUnitsPerMeter();
		float volumeUnits = lengthUnits * lengthUnits * lengthUnits;

		// In 2D I'm assuming unit depth
		float airDensity = 1.2250f / ( volumeUnits );

		Vec2 force = default;
		float torque = 0.0f;

		switch ( shape.type )
		{
			case ShapeType.circleShape:
			{
				float radius = shape.circle.radius;
				Vec2 centroid = shape.localCentroid;
				Vec2 lever = RotateVector( transform.q, Sub( centroid, sim.localCenter ) );
				Vec2 shapeVelocity = Add( state.linearVelocity, CrossSV( state.angularVelocity, lever ) );
				Vec2 relativeVelocity = MulSub( wind, drag, shapeVelocity );
				float speed;
				Vec2 direction = GetLengthAndNormalize( out speed, relativeVelocity );
				float projectedArea = 2.0f * radius;
				force = MulSV( 0.5f * airDensity * projectedArea * speed * speed, direction );
				torque = Cross( lever, force );
			}
			break;

			case ShapeType.capsuleShape:
			{
				Vec2 centroid = shape.localCentroid;
				Vec2 lever = RotateVector( transform.q, Sub( centroid, sim.localCenter ) );
				Vec2 shapeVelocity = Add( state.linearVelocity, CrossSV( state.angularVelocity, lever ) );
				Vec2 relativeVelocity = MulSub( wind, drag, shapeVelocity );
				float speed;
				Vec2 direction = GetLengthAndNormalize( out speed, relativeVelocity );

				Vec2 d = Sub( shape.capsule.center2, shape.capsule.center1 );
				d = RotateVector( transform.q, d );

				float radius = shape.capsule.radius;
				float projectedArea = 2.0f * radius + AbsFloat( Cross( d, direction ) );

				// Normal that opposes the wind
				Vec2 normal = LeftPerp( Normalize( d ) );
				normal = Dot( normal, direction ) > 0.0f ? Neg( normal ) : normal;

				// portion of wind that is perpendicular to surface
				Vec2 liftDirection = CrossSV( Cross( normal, direction ), direction );

				float forceMagnitude = 0.5f * airDensity * projectedArea * speed * speed;
				force = MulSV( forceMagnitude, MulAdd( direction, lift, liftDirection ) );

				Vec2 edgeLever = MulAdd( lever, radius, normal );
				torque = Cross( edgeLever, force );
			}
			break;

			case ShapeType.polygonShape:
			{
				Vec2 centroid = shape.localCentroid;
				Vec2 lever = RotateVector( transform.q, Sub( centroid, sim.localCenter ) );
				Vec2 shapeVelocity = Add( state.linearVelocity, CrossSV( state.angularVelocity, lever ) );
				Vec2 relativeVelocity = MulSub( wind, drag, shapeVelocity );
				float speed;
				Vec2 direction = GetLengthAndNormalize( out speed, relativeVelocity );

				// polygon radius is ignored for simplicity
				int count = shape.polygon.count;
				Span<Vec2> vertices = shape.polygon.vertices;
				Vec2 v1 = vertices[count - 1];
				for ( int i = 0; i < count; ++i )
				{
					Vec2 v2 = vertices[i];
					Vec2 d = Sub( v2, v1 );
					Vec2 edgeCenter = Lerp( v1, v2, 0.5f );
					v1 = v2;

					d = RotateVector( transform.q, d );

					float projectedArea = Cross( d, direction );
					if ( projectedArea <= 0.0f )
					{
						// back facing
						continue;
					}

					Vec2 normal = RightPerp( Normalize( d ) );

					// portion of wind that is perpendicular to surface
					Vec2 liftDirection = CrossSV( Cross( normal, direction ), direction );

					float forceMagnitude = 0.5f * airDensity * projectedArea * speed * speed;
					Vec2 f = MulSV( forceMagnitude, MulAdd( direction, lift, liftDirection ) );

					Vec2 edgeLever = RotateVector( transform.q, Sub( edgeCenter, sim.localCenter ) );

					force = Add( force, f );
					torque += Cross( edgeLever, f );
				}
			}
			break;

			default:
				break;
		}

		sim.force = Add( sim.force, force );
		sim.torque += torque;
	}

}
