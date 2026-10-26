// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	internal ref struct SensorQueryContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => SensorQueryCallback( proxyId, userData, ref this );

		public World world;
		public ref SensorTaskContext taskContext;
		public ref Sensor sensor;
		public ref Shape sensorShape;
		public WorldTransform transform;
	}

	// Sensor shapes need to
	// - detect begin and end overlap events
	// - events must be reported in deterministic order
	// - maintain an active list of overlaps for query

	// Assumption
	// - sensors don't detect shapes on the same body

	// Algorithm
	// Query all sensors for overlaps
	// Check against previous overlaps

	// Data structures
	// Each sensor has an double buffered array of overlaps
	// These overlaps use a shape reference with index and generation

	private static bool SensorQueryCallback( int proxyId, ulong userData, ref SensorQueryContext queryContext )
	{
		int shapeId = (int)userData;
		ref Shape sensorShape = ref queryContext.sensorShape;
		int sensorShapeId = sensorShape.id;

		if ( shapeId == sensorShapeId )
		{
			return true;
		}

		World world = queryContext.world;
		ref Shape otherShape = ref world.shapes[shapeId];
		// Are sensor events enabled on the other shape?
		if ( otherShape.enableSensorEvents == false )
		{
			return true;
		}

		// Skip shapes on the same body
		if ( otherShape.bodyId == sensorShape.bodyId )
		{
			return true;
		}

		// Check filter
		if ( ShouldShapesCollide( sensorShape.filter, otherShape.filter ) == false )
		{
			return true;
		}

		// Custom user filter
		if ( sensorShape.enableCustomFiltering || otherShape.enableCustomFiltering )
		{
			ref CustomFilterCallbackFcn customFilterFcn = ref queryContext.world.customFilterFcn;
			if ( Unsafe.IsNullRef( ref customFilterFcn ) == false )
			{
				ShapeId idA = new ShapeId { index1 = sensorShapeId + 1, world0 = world.worldId, generation = sensorShape.generation };
				ShapeId idB = new ShapeId { index1 = shapeId + 1, world0 = world.worldId, generation = otherShape.generation };
				bool shouldCollide = customFilterFcn( idA, idB, queryContext.world.customFilterContext );
				if ( shouldCollide == false )
				{
					return true;
				}
			}
		}

		// The relative pose is differenced in double so sensor overlap stays exact far from the origin
		WorldTransform otherTransform = GetBodyTransform( world, otherShape.bodyId );

		DistanceInput input = default;
		input.proxyA = MakeShapeDistanceProxy( sensorShape );
		input.proxyB = MakeShapeDistanceProxy( otherShape );
		input.transform = InvMulWorldTransforms( queryContext.transform, otherTransform );
		input.useRadii = true;
		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( ref input, ref cache );

		bool overlaps = output.distance < 10.0f * FLT_EPSILON;
		if ( overlaps == false )
		{
			return true;
		}

		// Record the overlap
		ref Sensor sensor = ref queryContext.sensor;
		ref Visitor shapeRef = ref sensor.overlaps2.Emplace();
		shapeRef.shapeId = shapeId;
		shapeRef.generation = otherShape.generation;

		return true;
	}

	private static int CompareVisitors( Visitor sa, Visitor sb )
	{
		if ( sa.shapeId < sb.shapeId )
		{
			return -1;
		}

		return 1;
	}

	private static void SensorTask( int startIndex, int endIndex, int threadIndex, object? context )
	{
		World world = (World)context!;
		ref SensorTaskContext taskContext = ref world.sensorTaskContexts.data[threadIndex];
		B2_ASSERT( startIndex < endIndex );

		ref FixedArray3<DynamicTree> trees = ref world.broadPhase.trees;
		for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
		{
			ref Sensor sensor = ref world.sensors[sensorIndex];
			ref Shape sensorShape = ref world.shapes[sensor.shapeId];
			// Swap overlap arrays
			B2Array<Visitor> temp = sensor.overlaps1;
			sensor.overlaps1 = sensor.overlaps2;
			sensor.overlaps2 = temp;
			sensor.overlaps2.Clear();

			// Append sensor hits
			int hitCount = sensor.hits.count;
			for ( int i = 0; i < hitCount; ++i )
			{
				sensor.overlaps2.Push( sensor.hits.data[i] );
			}

			// Clear the hits
			sensor.hits.Clear();

			ref Body body = ref world.bodies[sensorShape.bodyId];
			if ( body.setIndex == b2_disabledSet || sensorShape.enableSensorEvents == false )
			{
				if ( sensor.overlaps1.count != 0 )
				{
					// This sensor is dropping all overlaps because it has been disabled.
					SetBit( ref taskContext.eventBits, sensorIndex );
				}
				continue;
			}

			WorldTransform transform = GetBodyTransformQuick( world, ref body );

			SensorQueryContext queryContext = new SensorQueryContext { world = world, taskContext = ref taskContext, sensor = ref sensor, sensorShape = ref sensorShape, transform = transform };

			B2_ASSERT( sensorShape.sensorIndex == sensorIndex );
			AABB queryBounds = sensorShape.aabb;

			// Query all trees
			DynamicTree_OverlapAABB( trees[0], queryBounds, sensorShape.filter.maskBits, ref queryContext );
			DynamicTree_OverlapAABB( trees[1], queryBounds, sensorShape.filter.maskBits, ref queryContext );
			DynamicTree_OverlapAABB( trees[2], queryBounds, sensorShape.filter.maskBits, ref queryContext );

			// Sort the overlaps to enable finding begin and end events.
			new Span<Visitor>( sensor.overlaps2.data, 0, sensor.overlaps2.count ).Sort( CompareVisitors );

			// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
			int uniqueCount = 0;
			int overlapCount = sensor.overlaps2.count;
			Span<Visitor> overlapData = sensor.overlaps2.data;
			for ( int i = 0; i < overlapCount; ++i )
			{
				if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
				{
					overlapData[uniqueCount] = overlapData[i];
					uniqueCount += 1;
				}
			}
			sensor.overlaps2.count = uniqueCount;

			int count1 = sensor.overlaps1.count;
			int count2 = sensor.overlaps2.count;
			if ( count1 != count2 )
			{
				// something changed
				SetBit( ref taskContext.eventBits, sensorIndex );
			}
			else
			{
				for ( int i = 0; i < count1; ++i )
				{
					ref Visitor s1 = ref sensor.overlaps1.data[i];
					ref Visitor s2 = ref sensor.overlaps2.data[i];
					if ( s1.shapeId != s2.shapeId || s1.generation != s2.generation )
					{
						// something changed
						SetBit( ref taskContext.eventBits, sensorIndex );
						break;
					}
				}
			}
		}
	}

	internal static void OverlapSensors( World world )
	{
		int sensorCount = world.sensors.count;
		if ( sensorCount == 0 )
		{
			return;
		}

		B2_ASSERT( world.workerCount > 0 );
		for ( int i = 0; i < world.workerCount; ++i )
		{
			SetBitCountAndClear( ref world.sensorTaskContexts.data[i].eventBits, sensorCount );
		}

		// Parallel-for sensors overlaps
		int minRange = 16;
		ParallelFor( world, SensorTask, sensorCount, minRange, world );
		ref BitSet bitSet = ref world.sensorTaskContexts.data[0].eventBits;
		for ( int i = 1; i < world.workerCount; ++i )
		{
			InPlaceUnion( ref bitSet, ref world.sensorTaskContexts.data[i].eventBits );
		}

		// Iterate sensors bits and publish events
		// Process sensor state changes. Iterate over set bits
		Span<ulong> bits = bitSet.bits;
		uint blockCount = bitSet.blockCount;

		for ( uint k = 0; k < blockCount; ++k )
		{
			ulong word = bits[(int)k];
			while ( word != 0 )
			{
				uint ctz = CTZ64( word );
				int sensorIndex = (int)( 64 * k + ctz );

				ref Sensor sensor = ref world.sensors[sensorIndex];
				ref Shape sensorShape = ref world.shapes[sensor.shapeId];
				ShapeId sensorId = new ShapeId { index1 = sensor.shapeId + 1, world0 = world.worldId, generation = sensorShape.generation };

				int count1 = sensor.overlaps1.count;
				int count2 = sensor.overlaps2.count;
				ReadOnlySpan<Visitor> refs1 = sensor.overlaps1.data;
				ReadOnlySpan<Visitor> refs2 = sensor.overlaps2.data;
				// overlaps1 can have overlaps that end
				// overlaps2 can have overlaps that begin
				int index1 = 0, index2 = 0;
				while ( index1 < count1 && index2 < count2 )
				{
					ref readonly Visitor r1 = ref refs1[index1];
					ref readonly Visitor r2 = ref refs2[index2];
					if ( r1.shapeId == r2.shapeId )
					{
						if ( r1.generation < r2.generation )
						{
							// end
							ShapeId visitorId = new ShapeId { index1 = r1.shapeId + 1, world0 = world.worldId, generation = r1.generation };
							SensorEndTouchEvent @event = new SensorEndTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
							world.sensorEndEvents[world.endEventArrayIndex].Push( @event );
							index1 += 1;
						}
						else if ( r1.generation > r2.generation )
						{
							// begin
							ShapeId visitorId = new ShapeId { index1 = r2.shapeId + 1, world0 = world.worldId, generation = r2.generation };
							SensorBeginTouchEvent @event = new SensorBeginTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
							world.sensorBeginEvents.Push( @event );
							index2 += 1;
						}
						else
						{
							// persisted
							index1 += 1;
							index2 += 1;
						}
					}
					else if ( r1.shapeId < r2.shapeId )
					{
						// end
						ShapeId visitorId = new ShapeId { index1 = r1.shapeId + 1, world0 = world.worldId, generation = r1.generation };
						SensorEndTouchEvent @event = new SensorEndTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
						world.sensorEndEvents[world.endEventArrayIndex].Push( @event );
						index1 += 1;
					}
					else
					{
						// begin
						ShapeId visitorId = new ShapeId { index1 = r2.shapeId + 1, world0 = world.worldId, generation = r2.generation };
						SensorBeginTouchEvent @event = new SensorBeginTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
						world.sensorBeginEvents.Push( @event );
						index2 += 1;
					}
				}

				while ( index1 < count1 )
				{
					// end
					ref readonly Visitor r1 = ref refs1[index1];
					ShapeId visitorId = new ShapeId { index1 = r1.shapeId + 1, world0 = world.worldId, generation = r1.generation };
					SensorEndTouchEvent @event = new SensorEndTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
					world.sensorEndEvents[world.endEventArrayIndex].Push( @event );
					index1 += 1;
				}

				while ( index2 < count2 )
				{
					// begin
					ref readonly Visitor r2 = ref refs2[index2];
					ShapeId visitorId = new ShapeId { index1 = r2.shapeId + 1, world0 = world.worldId, generation = r2.generation };
					SensorBeginTouchEvent @event = new SensorBeginTouchEvent { sensorShapeId = sensorId, visitorShapeId = visitorId };
					world.sensorBeginEvents.Push( @event );
					index2 += 1;
				}

				// Clear the smallest set bit
				word = word & ( word - 1 );
			}
		}
	}

	internal static void DestroySensor( World world, ref Shape sensorShape )
	{
		ref Sensor sensor = ref world.sensors[sensorShape.sensorIndex];
		for ( int i = 0; i < sensor.overlaps2.count; ++i )
		{
			ref Visitor @ref = ref sensor.overlaps2.data[i];
			SensorEndTouchEvent @event = new SensorEndTouchEvent { sensorShapeId = new() { index1 = sensorShape.id + 1, world0 = world.worldId, generation = sensorShape.generation }, visitorShapeId = new() { index1 = @ref.shapeId + 1, world0 = world.worldId, generation = @ref.generation } };

			world.sensorEndEvents[world.endEventArrayIndex].Push( @event );
		}

		// Destroy sensor
		sensor.hits.Destroy();
		sensor.overlaps1.Destroy();
		sensor.overlaps2.Destroy();

		int movedIndex = world.sensors.RemoveSwap( sensorShape.sensorIndex );
		if ( movedIndex != NullIndex )
		{
			// Fixup moved sensor
			ref Sensor movedSensor = ref world.sensors[sensorShape.sensorIndex];
			ref Shape otherSensorShape = ref world.shapes[movedSensor.shapeId];
			otherSensorShape.sensorIndex = sensorShape.sensorIndex;
		}
	}

}
