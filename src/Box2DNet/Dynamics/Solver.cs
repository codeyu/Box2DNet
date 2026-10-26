// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// these are useful for solver testing
	private const int ITERATIONS = 1;
	private const int RELAX_ITERATIONS = 1;
	internal sealed class WorkerContext
	{
		public StepContext context;
		public int workerIndex;
		public object? userTask;
	}

	// Integrate velocities and apply damping
	private static void IntegrateVelocitiesTask( SolverBlock block, ref StepContext context )
	{
		Span<BodyState> states = context.states;
		Span<BodySim> sims = context.sims;
		B2_VALIDATE( block.startIndex + block.count <= context.world.solverSets.data[b2_awakeSet].bodyStates.count );

		Vec2 gravity = context.world.gravity;
		float h = context.h;

		for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
		{
			ref BodySim sim = ref sims[i];
			ref BodyState state = ref states[i];
			Vec2 v = state.linearVelocity;
			float w = state.angularVelocity;

			// Apply forces, torque, gravity, and damping
			// Apply damping.
			// Differential equation: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			float linearDamping = 1.0f / ( 1.0f + h * sim.linearDamping );
			float angularDamping = 1.0f / ( 1.0f + h * sim.angularDamping );

			// Gravity scale will be zero for kinematic bodies
			float gravityScale = sim.invMass > 0.0f ? sim.gravityScale : 0.0f;

			// lvd = h * im * f + h * g
			Vec2 linearVelocityDelta = Add( MulSV( h * sim.invMass, sim.force ), MulSV( h * gravityScale, gravity ) );
			float angularVelocityDelta = h * sim.invInertia * sim.torque;

			v = MulAdd( linearVelocityDelta, linearDamping, v );
			w = angularVelocityDelta + angularDamping * w;

			state.linearVelocity = v;
			state.angularVelocity = w;
		}
	}

	private static void IntegratePositionsTask( SolverBlock block, ref StepContext context )
	{
		B2_VALIDATE( block.startIndex + block.count <= context.world.solverSets.data[b2_awakeSet].bodyStates.count );

		Span<BodyState> states = context.states;
		float h = context.h;
		float maxLinearSpeed = context.maxLinearVelocity;
		float maxAngularSpeed = MaxRotation * context.inv_dt;
		float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
		float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

		for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
		{
			ref BodyState state = ref states[i];
			Vec2 v = state.linearVelocity;
			float w = state.angularVelocity;

			// Motion locks - these can be viewed as a constraint that comes last
			v.x = ( ( state.flags & b2_lockLinearX ) != 0 ) ? 0.0f : v.x;
			v.y = ( ( state.flags & b2_lockLinearY ) != 0 ) ? 0.0f : v.y;
			w = ( ( state.flags & b2_lockAngularZ ) != 0 ) ? 0.0f : w;

			// Clamp to max linear speed
			if ( Dot( v, v ) > maxLinearSpeedSquared )
			{
				float ratio = maxLinearSpeed / Length( v );
				v = MulSV( ratio, v );
				state.flags |= b2_isSpeedCapped;
			}

			// Clamp to max angular speed
			if ( w * w > maxAngularSpeedSquared && ( state.flags & b2_allowFastRotation ) == 0 )
			{
				float ratio = maxAngularSpeed / AbsFloat( w );
				w *= ratio;
				state.flags |= b2_isSpeedCapped;
			}

			state.linearVelocity = v;
			state.angularVelocity = w;
			state.deltaPosition = MulAdd( state.deltaPosition, h, state.linearVelocity );
			state.deltaRotation = IntegrateRotation( state.deltaRotation, h * state.angularVelocity );
		}
	}

	private const int B2_MAX_CONTINUOUS_SENSOR_HITS = 8;
	internal ref struct ContinuousContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => ContinuousQueryCallback( proxyId, userData, ref this );

		public World world;
		public ref BodySim fastBodySim;
		public ref Shape fastShape;
		public Vec2 centroid1;
		public Vec2 centroid2;
		public Sweep sweep;
		public Pos @base;
		public float fraction;
		public FixedArray8<SensorHit> sensorHits;
		public FixedArray8<float> sensorFractions;
		public int sensorCount;
	}

	private const float B2_CORE_FRACTION = 0.25f;
	// This is called from DynamicTree_OverlapAABB for continuous collision
	private static bool ContinuousQueryCallback( int proxyId, ulong userData, ref ContinuousContext continuousContext )
	{
		int shapeId = (int)userData;
		ref Shape fastShape = ref continuousContext.fastShape;
		ref BodySim fastBodySim = ref continuousContext.fastBodySim;
		B2_ASSERT( fastShape.sensorIndex == NullIndex );

		// Skip same shape
		if ( shapeId == fastShape.id )
		{
			return true;
		}

		World world = continuousContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		// Skip same body
		if ( shape.bodyId == fastShape.bodyId )
		{
			return true;
		}

		// Skip sensors unless the shapes want sensor events
		bool isSensor = shape.sensorIndex != NullIndex;
		if ( isSensor && ( shape.enableSensorEvents == false || fastShape.enableSensorEvents == false ) )
		{
			return true;
		}

		// Skip filtered shapes
		bool canCollide = ShouldShapesCollide( fastShape.filter, shape.filter );
		if ( canCollide == false )
		{
			return true;
		}

		ref Body body = ref world.bodies[shape.bodyId];
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		B2_ASSERT( body.type == BodyType.staticBody || ( fastBodySim.flags & b2_isBullet ) != 0 );

		// Skip bullets
		// Warning: it is only safe to read flags from other bodies, not body sims because there are
		// body sim flag writes in the continuous solver.
		if ( ( body.flags & b2_isBullet ) != 0 )
		{
			return true;
		}

		// Skip filtered bodies
		ref Body fastBody = ref world.bodies[fastBodySim.bodyId];
		canCollide = ShouldBodiesCollide( world, ref fastBody, ref body );
		if ( canCollide == false )
		{
			return true;
		}

		// Custom user filtering
		if ( shape.enableCustomFiltering || fastShape.enableCustomFiltering )
		{
			ref CustomFilterCallbackFcn customFilterFcn = ref world.customFilterFcn;
			if ( Unsafe.IsNullRef( ref customFilterFcn ) == false )
			{
				ShapeId idA = new ShapeId { index1 = shape.id + 1, world0 = world.worldId, generation = shape.generation };
				ShapeId idB = new ShapeId { index1 = fastShape.id + 1, world0 = world.worldId, generation = fastShape.generation };
				canCollide = customFilterFcn( idA, idB, world.customFilterContext );
				if ( canCollide == false )
				{
					return true;
				}
			}
		}

		// Early out on fast parallel movement over a chain shape.
		if ( shape.type == ShapeType.chainSegmentShape )
		{
			Transform transform = ToRelativeTransform( bodySim.transform, continuousContext.@base );
			Vec2 p1 = TransformPoint( transform, shape.chainSegment.segment.point1 );
			Vec2 p2 = TransformPoint( transform, shape.chainSegment.segment.point2 );
			Vec2 e = Sub( p2, p1 );
			float length;
			e = GetLengthAndNormalize( out length, e );
			if ( length > LinearSlop )
			{
				Vec2 c1 = continuousContext.centroid1;
				float separation1 = Cross( Sub( c1, p1 ), e );
				Vec2 c2 = continuousContext.centroid2;
				float separation2 = Cross( Sub( c2, p1 ), e );

				float coreDistance = B2_CORE_FRACTION * fastBodySim.minExtent;

				if ( separation1 < 0.0f || ( separation1 - separation2 < coreDistance && separation2 > coreDistance ) )
				{
					// Minimal clipping
					return true;
				}
			}
		}

		// todo_erin testing early out for segments
#if false
		if ( shape.type == ShapeType.segmentShape )
		{
			Transform transform = bodySim.transform;
			Vec2 p1 = TransformPoint( transform, shape.segment.point1 );
			Vec2 p2 = TransformPoint( transform, shape.segment.point2 );
			Vec2 e = Sub( p2, p1 );
			Vec2 c1 = continuousContext.centroid1;
			Vec2 c2 = continuousContext.centroid2;
			float offset1 = Cross( Sub( c1, p1 ), e );
			float offset2 = Cross( Sub( c2, p1 ), e );

			if ( offset1 > 0.0f && offset2 > 0.0f )
			{
				// Started behind or finished in front
				return true;
			}

			if ( offset1 < 0.0f && offset2 < 0.0f )
			{
				// Started behind or finished in front
				return true;
			}
		}
#endif

		TOIInput input = default;
		input.proxyA = MakeShapeDistanceProxy( shape );
		input.proxyB = MakeShapeDistanceProxy( fastShape );
		input.sweepA = MakeRelativeSweep( bodySim, continuousContext.@base );
		input.sweepB = continuousContext.sweep;
		input.maxFraction = continuousContext.fraction;

		TOIOutput output = ComputeTimeOfImpact( ref input );
		if ( isSensor )
		{
			// Only accept a sensor hit that is sooner than the current solid hit.
			if ( output.fraction <= continuousContext.fraction && continuousContext.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS )
			{
				int index = continuousContext.sensorCount;

				// The hit shape is a sensor
				SensorHit sensorHit = new SensorHit { sensorId = shape.id, visitorId = fastShape.id };

				continuousContext.sensorHits[index] = sensorHit;
				continuousContext.sensorFractions[index] = output.fraction;
				continuousContext.sensorCount += 1;
			}
		}
		else
		{
			float hitFraction = continuousContext.fraction;
			bool didHit = false;

			if ( 0.0f < output.fraction && output.fraction < continuousContext.fraction )
			{
				hitFraction = output.fraction;
				didHit = true;
			}
			else if ( 0.0f == output.fraction )
			{
				// fallback to TOI of a small circle around the fast shape centroid
				Vec2 centroid = GetShapeCentroid( fastShape );
				ShapeExtent extent = ComputeShapeExtent( fastShape, centroid );
				float radius = B2_CORE_FRACTION * extent.minExtent;
				input.proxyB = MakeProxy( ref centroid, 1, radius );
				output = ComputeTimeOfImpact( ref input );
				if ( 0.0f < output.fraction && output.fraction < continuousContext.fraction )
				{
					hitFraction = output.fraction;
					didHit = true;
				}
			}

			if ( didHit && ( shape.enablePreSolveEvents || fastShape.enablePreSolveEvents ) && world.preContinuousFcn != null )
			{
				ShapeId shapeIdA = new ShapeId { index1 = shape.id + 1, world0 = world.worldId, generation = shape.generation };
				ShapeId shapeIdB = new ShapeId { index1 = fastShape.id + 1, world0 = world.worldId, generation = fastShape.generation };

				// TOI runs in the base frame, lift the hit point back to world for the callback
				Pos worldPoint = OffsetPos( continuousContext.@base, output.point );
				didHit = world.preContinuousFcn( shapeIdA, shapeIdB, worldPoint, output.normal, world.preSolveContext );
			}

			if ( didHit )
			{
				continuousContext.fraction = hitFraction;
			}
		}

		// Continue query
		return true;
	}

	// Continuous collision of dynamic versus static
	private static void SolveContinuous( World world, int bodySimIndex, ref TaskContext taskContext, float dt )
	{
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		ref BodySim fastBodySim = ref awakeSet.bodySims[bodySimIndex];
		B2_VALIDATE( ( fastBodySim.flags & b2_isFast ) != 0 );

		// Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
		Pos @base = fastBodySim.center0;
		Sweep sweep = MakeRelativeSweep( fastBodySim, @base );

		Transform xf1 = default;
		xf1.q = sweep.q1;
		xf1.p = Sub( sweep.c1, RotateVector( sweep.q1, sweep.localCenter ) );

		Transform xf2 = default;
		xf2.q = sweep.q2;
		xf2.p = Sub( sweep.c2, RotateVector( sweep.q2, sweep.localCenter ) );

		DynamicTree staticTree = world.broadPhase.trees[BodyType.staticBody];
		DynamicTree kinematicTree = world.broadPhase.trees[BodyType.kinematicBody];
		DynamicTree dynamicTree = world.broadPhase.trees[BodyType.dynamicBody];
		ref Body fastBody = ref world.bodies[fastBodySim.bodyId];
		ContinuousContext context = default;
		context.world = world;
		context.sweep = sweep;
		context.@base = @base;
		context.fastBodySim = ref fastBodySim;
		context.fraction = 1.0f;

		bool isBullet = ( fastBodySim.flags & b2_isBullet ) != 0;

		int shapeId = fastBody.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape fastShape = ref world.shapes[shapeId];
			shapeId = fastShape.nextShapeId;

			context.fastShape = ref fastShape;
			context.centroid1 = TransformPoint( xf1, fastShape.localCentroid );
			context.centroid2 = TransformPoint( xf2, fastShape.localCentroid );

			AABB box1 = fastShape.aabb;

			// xf2 is in the base frame, compute the tight box near the origin then lift to world
			AABB box2 = OffsetAABB( ComputeShapeAABB( fastShape, MakeWorldTransform( xf2 ) ), @base );

			// Store this to avoid double computation in the case there is no impact event
			fastShape.aabb = box2;

			// No continuous collision for sensors (but still need the updated bounds)
			if ( fastShape.sensorIndex != NullIndex )
			{
				continue;
			}

			AABB sweptBox = AABB_Union( box1, box2 );

			DynamicTree_OverlapAABB( staticTree, sweptBox, DefaultMaskBits, ref context );

			if ( isBullet )
			{
				DynamicTree_OverlapAABB( kinematicTree, sweptBox, DefaultMaskBits, ref context );
				DynamicTree_OverlapAABB( dynamicTree, sweptBox, DefaultMaskBits, ref context );
			}
		}

		float speculativeDistance = SpeculativeDistance;

		if ( context.fraction < 1.0f )
		{
			// Handle time of impact event
			Rot q = NLerp( sweep.q1, sweep.q2, context.fraction );
			Vec2 c = Lerp( sweep.c1, sweep.c2, context.fraction );
			Vec2 origin = Sub( c, RotateVector( q, sweep.localCenter ) );

			// Advance body, lifting the base frame result back to world
			fastBodySim.transform.q = q;
			fastBodySim.transform.p = OffsetPos( @base, origin );
			fastBodySim.center = OffsetPos( @base, c );
			fastBodySim.rotation0 = q;
			fastBodySim.center0 = fastBodySim.center;

			// Warning: writing to the body sim flags means we should not read from other body sim flags in this function.
			fastBodySim.flags |= b2_hadTimeOfImpact;

			// Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
			ref BodyState fastBodyState = ref awakeSet.bodyStates[bodySimIndex];
			Vec2 v = fastBodyState.linearVelocity;
			float timeLoss = ( 1.0f - context.fraction ) * dt;
			Vec2 dv = MulSV( -timeLoss * fastBodySim.gravityScale, world.gravity );
			dv.x = ( ( fastBodyState.flags & b2_lockLinearX ) != 0 ) ? 0.0f : dv.x;
			dv.y = ( ( fastBodyState.flags & b2_lockLinearY ) != 0 ) ? 0.0f : dv.y;
			fastBodyState.linearVelocity = Add( v, dv );

			// Update body move event
			ref BodyMoveEvent @event = ref world.bodyMoveEvents[bodySimIndex];
			@event.transform = fastBodySim.transform;

			// Prepare AABBs for broad-phase.
			// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

			shapeId = fastBody.headShapeId;
			while ( shapeId != NullIndex )
			{
				ref Shape shape = ref world.shapes[shapeId];
				// Must recompute aabb at the interpolated transform
				AABB aabb = ComputeFatShapeAABB( shape, fastBodySim.transform, speculativeDistance );
				shape.aabb = aabb;

				ref AABB shapeFatAABB = ref world.fatAABBs.data[shapeId];
				if ( AABB_Contains( shapeFatAABB, aabb ) == false )
				{
					float margin = shape.aabbMargin;
					AABB fatAABB = default;
					fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
					fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
					fatAABB.upperBound.x = aabb.upperBound.x + margin;
					fatAABB.upperBound.y = aabb.upperBound.y + margin;
					shapeFatAABB = fatAABB;

					// Regular bodies mark the hierarchy as enlarged using atomic operations.
					// Bullets are handled separately at a later stage.
					if ( isBullet == false )
					{
						BroadPhase_MarkProxyMoved( world.broadPhase, shape.proxyKey, fatAABB );
					}
					else
					{
						fastBodySim.flags |= b2_enlargeBulletBounds;
					}
				}

				shapeId = shape.nextShapeId;
			}
		}
		else
		{
			// No time of impact event

			// Advance body
			fastBodySim.rotation0 = fastBodySim.transform.q;
			fastBodySim.center0 = fastBodySim.center;

			// Prepare AABBs for broad-phase
			shapeId = fastBody.headShapeId;
			while ( shapeId != NullIndex )
			{
				ref Shape shape = ref world.shapes[shapeId];
				// shape.aabb is still valid from above

				ref AABB shapeFatAABB = ref world.fatAABBs.data[shapeId];
				if ( AABB_Contains( shapeFatAABB, shape.aabb ) == false )
				{
					float margin = shape.aabbMargin;

					// Note: far from the origin the margin can be snapped to the nearest ULP.
					// This relevant for DP mode. So we lose the broad-phase hysteresis.
					AABB fatAABB = default;
					fatAABB.lowerBound.x = shape.aabb.lowerBound.x - margin;
					fatAABB.lowerBound.y = shape.aabb.lowerBound.y - margin;
					fatAABB.upperBound.x = shape.aabb.upperBound.x + margin;
					fatAABB.upperBound.y = shape.aabb.upperBound.y + margin;
					shapeFatAABB = fatAABB;

					if ( isBullet == false )
					{
						BroadPhase_MarkProxyMoved( world.broadPhase, shape.proxyKey, fatAABB );
					}
					else
					{
						fastBodySim.flags |= b2_enlargeBulletBounds;
					}
				}

				shapeId = shape.nextShapeId;
			}
		}

		// Push sensor hits on the the task context for serial processing.
		for ( int i = 0; i < context.sensorCount; ++i )
		{
			// Skip any sensor hits that occurred after a solid hit
			if ( context.sensorFractions[i] < context.fraction )
			{
				taskContext.sensorHits.Push( context.sensorHits[i] );
			}
		}
	}

	// Implements ParallelForCallback
	private static void FinalizeBodiesTask( int startIndex, int endIndex, int workerIndex, object? context )
	{
		StepContext stepContext = (StepContext)context!;
		World world = stepContext.world;
		Span<Body> bodies = world.bodies.data;
		Span<BodyState> states = stepContext.states;
		Span<BodySim> sims = stepContext.sims;
		B2_ASSERT( endIndex <= world.bodyMoveEvents.count );

		bool enableSleep = world.enableSleep;
		bool enableContinuous = world.enableContinuous;
		float timeStep = stepContext.dt;
		float invTimeStep = stepContext.inv_dt;
		ushort worldId = world.worldId;

		// The body move event array should already have the correct size
		Span<BodyMoveEvent> moveEvents = world.bodyMoveEvents.data;
		ref TaskContext taskContext = ref world.taskContexts.data[workerIndex];
		ref BitSet enlargedSimBitSet = ref taskContext.enlargedSimBitSet;
		ref BitSet awakeIslandBitSet = ref taskContext.awakeIslandBitSet;
		float speculativeDistance = SpeculativeDistance;

		for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
		{
			ref BodyState state = ref states[simIndex];
			ref BodySim sim = ref sims[simIndex];
			Vec2 v = state.linearVelocity;
			float w = state.angularVelocity;

			if ( IsValidVec2( v ) == false || IsValidFloat( w ) == false )
			{
				ref Body debugBody = ref bodies[sim.bodyId];
				Log( $"unstable: {debugBody.name}" );
			}

			B2_ASSERT( IsValidVec2( v ) );
			B2_ASSERT( IsValidFloat( w ) );

			sim.center = OffsetPos( sim.center, state.deltaPosition );
			sim.transform.q = NormalizeRot( MulRot( state.deltaRotation, sim.transform.q ) );

			// Use the velocity of the farthest point on the body to account for rotation.
			float maxVelocity = Length( v ) + AbsFloat( w ) * sim.maxExtent;

			// Sleep needs to observe position correction as well as true velocity.
			float maxDeltaPosition = Length( state.deltaPosition ) + AbsFloat( state.deltaRotation.s ) * sim.maxExtent;

			// Position correction is not as important for sleep as true velocity.
			float positionSleepFactor = 0.5f;
			float sleepVelocity = MaxFloat( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

			// reset state deltas
			state.deltaPosition = Vec2_zero;
			state.deltaRotation = Rot_identity;

			sim.transform.p = OffsetPos( sim.center, Neg( RotateVector( sim.transform.q, sim.localCenter ) ) );

			// cache miss here, however I need the shape list below
			ref Body body = ref bodies[sim.bodyId];
			body.bodyMoveIndex = simIndex;
			moveEvents[simIndex].transform = sim.transform;
			moveEvents[simIndex].bodyId = new BodyId { index1 = sim.bodyId + 1, world0 = worldId, generation = body.generation };
			moveEvents[simIndex].userData = body.userData;
			moveEvents[simIndex].fellAsleep = false;

			// reset applied force and torque
			sim.force = Vec2_zero;
			sim.torque = 0.0f;

			// If you hit this then it means you deferred mass computation but never called Body_UpdateMassFromShapes
			B2_ASSERT( ( body.flags & b2_dirtyMass ) == 0 );

			// Clear the transient flags (fast, speed capped, had TOI). These flags are conditionally set
			// as part of the code below.
			body.flags &= ~b2_bodyTransientFlags;
			sim.flags &= ~( b2_isFast | b2_bodyTransientFlags );

			// The body state flag knows about speed capping (used for debug draw).
			body.flags |= ( state.flags & b2_isSpeedCapped );
			state.flags &= ~b2_bodyTransientFlags;

			if ( enableSleep == false || ( body.flags & b2_enableSleep ) == 0 || sleepVelocity > body.sleepThreshold )
			{
				// Body is not sleepy
				body.sleepTime = 0.0f;

				float safetyFactor = body.safetyFactor;
				float maxMotion = MaxFloat( maxDeltaPosition, maxVelocity * timeStep );
				if ( body.type == BodyType.dynamicBody && enableContinuous && maxMotion > safetyFactor * sim.minExtent )
				{
					// This flag is used for debug draw and contact recycling.
					sim.flags |= b2_isFast;

					// Store fast bullets for processing later.
					// This is deterministic because the order of TOI sweeps doesn't matter.
					if ( ( sim.flags & b2_isBullet ) != 0 )
					{
						int bulletIndex = AtomicFetchAddInt( ref stepContext.bulletBodyCount, 1 );
						stepContext.bulletBodies[bulletIndex] = simIndex;
					}
					else
					{
						SolveContinuous( world, simIndex, ref taskContext, stepContext.dt );
					}
				}
				else
				{
					// Body is safe to advance
					sim.center0 = sim.center;
					sim.rotation0 = sim.transform.q;
				}
			}
			else
			{
				// Body is safe to advance and is falling asleep
				sim.center0 = sim.center;
				sim.rotation0 = sim.transform.q;
				body.sleepTime += timeStep;
			}

			// Any single body in an island can keep it awake
			ref Island island = ref world.islands[body.islandId];
			if ( body.sleepTime < TimeToSleep )
			{
				// keep island awake
				int islandIndex = island.localIndex;
				SetBit( ref awakeIslandBitSet, islandIndex );
			}
			else if ( island.constraintRemoveCount > 0 )
			{
				// Body wants to sleep but its island needs splitting first. Track the sleepiest candidate.
				// Break sleep time ties using the island id to ensure determinism. The cross worker reduction
				// breaks ties the same way.
				if ( body.sleepTime > taskContext.splitSleepTime ||
					 ( body.sleepTime == taskContext.splitSleepTime && body.islandId > taskContext.splitIslandId ) )
				{
					taskContext.splitIslandId = body.islandId;
					taskContext.splitSleepTime = body.sleepTime;
				}
			}

			// Update shapes AABBs
			WorldTransform transform = sim.transform;
			bool isFast = ( sim.flags & b2_isFast ) != 0;
			int shapeId = body.headShapeId;
			while ( shapeId != NullIndex )
			{
				ref Shape shape = ref world.shapes[shapeId];
				if ( isFast )
				{
					// For fast non-bullet bodies the AABB has already been updated in SolveContinuous
					// For fast bullet bodies the AABB will be updated at a later stage

					// Add to enlarged shapes regardless of AABB changes.
					// Bit-set to keep the move array sorted
					SetBit( ref enlargedSimBitSet, simIndex );
				}
				else
				{
					// Update bounds for slow bodies, including slow bullets.

					AABB aabb = ComputeFatShapeAABB( shape, transform, speculativeDistance );
					shape.aabb = aabb;

					ref AABB shapeFatAABB = ref world.fatAABBs.data[shapeId];
					if ( AABB_Contains( shapeFatAABB, aabb ) == false )
					{
						float margin = shape.aabbMargin;
						AABB fatAABB = default;
						fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
						fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
						fatAABB.upperBound.x = aabb.upperBound.x + margin;
						fatAABB.upperBound.y = aabb.upperBound.y + margin;
						shapeFatAABB = fatAABB;

						// Mark the hierarchy as enlarged using atomic operations.
						BroadPhase_MarkProxyMoved( world.broadPhase, shape.proxyKey, fatAABB );

						// Bit-set to keep the move array sorted
						SetBit( ref enlargedSimBitSet, simIndex );
					}
				}

				shapeId = shape.nextShapeId;
			}
		}
	}

	internal struct BlockDim
	{
		// number of items per block (except last block)
		public int size;

		// total number of blocks
		public int count;
	}

	// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
	// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
	// Each block of work has a minimum number of elements (block size). This in turn may limit the
	// number of blocks. If there are many elements then the block size is increased so there are
	// still at most M blocks of work per worker. M is a tunable number that has two goals:
	// 1. keep M small to reduce overhead
	// 2. keep M large enough for other workers to be able to steal work
	// The block size is a power of two to make math efficient.
	internal static BlockDim ComputeBlockCount( int itemCount, int minSize, int maxBlockCount )
	{
		BlockDim dim = default;
		if ( itemCount == 0 )
		{
			return dim;
		}

		if ( itemCount <= minSize * maxBlockCount )
		{
			dim.size = minSize;
		}
		else
		{
			dim.size = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
		}

		dim.count = ( itemCount + dim.size - 1 ) / dim.size;

		B2_ASSERT( dim.count >= 1 );
		B2_ASSERT( dim.size * dim.count >= itemCount );

		return dim;
	}

	// Initialize solver blocks for a contiguous range of items. Computes block size internally
	// from the same parameters used by ComputeBlockCount. The atomic claim counter is zeroed
	// so workers can CAS (0, 1) on the first stage that owns these blocks.
	private static void InitBlocks( Span<SyncBlock> blocks, BlockDim dim, int itemCount, SolverBlockType blockType, byte colorIndex )
	{
		if ( dim.count == 0 )
		{
			return;
		}

		B2_ASSERT( itemCount >= dim.count );

		// Compute the number of elements per block
		int blockSize = dim.size;

		// Simulation too big
		B2_ASSERT( blockSize <= ushort.MaxValue );

		for ( int i = 0; i < dim.count; ++i )
		{
			blocks[i].block.startIndex = i * blockSize;
			blocks[i].block.count = (ushort)blockSize;
			blocks[i].block.blockType = (byte)blockType;
			blocks[i].block.colorIndex = colorIndex;
			AtomicStoreInt( ref blocks[i].syncIndex, 0 );
		}

		// The last block may not be full
		blocks[dim.count - 1].block.count = (ushort)( itemCount - ( dim.count - 1 ) * blockSize );

		B2_VALIDATE( blocks[dim.count - 1].block.count <= blockSize );
		B2_VALIDATE( ( dim.count - 1 ) * dim.size + blocks[dim.count - 1].block.count == itemCount );
	}

	internal static int InitStage( Span<SolverStage> stages, int stageIndex, SolverStageType type, SyncBlock[] blocks, int blockStart, int blockCount, byte colorIndex )
	{
		ref SolverStage stage = ref stages[stageIndex];
		stage.type = type;
		stage.blocks = blocks;
		stage.blockStart = blockStart;
		stage.blockCount = blockCount;
		stage.colorIndex = colorIndex;
		AtomicStoreInt( ref stage.completionCount, 0 );
		return stageIndex + 1;
	}

	// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
	// All iterations of a given color share the same SyncBlock array so the per-block syncIndex
	// grows monotonically across stages within that color.
	private static int InitColorStages( Span<SolverStage> stages, int stageIndex, SolverStageType type, int iterations, int activeColorCount, SyncBlock[] graphBlocks, Span<int> colorBlocks, Span<int> colorBlockCounts, Span<int> activeColorIndices )
	{
		for ( int j = 0; j < iterations; ++j )
		{
			for ( int i = 0; i < activeColorCount; ++i )
			{
				stageIndex = InitStage( stages, stageIndex, type, graphBlocks, colorBlocks[i], colorBlockCounts[i], (byte)activeColorIndices[i] );
			}
		}
		return stageIndex;
	}

	private static void ExecuteBlock( ref SolverStage stage, ref StepContext context, SolverBlock block, int workerIndex )
	{
		SolverStageType stageType = stage.type;
		SolverBlockType blockType = (SolverBlockType)block.blockType;

		switch ( stageType )
		{
			case SolverStageType.stagePrepareJoints:
				PrepareJointsTask( block, ref context );
				break;

			case SolverStageType.stagePrepareContacts:
				PrepareContacts_Wide( block, ref context );
				break;

			case SolverStageType.stageIntegrateVelocities:
				IntegrateVelocitiesTask( block, ref context );
				break;

			case SolverStageType.stageWarmStart:
				if ( blockType == SolverBlockType.graphContactBlock )
				{
					WarmStartContacts_Wide( block, ref context );
				}
				else if ( blockType == SolverBlockType.graphJointBlock )
				{
					WarmStartJointsTask( block, ref context );
				}
				break;

			case SolverStageType.stageSolve:
				if ( blockType == SolverBlockType.graphContactBlock )
				{
					PushContacts_Wide( block, ref context );
				}
				else if ( blockType == SolverBlockType.graphJointBlock )
				{
					bool useBias = true;
					SolveJointsTask( block, ref context, useBias, workerIndex );
				}
				break;

			case SolverStageType.stageIntegratePositions:
				IntegratePositionsTask( block, ref context );
				break;

			case SolverStageType.stageRelax:
				if ( blockType == SolverBlockType.graphContactBlock )
				{
					SolveContacts_Wide( block, ref context );
				}
				else if ( blockType == SolverBlockType.graphJointBlock )
				{
					bool useBias = false;
					SolveJointsTask( block, ref context, useBias, workerIndex );
				}
				break;

			case SolverStageType.stageRestitution:
				if ( blockType == SolverBlockType.graphContactBlock )
				{
					ApplyRestitution_Wide( block, ref context );
				}
				break;

			case SolverStageType.stageStoreImpulses:
				StoreImpulses_Wide( block, ref context, workerIndex );
				break;
		}
	}

	// This staggers the worker start indices so they avoid touching the same solver blocks
	internal static int GetWorkerStartIndex( int workerIndex, int blockCount, int workerCount )
	{
		if ( blockCount <= workerCount )
		{
			return workerIndex < blockCount ? workerIndex : NullIndex;
		}

		int blocksPerWorker = blockCount / workerCount;
		int remainder = blockCount - blocksPerWorker * workerCount;
		return blocksPerWorker * workerIndex + MinInt( remainder, workerIndex );
	}

	// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
	// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
	private static void ExecuteStage( ref SolverStage stage, ref StepContext context, int previousSyncIndex, int syncIndex, int workerIndex )
	{
		int completedCount = 0;
		Span<SyncBlock> blocks = stage.blocks.AsSpan( stage.blockStart, stage.blockCount );
		int blockCount = stage.blockCount;

		int startIndex = GetWorkerStartIndex( workerIndex, blockCount, context.workerCount );
		if ( startIndex == NullIndex )
		{
			return;
		}

		B2_ASSERT( 0 <= startIndex && startIndex < blockCount );

		int blockIndex = startIndex;
		for ( int i = 0; i < blockCount; ++i )
		{
			// Read before attempting the CAS.
			if ( AtomicLoadInt( ref blocks[blockIndex].syncIndex ) == previousSyncIndex &&
				 AtomicCompareExchangeInt( ref blocks[blockIndex].syncIndex, previousSyncIndex, syncIndex ) )
			{
				B2_ASSERT( stage.type != SolverStageType.stagePrepareContacts || syncIndex < 2 );
				B2_ASSERT( completedCount < blockCount );

				// Pass the descriptor by value -- the wrapping SyncBlock holds the atomic
				// syncIndex but we only copy .block, so the copy never aliases the CAS target.
				ExecuteBlock( ref stage, ref context, blocks[blockIndex].block, workerIndex );
				completedCount += 1;
			}

			blockIndex += 1;
			if ( blockIndex >= blockCount )
			{
				blockIndex = 0;
			}
		}

		AtomicFetchAddInt( ref stage.completionCount, completedCount );
	}

	// Execute a stage on worker 0 (main thread).
	private static void ExecuteMainStage( ref SolverStage stage, ref StepContext context, uint syncBits )
	{
		int blockCount = stage.blockCount;
		if ( blockCount == 0 )
		{
			return;
		}

		int workerIndex = 0;

		if ( blockCount == 1 )
		{
			ExecuteBlock( ref stage, ref context, stage.blocks[stage.blockStart].block, workerIndex );
		}
		else
		{
			AtomicStoreU32( ref context.atomicSyncBits, syncBits );

			int syncIndex = (int)( ( syncBits >> 16 ) & 0xFFFF );
			B2_ASSERT( syncIndex > 0 );
			int previousSyncIndex = syncIndex - 1;

			ExecuteStage( ref stage, ref context, previousSyncIndex, syncIndex, workerIndex );

			// Spin waiting for thieves to finish
			while ( AtomicLoadInt( ref stage.completionCount ) != blockCount )
			{
				Pause();
			}

			AtomicStoreInt( ref stage.completionCount, 0 );
		}
	}

	// Parallel solver task
	private static void SolverTask( object? taskContext )
	{
		WorkerContext workerContext = (WorkerContext)taskContext!;
		int workerIndex = workerContext.workerIndex;
		StepContext context = workerContext.context;
		int activeColorCount = context.activeColorCount;
		Span<SolverStage> stages = context.stages;
		ref Profile profile = ref context.world.profile;
		if ( workerIndex == 0 )
		{
			// The orchestrator slot is a race. The calling thread of World_Step also enters here
			// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
			// out of order, has fewer threads than workerCount, or runs the task synchronously
			// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
			// returns and lets the spinner-only path handle workers >0.
			if ( AtomicCompareExchangeInt( ref context.mainClaimed, 0, 1 ) == false )
			{
				return;
			}

			// Main thread synchronizes the workers and does work itself.
			//
			// This single task is able to fully complete all work even if all other workers are
			// blocked, so a fully serial task system still drives the simulation forward.

			// Stages are re-used by loops so that I don't need more stages for large substep counts.
			// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
			// The stage index and sync indices are combined in to sync bits for atomic synchronization.
			// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
			// setup makes this easy to do.

			/*
			Stage sequence
			SolverStageType.stagePrepareJoints,
			SolverStageType.stagePrepareContacts,
			SolverStageType.stageIntegrateVelocities,
			SolverStageType.stageWarmStart,
			SolverStageType.stageSolve,
			SolverStageType.stageIntegratePositions,
			SolverStageType.stageRelax,
			SolverStageType.stageRestitution,
			SolverStageType.stageStoreImpulses
			*/

			ulong ticks = GetTicks();

			int bodySyncIndex = 1;
			int stageIndex = 0;

			// Prepare joint constraints
			int jointSyncIndex = 1;
			uint syncBits = ( (uint)jointSyncIndex << 16 ) | (uint)stageIndex;
			B2_ASSERT( stages[stageIndex].type == SolverStageType.stagePrepareJoints );
			ExecuteMainStage( ref stages[stageIndex], ref context, syncBits );
			stageIndex += 1;
			jointSyncIndex += 1;

			// Prepare contact constraints
			int contactSyncIndex = 1;
			syncBits = ( (uint)contactSyncIndex << 16 ) | (uint)stageIndex;
			B2_ASSERT( stages[stageIndex].type == SolverStageType.stagePrepareContacts );
			ExecuteMainStage( ref stages[stageIndex], ref context, syncBits );
			stageIndex += 1;
			contactSyncIndex += 1;

			// Single-threaded overflow work. These constraints don't fit in the graph coloring.
			PrepareJoints_Overflow( ref context );
			PrepareContacts_Overflow( ref context );

			profile.prepareConstraints += GetMillisecondsAndReset( ref ticks );

			int graphSyncIndex = 1;
			int subStepCount = context.subStepCount;
			for ( int subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex )
			{
				// stage index restarted each iteration
				// syncBits still increases monotonically because the upper bits increase each iteration
				int iterationStageIndex = stageIndex;

				// Integrate velocities
				syncBits = ( (uint)bodySyncIndex << 16 ) | (uint)iterationStageIndex;
				B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageIntegrateVelocities );
				ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
				iterationStageIndex += 1;
				bodySyncIndex += 1;

				profile.integrateVelocities += GetMillisecondsAndReset( ref ticks );

				// Warm start constraints
				WarmStartJoints_Overflow( ref context );
				WarmStartContacts_Overflow( ref context );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( (uint)graphSyncIndex << 16 ) | (uint)iterationStageIndex;
					B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageWarmStart );
					ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;

				profile.warmStart += GetMillisecondsAndReset( ref ticks );

				// Solve constraints
				bool useBias = true;
				for ( int j = 0; j < ITERATIONS; ++j )
				{
					// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
					SolveJoints_Overflow( ref context, useBias );
					PushContacts_Overflow( ref context );

					for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
					{
						syncBits = ( (uint)graphSyncIndex << 16 ) | (uint)iterationStageIndex;
						B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageSolve );
						ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
						iterationStageIndex += 1;
					}
					graphSyncIndex += 1;
				}

				profile.solveImpulses += GetMillisecondsAndReset( ref ticks );

				// Integrate positions
				B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageIntegratePositions );
				syncBits = ( (uint)bodySyncIndex << 16 ) | (uint)iterationStageIndex;
				ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
				iterationStageIndex += 1;
				bodySyncIndex += 1;

				profile.integratePositions += GetMillisecondsAndReset( ref ticks );

				// Relax constraints
				useBias = false;
				for ( int j = 0; j < RELAX_ITERATIONS; ++j )
				{
					SolveJoints_Overflow( ref context, useBias );
					SolveContacts_Overflow( ref context );
					for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
					{
						syncBits = ( (uint)graphSyncIndex << 16 ) | (uint)iterationStageIndex;
						B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageRelax );
						ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
						iterationStageIndex += 1;
					}
					graphSyncIndex += 1;
				}

				profile.relaxImpulses += GetMillisecondsAndReset( ref ticks );
			}

			// Advance the stage according to the sub-stepping tasks just completed
			// integrate velocities / warm start / solve / integrate positions / relax
			stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

			int restitutionIterations = context.world.restitutionIterations;
			if ( restitutionIterations > 0 && AtomicLoadInt( ref context.anyRestitution ) != 0 )
			{
				for ( int j = 0; j < restitutionIterations; ++j )
				{
					ApplyRestitution_Overflow( ref context );

					int iterationStageIndex = stageIndex;
					for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
					{
						syncBits = ( (uint)graphSyncIndex << 16 ) | (uint)iterationStageIndex;
						B2_ASSERT( stages[iterationStageIndex].type == SolverStageType.stageRestitution );
						ExecuteMainStage( ref stages[iterationStageIndex], ref context, syncBits );
						iterationStageIndex += 1;
					}
					graphSyncIndex += 1;
				}

				profile.restitution += GetMillisecondsAndReset( ref ticks );
			}

			stageIndex += activeColorCount;

			// Store impulses
			StoreImpulses_Overflow( ref context );

			syncBits = ( (uint)contactSyncIndex << 16 ) | (uint)stageIndex;
			B2_ASSERT( stages[stageIndex].type == SolverStageType.stageStoreImpulses );
			ExecuteMainStage( ref stages[stageIndex], ref context, syncBits );

			profile.storeImpulses += GetMillisecondsAndReset( ref ticks );

			// Signal workers to finish
			AtomicStoreU32( ref context.atomicSyncBits, uint.MaxValue );

			B2_ASSERT( stageIndex + 1 == context.stageCount );
			return;
		}

		// Worker spins and waits for work
		uint lastSyncBits = 0;
		// ulong maxSpinTime = 10;
		while ( true )
		{
			// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
			// parallel simulation with graph coloring.
			// todo improve this spinner
			uint syncBits;
			int spinCount = 0;
			while ( ( syncBits = AtomicLoadU32( ref context.atomicSyncBits ) ) == lastSyncBits )
			{
				if ( spinCount > 5 )
				{
					Yield();
					spinCount = 0;
				}
				else
				{
					// Using the cycle counter helps to account for variation in mm_pause timing across different
					// CPUs. However, this is X64 only.
					// ulong prev = __rdtsc();
					// do
					//{
					//	_mm_pause();
					//}
					// while ((__rdtsc() - prev) < maxSpinTime);
					// maxSpinTime += 10;

					Pause();
					Pause();
					spinCount += 1;
				}
			}

			if ( syncBits == uint.MaxValue )
			{
				// sentinel hit
				break;
			}

			int stageIndex = (int)( syncBits & 0xFFFF );
			B2_ASSERT( stageIndex < context.stageCount );

			int syncIndex = (int)( ( syncBits >> 16 ) & 0xFFFF );
			B2_ASSERT( syncIndex > 0 );

			int previousSyncIndex = syncIndex - 1;

			ref SolverStage stage = ref stages[stageIndex];
			ExecuteStage( ref stage, ref context, previousSyncIndex, syncIndex, workerIndex );

			lastSyncBits = syncBits;
		}
	}

	private static void BulletBodyTask( int startIndex, int endIndex, int workerIndex, object? context )
	{
		StepContext stepContext = (StepContext)context!;
		ref TaskContext taskContext = ref stepContext.world.taskContexts.data[workerIndex];
		B2_ASSERT( startIndex <= endIndex );

		for ( int i = startIndex; i < endIndex; ++i )
		{
			int simIndex = stepContext.bulletBodies[i];
			SolveContinuous( stepContext.world, simIndex, ref taskContext, stepContext.dt );
		}
	}

	// Solve with graph coloring
	internal static void Solve( World world, ref StepContext stepContext )
	{
		int simdShift = world.simdWidth == 8 ? 3 : 2;

		// Only count steps that advance the simulation
		world.stepIndex += 1;

		// Are there any awake bodies? This scenario should not be important for profiling.
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		int awakeBodyCount = awakeSet.bodySims.count;
		if ( awakeBodyCount == 0 )
		{
			return;
		}

		// Solve constraints using graph coloring
		{
			ulong setupTicks = GetTicks();

			// Prepare buffers for bullets
			AtomicStoreInt( ref stepContext.bulletBodyCount, 0 );
			stepContext.bulletBodies = world.stack.Alloc<int>( awakeBodyCount, "bullet bodies" );

			ConstraintGraph graph = world.constraintGraph;
			Span<GraphColor> colors = graph.colors;
			stepContext.sims = awakeSet.bodySims.data;
			stepContext.states = awakeSet.bodyStates.data;

			// count contacts, joints, and colors
			int activeColorCount = 0;
			for ( int i = 0; i < GraphColorCount - 1; ++i )
			{
				int perColorContactCount = colors[i].contactSims.count;
				int perColorJointCount = colors[i].jointSims.count;
				int occupancyCount = perColorContactCount + perColorJointCount;
				activeColorCount += occupancyCount > 0 ? 1 : 0;
			}

			// prepare for move events
			world.bodyMoveEvents.Resize( awakeBodyCount );

			int workerCount = world.workerCount;

			// Target 4 blocks per worker to allow work stealing
			int maxBlockCount = 4 * workerCount;

			// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
			int minBodiesPerBlock = 32;
			BlockDim bodyDim = ComputeBlockCount( awakeBodyCount, minBodiesPerBlock, maxBlockCount );

			const int minContactsPerBlock = 4;
			const int minJointsPerBlock = 4;

			// Configure blocks for tasks parallel-for each active graph color
			// The blocks are a mix of wide contact blocks and joint blocks
			FixedArray24<int> activeColorIndices = default;
			FixedArray24<int> colorContactCounts = default;
			FixedArray24<int> colorJointCounts = default;
			FixedArray24<BlockDim> graphContactDims = default;
			FixedArray24<BlockDim> graphJointDims = default;
			int graphBlockCount = 0;

			// c is the active color index
			int wideContactCount = 0;
			int jointCount = 0;
			int c = 0;
			for ( int i = 0; i < GraphColorCount - 1; ++i )
			{
				int colorContactCount = colors[i].contactSims.count;
				int colorJointCount = colors[i].jointSims.count;

				if ( colorContactCount + colorJointCount == 0 )
				{
					continue;
				}

				activeColorIndices[c] = i;

				// Ceiling for wide constraint count
				int colorContactCountW = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> simdShift ) + 1 : 0;
				wideContactCount += colorContactCountW;
				colorContactCounts[c] = colorContactCountW;

				colorJointCounts[c] = colorJointCount;
				jointCount += colorJointCount;

				// Graph solver block dimensions
				graphContactDims[c] = ComputeBlockCount( colorContactCountW, minContactsPerBlock, maxBlockCount );
				graphJointDims[c] = ComputeBlockCount( colorJointCount, minJointsPerBlock, maxBlockCount );
				graphBlockCount += graphContactDims[c].count + graphJointDims[c].count;

				c += 1;
			}
			activeColorCount = c;

			// Prepare and store run as one flat parallel-for over the entire wide constraint range,
			// partitioned into uniformly sized blocks. Color info is consulted inside the task via
			// a small span array, so blocks do not need to honor color boundaries here.
			BlockDim contactPrepareDim = ComputeBlockCount( wideContactCount, minContactsPerBlock, maxBlockCount );
			BlockDim jointPrepareDim = ComputeBlockCount( jointCount, minJointsPerBlock, maxBlockCount );

			int wideContactConstraintByteCount = GetWideContactConstraintByteCount( world.simdWidth );
			ContactConstraintW4[] wideContactConstraints = world.stack.Alloc<ContactConstraintW4>( wideContactCount, "contact constraint" );
			ref GraphColor overflow = ref colors[OverflowIndex];
			int overflowCount = overflow.contactSims.count;
			ContactConstraint[] overflowContacts = world.stack.Alloc<ContactConstraint>( overflowCount, "overflow contact constraint" );
			overflow.overflowConstraints = overflowContacts;

			// Build the span table for the flat prepare/store parallel-for while I slice the
			// wide constraint buffer across colors. One entry per active color plus a sentinel
			// at wideContactCount.
			ContactPrepareSpan[] contactPrepareSpans = new ContactPrepareSpan[GraphColorCount + 1];
			JointPrepareSpan[] jointPrepareSpans = new JointPrepareSpan[GraphColorCount + 1];

			// Distribute transient constraints to each graph color and prepare spans
			{
				int wideBase = 0;
				int jointBase = 0;
				for ( int i = 0; i < activeColorCount; ++i )
				{
					int j = activeColorIndices[i];
					ref GraphColor color = ref colors[j];
					int colorContactCount = color.contactSims.count;
					contactPrepareSpans[i].start = wideBase;
					contactPrepareSpans[i].count = colorContactCount;
					contactPrepareSpans[i].contacts = color.contactSims.data;

					if ( colorContactCount == 0 )
					{
						color.wideConstraintStart = 0;
						color.wideConstraintCount = 0;
					}
					else
					{
						color.wideConstraintStart = wideBase;

						int colorContactCountW = ( ( colorContactCount - 1 ) >> simdShift ) + 1;
						color.wideConstraintCount = colorContactCountW;

						// Zero remainder lanes in the tail wide slot so prepare workers don't need to
						// initialize them.
						if ( ( colorContactCount & ( world.simdWidth - 1 ) ) != 0 )
						{
							wideContactConstraints[wideBase + colorContactCountW - 1] = default;
						}

						wideBase += colorContactCountW;
					}

					jointPrepareSpans[i].start = jointBase;
					jointPrepareSpans[i].count = color.jointSims.count;
					jointPrepareSpans[i].joints = color.jointSims.data;
					jointBase += color.jointSims.count;
				}

				// Sentinel
				contactPrepareSpans[activeColorCount].start = wideContactCount;
				contactPrepareSpans[activeColorCount].count = 0;
				contactPrepareSpans[activeColorCount].contacts = default;
				B2_ASSERT( wideBase == wideContactCount );

				jointPrepareSpans[activeColorCount].start = jointCount;
				jointPrepareSpans[activeColorCount].count = 0;
				jointPrepareSpans[activeColorCount].joints = default;
				B2_ASSERT( jointBase == jointCount );
			}

			int stageCount = 0;

			// SolverStageType.stagePrepareJoints
			stageCount += 1;
			// SolverStageType.stagePrepareContacts
			stageCount += 1;
			// SolverStageType.stageIntegrateVelocities
			stageCount += 1;
			// SolverStageType.stageWarmStart
			stageCount += activeColorCount;
			// SolverStageType.stageSolve
			stageCount += ITERATIONS * activeColorCount;
			// SolverStageType.stageIntegratePositions
			stageCount += 1;
			// SolverStageType.stageRelax
			stageCount += RELAX_ITERATIONS * activeColorCount;
			// SolverStageType.stageRestitution
			stageCount += activeColorCount;
			// SolverStageType.stageStoreImpulses
			stageCount += 1;

			SolverStage[] stages = world.stack.Alloc<SolverStage>( stageCount, "stages" );
			SyncBlock[] bodyBlocks = world.stack.Alloc<SyncBlock>( bodyDim.count, "body blocks" );
			SyncBlock[] contactBlocks = world.stack.Alloc<SyncBlock>( contactPrepareDim.count, "contact blocks" );
			SyncBlock[] jointBlocks = world.stack.Alloc<SyncBlock>( jointPrepareDim.count, "joint blocks" );
			SyncBlock[] graphBlocks = world.stack.Alloc<SyncBlock>( graphBlockCount, "graph blocks" );
			// Split an awake island. This modifies:
			// - stack allocator
			// - world island array and solver set
			// - island indices on bodies, contacts, and joints
			// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
			// Note: cannot split islands in parallel with FinalizeBodies
			object? splitIslandTask = null;
			if ( world.splitIslandId != NullIndex )
			{
				if ( world.taskCount < MaxTasks )
				{
					splitIslandTask = world.enqueueTaskFcn( SplitIslandTask, world, world.userTaskContext );
					world.taskCount += 1;
					world.activeTaskCount += splitIslandTask == null ? 0 : 1;
				}
				else
				{
					SplitIslandTask( world );
				}
			}

			// Prepare body blocks
			InitBlocks( bodyBlocks, bodyDim, awakeBodyCount, SolverBlockType.bodyBlock, byte.MaxValue );

			// Prepare blocks as a single flat parallel-for over the whole constraint range.
			// The task walks spans to decode flat slot indices back to per-color arrays.
			InitBlocks( contactBlocks, contactPrepareDim, wideContactCount, SolverBlockType.contactBlock, byte.MaxValue );
			InitBlocks( jointBlocks, jointPrepareDim, jointCount, SolverBlockType.jointBlock, byte.MaxValue );

			// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
			Span<int> graphColorBlocks = stackalloc int[GraphColorCount];
			int baseGraphBlock = 0;
			Span<int> graphBlockCounts = stackalloc int[GraphColorCount];
			for ( int i = 0; i < activeColorCount; ++i )
			{
				graphColorBlocks[i] = baseGraphBlock;

				byte colorIndex = (byte)activeColorIndices[i];
				InitBlocks( graphBlocks.AsSpan( baseGraphBlock ), graphJointDims[i], colorJointCounts[i], SolverBlockType.graphJointBlock, colorIndex );
				baseGraphBlock += graphJointDims[i].count;

				InitBlocks( graphBlocks.AsSpan( baseGraphBlock ), graphContactDims[i], colorContactCounts[i], SolverBlockType.graphContactBlock, colorIndex );
				baseGraphBlock += graphContactDims[i].count;

				graphBlockCounts[i] = graphJointDims[i].count + graphContactDims[i].count;
			}

			B2_ASSERT( baseGraphBlock == graphBlockCount );

			int stage = 0;
			stage = InitStage( stages, stage, SolverStageType.stagePrepareJoints, jointBlocks, 0, jointPrepareDim.count, byte.MaxValue );
			stage = InitStage( stages, stage, SolverStageType.stagePrepareContacts, contactBlocks, 0, contactPrepareDim.count, byte.MaxValue );
			stage = InitStage( stages, stage, SolverStageType.stageIntegrateVelocities, bodyBlocks, 0, bodyDim.count, byte.MaxValue );
			stage = InitColorStages( stages, stage, SolverStageType.stageWarmStart, 1, activeColorCount, graphBlocks, graphColorBlocks, graphBlockCounts,
									 activeColorIndices );
			stage = InitColorStages( stages, stage, SolverStageType.stageSolve, ITERATIONS, activeColorCount, graphBlocks, graphColorBlocks, graphBlockCounts,
									 activeColorIndices );
			stage = InitStage( stages, stage, SolverStageType.stageIntegratePositions, bodyBlocks, 0, bodyDim.count, byte.MaxValue );
			stage = InitColorStages( stages, stage, SolverStageType.stageRelax, RELAX_ITERATIONS, activeColorCount, graphBlocks, graphColorBlocks,
									 graphBlockCounts, activeColorIndices );
			stage = InitColorStages( stages, stage, SolverStageType.stageRestitution, 1, activeColorCount, graphBlocks, graphColorBlocks, graphBlockCounts,
									 activeColorIndices );
			stage = InitStage( stages, stage, SolverStageType.stageStoreImpulses, contactBlocks, 0, contactPrepareDim.count, byte.MaxValue );

			B2_ASSERT( stage == stageCount );

			B2_ASSERT( workerCount <= MaxWorkers );
			WorkerContext[] workerContext = new WorkerContext[MaxWorkers];

			stepContext.graph = graph;
			stepContext.activeColorCount = activeColorCount;
			stepContext.workerCount = workerCount;
			stepContext.stageCount = stageCount;
			stepContext.stages = stages;
			stepContext.wideContactConstraints = wideContactConstraints;
			stepContext.contactPrepareSpans = contactPrepareSpans;
			stepContext.wideContactCount = wideContactCount;
			stepContext.jointPrepareSpans = jointPrepareSpans;
			AtomicStoreU32( ref stepContext.atomicSyncBits, 0 );
			AtomicStoreInt( ref stepContext.mainClaimed, 0 );

			world.profile.solverSetup = GetMillisecondsAndReset( ref setupTicks );
			ulong constraintTicks = GetTicks();

			int jointIdCapacity = GetIdCapacity( ref world.jointIdPool );
			int contactIdCapacity = GetIdCapacity( ref world.contactIdPool );
			for ( int i = 0; i < workerCount; ++i )
			{
				ref TaskContext taskContext = ref world.taskContexts[i];
				SetBitCountAndClear( ref taskContext.jointStateBitSet, jointIdCapacity );
				SetBitCountAndClear( ref taskContext.hitEventBitSet, contactIdCapacity );
				taskContext.hasHitEvents = false;

				workerContext[i] = new WorkerContext();
				workerContext[i].context = stepContext;
				workerContext[i].workerIndex = i;

				if ( world.taskCount < MaxTasks )
				{
					workerContext[i].userTask = world.enqueueTaskFcn( SolverTask, workerContext[i], world.userTaskContext );
					world.taskCount += 1;
					world.activeTaskCount += workerContext[i].userTask == null ? 0 : 1;
				}
				else
				{
					workerContext[i].userTask = null;
					SolverTask( workerContext[i] );
				}
			}

			// The calling thread of World_Step also enters SolverTask as worker 0 and races for the
			// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
			// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
			// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
			// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
			// path handle workers >0.
			WorkerContext callerContext = new WorkerContext { context = stepContext, workerIndex = 0, userTask = null };
			SolverTask( callerContext );

			// Finish constraint solve
			for ( int i = 0; i < workerCount; ++i )
			{
				if ( workerContext[i].userTask != null )
				{
					world.finishTaskFcn( workerContext[i].userTask, world.userTaskContext );
					world.activeTaskCount -= 1;
				}
			}

			// Finish island split
			if ( splitIslandTask != null )
			{
				world.finishTaskFcn( splitIslandTask, world.userTaskContext );
				world.activeTaskCount -= 1;
			}
			world.splitIslandId = NullIndex;

			world.profile.constraints = GetMillisecondsAndReset( ref constraintTicks );
			ulong transformTicks = GetTicks();

			// Prepare contact, enlarged body, and island bit sets used in body finalization.
			int awakeIslandCount = awakeSet.islandSims.count;
			for ( int i = 0; i < world.workerCount; ++i )
			{
				ref TaskContext taskContext = ref world.taskContexts.data[i];
				taskContext.sensorHits.count = 0;
				SetBitCountAndClear( ref taskContext.enlargedSimBitSet, awakeBodyCount );
				SetBitCountAndClear( ref taskContext.awakeIslandBitSet, awakeIslandCount );
				taskContext.splitIslandId = NullIndex;
				taskContext.splitSleepTime = 0.0f;
			}

			// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
			// broad-phase.
			if ( world.userTreeTask != null )
			{
				world.finishTaskFcn( world.userTreeTask, world.userTaskContext );
				world.userTreeTask = null;
				world.activeTaskCount -= 1;
			}

			ValidateNoEnlarged( world.broadPhase );

			// Finalize bodies. Must happen after the constraint solver and after island splitting.
			ParallelFor( world, FinalizeBodiesTask, awakeBodyCount, 64, stepContext );

			world.stack.Free( graphBlocks );
			world.stack.Free( jointBlocks );
			world.stack.Free( contactBlocks );
			world.stack.Free( bodyBlocks );
			world.stack.Free( stages );
			world.stack.Free( overflowContacts );
			world.stack.Free( wideContactConstraints );

			world.profile.transforms = GetMilliseconds( transformTicks );
		}

		// Report joint events
		{
			ulong jointEventTicks = GetTicks();

			// Gather bits for all joints that have force/torque events
			ref BitSet jointStateBitSet = ref world.taskContexts.data[0].jointStateBitSet;
			for ( int i = 1; i < world.workerCount; ++i )
			{
				InPlaceUnion( ref jointStateBitSet, ref world.taskContexts.data[i].jointStateBitSet );
			}

			{
				uint wordCount = jointStateBitSet.blockCount;
				Span<ulong> bits = jointStateBitSet.bits;
				Span<Joint> jointArray = world.joints.data;
				ushort worldIndex0 = world.worldId;

				for ( uint k = 0; k < wordCount; ++k )
				{
					ulong word = bits[(int)k];
					while ( word != 0 )
					{
						uint ctz = CTZ64( word );
						int jointId = (int)( 64 * k + ctz );

						B2_ASSERT( jointId < world.joints.capacity );

						ref Joint joint = ref jointArray[jointId];
						B2_ASSERT( joint.setIndex == b2_awakeSet );

						JointEvent @event = new JointEvent { jointId = new() { index1 = jointId + 1, world0 = worldIndex0, generation = joint.generation }, userData = joint.userData };

						world.jointEvents.Push( @event );

						// Clear the smallest set bit
						word = word & ( word - 1 );
					}
				}
			}

			world.profile.jointEvents = GetMilliseconds( jointEventTicks );
		}

		// Report hit events
		{
			ulong hitTicks = GetTicks();

			B2_ASSERT( world.contactHitEvents.count == 0 );

			// Fast path: skip if no worker flagged any hit-event candidates during impulse storing.
			bool anyHitEvents = false;
			for ( int i = 0; i < world.workerCount; ++i )
			{
				if ( world.taskContexts.data[i].hasHitEvents )
				{
					anyHitEvents = true;
					break;
				}
			}

			if ( anyHitEvents )
			{
				// Union per-worker bits into worker 0's bit set.
				ref BitSet hitEventBitSet = ref world.taskContexts.data[0].hitEventBitSet;
				for ( int i = 1; i < world.workerCount; ++i )
				{
					if ( world.taskContexts.data[i].hasHitEvents )
					{
						InPlaceUnion( ref hitEventBitSet, ref world.taskContexts.data[i].hitEventBitSet );
					}
				}

				float threshold = world.hitEventThreshold;
				Span<GraphColor> colors = world.constraintGraph.colors;
				Span<Contact> contactArray = world.contacts.data;
				Span<Shape> shapeArray = world.shapes.data;
				ushort worldId = world.worldId;

				uint wordCount = hitEventBitSet.blockCount;
				Span<ulong> bits = hitEventBitSet.bits;
				for ( uint k = 0; k < wordCount; ++k )
				{
					ulong word = bits[(int)k];
					while ( word != 0 )
					{
						uint ctz = CTZ64( word );
						int contactId = (int)( 64 * k + ctz );

						ref Contact contact = ref contactArray[contactId];
						B2_ASSERT( contact.setIndex == b2_awakeSet && contact.colorIndex != NullIndex );

						ref GraphColor color = ref colors[contact.colorIndex];
						ref ContactSim contactSim = ref color.contactSims.data[contact.localIndex];
						ContactHitEvent @event = default;
						@event.approachSpeed = threshold;

						scoped ref ManifoldPoint bestPoint = ref Unsafe.NullRef<ManifoldPoint>();
						int pointCount = contactSim.manifold.pointCount;
						for ( int p = 0; p < pointCount; ++p )
						{
							ref ManifoldPoint mp = ref contactSim.manifold.points[p];
							float approachSpeed = -mp.normalVelocity;

							// Need to check total impulse because the point may be speculative and not colliding
							if ( approachSpeed > @event.approachSpeed && mp.totalNormalImpulse > 0.0f )
							{
								@event.approachSpeed = approachSpeed;
								bestPoint = ref mp;
							}
						}

						B2_VALIDATE( Unsafe.IsNullRef( ref bestPoint ) == false );

						if ( Unsafe.IsNullRef( ref bestPoint ) == false )
						{
							@event.normal = contactSim.manifold.normal;

							ref Shape shapeA = ref shapeArray[contactSim.shapeIdA];
							ref Shape shapeB = ref shapeArray[contactSim.shapeIdB];
							// World contact point reconstructed from a body center of mass and the matching
							// anchor. The anchors were built with the manifold, so a body that has moved since
							// drags the point with it. A static body has not moved, prefer one so the common
							// case of a fast body striking the world stays exact.
							ref Body bodyA = ref world.bodies[shapeA.bodyId];
							ref Body bodyB = ref world.bodies[shapeB.bodyId];
							if ( bodyA.type != BodyType.staticBody && bodyB.type == BodyType.staticBody )
							{
								ref BodySim bodySimB = ref GetBodySim( world, ref bodyB );
								@event.point = OffsetPos( bodySimB.center, bestPoint.anchorB );
							}
							else
							{
								ref BodySim bodySimA = ref GetBodySim( world, ref bodyA );
								@event.point = OffsetPos( bodySimA.center, bestPoint.anchorA );
							}

							@event.shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = worldId, generation = shapeA.generation };
							@event.shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = worldId, generation = shapeB.generation };

							@event.contactId = new ContactId { index1 = contact.contactId + 1, world0 = worldId, padding = 0, generation = contact.generation };

							world.contactHitEvents.Push( @event );
						}

						// Clear the smallest set bit
						word = word & ( word - 1 );
					}
				}
			}

			world.profile.hitEvents = GetMilliseconds( hitTicks );
		}

		{
			ulong refitTicks = GetTicks();

			BroadPhase bp = world.broadPhase;
			DynamicTree_Refit( bp.trees[BodyType.kinematicBody] );
			DynamicTree_Refit( bp.trees[BodyType.dynamicBody] );

			ValidateBroadphase( world.broadPhase );

			world.profile.refit = GetMilliseconds( refitTicks );
		}

		// Bullets are processed after the broad-phase refit so they can query the
		// final non-bullet world.
		int bulletBodyCount = AtomicLoadInt( ref stepContext.bulletBodyCount );
		if ( bulletBodyCount > 0 )
		{
			ulong bulletTicks = GetTicks();

			// Fast bullet bodies
			// Note: a bullet body may be moving slow
			int minRange = 8;
			ParallelFor( world, BulletBodyTask, bulletBodyCount, minRange, stepContext );

			BroadPhase broadPhase = world.broadPhase;
			DynamicTree dynamicTree = broadPhase.trees[BodyType.dynamicBody];
			// Fast array access is important here
			Span<Body> bodyArray = world.bodies.data;
			Span<BodySim> bodySimArray = awakeSet.bodySims.data;
			Span<Shape> shapeArray = world.shapes.data;
			Span<int> bulletBodySimIndices = stepContext.bulletBodies;
			// Serially enlarge broad-phase proxies for bullet shapes.
			// This loop has non-deterministic order but it shouldn't affect the result.
			for ( int i = 0; i < bulletBodyCount; ++i )
			{
				ref BodySim bulletBodySim = ref bodySimArray[bulletBodySimIndices[i]];
				// It is worth tracking this flag because bullets may be moving slowly.
				if ( ( bulletBodySim.flags & b2_enlargeBulletBounds ) == 0 )
				{
					continue;
				}

				// Clear flag
				bulletBodySim.flags &= ~b2_enlargeBulletBounds;

				int bodyId = bulletBodySim.bodyId;
				B2_ASSERT( 0 <= bodyId && bodyId < world.bodies.count );
				ref Body bulletBody = ref bodyArray[bodyId];
				int shapeId = bulletBody.headShapeId;
				while ( shapeId != NullIndex )
				{
					ref Shape shape = ref shapeArray[shapeId];
					int proxyKey = shape.proxyKey;
					int proxyId = B2_PROXY_ID( proxyKey );
					B2_VALIDATE( B2_PROXY_TYPE( proxyKey ) == BodyType.dynamicBody );

					AABB treeAABB = DynamicTree_GetProxyBounds( dynamicTree, proxyId );
					AABB shapeFatAABB = world.fatAABBs.data[shapeId];

					// Double check containment because a bullet can have multiple
					// shapes and maybe just one needs an update.
					if ( AABB_Contains( treeAABB, shapeFatAABB ) == false )
					{
						DynamicTree_EnlargeProxy( dynamicTree, proxyId, shapeFatAABB );
					}

					shapeId = shape.nextShapeId;
				}
			}

			world.profile.bullets = GetMilliseconds( bulletTicks );
		}

		// Need to free this even if no bullets got processed.
		world.stack.Free( stepContext.bulletBodies );
		stepContext.bulletBodies = default;
		AtomicStoreInt( ref stepContext.bulletBodyCount, 0 );

		// Report sensor hits. This may include bullets sensor hits.
		{
			ulong sensorHitTicks = GetTicks();

			int workerCount = world.workerCount;
			B2_ASSERT( workerCount == world.taskContexts.count );

			for ( int i = 0; i < workerCount; ++i )
			{
				ref TaskContext taskContext = ref world.taskContexts.data[i];
				int hitCount = taskContext.sensorHits.count;
				Span<SensorHit> hits = taskContext.sensorHits.data;
				for ( int j = 0; j < hitCount; ++j )
				{
					SensorHit hit = hits[j];
					ref Shape sensorShape = ref world.shapes[hit.sensorId];
					ref Shape visitor = ref world.shapes[hit.visitorId];
					ref Sensor sensor = ref world.sensors[sensorShape.sensorIndex];
					Visitor shapeRef = new Visitor { shapeId = hit.visitorId, generation = visitor.generation };
					sensor.hits.Push( shapeRef );
				}
			}

			world.profile.sensorHits = GetMilliseconds( sensorHitTicks );
		}

		// Island sleeping
		// This must be done last because putting islands to sleep invalidates the enlarged body bits.
		// todo_erin figure out how to do this in parallel with tree refit
		if ( world.enableSleep == true )
		{
			ulong sleepTicks = GetTicks();

			// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
			B2_ASSERT( world.splitIslandId == NullIndex );
			float splitSleepTimer = 0.0f;
			for ( int i = 0; i < world.workerCount; ++i )
			{
				ref TaskContext taskContext = ref world.taskContexts.data[i];
				if ( taskContext.splitIslandId != NullIndex && taskContext.splitSleepTime >= splitSleepTimer )
				{
					B2_ASSERT( taskContext.splitSleepTime > 0.0f );

					// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
					if ( taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId )
					{
						continue;
					}

					world.splitIslandId = taskContext.splitIslandId;
					splitSleepTimer = taskContext.splitSleepTime;
				}
			}

			ref BitSet awakeIslandBitSet = ref world.taskContexts.data[0].awakeIslandBitSet;
			for ( int i = 1; i < world.workerCount; ++i )
			{
				InPlaceUnion( ref awakeIslandBitSet, ref world.taskContexts.data[i].awakeIslandBitSet );
			}

			// Need to process in reverse because this moves islands to sleeping solver sets.
			Span<IslandSim> islands = awakeSet.islandSims.data;
			int count = awakeSet.islandSims.count;
			for ( int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1 )
			{
				if ( GetBit( awakeIslandBitSet, islandIndex ) == true )
				{
					// this island is still awake
					continue;
				}

				ref IslandSim island = ref islands[islandIndex];
				int islandId = island.islandId;

				TrySleepIsland( world, islandId );
			}

			ValidateSolverSets( world );

			world.profile.sleepIslands = GetMilliseconds( sleepTicks );
		}
	}

}
