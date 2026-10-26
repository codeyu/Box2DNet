// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
#if false
	// #define _CRT_SECURE_NO_WARNINGS
#endif





	private static readonly World[] worlds = MakeWorlds();

	private static World[] MakeWorlds()
	{
		World[] w = new World[MaxWorlds];
		for ( int i = 0; i < MaxWorlds; ++i )
		{
			w[i] = new World();
		}

		return w;
	}

	private static World GetUnlockedWorldFromId( WorldId id )
	{
		B2_ASSERT( 1 <= id.index1 && id.index1 <= MaxWorlds );
		World world = worlds[id.index1 - 1];
		B2_ASSERT( id.index1 == world.worldId + 1 );
		B2_ASSERT( id.generation == world.generation );

		// A world accessed from an id should not be locked
		if ( world.locked )
		{
			B2_ASSERT( false );
			return null;
		}
		return world;
	}

	internal static World GetWorldFromId( WorldId id )
	{
		B2_ASSERT( 1 <= id.index1 && id.index1 <= MaxWorlds );
		World world = worlds[id.index1 - 1];
		B2_ASSERT( id.index1 == world.worldId + 1 );
		B2_ASSERT( id.generation == world.generation );
		return world;
	}

	internal static World GetWorld( int index )
	{
		B2_ASSERT( 0 <= index && index < MaxWorlds );
		World world = worlds[index];
		B2_ASSERT( world.worldId == index );
		return world;
	}

	internal static World GetWorldLocked( int index )
	{
		B2_ASSERT( 0 <= index && index < MaxWorlds );
		World world = worlds[index];
		B2_ASSERT( world.worldId == index );
		if ( world.locked )
		{
			B2_ASSERT( false );
			return null;
		}

		return world;
	}

	private static object? DefaultAddTaskFcn( TaskCallbackFcn task, object? taskContext, object? userContext )
	{
		task( taskContext );
		return null;
	}

	private static void DefaultFinishTaskFcn( object? userTask, object? userContext )
	{
	}

	private static void CreateWorkerContexts( World world )
	{
		world.taskContexts = default;
		world.taskContexts.ResizeAndSetZero( world.workerCount );

		world.sensorTaskContexts = default;
		world.sensorTaskContexts.ResizeAndSetZero( world.workerCount );

		for ( int i = 0; i < world.workerCount; ++i )
		{
			world.taskContexts.data[i].sensorHits.CreateN( 8 );
			world.taskContexts.data[i].pairKeys = default;
			world.taskContexts.data[i].contactStateBitSet = CreateBitSet( 1024 );
			world.taskContexts.data[i].hitEventBitSet = CreateBitSet( 1024 );
			world.taskContexts.data[i].hasHitEvents = false;
			world.taskContexts.data[i].jointStateBitSet = CreateBitSet( 1024 );
			world.taskContexts.data[i].enlargedSimBitSet = CreateBitSet( 256 );
			world.taskContexts.data[i].awakeIslandBitSet = CreateBitSet( 256 );
			world.taskContexts.data[i].splitIslandId = NullIndex;

			world.sensorTaskContexts.data[i].eventBits = CreateBitSet( 128 );
		}
	}

	private static void DestroyWorkerContexts( World world )
	{
		for ( int i = 0; i < world.workerCount; ++i )
		{
			world.taskContexts.data[i].sensorHits.Destroy();
			world.taskContexts.data[i].pairKeys.Destroy();
			DestroyBitSet( ref world.taskContexts.data[i].contactStateBitSet );
			DestroyBitSet( ref world.taskContexts.data[i].hitEventBitSet );
			DestroyBitSet( ref world.taskContexts.data[i].jointStateBitSet );
			DestroyBitSet( ref world.taskContexts.data[i].enlargedSimBitSet );
			DestroyBitSet( ref world.taskContexts.data[i].awakeIslandBitSet );

			DestroyBitSet( ref world.sensorTaskContexts.data[i].eventBits );
		}

		world.taskContexts.Destroy();
		world.sensorTaskContexts.Destroy();
	}

	public static WorldId CreateWorld( in WorldDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidVec2( def.gravity ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.restitutionThreshold ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( def.restitutionIterations >= 0 ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.hitEventThreshold ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.contactHertz ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.contactDampingRatio ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.contactSpeed ) ) == false )
			{
				return default(WorldId);
			};
		if ( CheckInput( IsValidFloat( def.maximumLinearSpeed ) && def.maximumLinearSpeed > 0.0f ) == false )
			{
				return default(WorldId);
			};

		if ( IsDenormalFlushEnabled() )
		{
			Log( "Denormal flushing is enabled, determinism not supported" );
		}

		int worldId = NullIndex;
		for ( int i = 0; i < MaxWorlds; ++i )
		{
			if ( worlds[i].inUse == false )
			{
				worldId = i;
				break;
			}
		}

		if ( worldId == NullIndex )
		{
			return default(WorldId);
		}

		ushort generation = worlds[worldId].generation;

		World world = worlds[worldId] = new World();

		world.worldId = (ushort)worldId;
		world.generation = generation;
		world.inUse = true;

		world.stack = CreateStack( MaxInt( 2048, def.capacity.arenaByteCount ) );
		CreateBroadPhase( world.broadPhase, def.capacity );
		CreateGraph( world.constraintGraph, def.capacity );

		// pools
		world.bodyIdPool = CreateIdPool();

		int bodyCapacity = MaxInt( 16, def.capacity.staticBodyCount + def.capacity.dynamicBodyCount );
		world.bodies.CreateN( bodyCapacity );
		world.solverSets.CreateN( 8 );

		// add empty static, active, and disabled body sets
		world.solverSetIdPool = CreateIdPool();
		SolverSet set = default;

		// static set
		set.setIndex = AllocId( ref world.solverSetIdPool );
		world.solverSets.Push( set );
		world.solverSets.data[b2_staticSet].bodySims.Reserve( MaxInt( 16, def.capacity.staticBodyCount ) );
		B2_ASSERT( world.solverSets.data[b2_staticSet].setIndex == b2_staticSet );

		// disabled set
		set.setIndex = AllocId( ref world.solverSetIdPool );
		world.solverSets.Push( set );
		B2_ASSERT( world.solverSets.data[b2_disabledSet].setIndex == b2_disabledSet );

		// awake set
		set.setIndex = AllocId( ref world.solverSetIdPool );
		world.solverSets.Push( set );
		world.solverSets.data[b2_awakeSet].bodySims.Reserve( MaxInt( 16, def.capacity.dynamicBodyCount ) );
		world.solverSets.data[b2_awakeSet].bodyStates.Reserve( MaxInt( 16, def.capacity.dynamicBodyCount ) );
		world.solverSets.data[b2_awakeSet].contactSims.Reserve( MaxInt( 16, def.capacity.contactCount ) );
		B2_ASSERT( world.solverSets.data[b2_awakeSet].setIndex == b2_awakeSet );

		world.shapeIdPool = CreateIdPool();

		int shapeCapacity = MaxInt( 16, def.capacity.staticShapeCount + def.capacity.dynamicShapeCount );
		world.shapes.CreateN( shapeCapacity );
		world.fatAABBs.CreateN( shapeCapacity );

		world.chainIdPool = CreateIdPool();
		world.chainShapes.CreateN( 4 );

		world.contactIdPool = CreateIdPool();
		world.contacts.CreateN( MaxInt( 16, def.capacity.contactCount ) );

		world.jointIdPool = CreateIdPool();
		world.joints.CreateN( 16 );

		world.islandIdPool = CreateIdPool();
		world.islands.CreateN( MaxInt( 16, def.capacity.dynamicBodyCount ) );

		world.sensors.CreateN( 4 );

		world.bodyMoveEvents.CreateN( 4 );
		world.sensorBeginEvents.CreateN( 4 );
		world.sensorEndEvents[0].CreateN( 4 );
		world.sensorEndEvents[1].CreateN( 4 );
		world.contactBeginEvents.CreateN( 4 );
		world.contactEndEvents[0].CreateN( 4 );
		world.contactEndEvents[1].CreateN( 4 );
		world.contactHitEvents.CreateN( 4 );
		world.jointEvents.CreateN( 4 );
		world.endEventArrayIndex = 0;

		world.stepIndex = 0;
		world.splitIslandId = NullIndex;
		world.activeTaskCount = 0;
		world.taskCount = 0;
		world.gravity = def.gravity;
		world.hitEventThreshold = ClampFloat( def.hitEventThreshold, 0.0f, FLT_MAX );
		world.restitutionThreshold = ClampFloat( def.restitutionThreshold, 0.0f, FLT_MAX );
		// Clamp this to avoid overflowing the solver sync flags.
		world.restitutionIterations = ClampInt( def.restitutionIterations, 0, MaxRestitutionIterations );
		world.enableRestitutionPropagation = def.enableRestitutionPropagation;
		world.maxLinearSpeed = def.maximumLinearSpeed;
		world.contactSpeed = ClampFloat( def.contactSpeed, 0.0f, FLT_MAX );
		world.contactHertz = ClampFloat( def.contactHertz, 0.0f, FLT_MAX );
		world.contactDampingRatio = ClampFloat( def.contactDampingRatio, 0.0f, FLT_MAX );
		world.contactRecycleDistance = ContactRecycleDistance;

		if ( def.frictionCallback != null )
		{
			world.frictionCallback = def.frictionCallback;
		}

		if ( def.restitutionCallback != null )
		{
			world.restitutionCallback = def.restitutionCallback;
		}

		world.enableSleep = def.enableSleep;
		world.locked = false;
		world.enableWarmStarting = true;
		world.enableContinuous = def.enableContinuous;
		world.simdWidth = GetSIMDWidth();
		world.userTreeTask = null;
		world.userData = def.userData;

		if ( def.workerCount > 0 && def.enqueueTask != null && def.finishTask != null )
		{
			// External task system
			world.workerCount = MinInt( def.workerCount, MaxWorkers );
			world.enqueueTaskFcn = def.enqueueTask;
			world.finishTaskFcn = def.finishTask;
			world.userTaskContext = def.userTaskContext;
			world.scheduler = default;
		}
		else if ( def.workerCount > 1 )
		{
			// Built-in scheduler
			world.workerCount = MinInt( def.workerCount, MaxWorkers );
			world.scheduler = CreateScheduler( world.workerCount );
			world.enqueueTaskFcn = SchedulerEnqueueTask;
			world.finishTaskFcn = SchedulerFinishTask;
			world.userTaskContext = world.scheduler;
		}
		else
		{
			// Serial fallback
			world.workerCount = 1;
			world.enqueueTaskFcn = DefaultAddTaskFcn;
			world.finishTaskFcn = DefaultFinishTaskFcn;
			world.userTaskContext = null;
			world.scheduler = default;
		}

		CreateWorkerContexts( world );

		world.debugBodySet = CreateBitSet( 256 );
		world.debugJointSet = CreateBitSet( 256 );
		world.debugContactSet = CreateBitSet( 256 );
		world.debugIslandSet = CreateBitSet( 256 );

		// Recording is started by the host with World_StartRecording, never from the world def
		world.recording = null;

		// add one to worldId so that 0 represents a null WorldId
		return new WorldId { index1 = (ushort)( worldId + 1 ), generation = world.generation };
	}

	public static void DestroyWorld( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		// Detach any recording before teardown. The user owns and frees the recording buffer.
		if ( world.scheduler != null )
		{
			DestroyScheduler( world.scheduler );
			world.scheduler = default;
		}

		DestroyBitSet( ref world.debugBodySet );
		DestroyBitSet( ref world.debugJointSet );
		DestroyBitSet( ref world.debugContactSet );
		DestroyBitSet( ref world.debugIslandSet );

		DestroyWorkerContexts( world );

		world.bodyMoveEvents.Destroy();
		world.sensorBeginEvents.Destroy();
		world.sensorEndEvents[0].Destroy();
		world.sensorEndEvents[1].Destroy();
		world.contactBeginEvents.Destroy();
		world.contactEndEvents[0].Destroy();
		world.contactEndEvents[1].Destroy();
		world.contactHitEvents.Destroy();
		world.jointEvents.Destroy();

		int chainCapacity = world.chainShapes.count;
		for ( int i = 0; i < chainCapacity; ++i )
		{
			ref ChainShape chain = ref world.chainShapes.data[i];
			if ( chain.id != NullIndex )
			{
				FreeChainData( ref chain );
			}
			else
			{
				B2_ASSERT( chain.shapeIndices == null );
			}
		}

		int sensorCount = world.sensors.count;
		for ( int i = 0; i < sensorCount; ++i )
		{
			world.sensors.data[i].hits.Destroy();
			world.sensors.data[i].overlaps1.Destroy();
			world.sensors.data[i].overlaps2.Destroy();
		}

		world.sensors.Destroy();

		world.bodies.Destroy();
		world.shapes.Destroy();
		world.fatAABBs.Destroy();
		world.chainShapes.Destroy();
		world.contacts.Destroy();
		world.joints.Destroy();

		for ( int i = 0; i < world.islands.count; ++i )
		{
			world.islands.data[i].bodies.Destroy();
			world.islands.data[i].contacts.Destroy();
			world.islands.data[i].joints.Destroy();
		}
		world.islands.Destroy();

		// Destroy solver sets
		int setCapacity = world.solverSets.count;
		for ( int i = 0; i < setCapacity; ++i )
		{
			ref SolverSet set = ref world.solverSets.data[i];
			if ( set.setIndex != NullIndex )
			{
				DestroySolverSet( world, i );
			}
		}

		world.solverSets.Destroy();

		DestroyGraph( world.constraintGraph );
		DestroyBroadPhase( world.broadPhase );

		DestroyIdPool( ref world.bodyIdPool );
		DestroyIdPool( ref world.shapeIdPool );
		DestroyIdPool( ref world.chainIdPool );
		DestroyIdPool( ref world.contactIdPool );
		DestroyIdPool( ref world.jointIdPool );
		DestroyIdPool( ref world.islandIdPool );
		DestroyIdPool( ref world.solverSetIdPool );

		DestroyStack( world.stack );

		// Wipe world but preserve generation
		ushort generation = world.generation;
		world = worlds[world.worldId] = new World();
		world.worldId = 0;
		world.generation = (ushort)( generation + 1 );
	}

	internal static float RelativeCos( Rot a, Rot b )
	{
		return a.c * b.c + a.s * b.s;
	}

	internal static ref BodySim ResolveContactBodySim( World world, Span<BodySim> awakeSims, Span<BodySim> staticSims, int encodedIndex, int bodyId )
	{
		if ( encodedIndex >= 0 )
		{
			return ref awakeSims[encodedIndex];
		}

		if ( IsStaticSimIndex( encodedIndex ) )
		{
			return ref staticSims[-encodedIndex - 2];
		}

		return ref GetBodySim( world, ref world.bodies[bodyId] );
	}

	private static void CollideTask( int startIndex, int endIndex, int workerIndex, object? context )
	{
		StepContext stepContext = (StepContext)context!;
		World world = stepContext.world;
		ref TaskContext taskContext = ref world.taskContexts.data[workerIndex];
		ReadOnlySpan<ContactCollideSpan> spans = stepContext.collideSpans;
		Span<Shape> shapes = world.shapes.data;
		ReadOnlySpan<AABB> fatAABBs = world.fatAABBs.data;
		Span<SolverSet> solverSets = world.solverSets.data;
		Span<BodySim> awakeSims = solverSets[b2_awakeSet].bodySims.data;
		Span<BodySim> staticSims = solverSets[b2_staticSet].bodySims.data;
		B2_ASSERT( startIndex < endIndex );

		int spanIndex = 0;
		while ( spans[spanIndex + 1].start <= startIndex )
		{
			spanIndex += 1;
		}

		int spanStart = spans[spanIndex].start;
		int spanEnd = spans[spanIndex + 1].start;
		Span<ContactSim> spanBase = spans[spanIndex].contacts;
		float recycleDistance = world.contactRecycleDistance;
		float speculativeDistance = SpeculativeDistance;
		float recycleDistanceNonTouching = MinFloat( recycleDistance, speculativeDistance );

		for ( int contactIndex = startIndex; contactIndex < endIndex; ++contactIndex )
		{
			if ( contactIndex == spanEnd )
			{
				spanIndex += 1;
				spanStart = spans[spanIndex].start;
				spanEnd = spans[spanIndex + 1].start;
				spanBase = spans[spanIndex].contacts;
			}

			ref ContactSim contactSim = ref spanBase[( contactIndex - spanStart )];
			int contactId = contactSim.contactId;
			int shapeIdA = contactSim.shapeIdA;
			int shapeIdB = contactSim.shapeIdB;

			// Do proxies still overlap?
			bool overlap = OverlapV( fatAABBs[shapeIdA], fatAABBs[shapeIdB] );
			if ( overlap == false )
			{
				contactSim.simFlags |= b2_simDisjoint;
				contactSim.simFlags &= ~b2_simTouchingFlag;
				SetBit( ref taskContext.contactStateBitSet, contactId );
			}
			else
			{
				ref Shape shapeA = ref shapes[shapeIdA];
				ref Shape shapeB = ref shapes[shapeIdB];
				bool wasTouching = ( contactSim.simFlags & b2_simTouchingFlag ) != 0;

				// Update contact respecting shape/body order (A,B)
				int encodedA = contactSim.encodedBodySimA;
				int encodedB = contactSim.encodedBodySimB;
				ref BodySim bodySimA = ref ResolveContactBodySim( world, awakeSims, staticSims, encodedA, shapeA.bodyId );
				ref BodySim bodySimB = ref ResolveContactBodySim( world, awakeSims, staticSims, encodedB, shapeB.bodyId );
				WorldTransform transformA = bodySimA.transform;
				WorldTransform transformB = bodySimB.transform;

				// These may not be skipped by relative transform check below
				contactSim.invMassA = bodySimA.invMass;
				contactSim.invIA = bodySimA.invInertia;
				contactSim.invMassB = bodySimB.invMass;
				contactSim.invIB = bodySimB.invInertia;

				// Contact recycling optimization. Please cite this code if you use this optimization.
				// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
				// However, this allows larger relative motion and has fewer tuning parameters (just one).
				if ( recycleDistance > 0.0f && ( contactSim.simFlags & b2_simRelativeTransformValid ) != 0 &&
					 ( contactSim.simFlags & b2_contactRecycleFlag ) != 0 )
				{
					Rot cachedQA = contactSim.cachedRotationA;
					Rot cachedQB = contactSim.cachedRotationB;
					Transform xfc = contactSim.cachedRelativePose;
					Transform xf = InvMulWorldTransforms( transformA, transformB );

					float cosA = RelativeCos( transformA.q, cachedQA );
					float cosB = RelativeCos( transformB.q, cachedQB );
					float minCos = MinFloat( cosA, cosB );

					float maxExtentA = IsStaticSimIndex( encodedA ) ? 0.0f : bodySimA.maxExtent;
					float maxExtentB = IsStaticSimIndex( encodedB ) ? 0.0f : bodySimB.maxExtent;
					float maxExtent = MaxFloat( maxExtentA, maxExtentB );
					float distance = Distance( xf.p, xfc.p );
					Rot qr = InvMulRot( xf.q, xfc.q );

					// This metric is used for fast bodies and sleeping. It comes from conservative advancement.
					// Note that qr.s == sin(theta) ~= theta for small angles.
					// Need a tighter tolerance for non-touching shapes so that contacts are not missed.
					float tolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

					if ( minCos > ContactRecycleCosAngle && distance + maxExtent * AbsFloat( qr.s ) < tolerance )
					{
						Rot dqA = MulRot( transformA.q, InvertRot( cachedQA ) );
						Rot dqB = MulRot( transformB.q, InvertRot( cachedQB ) );
						Vec2 normal = contactSim.manifold.normal;

						// Minimize round-off
						Vec2 dc = SubPos( bodySimB.center, bodySimA.center );

						for ( int i = 0; i < contactSim.manifold.pointCount; ++i )
						{
							// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
							ref ManifoldPoint mp = ref contactSim.manifold.points[i];
							Vec2 rA = RotateVector( dqA, mp.anchorA );
							Vec2 rB = RotateVector( dqB, mp.anchorB );
							Vec2 dp = Add( dc, Sub( rB, rA ) );

							mp.separation = mp.baseSeparation + Dot( dp, normal );
							mp.normalVelocity = 0.0f;
							mp.persisted = true;
						}

						taskContext.recycledContactCount += 1;

						// Contact is recycled. This also skips updating other aspects of the contact
						// such as material parameters.
						continue;
					}
				}

				// Caching for contact recycling.
				contactSim.cachedRotationA = transformA.q;
				contactSim.cachedRotationB = transformB.q;
				contactSim.cachedRelativePose = InvMulWorldTransforms( transformA, transformB );
				contactSim.simFlags |= b2_simRelativeTransformValid;

				Vec2 centerOffsetA = RotateVector( transformA.q, bodySimA.localCenter );
				Vec2 centerOffsetB = RotateVector( transformB.q, bodySimB.localCenter );

				// This updates solid contacts
				bool touching =
					UpdateContact( world, ref contactSim, ref shapeA, transformA, centerOffsetA, ref shapeB, transformB, centerOffsetB );

				// State changes that affect island connectivity. Also affects contact events.
				if ( touching == true && wasTouching == false )
				{
					contactSim.simFlags |= b2_simStartedTouching;
					SetBit( ref taskContext.contactStateBitSet, contactId );
				}
				else if ( touching == false && wasTouching == true )
				{
					contactSim.simFlags |= b2_simStoppedTouching;
					SetBit( ref taskContext.contactStateBitSet, contactId );
				}

				if ( touching )
				{
					for ( int i = 0; i < contactSim.manifold.pointCount; ++i )
					{
						// Cache separation
						ref ManifoldPoint mp = ref contactSim.manifold.points[i];
						mp.baseSeparation = mp.separation;
					}
				}

				// To make this work, the time of impact code needs to adjust the target
				// distance based on the number of TOI events for a body.
				// if (touching && bodySimB.isFast)
				//{
				//	Manifold* manifold = &contactSim.manifold;
				//	int pointCount = manifold.pointCount;
				//	for (int i = 0; i < pointCount; ++i)
				//	{
				//		// trick the solver into pushing the fast shapes apart
				//		manifold.points[i].separation -= 0.25f * SpeculativeDistance;
				//	}
				//}
			}
		}
	}

	private static void AddNonTouchingContact( World world, ref Contact contact, ref ContactSim contactSim )
	{
		B2_ASSERT( contact.setIndex == b2_awakeSet );
		ref SolverSet set = ref world.solverSets[b2_awakeSet];
		contact.colorIndex = NullIndex;
		contact.localIndex = set.contactSims.count;

		ref ContactSim newContactSim = ref set.contactSims.Emplace();
		newContactSim = contactSim;
	}

	private static void RemoveNonTouchingContact( World world, int setIndex, int localIndex )
	{
		ref SolverSet set = ref world.solverSets[setIndex];
		int movedIndex = set.contactSims.RemoveSwap( localIndex );
		if ( movedIndex != NullIndex )
		{
			ref ContactSim movedContactSim = ref set.contactSims.data[localIndex];
			ref Contact movedContact = ref world.contacts[movedContactSim.contactId];
			B2_ASSERT( movedContact.setIndex == setIndex );
			B2_ASSERT( movedContact.localIndex == movedIndex );
			B2_ASSERT( movedContact.colorIndex == NullIndex );
			movedContact.localIndex = localIndex;
		}
	}

	// Narrow-phase collision
	private static void Collide( ref StepContext context )
	{
		World world = context.world;
		B2_ASSERT( world.workerCount > 0 );
		// gather contacts into a single array for easier parallel-for
		int contactCount = 0;
		Span<GraphColor> graphColors = world.constraintGraph.colors;
		for ( int i = 0; i < GraphColorCount; ++i )
		{
			contactCount += graphColors[i].contactSims.count;
		}

		int nonTouchingCount = world.solverSets.data[b2_awakeSet].contactSims.count;
		contactCount += nonTouchingCount;

		if ( contactCount == 0 )
		{
			return;
		}

		ContactCollideSpan[] collideSpans = new ContactCollideSpan[GraphColorCount + 2];
		int spanCount = 0;
		int contactIndex = 0;
		for ( int i = 0; i < GraphColorCount; ++i )
		{
			ref GraphColor color = ref graphColors[i];
			int count = color.contactSims.count;
			if ( count > 0 )
			{
				collideSpans[spanCount].start = contactIndex;
				collideSpans[spanCount].contacts = color.contactSims.data;
				spanCount += 1;
				contactIndex += count;
			}
		}

		if ( nonTouchingCount > 0 )
		{
			collideSpans[spanCount].start = contactIndex;
			collideSpans[spanCount].contacts = world.solverSets.data[b2_awakeSet].contactSims.data;
			spanCount += 1;
			contactIndex += nonTouchingCount;
		}

		B2_ASSERT( contactIndex == contactCount );
		B2_ASSERT( spanCount <= GraphColorCount + 1 );

		collideSpans[spanCount].start = contactCount;
		collideSpans[spanCount].contacts = default;

		context.collideSpans = collideSpans;

		// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
		int contactIdCapacity = GetIdCapacity( ref world.contactIdPool );
		for ( int i = 0; i < world.workerCount; ++i )
		{
			SetBitCountAndClear( ref world.taskContexts.data[i].contactStateBitSet, contactIdCapacity );
			world.taskContexts.data[i].recycledContactCount = 0;
		}

		// Task should take at least 40us on a 4GHz CPU (10K cycles)
		int minRange = 64;
		ParallelFor( world, CollideTask, contactCount, minRange, context );

		context.collideSpans = default;

		// Serially update contact state
		// Bitwise OR all contact bits
		ref BitSet bitSet = ref world.taskContexts.data[0].contactStateBitSet;
		for ( int i = 1; i < world.workerCount; ++i )
		{
			InPlaceUnion( ref bitSet, ref world.taskContexts.data[i].contactStateBitSet );
		}

		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		int endEventArrayIndex = world.endEventArrayIndex;

		ReadOnlySpan<Shape> shapes = world.shapes.data;
		ushort worldId = world.worldId;

		// Process contact state changes. Iterate over set bits
		for ( uint k = 0; k < bitSet.blockCount; ++k )
		{
			ulong bits = bitSet.bits[(int)k];
			while ( bits != 0 )
			{
				uint ctz = CTZ64( bits );
				int contactId = (int)( 64 * k + ctz );

				ref Contact contact = ref world.contacts[contactId];
				B2_ASSERT( contact.setIndex == b2_awakeSet );

				int colorIndex = contact.colorIndex;
				int localIndex = contact.localIndex;

				scoped ref ContactSim contactSim = ref Unsafe.NullRef<ContactSim>();
				if ( colorIndex != NullIndex )
				{
					// contact lives in constraint graph
					B2_ASSERT( 0 <= colorIndex && colorIndex < GraphColorCount );
					ref GraphColor color = ref graphColors[colorIndex];
					contactSim = ref color.contactSims[localIndex];
				}
				else
				{
					contactSim = ref awakeSet.contactSims[localIndex];
				}

				uint flags = contact.flags;
				uint simFlags = contactSim.simFlags;

				if ( ( simFlags & b2_simDisjoint ) != 0 )
				{
					// Bounding boxes no longer overlap
					DestroyContact( world, ref contact );
					contact = ref Unsafe.NullRef<Contact>();
					contactSim = ref Unsafe.NullRef<ContactSim>();
				}
				else if ( ( simFlags & b2_simStartedTouching ) != 0 )
				{
					B2_ASSERT( contact.islandId == NullIndex );

					if ( ( flags & b2_contactEnableContactEvents ) != 0 )
					{
						ref readonly Shape shapeA = ref shapes[contact.shapeIdA];
						ref readonly Shape shapeB = ref shapes[contact.shapeIdB];
						ShapeId shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = worldId, generation = shapeA.generation };
						ShapeId shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = worldId, generation = shapeB.generation };
						ContactId contactFullId = new ContactId { index1 = contactId + 1, world0 = worldId, padding = 0, generation = contact.generation };

						ContactBeginTouchEvent @event = new ContactBeginTouchEvent { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactFullId };
						world.contactBeginEvents.Push( @event );
					}

					B2_ASSERT( contactSim.manifold.pointCount > 0 );
					B2_ASSERT( contact.setIndex == b2_awakeSet );

					// Link first because this wakes colliding bodies and ensures the body sims
					// are in the correct place.
					contact.flags |= b2_contactTouchingFlag;
					LinkContact( world, ref contact );

					// Make sure these didn't change
					B2_ASSERT( contact.colorIndex == NullIndex );
					B2_ASSERT( contact.localIndex == localIndex );

					// Contact sim pointer may have become orphaned due to awake set growth,
					// so I just need to refresh it.
					contactSim = ref awakeSet.contactSims[localIndex];

					contactSim.simFlags &= ~b2_simStartedTouching;

					// Add first for memcpy
					AddContactToGraph( world, ref contactSim, ref contact );

					// This destroys the contact sim
					RemoveNonTouchingContact( world, b2_awakeSet, localIndex );

					contactSim = ref Unsafe.NullRef<ContactSim>();
				}
				else if ( ( simFlags & b2_simStoppedTouching ) != 0 )
				{
					contactSim.simFlags &= ~b2_simStoppedTouching;
					contact.flags &= ~b2_contactTouchingFlag;

					if ( ( contact.flags & b2_contactEnableContactEvents ) != 0 )
					{
						ref readonly Shape shapeA = ref shapes[contact.shapeIdA];
						ref readonly Shape shapeB = ref shapes[contact.shapeIdB];
						ShapeId shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = worldId, generation = shapeA.generation };
						ShapeId shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = worldId, generation = shapeB.generation };
						ContactId contactFullId = new ContactId { index1 = contactId + 1, world0 = worldId, padding = 0, generation = contact.generation };

						ContactEndTouchEvent @event = new ContactEndTouchEvent { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactFullId };
						world.contactEndEvents[endEventArrayIndex].Push( @event );
					}

					B2_ASSERT( contactSim.manifold.pointCount == 0 );

					UnlinkContact( world, ref contact );
					int bodyIdA = contact.edges[0].bodyId;
					int bodyIdB = contact.edges[1].bodyId;

					// Add first for memcpy
					AddNonTouchingContact( world, ref contact, ref contactSim );
					RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
					contact = ref Unsafe.NullRef<Contact>();
					contactSim = ref Unsafe.NullRef<ContactSim>();
				}

				// Clear the smallest set bit
				bits = bits & ( bits - 1 );
			}
		}

		ValidateSolverSets( world );
		ValidateContacts( world );
	}

	public static void World_Step( WorldId worldId, float timeStep, int subStepCount )
	{
		if ( CheckInput( IsValidFloat( timeStep ) ) == false )
			{
				return;
			};
		B2_ASSERT( 0 < subStepCount );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}

		// Record step inputs before simulation runs

		// Prepare to capture events
		// Ensure user does not access stale data if there is an early return
		world.bodyMoveEvents.Clear();
		world.sensorBeginEvents.Clear();
		world.contactBeginEvents.Clear();
		world.contactHitEvents.Clear();
		world.jointEvents.Clear();

		world.profile = default(Profile);
		world.locked = true;
		world.activeTaskCount = 0;
		world.taskCount = 0;

		if ( world.scheduler != null )
		{
			ResetScheduler( world.scheduler );
		}

		ulong stepTicks = GetTicks();

		{
			ref Capacity c = ref world.maxCapacity;
			c.staticShapeCount = MaxInt( c.staticShapeCount, world.broadPhase.trees[BodyType.staticBody].proxyCount );
			c.dynamicShapeCount = MaxInt( c.dynamicShapeCount, world.broadPhase.trees[BodyType.dynamicBody].proxyCount );

			int staticBodyCount = world.solverSets.data[b2_staticSet].bodySims.count;
			c.staticBodyCount = MaxInt( c.staticBodyCount, staticBodyCount );

			// this includes kinematic bodies
			int totalBodyCount = GetIdCount( ref world.bodyIdPool );
			c.dynamicBodyCount = MaxInt( c.dynamicBodyCount, totalBodyCount - staticBodyCount );

			int totalContactCount = GetIdCount( ref world.contactIdPool );
			c.contactCount = MaxInt( c.contactCount, totalContactCount );
		}

		// Update collision pairs and create contacts
		{
			ulong pairTicks = GetTicks();
			UpdateBroadPhasePairs( world );
			world.profile.pairs = GetMilliseconds( pairTicks );
		}

		StepContext context = new StepContext();
		context.world = world;
		context.dt = timeStep;
		context.subStepCount = MaxInt( 1, subStepCount );

		if ( timeStep > 0.0f )
		{
			context.inv_dt = 1.0f / timeStep;
			context.h = timeStep / context.subStepCount;
			context.inv_h = context.subStepCount * context.inv_dt;
		}
		else
		{
			context.inv_dt = 0.0f;
			context.h = 0.0f;
			context.inv_h = 0.0f;
		}

		world.inv_h = context.inv_h;
		world.inv_dt = context.inv_dt;

		// Hertz values get reduced for large time steps
		float contactHertz = MinFloat( world.contactHertz, 0.125f * context.inv_h );
		context.contactSoftness = MakeSoft( contactHertz, world.contactDampingRatio, context.h );
		context.staticSoftness = MakeSoft( 2.0f * contactHertz, world.contactDampingRatio, context.h );

		context.restitutionThreshold = world.restitutionThreshold;
		context.maxLinearVelocity = world.maxLinearSpeed;
		context.enableWarmStarting = world.enableWarmStarting;

		// Narrow phase : update contacts
		{
			ulong collideTicks = GetTicks();
			Collide( ref context );
			world.profile.collide = GetMilliseconds( collideTicks );
		}

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if ( timeStep > 0.0f )
		{
			ulong solveTicks = GetTicks();
			Solve( world, ref context );
			world.profile.solve = GetMilliseconds( solveTicks );
		}

		// Finish the tree task in case Solve didn't finish it
		if ( world.userTreeTask != null )
		{
			world.finishTaskFcn( world.userTreeTask, world.userTaskContext );
			world.userTreeTask = null;
			world.activeTaskCount -= 1;
			ValidateNoEnlarged( world.broadPhase );
		}

		// Update sensors
		{
			ulong sensorTicks = GetTicks();
			OverlapSensors( world );
			world.profile.sensors = GetMilliseconds( sensorTicks );
		}

		world.profile.step = GetMilliseconds( stepTicks );

		B2_ASSERT( GetStackAllocation( world.stack ) == 0 );

		world.maxCapacity.arenaByteCount = MaxInt( world.maxCapacity.arenaByteCount, GetMaxStackAllocation( world.stack ) );

		// Ensure stack is large enough
		GrowStack( world.stack );

		// Make sure all tasks that were started were also finished
		B2_ASSERT( world.activeTaskCount == 0 );
		// Swap end event array buffers
		world.endEventArrayIndex = 1 - world.endEventArrayIndex;
		world.sensorEndEvents[world.endEventArrayIndex].Clear();
		world.contactEndEvents[world.endEventArrayIndex].Clear();


		world.locked = false;

	}

	private static void DrawShape( ref DebugDraw draw, ref Shape shape, WorldTransform transform, HexColor color, bool drawChainNormals )
	{
		switch ( shape.type )
		{
			case ShapeType.capsuleShape:
			{
				ref Capsule capsule = ref shape.capsule;
				Pos p1 = TransformWorldPoint( transform, capsule.center1 );
				Pos p2 = TransformWorldPoint( transform, capsule.center2 );
				draw.DrawSolidCapsuleFcn( p1, p2, capsule.radius, color, draw.context );
			}
			break;

			case ShapeType.circleShape:
			{
				ref Circle circle = ref shape.circle;
				draw.DrawSolidCircleFcn( transform, circle.center, circle.radius, color, draw.context );
			}
			break;

			case ShapeType.polygonShape:
			{
				ref Polygon poly = ref shape.polygon;
				draw.DrawSolidPolygonFcn( transform, poly.vertices, poly.count, poly.radius, color, draw.context );
			}
			break;

			case ShapeType.segmentShape:
			{
				ref Segment segment = ref shape.segment;
				Pos p1 = TransformWorldPoint( transform, segment.point1 );
				Pos p2 = TransformWorldPoint( transform, segment.point2 );
				draw.DrawLineFcn( p1, p2, color, draw.context );
			}
			break;

			case ShapeType.chainSegmentShape:
			{
				ref Segment segment = ref shape.chainSegment.segment;
				Pos p1 = TransformWorldPoint( transform, segment.point1 );
				Pos p2 = TransformWorldPoint( transform, segment.point2 );
				draw.DrawLineFcn( p1, p2, color, draw.context );
				draw.DrawPointFcn( p2, 4.0f, color, draw.context );

				if ( drawChainNormals )
				{
					Pos c = LerpPos( p1, p2, 0.5f );
					Vec2 e = Normalize( SubPos( p2, p1 ) );
					Vec2 n = RightPerp( e );
					float L = 0.2f * GetLengthUnitsPerMeter();
					draw.DrawLineFcn( c, OffsetPos( c, MulSV( L, n ) ), HexColor.colorPaleGreen, draw.context );
				}
			}
			break;

			default:
				break;
		}
	}

	internal ref struct DrawContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => DrawQueryCallback( proxyId, userData, ref this );

		public World world;
		public ref DebugDraw draw;
	}

	private static bool DrawQueryCallback( int proxyId, ulong userData, ref DrawContext drawContext )
	{
		int shapeId = (int)userData;
		World world = drawContext.world;
		ref DebugDraw draw = ref drawContext.draw;
		ref Shape shape = ref world.shapes[shapeId];
		B2_ASSERT( shape.id == shapeId );
		ref Body body = ref world.bodies[shape.bodyId];
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		SetBit( ref world.debugBodySet, shape.bodyId );

		if ( draw.drawShapes )
		{
			HexColor color = default;
			if ( shape.material.customColor != 0 )
			{
				color = (HexColor)shape.material.customColor;
			}
			else if ( body.type == BodyType.dynamicBody && body.mass == 0.0f )
			{
				// Bad body
				color = HexColor.colorRed;
			}
			else if ( body.setIndex == b2_disabledSet )
			{
				color = HexColor.colorSlateGray;
			}
			else if ( shape.sensorIndex != NullIndex )
			{
				color = HexColor.colorWheat;
			}
			else if ( ( bodySim.flags & b2_hadTimeOfImpact ) != 0 )
			{
				color = HexColor.colorLime;
			}
			else if ( ( bodySim.flags & b2_isBullet ) != 0 && body.setIndex == b2_awakeSet )
			{
				color = HexColor.colorTurquoise;
			}
			else if ( ( body.flags & b2_isSpeedCapped ) != 0 )
			{
				color = HexColor.colorYellow;
			}
			else if ( ( bodySim.flags & b2_isFast ) != 0 )
			{
				color = HexColor.colorSalmon;
			}
			else if ( body.type == BodyType.staticBody )
			{
				color = HexColor.colorPaleGreen;
			}
			else if ( body.type == BodyType.kinematicBody )
			{
				color = HexColor.colorRoyalBlue;
			}
			else if ( body.setIndex == b2_awakeSet )
			{
				color = HexColor.colorPink;
			}
			else
			{
				color = HexColor.colorGray;
			}

			DrawShape( ref draw, ref shape, bodySim.transform, color, draw.drawChainNormals );
		}

		if ( draw.drawBounds )
		{
			draw.DrawBoundsFcn( world.fatAABBs.data[shapeId], HexColor.colorGold, draw.context );
		}

		return true;
	}

	// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
	// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
	public static void World_Draw( WorldId worldId, ref DebugDraw draw )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}

		B2_ASSERT( IsValidAABB( draw.drawingBounds ) );

		const float k_axisScale = 0.3f;
		HexColor speculativeColor = HexColor.colorGainsboro;
		HexColor addColor = HexColor.colorGreen;
		HexColor persistColor = HexColor.colorBlue;
		HexColor normalColor = HexColor.colorDimGray;
		HexColor impulseColor = HexColor.colorMagenta;
		HexColor frictionColor = HexColor.colorYellow;

		int bodyCapacity = GetIdCapacity( ref world.bodyIdPool );
		SetBitCountAndClear( ref world.debugBodySet, bodyCapacity );

		int jointCapacity = GetIdCapacity( ref world.jointIdPool );
		SetBitCountAndClear( ref world.debugJointSet, jointCapacity );

		int contactCapacity = GetIdCapacity( ref world.contactIdPool );
		SetBitCountAndClear( ref world.debugContactSet, contactCapacity );

		int islandCapacity = GetIdCapacity( ref world.islandIdPool );
		SetBitCountAndClear( ref world.debugIslandSet, islandCapacity );

		DrawContext drawContext = new DrawContext { world = world, draw = ref draw };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			DynamicTree_OverlapAABBAll( world.broadPhase.trees[i], draw.drawingBounds, ref drawContext );
		}

		uint wordCount = world.debugBodySet.blockCount;
		Span<ulong> bits = world.debugBodySet.bits;
		for ( uint k = 0; k < wordCount; ++k )
		{
			ulong word = bits[(int)k];
			while ( word != 0 )
			{
				uint ctz = CTZ64( word );
				uint bodyId = 64 * k + ctz;

				ref Body body = ref world.bodies[(int)bodyId];
				if ( draw.drawBodyNames && body.name[0] != 0 )
				{
					Vec2 offset = new Vec2( 0.1f, 0.1f );
					ref BodySim bodySim = ref GetBodySim( world, ref body );
					WorldTransform transform = new WorldTransform { p = bodySim.center, q = bodySim.transform.q };
					Pos p = TransformWorldPoint( transform, offset );
					draw.DrawStringFcn( p, body.name, HexColor.colorWhiteSmoke, draw.context );
				}

				if ( draw.drawMass && body.type == BodyType.dynamicBody )
				{
					Vec2 offset = new Vec2( 0.1f, 0.1f );
					ref BodySim bodySim = ref GetBodySim( world, ref body );
					WorldTransform transform = new WorldTransform { p = bodySim.center, q = bodySim.transform.q };
					draw.DrawLineFcn( bodySim.center0, bodySim.center, HexColor.colorWhiteSmoke, draw.context );
					draw.DrawTransformFcn( transform, draw.context );

					Pos p = TransformWorldPoint( transform, offset );
					string buffer = $"  {body.mass:F2}";
					draw.DrawStringFcn( p, buffer, HexColor.colorWhite, draw.context );
				}

				if ( draw.drawJoints )
				{
					int jointKey = body.headJointKey;
					while ( jointKey != NullIndex )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;
						ref Joint joint = ref world.joints[jointId];
						// avoid double draw
						if ( GetBit( ref world.debugJointSet, jointId ) == false )
						{
							DrawJoint( ref draw, world, ref joint );
							SetBit( ref world.debugJointSet, jointId );
						}

						jointKey = joint.edges[edgeIndex].nextKey;
					}
				}

				float linearSlop = LinearSlop;
				if ( draw.drawContacts && body.type == BodyType.dynamicBody )
				{
					int contactKey = body.headContactKey;
					while ( contactKey != NullIndex )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;
						ref Contact contact = ref world.contacts[contactId];
						contactKey = contact.edges[edgeIndex].nextKey;

						// avoid double draw
						if ( GetBit( ref world.debugContactSet, contactId ) == false )
						{
							ref ContactSim contactSim = ref GetContactSim( world, ref contact );
							ref Body bodyA = ref world.bodies[contact.edges[0].bodyId];
							ref BodySim bodySimA = ref GetBodySim( world, ref bodyA );
							ref Body bodyB = ref world.bodies[contact.edges[1].bodyId];
							ref BodySim bodySimB = ref GetBodySim( world, ref bodyB );
							int pointCount = contactSim.manifold.pointCount;
							Vec2 normal = contactSim.manifold.normal;
							string buffer = "";

							for ( int j = 0; j < pointCount; ++j )
							{
								ref ManifoldPoint mp = ref contactSim.manifold.points[j];
								Pos p = default;
								if ( draw.drawAnchorA )
								{
									p = OffsetPos( bodySimA.center, mp.anchorA );
								}
								else
								{
									p = OffsetPos( bodySimB.center, mp.anchorB );
								}

								if ( draw.drawGraphColors && contact.colorIndex != NullIndex )
								{
									// graph color
									float pointSize = contact.colorIndex == OverflowIndex ? 7.5f : 5.0f;
									draw.DrawPointFcn( p, pointSize, GetGraphColor( contact.colorIndex ), draw.context );
									// m_context.draw.DrawString(point.position, "%d", point.color);
								}
								else if ( mp.separation > linearSlop )
								{
									// Speculative
									draw.DrawPointFcn( p, 5.0f, speculativeColor, draw.context );
								}
								else if ( mp.persisted == false )
								{
									// Add
									draw.DrawPointFcn( p, 10.0f, addColor, draw.context );
								}
								else if ( mp.persisted == true )
								{
									// Persist
									draw.DrawPointFcn( p, 5.0f, persistColor, draw.context );
								}

								if ( draw.drawContactNormals )
								{
									Pos p1 = p;
									Pos p2 = OffsetPos( p1, MulSV( k_axisScale, normal ) );
									draw.DrawLineFcn( p1, p2, normalColor, draw.context );

									buffer = $" {mp.separation:F2}";
									draw.DrawStringFcn( p1, buffer, HexColor.colorWhite, draw.context );
								}
								else if ( draw.drawContactForces )
								{
									// todo validate
									// multiply by one-half due to relax iteration
									float force = 0.5f * mp.totalNormalImpulse * world.inv_dt;
									Pos p1 = p;
									Pos p2 = OffsetPos( p1, MulSV( draw.forceScale * force, normal ) );
									draw.DrawLineFcn( p1, p2, impulseColor, draw.context );
									buffer = $"{force:F1}";
									draw.DrawStringFcn( p1, buffer, HexColor.colorWhite, draw.context );
								}

								if ( draw.drawContactFeatures )
								{
									buffer = $"{mp.id}";
									draw.DrawStringFcn( p, buffer, HexColor.colorOrange, draw.context );
								}

								if ( draw.drawFrictionForces )
								{
									float force = 0.5f * mp.tangentImpulse * world.inv_h;
									Vec2 tangent = RightPerp( normal );
									Pos p1 = p;
									Pos p2 = OffsetPos( p1, MulSV( draw.forceScale * force, tangent ) );
									draw.DrawLineFcn( p1, p2, frictionColor, draw.context );
									buffer = $"{force:F1}";
									draw.DrawStringFcn( p1, buffer, HexColor.colorWhite, draw.context );
								}
							}

							SetBit( ref world.debugContactSet, contactId );
						}

						contactKey = contact.edges[edgeIndex].nextKey;
					}
				}

				if ( draw.drawIslands )
				{
					int islandId = body.islandId;
					if ( islandId != NullIndex && GetBit( ref world.debugIslandSet, islandId ) == false )
					{
						ref Island island = ref world.islands.data[islandId];
						if ( island.setIndex == NullIndex )
						{
							continue;
						}

						int shapeCount = 0;
						AABB aabb = new AABB { lowerBound = new( FLT_MAX, FLT_MAX ), upperBound = new( -FLT_MAX, -FLT_MAX ) };

						for ( int bodyIndex = 0; bodyIndex < island.bodies.count; ++bodyIndex )
						{
							int islandBodyId = island.bodies.data[bodyIndex];
							ref Body islandBody = ref world.bodies[islandBodyId];
							int shapeId = islandBody.headShapeId;
							while ( shapeId != NullIndex )
							{
								ref Shape shape = ref world.shapes[shapeId];
								aabb = AABB_Union( aabb, world.fatAABBs.data[shapeId] );
								shapeCount += 1;
								shapeId = shape.nextShapeId;
							}
						}

						if ( shapeCount > 0 )
						{
							draw.DrawBoundsFcn( aabb, HexColor.colorOrangeRed, draw.context );
						}

						SetBit( ref world.debugIslandSet, islandId );
					}
				}

				// Clear the smallest set bit
				word = word & ( word - 1 );
			}
		}
	}

	internal static bool ComputeWorldBounds( World world, ref AABB bounds )
	{
		AABB worldBounds = default;
		bool haveBounds = false;

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			DynamicTree tree = world.broadPhase.trees[i];
			if ( DynamicTree_GetProxyCount( tree ) == 0 )
			{
				continue;
			}

			AABB treeBounds = DynamicTree_GetRootBounds( tree );
			worldBounds = haveBounds ? AABB_Union( worldBounds, treeBounds ) : treeBounds;
			haveBounds = true;
		}

		bounds = worldBounds;
		return haveBounds;
	}

	public static AABB World_GetBounds( WorldId worldId )
	{
		World world = GetUnlockedWorldFromId( worldId );
		if ( world == null )
		{
			return default(AABB);
		}

		AABB bounds = default;
		ComputeWorldBounds( world, ref bounds );
		return bounds;
	}

	public static BodyEvents World_GetBodyEvents( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return default(BodyEvents);
		}

		int count = world.bodyMoveEvents.count;
		BodyEvents events = new BodyEvents { moveEvents = world.bodyMoveEvents.data, moveCount = count };
		return events;
	}

	public static SensorEvents World_GetSensorEvents( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return default(SensorEvents);
		}

		// Careful to use previous buffer
		int endEventArrayIndex = 1 - world.endEventArrayIndex;

		int beginCount = world.sensorBeginEvents.count;
		int endCount = world.sensorEndEvents[endEventArrayIndex].count;

		SensorEvents events = new SensorEvents { beginEvents = world.sensorBeginEvents.data, endEvents = world.sensorEndEvents[endEventArrayIndex].data, beginCount = beginCount, endCount = endCount };
		return events;
	}

	public static ContactEvents World_GetContactEvents( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return default(ContactEvents);
		}

		// Careful to use previous buffer
		int endEventArrayIndex = 1 - world.endEventArrayIndex;

		int beginCount = world.contactBeginEvents.count;
		int endCount = world.contactEndEvents[endEventArrayIndex].count;
		int hitCount = world.contactHitEvents.count;

		ContactEvents events = new ContactEvents { beginEvents = world.contactBeginEvents.data, endEvents = world.contactEndEvents[endEventArrayIndex].data, hitEvents = world.contactHitEvents.data, beginCount = beginCount, endCount = endCount, hitCount = hitCount };

		return events;
	}

	public static JointEvents World_GetJointEvents( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return default(JointEvents);
		}

		int count = world.jointEvents.count;
		JointEvents events = new JointEvents { jointEvents = world.jointEvents.data, count = count };
		return events;
	}

	public static bool World_IsValid( WorldId id )
	{
		if ( id.index1 < 1 || MaxWorlds < id.index1 )
		{
			return false;
		}

		World world = worlds[id.index1 - 1];
		if ( world.worldId != id.index1 - 1 )
		{
			// world is not allocated
			return false;
		}

		return id.generation == world.generation;
	}

	public static bool Body_IsValid( BodyId id )
	{
		if ( MaxWorlds <= id.world0 )
		{
			// invalid world
			return false;
		}

		World world = worlds[id.world0];
		if ( world.worldId != id.world0 )
		{
			// world is free
			return false;
		}

		if ( id.index1 < 1 || world.bodies.count < id.index1 )
		{
			// invalid index
			return false;
		}

		ref Body body = ref world.bodies.data[( id.index1 - 1 )];
		if ( body.setIndex == NullIndex )
		{
			// this was freed
			return false;
		}

		B2_ASSERT( body.localIndex != NullIndex );

		if ( body.generation != id.generation )
		{
			// this id is orphaned
			return false;
		}

		return true;
	}

	public static bool Shape_IsValid( ShapeId id )
	{
		if ( MaxWorlds <= id.world0 )
		{
			return false;
		}

		World world = worlds[id.world0];
		if ( world.worldId != id.world0 )
		{
			// world is free
			return false;
		}

		int shapeId = id.index1 - 1;
		if ( shapeId < 0 || world.shapes.count <= shapeId )
		{
			return false;
		}

		ref Shape shape = ref world.shapes.data[shapeId];
		if ( shape.id == NullIndex )
		{
			// shape is free
			return false;
		}

		B2_ASSERT( shape.id == shapeId );

		return id.generation == shape.generation;
	}

	public static bool Chain_IsValid( ChainId id )
	{
		if ( MaxWorlds <= id.world0 )
		{
			return false;
		}

		World world = worlds[id.world0];
		if ( world.worldId != id.world0 )
		{
			// world is free
			return false;
		}

		int chainId = id.index1 - 1;
		if ( chainId < 0 || world.chainShapes.count <= chainId )
		{
			return false;
		}

		ref ChainShape chain = ref world.chainShapes.data[chainId];
		if ( chain.id == NullIndex )
		{
			// chain is free
			return false;
		}

		B2_ASSERT( chain.id == chainId );

		return id.generation == chain.generation;
	}

	public static bool Joint_IsValid( JointId id )
	{
		if ( MaxWorlds <= id.world0 )
		{
			return false;
		}

		World world = worlds[id.world0];
		if ( world.worldId != id.world0 )
		{
			// world is free
			return false;
		}

		int jointId = id.index1 - 1;
		if ( jointId < 0 || world.joints.count <= jointId )
		{
			return false;
		}

		ref Joint joint = ref world.joints.data[jointId];
		if ( joint.jointId == NullIndex )
		{
			// joint is free
			return false;
		}

		B2_ASSERT( joint.jointId == jointId );

		return id.generation == joint.generation;
	}

	public static bool Contact_IsValid( ContactId id )
	{
		if ( MaxWorlds <= id.world0 )
		{
			return false;
		}

		World world = worlds[id.world0];
		if ( world.worldId != id.world0 )
		{
			// world is free
			return false;
		}

		int contactId = id.index1 - 1;
		if ( contactId < 0 || world.contacts.count <= contactId )
		{
			return false;
		}

		ref Contact contact = ref world.contacts.data[contactId];
		if ( contact.contactId == NullIndex )
		{
			// contact is free
			return false;
		}

		B2_ASSERT( contact.contactId == contactId );

		return id.generation == contact.generation;
	}

	public static void World_EnableSleeping( WorldId worldId, bool flag )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		if ( flag == world.enableSleep )
		{
			return;
		}

		world.enableSleep = flag;

		if ( flag == false )
		{
			int setCount = world.solverSets.count;
			for ( int i = b2_firstSleepingSet; i < setCount; ++i )
			{
				ref SolverSet set = ref world.solverSets[i];
				if ( set.bodySims.count > 0 )
				{
					WakeSolverSet( world, i );
				}
			}
		}
	}

	public static bool World_IsSleepingEnabled( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.enableSleep;
	}

	public static void World_EnableWarmStarting( WorldId worldId, bool flag )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.enableWarmStarting = flag;
	}

	public static bool World_IsWarmStartingEnabled( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.enableWarmStarting;
	}

	public static void World_EnableSSE2Fallback( WorldId worldId, bool flag )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}

		world.simdWidth = flag ? 4 : GetSIMDWidth();
	}

	public static int World_GetAwakeBodyCount( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		return awakeSet.bodySims.count;
	}

	public static void World_EnableContinuous( WorldId worldId, bool flag )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.enableContinuous = flag;
	}

	public static bool World_IsContinuousEnabled( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.enableContinuous;
	}

	public static void World_SetRestitutionThreshold( WorldId worldId, float value )
	{
		if ( CheckInput( IsValidFloat( value ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.restitutionThreshold = ClampFloat( value, 0.0f, FLT_MAX );
	}

	public static float World_GetRestitutionThreshold( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.restitutionThreshold;
	}

	public static void World_SetRestitutionIterations( WorldId worldId, int iterations )
	{
		if ( CheckInput( iterations >= 0 ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.restitutionIterations = ClampInt( iterations, 0, MaxRestitutionIterations );
	}

	public static int World_GetRestitutionIterations( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.restitutionIterations;
	}

	public static void World_EnableRestitutionPropagation( WorldId worldId, bool flag )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.enableRestitutionPropagation = flag;
	}

	public static bool World_IsRestitutionPropagationEnabled( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.enableRestitutionPropagation;
	}

	public static void World_SetHitEventThreshold( WorldId worldId, float value )
	{
		if ( CheckInput( IsValidFloat( value ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.hitEventThreshold = ClampFloat( value, 0.0f, FLT_MAX );
	}

	public static float World_GetHitEventThreshold( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.hitEventThreshold;
	}

	public static void World_SetContactTuning( WorldId worldId, float hertz, float dampingRatio, float pushSpeed )
	{
		if ( CheckInput( IsValidFloat( hertz ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( dampingRatio ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( pushSpeed ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.contactHertz = ClampFloat( hertz, 0.0f, FLT_MAX );
		world.contactDampingRatio = ClampFloat( dampingRatio, 0.0f, FLT_MAX );
		world.contactSpeed = ClampFloat( pushSpeed, 0.0f, FLT_MAX );
	}

	public static void World_GetContactTuning( WorldId worldId, ref float hertz, ref float dampingRatio, ref float pushSpeed )
	{
		World world = GetWorldFromId( worldId );
		hertz = world.contactHertz;
		dampingRatio = world.contactDampingRatio;
		pushSpeed = world.contactSpeed;
	}

	public static void World_SetContactRecycleDistance( WorldId worldId, float recycleDistance )
	{
		if ( CheckInput( IsValidFloat( recycleDistance ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.contactRecycleDistance = ClampFloat( recycleDistance, 0.0f, FLT_MAX );
	}

	public static float World_GetContactRecycleDistance( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.contactRecycleDistance;
	}

	public static void World_SetMaxLinearSpeed( WorldId worldId, float maximumLinearSpeed )
	{
		if ( CheckInput( IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		world.maxLinearSpeed = maximumLinearSpeed;
	}

	public static float World_GetMaxLinearSpeed( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.maxLinearSpeed;
	}

	public static Profile World_GetProfile( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.profile;
	}

	public static Counters World_GetCounters( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		Counters s = default;
		s.bodyCount = GetIdCount( ref world.bodyIdPool );
		s.shapeCount = GetIdCount( ref world.shapeIdPool );
		s.contactCount = GetIdCount( ref world.contactIdPool );
		s.jointCount = GetIdCount( ref world.jointIdPool );
		s.islandCount = GetIdCount( ref world.islandIdPool );

		DynamicTree staticTree = world.broadPhase.trees[BodyType.staticBody];
		s.staticTreeHeight = DynamicTree_GetHeight( staticTree );

		DynamicTree dynamicTree = world.broadPhase.trees[BodyType.dynamicBody];
		DynamicTree kinematicTree = world.broadPhase.trees[BodyType.kinematicBody];
		s.treeHeight = MaxInt( DynamicTree_GetHeight( dynamicTree ), DynamicTree_GetHeight( kinematicTree ) );

		s.stackUsed = GetMaxStackAllocation( world.stack );
		s.byteCount = (int)GetByteCount();
		s.taskCount = world.taskCount;

		s.recycledContactCount = 0;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			s.recycledContactCount += world.taskContexts.data[i].recycledContactCount;
		}

		s.awakeContactCount = 0;
		for ( int i = 0; i < GraphColorCount; ++i )
		{
			ref GraphColor color = ref world.constraintGraph.colors[i];
			s.colorCounts[i] = color.contactSims.count + color.jointSims.count;
			s.awakeContactCount += color.contactSims.count;
		}
		s.awakeContactCount += world.solverSets.data[b2_awakeSet].contactSims.count;

		return s;
	}

	public static Capacity World_GetMaxCapacity( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.maxCapacity;
	}

	public static void World_SetUserData( WorldId worldId, object? userData )
	{
		World world = GetWorldFromId( worldId );
		world.userData = userData;
	}

	public static object? World_GetUserData( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.userData;
	}

	public static void World_SetFrictionCallback( WorldId worldId, FrictionCallbackFcn callback )
	{
		World world = GetWorldFromId( worldId );
		if ( world.locked )
		{
			return;
		}

		world.frictionCallback = callback;
	}

	public static void World_SetRestitutionCallback( WorldId worldId, RestitutionCallbackFcn callback )
	{
		World world = GetWorldFromId( worldId );
		if ( world.locked )
		{
			return;
		}

		world.restitutionCallback = callback;
	}

	public static void World_SetWorkerCount( WorldId worldId, int count )
	{
		World world = GetUnlockedWorldFromId( worldId );
		if ( world == null )
		{
			return;
		}

		if ( count == world.workerCount )
		{
			return;
		}

		DestroyWorkerContexts( world );
		world.workerCount = ClampInt( count, 1, MaxWorkers );
		CreateWorkerContexts( world );
	}

	public static int World_GetWorkerCount( WorldId worldId )
	{
		World world = GetUnlockedWorldFromId( worldId );
		if ( world == null )
		{
			return 0;
		}

		return world.workerCount;
	}

	internal static void World_StartRecording( WorldId worldId, Recording recording )
	{
		// Must be a step boundary, so refuse a locked world
		World world = GetUnlockedWorldFromId( worldId );

		if ( world.preSolveFcn != null || world.preContinuousFcn != null || world.customFilterFcn != null )
		{
			Log( "World_StartRecording: preSolve and custom filtering are not supported when recording" );
			return;
		}

		// Recording is not ported yet
	}

	internal static void World_StopRecording( WorldId worldId )
	{
		World world = GetUnlockedWorldFromId( worldId );
		if ( world == null )
		{
			return;
		}

		// Recording is not ported yet
	}

	public static void World_DumpMemoryStats( WorldId worldId )
	{
		using System.IO.StreamWriter file = new System.IO.StreamWriter( "box2d_memory.txt" );

		World world = GetWorldFromId( worldId );
		int total = 0;

		// id pools
		int bodyIdBytes = GetIdBytes( ref world.bodyIdPool );
		int solverSetIdBytes = GetIdBytes( ref world.solverSetIdPool );
		int jointIdBytes = GetIdBytes( ref world.jointIdPool );
		int contactIdBytes = GetIdBytes( ref world.contactIdPool );
		int islandIdBytes = GetIdBytes( ref world.islandIdPool );
		int shapeIdBytes = GetIdBytes( ref world.shapeIdPool );
		int chainIdBytes = GetIdBytes( ref world.chainIdPool );
		total += bodyIdBytes + solverSetIdBytes + jointIdBytes + contactIdBytes + islandIdBytes + shapeIdBytes + chainIdBytes;

		file.WriteLine( $"id pools" );
		file.WriteLine( $"body ids: {bodyIdBytes}" );
		file.WriteLine( $"solver set ids: {solverSetIdBytes}" );
		file.WriteLine( $"joint ids: {jointIdBytes}" );
		file.WriteLine( $"contact ids: {contactIdBytes}" );
		file.WriteLine( $"island ids: {islandIdBytes}" );
		file.WriteLine( $"shape ids: {shapeIdBytes}" );
		file.WriteLine( $"chain ids: {chainIdBytes}" );
		file.WriteLine( $"" );

		// Islands own per-island body/contact/joint link arrays
		int islandLinkBytes = 0;
		for ( int i = 0; i < world.islands.count; ++i )
		{
			ref Island island = ref world.islands.data[i];
			islandLinkBytes += island.bodies.ByteCount();
			islandLinkBytes += island.contacts.ByteCount();
			islandLinkBytes += island.joints.ByteCount();
		}

		// world arrays
		int bodyArrayBytes = world.bodies.ByteCount();
		int solverSetArrayBytes = world.solverSets.ByteCount();
		int jointArrayBytes = world.joints.ByteCount();
		int contactArrayBytes = world.contacts.ByteCount();
		int islandArrayBytes = world.islands.ByteCount();
		int shapeArrayBytes = world.shapes.ByteCount() + world.fatAABBs.ByteCount();
		int chainArrayBytes = world.chainShapes.ByteCount();
		int sensorArrayBytes = world.sensors.ByteCount();
		total += bodyArrayBytes + solverSetArrayBytes + jointArrayBytes + contactArrayBytes + islandArrayBytes + islandLinkBytes +
				 shapeArrayBytes + chainArrayBytes + sensorArrayBytes;

		file.WriteLine( $"world arrays" );
		file.WriteLine( $"bodies: {bodyArrayBytes}" );
		file.WriteLine( $"solver sets: {solverSetArrayBytes}" );
		file.WriteLine( $"joints: {jointArrayBytes}" );
		file.WriteLine( $"contacts: {contactArrayBytes}" );
		file.WriteLine( $"islands: {islandArrayBytes}" );
		file.WriteLine( $"island links: {islandLinkBytes}" );
		file.WriteLine( $"shapes: {shapeArrayBytes}" );
		file.WriteLine( $"chains: {chainArrayBytes}" );
		file.WriteLine( $"sensors: {sensorArrayBytes}" );
		file.WriteLine( $"" );

		// Chain shapes own index and surface material arrays
		int chainDataBytes = 0;
		for ( int i = 0; i < world.chainShapes.count; ++i )
		{
			ref ChainShape chain = ref world.chainShapes.data[i];
			if ( chain.id == NullIndex )
			{
				continue;
			}

			chainDataBytes += chain.segmentCount * (int)Unsafe.SizeOf<int>();
		}

		// Sensors own overlap tracking arrays. The sensor array is dense.
		int sensorOverlapBytes = 0;
		for ( int i = 0; i < world.sensors.count; ++i )
		{
			ref Sensor sensor = ref world.sensors.data[i];
			sensorOverlapBytes += sensor.hits.ByteCount();
			sensorOverlapBytes += sensor.overlaps1.ByteCount();
			sensorOverlapBytes += sensor.overlaps2.ByteCount();
		}
		total += chainDataBytes + sensorOverlapBytes;

		file.WriteLine( $"owned arrays" );
		file.WriteLine( $"chain data: {chainDataBytes}" );
		file.WriteLine( $"sensor overlaps: {sensorOverlapBytes}" );
		file.WriteLine( $"" );

		// broad-phase
		int staticTreeBytes = (int)DynamicTree_GetByteCount( world.broadPhase.trees[BodyType.staticBody] );
		int kinematicTreeBytes = (int)DynamicTree_GetByteCount( world.broadPhase.trees[BodyType.kinematicBody] );
		int dynamicTreeBytes = (int)DynamicTree_GetByteCount( world.broadPhase.trees[BodyType.dynamicBody] );
		ref HashSet64 pairSet = ref world.broadPhase.pairSet;
		int pairSetBytes = GetHashSetBytes( ref pairSet );
		total += staticTreeBytes + kinematicTreeBytes + dynamicTreeBytes + pairSetBytes;

		file.WriteLine( $"broad-phase" );
		file.WriteLine( $"static tree: {staticTreeBytes}" );
		file.WriteLine( $"kinematic tree: {kinematicTreeBytes}" );
		file.WriteLine( $"dynamic tree: {dynamicTreeBytes}" );
		file.WriteLine( $"pairSet: {pairSetBytes} ({pairSet.count}, {pairSet.capacity})" );
		file.WriteLine( $"" );

		// solver sets
		int bodySimCapacity = 0;
		int bodyStateCapacity = 0;
		int jointSimCapacity = 0;
		int contactSimCapacity = 0;
		int islandSimCapacity = 0;
		int solverSetCapacity = world.solverSets.count;
		for ( int i = 0; i < solverSetCapacity; ++i )
		{
			ref SolverSet set = ref world.solverSets.data[i];
			if ( set.setIndex == NullIndex )
			{
				continue;
			}

			bodySimCapacity += set.bodySims.capacity;
			bodyStateCapacity += set.bodyStates.capacity;
			jointSimCapacity += set.jointSims.capacity;
			contactSimCapacity += set.contactSims.capacity;
			islandSimCapacity += set.islandSims.capacity;
		}

		int setBodySimBytes = bodySimCapacity * (int)Unsafe.SizeOf<BodySim>();
		int setBodyStateBytes = bodyStateCapacity * (int)Unsafe.SizeOf<BodyState>();
		int setJointSimBytes = jointSimCapacity * (int)Unsafe.SizeOf<JointSim>();
		int setContactSimBytes = contactSimCapacity * (int)Unsafe.SizeOf<ContactSim>();
		int setIslandSimBytes = islandSimCapacity * (int)Unsafe.SizeOf<IslandSim>();
		total += setBodySimBytes + setBodyStateBytes + setJointSimBytes + setContactSimBytes + setIslandSimBytes;

		file.WriteLine( $"solver sets" );
		file.WriteLine( $"body sim: {setBodySimBytes}" );
		file.WriteLine( $"body state: {setBodyStateBytes}" );
		file.WriteLine( $"joint sim: {setJointSimBytes}" );
		file.WriteLine( $"contact sim: {setContactSimBytes}" );
		file.WriteLine( $"island sim: {setIslandSimBytes}" );
		file.WriteLine( $"" );

		// constraint graph
		int bodyBitSetBytes = 0;
		contactSimCapacity = 0;
		jointSimCapacity = 0;
		for ( int i = 0; i < GraphColorCount; ++i )
		{
			ref GraphColor c = ref world.constraintGraph.colors[i];
			bodyBitSetBytes += GetBitSetBytes( ref c.bodySet );
			contactSimCapacity += c.contactSims.capacity;
			jointSimCapacity += c.jointSims.capacity;
		}

		int graphJointSimBytes = jointSimCapacity * (int)Unsafe.SizeOf<JointSim>();
		int graphContactSimBytes = contactSimCapacity * (int)Unsafe.SizeOf<ContactSim>();
		total += bodyBitSetBytes + graphJointSimBytes + graphContactSimBytes;

		file.WriteLine( $"constraint graph" );
		file.WriteLine( $"body bit sets: {bodyBitSetBytes}" );
		file.WriteLine( $"joint sim: {graphJointSimBytes}" );
		file.WriteLine( $"contact sim: {graphContactSimBytes}" );
		file.WriteLine( $"" );

		// Per worker task storage and its bit sets
		int taskContextBytes = world.taskContexts.ByteCount();
		for ( int i = 0; i < world.taskContexts.count; ++i )
		{
			ref TaskContext taskContext = ref world.taskContexts.data[i];
			taskContextBytes += taskContext.sensorHits.ByteCount();
			taskContextBytes += taskContext.pairKeys.ByteCount();
			taskContextBytes += GetBitSetBytes( ref taskContext.contactStateBitSet );
			taskContextBytes += GetBitSetBytes( ref taskContext.hitEventBitSet );
			taskContextBytes += GetBitSetBytes( ref taskContext.jointStateBitSet );
			taskContextBytes += GetBitSetBytes( ref taskContext.enlargedSimBitSet );
			taskContextBytes += GetBitSetBytes( ref taskContext.awakeIslandBitSet );
		}

		int sensorTaskContextBytes = world.sensorTaskContexts.ByteCount();
		for ( int i = 0; i < world.sensorTaskContexts.count; ++i )
		{
			ref SensorTaskContext taskContext = ref world.sensorTaskContexts.data[i];
			sensorTaskContextBytes += GetBitSetBytes( ref taskContext.eventBits );
		}
		total += taskContextBytes + sensorTaskContextBytes;

		file.WriteLine( $"task contexts" );
		file.WriteLine( $"worker: {taskContextBytes}" );
		file.WriteLine( $"sensor: {sensorTaskContextBytes}" );
		file.WriteLine( $"" );

		// Double buffered event arrays
		int eventBytes = 0;
		eventBytes += world.bodyMoveEvents.ByteCount();
		eventBytes += world.sensorBeginEvents.ByteCount();
		eventBytes += world.contactBeginEvents.ByteCount();
		eventBytes += world.sensorEndEvents[0].ByteCount();
		eventBytes += world.sensorEndEvents[1].ByteCount();
		eventBytes += world.contactEndEvents[0].ByteCount();
		eventBytes += world.contactEndEvents[1].ByteCount();
		eventBytes += world.contactHitEvents.ByteCount();
		eventBytes += world.jointEvents.ByteCount();
		total += eventBytes;

		file.WriteLine( $"events: {eventBytes}" );

		// Debug draw bit sets
		int debugBytes = 0;
		debugBytes += GetBitSetBytes( ref world.debugBodySet );
		debugBytes += GetBitSetBytes( ref world.debugJointSet );
		debugBytes += GetBitSetBytes( ref world.debugContactSet );
		debugBytes += GetBitSetBytes( ref world.debugIslandSet );
		total += debugBytes;

		file.WriteLine( $"debug draw: {debugBytes}" );

		// stack allocator
		total += GetStackCapacity( world.stack );
		file.WriteLine( $"stack allocator: {GetStackCapacity( world.stack )}" );

		file.WriteLine( $"total: {total}" );

	}

	internal ref struct WorldQueryContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => TreeQueryCallback( proxyId, userData, ref this );

		public World world;
		public OverlapResultCallbackFcn? fcn;
		public QueryFilter filter;
		public object? userContext;
	}

	private static bool TreeQueryCallback( int proxyId, ulong userData, ref WorldQueryContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return true;
		}

		ShapeId id = new ShapeId { index1 = shapeId + 1, world0 = world.worldId, generation = shape.generation };
		bool result = worldContext.fcn( id, worldContext.userContext );
		return result;
	}

	public static TreeStats World_OverlapAABB( WorldId worldId, Pos origin, AABB aabb, QueryFilter filter, OverlapResultCallbackFcn fcn, object? context )
	{
		TreeStats treeStats = default;

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return treeStats;
		}

		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidAABB( aabb ) );

		// Lift to a world float box with outward rounding so the conservative tree test never misses
		AABB worldBox = OffsetAABB( aabb, origin );

		WorldQueryContext worldContext = new WorldQueryContext { world = world, fcn = fcn, filter = filter, userContext = context };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			TreeStats treeResult =
				DynamicTree_OverlapAABB( world.broadPhase.trees[i], worldBox, filter.maskBits, ref worldContext );

			treeStats.nodeVisits += treeResult.nodeVisits;
			treeStats.leafVisits += treeResult.leafVisits;
		}


		return treeStats;
	}

	internal ref struct WorldOverlapContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => TreeOverlapCallback( proxyId, userData, ref this );

		public World world;
		public OverlapResultCallbackFcn? fcn;
		public QueryFilter filter;
		public ref ShapeProxy proxy;
		public Pos origin;
		public object? userContext;
	}

	private static bool TreeOverlapCallback( int proxyId, ulong userData, ref WorldOverlapContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return true;
		}

		// Re-center on the query origin so the distance test stays in float precision far from the world origin
		ref Body body = ref world.bodies[shape.bodyId];
		Transform transform = ToRelativeTransform( GetBodyTransformQuick( world, ref body ), worldContext.origin );

		DistanceInput input = default;
		input.proxyA = worldContext.proxy;
		input.proxyB = MakeShapeDistanceProxy( shape );
		input.transform = transform;
		input.useRadii = true;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( ref input, ref cache );

		float tolerance = 0.1f * LinearSlop;
		if ( output.distance > tolerance )
		{
			return true;
		}

		ShapeId id = new ShapeId { index1 = shape.id + 1, world0 = world.worldId, generation = shape.generation };
		bool result = worldContext.fcn( id, worldContext.userContext );
		return result;
	}

	public static TreeStats World_OverlapShape( WorldId worldId, Pos origin, in ShapeProxy proxy, QueryFilter filter, OverlapResultCallbackFcn fcn, object? context )
	{
		TreeStats treeStats = default;

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return treeStats;
		}

		B2_ASSERT( IsValidPos( origin ) );

		// Relative box lifted to world float with outward rounding, conservative for the tree
		AABB aabb = OffsetAABB( MakeAABB( proxy.points, proxy.count, proxy.radius ), origin );
		WorldOverlapContext worldContext = new WorldOverlapContext { world = world, fcn = fcn, filter = filter, proxy = proxy, origin = origin, userContext = context };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			TreeStats treeResult =
				DynamicTree_OverlapAABB( world.broadPhase.trees[i], aabb, filter.maskBits, ref worldContext );

			treeStats.nodeVisits += treeResult.nodeVisits;
			treeStats.leafVisits += treeResult.leafVisits;
		}


		return treeStats;
	}

	internal ref struct WorldRayCastContext : ITreeRayCastCallback
	{
		public float Invoke( in RayCastInput input, int proxyId, ulong userData ) => RayCastCallback( input, proxyId, userData, ref this );

		public World world;
		public CastResultCallbackFcn? fcn;
		public QueryFilter filter;
		public float fraction;
		public Pos origin;
		public object? userContext;
	}

	private static float RayCastCallback( in RayCastInput input, int proxyId, ulong userData, ref WorldRayCastContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return input.maxFraction;
		}

		ref Body body = ref world.bodies[shape.bodyId];
		WorldTransform bodyTransform = GetBodyTransformQuick( world, ref body );
		Transform transform = ToRelativeTransform( bodyTransform, worldContext.origin );

		RayCastInput localInput = input;
		localInput.origin = Vec2_zero;
		CastOutput output = RayCastShape( ref localInput, shape, transform );

		if ( output.hit )
		{
			ShapeId id = new ShapeId { index1 = shapeId + 1, world0 = world.worldId, generation = shape.generation };
			Pos point = OffsetPos( worldContext.origin, output.point );
			float fraction = worldContext.fcn( id, point, output.normal, output.fraction, worldContext.userContext );

			// The user may return -1 to skip this shape
			if ( 0.0f <= fraction && fraction <= 1.0f )
			{
				worldContext.fraction = fraction;
			}

			return fraction;
		}

		return input.maxFraction;
	}

	public static TreeStats World_CastRay( WorldId worldId, Pos origin, Vec2 translation, QueryFilter filter, CastResultCallbackFcn fcn, object? context )
	{
		TreeStats treeStats = default;

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return treeStats;
		}

		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidVec2( translation ) );

		// Tree traversal sees the origin truncated to float, displacing the ray by up to one
		// coordinate ULP, a graze sized miss tolerance at extreme range. Per-shape casts
		// re-difference against the full precision origin carried on the context.
		RayCastInput input = new RayCastInput { origin = ToVec2( origin ), translation = translation, maxFraction = 1.0f };

		WorldRayCastContext worldContext = new WorldRayCastContext { world = world, fcn = fcn, filter = filter, fraction = 1.0f, origin = origin, userContext = context };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			TreeStats treeResult =
				DynamicTree_CastRay( world.broadPhase.trees[i], ref input, filter.maskBits, ref worldContext );
			treeStats.nodeVisits += treeResult.nodeVisits;
			treeStats.leafVisits += treeResult.leafVisits;

			if ( worldContext.fraction == 0.0f )
			{
				break;
			}

			input.maxFraction = worldContext.fraction;
		}


		return treeStats;
	}

	// This callback finds the closest hit. This is the most common callback used in games.
	private static float RayCastClosestFcn( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )
	{
		// Ignore initial overlap
		if ( fraction == 0.0f )
		{
			return -1.0f;
		}

		ref RayResult rayResult = ref ( (StrongBox<RayResult>)context! ).Value;
		rayResult.shapeId = shapeId;
		rayResult.point = point;
		rayResult.normal = normal;
		rayResult.fraction = fraction;
		rayResult.hit = true;
		return fraction;
	}

	public static RayResult World_CastRayClosest( WorldId worldId, Pos origin, Vec2 translation, QueryFilter filter )
	{
		RayResult result = default;

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return result;
		}

		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidVec2( translation ) );

		RayCastInput input = new RayCastInput { origin = ToVec2( origin ), translation = translation, maxFraction = 1.0f };
		// The result is written through the callback context
		StrongBox<RayResult> resultBox = new StrongBox<RayResult>( result );
		WorldRayCastContext worldContext = new WorldRayCastContext { world = world, fcn = RayCastClosestFcn, filter = filter, fraction = 1.0f, origin = origin, userContext = resultBox };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			TreeStats treeResult =
				DynamicTree_CastRay( world.broadPhase.trees[i], ref input, filter.maskBits, ref worldContext );
			resultBox.Value.nodeVisits += treeResult.nodeVisits;
			resultBox.Value.leafVisits += treeResult.leafVisits;

			if ( worldContext.fraction == 0.0f )
			{
				break;
			}

			input.maxFraction = worldContext.fraction;
		}

		return resultBox.Value;
	}

	internal ref struct WorldShapeCastContext : ITreeBoxCastCallback
	{
		public float Invoke( in BoxCastInput input, int proxyId, ulong userData ) => ShapeCastCallback( input, proxyId, userData, ref this );

		public World world;
		public CastResultCallbackFcn? fcn;
		public QueryFilter filter;
		public float fraction;
		public Pos origin;
		// origin relative input
		public ShapeCastInput input;
		public object? userContext;
	}

	private static float ShapeCastCallback( in BoxCastInput input, int proxyId, ulong userData, ref WorldShapeCastContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return input.maxFraction;
		}

		// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
		// The tree input is world float and would lose the cast far from the origin.
		ShapeCastInput localInput = worldContext.input;
		localInput.maxFraction = input.maxFraction;

		ref Body body = ref world.bodies[shape.bodyId];
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		Transform localTransform = ToRelativeTransform( transform, worldContext.origin );

		CastOutput output = ShapeCastShape( ref localInput, shape, localTransform );

		if ( output.hit )
		{
			ShapeId id = new ShapeId { index1 = shapeId + 1, world0 = world.worldId, generation = shape.generation };

			Pos point = OffsetPos( worldContext.origin, output.point );
			float fraction = worldContext.fcn( id, point, output.normal, output.fraction, worldContext.userContext );

			// The user may return -1 to skip this shape
			if ( 0.0f <= fraction && fraction <= 1.0f )
			{
				worldContext.fraction = fraction;
			}

			return fraction;
		}

		return input.maxFraction;
	}

	public static TreeStats World_CastShape( WorldId worldId, Pos origin, in ShapeProxy proxy, Vec2 translation, QueryFilter filter, CastResultCallbackFcn fcn, object? context )
	{
		TreeStats treeStats = default;

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return treeStats;
		}

		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidVec2( translation ) );

		WorldShapeCastContext worldContext = default;
		worldContext.world = world;
		worldContext.fcn = fcn;
		worldContext.filter = filter;
		worldContext.fraction = 1.0f;
		worldContext.origin = origin;
		worldContext.input.proxy = proxy;
		worldContext.input.translation = translation;
		worldContext.input.maxFraction = 1.0f;
		worldContext.userContext = context;

		// Create a conservative world space box from the proxy and origin.
		AABB localBox = MakeAABB( proxy.points, proxy.count, proxy.radius );
		AABB box = OffsetAABB( localBox, origin );
		BoxCastInput treeInput = new BoxCastInput { box = box, translation = translation, maxFraction = 1.0f };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			TreeStats treeResult =
				DynamicTree_CastBox( world.broadPhase.trees[i], ref treeInput, filter.maskBits, ref worldContext );
			treeStats.nodeVisits += treeResult.nodeVisits;
			treeStats.leafVisits += treeResult.leafVisits;

			if ( worldContext.fraction == 0.0f )
			{
				break;
			}

			treeInput.maxFraction = worldContext.fraction;
		}


		return treeStats;
	}

	internal ref struct WorldMoverCastContext : ITreeBoxCastCallback
	{
		public float Invoke( in BoxCastInput input, int proxyId, ulong userData ) => MoverCastCallback( input, proxyId, userData, ref this );

		public World world;
		public QueryFilter filter;
		public float fraction;
		public Pos origin;
		// origin relative input
		public ShapeCastInput input;
	}

	private static float MoverCastCallback( in BoxCastInput input, int proxyId, ulong userData, ref WorldMoverCastContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return worldContext.fraction;
		}

		// Rebuild from the origin relative input, taking only the advancing fraction from the tree
		ShapeCastInput localInput = worldContext.input;
		localInput.maxFraction = input.maxFraction;

		ref Body body = ref world.bodies[shape.bodyId];
		WorldTransform bodyTransform = GetBodyTransformQuick( world, ref body );
		Transform transform = ToRelativeTransform( bodyTransform, worldContext.origin );

		CastOutput output = ShapeCastShape( ref localInput, shape, transform );
		if ( output.fraction == 0.0f )
		{
			// Ignore overlapping shapes
			return worldContext.fraction;
		}

		worldContext.fraction = output.fraction;
		return output.fraction;
	}

	public static float World_CastMover( WorldId worldId, Pos origin, in Capsule mover, Vec2 translation, QueryFilter filter )
	{
		B2_ASSERT( IsValidPos( origin ) );
		B2_ASSERT( IsValidVec2( translation ) );
		B2_ASSERT( mover.radius > 2.0f * LinearSlop );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return 1.0f;
		}

		WorldMoverCastContext worldContext = default;
		worldContext.world = world;
		worldContext.filter = filter;
		worldContext.fraction = 1.0f;
		worldContext.origin = origin;
		worldContext.input.proxy.points[0] = mover.center1;
		worldContext.input.proxy.points[1] = mover.center2;
		worldContext.input.proxy.count = 2;
		worldContext.input.proxy.radius = mover.radius;
		worldContext.input.translation = translation;
		worldContext.input.maxFraction = 1.0f;
		worldContext.input.canEncroach = true;

		// Bound the capsule in origin relative space then lift to a conservative world float box
		Vec2[] centers = { mover.center1, mover.center2 };
		AABB box = OffsetAABB( MakeAABB( centers, 2, mover.radius ), origin );
		BoxCastInput treeInput = new BoxCastInput { box = box, translation = translation, maxFraction = 1.0f };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			DynamicTree_CastBox( world.broadPhase.trees[i], ref treeInput, filter.maskBits, ref worldContext );

			if ( worldContext.fraction == 0.0f )
			{
				break;
			}

			treeInput.maxFraction = worldContext.fraction;
		}


		return worldContext.fraction;
	}

	internal ref struct WorldMoverContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => TreeCollideCallback( proxyId, userData, ref this );

		public World world;
		public PlaneResultCallbackFcn? fcn;
		public QueryFilter filter;
		public Capsule mover;
		public Pos origin;
		public object? userContext;
	}

	private static bool TreeCollideCallback( int proxyId, ulong userData, ref WorldMoverContext worldContext )
	{
		int shapeId = (int)userData;
		World world = worldContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		if ( ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
		{
			return true;
		}

		// Re-center on the query origin, the mover and the resulting planes are origin relative
		ref Body body = ref world.bodies[shape.bodyId];
		Transform transform = ToRelativeTransform( GetBodyTransformQuick( world, ref body ), worldContext.origin );

		PlaneResult result = CollideMover( worldContext.mover, shape, transform );

		// todo handle deep overlap
		if ( result.hit && IsNormalized( result.plane.normal ) )
		{
			ShapeId id = new ShapeId { index1 = shape.id + 1, world0 = world.worldId, generation = shape.generation };
			return worldContext.fcn( id, result, worldContext.userContext );
		}

		return true;
	}

	// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
	// not be worth it.
	public static void World_CollideMover( WorldId worldId, Pos origin, in Capsule mover, QueryFilter filter, PlaneResultCallbackFcn fcn, object? context )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}

		B2_ASSERT( IsValidPos( origin ) );

		Vec2 r = new Vec2( mover.radius, mover.radius );

		// Relative box lifted to world float with outward rounding, conservative for the tree
		AABB relBox = default;
		relBox.lowerBound = Sub( Min( mover.center1, mover.center2 ), r );
		relBox.upperBound = Add( Max( mover.center1, mover.center2 ), r );
		AABB aabb = OffsetAABB( relBox, origin );

		WorldMoverContext worldContext = new WorldMoverContext { world = world, fcn = fcn, filter = filter, mover = mover, origin = origin, userContext = context };

		for ( int i = 0; i < (int)BodyType.bodyTypeCount; ++i )
		{
			DynamicTree_OverlapAABB( world.broadPhase.trees[i], aabb, filter.maskBits, ref worldContext );
		}

	}

	public static void World_SetCustomFilterCallback( WorldId worldId, CustomFilterCallbackFcn fcn, object? context )
	{
		World world = GetWorldFromId( worldId );

		world.customFilterFcn = fcn;
		world.customFilterContext = context;
	}

	public static void World_SetPreSolveCallback( WorldId worldId, PreSolveCallbackFcn preSolveFcn, PreContinuousCallbackFcn preContinuousFcn, object? context )
	{
		World world = GetWorldFromId( worldId );

		world.preSolveFcn = preSolveFcn;
		world.preContinuousFcn = preContinuousFcn;
		world.preSolveContext = context;
	}

	public static void World_SetGravity( WorldId worldId, Vec2 gravity )
	{
		if ( CheckInput( IsValidVec2( gravity ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		world.gravity = gravity;
	}

	public static Vec2 World_GetGravity( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		return world.gravity;
	}

	internal ref struct ExplosionContext : ITreeQueryCallback
	{
		public bool Invoke( int proxyId, ulong userData ) => ExplosionCallback( proxyId, userData, ref this );

		public World world;
		public Pos position;
		public float radius;
		public float falloff;
		public float impulsePerLength;
	}

	private static bool ExplosionCallback( int proxyId, ulong userData, ref ExplosionContext explosionContext )
	{
		int shapeId = (int)userData;
		World world = explosionContext.world;
		ref Shape shape = ref world.shapes[shapeId];
		ref Body body = ref world.bodies[shape.bodyId];
		B2_ASSERT( body.type == BodyType.dynamicBody );

		WorldTransform xf = GetBodyTransformQuick( world, ref body );

		// Re-center the explosion into the shape local frame so distance and direction stay precise
		// far from the origin. Everything below runs in that near-origin frame.
		Vec2 localPosition = InvTransformWorldPoint( xf, explosionContext.position );

		DistanceInput input = default;
		input.proxyA = MakeShapeDistanceProxy( shape );
		input.proxyB = MakeProxy( ref localPosition, 1, 0.0f );
		input.transform = Transform_identity;
		input.useRadii = true;

		SimplexCache cache = default;
		DistanceOutput output = ComputeShapeDistance( ref input, ref cache );

		float radius = explosionContext.radius;
		float falloff = explosionContext.falloff;
		if ( output.distance > radius + falloff )
		{
			return true;
		}

		WakeBody( world, ref body );

		if ( body.setIndex != b2_awakeSet )
		{
			return true;
		}

		Vec2 closestPoint = output.pointA;
		if ( output.distance == 0.0f )
		{
			closestPoint = GetShapeCentroid( shape );
		}

		Vec2 direction = Sub( closestPoint, localPosition );
		if ( LengthSquared( direction ) > 100.0f * FLT_EPSILON * FLT_EPSILON )
		{
			direction = Normalize( direction );
		}
		else
		{
			direction = new Vec2( 1.0f, 0.0f );
		}

		Vec2 localLine = LeftPerp( direction );
		float perimeter = GetShapeProjectedPerimeter( shape, localLine );
		float scale = 1.0f;
		if ( output.distance > radius && falloff > 0.0f )
		{
			scale = ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
		}

		float magnitude = explosionContext.impulsePerLength * perimeter * scale;
		Vec2 impulse = MulSV( magnitude, RotateVector( xf.q, direction ) );

		int localIndex = body.localIndex;
		ref SolverSet set = ref world.solverSets[b2_awakeSet];
		ref BodyState state = ref set.bodyStates[localIndex];
		ref BodySim bodySim = ref set.bodySims[localIndex];
		state.linearVelocity = MulAdd( state.linearVelocity, bodySim.invMass, impulse );

		// Lever arm from the center of mass to the closest point, rotated to world
		Vec2 r = RotateVector( xf.q, Sub( closestPoint, bodySim.localCenter ) );
		state.angularVelocity += bodySim.invInertia * Cross( r, impulse );

		return true;
	}

	public static void World_Explode( WorldId worldId, in ExplosionDef explosionDef )
	{
		ulong maskBits = explosionDef.maskBits;
		Pos position = explosionDef.position;
		float radius = explosionDef.radius;
		float falloff = explosionDef.falloff;
		float impulsePerLength = explosionDef.impulsePerLength;

		if ( CheckInput( IsValidPos( position ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( radius ) && radius >= 0.0f ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( falloff ) && falloff >= 0.0f ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( impulsePerLength ) ) == false )
			{
				return;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		ExplosionContext explosionContext = new ExplosionContext { world = world, position = position, radius = radius, falloff = falloff, impulsePerLength = impulsePerLength };

		// The broad-phase tree is float, so translate a local query box out to world with outward rounding
		float extent = radius + falloff;
		AABB localBox = new AABB { lowerBound = new( -extent, -extent ), upperBound = new( extent, extent ) };
		AABB aabb = OffsetAABB( localBox, position );

		DynamicTree_OverlapAABB( world.broadPhase.trees[BodyType.dynamicBody], aabb, maskBits, ref explosionContext );
	}

	// This is for internal testing. Not optimized.
	public static void World_RebuildStaticTree( WorldId worldId )
	{
		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return;
		}


		DynamicTree staticTree = world.broadPhase.trees[BodyType.staticBody];
		// Moved proxies must retain the flag in the rebuild.
		int movedCount = 0;
		int[]? movedProxies = null;
		int proxyCount = DynamicTree_GetProxyCount( staticTree );
		if ( HasTreeMoved( staticTree ) )
		{
			movedProxies = Alloc<int>( proxyCount );
			movedCount = DynamicTree_GatherMovedProxies( staticTree, movedProxies );
		}

		DynamicTree_Rebuild( staticTree, true );

		for ( int i = 0; i < movedCount; ++i )
		{
			DynamicTree_MarkProxyMovedSerial( staticTree, movedProxies![i] );
		}

		if ( movedProxies != null )
		{
			Free( movedProxies, proxyCount );
		}
	}

#if BOX2D_VALIDATE
	// This validates island graph connectivity for each body
	internal static void ValidateConnectivity( World world )
	{
		Span<Body> bodies = world.bodies.data;
		int bodyCapacity = world.bodies.count;

		for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
		{
			ref Body body = ref bodies[bodyIndex];
			if ( body.id == NullIndex )
			{
				ValidateFreeId( ref world.bodyIdPool, bodyIndex );
				continue;
			}

			ValidateUsedId( ref world.bodyIdPool, bodyIndex );

			B2_ASSERT( bodyIndex == body.id );

			// Need to get the root island because islands are not merged until the next time step
			int bodyIslandId = body.islandId;
			int bodySetIndex = body.setIndex;

			int contactKey = body.headContactKey;
			while ( contactKey != NullIndex )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				ref Contact contact = ref world.contacts[contactId];
				bool touching = ( contact.flags & b2_contactTouchingFlag ) != 0;
				if ( touching )
				{
					if ( bodySetIndex != b2_staticSet )
					{
						int contactIslandId = contact.islandId;
						B2_ASSERT( contactIslandId == bodyIslandId );
					}
				}
				else
				{
					B2_ASSERT( contact.islandId == NullIndex );
				}

				contactKey = contact.edges[edgeIndex].nextKey;
			}

			int jointKey = body.headJointKey;
			while ( jointKey != NullIndex )
			{
				int jointId = jointKey >> 1;
				int edgeIndex = jointKey & 1;

				ref Joint joint = ref world.joints[jointId];
				int otherEdgeIndex = edgeIndex ^ 1;

				ref Body otherBody = ref world.bodies[joint.edges[otherEdgeIndex].bodyId];
				if ( bodySetIndex == b2_disabledSet || otherBody.setIndex == b2_disabledSet )
				{
					B2_ASSERT( joint.islandId == NullIndex );
				}
				else if ( bodySetIndex == b2_staticSet )
				{
					// Intentional nesting
					if ( otherBody.setIndex == b2_staticSet )
					{
						B2_ASSERT( joint.islandId == NullIndex );
					}
				}
				else if ( body.type != BodyType.dynamicBody && otherBody.type != BodyType.dynamicBody )
				{
					B2_ASSERT( joint.islandId == NullIndex );
				}
				else
				{
					int jointIslandId = joint.islandId;
					B2_ASSERT( jointIslandId == bodyIslandId );
				}

				jointKey = joint.edges[edgeIndex].nextKey;
			}
		}
	}

	// Validates solver sets, but not island connectivity
	internal static void ValidateSolverSets( World world )
	{
		B2_ASSERT( GetIdCapacity( ref world.bodyIdPool ) == world.bodies.count );
		B2_ASSERT( GetIdCapacity( ref world.contactIdPool ) == world.contacts.count );
		B2_ASSERT( GetIdCapacity( ref world.jointIdPool ) == world.joints.count );
		B2_ASSERT( GetIdCapacity( ref world.islandIdPool ) == world.islands.count );
		B2_ASSERT( GetIdCapacity( ref world.solverSetIdPool ) == world.solverSets.count );

		int activeSetCount = 0;
		int totalBodyCount = 0;
		int totalJointCount = 0;
		int totalContactCount = 0;
		int totalIslandCount = 0;

		// Validate all solver sets
		int setCount = world.solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			ref SolverSet set = ref world.solverSets.data[setIndex];
			if ( set.setIndex != NullIndex )
			{
				activeSetCount += 1;

				if ( setIndex == b2_staticSet )
				{
					B2_ASSERT( set.contactSims.count == 0 );
					B2_ASSERT( set.islandSims.count == 0 );
					B2_ASSERT( set.bodyStates.count == 0 );
				}
				else if ( setIndex == b2_disabledSet )
				{
					B2_ASSERT( set.islandSims.count == 0 );
					B2_ASSERT( set.bodyStates.count == 0 );
				}
				else if ( setIndex == b2_awakeSet )
				{
					B2_ASSERT( set.bodySims.count == set.bodyStates.count );
					B2_ASSERT( set.jointSims.count == 0 );
				}
				else
				{
					B2_ASSERT( set.bodyStates.count == 0 );
				}

				// Validate bodies
				{
					Span<Body> bodies = world.bodies.data;
					B2_ASSERT( set.bodySims.count >= 0 );
					totalBodyCount += set.bodySims.count;
					for ( int i = 0; i < set.bodySims.count; ++i )
					{
						ref BodySim bodySim = ref set.bodySims.data[i];
						int bodyId = bodySim.bodyId;
						B2_ASSERT( 0 <= bodyId && bodyId < world.bodies.count );
						ref Body body = ref bodies[bodyId];
						B2_ASSERT( body.setIndex == setIndex );
						B2_ASSERT( body.localIndex == i );

						uint syncedFlags = body.flags & ~( b2_isFast | b2_bodyTransientFlags );
						B2_ASSERT( ( bodySim.flags & syncedFlags ) == syncedFlags );

						ref BodyState bodyState = ref GetBodyState( world, ref body );
						if ( Unsafe.IsNullRef( ref bodyState ) == false )
						{
							B2_ASSERT( ( bodyState.flags & syncedFlags ) == syncedFlags );
						}

						if ( body.type == BodyType.dynamicBody )
						{
							B2_ASSERT( ( body.flags & b2_dynamicFlag ) != 0 );
						}

						if ( setIndex == b2_disabledSet )
						{
							B2_ASSERT( body.headContactKey == NullIndex );
						}

						// Validate body shapes
						int prevShapeId = NullIndex;
						int shapeId = body.headShapeId;
						while ( shapeId != NullIndex )
						{
							ref Shape shape = ref world.shapes[shapeId];
							B2_ASSERT( shape.id == shapeId );
							B2_ASSERT( shape.prevShapeId == prevShapeId );

							if ( setIndex == b2_disabledSet )
							{
								B2_ASSERT( shape.proxyKey == NullIndex );
							}
							else if ( setIndex == b2_staticSet )
							{
								B2_ASSERT( B2_PROXY_TYPE( shape.proxyKey ) == BodyType.staticBody );
							}
							else
							{
								BodyType proxyType = B2_PROXY_TYPE( shape.proxyKey );
								B2_ASSERT( proxyType == BodyType.kinematicBody || proxyType == BodyType.dynamicBody );
							}

							prevShapeId = shapeId;
							shapeId = shape.nextShapeId;
						}

						// Validate body contacts
						int contactKey = body.headContactKey;
						while ( contactKey != NullIndex )
						{
							int contactId = contactKey >> 1;
							int edgeIndex = contactKey & 1;

							ref Contact contact = ref world.contacts[contactId];
							B2_ASSERT( contact.setIndex != b2_staticSet );
							B2_ASSERT( contact.edges[0].bodyId == bodyId || contact.edges[1].bodyId == bodyId );
							contactKey = contact.edges[edgeIndex].nextKey;
						}

						// Validate body joints
						int jointKey = body.headJointKey;
						while ( jointKey != NullIndex )
						{
							int jointId = jointKey >> 1;
							int edgeIndex = jointKey & 1;

							ref Joint joint = ref world.joints[jointId];
							int otherEdgeIndex = edgeIndex ^ 1;

							ref Body otherBody = ref world.bodies[joint.edges[otherEdgeIndex].bodyId];
							if ( setIndex == b2_disabledSet || otherBody.setIndex == b2_disabledSet )
							{
								B2_ASSERT( joint.setIndex == b2_disabledSet );
							}
							else if ( setIndex == b2_staticSet && otherBody.setIndex == b2_staticSet )
							{
								B2_ASSERT( joint.setIndex == b2_staticSet );
							}
							else if ( body.type != BodyType.dynamicBody && otherBody.type != BodyType.dynamicBody )
							{
								B2_ASSERT( joint.setIndex == b2_staticSet );
							}
							else if ( setIndex == b2_awakeSet )
							{
								B2_ASSERT( joint.setIndex == b2_awakeSet );
							}
							else if ( setIndex >= b2_firstSleepingSet )
							{
								B2_ASSERT( joint.setIndex == setIndex );
							}

							ref JointSim jointSim = ref GetJointSim( world, ref joint );
							B2_ASSERT( jointSim.jointId == jointId );
							B2_ASSERT( jointSim.bodyIdA == joint.edges[0].bodyId );
							B2_ASSERT( jointSim.bodyIdB == joint.edges[1].bodyId );

							jointKey = joint.edges[edgeIndex].nextKey;
						}
					}
				}

				// Validate contacts
				{
					B2_ASSERT( set.contactSims.count >= 0 );
					totalContactCount += set.contactSims.count;
					for ( int i = 0; i < set.contactSims.count; ++i )
					{
						ref ContactSim contactSim = ref set.contactSims.data[i];
						ref Contact contact = ref world.contacts[contactSim.contactId];
						if ( setIndex == b2_awakeSet )
						{
							// contact should be non-touching if awake
							// or it could be this contact hasn't been transferred yet
							B2_ASSERT( contactSim.manifold.pointCount == 0 ||
									   ( contactSim.simFlags & b2_simStartedTouching ) != 0 );
						}
						B2_ASSERT( contact.setIndex == setIndex );
						B2_ASSERT( contact.colorIndex == NullIndex );
						B2_ASSERT( contact.localIndex == i );

						ref Body edgeBodyA = ref world.bodies[contact.edges[0].bodyId];
						ref Body edgeBodyB = ref world.bodies[contact.edges[1].bodyId];
						B2_ASSERT( contactSim.encodedBodySimA == EncodeBodySimIndex( edgeBodyA ) );
						B2_ASSERT( contactSim.encodedBodySimB == EncodeBodySimIndex( edgeBodyB ) );
					}
				}

				// Validate joints
				{
					B2_ASSERT( set.jointSims.count >= 0 );
					totalJointCount += set.jointSims.count;
					for ( int i = 0; i < set.jointSims.count; ++i )
					{
						ref JointSim jointSim = ref set.jointSims.data[i];
						ref Joint joint = ref world.joints[jointSim.jointId];
						B2_ASSERT( joint.setIndex == setIndex );
						B2_ASSERT( joint.colorIndex == NullIndex );
						B2_ASSERT( joint.localIndex == i );
					}
				}

				// Validate islands
				{
					B2_ASSERT( set.islandSims.count >= 0 );
					totalIslandCount += set.islandSims.count;
					for ( int i = 0; i < set.islandSims.count; ++i )
					{
						ref IslandSim islandSim = ref set.islandSims.data[i];
						ref Island island = ref world.islands[islandSim.islandId];
						B2_ASSERT( island.setIndex == setIndex );
						B2_ASSERT( island.localIndex == i );
					}
				}
			}
			else
			{
				B2_ASSERT( set.bodySims.count == 0 );
				B2_ASSERT( set.contactSims.count == 0 );
				B2_ASSERT( set.jointSims.count == 0 );
				B2_ASSERT( set.islandSims.count == 0 );
				B2_ASSERT( set.bodyStates.count == 0 );
			}
		}

		int setIdCount = GetIdCount( ref world.solverSetIdPool );
		B2_ASSERT( activeSetCount == setIdCount );

		int bodyIdCount = GetIdCount( ref world.bodyIdPool );
		B2_ASSERT( totalBodyCount == bodyIdCount );

		int islandIdCount = GetIdCount( ref world.islandIdPool );
		B2_ASSERT( totalIslandCount == islandIdCount );

		// Validate constraint graph
		for ( int colorIndex = 0; colorIndex < GraphColorCount; ++colorIndex )
		{
			ref GraphColor color = ref world.constraintGraph.colors[colorIndex];
			int bitCount = 0;

			B2_ASSERT( color.contactSims.count >= 0 );

			totalContactCount += color.contactSims.count;
			for ( int i = 0; i < color.contactSims.count; ++i )
			{
				ref ContactSim contactSim = ref color.contactSims.data[i];
				ref Contact contact = ref world.contacts[contactSim.contactId];
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				B2_ASSERT( contactSim.manifold.pointCount > 0 ||
						   ( contactSim.simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
				B2_ASSERT( contact.setIndex == b2_awakeSet );
				B2_ASSERT( contact.colorIndex == colorIndex );
				B2_ASSERT( contact.localIndex == i );

				int bodyIdA = contact.edges[0].bodyId;
				int bodyIdB = contact.edges[1].bodyId;

				ref Body edgeBodyA = ref world.bodies[bodyIdA];
				ref Body edgeBodyB = ref world.bodies[bodyIdB];
				B2_ASSERT( contactSim.encodedBodySimA == EncodeBodySimIndex( edgeBodyA ) );
				B2_ASSERT( contactSim.encodedBodySimB == EncodeBodySimIndex( edgeBodyB ) );

				if ( colorIndex < OverflowIndex )
				{
					B2_ASSERT( GetBit( ref color.bodySet, bodyIdA ) == ( edgeBodyA.type == BodyType.dynamicBody ) );
					B2_ASSERT( GetBit( ref color.bodySet, bodyIdB ) == ( edgeBodyB.type == BodyType.dynamicBody ) );

					bitCount += edgeBodyA.type == BodyType.dynamicBody ? 1 : 0;
					bitCount += edgeBodyB.type == BodyType.dynamicBody ? 1 : 0;
				}
			}

			B2_ASSERT( color.jointSims.count >= 0 );
			totalJointCount += color.jointSims.count;
			for ( int i = 0; i < color.jointSims.count; ++i )
			{
				ref JointSim jointSim = ref color.jointSims.data[i];
				ref Joint joint = ref world.joints[jointSim.jointId];
				B2_ASSERT( joint.setIndex == b2_awakeSet );
				B2_ASSERT( joint.colorIndex == colorIndex );
				B2_ASSERT( joint.localIndex == i );

				int bodyIdA = joint.edges[0].bodyId;
				int bodyIdB = joint.edges[1].bodyId;

				if ( colorIndex < OverflowIndex )
				{
					ref Body bodyA = ref world.bodies[bodyIdA];
					ref Body bodyB = ref world.bodies[bodyIdB];
					B2_ASSERT( GetBit( ref color.bodySet, bodyIdA ) == ( bodyA.type == BodyType.dynamicBody ) );
					B2_ASSERT( GetBit( ref color.bodySet, bodyIdB ) == ( bodyB.type == BodyType.dynamicBody ) );

					bitCount += bodyA.type == BodyType.dynamicBody ? 1 : 0;
					bitCount += bodyB.type == BodyType.dynamicBody ? 1 : 0;
				}
			}

			// Validate the bit population for this graph color
			B2_ASSERT( bitCount == CountSetBits( ref color.bodySet ) );
		}

		int contactIdCount = GetIdCount( ref world.contactIdPool );
		B2_ASSERT( totalContactCount == contactIdCount );
		B2_ASSERT( totalContactCount == (int)world.broadPhase.pairSet.count );

		int jointIdCount = GetIdCount( ref world.jointIdPool );
		B2_ASSERT( totalJointCount == jointIdCount );

	// Validate shapes
	// This is very slow on compounds
#if false
		int shapeCapacity = world.shapes.count;
		for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
		{
			ref Shape shape = ref world.shapes.data[shapeIndex];
			if (shape.id != shapeIndex)
			{
				continue;
			}

			ref Body body = ref world.bodies[shape.bodyId];
			ref SolverSet set = ref world.solverSets[body.setIndex];
			ref BodySim bodySim = ref set.bodySims[body.localIndex];
			B2_ASSERT(bodySim.bodyId == shape.bodyId);

			bool found = false;
			int shapeCount = 0;
			int index = body.headShapeId;
			while (index != NullIndex)
			{
				ref Shape s = ref world.shapes[index];
				if (index == shapeIndex)
				{
					found = true;
				}

				index = s.nextShapeId;
				shapeCount += 1;
			}

			B2_ASSERT(found);
			B2_ASSERT(shapeCount == body.shapeCount);
		}
#endif
	}

	// Validate contact touching status.
	internal static void ValidateContacts( World world )
	{
		int contactCount = world.contacts.count;
		B2_ASSERT( contactCount == GetIdCapacity( ref world.contactIdPool ) );
		int allocatedContactCount = 0;

		for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
		{
			ref Contact contact = ref world.contacts[contactIndex];
			if ( contact.contactId == NullIndex )
			{
				continue;
			}

			B2_ASSERT( contact.contactId == contactIndex );

			allocatedContactCount += 1;

			bool touching = ( contact.flags & b2_contactTouchingFlag ) != 0;

			int setId = contact.setIndex;

			if ( setId == b2_awakeSet )
			{
				if ( touching )
				{
					B2_ASSERT( 0 <= contact.colorIndex && contact.colorIndex < GraphColorCount );
				}
				else
				{
					B2_ASSERT( contact.colorIndex == NullIndex );
				}
			}
			else if ( setId >= b2_firstSleepingSet )
			{
				// Only touching contacts allowed in a sleeping set
				B2_ASSERT( touching == true );
			}
			else
			{
				// Sleeping and non-touching contacts belong in the disabled set
				B2_ASSERT( touching == false && setId == b2_disabledSet );
			}

			ref ContactSim contactSim = ref GetContactSim( world, ref contact );
			B2_ASSERT( contactSim.contactId == contactIndex );
			B2_ASSERT( contactSim.bodyIdA == contact.edges[0].bodyId );
			B2_ASSERT( contactSim.bodyIdB == contact.edges[1].bodyId );

			bool simTouching = ( contactSim.simFlags & b2_simTouchingFlag ) != 0;
			B2_ASSERT( touching == simTouching );

			B2_ASSERT( 0 <= contactSim.manifold.pointCount && contactSim.manifold.pointCount <= 2 );
		}

		int contactIdCount = GetIdCount( ref world.contactIdPool );
		B2_ASSERT( allocatedContactCount == contactIdCount );
	}

#else

	internal static void ValidateConnectivity( World world )
	{
	}

	internal static void ValidateSolverSets( World world )
	{
	}

	internal static void ValidateContacts( World world )
	{
	}

#endif

}
