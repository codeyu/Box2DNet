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


	// Contacts and determinism
	// A deterministic simulation requires contacts to exist in the same order in Island no matter the thread count.
	// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
	//
	// Creation:
	// - Contacts are created using results from UpdateBroadPhasePairs
	// - These results are ordered according to the order of the broad-phase move array
	// - The move array is ordered according to the shape creation order using a bitset.
	// - The island/shape/body order is determined by creation order
	// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
	//
	// Island linking:
	// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
	// - Awake contacts are solved in parallel and they generate contact state changes.
	// - These state changes may link islands together using union find.
	// - The state changes are ordered using a bit array that encompasses all contacts
	// - As long as contacts are created in deterministic order, island link order is deterministic.
	// - This keeps the order of contacts in islands deterministic

	// Manifold functions should compute important results in local space to improve precision. However, this
	// interface function takes two world transforms instead of a relative transform for these reasons:
	//
	// First:
	// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
	// solver does not need to access static body transforms. Not even in constraint preparation. This approach
	// has world space vectors yet retains precision.
	//
	// Second:
	// ManifoldPoint::point is very useful for debugging and it is in world space.
	//
	// Third:
	// The user may call the manifold functions directly and they should be easy to use and have easy to use
	// results.

	private static ref Contact GetContactFullId( World world, ContactId contactId )
	{
		int id = contactId.index1 - 1;
		ref Contact contact = ref world.contacts[id];
		B2_ASSERT( contact.contactId == id && contact.generation == contactId.generation );
		return ref contact;
	}

	public static ContactData Contact_GetData( ContactId contactId )
	{
		World world = GetWorld( contactId.world0 );
		ref Contact contact = ref GetContactFullId( world, contactId );
		ref ContactSim contactSim = ref GetContactSim( world, ref contact );
		ref readonly Shape shapeA = ref world.shapes[contact.shapeIdA];
		ref readonly Shape shapeB = ref world.shapes[contact.shapeIdB];
		ContactData data = new ContactData { contactId = contactId, shapeIdA = new() { index1 = shapeA.id + 1, world0 = (ushort)contactId.world0, generation = shapeA.generation }, shapeIdB = new() { index1 = shapeB.id + 1, world0 = (ushort)contactId.world0, generation = shapeB.generation }, manifold = contactSim.manifold };

		return data;
	}

	internal delegate LocalManifold ManifoldFcn( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache );

	internal struct ContactRegister
	{
		public ManifoldFcn? fcn;
		public bool primary;
	}

	private static LocalManifold CircleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideCircles( ref shapeA.circle, ref shapeB.circle, xf );
	}

	private static LocalManifold CapsuleAndCircleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideCapsuleAndCircle( ref shapeA.capsule, ref shapeB.circle, xf );
	}

	private static LocalManifold CapsuleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideCapsules( ref shapeA.capsule, ref shapeB.capsule, xf );
	}

	private static LocalManifold PolygonAndCircleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollidePolygonAndCircle( ref shapeA.polygon, ref shapeB.circle, xf );
	}

	private static LocalManifold PolygonAndCapsuleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollidePolygonAndCapsule( ref shapeA.polygon, ref shapeB.capsule, xf );
	}

	private static LocalManifold PolygonManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollidePolygons( ref shapeA.polygon, ref shapeB.polygon, xf );
	}

	private static LocalManifold SegmentAndCircleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideSegmentAndCircle( ref shapeA.segment, ref shapeB.circle, xf );
	}

	private static LocalManifold SegmentAndCapsuleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideSegmentAndCapsule( ref shapeA.segment, ref shapeB.capsule, xf );
	}

	private static LocalManifold SegmentAndPolygonManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideSegmentAndPolygon( ref shapeA.segment, ref shapeB.polygon, xf );
	}

	private static LocalManifold ChainSegmentAndCircleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideChainSegmentAndCircle( ref shapeA.chainSegment, ref shapeB.circle, xf );
	}

	private static LocalManifold ChainSegmentAndCapsuleManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideChainSegmentAndCapsule( ref shapeA.chainSegment, ref shapeB.capsule, xf, ref cache );
	}

	private static LocalManifold ChainSegmentAndPolygonManifold( in Shape shapeA, in Shape shapeB, Transform xf, ref SimplexCache cache )
	{
		return CollideChainSegmentAndPolygon( ref shapeA.chainSegment, ref shapeB.polygon, xf, ref cache );
	}

	// This works with DLL hot reloading.
	private static readonly ContactRegister[,] contactRegistry = MakeContactRegistry();

	private static ContactRegister[,] MakeContactRegistry()
	{
		ContactRegister[,] r = new ContactRegister[(int)ShapeType.shapeTypeCount, (int)ShapeType.shapeTypeCount];
		r[(int)ShapeType.circleShape, (int)ShapeType.circleShape] = new ContactRegister { fcn = CircleManifold, primary = true };
		r[(int)ShapeType.circleShape, (int)ShapeType.capsuleShape] = new ContactRegister { fcn = CapsuleAndCircleManifold, primary = false };
		r[(int)ShapeType.circleShape, (int)ShapeType.segmentShape] = new ContactRegister { fcn = SegmentAndCircleManifold, primary = false };
		r[(int)ShapeType.circleShape, (int)ShapeType.polygonShape] = new ContactRegister { fcn = PolygonAndCircleManifold, primary = false };
		r[(int)ShapeType.circleShape, (int)ShapeType.chainSegmentShape] = new ContactRegister { fcn = ChainSegmentAndCircleManifold, primary = false };
		r[(int)ShapeType.capsuleShape, (int)ShapeType.circleShape] = new ContactRegister { fcn = CapsuleAndCircleManifold, primary = true };
		r[(int)ShapeType.capsuleShape, (int)ShapeType.capsuleShape] = new ContactRegister { fcn = CapsuleManifold, primary = true };
		r[(int)ShapeType.capsuleShape, (int)ShapeType.segmentShape] = new ContactRegister { fcn = SegmentAndCapsuleManifold, primary = false };
		r[(int)ShapeType.capsuleShape, (int)ShapeType.polygonShape] = new ContactRegister { fcn = PolygonAndCapsuleManifold, primary = false };
		r[(int)ShapeType.capsuleShape, (int)ShapeType.chainSegmentShape] = new ContactRegister { fcn = ChainSegmentAndCapsuleManifold, primary = false };
		r[(int)ShapeType.segmentShape, (int)ShapeType.circleShape] = new ContactRegister { fcn = SegmentAndCircleManifold, primary = true };
		r[(int)ShapeType.segmentShape, (int)ShapeType.capsuleShape] = new ContactRegister { fcn = SegmentAndCapsuleManifold, primary = true };
		r[(int)ShapeType.segmentShape, (int)ShapeType.polygonShape] = new ContactRegister { fcn = SegmentAndPolygonManifold, primary = true };
		r[(int)ShapeType.polygonShape, (int)ShapeType.circleShape] = new ContactRegister { fcn = PolygonAndCircleManifold, primary = true };
		r[(int)ShapeType.polygonShape, (int)ShapeType.capsuleShape] = new ContactRegister { fcn = PolygonAndCapsuleManifold, primary = true };
		r[(int)ShapeType.polygonShape, (int)ShapeType.segmentShape] = new ContactRegister { fcn = SegmentAndPolygonManifold, primary = false };
		r[(int)ShapeType.polygonShape, (int)ShapeType.polygonShape] = new ContactRegister { fcn = PolygonManifold, primary = true };
		r[(int)ShapeType.polygonShape, (int)ShapeType.chainSegmentShape] = new ContactRegister { fcn = ChainSegmentAndPolygonManifold, primary = false };
		r[(int)ShapeType.chainSegmentShape, (int)ShapeType.circleShape] = new ContactRegister { fcn = ChainSegmentAndCircleManifold, primary = true };
		r[(int)ShapeType.chainSegmentShape, (int)ShapeType.capsuleShape] = new ContactRegister { fcn = ChainSegmentAndCapsuleManifold, primary = true };
		r[(int)ShapeType.chainSegmentShape, (int)ShapeType.polygonShape] = new ContactRegister { fcn = ChainSegmentAndPolygonManifold, primary = true };
		return r;
	}

	internal static bool CanCollide( ShapeType typeA, ShapeType typeB )
	{
		return contactRegistry[(int)typeA, (int)typeB].fcn != null;
	}

	// WARNING: this should never fail to create a contact because the pair already exists in the pairSet.
	internal static void CreateContact( World world, ref Shape shapeA, ref Shape shapeB )
	{
		ShapeType type1 = shapeA.type;
		ShapeType type2 = shapeB.type;

		B2_ASSERT( 0 <= type1 && type1 < ShapeType.shapeTypeCount );
		B2_ASSERT( 0 <= type2 && type2 < ShapeType.shapeTypeCount );

		if ( contactRegistry[(int)type1, (int)type2].fcn == null )
		{
			// For example, no segment vs segment collision
			return;
		}

		if ( contactRegistry[(int)type1, (int)type2].primary == false )
		{
			// flip order
			CreateContact( world, ref shapeB, ref shapeA );
			return;
		}

		ref Body bodyA = ref world.bodies[shapeA.bodyId];
		ref Body bodyB = ref world.bodies[shapeB.bodyId];
		B2_ASSERT( bodyA.setIndex != b2_disabledSet && bodyB.setIndex != b2_disabledSet );
		B2_ASSERT( bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet );

		int setIndex;
		if ( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet )
		{
			setIndex = b2_awakeSet;
		}
		else
		{
			// sleeping and non-touching contacts live in the disabled set
			// later if this set is found to be touching then the sleeping
			// islands will be linked and the contact moved to the merged island
			setIndex = b2_disabledSet;
		}

		ref SolverSet set = ref world.solverSets[setIndex];
		// Create contact key and contact
		int contactId = AllocId( ref world.contactIdPool );
		if ( contactId == world.contacts.count )
		{
			world.contacts.Push( default(Contact) );
		}

		int shapeIdA = shapeA.id;
		int shapeIdB = shapeB.id;

		ref Contact contact = ref world.contacts[contactId];
		contact.contactId = contactId;
		contact.generation += 1;
		contact.setIndex = setIndex;
		contact.colorIndex = NullIndex;
		contact.localIndex = set.contactSims.count;
		contact.islandId = NullIndex;
		contact.islandIndex = NullIndex;
		contact.shapeIdA = shapeIdA;
		contact.shapeIdB = shapeIdB;
		contact.flags = 0;

		// Both bodies must enable recycling
		if ( ( bodyA.flags & b2_bodyEnableContactRecycling ) != 0 && ( bodyB.flags & b2_bodyEnableContactRecycling ) != 0 )
		{
			contact.flags |= b2_contactRecycleFlag;
		}

		B2_ASSERT( shapeA.sensorIndex == NullIndex && shapeB.sensorIndex == NullIndex );

		if ( shapeA.enableContactEvents || shapeB.enableContactEvents )
		{
			contact.flags |= b2_contactEnableContactEvents;
		}

		// Connect to body A
		{
			contact.edges[0].bodyId = shapeA.bodyId;
			contact.edges[0].prevKey = NullIndex;
			contact.edges[0].nextKey = bodyA.headContactKey;

			int keyA = ( contactId << 1 ) | 0;
			int headContactKey = bodyA.headContactKey;
			if ( headContactKey != NullIndex )
			{
				ref Contact headContact = ref world.contacts[headContactKey >> 1];
				headContact.edges[headContactKey & 1].prevKey = keyA;
			}
			bodyA.headContactKey = keyA;
			bodyA.contactCount += 1;
		}

		// Connect to body B
		{
			contact.edges[1].bodyId = shapeB.bodyId;
			contact.edges[1].prevKey = NullIndex;
			contact.edges[1].nextKey = bodyB.headContactKey;

			int keyB = ( contactId << 1 ) | 1;
			int headContactKey = bodyB.headContactKey;
			if ( bodyB.headContactKey != NullIndex )
			{
				ref Contact headContact = ref world.contacts[headContactKey >> 1];
				headContact.edges[headContactKey & 1].prevKey = keyB;
			}
			bodyB.headContactKey = keyB;
			bodyB.contactCount += 1;
		}

		// Add to pair set for fast lookup.
		ulong pairKey = HashSet64.ShapePairKey( shapeIdA, shapeIdB );
		AddKey( ref world.broadPhase.pairSet, pairKey );

		// Contacts are created as non-touching. Later if they are found to be touching
		// they will link islands and be moved into the constraint graph.
		ref ContactSim contactSim = ref set.contactSims.Emplace();
		contactSim.contactId = contactId;

#if BOX2D_VALIDATE
		contactSim.bodyIdA = shapeA.bodyId;
		contactSim.bodyIdB = shapeB.bodyId;
#endif

		contactSim.encodedBodySimA = EncodeBodySimIndex( bodyA );
		contactSim.encodedBodySimB = EncodeBodySimIndex( bodyB );
		contactSim.invMassA = 0.0f;
		contactSim.invIA = 0.0f;
		contactSim.invMassB = 0.0f;
		contactSim.invIB = 0.0f;
		contactSim.shapeIdA = shapeIdA;
		contactSim.shapeIdB = shapeIdB;
		contactSim.cache = emptySimplexCache;
		contactSim.manifold = default(Manifold);

		// These get updated in the narrow phase, but these are needed for first touch
		if ( world.frictionCallback == null )
		{
			contactSim.friction = MixFriction( shapeA.material.friction, shapeB.material.friction );
		}
		else
		{
			contactSim.friction = world.frictionCallback( shapeA.material.friction, shapeA.material.userMaterialId,
															shapeB.material.friction, shapeB.material.userMaterialId );
		}

		if ( world.restitutionCallback == null )
		{
			contactSim.restitution = MixRestitution( shapeA.material.restitution, shapeB.material.restitution );
		}
		else
		{
			contactSim.restitution = world.restitutionCallback( shapeA.material.restitution, shapeA.material.userMaterialId,
																  shapeB.material.restitution, shapeB.material.userMaterialId );
		}

		contactSim.tangentSpeed = 0.0f;
		contactSim.simFlags = contact.flags;

		if ( shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents )
		{
			contactSim.simFlags |= b2_simEnablePreSolveEvents;
		}
	}

	// A contact is destroyed when:
	// - broad-phase proxies stop overlapping
	// - a body is destroyed
	// - a body is disabled
	// - a body changes type from dynamic to kinematic or static
	// - a shape is destroyed
	// - contact filtering is modified
	internal static void DestroyContact( World world, ref Contact contact )
	{
		// Remove pair from set
		ulong pairKey = HashSet64.ShapePairKey( contact.shapeIdA, contact.shapeIdB );
		RemoveKey( ref world.broadPhase.pairSet, pairKey );

		ref ContactEdge edgeA = ref contact.edges[0];
		ref ContactEdge edgeB = ref contact.edges[1];
		int bodyIdA = edgeA.bodyId;
		int bodyIdB = edgeB.bodyId;
		ref Body bodyA = ref world.bodies[bodyIdA];
		ref Body bodyB = ref world.bodies[bodyIdB];
		uint flags = contact.flags;
		bool touching = ( flags & b2_contactTouchingFlag ) != 0;

		// End touch event
		if ( touching && ( flags & b2_contactEnableContactEvents ) != 0 )
		{
			ushort worldId = world.worldId;
			ref readonly Shape shapeA = ref world.shapes[contact.shapeIdA];
			ref readonly Shape shapeB = ref world.shapes[contact.shapeIdB];
			ShapeId shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = worldId, generation = shapeA.generation };
			ShapeId shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = worldId, generation = shapeB.generation };

			ContactId contactIdInner = new ContactId { index1 = contact.contactId + 1, world0 = world.worldId, padding = 0, generation = contact.generation };

			ContactEndTouchEvent @event = new ContactEndTouchEvent { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactIdInner };

			world.contactEndEvents[world.endEventArrayIndex].Push( @event );
		}

		// Remove from body A
		if ( edgeA.prevKey != NullIndex )
		{
			ref Contact prevContact = ref world.contacts[edgeA.prevKey >> 1];
			ref ContactEdge prevEdge = ref prevContact.edges[( edgeA.prevKey & 1 )];
			prevEdge.nextKey = edgeA.nextKey;
		}

		if ( edgeA.nextKey != NullIndex )
		{
			ref Contact nextContact = ref world.contacts[edgeA.nextKey >> 1];
			ref ContactEdge nextEdge = ref nextContact.edges[( edgeA.nextKey & 1 )];
			nextEdge.prevKey = edgeA.prevKey;
		}

		int contactId = contact.contactId;

		int edgeKeyA = ( contactId << 1 ) | 0;
		if ( bodyA.headContactKey == edgeKeyA )
		{
			bodyA.headContactKey = edgeA.nextKey;
		}

		bodyA.contactCount -= 1;

		// Remove from body B
		if ( edgeB.prevKey != NullIndex )
		{
			ref Contact prevContact = ref world.contacts[edgeB.prevKey >> 1];
			ref ContactEdge prevEdge = ref prevContact.edges[( edgeB.prevKey & 1 )];
			prevEdge.nextKey = edgeB.nextKey;
		}

		if ( edgeB.nextKey != NullIndex )
		{
			ref Contact nextContact = ref world.contacts[edgeB.nextKey >> 1];
			ref ContactEdge nextEdge = ref nextContact.edges[( edgeB.nextKey & 1 )];
			nextEdge.prevKey = edgeB.prevKey;
		}

		int edgeKeyB = ( contactId << 1 ) | 1;
		if ( bodyB.headContactKey == edgeKeyB )
		{
			bodyB.headContactKey = edgeB.nextKey;
		}

		bodyB.contactCount -= 1;

		// Remove contact from island.
		if ( contact.islandId != NullIndex )
		{
			UnlinkContact( world, ref contact );
		}

		if ( contact.colorIndex != NullIndex )
		{
			// contact is an active constraint
			B2_ASSERT( contact.setIndex == b2_awakeSet );
			RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex );
		}
		else
		{
			// contact is non-touching or is sleeping
			B2_ASSERT( contact.setIndex != b2_awakeSet || ( contact.flags & b2_contactTouchingFlag ) == 0 );
			ref SolverSet set = ref world.solverSets[contact.setIndex];
			int movedIndex = set.contactSims.RemoveSwap( contact.localIndex );
			if ( movedIndex != NullIndex )
			{
				ref ContactSim movedContactSim = ref set.contactSims.data[contact.localIndex];
				ref Contact movedContact = ref world.contacts[movedContactSim.contactId];
				movedContact.localIndex = contact.localIndex;
			}
		}

		// Free contact and id (preserve generation)
		contact.contactId = NullIndex;
		contact.setIndex = NullIndex;
		contact.colorIndex = NullIndex;
		contact.localIndex = NullIndex;
		FreeId( ref world.contactIdPool, contactId );

		if ( touching )
		{
			WakeBody( world, ref bodyA );
			WakeBody( world, ref bodyB );
		}
	}

	internal static ref ContactSim GetContactSim( World world, ref Contact contact )
	{
		if ( contact.setIndex == b2_awakeSet && contact.colorIndex != NullIndex )
		{
			// contact lives in constraint graph
			B2_ASSERT( 0 <= contact.colorIndex && contact.colorIndex < GraphColorCount );
			ref GraphColor color = ref world.constraintGraph.colors[contact.colorIndex];
			return ref color.contactSims[contact.localIndex];
		}

		ref SolverSet set = ref world.solverSets[contact.setIndex];
		return ref set.contactSims[contact.localIndex];
	}

	// Update the contact manifold and touching status.
	// Note: do not assume the shape AABBs are overlapping or are valid.
	internal static bool UpdateContact( World world, ref ContactSim contactSim, ref Shape shapeA, WorldTransform transformA, Vec2 centerOffsetA, ref Shape shapeB, WorldTransform transformB, Vec2 centerOffsetB )
	{
		// Save old manifold
		Manifold oldManifold = contactSim.manifold;

		// Compute the manifold in frame A, then marshal it to world anchors relative to each shape origin.
		// The relative pose differences the two world positions before the narrow phase runs in frame A,
		// so precision is retained far from the origin.
		// anchorB = worldPoint - pB = rot(qA, localAnchorA) + pA - pB = anchorA + (pA - pB)
		Transform relativeTransform = InvMulWorldTransforms( transformA, transformB );
		ref ManifoldFcn fcn = ref contactRegistry[(int)shapeA.type, (int)shapeB.type].fcn;
		LocalManifold local = fcn( shapeA, shapeB, relativeTransform, ref contactSim.cache );

		contactSim.manifold = default(Manifold);
		contactSim.manifold.normal = RotateVector( transformA.q, local.normal );
		contactSim.manifold.pointCount = local.pointCount;

		Vec2 originDelta = SubPos( transformA.p, transformB.p );
		for ( int i = 0; i < local.pointCount; ++i )
		{
			ref ManifoldPoint mp = ref contactSim.manifold.points[i];
			mp.anchorA = RotateVector( transformA.q, local.points[i].point );
			mp.anchorB = Add( mp.anchorA, originDelta );
			mp.separation = local.points[i].separation;
			mp.id = local.points[i].id;
		}

		// Keep these updated in case the values on the shapes are modified
		if ( world.frictionCallback == null )
		{
			contactSim.friction = MixFriction( shapeA.material.friction, shapeB.material.friction );
		}
		else
		{
			contactSim.friction = world.frictionCallback( shapeA.material.friction, shapeA.material.userMaterialId,
															shapeB.material.friction, shapeB.material.userMaterialId );
		}

		if ( world.restitutionCallback == null )
		{
			contactSim.restitution = MixRestitution( shapeA.material.restitution, shapeB.material.restitution );
		}
		else
		{
			contactSim.restitution = world.restitutionCallback( shapeA.material.restitution, shapeA.material.userMaterialId,
																  shapeB.material.restitution, shapeB.material.userMaterialId );
		}

		if ( shapeA.material.rollingResistance > 0.0f || shapeB.material.rollingResistance > 0.0f )
		{
			float radiusA = GetShapeRadius( shapeA );
			float radiusB = GetShapeRadius( shapeB );
			float maxRadius = MaxFloat( radiusA, radiusB );
			contactSim.rollingResistance =
				MaxFloat( shapeA.material.rollingResistance, shapeB.material.rollingResistance ) * maxRadius;
		}
		else
		{
			contactSim.rollingResistance = 0.0f;
		}

		contactSim.tangentSpeed = shapeA.material.tangentSpeed + shapeB.material.tangentSpeed;

		int pointCount = contactSim.manifold.pointCount;
		bool touching = pointCount > 0;

		if ( touching && world.preSolveFcn != null && ( contactSim.simFlags & b2_simEnablePreSolveEvents ) != 0 )
		{
			ShapeId shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = world.worldId, generation = shapeA.generation };
			ShapeId shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = world.worldId, generation = shapeB.generation };

			// This call assumes thread safety.
			world.preSolveFcn( shapeIdA, shapeIdB, ref contactSim.manifold, world.preSolveContext );

			// Keep these current.
			pointCount = contactSim.manifold.pointCount;
			touching = pointCount > 0;

			// Mark recycling dirty.
			if ( touching == false )
			{
				contactSim.simFlags &= ~b2_simRelativeTransformValid;
			}
		}

		if ( touching && ( shapeA.enableHitEvents || shapeB.enableHitEvents ) )
		{
			contactSim.simFlags |= b2_simEnableHitEvent;
		}
		else
		{
			contactSim.simFlags &= ~b2_simEnableHitEvent;
		}

		if ( pointCount > 0 )
		{
			contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			int unmatchedCount = 0;
			for ( int i = 0; i < pointCount; ++i )
			{
				ref ManifoldPoint mp2 = ref contactSim.manifold.points[i];
				// shift anchors to be center of mass relative
				mp2.anchorA = Sub( mp2.anchorA, centerOffsetA );
				mp2.anchorB = Sub( mp2.anchorB, centerOffsetB );

				mp2.normalImpulse = 0.0f;
				mp2.tangentImpulse = 0.0f;
				mp2.totalNormalImpulse = 0.0f;
				mp2.normalVelocity = 0.0f;
				mp2.persisted = false;

				ushort id2 = mp2.id;

				for ( int j = 0; j < oldManifold.pointCount; ++j )
				{
					ref ManifoldPoint mp1 = ref oldManifold.points[j];
					if ( mp1.id == id2 )
					{
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						mp2.persisted = true;

						break;
					}
				}

				unmatchedCount += mp2.persisted ? 0 : 1;
			}
		}

#if false
			// todo I haven't found an improvement from this yet
			// If there are unmatched new contact points, apply any left over old impulse.
			if (unmatchedCount > 0)
			{
				float unmatchedNormalImpulse = 0.0f;
				float unmatchedTangentImpulse = 0.0f;
				for (int i = 0; i < oldManifold.pointCount; ++i)
				{
					ref ManifoldPoint mp = ref oldManifold.points[i];
					unmatchedNormalImpulse += mp.normalImpulse;
					unmatchedTangentImpulse += mp.tangentImpulse;
				}

				float inverse = 1.0f / unmatchedCount;
				unmatchedNormalImpulse *= inverse;
				unmatchedTangentImpulse *= inverse;

				for ( int i = 0; i < pointCount; ++i )
				{
					ref ManifoldPoint mp2 = ref contactSim.manifold.points[i];
					if (mp2.persisted)
					{
						continue;
					}

					mp2.normalImpulse = unmatchedNormalImpulse;
					mp2.tangentImpulse = unmatchedTangentImpulse;
				}
			}
#endif

		if ( touching )
		{
			contactSim.simFlags |= b2_simTouchingFlag;
		}
		else
		{
			contactSim.simFlags &= ~b2_simTouchingFlag;
		}

		return touching;
	}

}
