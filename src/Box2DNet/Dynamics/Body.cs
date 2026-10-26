// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	private static void LimitVelocity( ref BodyState state, float maxLinearSpeed )
	{
		float v2 = LengthSquared( state.linearVelocity );
		if ( v2 > maxLinearSpeed * maxLinearSpeed )
		{
			state.linearVelocity = MulSV( maxLinearSpeed / MathF.Sqrt( v2 ), state.linearVelocity );
		}
	}

	// Refresh the body sim index on associated contact sims.
	internal static void RefreshBodyContactIndices( World world, ref Body body )
	{
		int encodedIndex = EncodeBodySimIndex( body );

		int contactKey = body.headContactKey;
		while ( contactKey != NullIndex )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			ref ContactSim contactSim = ref GetContactSim( world, ref contact );
			if ( edgeIndex == 0 )
			{
				contactSim.encodedBodySimA = encodedIndex;
			}
			else
			{
				contactSim.encodedBodySimB = encodedIndex;
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}
	}

	// Get a validated body from a world using an id.
	internal static ref Body GetBodyFullId( World world, BodyId bodyId )
	{
		B2_ASSERT( Body_IsValid( bodyId ) );

		// id index starts at one so that zero can represent null
		return ref world.bodies[bodyId.index1 - 1];
	}

	internal static WorldTransform GetBodyTransformQuick( World world, ref Body body )
	{
		ref SolverSet set = ref world.solverSets[body.setIndex];
		ref BodySim bodySim = ref set.bodySims[body.localIndex];
		return bodySim.transform;
	}

	internal static WorldTransform GetBodyTransform( World world, int bodyId )
	{
		ref Body body = ref world.bodies[bodyId];
		return GetBodyTransformQuick( world, ref body );
	}

	// Create a BodyId from a raw id.
	internal static BodyId MakeBodyId( World world, int bodyId )
	{
		ref Body body = ref world.bodies[bodyId];
		return new BodyId { index1 = bodyId + 1, world0 = world.worldId, generation = body.generation };
	}

	// Sync the body flags when something external changes them,
	// such as the user changing the motion locks.
	internal static void SyncBodyFlags( World world, ref Body body )
	{
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		// Preserve the sim only flags: fast for contact recycling, time of impact for debug draw.
		bodySim.flags = ( bodySim.flags & ( b2_isFast | b2_hadTimeOfImpact ) ) | ( body.flags & ~b2_bodyTransientFlags );

		ref BodyState bodyState = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref bodyState ) == false )
		{
			bodyState.flags = body.flags & ~b2_bodyTransientFlags;
		}
	}

	private static void CreateIslandForBody( World world, int setIndex, ref Body body )
	{
		B2_ASSERT( body.islandId == NullIndex );
		B2_ASSERT( setIndex != b2_disabledSet );

		ref Island island = ref CreateIsland( world, setIndex );
		island.bodies.Push( body.id );
		body.islandId = island.islandId;
		body.islandIndex = 0;

		ValidateIsland( world, island.islandId );
	}

	private static void RemoveBodyFromIsland( World world, ref Body body )
	{
		if ( body.islandId == NullIndex )
		{
			B2_ASSERT( body.islandIndex == NullIndex );
			return;
		}

		int islandId = body.islandId;
		ref Island island = ref world.islands[islandId];
		{
			int localIndex = body.islandIndex;
			int movedBodyId = island.bodies.data[island.bodies.count - 1];
			island.bodies.data[localIndex] = movedBodyId;
			B2_VALIDATE( world.bodies.data[movedBodyId].islandIndex == island.bodies.count - 1 );
			world.bodies.data[movedBodyId].islandIndex = localIndex;
			island.bodies.count -= 1;
		}

		if ( island.bodies.count == 0 )
		{
			// Destroy empty island
			B2_ASSERT( island.contacts.count == 0 );
			B2_ASSERT( island.joints.count == 0 );

			// Free the island
			DestroyIsland( world, island.islandId );
		}
		else
		{
			ValidateIsland( world, islandId );
		}

		body.islandId = NullIndex;
		body.islandIndex = NullIndex;
	}

	private static void DestroyBodyContacts( World world, ref Body body )
	{
		// Destroy the attached contacts
		int edgeKey = body.headContactKey;
		while ( edgeKey != NullIndex )
		{
			int contactId = edgeKey >> 1;
			int edgeIndex = edgeKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			edgeKey = contact.edges[edgeIndex].nextKey;
			DestroyContact( world, ref contact );
		}

		ValidateSolverSets( world );
	}

	public static BodyId CreateBody( WorldId worldId, in BodyDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidPos( def.position ) ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidRot( def.rotation ) ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidVec2( def.linearVelocity ) ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.angularVelocity ) ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.linearDamping ) && def.linearDamping >= 0.0f ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.angularDamping ) && def.angularDamping >= 0.0f ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.sleepThreshold ) && def.sleepThreshold >= 0.0f ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.safetyFactor ) && def.safetyFactor >= 0.0f ) == false )
			{
				return nullBodyId;
			};
		if ( CheckInput( IsValidFloat( def.gravityScale ) ) == false )
			{
				return nullBodyId;
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return nullBodyId;
		}

		bool isAwake = ( def.isAwake || def.enableSleep == false ) && def.isEnabled;

		// determine the solver set
		int setId;
		if ( def.isEnabled == false )
		{
			// any body type can be disabled
			setId = b2_disabledSet;
		}
		else if ( def.type == BodyType.staticBody )
		{
			setId = b2_staticSet;
		}
		else if ( isAwake == true )
		{
			setId = b2_awakeSet;
		}
		else
		{
			// new set for a sleeping body in its own island
			setId = AllocId( ref world.solverSetIdPool );
			if ( setId == world.solverSets.count )
			{
				// Create a zero initialized solver set. All sub-arrays are also zero initialized.
				world.solverSets.Push( default(SolverSet) );
			}
			else
			{
				B2_ASSERT( world.solverSets.data[setId].setIndex == NullIndex );
			}

			world.solverSets.data[setId].setIndex = setId;
		}

		B2_ASSERT( 0 <= setId && setId < world.solverSets.count );

		int bodyId = AllocId( ref world.bodyIdPool );

		uint lockFlags = 0;
		lockFlags |= def.motionLocks.linearX ? b2_lockLinearX : 0;
		lockFlags |= def.motionLocks.linearY ? b2_lockLinearY : 0;
		lockFlags |= def.motionLocks.angularZ ? b2_lockAngularZ : 0;

		ref SolverSet set = ref world.solverSets[setId];
		ref BodySim bodySim = ref set.bodySims.Emplace();
		bodySim = default(BodySim);
		bodySim.transform.p = def.position;
		bodySim.transform.q = def.rotation;
		bodySim.center = def.position;
		bodySim.rotation0 = bodySim.transform.q;
		bodySim.center0 = bodySim.center;
		bodySim.minExtent = Huge;
		bodySim.maxExtent = 0.0f;
		bodySim.linearDamping = def.linearDamping;
		bodySim.angularDamping = def.angularDamping;
		bodySim.gravityScale = def.gravityScale;
		bodySim.bodyId = bodyId;
		bodySim.flags = lockFlags;
		bodySim.flags |= def.isBullet ? b2_isBullet : 0;
		bodySim.flags |= def.allowFastRotation ? b2_allowFastRotation : 0;
		bodySim.flags |= def.type == BodyType.dynamicBody ? b2_dynamicFlag : 0;
		bodySim.flags |= def.enableSleep ? b2_enableSleep : 0;
		bodySim.flags |= def.enableContactRecycling ? b2_bodyEnableContactRecycling : 0;

		if ( setId == b2_awakeSet )
		{
			ref BodyState bodyState = ref set.bodyStates.Emplace();

			bodyState = default(BodyState);
			bodyState.linearVelocity = def.linearVelocity;
			bodyState.angularVelocity = def.angularVelocity;
			bodyState.deltaRotation = Rot_identity;
			bodyState.flags = bodySim.flags;
		}

		if ( bodyId == world.bodies.count )
		{
			world.bodies.Push( default(Body) );
		}
		else
		{
			B2_ASSERT( world.bodies.data[bodyId].id == NullIndex );
		}

		ref Body body = ref world.bodies[bodyId];
		if ( def.name != null )
		{
			body.name = def.name.Length > NameLength ? def.name.Substring( 0, NameLength ) : def.name;
		}
		else
		{
			body.name = default;
		}

		body.userData = def.userData;
		body.setIndex = setId;
		body.localIndex = set.bodySims.count - 1;
		body.generation += 1;
		body.headShapeId = NullIndex;
		body.shapeCount = 0;
		body.headChainId = NullIndex;
		body.headContactKey = NullIndex;
		body.contactCount = 0;
		body.headJointKey = NullIndex;
		body.jointCount = 0;
		body.islandId = NullIndex;
		body.islandIndex = NullIndex;
		body.bodyMoveIndex = NullIndex;
		body.id = bodyId;
		body.mass = 0.0f;
		body.inertia = 0.0f;
		body.sleepThreshold = def.sleepThreshold;
		body.sleepTime = 0.0f;
		body.safetyFactor = def.safetyFactor;
		body.type = def.type;
		body.flags = bodySim.flags;

		// dynamic and kinematic bodies that are enabled need a island
		if ( setId >= b2_awakeSet )
		{
			CreateIslandForBody( world, setId, ref body );
		}

		ValidateSolverSets( world );

		BodyId id = new BodyId { index1 = bodyId + 1, world0 = world.worldId, generation = body.generation };


		return id;
	}

	internal static bool WakeBody( World world, ref Body body )
	{
		if ( body.setIndex >= b2_firstSleepingSet )
		{
			WakeSolverSet( world, body.setIndex );
			ValidateSolverSets( world );
			return true;
		}

		return false;
	}

	public static void DestroyBody( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}

		// Record before destroying (body must still be valid)

		ref Body body = ref GetBodyFullId( world, bodyId );
		// Destroy the attached joints
		int edgeKey = body.headJointKey;
		while ( edgeKey != NullIndex )
		{
			int jointId = edgeKey >> 1;
			int edgeIndex = edgeKey & 1;

			ref Joint joint = ref world.joints[jointId];
			edgeKey = joint.edges[edgeIndex].nextKey;

			// Careful because this modifies the list being traversed
			DestroyJointInternal( world, ref joint );
		}

		// Destroy all contacts attached to this body.
		DestroyBodyContacts( world, ref body );

		// Destroy the attached shapes and their broad-phase proxies.
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			if ( shape.sensorIndex != NullIndex )
			{
				DestroySensor( world, ref shape );
			}

			DestroyShapeProxy( ref shape, world.broadPhase );

			// Return shape to free list.
			FreeId( ref world.shapeIdPool, shapeId );
			shape.id = NullIndex;

			shapeId = shape.nextShapeId;
		}

		// Destroy the attached chains. The associated shapes have already been destroyed above.
		int chainId = body.headChainId;
		while ( chainId != NullIndex )
		{
			ref ChainShape chain = ref world.chainShapes[chainId];
			FreeChainData( ref chain );

			// Return chain to free list.
			FreeId( ref world.chainIdPool, chainId );
			chain.id = NullIndex;

			chainId = chain.nextChainId;
		}

		RemoveBodyFromIsland( world, ref body );

		// Remove body sim from solver set that owns it
		ref SolverSet set = ref world.solverSets[body.setIndex];
		ref Body movedBody = ref RemoveBodySim( world, ref set, body.localIndex );
		if ( Unsafe.IsNullRef( ref movedBody ) == false )
		{
			RefreshBodyContactIndices( world, ref movedBody );
		}

		// Remove body state from awake set
		if ( body.setIndex == b2_awakeSet )
		{
			set.bodyStates.RemoveSwap( body.localIndex );
		}
		else if ( set.setIndex >= b2_firstSleepingSet && set.bodySims.count == 0 )
		{
			// Remove solver set if it is empty
			DestroySolverSet( world, set.setIndex );
		}

		// Free body and id (preserve body generation)
		FreeId( ref world.bodyIdPool, body.id );

		body.setIndex = NullIndex;
		body.localIndex = NullIndex;
		body.id = NullIndex;

		ValidateSolverSets( world );
	}

	public static int Body_GetContactCapacity( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		// Conservative and fast
		return body.contactCount;
	}

	public static int Body_GetContactData( BodyId bodyId, Span<ContactData> contactData, int capacity )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return 0;
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		int contactKey = body.headContactKey;
		int index = 0;
		while ( contactKey != NullIndex && index < capacity )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			// Is contact touching?
			if ( ( contact.flags & b2_contactTouchingFlag ) != 0 )
			{
				ref Shape shapeA = ref world.shapes[contact.shapeIdA];
				ref Shape shapeB = ref world.shapes[contact.shapeIdB];
				contactData[index].contactId = new ContactId { index1 = contact.contactId + 1, world0 = bodyId.world0, padding = 0, generation = contact.generation };
				contactData[index].shapeIdA = new ShapeId { index1 = shapeA.id + 1, world0 = bodyId.world0, generation = shapeA.generation };
				contactData[index].shapeIdB = new ShapeId { index1 = shapeB.id + 1, world0 = bodyId.world0, generation = shapeB.generation };

				ref ContactSim contactSim = ref GetContactSim( world, ref contact );
				contactData[index].manifold = contactSim.manifold;

				index += 1;
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}

		B2_ASSERT( index <= capacity );

		return index;
	}

	public static AABB Body_ComputeBounds( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return default(AABB);
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.headShapeId == NullIndex )
		{
			// No shapes, bracket the body origin so the box still contains the true position far away
			WorldTransform transform = GetBodyTransform( world, body.id );
			Vec2 lower = new Vec2( RoundDownFloat( transform.p.x ), RoundDownFloat( transform.p.y ) );
			Vec2 upper = new Vec2( RoundUpFloat( transform.p.x ), RoundUpFloat( transform.p.y ) );
			return new AABB { lowerBound = lower, upperBound = upper };
		}

		ref Shape shape = ref world.shapes[body.headShapeId];
		AABB aabb = shape.aabb;
		while ( shape.nextShapeId != NullIndex )
		{
			shape = ref world.shapes[shape.nextShapeId];
			aabb = AABB_Union( aabb, shape.aabb );
		}

		return aabb;
	}

	internal static void UpdateBodyMassData( World world, ref Body body )
	{
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		// Mass is no longer dirty
		body.flags &= ~b2_dirtyMass;

		// Compute mass data from shapes. Each shape has its own density.
		body.mass = 0.0f;
		body.inertia = 0.0f;

		bodySim.invMass = 0.0f;
		bodySim.invInertia = 0.0f;
		bodySim.localCenter = Vec2_zero;
		bodySim.minExtent = Huge;
		bodySim.maxExtent = 0.0f;

		// Static and kinematic sims have zero mass.
		if ( body.type != BodyType.dynamicBody )
		{
			bodySim.center = bodySim.transform.p;
			bodySim.center0 = bodySim.center;

			// Need extents for kinematic bodies for sleeping to work correctly.
			if ( body.type == BodyType.kinematicBody )
			{
				int shapeIdInner = body.headShapeId;
				while ( shapeIdInner != NullIndex )
				{
					ref readonly Shape s = ref world.shapes[shapeIdInner];
					ShapeExtent extent = ComputeShapeExtent( s, Vec2_zero );
					bodySim.minExtent = MinFloat( bodySim.minExtent, extent.minExtent );
					bodySim.maxExtent = MaxFloat( bodySim.maxExtent, extent.maxExtent );

					shapeIdInner = s.nextShapeId;
				}
			}

			return;
		}

		int shapeCount = body.shapeCount;
		Span<MassData> masses = world.stack.Alloc<MassData>( shapeCount, "mass data" );
		// Accumulate mass over all shapes.
		Vec2 localCenter = Vec2_zero;
		int shapeId = body.headShapeId;
		int shapeIndex = 0;
		while ( shapeId != NullIndex )
		{
			ref readonly Shape s = ref world.shapes[shapeId];
			shapeId = s.nextShapeId;

			if ( s.density == 0.0f )
			{
				masses[shapeIndex] = default(MassData);
				shapeIndex += 1;
				continue;
			}

			MassData massData = ComputeShapeMass( s );
			body.mass += massData.mass;
			localCenter = MulAdd( localCenter, massData.mass, massData.center );

			masses[shapeIndex] = massData;
			shapeIndex += 1;
		}

		// Compute center of mass.
		if ( body.mass > 0.0f )
		{
			bodySim.invMass = 1.0f / body.mass;
			localCenter = MulSV( bodySim.invMass, localCenter );
		}

		// Second loop to accumulate the rotational inertia about the center of mass
		for ( shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
		{
			MassData massData = masses[shapeIndex];
			if ( massData.mass == 0.0f )
			{
				continue;
			}

			// Shift to center of mass. This is safe because it can only increase.
			Vec2 offset = Sub( localCenter, massData.center );
			float inertia = massData.rotationalInertia + massData.mass * Dot( offset, offset );
			body.inertia += inertia;
		}

		world.stack.Free( masses );
		masses = null;

		B2_ASSERT( body.inertia >= 0.0f );

		if ( body.inertia > 0.0f && ( body.flags & b2_fixedRotation ) == 0 )
		{
			bodySim.invInertia = 1.0f / body.inertia;
		}
		else
		{
			body.inertia = 0.0f;
			bodySim.invInertia = 0.0f;
		}

		// Move center of mass.
		Pos oldCenter = bodySim.center;
		bodySim.localCenter = localCenter;
		bodySim.center = TransformWorldPoint( bodySim.transform, bodySim.localCenter );
		bodySim.center0 = bodySim.center;

		// Update center of mass velocity
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) == false )
		{
			Vec2 deltaLinear = CrossSV( state.angularVelocity, SubPos( bodySim.center, oldCenter ) );
			state.linearVelocity = Add( state.linearVelocity, deltaLinear );
		}

		// Compute body extents relative to center of mass
		shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref readonly Shape s = ref world.shapes[shapeId];
			ShapeExtent extent = ComputeShapeExtent( s, localCenter );
			bodySim.minExtent = MinFloat( bodySim.minExtent, extent.minExtent );
			bodySim.maxExtent = MaxFloat( bodySim.maxExtent, extent.maxExtent );

			shapeId = s.nextShapeId;
		}

		// When the center of mass changes, any cached contact manifold becomes invalid.
		int edgeKey = body.headContactKey;
		while ( edgeKey != NullIndex )
		{
			int contactId = edgeKey >> 1;
			int edgeIndex = edgeKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			ref ContactSim contactSim = ref GetContactSim( world, ref contact );
			contactSim.simFlags &= ~b2_simRelativeTransformValid;

			edgeKey = contact.edges[edgeIndex].nextKey;
		}
	}

	public static Pos Body_GetPosition( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return transform.p;
	}

	public static Rot Body_GetRotation( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return transform.q;
	}

	public static WorldTransform Body_GetTransform( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return GetBodyTransformQuick( world, ref body );
	}

	public static Vec2 Body_GetLocalPoint( BodyId bodyId, Pos worldPoint )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return InvTransformWorldPoint( transform, worldPoint );
	}

	public static Pos Body_GetWorldPoint( BodyId bodyId, Vec2 localPoint )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return TransformWorldPoint( transform, localPoint );
	}

	public static Vec2 Body_GetLocalVector( BodyId bodyId, Vec2 worldVector )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return InvRotateVector( transform.q, worldVector );
	}

	public static Vec2 Body_GetWorldVector( BodyId bodyId, Vec2 localVector )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		return RotateVector( transform.q, localVector );
	}

	public static void Body_SetTransform( BodyId bodyId, Pos position, Rot rotation )
	{
		if ( CheckInput( IsValidPos( position ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidRot( rotation ) ) == false )
			{
				return;
			};
		B2_ASSERT( Body_IsValid( bodyId ) );
		World world = GetWorld( bodyId.world0 );
		B2_ASSERT( world.locked == false );


		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		bodySim.transform.p = position;
		bodySim.transform.q = rotation;
		bodySim.center = TransformWorldPoint( bodySim.transform, bodySim.localCenter );

		bodySim.rotation0 = bodySim.transform.q;
		bodySim.center0 = bodySim.center;

		BroadPhase broadPhase = world.broadPhase;
		WorldTransform transform = bodySim.transform;
		float speculativeDistance = SpeculativeDistance;

		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
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

				// They body could be disabled
				if ( shape.proxyKey != NullIndex )
				{
					BroadPhase_MoveProxy( broadPhase, shape.proxyKey, fatAABB );
				}
			}

			shapeId = shape.nextShapeId;
		}
	}

	public static Vec2 Body_GetLinearVelocity( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) == false )
		{
			return state.linearVelocity;
		}
		return Vec2_zero;
	}

	public static float Body_GetAngularVelocity( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) == false )
		{
			return state.angularVelocity;
		}
		return 0.0f;
	}

	public static void Body_SetLinearVelocity( BodyId bodyId, Vec2 linearVelocity )
	{
		if ( CheckInput( IsValidVec2( linearVelocity ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );

		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type == BodyType.staticBody )
		{
			return;
		}

		if ( LengthSquared( linearVelocity ) > 0.0f )
		{
			WakeBody( world, ref body );
		}

		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) )
		{
			return;
		}

		state.linearVelocity = linearVelocity;
	}

	public static void Body_SetAngularVelocity( BodyId bodyId, float angularVelocity )
	{
		if ( CheckInput( IsValidFloat( angularVelocity ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type == BodyType.staticBody || ( body.flags & b2_lockAngularZ ) != 0 )
		{
			return;
		}

		if ( angularVelocity != 0.0f )
		{
			WakeBody( world, ref body );
		}

		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) )
		{
			return;
		}

		state.angularVelocity = angularVelocity;
	}

	public static void Body_SetTargetTransform( BodyId bodyId, WorldTransform target, float timeStep, bool wake )
	{
		if ( CheckInput( IsValidWorldTransform( target ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( timeStep ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( body.type == BodyType.staticBody || timeStep <= 0.0f )
		{
			return;
		}

		if ( body.setIndex != b2_awakeSet && wake == false )
		{
			return;
		}

		ref BodySim sim = ref GetBodySim( world, ref body );
		// Compute linear velocity. The center difference is taken in world precision then demoted
		Vec2 delta = SubPos( TransformWorldPoint( target, sim.localCenter ), sim.center );
		float invTimeStep = 1.0f / timeStep;
		Vec2 linearVelocity = MulSV( invTimeStep, delta );

		// Compute angular velocity
		Rot q1 = sim.transform.q;
		Rot q2 = target.q;
		float deltaAngle = RelativeAngle( q1, q2 );
		float angularVelocity = invTimeStep * deltaAngle;

		// Early out if the body is asleep already and the desired movement is small
		if ( body.setIndex != b2_awakeSet )
		{
			float maxVelocity = Length( linearVelocity ) + AbsFloat( angularVelocity ) * sim.maxExtent;

			// Return if velocity would be sleepy
			if ( maxVelocity < body.sleepThreshold )
			{
				return;
			}

			// Must wake for state to exist
			WakeBody( world, ref body );
		}

		B2_ASSERT( body.setIndex == b2_awakeSet );

		ref BodyState state = ref GetBodyState( world, ref body );
		state.linearVelocity = linearVelocity;
		state.angularVelocity = angularVelocity;
	}

	public static Vec2 Body_GetLocalPointVelocity( BodyId bodyId, Vec2 localPoint )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) )
		{
			return Vec2_zero;
		}

		ref SolverSet set = ref world.solverSets[body.setIndex];
		ref BodySim bodySim = ref set.bodySims[body.localIndex];
		Vec2 r = RotateVector( bodySim.transform.q, Sub( localPoint, bodySim.localCenter ) );
		Vec2 v = Add( state.linearVelocity, CrossSV( state.angularVelocity, r ) );
		return v;
	}

	public static Vec2 Body_GetWorldPointVelocity( BodyId bodyId, Pos worldPoint )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) )
		{
			return Vec2_zero;
		}

		ref SolverSet set = ref world.solverSets[body.setIndex];
		ref BodySim bodySim = ref set.bodySims[body.localIndex];
		Vec2 r = SubPos( worldPoint, bodySim.center );
		Vec2 v = Add( state.linearVelocity, CrossSV( state.angularVelocity, r ) );
		return v;
	}

	public static void Body_ApplyForce( BodyId bodyId, Vec2 force, Pos point, bool wake )
	{
		if ( CheckInput( IsValidVec2( force ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidPos( point ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			ref BodySim bodySim = ref GetBodySim( world, ref body );
			bodySim.force = Add( bodySim.force, force );
			bodySim.torque += Cross( SubPos( point, bodySim.center ), force );
		}
	}

	public static void Body_ApplyForceToCenter( BodyId bodyId, Vec2 force, bool wake )
	{
		if ( CheckInput( IsValidVec2( force ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			ref BodySim bodySim = ref GetBodySim( world, ref body );
			bodySim.force = Add( bodySim.force, force );
		}
	}

	public static void Body_ApplyTorque( BodyId bodyId, float torque, bool wake )
	{
		if ( CheckInput( IsValidFloat( torque ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			ref BodySim bodySim = ref GetBodySim( world, ref body );
			bodySim.torque += torque;
		}
	}

	public static void Body_ClearForces( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		bodySim.force = Vec2_zero;
		bodySim.torque = 0.0f;
	}

	public static void Body_ApplyLinearImpulse( BodyId bodyId, Vec2 impulse, Pos point, bool wake )
	{
		if ( CheckInput( IsValidVec2( impulse ) ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidPos( point ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			int localIndex = body.localIndex;
			ref SolverSet set = ref world.solverSets[b2_awakeSet];
			ref BodyState state = ref set.bodyStates[localIndex];
			ref BodySim bodySim = ref set.bodySims[localIndex];
			state.linearVelocity = MulAdd( state.linearVelocity, bodySim.invMass, impulse );
			state.angularVelocity += bodySim.invInertia * Cross( SubPos( point, bodySim.center ), impulse );

			LimitVelocity( ref state, world.maxLinearSpeed );
		}
	}

	public static void Body_ApplyLinearImpulseToCenter( BodyId bodyId, Vec2 impulse, bool wake )
	{
		if ( CheckInput( IsValidVec2( impulse ) ) == false )
			{
				return;
			};

		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			int localIndex = body.localIndex;
			ref SolverSet set = ref world.solverSets[b2_awakeSet];
			ref BodyState state = ref set.bodyStates[localIndex];
			ref BodySim bodySim = ref set.bodySims[localIndex];
			state.linearVelocity = MulAdd( state.linearVelocity, bodySim.invMass, impulse );

			LimitVelocity( ref state, world.maxLinearSpeed );
		}
	}

	public static void Body_ApplyAngularImpulse( BodyId bodyId, float impulse, bool wake )
	{
		if ( CheckInput( IsValidFloat( impulse ) ) == false )
			{
				return;
			};
		B2_ASSERT( Body_IsValid( bodyId ) );
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.type != BodyType.dynamicBody || body.setIndex == b2_disabledSet )
		{
			return;
		}

		if ( wake && body.setIndex >= b2_firstSleepingSet )
		{
			// this will not invalidate body pointer
			WakeBody( world, ref body );
		}

		if ( body.setIndex == b2_awakeSet )
		{
			int localIndex = body.localIndex;
			ref SolverSet set = ref world.solverSets[b2_awakeSet];
			ref BodyState state = ref set.bodyStates[localIndex];
			ref BodySim bodySim = ref set.bodySims[localIndex];
			state.angularVelocity += bodySim.invInertia * impulse;
		}
	}

	public static BodyType Body_GetType( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.type;
	}

	// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
	// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
	// todo with a bit more effort I could support an option to let the body sleep
	//
	// Revised steps:
	// 1 Skip disabled bodies
	// 2 Destroy all contacts on the body
	// 3 Wake the body
	// 4 For all joints attached to the body
	//  - wake attached bodies
	//  - remove from island
	//  - move to static set temporarily
	// 5 Change the body type and transfer the body
	// 6 If the body was static
	//   - create an island for the body
	//   Else if the body is becoming static
	//   - remove it from the island
	// 7 For all joints
	//  - if either body is non-static
	//    - link into island
	//    - transfer to constraint graph
	// 8 For all shapes
	//  - Destroy proxy in old tree
	//  - Create proxy in new tree
	// Notes:
	// - the implementation below tries to minimize the number of predicates, so some
	//   operations may have no effect, such as transferring a joint to the same set
	public static void Body_SetType( BodyId bodyId, BodyType type )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		BodyType originalType = body.type;
		if ( originalType == type )
		{
			return;
		}

		// Stage 1: skip disabled bodies
		if ( body.setIndex == b2_disabledSet )
		{
			// Disabled bodies don't change solver sets or islands when they change type.
			body.type = type;

			if ( type == BodyType.dynamicBody )
			{
				body.flags |= b2_dynamicFlag;
			}
			else
			{
				body.flags &= ~b2_dynamicFlag;
			}

			SyncBodyFlags( world, ref body );

			// Body type affects the mass properties
			UpdateBodyMassData( world, ref body );
			return;
		}

		// Stage 2: destroy all contacts
		DestroyBodyContacts( world, ref body );

		// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
		// all bodies in the same sleeping solver set.
		WakeBody( world, ref body );

		// Stage 4: move joints to temporary storage
		ref SolverSet staticSet = ref world.solverSets[b2_staticSet];
		int jointKey = body.headJointKey;
		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			jointKey = joint.edges[edgeIndex].nextKey;

			// Joint may be disabled by other body
			if ( joint.setIndex == b2_disabledSet )
			{
				continue;
			}

			// Wake attached bodies. The WakeBody call above does not wake bodies
			// attached to a static body. But it is necessary because the body may have
			// no joints.
			ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
			ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
			WakeBody( world, ref bodyA );
			WakeBody( world, ref bodyB );

			// Remove joint from island
			UnlinkJoint( world, ref joint );

			// It is necessary to transfer all joints to the static set
			// so they can be added to the constraint graph below and acquire consistent colors.
			ref SolverSet jointSourceSet = ref world.solverSets[joint.setIndex];
			TransferJoint( world, ref staticSet, ref jointSourceSet, ref joint );
		}

		// Stage 5: change the body type and transfer body
		body.type = type;

		if ( type == BodyType.dynamicBody )
		{
			body.flags |= b2_dynamicFlag;
		}
		else
		{
			body.flags &= ~b2_dynamicFlag;
		}

		ref SolverSet awakeSet = ref world.solverSets[b2_awakeSet];
		ref SolverSet sourceSet = ref world.solverSets[body.setIndex];
		ref SolverSet targetSet = ref type == BodyType.staticBody ? ref staticSet : ref awakeSet;
		// Transfer body
		TransferBody( world, ref targetSet, ref sourceSet, ref body );

		// Stage 6: update island participation for the body
		if ( originalType == BodyType.staticBody )
		{
			// Create island for body
			CreateIslandForBody( world, b2_awakeSet, ref body );
		}
		else if ( type == BodyType.staticBody )
		{
			// Remove body from island.
			RemoveBodyFromIsland( world, ref body );
		}

		// Stage 7: Transfer joints to the target set
		jointKey = body.headJointKey;
		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			jointKey = joint.edges[edgeIndex].nextKey;

			// Joint may be disabled by other body
			if ( joint.setIndex == b2_disabledSet )
			{
				continue;
			}

			// All joints were transferred to the static set in an earlier stage
			B2_ASSERT( joint.setIndex == b2_staticSet );

			ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
			ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
			B2_ASSERT( bodyA.setIndex == b2_staticSet || bodyA.setIndex == b2_awakeSet );
			B2_ASSERT( bodyB.setIndex == b2_staticSet || bodyB.setIndex == b2_awakeSet );

			if ( bodyA.type == BodyType.dynamicBody || bodyB.type == BodyType.dynamicBody )
			{
				TransferJoint( world, ref awakeSet, ref staticSet, ref joint );
			}
		}

		// Recreate shape proxies in broadphase
		WorldTransform transform = GetBodyTransformQuick( world, ref body );
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			shapeId = shape.nextShapeId;
			DestroyShapeProxy( ref shape, world.broadPhase );
			bool forcePairCreation = true;
			CreateShapeProxy( world, ref shape, type, transform, forcePairCreation );
		}

		// Relink all joints
		jointKey = body.headJointKey;
		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			jointKey = joint.edges[edgeIndex].nextKey;

			int otherEdgeIndex = edgeIndex ^ 1;
			int otherBodyId = joint.edges[otherEdgeIndex].bodyId;
			ref Body otherBody = ref world.bodies[otherBodyId];
			if ( otherBody.setIndex == b2_disabledSet )
			{
				continue;
			}

			if ( body.type != BodyType.dynamicBody && otherBody.type != BodyType.dynamicBody )
			{
				continue;
			}

			LinkJoint( world, ref joint );
		}

		SyncBodyFlags( world, ref body );

		// Body type affects the mass
		UpdateBodyMassData( world, ref body );

		ValidateSolverSets( world );
		ValidateIsland( world, body.islandId );
	}

	public static void Body_SetName( BodyId bodyId, string name )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( name != null )
		{
			body.name = name.Length > NameLength ? name.Substring( 0, NameLength ) : name;
		}
		else
		{
			body.name = default;
		}
	}

	public static string? Body_GetName( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.name;
	}

	public static void Body_SetUserData( BodyId bodyId, object? userData )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		body.userData = userData;
	}

	public static object? Body_GetUserData( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.userData;
	}

	public static float Body_GetMass( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.mass;
	}

	public static float Body_GetRotationalInertia( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.inertia;
	}

	public static Vec2 Body_GetLocalCenter( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.localCenter;
	}

	public static Pos Body_GetWorldCenter( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.center;
	}

	public static void Body_SetMassData( BodyId bodyId, MassData massData )
	{
		if ( CheckInput( IsValidFloat( massData.mass ) && massData.mass >= 0.0f ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( massData.rotationalInertia ) && massData.rotationalInertia >= 0.0f ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidVec2( massData.center ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		body.mass = massData.mass;
		body.inertia = massData.rotationalInertia;
		bodySim.localCenter = massData.center;

		Pos oldCenter = bodySim.center;
		Pos center = TransformWorldPoint( bodySim.transform, massData.center );
		bodySim.center = center;
		bodySim.center0 = center;

		// Update center of mass velocity
		ref BodyState state = ref GetBodyState( world, ref body );
		if ( Unsafe.IsNullRef( ref state ) == false )
		{
			Vec2 deltaLinear = CrossSV( state.angularVelocity, SubPos( bodySim.center, oldCenter ) );
			state.linearVelocity = Add( state.linearVelocity, deltaLinear );
		}

		bodySim.invMass = body.mass > 0.0f ? 1.0f / body.mass : 0.0f;
		bodySim.invInertia = body.inertia > 0.0f ? 1.0f / body.inertia : 0.0f;

		// Update extents using supplied mass center.
		bodySim.minExtent = Huge;
		bodySim.maxExtent = 0.0f;
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref readonly Shape s = ref world.shapes[shapeId];
			ShapeExtent extent = ComputeShapeExtent( s, massData.center );
			bodySim.minExtent = MinFloat( bodySim.minExtent, extent.minExtent );
			bodySim.maxExtent = MaxFloat( bodySim.maxExtent, extent.maxExtent );
			shapeId = s.nextShapeId;
		}

		// When the center of mass changes, any cached contact manifold becomes invalid.
		int edgeKey = body.headContactKey;
		while ( edgeKey != NullIndex )
		{
			int contactId = edgeKey >> 1;
			int edgeIndex = edgeKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			ref ContactSim contactSim = ref GetContactSim( world, ref contact );
			contactSim.simFlags &= ~b2_simRelativeTransformValid;

			edgeKey = contact.edges[edgeIndex].nextKey;
		}

		// Motion locks take priority over mass data.
		if ( ( body.flags & b2_fixedRotation ) != 0 )
		{
			body.inertia = 0.0f;
			bodySim.invInertia = 0.0f;
		}

		body.flags &= ~b2_dirtyMass;
	}

	public static MassData Body_GetMassData( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		MassData massData = new MassData { mass = body.mass, center = bodySim.localCenter, rotationalInertia = body.inertia };
		return massData;
	}

	public static void Body_UpdateMassFromShapes( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		UpdateBodyMassData( world, ref body );
	}

	public static void Body_SetLinearDamping( BodyId bodyId, float linearDamping )
	{
		if ( CheckInput( IsValidFloat( linearDamping ) && linearDamping >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		bodySim.linearDamping = linearDamping;
	}

	public static float Body_GetLinearDamping( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.linearDamping;
	}

	public static void Body_SetAngularDamping( BodyId bodyId, float angularDamping )
	{
		if ( CheckInput( IsValidFloat( angularDamping ) && angularDamping >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		bodySim.angularDamping = angularDamping;
	}

	public static float Body_GetAngularDamping( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.angularDamping;
	}

	public static void Body_SetGravityScale( BodyId bodyId, float gravityScale )
	{
		B2_ASSERT( Body_IsValid( bodyId ) );
		if ( CheckInput( IsValidFloat( gravityScale ) ) == false )
			{
				return;
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		bodySim.gravityScale = gravityScale;
	}

	public static float Body_GetGravityScale( BodyId bodyId )
	{
		B2_ASSERT( Body_IsValid( bodyId ) );
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.gravityScale;
	}

	public static bool Body_IsAwake( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.setIndex == b2_awakeSet;
	}

	public static void Body_SetAwake( BodyId bodyId, bool awake )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( awake && body.setIndex >= b2_firstSleepingSet )
		{
			WakeBody( world, ref body );
		}
		else if ( awake == false && body.setIndex == b2_awakeSet )
		{
			ref Island island = ref world.islands[body.islandId];
			if ( island.constraintRemoveCount > 0 )
			{
				// Must split the island before sleeping. This is expensive.
				SplitIsland( world, body.islandId );
			}

			TrySleepIsland( world, body.islandId );
		}
	}

	public static void Body_WakeTouching( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		int contactKey = body.headContactKey;
		while ( contactKey != NullIndex )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			ref Shape shapeA = ref world.shapes[contact.shapeIdA];
			ref Shape shapeB = ref world.shapes[contact.shapeIdB];
			if ( shapeA.bodyId == bodyId.index1 - 1 )
			{
				ref Body otherBody = ref world.bodies[shapeB.bodyId];
				WakeBody( world, ref otherBody );
			}
			else
			{
				ref Body otherBody = ref world.bodies[shapeA.bodyId];
				WakeBody( world, ref otherBody );
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}
	}

	public static bool Body_IsEnabled( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.setIndex != b2_disabledSet;
	}

	public static bool Body_IsSleepingEnabled( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return ( body.flags & b2_enableSleep ) == b2_enableSleep;
	}

	public static void Body_SetSleepThreshold( BodyId bodyId, float sleepThreshold )
	{
		if ( CheckInput( IsValidFloat( sleepThreshold ) && sleepThreshold >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		body.sleepThreshold = sleepThreshold;
	}

	public static float Body_GetSleepThreshold( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.sleepThreshold;
	}

	public static void Body_SetSafetyFactor( BodyId bodyId, float safetyFactor )
	{
		if ( CheckInput( IsValidFloat( safetyFactor ) && safetyFactor >= 0.0f ) == false )
			{
				return;
			};
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}

		ref Body body = ref GetBodyFullId( world, bodyId );
		body.safetyFactor = safetyFactor;
	}

	public static float Body_GetSafetyFactor( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.safetyFactor;
	}

	public static void Body_EnableSleeping( BodyId bodyId, bool enableSleep )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		bool flag = ( body.flags & b2_enableSleep ) == b2_enableSleep;
		if ( enableSleep == flag )
		{
			return;
		}

		body.flags = enableSleep ? body.flags | b2_enableSleep : body.flags & ~b2_enableSleep;
		SyncBodyFlags( world, ref body );

		if ( enableSleep == false )
		{
			WakeBody( world, ref body );
		}
	}

	// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
	// The most challenging aspect is that joints may connect to bodies that are not disabled.
	public static void Body_Disable( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.setIndex == b2_disabledSet )
		{
			return;
		}

		// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
		// This is necessary even for static bodies.
		DestroyBodyContacts( world, ref body );

		// The current solver set of the body
		ref SolverSet set = ref world.solverSets[body.setIndex];
		// Disabled bodies and connected joints are moved to the disabled set
		ref SolverSet disabledSet = ref world.solverSets[b2_disabledSet];
		// Unlink joints and transfer them to the disabled set
		int jointKey = body.headJointKey;
		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			jointKey = joint.edges[edgeIndex].nextKey;

			// joint may already be disabled by other body
			if ( joint.setIndex == b2_disabledSet )
			{
				continue;
			}

			B2_ASSERT( joint.setIndex == set.setIndex || set.setIndex == b2_staticSet || joint.setIndex == b2_staticSet );

			// Remove joint from island
			UnlinkJoint( world, ref joint );

			// Transfer joint to disabled set
			ref SolverSet jointSet = ref world.solverSets[joint.setIndex];
			TransferJoint( world, ref disabledSet, ref jointSet, ref joint );
		}

		// Remove shapes from broad-phase
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			shapeId = shape.nextShapeId;
			DestroyShapeProxy( ref shape, world.broadPhase );
		}

		// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
		RemoveBodyFromIsland( world, ref body );

		// Transfer body sim
		TransferBody( world, ref disabledSet, ref set, ref body );

		ValidateConnectivity( world );
		ValidateSolverSets( world );
	}

	public static void Body_Enable( BodyId bodyId )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( body.setIndex != b2_disabledSet )
		{
			return;
		}

		ref SolverSet disabledSet = ref world.solverSets[b2_disabledSet];
		int setId = body.type == BodyType.staticBody ? b2_staticSet : b2_awakeSet;
		ref SolverSet targetSet = ref world.solverSets[setId];
		TransferBody( world, ref targetSet, ref disabledSet, ref body );

		WorldTransform transform = GetBodyTransformQuick( world, ref body );

		// Add shapes to broad-phase
		BodyType proxyType = body.type;
		bool forcePairCreation = true;
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			shapeId = shape.nextShapeId;

			CreateShapeProxy( world, ref shape, proxyType, transform, forcePairCreation );
		}

		if ( setId != b2_staticSet )
		{
			CreateIslandForBody( world, setId, ref body );
		}

		// Transfer joints. If the other body is disabled, don't transfer.
		// If the other body is sleeping, wake it.
		int jointKey = body.headJointKey;
		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			B2_ASSERT( joint.setIndex == b2_disabledSet );
			B2_ASSERT( joint.islandId == NullIndex );

			jointKey = joint.edges[edgeIndex].nextKey;

			ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
			ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
			if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
			{
				// one body is still disabled
				continue;
			}

			// Transfer joint first
			int jointSetId;
			if ( bodyA.type != BodyType.dynamicBody && bodyB.type != BodyType.dynamicBody )
			{
				jointSetId = b2_staticSet;
			}
			else if ( bodyA.setIndex == b2_staticSet )
			{
				jointSetId = bodyB.setIndex;
			}
			else
			{
				jointSetId = bodyA.setIndex;
			}

			ref SolverSet jointSet = ref world.solverSets[jointSetId];
			TransferJoint( world, ref jointSet, ref disabledSet, ref joint );

			// Now that the joint is in the correct set, I can link the joint in the island.
			if ( jointSetId != b2_staticSet )
			{
				LinkJoint( world, ref joint );
			}
		}

		ValidateSolverSets( world );
	}

	public static void Body_SetMotionLocks( BodyId bodyId, MotionLocks locks )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		uint newFlags = 0;
		newFlags |= locks.linearX ? b2_lockLinearX : 0;
		newFlags |= locks.linearY ? b2_lockLinearY : 0;
		newFlags |= locks.angularZ ? b2_lockAngularZ : 0;

		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( ( body.flags & b2_allLocks ) != newFlags )
		{
			body.flags &= ~b2_allLocks;
			body.flags |= newFlags;

			SyncBodyFlags( world, ref body );

			ref BodyState state = ref GetBodyState( world, ref body );
			if ( Unsafe.IsNullRef( ref state ) == false )
			{
				if ( locks.linearX )
				{
					state.linearVelocity.x = 0.0f;
				}

				if ( locks.linearY )
				{
					state.linearVelocity.y = 0.0f;
				}

				if ( locks.angularZ )
				{
					state.angularVelocity = 0.0f;
				}
			}

			// Motion locks can affect mass properties.
			UpdateBodyMassData( world, ref body );
		}
	}

	public static MotionLocks Body_GetMotionLocks( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		MotionLocks locks = default;
		locks.linearX = ( body.flags & b2_lockLinearX ) != 0;
		locks.linearY = ( body.flags & b2_lockLinearY ) != 0;
		locks.angularZ = ( body.flags & b2_lockAngularZ ) != 0;
		return locks;
	}

	public static void Body_SetBullet( BodyId bodyId, bool flag )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		uint newFlag = flag ? b2_isBullet : 0;

		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( ( body.flags & b2_isBullet ) == newFlag )
		{
			return;
		}

		body.flags &= ~b2_isBullet;
		body.flags |= newFlag;

		SyncBodyFlags( world, ref body );
	}

	public static bool Body_IsBullet( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return ( bodySim.flags & b2_isBullet ) != 0;
	}

	public static void Body_EnableContactRecycling( BodyId bodyId, bool flag )
	{
		World world = GetWorldLocked( bodyId.world0 );
		if ( world == null )
		{
			return;
		}


		uint newFlag = flag ? b2_bodyEnableContactRecycling : 0;

		ref Body body = ref GetBodyFullId( world, bodyId );
		if ( ( body.flags & b2_bodyEnableContactRecycling ) == newFlag )
		{
			return;
		}

		body.flags &= ~b2_bodyEnableContactRecycling;
		body.flags |= newFlag;

		SyncBodyFlags( world, ref body );
	}

	public static bool Body_IsContactRecyclingEnabled( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return ( body.flags & b2_bodyEnableContactRecycling ) != 0;
	}

	public static void Body_EnableContactEvents( BodyId bodyId, bool flag )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			shape.enableContactEvents = flag;
			shapeId = shape.nextShapeId;
		}
	}

	public static void Body_EnableHitEvents( BodyId bodyId, bool flag )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		int shapeId = body.headShapeId;
		while ( shapeId != NullIndex )
		{
			ref Shape shape = ref world.shapes[shapeId];
			shape.enableHitEvents = flag;
			shapeId = shape.nextShapeId;
		}
	}

	public static WorldId Body_GetWorld( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		return new WorldId { index1 = (ushort)( bodyId.world0 + 1 ), generation = world.generation };
	}

	public static int Body_GetShapeCount( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.shapeCount;
	}

	public static int Body_GetShapes( BodyId bodyId, Span<ShapeId> shapeArray, int capacity )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		int shapeId = body.headShapeId;
		int shapeCount = 0;
		while ( shapeId != NullIndex && shapeCount < capacity )
		{
			ref Shape shape = ref world.shapes[shapeId];
			ShapeId id = new ShapeId { index1 = shape.id + 1, world0 = bodyId.world0, generation = shape.generation };
			shapeArray[shapeCount] = id;
			shapeCount += 1;

			shapeId = shape.nextShapeId;
		}

		return shapeCount;
	}

	public static int Body_GetJointCount( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		return body.jointCount;
	}

	public static int Body_GetJoints( BodyId bodyId, Span<JointId> jointArray, int capacity )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		int jointKey = body.headJointKey;

		int jointCount = 0;
		while ( jointKey != NullIndex && jointCount < capacity )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			ref Joint joint = ref world.joints[jointId];
			JointId id = new JointId { index1 = jointId + 1, world0 = bodyId.world0, generation = joint.generation };
			jointArray[jointCount] = id;
			jointCount += 1;

			jointKey = joint.edges[edgeIndex].nextKey;
		}

		return jointCount;
	}

	internal static bool ShouldBodiesCollide( World world, ref Body bodyA, ref Body bodyB )
	{
		if ( bodyA.type != BodyType.dynamicBody && bodyB.type != BodyType.dynamicBody )
		{
			return false;
		}

		int jointKey;
		int otherBodyId;
		if ( bodyA.jointCount < bodyB.jointCount )
		{
			jointKey = bodyA.headJointKey;
			otherBodyId = bodyB.id;
		}
		else
		{
			jointKey = bodyB.headJointKey;
			otherBodyId = bodyA.id;
		}

		while ( jointKey != NullIndex )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;
			int otherEdgeIndex = edgeIndex ^ 1;

			ref Joint joint = ref world.joints[jointId];
			if ( joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId )
			{
				return false;
			}

			jointKey = joint.edges[edgeIndex].nextKey;
		}

		return true;
	}

	public static float Body_GetMinExtent( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.minExtent;
	}

	public static float Body_GetMaxExtent( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.maxExtent;
	}

	public static float Body_GetMaxExtentFromOrigin( BodyId bodyId )
	{
		World world = GetWorld( bodyId.world0 );
		ref Body body = ref GetBodyFullId( world, bodyId );
		ref BodySim bodySim = ref GetBodySim( world, ref body );
		return bodySim.maxExtent + Length( bodySim.localCenter );
	}

}
