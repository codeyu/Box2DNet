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


	private static JointDef DefaultJointDef()
	{
		JointDef def = default;
		def.localFrameA.q = Rot_identity;
		def.localFrameB.q = Rot_identity;
		def.forceThreshold = FLT_MAX;
		def.torqueThreshold = FLT_MAX;
		def.constraintHertz = 60.0f;
		def.constraintDampingRatio = 2.0f;
		def.drawScale = GetLengthUnitsPerMeter();
		return def;
	}

	public static DistanceJointDef DefaultDistanceJointDef()
	{
		DistanceJointDef def = default;
		def.@base = DefaultJointDef();
		def.lowerSpringForce = -FLT_MAX;
		def.upperSpringForce = FLT_MAX;
		def.length = 1.0f;
		def.maxLength = Huge;
		def.internalValue = SecretCookie;
		return def;
	}

	public static FilterJointDef DefaultFilterJointDef()
	{
		FilterJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static MotorJointDef DefaultMotorJointDef()
	{
		MotorJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static MoverJointDef DefaultMoverJointDef()
	{
		MoverJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static PogoJointDef DefaultPogoJointDef()
	{
		PogoJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static PrismaticJointDef DefaultPrismaticJointDef()
	{
		PrismaticJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static RevoluteJointDef DefaultRevoluteJointDef()
	{
		RevoluteJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static WeldJointDef DefaultWeldJointDef()
	{
		WeldJointDef def = default;
		def.@base = DefaultJointDef();
		def.internalValue = SecretCookie;
		return def;
	}

	public static WheelJointDef DefaultWheelJointDef()
	{
		WheelJointDef def = default;
		def.@base = DefaultJointDef();
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.7f;
		def.internalValue = SecretCookie;
		return def;
	}

	public static ExplosionDef DefaultExplosionDef()
	{
		ExplosionDef def = default;
		def.maskBits = DefaultMaskBits;
		return def;
	}

	internal static ref Joint GetJointFullId( World world, JointId jointId )
	{
		int id = jointId.index1 - 1;
		ref Joint joint = ref world.joints[id];
		B2_ASSERT( joint.jointId == id && joint.generation == jointId.generation );
		return ref joint;
	}

	internal static ref JointSim GetJointSim( World world, ref Joint joint )
	{
		if ( joint.setIndex == b2_awakeSet )
		{
			B2_ASSERT( 0 <= joint.colorIndex && joint.colorIndex < GraphColorCount );
			ref GraphColor color = ref world.constraintGraph.colors[joint.colorIndex];
			return ref color.jointSims[joint.localIndex];
		}

		ref SolverSet set = ref world.solverSets[joint.setIndex];
		return ref set.jointSims[joint.localIndex];
	}

	internal static ref JointSim GetJointSimCheckType( JointId jointId, JointType type )
	{
		World world = GetWorld( jointId.world0 );
		B2_ASSERT( world.locked == false );
		if ( world.locked )
		{
			return ref Unsafe.NullRef<JointSim>();
		}

		ref Joint joint = ref GetJointFullId( world, jointId );
		B2_ASSERT( joint.type == type );
		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		B2_ASSERT( jointSim.type == type );
		return ref jointSim;
	}

	private static void DestroyContactsBetweenBodies( World world, ref Body bodyA, ref Body bodyB )
	{
		int contactKey;
		int otherBodyId;

		// use the smaller of the two contact lists
		if ( bodyA.contactCount < bodyB.contactCount )
		{
			contactKey = bodyA.headContactKey;
			otherBodyId = bodyB.id;
		}
		else
		{
			contactKey = bodyB.headContactKey;
			otherBodyId = bodyA.id;
		}

		// destroy the contacts
		while ( contactKey != NullIndex )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			ref Contact contact = ref world.contacts[contactId];
			contactKey = contact.edges[edgeIndex].nextKey;

			int otherEdgeIndex = edgeIndex ^ 1;
			if ( contact.edges[otherEdgeIndex].bodyId == otherBodyId )
			{
				// Careful, this removes the contact from the current doubly linked list
				DestroyContact( world, ref contact );
			}
		}

		ValidateSolverSets( world );
	}

	private static bool IsValidJointDef( in JointDef def )
	{
		return IsValidTransform( def.localFrameA ) && IsValidTransform( def.localFrameB ) &&
			   IsValidFloat( def.forceThreshold ) && def.forceThreshold >= 0.0f && IsValidFloat( def.torqueThreshold ) &&
			   def.torqueThreshold >= 0.0f && IsValidFloat( def.constraintHertz ) && def.constraintHertz >= 0.0f &&
			   IsValidFloat( def.constraintDampingRatio ) && def.constraintDampingRatio >= 0.0f &&
			   IsValidFloat( def.drawScale );
	}

	internal ref struct JointPair
	{
		public ref Joint joint;
		public ref JointSim jointSim;
	}

	private static JointPair CreateJoint( World world, in JointDef def, JointType type )
	{
		B2_ASSERT( IsValidTransform( def.localFrameA ) );
		B2_ASSERT( IsValidTransform( def.localFrameB ) );
		B2_ASSERT( world.worldId == def.bodyIdA.world0 );
		B2_ASSERT( world.worldId == def.bodyIdB.world0 );
		B2_ASSERT( ( def.bodyIdA ).Equals( def.bodyIdB ) == false );

		ref Body bodyA = ref GetBodyFullId( world, def.bodyIdA );
		ref Body bodyB = ref GetBodyFullId( world, def.bodyIdB );
		// If the joint prevents collisions, then destroy all contacts between attached bodies.
		// Do this first so world arrays are not disturbed in the following code.
		if ( def.collideConnected == false )
		{
			DestroyContactsBetweenBodies( world, ref bodyA, ref bodyB );
		}

		int bodyIdA = bodyA.id;
		int bodyIdB = bodyB.id;
		int maxSetIndex = MaxInt( bodyA.setIndex, bodyB.setIndex );

		// Create joint id and joint
		int jointId = AllocId( ref world.jointIdPool );
		if ( jointId == world.joints.count )
		{
			world.joints.Push( default(Joint) );
		}

		ref Joint joint = ref world.joints[jointId];
		joint.jointId = jointId;
		joint.userData = def.userData;
		joint.generation += 1;
		joint.setIndex = NullIndex;
		joint.colorIndex = NullIndex;
		joint.localIndex = NullIndex;
		joint.islandId = NullIndex;
		joint.islandIndex = NullIndex;
		joint.drawScale = def.drawScale;
		joint.type = type;
		joint.collideConnected = def.collideConnected;

		// Doubly linked list on bodyA
		joint.edges[0].bodyId = bodyIdA;
		joint.edges[0].prevKey = NullIndex;
		joint.edges[0].nextKey = bodyA.headJointKey;

		int keyA = ( jointId << 1 ) | 0;
		if ( bodyA.headJointKey != NullIndex )
		{
			ref Joint jointA = ref world.joints[bodyA.headJointKey >> 1];
			ref JointEdge edgeA = ref jointA.edges[( bodyA.headJointKey & 1 )];
			edgeA.prevKey = keyA;
		}
		bodyA.headJointKey = keyA;
		bodyA.jointCount += 1;

		// Doubly linked list on bodyB
		joint.edges[1].bodyId = bodyIdB;
		joint.edges[1].prevKey = NullIndex;
		joint.edges[1].nextKey = bodyB.headJointKey;

		int keyB = ( jointId << 1 ) | 1;
		if ( bodyB.headJointKey != NullIndex )
		{
			ref Joint jointB = ref world.joints[bodyB.headJointKey >> 1];
			ref JointEdge edgeB = ref jointB.edges[( bodyB.headJointKey & 1 )];
			edgeB.prevKey = keyB;
		}
		bodyB.headJointKey = keyB;
		bodyB.jointCount += 1;

		ref JointSim jointSim = ref Unsafe.NullRef<JointSim>();
		if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
		{
			// if either body is disabled, create in disabled set
			ref SolverSet set = ref world.solverSets[b2_disabledSet];
			joint.setIndex = b2_disabledSet;
			joint.localIndex = set.jointSims.count;

			jointSim = ref set.jointSims.Emplace();
			jointSim = default;

			jointSim.jointId = jointId;
			jointSim.bodyIdA = bodyIdA;
			jointSim.bodyIdB = bodyIdB;
		}
		else if ( bodyA.type != BodyType.dynamicBody && bodyB.type != BodyType.dynamicBody )
		{
			// joint is not attached to a dynamic body
			ref SolverSet set = ref world.solverSets[b2_staticSet];
			joint.setIndex = b2_staticSet;
			joint.localIndex = set.jointSims.count;

			jointSim = ref set.jointSims.Emplace();
			jointSim = default;

			jointSim.jointId = jointId;
			jointSim.bodyIdA = bodyIdA;
			jointSim.bodyIdB = bodyIdB;
		}
		else if ( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet )
		{
			// if either body is sleeping, wake it
			if ( maxSetIndex >= b2_firstSleepingSet )
			{
				WakeSolverSet( world, maxSetIndex );
			}

			joint.setIndex = b2_awakeSet;

			jointSim = ref CreateJointInGraph( world, ref joint );
			jointSim.jointId = jointId;
			jointSim.bodyIdA = bodyIdA;
			jointSim.bodyIdB = bodyIdB;
		}
		else
		{
			// joint connected between sleeping and/or static bodies
			B2_ASSERT( bodyA.setIndex >= b2_firstSleepingSet || bodyB.setIndex >= b2_firstSleepingSet );
			B2_ASSERT( bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet );

			// joint should go into the sleeping set (not static set)
			int setIndex = maxSetIndex;

			ref SolverSet set = ref world.solverSets[setIndex];
			joint.setIndex = setIndex;
			joint.localIndex = set.jointSims.count;

			jointSim = ref set.jointSims.Emplace();
			jointSim = default;

			// These must be set to accommodate the merge below
			jointSim.jointId = jointId;
			jointSim.bodyIdA = bodyIdA;
			jointSim.bodyIdB = bodyIdB;

			if ( bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= b2_firstSleepingSet &&
				 bodyB.setIndex >= b2_firstSleepingSet )
			{
				// merge sleeping sets
				MergeSolverSets( world, bodyA.setIndex, bodyB.setIndex );
				B2_ASSERT( bodyA.setIndex == bodyB.setIndex );

				// fix potentially invalid set index
				setIndex = bodyA.setIndex;

				ref SolverSet mergedSet = ref world.solverSets[setIndex];
				// Careful! The joint sim pointer was orphaned by the set merge.
				jointSim = ref mergedSet.jointSims[joint.localIndex];
			}

			B2_ASSERT( joint.setIndex == setIndex );
		}

		jointSim.localFrameA = def.localFrameA;
		jointSim.localFrameB = def.localFrameB;
		jointSim.type = type;
		jointSim.constraintHertz = def.constraintHertz;
		jointSim.constraintDampingRatio = def.constraintDampingRatio;
		jointSim.constraintSoftness = new Softness { biasRate = 0.0f, massScale = 1.0f, impulseScale = 0.0f };

		B2_ASSERT( IsValidFloat( def.forceThreshold ) && def.forceThreshold >= 0.0f );
		B2_ASSERT( IsValidFloat( def.torqueThreshold ) && def.torqueThreshold >= 0.0f );

		jointSim.forceThreshold = def.forceThreshold;
		jointSim.torqueThreshold = def.torqueThreshold;

		B2_ASSERT( jointSim.jointId == jointId );
		B2_ASSERT( jointSim.bodyIdA == bodyIdA );
		B2_ASSERT( jointSim.bodyIdB == bodyIdB );

		if ( joint.setIndex > b2_disabledSet )
		{
			// Add edge to island graph
			LinkJoint( world, ref joint );
		}

		ValidateSolverSets( world );

		return new JointPair { joint = ref joint, jointSim = ref jointSim };
	}

	public static JointId CreateDistanceJoint( WorldId worldId, in DistanceJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.length ) && def.length > 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.hertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.dampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.minLength ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxLength ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxMotorForce ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.motorSpeed ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.lowerSpringForce ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.upperSpringForce ) ) == false )
			{
				return default(JointId);
			};
		B2_ASSERT( def.lowerSpringForce <= def.upperSpringForce );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.distanceJoint );

		ref JointSim joint = ref pair.jointSim;
		DistanceJoint empty = default;
		joint.distanceJoint = empty;
		joint.distanceJoint.length = MaxFloat( def.length, LinearSlop );
		joint.distanceJoint.hertz = def.hertz;
		joint.distanceJoint.dampingRatio = def.dampingRatio;
		joint.distanceJoint.minLength = MaxFloat( def.minLength, LinearSlop );
		joint.distanceJoint.maxLength = MaxFloat( def.minLength, def.maxLength );
		joint.distanceJoint.maxMotorForce = def.maxMotorForce;
		joint.distanceJoint.motorSpeed = def.motorSpeed;
		joint.distanceJoint.enableSpring = def.enableSpring;
		joint.distanceJoint.lowerSpringForce = MinFloat( def.lowerSpringForce, def.upperSpringForce );
		joint.distanceJoint.upperSpringForce = MaxFloat( def.lowerSpringForce, def.upperSpringForce );
		joint.distanceJoint.enableLimit = def.enableLimit;
		joint.distanceJoint.enableMotor = def.enableMotor;
		joint.distanceJoint.impulse = 0.0f;
		joint.distanceJoint.lowerImpulse = 0.0f;
		joint.distanceJoint.upperImpulse = 0.0f;
		joint.distanceJoint.motorImpulse = 0.0f;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateFilterJoint( WorldId worldId, in FilterJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.filterJoint );

		ref JointSim joint = ref pair.jointSim;
		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateMotorJoint( WorldId worldId, in MotorJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidVec2( def.linearVelocity ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxVelocityForce ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.angularVelocity ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxVelocityTorque ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.linearHertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.linearDampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxSpringForce ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.angularHertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.angularDampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxSpringTorque ) ) == false )
			{
				return default(JointId);
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.motorJoint );
		ref JointSim joint = ref pair.jointSim;
		joint.motorJoint = default(MotorJoint);
		joint.motorJoint.linearVelocity = def.linearVelocity;
		joint.motorJoint.maxVelocityForce = def.maxVelocityForce;
		joint.motorJoint.angularVelocity = def.angularVelocity;
		joint.motorJoint.maxVelocityTorque = def.maxVelocityTorque;
		joint.motorJoint.linearHertz = def.linearHertz;
		joint.motorJoint.linearDampingRatio = def.linearDampingRatio;
		joint.motorJoint.maxSpringForce = def.maxSpringForce;
		joint.motorJoint.angularHertz = def.angularHertz;
		joint.motorJoint.angularDampingRatio = def.angularDampingRatio;
		joint.motorJoint.maxSpringTorque = def.maxSpringTorque;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateMoverJoint( WorldId worldId, in MoverJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidVec2( def.linearVelocity ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidVec2( def.maxVelocityForce ) ) == false )
			{
				return default(JointId);
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.moverJoint );
		ref JointSim joint = ref pair.jointSim;
		joint.moverJoint = default(MoverJoint);
		joint.moverJoint.linearVelocity = def.linearVelocity;
		joint.moverJoint.maxVelocityForce = def.maxVelocityForce;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreatePogoJoint( WorldId worldId, in PogoJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidVec2( def.normal ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.restLength ) && def.restLength >= 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.hertz ) && def.hertz >= 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.dampingRatio ) && def.dampingRatio >= 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxTensionForce ) && def.maxTensionForce >= 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxCompressionForce ) && def.maxCompressionForce >= 0.0f ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.impulse ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.velocity ) ) == false )
			{
				return default(JointId);
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.pogoJoint );

		ref JointSim joint = ref pair.jointSim;
		joint.pogoJoint = default(PogoJoint);
		joint.pogoJoint.restLength = def.restLength;
		joint.pogoJoint.hertz = def.hertz;
		joint.pogoJoint.dampingRatio = def.dampingRatio;
		joint.pogoJoint.maxTensionForce = def.maxTensionForce;
		joint.pogoJoint.maxCompressionForce = def.maxCompressionForce;
		joint.pogoJoint.normal = def.normal;
		joint.pogoJoint.impulse = def.impulse;
		joint.pogoJoint.velocity = def.velocity;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreatePrismaticJoint( WorldId worldId, in PrismaticJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.hertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.dampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.targetTranslation ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.lowerTranslation ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.upperTranslation ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxMotorForce ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.motorSpeed ) ) == false )
			{
				return default(JointId);
			};
		B2_ASSERT( def.lowerTranslation <= def.upperTranslation );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.prismaticJoint );

		ref JointSim joint = ref pair.jointSim;
		joint.prismaticJoint = default(PrismaticJoint);
		joint.prismaticJoint.hertz = def.hertz;
		joint.prismaticJoint.dampingRatio = def.dampingRatio;
		joint.prismaticJoint.targetTranslation = def.targetTranslation;
		joint.prismaticJoint.lowerTranslation = MinFloat( def.lowerTranslation, def.upperTranslation );
		joint.prismaticJoint.upperTranslation = MaxFloat( def.lowerTranslation, def.upperTranslation );
		joint.prismaticJoint.maxMotorForce = def.maxMotorForce;
		joint.prismaticJoint.motorSpeed = def.motorSpeed;
		joint.prismaticJoint.enableSpring = def.enableSpring;
		joint.prismaticJoint.enableLimit = def.enableLimit;
		joint.prismaticJoint.enableMotor = def.enableMotor;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateRevoluteJoint( WorldId worldId, in RevoluteJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.targetAngle ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.hertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.dampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.lowerAngle ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.upperAngle ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxMotorTorque ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.motorSpeed ) ) == false )
			{
				return default(JointId);
			};
		B2_ASSERT( def.lowerAngle <= def.upperAngle );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.revoluteJoint );

		ref JointSim joint = ref pair.jointSim;
		RevoluteJoint empty = default;
		joint.revoluteJoint = empty;

		joint.revoluteJoint.targetAngle = ClampFloat( def.targetAngle, -Pi, Pi );
		joint.revoluteJoint.hertz = def.hertz;
		joint.revoluteJoint.dampingRatio = def.dampingRatio;

		float lowerAngle = MinFloat( def.lowerAngle, def.upperAngle );
		float upperAngle = MaxFloat( def.lowerAngle, def.upperAngle );
		joint.revoluteJoint.lowerAngle = ClampFloat( lowerAngle, -0.99f * Pi, 0.99f * Pi );
		joint.revoluteJoint.upperAngle = ClampFloat( upperAngle, -0.99f * Pi, 0.99f * Pi );

		joint.revoluteJoint.maxMotorTorque = def.maxMotorTorque;
		joint.revoluteJoint.motorSpeed = def.motorSpeed;
		joint.revoluteJoint.enableSpring = def.enableSpring;
		joint.revoluteJoint.enableLimit = def.enableLimit;
		joint.revoluteJoint.enableMotor = def.enableMotor;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateWeldJoint( WorldId worldId, in WeldJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.linearHertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.angularHertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.linearDampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.angularDampingRatio ) ) == false )
			{
				return default(JointId);
			};

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.weldJoint );

		ref JointSim joint = ref pair.jointSim;
		WeldJoint empty = default;
		joint.weldJoint = empty;
		joint.weldJoint.linearHertz = def.linearHertz;
		joint.weldJoint.linearDampingRatio = def.linearDampingRatio;
		joint.weldJoint.angularHertz = def.angularHertz;
		joint.weldJoint.angularDampingRatio = def.angularDampingRatio;
		joint.weldJoint.linearImpulse = Vec2_zero;
		joint.weldJoint.angularImpulse = 0.0f;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	public static JointId CreateWheelJoint( WorldId worldId, in WheelJointDef def )
	{
		B2_ASSERT( def.internalValue == SecretCookie );
		if ( CheckInput( IsValidJointDef( def.@base ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.hertz ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.dampingRatio ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.lowerTranslation ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.upperTranslation ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.maxMotorTorque ) ) == false )
			{
				return default(JointId);
			};
		if ( CheckInput( IsValidFloat( def.motorSpeed ) ) == false )
			{
				return default(JointId);
			};
		B2_ASSERT( def.lowerTranslation <= def.upperTranslation );

		World world = GetWorldFromId( worldId );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return default(JointId);
		}

		JointPair pair = CreateJoint( world, def.@base, JointType.wheelJoint );

		ref JointSim joint = ref pair.jointSim;
		joint.wheelJoint = default(WheelJoint);
		joint.wheelJoint.perpMass = 0.0f;
		joint.wheelJoint.axialMass = 0.0f;
		joint.wheelJoint.motorImpulse = 0.0f;
		joint.wheelJoint.lowerImpulse = 0.0f;
		joint.wheelJoint.upperImpulse = 0.0f;
		joint.wheelJoint.lowerTranslation = MinFloat( def.lowerTranslation, def.upperTranslation );
		joint.wheelJoint.upperTranslation = MaxFloat( def.lowerTranslation, def.upperTranslation );
		joint.wheelJoint.maxMotorTorque = def.maxMotorTorque;
		joint.wheelJoint.motorSpeed = def.motorSpeed;
		joint.wheelJoint.hertz = def.hertz;
		joint.wheelJoint.dampingRatio = def.dampingRatio;
		joint.wheelJoint.enableSpring = def.enableSpring;
		joint.wheelJoint.enableLimit = def.enableLimit;
		joint.wheelJoint.enableMotor = def.enableMotor;

		JointId jointId = new JointId { index1 = joint.jointId + 1, world0 = world.worldId, generation = pair.joint.generation };


		return jointId;
	}

	internal static void DestroyJointInternal( World world, ref Joint joint )
	{
		int jointId = joint.jointId;

		ref JointEdge edgeA = ref joint.edges[0];
		ref JointEdge edgeB = ref joint.edges[1];
		int idA = edgeA.bodyId;
		int idB = edgeB.bodyId;
		ref Body bodyA = ref world.bodies[idA];
		ref Body bodyB = ref world.bodies[idB];
		// Remove from body A
		if ( edgeA.prevKey != NullIndex )
		{
			ref Joint prevJoint = ref world.joints[edgeA.prevKey >> 1];
			ref JointEdge prevEdge = ref prevJoint.edges[( edgeA.prevKey & 1 )];
			prevEdge.nextKey = edgeA.nextKey;
		}

		if ( edgeA.nextKey != NullIndex )
		{
			ref Joint nextJoint = ref world.joints[edgeA.nextKey >> 1];
			ref JointEdge nextEdge = ref nextJoint.edges[( edgeA.nextKey & 1 )];
			nextEdge.prevKey = edgeA.prevKey;
		}

		int edgeKeyA = ( jointId << 1 ) | 0;
		if ( bodyA.headJointKey == edgeKeyA )
		{
			bodyA.headJointKey = edgeA.nextKey;
		}

		bodyA.jointCount -= 1;

		// Remove from body B
		if ( edgeB.prevKey != NullIndex )
		{
			ref Joint prevJoint = ref world.joints[edgeB.prevKey >> 1];
			ref JointEdge prevEdge = ref prevJoint.edges[( edgeB.prevKey & 1 )];
			prevEdge.nextKey = edgeB.nextKey;
		}

		if ( edgeB.nextKey != NullIndex )
		{
			ref Joint nextJoint = ref world.joints[edgeB.nextKey >> 1];
			ref JointEdge nextEdge = ref nextJoint.edges[( edgeB.nextKey & 1 )];
			nextEdge.prevKey = edgeB.prevKey;
		}

		int edgeKeyB = ( jointId << 1 ) | 1;
		if ( bodyB.headJointKey == edgeKeyB )
		{
			bodyB.headJointKey = edgeB.nextKey;
		}

		bodyB.jointCount -= 1;

		if ( joint.islandId != NullIndex )
		{
			B2_ASSERT( joint.setIndex > b2_disabledSet );
			UnlinkJoint( world, ref joint );
		}
		else
		{
			B2_ASSERT( joint.setIndex <= b2_disabledSet );
		}

		// Remove joint from solver set that owns it
		int setIndex = joint.setIndex;
		int localIndex = joint.localIndex;

		if ( setIndex == b2_awakeSet )
		{
			RemoveJointFromGraph( world, joint.edges[0].bodyId, joint.edges[1].bodyId, joint.colorIndex, localIndex );
		}
		else
		{
			ref SolverSet set = ref world.solverSets[setIndex];
			int movedIndex = set.jointSims.RemoveSwap( localIndex );
			if ( movedIndex != NullIndex )
			{
				// Fix moved joint
				ref JointSim movedJointSim = ref set.jointSims.data[localIndex];
				int movedId = movedJointSim.jointId;
				ref Joint movedJoint = ref world.joints[movedId];
				B2_ASSERT( movedJoint.localIndex == movedIndex );
				movedJoint.localIndex = localIndex;
			}
		}

		// Free joint and id (preserve joint generation)
		joint.setIndex = NullIndex;
		joint.localIndex = NullIndex;
		joint.colorIndex = NullIndex;
		joint.jointId = NullIndex;
		FreeId( ref world.jointIdPool, jointId );

		WakeBody( world, ref bodyA );
		WakeBody( world, ref bodyB );

		ValidateSolverSets( world );
	}

	public static void DestroyJoint( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		B2_ASSERT( world.locked == false );

		if ( world.locked )
		{
			return;
		}


		ref Joint joint = ref GetJointFullId( world, jointId );
		DestroyJointInternal( world, ref joint );
	}

	public static JointType Joint_GetType( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return joint.type;
	}

	public static BodyId Joint_GetBodyA( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return MakeBodyId( world, joint.edges[0].bodyId );
	}

	public static BodyId Joint_GetBodyB( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return MakeBodyId( world, joint.edges[1].bodyId );
	}

	public static WorldId Joint_GetWorld( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		return new WorldId { index1 = (ushort)( jointId.world0 + 1 ), generation = world.generation };
	}

	public static void Joint_SetLocalFrameA( JointId jointId, Transform localFrame )
	{
		if ( CheckInput( IsValidTransform( localFrame ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		jointSim.localFrameA = localFrame;
	}

	public static Transform Joint_GetLocalFrameA( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		return jointSim.localFrameA;
	}

	public static void Joint_SetLocalFrameB( JointId jointId, Transform localFrame )
	{
		if ( CheckInput( IsValidTransform( localFrame ) ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		jointSim.localFrameB = localFrame;
	}

	public static Transform Joint_GetLocalFrameB( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		return jointSim.localFrameB;
	}

	public static void Joint_SetCollideConnected( JointId jointId, bool shouldCollide )
	{
		World world = GetWorldLocked( jointId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Joint joint = ref GetJointFullId( world, jointId );
		if ( joint.collideConnected == shouldCollide )
		{
			return;
		}

		joint.collideConnected = shouldCollide;

		ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
		ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
		if ( shouldCollide )
		{
			BodyType typeA = bodyA.type;
			BodyType typeB = bodyB.type;

			int shapeId = NullIndex;
			if (typeA == BodyType.dynamicBody && typeB == BodyType.dynamicBody)
			{
				shapeId = bodyA.shapeCount < bodyB.shapeCount ? bodyA.headShapeId : bodyB.headShapeId;
			}
			else if ( typeA == BodyType.dynamicBody )
			{
				shapeId = bodyA.headShapeId;
			}
			else if ( typeB == BodyType.dynamicBody )
			{
				shapeId = bodyB.headShapeId;
			}

			while ( shapeId != NullIndex )
			{
				ref Shape shape = ref world.shapes[shapeId];
				if ( shape.proxyKey != NullIndex )
				{
					BroadPhase_MarkProxyMovedSerial( world.broadPhase, shape.proxyKey );
				}

				shapeId = shape.nextShapeId;
			}
		}
		else
		{
			DestroyContactsBetweenBodies( world, ref bodyA, ref bodyB );
		}
	}

	public static bool Joint_GetCollideConnected( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return joint.collideConnected;
	}

	public static void Joint_SetUserData( JointId jointId, object? userData )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		joint.userData = userData;
	}

	public static object? Joint_GetUserData( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return joint.userData;
	}

	public static void Joint_WakeBodies( JointId jointId )
	{
		World world = GetWorldLocked( jointId.world0 );
		if ( world == null )
		{
			return;
		}


		ref Joint joint = ref GetJointFullId( world, jointId );
		ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
		ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
		WakeBody( world, ref bodyA );
		WakeBody( world, ref bodyB );
	}

	internal static void GetJointReaction( ref JointSim sim, float invTimeStep, ref float force, ref float torque )
	{
		float linearImpulse = 0.0f;
		float angularImpulse = 0.0f;

		switch ( sim.type )
		{
			case JointType.distanceJoint:
			{
				ref DistanceJoint joint = ref sim.distanceJoint;
				linearImpulse = AbsFloat( joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse );
			}
			break;

			case JointType.motorJoint:
			{
				ref MotorJoint joint = ref sim.motorJoint;
				linearImpulse = Length( Add( joint.linearVelocityImpulse, joint.linearSpringImpulse ) );
				angularImpulse = AbsFloat( joint.angularVelocityImpulse + joint.angularSpringImpulse );
			}
			break;

			case JointType.moverJoint:
			{
				ref MoverJoint joint = ref sim.moverJoint;
				linearImpulse = Length( joint.linearVelocityImpulse );
			}
			break;

			case JointType.pogoJoint:
			{
				ref PogoJoint joint = ref sim.pogoJoint;
				linearImpulse = AbsFloat( joint.impulse );
			}
			break;

			case JointType.prismaticJoint:
			{
				ref PrismaticJoint joint = ref sim.prismaticJoint;
				float perpImpulse = joint.impulse.x;
				float axialImpulse = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
				linearImpulse = MathF.Sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
				angularImpulse = AbsFloat( joint.impulse.y );
			}
			break;

			case JointType.revoluteJoint:
			{
				ref RevoluteJoint joint = ref sim.revoluteJoint;
				linearImpulse = Length( joint.linearImpulse );
				angularImpulse = AbsFloat( joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse );
			}
			break;

			case JointType.weldJoint:
			{
				ref WeldJoint joint = ref sim.weldJoint;
				linearImpulse = Length( joint.linearImpulse );
				angularImpulse = AbsFloat( joint.angularImpulse );
			}
			break;

			case JointType.wheelJoint:
			{
				ref WheelJoint joint = ref sim.wheelJoint;
				float perpImpulse = joint.perpImpulse;
				float axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;
				linearImpulse = MathF.Sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
				angularImpulse = AbsFloat( joint.motorImpulse );
			}
			break;

			default:
				break;
		}

		force = linearImpulse * invTimeStep;
		torque = angularImpulse * invTimeStep;
	}

	private static Vec2 GetJointConstraintForce( World world, ref Joint joint )
	{
		ref JointSim @base = ref GetJointSim( world, ref joint );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
				return GetDistanceJointForce( world, ref @base );

			case JointType.motorJoint:
				return GetMotorJointForce( world, ref @base );

			case JointType.moverJoint:
				return GetMoverJointForce( world, ref @base );

			case JointType.filterJoint:
				return Vec2_zero;

			case JointType.pogoJoint:
				return GetPogoJointForce( world, ref @base );

			case JointType.prismaticJoint:
				return GetPrismaticJointForce( world, ref @base );

			case JointType.revoluteJoint:
				return GetRevoluteJointForce( world, ref @base );

			case JointType.weldJoint:
				return GetWeldJointForce( world, ref @base );

			case JointType.wheelJoint:
				return GetWheelJointForce( world, ref @base );

			default:
				B2_ASSERT( false );
				return Vec2_zero;
		}
	}

	private static float GetJointConstraintTorque( World world, ref Joint joint )
	{
		ref JointSim @base = ref GetJointSim( world, ref joint );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
				return 0.0f;

			case JointType.motorJoint:
				return GetMotorJointTorque( world, ref @base );

			case JointType.filterJoint:
				return 0.0f;

			case JointType.prismaticJoint:
				return GetPrismaticJointTorque( world, ref @base );

			case JointType.revoluteJoint:
				return GetRevoluteJointTorque( world, ref @base );

			case JointType.weldJoint:
				return GetWeldJointTorque( world, ref @base );

			case JointType.wheelJoint:
				return GetWheelJointTorque( world, ref @base );

			default:
				B2_ASSERT( false );
				return 0.0f;
		}
	}

	public static Vec2 Joint_GetConstraintForce( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return GetJointConstraintForce( world, ref joint );
	}

	public static float Joint_GetConstraintTorque( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		return GetJointConstraintTorque( world, ref joint );
	}

	public static float Joint_GetLinearSeparation( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );

		// Relative to body A so the difference stays in float precision far from the origin
		WorldTransform wxfA = GetBodyTransform( world, joint.edges[0].bodyId );
		Transform xfA = ToRelativeTransform( wxfA, wxfA.p );
		Transform xfB = ToRelativeTransform( GetBodyTransform( world, joint.edges[1].bodyId ), wxfA.p );

		Vec2 pA = TransformPoint( xfA, @base.localFrameA.p );
		Vec2 pB = TransformPoint( xfB, @base.localFrameB.p );
		Vec2 dp = Sub( pB, pA );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
			{
				ref DistanceJoint distanceJoint = ref @base.distanceJoint;
				float length = Length( dp );
				if ( distanceJoint.enableSpring )
				{
					if ( distanceJoint.enableLimit )
					{
						if ( length < distanceJoint.minLength )
						{
							return distanceJoint.minLength - length;
						}

						if ( length > distanceJoint.maxLength )
						{
							return length - distanceJoint.maxLength;
						}

						return 0.0f;
					}

					return 0.0f;
				}

				return AbsFloat( length - distanceJoint.length );
			}

			case JointType.motorJoint:
				return 0.0f;

			case JointType.filterJoint:
				return 0.0f;

			case JointType.prismaticJoint:
			{
				ref PrismaticJoint prismaticJoint = ref @base.prismaticJoint;
				Vec2 axisA = RotateVector( xfA.q, new Vec2( 1.0f, 0.0f ) );
				Vec2 perpA = LeftPerp( axisA );
				float perpendicularSeparation = AbsFloat( Dot( perpA, dp ) );
				float limitSeparation = 0.0f;

				if ( prismaticJoint.enableLimit )
				{
					float translation = Dot( axisA, dp );
					if ( translation < prismaticJoint.lowerTranslation )
					{
						limitSeparation = prismaticJoint.lowerTranslation - translation;
					}

					if ( prismaticJoint.upperTranslation < translation )
					{
						limitSeparation = translation - prismaticJoint.upperTranslation;
					}
				}

				return MathF.Sqrt( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
			}

			case JointType.revoluteJoint:
				return Length( dp );

			case JointType.weldJoint:
			{
				ref WeldJoint weldJoint = ref @base.weldJoint;
				if ( weldJoint.linearHertz == 0.0f )
				{
					return Length( dp );
				}

				return 0.0f;
			}

			case JointType.wheelJoint:
			{
				ref WheelJoint wheelJoint = ref @base.wheelJoint;
				Vec2 axisA = RotateVector( xfA.q, new Vec2( 1.0f, 0.0f ) );
				Vec2 perpA = LeftPerp( axisA );
				float perpendicularSeparation = AbsFloat( Dot( perpA, dp ) );
				float limitSeparation = 0.0f;

				if ( wheelJoint.enableLimit )
				{
					float translation = Dot( axisA, dp );
					if ( translation < wheelJoint.lowerTranslation )
					{
						limitSeparation = wheelJoint.lowerTranslation - translation;
					}

					if ( wheelJoint.upperTranslation < translation )
					{
						limitSeparation = translation - wheelJoint.upperTranslation;
					}
				}

				return MathF.Sqrt( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
			}

			default:
				B2_ASSERT( false );
				return 0.0f;
		}
	}

	public static float Joint_GetAngularSeparation( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );

		Rot qA = GetBodyTransform( world, joint.edges[0].bodyId ).q;
		Rot qB = GetBodyTransform( world, joint.edges[1].bodyId ).q;
		float relativeAngle = RelativeAngle( qA, qB );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
				return 0.0f;

			case JointType.motorJoint:
				return 0.0f;

			case JointType.filterJoint:
				return 0.0f;

			case JointType.prismaticJoint:
			{
				return relativeAngle;
			}

			case JointType.revoluteJoint:
			{
				ref RevoluteJoint revoluteJoint = ref @base.revoluteJoint;
				if ( revoluteJoint.enableLimit )
				{
					float angle = relativeAngle;
					if ( angle < revoluteJoint.lowerAngle )
					{
						return revoluteJoint.lowerAngle - angle;
					}

					if ( revoluteJoint.upperAngle < angle )
					{
						return angle - revoluteJoint.upperAngle;
					}
				}

				return 0.0f;
			}

			case JointType.weldJoint:
			{
				ref WeldJoint weldJoint = ref @base.weldJoint;
				if ( weldJoint.angularHertz == 0.0f )
				{
					return relativeAngle;
				}

				return 0.0f;
			}

			case JointType.wheelJoint:
				return 0.0f;

			default:
				B2_ASSERT( false );
				return 0.0f;
		}
	}

	public static void Joint_SetConstraintTuning( JointId jointId, float hertz, float dampingRatio )
	{
		if ( CheckInput( IsValidFloat( hertz ) && hertz >= 0.0f ) == false )
			{
				return;
			};
		if ( CheckInput( IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		@base.constraintHertz = hertz;
		@base.constraintDampingRatio = dampingRatio;
	}

	public static void Joint_GetConstraintTuning( JointId jointId, ref float hertz, ref float dampingRatio )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		hertz = @base.constraintHertz;
		dampingRatio = @base.constraintDampingRatio;
	}

	public static void Joint_SetForceThreshold( JointId jointId, float threshold )
	{
		if ( CheckInput( IsValidFloat( threshold ) && threshold >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		@base.forceThreshold = threshold;
	}

	public static float Joint_GetForceThreshold( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		return @base.forceThreshold;
	}

	public static void Joint_SetTorqueThreshold( JointId jointId, float threshold )
	{
		if ( CheckInput( IsValidFloat( threshold ) && threshold >= 0.0f ) == false )
			{
				return;
			};

		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		@base.torqueThreshold = threshold;
	}

	public static float Joint_GetTorqueThreshold( JointId jointId )
	{
		World world = GetWorld( jointId.world0 );
		ref Joint joint = ref GetJointFullId( world, jointId );
		ref JointSim @base = ref GetJointSim( world, ref joint );
		return @base.torqueThreshold;
	}

	internal static void PrepareJoint( ref JointSim joint, ref StepContext context )
	{
		// Clamp joint hertz based on the time step to reduce jitter.
		float hertz = MinFloat( joint.constraintHertz, 0.25f * context.inv_h );
		joint.constraintSoftness = MakeSoft( hertz, joint.constraintDampingRatio, context.h );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
				PrepareDistanceJoint( ref joint, ref context );
				break;

			case JointType.motorJoint:
				PrepareMotorJoint( ref joint, ref context );
				break;

			case JointType.moverJoint:
				PrepareMoverJoint( ref joint, ref context );
				break;

			case JointType.filterJoint:
				break;

			case JointType.pogoJoint:
				PreparePogoJoint( ref joint, ref context );
				break;

			case JointType.prismaticJoint:
				PreparePrismaticJoint( ref joint, ref context );
				break;

			case JointType.revoluteJoint:
				PrepareRevoluteJoint( ref joint, ref context );
				break;

			case JointType.weldJoint:
				PrepareWeldJoint( ref joint, ref context );
				break;

			case JointType.wheelJoint:
				PrepareWheelJoint( ref joint, ref context );
				break;

			default:
				B2_ASSERT( false );
				break;
		}
	}

	internal static void WarmStartJoint( ref JointSim joint, ref StepContext context )
	{
		switch ( joint.type )
		{
			case JointType.distanceJoint:
				WarmStartDistanceJoint( ref joint, ref context );
				break;

			case JointType.motorJoint:
				WarmStartMotorJoint( ref joint, ref context );
				break;

			case JointType.moverJoint:
				WarmStartMoverJoint( ref joint, ref context );
				break;

			case JointType.filterJoint:
				break;

			case JointType.pogoJoint:
				WarmStartPogoJoint( ref joint, ref context );
				break;

			case JointType.prismaticJoint:
				WarmStartPrismaticJoint( ref joint, ref context );
				break;

			case JointType.revoluteJoint:
				WarmStartRevoluteJoint( ref joint, ref context );
				break;

			case JointType.weldJoint:
				WarmStartWeldJoint( ref joint, ref context );
				break;

			case JointType.wheelJoint:
				WarmStartWheelJoint( ref joint, ref context );
				break;

			default:
				B2_ASSERT( false );
				break;
		}
	}

	internal static void SolveJoint( ref JointSim joint, ref StepContext context, bool useBias )
	{
		switch ( joint.type )
		{
			case JointType.distanceJoint:
				SolveDistanceJoint( ref joint, ref context, useBias );
				break;

			case JointType.motorJoint:
				SolveMotorJoint( ref joint, ref context );
				break;

			case JointType.moverJoint:
				SolveMoverJoint( ref joint, ref context );
				break;

			case JointType.filterJoint:
				break;

			case JointType.pogoJoint:
				SolvePogoJoint( ref joint, ref context, useBias );
				break;

			case JointType.prismaticJoint:
				SolvePrismaticJoint( ref joint, ref context, useBias );
				break;

			case JointType.revoluteJoint:
				SolveRevoluteJoint( ref joint, ref context, useBias );
				break;

			case JointType.weldJoint:
				SolveWeldJoint( ref joint, ref context, useBias );
				break;

			case JointType.wheelJoint:
				SolveWheelJoint( ref joint, ref context, useBias );
				break;

			default:
				B2_ASSERT( false );
				break;
		}
	}

	internal static void PrepareJoints_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		Span<JointSim> joints = graph.colors[OverflowIndex].jointSims.data;
		int jointCount = graph.colors[OverflowIndex].jointSims.count;

		for ( int i = 0; i < jointCount; ++i )
		{
			ref JointSim joint = ref joints[i];
			PrepareJoint( ref joint, ref context );
		}
	}

	internal static void WarmStartJoints_Overflow( ref StepContext context )
	{
		ConstraintGraph graph = context.graph;
		Span<JointSim> joints = graph.colors[OverflowIndex].jointSims.data;
		int jointCount = graph.colors[OverflowIndex].jointSims.count;

		for ( int i = 0; i < jointCount; ++i )
		{
			ref JointSim joint = ref joints[i];
			WarmStartJoint( ref joint, ref context );
		}
	}

	internal static void SolveJoints_Overflow( ref StepContext context, bool useBias )
	{
		ConstraintGraph graph = context.graph;
		Span<JointSim> joints = graph.colors[OverflowIndex].jointSims.data;
		int jointCount = graph.colors[OverflowIndex].jointSims.count;

		for ( int i = 0; i < jointCount; ++i )
		{
			ref JointSim joint = ref joints[i];
			SolveJoint( ref joint, ref context, useBias );
		}
	}

	internal static void PrepareJointsTask( SolverBlock block, ref StepContext context )
	{
		Span<JointPrepareSpan> spans = context.jointPrepareSpans;
		int index = block.startIndex;
		int endIndex = block.startIndex + block.count;

		// Find color for start index. Linear search but fast.
		int colorIndex = 0;
		while ( spans[colorIndex + 1].start <= index )
		{
			colorIndex += 1;
		}

		// Loop over block
		while ( index < endIndex )
		{
			int colorStart = spans[colorIndex].start;
			int colorEndIndex = MinInt( spans[colorIndex + 1].start, endIndex );
			Span<JointSim> joints = spans[colorIndex].joints;
			// Loop over color
			for ( ; index < colorEndIndex; ++index )
			{
				B2_ASSERT( 0 <= index - colorStart && index - colorStart < spans[colorIndex].count );
				ref JointSim joint = ref joints[( index - colorStart )];
				PrepareJoint( ref joint, ref context );
			}

			// Advance to next color
			colorIndex += 1;
		}
	}

	internal static void WarmStartJointsTask( SolverBlock block, ref StepContext context )
	{
		ref GraphColor color = ref context.graph.colors[block.colorIndex];
		Span<JointSim> joints = color.jointSims.data;
		for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
		{
			ref JointSim joint = ref joints[i];
			WarmStartJoint( ref joint, ref context );
		}
	}

	internal static void SolveJointsTask( SolverBlock block, ref StepContext context, bool useBias, int workerIndex )
	{
		ref GraphColor color = ref context.graph.colors[block.colorIndex];
		Span<JointSim> joints = color.jointSims.data;
		B2_ASSERT( 0 <= block.startIndex && block.startIndex + block.count <= color.jointSims.count );

		ref BitSet jointStateBitSet = ref context.world.taskContexts.data[workerIndex].jointStateBitSet;
		for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
		{
			ref JointSim joint = ref joints[i];
			SolveJoint( ref joint, ref context, useBias );

			if ( useBias && ( joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX ) &&
				 GetBit( jointStateBitSet, joint.jointId ) == false )
			{
				float force = 0.0f, torque = 0.0f;
				GetJointReaction( ref joint, context.inv_h, ref force, ref torque );

				// Check thresholds. A zero threshold means all awake joints get reported.
				if ( force >= joint.forceThreshold || torque >= joint.torqueThreshold )
				{
					// Flag this joint for processing.
					SetBit( ref jointStateBitSet, joint.jointId );
				}
			}
		}
	}

	internal static void DrawJoint( ref DebugDraw draw, World world, ref Joint joint )
	{
		ref Body bodyA = ref world.bodies[joint.edges[0].bodyId];
		ref Body bodyB = ref world.bodies[joint.edges[1].bodyId];
		if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
		{
			return;
		}

		ref JointSim jointSim = ref GetJointSim( world, ref joint );
		WorldTransform xfA = GetBodyTransformQuick( world, ref bodyA );
		WorldTransform xfB = GetBodyTransformQuick( world, ref bodyB );

		Pos pA = TransformWorldPoint( xfA, jointSim.localFrameA.p );
		Pos pB = TransformWorldPoint( xfB, jointSim.localFrameB.p );

		float scale = MaxFloat( 0.0001f, draw.jointScale * joint.drawScale );

		switch ( joint.type )
		{
			case JointType.distanceJoint:
				DrawDistanceJoint( ref draw, ref jointSim, xfA, xfB );
				break;

			case JointType.filterJoint:
				draw.DrawPointFcn( pA, 8.0f, HexColor.colorLightSkyBlue, draw.context );
				draw.DrawPointFcn( pB, 8.0f, HexColor.colorLightSkyBlue, draw.context );
				if (draw.drawJointExtras)
				{
					draw.DrawLineFcn( pA, pB, HexColor.colorGold, draw.context );
				}
				break;

			case JointType.motorJoint:
				draw.DrawPointFcn( pA, 8.0f, HexColor.colorYellowGreen, draw.context );
				draw.DrawPointFcn( pB, 8.0f, HexColor.colorPlum, draw.context );
				if (draw.drawJointExtras)
				{
					draw.DrawLineFcn( pA, pB, HexColor.colorLightGray, draw.context );
				}
				break;

			case JointType.moverJoint:
				draw.DrawPointFcn( pB, 8.0f, HexColor.colorPlum, draw.context );
				break;

			case JointType.pogoJoint:
				draw.DrawLineFcn( pA, pB, HexColor.colorLightGray, draw.context );
				break;

			case JointType.prismaticJoint:
				DrawPrismaticJoint( ref draw, ref jointSim, xfA, xfB, scale );
				break;

			case JointType.revoluteJoint:
				DrawRevoluteJoint( ref draw, ref jointSim, xfA, xfB, scale );
				break;

			case JointType.weldJoint:
				DrawWeldJoint( ref draw, ref jointSim, xfA, xfB, scale );
				break;

			case JointType.wheelJoint:
				DrawWheelJoint( ref draw, ref jointSim, xfA, xfB, scale );
				break;

			default:
				break;
		}

		if ( draw.drawGraphColors )
		{
			int colorIndex = joint.colorIndex;
			if ( colorIndex != NullIndex )
			{
				Pos p = LerpPos( pA, pB, 0.5f );
				draw.DrawPointFcn( p, 5.0f, GetGraphColor( colorIndex ), draw.context );
			}
		}

		if ( draw.drawJointExtras )
		{
			Vec2 force = GetJointConstraintForce( world, ref joint );
			float torque = GetJointConstraintTorque( world, ref joint );
			Pos p = LerpPos( pA, pB, 0.5f );

			draw.DrawLineFcn( p, OffsetPos( p, MulSV( 0.001f, force ) ), HexColor.colorAzure, draw.context );

			string buffer = $"f = [{force.x:G4}, {force.y:G4}], t = {torque:G4}";
			draw.DrawStringFcn( p, buffer, HexColor.colorAzure, draw.context );
		}
	}

}
