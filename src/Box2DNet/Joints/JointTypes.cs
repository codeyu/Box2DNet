// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	/// A joint edge is used to connect bodies and joints together
	/// in a joint graph where each body is a node and each joint
	/// is an edge. A joint edge belongs to a doubly linked list
	/// maintained in each attached body. Each joint has two joint
	/// nodes, one for each attached body.
	internal struct JointEdge
	{
		public int bodyId;
		public int prevKey;
		public int nextKey;
	}

	// Map from JointId to Joint in the solver sets
	internal struct Joint
	{
		public object? userData;

		// index of simulation set stored in World
		// NullIndex when slot is free
		public int setIndex;

		// index into the constraint graph color array, may be NullIndex for sleeping/disabled joints
		// NullIndex when slot is free
		public int colorIndex;

		// joint index within set or graph color
		// NullIndex when slot is free
		public int localIndex;

		public FixedArray2<JointEdge> edges;

		public int jointId;
		public int islandId;

		// Index into the island's joints array for O(1) swap-removal.
		// NullIndex when not in an island.
		public int islandIndex;

		public float drawScale;

		public JointType type;

		// This is monotonically advanced when a body is allocated in this slot
		// Used to check for invalid JointId
		public ushort generation;

		public bool collideConnected;

	}

	// The pogo joint is a hybrid between a contact and a joint
	internal struct PogoJoint
	{
		// Settings
		public Vec2 normal;
		public float restLength;
		public float hertz;
		public float dampingRatio;
		public float maxTensionForce;
		public float maxCompressionForce;

		// Runtime
		public float impulse;
		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public float linearMass;
		public float velocity;
	}

	internal struct DistanceJoint
	{
		public float length;
		public float hertz;
		public float dampingRatio;
		public float lowerSpringForce;
		public float upperSpringForce;
		public float minLength;
		public float maxLength;

		public float maxMotorForce;
		public float motorSpeed;

		public float impulse;
		public float lowerImpulse;
		public float upperImpulse;
		public float motorImpulse;

		public int indexA;
		public int indexB;
		public Vec2 anchorA;
		public Vec2 anchorB;
		public Vec2 deltaCenter;
		public Softness distanceSoftness;
		public float axialMass;

		public bool enableSpring;
		public bool enableLimit;
		public bool enableMotor;
	}

	internal struct MotorJoint
	{
		public Vec2 linearVelocity;
		public float maxVelocityForce;
		public float angularVelocity;
		public float maxVelocityTorque;
		public float linearHertz;
		public float linearDampingRatio;
		public float maxSpringForce;
		public float angularHertz;
		public float angularDampingRatio;
		public float maxSpringTorque;

		public Vec2 linearVelocityImpulse;
		public float angularVelocityImpulse;
		public Vec2 linearSpringImpulse;
		public float angularSpringImpulse;

		public Softness linearSpring;
		public Softness angularSpring;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public Mat22 linearMass;
		public float angularMass;
	}

	internal struct MoverJoint
	{
		public Vec2 linearVelocity;
		public Vec2 maxVelocityForce;

		public Vec2 linearVelocityImpulse;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public float linearMass;
	}

	internal struct PrismaticJoint
	{
		public Vec2 impulse;
		public float springImpulse;
		public float motorImpulse;
		public float lowerImpulse;
		public float upperImpulse;
		public float hertz;
		public float dampingRatio;
		public float targetTranslation;
		public float maxMotorForce;
		public float motorSpeed;
		public float lowerTranslation;
		public float upperTranslation;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public Softness springSoftness;

		public bool enableSpring;
		public bool enableLimit;
		public bool enableMotor;
	}

	internal struct RevoluteJoint
	{
		public Vec2 linearImpulse;
		public float springImpulse;
		public float motorImpulse;
		public float lowerImpulse;
		public float upperImpulse;
		public float hertz;
		public float dampingRatio;
		public float targetAngle;
		public float maxMotorTorque;
		public float motorSpeed;
		public float lowerAngle;
		public float upperAngle;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public float axialMass;
		public Softness springSoftness;

		public bool enableSpring;
		public bool enableMotor;
		public bool enableLimit;
	}

	internal struct WeldJoint
	{
		public float linearHertz;
		public float linearDampingRatio;
		public float angularHertz;
		public float angularDampingRatio;

		public Softness linearSpring;
		public Softness angularSpring;
		public Vec2 linearImpulse;
		public float angularImpulse;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public float axialMass;
	}

	internal struct WheelJoint
	{
		public float perpImpulse;
		public float motorImpulse;
		public float springImpulse;
		public float lowerImpulse;
		public float upperImpulse;
		public float maxMotorTorque;
		public float motorSpeed;
		public float lowerTranslation;
		public float upperTranslation;
		public float hertz;
		public float dampingRatio;

		public int indexA;
		public int indexB;
		public Transform frameA;
		public Transform frameB;
		public Vec2 deltaCenter;
		public float perpMass;
		public float motorMass;
		public float axialMass;
		public Softness springSoftness;

		public bool enableSpring;
		public bool enableMotor;
		public bool enableLimit;
	}

	/// The base joint class. Joints are used to constraint two bodies together in
	/// various fashions. Some joints also feature limits and motors.
	internal struct JointSim
	{
		public int jointId;

		public int bodyIdA;
		public int bodyIdB;

		public JointType type;

		public Transform localFrameA;
		public Transform localFrameB;

		public float invMassA;
		public float invMassB;
		public float invIA;
		public float invIB;

		public float constraintHertz;
		public float constraintDampingRatio;

		public Softness constraintSoftness;

		public float forceThreshold;
		public float torqueThreshold;
		public JointSimUnion u;
		[UnscopedRef] public ref DistanceJoint distanceJoint => ref u.distanceJoint;
		[UnscopedRef] public ref MotorJoint motorJoint => ref u.motorJoint;
		[UnscopedRef] public ref MoverJoint moverJoint => ref u.moverJoint;
		[UnscopedRef] public ref PogoJoint pogoJoint => ref u.pogoJoint;
		[UnscopedRef] public ref RevoluteJoint revoluteJoint => ref u.revoluteJoint;
		[UnscopedRef] public ref PrismaticJoint prismaticJoint => ref u.prismaticJoint;
		[UnscopedRef] public ref WeldJoint weldJoint => ref u.weldJoint;
		[UnscopedRef] public ref WheelJoint wheelJoint => ref u.wheelJoint;
	}

	[StructLayout( LayoutKind.Explicit )]
	internal struct JointSimUnion
	{
		[FieldOffset( 0 )] public DistanceJoint distanceJoint;
		[FieldOffset( 0 )] public MotorJoint motorJoint;
		[FieldOffset( 0 )] public MoverJoint moverJoint;
		[FieldOffset( 0 )] public PogoJoint pogoJoint;
		[FieldOffset( 0 )] public RevoluteJoint revoluteJoint;
		[FieldOffset( 0 )] public PrismaticJoint prismaticJoint;
		[FieldOffset( 0 )] public WeldJoint weldJoint;
		[FieldOffset( 0 )] public WheelJoint wheelJoint;
	}















}
