// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Box2DNet.B2;
using static Box2DNet.Tests.TestMacros;

namespace Box2DNet.Tests;

public static partial class HumanShared
{
	// From shared/human.h
	internal enum BoneId
	{
		bone_hip = 0,
		bone_torso = 1,
		bone_head = 2,
		bone_upperLeftLeg = 3,
		bone_lowerLeftLeg = 4,
		bone_upperRightLeg = 5,
		bone_lowerRightLeg = 6,
		bone_upperLeftArm = 7,
		bone_lowerLeftArm = 8,
		bone_upperRightArm = 9,
		bone_lowerRightArm = 10,
		bone_count = 11,
	}

	internal struct Bone
	{
		public BodyId bodyId;
		public JointId jointId;
		public float frictionScale;
		public float maxTorque;
		public int parentIndex;
	}

	internal struct Human
	{
		public Bone[] bones;
		public float frictionTorque;
		public float originalScale;
		public float scale;
		public bool isSpawned;
	}

	internal static void CreateHuman( ref Human human, WorldId worldId, Pos position, float scale, float frictionTorque, float hertz, float dampingRatio, int groupIndex, object? userData, bool colorize )
	{
		assert( human.isSpawned == false );

		for ( int i = 0; i < bone_count; ++i )
		{
			human.bones[i].bodyId = nullBodyId;
			human.bones[i].jointId = nullJointId;
			human.bones[i].frictionScale = 1.0f;
			human.bones[i].parentIndex = -1;
		}

		human.originalScale = scale;
		human.scale = scale;
		human.frictionTorque = frictionTorque;

		BodyDef bodyDef = DefaultBodyDef();
		bodyDef.type = BodyType.dynamicBody;
		bodyDef.sleepThreshold = 0.1f;
		bodyDef.userData = userData;

		ShapeDef shapeDef = DefaultShapeDef();
		shapeDef.material.friction = 0.2f;
		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = ( 1 | 2 );

		ShapeDef footShapeDef = shapeDef;
		footShapeDef.material.friction = 0.05f;

		// feet don't collide with ragdolls
		footShapeDef.filter.categoryBits = 2;
		footShapeDef.filter.maskBits = 1;

		if ( colorize )
		{
			footShapeDef.material.customColor = HexColor.colorSaddleBrown;
		}

		float s = scale;
		float maxTorque = frictionTorque * s;
		bool enableMotor = true;
		bool enableLimit = true;
		float drawSize = 0.05f;

		HexColor shirtColor = HexColor.colorMediumTurquoise;
		HexColor pantColor = HexColor.colorDodgerBlue;

		HexColor[] skinColors = { HexColor.colorNavajoWhite, HexColor.colorLightYellow, HexColor.colorPeru, HexColor.colorTan };
		HexColor skinColor = skinColors[groupIndex % 4];

		// hip
		{
			Bone* bone = human.bones + bone_hip;
			bone.parentIndex = -1;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.95f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "hip";

			bone.bodyId = CreateBody( worldId, ref bodyDef );

			if ( colorize )
			{
				shapeDef.material.customColor = pantColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.02f * s ), center2 = new( 0.0f, 0.02f * s ), radius = 0.095f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );
		}

		// torso
		{
			Bone* bone = human.bones + bone_torso;
			bone.parentIndex = bone_hip;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 1.2f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "torso";

			// bodyDef.type = BodyType.staticBody;
			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.5f;
			bodyDef.type = BodyType.dynamicBody;

			if ( colorize )
			{
				shapeDef.material.customColor = shirtColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.135f * s ), center2 = new( 0.0f, 0.135f * s ), radius = 0.09f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.0f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.25f * Pi;
			jointDef.upperAngle = 0.0f;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// head
		{
			Bone* bone = human.bones + bone_head;
			bone.parentIndex = bone_torso;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f * s, 1.475f * s ) );
			bodyDef.linearDamping = 0.1f;
			bodyDef.name = "head";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.25f;

			if ( colorize )
			{
				shapeDef.material.customColor = skinColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.038f * s ), center2 = new( 0.0f, 0.039f * s ), radius = 0.075f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			//// neck
			// capsule = { { 0.0f, -0.12f * s }, { 0.0f, -0.08f * s }, 0.05f * s };
			// CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.4f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.3f * Pi;
			jointDef.upperAngle = 0.1f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// upper left leg
		{
			Bone* bone = human.bones + bone_upperLeftLeg;
			bone.parentIndex = bone_hip;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.775f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "upper_left_leg";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 1.0f;

			if ( colorize )
			{
				shapeDef.material.customColor = pantColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.06f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 0.9f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.05f * Pi;
			jointDef.upperAngle = 0.4f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		Vec2[] points = { new( -0.03f * s, -0.185f * s ),
			new( 0.11f * s, -0.185f * s ),
			new( 0.11f * s, -0.16f * s ),
			new( -0.03f * s, -0.14f * s ), };

		Hull footHull = ComputeHull( points, 4 );
		Polygon footPolygon = MakePolygon( ref footHull, 0.015f * s );

		// lower left leg
		{
			Bone* bone = human.bones + bone_lowerLeftLeg;
			bone.parentIndex = bone_upperLeftLeg;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.475f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "lower_left_leg";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.5f;

			if ( colorize )
			{
				shapeDef.material.customColor = pantColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.155f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.045f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			// Polygon box = MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
			// CreatePolygonShape(bone.bodyId, ref shapeDef, ref box);

			// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
			// CreateCapsuleShape( bone.bodyId, ref footShapeDef, ref capsule );

			CreatePolygonShape( bone.bodyId, ref footShapeDef, ref footPolygon );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 0.625f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.5f * Pi;
			jointDef.upperAngle = -0.02f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// upper right leg
		{
			Bone* bone = human.bones + bone_upperRightLeg;
			bone.parentIndex = bone_hip;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.775f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "upper_right_leg";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 1.0f;

			if ( colorize )
			{
				shapeDef.material.customColor = pantColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.06f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 0.9f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.05f * Pi;
			jointDef.upperAngle = 0.4f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// lower right leg
		{
			Bone* bone = human.bones + bone_lowerRightLeg;
			bone.parentIndex = bone_upperRightLeg;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.475f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "lower_right_leg";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.5f;

			if ( colorize )
			{
				shapeDef.material.customColor = pantColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.155f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.045f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			// Polygon box = MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
			// CreatePolygonShape(bone.bodyId, ref shapeDef, ref box);

			// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
			// CreateCapsuleShape( bone.bodyId, ref footShapeDef, ref capsule );

			CreatePolygonShape( bone.bodyId, ref footShapeDef, ref footPolygon );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 0.625f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.5f * Pi;
			jointDef.upperAngle = -0.02f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// upper left arm
		{
			Bone* bone = human.bones + bone_upperLeftArm;
			bone.parentIndex = bone_torso;
			bone.frictionScale = 0.5f;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 1.225f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "upper_left_arm";

			bone.bodyId = CreateBody( worldId, ref bodyDef );

			if ( colorize )
			{
				shapeDef.material.customColor = shirtColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.035f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.35f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.1f * Pi;
			jointDef.upperAngle = 0.8f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// lower left arm
		{
			Bone* bone = human.bones + bone_lowerLeftArm;
			bone.parentIndex = bone_upperLeftArm;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.975f * s ) );
			bodyDef.linearDamping = 0.1f;
			bodyDef.name = "lower_left_arm";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.1f;

			if ( colorize )
			{
				shapeDef.material.customColor = skinColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.03f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.1f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameA.q = MakeRot( 0.25f * Pi );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.2f * Pi;
			jointDef.upperAngle = 0.3f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// upper right arm
		{
			Bone* bone = human.bones + bone_upperRightArm;
			bone.parentIndex = bone_torso;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 1.225f * s ) );
			bodyDef.linearDamping = 0.0f;
			bodyDef.name = "upper_right_arm";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.5f;

			if ( colorize )
			{
				shapeDef.material.customColor = shirtColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.035f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.35f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.1f * Pi;
			jointDef.upperAngle = 0.8f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		// lower right arm
		{
			Bone* bone = human.bones + bone_lowerRightArm;
			bone.parentIndex = bone_upperRightArm;

			bodyDef.position = OffsetPos( position, new Vec2( 0.0f, 0.975f * s ) );
			bodyDef.linearDamping = 0.1f;
			bodyDef.name = "lower_right_arm";

			bone.bodyId = CreateBody( worldId, ref bodyDef );
			bone.frictionScale = 0.1f;

			if ( colorize )
			{
				shapeDef.material.customColor = skinColor;
			}

			Capsule capsule = new Capsule { center1 = new( 0.0f, -0.125f * s ), center2 = new( 0.0f, 0.125f * s ), radius = 0.03f * s };
			CreateCapsuleShape( bone.bodyId, ref shapeDef, ref capsule );

			Pos pivot = OffsetPos( position, new Vec2( 0.0f, 1.1f * s ) );
			RevoluteJointDef jointDef = DefaultRevoluteJointDef();
			jointDef.@base.bodyIdA = human.bones[bone.parentIndex].bodyId;
			jointDef.@base.bodyIdB = bone.bodyId;
			jointDef.@base.localFrameA.p = Body_GetLocalPoint( jointDef.@base.bodyIdA, pivot );
			jointDef.@base.localFrameA.q = MakeRot( 0.25f * Pi );
			jointDef.@base.localFrameB.p = Body_GetLocalPoint( jointDef.@base.bodyIdB, pivot );
			jointDef.enableLimit = enableLimit;
			jointDef.lowerAngle = -0.2f * Pi;
			jointDef.upperAngle = 0.3f * Pi;
			jointDef.enableMotor = enableMotor;
			jointDef.maxMotorTorque = bone.frictionScale * maxTorque;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.@base.drawScale = drawSize;

			bone.jointId = CreateRevoluteJoint( worldId, ref jointDef );
		}

		human.isSpawned = true;
	}

	internal static void DestroyHuman( ref Human human )
	{
		assert( human.isSpawned == true );

		for ( int i = 0; i < bone_count; ++i )
		{
			if ( human.bones[i].jointId.IsNull )
			{
				continue;
			}

			DestroyJoint( human.bones[i].jointId );
			human.bones[i].jointId = nullJointId;
		}

		for ( int i = 0; i < bone_count; ++i )
		{
			if ( human.bones[i].bodyId.IsNull )
			{
				continue;
			}

			DestroyBody( human.bones[i].bodyId );
			human.bones[i].bodyId = nullBodyId;
		}

		human.isSpawned = false;
	}

	internal static void Human_SetVelocity( ref Human human, Vec2 velocity )
	{
		for ( int i = 0; i < bone_count; ++i )
		{
			BodyId bodyId = human.bones[i].bodyId;

			if ( bodyId.IsNull )
			{
				continue;
			}

			Body_SetLinearVelocity( bodyId, velocity );
		}
	}

	internal static void Human_ApplyRandomAngularImpulse( ref Human human, float magnitude )
	{
		assert( human.isSpawned == true );
		float impulse = RandomFloatRange( -magnitude, magnitude );
		Body_ApplyAngularImpulse( human.bones[bone_torso].bodyId, impulse, true );
	}

	internal static void Human_SetJointFrictionTorque( ref Human human, float torque )
	{
		assert( human.isSpawned == true );
		if ( torque == 0.0f )
		{
			for ( int i = 1; i < bone_count; ++i )
			{
				RevoluteJoint_EnableMotor( human.bones[i].jointId, false );
			}
		}
		else
		{
			for ( int i = 1; i < bone_count; ++i )
			{
				RevoluteJoint_EnableMotor( human.bones[i].jointId, true );
				float scale = human.scale * human.bones[i].frictionScale;
				RevoluteJoint_SetMaxMotorTorque( human.bones[i].jointId, scale * torque );
			}
		}
	}

	internal static void Human_SetJointSpringHertz( ref Human human, float hertz )
	{
		assert( human.isSpawned == true );
		if ( hertz == 0.0f )
		{
			for ( int i = 1; i < bone_count; ++i )
			{
				RevoluteJoint_EnableSpring( human.bones[i].jointId, false );
			}
		}
		else
		{
			for ( int i = 1; i < bone_count; ++i )
			{
				RevoluteJoint_EnableSpring( human.bones[i].jointId, true );
				RevoluteJoint_SetSpringHertz( human.bones[i].jointId, hertz );
			}
		}
	}

	internal static void Human_SetJointDampingRatio( ref Human human, float dampingRatio )
	{
		assert( human.isSpawned == true );
		for ( int i = 1; i < bone_count; ++i )
		{
			RevoluteJoint_SetSpringDampingRatio( human.bones[i].jointId, dampingRatio );
		}
	}

	internal static void Human_EnableSensorEvents( ref Human human, bool enable )
	{
		assert( human.isSpawned == true );
		BodyId bodyId = human.bones[bone_torso].bodyId;

		ShapeId shapeId = default;
		int count = Body_GetShapes( bodyId, shapeId, 1 );
		if ( count == 1 )
		{
			Shape_EnableSensorEvents( shapeId, enable );
		}
	}

	internal static void Human_SetScale( ref Human human, float scale )
	{
		assert( human.isSpawned == true );
		assert( 0.01f < scale && scale < 100.0f );
		assert( 0.0f < human.scale );

		float ratio = scale / human.scale;

		// Torque scales by pow(length, 4) due to mass change and length change. However, gravity is also a factor
		// so I'm using pow(length, 3)
		float originalRatio = scale / human.originalScale;
		float frictionTorque = ( originalRatio * originalRatio * originalRatio ) * human.frictionTorque;

		Pos origin = Body_GetPosition( human.bones[0].bodyId );

		for ( int boneIndex = 0; boneIndex < bone_count; ++boneIndex )
		{
			Bone* bone = human.bones + boneIndex;

			if ( boneIndex > 0 )
			{
				// Scale the bone offset from the root in float, the offset is bone scale
				WorldTransform worldTransform = Body_GetTransform( bone.bodyId );
				Vec2 d = MulSV( ratio, SubPos( worldTransform.p, origin ) );
				Body_SetTransform( bone.bodyId, OffsetPos( origin, d ), worldTransform.q );

				Transform localFrameA = Joint_GetLocalFrameA( bone.jointId );
				Transform localFrameB = Joint_GetLocalFrameB( bone.jointId );
				localFrameA.p = MulSV( ratio, localFrameA.p );
				localFrameB.p = MulSV( ratio, localFrameB.p );
				Joint_SetLocalFrameA( bone.jointId, localFrameA );
				Joint_SetLocalFrameB( bone.jointId, localFrameB );

				JointType type = Joint_GetType( bone.jointId );
				if ( type == JointType.revoluteJoint )
				{
					RevoluteJoint_SetMaxMotorTorque( bone.jointId, bone.frictionScale * frictionTorque );
				}
			}

			FixedArray2<ShapeId> shapeIds = default;
			int shapeCount = Body_GetShapes( bone.bodyId, shapeIds, 2 );
			for ( int shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
			{
				ShapeType type = Shape_GetType( shapeIds[shapeIndex] );
				if ( type == ShapeType.capsuleShape )
				{
					Capsule capsule = Shape_GetCapsule( shapeIds[shapeIndex] );
					capsule.center1 = MulSV( ratio, capsule.center1 );
					capsule.center2 = MulSV( ratio, capsule.center2 );
					capsule.radius *= ratio;
					Shape_SetCapsule( shapeIds[shapeIndex], ref capsule );
				}
				else if ( type == ShapeType.polygonShape )
				{
					Polygon polygon = Shape_GetPolygon( shapeIds[shapeIndex] );
					for ( int pointIndex = 0; pointIndex < polygon.count; ++pointIndex )
					{
						polygon.vertices[pointIndex] = MulSV( ratio, polygon.vertices[pointIndex] );
					}

					polygon.centroid = MulSV( ratio, polygon.centroid );
					polygon.radius *= ratio;

					Shape_SetPolygon( shapeIds[shapeIndex], ref polygon );
				}
			}

			Body_UpdateMassFromShapes( bone.bodyId );
		}

		human.scale = scale;
	}

}
